"""FASE 5 (tesi): quanto tempo fa risparmiare il sistema completo (correzione AI del Kalman + controllo
predittivo di velocita') rispetto al robot "prima" di questi due livelli (solo Kalman, velocita' costante),
sugli stessi scenari. Un'idea generale del risparmio, non una misura di precisione: poche mappe
rappresentative a densita' realistica, un mix comportamentale, niente rifiniture come l'isteresi del
percorso (stabilizza quale percorso scegliere a parita' di costo, non cambia quanto ci mette a percorrerlo).

La macchia di probabilita' qui lavora direttamente sulla griglia di movimento (sottocelle_per_lato=1: e' il
caso limite gia' previsto da macchia_diffusione, "coincide con la griglia di movimento") invece che sulla
sotto-griglia fine usata in main.py per il rendering smussato - quella finezza e' visiva, non cambia le
decisioni di percorso.

In fondo al file, una piccola conversione tempo->soldi: prende il risparmio percentuale misurato e le tue
stime (costo orario del robot, ore di operativita' al giorno) - quei numeri vanno cercati/stimati da fonti
esterne (progetti di robot simili gia' esistenti), qui sono solo segnaposto da modificare.

Uso:
    py verifica_beneficio_ai.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import time
import multiprocessing as mp
import zlib
import numpy as np
import torch
import main as sim
import training_scenari as ts
from genera_dataset_previsione import _ricalcola_percorsi, _muovi_e_gestisci_stato, _rileva_e_traccia
from allena_previsione import CorrezioneKalman, prepara_input, FINESTRA_STORICO_FRAME, FILE_MODELLO

FRAME_SIMULAZIONE = 2400  # 40s di simulazione per scenario/condizione
MAPPE_CAMPIONE = ["aperta", "ostacoli_organici", "corridoio", "porta_stretta", "incrocio"]
DENSITA_CAMPIONE = "denso"
MIX_CAMPIONE = "misto"

torch.set_num_threads(1)  # vedi lo stesso commento in main.py: batch minuscoli, il multithreading di torch costa piu' di quanto renda


def _carica_modello():
    modello = CorrezioneKalman()
    modello.load_state_dict(torch.load(FILE_MODELLO, map_location="cpu"))
    modello.eval()
    return modello


def _blob_costo(griglia, tracciamento_lidar, storico_posizioni, usa_ai, modello_ai):
    """Ricostruisce mappa_costo_probabilita come in main.py (batch unico per la correzione AI, vedi lo
    stesso ottimizzazione li'), sulla griglia di movimento diretta invece che sulla sotto-griglia fine."""
    correzioni_ai = {}
    if usa_ai:
        pid_ok, storici_ok = [], []
        for pid, traccia in tracciamento_lidar.items():
            if traccia.get("ferma", False):
                continue
            storico_pid = storico_posizioni.get(pid)
            if storico_pid is not None and len(storico_pid) == FINESTRA_STORICO_FRAME:
                pid_ok.append(pid)
                storici_ok.append(storico_pid)
        if pid_ok:
            batch = prepara_input(np.array(storici_ok, dtype=np.float32))
            with torch.no_grad():
                correzioni = modello_ai(torch.from_numpy(batch)).numpy()
            correzioni_ai = dict(zip(pid_ok, correzioni))

    mappa_costo = {}
    for pid, traccia in tracciamento_lidar.items():
        e_ferma = traccia.get("ferma", False)
        frame_futuri = 0 if e_ferma else sim.PREVISIONE_BLOB_FRAME
        direzione = None if e_ferma else (traccia["vx"], traccia["vy"])
        x_prev, y_prev = sim.previsione_posizione_kalman(traccia, frame_futuri)
        correzione = correzioni_ai.get(pid)
        if correzione is not None:
            x_prev += float(correzione[0])
            y_prev += float(correzione[1])
        rf = max(0, min(sim.Y_TOT - 1, int(y_prev // sim.DIM_NODO)))
        cf = max(0, min(sim.X_TOT - 1, int(x_prev // sim.DIM_NODO)))
        for cella, peso in sim.macchia_diffusione(griglia, rf, cf, sim.RAGGIO_BLOB_CELLE, sim.DECADIMENTO_BLOB, 1, direzione=direzione).items():
            costo = peso * sim.COSTO_MASSIMO_PROBABILITA
            if cella not in mappa_costo or mappa_costo[cella] < costo:
                mappa_costo[cella] = costo
    return mappa_costo


def esegui(nome_mappa, condizione, seed, num_frame=FRAME_SIMULAZIONE):
    """condizione 'prima' = solo Kalman, velocita' costante (il robot ante-tesi). 'dopo' = Kalman+AI e
    controllo predittivo di velocita' (il robot attuale)."""
    usa_ai = usa_velocita = condizione == "dopo"
    modello_ai = _carica_modello() if usa_ai else None

    scenario = ts.costruisci_scenario(nome_mappa, DENSITA_CAMPIONE, MIX_CAMPIONE, seed=seed)
    griglia = scenario["griglia"]
    membri_gruppi_mobili = [m for g in scenario["gruppi"] if g["mobile"] for m in g["membri"]]
    membri_gruppi_fermi = [m for g in scenario["gruppi"] if not g["mobile"] for m in g["membri"]]
    tutte_mobili = scenario["persone"] + scenario["corridori"] + membri_gruppi_mobili
    tutte_le_persone = tutte_mobili + scenario["persone_ferme"] + membri_gruppi_fermi
    celle_persone_ferme = {(pf["r"], pf["c"]) for pf in scenario["persone_ferme"]}
    celle_persone_ferme |= {(m["r"], m["c"]) for m in membri_gruppi_fermi}

    nodo_iniziale = sim.cella_libera_casuale(griglia)
    stato_robot = {"x": nodo_iniziale.cx, "y": nodo_iniziale.cy, "percorso": [], "nodo_target": sim.cella_libera_casuale(griglia)}
    tracciamento_lidar, storico_posizioni = {}, {}

    range_evitamento_quad = sim.RANGE_EVITAMENTO_PERSONE ** 2
    range_evitamento_robot = sim.RAGGIO_ROBOT_DEFAULT + sim.RAGGIO_PERSONA_DEFAULT

    mappa_costo = {}
    numero_arrivi = 0
    for contatore_frame in range(num_frame):
        robot_x, robot_y = stato_robot["x"], stato_robot["y"]

        celle_bloccate_persone = {(int(p2["y"] // sim.DIM_NODO), int(p2["x"] // sim.DIM_NODO)) for p2 in tutte_mobili}
        celle_bloccate_persone |= celle_persone_ferme
        celle_bloccate_comune = celle_bloccate_persone | {(int(robot_y // sim.DIM_NODO), int(robot_x // sim.DIM_NODO))}

        _ricalcola_percorsi(tutte_mobili, contatore_frame, griglia, celle_bloccate_comune)
        griglia_spaziale = sim.costruisci_griglia_spaziale(tutte_mobili, sim.DIM_NODO)
        _muovi_e_gestisci_stato(tutte_mobili, griglia, griglia_spaziale, robot_x, robot_y, range_evitamento_quad, range_evitamento_robot)

        _rileva_e_traccia(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)

        if contatore_frame % sim.INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME == 0:
            mappa_costo = _blob_costo(griglia, tracciamento_lidar, storico_posizioni, usa_ai, modello_ai)

        ricalcolo_dovuto = contatore_frame % sim.INTERVALLO_RICALCOLO_ROBOT_FRAME == 0
        if not stato_robot["percorso"] or ricalcolo_dovuto:
            nodo_target = stato_robot["nodo_target"]
            stato_robot["percorso"] = sim.algoritmo_a_star(
                griglia, (stato_robot["x"], stato_robot["y"]), (nodo_target.r, nodo_target.c),
                celle_bloccate=celle_bloccate_persone, mappa_costo_extra=mappa_costo)

        velocita_nominale = sim.VELOCITA_ROBOT
        fattore = 1.0
        if usa_velocita:
            fattore = sim.fattore_velocita_da_conflitto(
                stato_robot["x"], stato_robot["y"], stato_robot["percorso"], tracciamento_lidar,
                velocita_nominale, sim.RAGGIO_ROBOT_DEFAULT, sim.RAGGIO_PERSONA_DEFAULT)

        x, y, arrivato, bloccato = sim.passo_movimento(
            stato_robot["x"], stato_robot["y"], stato_robot["percorso"], velocita_nominale * fattore, stato_robot["nodo_target"], griglia)
        stato_robot["x"], stato_robot["y"] = x, y
        if bloccato:
            stato_robot["percorso"] = []
        if arrivato:
            numero_arrivi += 1
            stato_robot["nodo_target"] = sim.cella_libera_casuale(griglia)
            stato_robot["percorso"] = []

    tempo_medio_per_arrivo = num_frame / numero_arrivi if numero_arrivi > 0 else None
    return nome_mappa, condizione, numero_arrivi, tempo_medio_per_arrivo


def _esegui_pool(argomenti):
    return esegui(*argomenti)


def confronta(mappe):
    combinazioni = [(m, cond) for m in mappe for cond in ("prima", "dopo")]
    argomenti = [(m, cond, zlib.crc32(f"{m}|{cond}".encode())) for m, cond in combinazioni]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Uso {n_worker} processi in parallelo su {len(combinazioni)} combinazioni ({len(mappe)} mappe x 2 condizioni).\n")

    risultati = {m: {} for m in mappe}
    inizio = time.time()
    completati = 0
    with mp.Pool(n_worker) as pool:
        for nome_mappa, condizione, numero_arrivi, tempo_medio in pool.imap_unordered(_esegui_pool, argomenti):
            completati += 1
            risultati[nome_mappa][condizione] = (numero_arrivi, tempo_medio)
            tempo_str = f"{tempo_medio:.0f} frame/arrivo" if tempo_medio else "MAI arrivato"
            print(f"[{completati}/{len(combinazioni)}] {nome_mappa} ({condizione}): {numero_arrivi} arrivi -> {tempo_str} ({time.time() - inizio:.0f}s trascorsi)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    print("=== Tempo per completare uno spostamento: prima vs dopo (correzione AI + controllo velocita') ===")
    risparmi = []
    for nome_mappa in mappe:
        prima = risultati[nome_mappa].get("prima")
        dopo = risultati[nome_mappa].get("dopo")
        if not prima or not dopo or prima[1] is None or dopo[1] is None:
            print(f"\n{nome_mappa}: dati insufficienti (arrivi troppo pochi in almeno una condizione)")
            continue
        risparmio = (1 - dopo[1] / prima[1]) * 100
        risparmi.append(risparmio)
        print(f"\n{nome_mappa}:")
        print(f"  prima: {prima[1]:.0f} frame/arrivo ({prima[0]} arrivi)")
        print(f"  dopo:  {dopo[1]:.0f} frame/arrivo ({dopo[0]} arrivi)")
        print(f"  risparmio di tempo: {risparmio:+.1f}%")

    if risparmi:
        print(f"\nRisparmio medio sulle {len(risparmi)} mappe testate: {sum(risparmi) / len(risparmi):+.1f}%")
        _calcolatore_risparmio(sum(risparmi) / len(risparmi))


def _calcolatore_risparmio(percentuale_risparmio_tempo):
    """Conversione tempo->soldi: segnaposto da modificare con stime reali (costo orario di gestione di un
    robot di servizio simile, ore di operativita' al giorno) - vedi il commento in testa al file. Qui solo
    per dare un ordine di grandezza, non un numero da citare come preciso."""
    COSTO_ORARIO_ROBOT_EUR = 15.0  # placeholder: costo di gestione/ammortamento stimato all'ora
    ORE_OPERATIVITA_GIORNO = 8.0   # placeholder: ore al giorno in cui il robot e' operativo

    tempo_risparmiato_ore_giorno = ORE_OPERATIVITA_GIORNO * (percentuale_risparmio_tempo / 100)
    risparmio_giornaliero = tempo_risparmiato_ore_giorno * COSTO_ORARIO_ROBOT_EUR
    print(f"\n=== Stima di massima (NON i costi di implementazione dell'AI - solo il beneficio) ===")
    print(f"Con costo orario stimato {COSTO_ORARIO_ROBOT_EUR:.0f} EUR/h e {ORE_OPERATIVITA_GIORNO:.0f}h/giorno di operativita':")
    print(f"  tempo equivalente risparmiato: ~{tempo_risparmiato_ore_giorno:.2f}h/giorno")
    print(f"  risparmio stimato: ~{risparmio_giornaliero:.1f} EUR/giorno, ~{risparmio_giornaliero * 260:.0f} EUR/anno (260 giorni lavorativi)")
    print("(modifica COSTO_ORARIO_ROBOT_EUR e ORE_OPERATIVITA_GIORNO in questo file con stime da progetti di robot simili)")


if __name__ == "__main__":
    mappe = sys.argv[1:] if len(sys.argv) > 1 else MAPPE_CAMPIONE
    confronta(mappe)

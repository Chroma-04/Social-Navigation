"""Verifica che la correzione AI funzioni DENTRO il ciclo di simulazione del test, non solo sul dataset.

Il +12.2% riportato dal training e' una misura ad anello aperto, su dati registrati. Qui si rifa' la
stessa misura ad anello chiuso, mentre il robot naviga davvero: ad ogni aggiornamento della macchia si
registrano la previsione del Kalman e quella corretta dall'AI, e quando il frame futuro arriva si
confrontano entrambe con la posizione VERA della persona.

Se nel loop l'AI risulta peggiore del Kalman c'e' un problema reale (segno, scala, o scarto fra come sono
stati raccolti i dati e come gira il test). Se risulta migliore, la percezione fa il suo lavoro e il
problema del tempo di percorrenza sta a valle, nella catena previsione -> costo -> percorso -> tempo.
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import math
import collections
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # la radice del progetto
import main as sim
import training_scenari as ts
import testing as t
from ai_predittiva.genera_dataset_previsione import _ricalcola_percorsi, _muovi_e_gestisci_stato, _rileva_e_traccia
from ai_predittiva.allena_previsione import prepara_input, FINESTRA_STORICO_FRAME

POPOLAZIONE = 300
SEED = 12345
import random


def esegui_strumentata(nome_mappa="training_mappa_01", popolazione_totale=POPOLAZIONE, seed=SEED):
    t._inizializza_worker()
    modello = t._modello_ai_worker

    pop = t.popolazione_da_totale(popolazione_totale)
    raggio_sicurezza = t._geometria_salvata()['raggio_sicurezza']

    random.seed(seed)
    griglia = [[sim.Nodo(r, c) for c in range(sim.X_TOT)] for r in range(sim.Y_TOT)]
    sim.crea_bordi(griglia)
    ts.MAPPE[nome_mappa](griglia)

    persone = [sim.crea_persona(griglia) for _ in range(pop["numero_persone"])]
    corridori = [sim.crea_corridore(griglia) for _ in range(pop["numero_corridori"])]
    persone_ferme = [sim.crea_persona_ferma(griglia) for _ in range(pop["numero_persone_ferme"])]
    gruppi = [sim.crea_gruppo(griglia) for _ in range(pop["numero_gruppi"])]

    membri_mobili = [m for g in gruppi if g["mobile"] for m in g["membri"]]
    membri_fermi = [m for g in gruppi if not g["mobile"] for m in g["membri"]]
    tutte_mobili = persone + corridori + membri_mobili
    tutte_le_persone = tutte_mobili + persone_ferme + membri_fermi
    celle_ferme = {(pf["r"], pf["c"]) for pf in persone_ferme}
    celle_ferme |= {(m["r"], m["c"]) for m in membri_fermi}
    entita_per_pid = {id(p): p for p in tutte_le_persone}

    for p in tutte_mobili:
        p["fattore_velocita"] *= t.ACCELERAZIONE

    orizzonte = max(1, sim.PREVISIONE_BLOB_FRAME // t.ACCELERAZIONE)
    intervallo_ricalcolo = max(1, sim.INTERVALLO_RICALCOLO_ROBOT_FRAME // t.ACCELERAZIONE)
    intervallo_macchia = max(1, sim.INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME // t.ACCELERAZIONE)
    sim.FRAME_REAZIONE_VELOCITA_ROBOT = max(1, t._FRAME_REAZIONE_ORIGINALE // t.ACCELERAZIONE)

    partenza = t._cella_vicina_angolo(griglia, 0, 0)
    arrivo = t._cella_vicina_angolo(griglia, sim.Y_TOT - 1, sim.X_TOT - 1)
    robot = {"x": partenza.cx, "y": partenza.cy, "percorso": []}
    tracciamento_lidar, storico_posizioni = {}, {}

    range_evitamento_quad = sim.RANGE_EVITAMENTO_PERSONE ** 2
    range_evitamento_robot = sim.RAGGIO_ROBOT_DEFAULT + sim.RAGGIO_PERSONA_DEFAULT
    velocita_nominale = sim.VELOCITA_ROBOT * t.ACCELERAZIONE

    in_sospeso = collections.deque()
    err_kalman, err_ai = [], []
    costi = {}

    for frame in range(t.FRAME_MAX):
        robot_x, robot_y = robot["x"], robot["y"]
        celle_bloccate = {(int(p["y"] // sim.DIM_NODO), int(p["x"] // sim.DIM_NODO)) for p in tutte_mobili}
        celle_bloccate |= celle_ferme
        celle_comune = celle_bloccate | {(int(robot_y // sim.DIM_NODO), int(robot_x // sim.DIM_NODO))}

        _ricalcola_percorsi(tutte_mobili, frame, griglia, celle_comune)
        griglia_spaziale = sim.costruisci_griglia_spaziale(tutte_mobili, sim.DIM_NODO)
        _muovi_e_gestisci_stato(tutte_mobili, griglia, griglia_spaziale, robot_x, robot_y,
                                range_evitamento_quad, range_evitamento_robot)
        _rileva_e_traccia(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)

        # risolve le previsioni maturate: confronto con la posizione VERA di adesso
        while in_sospeso and in_sospeso[0]["frame_maturazione"] <= frame:
            voce = in_sospeso.popleft()
            ent = entita_per_pid.get(voce["pid"])
            if ent is None:
                continue
            vero = (ent["x"], ent["y"])
            err_kalman.append(math.hypot(voce["kalman"][0] - vero[0], voce["kalman"][1] - vero[1]))
            err_ai.append(math.hypot(voce["ai"][0] - vero[0], voce["ai"][1] - vero[1]))

        if frame % intervallo_macchia == 0:
            # stesse previsioni che alimentano la macchia di costo, ma registrate per la verifica
            pid_ok, storici_ok = [], []
            for pid, traccia in tracciamento_lidar.items():
                if traccia.get("ferma", False):
                    continue
                st = storico_posizioni.get(pid)
                if st is not None and len(st) == FINESTRA_STORICO_FRAME:
                    pid_ok.append(pid)
                    storici_ok.append(st)
            if pid_ok:
                batch = prepara_input(np.array(storici_ok, dtype=np.float32))
                with torch.no_grad():
                    correzioni = modello(torch.from_numpy(batch)).numpy()
                for pid, corr in zip(pid_ok, correzioni):
                    kx, ky = sim.previsione_posizione_kalman(tracciamento_lidar[pid], orizzonte)
                    in_sospeso.append({"frame_maturazione": frame + orizzonte, "pid": pid,
                                       "kalman": (kx, ky), "ai": (kx + float(corr[0]), ky + float(corr[1]))})
            costi = t._mappa_costo(griglia, tracciamento_lidar, storico_posizioni, True, modello, orizzonte)

        if not robot["percorso"] or frame % intervallo_ricalcolo == 0:
            robot["percorso"] = sim.algoritmo_a_star(griglia, (robot["x"], robot["y"]), (arrivo.r, arrivo.c),
                                                     celle_bloccate=celle_bloccate, mappa_costo_extra=costi)

        if raggio_sicurezza > 0 and any(math.hypot(robot["x"] - p["x"], robot["y"] - p["y"]) < raggio_sicurezza
                                        for p in tutte_mobili):
            continue

        fattore = sim.fattore_velocita_da_conflitto(robot["x"], robot["y"], robot["percorso"],
                                                    tracciamento_lidar, velocita_nominale,
                                                    sim.RAGGIO_ROBOT_DEFAULT, sim.RAGGIO_PERSONA_DEFAULT)
        x, y, arrivato, bloccato = sim.passo_movimento(robot["x"], robot["y"], robot["percorso"],
                                                       velocita_nominale * fattore, arrivo, griglia)
        robot["x"], robot["y"] = x, y
        if bloccato:
            robot["percorso"] = []
        if arrivato:
            break

    return err_kalman, err_ai, frame + 1


if __name__ == "__main__":
    print(f"Corsa strumentata: {POPOLAZIONE} persone, seed {SEED}, accelerazione {t.ACCELERAZIONE}x\n")
    ek, ea, frame = esegui_strumentata()
    if not ek:
        print("Nessuna previsione maturata: niente da confrontare.")
        sys.exit(0)
    ek, ea = np.array(ek), np.array(ea)
    print(f"Corsa terminata in {frame} frame, {len(ek)} previsioni verificate contro la posizione vera\n")
    print("=== Errore di previsione DENTRO il loop di simulazione ===")
    print(f"{'':16}{'medio':>10}{'mediano':>10}{'90-perc':>10}")
    print(f"{'Kalman solo':16}{ek.mean():10.1f}{np.median(ek):10.1f}{np.percentile(ek, 90):10.1f}")
    print(f"{'Kalman + AI':16}{ea.mean():10.1f}{np.median(ea):10.1f}{np.percentile(ea, 90):10.1f}")
    migl = (1 - ea.mean() / ek.mean()) * 100
    print(f"\nMiglioramento nel loop: {migl:+.1f}%   (training ad anello aperto riportava +12.2%)")
    print(f"Previsioni in cui l'AI e' piu' vicina alla verita': {(ea < ek).mean() * 100:.1f}%")

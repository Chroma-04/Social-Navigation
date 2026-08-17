"""Misura empirica del tetto di densita' che ogni mappa puo' sostenere prima che la folla comprometta il
robot, invece di stabilirlo a tavolino con un numero fisso: fa girare ogni mappa a densita' crescente
(oltre i tre preset rado/medio/denso di training_scenari.py, che restano invariati e continuano a essere
quelli usati per generare il dataset di training) e misura quanto il robot impiega a completare i suoi
spostamenti punto a punto, non quanta distanza percorre.

La distanza grezza percorsa NON e' una buona misura qui: in questa simulazione il robot non ha mai una
vera collisione fisica con le persone (sono loro a essere respinte da lui, non viceversa, e il movimento
si ferma solo sui muri veri), quindi anche circondato continua a camminare a piena velocita' - si limita a
fare un giro piu' lungo per aggirare le celle occupate (costo A* extra, non blocco assoluto). Un robot che
gira per aggirare la folla accumula comunque distanza ma non progredisce: quello che risente davvero della
densita' e' quanto tempo impiega a chiudere ciascun tragitto (assegna un punto casuale, cammina, arriva,
ne assegna un altro), che si allunga sempre di piu' quando gli aggiramenti diventano piu' frequenti/ampi.

Il robot qui usa solo il costo estra per le celle occupate (COSTO_CELLA_OCCUPATA, lo stesso di main.py),
non la macchia di probabilita' ne' il controllo predittivo di velocita': l'obiettivo e' isolare il tetto
strutturale "quanta folla la mappa puo' fisicamente reggere", non quanto le euristiche del robot vero
aiutano a conviverci (quello e' materia dell'harness di valutazione, non di questa verifica).

Uso:
    py verifica_densita_massima.py             tutte le mappe, mix "misto"
    py verifica_densita_massima.py nome_mappa   solo una mappa (utile per iterare piu' in fretta)
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import time
import multiprocessing as mp
import zlib

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # i moduli condivisi stanno nella radice
for _percorso in (_RADICE, os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)
import main as sim
import training_scenari as ts
from genera_dataset_previsione import _ricalcola_percorsi, _muovi_e_gestisci_stato

FRAME_SIMULAZIONE = 1800  # 30s di simulazione per ogni combinazione mappa/livello di densita'
SOGLIA_MOBILITA = 0.5  # sotto questa frazione del tempo-per-arrivo alla densita' piu' bassa testata, la mappa e' considerata satura
MOLTIPLICATORI_SWEEP = [0.5, 1.0, 1.5, 2.0, 3.0]  # multipli del budget del preset "denso"
MIX_RAPPRESENTATIVO = "misto"  # un solo mix per lo sweep: la composizione influenza il tetto meno della densita' pura
# bersagli su TUTTA la mappa (non vicini al robot): su una mappa con un collo di bottiglia strutturale
# (porta_stretta, doppia_porta, corridoio) un bersaglio vicino resta spesso nella stessa stanza/tratto, senza
# mai costringere il robot ad attraversare il varco che si vuole mettere sotto stress - il test tornerebbe
# "va tutto bene" solo perche' non ha mai provato il punto critico, non perche' lo sia davvero. Con bersagli
# globali si hanno meno arrivi nello stesso tempo, ma quelli che contano: se il robot resta bloccato in un
# collo di bottiglia affollato, un arrivo a zero e' gia' di per se' un segnale (non solo "poco preciso")


def _densita_da_moltiplicatore(k):
    base = ts.LIVELLI_DENSITA["denso"]
    return {"budget_individui": base["budget_individui"] * k, "numero_gruppi_base": base["numero_gruppi_base"] * k}


def misura_mobilita(nome_mappa, moltiplicatore, seed, num_frame=FRAME_SIMULAZIONE):
    """Fa girare la mappa al livello di densita' richiesto e restituisce (numero_arrivi, tempo_medio_per_
    arrivo_in_frame): il robot passa continuamente da un punto casuale al successivo (come nell'harness di
    dataset, ma qui con il percorso penalizzato dalle celle occupate) e ogni volta che ne raggiunge uno gli
    se ne assegna subito un altro, cosi' il tempo medio fra un arrivo e il successivo riflette quanto la
    folla lo costringe a girarci intorno."""
    scenario = ts.costruisci_scenario(nome_mappa, _densita_da_moltiplicatore(moltiplicatore), MIX_RAPPRESENTATIVO, seed=seed)
    griglia = scenario["griglia"]

    membri_gruppi_mobili = [m for g in scenario["gruppi"] if g["mobile"] for m in g["membri"]]
    membri_gruppi_fermi = [m for g in scenario["gruppi"] if not g["mobile"] for m in g["membri"]]
    tutte_mobili = scenario["persone"] + scenario["corridori"] + membri_gruppi_mobili
    n_persone_totale = len(tutte_mobili) + len(scenario["persone_ferme"]) + len(membri_gruppi_fermi)
    celle_persone_ferme = {(pf["r"], pf["c"]) for pf in scenario["persone_ferme"]}
    celle_persone_ferme |= {(m["r"], m["c"]) for m in membri_gruppi_fermi}

    nodo_iniziale_robot = sim.cella_libera_casuale(griglia)
    stato_robot = {
        "x": nodo_iniziale_robot.cx, "y": nodo_iniziale_robot.cy, "percorso": [],
        "nodo_target": sim.cella_libera_casuale(griglia),
    }

    range_evitamento_quad = sim.RANGE_EVITAMENTO_PERSONE ** 2
    range_evitamento_robot = sim.RAGGIO_ROBOT_DEFAULT + sim.RAGGIO_PERSONA_DEFAULT

    numero_arrivi = 0
    for contatore_frame in range(num_frame):
        robot_x, robot_y = stato_robot["x"], stato_robot["y"]

        celle_bloccate_persone = {(int(p2["y"] // sim.DIM_NODO), int(p2["x"] // sim.DIM_NODO)) for p2 in tutte_mobili}
        celle_bloccate_persone |= celle_persone_ferme
        celle_bloccate_comune = celle_bloccate_persone | {(int(robot_y // sim.DIM_NODO), int(robot_x // sim.DIM_NODO))}

        ricalcolo_dovuto = contatore_frame % sim.INTERVALLO_RICALCOLO_ROBOT_FRAME == 0
        if not stato_robot["percorso"] or ricalcolo_dovuto:
            nodo_target = stato_robot["nodo_target"]
            stato_robot["percorso"] = sim.algoritmo_a_star(
                griglia, (stato_robot["x"], stato_robot["y"]), (nodo_target.r, nodo_target.c), celle_bloccate=celle_bloccate_persone)

        x, y, arrivato, bloccato = sim.passo_movimento(
            stato_robot["x"], stato_robot["y"], stato_robot["percorso"], sim.VELOCITA_ROBOT, stato_robot["nodo_target"], griglia)
        stato_robot["x"], stato_robot["y"] = x, y
        if bloccato:
            stato_robot["percorso"] = []
        if arrivato:
            numero_arrivi += 1
            stato_robot["nodo_target"] = sim.cella_libera_casuale(griglia)
            stato_robot["percorso"] = []

        _ricalcola_percorsi(tutte_mobili, contatore_frame, griglia, celle_bloccate_comune)
        griglia_spaziale = sim.costruisci_griglia_spaziale(tutte_mobili, sim.DIM_NODO)
        _muovi_e_gestisci_stato(tutte_mobili, griglia, griglia_spaziale, robot_x, robot_y, range_evitamento_quad, range_evitamento_robot)

    tempo_medio_per_arrivo = num_frame / numero_arrivi if numero_arrivi > 0 else None
    return nome_mappa, moltiplicatore, n_persone_totale, numero_arrivi, tempo_medio_per_arrivo


def _misura_mobilita_pool(argomenti):
    return misura_mobilita(*argomenti)


def verifica_mappe(mappe):
    combinazioni = [(m, k) for m in mappe for k in MOLTIPLICATORI_SWEEP]
    argomenti = [(m, k, zlib.crc32(f"{m}|{k}".encode())) for m, k in combinazioni]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Uso {n_worker} processi in parallelo su {len(combinazioni)} combinazioni ({len(mappe)} mappe x {len(MOLTIPLICATORI_SWEEP)} livelli).\n")

    risultati = {m: [] for m in mappe}
    inizio = time.time()
    completati = 0
    with mp.Pool(n_worker) as pool:
        for nome_mappa, moltiplicatore, n_persone, numero_arrivi, tempo_medio in pool.imap_unordered(_misura_mobilita_pool, argomenti):
            completati += 1
            risultati[nome_mappa].append((moltiplicatore, n_persone, numero_arrivi, tempo_medio))
            tempo_str = f"{tempo_medio:.0f} frame/arrivo" if tempo_medio else "MAI arrivato"
            print(f"[{completati}/{len(combinazioni)}] {nome_mappa} x{moltiplicatore}: {n_persone} persone, {numero_arrivi} arrivi -> {tempo_str} ({time.time() - inizio:.0f}s trascorsi)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    print("=== Tetto di densita' per mappa (soglia: tempo-per-arrivo raddoppiato rispetto alla densita' piu' bassa testata) ===")
    for nome_mappa in mappe:
        righe = sorted(risultati[nome_mappa])
        tempo_base = righe[0][3]  # tempo-per-arrivo alla densita' piu' bassa dello sweep (x0.5): riferimento "quasi senza folla"
        if tempo_base is None:
            print(f"\n{nome_mappa}: nessun arrivo nemmeno alla densita' minima testata (x{righe[0][0]}) - mappa troppo piccola/lenta per questo sweep, andrebbe guardata a parte")
            continue
        print(f"\n{nome_mappa} (riferimento a x{righe[0][0]}: {tempo_base:.0f} frame/arrivo):")
        riga_soglia = None
        for moltiplicatore, n_persone, numero_arrivi, tempo_medio in righe:
            if tempo_medio is None:
                mobilita = 0.0
            else:
                mobilita = min(1.0, tempo_base / tempo_medio)
            marcatore = "  <-- sotto soglia" if mobilita < SOGLIA_MOBILITA else ""
            print(f"  x{moltiplicatore:<4} ({n_persone:3} persone, {numero_arrivi:3} arrivi): mobilita' relativa {mobilita:.2f}{marcatore}")
            if riga_soglia is None and mobilita < SOGLIA_MOBILITA:
                riga_soglia = (moltiplicatore, n_persone)
        if riga_soglia:
            print(f"  Tetto stimato: ~{riga_soglia[1]} persone (x{riga_soglia[0]} del preset 'denso')")
        else:
            print(f"  Nessuna soglia raggiunta entro lo sweep testato (fino a x{MOLTIPLICATORI_SWEEP[-1]})")


if __name__ == "__main__":
    mappe = [sys.argv[1]] if len(sys.argv) > 1 else list(ts.MAPPE.keys())
    verifica_mappe(mappe)

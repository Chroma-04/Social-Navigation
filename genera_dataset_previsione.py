"""FASE 1 (Livello 2 della tesi - previsione): genera il dataset per allenare la correzione AI del Kalman.

Fa girare gli scenari di training_scenari.py in modalita' headless a velocita' massima (nessun rendering,
nessun limite di 60 FPS reali - si simula un frame dopo l'altro il piu' in fretta possibile), con il robot
che gira fra punti casuali della mappa per raccogliere dati da molte posizioni/angolazioni del lidar
diverse, non da un solo punto fisso.

Per ogni persona rilevata dal lidar, ad intervalli regolari registra:
  - uno storico delle ultime posizioni/velocita' stimate dal Kalman (input per il modello),
  - la previsione del Kalman a un orizzonte futuro fisso (baseline da battere),
  - la posizione reale in cui la persona si e' davvero trovata quell'orizzonte dopo (etichetta) - risolta
    "a tempo debito" leggendo la posizione vera dell'entita' quando il frame futuro viene raggiunto, non
    stimata in altro modo.

Il file di movimento (_muovi_e_gestisci_stato) replica intenzionalmente la stessa logica del blocco
"Movimento" in main.py (arrivo -> attesa -> nuova destinazione, inseguimento del leader nei gruppi):
i dati devono riflettere lo stesso comportamento che gira dal vivo, altrimenti il modello imparerebbe
pattern diversi da quelli che poi dovra' correggere.

Uso:
    py genera_dataset_previsione.py             modalita' rapida: poche combinazioni, run brevi (per verificare che funzioni)
    py genera_dataset_previsione.py --completo   tutte le combinazioni mappa/densita/mix, run piu' lunghe (raccolta vera)
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import math
import random
import time
import collections
import multiprocessing as mp
import zlib
import numpy as np
import main as sim
import training_scenari as ts

# --- parametri del dataset ---
FINESTRA_STORICO_FRAME = 30        # 0.5s di storico (posizione+velocita' filtrate dal Kalman) dato in input al modello - deve restare identica alla costante omonima in allena_previsione.py
ORIZZONTE_PREVISIONE_FRAME = sim.PREVISIONE_BLOB_FRAME  # 1s nel futuro: stesso orizzonte usato dal vivo per la macchia di probabilita'
INTERVALLO_CAMPIONAMENTO_FRAME = 5  # non registra una previsione ad ogni singolo frame per persona: frame consecutivi sono quasi identici

FRAME_PER_SCENARIO = {"rapido": 900, "completo": 3600}  # 15s / 60s di simulazione per combinazione
COMBINAZIONI_RAPIDO = {"mappe": ["aperta", "corridoio", "porta_stretta"], "densita": ["rado", "medio"], "mix": ["misto"]}

CARTELLA_DATASET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")


def _nuovo_target(griglia):
    return sim.cella_libera_casuale(griglia)


def _aggiorna_robot(stato, griglia):
    """Il robot gira fra punti casuali della mappa: qui non e' soggetto di studio, serve solo a dare
    al lidar molte posizioni/angolazioni diverse da cui osservare le persone - nessuna logica di
    costo/probabilita', solo A* diretto verso il prossimo punto."""
    if not stato["percorso"]:
        nodo = _nuovo_target(griglia)
        stato["nodo_target"] = nodo
        stato["percorso"] = sim.algoritmo_a_star(griglia, (stato["x"], stato["y"]), (nodo.r, nodo.c))
    x, y, arrivato, bloccato = sim.passo_movimento(stato["x"], stato["y"], stato["percorso"], sim.VELOCITA_ROBOT, stato["nodo_target"], griglia)
    stato["x"], stato["y"] = x, y
    if bloccato or arrivato:
        stato["percorso"] = []


def _ricalcola_percorsi(tutte_mobili, contatore_frame, griglia, celle_bloccate_comune):
    for p in tutte_mobili:
        if p["stato"] != "movimento":
            continue
        dovuto = (contatore_frame + p["offset_ricalcolo"]) % sim.INTERVALLO_RICALCOLO_PERSONE_FRAME == 0
        if p["percorso"] is None and not dovuto:
            continue
        if p["percorso"] and not dovuto:
            continue
        p["percorso"] = sim.algoritmo_a_star(
            griglia, (p["x"], p["y"]), p["target"], rumore_seed=p["rumore_seed"], celle_bloccate=celle_bloccate_comune)


def _muovi_e_gestisci_stato(tutte_mobili, griglia, griglia_spaziale, robot_x, robot_y,
                             range_evitamento_quad, range_evitamento_robot):
    """Replica del blocco 'Movimento' di main.py: stessa logica di arrivo/attesa/nuova destinazione e di
    inseguimento del leader nei gruppi, per non generare dati che si comportano diversamente da come si
    comportera' poi il sistema dal vivo."""
    for p in tutte_mobili:
        if p["stato"] == "movimento":
            if p["percorso"] is None:
                continue
            nodo_target_p = griglia[p["target"][0]][p["target"][1]]
            velocita_p = sim.VELOCITA_PERSONA * p["fattore_velocita"]
            non_capofila = "gruppo" in p and not p.get("capofila")
            if non_capofila:
                leader = p["gruppo"]["membri"][0]
                distanza_leader = math.hypot(p["x"] - leader["x"], p["y"] - leader["y"])
                if distanza_leader > sim.GRUPPO_DISTANZA_COESIONE_PX:
                    velocita_p *= min(sim.GRUPPO_FATTORE_RINCORSA_MAX, 1.0 + (distanza_leader - sim.GRUPPO_DISTANZA_COESIONE_PX) / sim.GRUPPO_DISTANZA_COESIONE_PX)
            x_prima, y_prima = p["x"], p["y"]
            p["x"], p["y"], arrivata, passo_muro = sim.passo_movimento(p["x"], p["y"], p["percorso"], velocita_p, nodo_target_p, griglia)
            if passo_muro:
                p["percorso"] = []

            rep_x, rep_y = sim.repulsione_vicini(p, sim.vicini_spaziali(p, griglia_spaziale, sim.DIM_NODO), range_evitamento_quad)
            p["x"], p["y"] = sim.sposta_con_vettore(p["x"], p["y"], rep_x, rep_y, sim.FORZA_REPULSIONE_PERSONE, griglia)

            rrep_x, rrep_y = sim.repulsione_punto(p, robot_x, robot_y, range_evitamento_robot)
            p["x"], p["y"] = sim.sposta_con_vettore(p["x"], p["y"], rrep_x, rrep_y, sim.FORZA_REPULSIONE_ROBOT, griglia)

            if non_capofila:
                spostamento = math.hypot(p["x"] - x_prima, p["y"] - y_prima)
                p["frame_fermo"] = 0 if spostamento > 0.3 else p.get("frame_fermo", 0) + 1
                leader = p["gruppo"]["membri"][0]
                distanza_leader = math.hypot(p["x"] - leader["x"], p["y"] - leader["y"])
                deve_inseguire_leader = (
                    distanza_leader > sim.GRUPPO_DISTANZA_MAX_DIVERGENZA_PX
                    or (p["frame_fermo"] > sim.GRUPPO_FRAME_BLOCCO_SOGLIA and distanza_leader > sim.GRUPPO_DISTANZA_COESIONE_PX)
                )
                if deve_inseguire_leader and not p.get("inseguendo_leader"):
                    p["percorso"] = []
                if deve_inseguire_leader or p.get("inseguendo_leader"):
                    if distanza_leader <= sim.GRUPPO_DISTANZA_COESIONE_PX:
                        p["inseguendo_leader"] = False
                        p["target"] = p["gruppo"]["target_comune"]
                        p["percorso"] = []
                    else:
                        p["inseguendo_leader"] = True
                        p["frame_fermo"] = 0
                        p["target"] = (max(0, min(sim.Y_TOT - 1, int(leader["y"] // sim.DIM_NODO))),
                                       max(0, min(sim.X_TOT - 1, int(leader["x"] // sim.DIM_NODO))))

            if arrivata and not (non_capofila and p.get("inseguendo_leader")):
                p["stato"] = "attesa"
                p["attesa_timer"] = random.randint(sim.ATTESA_PERSONA_MIN_FRAME, sim.ATTESA_PERSONA_MAX_FRAME)

        elif p["stato"] == "attesa":
            rep_x, rep_y = sim.repulsione_vicini(p, sim.vicini_spaziali(p, griglia_spaziale, sim.DIM_NODO), range_evitamento_quad)
            p["x"], p["y"] = sim.sposta_con_vettore(p["x"], p["y"], rep_x, rep_y, sim.FORZA_REPULSIONE_PERSONE, griglia)

            rrep_x, rrep_y = sim.repulsione_punto(p, robot_x, robot_y, range_evitamento_robot)
            p["x"], p["y"] = sim.sposta_con_vettore(p["x"], p["y"], rrep_x, rrep_y, sim.FORZA_REPULSIONE_ROBOT, griglia)

            p["attesa_timer"] -= 1
            if p["attesa_timer"] <= 0:
                if "gruppo" in p and not p.get("capofila"):
                    p["attesa_timer"] = random.randint(sim.ATTESA_PERSONA_MIN_FRAME, sim.ATTESA_PERSONA_MAX_FRAME)
                    continue
                if random.random() < sim.PROBABILITA_PERCORSO_BREVE:
                    nuovo_nodo_target = sim.cella_libera_vicina(griglia, p["x"], p["y"], sim.RAGGIO_PERCORSO_BREVE)
                else:
                    nuovo_nodo_target = sim.cella_libera_casuale(griglia)
                nuovo_target = (nuovo_nodo_target.r, nuovo_nodo_target.c)
                p["target"] = nuovo_target
                p["percorso"] = []
                p["stato"] = "movimento"
                if p.get("capofila"):
                    p["gruppo"]["target_comune"] = nuovo_target
                    for compagno in p["gruppo"]["membri"]:
                        if compagno is not p:
                            compagno["target"] = nuovo_target
                            compagno["percorso"] = []
                            compagno["stato"] = "movimento"
                            compagno["inseguendo_leader"] = False


def _rileva_e_traccia(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
    """Stesso rilevamento lidar + tracciamento Kalman di main.py: nessun 'coasting', una traccia persa
    (fuori raggio o dietro un muro) viene abbandonata subito, non tenuta in vita a occhio."""
    id_rilevati_ora = set()
    for p in tutte_le_persone:
        d = math.hypot(p["x"] - robot_x, p["y"] - robot_y)
        if sim.RAGGIO_LIDAR_DEFAULT > 0 and d < sim.RAGGIO_LIDAR_DEFAULT and sim.linea_di_vista_libera(griglia, robot_x, robot_y, p["x"], p["y"]):
            angolo_rumore = random.uniform(0, 2 * math.pi)
            raggio_rumore = random.uniform(0, sim.RUMORE_LIDAR_PX)
            xr = p["x"] + raggio_rumore * math.cos(angolo_rumore)
            yr = p["y"] + raggio_rumore * math.sin(angolo_rumore)
            pid = id(p)
            id_rilevati_ora.add(pid)
            if pid not in tracciamento_lidar:
                tracciamento_lidar[pid] = sim.nuova_traccia_kalman(xr, yr)
                storico_posizioni[pid] = collections.deque(maxlen=FINESTRA_STORICO_FRAME)
            else:
                sim.aggiorna_traccia_kalman(tracciamento_lidar[pid], xr, yr)
            traccia = tracciamento_lidar[pid]
            traccia["ferma"] = p.get("stato", "attesa") == "attesa"
            storico_posizioni[pid].append((traccia["x"], traccia["y"], traccia["vx"], traccia["vy"]))

    for pid_vecchio in list(tracciamento_lidar.keys()):
        if pid_vecchio not in id_rilevati_ora:
            del tracciamento_lidar[pid_vecchio]
            storico_posizioni.pop(pid_vecchio, None)


def _registra_previsioni(tracciamento_lidar, storico_posizioni, contatore_frame, previsioni_in_sospeso):
    if contatore_frame % INTERVALLO_CAMPIONAMENTO_FRAME != 0:
        return
    for pid, traccia in tracciamento_lidar.items():
        storico = storico_posizioni[pid]
        if len(storico) < FINESTRA_STORICO_FRAME:
            continue  # non ha ancora abbastanza storico continuo (traccia troppo giovane)
        pred_x, pred_y = sim.previsione_posizione_kalman(traccia, ORIZZONTE_PREVISIONE_FRAME)
        previsioni_in_sospeso.append({
            "frame_maturazione": contatore_frame + ORIZZONTE_PREVISIONE_FRAME,
            "pid": pid,
            "storico": np.array(storico, dtype=np.float32),
            "pred": (pred_x, pred_y),
        })


def _risolvi_previsioni_mature(previsioni_in_sospeso, contatore_frame, entita_per_pid, campioni, etichetta_scenario):
    while previsioni_in_sospeso and previsioni_in_sospeso[0]["frame_maturazione"] <= contatore_frame:
        pending = previsioni_in_sospeso.popleft()
        entita = entita_per_pid.get(pending["pid"])
        if entita is None:
            continue
        campioni["storico"].append(pending["storico"])
        campioni["pred_kalman"].append(pending["pred"])
        campioni["verita"].append((entita["x"], entita["y"]))
        campioni["scenario"].append(etichetta_scenario)


def esegui_scenario(nome_mappa, nome_densita, nome_mix, num_frame, seed):
    """Fa girare un'unica combinazione mappa/densita/mix e restituisce i propri campioni (invece di
    scriverli su una struttura condivisa): cosi' la funzione e' autonoma e puo' girare in un processo
    separato del pool, senza bisogno di sincronizzazione fra scenari diversi (sono comunque indipendenti
    l'uno dall'altro, nessun dato va condiviso durante il calcolo)."""
    campioni = {"storico": [], "pred_kalman": [], "verita": [], "scenario": []}
    scenario = ts.costruisci_scenario(nome_mappa, nome_densita, nome_mix, seed=seed)
    griglia = scenario["griglia"]

    membri_gruppi_mobili = [m for g in scenario["gruppi"] if g["mobile"] for m in g["membri"]]
    membri_gruppi_fermi = [m for g in scenario["gruppi"] if not g["mobile"] for m in g["membri"]]
    tutte_mobili = scenario["persone"] + scenario["corridori"] + membri_gruppi_mobili
    tutte_le_persone = tutte_mobili + scenario["persone_ferme"] + membri_gruppi_fermi
    entita_per_pid = {id(p): p for p in tutte_le_persone}

    nodo_iniziale_robot = sim.cella_libera_casuale(griglia)
    stato_robot = {"x": nodo_iniziale_robot.cx, "y": nodo_iniziale_robot.cy, "percorso": [], "nodo_target": nodo_iniziale_robot}

    tracciamento_lidar, storico_posizioni = {}, {}
    previsioni_in_sospeso = collections.deque()
    etichetta_scenario = f"{nome_mappa}|{nome_densita}|{nome_mix}"

    range_evitamento_quad = sim.RANGE_EVITAMENTO_PERSONE ** 2
    range_evitamento_robot = sim.RAGGIO_ROBOT_DEFAULT + sim.RAGGIO_PERSONA_DEFAULT

    for contatore_frame in range(num_frame):
        _aggiorna_robot(stato_robot, griglia)
        robot_x, robot_y = stato_robot["x"], stato_robot["y"]

        celle_persone_ferme = {(pf["r"], pf["c"]) for pf in scenario["persone_ferme"]}
        celle_persone_ferme |= {(m["r"], m["c"]) for m in membri_gruppi_fermi}
        celle_bloccate_comune = {(int(p2["y"] // sim.DIM_NODO), int(p2["x"] // sim.DIM_NODO)) for p2 in tutte_mobili}
        celle_bloccate_comune |= celle_persone_ferme
        celle_bloccate_comune.add((int(robot_y // sim.DIM_NODO), int(robot_x // sim.DIM_NODO)))

        _ricalcola_percorsi(tutte_mobili, contatore_frame, griglia, celle_bloccate_comune)

        griglia_spaziale = sim.costruisci_griglia_spaziale(tutte_mobili, sim.DIM_NODO)
        _muovi_e_gestisci_stato(tutte_mobili, griglia, griglia_spaziale, robot_x, robot_y, range_evitamento_quad, range_evitamento_robot)

        _rileva_e_traccia(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)
        _registra_previsioni(tracciamento_lidar, storico_posizioni, contatore_frame, previsioni_in_sospeso)
        _risolvi_previsioni_mature(previsioni_in_sospeso, contatore_frame, entita_per_pid, campioni, etichetta_scenario)

    return nome_mappa, nome_densita, nome_mix, len(tutte_le_persone), campioni


def _esegui_scenario_pool(argomenti):
    """Adatta esegui_scenario alla firma a un solo argomento richiesta da Pool.imap_unordered."""
    return esegui_scenario(*argomenti)


def genera_dataset(mappe, densita_livelli, mix_livelli, num_frame, file_output, n_worker=None):
    """Dispaccia ogni combinazione mappa/densita/mix come un task indipendente su un pool di processi
    (ognuno gira su un core diverso della CPU): sono scenari completamente indipendenti fra loro, quindi
    si parallelizzano senza nessuna sincronizzazione, con uno speedup vicino al numero di core disponibili
    invece di girare una combinazione alla volta in sequenza."""
    campioni = {"storico": [], "pred_kalman": [], "verita": [], "scenario": []}
    combinazioni = [(m, d, x) for m in mappe for d in densita_livelli for x in mix_livelli]
    # zlib.crc32 invece di hash(): hash() su stringhe e' randomizzato ad ogni avvio di Python (PYTHONHASHSEED),
    # darebbe scenari diversi ad ogni run pur passando lo stesso seed nominale - crc32 e' deterministico
    argomenti = [(m, d, x, num_frame, zlib.crc32(f"{m}|{d}|{x}".encode())) for m, d, x in combinazioni]

    n_worker = n_worker or min(8, os.cpu_count() or 4)
    print(f"Uso {n_worker} processi in parallelo su {len(combinazioni)} combinazioni.\n")

    inizio = time.time()
    completati = 0
    with mp.Pool(n_worker) as pool:
        for nome_mappa, nome_densita, nome_mix, n_persone, campioni_scenario in pool.imap_unordered(_esegui_scenario_pool, argomenti):
            completati += 1
            for chiave in campioni:
                campioni[chiave].extend(campioni_scenario[chiave])
            print(f"[{completati}/{len(combinazioni)}] {nome_mappa}/{nome_densita}/{nome_mix}: {n_persone} persone, "
                  f"{num_frame} frame -> {len(campioni['verita'])} campioni totali finora "
                  f"({time.time() - inizio:.0f}s trascorsi)")

    durata = time.time() - inizio
    n_campioni = len(campioni["verita"])
    print(f"\nGenerazione completata in {durata:.1f}s, {n_campioni} campioni totali.")
    if n_campioni == 0:
        print("Nessun campione raccolto (nessuna persona rilevata abbastanza a lungo): niente da salvare.")
        return

    pred = np.array(campioni["pred_kalman"], dtype=np.float32)
    verita = np.array(campioni["verita"], dtype=np.float32)
    errore_kalman_px = np.linalg.norm(pred - verita, axis=1)
    print(f"Baseline Kalman (senza correzione): errore medio {errore_kalman_px.mean():.1f}px, "
          f"mediano {np.median(errore_kalman_px):.1f}px, 90-esimo percentile {np.percentile(errore_kalman_px, 90):.1f}px")

    os.makedirs(CARTELLA_DATASET, exist_ok=True)
    path_output = os.path.join(CARTELLA_DATASET, file_output)
    np.savez_compressed(
        path_output,
        storico=np.array(campioni["storico"], dtype=np.float32),
        pred_kalman=pred,
        verita=verita,
        scenario=np.array(campioni["scenario"], dtype="U64"),
    )
    print(f"Dataset salvato in: {path_output}")


if __name__ == "__main__":
    modalita_completa = "--completo" in sys.argv
    if modalita_completa:
        mappe = list(ts.MAPPE.keys())
        densita_livelli = list(ts.LIVELLI_DENSITA.keys())
        mix_livelli = list(ts.MIX_COMPORTAMENTALE.keys())
        num_frame = FRAME_PER_SCENARIO["completo"]
        file_output = "dataset_previsione.npz"
    else:
        mappe = COMBINAZIONI_RAPIDO["mappe"]
        densita_livelli = COMBINAZIONI_RAPIDO["densita"]
        mix_livelli = COMBINAZIONI_RAPIDO["mix"]
        num_frame = FRAME_PER_SCENARIO["rapido"]
        file_output = "dataset_previsione_rapido.npz"
        print("Modalita' rapida (poche combinazioni, run brevi) - usa --completo per il dataset vero\n")
    genera_dataset(mappe, densita_livelli, mix_livelli, num_frame, file_output)

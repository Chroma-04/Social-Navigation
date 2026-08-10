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
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # la radice del progetto: main.py e training_scenari.py stanno li'
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
# ACCELERAZIONE: deve restare identica a testing.ACCELERAZIONE. Un frame vale N frame "normali" (robot e
# persone percorrono N volte la distanza, gli orizzonti espressi in frame si dividono per N), quindi lo
# stesso scenario costa N volte meno calcolo. Il modello impara la dinamica a QUESTO passo temporale:
# usarlo in inferenza a un passo diverso lo manda fuori distribuzione (prepara_input gli passa posizioni
# relative e velocita' grezze, che scalano entrambe con l'accelerazione), per questo dataset, training e
# testing devono condividere lo stesso valore.
ACCELERAZIONE = 5
FINESTRA_STORICO_FRAME = 30        # storico (posizione+velocita' filtrate dal Kalman) dato in input al modello - deve restare identica alla costante omonima in allena_previsione.py. NON si divide per l'accelerazione: e' la forma dell'input della rete, non un orizzonte temporale
ORIZZONTE_PREVISIONE_FRAME = max(1, sim.PREVISIONE_BLOB_FRAME // ACCELERAZIONE)  # stesso orizzonte usato dal vivo per la macchia di probabilita', riscalato dall'accelerazione
INTERVALLO_CAMPIONAMENTO_FRAME = max(1, 5 // ACCELERAZIONE)  # non registra una previsione ad ogni singolo frame per persona: frame consecutivi sono quasi identici (a passo accelerato lo sono meno, quindi si campiona piu' spesso)

FRAME_PER_SCENARIO = {"rapido": max(1, 900 // ACCELERAZIONE), "completo": max(1, 3600 // ACCELERAZIONE)}  # 15s / 60s equivalenti di simulazione per combinazione
COMBINAZIONI_RAPIDO = {"mappe": ["aperta", "corridoio", "porta_stretta"], "densita": ["rado", "medio"], "mix": ["misto"]}

# --- MODALITA' SPECIALIZZATA: sweep di densita' su un solo ambiente ---
# I preset rado/medio/denso di training_scenari (7/22/40 individui sulla mappa di riferimento) coprivano il
# caso generico, ma il robot viene poi provato con centinaia di persone in scena. A 40 individui la gente
# cammina quasi dritta e il Kalman da solo basta gia': e' ad alta densita' che le persone si deviano di
# continuo a vicenda e nascono i residui non lineari che il modello deve imparare a correggere. Qui si
# spazza l'intero intervallo fino a POPOLAZIONE_MAX, cosi' il modello vede sia le densita' basse sia
# quelle a cui viene poi testato, invece di essere allenato lontano dal punto di lavoro.
POPOLAZIONE_MIN = 50   # sotto questa soglia non c'e' quasi nessuno da tracciare: scenari che non producono campioni utili (a 0 persone, letteralmente zero)
POPOLAZIONE_MAX = 550  # copre la configurazione di esercizio (565 individui: 120/100/170/50) senza spingersi oltre, dove il costo di calcolo cresce molto e il beneficio dell'AI tende comunque a saturare
POPOLAZIONE_PASSO = 50
# Proporzioni della folla: persone normali / corridori / ferme / membri di gruppo. I gruppi non entrano in
# 'budget_individui' (sono entita' composte, contate a parte), quindi la loro quota si ottiene tarando
# numero_gruppi_base - vedi _densita_per_popolazione.
#
# Ricavate dalla configurazione reale in cui viene usato il simulatore (pannello tecnico: 120 persone,
# 100 corridori, 170 ferme, 50 gruppi): i 50 gruppi portano ~175 membri, quindi il totale e' 565 e i
# membri di gruppo sono quasi un terzo della folla, non un quinto. Allenare su proporzioni diverse da
# quelle di esercizio sarebbe uno svantaggio gratuito, visto che proprio i membri di gruppo (che
# rincorrono il capofila accelerando fino a GRUPPO_FATTORE_RINCORSA_MAX) sono i casi su cui il Kalman
# sbaglia di piu' e su cui la correzione AI ha piu' da guadagnare.
QUOTA_NORMALI, QUOTA_CORRIDORI, QUOTA_FERME, QUOTA_GRUPPI = 0.212, 0.177, 0.301, 0.310
MIX_SPECIALIZZATO = {
    "prop_normali": QUOTA_NORMALI, "prop_corridori": QUOTA_CORRIDORI, "prop_ferme": QUOTA_FERME,
    "moltiplicatore_gruppi": 1.0,
}


def popolazione_da_totale(totale):
    """Conteggi assoluti (persone/corridori/ferme/gruppi) per una folla di 'totale' individui con le quote
    QUOTA_*. E' la stessa ricetta usata per generare il dataset, riesposta in forma di conteggi diretti
    cosi' che testing.py possa costruire la stessa folla ad ogni livello di densita' dello sweep: training
    e valutazione restano sulla stessa distribuzione, invece di allenare su una composizione e misurare
    su un'altra."""
    return {
        "numero_persone": max(0, round(totale * QUOTA_NORMALI)),
        "numero_corridori": max(0, round(totale * QUOTA_CORRIDORI)),
        "numero_persone_ferme": max(0, round(totale * QUOTA_FERME)),
        "numero_gruppi": max(0, round(totale * QUOTA_GRUPPI / MEMBRI_MEDI_PER_GRUPPO)),
    }
MEMBRI_MEDI_PER_GRUPPO = sim.GRUPPO_MEMBRI_MEDIA


def _fattore_area(nome_mappa):
    """Rapporto fra l'area libera della mappa e CELLE_LIBERE_RIFERIMENTO: e' lo stesso fattore con cui
    costruisci_scenario scala i budget, quindi va invertito qui per centrare una popolazione ASSOLUTA."""
    griglia = [[sim.Nodo(r, c) for c in range(sim.X_TOT)] for r in range(sim.Y_TOT)]
    sim.crea_bordi(griglia)
    costruttore = ts.MAPPE.get(nome_mappa) or ts.MAPPE_TEST.get(nome_mappa)
    costruttore(griglia)
    celle_libere = sum(1 for riga in griglia for n in riga if n.tipo == "libero")
    return celle_libere / ts.CELLE_LIBERE_RIFERIMENTO


def _densita_per_popolazione(popolazione_totale, fattore_area):
    """Dizionario di densita' che produce circa 'popolazione_totale' individui sulla mappa data, con le
    quote di QUOTA_*. costruisci_scenario moltiplica per fattore_area, quindi qui si divide."""
    budget_individui = popolazione_totale * (QUOTA_NORMALI + QUOTA_CORRIDORI + QUOTA_FERME) / fattore_area
    gruppi = popolazione_totale * QUOTA_GRUPPI / MEMBRI_MEDI_PER_GRUPPO / fattore_area
    return {"budget_individui": round(budget_individui), "numero_gruppi_base": round(gruppi)}

CARTELLA_DATASET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")
_FRAME_REAZIONE_ORIGINALE = sim.FRAME_REAZIONE_VELOCITA_ROBOT  # valore di main.py prima della riscalatura per accelerazione: si riparte sempre da qui invece di dividere ripetutamente lo stesso modulo


def _nuovo_target(griglia):
    return sim.cella_libera_casuale(griglia)


def _aggiorna_robot(stato, griglia, tracciamento_lidar, moltiplicatore_velocita=ACCELERAZIONE):
    """Il robot gira fra punti casuali della mappa: qui il PERCORSO non e' soggetto di studio (serve solo
    a dare al lidar molte posizioni/angolazioni diverse da cui osservare le persone), quindi resta A*
    diretto senza costo di probabilita'/AI. La VELOCITA' pero' replica il controllo predittivo di main.py
    (stessa fattore_velocita_da_conflitto): la velocita' del robot cambia dove si trova nello spazio nel
    tempo, e quindi quando/quanto spinge via le persone vicine (repulsione_punto) - un robot che si muove
    diversamente da quello vero e' un'altra fonte di scarto fra i dati di training e il comportamento dal
    vivo, lo stesso principio per cui _muovi_e_gestisci_stato replica il blocco 'Movimento' di main.py."""
    if not stato["percorso"]:
        nodo = _nuovo_target(griglia)
        stato["nodo_target"] = nodo
        stato["percorso"] = sim.algoritmo_a_star(griglia, (stato["x"], stato["y"]), (nodo.r, nodo.c))
    velocita_nominale = sim.VELOCITA_ROBOT * moltiplicatore_velocita
    fattore = sim.fattore_velocita_da_conflitto(
        stato["x"], stato["y"], stato["percorso"], tracciamento_lidar, velocita_nominale,
        sim.RAGGIO_ROBOT_DEFAULT, sim.RAGGIO_PERSONA_DEFAULT)
    x, y, arrivato, bloccato = sim.passo_movimento(stato["x"], stato["y"], stato["percorso"], velocita_nominale * fattore, stato["nodo_target"], griglia)
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


def esegui_scenario(nome_mappa, nome_densita, nome_mix, num_frame, seed, valore_densita=None, etichetta_extra="", valore_mix=None):
    """Fa girare un'unica combinazione mappa/densita/mix e restituisce i propri campioni (invece di
    scriverli su una struttura condivisa): cosi' la funzione e' autonoma e puo' girare in un processo
    separato del pool, senza bisogno di sincronizzazione fra scenari diversi (sono comunque indipendenti
    l'uno dall'altro, nessun dato va condiviso durante il calcolo).

    'valore_densita' permette di passare un dizionario di densita' grezzo (vedi DENSITA_ALTE) tenendo
    'nome_densita' come sola etichetta leggibile; se None, 'nome_densita' viene usato anche come chiave in
    LIVELLI_DENSITA. 'etichetta_extra' distingue fra loro le ripetizioni della stessa combinazione, che
    hanno seed diversi: l'etichetta di scenario e' l'unita' su cui allena_previsione.py divide train/test,
    quindi ripetizioni diverse devono avere etichette diverse per non finire spezzate a meta'."""
    campioni = {"storico": [], "pred_kalman": [], "verita": [], "scenario": []}
    densita_effettiva = nome_densita if valore_densita is None else valore_densita
    mix_effettivo = nome_mix if valore_mix is None else valore_mix
    scenario = ts.costruisci_scenario(nome_mappa, densita_effettiva, mix_effettivo, seed=seed)
    griglia = scenario["griglia"]

    membri_gruppi_mobili = [m for g in scenario["gruppi"] if g["mobile"] for m in g["membri"]]
    membri_gruppi_fermi = [m for g in scenario["gruppi"] if not g["mobile"] for m in g["membri"]]
    tutte_mobili = scenario["persone"] + scenario["corridori"] + membri_gruppi_mobili
    tutte_le_persone = tutte_mobili + scenario["persone_ferme"] + membri_gruppi_fermi
    entita_per_pid = {id(p): p for p in tutte_le_persone}
    # accelerazione: le persone coprono ACCELERAZIONE volte la distanza per frame (stesso meccanismo usato
    # in testing.py, moltiplicando fattore_velocita invece di toccare _muovi_e_gestisci_stato)
    for p in tutte_mobili:
        p["fattore_velocita"] *= ACCELERAZIONE
    sim.FRAME_REAZIONE_VELOCITA_ROBOT = max(1, _FRAME_REAZIONE_ORIGINALE // ACCELERAZIONE)

    nodo_iniziale_robot = sim.cella_libera_casuale(griglia)
    stato_robot = {"x": nodo_iniziale_robot.cx, "y": nodo_iniziale_robot.cy, "percorso": [], "nodo_target": nodo_iniziale_robot}

    tracciamento_lidar, storico_posizioni = {}, {}
    previsioni_in_sospeso = collections.deque()
    etichetta_scenario = f"{nome_mappa}|{nome_densita}|{nome_mix}{etichetta_extra}"

    range_evitamento_quad = sim.RANGE_EVITAMENTO_PERSONE ** 2
    range_evitamento_robot = sim.RAGGIO_ROBOT_DEFAULT + sim.RAGGIO_PERSONA_DEFAULT

    for contatore_frame in range(num_frame):
        # posizione del robot dall'inizio del frame (= fine del frame precedente): usata per rilevamento/
        # repulsione di questo frame, esattamente come in main.py dove il rilevamento lidar precede il
        # blocco Movimento del robot - il robot si sposta per ultimo, usando le tracce appena aggiornate
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

        _aggiorna_robot(stato_robot, griglia, tracciamento_lidar)

    return nome_mappa, nome_densita, nome_mix, len(tutte_le_persone), campioni


def _esegui_scenario_pool(argomenti):
    """Adatta esegui_scenario alla firma a un solo argomento richiesta da Pool.imap_unordered."""
    return esegui_scenario(*argomenti)


def genera_dataset(mappe, densita_livelli, mix_livelli, num_frame, file_output, n_worker=None, ripetizioni=1):
    """Dispaccia ogni combinazione mappa/densita/mix come un task indipendente su un pool di processi
    (ognuno gira su un core diverso della CPU): sono scenari completamente indipendenti fra loro, quindi
    si parallelizzano senza nessuna sincronizzazione, con uno speedup vicino al numero di core disponibili
    invece di girare una combinazione alla volta in sequenza.

    'ripetizioni' fa girare ogni combinazione piu' volte con seed diversi: le persone nascono in punti
    diversi e scelgono destinazioni diverse, quindi ogni ripetizione produce traiettorie genuinamente
    nuove pur sulla stessa mappa. E' il modo per accumulare tanti dati su POCHE mappe (allenamento
    specializzato su un ambiente noto) invece di doverne aggiungere altre.

    'densita_livelli' accetta sia nomi di LIVELLI_DENSITA sia chiavi di DENSITA_ALTE: le seconde vengono
    passate a costruisci_scenario come dizionario grezzo, tenendo il nome come sola etichetta."""
    campioni = {"storico": [], "pred_kalman": [], "verita": [], "scenario": []}
    # ogni voce di densita_livelli/mix_livelli e' o un nome (chiave di LIVELLI_DENSITA/MIX_COMPORTAMENTALE)
    # o una coppia (etichetta, dizionario grezzo): la seconda forma serve alla modalita' specializzata, che
    # usa densita' e proporzioni su misura senza aggiungerle alle tabelle condivise
    def _scomponi(voce):
        return voce if isinstance(voce, tuple) else (voce, None)

    combinazioni = [(m, d, x) for m in mappe for d in densita_livelli for x in mix_livelli]
    # zlib.crc32 invece di hash(): hash() su stringhe e' randomizzato ad ogni avvio di Python (PYTHONHASHSEED),
    # darebbe scenari diversi ad ogni run pur passando lo stesso seed nominale - crc32 e' deterministico
    argomenti = []
    for m, d, x in combinazioni:
        nome_d, valore_d = _scomponi(d)
        nome_x, valore_x = _scomponi(x)
        for i in range(ripetizioni):
            etichetta_extra = f"|r{i}" if ripetizioni > 1 else ""
            argomenti.append((m, nome_d, nome_x, num_frame,
                              zlib.crc32(f"{m}|{nome_d}|{nome_x}|{i}".encode()), valore_d, etichetta_extra, valore_x))

    n_worker = n_worker or min(8, os.cpu_count() or 4)
    print(f"Uso {n_worker} processi in parallelo su {len(argomenti)} scenari "
          f"({len(combinazioni)} combinazioni x {ripetizioni} ripetizioni).\n")

    inizio = time.time()
    completati = 0
    with mp.Pool(n_worker) as pool:
        for nome_mappa, nome_densita, nome_mix, n_persone, campioni_scenario in pool.imap_unordered(_esegui_scenario_pool, argomenti):
            completati += 1
            for chiave in campioni:
                campioni[chiave].extend(campioni_scenario[chiave])
            print(f"[{completati}/{len(argomenti)}] {nome_mappa}/{nome_densita}/{nome_mix}: {n_persone} persone, "
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


MAPPA_SPECIALIZZATO = "training_mappa_01"  # ambiente di deployment noto su cui specializzare il modello
RIPETIZIONI_SPECIALIZZATO = 10  # scenari indipendenti per ogni livello di popolazione: senza seed condiviso, ognuno genera persone e destinazioni diverse


if __name__ == "__main__":
    modalita_completa = "--completo" in sys.argv
    if "--specializzato" in sys.argv:
        # Allenamento specializzato su un singolo ambiente noto, spazzando l'intero intervallo di
        # popolazione a cui il robot verra' poi provato: niente varieta' di mappe (scelta consapevole -
        # il modello che ne esce vale per QUELL'ambiente, non e' una prova di generalizzazione ma una
        # specializzazione sul luogo di installazione), tanti scenari ripetuti con seed diversi al posto suo.
        fattore = _fattore_area(MAPPA_SPECIALIZZATO)
        popolazioni = list(range(POPOLAZIONE_MIN, POPOLAZIONE_MAX + 1, POPOLAZIONE_PASSO))
        densita_livelli = [(f"pop{p}", _densita_per_popolazione(p, fattore)) for p in popolazioni]
        mix_livelli = [("specializzato", MIX_SPECIALIZZATO)]
        num_frame = FRAME_PER_SCENARIO["completo"]
        file_output = f"dataset_previsione_specializzato_{ACCELERAZIONE}x.npz"
        print(f"Modalita' specializzata su {MAPPA_SPECIALIZZATO} (area libera {fattore:.2f}x il riferimento)")
        print(f"Popolazione da {POPOLAZIONE_MIN} a {POPOLAZIONE_MAX} a passi di {POPOLAZIONE_PASSO} "
              f"({len(popolazioni)} livelli) x {RIPETIZIONI_SPECIALIZZATO} ripetizioni, accelerazione {ACCELERAZIONE}x")
        print(f"Quote folla: {QUOTA_NORMALI:.0%} normali / {QUOTA_CORRIDORI:.0%} corridori / "
              f"{QUOTA_FERME:.0%} ferme / {QUOTA_GRUPPI:.0%} gruppi\n")
        genera_dataset([MAPPA_SPECIALIZZATO], densita_livelli, mix_livelli, num_frame, file_output,
                       ripetizioni=RIPETIZIONI_SPECIALIZZATO)
        sys.exit(0)
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

"""VALIDAZIONE APPAIATA: quanto guadagna il robot con la correzione AI + controllo predittivo di velocita'.

Confronta due robot sulla stessa identica traversata:
    "prima"  solo Kalman, velocita' costante   (il robot ante-tesi)
    "dopo"   Kalman + correzione AI + velocita' predittiva sugli incroci

DISEGNO SPERIMENTALE - numeri casuali comuni (paired test)
Ogni replica ha un seed: con lo stesso seed le due condizioni partono dalla folla IDENTICA (stesse
posizioni iniziali, stesse destinazioni, stessa composizione dei gruppi). L'unica differenza e' il robot.
Cosi' la varianza fra scenari diversi - che e' enorme, e che in un confronto non appaiato richiede
decine di repliche per essere mediata via - esce dal confronto, e si misura direttamente la differenza
attribuibile all'AI. Le traiettorie poi divergono comunque (un robot che rallenta spinge via le persone
in modo diverso), ma partire dalla stessa configurazione rende il test molto piu' sensibile a parita' di
repliche. E' la tecnica standard di riduzione della varianza nelle simulazioni.

Si riportano quindi statistiche APPAIATE: differenza media per coppia, e quante coppie vede l'AI vincere.
La media semplice dei due gruppi resta stampata per riferimento, ma e' la statistica debole.

PERCORSO
Sempre lo stesso: dalla cella libera piu' vicina all'angolo in alto a sinistra a quella piu' vicina
all'angolo in basso a destra - la diagonale completa della mappa. Un tragitto lungo attraversa piu' zone
e piu' incroci di uno breve, quindi mette alla prova la navigazione invece di misurare rumore.

STOP DI SICUREZZA
Replicato da main.py: se una persona in movimento entra nel raggio di sicurezza, il robot si ferma del
tutto per quel frame. E' il meccanismo attraverso cui il controllo predittivo di velocita' ripaga -
rallentare in anticipo serve proprio a NON finire fermi. Senza modellarlo, rallentare sarebbe solo un
costo senza beneficio possibile e la condizione "dopo" risulterebbe peggiore per costruzione. Il tempo
passato fermo e' riportato come metrica a se': se l'AI riduce gli stop ma non il tempo totale, si vede.

PASSO TEMPORALE
ACCELERAZIONE (importata da genera_dataset_previsione, per non poter divergere) alza la distanza
percorsa per frame e abbassa in proporzione gli orizzonti espressi in frame, cosi' la stessa corsa costa
N volte meno calcolo. Il modello AI e' legato al passo su cui e' stato allenato - riceve posizioni
relative e velocita' grezze, che scalano entrambe con l'accelerazione - quindi dataset, training e
validazione devono condividere lo stesso valore, e qui si carica il modello specializzato corrispondente.

Uso:
    py testing.py                    sweep completo di popolazione su training_mappa_01
    py testing.py nome_mappa         sweep su un'altra mappa di MAPPE/MAPPE_TEST
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import time
import math
import random
import csv
import zlib
import multiprocessing as mp
import numpy as np
import torch
import main as sim
import training_scenari as ts
from ai_predittiva.genera_dataset_previsione import (_ricalcola_percorsi, _muovi_e_gestisci_stato, _rileva_e_traccia,
                                       popolazione_da_totale, ACCELERAZIONE,
                                       POPOLAZIONE_MIN, POPOLAZIONE_MAX, POPOLAZIONE_PASSO)
from ai_predittiva.allena_previsione import (CorrezioneKalman, modello_da_file, prepara_input, FINESTRA_STORICO_FRAME, FILE_MODELLO,
                               FILE_MODELLO_SPECIALIZZATO, DIMENSIONE_NASCOSTA, DIMENSIONE_NASCOSTA_SPECIALIZZATO)

CARTELLA_RISULTATI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "risultati_test")
FILE_RISULTATI_CSV = os.path.join(CARTELLA_RISULTATI, "validazione.csv")
FILE_COPPIE_CSV = os.path.join(CARTELLA_RISULTATI, "validazione_coppie.csv")  # una riga per singola coppia appaiata, per rifare i conti/grafici a mano
FILE_CONFIGURAZIONE_PANNELLO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_pannello.json")

MAPPA_DEFAULT = "training_mappa_01"
# risoluzione della griglia di pianificazione del ROBOT, come multiplo di quella della folla (vedi
# FATTORE_GRIGLIA_ROBOT in main.py). Il default resta 1 - cioe' il comportamento storico - perche' e' un
# fattore sperimentale: va variato esplicitamente da chi confronta, non ereditato di nascosto, altrimenti
# le misure gia' fatte cambierebbero significato senza che si veda dal codice del confronto.
FATTORE_GRIGLIA_DEFAULT = 1
REPLICHE = 8          # gruppi appaiati (stesso seed, tutte le condizioni) per ogni livello di popolazione
# 450s equivalenti: oltre, la corsa conta come "non arrivata" invece di proseguire all'infinito.
# Il limite era 150s quando il robot viaggiava a 3,0 m/s e la traversata ne richiedeva ~50. Con la
# velocita' portata al valore realistico di 1,0 m/s i tempi triplicano e 150s li taglierebbe tutti:
# il tetto va scalato con la velocita', altrimenti smette di essere una salvaguardia contro le corse
# infinite e diventa un filtro sulle corse normali.
FRAME_MAX = max(1, 27000 // ACCELERAZIONE)

# --- DISEGNO FATTORIALE 2x2 ---
# I due componenti aggiunti dalla tesi sono indipendenti e vanno misurati separatamente: accendendoli
# insieme si ottiene un saldo unico, e se uno aiuta mentre l'altro danneggia non si puo' capire quale sia
# quale. Qui ogni combinazione gira sullo STESSO seed, quindi i confronti sono tutti appaiati fra loro.
#   nome -> (usa correzione AI sulla mappa di costo, usa controllo predittivo di velocita')
# (correzione AI, controllo di velocita', controllo di posizione, strada libera al massimo).
# Le prime quattro voci sono quelle su cui poggia il capitolo 5 e i loro nomi non vanno cambiati:
# li usano i CSV gia' prodotti
CONDIZIONI = {
    "base": (False, False, False, False),          # il robot ante-tesi: solo Kalman, velocita' costante
    "solo_ai": (True, False, False, False),        # correzione AI sulla macchia, velocita' costante
    "solo_velocita": (False, True, False, False),  # velocita' predittiva sopra al Kalman puro
    "completo": (True, True, False, False),        # AI + velocita'
    # controllo di POSIZIONE: trasla il robot di lato per aggirare le persone invece di rallentare.
    # Va misurato da solo prima che in combinazione, per non confondere i due contributi
    "solo_posizione": (False, False, True, False),
    "velocita_posizione": (False, True, True, False),
    "completo_posizione": (True, True, True, False),
    # STRADA LIBERA AL MASSIMO: il controllo di velocita' punta a non avere nessuno davanti e a
    # correre quando non c'e', invece di limitarsi a frenare. Cambia il regime neutro da 1.0 al
    # massimo della banda, quindi va confrontato con solo_velocita e non solo con base
    "velocita_libera": (False, True, False, True),
    "libera_posizione": (False, True, True, True),
}
CONDIZIONE_RIFERIMENTO = "base"
# Per la diagnosi non servono tutti gli 11 livelli: bastano pochi livelli rappresentativi con abbastanza
# repliche, perche' la domanda ("quale componente paga e quale costa") non dipende da una risoluzione fine
# in densita'. Lo sweep completo si fa dopo, sulla configurazione che risulta migliore.
LIVELLI_DIAGNOSTICI = [150, 300, 450]
# Giro esplorativo: una sola replica per livello, cinque livelli fino alla popolazione massima. Serve a
# vedere in fretta se un effetto esiste e di che segno e', non a quantificarlo: con una replica sola per
# livello non c'e' alcuna stima dell'incertezza, e la differenza fra due condizioni su un singolo seed puo'
# essere interamente dovuta a quale folla e' capitata. L'unica cosa che rende leggibile un giro cosi' e' la
# COERENZA fra i livelli: un effetto vero mantiene lo stesso segno salendo di densita', il rumore no.
LIVELLI_RAPIDI = [150, 250, 350, 450, 550]

torch.set_num_threads(1)  # stesso motivo di main.py: batch minuscoli, il multithreading di torch costa piu' di quanto renda

_FRAME_REAZIONE_ORIGINALE = sim.FRAME_REAZIONE_VELOCITA_ROBOT  # valore prima di qualsiasi riscalatura, per ripartire sempre da qui
_modello_ai_worker = None  # caricato una volta per processo worker, non ad ogni corsa


# --------------------------------------------------------------------------------------------------
# Preparazione dei worker
# --------------------------------------------------------------------------------------------------

def _inizializza_worker():
    """Inizializzatore del pool (stesso pattern di main.py): carica il modello una volta sola quando il
    worker parte, invece che ad ogni corsa. Con ACCELERAZIONE > 1 serve il modello allenato allo stesso
    passo: quello a passo normale vedrebbe input fuori distribuzione."""
    global _modello_ai_worker
    usa_specializzato = ACCELERAZIONE > 1 and os.path.exists(FILE_MODELLO_SPECIALIZZATO)
    if ACCELERAZIONE > 1 and not usa_specializzato:
        print(f"ATTENZIONE: accelerazione {ACCELERAZIONE}x ma manca "
              f"{os.path.basename(FILE_MODELLO_SPECIALIZZATO)}: uso il modello a passo normale, che a "
              f"questo passo lavora fuori distribuzione. Allenalo con: "
              f"py allena_previsione.py --specializzato")
    path = FILE_MODELLO_SPECIALIZZATO if usa_specializzato else FILE_MODELLO
    _modello_ai_worker = modello_da_file(path)


# --------------------------------------------------------------------------------------------------
# Una singola corsa
# --------------------------------------------------------------------------------------------------

def _cella_vicina_angolo(griglia, r_angolo, c_angolo):
    """Cella libera piu' vicina all'angolo indicato: gli angoli veri sono muro (bordo della mappa)."""
    libere = [n for riga in griglia for n in riga if n.tipo == "libero"]
    return min(libere, key=lambda n: (n.r - r_angolo) ** 2 + (n.c - c_angolo) ** 2)


def _mappa_costo(griglia, tracciamento_lidar, storico_posizioni, usa_ai, modello_ai, orizzonte,
                 fattore_griglia=1):
    """Ricostruisce la macchia di probabilita' come in main.py (un solo batch per la correzione AI,
    stessa ottimizzazione), sulla griglia su cui pianifica il robot. Le persone ferme non vengono
    proiettate nel futuro: restano dove sono.

    Con fattore_griglia > 1 la macchia si costruisce alla risoluzione fine del robot: il raggio in
    sottocelle e il decadimento per sottocella sono riscalati in modo che l'ellisse resti fisicamente
    IDENTICA (stessi pixel di estensione, stesso profilo di decadimento) - cambia solo quanto finemente
    e' campionata. Cosi' il confronto fra le due risoluzioni isola l'effetto della quantizzazione e non
    lo confonde con una macchia piu' piccola o piu' morbida."""
    correzioni_ai = {}
    if usa_ai:
        pid_ok, storici_ok = [], []
        for pid, traccia in tracciamento_lidar.items():
            if traccia.get("ferma", False):
                continue
            storico = storico_posizioni.get(pid)
            if storico is not None and len(storico) == FINESTRA_STORICO_FRAME:
                pid_ok.append(pid)
                storici_ok.append(storico)
        if pid_ok:
            batch = prepara_input(np.array(storici_ok, dtype=np.float32))
            with torch.no_grad():
                correzioni = modello_ai(torch.from_numpy(batch)).numpy()
            correzioni_ai = dict(zip(pid_ok, correzioni))

    dim_robot = sim.DIM_NODO / fattore_griglia
    raggio_sottocelle = sim.RAGGIO_BLOB_CELLE * fattore_griglia      # stesso raggio in pixel
    decadimento = sim.DECADIMENTO_BLOB ** (1 / fattore_griglia)      # stesso decadimento per pixel
    # costo per cella dimezzato quando le celle sono la meta': le distanze si accumulano in pixel ma i
    # costi per cella attraversata, quindi senza questo la macchia peserebbe il doppio (vedi
    # algoritmo_a_star in main.py)
    costo_massimo = sim.COSTO_MASSIMO_PROBABILITA / fattore_griglia
    righe, colonne = sim.Y_TOT * fattore_griglia, sim.X_TOT * fattore_griglia

    costi = {}
    for pid, traccia in tracciamento_lidar.items():
        e_ferma = traccia.get("ferma", False)
        frame_futuri = 0 if e_ferma else orizzonte
        direzione = None if e_ferma else (traccia["vx"], traccia["vy"])
        x_prev, y_prev = sim.previsione_posizione_kalman(traccia, frame_futuri)
        correzione = correzioni_ai.get(pid)
        if correzione is not None:
            x_prev += float(correzione[0])
            y_prev += float(correzione[1])
        rf = max(0, min(righe - 1, int(y_prev // dim_robot)))
        cf = max(0, min(colonne - 1, int(x_prev // dim_robot)))
        macchia = sim.macchia_diffusione(griglia, rf, cf, raggio_sottocelle, decadimento, fattore_griglia,
                                         direzione=direzione)
        for cella, peso in macchia.items():
            costo = peso * costo_massimo
            if cella not in costi or costi[cella] < costo:
                costi[cella] = costo
    return costi


def esegui_corsa(nome_mappa, condizione, popolazione, seed, fattore_griglia=FATTORE_GRIGLIA_DEFAULT):
    """Una traversata angolo->angolo. 'condizione': 'prima' = solo Kalman a velocita' costante,
    'dopo' = Kalman + correzione AI + velocita' predittiva. Con lo stesso 'seed' la folla iniziale e'
    identica fra le due condizioni (vedi il disegno sperimentale in testa al file).

    'fattore_griglia' e' la risoluzione della griglia su cui pianifica il ROBOT, come multiplo di quella
    della folla (toggle "Griglia fine robot" del pannello). La folla resta sempre su DIM_NODO.

    Ritorna (arrivato, frame_impiegati, frame_fermo_sicurezza); frame_impiegati vale FRAME_MAX se non e'
    arrivato entro il tetto."""
    usa_ai, usa_velocita, usa_posizione, strada_libera = CONDIZIONI[condizione]
    if usa_ai and _modello_ai_worker is None:
        _inizializza_worker()  # chiamata diretta fuori dal pool (test manuali): l'inizializzatore non e' mai girato
    modello_ai = _modello_ai_worker if usa_ai else None

    random.seed(seed)

    # --- ambiente e folla ---
    griglia = ts.costruisci_griglia(nome_mappa)

    persone = [sim.crea_persona(griglia) for _ in range(popolazione["numero_persone"])]
    corridori = [sim.crea_corridore(griglia) for _ in range(popolazione["numero_corridori"])]
    persone_ferme = [sim.crea_persona_ferma(griglia) for _ in range(popolazione["numero_persone_ferme"])]
    gruppi = [sim.crea_gruppo(griglia) for _ in range(popolazione["numero_gruppi"])]
    raggio_sicurezza = popolazione.get("raggio_sicurezza", sim.RAGGIO_SICUREZZA_DEFAULT)
    raggio_robot = popolazione.get("raggio_robot", sim.RAGGIO_ROBOT_DEFAULT)
    raggio_persona = popolazione.get("raggio_persona", sim.RAGGIO_PERSONA_DEFAULT)
    # _rileva_e_traccia legge RAGGIO_LIDAR_DEFAULT dal modulo (codice condiviso): si riscrive il valore
    # in questo processo di test, senza modificare main.py - i worker sono processi separati
    sim.RAGGIO_LIDAR_DEFAULT = popolazione.get("raggio_lidar", sim.RAGGIO_LIDAR_DEFAULT)

    membri_mobili = [m for g in gruppi if g["mobile"] for m in g["membri"]]
    membri_fermi = [m for g in gruppi if not g["mobile"] for m in g["membri"]]
    tutte_mobili = persone + corridori + membri_mobili
    tutte_le_persone = tutte_mobili + persone_ferme + membri_fermi
    celle_ferme = {(pf["r"], pf["c"]) for pf in persone_ferme}
    celle_ferme |= {(m["r"], m["c"]) for m in membri_fermi}

    # accelerazione: come main.py col cursore "Velocita simulazione", ma applicata moltiplicando
    # fattore_velocita invece di toccare _muovi_e_gestisci_stato (che e' codice condiviso)
    for p in tutte_mobili:
        p["fattore_velocita"] *= ACCELERAZIONE

    # orizzonti in frame, riscalati: a passo accelerato "un secondo nel futuro" sono meno frame. Senza
    # questa divisione la previsione proietterebbe le persone ACCELERAZIONE volte troppo avanti, fuori
    # mappa, e la macchia di probabilita' diventerebbe rumore per ENTRAMBE le condizioni
    orizzonte_previsione = max(1, sim.PREVISIONE_BLOB_FRAME // ACCELERAZIONE)
    intervallo_ricalcolo = max(1, sim.INTERVALLO_RICALCOLO_ROBOT_FRAME // ACCELERAZIONE)
    intervallo_macchia = max(1, sim.INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME // ACCELERAZIONE)
    # letto dentro fattore_velocita_da_conflitto (codice condiviso col robot vero): si riscala il valore
    # del modulo in questo processo di test, senza modificare main.py - i worker sono processi separati
    sim.FRAME_REAZIONE_VELOCITA_ROBOT = max(1, _FRAME_REAZIONE_ORIGINALE // ACCELERAZIONE)

    # --- robot ---
    partenza = _cella_vicina_angolo(griglia, 0, 0)
    arrivo = _cella_vicina_angolo(griglia, sim.Y_TOT - 1, sim.X_TOT - 1)
    robot = {"x": partenza.cx, "y": partenza.cy, "percorso": []}
    tracciamento_lidar, storico_posizioni = {}, {}
    # griglia di pianificazione del robot: identica a `griglia` con fattore 1, altrimenti piu' fine.
    # Arrivo e celle bloccate vanno riespressi nelle sue coordinate
    griglia_robot = sim.crea_griglia_robot(griglia, fattore_griglia)
    dim_robot = sim.DIM_NODO / fattore_griglia
    arrivo_robot = griglia_robot[arrivo.r * fattore_griglia + fattore_griglia // 2][
        arrivo.c * fattore_griglia + fattore_griglia // 2]
    # alone di rispetto attorno alle persone ferme, come main.py (vedi COSTO_ALONE_PERSONE_FERME): senza,
    # il robot le rasenta o le attraversa, perche' non ostruiscono fisicamente il moto e non attivano
    # l'arresto di sicurezza. Qui e' precalcolato una volta sola (sono immobili per definizione) e su
    # TUTTE, non solo su quelle rilevate dal lidar: in questo banco di prova anche le celle occupate sono
    # gia' note al robot senza passare dal rilevamento, la semplificazione e' quella e vale identica in
    # tutte le condizioni, quindi non ne favorisce nessuna
    raggio_alone_ferme = max(raggio_sicurezza, raggio_robot + raggio_persona) + sim.MARGINE_ALONE_PERSONE_FERME_PX
    costo_alone_ferme = {}
    portata_alone = int(raggio_alone_ferme // dim_robot) + 1
    righe_robot, colonne_robot = len(griglia_robot), len(griglia_robot[0])
    for pf in persone_ferme + membri_fermi:
        r0, c0 = int(pf["y"] // dim_robot), int(pf["x"] // dim_robot)
        for dr in range(-portata_alone, portata_alone + 1):
            for dc in range(-portata_alone, portata_alone + 1):
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < righe_robot and 0 <= c < colonne_robot):
                    continue
                if math.hypot((c + 0.5) * dim_robot - pf["x"], (r + 0.5) * dim_robot - pf["y"]) <= raggio_alone_ferme:
                    costo_alone_ferme[(r, c)] = sim.COSTO_ALONE_PERSONE_FERME * dim_robot / sim.DIM_NODO
    # isteresi sul percorso, come main.py: senza, due percorsi di costo quasi identico si alternano ad
    # ogni ricalcolo per pura fluttuazione della mappa di costo. Era assente da questo banco di prova, e
    # la sua assenza penalizza in modo specifico le condizioni che rendono la mappa piu' rumorosa (l'AI,
    # e la griglia fine che moltiplica i percorsi quasi equivalenti): senza, il confronto e' viziato
    budget_isteresi_px = sim.CELLE_ISTERESI_PERCORSO * sim.DIM_NODO
    costo_isteresi = sim.COSTO_ISTERESI_PERCORSO / fattore_griglia

    range_evitamento_quad = sim.RANGE_EVITAMENTO_PERSONE ** 2
    range_evitamento_robot = raggio_robot + raggio_persona
    velocita_nominale = sim.VELOCITA_ROBOT * ACCELERAZIONE

    costi = {}
    frame_fermo = 0
    # velocita' corrente come frazione della nominale. Parte da zero perche' il robot parte da fermo:
    # con i limiti di accelerazione di main.py la partenza costa, esattamente come ogni ripartenza
    fattore_corrente = 0.0
    for contatore_frame in range(FRAME_MAX):
        robot_x, robot_y = robot["x"], robot["y"]

        # celle occupate: quelle della FOLLA restano sulla griglia grossa (e' la loro griglia di
        # pianificazione), quelle viste dal ROBOT sulla sua. E' qui che nasce il guadagno sui varchi:
        # una persona larga ~14px blocca una cella da 15px invece di annerirne una da 30
        celle_bloccate_comune = {(int(p["y"] // sim.DIM_NODO), int(p["x"] // sim.DIM_NODO)) for p in tutte_mobili}
        celle_bloccate_comune |= celle_ferme
        celle_bloccate_comune.add((int(robot_y // sim.DIM_NODO), int(robot_x // sim.DIM_NODO)))
        if fattore_griglia == 1:
            celle_bloccate = celle_bloccate_comune - {(int(robot_y // sim.DIM_NODO), int(robot_x // sim.DIM_NODO))}
        else:
            celle_bloccate = {(int(p["y"] // dim_robot), int(p["x"] // dim_robot)) for p in tutte_mobili}
            celle_bloccate |= {(int(p["y"] // dim_robot), int(p["x"] // dim_robot))
                               for p in persone_ferme + membri_fermi}

        _ricalcola_percorsi(tutte_mobili, contatore_frame, griglia, celle_bloccate_comune)
        griglia_spaziale = sim.costruisci_griglia_spaziale(tutte_mobili, sim.DIM_NODO)
        _muovi_e_gestisci_stato(tutte_mobili, griglia, griglia_spaziale, robot_x, robot_y,
                                range_evitamento_quad, range_evitamento_robot)
        _rileva_e_traccia(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)

        if contatore_frame % intervallo_macchia == 0:
            costi = _mappa_costo(griglia, tracciamento_lidar, storico_posizioni, usa_ai, modello_ai,
                                 orizzonte_previsione, fattore_griglia)

        if not robot["percorso"] or contatore_frame % intervallo_ricalcolo == 0:
            # sconto sulla porzione di percorso gia' in corso, camminando lungo la geometria reale dei
            # segmenti (con Theta* un segmento puo' coprire molte celle): portata fissa in pixel,
            # campionamento e sconto alla risoluzione della griglia del robot
            costo_extra = dict(costi)
            distanza_scontata = 0.0
            for i in range(len(robot["percorso"]) - 1):
                if distanza_scontata >= budget_isteresi_px:
                    break
                a, b = robot["percorso"][i], robot["percorso"][i + 1]
                lunghezza = math.hypot(b.cx - a.cx, b.cy - a.cy)
                passi = max(1, int(lunghezza / dim_robot))
                for passo_i in range(passi + 1):
                    if distanza_scontata >= budget_isteresi_px:
                        break
                    q = passo_i / passi
                    cella = (int((a.cy + (b.cy - a.cy) * q) // dim_robot),
                             int((a.cx + (b.cx - a.cx) * q) // dim_robot))
                    costo_extra[cella] = costo_extra.get(cella, 0.0) - costo_isteresi
                    distanza_scontata += lunghezza / passi

            # l'alone vince sullo sconto dell'isteresi: un percorso che passa addosso a una persona ferma
            # va cambiato, non protetto
            for cella_alone, costo_cella in costo_alone_ferme.items():
                costo_extra[cella_alone] = max(costo_extra.get(cella_alone, 0.0), costo_cella)

            robot["percorso"] = sim.algoritmo_a_star(griglia_robot, (robot["x"], robot["y"]),
                                                     (arrivo_robot.r, arrivo_robot.c),
                                                     celle_bloccate=celle_bloccate, mappa_costo_extra=costo_extra)

        # stop di sicurezza, identico a main.py: una persona in movimento troppo vicina blocca del tutto
        # il robot per questo frame (vedi il commento in testa al file sul perche' e' essenziale). Solo
        # i corpi mobili: su una persona ferma lo stop non si scioglierebbe mai (vedi main.py)
        if raggio_sicurezza > 0 and any(math.hypot(robot["x"] - p["x"], robot["y"] - p["y"]) < raggio_sicurezza
                                        for p in tutte_mobili):
            frame_fermo += 1
            fattore_corrente = 0.0  # arresto d'emergenza: la ripartenza paga l'intera rampa
            continue

        fattore_voluto = 1.0
        if usa_velocita:
            fattore_voluto = sim.fattore_velocita_da_conflitto(
                robot["x"], robot["y"], robot["percorso"], tracciamento_lidar, velocita_nominale,
                raggio_robot, raggio_persona, strada_libera_al_massimo=strada_libera)
        # il limite di accelerazione vale in ENTRAMBE le condizioni: l'inerzia e' una proprieta' del
        # veicolo, non del controllo. Un passo di simulazione vale ACCELERAZIONE fotogrammi reali
        fattore = sim.limita_accelerazione(fattore_voluto, fattore_corrente, ACCELERAZIONE)
        fattore_corrente = fattore

        x, y, arrivato, bloccato = sim.passo_movimento(robot["x"], robot["y"], robot["percorso"],
                                                       velocita_nominale * fattore, arrivo_robot, griglia_robot)
        robot["x"], robot["y"] = x, y
        # controllo di posizione: si SOVRAPPONE al passo lungo il percorso invece di sostituirlo.
        # La griglia e' quella grossa perche' sposta_con_vettore indicizza con DIM_NODO; i muri
        # sono gli stessi nelle due risoluzioni. Un passo vale ACCELERAZIONE fotogrammi reali
        if usa_posizione:
            scarto_x, scarto_y = sim.offset_evitamento_predittivo(
                robot["x"], robot["y"], robot["percorso"], tracciamento_lidar, griglia, ACCELERAZIONE)
            robot["x"], robot["y"] = sim.sposta_con_vettore(robot["x"], robot["y"],
                                                            scarto_x, scarto_y, 1.0, griglia)
        if bloccato:
            robot["percorso"] = []
        if arrivato:
            return True, contatore_frame + 1, frame_fermo

    return False, FRAME_MAX, frame_fermo


def _esegui_gruppo(argomenti):
    """Un GRUPPO appaiato: tutte le condizioni di CONDIZIONI sullo stesso seed, nello stesso processo e
    una dopo l'altra. Tenerle insieme invece di dispacciarle come task separati garantisce che il gruppo
    sia sempre completo (nessuna condizione persa) e che tutte vedano lo stesso stato iniziale.
    L'ordine e' ininfluente: ogni corsa riparte da random.seed(seed) e ricostruisce mappa, folla e robot
    da zero, quindi nessuno stato passa dall'una all'altra. Verificato empiricamente: la stessa corsa con
    lo stesso seed da' risultati identici al frame, indipendentemente da cosa e' girato prima."""
    nome_mappa, popolazione, totale, seed, condizioni = argomenti
    risultati = {}
    for condizione in condizioni:
        arrivato, frame, fermo = esegui_corsa(nome_mappa, condizione, popolazione, seed)
        risultati[condizione] = (arrivato, frame, fermo)
    return totale, seed, risultati


# --------------------------------------------------------------------------------------------------
# Sweep di popolazione + statistiche appaiate
# --------------------------------------------------------------------------------------------------

def _secondi(frame):
    """Frame simulati -> secondi equivalenti a velocita' normale (ogni frame vale ACCELERAZIONE frame)."""
    return frame / 60 * ACCELERAZIONE


def _statistiche_appaiate(coppie):
    """coppie: lista di (tempo_prima, tempo_dopo). Ritorna media delle differenze, errore standard,
    percentuale di miglioramento e quante volte l'AI ha vinto. La differenza per coppia e' la statistica
    forte: elimina la variabilita' fra scenari, che e' la fonte di rumore dominante."""
    differenze = [prima - dopo for prima, dopo in coppie]  # positivo = l'AI ci mette meno
    n = len(differenze)
    media = sum(differenze) / n
    if n > 1:
        varianza = sum((d - media) ** 2 for d in differenze) / (n - 1)
        errore_standard = (varianza / n) ** 0.5
    else:
        errore_standard = float("nan")
    media_prima = sum(p for p, _ in coppie) / n
    percentuale = media / media_prima * 100 if media_prima > 0 else float("nan")
    vittorie = sum(1 for d in differenze if d > 0)
    return media, errore_standard, percentuale, vittorie


def confronta(nome_mappa=MAPPA_DEFAULT, repliche=REPLICHE, livelli=None, condizioni=None):
    """'condizioni' permette di restringere il confronto a un sottoinsieme (deve comunque contenere
    CONDIZIONE_RIFERIMENTO, che e' il termine di paragone di tutte le altre): con due sole condizioni il
    numero di corse si dimezza, al prezzo di non poter piu' separare il contributo dei due componenti."""
    livelli = livelli or list(range(POPOLAZIONE_MIN, POPOLAZIONE_MAX + 1, POPOLAZIONE_PASSO))
    condizioni = list(condizioni or CONDIZIONI)
    if CONDIZIONE_RIFERIMENTO not in condizioni:
        raise ValueError(f"le condizioni devono includere '{CONDIZIONE_RIFERIMENTO}': e' il riferimento del confronto appaiato")
    geometria = _geometria_salvata()

    compiti = []
    for totale in livelli:
        popolazione = popolazione_da_totale(totale)
        popolazione.update(geometria)
        for i in range(repliche):
            # crc32 e non hash(): hash() sulle stringhe e' randomizzato ad ogni avvio di Python, il
            # seed nominale non sarebbe riproducibile fra un run e l'altro
            seed = zlib.crc32(f"{nome_mappa}|{totale}|{i}".encode())
            compiti.append((nome_mappa, popolazione, totale, seed, condizioni))
    # i livelli densi costano molto piu' calcolo: mescolati si spargono fra i worker invece di
    # accumularsi tutti in coda, dove lascerebbero i worker scarichi ad aspettarne uno solo
    random.shuffle(compiti)

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Validazione appaiata fattoriale su {nome_mappa}")
    print(f"  livelli di popolazione: {livelli}")
    print(f"  condizioni: {', '.join(condizioni)}")
    print(f"  {len(compiti)} gruppi appaiati x {len(condizioni)} condizioni = {len(compiti) * len(condizioni)} corse")
    print(f"  geometria da config: robot {geometria['raggio_robot']:.1f}px, persone {geometria['raggio_persona']:.1f}px, "
          f"sicurezza {geometria['raggio_sicurezza']:.1f}px, lidar {geometria['raggio_lidar']:.0f}px")
    print(f"  accelerazione {ACCELERAZIONE}x, {n_worker} processi\n")

    risultati = {t: [] for t in livelli}   # totale -> lista di (seed, risultati per condizione)
    inizio = time.time()
    completate = 0
    with mp.Pool(n_worker, initializer=_inizializza_worker) as pool:
        for totale, seed, esito in pool.imap_unordered(_esegui_gruppo, compiti, chunksize=1):
            completate += 1
            risultati[totale].append((seed, esito))
            if completate % max(1, len(compiti) // 20) == 0 or completate == len(compiti):
                print(f"  [{completate}/{len(compiti)}] gruppi completati ({time.time() - inizio:.0f}s)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta(nome_mappa, livelli, risultati, condizioni)


def _riporta(nome_mappa, livelli, risultati, condizioni=None):
    """Ogni condizione confrontata con CONDIZIONE_RIFERIMENTO sugli stessi seed. I gruppi in cui anche una
    sola condizione non e' arrivata vengono scartati interi: confrontare condizioni su insiemi di seed
    diversi romperebbe l'appaiamento, che e' l'unica ragione per cui questo test e' sensibile."""
    condizioni = list(condizioni or CONDIZIONI)
    altre = [c for c in condizioni if c != CONDIZIONE_RIFERIMENTO]
    print(f"=== Confronto appaiato contro '{CONDIZIONE_RIFERIMENTO}': stessa folla, stesso percorso, robot diverso ===")
    print("(differenza positiva = arriva prima della base; 'vinti' = gruppi in cui ha fatto meglio)\n")
    intestazione = (f"{'persone':>8}{'condizione':>15}{'t_base':>9}{'t_cond':>9}{'diff':>9}{'err.std':>9}"
                    f"{'miglior.':>10}{'vinti':>9}{'fermo_base':>12}{'fermo_cond':>12}")
    print(intestazione)
    print("-" * len(intestazione))

    righe, righe_gruppi = [], []
    differenze_globali = {c: [] for c in altre}
    for totale in livelli:
        gruppi_validi = []
        scartati = 0
        for seed, esito in risultati[totale]:
            if not all(esito[c][0] for c in condizioni):
                scartati += 1
                continue
            gruppi_validi.append((seed, esito))
        if not gruppi_validi:
            print(f"{totale:>8}   nessun gruppo completo (in almeno una condizione il robot non e' arrivato)")
            continue

        for seed, esito in gruppi_validi:
            riga = {"mappa": nome_mappa, "popolazione": totale, "seed": seed}
            for c in condizioni:
                riga[f"tempo_{c}_s"] = round(_secondi(esito[c][1]), 2)
                riga[f"fermo_{c}_s"] = round(_secondi(esito[c][2]), 2)
            righe_gruppi.append(riga)

        base_tempi = [_secondi(e[CONDIZIONE_RIFERIMENTO][1]) for _, e in gruppi_validi]
        base_fermi = [_secondi(e[CONDIZIONE_RIFERIMENTO][2]) for _, e in gruppi_validi]
        media_base = sum(base_tempi) / len(base_tempi)
        media_fermo_base = sum(base_fermi) / len(base_fermi)

        for c in altre:
            tempi_c = [_secondi(e[c][1]) for _, e in gruppi_validi]
            fermi_c = [_secondi(e[c][2]) for _, e in gruppi_validi]
            coppie = list(zip(base_tempi, tempi_c))
            media_diff, err_std, percentuale, vittorie = _statistiche_appaiate(coppie)
            differenze_globali[c].extend(b - x for b, x in coppie)
            media_c = sum(tempi_c) / len(tempi_c)
            media_fermo_c = sum(fermi_c) / len(fermi_c)
            print(f"{totale:>8}{c:>15}{media_base:>9.2f}{media_c:>9.2f}{media_diff:>+9.2f}{err_std:>9.2f}"
                  f"{percentuale:>+9.1f}%{vittorie:>5}/{len(coppie):<3}{media_fermo_base:>12.2f}{media_fermo_c:>12.2f}")
            righe.append({"mappa": nome_mappa, "popolazione": totale, "condizione": c,
                          "gruppi": len(coppie), "tempo_base_s": round(media_base, 2),
                          "tempo_condizione_s": round(media_c, 2),
                          "differenza_media_s": round(media_diff, 2),
                          "errore_standard_s": round(err_std, 2),
                          "miglioramento_percento": round(percentuale, 1),
                          "gruppi_vinti": vittorie,
                          "fermo_base_s": round(media_fermo_base, 2),
                          "fermo_condizione_s": round(media_fermo_c, 2),
                          "gruppi_scartati": scartati})
        print()

    if righe:
        _salva_csv(FILE_RISULTATI_CSV, righe)
        _salva_csv(FILE_COPPIE_CSV, righe_gruppi)
        print(f"Tabella per livello/condizione: {FILE_RISULTATI_CSV}")
        print(f"Singoli gruppi appaiati:        {FILE_COPPIE_CSV}")

        print(f"\n=== Sintesi su tutti i livelli (contro '{CONDIZIONE_RIFERIMENTO}') ===")
        print(f"{'condizione':>15}{'diff.media':>12}{'err.std':>10}{'rapporto':>10}{'vinti':>10}")
        migliore, migliore_diff = None, -float("inf")
        for c in altre:
            d = differenze_globali[c]
            n = len(d)
            media = sum(d) / n
            dev = (sum((x - media) ** 2 for x in d) / (n - 1)) ** 0.5 if n > 1 else float("nan")
            err = dev / n ** 0.5 if n > 1 else float("nan")
            # rapporto fra differenza media e suo errore standard: sopra ~2 la differenza e' solida
            # rispetto al rumore, sotto ~1 e' indistinguibile da zero anche se la media non e' nulla
            rapporto = abs(media / err) if err and err == err and err > 0 else float("nan")
            vinti = sum(1 for x in d if x > 0)
            print(f"{c:>15}{media:>+12.2f}{err:>10.2f}{rapporto:>10.1f}{vinti:>5}/{n:<4}")
            if media > migliore_diff:
                migliore, migliore_diff = c, media
        print(f"\nCondizione migliore: '{migliore}' ({migliore_diff:+.2f}s rispetto alla base)")


def _geometria_salvata():
    """Raggi dalla configurazione salvata dal pannello di main.py, cosi' la validazione misura lo stesso
    robot che viene realmente usato nel simulatore invece dei valori di default. Contano perche' decidono
    quanto spazio di manovra ha il robot: raggi piu' piccoli lasciano passaggi praticabili dove prima
    c'era un tappo, e quindi danno all'A* piu' rotte alternative fra cui la mappa di costo puo'
    effettivamente far pendere la scelta.

    NB: non toccano la dinamica interna della folla (la repulsione fra persone usa
    RANGE_EVITAMENTO_PERSONE, costante a se'), quindi il modello allenato resta valido: cambia solo
    l'ingombro del robot e il raggio entro cui spinge via le persone."""
    import json
    valori = {}
    if os.path.exists(FILE_CONFIGURAZIONE_PANNELLO):
        with open(FILE_CONFIGURAZIONE_PANNELLO, "r") as f:
            valori = json.load(f)
    return {
        "raggio_sicurezza": valori.get("raggio_sicurezza", sim.RAGGIO_SICUREZZA_DEFAULT),
        "raggio_robot": valori.get("raggio_robot", sim.RAGGIO_ROBOT_DEFAULT),
        "raggio_persona": valori.get("raggio_persona", sim.RAGGIO_PERSONA_DEFAULT),
        "raggio_lidar": valori.get("raggio_lidar", sim.RAGGIO_LIDAR_DEFAULT),
    }


def _salva_csv(path, righe):
    os.makedirs(CARTELLA_RISULTATI, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)


def _stima_risparmio(percentuale_risparmio):
    """Conversione tempo->soldi, solo ordine di grandezza: i due valori qui sotto sono segnaposto da
    sostituire con stime prese da progetti di robot di servizio reali. Non include i costi di
    implementazione dell'AI (sviluppo, dati, inferenza): e' il solo lato del beneficio."""
    COSTO_ORARIO_EUR = 15.0
    ORE_AL_GIORNO = 8.0
    ore_risparmiate = ORE_AL_GIORNO * (percentuale_risparmio / 100)
    al_giorno = ore_risparmiate * COSTO_ORARIO_EUR
    print(f"\n=== Stima di massima (solo beneficio, NON i costi di implementazione dell'AI) ===")
    print(f"Con {COSTO_ORARIO_EUR:.0f} EUR/h e {ORE_AL_GIORNO:.0f}h/giorno di operativita':")
    print(f"  ~{ore_risparmiate:.2f}h/giorno risparmiate -> ~{al_giorno:.1f} EUR/giorno, "
          f"~{al_giorno * 260:.0f} EUR/anno (260 giorni lavorativi)")
    print("(sostituisci COSTO_ORARIO_EUR e ORE_AL_GIORNO con stime da progetti reali)")


if __name__ == "__main__":
    argomenti = [a for a in sys.argv[1:] if not a.startswith("--")]
    mappa = argomenti[0] if argomenti else MAPPA_DEFAULT
    if "--rapido" in sys.argv:
        # dieci semi diversi alla sola popolazione massima, e solo AI acceso contro AI spento: 20 corse.
        # E' il confronto piu' stretto possibile sulla domanda "l'AI fa arrivare prima?", al prezzo di non
        # sapere nulla del controllo di velocita' ne' di come l'effetto vari con la densita'
        confronta(mappa, repliche=10, livelli=[POPOLAZIONE_MAX], condizioni=["base", "solo_ai"])
    elif "--diagnostica" in sys.argv:
        # pochi livelli rappresentativi: serve a capire quale componente paga, non a tracciare la curva
        confronta(mappa, livelli=LIVELLI_DIAGNOSTICI)
    else:
        confronta(mappa)

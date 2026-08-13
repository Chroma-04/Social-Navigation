import os
os.environ["SDL_VIDEO_CENTERED"] = "1"
import pygame
import math
import json
import random
import heapq
import collections
import multiprocessing as mp

# --- CONFIGURAZIONE ---
X_TOT_STANDARD, Y_TOT_STANDARD = 80, 60
X_TOT_POVO, Y_TOT_POVO = 44, 73  # griglia dedicata alla planimetria reale (modalita' Povo, tasto P)
# griglie riconosciute al caricamento di una mappa: da queste si deduce in quale modalita' entrare
DIMENSIONI_GRIGLIA_NOTE = ((X_TOT_STANDARD, Y_TOT_STANDARD), (X_TOT_POVO, Y_TOT_POVO))
# dimensioni correnti: non sono costanti, il tasto P le riassegna tramite applica_dimensioni_griglia
X_TOT, Y_TOT = X_TOT_STANDARD, Y_TOT_STANDARD
DIM_NODO = 30
LARGHEZZA, ALTEZZA = X_TOT * DIM_NODO, Y_TOT * DIM_NODO
VELOCITA_ROBOT = 1.5
VELOCITA_PERSONA = 1.0
# --- controllo predittivo di velocita' del robot ---
FATTORE_VELOCITA_ROBOT_MIN = 0.35  # velocita' minima, come frazione di VELOCITA_ROBOT
FATTORE_VELOCITA_ROBOT_MAX = 1.25  # velocita' massima (sprint) come frazione di VELOCITA_ROBOT
DISTANZA_LOOKAHEAD_VELOCITA_PX = DIM_NODO * 5  # quanto avanti si guarda sul percorso
MARGINE_SICUREZZA_CONFLITTO_PX = 6  # franco oltre la somma dei raggi robot+persona
FRAME_REAZIONE_VELOCITA_ROBOT = 90  # orizzonte entro cui un conflitto previsto fa rallentare
ATTESA_PERSONA_MIN_FRAME = 1 * 60  # attesa minima dopo l'arrivo, in frame (60 FPS)
ATTESA_PERSONA_MAX_FRAME = 4 * 60  # attesa massima dopo l'arrivo, in frame
PROBABILITA_PERCORSO_BREVE = 0.35  # probabilita' che la prossima meta sia vicina invece che casuale
RAGGIO_PERCORSO_BREVE = DIM_NODO * 6  # distanza massima di un "percorso breve", in pixel
RANGE_EVITAMENTO_PERSONE = 15  # px: distanza sotto cui scatta la repulsione fra persone
FORZA_REPULSIONE_PERSONE = 1.3  # px/frame massimi di spinta fra persone
COSTO_CELLA_OCCUPATA = DIM_NODO * 10  # penalita' A* per cella occupata: forte ma non un divieto
# alone di rispetto attorno alle persone ferme rilevate: le tiene a distanza dal pianificatore invece
# che dall'arresto di sicurezza, che su un corpo immobile non si scioglierebbe mai. Costo alto ma
# finito, cosi' un varco tappato resta percorribile in ultima istanza
COSTO_ALONE_PERSONE_FERME = COSTO_CELLA_OCCUPATA * 2
MARGINE_ALONE_PERSONE_FERME_PX = 6.0  # px di franco oltre l'ingombro dei corpi
INTERVALLO_RICALCOLO_ROBOT_FRAME = 60    # ricalcolo percorso robot, in frame
INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME = 6  # ricalcolo macchia di probabilita', in frame
INTERVALLO_RICALCOLO_PERSONE_FRAME = 10  # ricalcolo percorso persone, in frame
CARTELLA_MAPPE_SALVATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappe_salvate")
CARTELLA_MAPPE_TRAINING = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappe_training")
CARTELLA_MAPPE_TESTING = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappe_testing")
CARTELLA_PLANIMETRIE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "planimetrie")
# gli slot 0-2 stanno sulla griglia standard, l'ultimo sulla griglia Povo: il menu ne mostra uno solo
# dei due insiemi per volta, perche' una mappa 44x73 caricata su griglia 80x60 sposta i muri
SLOT_MAPPA_POVO = 3
FILE_MAPPA_SLOT = [
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_salvata.json"),
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_salvata_2.json"),
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_salvata_3.json"),
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_povo.json"),
]
PASSWORD_SALVATAGGIO = "1258"
# overlay della planimetria: trasparenza, e allineamento fine rispetto alla griglia (frecce e PagSu/PagGiu)
ALPHA_PLANIMETRIA_MIN, ALPHA_PLANIMETRIA_MAX = 0, 255
ALPHA_PLANIMETRIA_DEFAULT = 90
PASSO_OFFSET_PLANIMETRIA_PX = 5  # px-mappa per pressione di freccia
PASSO_SCALA_PLANIMETRIA = 0.01
SCALA_PLANIMETRIA_MIN, SCALA_PLANIMETRIA_MAX = 0.2, 3.0
PIXEL_MAX_SMOOTHSCALE = 12_000_000  # oltre questa area la planimetria si riscala col metodo veloce
PASSO_SCROLL_PANNELLO = 40  # px di scorrimento del pannello tecnico per scatto di rotella
FILE_CONFIGURAZIONE_PANNELLO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_pannello.json")
VEL_MULT_MIN, VEL_MULT_MAX = 0.2, 25.0
NUMERO_PERSONE_DEFAULT = 0
NUMERO_PERSONE_MAX = 200
NUMERO_PERSONE_FERME_DEFAULT = 0
NUMERO_PERSONE_FERME_MAX = 200
NUMERO_CORRIDORI_DEFAULT = 0
NUMERO_CORRIDORI_MAX = 200
FATTORE_VELOCITA_CORRIDORE = 2.0  # moltiplicatore di velocita' dei corridori
NUMERO_GRUPPI_DEFAULT = 0
NUMERO_GRUPPI_MAX = 200
GRUPPO_MEMBRI_MIN, GRUPPO_MEMBRI_MAX = 2, 7
GRUPPO_MEMBRI_MEDIA = 3.5  # numero di componenti, estratto da una gaussiana
GRUPPO_MEMBRI_DEV_STD = 1.1
GRUPPO_RAGGIO_FORMAZIONE_PX = int(DIM_NODO * 1.0)  # distanza dei membri dal capofila
GRUPPO_RUMORE_RAGGIO = 0.3  # variazione casuale sul raggio di ciascun membro (+-30%)
GRUPPO_RUMORE_ANGOLO = 0.4  # rad di variazione casuale sull'angolo di ciascun membro
PROBABILITA_GRUPPO_MOBILE = 0.8  # frazione di gruppi che si muove
GRUPPO_DISTANZA_COESIONE_PX = int(DIM_NODO * 1.5)  # sotto questa distanza dal capofila non scatta la rincorsa
GRUPPO_FATTORE_RINCORSA_MAX = 2.5  # accelerazione massima di un membro in rincorsa
GRUPPO_FRAME_BLOCCO_SOGLIA = 90  # frame fermo prima di puntare dritto al capofila
GRUPPO_DISTANZA_MAX_DIVERGENZA_PX = int(DIM_NODO * 5)  # oltre questa distanza punta dritto al capofila
ZOOM_MIN, ZOOM_MAX = 0.25, 4.0
MARGINE_PAN = 100  # spazio bianco massimo attorno alla mappa, in pixel
RAGGIO_SICUREZZA_MIN, RAGGIO_SICUREZZA_MAX = 0, DIM_NODO * 6
RAGGIO_SICUREZZA_DEFAULT = 12
ALPHA_ZONA_SICUREZZA = 70  # trasparenza del cerchio (0-255)
RAGGIO_ROBOT_MIN, RAGGIO_ROBOT_MAX = 5, int(DIM_NODO * 1.5)
RAGGIO_ROBOT_DEFAULT = 7  # raggio fisico del robot
RAGGIO_PERSONA_MIN, RAGGIO_PERSONA_MAX = 3, int(DIM_NODO * 0.9)
RAGGIO_PERSONA_DEFAULT = 8  # raggio fisico delle persone
FORZA_REPULSIONE_ROBOT = 1.3  # px/frame massimi di spinta del robot sulle persone
RAGGIO_LIDAR_MIN, RAGGIO_LIDAR_MAX = 0, DIM_NODO * 10
RAGGIO_LIDAR_DEFAULT = DIM_NODO * 4  # raggio di rilevamento del lidar simulato
RUMORE_LIDAR_PX = 8  # px massimi di rumore sulla posizione percepita
ALPHA_ZONA_LIDAR = 35  # trasparenza del cerchio del lidar (0-255)
KALMAN_RUMORE_MISURA = (RUMORE_LIDAR_PX / 2) ** 2  # varianza di misura
KALMAN_RUMORE_PROCESSO = 0.05  # varianza di processo sulla velocita', per frame
KALMAN_INCERTEZZA_INIZIALE_POS = 50.0 ** 2  # varianza di posizione di una traccia nuova
KALMAN_INCERTEZZA_INIZIALE_VEL = 5.0 ** 2  # varianza di velocita' di una traccia nuova
PREVISIONE_BLOB_FRAME = 60  # orizzonte di proiezione del centro della macchia (60 frame = 1s)
# raggio della macchia di probabilita', in celle. Provato anche a 1.3 (dimensionato sull'errore medio
# di previsione, 0,60 m): differenza sui tempi indistinguibile, vedi risultati_test/confronto_ai_blob13.md
RAGGIO_BLOB_CELLE = 3.0
DECADIMENTO_BLOB = 0.75  # decadimento del peso per ogni passo-cella di diffusione
# allungamento della macchia lungo la direzione di marcia stimata: un passo di diffusione allineato
# con l'heading costa meno, uno perpendicolare di piu'. 0 = macchia isotropa
ALLUNGAMENTO_BLOB = 0.55
VELOCITA_MINIMA_AFFIDABILE = 0.15  # soglia numerica per non normalizzare un heading quasi nullo
COSTO_MASSIMO_PROBABILITA = COSTO_CELLA_OCCUPATA * 0.2  # costo A* al centro della macchia (peso 1.0)
COLORE_BLOB_PROBABILITA = (40, 130, 255)
# isteresi: sconto di costo sulle prossime celle del percorso gia' scelto, per non ribaltare la scelta
# fra due rotte quasi equivalenti a ogni ricalcolo. Sotto DIM_NODO, cosi' un passo non costa mai negativo
COSTO_ISTERESI_PERCORSO = DIM_NODO * 0.6
CELLE_ISTERESI_PERCORSO = 10
ALPHA_MAX_BLOB = 140  # trasparenza al centro della macchia
# suddivisione usata SOLO per calcolare e disegnare la macchia: A* e movimento restano sulla griglia
# di movimento. Regolabile dal pannello tecnico
SOTTOCELLE_PER_LATO_MIN, SOTTOCELLE_PER_LATO_MAX = 1, 10
SOTTOCELLE_PER_LATO_DEFAULT = 1
# griglia di pianificazione del robot, come multiplo di quella della folla (toggle "Griglia fine robot").
# Fattore 2 = celle da 15px, circa il diametro di una persona: piu' fine, un corpo coprirebbe piu' celle
FATTORE_GRIGLIA_ROBOT = 2
GRIGLIA_FINE_DEFAULT = True
CELLE_SOTTOGRIGLIA_VISIBILI = 12  # entro quante celle dal robot si disegna la griglia fine
# passo di campionamento dei controlli di visibilita' (lidar e scorciatoie Theta*), in pixel: fissato
# sulla griglia di movimento, che e' lo spessore del muro piu' sottile possibile
PASSO_CAMPIONAMENTO_VISIBILITA = DIM_NODO / 4
FATTORE_VELOCITA_MEDIA = 1.0    # media della gaussiana sulle velocita' individuali
FATTORE_VELOCITA_DEV_STD = 0.25
FATTORE_VELOCITA_MIN, FATTORE_VELOCITA_MAX = 0.4, 2.5
RUMORE_PERCORSO_PERSONA = 15  # scostamento dei percorsi delle persone dall'ottimo (0 = disattivato)

# Colori
BIANCO, GRIGIO = (255, 255, 255), (210, 210, 210)
ROSSO, BLU = (255, 0, 0), (0, 100, 255)
ARANCIONE = (240, 140, 0)
VERDE = (30, 160, 60)  # persone non rilevate dal lidar
NERO = (30, 30, 30)
GRIGIO_SCURO = (90, 90, 90)
GRIGIO_PAVIMENTO = (195, 195, 195)  # sfondo della mappa
GRIGIO_BORDO_CELLA = (182, 182, 182)  # bordo delle celle libere
GRIGIO_SOTTOGRIGLIA = (168, 176, 196)  # suddivisioni della griglia fine del robot
COLORE_LIDAR = (235, 200, 0)

class Nodo:
    # dim_nodo sta sul nodo e non nella costante globale perche' coesistono due griglie a risoluzione
    # diversa: quella della folla e quella piu' fine del robot
    def __init__(self, r, c, dim_nodo=DIM_NODO):
        self.r, self.c = r, c
        self.dim = dim_nodo
        self.x, self.y = c * dim_nodo, r * dim_nodo
        self.cx = self.x + dim_nodo // 2
        self.cy = self.y + dim_nodo // 2
        self.g = self.h = self.f = 0
        self.genitore = None
        self.tipo = "libero"

    def reset_calcoli(self):
        self.g = self.h = self.f = 0
        self.genitore = None

def algoritmo_a_star(griglia, inizio_pos_pixel, fine_pos_griglia, rumore_seed=None, celle_bloccate=None, mappa_costo_extra=None):
    # geometria letta dalla griglia ricevuta: la stessa funzione serve folla e robot. Il costo per cella
    # si riscala con il lato, cosi' la penalita' totale di un ostacolo non dipende dalla risoluzione
    dim = griglia[0][0].dim
    righe, colonne = len(griglia), len(griglia[0])
    costo_occupata = COSTO_CELLA_OCCUPATA * dim / DIM_NODO
    c_inizio = int(inizio_pos_pixel[0] // dim)
    r_inizio = int(inizio_pos_pixel[1] // dim)
    r_fine, c_fine = fine_pos_griglia
    r_inizio = max(0, min(righe - 1, r_inizio))
    c_inizio = max(0, min(colonne - 1, c_inizio))

    nodo_inizio = griglia[r_inizio][c_inizio]
    nodo_fine = griglia[r_fine][c_fine]

    # la griglia non viene azzerata all'inizio: ogni chiamata ripulisce alla fine i soli nodi toccati
    contatore = 0  # tie-breaker per lo heap: i Nodo non sono confrontabili fra loro
    open_heap = [(0, contatore, nodo_inizio)]
    in_open = {nodo_inizio: 0}  # nodo -> miglior g noto; e' anche l'insieme dei nodi da ripulire
    closed_list = set()

    while open_heap:
        _, _, attuale = heapq.heappop(open_heap)
        if attuale in closed_list:
            continue
        closed_list.add(attuale)

        if attuale == nodo_fine:
            cammino = []
            nodo_percorso = attuale
            while nodo_percorso:
                cammino.append(nodo_percorso)
                nodo_percorso = nodo_percorso.genitore
            for n in in_open:
                n.reset_calcoli()
            return cammino[::-1]

        for m in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            r, c = attuale.r + m[0], attuale.c + m[1]
            if 0 <= r < righe and 0 <= c < colonne:
                vicino = griglia[r][c]
                if vicino in closed_list or vicino.tipo == "muro": continue
                if m[0] != 0 and m[1] != 0:
                    # diagonale vietata se taglia l'angolo fra due muri: in coordinate continue quel
                    # passo e' impossibile e passo_movimento lo respingerebbe sempre
                    if griglia[attuale.r][c].tipo == "muro" or griglia[r][attuale.c].tipo == "muro":
                        continue
                dist = math.sqrt((vicino.cx - attuale.cx)**2 + (vicino.cy - attuale.cy)**2)
                if rumore_seed is not None:
                    # rumore deterministico per cella+persona: ricalcolando, la stessa persona rifa'
                    # sempre la stessa scelta di corsia invece di zigzagare
                    perturbazione = math.sin(vicino.r * 12.9898 + vicino.c * 78.233 + rumore_seed) * RUMORE_PERCORSO_PERSONA
                    dist = max(1.0, dist + perturbazione)
                if celle_bloccate and (r, c) in celle_bloccate:
                    dist += costo_occupata  # penalita', non divieto: un varco a 1 cella resta percorribile
                if mappa_costo_extra:
                    dist += mappa_costo_extra.get((r, c), 0.0)  # costo morbido, es. macchia di probabilita'
                nuovo_g = attuale.g + dist
                nuovo_genitore = attuale

                # Theta*: se il nonno vede il vicino in linea libera, collegarlo direttamente a lui
                # accorcia il percorso e lo svincola dagli 8 angoli della griglia
                if attuale.genitore is not None and scorciatoia_libera(griglia, attuale.genitore, vicino, celle_bloccate, mappa_costo_extra):
                    dist_scorciatoia = math.hypot(vicino.cx - attuale.genitore.cx, vicino.cy - attuale.genitore.cy)
                    nuovo_g_scorciatoia = attuale.genitore.g + dist_scorciatoia
                    if nuovo_g_scorciatoia < nuovo_g:
                        nuovo_g = nuovo_g_scorciatoia
                        nuovo_genitore = attuale.genitore

                if vicino not in in_open or nuovo_g < in_open[vicino]:
                    vicino.g = nuovo_g
                    vicino.h = math.sqrt((vicino.cx - nodo_fine.cx)**2 + (vicino.cy - nodo_fine.cy)**2)
                    vicino.f = vicino.g + vicino.h
                    vicino.genitore = nuovo_genitore
                    in_open[vicino] = nuovo_g
                    contatore += 1
                    heapq.heappush(open_heap, (vicino.f, contatore, vicino))
    for n in in_open:
        n.reset_calcoli()
    return []

def crea_griglia_robot(griglia_base, fattore):
    """Griglia di pianificazione del robot, 'fattore' volte piu' fine di quella della folla.

    I muri sono gli stessi, solo suddivisi: il guadagno sta nella granularita' di cio' che vi si appoggia
    sopra (celle occupate, macchia di probabilita', isteresi). Con fattore 1 restituisce l'originale."""
    if fattore <= 1:
        return griglia_base
    dim = DIM_NODO // fattore
    righe, colonne = len(griglia_base) * fattore, len(griglia_base[0]) * fattore
    fine = [[Nodo(r, c, dim) for c in range(colonne)] for r in range(righe)]
    for r in range(righe):
        riga_base = griglia_base[r // fattore]
        riga_fine = fine[r]
        for c in range(colonne):
            if riga_base[c // fattore].tipo == "muro":
                riga_fine[c].tipo = "muro"
    return fine

def applica_dimensioni_griglia(x_tot, y_tot):
    """Riassegna le dimensioni della griglia valide per tutto il modulo.

    X_TOT/Y_TOT sono lette come globali da crea_bordi, sottocella_e_muro, macchia_diffusione, dai clamp
    del rilevamento lidar e dall'inizializzazione dei worker: passare qui e' meno invasivo che filtrare
    le dimensioni attraverso ogni firma. LARGHEZZA/ALTEZZA vanno ricalcolate insieme, perche'
    limita_zoom_pan ci appoggia sopra l'estensione della mappa."""
    global X_TOT, Y_TOT, LARGHEZZA, ALTEZZA
    X_TOT, Y_TOT = x_tot, y_tot
    LARGHEZZA, ALTEZZA = X_TOT * DIM_NODO, Y_TOT * DIM_NODO

def crea_bordi(griglia):
    for r in range(Y_TOT):
        for c in range(X_TOT):
            if r == 0 or r == Y_TOT - 1 or c == 0 or c == X_TOT - 1:
                griglia[r][c].tipo = "muro"

def limita_zoom_pan(zoom, offset_x, offset_y, win_w, win_h):
    """Blocca zoom entro [ZOOM_MIN, ZOOM_MAX] e impedisce di scorrere nel vuoto oltre la mappa."""
    zoom = max(ZOOM_MIN, min(ZOOM_MAX, zoom))
    grid_w, grid_h = LARGHEZZA * zoom, ALTEZZA * zoom

    if grid_w <= win_w:
        offset_x = (win_w - grid_w) / 2
    else:
        offset_x = max(win_w - grid_w - MARGINE_PAN, min(MARGINE_PAN, offset_x))

    if grid_h <= win_h:
        offset_y = (win_h - grid_h) / 2
    else:
        offset_y = max(win_h - grid_h - MARGINE_PAN, min(MARGINE_PAN, offset_y))

    return zoom, offset_x, offset_y

def cella_libera_casuale(griglia):
    libere = [n for riga in griglia for n in riga if n.tipo == "libero"]
    return random.choice(libere)

def cella_libera_3x3_casuale(griglia):
    """Cella libera con tutto il blocco 3x3 attorno libero: da' a un gruppo lo spazio per generarsi.
    Se non ne esistono, ripiega su una cella libera qualsiasi."""
    candidate = [
        griglia[r][c]
        for r in range(1, Y_TOT - 1) for c in range(1, X_TOT - 1)
        if griglia[r][c].tipo == "libero"
        and all(griglia[r + dr][c + dc].tipo == "libero" for dr in (-1, 0, 1) for dc in (-1, 0, 1))
    ]
    if candidate:
        return random.choice(candidate)
    return cella_libera_casuale(griglia)

def cella_libera_vicina(griglia, x, y, raggio):
    """Cella libera entro un raggio da (x, y). Se non ne trova, ripiega su una casuale."""
    vicine = [n for riga in griglia for n in riga if n.tipo == "libero" and math.hypot(n.cx - x, n.cy - y) <= raggio]
    if vicine:
        return random.choice(vicine)
    return cella_libera_casuale(griglia)

def crea_persona(griglia, fattore_velocita=None):
    if fattore_velocita is None:
        fattore_velocita = max(FATTORE_VELOCITA_MIN, min(FATTORE_VELOCITA_MAX, random.gauss(FATTORE_VELOCITA_MEDIA, FATTORE_VELOCITA_DEV_STD)))
    nodo_iniziale = cella_libera_casuale(griglia)
    nodo_target = cella_libera_casuale(griglia)
    return {
        "x": nodo_iniziale.cx, "y": nodo_iniziale.cy,
        "target": (nodo_target.r, nodo_target.c),
        "percorso": None,  # None = mai calcolato, [] = invalidato e da ricalcolare subito
        "stato": "movimento",
        "attesa_timer": 0,
        "rumore_seed": random.uniform(0, 1000),
        "fattore_velocita": fattore_velocita,
        "offset_ricalcolo": random.randint(0, INTERVALLO_RICALCOLO_PERSONE_FRAME - 1),
        "priorita": random.random(),  # rompe gli stalli simmetrici: chi ha priorita' minore cede il passo
    }

def crea_corridore(griglia):
    return crea_persona(griglia, fattore_velocita=FATTORE_VELOCITA_CORRIDORE)

def sincronizza_persone(persone, numero, griglia, fabbrica=crea_persona):
    while len(persone) < numero:
        persone.append(fabbrica(griglia))
    while len(persone) > numero:
        persone.pop()

def crea_persona_ferma(griglia):
    nodo = cella_libera_casuale(griglia)
    return {"x": nodo.cx, "y": nodo.cy, "r": nodo.r, "c": nodo.c}

def sincronizza_persone_ferme(persone_ferme, numero, griglia):
    while len(persone_ferme) < numero:
        persone_ferme.append(crea_persona_ferma(griglia))
    while len(persone_ferme) > numero:
        persone_ferme.pop()

def offset_formazione_gruppo(numero_altri_membri):
    """Posizioni in cerchio attorno all'ancora, con rumore su raggio e angolo di ciascun membro."""
    if numero_altri_membri <= 0:
        return []
    angolo_iniziale = random.uniform(0, 2 * math.pi)
    offset = []
    for i in range(numero_altri_membri):
        angolo = angolo_iniziale + 2 * math.pi * i / numero_altri_membri + random.uniform(-GRUPPO_RUMORE_ANGOLO, GRUPPO_RUMORE_ANGOLO)
        raggio = GRUPPO_RAGGIO_FORMAZIONE_PX * random.uniform(1 - GRUPPO_RUMORE_RAGGIO, 1 + GRUPPO_RUMORE_RAGGIO)
        offset.append((raggio * math.cos(angolo), raggio * math.sin(angolo)))
    return offset

def crea_gruppo(griglia):
    """Gruppo di 2-7 persone. Se fermo, e' un cluster statico attorno a un'ancora. Se mobile, i membri
    condividono la destinazione ma hanno ciascuno il proprio A*, e chi resta indietro accelera."""
    numero_membri = round(random.gauss(GRUPPO_MEMBRI_MEDIA, GRUPPO_MEMBRI_DEV_STD))
    numero_membri = max(GRUPPO_MEMBRI_MIN, min(GRUPPO_MEMBRI_MAX, numero_membri))

    if random.random() < PROBABILITA_GRUPPO_MOBILE:
        ancora = cella_libera_3x3_casuale(griglia)
        nodo_target = cella_libera_casuale(griglia)
        target_comune = (nodo_target.r, nodo_target.c)
        # andatura e rumore di percorso comuni a tutto il gruppo: con valori individuali i membri si
        # sfaldano anche in area aperta, scegliendo corsie diverse o camminando a velocita' diverse
        fattore_velocita_gruppo = max(FATTORE_VELOCITA_MIN, min(FATTORE_VELOCITA_MAX, random.gauss(FATTORE_VELOCITA_MEDIA, FATTORE_VELOCITA_DEV_STD)))
        rumore_seed_gruppo = random.uniform(0, 1000)
        membri = []
        for dx, dy in offset_formazione_gruppo(numero_membri):
            nodo_spawn = cella_libera_vicina(griglia, ancora.cx + dx, ancora.cy + dy, DIM_NODO * 2)
            p = crea_persona(griglia, fattore_velocita=fattore_velocita_gruppo)
            p["x"], p["y"] = nodo_spawn.cx, nodo_spawn.cy
            p["target"] = target_comune
            p["rumore_seed"] = rumore_seed_gruppo
            membri.append(p)
        gruppo = {"mobile": True, "membri": membri, "target_comune": target_comune}
        membri[0]["capofila"] = True  # decide quando il gruppo riparte verso una nuova destinazione
        for m in membri:
            m["gruppo"] = gruppo
        return gruppo

    ancora = cella_libera_3x3_casuale(griglia)
    membri = [{"x": ancora.cx, "y": ancora.cy, "r": ancora.r, "c": ancora.c}]
    for dx, dy in offset_formazione_gruppo(numero_membri - 1):
        mx, my = ancora.cx + dx, ancora.cy + dy
        r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
        if not (0 <= r < Y_TOT and 0 <= c < X_TOT and griglia[r][c].tipo != "muro"):
            mx, my, r, c = ancora.cx, ancora.cy, ancora.r, ancora.c
        membri.append({"x": mx, "y": my, "r": r, "c": c})
    return {"mobile": False, "membri": membri}

def sincronizza_gruppi(gruppi, numero, griglia):
    while len(gruppi) < numero:
        gruppi.append(crea_gruppo(griglia))
    while len(gruppi) > numero:
        gruppi.pop()

def costruisci_griglia_spaziale(entita_list, dim_bucket):
    """Indicizza le entita' in bucket quadrati di lato dim_bucket, per cercare i vicini in O(n)
    guardando poche celle invece di confrontare ogni entita' con tutte le altre."""
    griglia_spaziale = {}
    for e in entita_list:
        chiave = (int(e["y"] // dim_bucket), int(e["x"] // dim_bucket))
        griglia_spaziale.setdefault(chiave, []).append(e)
    return griglia_spaziale

def vicini_spaziali(entita, griglia_spaziale, dim_bucket):
    """Entita' nel bucket di 'entita' e negli 8 adiacenti: basta perche' dim_bucket e' maggiore di
    RANGE_EVITAMENTO_PERSONE, quindi nessun vicino utile puo' trovarsi piu' lontano."""
    r, c = int(entita["y"] // dim_bucket), int(entita["x"] // dim_bucket)
    vicini = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            vicini.extend(griglia_spaziale.get((r + dr, c + dc), ()))
    return vicini

def repulsione_vicini(entita, vicini, range_evitamento_quad):
    """Vettore di spinta lontano dai vicini entro il raggio di evitamento, proporzionale alla vicinanza.
    Il peso tiene conto della priorita': in un incrocio chi ha priorita' minore cede il passo."""
    rep_x, rep_y = 0.0, 0.0
    for v in vicini:
        if v is entita:
            continue
        dx, dy = entita["x"] - v["x"], entita["y"] - v["y"]
        d2 = dx * dx + dy * dy
        if 0 < d2 < range_evitamento_quad:
            d = math.sqrt(d2)
            peso = (RANGE_EVITAMENTO_PERSONE - d) / RANGE_EVITAMENTO_PERSONE
            peso *= max(0.15, 1.0 + (v.get("priorita", 0.5) - entita.get("priorita", 0.5)))
            rep_x += (dx / d) * peso
            rep_y += (dy / d) * peso
    return rep_x, rep_y

def repulsione_punto(entita, px, py, raggio):
    """Come repulsione_vicini ma da un singolo punto (il corpo del robot), senza pesi di priorita'."""
    dx, dy = entita["x"] - px, entita["y"] - py
    d2 = dx * dx + dy * dy
    raggio_quad = raggio * raggio
    if 0 < d2 < raggio_quad:
        d = math.sqrt(d2)
        peso = (raggio - d) / raggio
        return (dx / d) * peso, (dy / d) * peso
    return 0.0, 0.0

def linea_di_vista_libera(griglia, x1, y1, x2, y2):
    """True se il segmento non attraversa nessun muro: simula l'occlusione del lidar. Campiona a passi
    piu' fitti del lato di una cella, cosi' non puo' saltare un muro sottile."""
    dist = math.hypot(x2 - x1, y2 - y1)
    if dist == 0:
        return True
    dim = griglia[0][0].dim
    righe, colonne = len(griglia), len(griglia[0])
    passi = max(1, int(dist / PASSO_CAMPIONAMENTO_VISIBILITA))
    for i in range(passi + 1):
        t = i / passi
        x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
        r, c = int(y // dim), int(x // dim)
        if 0 <= r < righe and 0 <= c < colonne and griglia[r][c].tipo == "muro":
            return False
    return True

def scorciatoia_libera(griglia, nodo_a, nodo_b, celle_bloccate, mappa_costo_extra):
    """True se il segmento e' libero per una scorciatoia Theta*: nessun muro, nessuna cella occupata e
    nessun costo extra positivo. La scorciatoia salta i nodi intermedi, quindi ne bypasserebbe i costi."""
    dist = math.hypot(nodo_b.cx - nodo_a.cx, nodo_b.cy - nodo_a.cy)
    if dist == 0:
        return True
    dim = griglia[0][0].dim
    righe, colonne = len(griglia), len(griglia[0])
    passi = max(1, int(dist / PASSO_CAMPIONAMENTO_VISIBILITA))
    for i in range(passi + 1):
        t = i / passi
        x, y = nodo_a.cx + (nodo_b.cx - nodo_a.cx) * t, nodo_a.cy + (nodo_b.cy - nodo_a.cy) * t
        r, c = int(y // dim), int(x // dim)
        if not (0 <= r < righe and 0 <= c < colonne) or griglia[r][c].tipo == "muro":
            return False
        if celle_bloccate and (r, c) in celle_bloccate:
            return False
        if mappa_costo_extra and mappa_costo_extra.get((r, c), 0.0) > 0:
            return False
    return True

def colore_rilevamento(px, py, robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia):
    """Verde se non rilevata, arancione se rilevata dal lidar, rosso se dentro la zona di sicurezza.
    La zona di sicurezza e' un limite fisico e resta attiva anche dietro un muro."""
    d = math.hypot(px - robot_x, py - robot_y)
    if raggio_sicurezza > 0 and d < raggio_sicurezza:
        return ROSSO
    if raggio_lidar > 0 and d < raggio_lidar and linea_di_vista_libera(griglia, robot_x, robot_y, px, py):
        return ARANCIONE
    return VERDE

def kalman_predict_asse(pos, vel, pxx, pxv, pvv, dt=1.0):
    """Passo di previsione di un Kalman 1D a velocita' costante (stato: posizione, velocita').
    Applicato separatamente ai due assi, che in questo modello non sono correlati."""
    pos_n = pos + vel * dt
    vel_n = vel
    pxx_n = pxx + 2 * dt * pxv + dt * dt * pvv
    pxv_n = pxv + dt * pvv
    pvv_n = pvv + KALMAN_RUMORE_PROCESSO
    return pos_n, vel_n, pxx_n, pxv_n, pvv_n

def kalman_update_asse(pos, vel, pxx, pxv, pvv, misura):
    """Passo di correzione: integra una posizione misurata, pesandola con il guadagno di Kalman."""
    innovazione = misura - pos
    s = pxx + KALMAN_RUMORE_MISURA
    kx = pxx / s
    kv = pxv / s
    pos_n = pos + kx * innovazione
    vel_n = vel + kv * innovazione
    pxx_n = (1 - kx) * pxx
    pxv_n = (1 - kx) * pxv
    pvv_n = pvv - kv * pxv
    return pos_n, vel_n, pxx_n, pxv_n, pvv_n

def nuova_traccia_kalman(x, y):
    return {
        "x": x, "vx": 0.0, "pxx": KALMAN_INCERTEZZA_INIZIALE_POS, "pxv": 0.0, "pvv_x": KALMAN_INCERTEZZA_INIZIALE_VEL,
        "y": y, "vy": 0.0, "pyy": KALMAN_INCERTEZZA_INIZIALE_POS, "pyv": 0.0, "pvv_y": KALMAN_INCERTEZZA_INIZIALE_VEL,
    }

def aggiorna_traccia_kalman(traccia, x_misurato, y_misurato):
    """Un passo predici+correggi per frame, sui due assi indipendenti."""
    traccia["x"], traccia["vx"], traccia["pxx"], traccia["pxv"], traccia["pvv_x"] = kalman_predict_asse(
        traccia["x"], traccia["vx"], traccia["pxx"], traccia["pxv"], traccia["pvv_x"])
    traccia["x"], traccia["vx"], traccia["pxx"], traccia["pxv"], traccia["pvv_x"] = kalman_update_asse(
        traccia["x"], traccia["vx"], traccia["pxx"], traccia["pxv"], traccia["pvv_x"], x_misurato)
    traccia["y"], traccia["vy"], traccia["pyy"], traccia["pyv"], traccia["pvv_y"] = kalman_predict_asse(
        traccia["y"], traccia["vy"], traccia["pyy"], traccia["pyv"], traccia["pvv_y"])
    traccia["y"], traccia["vy"], traccia["pyy"], traccia["pyv"], traccia["pvv_y"] = kalman_update_asse(
        traccia["y"], traccia["vy"], traccia["pyy"], traccia["pyv"], traccia["pvv_y"], y_misurato)

def previsione_posizione_kalman(traccia, frame_futuri):
    """Proietta la posizione stimata di 'frame_futuri' frame, senza nuove misure: e' il centro della
    macchia di probabilita'."""
    return traccia["x"] + traccia["vx"] * frame_futuri, traccia["y"] + traccia["vy"] * frame_futuri

def sottocella_e_muro(griglia, rf, cf, sottocelle_per_lato):
    """True se la sottocella cade dentro un muro della griglia di movimento, o e' fuori mappa."""
    r_cella, c_cella = int(rf // sottocelle_per_lato), int(cf // sottocelle_per_lato)
    if not (0 <= r_cella < Y_TOT and 0 <= c_cella < X_TOT):
        return True
    return griglia[r_cella][c_cella].tipo == "muro"

_DIREZIONI_DIFFUSIONE = ((0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                          (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))

def macchia_diffusione(griglia, rf_centro, cf_centro, raggio_sottocelle, decadimento, sottocelle_per_lato, direzione=None):
    """Diffonde un peso (1.0 al centro) attraverso lo spazio libero con un Dijkstra a 8 direzioni sulla
    sotto-griglia: i muri bloccano il flusso, quindi la macchia si piega agli angoli e si stringe nelle
    porte. Il decadimento segue la distanza percorsa, non il numero di passi, cosi' il fronte e' tondo.

    'direzione' (vx, vy) rende la diffusione anisotropa: i passi allineati con la marcia costano meno e
    i perpendicolari di piu', producendo un ellissoide orientato. None ricade sul caso isotropo."""
    y_tot_fine, x_tot_fine = Y_TOT * sottocelle_per_lato, X_TOT * sottocelle_per_lato
    if sottocella_e_muro(griglia, rf_centro, cf_centro, sottocelle_per_lato):
        return {}

    hx = hy = 0.0
    if direzione is not None:
        norma_direzione = math.hypot(direzione[0], direzione[1])
        if norma_direzione > VELOCITA_MINIMA_AFFIDABILE:  # sotto soglia l'heading e' inaffidabile: resta isotropo
            hx, hy = direzione[0] / norma_direzione, direzione[1] / norma_direzione

    distanza_massima = min(raggio_sottocelle, math.log(0.02) / math.log(decadimento))
    distanze = {(rf_centro, cf_centro): 0.0}
    coda = [(0.0, rf_centro, cf_centro)]
    while coda:
        dist, r, c = heapq.heappop(coda)
        if dist > distanze.get((r, c), math.inf):
            continue
        for dr, dc, passo in _DIREZIONI_DIFFUSIONE:
            rv, cv = r + dr, c + dc
            if hx or hy:
                # colonna = asse x, riga = asse y, come l'heading (vx, vy)
                norma_passo = math.hypot(dc, dr)
                cos_theta = (dc * hx + dr * hy) / norma_passo
                # sconto in avanti e sovrapprezzo di lato: senza il secondo la macchia si allungherebbe
                # senza restringersi, quindi crescerebbe in ogni direzione
                fattore_direzionale = max(0.35, (1.0 + ALLUNGAMENTO_BLOB) - 2 * ALLUNGAMENTO_BLOB * cos_theta * cos_theta)
                passo = passo * fattore_direzionale
            nuova_dist = dist + passo
            if nuova_dist > distanza_massima:
                continue
            if not (0 <= rv < y_tot_fine and 0 <= cv < x_tot_fine) or nuova_dist >= distanze.get((rv, cv), math.inf) or sottocella_e_muro(griglia, rv, cv, sottocelle_per_lato):
                continue
            if dr != 0 and dc != 0 and (sottocella_e_muro(griglia, r + dr, c, sottocelle_per_lato) or sottocella_e_muro(griglia, r, c + dc, sottocelle_per_lato)):
                continue  # niente tagli d'angolo: la diagonale non deve sfiorare lo spigolo di un muro
            distanze[(rv, cv)] = nuova_dist
            heapq.heappush(coda, (nuova_dist, rv, cv))
    return {cella: decadimento ** dist for cella, dist in distanze.items()}

def sposta_con_vettore(x, y, vx, vy, forza, griglia):
    """Applica lo spostamento (vx, vy) * forza (repulsione o attrazione), senza attraversare muri."""
    if vx == 0 and vy == 0:
        return x, y
    nx, ny = x + vx * forza, y + vy * forza
    r, c = int(ny // DIM_NODO), int(nx // DIM_NODO)
    if 0 <= r < Y_TOT and 0 <= c < X_TOT and griglia[r][c].tipo != "muro":
        return nx, ny
    return x, y

def passo_movimento(x, y, percorso, velocita, nodo_target, griglia):
    """Avanza di un passo lungo il percorso, consumando i nodi raggiunti.
    Ritorna (x, y, arrivato, bloccato); 'bloccato' segnala che il passo taglierebbe un muro e che il
    chiamante deve ricalcolare subito invece di aspettare il prossimo intervallo."""
    if len(percorso) > 1:
        tx, ty = percorso[1].cx, percorso[1].cy
    elif len(percorso) == 1:
        tx, ty = nodo_target.cx, nodo_target.cy
    else:
        return x, y, False, False

    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx**2 + dy**2)

    if len(percorso) <= 1 and dist <= RANGE_EVITAMENTO_PERSONE:
        # gia' entro il raggio di repulsione dalla meta: arrivata, senza spostarla sul punto esatto
        # (piu' entita' dirette allo stesso target si sovrapporrebbero tutte nello stesso pixel)
        return x, y, True, False

    if dist > velocita:
        nx, ny = x + (dx / dist) * velocita, y + (dy / dist) * velocita
        dim = griglia[0][0].dim
        r, c = int(ny // dim), int(nx // dim)
        if not (0 <= r < len(griglia) and 0 <= c < len(griglia[0])) or griglia[r][c].tipo == "muro":
            return x, y, False, True
        return nx, ny, False, False

    if len(percorso) > 1:
        percorso.pop(0)
    return tx, ty, len(percorso) <= 1, False

def fattore_velocita_da_conflitto(x, y, percorso, tracciamento_lidar, velocita_nominale, raggio_robot, raggio_persona,
                                   distanza_lookahead_px=DISTANZA_LOOKAHEAD_VELOCITA_PX):
    """Frazione della velocita' nominale del robot, fra FATTORE_VELOCITA_ROBOT_MIN e MAX.

    E' un controllo spazio-temporale: la traiettoria prevista di ogni persona tracciata viene confrontata
    con la posizione del robot lungo il proprio percorso allo stesso istante, non nello stesso punto.
    Tre esiti: nessun conflitto previsto -> velocita' nominale; conflitto evitabile accelerando ->
    sprint per passare per primo; conflitto inevitabile -> rallenta, tanto piu' quanto e' imminente."""
    if not percorso or velocita_nominale <= 0:
        return 1.0
    persone_dinamiche = [t for t in tracciamento_lidar.values() if not t.get("ferma", False)]
    if not persone_dinamiche:
        return 1.0

    velocita_sprint = velocita_nominale * FATTORE_VELOCITA_ROBOT_MAX
    soglia_sicurezza = raggio_robot + raggio_persona + MARGINE_SICUREZZA_CONFLITTO_PX
    tempo_minimo_conflitto_nominale = None
    tempo_minimo_conflitto_sprint = None
    x_prec, y_prec = x, y
    distanza_percorsa = 0.0
    for nodo in percorso:
        if distanza_percorsa >= distanza_lookahead_px:
            break
        lunghezza_segmento = math.hypot(nodo.cx - x_prec, nodo.cy - y_prec)
        passi_segmento = max(1, int(lunghezza_segmento / DIM_NODO))
        for passo_i in range(1, passi_segmento + 1):
            if distanza_percorsa >= distanza_lookahead_px:
                break
            t = passo_i / passi_segmento
            xc = x_prec + (nodo.cx - x_prec) * t
            yc = y_prec + (nodo.cy - y_prec) * t
            distanza_percorsa += lunghezza_segmento / passi_segmento
            tempo_arrivo_nominale = distanza_percorsa / velocita_nominale
            tempo_arrivo_sprint = distanza_percorsa / velocita_sprint
            for traccia in persone_dinamiche:
                if tempo_minimo_conflitto_nominale is None or tempo_arrivo_nominale < tempo_minimo_conflitto_nominale:
                    xp, yp = previsione_posizione_kalman(traccia, tempo_arrivo_nominale)
                    if math.hypot(xc - xp, yc - yp) < soglia_sicurezza:
                        tempo_minimo_conflitto_nominale = tempo_arrivo_nominale
                if tempo_minimo_conflitto_sprint is None or tempo_arrivo_sprint < tempo_minimo_conflitto_sprint:
                    xp_s, yp_s = previsione_posizione_kalman(traccia, tempo_arrivo_sprint)
                    if math.hypot(xc - xp_s, yc - yp_s) < soglia_sicurezza:
                        tempo_minimo_conflitto_sprint = tempo_arrivo_sprint
        x_prec, y_prec = nodo.cx, nodo.cy

    if tempo_minimo_conflitto_nominale is None:
        return 1.0  # nessun conflitto previsto entro l'orizzonte
    if tempo_minimo_conflitto_sprint is None:
        return FATTORE_VELOCITA_ROBOT_MAX  # sprintando si passa prima della persona
    urgenza = 1.0 - min(1.0, tempo_minimo_conflitto_nominale / FRAME_REAZIONE_VELOCITA_ROBOT)
    return 1.0 - urgenza * (1.0 - FATTORE_VELOCITA_ROBOT_MIN)

def carica_planimetria(cartella=CARTELLA_PLANIMETRIE):
    """Prima immagine trovata nella cartella delle planimetrie, da usare come sfondo su cui ricalcare i
    muri a mano. Ritorna None se la cartella manca o non contiene immagini leggibili: l'overlay e'
    facoltativo e la modalita' Povo funziona lo stesso."""
    if not os.path.isdir(cartella):
        return None
    for nome in sorted(os.listdir(cartella)):
        if nome.lower().endswith((".png", ".jpg", ".jpeg")):
            try:
                return pygame.image.load(os.path.join(cartella, nome)).convert_alpha()
            except pygame.error:
                continue
    return None

def salva_mappa(griglia, path, con_dimensioni=False):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    muri = [[n.r, n.c] for riga in griglia for n in riga if n.tipo == "muro"]
    with open(path, "w") as f:
        # lo slot Povo salva anche le dimensioni usate, cosi' il caricamento puo' rifiutare una mappa
        # nata su una griglia diversa invece di piazzarne i muri fuori posto
        json.dump({"x_tot": X_TOT, "y_tot": Y_TOT, "muri": muri} if con_dimensioni else muri, f)

def salva_mappa_training(griglia, cartella=CARTELLA_MAPPE_TRAINING):
    """Salva la mappa con nome progressivo nella cartella delle mappe di training, che
    training_scenari.py scandisce: ogni mappa salvata qui diventa subito usabile per gli scenari."""
    os.makedirs(cartella, exist_ok=True)
    i = 1
    while os.path.exists(os.path.join(cartella, f"mappa_{i:02d}.json")):
        i += 1
    path = os.path.join(cartella, f"mappa_{i:02d}.json")
    salva_mappa(griglia, path)
    return path

def salva_configurazione_pannello(configurazione, path):
    with open(path, "w") as f:
        json.dump(configurazione, f)

def carica_configurazione_pannello(path):
    if not os.path.exists(path):
        return None
    with open(path, "r") as f:
        return json.load(f)

def leggi_mappa(path):
    """(muri, x_tot, y_tot) letti dal file. Le dimensioni sono None nel formato vecchio, che contiene
    la sola lista dei muri."""
    with open(path, "r") as f:
        dati = json.load(f)
    if isinstance(dati, dict):
        return dati.get("muri", []), dati.get("x_tot"), dati.get("y_tot")
    return dati, None, None

def dimensioni_mappa(path):
    """Griglia su cui e' nata una mappa salvata. Il formato vecchio non riporta le dimensioni: quelle
    mappe sono tutte anteriori alla modalita' Povo, quindi sono per forza sulla griglia standard."""
    _, x_tot_salvato, y_tot_salvato = leggi_mappa(path)
    if x_tot_salvato is None:
        return X_TOT_STANDARD, Y_TOT_STANDARD
    return x_tot_salvato, y_tot_salvato

def carica_mappa(griglia, path):
    if not os.path.exists(path):
        return False
    muri, _, _ = leggi_mappa(path)
    for riga in griglia:
        for n in riga:
            n.tipo = "libero"
    for r, c in muri:
        if 0 <= r < Y_TOT and 0 <= c < X_TOT:
            griglia[r][c].tipo = "muro"
    crea_bordi(griglia)
    return True

# --- pool di processi per il ricalcolo parallelo dei percorsi della folla ---
# Ogni worker costruisce la propria copia della griglia una volta sola all'avvio; ad ogni dispatch
# viaggia solo il payload leggero. Va ricreato quando i muri cambiano: finche' sono in modifica si
# ricade sul calcolo sequenziale
_pool_worker_griglia = None

def _pool_inizializza_worker(celle_muro, x_tot, y_tot):
    global _pool_worker_griglia
    # con lo spawn di Windows il worker re-importa il modulo e riparte dalle dimensioni di default:
    # vanno riapplicate qui, altrimenti in modalita' Povo la sua griglia avrebbe la misura sbagliata
    applica_dimensioni_griglia(x_tot, y_tot)
    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)
    for r, c in celle_muro:
        griglia[r][c].tipo = "muro"
    _pool_worker_griglia = griglia

def _pool_calcola_percorso(richiesta):
    inizio_pos_pixel, target_griglia, rumore_seed, celle_bloccate = richiesta
    return algoritmo_a_star(_pool_worker_griglia, inizio_pos_pixel, target_griglia, rumore_seed=rumore_seed, celle_bloccate=celle_bloccate)

def _celle_muro_di(griglia):
    return [(n.r, n.c) for riga in griglia for n in riga if n.tipo == "muro"]

def _crea_pool_persone(griglia):
    n_worker = min(8, os.cpu_count() or 4)
    return mp.Pool(n_worker, initializer=_pool_inizializza_worker, initargs=(_celle_muro_di(griglia), X_TOT, Y_TOT))

def main():
    # import locali e non di modulo: i worker del pool re-importano main.py per intero ma non usano
    # l'AI, e caricare torch/CUDA in ognuno allungava l'avvio di decine di secondi
    import numpy as np
    import torch
    from ai_predittiva.allena_previsione import CorrezioneKalman, prepara_input, FINESTRA_STORICO_FRAME, FILE_MODELLO
    torch.set_num_threads(1)  # batch minuscoli: sincronizzare i thread costa piu' del calcolo

    pygame.init()
    fullscreen = False

    # Adatta la finestra iniziale allo schermo (max 90% di larghezza/altezza disponibili)
    info = pygame.display.Info()
    win_w = min(LARGHEZZA, int(info.current_w * 0.9))
    win_h = min(ALTEZZA, int(info.current_h * 0.9))
    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
    pygame.key.start_text_input()
    pygame.display.set_caption("Simulazione - Zoom: Rote. | Pan: Tasto DX | Muri: Tasto W | P: Mappa Povo | F11: Fullscreen | F5: Salva | F9: Carica | T: Salva mappa training | TAB: Pannello")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 32)
    font_piccolo = pygame.font.SysFont(None, 24)
    font_grande = pygame.font.SysFont(None, 48)

    chiedendo_password = False
    input_password = ""
    errore_timer = 0
    menu_salvataggio_aperto = False  # F5: prima si sceglie lo slot, poi si chiede la password
    menu_caricamento_aperto = False  # F9: si sceglie lo slot da caricare
    slot_da_salvare = None
    slot_vuoto_timer = 0
    modalita_rettangolo = False  # M: in attesa del primo/secondo click per riempire un'area rettangolare
    primo_punto_rettangolo = None
    messaggio_mappa_training_timer = 0
    messaggio_mappa_training_testo = ""
    frame_inizio_target = None  # frame in cui e' stato assegnato il target attuale (click): serve a calcolare il tempo di percorrenza all'arrivo
    messaggio_arrivo_timer = 0
    messaggio_arrivo_testo = ""

    # --- modalita' Povo: griglia dedicata alla planimetria reale e relativo overlay ---
    modalita_povo = False
    # la commutazione e il caricamento non avvengono dentro la gestione eventi ma subito dopo, in un
    # punto solo: cosi' scegliere una mappa nata sull'altra griglia porta con se' il cambio di griglia
    modalita_povo_richiesta = False
    mappa_da_caricare = None
    rigenera_popolazione = False
    planimetria_originale = None   # immagine a risoluzione piena, sorgente di ogni riscalatura
    planimetria_scalata = None     # versione a schermo, rigenerata solo al variare di zoom o scala
    planimetria_chiave_scalata = None
    planimetria_visibile = True
    alpha_planimetria = ALPHA_PLANIMETRIA_DEFAULT
    trascinando_alpha_planimetria = False
    scala_planimetria_x = scala_planimetria_y = 1.0
    offset_planimetria_x = offset_planimetria_y = 0.0
    messaggio_mappa_timer = 0  # avvisi su caricamento mappa rifiutato o modalita' cambiata
    messaggio_mappa_testo = ("", VERDE)

    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)
    # griglia su cui pianifica solo il robot; la folla continua a usare `griglia`
    griglia_fine_attiva = GRIGLIA_FINE_DEFAULT
    fattore_robot = FATTORE_GRIGLIA_ROBOT if griglia_fine_attiva else 1
    dim_robot = DIM_NODO // fattore_robot
    griglia_robot = crea_griglia_robot(griglia, fattore_robot)
    griglia_robot_da_ricostruire = False

    # correzione AI sopra al Kalman: allenata offline, qui solo inferenza. Se il file non esiste, il
    # toggle in pannello resta senza effetto e il sistema usa il solo Kalman
    modello_correzione_ai = CorrezioneKalman()
    modello_ai_disponibile = os.path.exists(FILE_MODELLO)
    if modello_ai_disponibile:
        modello_correzione_ai.load_state_dict(torch.load(FILE_MODELLO, map_location="cpu"))
        modello_correzione_ai.eval()

    # pool per il ricalcolo parallelo dei percorsi; i due flag tengono traccia di quando i muri cambiano,
    # perche' in quel caso la copia della griglia nei worker va rigenerata
    pool_persone = _crea_pool_persone(griglia)
    muri_modificati = False
    frame_ultima_modifica_muro = -9999
    richiesta_percorsi_pendente = None  # batch in volo presso il pool, None se nessuno
    persone_richiesta_percorsi_pendente = []

    # --- VARIABILI ZOOM E PAN ---
    zoom = 1.0
    offset_x, offset_y = 0, 0
    trascinando = False
    ultima_pos_mouse = (0, 0)
    ultima_cella_w = None
    seguendo_robot = False  # F: la telecamera segue il robot, F di nuovo per staccarsi

    robot_x = 1 * DIM_NODO + DIM_NODO // 2
    robot_y = 1 * DIM_NODO + DIM_NODO // 2
    target_pos = None
    percorso = []
    celle_rilevate_precedenti = set()  # celle rilevate dal lidar nel frame precedente, senza rumore
    tracciamento_lidar = {}  # id(persona) -> traccia Kalman, solo per le persone attualmente rilevate
    storico_posizioni_lidar = {}  # id(persona) -> ultime FINESTRA_STORICO_FRAME stime: input della rete
    mappa_pesi_render = {}  # ultima macchia calcolata, per sottocella: persiste fra un ricalcolo e l'altro
    mappa_pesi_render_dim_sottocella = DIM_NODO / SOTTOCELLE_PER_LATO_DEFAULT  # lato usato per quella macchia
    mappa_costo_probabilita = {}  # costo extra per l'A* derivato dalla macchia, per cella

    # --- NUMERO PERSONE/CORRIDORI/FERME/GRUPPI E PANNELLO TECNICO (valori di default) ---
    numero_persone = NUMERO_PERSONE_DEFAULT
    numero_corridori = NUMERO_CORRIDORI_DEFAULT
    numero_persone_ferme = NUMERO_PERSONE_FERME_DEFAULT
    numero_gruppi = NUMERO_GRUPPI_DEFAULT
    pannello_aperto = False
    scroll_pannello = 0  # px di cui il pannello e' alzato, quando non ci sta tutto nella finestra
    moltiplicatore_velocita = 1.0
    trascinando_slider = False
    modificando_persone = False
    input_numero_persone = ""
    modificando_corridori = False
    input_numero_corridori = ""
    modificando_persone_ferme = False
    input_numero_persone_ferme = ""
    modificando_gruppi = False
    input_numero_gruppi = ""
    raggio_sicurezza = RAGGIO_SICUREZZA_DEFAULT
    trascinando_raggio = False
    raggio_robot = RAGGIO_ROBOT_DEFAULT
    trascinando_raggio_robot = False
    raggio_lidar = RAGGIO_LIDAR_DEFAULT
    trascinando_raggio_lidar = False
    raggio_persona = RAGGIO_PERSONA_DEFAULT
    trascinando_raggio_persona = False
    definizione_ellissoidi = SOTTOCELLE_PER_LATO_DEFAULT
    trascinando_definizione_ellissoidi = False
    sicurezza_attiva = True
    lidar_attivo = True
    ellissoidi_attivi = True
    # correzione AI: spenta di default, il toggle riguarda solo la correzione e il Kalman resta attivo
    # sotto. Migliora la previsione ma non i tempi di percorrenza, vedi ai_predittiva/__init__.py
    ai_attiva = False
    controllo_velocita_attivo = True  # rallenta il robot in base al rischio previsto sul percorso
    configurazione_salvataggio_timer = 0  # conferma visiva dopo il click sui pulsanti del pannello
    configurazione_messaggio = ("", VERDE)

    # se esiste una configurazione salvata dal pannello, sovrascrive i default appena impostati
    configurazione_salvata = carica_configurazione_pannello(FILE_CONFIGURAZIONE_PANNELLO)
    if configurazione_salvata:
        numero_persone = configurazione_salvata.get("numero_persone", numero_persone)
        numero_corridori = configurazione_salvata.get("numero_corridori", numero_corridori)
        numero_persone_ferme = configurazione_salvata.get("numero_persone_ferme", numero_persone_ferme)
        numero_gruppi = configurazione_salvata.get("numero_gruppi", numero_gruppi)
        moltiplicatore_velocita = configurazione_salvata.get("moltiplicatore_velocita", moltiplicatore_velocita)
        raggio_sicurezza = configurazione_salvata.get("raggio_sicurezza", raggio_sicurezza)
        raggio_robot = configurazione_salvata.get("raggio_robot", raggio_robot)
        raggio_lidar = configurazione_salvata.get("raggio_lidar", raggio_lidar)
        raggio_persona = configurazione_salvata.get("raggio_persona", raggio_persona)
        definizione_ellissoidi = configurazione_salvata.get("definizione_ellissoidi", definizione_ellissoidi)
        sicurezza_attiva = configurazione_salvata.get("sicurezza_attiva", sicurezza_attiva)
        lidar_attivo = configurazione_salvata.get("lidar_attivo", lidar_attivo)
        ellissoidi_attivi = configurazione_salvata.get("ellissoidi_attivi", ellissoidi_attivi)
        ai_attiva = configurazione_salvata.get("ai_attiva", ai_attiva)
        controllo_velocita_attivo = configurazione_salvata.get("controllo_velocita_attivo", controllo_velocita_attivo)
        planimetria_visibile = configurazione_salvata.get("planimetria_visibile", planimetria_visibile)
        alpha_planimetria = configurazione_salvata.get("alpha_planimetria", alpha_planimetria)
        scala_planimetria_x = configurazione_salvata.get("scala_planimetria_x", scala_planimetria_x)
        scala_planimetria_y = configurazione_salvata.get("scala_planimetria_y", scala_planimetria_y)
        offset_planimetria_x = configurazione_salvata.get("offset_planimetria_x", offset_planimetria_x)
        offset_planimetria_y = configurazione_salvata.get("offset_planimetria_y", offset_planimetria_y)

    # --- popolazione iniziale ---
    persone = [crea_persona(griglia) for _ in range(numero_persone)]
    corridori = [crea_corridore(griglia) for _ in range(numero_corridori)]
    persone_ferme = [crea_persona_ferma(griglia) for _ in range(numero_persone_ferme)]
    gruppi = [crea_gruppo(griglia) for _ in range(numero_gruppi)]

    contatore_frame = 0

    running = True
    while running:
        screen.fill(GRIGIO_PAVIMENTO)
        
        # conversione mappa <-> schermo
        def t_s(x, y): return int(x * zoom + offset_x), int(y * zoom + offset_y)
        def t_m(sx, sy): return (sx - offset_x) / zoom, (sy - offset_y) / zoom

        # 0b. planimetria di sfondo: sta sotto la scacchiera, cosi' la griglia resta leggibile sopra e si
        # possono ricalcare i muri. Segue zoom e pan come la griglia, piu' scala e offset di allineamento
        if modalita_povo and planimetria_visibile and planimetria_originale is not None:
            chiave_scala = (round(zoom, 4), round(scala_planimetria_x, 3), round(scala_planimetria_y, 3))
            if chiave_scala != planimetria_chiave_scalata:
                # riscalatura sempre dall'originale, e solo quando cambia: uno smoothscale per frame
                # costerebbe caro, e ripartire dall'ultima scalata accumulerebbe perdita di qualita'
                larghezza_planimetria = max(1, int(LARGHEZZA * zoom * scala_planimetria_x))
                altezza_planimetria = max(1, int(ALTEZZA * zoom * scala_planimetria_y))
                # oltre il budget di pixel si passa alla scalatura veloce: a zoom alto lo smoothscale
                # costa decimi di secondo, e a quell'ingrandimento la differenza non si vede
                riscala = (pygame.transform.scale
                           if larghezza_planimetria * altezza_planimetria > PIXEL_MAX_SMOOTHSCALE
                           else pygame.transform.smoothscale)
                planimetria_scalata = riscala(planimetria_originale, (larghezza_planimetria, altezza_planimetria))
                planimetria_chiave_scalata = chiave_scala
            planimetria_scalata.set_alpha(int(alpha_planimetria))
            screen.blit(planimetria_scalata, (int(offset_x + offset_planimetria_x * zoom),
                                              int(offset_y + offset_planimetria_y * zoom)))

        # 1. griglia, solo le celle visibili nella viewport
        cella_px = DIM_NODO * zoom
        col_min = max(0, int(-offset_x / cella_px))
        col_max = min(X_TOT - 1, int((screen.get_width() - offset_x) / cella_px))
        row_min = max(0, int(-offset_y / cella_px))
        row_max = min(Y_TOT - 1, int((screen.get_height() - offset_y) / cella_px))

        for r in range(row_min, row_max + 1):
            for c in range(col_min, col_max + 1):
                n = griglia[r][c]
                # bordo preso dalla cella successiva: combaciano senza fessure da arrotondamento
                sx, sy = t_s(n.x, n.y)
                ex, ey = t_s(n.x + DIM_NODO, n.y + DIM_NODO)
                rect = (sx, sy, ex - sx, ey - sy)
                if n.tipo == "muro":
                    pygame.draw.rect(screen, NERO, rect)
                else:
                    pygame.draw.rect(screen, GRIGIO_BORDO_CELLA, rect, 1)
                    # griglia fine del robot, disegnata solo attorno a lui e solo se leggibile a schermo:
                    # su tutta la viewport sarebbero migliaia di linee per frame
                    if (fattore_robot > 1 and (ex - sx) >= 16
                            and abs(n.cx - robot_x) < CELLE_SOTTOGRIGLIA_VISIBILI * DIM_NODO
                            and abs(n.cy - robot_y) < CELLE_SOTTOGRIGLIA_VISIBILI * DIM_NODO):
                        for k in range(1, fattore_robot):
                            q = k / fattore_robot
                            xk = sx + int((ex - sx) * q)
                            yk = sy + int((ey - sy) * q)
                            pygame.draw.line(screen, GRIGIO_SOTTOGRIGLIA, (xk, sy), (xk, ey - 1))
                            pygame.draw.line(screen, GRIGIO_SOTTOGRIGLIA, (sx, yk), (ex - 1, yk))

        # layout del pannello tecnico, ricalcolato ogni frame per seguire il ridimensionamento. Su una
        # finestra piu' bassa del pannello i controlli in fondo starebbero fuori schermo: lo scorrimento
        # alza tutto il blocco, e ogni rettangolo figlio lo segue perche' e' ancorato a panel_rect.y
        panel_w, panel_h = 370, 850
        scroll_pannello_max = max(0, panel_h - (screen.get_height() - 40))
        scroll_pannello = max(0, min(scroll_pannello_max, scroll_pannello))
        panel_rect = pygame.Rect(screen.get_width() - panel_w - 20, 20 - scroll_pannello, panel_w, panel_h)
        slider_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 75, panel_w - 30, 8)
        numero_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 100, 70, 30)
        numero_corridori_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 135, 70, 30)
        numero_gruppi_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 170, 70, 30)
        numero_ferme_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 205, 70, 30)
        slider_sicurezza_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 275, panel_w - 30, 8)
        slider_robot_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 350, panel_w - 30, 8)
        slider_lidar_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 425, panel_w - 30, 8)
        slider_persona_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 500, panel_w - 30, 8)
        slider_definizione_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 575, panel_w - 30, 8)
        checkbox_sicurezza_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 243, 20, 20)
        checkbox_lidar_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 393, 20, 20)
        checkbox_ellissoidi_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 543, 20, 20)
        checkbox_ai_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 608, 20, 20)
        checkbox_velocita_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 645, 20, 20)
        checkbox_griglia_fine_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 682, 20, 20)
        checkbox_planimetria_rect = pygame.Rect(panel_rect.x + panel_w - 35, panel_rect.y + 719, 20, 20)
        slider_planimetria_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 751, panel_w - 30, 8)
        # pulsanti sempre in fondo al pannello, per avere l'ambiente di test preferito pronto ad ogni avvio
        larghezza_pulsante_config = (panel_w - 30 - 20) // 3
        button_salva_config_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 800, larghezza_pulsante_config, 35)
        button_carica_config_rect = pygame.Rect(button_salva_config_rect.right + 10, panel_rect.y + 800, larghezza_pulsante_config, 35)
        button_reset_config_rect = pygame.Rect(button_carica_config_rect.right + 10, panel_rect.y + 800, larghezza_pulsante_config, 35)

        # 2. Eventi
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

            if event.type == pygame.VIDEORESIZE and not fullscreen:
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)

            if event.type == pygame.MOUSEBUTTONDOWN and modalita_rettangolo:
                if event.button == 1:
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    r = max(1, min(Y_TOT - 2, r))
                    c = max(1, min(X_TOT - 2, c))
                    if primo_punto_rettangolo is None:
                        primo_punto_rettangolo = (r, c)
                    else:
                        r1, c1 = primo_punto_rettangolo
                        r_min, r_max = min(r1, r), max(r1, r)
                        c_min, c_max = min(c1, c), max(c1, c)
                        # il tipo del primo punto cliccato decide il riempimento dell'intero rettangolo
                        nuovo_tipo = "libero" if griglia[r1][c1].tipo == "muro" else "muro"
                        for rr in range(r_min, r_max + 1):
                            for cc in range(c_min, c_max + 1):
                                griglia[rr][cc].tipo = nuovo_tipo
                        muri_modificati = True
                        frame_ultima_modifica_muro = contatore_frame
                        modalita_rettangolo = False
                        primo_punto_rettangolo = None

            if event.type == pygame.MOUSEBUTTONDOWN and not modalita_rettangolo:
                if event.button == 1 and modificando_persone and not numero_box_rect.collidepoint(event.pos):
                    modificando_persone = False  # click fuori dal campo: annulla la modifica
                if event.button == 1 and modificando_corridori and not numero_corridori_box_rect.collidepoint(event.pos):
                    modificando_corridori = False
                if event.button == 1 and modificando_persone_ferme and not numero_ferme_box_rect.collidepoint(event.pos):
                    modificando_persone_ferme = False
                if event.button == 1 and modificando_gruppi and not numero_gruppi_box_rect.collidepoint(event.pos):
                    modificando_gruppi = False

                if event.button in (4, 5) and pannello_aperto and panel_rect.collidepoint(event.pos):
                    # rotella sopra il pannello: scorre il pannello invece di zoomare la mappa
                    scroll_pannello += PASSO_SCROLL_PANNELLO if event.button == 5 else -PASSO_SCROLL_PANNELLO
                elif event.button == 1 and pannello_aperto and panel_rect.collidepoint(event.pos):
                    if numero_box_rect.collidepoint(event.pos):
                        modificando_persone = True
                        input_numero_persone = str(numero_persone)
                    elif numero_corridori_box_rect.collidepoint(event.pos):
                        modificando_corridori = True
                        input_numero_corridori = str(numero_corridori)
                    elif numero_gruppi_box_rect.collidepoint(event.pos):
                        modificando_gruppi = True
                        input_numero_gruppi = str(numero_gruppi)
                    elif numero_ferme_box_rect.collidepoint(event.pos):
                        modificando_persone_ferme = True
                        input_numero_persone_ferme = str(numero_persone_ferme)
                    elif slider_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_slider = True
                        rel = (event.pos[0] - slider_rect.x) / slider_rect.width
                        moltiplicatore_velocita = VEL_MULT_MIN + max(0, min(1, rel)) * (VEL_MULT_MAX - VEL_MULT_MIN)
                    elif slider_sicurezza_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_raggio = True
                        rel = (event.pos[0] - slider_sicurezza_rect.x) / slider_sicurezza_rect.width
                        raggio_sicurezza = RAGGIO_SICUREZZA_MIN + max(0, min(1, rel)) * (RAGGIO_SICUREZZA_MAX - RAGGIO_SICUREZZA_MIN)
                    elif slider_robot_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_raggio_robot = True
                        rel = (event.pos[0] - slider_robot_rect.x) / slider_robot_rect.width
                        raggio_robot = RAGGIO_ROBOT_MIN + max(0, min(1, rel)) * (RAGGIO_ROBOT_MAX - RAGGIO_ROBOT_MIN)
                    elif slider_lidar_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_raggio_lidar = True
                        rel = (event.pos[0] - slider_lidar_rect.x) / slider_lidar_rect.width
                        raggio_lidar = RAGGIO_LIDAR_MIN + max(0, min(1, rel)) * (RAGGIO_LIDAR_MAX - RAGGIO_LIDAR_MIN)
                    elif slider_persona_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_raggio_persona = True
                        rel = (event.pos[0] - slider_persona_rect.x) / slider_persona_rect.width
                        raggio_persona = RAGGIO_PERSONA_MIN + max(0, min(1, rel)) * (RAGGIO_PERSONA_MAX - RAGGIO_PERSONA_MIN)
                    elif slider_definizione_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_definizione_ellissoidi = True
                        rel = (event.pos[0] - slider_definizione_rect.x) / slider_definizione_rect.width
                        definizione_ellissoidi = SOTTOCELLE_PER_LATO_MIN + max(0, min(1, rel)) * (SOTTOCELLE_PER_LATO_MAX - SOTTOCELLE_PER_LATO_MIN)
                    elif slider_planimetria_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_alpha_planimetria = True
                        rel = (event.pos[0] - slider_planimetria_rect.x) / slider_planimetria_rect.width
                        alpha_planimetria = ALPHA_PLANIMETRIA_MIN + max(0, min(1, rel)) * (ALPHA_PLANIMETRIA_MAX - ALPHA_PLANIMETRIA_MIN)
                    elif checkbox_planimetria_rect.collidepoint(event.pos):
                        planimetria_visibile = not planimetria_visibile
                    elif checkbox_sicurezza_rect.collidepoint(event.pos):
                        sicurezza_attiva = not sicurezza_attiva
                    elif checkbox_lidar_rect.collidepoint(event.pos):
                        lidar_attivo = not lidar_attivo
                    elif checkbox_ellissoidi_rect.collidepoint(event.pos):
                        ellissoidi_attivi = not ellissoidi_attivi
                    elif checkbox_ai_rect.collidepoint(event.pos):
                        ai_attiva = not ai_attiva
                    elif checkbox_velocita_rect.collidepoint(event.pos):
                        controllo_velocita_attivo = not controllo_velocita_attivo
                    elif checkbox_griglia_fine_rect.collidepoint(event.pos):
                        griglia_fine_attiva = not griglia_fine_attiva  # la griglia viene rifatta a inizio frame
                    elif button_salva_config_rect.collidepoint(event.pos):
                        salva_configurazione_pannello({
                            "numero_persone": numero_persone, "numero_corridori": numero_corridori,
                            "numero_persone_ferme": numero_persone_ferme, "numero_gruppi": numero_gruppi,
                            "moltiplicatore_velocita": moltiplicatore_velocita,
                            "raggio_sicurezza": raggio_sicurezza, "raggio_robot": raggio_robot,
                            "raggio_lidar": raggio_lidar, "raggio_persona": raggio_persona,
                            "definizione_ellissoidi": definizione_ellissoidi,
                            "sicurezza_attiva": sicurezza_attiva, "lidar_attivo": lidar_attivo,
                            "ellissoidi_attivi": ellissoidi_attivi, "ai_attiva": ai_attiva,
                            "controllo_velocita_attivo": controllo_velocita_attivo,
                            "griglia_fine_attiva": griglia_fine_attiva,
                            "planimetria_visibile": planimetria_visibile,
                            "alpha_planimetria": alpha_planimetria,
                            "scala_planimetria_x": scala_planimetria_x,
                            "scala_planimetria_y": scala_planimetria_y,
                            "offset_planimetria_x": offset_planimetria_x,
                            "offset_planimetria_y": offset_planimetria_y,
                        }, FILE_CONFIGURAZIONE_PANNELLO)
                        configurazione_messaggio = ("Configurazione salvata", VERDE)
                        configurazione_salvataggio_timer = 90
                    elif button_carica_config_rect.collidepoint(event.pos):
                        configurazione_da_caricare = carica_configurazione_pannello(FILE_CONFIGURAZIONE_PANNELLO)
                        if configurazione_da_caricare:
                            numero_persone = configurazione_da_caricare.get("numero_persone", numero_persone)
                            numero_corridori = configurazione_da_caricare.get("numero_corridori", numero_corridori)
                            numero_persone_ferme = configurazione_da_caricare.get("numero_persone_ferme", numero_persone_ferme)
                            numero_gruppi = configurazione_da_caricare.get("numero_gruppi", numero_gruppi)
                            sincronizza_persone(persone, numero_persone, griglia)
                            sincronizza_persone(corridori, numero_corridori, griglia, fabbrica=crea_corridore)
                            sincronizza_persone_ferme(persone_ferme, numero_persone_ferme, griglia)
                            sincronizza_gruppi(gruppi, numero_gruppi, griglia)
                            moltiplicatore_velocita = configurazione_da_caricare.get("moltiplicatore_velocita", moltiplicatore_velocita)
                            raggio_sicurezza = configurazione_da_caricare.get("raggio_sicurezza", raggio_sicurezza)
                            raggio_robot = configurazione_da_caricare.get("raggio_robot", raggio_robot)
                            raggio_lidar = configurazione_da_caricare.get("raggio_lidar", raggio_lidar)
                            raggio_persona = configurazione_da_caricare.get("raggio_persona", raggio_persona)
                            definizione_ellissoidi = configurazione_da_caricare.get("definizione_ellissoidi", definizione_ellissoidi)
                            sicurezza_attiva = configurazione_da_caricare.get("sicurezza_attiva", sicurezza_attiva)
                            lidar_attivo = configurazione_da_caricare.get("lidar_attivo", lidar_attivo)
                            ellissoidi_attivi = configurazione_da_caricare.get("ellissoidi_attivi", ellissoidi_attivi)
                            ai_attiva = configurazione_da_caricare.get("ai_attiva", ai_attiva)
                            controllo_velocita_attivo = configurazione_da_caricare.get("controllo_velocita_attivo", controllo_velocita_attivo)
                            griglia_fine_attiva = configurazione_da_caricare.get("griglia_fine_attiva", griglia_fine_attiva)
                            planimetria_visibile = configurazione_da_caricare.get("planimetria_visibile", planimetria_visibile)
                            alpha_planimetria = configurazione_da_caricare.get("alpha_planimetria", alpha_planimetria)
                            scala_planimetria_x = configurazione_da_caricare.get("scala_planimetria_x", scala_planimetria_x)
                            scala_planimetria_y = configurazione_da_caricare.get("scala_planimetria_y", scala_planimetria_y)
                            offset_planimetria_x = configurazione_da_caricare.get("offset_planimetria_x", offset_planimetria_x)
                            offset_planimetria_y = configurazione_da_caricare.get("offset_planimetria_y", offset_planimetria_y)
                            configurazione_messaggio = ("Configurazione caricata", VERDE)
                        else:
                            configurazione_messaggio = ("Nessuna configurazione salvata", ROSSO)
                        configurazione_salvataggio_timer = 90
                    elif button_reset_config_rect.collidepoint(event.pos):
                        # riporta ai default solo l'ambiente corrente: il file salvato resta intatto
                        numero_persone, numero_corridori = NUMERO_PERSONE_DEFAULT, NUMERO_CORRIDORI_DEFAULT
                        numero_persone_ferme, numero_gruppi = NUMERO_PERSONE_FERME_DEFAULT, NUMERO_GRUPPI_DEFAULT
                        sincronizza_persone(persone, numero_persone, griglia)
                        sincronizza_persone(corridori, numero_corridori, griglia, fabbrica=crea_corridore)
                        sincronizza_persone_ferme(persone_ferme, numero_persone_ferme, griglia)
                        sincronizza_gruppi(gruppi, numero_gruppi, griglia)
                        moltiplicatore_velocita = 1.0
                        raggio_sicurezza, raggio_robot = RAGGIO_SICUREZZA_DEFAULT, RAGGIO_ROBOT_DEFAULT
                        raggio_lidar, raggio_persona = RAGGIO_LIDAR_DEFAULT, RAGGIO_PERSONA_DEFAULT
                        definizione_ellissoidi = SOTTOCELLE_PER_LATO_DEFAULT
                        sicurezza_attiva = lidar_attivo = ellissoidi_attivi = ai_attiva = controllo_velocita_attivo = True
                        griglia_fine_attiva = GRIGLIA_FINE_DEFAULT
                        planimetria_visibile = True
                        alpha_planimetria = ALPHA_PLANIMETRIA_DEFAULT
                        scala_planimetria_x = scala_planimetria_y = 1.0
                        offset_planimetria_x = offset_planimetria_y = 0.0
                        planimetria_chiave_scalata = None
                        configurazione_messaggio = ("Configurazione ripristinata", VERDE)
                        configurazione_salvataggio_timer = 90
                elif event.button == 4: zoom *= 1.1 # Zoom In
                elif event.button == 5: zoom /= 1.1 # Zoom Out
                elif event.button == 3: # Inizio Pan
                    trascinando = True
                    ultima_pos_mouse = event.pos
                elif event.button == 1 and not pygame.key.get_pressed()[pygame.K_w]: # Meta (esclude un click mentre si disegnano muri con W, altrimenti comanda il robot per sbaglio)
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    if 0 <= r < Y_TOT and 0 <= c < X_TOT:
                        target_pos = (r, c)
                        percorso = []  # forza ricalcolo immediato verso il nuovo target
                        frame_inizio_target = contatore_frame

            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: trascinando = False
                if event.button == 1:
                    trascinando_slider = False
                    trascinando_raggio = False
                    trascinando_raggio_robot = False
                    trascinando_raggio_lidar = False
                    trascinando_raggio_persona = False
                    trascinando_definizione_ellissoidi = False
                    trascinando_alpha_planimetria = False

            if event.type == pygame.MOUSEMOTION:
                if trascinando_slider:
                    rel = (event.pos[0] - slider_rect.x) / slider_rect.width
                    moltiplicatore_velocita = VEL_MULT_MIN + max(0, min(1, rel)) * (VEL_MULT_MAX - VEL_MULT_MIN)
                    continue
                if trascinando_raggio:
                    rel = (event.pos[0] - slider_sicurezza_rect.x) / slider_sicurezza_rect.width
                    raggio_sicurezza = RAGGIO_SICUREZZA_MIN + max(0, min(1, rel)) * (RAGGIO_SICUREZZA_MAX - RAGGIO_SICUREZZA_MIN)
                    continue
                if trascinando_raggio_robot:
                    rel = (event.pos[0] - slider_robot_rect.x) / slider_robot_rect.width
                    raggio_robot = RAGGIO_ROBOT_MIN + max(0, min(1, rel)) * (RAGGIO_ROBOT_MAX - RAGGIO_ROBOT_MIN)
                    continue
                if trascinando_raggio_lidar:
                    rel = (event.pos[0] - slider_lidar_rect.x) / slider_lidar_rect.width
                    raggio_lidar = RAGGIO_LIDAR_MIN + max(0, min(1, rel)) * (RAGGIO_LIDAR_MAX - RAGGIO_LIDAR_MIN)
                    continue
                if trascinando_raggio_persona:
                    rel = (event.pos[0] - slider_persona_rect.x) / slider_persona_rect.width
                    raggio_persona = RAGGIO_PERSONA_MIN + max(0, min(1, rel)) * (RAGGIO_PERSONA_MAX - RAGGIO_PERSONA_MIN)
                    continue
                if trascinando_definizione_ellissoidi:
                    rel = (event.pos[0] - slider_definizione_rect.x) / slider_definizione_rect.width
                    definizione_ellissoidi = SOTTOCELLE_PER_LATO_MIN + max(0, min(1, rel)) * (SOTTOCELLE_PER_LATO_MAX - SOTTOCELLE_PER_LATO_MIN)
                    continue
                if trascinando_alpha_planimetria:
                    rel = (event.pos[0] - slider_planimetria_rect.x) / slider_planimetria_rect.width
                    alpha_planimetria = ALPHA_PLANIMETRIA_MIN + max(0, min(1, rel)) * (ALPHA_PLANIMETRIA_MAX - ALPHA_PLANIMETRIA_MIN)
                    continue

                # Gestione Panning
                if trascinando:
                    dx, dy = event.pos[0] - ultima_pos_mouse[0], event.pos[1] - ultima_pos_mouse[1]
                    offset_x += dx
                    offset_y += dy
                    ultima_pos_mouse = event.pos

                # muri: posizionamento e rimozione trascinando con W premuto
                keys = pygame.key.get_pressed()
                if keys[pygame.K_w]:
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    # i bordi esterni non si modificano
                    if 0 < r < Y_TOT - 1 and 0 < c < X_TOT - 1 and (r, c) != ultima_cella_w:
                        griglia[r][c].tipo = "libero" if griglia[r][c].tipo == "muro" else "muro"
                        muri_modificati = True
                        frame_ultima_modifica_muro = contatore_frame
                        ultima_cella_w = (r, c)

            if event.type == pygame.TEXTINPUT:
                if chiedendo_password:
                    input_password += event.text
                elif modificando_persone and event.text.isdigit():
                    input_numero_persone += event.text
                elif modificando_corridori and event.text.isdigit():
                    input_numero_corridori += event.text
                elif modificando_persone_ferme and event.text.isdigit():
                    input_numero_persone_ferme += event.text
                elif modificando_gruppi and event.text.isdigit():
                    input_numero_gruppi += event.text

            if event.type == pygame.KEYDOWN:
                if chiedendo_password:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_password == PASSWORD_SALVATAGGIO:
                            # tutti gli slot registrano le dimensioni: al caricamento e' cosi' possibile
                            # riconoscere su quale griglia e' nata la mappa e adeguarsi
                            salva_mappa(griglia, FILE_MAPPA_SLOT[slot_da_salvare], con_dimensioni=True)
                            chiedendo_password = False
                        else:
                            errore_timer = 90
                        input_password = ""
                    elif event.key == pygame.K_ESCAPE:
                        chiedendo_password = False
                        input_password = ""
                    elif event.key == pygame.K_BACKSPACE:
                        input_password = input_password[:-1]
                    continue

                if menu_salvataggio_aperto or menu_caricamento_aperto:
                    indice_slot = {
                        pygame.K_1: 0, pygame.K_KP1: 0,
                        pygame.K_2: 1, pygame.K_KP2: 1,
                        pygame.K_3: 2, pygame.K_KP3: 2,
                        pygame.K_p: SLOT_MAPPA_POVO,
                    }.get(event.key)
                    if indice_slot is not None:
                        if menu_salvataggio_aperto:
                            # lo slot si sceglie subito, la password viene chiesta dopo
                            slot_da_salvare = indice_slot
                            menu_salvataggio_aperto = False
                            chiedendo_password = True
                            input_password = ""
                        else:
                            if os.path.exists(FILE_MAPPA_SLOT[indice_slot]):
                                x_mappa, y_mappa = dimensioni_mappa(FILE_MAPPA_SLOT[indice_slot])
                                if (x_mappa, y_mappa) in DIMENSIONI_GRIGLIA_NOTE:
                                    # e' la griglia ad adeguarsi alla mappa scelta, non il contrario:
                                    # caricarla sulla griglia sbagliata sposterebbe i muri
                                    modalita_povo_richiesta = (x_mappa, y_mappa) == (X_TOT_POVO, Y_TOT_POVO)
                                    mappa_da_caricare = FILE_MAPPA_SLOT[indice_slot]
                                else:
                                    messaggio_mappa_testo = (
                                        f"Mappa {x_mappa}x{y_mappa}: nessuna griglia di questa misura", ROSSO)
                                    messaggio_mappa_timer = 180
                                menu_caricamento_aperto = False
                            else:
                                slot_vuoto_timer = 90  # slot senza mappa salvata: lampeggia un avviso, il menu resta aperto
                    elif event.key == pygame.K_ESCAPE:
                        menu_salvataggio_aperto = False
                        menu_caricamento_aperto = False
                    continue

                if modificando_persone:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_numero_persone.isdigit():
                            numero_persone = max(0, min(NUMERO_PERSONE_MAX, int(input_numero_persone)))
                            sincronizza_persone(persone, numero_persone, griglia)
                        modificando_persone = False
                    elif event.key == pygame.K_ESCAPE:
                        modificando_persone = False
                    elif event.key == pygame.K_BACKSPACE:
                        input_numero_persone = input_numero_persone[:-1]
                    continue

                if modificando_corridori:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_numero_corridori.isdigit():
                            numero_corridori = max(0, min(NUMERO_CORRIDORI_MAX, int(input_numero_corridori)))
                            sincronizza_persone(corridori, numero_corridori, griglia, fabbrica=crea_corridore)
                        modificando_corridori = False
                    elif event.key == pygame.K_ESCAPE:
                        modificando_corridori = False
                    elif event.key == pygame.K_BACKSPACE:
                        input_numero_corridori = input_numero_corridori[:-1]
                    continue

                if modificando_persone_ferme:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_numero_persone_ferme.isdigit():
                            numero_persone_ferme = max(0, min(NUMERO_PERSONE_FERME_MAX, int(input_numero_persone_ferme)))
                            sincronizza_persone_ferme(persone_ferme, numero_persone_ferme, griglia)
                        modificando_persone_ferme = False
                    elif event.key == pygame.K_ESCAPE:
                        modificando_persone_ferme = False
                    elif event.key == pygame.K_BACKSPACE:
                        input_numero_persone_ferme = input_numero_persone_ferme[:-1]
                    continue

                if modificando_gruppi:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_numero_gruppi.isdigit():
                            numero_gruppi = max(0, min(NUMERO_GRUPPI_MAX, int(input_numero_gruppi)))
                            sincronizza_gruppi(gruppi, numero_gruppi, griglia)
                        modificando_gruppi = False
                    elif event.key == pygame.K_ESCAPE:
                        modificando_gruppi = False
                    elif event.key == pygame.K_BACKSPACE:
                        input_numero_gruppi = input_numero_gruppi[:-1]
                    continue

                # W: inverte la singola cella e avvia il trascinamento
                if event.key == pygame.K_w:
                    mpos = pygame.mouse.get_pos()
                    mx, my = t_m(mpos[0], mpos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    if 0 < r < Y_TOT - 1 and 0 < c < X_TOT - 1:
                        griglia[r][c].tipo = "muro" if griglia[r][c].tipo == "libero" else "libero"
                        muri_modificati = True
                        frame_ultima_modifica_muro = contatore_frame
                        ultima_cella_w = (r, c)

                if event.key == pygame.K_r:
                    for riga in griglia:
                        for n in riga: n.tipo = "libero"
                    crea_bordi(griglia)
                    muri_modificati = True
                    frame_ultima_modifica_muro = contatore_frame
                    robot_x, robot_y = 1.5 * DIM_NODO, 1.5 * DIM_NODO
                    target_pos, percorso = None, []
                if event.key == pygame.K_s: target_pos, percorso = None, []

                if event.key == pygame.K_f:
                    seguendo_robot = not seguendo_robot

                if event.key == pygame.K_m:
                    modalita_rettangolo = True
                    primo_punto_rettangolo = None
                if event.key == pygame.K_ESCAPE and modalita_rettangolo:
                    modalita_rettangolo = False
                    primo_punto_rettangolo = None

                if event.key == pygame.K_F5:
                    menu_salvataggio_aperto = True
                if event.key == pygame.K_F9:
                    menu_caricamento_aperto = True

                # P: alterna fra griglia standard e griglia della planimetria di Povo. La commutazione
                # vera avviene fuori dalla gestione eventi, nel blocco unico dopo il ciclo
                if event.key == pygame.K_p:
                    modalita_povo_richiesta = not modalita_povo_richiesta

                # allineamento fine della planimetria: frecce spostano, PagSu/PagGiu scalano in modo uniforme
                if modalita_povo and planimetria_originale is not None:
                    if event.key == pygame.K_LEFT:
                        offset_planimetria_x -= PASSO_OFFSET_PLANIMETRIA_PX
                    elif event.key == pygame.K_RIGHT:
                        offset_planimetria_x += PASSO_OFFSET_PLANIMETRIA_PX
                    elif event.key == pygame.K_UP:
                        offset_planimetria_y -= PASSO_OFFSET_PLANIMETRIA_PX
                    elif event.key == pygame.K_DOWN:
                        offset_planimetria_y += PASSO_OFFSET_PLANIMETRIA_PX
                    elif event.key == pygame.K_PAGEUP:
                        scala_planimetria_x = min(SCALA_PLANIMETRIA_MAX, scala_planimetria_x + PASSO_SCALA_PLANIMETRIA)
                        scala_planimetria_y = min(SCALA_PLANIMETRIA_MAX, scala_planimetria_y + PASSO_SCALA_PLANIMETRIA)
                    elif event.key == pygame.K_PAGEDOWN:
                        scala_planimetria_x = max(SCALA_PLANIMETRIA_MIN, scala_planimetria_x - PASSO_SCALA_PLANIMETRIA)
                        scala_planimetria_y = max(SCALA_PLANIMETRIA_MIN, scala_planimetria_y - PASSO_SCALA_PLANIMETRIA)

                if event.key == pygame.K_t:
                    path_salvata = salva_mappa_training(griglia)
                    messaggio_mappa_training_testo = f"Mappa training salvata: {os.path.basename(path_salvata)}"
                    messaggio_mappa_training_timer = 90

                if event.key == pygame.K_TAB:
                    pannello_aperto = not pannello_aperto

                if event.key == pygame.K_F11:
                    fullscreen = not fullscreen
                    if fullscreen:
                        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                    else:
                        screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
                elif event.key == pygame.K_ESCAPE and fullscreen:
                    fullscreen = False
                    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)

            if event.type == pygame.KEYUP and event.key == pygame.K_w:
                ultima_cella_w = None

        # commutazione della griglia, in un punto solo: la chiedono sia il tasto P sia la scelta di una
        # mappa nata sull'altra griglia. Cambiando le dimensioni globali va rifatto tutto cio' che le ha
        # gia' incorporate, a partire dai worker del pool, che ne tengono una copia propria
        if modalita_povo_richiesta != modalita_povo:
            modalita_povo = modalita_povo_richiesta
            if modalita_povo:
                applica_dimensioni_griglia(X_TOT_POVO, Y_TOT_POVO)
                planimetria_originale = carica_planimetria()
            else:
                applica_dimensioni_griglia(X_TOT_STANDARD, Y_TOT_STANDARD)
                planimetria_originale = planimetria_scalata = None
            planimetria_chiave_scalata = None

            griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
            crea_bordi(griglia)
            griglia_robot_da_ricostruire = True
            pool_persone.terminate()
            pool_persone = _crea_pool_persone(griglia)
            richiesta_percorsi_pendente = None
            persone_richiesta_percorsi_pendente = []
            muri_modificati = False
            robot_x, robot_y = 1.5 * DIM_NODO, 1.5 * DIM_NODO
            rigenera_popolazione = True
            messaggio_mappa_testo = (
                f"Modalita' Povo: griglia {X_TOT}x{Y_TOT}" if modalita_povo else f"Griglia standard {X_TOT}x{Y_TOT}", VERDE)
            messaggio_mappa_timer = 120

        # caricamento differito: arriva qui dopo l'eventuale cambio di griglia, cosi' i muri finiscono
        # sempre sulla griglia della misura per cui la mappa era stata salvata
        if mappa_da_caricare is not None:
            if carica_mappa(griglia, mappa_da_caricare):
                muri_modificati = True
                frame_ultima_modifica_muro = contatore_frame
                rigenera_popolazione = True
                messaggio_mappa_testo = (f"Mappa caricata: {os.path.basename(mappa_da_caricare)} ({X_TOT}x{Y_TOT})", VERDE)
                messaggio_mappa_timer = 120
            mappa_da_caricare = None

        # ripopolamento, comune al cambio di griglia e al caricamento di una mappa: le posizioni vecchie
        # non hanno piu' senso, e le stime del lidar si riferiscono alla mappa precedente
        if rigenera_popolazione:
            rigenera_popolazione = False
            target_pos, percorso = None, []
            frame_inizio_target = None
            persone = [crea_persona(griglia) for _ in range(numero_persone)]
            corridori = [crea_corridore(griglia) for _ in range(numero_corridori)]
            persone_ferme = [crea_persona_ferma(griglia) for _ in range(numero_persone_ferme)]
            gruppi = [crea_gruppo(griglia) for _ in range(numero_gruppi)]
            celle_rilevate_precedenti = set()
            tracciamento_lidar = {}
            storico_posizioni_lidar = {}
            mappa_pesi_render = {}
            mappa_costo_probabilita = {}
            modalita_rettangolo = False
            primo_punto_rettangolo = None
            ultima_cella_w = None
            # la mappa nuova puo' avere un muro dove stava il robot: lo ricolloca
            r_robot, c_robot = int(robot_y // DIM_NODO), int(robot_x // DIM_NODO)
            if not (0 <= r_robot < Y_TOT and 0 <= c_robot < X_TOT) or griglia[r_robot][c_robot].tipo == "muro":
                nodo_robot = cella_libera_casuale(griglia)
                robot_x, robot_y = nodo_robot.cx, nodo_robot.cy

        zoom, offset_x, offset_y = limita_zoom_pan(zoom, offset_x, offset_y, screen.get_width(), screen.get_height())

        # la griglia del robot si ricostruisce solo al cambio di toggle o a fine modifica dei muri. Il
        # percorso in corso va buttato: i suoi nodi appartengono alla griglia vecchia
        fattore_robot_voluto = FATTORE_GRIGLIA_ROBOT if griglia_fine_attiva else 1
        if fattore_robot_voluto != fattore_robot or griglia_robot_da_ricostruire:
            fattore_robot = fattore_robot_voluto
            dim_robot = DIM_NODO // fattore_robot
            griglia_robot = crea_griglia_robot(griglia, fattore_robot)
            griglia_robot_da_ricostruire = False
            percorso = []

        # 3. Movimento
        tempo_di_ricalcolare_robot = contatore_frame % INTERVALLO_RICALCOLO_ROBOT_FRAME == 0
        # i toggle sicurezza/lidar/ellissoidi del pannello nascondono solo il disegno: arresto,
        # rilevamento e costo di probabilita' restano sempre attivi

        membri_gruppi_mobili = [m for g in gruppi if g["mobile"] for m in g["membri"]]
        membri_gruppi_fermi = [m for g in gruppi if not g["mobile"] for m in g["membri"]]
        tutte_mobili = persone + corridori + membri_gruppi_mobili  # persone, corridori e membri dei gruppi si muovono ed evitano gli altri tutti allo stesso modo
        tutte_le_persone = tutte_mobili + persone_ferme + membri_gruppi_fermi

        # l'arresto vale solo per i corpi mobili: e' un meccanismo di emergenza e presuppone che la
        # situazione si sciolga da sola. Su una persona ferma non si scioglierebbe mai, quindi quelle
        # si tengono a distanza a monte, nel pianificatore (vedi COSTO_ALONE_PERSONE_FERME)
        robot_bloccato = raggio_sicurezza > 0 and any(
            math.hypot(robot_x - p["x"], robot_y - p["y"]) < raggio_sicurezza for p in tutte_mobili
        )

        celle_persone_ferme = {(pf["r"], pf["c"]) for pf in persone_ferme}
        celle_persone_ferme |= {(m["r"], m["c"]) for m in membri_gruppi_fermi}

        # rilevamento lidar: i muri il robot li conosce a priori, le persone solo entro raggio_lidar.
        # Per decidere se ricalcolare si usano le celle vere e non quelle rumorose, altrimenti il rumore
        # ricampionato ogni frame forzerebbe un ricalcolo continuo anche per una persona immobile
        celle_rilevate = set()
        celle_rilevate_rumorose = set()
        id_rilevati_ora = set()
        ferme_rilevate = []  # posizioni percepite delle sole persone ferme: attorno a queste si costruisce l'alone
        for p in tutte_le_persone:
            d = math.hypot(p["x"] - robot_x, p["y"] - robot_y)
            if raggio_lidar > 0 and d < raggio_lidar and linea_di_vista_libera(griglia, robot_x, robot_y, p["x"], p["y"]):
                r_vera, c_vera = int(p["y"] // DIM_NODO), int(p["x"] // DIM_NODO)
                celle_rilevate.add((r_vera, c_vera))
                angolo_rumore = random.uniform(0, 2 * math.pi)
                raggio_rumore = random.uniform(0, RUMORE_LIDAR_PX)
                xr, yr = p["x"] + raggio_rumore * math.cos(angolo_rumore), p["y"] + raggio_rumore * math.sin(angolo_rumore)
                # celle bloccate nella griglia del robot, piu' fine: e' qui che nascono i varchi fra due
                # persone vicine. celle_rilevate sopra resta sulla griglia grossa, serve solo al confronto
                r_rum = max(0, min(Y_TOT * fattore_robot - 1, int(yr // dim_robot)))
                c_rum = max(0, min(X_TOT * fattore_robot - 1, int(xr // dim_robot)))
                celle_rilevate_rumorose.add((r_rum, c_rum))
                # solo persone ferme e membri statici sono privi della chiave "stato": sono gli immobili
                # per costruzione, gli unici che hanno bisogno dell'alone
                if "stato" not in p:
                    ferme_rilevate.append((xr, yr))

                # traccia Kalman solo per le persone attualmente rilevate: su tutta la popolazione il
                # costo esploderebbe
                pid = id(p)
                id_rilevati_ora.add(pid)
                if pid not in tracciamento_lidar:
                    tracciamento_lidar[pid] = nuova_traccia_kalman(xr, yr)
                    storico_posizioni_lidar[pid] = collections.deque(maxlen=FINESTRA_STORICO_FRAME)
                else:
                    aggiorna_traccia_kalman(tracciamento_lidar[pid], xr, yr)
                # lo stato "ferma" viene letto dalla simulazione, non stimato: col rumore del lidar la
                # velocita' residua di un target immobile si sovrappone a quella di un pedone lentissimo,
                # quindi nessuna soglia sulla sola velocita' stimata separerebbe bene i due casi
                tracciamento_lidar[pid]["ferma"] = p.get("stato", "attesa") == "attesa"
                traccia_pid = tracciamento_lidar[pid]
                storico_posizioni_lidar[pid].append((traccia_pid["x"], traccia_pid["y"], traccia_pid["vx"], traccia_pid["vy"]))

        # le tracce perse (fuori raggio o dietro un muro) si abbandonano subito, senza estrapolazione
        for pid_vecchio in list(tracciamento_lidar.keys()):
            if pid_vecchio not in id_rilevati_ora:
                del tracciamento_lidar[pid_vecchio]
                storico_posizioni_lidar.pop(pid_vecchio, None)

        # alone attorno alle persone ferme rilevate: il raggio e' il piu' largo fra zona di sicurezza e
        # contatto fra i corpi, piu' un franco fisso
        raggio_alone_ferme = max(raggio_sicurezza, raggio_robot + raggio_persona) + MARGINE_ALONE_PERSONE_FERME_PX
        mappa_costo_alone_ferme = {}
        if ferme_rilevate:
            costo_alone = COSTO_ALONE_PERSONE_FERME * dim_robot / DIM_NODO
            portata_celle = int(raggio_alone_ferme // dim_robot) + 1
            righe_robot, colonne_robot = Y_TOT * fattore_robot, X_TOT * fattore_robot
            for xf, yf in ferme_rilevate:
                r0, c0 = int(yf // dim_robot), int(xf // dim_robot)
                for dr in range(-portata_celle, portata_celle + 1):
                    for dc in range(-portata_celle, portata_celle + 1):
                        r, c = r0 + dr, c0 + dc
                        if not (0 <= r < righe_robot and 0 <= c < colonne_robot):
                            continue
                        # si confronta il centro della cella: e' quello che il percorso attraversa
                        if math.hypot((c + 0.5) * dim_robot - xf, (r + 0.5) * dim_robot - yf) <= raggio_alone_ferme:
                            mappa_costo_alone_ferme[(r, c)] = costo_alone

        # il percorso da proteggere con l'isteresi va catturato prima dell'azzeramento qui sotto,
        # altrimenti in scena affollata l'isteresi proteggerebbe quasi sempre una lista vuota
        percorso_precedente_per_isteresi = percorso

        if celle_rilevate != celle_rilevate_precedenti:
            percorso = []  # e' cambiato l'insieme di persone rilevate: il percorso pianificato potrebbe non essere piu' valido
        celle_rilevate_precedenti = celle_rilevate

        # macchia di probabilita': ogni traccia viene proiettata in avanti e il suo peso diffuso nello
        # spazio libero, dando un costo morbido invece del blocco secco della posizione rilevata. Ha una
        # cadenza propria, indipendente dal ricalcolo del percorso, altrimenti l'ellisse si muove a scatti
        if contatore_frame % INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME == 0:
            # slider "definizione ellissoidi": sottocelle per lato che compongono la macchia
            sottocelle_per_lato = max(SOTTOCELLE_PER_LATO_MIN, round(definizione_ellissoidi))
            # la macchia deve essere un multiplo esatto della griglia del robot, cosi' l'aggregazione
            # qui sotto e' una divisione intera senza resti
            sottocelle_per_lato = fattore_robot * max(1, round(sottocelle_per_lato / fattore_robot))
            dim_sottocella = DIM_NODO / sottocelle_per_lato
            y_tot_fine, x_tot_fine = Y_TOT * sottocelle_per_lato, X_TOT * sottocelle_per_lato
            raggio_blob_sottocelle = RAGGIO_BLOB_CELLE * sottocelle_per_lato
            decadimento_blob_sottocella = DECADIMENTO_BLOB ** (1 / sottocelle_per_lato)

            # un solo forward pass con tutte le tracce impilate: su input cosi' piccoli il costo fisso
            # per chiamata di torch domina, quindi un batch unico e' molto piu' leggero di un ciclo
            correzioni_ai = {}
            if ai_attiva and modello_ai_disponibile:
                pid_da_correggere = []
                storici_da_correggere = []
                for pid, traccia in tracciamento_lidar.items():
                    if traccia.get("ferma", False):
                        continue
                    storico_pid = storico_posizioni_lidar.get(pid)
                    if storico_pid is not None and len(storico_pid) == FINESTRA_STORICO_FRAME:
                        pid_da_correggere.append(pid)
                        storici_da_correggere.append(storico_pid)
                if pid_da_correggere:
                    storico_batch = prepara_input(np.array(storici_da_correggere, dtype=np.float32))
                    with torch.no_grad():
                        correzioni = modello_correzione_ai(torch.from_numpy(storico_batch)).numpy()
                    correzioni_ai = dict(zip(pid_da_correggere, correzioni))

            mappa_pesi_render = {}  # chiavi in sottocelle (griglia fine), solo per la macchia/il disegno
            for pid, traccia in tracciamento_lidar.items():
                # persona ferma: niente proiezione ne' orientamento, altrimenti il rumore residuo
                # verrebbe amplificato dall'orizzonte e la macchia si muoverebbe a persona immobile
                e_ferma = traccia.get("ferma", False)
                frame_futuri_effettivi = 0 if e_ferma else PREVISIONE_BLOB_FRAME
                direzione_traccia = None if e_ferma else (traccia["vx"], traccia["vy"])
                x_prev, y_prev = previsione_posizione_kalman(traccia, frame_futuri_effettivi)
                correzione = correzioni_ai.get(pid)
                if correzione is not None:
                    x_prev += float(correzione[0])
                    y_prev += float(correzione[1])
                rf_prev = max(0, min(y_tot_fine - 1, int(y_prev // dim_sottocella)))
                cf_prev = max(0, min(x_tot_fine - 1, int(x_prev // dim_sottocella)))
                # unione probabilistica fra ellissoidi di persone diverse: dove si sovrappongono il peso
                # cresce ma satura verso 1.0, invece di sommarsi linearmente
                for sottocella, peso in macchia_diffusione(griglia, rf_prev, cf_prev, raggio_blob_sottocelle, decadimento_blob_sottocella, sottocelle_per_lato, direzione=direzione_traccia).items():
                    peso_precedente = mappa_pesi_render.get(sottocella, 0.0)
                    mappa_pesi_render[sottocella] = 1.0 - (1.0 - peso_precedente) * (1.0 - peso)
            # per l'A* le sottocelle si aggregano nella cella della griglia del robot che le contiene,
            # prendendo il peso massimo. Il costo si riscala con il lato della cella per la stessa
            # ragione di COSTO_CELLA_OCCUPATA: la penalita' totale non deve dipendere dalla risoluzione
            rapporto_collasso = max(1, sottocelle_per_lato // fattore_robot)
            costo_massimo_probabilita = COSTO_MASSIMO_PROBABILITA * dim_robot / DIM_NODO
            mappa_costo_probabilita = {}
            for (rf, cf), peso in mappa_pesi_render.items():
                cella = (rf // rapporto_collasso, cf // rapporto_collasso)
                costo = peso * costo_massimo_probabilita
                if cella not in mappa_costo_probabilita or mappa_costo_probabilita[cella] < costo:
                    mappa_costo_probabilita[cella] = costo
            mappa_pesi_render_dim_sottocella = dim_sottocella  # ricordata per il disegno, che avviene in un punto diverso del frame

        if target_pos and not robot_bloccato:
            # il click da' una cella della griglia grossa: se ne prende la sottocella centrale
            target_robot = (target_pos[0] * fattore_robot + fattore_robot // 2,
                            target_pos[1] * fattore_robot + fattore_robot // 2)
            if not percorso or tempo_di_ricalcolare_robot:
                # isteresi: sconto sulla prossima porzione del percorso in corso. Lo sconto cammina
                # lungo la geometria dei segmenti invece di contare i nodi, perche' con Theta* un solo
                # segmento puo' coprire molte celle. Budget in pixel, sconto per cella riscalato
                mappa_costo_extra_robot = dict(mappa_costo_probabilita)
                budget_isteresi_px = CELLE_ISTERESI_PERCORSO * DIM_NODO
                costo_isteresi = COSTO_ISTERESI_PERCORSO * dim_robot / DIM_NODO
                distanza_isteresi_percorsa = 0.0
                for i in range(len(percorso_precedente_per_isteresi) - 1):
                    if distanza_isteresi_percorsa >= budget_isteresi_px:
                        break
                    a, b = percorso_precedente_per_isteresi[i], percorso_precedente_per_isteresi[i + 1]
                    lunghezza_segmento = math.hypot(b.cx - a.cx, b.cy - a.cy)
                    passi_segmento = max(1, int(lunghezza_segmento / dim_robot))
                    for passo_i in range(passi_segmento + 1):
                        if distanza_isteresi_percorsa >= budget_isteresi_px:
                            break
                        t = passo_i / passi_segmento
                        x = a.cx + (b.cx - a.cx) * t
                        y = a.cy + (b.cy - a.cy) * t
                        cella = (int(y // dim_robot), int(x // dim_robot))
                        mappa_costo_extra_robot[cella] = mappa_costo_extra_robot.get(cella, 0.0) - costo_isteresi
                        distanza_isteresi_percorsa += lunghezza_segmento / passi_segmento

                # l'alone si applica dopo l'isteresi e vince su di essa: un percorso che passa addosso
                # a una persona ferma va cambiato, non protetto
                for cella_alone, costo_alone_cella in mappa_costo_alone_ferme.items():
                    mappa_costo_extra_robot[cella_alone] = max(mappa_costo_extra_robot.get(cella_alone, 0.0), costo_alone_cella)

                percorso = algoritmo_a_star(griglia_robot, (robot_x, robot_y), target_robot, celle_bloccate=celle_rilevate_rumorose, mappa_costo_extra=mappa_costo_extra_robot)
            meta_nodo = griglia_robot[target_robot[0]][target_robot[1]]
            velocita_nominale_robot = VELOCITA_ROBOT * moltiplicatore_velocita
            fattore_velocita_rischio = fattore_velocita_da_conflitto(
                robot_x, robot_y, percorso, tracciamento_lidar, velocita_nominale_robot, raggio_robot, raggio_persona
            ) if controllo_velocita_attivo else 1.0
            robot_x, robot_y, arrivato, passo_muro = passo_movimento(robot_x, robot_y, percorso, velocita_nominale_robot * fattore_velocita_rischio, meta_nodo, griglia_robot)
            if passo_muro:
                percorso = []  # il passo tagliava un muro: ricalcola subito
            if arrivato:
                if frame_inizio_target is not None:
                    # tempo equivalente a velocita' 1x: i frame grezzi sottostimano di un fattore pari
                    # al moltiplicatore di simulazione
                    tempo_impiegato_s = (contatore_frame - frame_inizio_target) / 60 * moltiplicatore_velocita
                    messaggio_arrivo_testo = f"Obiettivo raggiunto in {tempo_impiegato_s:.1f}s"
                    messaggio_arrivo_timer = 180  # 3 secondi a 60 FPS
                    frame_inizio_target = None
                target_pos = None

        # 3b. Movimento persone, corridori e membri dei gruppi (stessa logica per tutti, cambia solo la velocita')
        range_evitamento_quad = RANGE_EVITAMENTO_PERSONE ** 2
        range_evitamento_robot = raggio_robot + raggio_persona  # raggio robot + raggio persona (entrambi regolabili): i corpi non si sovrappongono mai
        # bucket spaziali per trovare i vicini di ciascuna entita' senza confrontarla con tutte le altre
        # (O(n) invece di O(n^2)): fondamentale con centinaia di persone in scena
        griglia_spaziale = costruisci_griglia_spaziale(tutte_mobili, DIM_NODO)
        # celle occupate da qualunque entita', calcolate una volta per frame e condivise da tutti gli A*:
        # ricalcolarle per ciascuna persona scalava male col numero di persone
        celle_bloccate_comune = {
            (int(p2["y"] // DIM_NODO), int(p2["x"] // DIM_NODO)) for p2 in tutte_mobili
        }
        celle_bloccate_comune |= celle_persone_ferme
        celle_bloccate_comune.add((int(robot_y // DIM_NODO), int(robot_x // DIM_NODO)))

        # il pool si ricrea con la griglia aggiornata solo dopo qualche frame dall'ultima modifica dei
        # muri, per non rifarlo decine di volte al secondo mentre si trascina il tasto W
        if muri_modificati and contatore_frame - frame_ultima_modifica_muro > 15:
            pool_persone.terminate()
            pool_persone = _crea_pool_persone(griglia)
            griglia_robot_da_ricostruire = True  # anche la griglia fine del robot copia i muri, va rifatta
            muri_modificati = False
            richiesta_percorsi_pendente = None  # risultati del pool appena terminato: ormai invalidi

        # ricalcolo percorsi, separato dal movimento cosi' il batch del frame va in blocco al pool.
        # Il primo calcolo di ciascuna persona viene spalmato sui primi frame tramite offset_ricalcolo:
        # e' l'unico caso in cui centinaia di persone ne avrebbero bisogno nello stesso istante
        persone_da_ricalcolare = []
        batch_richieste = []
        for p in tutte_mobili:
            if p["stato"] != "movimento":
                continue
            tempo_di_ricalcolare_questa_persona = (contatore_frame + p["offset_ricalcolo"]) % INTERVALLO_RICALCOLO_PERSONE_FRAME == 0
            if p["percorso"] is None and not tempo_di_ricalcolare_questa_persona:
                continue  # in attesa del proprio turno per il primo calcolo
            if p["percorso"] is not None and p["percorso"] and not tempo_di_ricalcolare_questa_persona:
                continue  # percorso ancora valido, non e' il suo turno per il ricalcolo periodico
            persone_da_ricalcolare.append(p)
            batch_richieste.append(((p["x"], p["y"]), p["target"], p["rumore_seed"], celle_bloccate_comune))

        # mentre si stanno disegnando i muri il ricalcolo si salta del tutto invece di ricadere sul
        # calcolo sequenziale: chi lo attendeva lo ottiene al turno successivo, restando intanto
        # sull'ultimo percorso valido

        # raccolta asincrona del batch precedente: su mappe intricate un batch puo' impiegare piu' di un
        # frame, e con una map() bloccante il loop si fermerebbe fino al risultato piu' lento
        if richiesta_percorsi_pendente is not None and richiesta_percorsi_pendente.ready():
            for p, percorso_calcolato in zip(persone_richiesta_percorsi_pendente, richiesta_percorsi_pendente.get()):
                p["percorso"] = percorso_calcolato
            richiesta_percorsi_pendente = None

        # un nuovo batch parte solo se il precedente e' stato raccolto, per non accumularne di concorrenti
        if batch_richieste and not muri_modificati and richiesta_percorsi_pendente is None:
            richiesta_percorsi_pendente = pool_persone.map_async(_pool_calcola_percorso, batch_richieste)
            persone_richiesta_percorsi_pendente = persone_da_ricalcolare

        for p in tutte_mobili:
            if p["stato"] == "movimento":
                if p["percorso"] is None:
                    continue  # in attesa del proprio turno per il primo calcolo
                nodo_target_p = griglia[p["target"][0]][p["target"][1]]
                velocita_p = VELOCITA_PERSONA * moltiplicatore_velocita * p["fattore_velocita"]
                non_capofila = "gruppo" in p and not p.get("capofila")
                if non_capofila:
                    # nessuna attrazione verso il capofila, che incastrava i membri contro i muri: solo
                    # piu' velocita' quando resta indietro, camminando comunque sul proprio percorso
                    leader = p["gruppo"]["membri"][0]
                    distanza_leader = math.hypot(p["x"] - leader["x"], p["y"] - leader["y"])
                    if distanza_leader > GRUPPO_DISTANZA_COESIONE_PX:
                        velocita_p *= min(GRUPPO_FATTORE_RINCORSA_MAX, 1.0 + (distanza_leader - GRUPPO_DISTANZA_COESIONE_PX) / GRUPPO_DISTANZA_COESIONE_PX)
                p["velocita_attuale"] = velocita_p
                x_prima, y_prima = p["x"], p["y"]
                p["x"], p["y"], arrivata, passo_muro = passo_movimento(p["x"], p["y"], p["percorso"], velocita_p, nodo_target_p, griglia)
                if passo_muro:
                    p["percorso"] = []  # il passo tagliava un muro: ricalcola subito invece di aspettare il prossimo intervallo

                # scarto proporzionale per non compenetrare le persone vicine
                rep_x, rep_y = repulsione_vicini(p, vicini_spaziali(p, griglia_spaziale, DIM_NODO), range_evitamento_quad)
                p["x"], p["y"] = sposta_con_vettore(p["x"], p["y"], rep_x, rep_y, FORZA_REPULSIONE_PERSONE, griglia)

                # le persone possono entrare nella zona di sicurezza del robot ma non nel suo corpo
                rrep_x, rrep_y = repulsione_punto(p, robot_x, robot_y, range_evitamento_robot)
                p["x"], p["y"] = sposta_con_vettore(p["x"], p["y"], rrep_x, rrep_y, FORZA_REPULSIONE_ROBOT, griglia)

                if non_capofila:
                    spostamento = math.hypot(p["x"] - x_prima, p["y"] - y_prima)
                    p["frame_fermo"] = 0 if spostamento > 0.3 else p.get("frame_fermo", 0) + 1
                    leader = p["gruppo"]["membri"][0]
                    distanza_leader = math.hypot(p["x"] - leader["x"], p["y"] - leader["y"])
                    # si punta dritto al capofila se il membro e' bloccato da tempo o si e' allontanato
                    # troppo: puo' aver imboccato una rotta opposta, e li' accelerare non servirebbe
                    deve_inseguire_leader = (
                        distanza_leader > GRUPPO_DISTANZA_MAX_DIVERGENZA_PX
                        or (p["frame_fermo"] > GRUPPO_FRAME_BLOCCO_SOGLIA and distanza_leader > GRUPPO_DISTANZA_COESIONE_PX)
                    )
                    if deve_inseguire_leader and not p.get("inseguendo_leader"):
                        p["percorso"] = []  # appena entrato in modalita' inseguimento: ricalcola subito verso il leader, non tra 10 frame
                    if deve_inseguire_leader or p.get("inseguendo_leader"):
                        if distanza_leader <= GRUPPO_DISTANZA_COESIONE_PX:
                            # riagganciato: torna a puntare all'obiettivo condiviso del gruppo
                            p["inseguendo_leader"] = False
                            p["target"] = p["gruppo"]["target_comune"]
                            p["percorso"] = []
                        else:
                            # la meta segue il capofila frame per frame: congelandola al punto iniziale
                            # si arriverebbe a un punto ormai vecchio, fermandosi mentre lui si allontana
                            p["inseguendo_leader"] = True
                            p["frame_fermo"] = 0
                            p["target"] = (max(0, min(Y_TOT - 1, int(leader["y"] // DIM_NODO))), max(0, min(X_TOT - 1, int(leader["x"] // DIM_NODO))))

                # inseguendo il capofila, arrivare al suo punto attuale non e' un vero arrivo
                if arrivata and not (non_capofila and p.get("inseguendo_leader")):
                    p["stato"] = "attesa"
                    p["attesa_timer"] = random.randint(ATTESA_PERSONA_MIN_FRAME, ATTESA_PERSONA_MAX_FRAME)
            elif p["stato"] == "attesa":
                # anche da ferme mantengono una distanza minima fra loro, invece di ammassarsi
                rep_x, rep_y = repulsione_vicini(p, vicini_spaziali(p, griglia_spaziale, DIM_NODO), range_evitamento_quad)
                p["x"], p["y"] = sposta_con_vettore(p["x"], p["y"], rep_x, rep_y, FORZA_REPULSIONE_PERSONE, griglia)

                rrep_x, rrep_y = repulsione_punto(p, robot_x, robot_y, range_evitamento_robot)
                p["x"], p["y"] = sposta_con_vettore(p["x"], p["y"], rrep_x, rrep_y, FORZA_REPULSIONE_ROBOT, griglia)

                p["attesa_timer"] -= 1
                if p["attesa_timer"] <= 0:
                    if "gruppo" in p and not p.get("capofila"):
                        p["attesa_timer"] = random.randint(ATTESA_PERSONA_MIN_FRAME, ATTESA_PERSONA_MAX_FRAME)  # aspetta che il capofila decida la prossima destinazione del gruppo
                        continue
                    if random.random() < PROBABILITA_PERCORSO_BREVE:
                        nuovo_nodo_target = cella_libera_vicina(griglia, p["x"], p["y"], RAGGIO_PERCORSO_BREVE)
                    else:
                        nuovo_nodo_target = cella_libera_casuale(griglia)
                    nuovo_target = (nuovo_nodo_target.r, nuovo_nodo_target.c)
                    p["target"] = nuovo_target
                    p["percorso"] = []  # forza ricalcolo immediato verso il nuovo target
                    p["stato"] = "movimento"
                    if p.get("capofila"):
                        # il capofila reindirizza tutti gli altri membri verso la nuova destinazione
                        p["gruppo"]["target_comune"] = nuovo_target
                        for compagno in p["gruppo"]["membri"]:
                            if compagno is not p:
                                compagno["target"] = nuovo_target
                                compagno["percorso"] = []
                                compagno["stato"] = "movimento"
                                compagno["inseguendo_leader"] = False

        contatore_frame += 1

        if seguendo_robot:
            offset_x = screen.get_width() / 2 - robot_x * zoom
            offset_y = screen.get_height() / 2 - robot_y * zoom
            zoom, offset_x, offset_y = limita_zoom_pan(zoom, offset_x, offset_y, screen.get_width(), screen.get_height())

        # 4. rendering
        # raggi usati solo per il disegno: 0 se il toggle e' spento, mentre il meccanismo vero sopra ha
        # gia' usato i raggi reali
        raggio_sicurezza_effettivo = raggio_sicurezza if sicurezza_attiva else 0
        raggio_lidar_effettivo = raggio_lidar if lidar_attivo else 0

        # celle che il pianificatore considera occupate, disegnate alla risoluzione della griglia del
        # robot: e' la resa visiva del guadagno sui varchi
        if lidar_attivo and celle_rilevate_rumorose:
            # una sola Surface riusata per tutte le celle: allocarne una per cella ad ogni frame costava
            sx0, sy0 = t_s(0, 0)
            sx1, sy1 = t_s(dim_robot, dim_robot)
            blocco_surf = pygame.Surface((max(1, sx1 - sx0), max(1, sy1 - sy0)), pygame.SRCALPHA)
            blocco_surf.fill((*ARANCIONE, 70))
            for (r_b, c_b) in celle_rilevate_rumorose:
                screen.blit(blocco_surf, t_s(c_b * dim_robot, r_b * dim_robot))

        if percorso and len(percorso) > 1:
            punti = [t_s(n.cx, n.cy) for n in percorso]
            pygame.draw.lines(screen, ROSSO, False, punti, 2)

        # macchie di probabilita', disegnate sulla sotto-griglia fine e solo dove il peso e' stato
        # calcolato: piu' opache al centro
        if ellissoidi_attivi:
            for (rf_b, cf_b), peso in mappa_pesi_render.items():
                sx, sy = t_s(cf_b * mappa_pesi_render_dim_sottocella, rf_b * mappa_pesi_render_dim_sottocella)
                ex, ey = t_s((cf_b + 1) * mappa_pesi_render_dim_sottocella, (rf_b + 1) * mappa_pesi_render_dim_sottocella)
                larghezza, altezza = max(1, ex - sx), max(1, ey - sy)
                blob_surf = pygame.Surface((larghezza, altezza), pygame.SRCALPHA)
                alpha = int(min(255, peso * ALPHA_MAX_BLOB))
                blob_surf.fill((*COLORE_BLOB_PROBABILITA, alpha))
                screen.blit(blob_surf, (sx, sy))

        rx, ry = t_s(robot_x, robot_y)

        if raggio_lidar_effettivo > 0:
            raggio_lidar_px = max(1, int(raggio_lidar_effettivo * zoom))
            lidar_surf = pygame.Surface((raggio_lidar_px * 2, raggio_lidar_px * 2), pygame.SRCALPHA)
            pygame.draw.circle(lidar_surf, (*COLORE_LIDAR, ALPHA_ZONA_LIDAR), (raggio_lidar_px, raggio_lidar_px), raggio_lidar_px)
            screen.blit(lidar_surf, (rx - raggio_lidar_px, ry - raggio_lidar_px))

        if raggio_sicurezza_effettivo > 0:
            raggio_px = max(1, int(raggio_sicurezza_effettivo * zoom))
            zona_surf = pygame.Surface((raggio_px * 2, raggio_px * 2), pygame.SRCALPHA)
            pygame.draw.circle(zona_surf, (255, 0, 0, ALPHA_ZONA_SICUREZZA), (raggio_px, raggio_px), raggio_px)
            screen.blit(zona_surf, (rx - raggio_px, ry - raggio_px))

        pygame.draw.circle(screen, BLU, (rx, ry), int(raggio_robot * zoom))
        if target_pos:
            tx, ty = t_s(target_pos[1]*DIM_NODO + DIM_NODO//2, target_pos[0]*DIM_NODO + DIM_NODO//2)
            pygame.draw.circle(screen, ROSSO, (tx, ty), int((DIM_NODO//4) * zoom), 2)

        if modalita_rettangolo and primo_punto_rettangolo:
            r1, c1 = primo_punto_rettangolo
            mx, my = pygame.mouse.get_pos()
            mgx, mgy = t_m(mx, my)
            r2 = max(1, min(Y_TOT - 2, int(mgy // DIM_NODO)))
            c2 = max(1, min(X_TOT - 2, int(mgx // DIM_NODO)))
            r_min, r_max = min(r1, r2), max(r1, r2)
            c_min, c_max = min(c1, c2), max(c1, c2)
            sx, sy = t_s(c_min * DIM_NODO, r_min * DIM_NODO)
            ex, ey = t_s((c_max + 1) * DIM_NODO, (r_max + 1) * DIM_NODO)
            pygame.draw.rect(screen, ROSSO, (sx, sy, ex - sx, ey - sy), 2)

        for p in tutte_mobili:
            px, py = t_s(p["x"], p["y"])
            colore_p = colore_rilevamento(p["x"], p["y"], robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia)
            pygame.draw.circle(screen, colore_p, (px, py), int(raggio_persona * zoom))

        for pf in persone_ferme:
            fx, fy = t_s(pf["x"], pf["y"])
            colore_pf = colore_rilevamento(pf["x"], pf["y"], robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia)
            pygame.draw.circle(screen, colore_pf, (fx, fy), int(raggio_persona * zoom))

        for g in gruppi:
            if g["mobile"]:
                continue  # i membri dei gruppi mobili sono gia' disegnati sopra (sono inclusi in tutte_mobili)
            for m in g["membri"]:
                mpx, mpy = t_s(m["x"], m["y"])
                colore_m = colore_rilevamento(m["x"], m["y"], robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia)
                pygame.draw.circle(screen, colore_m, (mpx, mpy), int(raggio_persona * zoom))

        if modalita_rettangolo:
            msg = "Clicca il secondo punto (Esc annulla)" if primo_punto_rettangolo else "Clicca il primo punto (Esc annulla)"
            aiuto_txt = font.render(msg, True, ROSSO)
            screen.blit(aiuto_txt, (10, 10))

        if messaggio_mappa_training_timer > 0:
            messaggio_mappa_training_timer -= 1
            msg_txt = font.render(messaggio_mappa_training_testo, True, VERDE)
            screen.blit(msg_txt, (10, 40))

        if messaggio_mappa_timer > 0:
            messaggio_mappa_timer -= 1
            testo_mappa, colore_mappa = messaggio_mappa_testo
            screen.blit(font.render(testo_mappa, True, colore_mappa), (10, 70))

        if messaggio_arrivo_timer > 0:
            messaggio_arrivo_timer -= 1
            arrivo_txt = font_grande.render(messaggio_arrivo_testo, True, VERDE)
            screen.blit(arrivo_txt, arrivo_txt.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2)))

        # 5. richiesta password per il salvataggio
        if errore_timer > 0:
            errore_timer -= 1
        if chiedendo_password or errore_timer > 0:
            box_w, box_h = 360, 110
            box_x, box_y = (screen.get_width() - box_w) // 2, (screen.get_height() - box_h) // 2
            pygame.draw.rect(screen, BIANCO, (box_x, box_y, box_w, box_h))
            pygame.draw.rect(screen, NERO, (box_x, box_y, box_w, box_h), 2)

            titolo = font.render("Password per salvare (Invio/Esc)", True, NERO)
            screen.blit(titolo, (box_x + 15, box_y + 15))

            mascherata = "*" * len(input_password)
            campo = font.render(mascherata if mascherata else " ", True, NERO)
            pygame.draw.rect(screen, GRIGIO, (box_x + 15, box_y + 50, box_w - 30, 30), 1)
            screen.blit(campo, (box_x + 20, box_y + 55))

            if errore_timer > 0:
                errore_txt = font.render("Password errata", True, ROSSO)
                screen.blit(errore_txt, (box_x + 15, box_y + 85))

        # 5b. menu di scelta dello slot mappa
        if slot_vuoto_timer > 0:
            slot_vuoto_timer -= 1
        if menu_salvataggio_aperto or menu_caricamento_aperto:
            # tutti gli slot, con la misura della griglia a cui appartengono: caricandone uno dell'altra
            # modalita' e' la griglia ad adeguarsi, quindi non c'e' motivo di nasconderli
            slot_visibili = [(i, str(i + 1)) for i in range(SLOT_MAPPA_POVO)] + [(SLOT_MAPPA_POVO, "P")]
            box_slot_w, box_slot_h = 430, 90 + 40 * len(slot_visibili)
            box_slot_x = (screen.get_width() - box_slot_w) // 2
            box_slot_y = (screen.get_height() - box_slot_h) // 2
            pygame.draw.rect(screen, BIANCO, (box_slot_x, box_slot_y, box_slot_w, box_slot_h))
            pygame.draw.rect(screen, NERO, (box_slot_x, box_slot_y, box_slot_w, box_slot_h), 2)

            titolo_slot = font.render("Esc annulla", True, NERO)
            screen.blit(titolo_slot, (box_slot_x + 15, box_slot_y + 15))

            for riga, (indice_slot_mostrato, tasto_slot) in enumerate(slot_visibili):
                path = FILE_MAPPA_SLOT[indice_slot_mostrato]
                if os.path.exists(path):
                    x_mappa, y_mappa = dimensioni_mappa(path)
                    stato = f"{x_mappa}x{y_mappa}"
                    # la griglia corrente si distingue in nero, l'altra in grigio: caricandola si cambia
                    colore_slot = NERO if (x_mappa, y_mappa) == (X_TOT, Y_TOT) else GRIGIO_SCURO
                else:
                    stato, colore_slot = "vuoto", GRIGIO_SCURO
                riga_slot = font.render(f"{tasto_slot}. {os.path.basename(path)} - {stato}", True, colore_slot)
                screen.blit(riga_slot, (box_slot_x + 15, box_slot_y + 55 + riga * 40))

            if menu_caricamento_aperto and slot_vuoto_timer > 0:
                avviso_slot = font.render("Slot vuoto: nessuna mappa da caricare", True, ROSSO)
                screen.blit(avviso_slot, (box_slot_x + 15, box_slot_y + box_slot_h - 30))

        # 6. pannello tecnico
        if pannello_aperto:
            pygame.draw.rect(screen, BIANCO, panel_rect)
            pygame.draw.rect(screen, NERO, panel_rect, 2)

            # indicatore di scorrimento sul bordo destro, solo se il pannello eccede la finestra: sta in
            # coordinate schermo e non del pannello, perche' rappresenta la parte visibile
            if scroll_pannello_max > 0:
                barra_x = panel_rect.x + panel_w - 8
                barra_y, barra_h = 20, screen.get_height() - 40
                pygame.draw.rect(screen, GRIGIO, (barra_x, barra_y, 4, barra_h))
                cursore_h = max(20, int(barra_h * barra_h / panel_h))
                cursore_y = barra_y + int((barra_h - cursore_h) * scroll_pannello / scroll_pannello_max)
                pygame.draw.rect(screen, GRIGIO_SCURO, (barra_x, cursore_y, 4, cursore_h))

            titolo = font.render("Pannello tecnico (TAB)", True, NERO)
            screen.blit(titolo, (panel_rect.x + 15, panel_rect.y + 10))

            vel_txt = font.render(f"Velocita simulazione: {moltiplicatore_velocita:.2f}x", True, NERO)
            screen.blit(vel_txt, (panel_rect.x + 15, panel_rect.y + 45))

            pygame.draw.rect(screen, GRIGIO, slider_rect)
            rel = (moltiplicatore_velocita - VEL_MULT_MIN) / (VEL_MULT_MAX - VEL_MULT_MIN)
            handle_x = slider_rect.x + int(rel * slider_rect.width)
            pygame.draw.circle(screen, BLU, (handle_x, slider_rect.centery), 9)

            pers_txt = font.render("Persone:", True, NERO)
            screen.blit(pers_txt, (panel_rect.x + 15, panel_rect.y + 105))

            pygame.draw.rect(screen, BIANCO, numero_box_rect)
            pygame.draw.rect(screen, BLU if modificando_persone else GRIGIO, numero_box_rect, 2)
            valore_mostrato = input_numero_persone if modificando_persone else str(numero_persone)
            numero_txt = font.render(valore_mostrato, True, NERO)
            screen.blit(numero_txt, (numero_box_rect.x + 8, numero_box_rect.y + 4))

            corridori_txt = font.render("Corridori:", True, NERO)
            screen.blit(corridori_txt, (panel_rect.x + 15, panel_rect.y + 140))

            pygame.draw.rect(screen, BIANCO, numero_corridori_box_rect)
            pygame.draw.rect(screen, BLU if modificando_corridori else GRIGIO, numero_corridori_box_rect, 2)
            valore_corridori_mostrato = input_numero_corridori if modificando_corridori else str(numero_corridori)
            numero_corridori_txt = font.render(valore_corridori_mostrato, True, NERO)
            screen.blit(numero_corridori_txt, (numero_corridori_box_rect.x + 8, numero_corridori_box_rect.y + 4))

            gruppi_txt = font.render("Gruppi:", True, NERO)
            screen.blit(gruppi_txt, (panel_rect.x + 15, panel_rect.y + 175))

            pygame.draw.rect(screen, BIANCO, numero_gruppi_box_rect)
            pygame.draw.rect(screen, BLU if modificando_gruppi else GRIGIO, numero_gruppi_box_rect, 2)
            valore_gruppi_mostrato = input_numero_gruppi if modificando_gruppi else str(numero_gruppi)
            numero_gruppi_txt = font.render(valore_gruppi_mostrato, True, NERO)
            screen.blit(numero_gruppi_txt, (numero_gruppi_box_rect.x + 8, numero_gruppi_box_rect.y + 4))

            ferme_txt = font.render("Persone ferme:", True, NERO)
            screen.blit(ferme_txt, (panel_rect.x + 15, panel_rect.y + 210))

            pygame.draw.rect(screen, BIANCO, numero_ferme_box_rect)
            pygame.draw.rect(screen, BLU if modificando_persone_ferme else GRIGIO, numero_ferme_box_rect, 2)
            valore_ferme_mostrato = input_numero_persone_ferme if modificando_persone_ferme else str(numero_persone_ferme)
            numero_ferme_txt = font.render(valore_ferme_mostrato, True, NERO)
            screen.blit(numero_ferme_txt, (numero_ferme_box_rect.x + 8, numero_ferme_box_rect.y + 4))

            sicurezza_txt = font.render(f"Zona sicurezza robot: {int(raggio_sicurezza)}px", True, NERO)
            screen.blit(sicurezza_txt, (panel_rect.x + 15, panel_rect.y + 245))
            pygame.draw.rect(screen, ROSSO if sicurezza_attiva else BIANCO, checkbox_sicurezza_rect)
            pygame.draw.rect(screen, NERO, checkbox_sicurezza_rect, 2)

            pygame.draw.rect(screen, GRIGIO, slider_sicurezza_rect)
            rel_sic = (raggio_sicurezza - RAGGIO_SICUREZZA_MIN) / (RAGGIO_SICUREZZA_MAX - RAGGIO_SICUREZZA_MIN)
            handle_sic_x = slider_sicurezza_rect.x + int(rel_sic * slider_sicurezza_rect.width)
            pygame.draw.circle(screen, ROSSO, (handle_sic_x, slider_sicurezza_rect.centery), 9)

            robot_txt = font.render(f"Dimensione robot: {int(raggio_robot)}px", True, NERO)
            screen.blit(robot_txt, (panel_rect.x + 15, panel_rect.y + 320))

            pygame.draw.rect(screen, GRIGIO, slider_robot_rect)
            rel_robot = (raggio_robot - RAGGIO_ROBOT_MIN) / (RAGGIO_ROBOT_MAX - RAGGIO_ROBOT_MIN)
            handle_robot_x = slider_robot_rect.x + int(rel_robot * slider_robot_rect.width)
            pygame.draw.circle(screen, BLU, (handle_robot_x, slider_robot_rect.centery), 9)

            lidar_txt = font.render(f"Raggio lidar: {int(raggio_lidar)}px", True, NERO)
            screen.blit(lidar_txt, (panel_rect.x + 15, panel_rect.y + 395))
            pygame.draw.rect(screen, COLORE_LIDAR if lidar_attivo else BIANCO, checkbox_lidar_rect)
            pygame.draw.rect(screen, NERO, checkbox_lidar_rect, 2)

            pygame.draw.rect(screen, GRIGIO, slider_lidar_rect)
            rel_lidar = (raggio_lidar - RAGGIO_LIDAR_MIN) / (RAGGIO_LIDAR_MAX - RAGGIO_LIDAR_MIN)
            handle_lidar_x = slider_lidar_rect.x + int(rel_lidar * slider_lidar_rect.width)
            pygame.draw.circle(screen, COLORE_LIDAR, (handle_lidar_x, slider_lidar_rect.centery), 9)

            persona_txt = font.render(f"Dimensione persone: {int(raggio_persona)}px", True, NERO)
            screen.blit(persona_txt, (panel_rect.x + 15, panel_rect.y + 470))

            pygame.draw.rect(screen, GRIGIO, slider_persona_rect)
            rel_persona = (raggio_persona - RAGGIO_PERSONA_MIN) / (RAGGIO_PERSONA_MAX - RAGGIO_PERSONA_MIN)
            handle_persona_x = slider_persona_rect.x + int(rel_persona * slider_persona_rect.width)
            pygame.draw.circle(screen, ARANCIONE, (handle_persona_x, slider_persona_rect.centery), 9)

            sottocelle_per_lato_mostrato = max(SOTTOCELLE_PER_LATO_MIN, round(definizione_ellissoidi))
            definizione_txt = font.render(f"Definizione ellissoidi: {sottocelle_per_lato_mostrato}x{sottocelle_per_lato_mostrato}", True, NERO)
            screen.blit(definizione_txt, (panel_rect.x + 15, panel_rect.y + 545))
            pygame.draw.rect(screen, COLORE_BLOB_PROBABILITA if ellissoidi_attivi else BIANCO, checkbox_ellissoidi_rect)
            pygame.draw.rect(screen, NERO, checkbox_ellissoidi_rect, 2)

            pygame.draw.rect(screen, GRIGIO, slider_definizione_rect)
            rel_definizione = (definizione_ellissoidi - SOTTOCELLE_PER_LATO_MIN) / (SOTTOCELLE_PER_LATO_MAX - SOTTOCELLE_PER_LATO_MIN)
            handle_definizione_x = slider_definizione_rect.x + int(rel_definizione * slider_definizione_rect.width)
            pygame.draw.circle(screen, COLORE_BLOB_PROBABILITA, (handle_definizione_x, slider_definizione_rect.centery), 9)

            ai_label = "AI di predizione" if modello_ai_disponibile else "AI di predizione - modello non trovato"
            ai_txt = font.render(ai_label, True, NERO if modello_ai_disponibile else GRIGIO)
            screen.blit(ai_txt, (panel_rect.x + 15, panel_rect.y + 610))
            pygame.draw.rect(screen, VERDE if (ai_attiva and modello_ai_disponibile) else BIANCO, checkbox_ai_rect)
            pygame.draw.rect(screen, NERO, checkbox_ai_rect, 2)

            velocita_txt = font.render("Velocita' predittiva (incroci)", True, NERO)
            screen.blit(velocita_txt, (panel_rect.x + 15, panel_rect.y + 647))
            pygame.draw.rect(screen, VERDE if controllo_velocita_attivo else BIANCO, checkbox_velocita_rect)
            pygame.draw.rect(screen, NERO, checkbox_velocita_rect, 2)

            griglia_fine_txt = font.render(
                f"Griglia fine robot ({dim_robot}px)" if griglia_fine_attiva else f"Griglia fine robot ({DIM_NODO}px)",
                True, NERO)
            screen.blit(griglia_fine_txt, (panel_rect.x + 15, panel_rect.y + 684))
            pygame.draw.rect(screen, VERDE if griglia_fine_attiva else BIANCO, checkbox_griglia_fine_rect)
            pygame.draw.rect(screen, NERO, checkbox_griglia_fine_rect, 2)

            # planimetria: trasparenza e allineamento, attivi solo in modalita' Povo (tasto P)
            planimetria_caricata = modalita_povo and planimetria_originale is not None
            planimetria_label = f"Planimetria: {int(alpha_planimetria)}" if planimetria_caricata else "Planimetria (P per la mappa Povo)"
            planimetria_txt = font.render(planimetria_label, True, NERO if planimetria_caricata else GRIGIO_SCURO)
            screen.blit(planimetria_txt, (panel_rect.x + 15, panel_rect.y + 721))
            pygame.draw.rect(screen, ARANCIONE if (planimetria_visibile and planimetria_caricata) else BIANCO, checkbox_planimetria_rect)
            pygame.draw.rect(screen, NERO, checkbox_planimetria_rect, 2)

            pygame.draw.rect(screen, GRIGIO, slider_planimetria_rect)
            rel_planimetria = (alpha_planimetria - ALPHA_PLANIMETRIA_MIN) / (ALPHA_PLANIMETRIA_MAX - ALPHA_PLANIMETRIA_MIN)
            handle_planimetria_x = slider_planimetria_rect.x + int(rel_planimetria * slider_planimetria_rect.width)
            pygame.draw.circle(screen, ARANCIONE, (handle_planimetria_x, slider_planimetria_rect.centery), 9)

            allineamento_txt = font_piccolo.render(
                f"scala {scala_planimetria_x:.2f}x{scala_planimetria_y:.2f}  offset {int(offset_planimetria_x)},{int(offset_planimetria_y)} px"
                "  (frecce / PagSu-PagGiu)", True, GRIGIO_SCURO)
            screen.blit(allineamento_txt, (panel_rect.x + 15, panel_rect.y + 770))

            # pulsanti che salvano, ricaricano o azzerano su disco la configurazione del pannello
            pygame.draw.rect(screen, VERDE, button_salva_config_rect)
            pygame.draw.rect(screen, NERO, button_salva_config_rect, 2)
            salva_txt = font.render("Salva config.", True, BIANCO)
            screen.blit(salva_txt, salva_txt.get_rect(center=button_salva_config_rect.center))

            pygame.draw.rect(screen, BLU, button_carica_config_rect)
            pygame.draw.rect(screen, NERO, button_carica_config_rect, 2)
            carica_txt = font.render("Carica config.", True, BIANCO)
            screen.blit(carica_txt, carica_txt.get_rect(center=button_carica_config_rect.center))

            pygame.draw.rect(screen, ROSSO, button_reset_config_rect)
            pygame.draw.rect(screen, NERO, button_reset_config_rect, 2)
            reset_txt = font.render("Pulisci config.", True, BIANCO)
            screen.blit(reset_txt, reset_txt.get_rect(center=button_reset_config_rect.center))

            if configurazione_salvataggio_timer > 0:
                configurazione_salvataggio_timer -= 1
                testo_messaggio, colore_messaggio = configurazione_messaggio
                conferma_txt = font_piccolo.render(testo_messaggio, True, colore_messaggio)
                screen.blit(conferma_txt, (panel_rect.x + 15, panel_rect.y + 692))

        totale_persone_simulazione = len(persone) + len(corridori) + len(persone_ferme) + sum(len(g["membri"]) for g in gruppi)
        contatore_persone_txt = font.render(f"Persone in simulazione: {totale_persone_simulazione}", True, NERO)
        screen.blit(contatore_persone_txt, contatore_persone_txt.get_rect(bottomright=(screen.get_width() - 10, screen.get_height() - 10)))

        pygame.display.flip()
        clock.tick(60)

    pool_persone.terminate()
    pool_persone.join()
    pygame.quit()

if __name__ == "__main__":
    main()
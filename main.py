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
X_TOT, Y_TOT = 80, 60
DIM_NODO = 30
LARGHEZZA, ALTEZZA = X_TOT * DIM_NODO, Y_TOT * DIM_NODO
VELOCITA_ROBOT = 1.5
VELOCITA_PERSONA = 1.0
# controllo di velocita' (Livello 3, versione euristica): il robot rallenta in proporzione al rischio
# (macchia di probabilita') incontrato guardando avanti sul proprio percorso gia' pianificato, invece di
# muoversi sempre alla stessa velocita' e lasciare che sia solo il percorso a cambiare - frenata anticipata
# e morbida invece di svolte secche. Riusa il costo di probabilita' gia' calcolato per l'A*, nessun dato
# o modello nuovo necessario
FATTORE_VELOCITA_ROBOT_MIN = 0.35  # velocita' minima (frazione di VELOCITA_ROBOT) quando il conflitto previsto e' imminente e nemmeno sprintare lo evita
FATTORE_VELOCITA_ROBOT_MAX = 1.25  # margine di sprint (sopra la velocita' nominale) per superare un incrocio prima che la persona ci arrivi, invece di doverlo aspettare fermo
DISTANZA_LOOKAHEAD_VELOCITA_PX = DIM_NODO * 5  # quanto avanti sul percorso si guarda per decidere quanto rallentare
MARGINE_SICUREZZA_CONFLITTO_PX = 6  # oltre la somma dei raggi robot+persona, margine extra di cautela nel controllo predittivo
FRAME_REAZIONE_VELOCITA_ROBOT = 90  # orizzonte (in frame) entro cui un incrocio previsto inizia a far rallentare: piu' e' vicino nel tempo, piu' forte il rallentamento
ATTESA_PERSONA_MIN_FRAME = 1 * 60  # attesa minima dopo l'arrivo, in frame (60 FPS)
ATTESA_PERSONA_MAX_FRAME = 4 * 60  # attesa massima dopo l'arrivo, in frame
PROBABILITA_PERCORSO_BREVE = 0.35  # probabilita' che la prossima destinazione sia vicina invece che casuale ovunque
RAGGIO_PERCORSO_BREVE = DIM_NODO * 6  # distanza massima (in pixel) per un "percorso breve"
RANGE_EVITAMENTO_PERSONE = 15  # px: sotto questa distanza da un'altra persona scatta una spinta di repulsione
FORZA_REPULSIONE_PERSONE = 1.3  # px/frame massimi di spinta continua fra persone/corridori/leader (a distanza 0)
COSTO_CELLA_OCCUPATA = DIM_NODO * 10  # penalita' di costo A* per una cella occupata da un'altra entita': forte ma non un divieto assoluto (evita percorsi vuoti nei passaggi a 1 cella)
INTERVALLO_RICALCOLO_ROBOT_FRAME = 60    # ricalcolo percorso robot: 60 FPS / questo valore = volte al secondo
INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME = 6  # ricalcolo macchia di probabilita': 60 FPS / questo valore = volte al secondo (10) - separato dal ricalcolo del percorso, cosi' l'ellisse resta aggiornata anche quando il percorso attuale e' ancora valido
INTERVALLO_RICALCOLO_PERSONE_FRAME = 10  # ricalcolo percorso persone: 60 FPS / questo valore = volte al secondo
CARTELLA_MAPPE_SALVATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappe_salvate")
CARTELLA_MAPPE_TRAINING = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappe_training")
CARTELLA_MAPPE_TESTING = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappe_testing")
FILE_MAPPA_SLOT = [
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_salvata.json"),
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_salvata_2.json"),
    os.path.join(CARTELLA_MAPPE_SALVATE, "mappa_salvata_3.json"),
]
PASSWORD_SALVATAGGIO = "1258"
FILE_CONFIGURAZIONE_PANNELLO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_pannello.json")
VEL_MULT_MIN, VEL_MULT_MAX = 0.2, 25.0
NUMERO_PERSONE_DEFAULT = 0
NUMERO_PERSONE_MAX = 200
NUMERO_PERSONE_FERME_DEFAULT = 0
NUMERO_PERSONE_FERME_MAX = 200
NUMERO_CORRIDORI_DEFAULT = 0
NUMERO_CORRIDORI_MAX = 200
FATTORE_VELOCITA_CORRIDORE = 2.0  # i corridori vanno sempre al doppio della velocita' base
NUMERO_GRUPPI_DEFAULT = 0
NUMERO_GRUPPI_MAX = 200
GRUPPO_MEMBRI_MIN, GRUPPO_MEMBRI_MAX = 2, 7
GRUPPO_MEMBRI_MEDIA = 3.5  # numero di componenti a gaussiana: piu' probabile intorno a 3-4, raro vicino a 2 o 7
GRUPPO_MEMBRI_DEV_STD = 1.1
GRUPPO_RAGGIO_FORMAZIONE_PX = int(DIM_NODO * 1.0)  # distanza di base dei membri dal leader/ancora
GRUPPO_RUMORE_RAGGIO = 0.3  # +-30% di variazione casuale sul raggio di ciascun membro: forma meno geometrica/perfetta
GRUPPO_RUMORE_ANGOLO = 0.4  # rad di variazione casuale sull'angolo di ciascun membro rispetto alla spaziatura regolare
PROBABILITA_GRUPPO_MOBILE = 0.8  # l'80% dei gruppi si muove, il 20% resta fermo
GRUPPO_DISTANZA_COESIONE_PX = int(DIM_NODO * 1.5)  # sotto questa distanza dal leader non scatta la rincorsa: e' gia' abbastanza vicino
GRUPPO_FATTORE_RINCORSA_MAX = 2.5  # quanto puo' accelerare al massimo un membro lontano dal leader per riprenderlo
GRUPPO_FRAME_BLOCCO_SOGLIA = 90  # frame (~1.5s) fermo prima di rinunciare all'obiettivo del gruppo e puntare dritto al leader
GRUPPO_DISTANZA_MAX_DIVERGENZA_PX = int(DIM_NODO * 5)  # oltre questa distanza dal leader (anche se in movimento, non bloccato) punta dritto a lui: il proprio A* potrebbe aver imboccato una rotta completamente diversa dalla sua
ZOOM_MIN, ZOOM_MAX = 0.25, 4.0
MARGINE_PAN = 100  # spazio bianco massimo attorno alla mappa, in pixel
RAGGIO_SICUREZZA_MIN, RAGGIO_SICUREZZA_MAX = 0, DIM_NODO * 6
RAGGIO_SICUREZZA_DEFAULT = 12
ALPHA_ZONA_SICUREZZA = 70  # trasparenza del cerchio (0-255)
RAGGIO_ROBOT_MIN, RAGGIO_ROBOT_MAX = 5, int(DIM_NODO * 1.5)
RAGGIO_ROBOT_DEFAULT = 7  # raggio "fisico" del robot (lo stesso con cui viene disegnato di default)
RAGGIO_PERSONA_MIN, RAGGIO_PERSONA_MAX = 3, int(DIM_NODO * 0.9)
RAGGIO_PERSONA_DEFAULT = 8  # raggio con cui vengono disegnate le persone
FORZA_REPULSIONE_ROBOT = 1.3  # px/frame massimi di spinta continua lontano dal robot (a distanza 0)
RAGGIO_LIDAR_MIN, RAGGIO_LIDAR_MAX = 0, DIM_NODO * 10
RAGGIO_LIDAR_DEFAULT = DIM_NODO * 4  # raggio di rilevamento del "lidar" simulato del robot
RUMORE_LIDAR_PX = 8  # px massimi di rumore casuale sulla posizione percepita di una persona rilevata
ALPHA_ZONA_LIDAR = 35  # trasparenza del cerchio del lidar (0-255), piu' tenue della zona di sicurezza
KALMAN_RUMORE_MISURA = (RUMORE_LIDAR_PX / 2) ** 2  # varianza di misura: quanto ci si fida di una singola posizione percepita
KALMAN_RUMORE_PROCESSO = 0.05  # varianza aggiunta alla velocita' stimata ad ogni frame (quanto puo' cambiare "di suo")
KALMAN_INCERTEZZA_INIZIALE_POS = 50.0 ** 2  # varianza di posizione iniziale per una traccia appena avvistata
KALMAN_INCERTEZZA_INIZIALE_VEL = 5.0 ** 2  # varianza di velocita' iniziale per una traccia appena avvistata
PREVISIONE_BLOB_FRAME = 60  # quanti frame nel futuro si proietta il centro della macchia (60 = 1s)
RAGGIO_BLOB_CELLE = 3.0  # raggio massimo (in celle della griglia di movimento) della diffusione della macchia di probabilita': isotropa (persona ferma) = ~7x7 celle, allungata (in moto) = fino a ~13 celle lungo la marcia e ~5 di lato
DECADIMENTO_BLOB = 0.75  # fattore di decadimento del peso per ogni passo-cella di diffusione attraverso lo spazio libero
# quando piu' ellissoidi si sovrappongono, il peso si combina come un'unione di probabilita'
# indipendenti (1 - prodotto delle probabilita' di NON esserci) invece di sommarsi linearmente: cresce
# in modo molto piu' morbido (i contributi successivi contano sempre meno) e non serve un tetto
# artificiale, perche' il risultato satura naturalmente a 1.0 - lo stesso costo massimo di una singola
# macchia piena, mai di piu'. Una somma lineare pura faceva crescere il costo troppo in fretta con poche
# persone vicine, spingendo il robot a fare giri lunghissimi anche quando conveniva passarci in mezzo.
# quanto la macchia si allunga lungo la direzione di marcia stimata (velocita' del Kalman): un passo di
# diffusione allineato con l'heading costa meno (la macchia si allunga avanti/indietro), uno
# perpendicolare costa piu' del normale (si restringe ai lati) - il risultato e' un ellissoide orientato
# vero (piu' incertezza lungo la marcia, meno di lato) invece della macchia isotropa (stesso raggio in
# ogni direzione). 0 = nessun allungamento (comportamento isotropo di prima); il minimo del fattore resta
# ben sopra zero apposta, altrimenti un passo quasi gratuito nella direzione dell'heading farebbe
# esplodere la macchia.
ALLUNGAMENTO_BLOB = 0.55
# soglia di sicurezza puramente numerica dentro macchia_diffusione (evita di normalizzare un vettore
# heading vicino a zero): NON e' piu' il modo con cui si decide se una persona e' "ferma" (con
# RUMORE_LIDAR_PX=8 la velocita' residua di un target davvero fermo si sovrappone troppo a quella di un
# pedone lentissimo per separarli con una soglia sulla sola velocita' stimata - vedi il flag "ferma"
# sulle tracce Kalman, che usa invece lo stato reale della simulazione).
VELOCITA_MINIMA_AFFIDABILE = 0.15
COSTO_MASSIMO_PROBABILITA = COSTO_CELLA_OCCUPATA * 0.2  # costo extra A* al centro della macchia (peso 1.0): piu' morbido del blocco per rilevamento diretto. A 0.5 il robot preferiva un giro enorme pur di evitare una porta coperta da piu' persone sovrapposte, invece di passarci con cautela (misurato: a 150 fa un giro di 2106px invece di 1077px per una deviazione cauta) - 0.2 resta sotto la soglia empirica (~100-105) a cui scatta il giro assurdo
COLORE_BLOB_PROBABILITA = (40, 130, 255)
# isteresi sul percorso del robot: senza uno sconto sulla strada gia' scelta, quando due percorsi hanno
# costo quasi identico basta una piccola fluttuazione di rumore (macchia di probabilita' ricalcolata,
# rilevamento con posizione rumorosa) a far scambiare quale dei due sembra il migliore ad ogni ricalcolo,
# facendo continuare il robot a cambiare idea invece di avanzare. Lo sconto si applica solo alle prossime
# celle del percorso attuale (non a tutto il tragitto): cosi' non si "blocca" per sempre su una scelta
# vecchia in un percorso lungo, resta comunque libero di cambiare rotta se emerge un'alternativa
# chiaramente migliore piu' avanti. Il valore resta sotto la distanza geometrica minima fra due celle
# adiacenti (DIM_NODO) cosi' il costo di un passo non puo' mai diventare negativo.
COSTO_ISTERESI_PERCORSO = DIM_NODO * 0.6
CELLE_ISTERESI_PERCORSO = 10
ALPHA_MAX_BLOB = 140  # trasparenza massima (al centro della macchia) del rendering termico
# la macchia viene calcolata e disegnata su una sotto-griglia piu' fine (ogni cella di movimento
# diventa "definizione"^2 sottocelle) SOLO per lei: l'A* e il movimento restano sulla griglia
# originale, invariati. Regolabile dal pannello tecnico: al minimo (1) e' un quadrato identico alla
# griglia di movimento (comportamento originale), al massimo le sottocelle sono cosi' piccole che la
# macchia appare pressoche' un ellissoide continuo, indistinguibile a occhio dai quadretti.
SOTTOCELLE_PER_LATO_MIN, SOTTOCELLE_PER_LATO_MAX = 1, 10
SOTTOCELLE_PER_LATO_DEFAULT = 1
FATTORE_VELOCITA_MEDIA = 1.0    # media della gaussiana (1.0 = velocita' base)
FATTORE_VELOCITA_DEV_STD = 0.25  # deviazione standard: la maggior parte cammina, code = passeggiano/corrono
FATTORE_VELOCITA_MIN, FATTORE_VELOCITA_MAX = 0.4, 2.5  # limiti per evitare fermi o assurdamente veloci
RUMORE_PERCORSO_PERSONA = 15  # quanto i percorsi delle persone si discostano dall'ottimo matematico (0 = disattivato)

# Colori
BIANCO, GRIGIO = (255, 255, 255), (210, 210, 210)
ROSSO, BLU = (255, 0, 0), (0, 100, 255)
ARANCIONE = (240, 140, 0)
VERDE = (30, 160, 60)  # persone non rilevate dal lidar del robot (fuori dal suo raggio d'azione)
NERO = (30, 30, 30)
GRIGIO_SCURO = (90, 90, 90)
GRIGIO_PAVIMENTO = (195, 195, 195)  # sfondo della mappa
GRIGIO_BORDO_CELLA = (182, 182, 182)  # bordo delle celle libere, un pochino piu' scuro dello sfondo
COLORE_LIDAR = (235, 200, 0)

class Nodo:
    def __init__(self, r, c):
        self.r, self.c = r, c
        self.x, self.y = c * DIM_NODO, r * DIM_NODO
        self.cx = self.x + DIM_NODO // 2
        self.cy = self.y + DIM_NODO // 2
        self.g = self.h = self.f = 0
        self.genitore = None
        self.tipo = "libero"

    def reset_calcoli(self):
        self.g = self.h = self.f = 0
        self.genitore = None

def algoritmo_a_star(griglia, inizio_pos_pixel, fine_pos_griglia, rumore_seed=None, celle_bloccate=None, mappa_costo_extra=None):
    c_inizio = int(inizio_pos_pixel[0] // DIM_NODO)
    r_inizio = int(inizio_pos_pixel[1] // DIM_NODO)
    r_fine, c_fine = fine_pos_griglia
    r_inizio = max(0, min(Y_TOT - 1, r_inizio))
    c_inizio = max(0, min(X_TOT - 1, c_inizio))

    nodo_inizio = griglia[r_inizio][c_inizio]
    nodo_fine = griglia[r_fine][c_fine]

    # NB: la griglia non viene azzerata qui all'inizio - ogni chiamata pulisce da sola, alla fine, solo i
    # nodi che ha effettivamente toccato (vedi 'in_open' sotto e i vari return). Resettare tutti i 4800 nodi
    # ad ogni chiamata (come prima) costava tempo fisso indipendente dalla lunghezza del percorso: con
    # centinaia di persone che ricalcolano il percorso piu' volte al secondo era una spesa enorme e quasi
    # tutta inutile per percorsi brevi che toccano solo una piccola porzione della mappa.

    contatore = 0  # tie-breaker per lo heap (i Nodo non sono confrontabili tra loro)
    open_heap = [(0, contatore, nodo_inizio)]
    in_open = {nodo_inizio: 0}  # nodo -> miglior g conosciuto finche' non viene chiuso; e' anche l'insieme
                                 # completo dei nodi toccati da questa ricerca, da ripulire prima di uscire
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
            if 0 <= r < Y_TOT and 0 <= c < X_TOT:
                vicino = griglia[r][c]
                if vicino in closed_list or vicino.tipo == "muro": continue
                if m[0] != 0 and m[1] != 0:
                    # movimento diagonale: vietato se taglia l'angolo tra due muri adiacenti, altrimenti
                    # il grafo lo considera valido ma in coordinate continue e' un passo impossibile da
                    # compiere (passo_movimento lo respinge sempre) - la persona restava bloccata per sempre
                    if griglia[attuale.r][c].tipo == "muro" or griglia[r][attuale.c].tipo == "muro":
                        continue
                dist = math.sqrt((vicino.cx - attuale.cx)**2 + (vicino.cy - attuale.cy)**2)
                if rumore_seed is not None:
                    # rumore deterministico per cella+persona: la stessa persona rifa' sempre
                    # la stessa scelta di "corsia" ricalcolando, invece di zigzagare a caso
                    perturbazione = math.sin(vicino.r * 12.9898 + vicino.c * 78.233 + rumore_seed) * RUMORE_PERCORSO_PERSONA
                    dist = max(1.0, dist + perturbazione)
                if celle_bloccate and (r, c) in celle_bloccate:
                    # penalita' pesante, non un divieto assoluto: se esiste un'alternativa la preferisce,
                    # ma se quella cella occupata e' l'unico passaggio (es. una porta larga 1 cella) la
                    # attraversa comunque invece di restituire un percorso vuoto e restare bloccata per sempre
                    dist += COSTO_CELLA_OCCUPATA
                if mappa_costo_extra:
                    # costo morbido e graduale (es. la macchia di probabilita' del lidar predittivo del
                    # robot): a differenza di celle_bloccate non e' un blocco secco, il peso varia da
                    # cella a cella invece di essere lo stesso ovunque
                    dist += mappa_costo_extra.get((r, c), 0.0)
                nuovo_g = attuale.g + dist
                nuovo_genitore = attuale

                # Theta* (any-angle): se il "nonno" ha una scorciatoia libera verso il vicino (nessun
                # muro, nessuna cella occupata o a costo positivo nel mezzo), collegarsi direttamente a
                # lui invece che passare dal genitore intermedio da' un percorso piu' corto e non
                # vincolato ai soli 8 angoli della griglia - e' quello che rende i tragitti smussati
                # invece che a gradini, riusando la stessa idea del raycasting gia' usato per il lidar
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
    """Cella libera con un blocco 3x3 di celle libere attorno a se' (se stessa inclusa): garantisce che un
    gruppo abbia davvero spazio per generarsi, invece di nascere incastrato contro un muro. Se non ne trova
    (mappa molto densa di muri), ripiega su una cella libera qualsiasi."""
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
    """Cella libera entro un certo raggio da (x, y), per generare percorsi brevi. Se non ne trova, ripiega su una casuale."""
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
        "percorso": None,  # None = "mai calcolato ancora" (distinto da [] = "invalidato, ricalcola subito"): permette di spalmare il primo calcolo su piu' frame, vedi offset_ricalcolo piu' sotto
        "stato": "movimento",
        "attesa_timer": 0,
        "rumore_seed": random.uniform(0, 1000),
        "fattore_velocita": fattore_velocita,
        "offset_ricalcolo": random.randint(0, INTERVALLO_RICALCOLO_PERSONE_FRAME - 1),
        "priorita": random.random(),  # per rompere gli stalli simmetrici: a parita' di distanza, chi ha priorita' minore cede il passo
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
    """Posizioni disposte grossomodo in cerchio attorno al leader/ancora, con un po' di rumore casuale su
    raggio e angolo di ciascun membro: forma piu' organica e meno geometricamente perfetta di un cerchio esatto."""
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
    """Gruppo di 2-7 persone. Se fermo: cluster statico attorno a un'ancora. Se mobile: membri alla pari,
    tutti con lo stesso obiettivo (target_comune) e ognuno col proprio A* indipendente (quindi superano porte
    e corridoi stretti mettendosi in fila esattamente come le persone singole, senza formazione rigida) - chi
    resta indietro accelera in proporzione alla distanza dal leader (membri[0]) invece di essere attratto/
    trascinato verso di lui, e se resta bloccato a lungo ripiega su un percorso diretto verso la sua posizione."""
    numero_membri = round(random.gauss(GRUPPO_MEMBRI_MEDIA, GRUPPO_MEMBRI_DEV_STD))
    numero_membri = max(GRUPPO_MEMBRI_MIN, min(GRUPPO_MEMBRI_MAX, numero_membri))

    if random.random() < PROBABILITA_GRUPPO_MOBILE:
        ancora = cella_libera_3x3_casuale(griglia)
        nodo_target = cella_libera_casuale(griglia)
        target_comune = (nodo_target.r, nodo_target.c)
        # un gruppo cammina tutto alla stessa andatura (estratta una volta sola, come per le persone singole)
        # invece che ogni membro con una velocita' indipendente: altrimenti si sfaldano anche senza congestione
        fattore_velocita_gruppo = max(FATTORE_VELOCITA_MIN, min(FATTORE_VELOCITA_MAX, random.gauss(FATTORE_VELOCITA_MEDIA, FATTORE_VELOCITA_DEV_STD)))
        # stesso motivo per il rumore del percorso: se ogni membro calcola l'A* con un rumore_seed diverso,
        # ognuno preferisce una "corsia" leggermente diversa e puo' finire su un percorso molto piu' lungo
        # degli altri anche in area completamente aperta, restando indietro senza nessun ostacolo reale
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
        membri[0]["capofila"] = True  # decide quando il gruppo si ferma/riparte verso una nuova destinazione condivisa
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
    """Raggruppa le entita' in bucket quadrati di lato dim_bucket, indicizzati per cella: permette di trovare
    i vicini di un'entita' controllando solo poche celle invece di tutte le altre entita' (O(n) invece di
    O(n^2) man mano che il numero di persone cresce)."""
    griglia_spaziale = {}
    for e in entita_list:
        chiave = (int(e["y"] // dim_bucket), int(e["x"] // dim_bucket))
        griglia_spaziale.setdefault(chiave, []).append(e)
    return griglia_spaziale

def vicini_spaziali(entita, griglia_spaziale, dim_bucket):
    """Entita' nel bucket di 'entita' e negli 8 adiacenti: sufficiente per non perdere nessun vicino entro
    RANGE_EVITAMENTO_PERSONE, dato che dim_bucket (DIM_NODO) e' maggiore del raggio di evitamento."""
    r, c = int(entita["y"] // dim_bucket), int(entita["x"] // dim_bucket)
    vicini = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            vicini.extend(griglia_spaziale.get((r + dr, c + dc), ()))
    return vicini

def repulsione_vicini(entita, vicini, range_evitamento_quad):
    """Vettore di spinta continua lontano dai vicini entro il raggio di evitamento: piu' vicino = spinta piu'
    forte, in modo proporzionale (non un blocco secco a scatti). Chi ha priorita' minore viene spinto con piu'
    forza, chi ha priorita' maggiore devia pochissimo: cosi' in un incrocio qualcuno passa sempre per primo,
    con una transizione fluida invece di uno stallo oscillante."""
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
    """Come repulsione_vicini ma da un singolo punto fisso (es. il corpo del robot) invece che da una
    lista di entita': stessa logica di spinta proporzionale alla distanza, senza pesi di priorita'."""
    dx, dy = entita["x"] - px, entita["y"] - py
    d2 = dx * dx + dy * dy
    raggio_quad = raggio * raggio
    if 0 < d2 < raggio_quad:
        d = math.sqrt(d2)
        peso = (raggio - d) / raggio
        return (dx / d) * peso, (dy / d) * peso
    return 0.0, 0.0

def linea_di_vista_libera(griglia, x1, y1, x2, y2):
    """True se il segmento fra i due punti non attraversa nessuna cella muro: simula l'occlusione del
    lidar del robot (non rileva una persona nascosta dietro una parete). Campiona il segmento a passi
    piu' fitti della dimensione di una cella, cosi' non puo' saltare un muro sottile una cella."""
    dist = math.hypot(x2 - x1, y2 - y1)
    if dist == 0:
        return True
    passi = max(1, int(dist / (DIM_NODO / 4)))
    for i in range(passi + 1):
        t = i / passi
        x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
        r, c = int(y // DIM_NODO), int(x // DIM_NODO)
        if 0 <= r < Y_TOT and 0 <= c < X_TOT and griglia[r][c].tipo == "muro":
            return False
    return True

def scorciatoia_libera(griglia, nodo_a, nodo_b, celle_bloccate, mappa_costo_extra):
    """True se il segmento fra due nodi e' davvero libero per una scorciatoia 'any-angle' (Theta*): non
    solo nessun muro (come linea_di_vista_libera), ma anche nessuna cella occupata o con costo extra
    positivo (macchia di probabilita') lungo il tragitto. Una scorciatoia salta i nodi intermedi della
    griglia, quindi deve valere solo attraverso spazio davvero libero: altrimenti bypasserebbe i costi
    morbidi che servono a far scartare al robot le persone rilevate o la loro incertezza prevista."""
    dist = math.hypot(nodo_b.cx - nodo_a.cx, nodo_b.cy - nodo_a.cy)
    if dist == 0:
        return True
    passi = max(1, int(dist / (DIM_NODO / 4)))
    for i in range(passi + 1):
        t = i / passi
        x, y = nodo_a.cx + (nodo_b.cx - nodo_a.cx) * t, nodo_a.cy + (nodo_b.cy - nodo_a.cy) * t
        r, c = int(y // DIM_NODO), int(x // DIM_NODO)
        if not (0 <= r < Y_TOT and 0 <= c < X_TOT) or griglia[r][c].tipo == "muro":
            return False
        if celle_bloccate and (r, c) in celle_bloccate:
            return False
        if mappa_costo_extra and mappa_costo_extra.get((r, c), 0.0) > 0:
            return False
    return True

def colore_rilevamento(px, py, robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia):
    """Colore di una persona in base alla distanza dal robot: verde se fuori dal raggio del lidar o
    nascosta da un muro (non rilevata), arancione se rilevata dal lidar, rosso se dentro la zona di
    sicurezza del robot (la zona di sicurezza e' un limite fisico, non un rilevamento: resta attiva
    anche dietro un muro sottile, per prudenza)."""
    d = math.hypot(px - robot_x, py - robot_y)
    if raggio_sicurezza > 0 and d < raggio_sicurezza:
        return ROSSO
    if raggio_lidar > 0 and d < raggio_lidar and linea_di_vista_libera(griglia, robot_x, robot_y, px, py):
        return ARANCIONE
    return VERDE

def kalman_predict_asse(pos, vel, pxx, pxv, pvv, dt=1.0):
    """Passo di previsione di un filtro di Kalman 1D a velocita' costante (stato: posizione, velocita').
    Usato due volte indipendentemente (asse x e asse y) invece di un vero filtro 2D: gli assi non sono
    correlati in questo modello, quindi la semplificazione non perde precisione ed evita l'algebra
    matriciale. La covarianza cresce ad ogni passo (piu' incertezza piu' si proietta lontano nel futuro)."""
    pos_n = pos + vel * dt
    vel_n = vel
    pxx_n = pxx + 2 * dt * pxv + dt * dt * pvv
    pxv_n = pxv + dt * pvv
    pvv_n = pvv + KALMAN_RUMORE_PROCESSO
    return pos_n, vel_n, pxx_n, pxv_n, pvv_n

def kalman_update_asse(pos, vel, pxx, pxv, pvv, misura):
    """Passo di correzione: integra una nuova posizione misurata (rumorosa) riducendo l'incertezza,
    con un peso (guadagno di Kalman) proporzionale a quanto ci si fida della stima attuale rispetto
    al rumore di misura KALMAN_RUMORE_MISURA."""
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
    """Proietta in avanti (sola previsione, senza nuove misure) la posizione stimata di 'frame_futuri'
    frame: e' il centro su cui viene fatta partire la macchia di probabilita'."""
    return traccia["x"] + traccia["vx"] * frame_futuri, traccia["y"] + traccia["vy"] * frame_futuri

def sottocella_e_muro(griglia, rf, cf, sottocelle_per_lato):
    """True se la sottocella (rf, cf, sulla griglia fine 'sottocelle_per_lato' volte piu' densa) cade
    in una cella muro della griglia di movimento originale, o e' fuori mappa."""
    r_cella, c_cella = int(rf // sottocelle_per_lato), int(cf // sottocelle_per_lato)
    if not (0 <= r_cella < Y_TOT and 0 <= c_cella < X_TOT):
        return True
    return griglia[r_cella][c_cella].tipo == "muro"

_DIREZIONI_DIFFUSIONE = ((0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                          (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))

def macchia_diffusione(griglia, rf_centro, cf_centro, raggio_sottocelle, decadimento, sottocelle_per_lato, direzione=None):
    """Espande un peso (1.0 al centro) attraverso lo spazio libero a partire dalla sottocella centrale
    (Dijkstra a 8 direzioni sulla griglia fine, con le diagonali che costano radice di 2 invece di 1):
    i muri (letti dalla griglia di movimento originale) bloccano il flusso, quindi la macchia si piega
    attorno agli angoli e si stringe nelle porte invece di attraversare i muri in linea retta; il
    decadimento e' funzione della distanza euclidea percorsa (non del numero di passi), quindi il fronte
    d'onda approssima un cerchio/ellisse invece di un rombo. Lavora sulla sotto-griglia
    ('sottocelle_per_lato' volte piu' densa, regolabile dal pannello) solo per la macchia stessa:
    l'A* e il movimento non la vedono.

    'direzione' (vx, vy, es. la velocita' stimata dal Kalman) rende la diffusione anisotropa: i passi
    allineati con la direzione di marcia costano meno (la macchia si allunga in avanti/indietro lungo
    l'heading), quelli perpendicolari costano il normale - il risultato e' un ellissoide orientato vero
    invece della macchia isotropa. None (o velocita' troppo piccola per fidarsi dell'heading) ricade sul
    comportamento isotropo di prima."""
    y_tot_fine, x_tot_fine = Y_TOT * sottocelle_per_lato, X_TOT * sottocelle_per_lato
    if sottocella_e_muro(griglia, rf_centro, cf_centro, sottocelle_per_lato):
        return {}

    hx = hy = 0.0
    if direzione is not None:
        norma_direzione = math.hypot(direzione[0], direzione[1])
        if norma_direzione > VELOCITA_MINIMA_AFFIDABILE:  # velocita' troppo piccola: heading inaffidabile, resta isotropo
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
                # colonna = asse x, riga = asse y: stesso sistema di coordinate dell'heading (vx, vy)
                norma_passo = math.hypot(dc, dr)
                cos_theta = (dc * hx + dr * hy) / norma_passo
                # simmetrico: allineato con l'heading (cos^2=1) costa meno -> si allunga avanti/indietro;
                # perpendicolare (cos^2=0) costa piu' del normale -> si restringe ai lati. Con solo lo
                # sconto in avanti (senza il sovrapprezzo laterale) la macchia crescerebbe ovunque invece
                # di restringersi ai lati come un vero ellissoide
                fattore_direzionale = max(0.35, (1.0 + ALLUNGAMENTO_BLOB) - 2 * ALLUNGAMENTO_BLOB * cos_theta * cos_theta)
                passo = passo * fattore_direzionale
            nuova_dist = dist + passo
            if nuova_dist > distanza_massima:
                continue
            if not (0 <= rv < y_tot_fine and 0 <= cv < x_tot_fine) or nuova_dist >= distanze.get((rv, cv), math.inf) or sottocella_e_muro(griglia, rv, cv, sottocelle_per_lato):
                continue
            if dr != 0 and dc != 0 and (sottocella_e_muro(griglia, r + dr, c, sottocelle_per_lato) or sottocella_e_muro(griglia, r, c + dc, sottocelle_per_lato)):
                continue  # niente tagli d'angolo: una mossa diagonale non deve "sfiorare" lo spigolo di un muro
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
    """Avanza di un passo lungo il percorso. Ritorna (nuovo_x, nuovo_y, arrivato_a_destinazione, bloccato).
    Consuma i nodi intermedi man mano che vengono raggiunti, cosi' il percorso resta
    percorribile per piu' frame anche se non viene ricalcolato ad ogni frame. `bloccato` e' True quando il
    passo verso il prossimo nodo taglierebbe un muro (es. l'entita' e' stata spostata fuori dalla linea del
    percorso da repulsione/coesione): il chiamante dovrebbe forzare un ricalcolo immediato invece di aspettare
    il prossimo intervallo periodico, altrimenti l'entita' resterebbe ferma inutilmente per diversi frame."""
    if len(percorso) > 1:
        tx, ty = percorso[1].cx, percorso[1].cy
    elif len(percorso) == 1:
        tx, ty = nodo_target.cx, nodo_target.cy
    else:
        return x, y, False, False

    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx**2 + dy**2)

    if len(percorso) <= 1 and dist <= RANGE_EVITAMENTO_PERSONE:
        # gia' abbastanza vicina alla destinazione finale (entro il raggio di repulsione): la considera
        # arrivata senza teletrasportarla sul punto esatto (che con piu' persone dirette allo stesso target
        # le farebbe sovrapporre tutte nello stesso pixel) - resta dov'e', a distanza naturale dagli altri
        return x, y, True, False

    if dist > velocita:
        nx, ny = x + (dx / dist) * velocita, y + (dy / dist) * velocita
        r, c = int(ny // DIM_NODO), int(nx // DIM_NODO)
        if not (0 <= r < Y_TOT and 0 <= c < X_TOT) or griglia[r][c].tipo == "muro":
            return x, y, False, True
        return nx, ny, False, False

    if len(percorso) > 1:
        percorso.pop(0)
    return tx, ty, len(percorso) <= 1, False

def fattore_velocita_da_conflitto(x, y, percorso, tracciamento_lidar, velocita_nominale, raggio_robot, raggio_persona,
                                   distanza_lookahead_px=DISTANZA_LOOKAHEAD_VELOCITA_PX):
    """Frazione (fra FATTORE_VELOCITA_ROBOT_MIN e FATTORE_VELOCITA_ROBOT_MAX) della velocita' nominale del
    robot: un controllo predittivo spazio-TEMPORALE, non un rallentamento generico vicino a una zona di
    rischio. Per ogni persona tracciata (non ferma) si proietta la sua traiettoria lineare con la stessa
    previsione usata per le macchie di probabilita' (stessa fonte dati, riusata qui invece che passando
    dalla griglia dei costi gia' rasterizzata) e si confronta con dove sarebbe il robot lungo il proprio
    percorso ALLO STESSO ISTANTE, non nello stesso punto in astratto.

    L'obiettivo e' non dover mai fermarsi del tutto: se non c'e' nessun incrocio previsto, velocita'
    nominale (nessuna ragione di sprintare se non c'e' nessuno da schivare). Se c'e' un incrocio previsto
    ma sprintare (fino a FATTORE_VELOCITA_ROBOT_MAX) permetterebbe di superare quel punto prima che la
    persona ci arrivi, il robot accelera invece di rallentare - il caso tipico di un attraversamento che si
    sta per chiudere ma si può ancora anticipare. Solo se nemmeno sprintando si evita l'incrocio, il robot
    rallenta (fino a FATTORE_VELOCITA_ROBOT_MIN) per lasciar passare la persona per prima, tanto piu' quanto
    piu' l'incrocio e' vicino nel tempo."""
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
        return 1.0  # nessun incrocio previsto entro l'orizzonte: velocita' nominale, niente da evitare
    if tempo_minimo_conflitto_sprint is None:
        return FATTORE_VELOCITA_ROBOT_MAX  # sprintando si supera l'incrocio prima che la persona ci arrivi
    urgenza = 1.0 - min(1.0, tempo_minimo_conflitto_nominale / FRAME_REAZIONE_VELOCITA_ROBOT)
    return 1.0 - urgenza * (1.0 - FATTORE_VELOCITA_ROBOT_MIN)

def salva_mappa(griglia, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    muri = [[n.r, n.c] for riga in griglia for n in riga if n.tipo == "muro"]
    with open(path, "w") as f:
        json.dump(muri, f)

def salva_mappa_training(griglia, cartella=CARTELLA_MAPPE_TRAINING):
    """Salva la mappa disegnata a mano come nuovo file libero (senza password, nome automatico progressivo)
    nella cartella dedicata alle mappe di training - separata dai 3 slot protetti dell'ambiente di test
    personale. training_scenari.py scandisce questa cartella, quindi ogni mappa salvata qui diventa
    automaticamente disponibile per generare scenari, senza bisogno di scrivere codice."""
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

def carica_mappa(griglia, path):
    if not os.path.exists(path):
        return False
    with open(path, "r") as f:
        muri = json.load(f)
    for riga in griglia:
        for n in riga:
            n.tipo = "libero"
    for r, c in muri:
        if 0 <= r < Y_TOT and 0 <= c < X_TOT:
            griglia[r][c].tipo = "muro"
    crea_bordi(griglia)
    return True

# --- POOL DI PROCESSI PER IL RICALCOLO PARALLELO DEI PERCORSI (persone/corridori/gruppi) ---
# Misurato: sullo scenario denso (~234 persone, carico reale di ~24 ricalcoli A* per frame a regime)
# un pool persistente a 8 worker porta il costo di quel batch da ~175ms a ~40ms/frame (4.4x). Ogni
# worker costruisce la propria copia della griglia UNA sola volta all'avvio (initializer): ad ogni
# dispatch si trasmette solo il payload leggero (partenza, destinazione, seed, celle bloccate), mai
# l'intera griglia. Il pool va ricreato quando i muri cambiano (editor mappe): per questo e' usato solo
# quando 'muri_modificati' e' False, altrimenti si ricade sul calcolo sequenziale (sempre corretto).
_pool_worker_griglia = None

def _pool_inizializza_worker(celle_muro):
    global _pool_worker_griglia
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
    return mp.Pool(n_worker, initializer=_pool_inizializza_worker, initargs=(_celle_muro_di(griglia),))

def main():
    # import locali (non a livello di modulo): main.py viene re-importato per intero in ogni processo
    # worker del pool multiprocessing (spawn su Windows), che pero' calcola solo percorsi A* e non usa
    # mai l'AI - tenerli qui invece che in cima al file evita ai worker di caricare inutilmente
    # torch/CUDA ad ogni avvio del pool, che allungava l'avvio della simulazione di decine di secondi
    import numpy as np
    import torch
    from ai_predittiva.allena_previsione import CorrezioneKalman, prepara_input, FINESTRA_STORICO_FRAME, FILE_MODELLO
    # i forward pass della correzione AI sono minuscoli (batch di poche decine di persone, GRU a 32
    # unita'): il multithreading intra-op di default di torch (quanti core ha la CPU) spende piu' tempo
    # a sincronizzare i thread che a calcolare, e quella sincronizzazione compete col loop di pygame per
    # il tempo di CPU - a thread singolo il forward pass e' piu' veloce e non lagga il rendering
    torch.set_num_threads(1)

    pygame.init()
    fullscreen = False

    # Adatta la finestra iniziale allo schermo (max 90% di larghezza/altezza disponibili)
    info = pygame.display.Info()
    win_w = min(LARGHEZZA, int(info.current_w * 0.9))
    win_h = min(ALTEZZA, int(info.current_h * 0.9))
    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
    pygame.key.start_text_input()
    pygame.display.set_caption("Simulazione - Zoom: Rote. | Pan: Tasto DX | Muri: Tasto W | F11: Fullscreen | F5: Salva | F9: Carica | T: Salva mappa training | TAB: Pannello")
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
    
    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)

    # modello di correzione AI sopra al Kalman (Livello 2): allenato offline da allena_previsione.py sul
    # dataset di genera_dataset_previsione.py, caricato qui solo per l'inferenza (nessun training dal
    # vivo). Se il file non esiste ancora (non e' mai stato allenato) il toggle in pannello resta
    # semplicemente senza effetto, il sistema si comporta come prima (solo Kalman)
    modello_correzione_ai = CorrezioneKalman()
    modello_ai_disponibile = os.path.exists(FILE_MODELLO)
    if modello_ai_disponibile:
        modello_correzione_ai.load_state_dict(torch.load(FILE_MODELLO, map_location="cpu"))
        modello_correzione_ai.eval()

    # pool di processi persistente per il ricalcolo parallelo dei percorsi (vedi commento sopra la
    # definizione): 'muri_modificati' e 'frame_ultima_modifica_muro' tengono traccia di quando i muri
    # cambiano (editor mappe) per sapere quando la copia della griglia nei worker e' da rigenerare
    pool_persone = _crea_pool_persone(griglia)
    muri_modificati = False
    frame_ultima_modifica_muro = -9999
    # batch di ricalcolo percorsi in sospeso presso il pool (vedi commento piu' sotto, dove veniva prima
    # dispacciato con .map() bloccante): None quando nessun batch e' in volo
    richiesta_percorsi_pendente = None
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
    celle_rilevate_precedenti = set()  # posizioni (vere, senza rumore) rilevate dal lidar nel frame precedente
    tracciamento_lidar = {}  # id(persona) -> traccia Kalman, solo per le persone attualmente rilevate dal lidar
    storico_posizioni_lidar = {}  # id(persona) -> deque delle ultime FINESTRA_STORICO_FRAME posizioni/velocita' filtrate dal Kalman: input della correzione AI (stessa finestra usata in training)
    mappa_pesi_render = {}  # ultima macchia di probabilita' calcolata (per sottocella, peso 0-1): persiste fra un ricalcolo e l'altro per il disegno
    mappa_pesi_render_dim_sottocella = DIM_NODO / SOTTOCELLE_PER_LATO_DEFAULT  # dimensione delle sottocelle usata per l'ultima macchia calcolata (per disegnarla con le coordinate giuste anche se lo slider e' cambiato nel frattempo)
    mappa_costo_probabilita = {}  # ultimo costo extra per l'A* derivato dalla macchia (per cella grossa): persiste perche' ora si aggiorna a una cadenza propria, separata dal ricalcolo del percorso

    # --- NUMERO PERSONE/CORRIDORI/FERME/GRUPPI E PANNELLO TECNICO (valori di default) ---
    numero_persone = NUMERO_PERSONE_DEFAULT
    numero_corridori = NUMERO_CORRIDORI_DEFAULT
    numero_persone_ferme = NUMERO_PERSONE_FERME_DEFAULT
    numero_gruppi = NUMERO_GRUPPI_DEFAULT
    pannello_aperto = False
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
    # AI di predizione (correzione neurale del residuo del Kalman): DISATTIVATA di default. Il toggle
    # riguarda solo la correzione - il Kalman classico resta sempre attivo sotto. Spenta perche' le misure
    # mostrano che migliora la previsione (-9.2% di errore nel ciclo reale) ma non il tempo di percorrenza:
    # il guadagno viene assorbito dalla macchia di probabilita', larga 90px contro ~38px di incertezza
    # effettiva. Le quattro cause misurate sono documentate in ai_predittiva/__init__.py
    ai_attiva = False
    controllo_velocita_attivo = True  # rallenta il robot in base al rischio davanti sul suo percorso (Livello 3 euristico)
    configurazione_salvataggio_timer = 0  # breve conferma visiva dopo il click su uno dei pulsanti in fondo al pannello
    configurazione_messaggio = ("", VERDE)

    # configurazione salvata dal pannello tecnico (pulsante "Salva configurazione"): se presente
    # sovrascrive i valori di default appena impostati, cosi' l'ambiente di test preferito e' gia'
    # pronto all'avvio invece di doverlo riconfigurare ogni volta
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

    # --- PERSONE / CORRIDORI (doppia velocita') / FERME (ostacoli statici) / GRUPPI ---
    persone = [crea_persona(griglia) for _ in range(numero_persone)]
    corridori = [crea_corridore(griglia) for _ in range(numero_corridori)]
    persone_ferme = [crea_persona_ferma(griglia) for _ in range(numero_persone_ferme)]
    gruppi = [crea_gruppo(griglia) for _ in range(numero_gruppi)]

    contatore_frame = 0

    running = True
    while running:
        screen.fill(GRIGIO_PAVIMENTO)
        
        # Funzioni di conversione coordinate
        def t_s(x, y): return int(x * zoom + offset_x), int(y * zoom + offset_y)
        def t_m(sx, sy): return (sx - offset_x) / zoom, (sy - offset_y) / zoom

        # 1. Disegno Scacchiera (solo le celle visibili nella viewport, per performance)
        cella_px = DIM_NODO * zoom
        col_min = max(0, int(-offset_x / cella_px))
        col_max = min(X_TOT - 1, int((screen.get_width() - offset_x) / cella_px))
        row_min = max(0, int(-offset_y / cella_px))
        row_max = min(Y_TOT - 1, int((screen.get_height() - offset_y) / cella_px))

        for r in range(row_min, row_max + 1):
            for c in range(col_min, col_max + 1):
                n = griglia[r][c]
                # Bordo destro/inferiore preso dalla cella successiva: combaciano sempre, senza fessure da arrotondamento
                sx, sy = t_s(n.x, n.y)
                ex, ey = t_s(n.x + DIM_NODO, n.y + DIM_NODO)
                rect = (sx, sy, ex - sx, ey - sy)
                if n.tipo == "muro":
                    pygame.draw.rect(screen, NERO, rect)
                else:
                    pygame.draw.rect(screen, GRIGIO_BORDO_CELLA, rect, 1)

        # Layout pannello tecnico (ricalcolato ogni frame per seguire il resize)
        panel_w, panel_h = 370, 765
        panel_rect = pygame.Rect(screen.get_width() - panel_w - 20, 20, panel_w, panel_h)
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
        # pulsanti sempre in fondo al pannello, per avere l'ambiente di test preferito pronto ad ogni avvio
        larghezza_pulsante_config = (panel_w - 30 - 20) // 3
        button_salva_config_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 715, larghezza_pulsante_config, 35)
        button_carica_config_rect = pygame.Rect(button_salva_config_rect.right + 10, panel_rect.y + 715, larghezza_pulsante_config, 35)
        button_reset_config_rect = pygame.Rect(button_carica_config_rect.right + 10, panel_rect.y + 715, larghezza_pulsante_config, 35)

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
                        # il tipo del primo punto cliccato decide il riempimento di tutto il rettangolo:
                        # se era un muro lo svuota, se era libero lo riempie di muri (come il toggle di W,
                        # ma applicato in blocco invece che cella per cella)
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

                if event.button == 1 and pannello_aperto and panel_rect.collidepoint(event.pos):
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
                            configurazione_messaggio = ("Configurazione caricata", VERDE)
                        else:
                            configurazione_messaggio = ("Nessuna configurazione salvata", ROSSO)
                        configurazione_salvataggio_timer = 90
                    elif button_reset_config_rect.collidepoint(event.pos):
                        # azzera solo l'ambiente ATTUALE ai valori di default: non tocca il file salvato,
                        # cosi' si puo' pulire e poi ricaricare con "Carica config." l'ambiente di prima
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

                # Gestione Panning
                if trascinando:
                    dx, dy = event.pos[0] - ultima_pos_mouse[0], event.pos[1] - ultima_pos_mouse[1]
                    offset_x += dx
                    offset_y += dy
                    ultima_pos_mouse = event.pos

                # --- AGGIUNTA: POSIZIONAMENTO/RIMOZIONE MURI CON TASTO 'W' (trascinando) ---
                keys = pygame.key.get_pressed()
                if keys[pygame.K_w]:
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    # Evitiamo di modificare i bordi esterni e restiamo nei limiti
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
                            salva_mappa(griglia, FILE_MAPPA_SLOT[slot_da_salvare])
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
                    }.get(event.key)
                    if indice_slot is not None:
                        if menu_salvataggio_aperto:
                            # lo slot si sceglie subito, la password viene chiesta solo dopo (F5 la richiede comunque)
                            slot_da_salvare = indice_slot
                            menu_salvataggio_aperto = False
                            chiedendo_password = True
                            input_password = ""
                        else:
                            if os.path.exists(FILE_MAPPA_SLOT[indice_slot]):
                                if carica_mappa(griglia, FILE_MAPPA_SLOT[indice_slot]):
                                    muri_modificati = True
                                    frame_ultima_modifica_muro = contatore_frame
                                    target_pos, percorso = None, []
                                    persone = [crea_persona(griglia) for _ in range(numero_persone)]
                                    corridori = [crea_corridore(griglia) for _ in range(numero_corridori)]
                                    persone_ferme = [crea_persona_ferma(griglia) for _ in range(numero_persone_ferme)]
                                    gruppi = [crea_gruppo(griglia) for _ in range(numero_gruppi)]
                                    # la mappa nuova puo' avere un muro dove si trovava il robot: lo
                                    # ricolloca in una casella libera invece di lasciarlo incastrato
                                    r_robot, c_robot = int(robot_y // DIM_NODO), int(robot_x // DIM_NODO)
                                    if not (0 <= r_robot < Y_TOT and 0 <= c_robot < X_TOT) or griglia[r_robot][c_robot].tipo == "muro":
                                        nodo_robot = cella_libera_casuale(griglia)
                                        robot_x, robot_y = nodo_robot.cx, nodo_robot.cy
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

                # Toggle muro singolo alla pressione di W (e inizio del trascinamento)
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

        zoom, offset_x, offset_y = limita_zoom_pan(zoom, offset_x, offset_y, screen.get_width(), screen.get_height())

        # 3. Movimento
        tempo_di_ricalcolare_robot = contatore_frame % INTERVALLO_RICALCOLO_ROBOT_FRAME == 0
        # NB: sicurezza_attiva / lidar_attivo / ellissoidi_attivi (pannello tecnico) nascondono SOLO il
        # disegno (cerchi, colori delle persone, macchie): il meccanismo sottostante (stop di sicurezza,
        # rilevamento, costo di probabilita' nell'A*) resta sempre attivo, i toggle non lo influenzano

        membri_gruppi_mobili = [m for g in gruppi if g["mobile"] for m in g["membri"]]
        tutte_mobili = persone + corridori + membri_gruppi_mobili  # persone, corridori e membri dei gruppi si muovono ed evitano gli altri tutti allo stesso modo

        robot_bloccato = raggio_sicurezza > 0 and any(
            math.hypot(robot_x - p["x"], robot_y - p["y"]) < raggio_sicurezza for p in tutte_mobili
        )

        celle_persone_ferme = {(pf["r"], pf["c"]) for pf in persone_ferme}
        celle_persone_ferme |= {(m["r"], m["c"]) for g in gruppi if not g["mobile"] for m in g["membri"]}

        # rilevamento "lidar" del robot: la planimetria (muri) la conosce gia' a priori, ma le persone
        # (in movimento o ferme) diventano ostacoli noti per il suo A* solo entro raggio_lidar. Il
        # confronto per decidere se serve un ricalcolo usa le celle vere, non quelle rumorose: il rumore
        # da solo (ricampionato ogni frame) cambierebbe cella a ogni frame anche per una persona immobile,
        # forzando un ricalcolo continuo inutile. Il rumore si applica solo al costo passato all'A*, per
        # simulare l'incertezza sulla posizione percepita.
        tutte_le_persone = tutte_mobili + persone_ferme + [m for g in gruppi if not g["mobile"] for m in g["membri"]]
        celle_rilevate = set()
        celle_rilevate_rumorose = set()
        id_rilevati_ora = set()
        for p in tutte_le_persone:
            d = math.hypot(p["x"] - robot_x, p["y"] - robot_y)
            if raggio_lidar > 0 and d < raggio_lidar and linea_di_vista_libera(griglia, robot_x, robot_y, p["x"], p["y"]):
                r_vera, c_vera = int(p["y"] // DIM_NODO), int(p["x"] // DIM_NODO)
                celle_rilevate.add((r_vera, c_vera))
                angolo_rumore = random.uniform(0, 2 * math.pi)
                raggio_rumore = random.uniform(0, RUMORE_LIDAR_PX)
                xr, yr = p["x"] + raggio_rumore * math.cos(angolo_rumore), p["y"] + raggio_rumore * math.sin(angolo_rumore)
                r_rum = max(0, min(Y_TOT - 1, int(yr // DIM_NODO)))
                c_rum = max(0, min(X_TOT - 1, int(xr // DIM_NODO)))
                celle_rilevate_rumorose.add((r_rum, c_rum))

                # traccia Kalman: SOLO per le persone attualmente rilevate (mai per tutta la
                # popolazione), altrimenti con centinaia di persone il costo esploderebbe. Sempre
                # attiva indipendentemente da ellissoidi_attivi: quel toggle nasconde solo il disegno,
                # il costo di probabilita' nell'A* del robot resta comunque in funzione
                pid = id(p)
                id_rilevati_ora.add(pid)
                if pid not in tracciamento_lidar:
                    tracciamento_lidar[pid] = nuova_traccia_kalman(xr, yr)
                    storico_posizioni_lidar[pid] = collections.deque(maxlen=FINESTRA_STORICO_FRAME)
                else:
                    aggiorna_traccia_kalman(tracciamento_lidar[pid], xr, yr)
                # stato "ferma" noto con certezza dalla simulazione stessa (non stimato dal rumore): le
                # persone_ferme e i membri statici dei gruppi non hanno affatto la chiave "stato", le
                # persone/corridori/membri mobili ce l'hanno e valgono "attesa" quando sono davvero ferme
                # in questo istante. Una soglia sulla sola velocita' stimata dal Kalman non basta: con
                # RUMORE_LIDAR_PX=8 il rumore residuo di un target davvero fermo (fino a ~1.3px/frame nei
                # casi peggiori) si sovrappone quasi del tutto alla velocita' di un pedone lentissimo
                # (0.4px/frame), quindi non esiste una soglia che separi bene i due casi
                tracciamento_lidar[pid]["ferma"] = p.get("stato", "attesa") == "attesa"
                traccia_pid = tracciamento_lidar[pid]
                storico_posizioni_lidar[pid].append((traccia_pid["x"], traccia_pid["y"], traccia_pid["vx"], traccia_pid["vy"]))

        # le tracce di persone non piu' rilevate (uscite dal raggio o nascoste da un muro) vengono
        # abbandonate subito, nessun "coasting": coerente con "il robot ragiona solo su cio' che percepisce"
        for pid_vecchio in list(tracciamento_lidar.keys()):
            if pid_vecchio not in id_rilevati_ora:
                del tracciamento_lidar[pid_vecchio]
                storico_posizioni_lidar.pop(pid_vecchio, None)

        # percorso 'da proteggere' per l'isteresi (vedi piu' sotto): va catturato PRIMA dell'eventuale
        # azzeramento qui sotto, altrimenti ogni volta che l'insieme delle persone rilevate cambia anche
        # di una sola cella (capita spessissimo con una scena affollata) l'isteresi si ritroverebbe a
        # proteggere una lista vuota, cioe' a non fare nulla
        percorso_precedente_per_isteresi = percorso

        if celle_rilevate != celle_rilevate_precedenti:
            percorso = []  # e' cambiato l'insieme di persone rilevate: il percorso pianificato potrebbe non essere piu' valido
        celle_rilevate_precedenti = celle_rilevate

        # macchia di probabilita': aggiornata alla propria cadenza (10 volte al secondo di default),
        # indipendente da quando il robot ricalcola davvero il percorso - si proietta in avanti ogni
        # traccia rilevata e si fa diffondere il suo peso attraverso lo spazio libero, cosi' il costo
        # extra e' morbido (decresce dal centro previsto) invece del blocco secco usato per la posizione
        # rilevata adesso. Separata dal ricalcolo del percorso: prima scattavano insieme (una volta al
        # secondo, o quando cambiava l'insieme di celle rilevate), rendendo l'ellisse visibilmente "a
        # scatti" invece che fluida.
        if contatore_frame % INTERVALLO_AGGIORNAMENTO_MACCHIA_FRAME == 0:
            # "definizione ellissoidi" (pannello tecnico): quante sottocelle per lato compone la macchia.
            # Al minimo (1) coincide con la griglia di movimento (quadrati), al massimo le sottocelle
            # sono cosi' piccole che la macchia appare un ellissoide continuo
            sottocelle_per_lato = max(SOTTOCELLE_PER_LATO_MIN, round(definizione_ellissoidi))
            dim_sottocella = DIM_NODO / sottocelle_per_lato
            y_tot_fine, x_tot_fine = Y_TOT * sottocelle_per_lato, X_TOT * sottocelle_per_lato
            raggio_blob_sottocelle = RAGGIO_BLOB_CELLE * sottocelle_per_lato
            decadimento_blob_sottocella = DECADIMENTO_BLOB ** (1 / sottocelle_per_lato)

            # correzione AI (Livello 2): un solo forward pass "a batch" per tutte le persone invece di uno
            # per persona - il costo fisso per chiamata di torch (creazione tensori, dispatch del modulo,
            # ecc.) domina su un input cosi' piccolo, quindi farne una sola con tutte le tracce impilate
            # e' molto piu' leggero che ripeterla in un ciclo, a parita' di risultato numerico
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
                # persona davvero ferma (stato noto dalla simulazione, non stimato dal rumore): niente
                # proiezione in avanti ne' orientamento, altrimenti anche un residuo di rumore verrebbe
                # amplificato di PREVISIONE_BLOB_FRAME volte e la macchia si sposterebbe/orienterebbe
                # visibilmente ad ogni aggiornamento pur restando la persona immobile
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
                # combinazione (non massimo) fra gli ellissoidi di persone diverse: dove si sovrappongono
                # il rischio cresce un po' rispetto alla singola persona piu' vicina, ma satura verso 1.0
                # invece di sommarsi linearmente - vedi il commento sopra la definizione della macchia
                for sottocella, peso in macchia_diffusione(griglia, rf_prev, cf_prev, raggio_blob_sottocelle, decadimento_blob_sottocella, sottocelle_per_lato, direzione=direzione_traccia).items():
                    peso_precedente = mappa_pesi_render.get(sottocella, 0.0)
                    mappa_pesi_render[sottocella] = 1.0 - (1.0 - peso_precedente) * (1.0 - peso)
            # per l'A* (che resta sulla griglia originale) si aggregano le sottocelle nella cella grossa
            # che le contiene, prendendo il peso massimo fra quelle che vi cadono dentro
            mappa_costo_probabilita = {}
            for (rf, cf), peso in mappa_pesi_render.items():
                cella = (rf // sottocelle_per_lato, cf // sottocelle_per_lato)
                costo = peso * COSTO_MASSIMO_PROBABILITA
                if cella not in mappa_costo_probabilita or mappa_costo_probabilita[cella] < costo:
                    mappa_costo_probabilita[cella] = costo
            mappa_pesi_render_dim_sottocella = dim_sottocella  # ricordata per il disegno, che avviene in un punto diverso del frame

        if target_pos and not robot_bloccato:
            if not percorso or tempo_di_ricalcolare_robot:
                # isteresi: sconto sulla prossima porzione del percorso gia' in corso, per non ribaltare
                # la scelta fra due percorsi quasi equivalenti a ogni ricalcolo solo per rumore (vedi
                # commento sulla costante COSTO_ISTERESI_PERCORSO). Con Theta* il percorso puo' avere
                # pochissimi waypoint (una scorciatoia lunga = un solo segmento fra due nodi anche molto
                # distanti), quindi lo sconto cammina lungo la geometria reale dei segmenti campionando
                # ogni DIM_NODO px, invece di contare le prime N voci della lista - cosi' copre sempre la
                # stessa portata fisica (~CELLE_ISTERESI_PERCORSO celle) indipendentemente da quanti nodi
                # produce la ricerca any-angle.
                mappa_costo_extra_robot = dict(mappa_costo_probabilita)
                budget_isteresi_px = CELLE_ISTERESI_PERCORSO * DIM_NODO
                distanza_isteresi_percorsa = 0.0
                for i in range(len(percorso_precedente_per_isteresi) - 1):
                    if distanza_isteresi_percorsa >= budget_isteresi_px:
                        break
                    a, b = percorso_precedente_per_isteresi[i], percorso_precedente_per_isteresi[i + 1]
                    lunghezza_segmento = math.hypot(b.cx - a.cx, b.cy - a.cy)
                    passi_segmento = max(1, int(lunghezza_segmento / DIM_NODO))
                    for passo_i in range(passi_segmento + 1):
                        if distanza_isteresi_percorsa >= budget_isteresi_px:
                            break
                        t = passo_i / passi_segmento
                        x = a.cx + (b.cx - a.cx) * t
                        y = a.cy + (b.cy - a.cy) * t
                        cella = (int(y // DIM_NODO), int(x // DIM_NODO))
                        mappa_costo_extra_robot[cella] = mappa_costo_extra_robot.get(cella, 0.0) - COSTO_ISTERESI_PERCORSO
                        distanza_isteresi_percorsa += lunghezza_segmento / passi_segmento

                percorso = algoritmo_a_star(griglia, (robot_x, robot_y), target_pos, celle_bloccate=celle_rilevate_rumorose, mappa_costo_extra=mappa_costo_extra_robot)
            meta_nodo = griglia[target_pos[0]][target_pos[1]]
            velocita_nominale_robot = VELOCITA_ROBOT * moltiplicatore_velocita
            fattore_velocita_rischio = fattore_velocita_da_conflitto(
                robot_x, robot_y, percorso, tracciamento_lidar, velocita_nominale_robot, raggio_robot, raggio_persona
            ) if controllo_velocita_attivo else 1.0
            robot_x, robot_y, arrivato, passo_muro = passo_movimento(robot_x, robot_y, percorso, velocita_nominale_robot * fattore_velocita_rischio, meta_nodo, griglia)
            if passo_muro:
                percorso = []  # il passo tagliava un muro: ricalcola subito invece di aspettare il prossimo intervallo
            if arrivato:
                if frame_inizio_target is not None:
                    # a velocita' di simulazione x10 il robot copre 10x la distanza per frame, quindi in
                    # frame "grezzi" ci mette 1/10 del tempo: si moltiplica per il fattore di velocita'
                    # corrente per mostrare il tempo equivalente a velocita' normale (1x), non il tempo
                    # grezzo di frame che sottostima quanto ci metterebbe davvero un robot reale
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
        # cella occupata da qualunque entita' (mobile, ferma o il robot): calcolata una sola volta per frame
        # (non per persona) e usata come penalita' di costo comune nell'A* di tutti - il costo di ricalcolarla
        # per ciascuna persona scalava male col numero di persone; la stessa cella "occupata da se stessi" non
        # cambia il percorso (il nodo di partenza non viene mai rivisitato da un A* corretto)
        celle_bloccate_comune = {
            (int(p2["y"] // DIM_NODO), int(p2["x"] // DIM_NODO)) for p2 in tutte_mobili
        }
        celle_bloccate_comune |= celle_persone_ferme
        celle_bloccate_comune.add((int(robot_y // DIM_NODO), int(robot_x // DIM_NODO)))

        # se i muri sono stati modificati (editor mappe) e sono passati abbastanza frame da quando e'
        # cambiato l'ultimo (evita di ricreare il pool decine di volte al secondo mentre si trascina il
        # tasto W), il pool va ricreato con la griglia aggiornata: finche' resta 'sporco' si ricade sul
        # calcolo sequenziale, sempre corretto anche se piu' lento
        if muri_modificati and contatore_frame - frame_ultima_modifica_muro > 15:
            pool_persone.terminate()
            pool_persone = _crea_pool_persone(griglia)
            muri_modificati = False
            richiesta_percorsi_pendente = None  # risultati del pool appena terminato: ormai invalidi

        # ricalcolo percorsi: fase separata dal movimento cosi' il batch di chi deve ricalcolare in
        # questo frame puo' essere dispacciato in blocco al pool di processi persistente (paralleliz-
        # zato sui core della CPU) invece che uno alla volta nel processo principale. Il primo calcolo
        # di ciascuna persona (percorso ancora None, appena creata) e' l'unico caso in cui centinaia di
        # persone ne avrebbero bisogno nello stesso istante (frame 0): e' l'unico che vale la pena
        # spalmare sui primi frame invece di farlo scattare tutto insieme, usando lo stesso
        # offset_ricalcolo gia' usato per il ricalcolo periodico. I ricalcoli successivi (percorso
        # invalidato da un ostacolo, nuovo target, ecc.) restano immediati: sono singoli eventi gia'
        # naturalmente distribuiti nel tempo, non un picco simultaneo
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

        # mentre i muri sono "sporchi" (si sta disegnando: vedi il commento sopra la ricreazione del pool)
        # si salta del tutto il ricalcolo di questo frame invece di ricadere sul calcolo sequenziale nel
        # thread principale: con molte persone, ricalcolare uno alla volta ad ogni frame di trascinamento
        # del tasto W causava un impuntamento evidente. Chi aspettava un ricalcolo lo ottiene comunque al
        # turno successivo (al massimo una manciata di frame dopo che il disegno si ferma), nel frattempo
        # resta semplicemente fermo sull'ultimo percorso valido invece di intasare il frame corrente

        # raccolta ASINCRONA del batch precedente, se pronto: su mappe intricate un singolo A* esplora
        # molti piu' nodi per aggirare i muri rispetto a uno spazio aperto, quindi il batch puo' impiegare
        # piu' di un frame a tornare. Con .map() (bloccante) l'intero gioco si fermava fino al risultato
        # piu' lento del batch, causando lo scatto periodico che si vede sulle mappe con molti ostacoli.
        # Con .map_async() il loop principale continua a girare a piena velocita' mentre i worker
        # calcolano in sottofondo: i risultati si applicano appena pronti, nel frattempo ciascuna persona
        # resta sul suo ultimo percorso valido (stesso principio di tolleranza gia' usato sopra)
        if richiesta_percorsi_pendente is not None and richiesta_percorsi_pendente.ready():
            for p, percorso_calcolato in zip(persone_richiesta_percorsi_pendente, richiesta_percorsi_pendente.get()):
                p["percorso"] = percorso_calcolato
            richiesta_percorsi_pendente = None

        # un nuovo batch parte solo se il precedente e' gia' stato raccolto: chi resta escluso perche' il
        # pool e' ancora occupato riprovera' al proprio prossimo turno periodico, senza accumulare batch
        # concorrenti sullo stesso pool
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
                    # niente attrazione/trascinamento verso il leader (si incastrava contro i muri vicino
                    # alle porte): solo, quando resta indietro oltre la normale distanza di gruppo, piu'
                    # velocita' in proporzione a quanto e' distante, camminando comunque sul proprio percorso
                    # valido. Entro la distanza normale nessun boost: cosi' cammina alla stessa identica
                    # velocita' degli altri finche' resta vicino, invece di avere sempre un moltiplicatore
                    # leggermente diverso da chiunque altro anche a pochi passi dal leader
                    leader = p["gruppo"]["membri"][0]
                    distanza_leader = math.hypot(p["x"] - leader["x"], p["y"] - leader["y"])
                    if distanza_leader > GRUPPO_DISTANZA_COESIONE_PX:
                        velocita_p *= min(GRUPPO_FATTORE_RINCORSA_MAX, 1.0 + (distanza_leader - GRUPPO_DISTANZA_COESIONE_PX) / GRUPPO_DISTANZA_COESIONE_PX)
                p["velocita_attuale"] = velocita_p
                x_prima, y_prima = p["x"], p["y"]
                p["x"], p["y"], arrivata, passo_muro = passo_movimento(p["x"], p["y"], p["percorso"], velocita_p, nodo_target_p, griglia)
                if passo_muro:
                    p["percorso"] = []  # il passo tagliava un muro: ricalcola subito invece di aspettare il prossimo intervallo

                # scarto continuo e proporzionale per non trapassare le altre persone vicine (niente blocchi a scatti)
                rep_x, rep_y = repulsione_vicini(p, vicini_spaziali(p, griglia_spaziale, DIM_NODO), range_evitamento_quad)
                p["x"], p["y"] = sposta_con_vettore(p["x"], p["y"], rep_x, rep_y, FORZA_REPULSIONE_PERSONE, griglia)

                # il robot e' un ostacolo fisico: le persone possono avvicinarsi ed entrare nella sua zona di
                # sicurezza (che riguarda solo il robot, non loro), ma non compenetrare il suo corpo - lo aggirano
                rrep_x, rrep_y = repulsione_punto(p, robot_x, robot_y, range_evitamento_robot)
                p["x"], p["y"] = sposta_con_vettore(p["x"], p["y"], rrep_x, rrep_y, FORZA_REPULSIONE_ROBOT, griglia)

                if non_capofila:
                    spostamento = math.hypot(p["x"] - x_prima, p["y"] - y_prima)
                    p["frame_fermo"] = 0 if spostamento > 0.3 else p.get("frame_fermo", 0) + 1
                    leader = p["gruppo"]["membri"][0]
                    distanza_leader = math.hypot(p["x"] - leader["x"], p["y"] - leader["y"])
                    # va inseguito direttamente il leader se: e' rimasto bloccato a lungo (es. incrocio
                    # affollato vicino a una porta) oppure si e' allontanato troppo pur muovendosi (il proprio
                    # A* puo' aver imboccato una rotta completamente diversa dalla sua verso l'obiettivo comune,
                    # es. aggirando un ostacolo su un lato opposto - la sola rincorsa di velocita' non basta
                    # perche' corre veloce nella direzione sbagliata)
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
                            # il leader e' vivo e si muove: la meta va aggiornata ogni frame alla sua
                            # posizione attuale, non congelata al punto in cui e' scattato l'inseguimento
                            # (altrimenti lo si raggiunge ma e' un punto ormai vecchio, ci si "arriva" e ci
                            # si ferma in attesa mentre il leader vero continua ad allontanarsi)
                            p["inseguendo_leader"] = True
                            p["frame_fermo"] = 0
                            p["target"] = (max(0, min(Y_TOT - 1, int(leader["y"] // DIM_NODO))), max(0, min(X_TOT - 1, int(leader["x"] // DIM_NODO))))

                # mentre insegue il leader "arrivare" al punto attuale non e' un vero arrivo (lui si muove
                # ancora): non si ferma in attesa, continua a rincorrerlo finche' non lo riprende davvero
                if arrivata and not (non_capofila and p.get("inseguendo_leader")):
                    p["stato"] = "attesa"
                    p["attesa_timer"] = random.randint(ATTESA_PERSONA_MIN_FRAME, ATTESA_PERSONA_MAX_FRAME)
            elif p["stato"] == "attesa":
                # anche da ferme mantengono un minimo di distanza dalle altre persone vicine (come se si
                # fossero fermate a parlare a distanza naturale, non ammassate tutte sullo stesso punto)
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
                        # il capofila decide per tutto il gruppo: interrompe l'attesa/il percorso degli altri
                        # membri e li reindirizza subito verso la stessa nuova destinazione condivisa
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

        # 4. Rendering (Invariato)
        # raggi "effettivi" SOLO per il disegno (cerchi, colore delle persone): 0 se il toggle relativo
        # e' spento nel pannello, cosi' non si vede nulla, ma il meccanismo vero (sopra, in "3. Movimento")
        # ha gia' usato i raggi reali e non e' influenzato da questi toggle
        raggio_sicurezza_effettivo = raggio_sicurezza if sicurezza_attiva else 0
        raggio_lidar_effettivo = raggio_lidar if lidar_attivo else 0

        if percorso and len(percorso) > 1:
            punti = [t_s(n.cx, n.cy) for n in percorso]
            pygame.draw.lines(screen, ROSSO, False, punti, 2)

        # macchie di probabilita' (previsione lidar): disegnate sulla sotto-griglia fine (solo le
        # sottocelle con un peso calcolato, mai su tutta la mappa) - tonalita' termica blu, piu' opaco
        # al centro della macchia, gradiente piu' morbido rispetto alla griglia di movimento
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

        if messaggio_arrivo_timer > 0:
            messaggio_arrivo_timer -= 1
            arrivo_txt = font_grande.render(messaggio_arrivo_testo, True, VERDE)
            screen.blit(arrivo_txt, arrivo_txt.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2)))

        # 5. Overlay richiesta password (salvataggio F5)
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

        # 5b. Overlay menu slot mappa (F5 salva / F9 carica)
        if slot_vuoto_timer > 0:
            slot_vuoto_timer -= 1
        if menu_salvataggio_aperto or menu_caricamento_aperto:
            box_slot_w, box_slot_h = 360, 210
            box_slot_x = (screen.get_width() - box_slot_w) // 2
            box_slot_y = (screen.get_height() - box_slot_h) // 2
            pygame.draw.rect(screen, BIANCO, (box_slot_x, box_slot_y, box_slot_w, box_slot_h))
            pygame.draw.rect(screen, NERO, (box_slot_x, box_slot_y, box_slot_w, box_slot_h), 2)

            titolo_slot = font.render("Esc annulla", True, NERO)
            screen.blit(titolo_slot, (box_slot_x + 15, box_slot_y + 15))

            for i, path in enumerate(FILE_MAPPA_SLOT):
                stato = "occupato" if os.path.exists(path) else "vuoto"
                riga_slot = font.render(f"{i + 1}. {os.path.basename(path)} - {stato}", True, NERO)
                screen.blit(riga_slot, (box_slot_x + 15, box_slot_y + 55 + i * 40))

            if menu_caricamento_aperto and slot_vuoto_timer > 0:
                avviso_slot = font.render("Slot vuoto: nessuna mappa da caricare", True, ROSSO)
                screen.blit(avviso_slot, (box_slot_x + 15, box_slot_y + box_slot_h - 30))

        # 6. Pannello tecnico (TAB per aprire/chiudere)
        if pannello_aperto:
            pygame.draw.rect(screen, BIANCO, panel_rect)
            pygame.draw.rect(screen, NERO, panel_rect, 2)

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

            # pulsanti in fondo al pannello: salvano/azzerano su disco la configurazione (numeri, raggi,
            # toggle, velocita'), cosi' l'ambiente di test preferito e' pronto ad ogni riavvio del gioco
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
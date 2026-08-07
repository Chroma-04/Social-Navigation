import os
os.environ["SDL_VIDEO_CENTERED"] = "1"
import pygame
import math
import json
import random
import heapq

# --- CONFIGURAZIONE ---
X_TOT, Y_TOT = 80, 60
DIM_NODO = 30
LARGHEZZA, ALTEZZA = X_TOT * DIM_NODO, Y_TOT * DIM_NODO
VELOCITA_ROBOT = 1.5
VELOCITA_PERSONA = 1.0
ATTESA_PERSONA_MIN_FRAME = 1 * 60  # attesa minima dopo l'arrivo, in frame (60 FPS)
ATTESA_PERSONA_MAX_FRAME = 4 * 60  # attesa massima dopo l'arrivo, in frame
PROBABILITA_PERCORSO_BREVE = 0.35  # probabilita' che la prossima destinazione sia vicina invece che casuale ovunque
RAGGIO_PERCORSO_BREVE = DIM_NODO * 6  # distanza massima (in pixel) per un "percorso breve"
RANGE_EVITAMENTO_PERSONE = 15  # px: sotto questa distanza da un'altra persona scatta una spinta di repulsione
FORZA_REPULSIONE_PERSONE = 1.3  # px/frame massimi di spinta continua fra persone/corridori/leader (a distanza 0)
COSTO_CELLA_OCCUPATA = DIM_NODO * 10  # penalita' di costo A* per una cella occupata da un'altra entita': forte ma non un divieto assoluto (evita percorsi vuoti nei passaggi a 1 cella)
INTERVALLO_RICALCOLO_ROBOT_FRAME = 60    # ricalcolo percorso robot: 60 FPS / questo valore = volte al secondo
INTERVALLO_RICALCOLO_PERSONE_FRAME = 10  # ricalcolo percorso persone: 60 FPS / questo valore = volte al secondo
FILE_MAPPA_SLOT = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappa_salvata.json"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappa_salvata_2.json"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappa_salvata_3.json"),
]
PASSWORD_SALVATAGGIO = "1258"
VEL_MULT_MIN, VEL_MULT_MAX = 0.2, 3.0
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
RAGGIO_SICUREZZA_DEFAULT = 20
ALPHA_ZONA_SICUREZZA = 70  # trasparenza del cerchio (0-255)
RAGGIO_ROBOT_MIN, RAGGIO_ROBOT_MAX = 5, int(DIM_NODO * 1.5)
RAGGIO_ROBOT_DEFAULT = DIM_NODO // 3  # raggio "fisico" del robot (lo stesso con cui viene disegnato di default)
FORZA_REPULSIONE_ROBOT = 1.3  # px/frame massimi di spinta continua lontano dal robot (a distanza 0)
RAGGIO_LIDAR_MIN, RAGGIO_LIDAR_MAX = 0, DIM_NODO * 10
RAGGIO_LIDAR_DEFAULT = DIM_NODO * 4  # raggio di rilevamento del "lidar" simulato del robot
RUMORE_LIDAR_PX = 8  # px massimi di rumore casuale sulla posizione percepita di una persona rilevata
ALPHA_ZONA_LIDAR = 35  # trasparenza del cerchio del lidar (0-255), piu' tenue della zona di sicurezza
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
COLORE_LIDAR = (0, 120, 220)

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

def algoritmo_a_star(griglia, inizio_pos_pixel, fine_pos_griglia, rumore_seed=None, celle_bloccate=None):
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
                nuovo_g = attuale.g + dist
                if vicino not in in_open or nuovo_g < in_open[vicino]:
                    vicino.g = nuovo_g
                    vicino.h = math.sqrt((vicino.cx - nodo_fine.cx)**2 + (vicino.cy - nodo_fine.cy)**2)
                    vicino.f = vicino.g + vicino.h
                    vicino.genitore = attuale
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
        "percorso": [],
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

def salva_mappa(griglia, path):
    muri = [[n.r, n.c] for riga in griglia for n in riga if n.tipo == "muro"]
    with open(path, "w") as f:
        json.dump(muri, f)

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

def main():
    pygame.init()
    fullscreen = False

    # Adatta la finestra iniziale allo schermo (max 90% di larghezza/altezza disponibili)
    info = pygame.display.Info()
    win_w = min(LARGHEZZA, int(info.current_w * 0.9))
    win_h = min(ALTEZZA, int(info.current_h * 0.9))
    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
    pygame.key.start_text_input()
    pygame.display.set_caption("Simulazione - Zoom: Rote. | Pan: Tasto DX | Muri: Tasto W | F11: Fullscreen | F5: Salva | F9: Carica | TAB: Pannello")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 32)

    chiedendo_password = False
    input_password = ""
    errore_timer = 0
    menu_salvataggio_aperto = False  # F5: prima si sceglie lo slot, poi si chiede la password
    menu_caricamento_aperto = False  # F9: si sceglie lo slot da caricare
    slot_da_salvare = None
    slot_vuoto_timer = 0
    modalita_rettangolo = False  # M: in attesa del primo/secondo click per riempire un'area rettangolare
    primo_punto_rettangolo = None
    
    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)

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

    # --- PERSONE ---
    numero_persone = NUMERO_PERSONE_DEFAULT
    persone = [crea_persona(griglia) for _ in range(numero_persone)]

    # --- CORRIDORI (persone al doppio della velocita' base) ---
    numero_corridori = NUMERO_CORRIDORI_DEFAULT
    corridori = [crea_corridore(griglia) for _ in range(numero_corridori)]

    # --- PERSONE FERME (ostacoli statici) ---
    numero_persone_ferme = NUMERO_PERSONE_FERME_DEFAULT
    persone_ferme = [crea_persona_ferma(griglia) for _ in range(numero_persone_ferme)]

    # --- GRUPPI ---
    numero_gruppi = NUMERO_GRUPPI_DEFAULT
    gruppi = [crea_gruppo(griglia) for _ in range(numero_gruppi)]

    # --- PANNELLO TECNICO ---
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
        panel_w, panel_h = 300, 475
        panel_rect = pygame.Rect(screen.get_width() - panel_w - 20, 20, panel_w, panel_h)
        slider_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 75, panel_w - 30, 8)
        numero_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 100, 70, 30)
        numero_corridori_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 135, 70, 30)
        numero_gruppi_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 170, 70, 30)
        numero_ferme_box_rect = pygame.Rect(panel_rect.x + 215, panel_rect.y + 205, 70, 30)
        slider_sicurezza_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 275, panel_w - 30, 8)
        slider_robot_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 350, panel_w - 30, 8)
        slider_lidar_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 425, panel_w - 30, 8)

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
                elif event.button == 4: zoom *= 1.1 # Zoom In
                elif event.button == 5: zoom /= 1.1 # Zoom Out
                elif event.button == 3: # Inizio Pan
                    trascinando = True
                    ultima_pos_mouse = event.pos
                elif event.button == 1: # Meta
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    if 0 <= r < Y_TOT and 0 <= c < X_TOT:
                        target_pos = (r, c)
                        percorso = []  # forza ricalcolo immediato verso il nuovo target

            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: trascinando = False
                if event.button == 1:
                    trascinando_slider = False
                    trascinando_raggio = False
                    trascinando_raggio_robot = False
                    trascinando_raggio_lidar = False

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
                        ultima_cella_w = (r, c)

                if event.key == pygame.K_r:
                    for riga in griglia:
                        for n in riga: n.tipo = "libero"
                    crea_bordi(griglia)
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

        if celle_rilevate != celle_rilevate_precedenti:
            percorso = []  # e' cambiato l'insieme di persone rilevate: il percorso pianificato potrebbe non essere piu' valido
        celle_rilevate_precedenti = celle_rilevate

        if target_pos and not robot_bloccato:
            if not percorso or tempo_di_ricalcolare_robot:
                percorso = algoritmo_a_star(griglia, (robot_x, robot_y), target_pos, celle_bloccate=celle_rilevate_rumorose)
            meta_nodo = griglia[target_pos[0]][target_pos[1]]
            robot_x, robot_y, arrivato, passo_muro = passo_movimento(robot_x, robot_y, percorso, VELOCITA_ROBOT * moltiplicatore_velocita, meta_nodo, griglia)
            if passo_muro:
                percorso = []  # il passo tagliava un muro: ricalcola subito invece di aspettare il prossimo intervallo
            if arrivato:
                target_pos = None

        # 3b. Movimento persone, corridori e membri dei gruppi (stessa logica per tutti, cambia solo la velocita')
        range_evitamento_quad = RANGE_EVITAMENTO_PERSONE ** 2
        range_evitamento_robot = raggio_robot + DIM_NODO // 3  # raggio robot (regolabile) + raggio persona: i corpi non si sovrappongono mai
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
        for p in tutte_mobili:
            if p["stato"] == "movimento":
                tempo_di_ricalcolare_questa_persona = (contatore_frame + p["offset_ricalcolo"]) % INTERVALLO_RICALCOLO_PERSONE_FRAME == 0
                if not p["percorso"] or tempo_di_ricalcolare_questa_persona:
                    p["percorso"] = algoritmo_a_star(griglia, (p["x"], p["y"]), p["target"], rumore_seed=p["rumore_seed"], celle_bloccate=celle_bloccate_comune)
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
        if percorso and len(percorso) > 1:
            punti = [t_s(n.cx, n.cy) for n in percorso]
            pygame.draw.lines(screen, ROSSO, False, punti, 2)

        rx, ry = t_s(robot_x, robot_y)

        if raggio_lidar > 0:
            raggio_lidar_px = max(1, int(raggio_lidar * zoom))
            lidar_surf = pygame.Surface((raggio_lidar_px * 2, raggio_lidar_px * 2), pygame.SRCALPHA)
            pygame.draw.circle(lidar_surf, (*COLORE_LIDAR, ALPHA_ZONA_LIDAR), (raggio_lidar_px, raggio_lidar_px), raggio_lidar_px)
            screen.blit(lidar_surf, (rx - raggio_lidar_px, ry - raggio_lidar_px))

        if raggio_sicurezza > 0:
            raggio_px = max(1, int(raggio_sicurezza * zoom))
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
            pygame.draw.circle(screen, colore_p, (px, py), int((DIM_NODO//3) * zoom))

        for pf in persone_ferme:
            fx, fy = t_s(pf["x"], pf["y"])
            colore_pf = colore_rilevamento(pf["x"], pf["y"], robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia)
            pygame.draw.circle(screen, colore_pf, (fx, fy), int((DIM_NODO//3) * zoom))

        for g in gruppi:
            if g["mobile"]:
                continue  # i membri dei gruppi mobili sono gia' disegnati sopra (sono inclusi in tutte_mobili)
            for m in g["membri"]:
                mpx, mpy = t_s(m["x"], m["y"])
                colore_m = colore_rilevamento(m["x"], m["y"], robot_x, robot_y, raggio_lidar, raggio_sicurezza, griglia)
                pygame.draw.circle(screen, colore_m, (mpx, mpy), int((DIM_NODO//3) * zoom))

        if modalita_rettangolo:
            msg = "Clicca il secondo punto (Esc annulla)" if primo_punto_rettangolo else "Clicca il primo punto (Esc annulla)"
            aiuto_txt = font.render(msg, True, ROSSO)
            screen.blit(aiuto_txt, (10, 10))

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

            pygame.draw.rect(screen, GRIGIO, slider_lidar_rect)
            rel_lidar = (raggio_lidar - RAGGIO_LIDAR_MIN) / (RAGGIO_LIDAR_MAX - RAGGIO_LIDAR_MIN)
            handle_lidar_x = slider_lidar_rect.x + int(rel_lidar * slider_lidar_rect.width)
            pygame.draw.circle(screen, COLORE_LIDAR, (handle_lidar_x, slider_lidar_rect.centery), 9)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
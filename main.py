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
INTERVALLO_RICALCOLO_ROBOT_FRAME = 60    # ricalcolo percorso robot: 60 FPS / questo valore = volte al secondo
INTERVALLO_RICALCOLO_PERSONE_FRAME = 10  # ricalcolo percorso persone: 60 FPS / questo valore = volte al secondo
FILE_MAPPA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mappa_salvata.json")
PASSWORD_SALVATAGGIO = "1258"
VEL_MULT_MIN, VEL_MULT_MAX = 0.2, 3.0
NUMERO_PERSONE_DEFAULT = 1
NUMERO_PERSONE_MAX = 50
ZOOM_MIN, ZOOM_MAX = 0.25, 4.0
MARGINE_PAN = 100  # spazio bianco massimo attorno alla mappa, in pixel
RAGGIO_SICUREZZA_MIN, RAGGIO_SICUREZZA_MAX = 0, DIM_NODO * 6
RAGGIO_SICUREZZA_DEFAULT = 20
ALPHA_ZONA_SICUREZZA = 70  # trasparenza del cerchio (0-255)
VARIAZIONE_VELOCITA_PERSONA = 0.15  # +-15% di velocita' individuale casuale
RUMORE_PERCORSO_PERSONA = 15  # quanto i percorsi delle persone si discostano dall'ottimo matematico (0 = disattivato)

# Colori
BIANCO, GRIGIO = (255, 255, 255), (210, 210, 210)
ROSSO, BLU = (255, 0, 0), (0, 100, 255)
ARANCIONE = (240, 140, 0)
NERO = (30, 30, 30)
GRIGIO_SCURO = (90, 90, 90)

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

def algoritmo_a_star(griglia, inizio_pos_pixel, fine_pos_griglia, rumore_seed=None):
    c_inizio = int(inizio_pos_pixel[0] // DIM_NODO)
    r_inizio = int(inizio_pos_pixel[1] // DIM_NODO)
    r_fine, c_fine = fine_pos_griglia
    r_inizio = max(0, min(Y_TOT - 1, r_inizio))
    c_inizio = max(0, min(X_TOT - 1, c_inizio))

    nodo_inizio = griglia[r_inizio][c_inizio]
    nodo_fine = griglia[r_fine][c_fine]
    
    for riga in griglia:
        for n in riga: n.reset_calcoli()

    contatore = 0  # tie-breaker per lo heap (i Nodo non sono confrontabili tra loro)
    open_heap = [(0, contatore, nodo_inizio)]
    in_open = {nodo_inizio: 0}  # nodo -> miglior g conosciuto finche' non viene chiuso
    closed_list = set()

    while open_heap:
        _, _, attuale = heapq.heappop(open_heap)
        if attuale in closed_list:
            continue
        closed_list.add(attuale)

        if attuale == nodo_fine:
            cammino = []
            while attuale:
                cammino.append(attuale)
                attuale = attuale.genitore
            return cammino[::-1]

        for m in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            r, c = attuale.r + m[0], attuale.c + m[1]
            if 0 <= r < Y_TOT and 0 <= c < X_TOT:
                vicino = griglia[r][c]
                if vicino in closed_list or vicino.tipo == "muro": continue
                dist = math.sqrt((vicino.cx - attuale.cx)**2 + (vicino.cy - attuale.cy)**2)
                if rumore_seed is not None:
                    # rumore deterministico per cella+persona: la stessa persona rifa' sempre
                    # la stessa scelta di "corsia" ricalcolando, invece di zigzagare a caso
                    perturbazione = math.sin(vicino.r * 12.9898 + vicino.c * 78.233 + rumore_seed) * RUMORE_PERCORSO_PERSONA
                    dist = max(1.0, dist + perturbazione)
                nuovo_g = attuale.g + dist
                if vicino not in in_open or nuovo_g < in_open[vicino]:
                    vicino.g = nuovo_g
                    vicino.h = math.sqrt((vicino.cx - nodo_fine.cx)**2 + (vicino.cy - nodo_fine.cy)**2)
                    vicino.f = vicino.g + vicino.h
                    vicino.genitore = attuale
                    in_open[vicino] = nuovo_g
                    contatore += 1
                    heapq.heappush(open_heap, (vicino.f, contatore, vicino))
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

def cella_libera_vicina(griglia, x, y, raggio):
    """Cella libera entro un certo raggio da (x, y), per generare percorsi brevi. Se non ne trova, ripiega su una casuale."""
    vicine = [n for riga in griglia for n in riga if n.tipo == "libero" and math.hypot(n.cx - x, n.cy - y) <= raggio]
    if vicine:
        return random.choice(vicine)
    return cella_libera_casuale(griglia)

def crea_persona(griglia):
    nodo_iniziale = cella_libera_casuale(griglia)
    nodo_target = cella_libera_casuale(griglia)
    return {
        "x": nodo_iniziale.cx, "y": nodo_iniziale.cy,
        "target": (nodo_target.r, nodo_target.c),
        "percorso": [],
        "stato": "movimento",
        "attesa_timer": 0,
        "rumore_seed": random.uniform(0, 1000),
        "fattore_velocita": random.uniform(1 - VARIAZIONE_VELOCITA_PERSONA, 1 + VARIAZIONE_VELOCITA_PERSONA),
    }

def sincronizza_persone(persone, numero, griglia):
    while len(persone) < numero:
        persone.append(crea_persona(griglia))
    while len(persone) > numero:
        persone.pop()

def passo_movimento(x, y, percorso, velocita, nodo_target):
    """Avanza di un passo lungo il percorso. Ritorna (nuovo_x, nuovo_y, arrivato_a_destinazione).
    Consuma i nodi intermedi man mano che vengono raggiunti, cosi' il percorso resta
    percorribile per piu' frame anche se non viene ricalcolato ad ogni frame."""
    if len(percorso) > 1:
        tx, ty = percorso[1].cx, percorso[1].cy
    elif len(percorso) == 1:
        tx, ty = nodo_target.cx, nodo_target.cy
    else:
        return x, y, False

    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx**2 + dy**2)
    if dist > velocita:
        return x + (dx / dist) * velocita, y + (dy / dist) * velocita, False

    if len(percorso) > 1:
        percorso.pop(0)
    return tx, ty, len(percorso) <= 1

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
    
    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)

    # --- VARIABILI ZOOM E PAN ---
    zoom = 1.0
    offset_x, offset_y = 0, 0
    trascinando = False
    ultima_pos_mouse = (0, 0)
    ultima_cella_w = None

    robot_x = 1 * DIM_NODO + DIM_NODO // 2
    robot_y = 1 * DIM_NODO + DIM_NODO // 2
    target_pos = None
    percorso = []

    # --- PERSONE ---
    numero_persone = NUMERO_PERSONE_DEFAULT
    persone = [crea_persona(griglia) for _ in range(numero_persone)]

    # --- PANNELLO TECNICO ---
    pannello_aperto = False
    moltiplicatore_velocita = 1.0
    trascinando_slider = False
    modificando_persone = False
    input_numero_persone = ""
    raggio_sicurezza = RAGGIO_SICUREZZA_DEFAULT
    trascinando_raggio = False

    contatore_frame = 0

    running = True
    while running:
        screen.fill(BIANCO)
        
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
                    pygame.draw.rect(screen, GRIGIO, rect, 1)

        # Layout pannello tecnico (ricalcolato ogni frame per seguire il resize)
        panel_w, panel_h = 300, 225
        panel_rect = pygame.Rect(screen.get_width() - panel_w - 20, 20, panel_w, panel_h)
        slider_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 75, panel_w - 30, 8)
        numero_box_rect = pygame.Rect(panel_rect.x + 110, panel_rect.y + 100, 60, 30)
        slider_sicurezza_rect = pygame.Rect(panel_rect.x + 15, panel_rect.y + 195, panel_w - 30, 8)

        # 2. Eventi
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

            if event.type == pygame.VIDEORESIZE and not fullscreen:
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1 and modificando_persone and not numero_box_rect.collidepoint(event.pos):
                    modificando_persone = False  # click fuori dal campo: annulla la modifica

                if event.button == 1 and pannello_aperto and panel_rect.collidepoint(event.pos):
                    if numero_box_rect.collidepoint(event.pos):
                        modificando_persone = True
                        input_numero_persone = str(numero_persone)
                    elif slider_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_slider = True
                        rel = (event.pos[0] - slider_rect.x) / slider_rect.width
                        moltiplicatore_velocita = VEL_MULT_MIN + max(0, min(1, rel)) * (VEL_MULT_MAX - VEL_MULT_MIN)
                    elif slider_sicurezza_rect.inflate(0, 20).collidepoint(event.pos):
                        trascinando_raggio = True
                        rel = (event.pos[0] - slider_sicurezza_rect.x) / slider_sicurezza_rect.width
                        raggio_sicurezza = RAGGIO_SICUREZZA_MIN + max(0, min(1, rel)) * (RAGGIO_SICUREZZA_MAX - RAGGIO_SICUREZZA_MIN)
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

            if event.type == pygame.MOUSEMOTION:
                if trascinando_slider:
                    rel = (event.pos[0] - slider_rect.x) / slider_rect.width
                    moltiplicatore_velocita = VEL_MULT_MIN + max(0, min(1, rel)) * (VEL_MULT_MAX - VEL_MULT_MIN)
                    continue
                if trascinando_raggio:
                    rel = (event.pos[0] - slider_sicurezza_rect.x) / slider_sicurezza_rect.width
                    raggio_sicurezza = RAGGIO_SICUREZZA_MIN + max(0, min(1, rel)) * (RAGGIO_SICUREZZA_MAX - RAGGIO_SICUREZZA_MIN)
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

            if event.type == pygame.KEYDOWN:
                if chiedendo_password:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_password == PASSWORD_SALVATAGGIO:
                            salva_mappa(griglia, FILE_MAPPA)
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

                if modificando_persone:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        if input_numero_persone.isdigit():
                            numero_persone = max(1, min(NUMERO_PERSONE_MAX, int(input_numero_persone)))
                            sincronizza_persone(persone, numero_persone, griglia)
                        modificando_persone = False
                    elif event.key == pygame.K_ESCAPE:
                        modificando_persone = False
                    elif event.key == pygame.K_BACKSPACE:
                        input_numero_persone = input_numero_persone[:-1]
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

                if event.key == pygame.K_F5:
                    chiedendo_password = True
                    input_password = ""
                if event.key == pygame.K_F9:
                    if carica_mappa(griglia, FILE_MAPPA):
                        target_pos, percorso = None, []
                        persone = [crea_persona(griglia) for _ in range(numero_persone)]

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
        tempo_di_ricalcolare_persone = contatore_frame % INTERVALLO_RICALCOLO_PERSONE_FRAME == 0

        robot_bloccato = raggio_sicurezza > 0 and any(
            math.hypot(robot_x - p["x"], robot_y - p["y"]) < raggio_sicurezza for p in persone
        )

        if target_pos and not robot_bloccato:
            if not percorso or tempo_di_ricalcolare_robot:
                percorso = algoritmo_a_star(griglia, (robot_x, robot_y), target_pos)
            meta_nodo = griglia[target_pos[0]][target_pos[1]]
            robot_x, robot_y, arrivato = passo_movimento(robot_x, robot_y, percorso, VELOCITA_ROBOT * moltiplicatore_velocita, meta_nodo)
            if arrivato:
                target_pos = None

        # 3b. Movimento persone
        for p in persone:
            if p["stato"] == "movimento":
                if not p["percorso"] or tempo_di_ricalcolare_persone:
                    p["percorso"] = algoritmo_a_star(griglia, (p["x"], p["y"]), p["target"], rumore_seed=p["rumore_seed"])
                nodo_target_p = griglia[p["target"][0]][p["target"][1]]
                velocita_p = VELOCITA_PERSONA * moltiplicatore_velocita * p["fattore_velocita"]
                p["x"], p["y"], arrivata = passo_movimento(p["x"], p["y"], p["percorso"], velocita_p, nodo_target_p)
                if arrivata:
                    p["stato"] = "attesa"
                    p["attesa_timer"] = random.randint(ATTESA_PERSONA_MIN_FRAME, ATTESA_PERSONA_MAX_FRAME)
            elif p["stato"] == "attesa":
                p["attesa_timer"] -= 1
                if p["attesa_timer"] <= 0:
                    if random.random() < PROBABILITA_PERCORSO_BREVE:
                        nuovo_nodo_target = cella_libera_vicina(griglia, p["x"], p["y"], RAGGIO_PERCORSO_BREVE)
                    else:
                        nuovo_nodo_target = cella_libera_casuale(griglia)
                    p["target"] = (nuovo_nodo_target.r, nuovo_nodo_target.c)
                    p["percorso"] = []  # forza ricalcolo immediato verso il nuovo target
                    p["stato"] = "movimento"

        contatore_frame += 1

        # 4. Rendering (Invariato)
        if percorso and len(percorso) > 1:
            punti = [t_s(n.cx, n.cy) for n in percorso]
            pygame.draw.lines(screen, ROSSO, False, punti, 2)

        rx, ry = t_s(robot_x, robot_y)

        if raggio_sicurezza > 0:
            raggio_px = max(1, int(raggio_sicurezza * zoom))
            zona_surf = pygame.Surface((raggio_px * 2, raggio_px * 2), pygame.SRCALPHA)
            pygame.draw.circle(zona_surf, (255, 0, 0, ALPHA_ZONA_SICUREZZA), (raggio_px, raggio_px), raggio_px)
            screen.blit(zona_surf, (rx - raggio_px, ry - raggio_px))

        pygame.draw.circle(screen, BLU, (rx, ry), int((DIM_NODO//3) * zoom))
        if target_pos:
            tx, ty = t_s(target_pos[1]*DIM_NODO + DIM_NODO//2, target_pos[0]*DIM_NODO + DIM_NODO//2)
            pygame.draw.circle(screen, ROSSO, (tx, ty), int((DIM_NODO//4) * zoom), 2)

        for p in persone:
            px, py = t_s(p["x"], p["y"])
            pygame.draw.circle(screen, ARANCIONE, (px, py), int((DIM_NODO//3) * zoom))

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

            sicurezza_txt = font.render(f"Zona sicurezza robot: {int(raggio_sicurezza)}px", True, NERO)
            screen.blit(sicurezza_txt, (panel_rect.x + 15, panel_rect.y + 165))

            pygame.draw.rect(screen, GRIGIO, slider_sicurezza_rect)
            rel_sic = (raggio_sicurezza - RAGGIO_SICUREZZA_MIN) / (RAGGIO_SICUREZZA_MAX - RAGGIO_SICUREZZA_MIN)
            handle_sic_x = slider_sicurezza_rect.x + int(rel_sic * slider_sicurezza_rect.width)
            pygame.draw.circle(screen, ROSSO, (handle_sic_x, slider_sicurezza_rect.centery), 9)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
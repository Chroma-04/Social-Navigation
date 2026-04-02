import pygame
import math

# --- CONFIGURAZIONE ---
DIM_NODO = 30
LARGHEZZA, ALTEZZA = X_TOT * DIM_NODO, Y_TOT * DIM_NODO
VELOCITA_ROBOT = 1.5 

# Colori
BIANCO, GRIGIO = (255, 255, 255), (210, 210, 210)
ROSSO, BLU = (255, 0, 0), (0, 100, 255)
NERO = (30, 30, 30)

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

def algoritmo_a_star(griglia, inizio_pos_pixel, fine_pos_griglia):
    c_inizio = int(inizio_pos_pixel[0] // DIM_NODO)
    r_inizio = int(inizio_pos_pixel[1] // DIM_NODO)
    r_fine, c_fine = fine_pos_griglia
    r_inizio = max(0, min(Y_TOT - 1, r_inizio))
    c_inizio = max(0, min(X_TOT - 1, c_inizio))

    nodo_inizio = griglia[r_inizio][c_inizio]
    nodo_fine = griglia[r_fine][c_fine]
    
    for riga in griglia: 
        for n in riga: n.reset_calcoli()

    open_list = [nodo_inizio]
    closed_list = set()

    while open_list:
        attuale = min(open_list, key=lambda n: n.f)
        open_list.remove(attuale)
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
                nuovo_g = attuale.g + dist
                if vicino not in open_list or nuovo_g < vicino.g:
                    vicino.g = nuovo_g
                    vicino.h = math.sqrt((vicino.cx - nodo_fine.cx)**2 + (vicino.cy - nodo_fine.cy)**2)
                    vicino.f = vicino.g + vicino.h
                    vicino.genitore = attuale
                    if vicino not in open_list: open_list.append(vicino)
    return []

def crea_bordi(griglia):
    for r in range(Y_TOT):
        for c in range(X_TOT):
            if r == 0 or r == Y_TOT - 1 or c == 0 or c == X_TOT - 1:
                griglia[r][c].tipo = "muro"

def main():
    pygame.init()
    screen = pygame.display.set_mode((LARGHEZZA, ALTEZZA))
    pygame.display.set_caption("Simulazione - Zoom: Rote. | Pan: Tasto DX | Muri: Tasto W")
    clock = pygame.time.Clock()
    
    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)

    # --- VARIABILI ZOOM E PAN ---
    zoom = 1.0
    offset_x, offset_y = 0, 0
    trascinando = False
    ultima_pos_mouse = (0, 0)

    robot_x = 1 * DIM_NODO + DIM_NODO // 2
    robot_y = 1 * DIM_NODO + DIM_NODO // 2
    target_pos = None
    percorso = []

    running = True
    while running:
        screen.fill(BIANCO)
        
        # Funzioni di conversione coordinate
        def t_s(x, y): return int(x * zoom + offset_x), int(y * zoom + offset_y)
        def t_m(sx, sy): return (sx - offset_x) / zoom, (sy - offset_y) / zoom

        # 1. Disegno Scacchiera
        for riga in griglia:
            for n in riga:
                sx, sy = t_s(n.x, n.y)
                dim_z = int(DIM_NODO * zoom)
                if n.tipo == "muro":
                    pygame.draw.rect(screen, NERO, (sx, sy, dim_z, dim_z))
                pygame.draw.rect(screen, GRIGIO, (sx, sy, dim_z, dim_z), 1)

        # 2. Eventi
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4: zoom *= 1.1 # Zoom In
                elif event.button == 5: zoom /= 1.1 # Zoom Out
                elif event.button == 3: # Inizio Pan
                    trascinando = True
                    ultima_pos_mouse = event.pos
                elif event.button == 1: # Meta
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    if 0 <= r < Y_TOT and 0 <= c < X_TOT:
                        target_pos = (r, c)

            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: trascinando = False

            if event.type == pygame.MOUSEMOTION:
                # Gestione Panning
                if trascinando:
                    dx, dy = event.pos[0] - ultima_pos_mouse[0], event.pos[1] - ultima_pos_mouse[1]
                    offset_x += dx
                    offset_y += dy
                    ultima_pos_mouse = event.pos
                
                # --- AGGIUNTA: POSIZIONAMENTO MURI CON TASTO 'W' ---
                keys = pygame.key.get_pressed()
                if keys[pygame.K_w]:
                    mx, my = t_m(event.pos[0], event.pos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    # Evitiamo di modificare i bordi esterni e restiamo nei limiti
                    if 0 < r < Y_TOT - 1 and 0 < c < X_TOT - 1:
                        griglia[r][c].tipo = "muro"

            if event.type == pygame.KEYDOWN:
                # Toggle muro singolo con pressione singola di W (opzionale)
                if event.key == pygame.K_w:
                    mpos = pygame.mouse.get_pos()
                    mx, my = t_m(mpos[0], mpos[1])
                    r, c = int(my // DIM_NODO), int(mx // DIM_NODO)
                    if 0 < r < Y_TOT - 1 and 0 < c < X_TOT - 1:
                        griglia[r][c].tipo = "muro" if griglia[r][c].tipo == "libero" else "libero"

                if event.key == pygame.K_r:
                    for riga in griglia:
                        for n in riga: n.tipo = "libero"
                    crea_bordi(griglia)
                    robot_x, robot_y = 1.5 * DIM_NODO, 1.5 * DIM_NODO
                    target_pos, percorso = None, []
                if event.key == pygame.K_s: target_pos, percorso = None, []

        # 3. Movimento (Invariato)
        if target_pos:
            percorso = algoritmo_a_star(griglia, (robot_x, robot_y), target_pos)
            if len(percorso) > 1:
                prossimo = percorso[1]
                tx, ty = prossimo.cx, prossimo.cy
                dx, dy = tx - robot_x, ty - robot_y
                dist = math.sqrt(dx**2 + dy**2)
                if dist > VELOCITA_ROBOT:
                    robot_x += (dx / dist) * VELOCITA_ROBOT
                    robot_y += (dy / dist) * VELOCITA_ROBOT
                else: robot_x, robot_y = tx, ty
            elif len(percorso) == 1:
                meta_nodo = griglia[target_pos[0]][target_pos[1]]
                dx, dy = meta_nodo.cx - robot_x, meta_nodo.cy - robot_y
                dist = math.sqrt(dx**2 + dy**2)
                if dist > VELOCITA_ROBOT:
                    robot_x += (dx / dist) * VELOCITA_ROBOT
                    robot_y += (dy / dist) * VELOCITA_ROBOT
                else:
                    robot_x, robot_y = meta_nodo.cx, meta_nodo.cy
                    target_pos = None

        # 4. Rendering (Invariato)
        if percorso and len(percorso) > 1:
            punti = [t_s(n.cx, n.cy) for n in percorso]
            pygame.draw.lines(screen, ROSSO, False, punti, 2)

        rx, ry = t_s(robot_x, robot_y)
        pygame.draw.circle(screen, BLU, (rx, ry), int((DIM_NODO//3) * zoom))
        if target_pos:
            tx, ty = t_s(target_pos[1]*DIM_NODO + DIM_NODO//2, target_pos[0]*DIM_NODO + DIM_NODO//2)
            pygame.draw.circle(screen, ROSSO, (tx, ty), int((DIM_NODO//4) * zoom), 2)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
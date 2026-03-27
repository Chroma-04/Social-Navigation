import pygame
import math

# --- CONFIGURAZIONE ---
X_TOT, Y_TOT = 30, 20
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
        # Definiamo il centro una volta per tutte
        self.cx = self.x + DIM_NODO // 2
        self.cy = self.y + DIM_NODO // 2
        self.g = self.h = self.f = 0
        self.genitore = None
        self.tipo = "libero"

    def reset_calcoli(self):
        self.g = self.h = self.f = 0
        self.genitore = None

def algoritmo_a_star(griglia, inizio_pos_pixel, fine_pos_griglia):
    # Troviamo la cella basandoci su dove si trova il centro del robot
    c_inizio = int(inizio_pos_pixel[0] // DIM_NODO)
    r_inizio = int(inizio_pos_pixel[1] // DIM_NODO)
    
    r_fine, c_fine = fine_pos_griglia
    
    # Check bordi
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
                
                # Calcolo euristica e costo basato sui centri
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
    pygame.display.set_caption("Robot al CENTRO PERFETTO")
    clock = pygame.time.Clock()
    
    griglia = [[Nodo(r, c) for c in range(X_TOT)] for r in range(Y_TOT)]
    crea_bordi(griglia)

    # Inizializzazione al centro della cella (1,1)
    robot_x = 1 * DIM_NODO + DIM_NODO // 2
    robot_y = 1 * DIM_NODO + DIM_NODO // 2
    target_pos = None
    percorso = []

    running = True
    while running:
        screen.fill(BIANCO)
        
        for riga in griglia:
            for n in riga:
                if n.tipo == "muro":
                    pygame.draw.rect(screen, NERO, (n.x, n.y, DIM_NODO, DIM_NODO))
                pygame.draw.rect(screen, GRIGIO, (n.x, n.y, DIM_NODO, DIM_NODO), 1)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                r, c = my // DIM_NODO, mx // DIM_NODO
                if 0 < r < Y_TOT-1 and 0 < c < X_TOT-1:
                    if event.button == 1: target_pos = (r, c)
                    elif event.button == 3:
                        griglia[r][c].tipo = "muro" if griglia[r][c].tipo == "libero" else "libero"

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    for riga in griglia:
                        for n in riga: n.tipo = "libero"
                    crea_bordi(griglia)
                    robot_x, robot_y = 1.5 * DIM_NODO, 1.5 * DIM_NODO
                    target_pos, percorso = None, []
                if event.key == pygame.K_s:
                    target_pos, percorso = None, []

        # --- MOVIMENTO CORRETTO ---
        if target_pos:
            percorso = algoritmo_a_star(griglia, (robot_x, robot_y), target_pos)
            
            if len(percorso) > 1:
                # Puntiamo al centro della PROSSIMA cella
                prossimo = percorso[1]
                tx, ty = prossimo.cx, prossimo.cy
                
                dx, dy = tx - robot_x, ty - robot_y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > VELOCITA_ROBOT:
                    robot_x += (dx / dist) * VELOCITA_ROBOT
                    robot_y += (dy / dist) * VELOCITA_ROBOT
                else:
                    # Raggiunto il centro della cella intermedia
                    robot_x, robot_y = tx, ty
            elif len(percorso) == 1:
                # Siamo già nella cella meta, puntiamo al suo centro esatto
                meta_nodo = griglia[target_pos[0]][target_pos[1]]
                dx, dy = meta_nodo.cx - robot_x, meta_nodo.cy - robot_y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > VELOCITA_ROBOT:
                    robot_x += (dx / dist) * VELOCITA_ROBOT
                    robot_y += (dy / dist) * VELOCITA_ROBOT
                else:
                    robot_x, robot_y = meta_nodo.cx, meta_nodo.cy
                    target_pos = None # STOP DEFINITIVO

        # Disegno Linea Rossa tra i centri
        if percorso and len(percorso) > 1:
            punti = [(n.cx, n.cy) for n in percorso]
            pygame.draw.lines(screen, ROSSO, False, punti, 2)

        # Disegno Robot (Blu) e Meta (Rosso)
        pygame.draw.circle(screen, BLU, (int(robot_x), int(robot_y)), DIM_NODO//3)
        if target_pos:
            mx = target_pos[1] * DIM_NODO + DIM_NODO // 2
            my = target_pos[0] * DIM_NODO + DIM_NODO // 2
            pygame.draw.circle(screen, ROSSO, (mx, my), DIM_NODO//4, 2)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
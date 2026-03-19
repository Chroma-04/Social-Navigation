import pygame
import math

# --- CONFIGURAZIONE ---
DIM_NODO = 40 
LARGHEZZA, ALTEZZA = 800, 600
NUM_COLONNE, NUM_RIGHE = LARGHEZZA // DIM_NODO, ALTEZZA // DIM_NODO

# Colori
BIANCO, GRIGIO = (255, 255, 255), (210, 210, 210)
ROSSO, BLU = (255, 0, 0), (0, 100, 255)
NERO, GIALLO = (30, 30, 30), (255, 220, 0) # Muro Statico, Ostacolo Imprevisto

class Nodo:
    def __init__(self, r, c):
        self.r, self.c = r, c
        self.x, self.y = c * DIM_NODO, r * DIM_NODO
        self.g = self.h = self.f = 0
        self.genitore = None
        self.tipo = "libero" # "libero", "muro", "imprevisto"

    def reset_calcoli(self):
        self.g = self.h = self.f = 0
        self.genitore = None

def algoritmo_a_star(griglia, inizio_pos, fine_pos):
    # Se meta o partenza sono ostacoli, nessun percorso
    if griglia[fine_pos[0]][fine_pos[1]].tipo != "libero" or \
       griglia[inizio_pos[0]][inizio_pos[1]].tipo != "libero":
        return []

    nodo_inizio = griglia[inizio_pos[0]][inizio_pos[1]]
    nodo_fine = griglia[fine_pos[0]][fine_pos[1]]
    
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
            if 0 <= r < NUM_RIGHE and 0 <= c < NUM_COLONNE:
                vicino = griglia[r][c]
                # Entrambi i tipi di ostacoli bloccano il percorso
                if vicino in closed_list or vicino.tipo != "libero": continue
                
                # CALCOLO G PRECISO (Distanza Geometrica Reale)
                # Questo risolve il problema della "collina", rendendo la traiettoria dritta
                dist_geometrica = math.sqrt((vicino.x - attuale.x)**2 + (vicino.y - attuale.y)**2)
                nuovo_g = attuale.g + dist_geometrica
                
                if vicino not in open_list or nuovo_g < vicino.g:
                    vicino.g = nuovo_g
                    # Euristica h: Distanza Geometrica Reale (Euclidea)
                    vicino.h = math.sqrt((vicino.x - nodo_fine.x)**2 + (vicino.y - nodo_fine.y)**2)
                    vicino.f = vicino.g + vicino.h
                    vicino.genitore = attuale
                    if vicino not in open_list: open_list.append(vicino)
    return []

def main():
    pygame.init()
    screen = pygame.display.set_mode((LARGHEZZA, ALTEZZA))
    pygame.display.set_caption("A* Interattivo: SX=Meta, DX=Cicla Ostacoli, R=Reset")
    
    griglia = [[Nodo(r, c) for c in range(NUM_COLONNE)] for r in range(NUM_RIGHE)]
    robot_pos = (NUM_RIGHE - 2, 1) # Basso a SX
    target_pos = None
    percorso = []

    running = True
    while running:
        screen.fill(BIANCO)
        
        # 1. Disegno Nodi, Muri e Ostacoli
        for riga in griglia:
            for n in riga:
                if n.tipo == "muro":
                    pygame.draw.rect(screen, NERO, (n.x, n.y, DIM_NODO, DIM_NODO))
                elif n.tipo == "imprevisto":
                    pygame.draw.rect(screen, GIALLO, (n.x, n.y, DIM_NODO, DIM_NODO))
                pygame.draw.rect(screen, GRIGIO, (n.x, n.y, DIM_NODO, DIM_NODO), 1)

        # 2. Gestione Eventi e Mouse
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Click Mouse
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                r, c = my // DIM_NODO, mx // DIM_NODO
                
                # Click Sinistro (1) -> Imposta Meta
                if event.button == 1:
                    if 0 <= r < NUM_RIGHE and 0 <= c < NUM_COLONNE:
                        target_pos = (r, c)
                
                # Click Destro (3) -> Cicla Ostacoli (Libero -> Nero -> Giallo)
                elif event.button == 3:
                    if 0 <= r < NUM_RIGHE and 0 <= c < NUM_COLONNE:
                        if griglia[r][c].tipo == "libero":
                            griglia[r][c].tipo = "muro" # Guadagno Infinito, Statico
                        elif griglia[r][c].tipo == "muro":
                            griglia[r][c].tipo = "imprevisto" # Guadagno Infinito, Imprevisto
                        else:
                            griglia[r][c].tipo = "libero"

            # Tasto 'R' sulla tastiera -> Reset Mappa
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    for riga in griglia:
                        for n in riga:
                            n.tipo = "libero"
                    percorso, target_pos = [], None

        # 3. Ricalcolo dinamico percorso se c'è un target
        if target_pos:
            percorso = algoritmo_a_star(griglia, robot_pos, target_pos)

        # 4. Disegno Percorso (Linea Rossa)
        if percorso:
            punti = [(n.c * DIM_NODO + DIM_NODO//2, n.r * DIM_NODO + DIM_NODO//2) for n in percorso]
            if len(punti) > 1:
                pygame.draw.lines(screen, ROSSO, False, punti, 4)
        
        # 5. Disegno Robot (Blu) e Meta (Rosso)
        pygame.draw.circle(screen, BLU, (robot_pos[1]*DIM_NODO + 20, robot_pos[0]*DIM_NODO + 20), 15)
        if target_pos:
            pygame.draw.circle(screen, ROSSO, (target_pos[1]*DIM_NODO + 20, target_pos[0]*DIM_NODO + 20), 10, 2)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
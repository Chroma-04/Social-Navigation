import pygame
import math

# --- CONFIGURAZIONE MODIFICABILE ---
X_TOT = 30  # Numero di colonne (scacchi in orizzontale)
Y_TOT = 20  # Numero di righe (scacchi in verticale)
DIM_NODO = 30 # Dimensione di ogni singolo scacco 

# Il software calcola da solo la finestra in base a X_TOT e Y_TOT
LARGHEZZA, ALTEZZA = X_TOT * DIM_NODO, Y_TOT * DIM_NODO
NUM_COLONNE, NUM_RIGHE = X_TOT, Y_TOT

# Colori
BIANCO, GRIGIO = (255, 255, 255), (210, 210, 210)
ROSSO, BLU = (255, 0, 0), (0, 100, 255)
NERO, GIALLO = (30, 30, 30), (255, 220, 0)

class Nodo:
    def __init__(self, r, c):
        self.r, self.c = r, c
        self.x, self.y = c * DIM_NODO, r * DIM_NODO
        self.g = self.h = self.f = 0
        self.genitore = None
        self.tipo = "libero"

    def reset_calcoli(self):
        self.g = self.h = self.f = 0
        self.genitore = None

def algoritmo_a_star(griglia, inizio_pos, fine_pos):
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
                if vicino in closed_list or vicino.tipo != "libero": continue
                
                dist_geometrica = math.sqrt((vicino.x - attuale.x)**2 + (vicino.y - attuale.y)**2)
                nuovo_g = attuale.g + dist_geometrica
                
                if vicino not in open_list or nuovo_g < vicino.g:
                    vicino.g = nuovo_g
                    vicino.h = math.sqrt((vicino.x - nodo_fine.x)**2 + (vicino.y - nodo_fine.y)**2)
                    vicino.f = vicino.g + vicino.h
                    vicino.genitore = attuale
                    if vicino not in open_list: open_list.append(vicino)
    return []

def crea_bordi(griglia):
    """Crea i muri neri perimetrali"""
    for r in range(NUM_RIGHE):
        for c in range(NUM_COLONNE):
            # Se è la prima o l'ultima riga, o la prima o l'ultima colonna
            if r == 0 or r == NUM_RIGHE - 1 or c == 0 or c == NUM_COLONNE - 1:
                griglia[r][c].tipo = "muro"

def main():
    pygame.init()
    screen = pygame.display.set_mode((LARGHEZZA, ALTEZZA))
    pygame.display.set_caption(f"Simulatore {X_TOT}x{Y_TOT} - SX: Meta, DX: Ostacoli, R: Reset")
    
    griglia = [[Nodo(r, c) for c in range(NUM_COLONNE)] for r in range(NUM_RIGHE)]
    
    # Crea i bordi neri all'avvio
    crea_bordi(griglia)

    # Posiziona il robot all'interno dei bordi (riga 1, colonna 1)
    robot_pos = (1, 1) 
    target_pos = None
    percorso = []

    running = True
    while running:
        screen.fill(BIANCO)
        
        for riga in griglia:
            for n in riga:
                if n.tipo == "muro":
                    pygame.draw.rect(screen, NERO, (n.x, n.y, DIM_NODO, DIM_NODO))
                elif n.tipo == "imprevisto":
                    pygame.draw.rect(screen, GIALLO, (n.x, n.y, DIM_NODO, DIM_NODO))
                pygame.draw.rect(screen, GRIGIO, (n.x, n.y, DIM_NODO, DIM_NODO), 1)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                r, c = my // DIM_NODO, mx // DIM_NODO
                
                if 0 <= r < NUM_RIGHE and 0 <= c < NUM_COLONNE:
                    # Impedisce di sovrascrivere i bordi esterni
                    if r == 0 or r == NUM_RIGHE - 1 or c == 0 or c == NUM_COLONNE - 1:
                        continue

                    if event.button == 1:
                        target_pos = (r, c)
                    elif event.button == 3:
                        if griglia[r][c].tipo == "libero":
                            griglia[r][c].tipo = "muro"
                        elif griglia[r][c].tipo == "muro":
                            griglia[r][c].tipo = "imprevisto"
                        else:
                            griglia[r][c].tipo = "libero"

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    for riga in griglia:
                        for n in riga:
                            n.tipo = "libero"
                    crea_bordi(griglia) # Ricrea i bordi dopo il reset
                    percorso, target_pos = [], None

        if target_pos:
            percorso = algoritmo_a_star(griglia, robot_pos, target_pos)

        if percorso:
            punti = [(n.c * DIM_NODO + DIM_NODO//2, n.r * DIM_NODO + DIM_NODO//2) for n in percorso]
            if len(punti) > 1:
                pygame.draw.lines(screen, ROSSO, False, punti, 3)
        
        pygame.draw.circle(screen, BLU, (robot_pos[1]*DIM_NODO + DIM_NODO//2, robot_pos[0]*DIM_NODO + DIM_NODO//2), DIM_NODO//3)
        if target_pos:
            pygame.draw.circle(screen, ROSSO, (target_pos[1]*DIM_NODO + DIM_NODO//2, target_pos[0]*DIM_NODO + DIM_NODO//2), DIM_NODO//4, 2)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
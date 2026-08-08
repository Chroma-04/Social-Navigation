"""Utility di supporto: renderizza in PNG le mappe disponibili per il training (sia quelle procedurali in
training_scenari.py, sia in futuro quelle disegnate a mano e salvate in mappe_training/), cosi' si possono
rivedere senza dover aprire il simulatore. Uso: py visualizza_mappe.py [cartella_output]
"""
import os
import sys
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame
import main as sim
import training_scenari as ts

MURO = (60, 60, 60)
LIBERO = (235, 235, 235)
BORDO_CELLA = (215, 215, 215)
COLORE_BADGE = (30, 110, 220)
COLORE_NUMERO = (255, 255, 255)


def disegna_mappa(griglia, scala=4):
    superficie = pygame.Surface((sim.X_TOT * scala, sim.Y_TOT * scala))
    superficie.fill(LIBERO)
    for riga in griglia:
        for n in riga:
            if n.tipo == "muro":
                rect = (n.c * scala, n.r * scala, scala, scala)
                pygame.draw.rect(superficie, MURO, rect)
    return superficie


def disegna_numero(superficie, numero, font):
    """Cerchietto numerato in alto a sinistra, cosi' ogni mappa si puo' richiamare per numero invece che
    per nome (piu' comodo quando se ne confrontano tante in una volta)."""
    centro = (24, 24)
    pygame.draw.circle(superficie, COLORE_BADGE, centro, 20)
    testo = font.render(str(numero), True, COLORE_NUMERO)
    superficie.blit(testo, testo.get_rect(center=centro))


def salva_anteprima(numero, nome_mappa, builder, cartella_output, font):
    griglia = [[sim.Nodo(r, c) for c in range(sim.X_TOT)] for r in range(sim.Y_TOT)]
    sim.crea_bordi(griglia)
    builder(griglia)
    superficie = disegna_mappa(griglia)
    disegna_numero(superficie, numero, font)
    path = os.path.join(cartella_output, f"mappa_{numero:02d}_{nome_mappa}.png")
    pygame.image.save(superficie, path)
    return path


if __name__ == "__main__":
    pygame.init()
    cartella_output = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "anteprime_mappe")
    os.makedirs(cartella_output, exist_ok=True)
    for vecchio in os.listdir(cartella_output):  # rimuove le anteprime della run precedente (nomi/numeri possono cambiare)
        if vecchio.endswith(".png"):
            os.remove(os.path.join(cartella_output, vecchio))
    font = pygame.font.SysFont(None, 28, bold=True)
    print(f"{'#':>3}  nome mappa")
    for numero, (nome_mappa, builder) in enumerate(ts.MAPPE.items(), start=1):
        path = salva_anteprima(numero, nome_mappa, builder, cartella_output, font)
        print(f"{numero:>3}  {nome_mappa}  ->  {path}")

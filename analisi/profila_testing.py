"""Dove va il tempo di calcolo di una corsa di validazione.

Misura da cui e' emerso che il collo di bottiglia e' il pathfinding, non l'inferenza del modello:
sullo scenario denso il 97% del tempo sta dentro algoritmo_a_star, e di quello il 96% dentro
scorciatoia_libera (il controllo di visibilita' del Theta*, chiamato ~21 milioni di volte in 300 frame).

E' il numero che giustifica due scelte del progetto: la griglia a DIM_NODO=30px (una griglia quattro
volte piu' fine moltiplicherebbe per 16 le celle da esplorare, rendendo la validazione impraticabile) e
l'uso dell'accelerazione del passo temporale al posto di ottimizzazioni del pianificatore, che e' codice
condiviso con il robot vero e non va toccato per far girare i test piu' in fretta.

Uso:
    py analisi/profila_testing.py [popolazione] [condizione]
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import cProfile
import pstats
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # la radice del progetto
import testing as t

POPOLAZIONE = int(sys.argv[1]) if len(sys.argv) > 1 else 300
CONDIZIONE = sys.argv[2] if len(sys.argv) > 2 else "completo"
MAPPA = "training_mappa_01"
SEED = 12345

if __name__ == "__main__":
    t._inizializza_worker()
    popolazione = t.popolazione_da_totale(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())

    print(f"Profilo: {MAPPA}, {POPOLAZIONE} persone, condizione '{CONDIZIONE}', "
          f"accelerazione {t.ACCELERAZIONE}x\n")

    inizio = time.time()
    profiler = cProfile.Profile()
    profiler.enable()
    arrivato, frame, fermo = t.esegui_corsa(MAPPA, CONDIZIONE, popolazione, SEED)
    profiler.disable()
    durata = time.time() - inizio

    print(f"Arrivato: {arrivato} in {frame} frame ({t._secondi(frame):.1f}s equivalenti), "
          f"{fermo} frame fermo per sicurezza")
    print(f"Costo di calcolo reale: {durata:.1f}s ({durata / frame * 1000:.1f}ms per frame simulato)\n")

    stats = pstats.Stats(profiler)
    stats.sort_stats("cumulative")
    stats.print_stats(15)

    print("\n\n=== Ordinato per tempo proprio (tottime), non cumulativo ===")
    stats.sort_stats("tottime")
    stats.print_stats(15)

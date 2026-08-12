"""Quanto costa una singola ricerca di percorso al variare della MAPPA.

Domanda a cui risponde: il calo di prestazioni che si osserva caricando una mappa viene
davvero dall'esplosione dei nodi esplorati dall'A*/Theta* (euristica euclidea resa
fuorviante dai muri), o la causa e' altrove?

Cronometra la stessa ricerca - angolo alto-sinistra -> angolo basso-destra, nessuna
persona in scena, nessuna mappa di costo - su tre ambienti di complessita' crescente e
alle due risoluzioni della griglia del robot. Riporta anche quante volte viene invocato
il controllo di visibilita' del Theta* (scorciatoia_libera), che il profiling indica come
l'operazione dominante: e' quello il moltiplicatore che trasforma "piu' nodi esplorati"
in "molto piu' tempo".

Nessuna persona e nessun costo extra: si isola l'effetto della sola struttura dei muri.

Uso:
    py analisi/profila_astar_mappe.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # la radice del progetto
import main as sim
import training_scenari as ts

RIPETIZIONI = 5
MAPPE = ["(vuota)", "training_mappa_01", "porta_stretta", "incrocio"]
FATTORI_GRIGLIA = [1, 2]  # 1 = celle da 30px (folla), 2 = griglia fine del robot (15px)

# contatore delle chiamate al controllo di visibilita': algoritmo_a_star risolve
# scorciatoia_libera come globale del modulo, quindi sostituirla qui viene visto
_conteggio = {"scorciatoie": 0}
_scorciatoia_originale = sim.scorciatoia_libera


def _scorciatoia_contata(*argomenti, **parametri):
    _conteggio["scorciatoie"] += 1
    return _scorciatoia_originale(*argomenti, **parametri)


sim.scorciatoia_libera = _scorciatoia_contata


def _cella_vicina_angolo(griglia, r_angolo, c_angolo):
    """Cella libera piu' vicina all'angolo indicato: gli angoli veri sono muro."""
    libere = [n for riga in griglia for n in riga if n.tipo == "libero"]
    return min(libere, key=lambda n: (n.r - r_angolo) ** 2 + (n.c - c_angolo) ** 2)


def _costruisci(nome_mappa):
    griglia = [[sim.Nodo(r, c) for c in range(sim.X_TOT)] for r in range(sim.Y_TOT)]
    sim.crea_bordi(griglia)
    if nome_mappa != "(vuota)":
        costruttore = ts.MAPPE.get(nome_mappa) or ts.MAPPE_TEST.get(nome_mappa)
        if costruttore is None:
            raise KeyError(f"Mappa '{nome_mappa}' non trovata")
        costruttore(griglia)
    return griglia


def _lunghezza(percorso):
    import math
    return sum(math.hypot(b.cx - a.cx, b.cy - a.cy) for a, b in zip(percorso, percorso[1:]))


def misura(nome_mappa, fattore):
    griglia = _costruisci(nome_mappa)
    celle_libere = sum(1 for riga in griglia for n in riga if n.tipo == "libero")

    partenza = _cella_vicina_angolo(griglia, 0, 0)
    arrivo = _cella_vicina_angolo(griglia, sim.Y_TOT - 1, sim.X_TOT - 1)

    griglia_robot = sim.crea_griglia_robot(griglia, fattore)
    arrivo_robot = (arrivo.r * fattore + fattore // 2, arrivo.c * fattore + fattore // 2)
    inizio_px = (partenza.cx, partenza.cy)

    _conteggio["scorciatoie"] = 0
    t0 = time.perf_counter()
    for _ in range(RIPETIZIONI):
        percorso = sim.algoritmo_a_star(griglia_robot, inizio_px, arrivo_robot)
    durata = (time.perf_counter() - t0) / RIPETIZIONI
    scorciatoie = _conteggio["scorciatoie"] / RIPETIZIONI

    return {
        "celle_libere": celle_libere,
        "ms": durata * 1000,
        "scorciatoie": scorciatoie,
        "nodi_percorso": len(percorso),
        "lunghezza_m": _lunghezza(percorso) / 60.0 if percorso else 0.0,  # 60 px = 1 m
    }


if __name__ == "__main__":
    print(f"Una ricerca angolo->angolo, nessuna persona, nessuna mappa di costo.")
    print(f"Media su {RIPETIZIONI} ripetizioni.\n")

    intestazione = (f"{'mappa':>20}{'cella':>8}{'libere':>9}{'tempo':>11}"
                    f"{'controlli vis.':>16}{'nodi':>7}{'lungh.':>10}")
    print(intestazione)
    print("-" * len(intestazione))

    riferimento = {}
    for nome in MAPPE:
        for fattore in FATTORI_GRIGLIA:
            try:
                r = misura(nome, fattore)
            except KeyError as e:
                print(f"{nome:>20}   {e}")
                break
            lato = sim.DIM_NODO // fattore
            print(f"{nome:>20}{lato:>6}px{r['celle_libere']:>9}{r['ms']:>10.1f}ms"
                  f"{r['scorciatoie']:>16,.0f}{r['nodi_percorso']:>7}{r['lunghezza_m']:>9.1f}m")
            if nome == "(vuota)":
                riferimento[fattore] = r
        print()

    print("=== Rapporto rispetto alla mappa vuota, a parita' di risoluzione ===")
    print(f"{'mappa':>20}{'cella':>8}{'x tempo':>10}{'x controlli':>14}")
    for nome in MAPPE:
        if nome == "(vuota)":
            continue
        for fattore in FATTORI_GRIGLIA:
            try:
                r = misura(nome, fattore)
            except KeyError:
                break
            base = riferimento.get(fattore)
            if not base:
                continue
            lato = sim.DIM_NODO // fattore
            rap_t = r["ms"] / base["ms"] if base["ms"] > 0 else float("nan")
            rap_s = r["scorciatoie"] / base["scorciatoie"] if base["scorciatoie"] > 0 else float("nan")
            print(f"{nome:>20}{lato:>6}px{rap_t:>9.1f}x{rap_s:>13.1f}x")

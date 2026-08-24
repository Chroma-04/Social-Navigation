# -*- coding: utf-8 -*-
"""La differenza fra A* e Theta*, disegnata invece che raccontata.

I due percorsi non sono ridisegnati a mano: si eseguono davvero le due ricerche
sulla stessa griglia, cosi' la figura non puo' mentire sulla forma del risultato.
"""
import heapq
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

TESTO, TENUE, ACCENTO, FILETTO = "#262626", "#828282", "#9B1E2D", "#D7D7D7"
BLU = "#2C5F8A"

LARGO, ALTO = 16, 10
PARTENZA, ARRIVO = (1, 1), (14, 8)

# due blocchi sfalsati: obbligano a girare, ed e' li' che le due forme si separano
OSTACOLI = {(x, y) for x in range(5, 8) for y in range(0, 6)}
OSTACOLI |= {(x, y) for x in range(10, 13) for y in range(5, 10)}

VICINI = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def libera(c):
    x, y = c
    return 0 <= x < LARGO and 0 <= y < ALTO and c not in OSTACOLI


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def visibile(a, b):
    """Linea di vista campionata: e' il controllo che in Theta* costa quasi tutto."""
    passi = int(dist(a, b) * 8) + 1
    for i in range(passi + 1):
        t = i / passi
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if not libera((int(round(x)), int(round(y)))):
            return False
    return True


def ricerca(any_angle):
    g = {PARTENZA: 0.0}
    padre = {PARTENZA: PARTENZA}
    coda = [(dist(PARTENZA, ARRIVO), PARTENZA)]
    chiusi = set()
    while coda:
        _, corrente = heapq.heappop(coda)
        if corrente == ARRIVO:
            break
        if corrente in chiusi:
            continue
        chiusi.add(corrente)
        for dx, dy in VICINI:
            vicino = (corrente[0] + dx, corrente[1] + dy)
            if not libera(vicino) or vicino in chiusi:
                continue
            nonno = padre[corrente]
            # il passo che distingue i due algoritmi: se il nonno vede il vicino,
            # il segmento si tende e l'angolo intermedio sparisce
            if any_angle and visibile(nonno, vicino):
                base, costo = nonno, g[nonno] + dist(nonno, vicino)
            else:
                base, costo = corrente, g[corrente] + dist(corrente, vicino)
            if costo < g.get(vicino, float("inf")):
                g[vicino], padre[vicino] = costo, base
                heapq.heappush(coda, (costo + dist(vicino, ARRIVO), vicino))
    percorso, nodo = [ARRIVO], ARRIVO
    while nodo != PARTENZA:
        nodo = padre[nodo]
        percorso.append(nodo)
    return percorso[::-1], g[ARRIVO]


def cambi_di_direzione(percorso):
    """Vertici veri: i punti allineati con i due vicini non sono una svolta."""
    cambi = 0
    for i in range(1, len(percorso) - 1):
        (ax_, ay), (bx, by), (cx, cy) = percorso[i - 1], percorso[i], percorso[i + 1]
        primo = math.atan2(by - ay, bx - ax_)
        secondo = math.atan2(cy - by, cx - bx)
        if abs((secondo - primo + math.pi) % (2 * math.pi) - math.pi) > 1e-9:
            cambi += 1
    return cambi


def pannello(ax, percorso, lunghezza, titolo, sottotitolo, coda, colore):
    for x in range(LARGO + 1):
        ax.plot([x, x], [0, ALTO], color=FILETTO, lw=0.5, zorder=0)
    for y in range(ALTO + 1):
        ax.plot([0, LARGO], [y, y], color=FILETTO, lw=0.5, zorder=0)
    for x, y in OSTACOLI:
        ax.add_patch(Rectangle((x - .5, y - .5), 1, 1, color="#C9C9C9", lw=0, zorder=1))

    xs = [p[0] for p in percorso]
    ys = [p[1] for p in percorso]
    ax.plot(xs, ys, color=colore, lw=2.6, solid_joinstyle="round", zorder=3)
    ax.plot(xs, ys, "o", color=colore, ms=4.5, zorder=4)
    for nome, punto in (("", PARTENZA), ("", ARRIVO)):
        ax.plot(*punto, "o", ms=11, mfc="white", mec=TESTO, mew=1.6, zorder=5)

    ax.set_xlim(-0.6, LARGO - 0.4)
    ax.set_ylim(-0.6, ALTO - 0.4)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(titolo, fontsize=15, color=TESTO, family="Georgia", pad=14)
    ax.text(0.5, -0.06, sottotitolo, transform=ax.transAxes, ha="center", va="top",
            fontsize=11, color=TENUE, family="Georgia")
    ax.text(0.5, -0.15, "lunghezza %s · %s" % (("%.2f" % lunghezza).replace(".", ","), coda),
            transform=ax.transAxes, ha="center", va="top", fontsize=10, color=colore)


strada_a, costo_a = ricerca(False)
strada_t, costo_t = ricerca(True)

fig, assi = plt.subplots(1, 2, figsize=(12.6, 4.6))
fig.patch.set_facecolor("white")
pannello(assi[0], strada_a, costo_a, "A*",
         "ogni passo è uno degli otto multipli di 45°",
         "8 direzioni ammesse, %d vertici" % len(strada_a), BLU)
pannello(assi[1], strada_t, costo_t, "Theta*",
         "il segmento si tende quando la vista è libera",
         "nessun vincolo d'angolo, %d vertici" % len(strada_t), ACCENTO)
fig.subplots_adjust(left=0.02, right=0.98, top=0.88, bottom=0.16, wspace=0.06)

destinazione = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "confronto_astar_theta.png")
fig.savefig(destinazione, dpi=220, facecolor="white")
print("A*     lunghezza %.3f, %d vertici" % (costo_a, len(strada_a)))
print("Theta* lunghezza %.3f, %d vertici" % (costo_t, len(strada_t)))
print("guadagno %.1f%%" % ((costo_a - costo_t) / costo_a * 100))
print("scritto", destinazione)

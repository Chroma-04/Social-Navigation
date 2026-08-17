"""Quanto vedrebbe il robot allargando il lidar, sulla planimetria di Povo.

Domanda preliminare allo sweep dei tempi, e serve a non sprecarlo. Nei corridoi da 1,5 m di Povo la
linea di vista e' tagliata dai muri: un lidar da 5 m potrebbe tracciare quasi le stesse persone di uno
da 2 m, perche' il resto e' dietro un angolo. Se e' cosi', variare il raggio non cambia l'informazione
disponibile al predittore e lo sweep dei tempi esce piatto per un motivo geometrico banale, senza dire
nulla sul meccanismo.

Cosa misura, per ogni raggio candidato e a ogni frame di una corsa vera:
  - persone entro il raggio in LINEA D'ARIA (limite superiore, muri ignorati);
  - persone effettivamente TRACCIATE, cioe' entro il raggio e in linea di vista libera;
  - quante di queste sono in MOVIMENTO, le uniche per cui la previsione abbia senso: su una persona
    ferma il Kalman non ha nulla da prevedere e la correzione non ha nulla da correggere.

Il rapporto fra tracciate e in linea d'aria e' la frazione di mondo che i muri portano via.

Nota sull'approssimazione: la traiettoria del robot dipende dal raggio con cui gira davvero (vede
persone diverse, quindi pianifica diversamente). Qui si misurano TUTTI i raggi lungo l'unica traiettoria
prodotta dal raggio corrente. E' la domanda giusta per uno screening - "da dove il robot passa
realmente, quanto vedrebbe di piu'?" - ma non sostituisce lo sweep vero.

Uso:
    py TEST/osservabilita_lidar.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import math
import statistics

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.dirname(os.path.abspath(__file__)),
                  os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim

MAPPA = "povo"
NUMERO_SEMI = 3  # bastano: la domanda e' geometrica, non statistica

# identica alla serie confronto_ai_povo_d070, cosi' i numeri sono confrontabili con le campagne sui tempi
POPOLAZIONE = {
    "numero_persone": 72,
    "numero_corridori": 66,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}

# 123px e' il valore del pannello usato in tutte le campagne; 300px = DIM_NODO*10 e' il massimo dello slider
RAGGI = [123, 180, 240, 300]

_conteggi = {r: {"aria": [], "tracciate": [], "mobili": []} for r in RAGGI}
_originale = t._rileva_e_traccia


def _spia(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
    for raggio in RAGGI:
        aria = tracciate = mobili = 0
        for p in tutte_le_persone:
            if math.hypot(p["x"] - robot_x, p["y"] - robot_y) >= raggio:
                continue
            aria += 1
            if not sim.linea_di_vista_libera(griglia, robot_x, robot_y, p["x"], p["y"]):
                continue
            tracciate += 1
            if p.get("stato", "attesa") != "attesa":
                mobili += 1
        _conteggi[raggio]["aria"].append(aria)
        _conteggi[raggio]["tracciate"].append(tracciate)
        _conteggi[raggio]["mobili"].append(mobili)
    return _originale(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)


def main():
    t._rileva_e_traccia = _spia
    popolazione = dict(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())

    import zlib
    semi = [zlib.crc32(f"{MAPPA}|confronto_ai_povo|{i}".encode()) for i in range(NUMERO_SEMI)]
    print(f"Osservabilita' del lidar su {MAPPA}, {NUMERO_SEMI} corse (condizione base)\n")
    for seme in semi:
        arrivato, frame, _ = t.esegui_corsa(MAPPA, "base", popolazione, seme)
        print(f"  seme {seme}: {t._secondi(frame):.1f}s, arrivato={arrivato}")

    frame_totali = len(_conteggi[RAGGI[0]]["aria"])
    print(f"\n{frame_totali} frame osservati\n")
    intestazione = f"{'raggio':>10}{'metri':>8}{'in aria':>10}{'tracciate':>12}{'% vista':>10}{'mobili':>9}{'vs 123px':>10}"
    print(intestazione)
    print("-" * len(intestazione))

    base_mobili = None
    for raggio in RAGGI:
        aria = statistics.mean(_conteggi[raggio]["aria"])
        tracciate = statistics.mean(_conteggi[raggio]["tracciate"])
        mobili = statistics.mean(_conteggi[raggio]["mobili"])
        if base_mobili is None:
            base_mobili = mobili
        quota = tracciate / aria * 100 if aria > 0 else 0.0
        crescita = mobili / base_mobili if base_mobili > 0 else 0.0
        print(f"{raggio:>10}{raggio / 60:>8.2f}{aria:>10.1f}{tracciate:>12.1f}"
              f"{quota:>9.0f}%{mobili:>9.1f}{crescita:>9.2f}x")

    print("\n'in aria' ignora i muri, 'tracciate' li rispetta: la differenza e' quanto mondo tolgono i")
    print("corridoi. 'mobili' e' cio' che conta per la previsione - le persone ferme non si prevedono.")


if __name__ == "__main__":
    main()

"""Riesegue alcune corse archiviate e verifica che il codice odierno dia gli stessi numeri.

Le misure in archivio riportano 'a4c8b82+modifiche-non-committate': non esiste un hash che
identifichi il codice che le ha prodotte. Se le corse rieseguite oggi coincidono con quelle
archiviate, allora lo stato attuale del repository E' quel codice, e lo si puo' dichiarare in
tesi come riferimento riproducibile. Se differiscono, qualcosa e' cambiato dopo la campagna e
va individuato prima di consegnare.

Si controllano tempo, tempo fermo e distanza: le tre grandezze che finiscono nelle tabelle.

    py ibridazione_velocita/verifica_riproducibilita.py            2 scene
    py ibridazione_velocita/verifica_riproducibilita.py --scene=4  piu' scene
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import math
import time

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_QUI = os.path.dirname(os.path.abspath(__file__))
for _p in (_QUI, _RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import testing as t
import confronto_controlli as cc

ARCHIVIO = os.path.join(t.CARTELLA_RISULTATI, "confronto_completo_velocita_libera.csv")
TOLLERANZA = 1e-4   # i CSV sono arrotondati a quattro decimali: sotto questa soglia e' identita'


def _corsa(etichetta, seme, popolazione, condizione):
    traiettoria, distanze = [], []
    originale = t._rileva_e_traccia

    def _con_misure(tutte, rx, ry, griglia, tracce, storico):
        minimo = None
        for p in tutte:
            if "stato" not in p:
                continue
            d = math.hypot(p["x"] - rx, p["y"] - ry)
            if minimo is None or d < minimo:
                minimo = d
        if minimo is not None:
            distanze.append(minimo)
        traiettoria.append((rx, ry))
        return originale(tutte, rx, ry, griglia, tracce, storico)

    t._rileva_e_traccia = _con_misure
    try:
        _, frame, frame_fermo = t.esegui_corsa("pova" if False else "povo", condizione,
                                               popolazione, seme)
    finally:
        t._rileva_e_traccia = originale
    tempo_s = t._secondi(frame)
    return tempo_s, t._secondi(frame_fermo), cc._metriche(traiettoria, distanze, tempo_s)


def principale():
    quante = 2
    for a in sys.argv[1:]:
        if a.startswith("--scene="):
            quante = int(a.split("=", 1)[1])

    livelli = {f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}": cc._popolazione_scalata(f)
               for f in cc.FATTORI}
    with open(ARCHIVIO, newline="", encoding="utf-8") as fh:
        righe = [r for r in csv.DictReader(fh) if r["condizione"] == "velocita_libera"]
    righe = righe[:quante]

    print(f"riesecuzione di {len(righe)} scene archiviate, confronto con i valori nel CSV\n")
    tutto_ok = True
    for r in righe:
        popolazione = livelli[r["livello"]]
        for braccio, condizione in (("base", "base"), ("variante", "velocita_libera")):
            avvio = time.time()
            tempo_s, fermo_s, metriche = _corsa(r["livello"], int(r["seme"]), popolazione, condizione)
            atteso = {
                "tempo": float(r[f"tempo_s_{braccio}"]),
                "fermo": float(r[f"fermo_s_{braccio}"]),
                "distanza": float(r[f"distanza_media_m_{braccio}"]),
            }
            ottenuto = {"tempo": tempo_s, "fermo": fermo_s,
                        "distanza": metriche["distanza_media_m"]}
            esiti = []
            for k in atteso:
                scarto = abs(atteso[k] - ottenuto[k])
                esiti.append(f"{k} {ottenuto[k]:.4f} vs {atteso[k]:.4f} (scarto {scarto:.6f})")
                if scarto > TOLLERANZA:
                    tutto_ok = False
            stato = "OK " if all(abs(atteso[k] - ottenuto[k]) <= TOLLERANZA for k in atteso) else "DIVERSO"
            print(f"  [{stato}] {r['livello']:>7} seme {r['seme']:>11} {braccio:>8} "
                  f"({time.time()-avvio:.0f}s)")
            for e in esiti:
                print(f"          {e}")

    print("\n" + ("RIPRODUCIBILE: il codice attuale rigenera i numeri archiviati."
                  if tutto_ok else
                  "DIVERGENZA: il codice e' cambiato dopo la campagna. Va individuato cosa."))


if __name__ == "__main__":
    principale()

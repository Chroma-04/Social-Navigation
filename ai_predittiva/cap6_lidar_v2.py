"""Tabella 6 rifatta con la rete v2: vantaggio della correzione al crescere della portata del lidar.

Stesso impianto delle altre due campagne del capitolo 6: la cinematica originaria viene
ripristinata dentro il worker, il braccio di riferimento resta quello in archivio e si riesegue
soltanto il braccio con la rete. Quattro portate per dieci semi appaiati: 40 corse.

La tesi dichiara questa misura come screening e non come risultato conclusivo, e lo resta: dieci
coppie per livello non bastano a stabilire nulla. Serve pero' rifarla, perche' la riga a portata
di esercizio e' quella che in tesi viene discussa come artefatto della ridotta numerosita'.

    py ai_predittiva/cap6_lidar_v2.py             40 corse
    py ai_predittiva/cap6_lidar_v2.py --tabella   ricalcola dal CSV senza simulare
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import time
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import allena_previsione as ap
from cap6_tempi_v2 import (VELOCITA_ROBOT_CAP6, VELOCITA_PERSONA_CAP6, SENZA_LIMITI,
                           FILE_V2, POPOLAZIONE_D070)

FILE_ARCHIVIO = os.path.join(t.CARTELLA_RISULTATI, "sweep_lidar_povo_d070.csv")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "cap6_lidar_v2.csv")
PIXEL_PER_METRO = 30.0


def _inizializza():
    sim.VELOCITA_ROBOT = VELOCITA_ROBOT_CAP6
    sim.VELOCITA_PERSONA = VELOCITA_PERSONA_CAP6
    sim.ACCELERAZIONE_ROBOT_M_S2 = SENZA_LIMITI
    sim.DECELERAZIONE_ROBOT_M_S2 = SENZA_LIMITI
    t._modello_ai_worker = ap.modello_da_file(FILE_V2)


def _corsa(argomenti):
    """`raggio` e' la chiave dell'archivio (arrotondata a un decimale), `raggio_vero` il valore
    effettivamente impiegato. Sulla riga di esercizio i due differiscono: l'archivio registra
    122,6 px mentre la geometria salvata vale 122,647, e passare il valore arrotondato produce
    una corsa diversa da quella con cui il braccio di riferimento e' stato misurato."""
    raggio, raggio_vero, seme, popolazione = argomenti
    pop = dict(popolazione)
    pop["raggio_lidar"] = raggio_vero   # letto da esegui_corsa, che riscrive sim.RAGGIO_LIDAR_DEFAULT
    arrivato, frame, frame_fermo = t.esegui_corsa("povo", "solo_ai", pop, seme)
    return raggio, seme, arrivato, t._secondi(frame), t._secondi(frame_fermo)


def _archivio():
    with open(FILE_ARCHIVIO, newline="", encoding="utf-8") as fh:
        return {(round(float(r["raggio_lidar_px"]), 3), int(r["seme"])): r
                for r in csv.DictReader(fh)}


def _prospetto(righe):
    print("\n" + "=" * 96)
    print("TABELLA 6 CON LA RETE v2 --- vantaggio della correzione al crescere della portata")
    print("=" * 96)
    print("  variazione positiva = la rete fa meglio (tempo piu' breve)\n")
    print(f"  {'portata':>10}{'coppie':>8}{'senza corr.':>13}{'con v2':>10}"
          f"{'variaz.':>10}{'rapp.':>8}{'vinte':>9}")
    raggi = sorted({r["raggio_lidar_px"] for r in righe})
    tutte = []
    for raggio in raggi:
        sotto = [r for r in righe if r["raggio_lidar_px"] == raggio]
        a = [r["tempo_base_s"] for r in sotto]
        b = [r["tempo_v2_s"] for r in sotto]
        rel = [(x - y) / x * 100 for x, y in zip(a, b)]
        tutte.extend(rel)
        n = len(rel)
        media = statistics.mean(rel)
        errore = statistics.stdev(rel) / n ** 0.5 if n > 1 else float("inf")
        rapporto = abs(media / errore) if errore > 0 else float("inf")
        print(f"  {raggio/PIXEL_PER_METRO:>9.2f}m{n:>8}{statistics.mean(a):>12.2f}s"
              f"{statistics.mean(b):>9.2f}s{media:>+9.1f}%{rapporto:>8.2f}"
              f"{sum(1 for d in rel if d > 0):>6}/{n}")
    media = statistics.mean(tutte)
    errore = statistics.stdev(tutte) / len(tutte) ** 0.5
    print(f"\n  AGGREGATO sulle {len(tutte)} ripetizioni: {media:+.2f}% "
          f"rapporto {abs(media/errore):.2f}")
    print("  ARCHIVIO con la rete v1: aggregato -1,97% rapporto 1,46;")
    print("  per portata -4,5% (2,10) | +1,4% (1,06) | -2,7% (0,47) | -2,0% (0,70)")


def principale():
    if "--tabella" in sys.argv:
        if not os.path.exists(FILE_CSV):
            print(f"{FILE_CSV} non trovato.")
            return
        with open(FILE_CSV, newline="", encoding="utf-8") as fh:
            righe = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fh)]
        _prospetto(righe)
        return
    if not os.path.exists(FILE_ARCHIVIO):
        print(f"manca {FILE_ARCHIVIO}")
        return

    archivio = _archivio()
    popolazione = dict(POPOLAZIONE_D070)
    popolazione.update(t._geometria_salvata())
    salvato = float(popolazione["raggio_lidar"])
    def _vero(raggio):
        # la riga di esercizio va eseguita al raggio non arrotondato, altrimenti non e' appaiata
        return salvato if abs(raggio - salvato) < 0.5 else raggio
    compiti = [(raggio, _vero(raggio), seme, popolazione) for raggio, seme in sorted(archivio)]

    n_worker = min(8, os.cpu_count() or 4)
    print("Tabella 6 con la rete v2, solo il braccio AI")
    print(f"  {len(compiti)} corse, {n_worker} processi, cinematica del capitolo 6 ripristinata\n")

    esiti, avvio, n = {}, time.time(), 0
    with mp.Pool(n_worker, initializer=_inizializza) as pool:
        for raggio, seme, arrivato, tempo, fermo in pool.imap_unordered(_corsa, compiti, chunksize=1):
            n += 1
            if arrivato:
                esiti[(round(raggio, 3), seme)] = (tempo, fermo)
            print(f"  [{n}/{len(compiti)}] raggio {raggio:>7.1f}px seme {seme}: {tempo:.2f}s "
                  f"({(time.time()-avvio)/60:.1f} min)", flush=True)
    print(f"\nCompletato in {(time.time()-avvio)/60:.1f} min")

    righe = []
    for chiave in sorted(archivio):
        if chiave not in esiti:
            continue
        r = archivio[chiave]
        tempo, fermo = esiti[chiave]
        righe.append({"raggio_lidar_px": chiave[0], "seme": chiave[1],
                      "tempo_base_s": float(r["tempo_base_s"]),
                      "tempo_v1_s": float(r["tempo_ai_s"]),
                      "tempo_v2_s": round(tempo, 4),
                      "fermo_base_s": float(r["fermo_base_s"]),
                      "fermo_v2_s": round(fermo, 4)})
    if not righe:
        print("Nessuna coppia valida.")
        return
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    _prospetto(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

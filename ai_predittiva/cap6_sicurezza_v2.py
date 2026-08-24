"""Tabella 4 rifatta con la rete v2: margine spaziale, esposizione e fluidita'.

COSA SOSTITUISCE. Le tre grandezze comportamentali del quadro riassuntivo che dipendono dalla
rete: la distanza media dai corpi in movimento (riportata come tendenza, +5,2%), la frazione di
tempo entro la soglia di prossimita' (non confermata in replica) e il jerk medio (nessuna
variazione). Sono misurate dalle due campagne di sicurezza, 30 + 50 coppie su semi disgiunti.

SOLO IL BRACCIO AI. Le due campagne condividono scene e cinematica con confronto_ai_povo_d070:
la prima riga di sicurezza_povo_d070.csv riporta 46,92 s per il riferimento e 47,67 s per la
rete, gli stessi valori dell'altra campagna sugli stessi semi. Ripristinando la cinematica del
capitolo 6 il braccio di riferimento e' riproducibile (verificato: 46,92 contro 46,92), quindi
va rieseguito soltanto il braccio con la rete.

DUE SCALE. Il banco originale converte i pixel in metri con 60 px/m, ereditati da quando una
cella valeva mezzo metro; la scala dichiarata in tesi e' invece 30 px/m (DIM_NODO 30,
METRI_PER_CELLA 1,0). Le distanze assolute della tesi al 6.2.2 sono quindi dimezzate rispetto a
quelle dei capitoli 7 e 8. Il prospetto riporta entrambe le convenzioni: la variazione
percentuale non ne dipende, i centimetri si'.

    py ai_predittiva/cap6_sicurezza_v2.py            80 corse
    py ai_predittiva/cap6_sicurezza_v2.py --semi=6   versione corta
    py ai_predittiva/cap6_sicurezza_v2.py --tabella  ricalcola dal CSV senza simulare
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
import test_sicurezza_povo as ts
import allena_previsione as ap
from cap6_tempi_v2 import (VELOCITA_ROBOT_CAP6, VELOCITA_PERSONA_CAP6, SENZA_LIMITI,
                           FILE_V1, FILE_V2)

FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "cap6_sicurezza_v2.csv")
ARCHIVI = ("sicurezza_povo_d070.csv", "sicurezza_povo_d070_replica.csv")
FATTORE_SCALA = ts.PIXEL_PER_METRO / 30.0   # dal banco originale alla scala dichiarata in tesi

GRANDEZZE = (("distanza_media_m", "distanza media (m)", True, True),
             ("distanza_minima_m", "distanza minima (m)", True, True),
             ("esposizione_percento", "frame sotto 0,5 m (%)", False, False),
             ("jerk_medio", "jerk medio (m/s^3)", False, False))

_PERCORSO = FILE_V2


def _inizializza(percorso):
    sim.VELOCITA_ROBOT = VELOCITA_ROBOT_CAP6
    sim.VELOCITA_PERSONA = VELOCITA_PERSONA_CAP6
    sim.ACCELERAZIONE_ROBOT_M_S2 = SENZA_LIMITI
    sim.DECELERAZIONE_ROBOT_M_S2 = SENZA_LIMITI
    t._modello_ai_worker = ap.modello_da_file(percorso)


def _corsa(argomenti):
    """Riusa l'intero strumento di misura del banco originale: cambia solo il modello caricato."""
    seme, popolazione = argomenti
    _, _, arrivato, tempo_s, fermo_s, misure = ts._una_corsa(("solo_ai", seme, popolazione))
    return seme, arrivato, tempo_s, misure


def _archivio():
    base = {}
    for nome in ARCHIVI:
        percorso = os.path.join(t.CARTELLA_RISULTATI, nome)
        if not os.path.exists(percorso):
            print(f"  ATTENZIONE: manca {nome}")
            continue
        with open(percorso, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                base[int(r["seme"])] = {c: float(r[f"{c}_base"]) for c, _, _, _ in GRANDEZZE}
                base[int(r["seme"])]["tempo_s"] = float(r["tempo_base_s"])
                base[int(r["seme"])]["v1"] = {c: float(r[f"{c}_ai"]) for c, _, _, _ in GRANDEZZE}
    return base


def _prospetto(righe):
    print("\n" + "=" * 104)
    print(f"TABELLA 4 CON LA RETE v2 --- grandezze comportamentali, {len(righe)} coppie appaiate")
    print("=" * 104)
    print("  segno + = la rete fa meglio (distanza piu' alta, esposizione e jerk piu' bassi)\n")
    print(f"  {'grandezza':<24}{'rete':>6}{'senza':>10}{'con rete':>10}{'variaz.':>10}"
          f"{'differenza':>13}{'rapp.':>8}{'vinte':>9}")
    for chiave, nome, meglio_alto, e_distanza in GRANDEZZE:
        for eti in ("v1", "v2"):
            a = [r[f"{chiave}_base"] for r in righe]
            b = [r[f"{chiave}_{eti}"] for r in righe]
            diff = [(y - x) if meglio_alto else (x - y) for x, y in zip(a, b)]
            n = len(diff)
            media = statistics.mean(diff)
            errore = statistics.stdev(diff) / n ** 0.5
            rapporto = abs(media / errore) if errore > 0 else float("inf")
            variazione = (statistics.mean(b) - statistics.mean(a)) / statistics.mean(a) * 100
            print(f"  {nome:<24}{eti:>6}{statistics.mean(a):>10.3f}{statistics.mean(b):>10.3f}"
                  f"{variazione:>+9.1f}%{media:>+13.4f}{rapporto:>8.2f}"
                  f"{sum(1 for d in diff if d > 0):>6}/{n}")
        print()

    print("  Le distanze qui sopra sono nella scala del banco originale (60 px/m), la stessa")
    print("  del 6.2.2. Nella scala dichiarata in tesi (30 px/m) vanno RADDOPPIATE:")
    for eti in ("v1", "v2"):
        a = [r["distanza_media_m_base"] for r in righe]
        b = [r[f"distanza_media_m_{eti}"] for r in righe]
        media = statistics.mean([(y - x) for x, y in zip(a, b)])
        print(f"    rete {eti}: {statistics.mean(a)*FATTORE_SCALA:.3f} -> "
              f"{statistics.mean(b)*FATTORE_SCALA:.3f} m, guadagno "
              f"{media*FATTORE_SCALA*100:+.1f} cm")
    print("\n  ARCHIVIO (tabella 4, rete v1): margine spaziale +5,2% (8 cm), tendenza;")
    print("  esposizione non confermata in replica; fluidita' nessuna variazione.")


def principale():
    if "--tabella" in sys.argv:
        if not os.path.exists(FILE_CSV):
            print(f"{FILE_CSV} non trovato.")
            return
        with open(FILE_CSV, newline="", encoding="utf-8") as fh:
            righe = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fh)]
        _prospetto(righe)
        return
    if not os.path.exists(FILE_V2):
        print(f"manca {FILE_V2}")
        return

    base = _archivio()
    popolazione = dict(ts.POPOLAZIONE)
    popolazione.update(t._geometria_salvata())
    semi = sorted(base)
    for a in sys.argv[1:]:
        if a.startswith("--semi="):
            semi = semi[:int(a.split("=", 1)[1])]
    compiti = [(s, popolazione) for s in semi]

    n_worker = min(8, os.cpu_count() or 4)
    print("Tabella 4 con la rete v2, solo il braccio AI")
    print(f"  {len(compiti)} corse, {n_worker} processi, cinematica del capitolo 6 ripristinata\n")

    esiti, avvio, n = {}, time.time(), 0
    with mp.Pool(n_worker, initializer=_inizializza, initargs=(FILE_V2,)) as pool:
        for seme, arrivato, tempo_s, misure in pool.imap_unordered(_corsa, compiti, chunksize=1):
            n += 1
            if arrivato:
                esiti[seme] = misure
            print(f"  [{n}/{len(compiti)}] seme {seme}: {tempo_s:.2f}s, media "
                  f"{misure['distanza_media_m']:.2f} m ({(time.time()-avvio)/60:.1f} min)", flush=True)
    print(f"\nCompletato in {(time.time()-avvio)/60:.1f} min")

    righe = []
    for seme in semi:
        if seme not in esiti:
            continue
        riga = {"seme": seme}
        for c, _, _, _ in GRANDEZZE:
            riga[f"{c}_base"] = round(base[seme][c], 4)
            riga[f"{c}_v1"] = round(base[seme]["v1"][c], 4)
            riga[f"{c}_v2"] = round(esiti[seme][c], 4)
        righe.append(riga)
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

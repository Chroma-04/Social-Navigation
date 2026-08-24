"""Capitolo 6 rifatto con la rete v2: campagne sui tempi, solo il braccio AI.

PERCHE' BASTA UN BRACCIO. Le campagne del capitolo 6 sono state eseguite con la cinematica
originaria del simulatore (robot 3,0 m/s, pedoni 2,0 m/s, corridori 4,0 m/s, nessun limite di
accelerazione), poi rivista. Ripristinando quelle tre costanti dentro il worker il simulatore
torna deterministicamente allo stato di allora: la verifica e' diretta e ha restituito 46,92 s
contro i 46,92 s in archivio sulla prima scena di confronto_ai_povo_d070. Il braccio di
riferimento non va quindi rifatto, e tutti i numeri della colonna 'senza correzione' della
tabella 3 restano validi. Si riesegue soltanto il braccio con la rete, sostituendo la v1 con la v2.

COSA COPRE. Le quattro campagne del capitolo che confrontano base contro solo_ai sui tempi:
  confronto_ai_povo_d051   50 coppie   12,7 persone / 100 m2   tabella 3, prima riga
  confronto_ai_povo_d064   30 coppie   16,0 persone / 100 m2   tabella 3, seconda riga
  confronto_ai_povo_d070   50 coppie   17,5 persone / 100 m2   tabella 3, terza riga
  confronto_ai_aperta_d070 20 coppie   arena priva di ostacoli  6.3, struttura dell'ambiente

La prova sulla PRESENZA della previsione (6.3, 37 coppie) non impiega la rete in nessuno dei due
bracci e non va rifatta.

    py ai_predittiva/cap6_tempi_v2.py --verifica   riesegue il braccio v1 su 3 scene e lo
                                                   confronta con l'archivio: se coincide, il
                                                   ripristino della cinematica e' corretto
    py ai_predittiva/cap6_tempi_v2.py              150 corse
    py ai_predittiva/cap6_tempi_v2.py --tabella    ricalcola dal CSV senza simulare
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

FILE_V2 = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt")
FILE_V1 = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato.pt")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "cap6_tempi_v2.csv")

# cinematica del capitolo 6, da ripristinare dentro il worker
VELOCITA_ROBOT_CAP6 = 1.5      # 3,0 m/s
VELOCITA_PERSONA_CAP6 = 1.0    # 2,0 m/s; i corridori seguono col fattore 2,0 gia' in vigore
SENZA_LIMITI = 1e9             # accelerazione e decelerazione praticamente istantanee

POPOLAZIONE_D051 = {"numero_persone": 52, "numero_corridori": 48,
                    "numero_persone_ferme": 84, "numero_gruppi": 20}
POPOLAZIONE_D070 = {"numero_persone": 72, "numero_corridori": 66,
                    "numero_persone_ferme": 116, "numero_gruppi": 27}
POPOLAZIONE_D064 = {"numero_persone": 65, "numero_corridori": 60,
                    "numero_persone_ferme": 105, "numero_gruppi": 25}
# l'arena aperta ha area molto maggiore: la stessa densita' richiede piu' individui
POPOLAZIONE_APERTA = {"numero_persone": 163, "numero_corridori": 150,
                      "numero_persone_ferme": 263, "numero_gruppi": 61}

CAMPAGNE = (
    ("d051",   "povo",   "confronto_ai_povo_d051.csv",   POPOLAZIONE_D051, "12,7 pers./100 m2"),
    ("d064",   "povo",   "confronto_ai_povo_d064.csv",   POPOLAZIONE_D064, "16,0 pers./100 m2"),
    ("d070",   "povo",   "confronto_ai_povo_d070.csv",   POPOLAZIONE_D070, "17,5 pers./100 m2"),
    ("aperta", "aperta", "confronto_ai_aperta_d070.csv", POPOLAZIONE_APERTA, "arena aperta"),
)

_PERCORSO_MODELLO = FILE_V2


def _inizializza(percorso):
    """Ripristina la cinematica del capitolo 6 e carica la rete scelta. Sotto spawn questo gira
    nel processo figlio, che e' l'unico posto in cui l'assegnazione e' visibile alla corsa."""
    sim.VELOCITA_ROBOT = VELOCITA_ROBOT_CAP6
    sim.VELOCITA_PERSONA = VELOCITA_PERSONA_CAP6
    sim.ACCELERAZIONE_ROBOT_M_S2 = SENZA_LIMITI
    sim.DECELERAZIONE_ROBOT_M_S2 = SENZA_LIMITI
    t._modello_ai_worker = ap.modello_da_file(percorso)


def _corsa(argomenti):
    campagna, mappa, seme, popolazione = argomenti
    arrivato, frame, frame_fermo = t.esegui_corsa(mappa, "solo_ai", popolazione, seme)
    return campagna, seme, arrivato, t._secondi(frame), t._secondi(frame_fermo)


def _archivio(nome_file):
    percorso = os.path.join(t.CARTELLA_RISULTATI, nome_file)
    if not os.path.exists(percorso):
        return {}
    with open(percorso, newline="", encoding="utf-8") as fh:
        return {int(r["seme"]): r for r in csv.DictReader(fh)}


def _compiti():
    popolazione_geometria = t._geometria_salvata()
    lavoro = []
    for campagna, mappa, nome_file, popolazione, _ in CAMPAGNE:
        pop = dict(popolazione)
        pop.update(popolazione_geometria)   # la geometria salvata e' parte della scena, verificata
        for seme in _archivio(nome_file):
            lavoro.append((campagna, mappa, seme, pop))
    return lavoro


def _prospetto(righe):
    print("\n" + "=" * 100)
    print("TABELLA 3 CON LA RETE v2 --- tempo di percorrenza, confronto appaiato")
    print("=" * 100)
    print("  differenza = senza correzione meno con correzione: POSITIVO = la rete fa meglio\n")
    print(f"  {'campagna':<22}{'coppie':>8}{'senza corr.':>13}{'con v2':>10}"
          f"{'differenza':>12}{'rapp.':>8}{'vinte':>9}")
    for campagna, _, _, _, etichetta in CAMPAGNE:
        sotto = [r for r in righe if r["campagna"] == campagna]
        if not sotto:
            continue
        for chiave, nome in (("tempo", "tempo"), ("fermo", "tempo fermo")):
            a = [r[f"{chiave}_base_s"] for r in sotto]
            b = [r[f"{chiave}_v2_s"] for r in sotto]
            diff = [x - y for x, y in zip(a, b)]
            n = len(diff)
            media = statistics.mean(diff)
            errore = statistics.stdev(diff) / n ** 0.5 if n > 1 else float("inf")
            rapporto = abs(media / errore) if errore > 0 else float("inf")
            print(f"  {etichetta + ' / ' + nome:<22}{n:>8}{statistics.mean(a):>12.2f}s"
                  f"{statistics.mean(b):>9.2f}s{media:>+11.2f}s{rapporto:>8.2f}"
                  f"{sum(1 for d in diff if d > 0):>6}/{n}")

    tutte = [r for r in righe if r["campagna"] != "aperta"]
    if tutte:
        a = [r["tempo_base_s"] for r in tutte]
        b = [r["tempo_v2_s"] for r in tutte]
        rel = [(x - y) / x * 100 for x, y in zip(a, b)]
        n = len(rel)
        media = statistics.mean(rel)
        errore = statistics.stdev(rel) / n ** 0.5
        print(f"\n  AGGREGATO sulle {n} ripetizioni di povo: {media:+.2f}% "
              f"IC 95% [{media-1.96*errore:+.2f}%, {media+1.96*errore:+.2f}%]")
        print("  ARCHIVIO con la rete v1: -0,14%, IC 95% da -1,46% a +1,17%")


def _verifica():
    """Riesegue il braccio v1 su alcune scene e lo confronta con l'archivio."""
    print("Verifica del ripristino della cinematica: braccio v1 contro archivio\n")
    tutti = _compiti()
    lavoro = [next(c for c in tutti if c[0] == campagna) for campagna, _, _, _, _ in CAMPAGNE]
    archivi = {c: _archivio(f) for c, _, f, _, _ in CAMPAGNE}
    with mp.Pool(min(4, len(lavoro)), initializer=_inizializza, initargs=(FILE_V1,)) as pool:
        for campagna, seme, arrivato, tempo, fermo in pool.imap_unordered(_corsa, lavoro):
            atteso = float(archivi[campagna][seme]["tempo_ai_s"])
            stato = "OK" if abs(tempo - atteso) < 0.01 else "DIVERSO"
            print(f"  {campagna:>7} seme {seme:>11}: {tempo:>7.2f} s   archivio {atteso:>7.2f} s"
                  f"   {stato}")


def principale():
    if "--tabella" in sys.argv:
        if not os.path.exists(FILE_CSV):
            print(f"{FILE_CSV} non trovato.")
            return
        with open(FILE_CSV, newline="", encoding="utf-8") as fh:
            righe = [{k: (v if k == "campagna" else float(v)) for k, v in r.items()}
                     for r in csv.DictReader(fh)]
        _prospetto(righe)
        return
    if "--verifica" in sys.argv:
        _verifica()
        return
    if not os.path.exists(FILE_V2):
        print(f"manca {FILE_V2}")
        return

    lavoro = _compiti()
    archivi = {c: _archivio(f) for c, _, f, _, _ in CAMPAGNE}
    n_worker = min(8, os.cpu_count() or 4)
    print(f"Capitolo 6 con la rete v2, solo il braccio AI")
    print(f"  {len(lavoro)} corse, {n_worker} processi, accelerazione {t.ACCELERAZIONE}x")
    print(f"  cinematica ripristinata: robot 3,0 m/s, pedoni 2,0 m/s, senza limiti\n")

    esiti, avvio, n = {}, time.time(), 0
    with mp.Pool(n_worker, initializer=_inizializza, initargs=(FILE_V2,)) as pool:
        for campagna, seme, arrivato, tempo, fermo in pool.imap_unordered(_corsa, lavoro, chunksize=1):
            n += 1
            if arrivato:
                esiti[(campagna, seme)] = (tempo, fermo)
            else:
                print(f"  {campagna} seme {seme}: non arrivato, scartata")
            print(f"  [{n}/{len(lavoro)}] {campagna:>7} seme {seme}: {tempo:.2f}s "
                  f"({(time.time()-avvio)/60:.1f} min)", flush=True)
    print(f"\nCompletato in {(time.time()-avvio)/60:.1f} min")

    righe = []
    for campagna, _, _, _, _ in CAMPAGNE:
        for seme, r in archivi[campagna].items():
            if (campagna, seme) not in esiti:
                continue
            tempo, fermo = esiti[(campagna, seme)]
            righe.append({"campagna": campagna, "seme": seme,
                          "tempo_base_s": float(r["tempo_base_s"]),
                          "tempo_v1_s": float(r["tempo_ai_s"]),
                          "tempo_v2_s": round(tempo, 4),
                          "fermo_base_s": float(r["fermo_base_s"]),
                          "fermo_v1_s": float(r["fermo_ai_s"]),
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

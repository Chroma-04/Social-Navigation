"""Tabella 9 della tesi rifatta con la rete v2: 82 coppie appaiate su tre campagne disgiunte.

COSA SOSTITUISCE. La tabella 9 riporta il confronto fra il controllo di velocita' alimentato dal
Kalman puro e lo stesso controllo alimentato da Kalman + rete v1 (previsione +12,1%), su 82 coppie
distribuite in tre campagne su semi mutuamente disgiunti. Qui si rifa' lo stesso confronto con la
rete v2 (+20,7%), sulle medesime 82 scene, cosi' il numero nuovo e' sostituibile al vecchio senza
cambiare nient'altro del capitolo.

PERCHE' SERVONO SOLO 58 CORSE E NON 164. Il braccio Kalman esiste gia' in archivio per tutte e tre
le campagne, ed e' riproducibile: la campagna a tre bracci di ai_velocita_v2.py ha restituito
131,069 s dove la tabella 8 riporta 131,07 sulle stesse scene. Il determinismo del simulatore
regge, e le corse col Kalman non vanno rifatte. Delle 82 scene, inoltre, 24 hanno gia' il braccio
v2 misurato da quella campagna: restano le 10 della replica e le 48 dell'estensione.

    py ai_controllo/ai_velocita_v2_completo.py            58 corse
    py ai_controllo/ai_velocita_v2_completo.py --tabella  ricalcola dal CSV senza simulare
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import math
import time
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "ibridazione_velocita"),
                  os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva"),
                  os.path.dirname(os.path.abspath(__file__))):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import confronto_controlli as cc
import ai_velocita as av
import allena_previsione as ap

CONDIZIONE = "velocita_libera"
FILE_V2 = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "ai_velocita_v2_82coppie.csv")
# gia' misurato dalla campagna a tre bracci: 24 scene, colonna del braccio rete_v2
FILE_TRE_BRACCI = os.path.join(t.CARTELLA_RISULTATI, "ai_velocita_v2_povo.csv")
ARCHIVI = ("ai_velocita_povo.csv", "ai_velocita_replica_povo.csv",
           "ai_velocita_estensione_povo.csv")

GRANDEZZE = (("tempo_s", "tempo di percorrenza (s)", True),
             ("numero_fermate", "numero di fermate", True),
             ("fermo_s", "tempo trascorso fermo (s)", True),
             ("distanza_media_m", "distanza dalle persone (m)", False),
             ("velocita_media_m_s", "velocita' media (m/s)", False))

_MODELLO = None


def _inizializza():
    """La rete v2 caricata una volta per worker. Sotto spawn gira nel figlio."""
    global _MODELLO
    _MODELLO = ap.modello_da_file(FILE_V2)


def _corsa(argomenti):
    """Una corsa di velocita_libera con la rete v2 dentro il controllo di velocita'."""
    livello, seme, popolazione = argomenti
    t._modello_ai_worker = _MODELLO
    orizzonte = max(1, sim.PREVISIONE_BLOB_FRAME // t.ACCELERAZIONE)

    traiettoria, distanze = [], []
    ripristina = av.installa(t, sim, _MODELLO, orizzonte)
    traccia_a_valle = t._rileva_e_traccia

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                    storico_posizioni):
        minimo = None
        for p in tutte_le_persone:
            if "stato" not in p:
                continue  # persone ferme escluse, come in tutte le campagne di questo canale
            d = math.hypot(p["x"] - robot_x, p["y"] - robot_y)
            if minimo is None or d < minimo:
                minimo = d
        if minimo is not None:
            distanze.append(minimo)
        traiettoria.append((robot_x, robot_y))
        return traccia_a_valle(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                               storico_posizioni)

    originale = t._rileva_e_traccia
    t._rileva_e_traccia = _con_misure
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", CONDIZIONE, popolazione, seme)
    finally:
        t._rileva_e_traccia = originale
        ripristina()

    tempo_s = t._secondi(frame)
    misure = cc._metriche(traiettoria, distanze, tempo_s)
    misure["tempo_s"] = tempo_s
    misure["fermo_s"] = t._secondi(frame_fermo)
    return livello, seme, arrivato, misure


def _archivio_kalman():
    """Il braccio Kalman delle tre campagne, indicizzato per (livello, seme)."""
    kalman = {}
    for nome in ARCHIVI:
        percorso = os.path.join(t.CARTELLA_RISULTATI, nome)
        if not os.path.exists(percorso):
            print(f"  ATTENZIONE: manca {nome}, campagna saltata")
            continue
        with open(percorso, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                kalman[(r["livello"], int(r["seme"]))] = {
                    c: float(r[f"{c}_kalman"]) for c, _, _ in GRANDEZZE}
    return kalman


def _v2_gia_misurate():
    """Il braccio rete_v2 della campagna a tre bracci: 24 scene gia' fatte."""
    if not os.path.exists(FILE_TRE_BRACCI):
        return {}
    fatte = {}
    with open(FILE_TRE_BRACCI, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            fatte[(r["livello"], int(r["seme"]))] = {
                c: float(r[f"{c}_rete_v2"]) for c, _, _ in GRANDEZZE}
    return fatte


def _confronta(coppie, meglio_basso):
    differenze = [(a - b) if meglio_basso else (b - a) for a, b in coppie]
    n = len(differenze)
    media = statistics.mean(differenze)
    errore = statistics.stdev(differenze) / n ** 0.5 if n > 1 else float("inf")
    # intervallo al 95% con il moltiplicatore normale, come nel resto del lavoro
    return media, errore, (abs(media / errore) if errore > 0 else float("inf")), \
        media - 1.96 * errore, media + 1.96 * errore, sum(1 for d in differenze if d > 0), n


def _prospetto(righe):
    print("\n" + "=" * 100)
    print(f"TABELLA 9 CON LA RETE v2 --- {len(righe)} coppie appaiate su tre campagne disgiunte")
    print("=" * 100)
    print("differenza = rete meno Kalman, orientata come nella tesi:")
    print("  su tempo, fermate e tempo fermo il segno POSITIVO indica che la rete peggiora;")
    print("  sulla distanza il segno positivo indica che la rete guadagna spazio.\n")
    print(f"  {'grandezza':<28}{'Kalman':>10}{'Kalman+v2':>12}{'differenza':>12}"
          f"{'IC 95%':>22}{'rapp.':>8}{'vinte':>9}")
    for chiave, nome, _ in GRANDEZZE:
        a = [r[f"{chiave}_kalman"] for r in righe]
        b = [r[f"{chiave}_v2"] for r in righe]
        # orientamento della tesi: differenza semplice rete - kalman
        diff = [y - x for x, y in zip(a, b)]
        n = len(diff)
        media = statistics.mean(diff)
        errore = statistics.stdev(diff) / n ** 0.5
        rapporto = abs(media / errore) if errore > 0 else float("inf")
        lo, hi = media - 1.96 * errore, media + 1.96 * errore
        vinte = sum(1 for d in diff if (d < 0 if chiave != "distanza_media_m" else d > 0))
        print(f"  {nome:<28}{statistics.mean(a):>10.3f}{statistics.mean(b):>12.3f}"
              f"{media:>+12.3f}{f'[{lo:+.3f}, {hi:+.3f}]':>22}{rapporto:>8.2f}{vinte:>6}/{n}")

    print("\n  test dei segni sui livelli di densita' (blocchi indipendenti)")
    livelli = sorted({r["livello"] for r in righe}, key=lambda s: int(s[3:]))
    for chiave, nome, _ in GRANDEZZE[:1] + GRANDEZZE[3:4]:
        favorevoli = 0
        for liv in livelli:
            sotto = [r for r in righe if r["livello"] == liv]
            da = statistics.mean(r[f"{chiave}_kalman"] for r in sotto)
            db = statistics.mean(r[f"{chiave}_v2"] for r in sotto)
            meglio = (db < da) if chiave != "distanza_media_m" else (db > da)
            favorevoli += 1 if meglio else 0
        print(f"    {nome:<28} {favorevoli}/{len(livelli)} livelli favorevoli alla rete")

    print("\n  ARCHIVIO --- gli stessi confronti con la rete v1 (tabella 9 della tesi):")
    print("    tempo +0,21 s rapp 0,09 | fermate +1,28 rapp 1,32 | fermo +0,35 s rapp 0,69"
          " | distanza -0,027 m rapp 0,57")


def principale():
    if "--tabella" in sys.argv:
        if not os.path.exists(FILE_CSV):
            print(f"{FILE_CSV} non trovato.")
            return
        with open(FILE_CSV, newline="", encoding="utf-8") as fh:
            righe = [{k: (v if k in ("mappa", "livello") else float(v)) for k, v in r.items()}
                     for r in csv.DictReader(fh)]
        _prospetto(righe)
        return
    if not os.path.exists(FILE_V2):
        print(f"manca {FILE_V2}")
        return

    kalman = _archivio_kalman()
    fatte = _v2_gia_misurate()
    livelli = {f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}": cc._popolazione_scalata(f)
               for f in cc.FATTORI}
    da_fare = [(liv, seme, livelli[liv]) for (liv, seme) in sorted(kalman, key=lambda k: (int(k[0][3:]), k[1]))
               if liv in livelli and (liv, seme) not in fatte]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Tabella 9 con la rete v2, condizione {CONDIZIONE} su povo")
    print(f"  {len(kalman)} coppie in archivio, {len(fatte)} bracci v2 gia' misurati, "
          f"{len(da_fare)} corse da eseguire")
    print(f"  {n_worker} processi, accelerazione {t.ACCELERAZIONE}x\n")

    esiti = dict(fatte)
    avvio, n = time.time(), 0
    if da_fare:
        with mp.Pool(n_worker, initializer=_inizializza) as pool:
            for livello, seme, arrivato, misure in pool.imap_unordered(_corsa, da_fare, chunksize=1):
                n += 1
                if arrivato:
                    esiti[(livello, seme)] = misure
                else:
                    print(f"  {livello} seme {seme}: non arrivato, scartata")
                print(f"  [{n}/{len(da_fare)}] {livello:>7} seme {seme}: "
                      f"{misure['tempo_s']:.1f}s ({(time.time()-avvio)/60:.1f} min)", flush=True)
        print(f"\nCompletato in {(time.time()-avvio)/60:.1f} min")

    righe = []
    for chiave in sorted(kalman, key=lambda k: (int(k[0][3:]), k[1])):
        if chiave not in esiti:
            continue
        liv, seme = chiave
        riga = {"mappa": "povo", "livello": liv, "seme": seme}
        for c, _, _ in GRANDEZZE:
            riga[f"{c}_kalman"] = round(kalman[chiave][c], 4)
            riga[f"{c}_v2"] = round(esiti[chiave][c], 4)
        righe.append(riga)

    if not righe:
        print("Nessuna coppia valida.")
        return
    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    _prospetto(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

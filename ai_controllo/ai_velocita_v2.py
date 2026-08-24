"""Le due reti dentro il CONTROLLO DI VELOCITA': cambia qualcosa fra il 12,1% e il 20,7%?

CONTESTO. E11 ha innestato la rete della tesi nel controllo di velocita' e ha trovato un esito
nullo su tre campagne (82 coppie aggregate, distanza -0,027 m a rapporto 0,57). Uno dei due limiti
dichiarati era la qualita' del modello: "un esito nullo va letto come 'questo modello non aiuta',
non come 'nessun modello aiuterebbe'". Il modello di E13 riduce l'errore di previsione del 20,7%
invece del 12,1%, quindi quel limite si puo' togliere di mezzo per misura invece che per ipotesi.

Sul canale PREVISIONE la risposta e' gia' arrivata (E14): fra le due reti non cambia nulla. Questo
banco fa la stessa domanda sul canale VELOCITA', che e' un punto di innesto diverso - la rete
alimenta sim.previsione_per_controllo invece della macchia di rischio - e va misurato a parte.

TRE BRACCI RIFATTI TUTTI, sugli stessi semi. E11 riusava il braccio Kalman dall'archivio; qui no,
e per un motivo preciso emerso in E14: le corse in archivio sono state prodotte su una geometria
precedente, e appaiare corse di epoche diverse introduce esattamente il genere di confondente
asimmetrico che in E12 aveva gia' prodotto un verdetto falso. Rifare anche il riferimento costa un
terzo di corse in piu' e toglie il dubbio.

  riferimento   sim.previsione_per_controllo = Kalman puro
  rete_v1       correzione_kalman_specializzato.pt      previsione +12,1%
  rete_v2       correzione_kalman_specializzato_v2.pt   previsione +20,7%

Condizione 'velocita_libera' in tutti e tre: cambia solo la sorgente di previsione del controllo.

I semi sono quelli dell'archivio (8 livelli x 3 semi = 24 scene), letti invece che rigenerati:
cosi' le scene sono le stesse su cui E11 ha misurato, e i due lavori restano confrontabili anche
se i valori assoluti sono cambiati con la mappa.

IPOTESI FISSATA PRIMA: il confronto rete_v2 contro rete_v1 sul tempo di percorrenza e sulla
distanza media. Il resto e' contesto.

    py ai_controllo/ai_velocita_v2.py             24 scene x 3 bracci = 72 corse
    py ai_controllo/ai_velocita_v2.py --semi=6    versione corta, 18 corse
    py ai_controllo/ai_velocita_v2.py --tabella   ricalcola dal CSV senza simulare
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
BRACCI = ("riferimento", "rete_v1", "rete_v2")
FILE_V2 = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt")
FILE_ARCHIVIO = os.path.join(t.CARTELLA_RISULTATI, "confronto_completo_velocita_libera.csv")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "ai_velocita_v2_povo.csv")

GRANDEZZE = (("tempo_s", "tempo di percorrenza (s)", True),
             ("fermo_s", "tempo trascorso fermo (s)", True),
             ("numero_fermate", "numero di fermate", True),
             ("distanza_media_m", "distanza dalle persone (m)", False),
             ("velocita_media_m_s", "velocita' media (m/s)", False))

_MODELLI = {}


def _inizializza():
    """Entrambe le reti caricate una volta per worker. Sotto spawn questo gira nel figlio: e'
    l'unico punto in cui l'assegnazione dei globali e' visibile alla corsa."""
    global _MODELLI
    t._inizializza_worker()
    _MODELLI = {"rete_v1": t._modello_ai_worker, "rete_v2": ap.modello_da_file(FILE_V2)}


def _corsa(argomenti):
    """Una corsa di velocita_libera con la sorgente di previsione scelta dal braccio."""
    braccio, etichetta, seme, popolazione = argomenti
    orizzonte = max(1, sim.PREVISIONE_BLOB_FRAME // t.ACCELERAZIONE)

    traiettoria, distanze = [], []
    # il braccio di riferimento non installa nulla: previsione_per_controllo resta il Kalman puro
    ripristina = (av.installa(t, sim, _MODELLI[braccio], orizzonte)
                  if braccio in _MODELLI else (lambda: None))
    traccia_a_valle = t._rileva_e_traccia

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                    storico_posizioni):
        minimo = None
        for p in tutte_le_persone:
            if "stato" not in p:
                continue  # persone ferme escluse: il robot le rasenta per costruzione
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
    return braccio, etichetta, seme, arrivato, misure


def _scene():
    """(livello, seme) dall'archivio: le stesse scene su cui ha misurato E11."""
    livelli = {f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}": cc._popolazione_scalata(f)
               for f in cc.FATTORI}
    chiavi = []
    with open(FILE_ARCHIVIO, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if r["condizione"] == CONDIZIONE and r["livello"] in livelli:
                chiavi.append((r["livello"], int(r["seme"])))
    chiavi.sort(key=lambda k: (int(k[0][3:]), k[1]))
    return [(e, s, livelli[e]) for e, s in chiavi]


def _confronta(coppie, meglio_basso):
    differenze = [(a - b) if meglio_basso else (b - a) for a, b in coppie]
    n = len(differenze)
    media = statistics.mean(differenze)
    errore = statistics.stdev(differenze) / n ** 0.5 if n > 1 else float("inf")
    rapporto = abs(media / errore) if errore > 0 else float("inf")
    return media, errore, rapporto, sum(1 for d in differenze if d > 0), n


def _prospetto(dati, livelli_per_scena):
    confronti = (("A) rete v1 (previsione +12,1%) contro riferimento", "riferimento", "rete_v1"),
                 ("B) rete v2 (previsione +20,7%) contro riferimento", "riferimento", "rete_v2"),
                 ("C) rete v2 contro rete v1   <-- E' LA DOMANDA", "rete_v1", "rete_v2"))
    n = len(next(iter(dati["riferimento"].values())))
    print("\n" + "=" * 100)
    print(f"LE DUE RETI DENTRO IL CONTROLLO DI VELOCITA' - {n} scene appaiate, 8 livelli, povo")
    print("=" * 100)
    print("segno + = il secondo braccio fa MEGLIO")

    for titolo, primo, secondo in confronti:
        print(f"\n{titolo}")
        print(f"  {'grandezza':<30}{'medie assolute':>20}{'differenza':>13}{'errore':>9}"
              f"{'rapp.':>8}{'vinte':>9}")
        for chiave, nome, meglio_basso in GRANDEZZE:
            a, b = dati[primo][chiave], dati[secondo][chiave]
            media, errore, rapporto, vinte, nn = _confronta(list(zip(a, b)), meglio_basso)
            print(f"  {nome:<30}{statistics.mean(a):>9.3f} ->{statistics.mean(b):>8.3f}"
                  f"{media:>+13.4f}{errore:>9.4f}{rapporto:>8.2f}{vinte:>6}/{nn}")

        # test dei segni sui livelli di densita', trattati come blocchi indipendenti
        print("    per livello di densita' (blocchi indipendenti):")
        for chiave, nome, meglio_basso in GRANDEZZE[:1] + GRANDEZZE[3:4]:
            favorevoli, totali = 0, 0
            for livello in sorted(set(livelli_per_scena), key=lambda s: int(s[3:])):
                indici = [i for i, l in enumerate(livelli_per_scena) if l == livello]
                da = statistics.mean(dati[primo][chiave][i] for i in indici)
                db = statistics.mean(dati[secondo][chiave][i] for i in indici)
                diff = (da - db) if meglio_basso else (db - da)
                totali += 1
                favorevoli += 1 if diff > 0 else 0
            print(f"      {nome:<28} {favorevoli}/{totali} livelli favorevoli")

    print("\n  Il rapporto e' media/errore standard: sotto 1 indistinguibile da zero, sopra 2 solido.")
    print("  ARCHIVIO (E11, rete v1 contro Kalman, semi in parte diversi e mappa precedente):")
    print("    aggregato 82 coppie   distanza -0,027 m rapp 0,57   tempo -0,21 s rapp 0,09")


def _carica_csv():
    with open(FILE_CSV, newline="", encoding="utf-8") as fh:
        righe = list(csv.DictReader(fh))
    dati = {b: {c: [float(r[f"{c}_{b}"]) for r in righe] for c, _, _ in GRANDEZZE} for b in BRACCI}
    return dati, [r["livello"] for r in righe]


def principale():
    if "--tabella" in sys.argv:
        if not os.path.exists(FILE_CSV):
            print(f"{FILE_CSV} non trovato.")
            return
        dati, livelli = _carica_csv()
        _prospetto(dati, livelli)
        return
    if not os.path.exists(FILE_V2):
        print(f"manca {FILE_V2}: allenalo con allena_con_arresto.py")
        return

    scene = _scene()
    for a in sys.argv[1:]:
        if a.startswith("--semi="):
            scene = scene[:int(a.split("=", 1)[1])]
    compiti = [(b, e, s, p) for e, s, p in scene for b in BRACCI]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Le due reti dentro il controllo di velocita', condizione {CONDIZIONE} su povo")
    print(f"  {len(scene)} scene x 3 bracci = {len(compiti)} corse, {n_worker} processi, "
          f"accelerazione {t.ACCELERAZIONE}x\n")

    esiti, avvio, fatte = {}, time.time(), 0
    with mp.Pool(n_worker, initializer=_inizializza) as pool:
        for braccio, etichetta, seme, arrivato, misure in pool.imap_unordered(
                _corsa, compiti, chunksize=1):
            fatte += 1
            esiti[(etichetta, seme, braccio)] = (arrivato, misure)
            print(f"  [{fatte}/{len(compiti)}] {braccio:<11} {etichetta:>7} seme {seme}: "
                  f"{misure['tempo_s']:.1f}s ({(time.time() - avvio)/60:.1f} min)", flush=True)

    print(f"\nCompletato in {(time.time() - avvio)/60:.1f} min")

    righe, dati, livelli = [], {b: {c: [] for c, _, _ in GRANDEZZE} for b in BRACCI}, []
    for etichetta, seme, _ in scene:
        if any((etichetta, seme, b) not in esiti for b in BRACCI):
            continue
        if not all(esiti[(etichetta, seme, b)][0] for b in BRACCI):
            print(f"  {etichetta} seme {seme}: scena scartata, un robot non e' arrivato")
            continue
        riga = {"mappa": "povo", "livello": etichetta, "seme": seme}
        for b in BRACCI:
            misure = esiti[(etichetta, seme, b)][1]
            for c, _, _ in GRANDEZZE:
                dati[b][c].append(misure[c])
                riga[f"{c}_{b}"] = round(misure[c], 4)
        righe.append(riga)
        livelli.append(etichetta)

    if not righe:
        print("Nessuna scena valida.")
        return
    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    _prospetto(dati, livelli)
    print(f"\nRighe per singola scena: {FILE_CSV}")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

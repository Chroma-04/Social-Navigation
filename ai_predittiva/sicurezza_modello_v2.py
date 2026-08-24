"""Il modello nuovo converte in spazio il guadagno di previsione che il vecchio non convertiva?

CONTESTO. La campagna di TEST/test_sicurezza_povo.py confronta il robot di riferimento con quello
che corregge la previsione con la rete neurale, e misura lo SPAZIO invece del tempo. Su cinquanta
coppie di semi disgiunti dava +7,7 cm di distanza media, rapporto 1,64: una tendenza, non un
effetto stabilito. Quella campagna girava con correzione_kalman_specializzato.pt, che riduce
l'errore di previsione del 12,1%. Il modello ottenuto con arresto anticipato (E13) lo riduce del
20,7%. La domanda e' diretta: quei nove punti in piu' di accuratezza si vedono nello spazio?

DISEGNO A TRE BRACCI SUGLI STESSI SEMI. Confrontare il risultato nuovo con quello archiviato
sarebbe un confronto fra insiemi di semi diversi, e la differenza fra le due reti e' piccola
rispetto alla varianza fra scene. Rifacendo anche il braccio della rete vecchia sugli stessi semi
il confronto v1 contro v2 diventa appaiato, ed e' l'unico modo di vedere una differenza di pochi
centimetri con trenta coppie.

  riferimento   Kalman puro, nessuna correzione                (condizione 'base')
  rete v1       correzione_kalman_specializzato.pt      +12,1% (condizione 'solo_ai')
  rete v2       correzione_kalman_specializzato_v2.pt   +20,7% (condizione 'solo_ai')

I due bracci con rete girano nella STESSA condizione del simulatore: cambia solo il file dei pesi,
scambiato dentro il worker prima della corsa. Nessun'altra differenza fra i due, per costruzione.

SEMI DISGIUNTI da entrambe le campagne precedenti: la prima usava gli indici 0-29, la replica
30-79, questa parte da 80. Nessuna scena ha mai suggerito l'ipotesi che qui si verifica.

SCALA CORRETTA. Il banco originale converte con 60 px/m, ereditati da quando una cella valeva
mezzo metro. La scala e' poi stata riancorata all'ingombro dei corpi: METRI_PER_CELLA = 1,0 su
DIM_NODO = 30, cioe' 30 px/m. Le distanze archiviate sono quindi DIMEZZATE rispetto alla scala
dichiarata in tesi. Qui si usa 30 px/m, e il prospetto riporta i numeri archiviati riscalati per
renderli confrontabili.

IPOTESI FISSATA PRIMA: la distanza media dalle persone in movimento. Le altre tre grandezze
restano nel prospetto come contesto, non come test.

Uso:
    py ai_predittiva/sicurezza_modello_v2.py              30 coppie x 3 bracci = 90 corse
    py ai_predittiva/sicurezza_modello_v2.py --semi=10    versione corta
    py ai_predittiva/sicurezza_modello_v2.py --tabella    ricalcola dal CSV, senza rifare le corse
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import time
import zlib
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import test_sicurezza_povo as ts
import allena_previsione as ap

PIXEL_PER_METRO = 30.0          # DIM_NODO = 30 px, METRI_PER_CELLA = 1,0
FATTORE_SCALA = ts.PIXEL_PER_METRO / PIXEL_PER_METRO   # dal banco originale alla scala vera
FILE_V2 = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "sicurezza_modello_v2_povo.csv")

NUMERO_SEMI = 30
PRIMO_SEME = 80                 # 0-29 prima campagna, 30-79 replica: questi non li ha visti nessuno
BRACCI = ("riferimento", "rete_v1", "rete_v2")
GRANDEZZE = ("distanza_media_m", "distanza_minima_m", "esposizione_percento", "jerk_medio")

_MODELLI = {}


def _inizializza():
    """Carica ENTRAMBE le reti una volta per worker. Su Windows il pool nasce con spawn, quindi
    questo codice gira dentro il processo figlio: assegnare i globali qui e' l'unico punto in cui
    l'assegnazione e' visibile alla corsa."""
    global _MODELLI
    t._inizializza_worker()                                  # carica la rete della tesi come sempre
    _MODELLI = {"rete_v1": t._modello_ai_worker, "rete_v2": ap.modello_da_file(FILE_V2)}


def _una_corsa(argomenti):
    braccio, seme, popolazione = argomenti
    # esegui_corsa legge il modello dal globale di testing: scambiarlo qui e' cio' che distingue i
    # due bracci con rete. Il braccio di riferimento non lo usa affatto (condizione 'base').
    if braccio in _MODELLI:
        t._modello_ai_worker = _MODELLI[braccio]
    condizione = "base" if braccio == "riferimento" else "solo_ai"
    _, _, arrivato, tempo_s, fermo_s, misure = ts._una_corsa((condizione, seme, popolazione))
    return braccio, seme, arrivato, tempo_s, fermo_s, misure


def _confronta(coppie, meglio_alto):
    """Media appaiata, errore standard e rapporto. Il segno e' sempre 'il secondo braccio meglio'."""
    differenze = [(b - a) if meglio_alto else (a - b) for a, b in coppie]
    n = len(differenze)
    media = statistics.mean(differenze)
    errore = statistics.stdev(differenze) / n ** 0.5 if n > 1 else float("inf")
    rapporto = abs(media / errore) if errore > 0 else float("inf")
    return media, errore, rapporto, sum(1 for d in differenze if d > 0), n


def _riporta(semi, esiti):
    righe = []
    valori = {b: {g: [] for g in GRANDEZZE} for b in BRACCI}
    tempi = {b: [] for b in BRACCI}
    for seme in semi:
        if any((seme, br) not in esiti for br in BRACCI):
            continue
        if not all(esiti[(seme, br)][0] for br in BRACCI):
            print(f"  seme {seme}: scena scartata, un robot non e' arrivato")
            continue
        riga = {"mappa": ts.MAPPA, "seme": seme}
        for br in BRACCI:
            _, tempo_s, fermo_s, misure = esiti[(seme, br)]
            riga[f"tempo_{br}_s"] = round(tempo_s, 2)
            riga[f"fermo_{br}_s"] = round(fermo_s, 2)
            tempi[br].append(tempo_s)
            for g in GRANDEZZE:
                v = misure[g] * (FATTORE_SCALA if g.startswith("distanza") else 1.0)
                valori[br][g].append(v)
                riga[f"{g}_{br}"] = round(v, 4)
        righe.append(riga)

    if not righe:
        print("Nessuna scena valida.")
        return

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    _prospetto(valori, tempi, len(righe))
    print(f"\nRighe per singola scena: {FILE_CSV}")


def _prospetto(valori, tempi, n):
    etichette = {"distanza_media_m": ("distanza media dalle persone (m)", True),
                 "distanza_minima_m": ("distanza minima (m)", True),
                 "esposizione_percento": ("frame sotto 0,5 m (%)", False),
                 "jerk_medio": ("jerk medio (m/s^3)", False)}
    confronti = (("A) rete v1 (previsione +12,1%) contro riferimento", "riferimento", "rete_v1"),
                 ("B) rete v2 (previsione +20,7%) contro riferimento", "riferimento", "rete_v2"),
                 ("C) rete v2 contro rete v1   <-- E' LA DOMANDA", "rete_v1", "rete_v2"))

    print("\n" + "=" * 98)
    print(f"SPAZIO GUADAGNATO DALLA CORREZIONE NEURALE - {n} scene appaiate, scala 30 px/m")
    print("=" * 98)
    for titolo, primo, secondo in confronti:
        print(f"\n{titolo}")
        print(f"  {'grandezza':<36}{'medie assolute':>20}{'differenza':>13}{'errore':>9}"
              f"{'rapp.':>8}{'vinte':>9}")
        for g, (nome, alto) in etichette.items():
            coppie = list(zip(valori[primo][g], valori[secondo][g]))
            media, errore, rapporto, vinte, nn = _confronta(coppie, alto)
            ma, mb = statistics.mean(valori[primo][g]), statistics.mean(valori[secondo][g])
            print(f"  {nome:<36}{ma:>9.3f} ->{mb:>8.3f}{media:>+13.4f}{errore:>9.4f}"
                  f"{rapporto:>8.2f}{vinte:>6}/{nn}")
        coppie_t = list(zip(tempi[primo], tempi[secondo]))
        media, errore, rapporto, vinte, nn = _confronta(coppie_t, False)
        print(f"  {'tempo di percorrenza (s)':<36}"
              f"{statistics.mean(tempi[primo]):>9.2f} ->{statistics.mean(tempi[secondo]):>8.2f}"
              f"{media:>+13.4f}{errore:>9.4f}{rapporto:>8.2f}{vinte:>6}/{nn}")

    print("\n  Il rapporto e' media/errore standard: sotto 1 indistinguibile da zero, sopra 2 solido.")
    print("  ARCHIVIO, riscalato a 30 px/m (rete v1 contro riferimento, semi diversi da questi):")
    print("    prima campagna  30 coppie  distanza media +0,072 m  rapporto 1,10")
    print("    replica         50 coppie  distanza media +0,077 m  rapporto 1,64")


def _da_csv():
    if not os.path.exists(FILE_CSV):
        print(f"{FILE_CSV} non trovato.")
        return
    with open(FILE_CSV, encoding="utf-8") as f:
        righe = list(csv.DictReader(f))
    valori = {b: {g: [float(r[f"{g}_{b}"]) for r in righe] for g in GRANDEZZE} for b in BRACCI}
    tempi = {b: [float(r[f"tempo_{b}_s"]) for r in righe] for b in BRACCI}
    _prospetto(valori, tempi, len(righe))


def main():
    numero = NUMERO_SEMI
    for a in sys.argv[1:]:
        if a.startswith("--semi="):
            numero = int(a.split("=", 1)[1])
    if "--tabella" in sys.argv:
        _da_csv()
        return
    if not os.path.exists(FILE_V2):
        print(f"manca {FILE_V2}: allenalo con allena_con_arresto.py")
        return

    popolazione = dict(ts.POPOLAZIONE)
    popolazione.update(t._geometria_salvata())
    semi = [zlib.crc32(f"{ts.MAPPA}|{ts.ETICHETTA_SEMI}|{i}".encode())
            for i in range(PRIMO_SEME, PRIMO_SEME + numero)]
    compiti = [(b, s, popolazione) for s in semi for b in BRACCI]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Spazio con rete v1, rete v2 e riferimento su {ts.MAPPA}, 0,70 pers/m2")
    print(f"  {numero} semi appaiati x 3 bracci = {len(compiti)} corse, {n_worker} processi, "
          f"accelerazione {t.ACCELERAZIONE}x\n")

    esiti, inizio, fatte = {}, time.time(), 0
    with mp.Pool(n_worker, initializer=_inizializza) as pool:
        for braccio, seme, arrivato, tempo_s, fermo_s, misure in pool.imap_unordered(
                _una_corsa, compiti, chunksize=1):
            fatte += 1
            esiti[(seme, braccio)] = (arrivato, tempo_s, fermo_s, misure)
            print(f"  [{fatte}/{len(compiti)}] {braccio:<11} seme {seme}: {tempo_s:.1f}s, "
                  f"media {misure['distanza_media_m'] * FATTORE_SCALA:.2f} m "
                  f"({(time.time() - inizio) / 60:.1f} min)", flush=True)

    print(f"\nCompletato in {(time.time() - inizio) / 60:.1f} min")
    _riporta(semi, esiti)


if __name__ == "__main__":
    main()

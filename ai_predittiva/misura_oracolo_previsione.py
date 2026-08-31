# -*- coding: utf-8 -*-
"""Quanto vale, per il pianificatore, una previsione perfetta del moto delle persone.

E' il TETTO del capitolo 5. La rete v2 toglie il 20,2% dell'errore di previsione e non cambia il
comportamento del robot; questo banco toglie il 100% dell'errore e misura cosa succede. Se non
succede niente nemmeno qui, il risultato negativo non e' un difetto della rete: e' una proprieta'
della catena, e nessun addestramento futuro potra' aggirarla passando per la mappa di costo.

DISEGNO. ENTRAMBI i bracci vengono eseguiti qui, appaiati sullo stesso seme: Kalman puro e
oracolo, stessa folla, stesso tragitto, stessa geometria. Due corse per seme.

PERCHE' NON SI RIUSA L'ARCHIVIO. risultati_test/confronto_ai_povo_d070.csv contiene gia' base e ai
per questi stessi semi, e appaiarcisi avrebbe dimezzato le corse. Ma quelle corse precedono la
planimetria realistica di Povo: sullo stesso seme davano 46,9 s dove oggi il Kalman ne da' 169,1.
Appaiare l'oracolo di oggi con il riferimento di allora produce un finto peggioramento di oltre
cento secondi, che e' interamente geometria e non previsione. Vedi E13 nel registro dei dati, dove
lo stesso salto (53 s -> 169 s) e' gia' documentato.

LA DOMANDA. ORACOLO contro KALMAN: quanto guadagnerebbe il robot se la previsione fosse esatta?
E' il tetto di qualunque ibridazione che passi per la macchia di probabilita'. Un confronto con la
rete, se servisse, va aggiunto come terzo braccio eseguito nella stessa campagna - non ripescato
dall'archivio.

IL NUMERO CHE SPIEGA IL PERCHE'. Oltre ai tempi si misura, a ogni previsione, se il centro esatto
cade nella STESSA CELLA di quello del Kalman. Quando ci cade, la macchia diffusa e' identica
bit per bit, quindi la mappa di costo e' identica, quindi il percorso non puo' cambiare: la
quantizzazione ha assorbito l'informazione prima che diventasse una decisione. E' la spiegazione
meccanica del risultato negativo, non una congettura su di esso, e si ottiene quasi gratis.

Si riporta anche lo scarto medio in metri fra previsione del Kalman e posizione vera: e' la stessa
grandezza che la rete riduce del 20,2%, misurata qui sulla scena reale invece che sul dataset.

Uso:
    py ai_predittiva/misura_oracolo_previsione.py --semi=6      # versione corta, per provare
    py ai_predittiva/misura_oracolo_previsione.py               # tutti i 50 semi in archivio
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import math
import time
import zlib
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_QUI = os.path.dirname(os.path.abspath(__file__))
for _percorso in (_QUI, _RADICE, os.path.join(_RADICE, "TEST"),
                  os.path.join(_RADICE, "ai_controllo")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import oracolo_previsione

MAPPA = "povo"
CONDIZIONE = "base"                  # Kalman puro, controlli spenti: l'oracolo sostituisce la sola macchia
ETICHETTA_SEMI = "confronto_ai_povo"  # invariata: stessi semi delle corse gia' in archivio
NUMERO_SEMI = 50

# identica alla serie confronto_ai_povo_d070: ~349 individui su 499 m2 = 0,70 pers/m2
POPOLAZIONE = {
    "numero_persone": 72,
    "numero_corridori": 66,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}

PIXEL_PER_METRO = 100.0 / sim.CM_PER_PIXEL
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "oracolo_previsione_povo_d070.csv")


def _una_corsa(argomenti):
    """Una corsa, con o senza oracolo, strumentata in entrambi i casi sullo scarto di previsione.

    Gli strumenti girano identici nei due bracci: l'oracolo viene comunque installato, e nel braccio
    di riferimento la sua previsione viene calcolata e misurata ma NON usata. Cosi' lo scarto, la
    coincidenza di cella e le cadute nel muro sono confrontabili fra i due, e il braccio Kalman resta
    esattamente la condizione 'base' - la sola cosa che cambia e' quale dei due valori viene tornato.
    """
    seed, popolazione, con_oracolo = argomenti

    scarti_px, stessa_cella, totale = [], 0, 0
    muro_kalman, muro_oracolo = 0, 0
    dim_robot = sim.DIM_NODO / t.FATTORE_GRIGLIA_DEFAULT
    fattore = t.FATTORE_GRIGLIA_DEFAULT
    righe_g, colonne_g = sim.Y_TOT * fattore, sim.X_TOT * fattore
    contenitore_griglia = []

    # la griglia non arriva all'aggancio della previsione: la si intercetta da _mappa_costo, che la
    # riceve come primo argomento e viene chiamata subito prima
    mappa_costo_originale = t._mappa_costo

    def _con_griglia(griglia, *resto, **chiavi):
        contenitore_griglia[:] = [griglia]
        return mappa_costo_originale(griglia, *resto, **chiavi)

    def _in_muro(x, y):
        if not contenitore_griglia:
            return None
        rf = max(0, min(righe_g - 1, int(y // dim_robot)))
        cf = max(0, min(colonne_g - 1, int(x // dim_robot)))
        return sim.sottocella_e_muro(contenitore_griglia[0], rf, cf, fattore)

    ripristina = oracolo_previsione.installa(t, sim)
    funzione_oracolo = sim.previsione_per_macchia

    def _con_scarto(traccia, frame_futuri):
        nonlocal stessa_cella, totale, muro_kalman, muro_oracolo
        x_o, y_o = funzione_oracolo(traccia, frame_futuri)
        if frame_futuri > 0:
            # stessa traccia, stesso istante: la differenza e' esattamente l'errore di previsione
            x_k, y_k = sim.previsione_posizione_kalman(traccia, frame_futuri)
            scarti_px.append(math.hypot(x_o - x_k, y_o - y_k))
            totale += 1
            # se i due centri cadono nella stessa cella, la macchia diffusa e' identica e nulla a
            # valle puo' accorgersi della differenza
            if (int(y_o // dim_robot), int(x_o // dim_robot)) == (int(y_k // dim_robot), int(x_k // dim_robot)):
                stessa_cella += 1
            # una previsione che cade in un muro non produce macchia: quella persona non pesa
            if _in_muro(x_k, y_k):
                muro_kalman += 1
            if _in_muro(x_o, y_o):
                muro_oracolo += 1
            if not con_oracolo:
                return x_k, y_k
        return x_o, y_o

    sim.previsione_per_macchia = _con_scarto
    t._mappa_costo = _con_griglia
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, CONDIZIONE, popolazione, seed)
    finally:
        t._mappa_costo = mappa_costo_originale
        ripristina()

    misure = {
        "scarto_medio_m": statistics.mean(scarti_px) / PIXEL_PER_METRO if scarti_px else 0.0,
        "scarto_max_m": max(scarti_px) / PIXEL_PER_METRO if scarti_px else 0.0,
        "stessa_cella_pct": stessa_cella / totale * 100 if totale else 0.0,
        "muro_kalman_pct": muro_kalman / totale * 100 if totale else 0.0,
        "muro_oracolo_pct": muro_oracolo / totale * 100 if totale else 0.0,
        "previsioni": totale,
    }
    return seed, con_oracolo, arrivato, t._secondi(frame), t._secondi(frame_fermo), misure

def main():
    numero_semi = NUMERO_SEMI
    for argomento in sys.argv[1:]:
        if argomento.startswith("--semi="):
            numero_semi = int(argomento.split("=", 1)[1])

    popolazione = dict(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(numero_semi)]
    # i due bracci sono dispacciati come corse singole e non a coppie: a parita' di seme la corsa e'
    # deterministica, quindi l'ordine e il processo non contano, e i core si riempiono meglio
    compiti = [(seme, popolazione, con_oracolo)
               for seme in semi for con_oracolo in (False, True)]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Macchia ORACOLO su {MAPPA}: quanto vale una previsione perfetta per il pianificatore")
    print(f"  {numero_semi} semi x 2 bracci = {len(compiti)} corse, entrambe eseguite adesso")
    print(f"  {n_worker} processi, accelerazione {t.ACCELERAZIONE}x\n")

    esiti, inizio = {}, time.time()
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for i, r in enumerate(pool.imap_unordered(_una_corsa, compiti, chunksize=1), 1):
            seed, con_oracolo, arrivato, tempo_s, fermo_s, misure = r
            esiti[(seed, con_oracolo)] = (arrivato, tempo_s, fermo_s, misure)
            stato = f"{tempo_s:.1f}s" if arrivato else "NON ARRIVATO"
            print(f"  [{i}/{len(compiti)}] seme {seed} / {'oracolo' if con_oracolo else 'kalman '}: "
                  f"{stato}, scarto {misure['scarto_medio_m']:.2f} m, "
                  f"stessa cella {misure['stessa_cella_pct']:.0f}%, "
                  f"nel muro K/O {misure['muro_kalman_pct']:.0f}%/{misure['muro_oracolo_pct']:.0f}% "
                  f"({time.time() - inizio:.0f}s)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta(semi, esiti)


def _riporta(semi, esiti):
    righe, differenze = [], {"tempo_s": [], "fermo_s": []}
    scarti, stesse_celle, muri_k, muri_o = [], [], [], []

    for seme in semi:
        kalman, oracolo = esiti.get((seme, False)), esiti.get((seme, True))
        if kalman is None or oracolo is None:
            continue
        if not (kalman[0] and oracolo[0]):
            print(f"  scartato: seme {seme}, almeno un braccio non arrivato")
            continue
        _, tempo_k, fermo_k, misure_k = kalman
        _, tempo_o, fermo_o, misure_o = oracolo
        differenze["tempo_s"].append(tempo_o - tempo_k)
        differenze["fermo_s"].append(fermo_o - fermo_k)
        # gli strumenti girano identici nei due bracci: si media sui due per non privilegiarne uno
        scarti.append((misure_k["scarto_medio_m"] + misure_o["scarto_medio_m"]) / 2)
        stesse_celle.append((misure_k["stessa_cella_pct"] + misure_o["stessa_cella_pct"]) / 2)
        muri_k.append(misure_k["muro_kalman_pct"])
        muri_o.append(misure_o["muro_oracolo_pct"])
        righe.append({"mappa": MAPPA, "seme": seme,
                      "tempo_kalman_s": round(tempo_k, 3), "tempo_oracolo_s": round(tempo_o, 3),
                      "differenza_s": round(tempo_o - tempo_k, 3),
                      "fermo_kalman_s": round(fermo_k, 3), "fermo_oracolo_s": round(fermo_o, 3),
                      "scarto_medio_m": round(misure_o["scarto_medio_m"], 4),
                      "stessa_cella_pct": round(misure_o["stessa_cella_pct"], 2),
                      "muro_kalman_pct": round(misure_k["muro_kalman_pct"], 2),
                      "muro_oracolo_pct": round(misure_o["muro_oracolo_pct"], 2)})

    if not righe:
        print("Nessuna coppia valida.")
        return

    print(f"Coppie valide: {len(righe)}")
    print(f"Errore di previsione eliminato dall'oracolo: {statistics.mean(scarti):.2f} m in media")
    print(f"Previsioni nella stessa cella del Kalman: {statistics.mean(stesse_celle):.1f}%")
    print("  (dove cadono nella stessa cella la mappa di costo e' identica: la decisione non puo' cambiare)")
    print(f"Previsioni che finiscono dentro un muro: Kalman {statistics.mean(muri_k):.1f}%, "
          f"oracolo {statistics.mean(muri_o):.1f}%")
    print("  (una previsione nel muro non produce macchia: quella persona sparisce dalla mappa di costo)\n")

    print("=== ORACOLO contro KALMAN   (il TETTO dell'ibridazione sulla previsione) ===")
    print(f"{'grandezza':>22}{'differenza':>13}{'SE':>9}{'rapporto':>11}{'vinte':>10}")
    print("-" * 65)
    for campo, etichetta in (("tempo_s", "tempo di percorrenza"), ("fermo_s", "tempo trascorso fermo")):
        d = differenze[campo]
        media = statistics.mean(d)
        se = statistics.stdev(d) / len(d) ** 0.5 if len(d) > 1 else float("nan")
        # rapporto fra effetto e sua incertezza: sotto 2 la differenza non e' distinguibile da zero
        rapporto = media / se if se and se == se and se > 0 else float("nan")
        vinte = sum(1 for x in d if x < 0)  # meno tempo = oracolo meglio
        print(f"{etichetta:>22}{media:>+13.2f}{se:>9.2f}{rapporto:>11.2f}{vinte:>7}/{len(d)}")
    print()

    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"Scritto {FILE_CSV}")


if __name__ == "__main__":
    mp.freeze_support()
    main()

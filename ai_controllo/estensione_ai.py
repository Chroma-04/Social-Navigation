"""Estende il confronto AI contro Kalman su semi nuovi, per portare l'effetto sopra il rumore.

PERCHE'. La prima campagna dava +12,3 cm di distanza con rapporto 2,30 su 24 coppie, la replica
su semi disgiunti -8,5 cm su 10 coppie: aggregate fanno +6,2 cm con rapporto 1,26, cioe' non
distinguibile da zero. Lo scarto tipico della differenza per coppia e' di circa 29 cm, venti volte
l'effetto cercato: con poche decine di coppie una campagna singola puo' uscire a 12 cm o a -8 cm
senza che la differenza voglia dire nulla. Per portare 6 cm sopra rapporto 2 servono circa 90
coppie, e questo banco serve ad arrivarci.

ESITO, a campagna conclusa: la stima dello scarto era ottimistica. Sulle 82 coppie complessive
vale 0,428 m e non 0,29, quindi per 6 cm servirebbero circa 200 coppie e non 90. Il punto e'
comunque risolto in altro modo: l'aggregato da -0,027 m con rapporto 0,57 e intervallo al 95%
fra -0,120 e +0,066 m, cioe' esclude qualunque guadagno superiore a 6,6 cm. Non serve piu'
campione per stabilire un effetto che l'intervallo confina sotto il valore d'interesse.

I semi sono nuovi e disgiunti da entrambe le campagne precedenti, e qui vanno eseguiti ENTRAMBI i
bracci: le corse col Kalman puro su questi semi non esistono in archivio, quindi non c'e' nulla da
riusare come si faceva prima.

    py ai_controllo/estensione_ai.py            6 semi x 8 livelli = 48 coppie (96 corse)
    py ai_controllo/estensione_ai.py --semi=3   versione corta
    py ai_controllo/estensione_ai.py --prova    elenca i compiti e non simula nulla
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
for _percorso in (_RADICE, os.path.join(_RADICE, "ibridazione_velocita"),
                  os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import confronto_controlli as cc
import ai_velocita as av

CONDIZIONE = "velocita_libera"
ETICHETTA_SEMI = "ai_estensione"   # disgiunta da 'ibridazione_velocita' e dai semi della replica
SEMI_DEFAULT = 6
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "ai_velocita_estensione_povo.csv")


def _corsa(argomenti):
    """Una corsa, col Kalman puro o con la rete innestata a seconda del braccio.

    Le intercettazioni si installano QUI: con lo spawn di Windows il worker re-importa i moduli da
    zero e non vedrebbe una patch fatta nel processo padre."""
    braccio, etichetta, seme, popolazione = argomenti
    ripristina = None
    if braccio == "ai":
        if t._modello_ai_worker is None:
            t._inizializza_worker()
        orizzonte = max(1, sim.PREVISIONE_BLOB_FRAME // t.ACCELERAZIONE)
        ripristina = av.installa(t, sim, t._modello_ai_worker, orizzonte)

    traiettoria, distanze = [], []
    traccia_precedente = t._rileva_e_traccia

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
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
        return traccia_precedente(tutte_le_persone, robot_x, robot_y, griglia,
                                  tracciamento_lidar, storico_posizioni)

    t._rileva_e_traccia = _con_misure
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", CONDIZIONE, popolazione, seme)
    finally:
        t._rileva_e_traccia = traccia_precedente
        if ripristina is not None:
            ripristina()

    tempo_s = t._secondi(frame)
    return braccio, etichetta, seme, tempo_s, t._secondi(frame_fermo), \
        cc._metriche(traiettoria, distanze, tempo_s)


def principale():
    numero_semi = SEMI_DEFAULT
    for argomento in sys.argv[1:]:
        if argomento.startswith("--semi="):
            numero_semi = int(argomento.split("=", 1)[1])

    # gli 8 livelli con campionamento solido delle campagne precedenti, per restare confrontabili
    livelli = [(f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * cc.FATTORI[i])}",
                cc._popolazione_scalata(cc.FATTORI[i])) for i in range(8)]
    semi = [zlib.crc32(f"povo|{ETICHETTA_SEMI}|{i}".encode()) for i in range(numero_semi)]
    compiti = [(braccio, etichetta, seme, popolazione)
               for etichetta, popolazione in livelli for seme in semi
               for braccio in ("kalman", "ai")]

    print(f"Estensione AI contro Kalman su semi nuovi")
    print(f"  {len(livelli)} livelli x {numero_semi} semi x 2 bracci = {len(compiti)} corse "
          f"({len(compiti)//2} coppie)")
    if "--prova" in sys.argv:
        for c in compiti[:10]:
            print("  ", c[0], c[1], c[2])
        return

    avvio = time.time()
    raccolta = {}
    with mp.Pool(min(8, os.cpu_count() or 4), initializer=t._inizializza_worker) as pool:
        for n, esito in enumerate(pool.imap_unordered(_corsa, compiti), 1):
            braccio, etichetta, seme, tempo_s, fermo_s, metriche = esito
            raccolta[(etichetta, seme, braccio)] = (tempo_s, fermo_s, metriche)
            print(f"  [{n}/{len(compiti)}] {braccio:>6} {etichetta} seme {seme}: {tempo_s:.1f}s "
                  f"({(time.time() - avvio)/60:.1f} min)", flush=True)

    campi = ["mappa", "livello", "seme",
             "tempo_s_kalman", "tempo_s_ai", "fermo_s_kalman", "fermo_s_ai",
             "numero_fermate_kalman", "numero_fermate_ai",
             "distanza_media_m_kalman", "distanza_media_m_ai",
             "velocita_media_m_s_kalman", "velocita_media_m_s_ai"]
    differenze = []
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=campi)
        scrittore.writeheader()
        for etichetta, _ in livelli:
            for seme in semi:
                k = raccolta.get((etichetta, seme, "kalman"))
                a = raccolta.get((etichetta, seme, "ai"))
                if k is None or a is None:
                    continue
                scrittore.writerow({
                    "mappa": "povo", "livello": etichetta, "seme": seme,
                    "tempo_s_kalman": round(k[0], 4), "tempo_s_ai": round(a[0], 4),
                    "fermo_s_kalman": round(k[1], 4), "fermo_s_ai": round(a[1], 4),
                    "numero_fermate_kalman": k[2]["numero_fermate"],
                    "numero_fermate_ai": a[2]["numero_fermate"],
                    "distanza_media_m_kalman": round(k[2]["distanza_media_m"], 4),
                    "distanza_media_m_ai": round(a[2]["distanza_media_m"], 4),
                    "velocita_media_m_s_kalman": round(k[2]["velocita_media_m_s"], 4),
                    "velocita_media_m_s_ai": round(a[2]["velocita_media_m_s"], 4),
                })
                differenze.append(a[2]["distanza_media_m"] - k[2]["distanza_media_m"])

    print(f"\nScritto {FILE_CSV}  ({(time.time() - avvio)/60:.1f} min totali)")
    if len(differenze) > 1:
        media = statistics.mean(differenze)
        errore = statistics.stdev(differenze) / math.sqrt(len(differenze))
        print(f"distanza, solo questa estensione: {media:+.3f} m  errore standard {errore:.3f}  "
              f"rapporto {abs(media)/errore:.2f}  su {len(differenze)} coppie")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

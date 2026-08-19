"""Il controllo di POSIZIONE da solo: velocita' costante, ma il robot puo' traslare di lato.

Finora la posizione e' stata misurata soltanto in 'libera_posizione', cioe' accesa insieme al
controllo di velocita': in quella condizione non e' possibile attribuire a uno dei due i +28 cm di
distanza osservati, e il campionamento era di un seme per livello. Qui la posizione e' l'unica cosa
accesa, quindi la differenza rispetto al riferimento e' sua per costruzione.

Riusa le corse di 'base' gia' in archivio invece di rifarle: sono deterministiche a parita' di
seme, e rieseguirle costerebbe il doppio del calcolo per riottenere gli stessi numeri.

    py ibridazione_velocita/solo_posizione.py            8 livelli x 3 semi  (24 corse)
    py ibridazione_velocita/solo_posizione.py --prova    stampa l'appaiamento e non simula nulla
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
for _percorso in (_QUI, _RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import confronto_controlli as cc

CONDIZIONE = "solo_posizione"
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "solo_posizione_povo.csv")
FILE_BASE = os.path.join(t.CARTELLA_RISULTATI, "confronto_completo_velocita_libera.csv")


def _corsa(argomenti):
    """Una corsa strumentata. Le intercettazioni si installano QUI: con lo spawn di Windows il
    worker re-importa i moduli da zero e non vedrebbe una patch fatta nel processo padre."""
    etichetta, seme, popolazione = argomenti
    traiettoria, distanze = [], []
    traccia_originale = t._rileva_e_traccia

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
        return traccia_originale(tutte_le_persone, robot_x, robot_y, griglia,
                                 tracciamento_lidar, storico_posizioni)

    t._rileva_e_traccia = _con_misure
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", CONDIZIONE, popolazione, seme)
    finally:
        t._rileva_e_traccia = traccia_originale

    tempo_s = t._secondi(frame)
    return etichetta, seme, arrivato, tempo_s, t._secondi(frame_fermo), \
        cc._metriche(traiettoria, distanze, tempo_s)


def _corse_base():
    """Le colonne _base delle campagne gia' fatte, indicizzate per (livello, seme)."""
    per_chiave = {}
    with open(FILE_BASE, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            per_chiave[(r["livello"], int(r["seme"]))] = r
    return per_chiave


def principale():
    livelli = [(f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}", cc._popolazione_scalata(f))
               for f in cc.FATTORI]
    semi = [zlib.crc32(f"povo|{cc.ETICHETTA_SEMI}|{i}".encode()) for i in range(cc.NUMERO_SEMI)]
    base = _corse_base()

    compiti = [(etichetta, seme, popolazione)
               for etichetta, popolazione in livelli for seme in semi
               if (etichetta, seme) in base]
    print(f"{len(compiti)} corse appaiate con altrettante corse di base gia' in archivio")
    if "--prova" in sys.argv:
        for etichetta, seme, _ in compiti:
            print(f"  {etichetta:>8} seme {seme:>12}  base {base[(etichetta, seme)]['tempo_s_base']:>9} s")
        return

    avvio = time.time()
    risultati = []
    with mp.Pool(min(8, os.cpu_count() or 4)) as pool:
        for n, esito in enumerate(pool.imap_unordered(_corsa, compiti), 1):
            risultati.append(esito)
            print(f"  [{n}/{len(compiti)}] {esito[0]} seme {esito[1]}: {esito[3]:.1f}s "
                  f"({(time.time() - avvio)/60:.1f} min trascorsi)", flush=True)

    campi = ["mappa", "livello", "seme", "condizione",
             "tempo_s_base", "tempo_s_variante", "fermo_s_base", "fermo_s_variante",
             "numero_fermate_base", "numero_fermate_variante",
             "distanza_media_m_base", "distanza_media_m_variante",
             "velocita_media_m_s_base", "velocita_media_m_s_variante"]
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=campi)
        scrittore.writeheader()
        for etichetta, seme, arrivato, tempo_s, fermo_s, metriche in risultati:
            b = base[(etichetta, seme)]
            scrittore.writerow({
                "mappa": "povo", "livello": etichetta, "seme": seme, "condizione": CONDIZIONE,
                "tempo_s_base": b["tempo_s_base"], "tempo_s_variante": round(tempo_s, 4),
                "fermo_s_base": b["fermo_s_base"], "fermo_s_variante": round(fermo_s, 4),
                "numero_fermate_base": b["numero_fermate_base"],
                "numero_fermate_variante": metriche["numero_fermate"],
                "distanza_media_m_base": b["distanza_media_m_base"],
                "distanza_media_m_variante": round(metriche["distanza_media_m"], 4),
                "velocita_media_m_s_base": b["velocita_media_m_s_base"],
                "velocita_media_m_s_variante": round(metriche["velocita_media_m_s"], 4),
            })
    print(f"\nScritto {FILE_CSV}  ({(time.time() - avvio)/60:.1f} min totali)")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

"""Il riferimento a velocita' costante corre a 1 m/s, il controllo puo' sprintare fino a 2.

I -17,6 s misurati da velocita_libera contro base sono quindi ambigui: possono venire dal
controllo, che sa quando puo' permettersi di correre, oppure soltanto dal tetto di velocita' piu'
alto. Le due spiegazioni si separano con un solo confronto: un riferimento SENZA alcun controllo
che viaggi a 2 m/s costanti, cioe' alla stessa velocita' di punta della variante.

  - se il riferimento veloce arriva nello stesso tempo, il guadagno era il tetto e non il controllo;
  - se arriva piu' tardi perche' si ferma di piu', il controllo si merita il guadagno: la velocita'
    di punta da sola non basta, serve sapere quando usarla.

Non produce le corse di base e velocita_libera: quelle esistono gia' in
confronto_completo_velocita_libera.csv e si riusano appaiate per (livello, seme), altrimenti si
spenderebbe il doppio del calcolo per riottenere numeri identici - le corse sono deterministiche a
parita' di seme.

    py ibridazione_velocita/riferimento_veloce.py            4 livelli x 3 semi  (12 corse)
    py ibridazione_velocita/riferimento_veloce.py --completo 8 livelli x 3 semi  (24 corse)
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
import main as sim
import confronto_controlli as cc

MOLTIPLICATORE_RIFERIMENTO = 2.0  # = FATTORE_VELOCITA_ROBOT_MAX: stessa velocita' di punta
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "riferimento_veloce_povo.csv")
FILE_ESISTENTI = os.path.join(t.CARTELLA_RISULTATI, "confronto_completo_velocita_libera.csv")


def _corsa_veloce(argomenti):
    """Una corsa del riferimento a velocita' doppia.

    La modifica a VELOCITA_ROBOT si applica QUI e non nel processo padre: con lo spawn di Windows
    il worker re-importa main da zero e non vedrebbe una patch fatta prima del fork."""
    etichetta, seme, popolazione = argomenti
    sim.VELOCITA_ROBOT = sim.VELOCITA_ROBOT * MOLTIPLICATORE_RIFERIMENTO

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
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", "base", popolazione, seme)
    finally:
        t._rileva_e_traccia = traccia_originale

    tempo_s = t._secondi(frame)
    return etichetta, seme, arrivato, tempo_s, t._secondi(frame_fermo), \
        cc._metriche(traiettoria, distanze, tempo_s)


def _righe_esistenti():
    """base e velocita_libera gia' misurate, indicizzate per (livello, seme)."""
    per_chiave = {}
    with open(FILE_ESISTENTI, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if r["condizione"] == "velocita_libera":
                per_chiave[(r["livello"], int(r["seme"]))] = r
    return per_chiave


def principale():
    completo = "--completo" in sys.argv
    indici = range(len(cc.FATTORI)) if completo else (1, 3, 5, 7)
    livelli = [(f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * cc.FATTORI[i])}",
                cc._popolazione_scalata(cc.FATTORI[i])) for i in indici]
    semi = [zlib.crc32(f"povo|{cc.ETICHETTA_SEMI}|{i}".encode()) for i in range(cc.NUMERO_SEMI)]
    esistenti = _righe_esistenti()

    compiti = [(etichetta, seme, popolazione)
               for etichetta, popolazione in livelli for seme in semi
               if (etichetta, seme) in esistenti]
    mancanti = len(livelli) * len(semi) - len(compiti)
    if mancanti:
        print(f"ATTENZIONE: {mancanti} combinazioni non hanno una corsa gia' misurata con cui "
              f"appaiarsi, e sono state saltate")

    print(f"Riferimento a {MOLTIPLICATORE_RIFERIMENTO:.0f}x la velocita' nominale, contro le corse "
          f"gia' in archivio")
    print(f"  {len(livelli)} livelli x {len(semi)} semi = {len(compiti)} corse")

    avvio = time.time()
    risultati = []
    with mp.Pool(min(8, os.cpu_count() or 4)) as pool:
        for n, esito in enumerate(pool.imap_unordered(_corsa_veloce, compiti), 1):
            risultati.append(esito)
            print(f"  [{n}/{len(compiti)}] {esito[0]} seme {esito[1]}: {esito[3]:.1f}s "
                  f"({time.time() - avvio:.0f}s trascorsi)", flush=True)

    campi = ["mappa", "livello", "seme", "tempo_s_riferimento_veloce", "tempo_s_velocita_libera",
             "fermo_s_riferimento_veloce", "fermo_s_velocita_libera",
             "numero_fermate_riferimento_veloce", "numero_fermate_velocita_libera",
             "distanza_media_m_riferimento_veloce", "distanza_media_m_velocita_libera",
             "velocita_media_m_s_riferimento_veloce", "velocita_media_m_s_velocita_libera"]
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=campi)
        scrittore.writeheader()
        for etichetta, seme, arrivato, tempo_s, fermo_s, metriche in risultati:
            vecchia = esistenti[(etichetta, seme)]
            scrittore.writerow({
                "mappa": "povo", "livello": etichetta, "seme": seme,
                "tempo_s_riferimento_veloce": round(tempo_s, 4),
                "tempo_s_velocita_libera": vecchia["tempo_s_variante"],
                "fermo_s_riferimento_veloce": round(fermo_s, 4),
                "fermo_s_velocita_libera": vecchia["fermo_s_variante"],
                "numero_fermate_riferimento_veloce": metriche["numero_fermate"],
                "numero_fermate_velocita_libera": vecchia["numero_fermate_variante"],
                "distanza_media_m_riferimento_veloce": round(metriche["distanza_media_m"], 4),
                "distanza_media_m_velocita_libera": vecchia["distanza_media_m_variante"],
                "velocita_media_m_s_riferimento_veloce": round(metriche["velocita_media_m_s"], 4),
                "velocita_media_m_s_velocita_libera": vecchia["velocita_media_m_s_variante"],
            })

    print(f"\nScritto {FILE_CSV}")
    diff = [r[3] - float(esistenti[(r[0], r[1])]["tempo_s_variante"]) for r in risultati]
    media = statistics.mean(diff)
    errore = statistics.stdev(diff) / math.sqrt(len(diff)) if len(diff) > 1 else float("nan")
    print(f"\nTempo: riferimento veloce meno controllo = {media:+.2f} s "
          f"(errore standard {errore:.2f}, rapporto {abs(media)/errore:.2f})")
    print(f"  corse in cui il CONTROLLO e' piu' rapido: "
          f"{sum(1 for d in diff if d > 0)}/{len(diff)}")
    print("\nSegno positivo = il controllo batte un riferimento che ha la stessa velocita' di punta,")
    print("quindi il guadagno non e' il tetto di velocita'. Segno nullo = era il tetto.")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

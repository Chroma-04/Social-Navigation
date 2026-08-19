"""La correzione neurale portata DENTRO il controllo di velocita'.

Oggi la rete non tocca il controllo di velocita': in testing.py la correzione entra solo in
_mappa_costo, cioe' sulle macchie di rischio che alimentano il pianificatore. Il controllo di
velocita' legge sim.previsione_per_controllo, che resta il Kalman puro anche nella condizione
'completo'. Questo modulo chiude quel buco: sostituisce la sorgente di previsione del solo
controllo di velocita' con Kalman + correzione appresa, e lascia tutto il resto identico.

Confronto prodotto: velocita_libera con Kalman  contro  velocita_libera con Kalman+rete, sugli
stessi semi. La differenza e' quindi attribuibile alla sola correzione.

DUE LIMITI DA DICHIARARE, perche' incidono sulla lettura del risultato.

  ORIZZONTE. La rete e' addestrata su un orizzonte fisso (quello della macchia di probabilita',
  riscalato dall'accelerazione). Il controllo di velocita' invece interroga la previsione a
  orizzonti VARIABILI, perche' chiede "dove sara' la persona quando io arrivo li'". La
  correzione viene percio' riscalata linearmente con il rapporto fra i due orizzonti e limitata
  a una volta se stessa: e' un'approssimazione, non una previsione addestrata a quell'orizzonte.

  DISTRIBUZIONE. Il modello in archivio e' stato addestrato prima dei due cambi di cinematica
  (velocita' del robot e dei pedoni). Finche' non viene riaddestrato lavora fuori distribuzione,
  e un esito nullo va letto come "questo modello non aiuta", non come "nessun modello aiuterebbe".

    py ai_controllo/ai_velocita.py            8 livelli x 3 semi  (24 corse)
    py ai_controllo/ai_velocita.py --prova    verifica l'appaiamento e non simula nulla
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

import numpy as np
import torch

import testing as t
import main as sim
import confronto_controlli as cc
from allena_previsione import prepara_input, FINESTRA_STORICO_FRAME

CONDIZIONE = "velocita_libera"   # il controllo su cui si innesta la rete
# --replica gira sui semi della seconda campagna, DISGIUNTI dai primi: un effetto appena sopra
# soglia (qui la distanza, rapporto 2,30) va replicato su semi indipendenti prima di dichiararlo.
# E' lo stesso metodo con cui in tesi e' caduta l'esposizione alla prossimita', che a 2,35 sembrava
# solida e in replica e' scesa a 1,07.
_REPLICA = "--replica" in sys.argv
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI,
                        "ai_velocita_replica_povo.csv" if _REPLICA else "ai_velocita_povo.csv")
FILE_ARCHIVIO = os.path.join(t.CARTELLA_RISULTATI,
                             "confronto_replica_velocita_libera.csv" if _REPLICA
                             else "confronto_completo_velocita_libera.csv")

# corrections calcolate una volta per fotogramma per tutte le tracce insieme: il controllo di
# velocita' interroga la previsione molte volte per fotogramma (una per traccia e per punto
# campionato del percorso), e una inferenza per chiamata renderebbe la corsa inutilizzabile
_CORREZIONI = {}
_STORICO = {}


def installa(testing, simulatore, modello, orizzonte_addestramento):
    """Sostituisce la sorgente di previsione del solo controllo di velocita'.

    Va chiamata DENTRO il processo che esegue la corsa: con lo spawn di Windows i worker
    re-importano i moduli da zero e non vedrebbero una patch fatta nel processo padre.

    Ritorna una funzione che ripristina lo stato precedente."""
    traccia_originale = testing._rileva_e_traccia
    previsione_originale = simulatore.previsione_per_controllo

    def _con_correzioni(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                        storico_posizioni):
        risultato = traccia_originale(tutte_le_persone, robot_x, robot_y, griglia,
                                      tracciamento_lidar, storico_posizioni)
        # la traccia non porta con se' la propria chiave: la si scrive dentro, cosi' la
        # previsione corretta puo' risalire allo storico della persona che rappresenta
        for pid, traccia in tracciamento_lidar.items():
            traccia["pid"] = pid

        _CORREZIONI.clear()
        _STORICO.clear()
        _STORICO.update(storico_posizioni)

        pid_ok, storici_ok = [], []
        for pid, traccia in tracciamento_lidar.items():
            if traccia.get("ferma", False):
                continue  # le persone ferme non vengono proiettate, quindi non c'e' nulla da correggere
            storico = storico_posizioni.get(pid)
            if storico is not None and len(storico) == FINESTRA_STORICO_FRAME:
                pid_ok.append(pid)
                storici_ok.append(storico)
        if pid_ok:
            batch = prepara_input(np.array(storici_ok, dtype=np.float32))
            with torch.no_grad():
                uscite = modello(torch.from_numpy(batch)).numpy()
            _CORREZIONI.update(zip(pid_ok, uscite))
        return risultato

    def _previsione_corretta(traccia, frame_futuri):
        base_x, base_y = previsione_originale(traccia, frame_futuri)
        correzione = _CORREZIONI.get(traccia.get("pid"))
        if correzione is None:
            return base_x, base_y
        # la rete corregge a un orizzonte fisso; qui l'orizzonte varia con il punto del percorso
        # a cui il robot arriverebbe. Si riscala linearmente e si limita a 1: estrapolare una
        # correzione oltre l'orizzonte su cui e' stata addestrata amplificherebbe l'errore
        scala = min(1.0, frame_futuri / orizzonte_addestramento) if orizzonte_addestramento else 1.0
        return base_x + float(correzione[0]) * scala, base_y + float(correzione[1]) * scala

    testing._rileva_e_traccia = _con_correzioni
    simulatore.previsione_per_controllo = _previsione_corretta

    def ripristina():
        testing._rileva_e_traccia = traccia_originale
        simulatore.previsione_per_controllo = previsione_originale
        _CORREZIONI.clear()
        _STORICO.clear()

    return ripristina


def _corsa(argomenti):
    """Una corsa di velocita_libera con la rete innestata nel controllo di velocita'."""
    etichetta, seme, popolazione = argomenti
    if t._modello_ai_worker is None:
        t._inizializza_worker()
    modello = t._modello_ai_worker
    orizzonte = max(1, sim.PREVISIONE_BLOB_FRAME // t.ACCELERAZIONE)

    traiettoria, distanze = [], []
    ripristina = installa(t, sim, modello, orizzonte)
    traccia_con_correzioni = t._rileva_e_traccia

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
        return traccia_con_correzioni(tutte_le_persone, robot_x, robot_y, griglia,
                                      tracciamento_lidar, storico_posizioni)

    t._rileva_e_traccia = _con_misure
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", CONDIZIONE, popolazione, seme)
    finally:
        ripristina()

    tempo_s = t._secondi(frame)
    return etichetta, seme, arrivato, tempo_s, t._secondi(frame_fermo), \
        cc._metriche(traiettoria, distanze, tempo_s)


def _archivio():
    """Le corse di velocita_libera gia' fatte col Kalman puro, per (livello, seme)."""
    per_chiave = {}
    with open(FILE_ARCHIVIO, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if r["condizione"] == CONDIZIONE:
                per_chiave[(r["livello"], int(r["seme"]))] = r
    return per_chiave


def principale():
    # i semi si leggono DALL'ARCHIVIO invece di rigenerarli: le due campagne li hanno prodotti con
    # ricette diverse (semi comuni la prima, un seme per livello la seconda) e rigenerarli qui
    # significherebbe appaiare corse che non condividono la scena
    livelli = {f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}": cc._popolazione_scalata(f)
               for f in cc.FATTORI}
    archivio = _archivio()
    compiti = [(etichetta, seme, livelli[etichetta])
               for etichetta, seme in sorted(archivio, key=lambda k: (int(k[0][3:]), k[1]))
               if etichetta in livelli]

    print(f"Rete innestata nel controllo di velocita', contro le stesse corse col Kalman puro")
    print(f"  {len(compiti)} corse appaiate")
    if "--prova" in sys.argv:
        for etichetta, seme, _ in compiti:
            print(f"  {etichetta:>8} seme {seme:>12}  kalman {archivio[(etichetta, seme)]['tempo_s_variante']:>9} s")
        return

    avvio = time.time()
    risultati = []
    with mp.Pool(min(8, os.cpu_count() or 4), initializer=t._inizializza_worker) as pool:
        for n, esito in enumerate(pool.imap_unordered(_corsa, compiti), 1):
            risultati.append(esito)
            print(f"  [{n}/{len(compiti)}] {esito[0]} seme {esito[1]}: {esito[3]:.1f}s "
                  f"({(time.time() - avvio)/60:.1f} min trascorsi)", flush=True)

    campi = ["mappa", "livello", "seme",
             "tempo_s_kalman", "tempo_s_ai", "fermo_s_kalman", "fermo_s_ai",
             "numero_fermate_kalman", "numero_fermate_ai",
             "distanza_media_m_kalman", "distanza_media_m_ai",
             "velocita_media_m_s_kalman", "velocita_media_m_s_ai"]
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=campi)
        scrittore.writeheader()
        for etichetta, seme, arrivato, tempo_s, fermo_s, metriche in risultati:
            k = archivio[(etichetta, seme)]
            scrittore.writerow({
                "mappa": "povo", "livello": etichetta, "seme": seme,
                "tempo_s_kalman": k["tempo_s_variante"], "tempo_s_ai": round(tempo_s, 4),
                "fermo_s_kalman": k["fermo_s_variante"], "fermo_s_ai": round(fermo_s, 4),
                "numero_fermate_kalman": k["numero_fermate_variante"],
                "numero_fermate_ai": metriche["numero_fermate"],
                "distanza_media_m_kalman": k["distanza_media_m_variante"],
                "distanza_media_m_ai": round(metriche["distanza_media_m"], 4),
                "velocita_media_m_s_kalman": k["velocita_media_m_s_variante"],
                "velocita_media_m_s_ai": round(metriche["velocita_media_m_s"], 4),
            })
    print(f"\nScritto {FILE_CSV}  ({(time.time() - avvio)/60:.1f} min totali)")


if __name__ == "__main__":
    mp.freeze_support()
    principale()

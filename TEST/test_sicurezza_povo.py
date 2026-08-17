"""L'ibridazione paga in sicurezza e fluidita' invece che in tempo?

Le campagne precedenti misurano solo il tempo, ed e' la metrica su cui in letteratura nessuno riporta
guadagni. Nei sistemi che ibridano un predittore appreso dentro un pianificatore, il guadagno di
accuratezza si converte in MARGINE: Social-Implicit dentro un MPC (arXiv 2508.07079) riporta +41,1% di
distanza minima dai pedoni (0,26 contro 0,19 m) e -28,4% di jerk, pagandoli con tempi di navigazione dal
13,8% al 25,5% piu' lunghi. La cautela costa tempo, e il guadagno di previsione la finanzia.

Questo banco misura le stesse grandezze sul sistema della tesi:

  - distanza minima robot-persona sull'intera corsa, e media dei minimi per frame. E' la metrica di
    sicurezza del confronto in letteratura;
  - esposizione: frazione di frame passati sotto mezzo metro da una persona in movimento;
  - jerk medio, cioe' la derivata terza della posizione del robot: la stessa misura di fluidita' del
    paper, ricavata dalla traiettoria effettivamente percorsa.

Le persone FERME sono escluse dalle distanze: il robot le rasenta per costruzione (l'alone le circonda
ma non impedisce il passaggio) e non sono cio' che la metrica di sicurezza vuole catturare. Contano i
corpi in movimento, gli unici che possono produrre un incontro non previsto.

Le due letture possibili, entrambe conclusive:
  - l'AI aumenta la distanza minima -> il compromesso della letteratura e' replicato su planimetria
    reale, e la tesi puo' concludere che l'ibridazione paga in sicurezza e non in efficienza;
  - non cambia nulla -> il guadagno di previsione (6 cm) non si converte in NULLA, ne' in tempo ne' in
    cautela, perche' e' inferiore alla risoluzione decisionale del pianificatore (0,25 m). E' un
    risultato che il lavoro citato non puo' osservare, avendo un guadagno molto sopra quella soglia.

Aggancio: _rileva_e_traccia riceve a ogni frame la posizione del robot e tutte le persone, quindi da li'
si ricavano sia le distanze sia la traiettoria del robot, e dalla traiettoria le derivate. Nessuna
modifica a testing.py, che e' condiviso col generatore di dataset.

Uso:
    py TEST/test_sicurezza_povo.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import time
import math
import zlib
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.dirname(os.path.abspath(__file__)),
                  os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim

MAPPA = "povo"
NUMERO_SEMI = 50
CONDIZIONI = ("base", "solo_ai")
ETICHETTA_SEMI = "confronto_ai_povo"

# Da quale indice partire nella successione dei semi. La prima campagna (30 semi, indici 0-29) ha
# prodotto l'ipotesi: l'esposizione alla prossimita' cala del 6,5%, rapporto 2,35, al limite della
# soglia una volta corretta per le quattro metriche osservate. Confermarla riusando quegli stessi semi
# varrebbe poco - si validerebbe un'ipotesi sui dati che l'hanno suggerita. Partendo da 30 i 50 semi
# sono DISGIUNTI dai precedenti: se l'effetto ricompare su dati che non l'hanno mai suggerito e'
# una replica indipendente, ed e' l'unica cosa che permette di scriverlo senza riserve.
# L'ipotesi da verificare e' UNA SOLA e fissata in anticipo: la frazione di frame sotto 0,5 m. Le
# altre tre metriche restano nel prospetto come contesto, non come test.
PRIMO_SEME = 30

# identica alla serie confronto_ai_povo_d070: ~349 individui su 499 m2 = 0,70 pers/m2
POPOLAZIONE = {
    "numero_persone": 72,
    "numero_corridori": 66,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}

PIXEL_PER_METRO = 60.0          # 1 cella = 30 px = 0,5 m
SOGLIA_PROSSIMITA_M = 0.5       # sotto questa distanza si conta l'esposizione
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "sicurezza_povo_d070_replica.csv")


def _una_corsa(argomenti):
    condizione, seed, popolazione = argomenti

    originale = t._rileva_e_traccia
    distanze = []      # minimo per frame, in px
    traiettoria = []   # posizione del robot per frame, in px

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
        minimo = None
        for p in tutte_le_persone:
            if p.get("stato", "attesa") == "attesa":
                continue  # le persone ferme non contano: vedi nota in testa al file
            d = math.hypot(p["x"] - robot_x, p["y"] - robot_y)
            if minimo is None or d < minimo:
                minimo = d
        if minimo is not None:
            distanze.append(minimo)
        traiettoria.append((robot_x, robot_y))
        return originale(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)

    t._rileva_e_traccia = _con_misure
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, condizione, popolazione, seed)
    finally:
        t._rileva_e_traccia = originale

    misure = _metriche(distanze, traiettoria)
    return condizione, seed, arrivato, t._secondi(frame), t._secondi(frame_fermo), misure


def _metriche(distanze, traiettoria):
    """Distanze in metri e jerk in m/s^3 dalla traiettoria grezza in pixel."""
    if not distanze:
        return {"distanza_minima_m": 0.0, "distanza_media_m": 0.0, "esposizione_percento": 0.0,
                "jerk_medio": 0.0}
    minime = [d / PIXEL_PER_METRO for d in distanze]
    soglia_px = SOGLIA_PROSSIMITA_M * PIXEL_PER_METRO
    esposizione = sum(1 for d in distanze if d < soglia_px) / len(distanze) * 100

    # jerk: derivata terza della posizione. dt e' il passo temporale equivalente a velocita' normale,
    # coerente con _secondi (ogni frame simulato vale ACCELERAZIONE frame reali)
    dt = t._secondi(1)
    jerk = 0.0
    if len(traiettoria) >= 4 and dt > 0:
        moduli = []
        for i in range(3, len(traiettoria)):
            # differenze finite del terzo ordine sulle ultime quattro posizioni
            jx = (traiettoria[i][0] - 3 * traiettoria[i - 1][0]
                  + 3 * traiettoria[i - 2][0] - traiettoria[i - 3][0]) / (dt ** 3)
            jy = (traiettoria[i][1] - 3 * traiettoria[i - 1][1]
                  + 3 * traiettoria[i - 2][1] - traiettoria[i - 3][1]) / (dt ** 3)
            moduli.append(math.hypot(jx, jy) / PIXEL_PER_METRO)
        jerk = statistics.mean(moduli) if moduli else 0.0

    return {"distanza_minima_m": min(minime), "distanza_media_m": statistics.mean(minime),
            "esposizione_percento": esposizione, "jerk_medio": jerk}


def main():
    popolazione = dict(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode())
            for i in range(PRIMO_SEME, PRIMO_SEME + NUMERO_SEMI)]
    compiti = [(c, s, popolazione) for s in semi for c in CONDIZIONI]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Sicurezza e fluidita' con e senza correzione AI, su {MAPPA}")
    print(f"  ~349 individui, 0,70 pers/m2, {NUMERO_SEMI} semi appaiati x 2 = {len(compiti)} corse")
    print(f"  {n_worker} processi, accelerazione {t.ACCELERAZIONE}x\n")

    esiti = {}
    inizio = time.time()
    completate = 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for condizione, seed, arrivato, tempo_s, fermo_s, misure in pool.imap_unordered(
                _una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(seed, condizione)] = (arrivato, tempo_s, fermo_s, misure)
            print(f"  [{completate}/{len(compiti)}] seme {seed} / {condizione}: {tempo_s:.1f}s, "
                  f"min {misure['distanza_minima_m']:.2f} m, media {misure['distanza_media_m']:.2f} m "
                  f"({time.time() - inizio:.0f}s trascorsi)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta(semi, esiti)


def _riporta(semi, esiti):
    righe = []
    coppie = {chiave: [] for chiave in ("distanza_minima_m", "distanza_media_m",
                                        "esposizione_percento", "jerk_medio")}
    for seme in semi:
        arrivato_b, tempo_b, fermo_b, mis_b = esiti[(seme, "base")]
        arrivato_a, tempo_a, fermo_a, mis_a = esiti[(seme, "solo_ai")]
        if not (arrivato_b and arrivato_a):
            print(f"seme {seme}: coppia scartata, un robot non e' arrivato")
            continue
        for chiave in coppie:
            coppie[chiave].append((mis_b[chiave], mis_a[chiave]))
        riga = {"mappa": MAPPA, "seme": seme,
                "tempo_base_s": round(tempo_b, 2), "tempo_ai_s": round(tempo_a, 2)}
        for chiave in coppie:
            riga[f"{chiave}_base"] = round(mis_b[chiave], 4)
            riga[f"{chiave}_ai"] = round(mis_a[chiave], 4)
        righe.append(riga)

    if not righe:
        print("Nessuna coppia valida.")
        return

    etichette = {"distanza_minima_m": ("distanza minima (m)", True),
                 "distanza_media_m": ("distanza media (m)", True),
                 "esposizione_percento": ("frame sotto 0,5 m (%)", False),
                 "jerk_medio": ("jerk medio (m/s^3)", False)}

    intestazione = f"{'metrica':>24}{'base':>11}{'con AI':>11}{'variazione':>13}{'rapporto':>11}{'vinte':>9}"
    print("=== L'ibridazione paga in sicurezza o fluidita'? ===")
    print("(per distanza e' meglio ALTO, per esposizione e jerk e' meglio BASSO)\n")
    print(intestazione)
    print("-" * len(intestazione))

    for chiave, (nome, meglio_alto) in etichette.items():
        valori = coppie[chiave]
        n = len(valori)
        media_b = statistics.mean(b for b, a in valori)
        media_a = statistics.mean(a for b, a in valori)
        # differenza sempre orientata come "miglioramento dell'AI"
        differenze = [(a - b) if meglio_alto else (b - a) for b, a in valori]
        media = statistics.mean(differenze)
        errore = statistics.stdev(differenze) / n ** 0.5 if n > 1 else float("inf")
        rapporto = abs(media / errore) if errore > 0 else float("inf")
        variazione = (media_a - media_b) / media_b * 100 if media_b else 0.0
        vinte = sum(1 for d in differenze if d > 0)
        print(f"{nome:>24}{media_b:>11.3f}{media_a:>11.3f}{variazione:>+12.1f}%{rapporto:>11.2f}{vinte:>6}/{n}")

    print("\nIl rapporto e' media/errore standard: sotto ~1 indistinguibile da zero, sopra ~2 solido.")
    print("Riferimento in letteratura (SI-MPC, arXiv 2508.07079): +41,1% di distanza minima e -28,4% di")
    print("jerk, pagati con tempi dal 13,8% al 25,5% piu' lunghi.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    main()

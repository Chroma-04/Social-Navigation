"""Sweep della portata del lidar: l'ibridazione della previsione paga se il robot vede piu' lontano?

Ipotesi che mette alla prova. In un orizzonte di previsione (1 s) il robot percorre 1,5 m, mentre il
lidar vede 2,05 m: il margine per accorgersi di qualcosa PRIMA che diventi un problema e' di 55 cm.
L'ipotesi e' che la correzione neurale non produca effetto sui tempi non perche' preveda male - migliora
la previsione del 12%, misurato - ma perche' l'informazione arriva troppo tardi perche' una decisione
diversa sia ancora possibile. Se e' cosi', allargando il lidar il vantaggio deve comparire.

Perche' il lidar e non l'orizzonte di previsione: allungare l'orizzonte aggiunge margine ma degrada la
previsione (piu' si proietta avanti, piu' il moto rettilineo uniforme sbaglia), due effetti opposti nello
stesso numero. Allargare il lidar aggiunge informazione senza toglierne: un effetto solo, interpretabile.

Perche' e' un confronto leale: il lidar piu' grande lo prende ANCHE il robot senza AI, che traccia piu'
persone col suo Kalman. Non si regala un vantaggio alla condizione sperimentale, si alza il livello per
entrambe e si guarda se la distanza fra le due cambia.

Il conteggio delle persone tracciate serve a distinguere due esiti che altrimenti si confonderebbero:
  - i tempi non cambiano E le tracciate non crescono -> i muri di Povo tagliano la vista, il raggio non
    porta informazione, l'esperimento non ha morso e non dice nulla sul meccanismo;
  - i tempi non cambiano MA le tracciate crescono -> il robot vede davvero di piu' e non ne ricava nulla:
    e' il risultato negativo forte, quello che chiude la domanda.

Disegno: NUMERO_SEMI semi appaiati per ogni raggio, due condizioni per seme, semi identici fra i raggi
(quindi appaiati anche nel confronto fra raggi). L'etichetta e' quella delle campagne precedenti, quindi
al raggio 123px i tempi devono riprodurre esattamente `confronto_ai_povo_d070.csv`.

Uso:
    py TEST/sweep_lidar_povo.py
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
for _percorso in (_RADICE, os.path.dirname(os.path.abspath(__file__)),
                  os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim

MAPPA = "povo"
NUMERO_SEMI = 10
CONDIZIONI = ("base", "solo_ai")
ETICHETTA_SEMI = "confronto_ai_povo"  # invariata: al raggio 123px si riproducono le corse gia' fatte

# incrementi crescenti: +57, +120, +180 px. Il primo livello e' il valore ESATTO del pannello (122.647,
# non 123 tondi), altrimenti il punto di ancoraggio non riprodurrebbe le corse gia' fatte e si perderebbe
# il confronto con i 50 semi di confronto_ai_povo_d070. 300px e' il massimo dello slider del pannello,
# 480px = 8 m la portata di un lidar di servizio reale, ben oltre l'articolazione della planimetria
RAGGIO_PANNELLO = t._geometria_salvata()["raggio_lidar"]
RAGGI = [RAGGIO_PANNELLO, 180, 300, 480]

# identica alla serie confronto_ai_povo_d070: ~349 individui su 499 m2 = 0,70 pers/m2
POPOLAZIONE = {
    "numero_persone": 72,
    "numero_corridori": 66,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}

FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "sweep_lidar_povo_d070.csv")


def _una_corsa(argomenti):
    """Una corsa, piu' la media di persone tracciate per frame.

    Il conteggio si ottiene avvolgendo _rileva_e_traccia nel processo worker invece di modificare
    testing.py: la funzione e' condivisa col generatore di dataset e non va toccata per una diagnostica.
    """
    condizione, seed, popolazione, raggio = argomenti
    popolazione = dict(popolazione)
    popolazione["raggio_lidar"] = raggio  # letto da testing.esegui_corsa, che riscrive sim.RAGGIO_LIDAR_DEFAULT

    originale = t._rileva_e_traccia
    tracciate = []

    def _con_conteggio(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
        risultato = originale(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni)
        tracciate.append(len(tracciamento_lidar))
        return risultato

    t._rileva_e_traccia = _con_conteggio
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, condizione, popolazione, seed)
    finally:
        t._rileva_e_traccia = originale

    media_tracciate = statistics.mean(tracciate) if tracciate else 0.0
    return raggio, condizione, seed, arrivato, t._secondi(frame), t._secondi(frame_fermo), media_tracciate


def main():
    popolazione = dict(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(NUMERO_SEMI)]
    compiti = [(c, s, popolazione, r) for r in RAGGI for s in semi for c in CONDIZIONI]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Sweep del raggio lidar su {MAPPA}, {NUMERO_SEMI} semi appaiati per raggio")
    print(f"  raggi: {', '.join(f'{r:.0f}px ({r / 60:.2f} m)' for r in RAGGI)}")
    print(f"  {POPOLAZIONE['numero_persone']} persone, {POPOLAZIONE['numero_corridori']} veloci, "
          f"{POPOLAZIONE['numero_persone_ferme']} ferme, {POPOLAZIONE['numero_gruppi']} gruppi "
          f"(~349 individui, 0,70 pers/m2)")
    print(f"  {len(compiti)} corse totali, {n_worker} processi, accelerazione {t.ACCELERAZIONE}x\n")

    esiti = {}
    inizio = time.time()
    completate = 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for raggio, condizione, seed, arrivato, tempo_s, fermo_s, tracciate in pool.imap_unordered(
                _una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(raggio, seed, condizione)] = (arrivato, tempo_s, fermo_s, tracciate)
            stato = f"{tempo_s:.1f}s" if arrivato else "NON ARRIVATO"
            print(f"  [{completate}/{len(compiti)}] {raggio:.0f}px seme {seed} / {condizione}: {stato} "
                  f"({tracciate:.1f} tracciate, {time.time() - inizio:.0f}s trascorsi)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta(semi, esiti)


def _riporta(semi, esiti):
    righe = []
    intestazione = (f"{'raggio':>8}{'metri':>7}{'tracciate':>11}{'base':>9}{'solo_ai':>10}"
                    f"{'differenza':>12}{'variazione':>12}{'rapporto':>10}{'vinte':>8}")
    print("=== Vantaggio dell'AI al crescere della portata del lidar ===")
    print("(differenza positiva = il robot con AI arriva prima)\n")
    print(intestazione)
    print("-" * len(intestazione))

    for raggio in RAGGI:
        differenze, percentuali, tempi_base, tempi_ai, tracciate = [], [], [], [], []
        for seme in semi:
            arrivato_b, tempo_b, fermo_b, tracc_b = esiti[(raggio, seme, "base")]
            arrivato_a, tempo_a, fermo_a, tracc_a = esiti[(raggio, seme, "solo_ai")]
            tracciate += [tracc_b, tracc_a]
            if not (arrivato_b and arrivato_a):
                print(f"{raggio:>8.0f}   coppia scartata al seme {seme}: un robot non e' arrivato")
                continue
            differenze.append(tempo_b - tempo_a)
            percentuali.append((tempo_b - tempo_a) / tempo_b * 100)
            tempi_base.append(tempo_b)
            tempi_ai.append(tempo_a)
            righe.append({"raggio_lidar_px": raggio, "seme": seme,
                          "tempo_base_s": round(tempo_b, 2), "tempo_ai_s": round(tempo_a, 2),
                          "differenza_s": round(tempo_b - tempo_a, 2),
                          "variazione_percento": round((tempo_b - tempo_a) / tempo_b * 100, 1),
                          "fermo_base_s": round(fermo_b, 2), "fermo_ai_s": round(fermo_a, 2),
                          "tracciate_base": round(tracc_b, 2), "tracciate_ai": round(tracc_a, 2)})
            righe[-1]["raggio_lidar_px"] = round(raggio, 1)

        if not differenze:
            continue
        n = len(differenze)
        media = statistics.mean(differenze)
        # rapporto fra la differenza media e il suo errore standard: sotto ~1 e' indistinguibile da zero
        errore = statistics.stdev(differenze) / n ** 0.5 if n > 1 else float("inf")
        rapporto = abs(media / errore) if errore > 0 else float("inf")
        vinte = sum(1 for d in differenze if d > 0)
        print(f"{raggio:>8.0f}{raggio / 60:>7.2f}{statistics.mean(tracciate):>11.1f}"
              f"{statistics.mean(tempi_base):>8.1f}s{statistics.mean(tempi_ai):>9.1f}s"
              f"{media:>+11.1f}s{statistics.mean(percentuali):>+11.1f}%{rapporto:>10.2f}{vinte:>5}/{n}")

    print("\n'tracciate' e' la media di persone sotto tracciamento per frame: se non cresce col raggio,")
    print("i muri stanno tagliando la vista e lo sweep non ha morso.")
    print(f"Con {NUMERO_SEMI} semi per raggio questo e' uno SCREENING: cerca una tendenza, non la misura.")
    print("Un raggio che mostri qualcosa va riportato a 50 semi prima di poterne scrivere in tesi.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    main()

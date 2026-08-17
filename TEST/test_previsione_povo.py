"""Prevedere serve? Robot con previsione contro robot che guarda solo il presente.

Le campagne precedenti confrontano `base` (Kalman + macchie proiettate in avanti) con `solo_ai` (le
stesse cose, piu' la correzione neurale). Nessuna delle due e' cieca: misurano quanto vale prevedere
6 cm MEGLIO, non quanto vale prevedere. Il risultato nullo si spiega solo se si sa dove si rompe la
catena, e questa e' la misura che lo dice.

Le due condizioni, entrambe SENZA correzione neurale:
  - 'con previsione': il robot di riferimento di tutte le campagne, orizzonte 1 s;
  - 'senza previsione': orizzonte azzerato. Il Kalman continua a tracciare e le macchie si disegnano
    lo stesso, ma sulla posizione ATTUALE della persona invece che su quella prevista. Il robot vede
    dove le persone sono, non dove staranno.

I due esiti sono entrambi informativi, il che rende il test degno di essere fatto:
  - senza previsione il robot e' molto piu' lento -> la catena percettiva funziona, e solo il
    raffinamento neurale da 6 cm non arriva a spostare una decisione presa su celle da 25 cm;
  - i tempi sono uguali -> l'intera macchina predittiva e' inerte. In un pianificatore che ripianifica
    quasi a ogni frame, prevedere dove sara' una persona fra un secondo vale poco: fra un sessantesimo
    di secondo si puo' guardare di nuovo e vedere dov'e' davvero. La previsione compete con
    l'osservazione diretta e perde. E' la conclusione piu' forte, e cambierebbe il capitolo.

Come si spegne la previsione: sim.PREVISIONE_BLOB_FRAME e' letto DENTRO esegui_corsa (riga
'orizzonte_previsione = max(1, sim.PREVISIONE_BLOB_FRAME // ACCELERAZIONE)'), quindi basta riscriverlo
nel processo worker prima della chiamata. Resta 1 frame per via del max(): a passo accelerato sono
3 millisecondi di proiezione, cioe' praticamente il presente. Nessuna modifica a testing.py, che e'
condiviso col generatore di dataset.

Popolazione e semi identici a confronto_ai_povo_d070: il braccio 'con previsione' deve riprodurre
esattamente i tempi gia' in archivio, ed e' una verifica di riproducibilita' gratuita.

Uso:
    py TEST/test_previsione_povo.py
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
NUMERO_SEMI = 50
ETICHETTA_SEMI = "confronto_ai_povo"  # invariata: il braccio con previsione riproduce l'archivio

# identica alla serie confronto_ai_povo_d070: ~349 individui su 499 m2 = 0,70 pers/m2
POPOLAZIONE = {
    "numero_persone": 72,
    "numero_corridori": 66,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}

FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "confronto_previsione_povo_d070.csv")

_ORIZZONTE_ORIGINALE = sim.PREVISIONE_BLOB_FRAME


def _una_corsa(argomenti):
    prevede, seed, popolazione = argomenti
    # il worker e' un processo separato (spawn): la globale va riscritta qui, non nel padre
    sim.PREVISIONE_BLOB_FRAME = _ORIZZONTE_ORIGINALE if prevede else 0
    t.sim.PREVISIONE_BLOB_FRAME = sim.PREVISIONE_BLOB_FRAME
    arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, "base", popolazione, seed)
    return prevede, seed, arrivato, t._secondi(frame), t._secondi(frame_fermo)


def main():
    popolazione = dict(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(NUMERO_SEMI)]
    compiti = [(p, s, popolazione) for s in semi for p in (True, False)]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Prevedere serve? Confronto appaiato su {MAPPA}, entrambe le condizioni senza correzione AI")
    print(f"  con previsione: orizzonte {_ORIZZONTE_ORIGINALE} frame (1 s)")
    print(f"  senza previsione: orizzonte azzerato, macchie sulla posizione attuale")
    print(f"  ~349 individui, 0,70 pers/m2")
    print(f"  {NUMERO_SEMI} semi x 2 = {len(compiti)} corse, {n_worker} processi, "
          f"accelerazione {t.ACCELERAZIONE}x\n")

    esiti = {}
    inizio = time.time()
    completate = 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for prevede, seed, arrivato, tempo_s, fermo_s in pool.imap_unordered(_una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(seed, prevede)] = (arrivato, tempo_s, fermo_s)
            etichetta = "con previsione" if prevede else "SENZA previsione"
            stato = f"{tempo_s:.1f}s" if arrivato else "NON ARRIVATO"
            print(f"  [{completate}/{len(compiti)}] seme {seed} / {etichetta}: {stato} "
                  f"({time.time() - inizio:.0f}s trascorsi)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta(semi, esiti)


def _riporta(semi, esiti):
    intestazione = (f"{'seme':>12}{'con prev.':>12}{'senza prev.':>14}{'differenza':>13}"
                    f"{'variazione':>13}{'fermo con':>12}{'fermo senza':>13}")
    print("=== Costo di spegnere la previsione (nessuna correzione AI in gioco) ===")
    print("(differenza positiva = prevedere fa arrivare prima)\n")
    print(intestazione)
    print("-" * len(intestazione))

    righe, differenze, percentuali = [], [], []
    for seme in semi:
        arrivato_c, tempo_c, fermo_c = esiti[(seme, True)]
        arrivato_s, tempo_s_, fermo_s = esiti[(seme, False)]
        if not (arrivato_c and arrivato_s):
            quale = "con previsione" if not arrivato_c else "senza previsione"
            print(f"{seme:>12}   coppia scartata: il robot '{quale}' non e' arrivato entro il limite")
            continue
        differenza = tempo_s_ - tempo_c  # positivo = senza previsione ci mette di piu'
        percentuale = differenza / tempo_s_ * 100
        differenze.append(differenza)
        percentuali.append(percentuale)
        print(f"{seme:>12}{tempo_c:>11.1f}s{tempo_s_:>13.1f}s{differenza:>+12.1f}s"
              f"{percentuale:>+12.1f}%{fermo_c:>11.1f}s{fermo_s:>12.1f}s")
        righe.append({"mappa": MAPPA, "seme": seme,
                      "tempo_con_previsione_s": round(tempo_c, 2),
                      "tempo_senza_previsione_s": round(tempo_s_, 2),
                      "differenza_s": round(differenza, 2),
                      "variazione_percento": round(percentuale, 1),
                      "fermo_con_previsione_s": round(fermo_c, 2),
                      "fermo_senza_previsione_s": round(fermo_s, 2)})

    if not differenze:
        print("\nNessuna coppia valida.")
        return

    n = len(differenze)
    media = statistics.mean(differenze)
    errore = statistics.stdev(differenze) / n ** 0.5 if n > 1 else float("inf")
    rapporto = abs(media / errore) if errore > 0 else float("inf")
    vinte = sum(1 for d in differenze if d > 0)
    print("-" * len(intestazione))
    print(f"{'media':>12}{'':>25}{media:>+12.1f}s{statistics.mean(percentuali):>+12.1f}%")
    print(f"\nCoppie in cui prevedere conviene: {vinte} su {n}")
    print(f"Errore standard: {errore:.2f}s (rapporto media/errore = {rapporto:.2f})")
    print(f"IC 95% sulla variazione: {statistics.mean(percentuali) - 1.96 * statistics.stdev(percentuali) / n ** 0.5:+.1f}% "
          f".. {statistics.mean(percentuali) + 1.96 * statistics.stdev(percentuali) / n ** 0.5:+.1f}%")
    if rapporto < 1:
        print("\nPrevedere non produce effetto rilevabile: l'intera macchina predittiva e' inerte in")
        print("questo pianificatore, e non solo il raffinamento neurale.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    main()

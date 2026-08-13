"""Confronto diretto e appaiato: robot CON correzione AI contro robot SENZA, a parita' di folla.

Domanda a cui risponde, e nient'altro: a parita' di scena, di percorso e di rumore, il robot che
usa la correzione neurale arriva prima di quello che usa il solo Kalman?

Disegno:
  - NUMERO_SEMI semi diversi, cioe' altrettante folle diverse (posizioni iniziali, destinazioni,
    comportamenti). Serve una numerosita' adeguata: i due robot partono dalla stessa folla, ma appena
    scelgono percorsi anche di poco diversi spingono via persone diverse e le due simulazioni
    divergono, quindi il seme comune controlla le condizioni iniziali e non l'intera traiettoria.
    L'appaiamento riduce la varianza molto meno di quanto faccia di norma, e la dispersione fra semi
    resta grande rispetto agli effetti che si vogliono misurare;
  - per ogni seme DUE corse, una per modello di robot, con lo STESSO seme: la folla di partenza e
    l'intera sequenza di numeri casuali sono identiche, quindi l'unica differenza fra le due corse e'
    il robot. E' un confronto appaiato: si guarda la differenza dentro ciascuna coppia, non la media
    di due gruppi indipendenti, che sarebbe molto piu' rumorosa;
  - tragitto fisso in tutte le corse, dall'angolo alto-sinistro a quello basso-destro (la cella libera
    piu' vicina all'angolo, dato che gli angoli veri sono muro);
  - 2 x NUMERO_SEMI corse in totale.

Il controllo predittivo di velocita' resta SPENTO in entrambe le condizioni. E' l'altro componente
aggiunto dalla tesi e va misurato a parte: accendendolo qui, una differenza nei tempi non sarebbe piu'
attribuibile alla sola AI.

PARALLELIZZAZIONE. main.py distribuisce sul pool i ricalcoli di percorso della folla, perche' deve
mostrare fluida UNA simulazione: li' conta la latenza del singolo frame. Qui il problema e' opposto -
ci sono 10 simulazioni indipendenti e nessuno le guarda - quindi conviene il parallelismo grossolano,
una corsa intera per processo. E' anche il motivo per cui questo file dispaccia le SINGOLE CORSE e non
le coppie: con 10 corse su 8 processi si riempiono meglio i core che con 5 coppie da 2 corse ciascuna,
che ne lascerebbero 3 fermi per meta' del tempo. L'appaiamento non ne risente, perche' a parita' di
seme la corsa e' deterministica: ogni corsa riparte da random.seed(seed) e ricostruisce tutto da zero,
quindi non importa in quale processo o in quale ordine venga eseguita.

Uso:
    py confronto_ai.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import time
import zlib
import multiprocessing as mp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import testing as t
import main as sim

NUMERO_SEMI = 20
MAPPA = "training_mappa_01"
CONDIZIONE_BASE, CONDIZIONE_AI = "base", "solo_ai"

# Popolazione: la configurazione del pannello tecnico, cioe' quella con cui il simulatore viene
# realmente usato. I 43 gruppi portano ~150 membri (media 3.5 componenti), quindi il totale in scena
# e' circa 510 individui.
POPOLAZIONE = {
    "numero_persone": 100,
    "numero_corridori": 90,
    "numero_persone_ferme": 170,
    "numero_gruppi": 43,
}

FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "confronto_ai.csv")


def _una_corsa(argomenti):
    condizione, seed, popolazione = argomenti
    arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, condizione, popolazione, seed)
    return condizione, seed, arrivato, t._secondi(frame), t._secondi(frame_fermo)


def main():
    popolazione = dict(POPOLAZIONE)
    popolazione.update(t._geometria_salvata())  # raggi robot/persona/sicurezza/lidar dal pannello

    totale_stimato = (POPOLAZIONE["numero_persone"] + POPOLAZIONE["numero_corridori"]
                      + POPOLAZIONE["numero_persone_ferme"]
                      + round(POPOLAZIONE["numero_gruppi"] * sim.GRUPPO_MEMBRI_MEDIA))
    semi = [zlib.crc32(f"{MAPPA}|confronto_ai|{i}".encode()) for i in range(NUMERO_SEMI)]
    compiti = [(c, s, popolazione) for s in semi for c in (CONDIZIONE_BASE, CONDIZIONE_AI)]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Confronto appaiato AI acceso / AI spento su {MAPPA}")
    print(f"  {POPOLAZIONE['numero_persone']} persone, {POPOLAZIONE['numero_corridori']} veloci, "
          f"{POPOLAZIONE['numero_persone_ferme']} ferme, {POPOLAZIONE['numero_gruppi']} gruppi "
          f"(~{totale_stimato} individui)")
    print(f"  geometria dal pannello: robot {popolazione['raggio_robot']:.1f}px, "
          f"persone {popolazione['raggio_persona']:.1f}px, sicurezza {popolazione['raggio_sicurezza']:.1f}px, "
          f"lidar {popolazione['raggio_lidar']:.0f}px")
    print(f"  {NUMERO_SEMI} semi x 2 robot = {len(compiti)} corse, tragitto fisso angolo->angolo")
    print(f"  accelerazione {t.ACCELERAZIONE}x, {n_worker} processi\n")

    esiti = {}
    inizio = time.time()
    completate = 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for condizione, seed, arrivato, tempo_s, fermo_s in pool.imap_unordered(_una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(seed, condizione)] = (arrivato, tempo_s, fermo_s)
            stato = f"{tempo_s:.1f}s" if arrivato else "NON ARRIVATO"
            print(f"  [{completate}/{len(compiti)}] seme {seed} / {condizione}: {stato} "
                  f"({time.time() - inizio:.0f}s trascorsi)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta(semi, esiti)


def _riporta(semi, esiti):
    intestazione = (f"{'seme':>12}{'senza AI':>11}{'con AI':>11}{'differenza':>13}"
                    f"{'variazione':>13}{'fermo base':>12}{'fermo AI':>11}")
    print("=== Confronto appaiato: stessa folla, stesso tragitto, robot diverso ===")
    print("(differenza positiva = il robot con AI arriva prima)\n")
    print(intestazione)
    print("-" * len(intestazione))

    righe, differenze, percentuali = [], [], []
    for seme in semi:
        arrivato_b, tempo_b, fermo_b = esiti[(seme, CONDIZIONE_BASE)]
        arrivato_a, tempo_a, fermo_a = esiti[(seme, CONDIZIONE_AI)]
        if not (arrivato_b and arrivato_a):
            # coppia incompleta: scartata intera, perche' confrontare un tempo di arrivo con un tempo
            # di scadenza non e' un confronto fra tempi di percorrenza
            quale = "senza AI" if not arrivato_b else "con AI"
            print(f"{seme:>12}   coppia scartata: il robot {quale} non e' arrivato entro il limite")
            continue
        differenza = tempo_b - tempo_a
        percentuale = differenza / tempo_b * 100
        differenze.append(differenza)
        percentuali.append(percentuale)
        print(f"{seme:>12}{tempo_b:>10.1f}s{tempo_a:>10.1f}s{differenza:>+12.1f}s"
              f"{percentuale:>+12.1f}%{fermo_b:>11.1f}s{fermo_a:>10.1f}s")
        righe.append({"mappa": MAPPA, "seme": seme, "tempo_base_s": round(tempo_b, 2),
                      "tempo_ai_s": round(tempo_a, 2), "differenza_s": round(differenza, 2),
                      "variazione_percento": round(percentuale, 1),
                      "fermo_base_s": round(fermo_b, 2), "fermo_ai_s": round(fermo_a, 2)})

    if not differenze:
        print("\nNessuna coppia valida: nessun confronto possibile.")
        return

    n = len(differenze)
    media = sum(differenze) / n
    media_perc = sum(percentuali) / n
    vinte = sum(1 for d in differenze if d > 0)
    print("-" * len(intestazione))
    print(f"{'media':>12}{'':>22}{media:>+12.1f}s{media_perc:>+12.1f}%")

    print(f"\nCoppie in cui l'AI arriva prima: {vinte} su {n}")
    if n > 1:
        varianza = sum((d - media) ** 2 for d in differenze) / (n - 1)
        errore_standard = (varianza / n) ** 0.5
        # rapporto fra la differenza media e il suo errore standard: sopra ~2 la differenza e' solida
        # rispetto alla dispersione fra i semi, sotto ~1 e' indistinguibile da zero
        rapporto = abs(media / errore_standard) if errore_standard > 0 else float("inf")
        print(f"Errore standard sulla differenza media: {errore_standard:.1f}s "
              f"(rapporto media/errore = {rapporto:.1f})")
        if rapporto < 1:
            print("Con questo rapporto la differenza NON e' distinguibile da zero: servono piu' semi "
                  "prima di poterne trarre una conclusione.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    main()

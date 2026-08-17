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

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # i moduli condivisi stanno nella radice
for _percorso in (_RADICE, os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)
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
    # la mappa viaggia nella tupla e non in una globale: con lo spawn di Windows il worker re-importa
    # il modulo da zero e di una globale riassegnata nel processo padre non saprebbe nulla
    condizione, seed, popolazione, mappa = argomenti
    arrivato, frame, frame_fermo = t.esegui_corsa(mappa, condizione, popolazione, seed)
    return condizione, seed, arrivato, t._secondi(frame), t._secondi(frame_fermo)


def esegui_confronto(mappa=MAPPA, conteggi=None, numero_semi=NUMERO_SEMI, file_csv=FILE_CSV,
                     etichetta="confronto_ai", condizioni=(CONDIZIONE_BASE, CONDIZIONE_AI)):
    """Il confronto appaiato descritto in testa al file, su una mappa e una popolazione qualsiasi.

    'etichetta' entra nel calcolo dei semi insieme al nome della mappa: cambiandola si ottiene una
    batteria di semi indipendente, lasciandola si riproducono esattamente le corse gia' fatte.

    'condizioni' e' la coppia (riferimento, variante) da confrontare, con i nomi di testing.CONDIZIONI:
    serve a mettere alla prova un componente diverso dalla correzione AI, per esempio il controllo
    predittivo di velocita' ('solo_velocita')."""
    conteggi = dict(conteggi if conteggi is not None else POPOLAZIONE)
    popolazione = dict(conteggi)
    popolazione.update(t._geometria_salvata())  # raggi robot/persona/sicurezza/lidar dal pannello

    totale_stimato = (conteggi["numero_persone"] + conteggi["numero_corridori"]
                      + conteggi["numero_persone_ferme"]
                      + round(conteggi["numero_gruppi"] * sim.GRUPPO_MEMBRI_MEDIA))
    riferimento, variante = condizioni
    semi = [zlib.crc32(f"{mappa}|{etichetta}|{i}".encode()) for i in range(numero_semi)]
    compiti = [(c, s, popolazione, mappa) for s in semi for c in condizioni]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Confronto appaiato '{variante}' contro '{riferimento}' su {mappa}")
    print(f"  {conteggi['numero_persone']} persone, {conteggi['numero_corridori']} veloci, "
          f"{conteggi['numero_persone_ferme']} ferme, {conteggi['numero_gruppi']} gruppi "
          f"(~{totale_stimato} individui)")
    print(f"  geometria dal pannello: robot {popolazione['raggio_robot']:.1f}px, "
          f"persone {popolazione['raggio_persona']:.1f}px, sicurezza {popolazione['raggio_sicurezza']:.1f}px, "
          f"lidar {popolazione['raggio_lidar']:.0f}px")
    print(f"  {numero_semi} semi x 2 robot = {len(compiti)} corse, tragitto fisso angolo->angolo")
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
    _riporta(semi, esiti, mappa, file_csv, condizioni)


def main():
    esegui_confronto()


def _riporta(semi, esiti, mappa=MAPPA, file_csv=FILE_CSV, condizioni=(CONDIZIONE_BASE, CONDIZIONE_AI)):
    riferimento, variante = condizioni
    intestazione = (f"{'seme':>12}{riferimento:>11}{variante:>15}{'differenza':>13}"
                    f"{'variazione':>13}{'fermo rif.':>12}{'fermo var.':>11}")
    print("=== Confronto appaiato: stessa folla, stesso tragitto, robot diverso ===")
    print(f"(differenza positiva = il robot '{variante}' arriva prima)\n")
    print(intestazione)
    print("-" * len(intestazione))

    righe, differenze, percentuali = [], [], []
    for seme in semi:
        arrivato_b, tempo_b, fermo_b = esiti[(seme, riferimento)]
        arrivato_a, tempo_a, fermo_a = esiti[(seme, variante)]
        if not (arrivato_b and arrivato_a):
            # coppia incompleta: scartata intera, perche' confrontare un tempo di arrivo con un tempo
            # di scadenza non e' un confronto fra tempi di percorrenza
            quale = riferimento if not arrivato_b else variante
            print(f"{seme:>12}   coppia scartata: il robot '{quale}' non e' arrivato entro il limite")
            continue
        differenza = tempo_b - tempo_a
        percentuale = differenza / tempo_b * 100
        differenze.append(differenza)
        percentuali.append(percentuale)
        print(f"{seme:>12}{tempo_b:>10.1f}s{tempo_a:>10.1f}s{differenza:>+12.1f}s"
              f"{percentuale:>+12.1f}%{fermo_b:>11.1f}s{fermo_a:>10.1f}s")
        righe.append({"mappa": mappa, "seme": seme, "tempo_base_s": round(tempo_b, 2),
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

    print(f"\nCoppie in cui '{variante}' arriva prima: {vinte} su {n}")
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
    with open(file_csv, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {file_csv}")


if __name__ == "__main__":
    main()

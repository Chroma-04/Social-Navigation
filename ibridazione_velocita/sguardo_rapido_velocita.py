"""Sguardo rapido: una sola coppia appaiata per ciascuna delle dieci densita'.

Venti corse in pochi minuti. NON e' una misura: con una coppia sola per livello non esiste
errore standard, e la differenza osservata a un dato livello puo' essere per intero rumore.
La dispersione in gioco e' nota e non e' piccola - nello screening precedente due corse
identiche per configurazione, a pop211, hanno dato 48,2 e 54,8 s: sei secondi e mezzo, cioe'
piu' dell'effetto che si sta cercando. Un livello preso da solo puo' quindi raccontare -2%
oppure +11% a seconda del seme capitato.

Cio' che questo banco puo' mostrare e' la FORMA lungo la densita', non i valori: dieci punti
rumorosi che salgono tutti insieme dicono qualcosa che un punto solo non direbbe. Si legge
la tendenza complessiva, e si ignora ogni singolo livello.

Il seme cambia da una densita' all'altra, apposta: con un seme unico per tutti i livelli una
folla particolarmente sfortunata si propagherebbe all'intera curva, e la forma che si vuole
leggere sarebbe quella di quel seme invece che quella dell'effetto.

Uso:
    py -u ibridazione_velocita/sguardo_rapido_velocita.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import time
import zlib
import statistics
import multiprocessing as mp

_QUI = os.path.dirname(os.path.abspath(__file__))
_RADICE = os.path.dirname(_QUI)
for _percorso in (_RADICE, _QUI, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import performance_velocita_classico as pv

# meno worker del solito: questo banco e' pensato per girare ACCANTO alla campagna completa,
# che ne occupa gia' otto. Sedici core logici li reggono entrambi senza che nessuno rallenti
WORKER = 5


def main():
    livelli = [(f"pop{round(sum(pv.POPOLAZIONE_BASE.values()) * f)}", pv._popolazione_scalata(f))
               for f in pv.FATTORI]
    # un seme diverso per livello: l'etichetta della densita' entra nella derivazione
    compiti = [(condizione, zlib.crc32(f"{pv.MAPPA}|sguardo_rapido|{etichetta}".encode()),
                popolazione, etichetta)
               for etichetta, popolazione in livelli
               for condizione in pv.CONDIZIONI]

    print(f"Sguardo rapido sul controllo di velocita': {len(livelli)} densita' x 1 coppia = {len(compiti)} corse")
    print(f"  {WORKER} processi (la campagna completa ne usa altri 8), accelerazione {t.ACCELERAZIONE}x")
    print("  ATTENZIONE: una coppia per livello non e' una misura. Si legge la forma, non i valori.\n")

    esiti = {}
    inizio = time.time()
    completate = 0
    with mp.Pool(WORKER, initializer=t._inizializza_worker) as pool:
        for etichetta, condizione, seed, arrivato, tempo_s, fermo_s, misure in pool.imap_unordered(
                pv._una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(etichetta, condizione)] = (arrivato, tempo_s, fermo_s, misure)
            print(f"  [{completate}/{len(compiti)}] {etichetta} / {condizione}: {tempo_s:.1f}s, "
                  f"{misure['numero_fermate']} fermate ({time.time() - inizio:.0f}s)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta([e for e, _ in livelli], esiti)


def _riporta(etichette, esiti):
    intestazione = (f"{'livello':>10}{'tempo b':>10}{'tempo c':>10}{'var.':>9}"
                    f"{'ferm. b':>9}{'ferm. c':>9}{'var.':>8}"
                    f"{'vel. b':>9}{'vel. c':>9}{'frenata':>9}{'sprint':>8}")
    print("=== Forma dell'effetto lungo la densita' (una coppia per livello) ===\n")
    print(intestazione)
    print("-" * len(intestazione))

    variazioni_tempo, variazioni_fermate = [], []
    for etichetta in etichette:
        if (etichetta, "base") not in esiti or (etichetta, "solo_velocita") not in esiti:
            continue
        arrivato_b, tempo_b, fermo_b, mis_b = esiti[(etichetta, "base")]
        arrivato_c, tempo_c, fermo_c, mis_c = esiti[(etichetta, "solo_velocita")]
        if not (arrivato_b and arrivato_c):
            print(f"{etichetta:>10}   coppia scartata: un robot non e' arrivato")
            continue
        var_tempo = (tempo_c - tempo_b) / tempo_b * 100 if tempo_b else 0.0
        fb, fc = mis_b["numero_fermate"], mis_c["numero_fermate"]
        var_fermate = (fc - fb) / fb * 100 if fb else 0.0
        variazioni_tempo.append(var_tempo)
        variazioni_fermate.append(var_fermate)
        print(f"{etichetta:>10}{tempo_b:>9.1f}s{tempo_c:>9.1f}s{var_tempo:>+8.1f}%"
              f"{fb:>9}{fc:>9}{var_fermate:>+7.0f}%"
              f"{mis_b['velocita_media_m_s']:>9.2f}{mis_c['velocita_media_m_s']:>9.2f}"
              f"{mis_c['frenata_percento']:>8.1f}%{mis_c['sprint_percento']:>7.1f}%")

    if variazioni_tempo:
        saliti = sum(1 for v in variazioni_tempo if v > 0)
        fermate_salite = sum(1 for v in variazioni_fermate if v > 0)
        print(f"\nTempo: peggiorato in {saliti} livelli su {len(variazioni_tempo)}, "
              f"mediana {statistics.median(variazioni_tempo):+.1f}%")
        print(f"Fermate: aumentate in {fermate_salite} livelli su {len(variazioni_fermate)}, "
              f"mediana {statistics.median(variazioni_fermate):+.0f}%")
        print("\nIl conteggio dei livelli concordi vale piu' del singolo valore: dieci punti")
        print("indipendenti che si muovono nella stessa direzione sono un segnale, uno no.")


if __name__ == "__main__":
    main()

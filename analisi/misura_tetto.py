"""Misura il TETTO recuperabile da un controllo di velocita' intelligente.

Fa correre il robot sul solito percorso (angolo alto-sinistra -> angolo basso-destra) con la
configurazione salvata dal pannello, e misura quanta parte del tempo passa FERMO per lo stop di
sicurezza. Quella frazione e' il massimo che un qualunque controllo - euristico o appreso - potrebbe
recuperare: non si puo' guadagnare piu' del tempo che si perde.

Due condizioni sugli stessi seed (confronto appaiato):
    base           solo Kalman, velocita' costante
    solo_velocita  stessa percezione, ma con il controllo predittivo di velocita' attuale
Cosi' oltre al tetto si vede se l'euristica scritta a mano oggi aiuta o danneggia.
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import json
import time
import zlib
import multiprocessing as mp

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # la radice del progetto
import testing as t

MAPPA = "training_mappa_01"
REPLICHE = 10
CONDIZIONI = ("base", "solo_velocita")


def _popolazione_dal_pannello():
    """I conteggi ESATTI salvati nel pannello, non la ricetta proporzionale: si misura la configurazione
    in cui il simulatore viene realmente usato."""
    with open(t.FILE_CONFIGURAZIONE_PANNELLO) as f:
        c = json.load(f)
    pop = {k: c[k] for k in ("numero_persone", "numero_corridori", "numero_persone_ferme", "numero_gruppi")}
    pop.update(t._geometria_salvata())
    return pop


def _lavoro(argomenti):
    pop, seed = argomenti
    esiti = {}
    for cond in CONDIZIONI:
        arrivato, frame, fermo = t.esegui_corsa(MAPPA, cond, pop, seed)
        esiti[cond] = (arrivato, frame, fermo)
    return seed, esiti


if __name__ == "__main__":
    pop = _popolazione_dal_pannello()
    totale_stimato = (pop["numero_persone"] + pop["numero_corridori"] + pop["numero_persone_ferme"]
                      + int(pop["numero_gruppi"] * 3.5))
    print(f"Mappa: {MAPPA}, percorso angolo->angolo, {REPLICHE} repliche, accelerazione {t.ACCELERAZIONE}x")
    print(f"Folla: {pop['numero_persone']} persone + {pop['numero_corridori']} corridori + "
          f"{pop['numero_persone_ferme']} ferme + {pop['numero_gruppi']} gruppi = ~{totale_stimato} individui")
    print(f"Geometria: robot {pop['raggio_robot']:.1f}px, persone {pop['raggio_persona']:.1f}px, "
          f"sicurezza {pop['raggio_sicurezza']:.1f}px, lidar {pop['raggio_lidar']:.0f}px\n")

    compiti = [(pop, zlib.crc32(f"{MAPPA}|tetto|{i}".encode())) for i in range(REPLICHE)]
    dati = {c: [] for c in CONDIZIONI}
    inizio = time.time()
    with mp.Pool(min(8, os.cpu_count() or 4), initializer=t._inizializza_worker) as pool:
        for seed, esiti in pool.imap_unordered(_lavoro, compiti):
            for c in CONDIZIONI:
                dati[c].append(esiti[c])
            b, v = esiti["base"], esiti["solo_velocita"]
            print(f"  seed {seed:>10}: base {t._secondi(b[1]):6.1f}s ({b[2] / b[1] * 100:4.1f}% fermo)"
                  f"   solo_velocita {t._secondi(v[1]):6.1f}s ({v[2] / v[1] * 100:4.1f}% fermo)"
                  f"{'' if b[0] and v[0] else '   [NON ARRIVATO]'}")

    print(f"\nCompletato in {time.time() - inizio:.0f}s\n")
    print("=== Quota di tempo passata ferma per lo stop di sicurezza ===")
    print(f"{'condizione':>15}{'tempo medio':>14}{'fermo medio':>14}{'% fermo':>10}{'arrivi':>9}")
    riepilogo = {}
    for c in CONDIZIONI:
        validi = [(f, s) for a, f, s in dati[c] if a]
        if not validi:
            print(f"{c:>15}   nessun arrivo")
            continue
        t_medio = sum(t._secondi(f) for f, _ in validi) / len(validi)
        f_medio = sum(t._secondi(s) for _, s in validi) / len(validi)
        quota = sum(s for _, s in validi) / sum(f for f, _ in validi) * 100
        riepilogo[c] = (t_medio, quota)
        print(f"{c:>15}{t_medio:>13.1f}s{f_medio:>13.1f}s{quota:>9.1f}%{len(validi):>5}/{REPLICHE}")

    if "base" in riepilogo:
        tetto = riepilogo["base"][1]
        print(f"\n>>> TETTO: un controllo di velocita' perfetto non puo' guadagnare piu' del {tetto:.1f}%")
        print(f"    (e' la frazione di tempo che il robot base passa fermo; realisticamente se ne")
        print(f"     recupera una parte, quindi attendersi circa la meta': ~{tetto / 2:.1f}%)")
    if "base" in riepilogo and "solo_velocita" in riepilogo:
        d = (1 - riepilogo["solo_velocita"][0] / riepilogo["base"][0]) * 100
        print(f"\n>>> L'euristica di velocita' ATTUALE: {d:+.1f}% sul tempo di percorrenza, e porta la")
        print(f"    quota di tempo fermo da {riepilogo['base'][1]:.1f}% a {riepilogo['solo_velocita'][1]:.1f}%")

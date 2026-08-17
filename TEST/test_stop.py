"""TEST STOP: quanto tempo il robot passa fermo per lo stop di sicurezza.

Fa correre il robot piu' volte sullo stesso percorso (angolo alto-sinistra -> angolo basso-destra) in una
configurazione fissata - mappa, popolazione, condizione - e riporta per ogni corsa la percentuale di
tempo passata immobile, piu' la statistica aggregata.

A COSA SERVE
Quella percentuale e' il TETTO di qualunque intervento che voglia far arrivare prima il robot evitando i
blocchi: non si puo' guadagnare piu' tempo di quanto se ne perde. Misurarla PRIMA di costruire un
controllo intelligente dice se vale la pena costruirlo. Se il robot perde il 3% del tempo fermo, nessun
modello - per quanto ben allenato - potra' fargli guadagnare piu' del 3%.

Lo stop di sicurezza e' replicato da main.py: se una persona in movimento entra nel raggio di sicurezza,
il robot non si muove affatto per quel frame.

COME LEGGERE I NUMERI
La dispersione fra corse e' alta (dipende da quanto sfortunati sono gli incontri lungo il tragitto),
quindi la media di poche corse ha un'incertezza non trascurabile: viene riportato anche l'errore
standard, e la percentuale aggregata e' calcolata sommando frame fermi e frame totali di tutte le corse
invece di mediare le percentuali - cosi' le corse lunghe pesano in proporzione alla loro durata.

USO
    py test_stop.py                          mappa e popolazione di default
    py test_stop.py 300                      300 individui
    py test_stop.py 300 20                   300 individui, 20 corse
    py test_stop.py 300 20 training_mappa_02 su un'altra mappa
    py test_stop.py 300 20 training_mappa_01 base,solo_velocita,completo

Ogni condizione puo' portare la risoluzione della griglia di pianificazione del ROBOT dopo i due punti,
come multiplo di quella della folla (1 = celle da 30px, 2 = celle da 15px; la folla resta sempre a 30px):

    py test_stop.py 0 6 training_mappa_01 base:1,base:2,solo_ai:1,solo_ai:2

Con piu' di una condizione i seed sono condivisi, quindi viene stampato anche il confronto APPAIATO
rispetto alla prima: e' il modo corretto di attribuire un guadagno alla griglia fine piuttosto che
all'AI, perche' le due cose si sommerebbero in una media non appaiata.

La popolazione e' distribuita nelle stesse proporzioni usate per il dataset di training (vedi
ai_predittiva/genera_dataset_previsione.popolazione_da_totale); passando 0 si usano invece i conteggi
esatti salvati nel pannello tecnico di main.py.
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import json
import time
import zlib
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # i moduli condivisi stanno nella radice
for _percorso in (_RADICE, os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)
import testing as t
import main as sim

POPOLAZIONE_DEFAULT = 0        # 0 = usa i conteggi esatti di config_pannello.json
CORSE_DEFAULT = 10
MAPPA_DEFAULT = "training_mappa_01"
CONDIZIONI_DEFAULT = ["base"]  # aggiungi "solo_ai", "solo_velocita", "completo" per confrontarle

FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "test_stop.csv")


def popolazione(totale):
    """totale > 0: conteggi ricavati dalle proporzioni del training. totale == 0: i numeri esatti salvati
    nel pannello tecnico, cioe' la configurazione in cui usi davvero il simulatore."""
    if totale > 0:
        pop = t.popolazione_da_totale(totale)
    else:
        with open(t.FILE_CONFIGURAZIONE_PANNELLO) as f:
            c = json.load(f)
        pop = {k: c[k] for k in ("numero_persone", "numero_corridori",
                                 "numero_persone_ferme", "numero_gruppi")}
    pop.update(t._geometria_salvata())  # raggi robot/persone/sicurezza/lidar dal pannello
    return pop


def _individui(pop):
    """Totale approssimativo di entita' in scena: i gruppi contano per i loro membri, non per uno."""
    return (pop["numero_persone"] + pop["numero_corridori"] + pop["numero_persone_ferme"]
            + int(round(pop["numero_gruppi"] * 3.5)))


def _analizza(etichetta):
    """'base' -> ('base', 1);  'base:2' -> ('base', 2). Il numero dopo i due punti e' la risoluzione della
    griglia di pianificazione del robot, come multiplo di quella della folla (1 = 30px, 2 = 15px)."""
    if ":" in etichetta:
        nome, fattore = etichetta.split(":", 1)
        return nome, int(fattore)
    return etichetta, t.FATTORE_GRIGLIA_DEFAULT


def _corsa(argomenti):
    mappa, pop, etichetta, seed = argomenti
    condizione, fattore = _analizza(etichetta)
    arrivato, frame, fermo = t.esegui_corsa(mappa, condizione, pop, seed, fattore_griglia=fattore)
    return etichetta, seed, arrivato, frame, fermo


def esegui(mappa=MAPPA_DEFAULT, totale=POPOLAZIONE_DEFAULT, corse=CORSE_DEFAULT,
           condizioni=None):
    condizioni = condizioni or CONDIZIONI_DEFAULT
    pop = popolazione(totale)

    print(f"Mappa: {mappa}   |   percorso: angolo alto-sinistra -> angolo basso-destra")
    print(f"Folla: {pop['numero_persone']} persone + {pop['numero_corridori']} corridori + "
          f"{pop['numero_persone_ferme']} ferme + {pop['numero_gruppi']} gruppi "
          f"= ~{_individui(pop)} individui")
    print(f"Geometria: robot {pop['raggio_robot']:.1f}px, persone {pop['raggio_persona']:.1f}px, "
          f"sicurezza {pop['raggio_sicurezza']:.1f}px, lidar {pop['raggio_lidar']:.0f}px")
    print(f"Corse: {corse} per condizione ({', '.join(condizioni)}), accelerazione {t.ACCELERAZIONE}x")
    for e in condizioni:
        nome, fattore = _analizza(e)
        print(f"   {e:<16} {nome}, griglia robot {sim.DIM_NODO // fattore}px "
              f"(folla sempre {sim.DIM_NODO}px)")
    print()

    # stesso insieme di seed per tutte le condizioni: se ne confronti piu' di una, i confronti sono
    # appaiati (stessa folla iniziale) invece che fra scenari scorrelati
    seed = [zlib.crc32(f"{mappa}|{totale}|{i}".encode()) for i in range(corse)]
    compiti = [(mappa, pop, c, s) for c in condizioni for s in seed]

    n_worker = min(8, os.cpu_count() or 4)
    risultati = {c: [] for c in condizioni}
    inizio = time.time()
    fatte = 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for condizione, s, arrivato, frame, fermo in pool.imap_unordered(_corsa, compiti, chunksize=1):
            fatte += 1
            risultati[condizione].append((s, arrivato, frame, fermo))
            print(f"  [{fatte}/{len(compiti)}] {condizione:<14} "
                  f"{t._secondi(frame):6.1f}s   fermo {t._secondi(fermo):5.1f}s "
                  f"({fermo / frame * 100:5.1f}%)"
                  f"{'' if arrivato else '   [NON ARRIVATO: tempo troncato al tetto]'}")

    print(f"\nCompletato in {time.time() - inizio:.0f}s")
    _tabelle(mappa, pop, risultati, condizioni)


def _tabelle(mappa, pop, risultati, condizioni):
    righe_csv = []
    for c in condizioni:
        dati = sorted(risultati[c], key=lambda r: r[0])
        validi = [(f, s) for _, a, f, s in dati if a]

        print(f"\n=== Corse singole - condizione '{c}' ===")
        print(f"{'#':>3}{'seed':>12}{'tempo':>10}{'fermo':>9}{'% fermo':>10}{'arrivato':>10}")
        for i, (s, a, frame, fermo) in enumerate(dati, 1):
            print(f"{i:>3}{s:>12}{t._secondi(frame):>9.1f}s{t._secondi(fermo):>8.1f}s"
                  f"{fermo / frame * 100:>9.1f}%{'si' if a else 'NO':>10}")
            righe_csv.append({"mappa": mappa, "individui": _individui(pop), "condizione": c,
                              "seed": s, "arrivato": "si" if a else "no",
                              "tempo_s": round(t._secondi(frame), 2),
                              "fermo_s": round(t._secondi(fermo), 2),
                              "percento_fermo": round(fermo / frame * 100, 1)})

        if not validi:
            print("  nessuna corsa arrivata a destinazione: statistiche non calcolabili")
            continue

        quote = [s / f * 100 for f, s in validi]
        n = len(quote)
        media = sum(quote) / n
        # errore standard della media: con poche corse e alta dispersione, dice quante cifre della media
        # sono davvero supportate dai dati
        if n > 1:
            dev = (sum((q - media) ** 2 for q in quote) / (n - 1)) ** 0.5
            err = dev / n ** 0.5
        else:
            dev = err = float("nan")
        # aggregata: somma dei frame fermi sul totale dei frame, cosi' le corse lunghe pesano di piu'
        aggregata = sum(s for _, s in validi) / sum(f for f, _ in validi) * 100

        print(f"\n  media % fermo      {media:5.1f}%  (errore standard {err:.1f}%, dev.std {dev:.1f}%)")
        print(f"  aggregata          {aggregata:5.1f}%  (frame fermi totali / frame totali)")
        print(f"  minimo - massimo   {min(quote):5.1f}% - {max(quote):.1f}%")
        print(f"  tempo medio        {sum(t._secondi(f) for f, _ in validi) / n:5.1f}s   "
              f"su {n}/{len(dati)} corse arrivate")

    if righe_csv:
        os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
        with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
            scrittore = csv.DictWriter(f, fieldnames=list(righe_csv[0].keys()))
            scrittore.writeheader()
            scrittore.writerows(righe_csv)
        print(f"\nDati salvati in: {FILE_CSV}")

    # confronto APPAIATO: stesso seed = stessa folla iniziale, quindi la differenza per seed elimina la
    # varianza fra scenari (che e' enorme) e lascia solo l'effetto della condizione. Si confronta tutto
    # con la prima condizione della lista
    if len(condizioni) > 1:
        riferimento = {s: (f, fe) for s, a, f, fe in risultati[condizioni[0]] if a}
        print(f"\n=== Confronto appaiato con '{condizioni[0]}' (stessi seed) ===")
        print(f"{'condizione':>18}{'d tempo':>12}{'err.std':>10}{'vittorie':>11}{'d % fermo':>12}")
        for c in condizioni[1:]:
            coppie = [(riferimento[s][0], f, riferimento[s][1] / riferimento[s][0] * 100, fe / f * 100)
                      for s, a, f, fe in risultati[c] if a and s in riferimento]
            if not coppie:
                print(f"{c:>18}   nessuna coppia valida")
                continue
            diff = [(t._secondi(dopo) - t._secondi(prima)) for prima, dopo, _, _ in coppie]
            n = len(diff)
            media = sum(diff) / n
            err = ((sum((d - media) ** 2 for d in diff) / (n - 1)) ** 0.5 / n ** 0.5) if n > 1 else float("nan")
            vittorie = sum(1 for d in diff if d < 0)  # tempo minore = meglio
            d_fermo = sum(qd - qp for _, _, qp, qd in coppie) / n
            print(f"{c:>18}{media:>+11.1f}s{err:>9.1f}s{vittorie:>7}/{n:<3}{d_fermo:>+11.1f}pp")

    principale = condizioni[0]
    validi = [(f, s) for _, a, f, s in risultati[principale] if a]
    if validi:
        tetto = sum(s for _, s in validi) / sum(f for f, _ in validi) * 100
        print(f"\n>>> TETTO ('{principale}'): il robot perde il {tetto:.1f}% del tempo fermo.")
        print(f"    Nessun controllo, per quanto intelligente, puo' farlo arrivare piu' del "
              f"{tetto:.1f}% prima")
        print(f"    evitando i blocchi - e realisticamente se ne recupera una parte, non tutto.")


if __name__ == "__main__":
    argomenti = sys.argv[1:]
    totale = int(argomenti[0]) if len(argomenti) > 0 else POPOLAZIONE_DEFAULT
    corse = int(argomenti[1]) if len(argomenti) > 1 else CORSE_DEFAULT
    mappa = argomenti[2] if len(argomenti) > 2 else MAPPA_DEFAULT
    condizioni = argomenti[3].split(",") if len(argomenti) > 3 else CONDIZIONI_DEFAULT
    esegui(mappa, totale, corse, condizioni)

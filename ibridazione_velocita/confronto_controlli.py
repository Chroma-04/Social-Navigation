"""Confronto appaiato di piu' controlli contro la stessa linea di base.

Generalizza performance_velocita_classico.py, che confrontava due sole condizioni. Qui ogni
condizione viene confrontata con `base` sugli STESSI semi e sugli stessi livelli di densita',
cosi' i controlli sono confrontabili anche fra loro e non solo ciascuno col riferimento.

COSA SI CONFRONTA
  base               il robot senza controlli: Kalman puro, velocita' nominale costante
  solo_velocita      il controllo di velocita' classico, gia' misurato e risultato nullo
  velocita_libera    lo stesso controllo con l'obiettivo ribaltato: se non c'e' nessuno davanti
                     nemmeno andando al massimo, si va al massimo. Nella versione classica quel
                     ramo era irraggiungibile a strada libera, e il robot non usava mai la banda
                     alta per il motivo per cui esiste
  solo_posizione     controllo di posizione: trasla di lato per aggirare invece di rallentare
  libera_posizione   i due insieme

PERCHE' TUTTE CONTRO base E NON A CASCATA. Confrontare ogni variante con la precedente
accumulerebbe le differenze e renderebbe impossibile attribuire un effetto: con un riferimento
comune ogni numero risponde alla domanda "quanto cambia rispetto al robot senza controlli".

DISEGNO. Semi comuni: a parita' di seme la folla iniziale e' identica in tutte le condizioni,
quindi le differenze non contengono la variabilita' della popolazione. Per ogni livello e ogni
grandezza si riporta media della differenza, errore standard, loro rapporto e coppie vinte; in
coda il test dei segni sui livelli, che chiede se il verso dell'effetto e' costante lungo la
densita' invece di quanto e' grande a una densita' sola.

TRE SEMI PER LIVELLO SONO UNO SCREENING. Bastano a vedere quali condizioni meritano attenzione
e dove; non bastano a stabilire un valore. Una condizione promettente va poi ripetuta con
cinquanta semi a un paio di livelli, come e' stato fatto per il confronto AI del capitolo 5.

Uso:
    py ibridazione_velocita/confronto_controlli.py            screening completo
    py ibridazione_velocita/confronto_controlli.py --corto    3 livelli, 2 semi
    py ibridazione_velocita/confronto_controlli.py --rapido   10 livelli, 1 seme, sola
                                                              libera_posizione: 20 corse

MODALITA' RAPIDA. Un solo seme per livello, diverso da livello a livello, e una sola variante.
Serve a vedere in fretta se l'effetto esiste e con che verso lungo tutta la densita'. NON
distingue quale dei due controlli faccia il lavoro - per quello servono le condizioni separate -
e con una replica per livello ogni singola riga e' rumore: l'unica lettura sensata e' quanti
livelli su dieci vanno nello stesso verso, che e' esattamente cio' che il test dei segni misura.
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
from math import comb

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_QUI = os.path.dirname(os.path.abspath(__file__))
for _percorso in (_QUI, _RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import configurazione as cfg

MAPPA = "povo"
RIFERIMENTO = "base"
VARIANTI = ("solo_velocita", "velocita_libera", "solo_posizione", "libera_posizione")
CONDIZIONI = (RIFERIMENTO,) + VARIANTI
ETICHETTA_SEMI = "ibridazione_velocita"   # stessi semi delle campagne precedenti
NUMERO_SEMI = 3

# corridori al 13% dei corpi mobili, totale 281 invariato: le etichette pop* restano confrontabili
# in numerosita' con le campagne precedenti, ma non in cinematica (velocita' e mescolamento diversi)
POPOLAZIONE_BASE = {"numero_persone": 120, "numero_corridori": 18,
                    "numero_persone_ferme": 116, "numero_gruppi": 27}
FATTORI = [0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20, 1.35, 1.50]

PIXEL_PER_METRO = 100.0 / sim.CM_PER_PIXEL
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "confronto_controlli_povo.csv")


def _file_csv():
    """In modalita' rapida il nome porta la variante, cosi' due prove rapide successive non si
    sovrascrivono a vicenda: sono misure diverse, non versioni della stessa."""
    varianti = tuple(a.split("=", 1)[1] for a in sys.argv if a.startswith("--variante="))
    prefisso = ("mirato" if "--mirato" in sys.argv else
                "replica" if "--replica" in sys.argv else
                "rapido" if "--rapido" in sys.argv else None)
    # il nome porta modalita' e variante: due misure diverse non devono finire sullo stesso
    # file. E' gia' successo di perdere un CSV sovrascrivendolo con quello di un'altra prova
    if prefisso or varianti:
        etichetta = varianti[0] if varianti else "tutte"
        return os.path.join(t.CARTELLA_RISULTATI,
                            f"confronto_{prefisso or 'completo'}_{etichetta}.csv")
    return FILE_CSV


def _popolazione_scalata(fattore):
    scalata = {chiave: max(0, round(valore * fattore)) for chiave, valore in POPOLAZIONE_BASE.items()}
    scalata.update(t._geometria_salvata())
    return scalata


def _una_corsa(argomenti):
    """Una corsa strumentata. Le intercettazioni si installano QUI: con lo spawn di Windows il
    worker re-importa i moduli da zero e non vedrebbe le modifiche fatte nel processo padre."""
    condizione, seed, popolazione, etichetta = argomenti

    traiettoria = []   # posizione del robot a ogni fotogramma, px
    distanze = []      # distanza dal corpo mobile piu' vicino, px

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
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, condizione, popolazione, seed)
    finally:
        t._rileva_e_traccia = traccia_originale

    tempo_s = t._secondi(frame)
    return etichetta, condizione, seed, arrivato, tempo_s, t._secondi(frame_fermo), \
        _metriche(traiettoria, distanze, tempo_s)


def _metriche(traiettoria, distanze, tempo_s):
    """Dalla traiettoria grezza in pixel alle grandezze riportate."""
    if len(traiettoria) < 2:
        return {"numero_fermate": 0, "distanza_media_m": 0.0, "velocita_media_m_s": 0.0}

    # un fotogramma e' fermo se la posizione non cambia; le FERMATE sono i blocchi contigui di
    # fotogrammi fermi, cioe' gli eventi, non i singoli fotogrammi
    percorso_px, fermate, fermo_prima = 0.0, 0, False
    for (x0, y0), (x1, y1) in zip(traiettoria, traiettoria[1:]):
        passo = math.hypot(x1 - x0, y1 - y0)
        percorso_px += passo
        fermo_ora = passo < 1e-6
        if fermo_ora and not fermo_prima:
            fermate += 1
        fermo_prima = fermo_ora

    return {
        "numero_fermate": fermate,
        "distanza_media_m": statistics.mean(distanze) / PIXEL_PER_METRO if distanze else 0.0,
        # velocita' effettiva: distingue "va piano" da "fa piu' strada", che sul solo tempo di
        # percorrenza sono indistinguibili - e il controllo di posizione allunga il percorso
        "velocita_media_m_s": (percorso_px / PIXEL_PER_METRO / tempo_s) if tempo_s > 0 else 0.0,
    }


GRANDEZZE = (("tempo_s", "tempo di percorrenza", "s", True),
             ("fermo_s", "tempo trascorso fermo", "s", True),
             ("numero_fermate", "numero di fermate", "", True),
             ("distanza_media_m", "distanza media dalle persone", "m", False),
             ("velocita_media_m_s", "velocita' media", "m/s", False))


def _segno_p(favorevoli, totali):
    """p bilaterale del test dei segni: quanto e' improbabile questo sbilanciamento fra livelli."""
    k = max(favorevoli, totali - favorevoli)
    return min(1.0, 2 * sum(comb(totali, i) for i in range(k, totali + 1)) / 2 ** totali)


def main():
    global VARIANTI, CONDIZIONI
    corto = "--corto" in sys.argv
    rapido = "--rapido" in sys.argv
    scelte = tuple(a.split("=", 1)[1] for a in sys.argv if a.startswith("--variante="))
    if scelte:
        ignote = [c for c in scelte if c not in t.CONDIZIONI]
        if ignote:
            raise SystemExit(f"condizioni sconosciute: {', '.join(ignote)}")
        VARIANTI = scelte
    elif rapido:
        VARIANTI = ("libera_posizione",)
    CONDIZIONI = (RIFERIMENTO,) + VARIANTI
    # --mirato: solo la finestra di densita' in cui il controllo di posizione ha spazio per
    # agire e abbastanza conflitti da evitare. Sotto pop126 non ci sono fermate da togliere,
    # sopra pop253 lo spazio per traslare si chiude e il controllo peggiora
    mirato = "--mirato" in sys.argv
    replica = "--replica" in sys.argv
    # --livelli=N tiene solo i primi N livelli di densita'. Le due densita' piu' alte costano
    # da sole piu' della meta' del tempo di ogni campagna, e sono quelle dove gia' sappiamo che
    # il controllo peggiora: escluderle triplica le repliche a parita' di attesa
    quanti_livelli = [int(a.split("=", 1)[1]) for a in sys.argv if a.startswith("--livelli=")]
    semi_richiesti = [int(a.split("=", 1)[1]) for a in sys.argv if a.startswith("--semi=")]
    if mirato:
        fattori = [0.45, 0.60, 0.90]      # pop126, pop169, pop253
    elif corto:
        fattori = FATTORI[1::4]
    else:
        fattori = FATTORI
    numero_semi = 2 if corto else (1 if rapido else NUMERO_SEMI)
    if semi_richiesti:
        numero_semi = semi_richiesti[0]
    if quanti_livelli:
        fattori = fattori[:quanti_livelli[0]]

    livelli = [(f"pop{round(sum(POPOLAZIONE_BASE.values()) * f)}", _popolazione_scalata(f))
               for f in fattori]
    if rapido and not mirato:
        # un seme DIVERSO per livello: dieci folle indipendenti invece della stessa ripetuta,
        # cosi' i dieci livelli sono dieci prove separate e il test dei segni ha senso.
        # --replica cambia il sale, quindi genera folle DISGIUNTE da quelle della prima prova:
        # e' la conferma su dati nuovi, non una ripetizione degli stessi
        sale = "replica" if replica else "rapido"
        semi_per_livello = {etichetta: [zlib.crc32(f"{MAPPA}|{sale}|{etichetta}".encode())]
                            for etichetta, _ in livelli}
    else:
        comuni = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(numero_semi)]
        semi_per_livello = {etichetta: comuni for etichetta, _ in livelli}
    semi = sorted({s for elenco in semi_per_livello.values() for s in elenco})
    compiti = [(condizione, seme, popolazione, etichetta)
               for etichetta, popolazione in livelli
               for seme in semi_per_livello[etichetta]
               for condizione in CONDIZIONI]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Confronto dei controlli su {MAPPA}, tutte le condizioni contro '{RIFERIMENTO}'")
    print(f"  {len(livelli)} livelli x {numero_semi} semi x {len(CONDIZIONI)} condizioni "
          f"= {len(compiti)} corse")
    print(f"  {n_worker} processi, accelerazione {t.ACCELERAZIONE}x, "
          f"pedoni {sim.VELOCITA_PERSONA * 2:.1f} m/s, "
          f"corridori {sim.VELOCITA_PERSONA * sim.FATTORE_VELOCITA_CORRIDORE * 2:.1f} m/s, "
          f"robot {sim.VELOCITA_ROBOT * sim.FATTORE_VELOCITA_ROBOT_MIN * 2:.1f}-"
          f"{sim.VELOCITA_ROBOT * sim.FATTORE_VELOCITA_ROBOT_MAX * 2:.1f} m/s")
    if corto:
        print("  MODALITA' CORTA: validazione del banco, non una misura")
    if rapido:
        print("  MODALITA' RAPIDA: una replica per livello. Si legge il verso su dieci livelli,")
        print("  non il valore su uno: ogni riga presa da sola e' rumore")
    print()

    esiti, inizio, completate = {}, time.time(), 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for etichetta, condizione, seed, arrivato, tempo_s, fermo_s, misure in pool.imap_unordered(
                _una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(etichetta, seed, condizione)] = dict(
                arrivato=arrivato, tempo_s=tempo_s, fermo_s=fermo_s, **misure)
            print(f"  [{completate}/{len(compiti)}] {etichetta} / {condizione}: {tempo_s:.1f}s, "
                  f"fermo {fermo_s:.1f}s, {misure['numero_fermate']} fermate "
                  f"({time.time() - inizio:.0f}s)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta([e for e, _ in livelli], semi, esiti)


def _riporta(etichette, semi, esiti):
    righe = []
    for etichetta in etichette:
        for seme in semi:
            riferimento = esiti.get((etichetta, seme, RIFERIMENTO))
            if riferimento is None or not riferimento["arrivato"]:
                continue
            for condizione in VARIANTI:
                variante = esiti.get((etichetta, seme, condizione))
                if variante is None or not variante["arrivato"]:
                    print(f"  coppia scartata: {etichetta} seme {seme} {condizione} non arrivato")
                    continue
                riga = {"mappa": MAPPA, "livello": etichetta, "seme": seme, "condizione": condizione}
                for chiave, _, _, _ in GRANDEZZE:
                    riga[f"{chiave}_base"] = round(riferimento[chiave], 4)
                    riga[f"{chiave}_variante"] = round(variante[chiave], 4)
                righe.append(riga)

    if not righe:
        print("Nessuna coppia valida.")
        return

    for chiave, nome, unita, meno_e_meglio in GRANDEZZE:
        verso = "negativo = la variante MIGLIORA" if meno_e_meglio else "positivo = la variante MIGLIORA"
        print(f"\n=== {nome.upper()} ===   ({verso})")
        print(f"{'condizione':>19}{'diff. media':>13}{'SE':>8}{'rapporto':>10}"
              f"{'vinte':>10}{'livelli':>10}{'p segni':>10}")
        print("-" * 80)
        for condizione in VARIANTI:
            differenze, medie_livello = [], []
            for etichetta in etichette:
                del_livello = [r[f"{chiave}_variante"] - r[f"{chiave}_base"] for r in righe
                               if r["livello"] == etichetta and r["condizione"] == condizione]
                if not del_livello:
                    continue
                differenze += del_livello
                medie_livello.append(statistics.mean(del_livello))
            if len(differenze) < 2:
                continue
            media = statistics.mean(differenze)
            se = statistics.stdev(differenze) / len(differenze) ** 0.5
            vinte = sum(1 for d in differenze if (d < 0) == meno_e_meglio and d != 0)
            favorevoli = sum(1 for m in medie_livello if (m < 0) == meno_e_meglio)
            print(f"{condizione:>19}{media:>+13.3f}{se:>8.3f}"
                  f"{abs(media / se) if se else float('inf'):>10.2f}"
                  f"{vinte:>7}/{len(differenze):<3}{favorevoli:>7}/{len(medie_livello):<3}"
                  f"{_segno_p(favorevoli, len(medie_livello)):>10.4f}")
        if unita:
            print(f"  unita': {unita}")

    print("\nRapporto sotto 1: indistinguibile dal rumore. Sopra 2: effetto stabilito.")
    print("Tre semi per livello sono uno screening: dicono DOVE guardare, non quanto vale.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    percorso_csv = _file_csv()
    with open(percorso_csv, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {percorso_csv}")


if __name__ == "__main__":
    main()

"""Misura il TETTO del controllo di posizione: la stessa manovra con previsione perfetta.

DISEGNO. Tre bracci sulle STESSE scene, appaiati per (livello, seme):

    riferimento         nessun controllo, velocita' costante        gia' in archivio
    posizione           controllo di posizione su Kalman            gia' in archivio
    posizione+ORACOLO   controllo di posizione su posizione vera    <- si misura qui

I primi due bracci non vengono rieseguiti: sono le colonne _base e _variante di
solo_posizione_povo.csv, e le corse sono deterministiche a parita' di seme. Si paga quindi solo
il terzo braccio, un terzo del calcolo, e l'appaiamento e' esatto per costruzione.

COSA DECIDE. Il criterio e' quello di tutto il lavoro: rapporto fra la media della differenza
appaiata e il suo errore standard. Sotto 1 la grandezza e' indistinguibile da zero, sopra 2 e'
stabilita; in piu' il test dei segni sui livelli di densita', che sono blocchi indipendenti.
La grandezza che conta e' il NUMERO DI FERMATE: e' l'unica su cui il controllo di posizione
abbia mai prodotto un effetto stabilito (-5,83, rapporto 2,85, 8 livelli su 8).

    py ai_posizione/misura_oracolo_posizione.py --semi=1   versione corta, 8 coppie
    py ai_posizione/misura_oracolo_posizione.py            campagna piena, 24 coppie
    py ai_posizione/misura_oracolo_posizione.py --prova    stampa l'appaiamento, non simula
    py ai_posizione/misura_oracolo_posizione.py --tabella  ricalcola dal CSV gia' scritto
    py ai_posizione/misura_oracolo_posizione.py --verifica riesegue 2 scene del braccio Kalman e
                                                           controlla che i numeri in archivio
                                                           siano ancora esattamente quelli
    py ai_posizione/misura_oracolo_posizione.py --orizzonte-reale
                                                           rifa' entrambi i bracci con
                                                           l'orizzonte che il controllo avrebbe
                                                           dal vivo (48 corse)
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import math
import time
import statistics
import multiprocessing as mp
from collections import defaultdict

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_QUI = os.path.dirname(os.path.abspath(__file__))
for _percorso in (_QUI, _RADICE, os.path.join(_RADICE, "ibridazione_velocita"),
                  os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import confronto_controlli as cc
import oracolo_posizione as op

CONDIZIONE = "solo_posizione"
FILE_ARCHIVIO = os.path.join(t.CARTELLA_RISULTATI, "solo_posizione_povo.csv")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "oracolo_posizione_povo.csv")
FILE_CSV_ORIZZONTE = os.path.join(t.CARTELLA_RISULTATI,
                                  "oracolo_posizione_orizzonte_reale_povo.csv")
AREA_M2 = 1997.0     # superficie calpestabile di povo, contata sulle celle libere
TOLLERANZA = 1e-4    # i CSV sono arrotondati a quattro decimali: sotto questa soglia e' identita'

BRACCI = ("riferimento", "posizione", "oracolo")
MISURATE = ("tempo_s", "fermo_s", "numero_fermate", "distanza_media_m", "velocita_media_m_s")

# nome, etichetta, cifre, verso (+1 = valore piu' alto significa "meglio")
GRANDEZZE = [
    ("numero_fermate",     "numero di fermate",      2, -1),
    ("fermo_s",            "tempo trascorso fermo",  2, -1),
    ("tempo_s",            "tempo di percorrenza",   2, -1),
    ("distanza_media_m",   "distanza dalle persone", 3, +1),
    ("velocita_media_m_s", "velocita' media",        3, +1),
    ("lunghezza_m",        "lunghezza del percorso", 2, -1),
]


def _corsa(argomenti):
    """Una corsa strumentata. Le intercettazioni si installano QUI: con lo spawn di Windows il
    worker re-importa i moduli da zero e non vedrebbe una patch fatta nel processo padre."""
    etichetta, seme, popolazione, con_oracolo, orizzonte = argomenti
    traiettoria, distanze = [], []

    # l'orizzonte si riscrive QUI per la stessa ragione delle intercettazioni: e' un globale di
    # modulo letto dentro offset_evitamento_predittivo, e il worker re-importa main da zero
    orizzonte_originale = sim.PREVISIONE_EVITAMENTO_FRAME
    if orizzonte is not None:
        sim.PREVISIONE_EVITAMENTO_FRAME = orizzonte
    ripristina_oracolo = op.installa(t, sim) if con_oracolo else (lambda: None)
    traccia_originale = t._rileva_e_traccia   # letta DOPO l'oracolo: le due patch si annidano

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                    storico_posizioni):
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
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", CONDIZIONE, popolazione, seme)
    finally:
        t._rileva_e_traccia = traccia_originale
        ripristina_oracolo()
        sim.PREVISIONE_EVITAMENTO_FRAME = orizzonte_originale

    tempo_s = t._secondi(frame)
    return etichetta, seme, con_oracolo, arrivato, tempo_s, t._secondi(frame_fermo), \
        cc._metriche(traiettoria, distanze, tempo_s)


def _archivio():
    """Le corse gia' fatte: colonne _base (riferimento) e _variante (posizione su Kalman)."""
    per_chiave = {}
    with open(FILE_ARCHIVIO, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            per_chiave[(r["livello"], int(r["seme"]))] = r
    return per_chiave


def _compiti(archivio, quanti_semi, orizzonte=None, bracci=(True,)):
    """Le corse oracolo da eseguire: gli stessi livelli e semi dell'archivio, troncati ai primi
    'quanti_semi' per livello - cosi' la versione corta e' un sottoinsieme esatto della lunga."""
    livelli = [f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}" for f in cc.FATTORI]
    popolazioni = {e: cc._popolazione_scalata(f) for e, f in zip(livelli, cc.FATTORI)}
    semi_per_livello = defaultdict(list)
    for etichetta, seme in sorted(archivio, key=lambda k: (int(k[0][3:]), k[1])):
        semi_per_livello[etichetta].append(seme)
    return [(etichetta, seme, popolazioni[etichetta], con_oracolo, orizzonte)
            for etichetta in livelli
            for seme in semi_per_livello.get(etichetta, [])[:quanti_semi]
            for con_oracolo in bracci]


# ------------------------------------------------------------------------------------------
# Statistica
# ------------------------------------------------------------------------------------------
def _rapporto(valori):
    """Media, errore standard, loro rapporto e intervallo di confidenza al 95%."""
    if len(valori) < 2:
        return (valori[0] if valori else 0.0), float("nan"), float("nan"), (float("nan"),) * 2
    media = statistics.mean(valori)
    errore = statistics.stdev(valori) / math.sqrt(len(valori))
    rapporto = abs(media) / errore if errore > 0 else float("inf")
    return media, errore, rapporto, (media - 1.96 * errore, media + 1.96 * errore)


def _giudizio(r):
    if math.isnan(r):
        return "?"
    return "STABILITO" if r >= 2.0 else ("tendenza" if r >= 1.3 else "NULLO")


def _segni(medie):
    """Test dei segni a due code su n livelli indipendenti, p=0,5 sotto l'ipotesi nulla."""
    n, k = len(medie), sum(1 for v in medie if v > 0)
    estremo = max(k, n - k)
    coda = sum(math.comb(n, i) for i in range(estremo, n + 1)) / 2 ** n
    return k, n, min(1.0, 2 * coda)


def _leggi_righe(percorso=None):
    with open(percorso or FILE_CSV, newline="", encoding="utf-8") as fh:
        righe = list(csv.DictReader(fh))
    # la lunghezza del percorso non e' misurata a parte: e' velocita' media per tempo. Serve
    # perche' il controllo di posizione devia, e sul solo tempo "va piano" e "fa piu' strada"
    # sono indistinguibili
    for r in righe:
        for braccio in BRACCI:
            r[f"lunghezza_m_{braccio}"] = (float(r[f"velocita_media_m_s_{braccio}"])
                                           * float(r[f"tempo_s_{braccio}"]))
    return righe


def _confronto(righe, sinistra, destra, titolo):
    """Tabella appaiata fra due bracci. Segno + = 'destra' fa meglio di 'sinistra'."""
    per_livello = defaultdict(lambda: defaultdict(list))
    print("\n" + titolo)
    print(f"  {'grandezza':<24} {'medie assolute':>21} {'differenza':>12} {'errore':>8} "
          f"{'rapp.':>6} {'IC 95%':>22}  giudizio")
    for chiave, etichetta, cifre, verso in GRANDEZZE:
        differenze, valori_a, valori_b = [], [], []
        for r in righe:
            a, b = float(r[f"{chiave}_{sinistra}"]), float(r[f"{chiave}_{destra}"])
            differenze.append(verso * (b - a))
            valori_a.append(a)
            valori_b.append(b)
            per_livello[r["livello"]][chiave].append(verso * (b - a))
        m, e, rap, (lo, hi) = _rapporto(differenze)
        assoluti = f"{statistics.mean(valori_a):.{cifre}f} -> {statistics.mean(valori_b):.{cifre}f}"
        ic = f"[{lo:+.{cifre}f}, {hi:+.{cifre}f}]"
        print(f"  {etichetta:<24} {assoluti:>21} {m:>+12.{cifre}f} {e:>8.{cifre}f} "
              f"{rap:>6.2f} {ic:>22}  {_giudizio(rap)}")

    print("\n  test dei segni sui livelli di densita' (blocchi indipendenti)")
    for chiave, etichetta, _, _ in GRANDEZZE:
        medie = [statistics.mean(per_livello[l][chiave]) for l in per_livello]
        k, n, p = _segni(medie)
        print(f"    {etichetta:<24} {k}/{n} livelli favorevoli   p={p:.4f}")


def tabella(percorso=None):
    righe = _leggi_righe(percorso)
    livelli = sorted({r["livello"] for r in righe}, key=lambda l: int(l[3:]))
    print("=" * 110)
    print("TETTO DEL CONTROLLO DI POSIZIONE: Kalman contro previsione perfetta")
    print(f"{len(righe)} scene appaiate, {len(livelli)} livelli di densita', "
          f"condizione '{CONDIZIONE}' su povo")
    print("=" * 110)

    _confronto(righe, "riferimento", "posizione",
               "A) CONTROLLO DI POSIZIONE contro RIFERIMENTO   (quanto vale il controllo)")
    _confronto(righe, "posizione", "oracolo",
               "B) ORACOLO contro CONTROLLO DI POSIZIONE   (quanto vale la previsione perfetta:"
               " E' IL TETTO)")
    _confronto(righe, "riferimento", "oracolo",
               "C) ORACOLO contro RIFERIMENTO   (il massimo ottenibile da questo canale)")

    print("\nPER DENSITA' (persone per 100 m2): numero di fermate nei tre bracci")
    print(f"  {'livello':>8} {'dens.':>6} {'n':>3} {'riferim.':>9} {'posizione':>10} "
          f"{'oracolo':>9} {'orac-pos':>9}")
    for livello in livelli:
        gruppo = [r for r in righe if r["livello"] == livello]
        v = {b: statistics.mean(float(r[f"numero_fermate_{b}"]) for r in gruppo) for b in BRACCI}
        print(f"  {livello:>8} {int(livello[3:]) / AREA_M2 * 100:>6.1f} {len(gruppo):>3} "
              f"{v['riferimento']:>9.2f} {v['posizione']:>10.2f} {v['oracolo']:>9.2f} "
              f"{v['oracolo'] - v['posizione']:>+9.2f}")


# ------------------------------------------------------------------------------------------
# Verifica che l'indirezione non abbia cambiato il braccio Kalman
# ------------------------------------------------------------------------------------------
def verifica(quante=2):
    """previsione_per_evitamento e' stata introdotta come indirezione con default identico alla
    funzione che sostituisce, quindi il braccio Kalman deve rigenerare esattamente i numeri in
    archivio. Se non lo fa, appaiare le corse nuove con quelle archiviate non e' lecito."""
    archivio = _archivio()
    chiavi = sorted(archivio, key=lambda k: (int(k[0][3:]), k[1]))[:quante]
    livelli = {f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}": cc._popolazione_scalata(f)
               for f in cc.FATTORI}
    print(f"riesecuzione di {len(chiavi)} scene del braccio Kalman, confronto con l'archivio\n")
    tutto_ok = True
    for etichetta, seme in chiavi:
        *_, tempo_s, fermo_s, metriche = _corsa((etichetta, seme, livelli[etichetta], False, None))
        r = archivio[(etichetta, seme)]
        atteso = {"tempo": float(r["tempo_s_variante"]),
                  "fermo": float(r["fermo_s_variante"]),
                  "fermate": float(r["numero_fermate_variante"]),
                  "distanza": float(r["distanza_media_m_variante"])}
        ottenuto = {"tempo": tempo_s, "fermo": fermo_s,
                    "fermate": float(metriche["numero_fermate"]),
                    "distanza": metriche["distanza_media_m"]}
        ok = all(abs(atteso[k] - ottenuto[k]) <= TOLLERANZA for k in atteso)
        tutto_ok = tutto_ok and ok
        print(f"  [{'OK' if ok else 'DIVERSO'}] {etichetta:>7} seme {seme:>11}")
        for k in atteso:
            print(f"          {k:<9} {ottenuto[k]:>10.4f} vs {atteso[k]:>10.4f}"
                  f"   scarto {abs(atteso[k] - ottenuto[k]):.6f}")
    print("\n" + ("IDENTICO: l'indirezione non ha cambiato nulla, l'appaiamento con l'archivio "
                  "e' lecito." if tutto_ok else
                  "DIVERGENZA: il braccio Kalman non e' piu' quello archiviato. NON appaiare."))


def orizzonte_reale(quanti_semi):
    """Rifa' il confronto con l'orizzonte che il controllo avrebbe DAL VIVO.

    offset_evitamento_predittivo legge PREVISIONE_EVITAMENTO_FRAME in passi di simulazione e non
    lo divide per ACCELERAZIONE, mentre la macchia del pianificatore lo fa (testing.py). Sui
    banchi accelerati il controllo di posizione guarda quindi cinque volte piu' avanti del
    dovuto, e a un orizzonte cosi' lungo la posizione vera diverge dalla retta del Kalman molto
    piu' che a un secondo. Il confronto di E12 resta appaiato e valido PER QUELL'ORIZZONTE, ma
    il tetto negativo potrebbe essere un artefatto della proiezione lunga.

    Qui si riesegue tutto a orizzonte corretto. ENTRAMBI i bracci vanno rifatti: quello Kalman in
    archivio e' stato prodotto a orizzonte 60, quindi non e' piu' appaiabile. Il braccio di
    riferimento invece resta buono, perche' il controllo di posizione li' e' spento e l'orizzonte
    non lo tocca. Sono 48 corse invece di 24.
    """
    archivio = _archivio()
    orizzonte = max(1, sim.PREVISIONE_EVITAMENTO_FRAME // t.ACCELERAZIONE)
    compiti = _compiti(archivio, quanti_semi, orizzonte, bracci=(False, True))
    print(f"orizzonte {sim.PREVISIONE_EVITAMENTO_FRAME} -> {orizzonte} passi "
          f"({orizzonte / 60 * t.ACCELERAZIONE:.1f} s reali)")
    print(f"{len(compiti)} corse: entrambi i bracci rifatti, il riferimento resta dall'archivio")

    avvio = time.time()
    per_chiave = defaultdict(dict)
    with mp.Pool(min(8, os.cpu_count() or 4)) as pool:
        for n, esito in enumerate(pool.imap_unordered(_corsa, compiti), 1):
            etichetta, seme, con_oracolo, _, tempo_s, fermo_s, metriche = esito
            per_chiave[(etichetta, seme)]["oracolo" if con_oracolo else "posizione"] = {
                "tempo_s": tempo_s, "fermo_s": fermo_s,
                "numero_fermate": metriche["numero_fermate"],
                "distanza_media_m": metriche["distanza_media_m"],
                "velocita_media_m_s": metriche["velocita_media_m_s"]}
            print(f"  [{n}/{len(compiti)}] {etichetta} seme {seme} "
                  f"{'oracolo' if con_oracolo else 'kalman':>7}: {tempo_s:.1f}s "
                  f"({(time.time() - avvio) / 60:.1f} min trascorsi)", flush=True)

    campi = ["mappa", "livello", "seme", "condizione"]
    for chiave in MISURATE:
        campi += [f"{chiave}_{braccio}" for braccio in BRACCI]
    with open(FILE_CSV_ORIZZONTE, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=campi)
        scrittore.writeheader()
        for (etichetta, seme), bracci in sorted(per_chiave.items(),
                                                key=lambda kv: (int(kv[0][0][3:]), kv[0][1])):
            if len(bracci) < 2:
                continue  # una corsa persa: la coppia non e' appaiabile e si scarta
            a = archivio[(etichetta, seme)]
            riga = {"mappa": "povo", "livello": etichetta, "seme": seme, "condizione": CONDIZIONE}
            for chiave in MISURATE:
                riga[f"{chiave}_riferimento"] = a[f"{chiave}_base"]
                for braccio in ("posizione", "oracolo"):
                    riga[f"{chiave}_{braccio}"] = round(float(bracci[braccio][chiave]), 4)
            scrittore.writerow(riga)
    print("")
    print(f"Scritto {FILE_CSV_ORIZZONTE}  ({(time.time() - avvio) / 60:.1f} min totali)")
    print("")
    tabella(FILE_CSV_ORIZZONTE)


def principale():
    quanti_semi = 3
    for a in sys.argv[1:]:
        if a.startswith("--semi="):
            quanti_semi = int(a.split("=", 1)[1])

    if "--tabella" in sys.argv:
        tabella(FILE_CSV_ORIZZONTE if "--orizzonte-reale" in sys.argv else FILE_CSV)
        return
    if "--verifica" in sys.argv:
        verifica()
        return
    if "--orizzonte-reale" in sys.argv:
        orizzonte_reale(quanti_semi)
        return

    archivio = _archivio()
    compiti = _compiti(archivio, quanti_semi)
    print(f"{len(compiti)} corse oracolo, appaiate con altrettante coppie gia' in archivio")
    if "--prova" in sys.argv:
        for etichetta, seme, *_ in compiti:
            r = archivio[(etichetta, seme)]
            print(f"  {etichetta:>8} seme {seme:>12}   riferimento {r['tempo_s_base']:>9} s"
                  f"   posizione {r['tempo_s_variante']:>9} s")
        return

    avvio = time.time()
    risultati = []
    with mp.Pool(min(8, os.cpu_count() or 4)) as pool:
        for n, esito in enumerate(pool.imap_unordered(_corsa, compiti), 1):
            risultati.append(esito)
            print(f"  [{n}/{len(compiti)}] {esito[0]} seme {esito[1]}: {esito[4]:.1f}s "
                  f"({(time.time() - avvio) / 60:.1f} min trascorsi)", flush=True)

    campi = ["mappa", "livello", "seme", "condizione"]
    for chiave in MISURATE:
        campi += [f"{chiave}_{braccio}" for braccio in BRACCI]
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as fh:
        scrittore = csv.DictWriter(fh, fieldnames=campi)
        scrittore.writeheader()
        for etichetta, seme, _, arrivato, tempo_s, fermo_s, metriche in risultati:
            a = archivio[(etichetta, seme)]
            oracolo = {"tempo_s": tempo_s, "fermo_s": fermo_s,
                       "numero_fermate": metriche["numero_fermate"],
                       "distanza_media_m": metriche["distanza_media_m"],
                       "velocita_media_m_s": metriche["velocita_media_m_s"]}
            riga = {"mappa": "povo", "livello": etichetta, "seme": seme, "condizione": CONDIZIONE}
            for chiave in MISURATE:
                riga[f"{chiave}_riferimento"] = a[f"{chiave}_base"]
                riga[f"{chiave}_posizione"] = a[f"{chiave}_variante"]
                riga[f"{chiave}_oracolo"] = round(float(oracolo[chiave]), 4)
            scrittore.writerow(riga)
    print(f"\nScritto {FILE_CSV}  ({(time.time() - avvio) / 60:.1f} min totali)\n")
    tabella()


if __name__ == "__main__":
    mp.freeze_support()
    principale()

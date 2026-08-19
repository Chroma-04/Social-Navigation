"""Quanto vale, per il controllo di velocita', conoscere il futuro.

Esegue il solo braccio ORACOLO e lo appaia con i dati gia' raccolti in
risultati_test/confronto_completo_velocita_libera.csv, che per ogni coppia (densita', seme)
contiene sia `base` sia `velocita_libera`. Gli stessi semi danno la stessa folla, quindi le tre
condizioni sono confrontabili senza rieseguire le prime due: 24 corse invece di 72.

LE DUE DOMANDE, in ordine di importanza:

  1. ORACOLO contro VELOCITA_LIBERA. E' la domanda vera: quanto guadagnerebbe il controllo se
     la previsione fosse perfetta? E' il TETTO di qualunque ibridazione che passi per la
     previsione. Se e' vicino a zero, addestrare una rete non serve, e lo si sa senza averla
     addestrata. Se e' grande, dice quanto margine c'e' e quanto vale provarci.

  2. ORACOLO contro BASE. Il guadagno complessivo del controllo perfetto rispetto al robot
     senza controlli. Serve a collocare il tetto sulla stessa scala delle misure gia' fatte.

Si misura inoltre il RESIDUO DI DECISIONE. A ogni chiamata del controllo si calcola il fattore
due volte, con l'oracolo e col Kalman, sulla stessa identica scena: la differenza e' cio' che
una rete dovrebbe imparare a produrre. Sono due informazioni diverse e vanno lette insieme:

  - se i due fattori quasi non differiscono, non c'e' niente da imparare - il Kalman sbaglia
    le posizioni in modi che non cambiano la decisione;
  - se differiscono spesso ma le prestazioni no, le decisioni diverse si compensano lungo il
    percorso, ed e' lo stesso meccanismo per cui nel capitolo 5 il 12,5% di previsione in piu'
    non arrivava ai tempi.

Uso:
    py ai_controllo/misura_oracolo.py
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
_QUI = os.path.dirname(os.path.abspath(__file__))
for _percorso in (_QUI, _RADICE, os.path.join(_RADICE, "TEST"),
                  os.path.join(_RADICE, "ai_predittiva"),
                  os.path.join(_RADICE, "ibridazione_velocita")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim
import oracolo
import configurazione as cfg

MAPPA = "povo"
CONDIZIONE = "velocita_libera"      # l'oracolo sostituisce la previsione DENTRO questa condizione
ETICHETTA_SEMI = "ibridazione_velocita"
NUMERO_SEMI = 3
FATTORI = [0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20]   # le 8 densita' gia' misurate
POPOLAZIONE_BASE = {"numero_persone": 120, "numero_corridori": 18,
                    "numero_persone_ferme": 116, "numero_gruppi": 27}

PIXEL_PER_METRO = 100.0 / sim.CM_PER_PIXEL
FILE_ESISTENTE = os.path.join(t.CARTELLA_RISULTATI, "confronto_completo_velocita_libera.csv")
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "oracolo_velocita_povo.csv")
SOGLIA_DECISIONE = 0.05   # sotto questa differenza i due fattori si considerano la stessa scelta


def _popolazione_scalata(fattore):
    scalata = {k: max(0, round(v * fattore)) for k, v in POPOLAZIONE_BASE.items()}
    scalata.update(t._geometria_salvata())
    return scalata


def _una_corsa(argomenti):
    """Una corsa col controllo oracolo, strumentata anche sul residuo di decisione."""
    seed, popolazione, etichetta = argomenti

    traiettoria, distanze, residui = [], [], []

    traccia_base = t._rileva_e_traccia

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
        return traccia_base(tutte_le_persone, robot_x, robot_y, griglia,
                            tracciamento_lidar, storico_posizioni)

    t._rileva_e_traccia = _con_misure
    ripristina = oracolo.installa(t, sim)          # installa DOPO, cosi' avvolge la misura

    funzione_oracolo = sim.previsione_per_controllo
    funzione_kalman = sim.previsione_posizione_kalman
    fattore_base = sim.fattore_velocita_da_conflitto

    def _con_residuo(*args, **kwargs):
        f_oracolo = fattore_base(*args, **kwargs)
        # stessa scena, stessa funzione, sola previsione diversa: la differenza e' imputabile
        # unicamente all'errore di previsione
        sim.previsione_per_controllo = funzione_kalman
        try:
            f_kalman = fattore_base(*args, **kwargs)
        finally:
            sim.previsione_per_controllo = funzione_oracolo
        residui.append((f_oracolo, f_kalman))
        return f_oracolo

    sim.fattore_velocita_da_conflitto = _con_residuo
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, CONDIZIONE, popolazione, seed)
    finally:
        sim.fattore_velocita_da_conflitto = fattore_base
        ripristina()
        t._rileva_e_traccia = traccia_base

    tempo_s = t._secondi(frame)
    return etichetta, seed, arrivato, tempo_s, t._secondi(frame_fermo), \
        _metriche(traiettoria, distanze, tempo_s), _residuo(residui)


def _metriche(traiettoria, distanze, tempo_s):
    if len(traiettoria) < 2:
        return {"numero_fermate": 0, "distanza_media_m": 0.0, "velocita_media_m_s": 0.0}
    percorso_px, fermate, fermo_prima = 0.0, 0, False
    for (x0, y0), (x1, y1) in zip(traiettoria, traiettoria[1:]):
        passo = math.hypot(x1 - x0, y1 - y0)
        percorso_px += passo
        fermo_ora = passo < 1e-6
        if fermo_ora and not fermo_prima:
            fermate += 1
        fermo_prima = fermo_ora
    return {"numero_fermate": fermate,
            "distanza_media_m": statistics.mean(distanze) / PIXEL_PER_METRO if distanze else 0.0,
            "velocita_media_m_s": (percorso_px / PIXEL_PER_METRO / tempo_s) if tempo_s > 0 else 0.0}


def _residuo(coppie):
    """Quanto le due previsioni portano a decisioni diverse."""
    if not coppie:
        return {"frazione_diverse": 0.0, "residuo_medio": 0.0, "residuo_max": 0.0,
                "oracolo_piu_veloce": 0.0}
    differenze = [o - k for o, k in coppie]
    diverse = [d for d in differenze if abs(d) > SOGLIA_DECISIONE]
    return {"frazione_diverse": len(diverse) / len(coppie) * 100,
            "residuo_medio": statistics.mean(abs(d) for d in differenze),
            "residuo_max": max(abs(d) for d in differenze),
            # verso del disaccordo: l'oracolo spinge o frena rispetto al Kalman?
            "oracolo_piu_veloce": sum(1 for d in differenze if d > SOGLIA_DECISIONE)
                                  / max(1, len(diverse)) * 100}


def _carica_esistenti():
    """base e velocita_libera per (densita', seme), dalla campagna gia' eseguita."""
    if not os.path.exists(FILE_ESISTENTE):
        raise SystemExit(f"manca {FILE_ESISTENTE}: esegui prima confronto_controlli.py "
                         f"--semi=3 --livelli=8 --variante=velocita_libera")
    esistenti = {}
    for r in csv.DictReader(open(FILE_ESISTENTE, encoding="utf-8")):
        esistenti[(r["livello"], int(r["seme"]))] = r
    return esistenti


CAMPI = ("tempo_s", "fermo_s", "numero_fermate", "distanza_media_m", "velocita_media_m_s")
NOMI = {"tempo_s": "tempo di percorrenza", "fermo_s": "tempo trascorso fermo",
        "numero_fermate": "numero di fermate", "distanza_media_m": "distanza dalle persone",
        "velocita_media_m_s": "velocita' media"}
MENO_E_MEGLIO = {"tempo_s": True, "fermo_s": True, "numero_fermate": True,
                 "distanza_media_m": False, "velocita_media_m_s": False}


def main():
    esistenti = _carica_esistenti()
    livelli = [(f"pop{round(sum(POPOLAZIONE_BASE.values()) * f)}", _popolazione_scalata(f))
               for f in FATTORI]
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(NUMERO_SEMI)]
    compiti = [(seme, popolazione, etichetta)
               for etichetta, popolazione in livelli for seme in semi]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Controllo ORACOLO su {MAPPA}: quanto vale conoscere il futuro")
    print(f"  {len(livelli)} densita' x {NUMERO_SEMI} semi = {len(compiti)} corse "
          f"(base e {CONDIZIONE} presi da {os.path.basename(FILE_ESISTENTE)})")
    print(f"  {n_worker} processi, accelerazione {t.ACCELERAZIONE}x\n")

    esiti, inizio = {}, time.time()
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for i, r in enumerate(pool.imap_unordered(_una_corsa, compiti, chunksize=1), 1):
            etichetta, seed, arrivato, tempo_s, fermo_s, misure, res = r
            esiti[(etichetta, seed)] = (arrivato, tempo_s, fermo_s, misure, res)
            print(f"  [{i}/{len(compiti)}] {etichetta}: {tempo_s:.1f}s, "
                  f"decisioni diverse {res['frazione_diverse']:.0f}%, "
                  f"residuo medio {res['residuo_medio']:.3f} ({time.time() - inizio:.0f}s)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta([e for e, _ in livelli], semi, esiti, esistenti)


def _riporta(etichette, semi, esiti, esistenti):
    righe, confronti = [], {("oracolo", rif): {c: [] for c in CAMPI} for rif in ("base", "controllo")}
    residui = []
    for etichetta in etichette:
        for seme in semi:
            esito = esiti.get((etichetta, seme))
            vecchio = esistenti.get((etichetta, seme))
            if esito is None or vecchio is None:
                continue
            arrivato, tempo_s, fermo_s, misure, res = esito
            if not arrivato:
                print(f"  scartata: {etichetta} seme {seme}, oracolo non arrivato")
                continue
            oracolo_v = {"tempo_s": tempo_s, "fermo_s": fermo_s, **misure}
            residui.append(res)
            riga = {"mappa": MAPPA, "livello": etichetta, "seme": seme}
            for c in CAMPI:
                b, ctrl, o = float(vecchio[f"{c}_base"]), float(vecchio[f"{c}_variante"]), oracolo_v[c]
                confronti[("oracolo", "base")][c].append(o - b)
                confronti[("oracolo", "controllo")][c].append(o - ctrl)
                riga[f"{c}_base"] = round(b, 4)
                riga[f"{c}_controllo"] = round(ctrl, 4)
                riga[f"{c}_oracolo"] = round(o, 4)
            riga.update({k: round(v, 4) for k, v in res.items()})
            righe.append(riga)

    if not righe:
        print("Nessuna coppia valida.")
        return

    for rif, titolo in (("controllo", "ORACOLO contro CONTROLLO CON KALMAN  (il TETTO dell'ibridazione)"),
                        ("base", "ORACOLO contro ROBOT SENZA CONTROLLI")):
        print(f"\n=== {titolo} ===")
        print(f"{'grandezza':>24}{'differenza':>13}{'SE':>8}{'rapporto':>10}{'vinte':>10}")
        print("-" * 66)
        for c in CAMPI:
            d = confronti[("oracolo", rif)][c]
            m = statistics.mean(d)
            se = statistics.stdev(d) / len(d) ** 0.5 if len(d) > 1 else float("nan")
            vinte = sum(1 for x in d if (x < 0) == MENO_E_MEGLIO[c] and x != 0)
            print(f"{NOMI[c]:>24}{m:>+13.3f}{se:>8.3f}"
                  f"{abs(m / se) if se and se == se and se > 0 else 0:>10.2f}{vinte:>7}/{len(d):<3}")
        print("  (negativo = meglio per tempo, fermo e fermate; positivo per distanza e velocita')")

    print("\n=== RESIDUO DI DECISIONE ===")
    print(f"  fotogrammi con decisione diversa: {statistics.mean(r['frazione_diverse'] for r in residui):.1f}%")
    print(f"  differenza media del fattore:     {statistics.mean(r['residuo_medio'] for r in residui):.3f}")
    print(f"  differenza massima osservata:     {max(r['residuo_max'] for r in residui):.3f}")
    print(f"  quando differiscono, l'oracolo spinge nel "
          f"{statistics.mean(r['oracolo_piu_veloce'] for r in residui):.0f}% dei casi")
    print("\n  Poche decisioni diverse -> non c'e' nulla da imparare: il Kalman sbaglia le")
    print("  posizioni in modi che non cambiano la scelta. Molte decisioni diverse ma")
    print("  prestazioni uguali -> le differenze si compensano lungo il percorso.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola corsa: {FILE_CSV}")
    percorso_cfg = cfg.salva_accanto(FILE_CSV, cfg.impronta(
        sim, t, popolazione_base=POPOLAZIONE_BASE, fattori=FATTORI, semi=semi,
        condizioni=["base", CONDIZIONE, "oracolo"], geometria=t._geometria_salvata()))
    print(f"Configurazione: {percorso_cfg}")


if __name__ == "__main__":
    main()

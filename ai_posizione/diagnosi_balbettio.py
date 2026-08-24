"""Perche' la previsione perfetta PEGGIORA il controllo di posizione: si guarda dentro.

E12 ha misurato che l'oracolo aumenta le fermate di 4,17 SENZA aumentare il tempo fermo. Il banco
esegue la STESSA scena due volte, una con Kalman e una con l'oracolo, e apre il controllo per
capire da dove viene il peggioramento. Stesso seme, quindi folla identica: la differenza e' sua.

PRIMA IPOTESI, SMENTITA DALLA MISURA. Si era proposto che il robot BALBETTASSE: il controllo
ricampiona i candidati a ogni fotogramma, e una previsione che salta gli farebbe cambiare idea di
continuo invece di impegnarsi in una manovra. I numeri dicono il contrario. Con l'oracolo il punto
previsto salta MENO (-68% sul medio, -87% sul mediano) e le inversioni di manovra calano da 5,2%
a 1,5%. La previsione instabile e' quella del KALMAN, non quella vera: proiettando in retta per
sessanta passi, il filtro moltiplica per sessanta il rumore della velocita' stimata, e il punto
previsto rimbalza. La posizione vera segue il percorso pianificato e scivola liscia.

SECONDA IPOTESI, quella che questo banco misura adesso. Se il campo di rischio diventa coerente,
il robot smette di ignorarlo e manovra DI PIU' - e i fotogrammi con manovra salgono infatti del
22%. Ma la traslazione si SOVRAPPONE al passo lungo il percorso invece di sostituirlo: la sua
componente lungo la direzione di marcia si somma all'avanzamento se e' positiva e lo consuma se
e' negativa. Un robot che si scansa meglio avanza di meno, e quando i due contributi si elidono
il passo netto e' nullo e _metriche conta una fermata senza che nessun arresto sia scattato.

LE GRANDEZZE. INGRESSO: di quanto si sposta fra fotogrammi il punto previsto all'orizzonte pieno.
USCITA: angolo fra manovre consecutive e quota di inversioni oltre 90 gradi. COSTO: componente
della manovra lungo la marcia, quota di manovre frenanti, passo netto medio e quota di passi
netti nulli.

    py ai_posizione/diagnosi_balbettio.py                  pop337, il livello con l'effetto peggiore
    py ai_posizione/diagnosi_balbettio.py --livello=pop211
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import math
import statistics

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

CM_PER_PIXEL = 100.0 / sim.DIM_NODO   # 1 cella = 1 m, quindi un pixel vale 1/DIM_NODO metri
SEME = 894407369                      # il primo seme della campagna E12


def _misura(livello, popolazione, con_oracolo):
    """Una corsa strumentata. Ritorna le statistiche di ingresso e di uscita del controllo."""
    ripristina_oracolo = op.installa(t, sim) if con_oracolo else (lambda: None)

    # la spia sulla previsione va installata DOPO l'oracolo, cosi' avvolge quella attiva
    previsione_attiva = sim.previsione_per_evitamento
    offset_originale = sim.offset_evitamento_predittivo
    orizzonte_pieno = float(sim.PREVISIONE_EVITAMENTO_FRAME)

    corrente, precedente = {}, {}
    salti = []                       # spostamento del punto previsto fra fotogrammi, in pixel
    manovre = []                     # i vettori (dx, dy) scelti, uno per fotogramma
    angoli, inversioni, moduli = [], 0, []
    # la manovra si SOVRAPPONE al passo lungo il percorso: la sua componente lungo la direzione
    # di marcia dice se aiuta l'avanzamento o lo consuma. Negativa = frena
    componenti_avanti, frenanti = [], 0
    posizioni, passi_netti = [], []  # posizione a ogni chiamata: il passo netto e' la differenza

    def _previsione_spiata(traccia, frame_futuri):
        px, py = previsione_attiva(traccia, frame_futuri)
        # solo l'orizzonte pieno: e' quello su cui le due previsioni divergono di piu', e
        # l'istante 0,0 e' esatto per costruzione in entrambe
        if abs(frame_futuri - orizzonte_pieno) < 1e-9:
            corrente[traccia.get("pid", id(traccia))] = (px, py)
        return px, py

    def _offset_spiato(x, y, percorso, tracciamento_lidar, griglia, passi_reali=1):
        nonlocal inversioni, frenanti
        corrente.clear()
        dx, dy = offset_originale(x, y, percorso, tracciamento_lidar, griglia, passi_reali)

        # INGRESSO: di quanto si e' mosso il punto previsto di ciascuna persona vista in
        # entrambi i fotogrammi. Le persone entrate o uscite dal tracciamento non contano
        for pid, (px, py) in corrente.items():
            vecchio = precedente.get(pid)
            if vecchio is not None:
                salti.append(math.hypot(px - vecchio[0], py - vecchio[1]))
        precedente.clear()
        precedente.update(corrente)

        # PASSO NETTO: fra due chiamate consecutive il robot ha fatto un passo avanti e una
        # traslazione. Se il totale e' nullo, _metriche conta una fermata pur senza arresto
        if posizioni:
            passi_netti.append(math.hypot(x - posizioni[-1][0], y - posizioni[-1][1]))
        posizioni.append((x, y))

        # USCITA: angolo fra manovre consecutive. I fotogrammi senza manovra non hanno
        # direzione e vengono saltati, altrimenti l'angolo sarebbe indefinito
        modulo = math.hypot(dx, dy)
        if modulo > 1e-9:
            moduli.append(modulo)
            if manovre:
                ax, ay = manovre[-1]
                coseno = (ax * dx + ay * dy) / (math.hypot(ax, ay) * modulo)
                angolo = math.degrees(math.acos(max(-1.0, min(1.0, coseno))))
                angoli.append(angolo)
                if angolo > 90.0:
                    inversioni += 1
            manovre.append((dx, dy))
            # componente della manovra lungo la direzione di marcia verso il nodo successivo
            if percorso:
                nodo = percorso[0]
                avanti = math.hypot(nodo.cx - x, nodo.cy - y)
                if avanti > 1e-9:
                    componente = (dx * (nodo.cx - x) + dy * (nodo.cy - y)) / avanti
                    componenti_avanti.append(componente)
                    if componente < 0:
                        frenanti += 1
        return dx, dy

    sim.previsione_per_evitamento = _previsione_spiata
    sim.offset_evitamento_predittivo = _offset_spiato
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa("povo", "solo_posizione", popolazione, SEME)
    finally:
        sim.previsione_per_evitamento = previsione_attiva
        sim.offset_evitamento_predittivo = offset_originale
        ripristina_oracolo()

    return {
        "arrivato": arrivato,
        "tempo_s": t._secondi(frame),
        "fermo_s": t._secondi(frame_fermo),
        "salto_medio_px": statistics.mean(salti) if salti else 0.0,
        "salto_mediano_px": statistics.median(salti) if salti else 0.0,
        "salti_oltre_1m": (100.0 * sum(1 for s in salti if s > sim.DIM_NODO) / len(salti)
                           if salti else 0.0),
        "campioni_salto": len(salti),
        "angolo_medio": statistics.mean(angoli) if angoli else 0.0,
        "angolo_mediano": statistics.median(angoli) if angoli else 0.0,
        "quota_inversioni": 100.0 * inversioni / len(angoli) if angoli else 0.0,
        "modulo_medio_px": statistics.mean(moduli) if moduli else 0.0,
        "fotogrammi_con_manovra": len(manovre),
        "componente_avanti_px": statistics.mean(componenti_avanti) if componenti_avanti else 0.0,
        "quota_frenanti": 100.0 * frenanti / len(componenti_avanti) if componenti_avanti else 0.0,
        "passo_netto_px": statistics.mean(passi_netti) if passi_netti else 0.0,
        "quota_passi_nulli": (100.0 * sum(1 for p in passi_netti if p < 1e-6) / len(passi_netti)
                              if passi_netti else 0.0),
    }


def principale():
    livello = "pop337"
    for a in sys.argv[1:]:
        if a.startswith("--livello="):
            livello = a.split("=", 1)[1]
    etichette = [f"pop{round(sum(cc.POPOLAZIONE_BASE.values()) * f)}" for f in cc.FATTORI]
    popolazione = cc._popolazione_scalata(cc.FATTORI[etichette.index(livello)])

    print(f"scena {livello}, seme {SEME}, condizione 'solo_posizione'")
    print(f"orizzonte del controllo: {sim.PREVISIONE_EVITAMENTO_FRAME} passi di simulazione "
          f"= {sim.PREVISIONE_EVITAMENTO_FRAME / 60 * t.ACCELERAZIONE:.1f} s reali "
          f"(accelerazione {t.ACCELERAZIONE}x)\n")

    esiti = {}
    for nome, con_oracolo in (("kalman", False), ("oracolo", True)):
        print(f"  corsa {nome}...", flush=True)
        esiti[nome] = _misura(livello, popolazione, con_oracolo)

    k, o = esiti["kalman"], esiti["oracolo"]
    righe = [
        ("ESITO DELLA CORSA", None, None, None),
        ("tempo di percorrenza", "tempo_s", "s", 1),
        ("tempo trascorso fermo", "fermo_s", "s", 1),
        ("INGRESSO: instabilita' della previsione", None, None, None),
        ("salto medio fra fotogrammi", "salto_medio_px", "px", 2),
        ("salto mediano fra fotogrammi", "salto_mediano_px", "px", 2),
        ("quota di salti oltre 1 m", "salti_oltre_1m", "%", 1),
        ("USCITA: instabilita' della manovra", None, None, None),
        ("angolo medio fra manovre", "angolo_medio", "gradi", 1),
        ("angolo mediano fra manovre", "angolo_mediano", "gradi", 1),
        ("QUOTA DI INVERSIONI (oltre 90 gradi)", "quota_inversioni", "%", 1),
        ("modulo medio della manovra", "modulo_medio_px", "px", 2),
        ("fotogrammi con manovra", "fotogrammi_con_manovra", "", 0),
        ("COSTO: la manovra contro l'avanzamento", None, None, None),
        ("componente lungo la marcia", "componente_avanti_px", "px", 3),
        ("quota di manovre frenanti", "quota_frenanti", "%", 1),
        ("passo netto medio", "passo_netto_px", "px", 3),
        ("quota di passi netti nulli", "quota_passi_nulli", "%", 1),
    ]
    print(f"\n  {'grandezza':<38} {'kalman':>10} {'oracolo':>10} {'variazione':>12}")
    for etichetta, chiave, unita, cifre in righe:
        if chiave is None:
            print(f"\n  {etichetta}")
            continue
        a, b = k[chiave], o[chiave]
        variazione = f"{100 * (b - a) / a:+.0f}%" if a else "-"
        print(f"    {etichetta:<36} {a:>10.{cifre}f} {b:>10.{cifre}f} {variazione:>12}"
              f"   {unita}")

    print(f"\n  campioni: {k['campioni_salto']} previsioni appaiate (kalman), "
          f"{o['campioni_salto']} (oracolo)")
    print("\n  LETTURA: l'ipotesi del balbettio - previsione instabile, manovra oscillante -")
    print("  e' stata SMENTITA dalla prima esecuzione: con l'oracolo il punto previsto salta")
    print("  MENO (-68%) e le inversioni di manovra calano (-71%). La proiezione in retta del")
    print("  Kalman amplifica per 60 passi il rumore sulla velocita' stimata, quindi e' LEI la")
    print("  previsione instabile. Questa seconda esecuzione misura l'ipotesi opposta: con un")
    print("  campo di rischio coerente il robot manovra DI PIU' (+22% di fotogrammi), e ogni")
    print("  traslazione si sottrae all'avanzamento perche' si sovrappone al passo lungo il")
    print("  percorso. Se la componente lungo la marcia e' negativa e il passo netto cala, il")
    print("  costo e' quello, non l'oscillazione.")


if __name__ == "__main__":
    principale()

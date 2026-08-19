"""Da dove arrivano le fermate, e quanto spazio laterale c'e' per schivarle.

La campagna performance_velocita_classico.py ha stabilito che il controllo di velocita'
non riduce le fermate a nessuna densita' (rapporto 0,29 su 50 coppie). Questo banco
risponde alle due domande successive: PERCHE', e COSA POTREBBE FUNZIONARE AL SUO POSTO.

L'IPOTESI SULLA CAUSA. Nel simulatore convivono due meccanismi che guardano cose diverse:

  - il CONTROLLO DI VELOCITA' (main.py, fattore_velocita_da_conflitto) prevede i conflitti
    entro 5 m LUNGO IL PERCORSO pianificato, con soglia 62 cm (raggio robot 19 cm +
    raggio persona 23 cm + 20 cm di franco). Il suo minimo e' 0,20: non ferma mai il robot;
  - l'ARRESTO DI SICUREZZA (testing.py, stesso codice di main.py) scatta SUBITO, a 33,5 cm,
    IN TUTTE LE DIREZIONI, su qualunque corpo mobile.

I raggi NON sono le costanti di main.py: t._geometria_salvata() legge config_pannello.json e
li sovrascrive. Le costanti darebbero 70 e 40 cm, i valori realmente in uso sono quelli sopra.
E' la ragione per cui ogni misura salva accanto a se' la propria configurazione: vedi
ibridazione_velocita/configurazione.py.

Se gli arresti sono innescati in prevalenza da persone che arrivano di lato o da dietro,
il controllo non poteva vederle: guarda solo avanti, lungo il percorso. In quel caso
nessuna taratura delle soglie aiuta, e serve una modifica strutturale.

LA SECONDA DOMANDA. L'alternativa in esame e' un controllo di POSIZIONE: lasciare che il
robot trasli lateralmente rispetto al percorso di Theta*, per aggirare le persone invece
di limitarsi a rallentare. Una traslazione richiede pero' spazio libero, e Povo ha porte
larghe una cella (1 m) e corridoi di tre. Se gli arresti avvengono nei punti stretti, la
traslazione non ha dove andare e il rimedio fallisce dove serve. Per questo a ogni arresto
si misura anche il varco libero perpendicolare alla marcia, sondando la griglia fino a 2 m
per lato e fermandosi al primo muro.

COME SI MISURA SENZA TOCCARE IL SIMULATORE. Nel ciclo di esegui_corsa, _rileva_e_traccia
viene chiamata a ogni fotogramma, mentre limita_accelerazione viene saltata quando
l'arresto scatta (e' l'unico `continue` del ciclo). Un fotogramma in cui la prima e' stata
chiamata e la seconda no e' quindi, per costruzione, un fotogramma di arresto.
Intercettando le due funzioni si ricostruisce ogni arresto senza modificare testing.py.

NON si intercetta passo_movimento, che sarebbe la scelta ovvia: _muovi_e_gestisci_stato la
chiama per OGNI PERSONA a ogni fotogramma, quindi il marcatore verrebbe azzerato dalla
folla e il banco riporterebbe zero arresti ovunque. limita_accelerazione e' invece chiamata
solo per il robot, una volta per fotogramma non arrestato, e in entrambe le condizioni.

L'angolo e' misurato fra la direzione di marcia del robot e la direzione in cui si trova
la persona che ha innescato l'arresto (la piu' vicina entro il raggio di sicurezza):
frontale entro 45 gradi, laterale fra 45 e 135, posteriore oltre 135.

Uso:
    py ibridazione_velocita/diagnosi_fermate.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import math
import zlib
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim

MAPPA = "povo"
CONDIZIONI = ("base", "solo_velocita")
ETICHETTA_SEMI = "ibridazione_velocita"   # stessi semi della campagna: le corse sono le stesse
NUMERO_SEMI = 3
FATTORI = [0.60, 1.05, 1.50]              # bassa, media, alta densita' della stessa serie

# corridori dal 48% al 13% dei corpi mobili, totale invariato: vedi performance_velocita_classico.py
POPOLAZIONE_BASE = {"numero_persone": 120, "numero_corridori": 18,
                    "numero_persone_ferme": 116, "numero_gruppi": 27}

PIXEL_PER_METRO = 100.0 / sim.CM_PER_PIXEL
SONDA_LATERALE_PX = 60      # 2 m per lato: oltre non serve, il corridoio piu' largo e' 3 celle
PASSO_SONDA_PX = 3


def _popolazione_scalata(fattore):
    scalata = {k: max(0, round(v * fattore)) for k, v in POPOLAZIONE_BASE.items()}
    scalata.update(t._geometria_salvata())
    return scalata


def _varco_libero(griglia, x, y, dx, dy):
    """Quanti px si puo' avanzare in direzione (dx, dy) prima di incontrare un muro."""
    percorsi = 0.0
    while percorsi < SONDA_LATERALE_PX:
        percorsi += PASSO_SONDA_PX
        r = int((y + dy * percorsi) // sim.DIM_NODO)
        c = int((x + dx * percorsi) // sim.DIM_NODO)
        if not (0 <= r < sim.Y_TOT and 0 <= c < sim.X_TOT) or griglia[r][c].tipo == "muro":
            return percorsi - PASSO_SONDA_PX
    return SONDA_LATERALE_PX


def _una_corsa(argomenti):
    """Intercetta le due funzioni e ricostruisce ogni arresto. Le patch vanno installate qui:
    con lo spawn di Windows il worker re-importa i moduli e non vedrebbe quelle del padre."""
    condizione, seed, popolazione, etichetta = argomenti
    raggio_sicurezza = popolazione.get("raggio_sicurezza", sim.RAGGIO_SICUREZZA_DEFAULT)

    stato = {"scatto": None, "mosso": False, "direzione": None,
             "fermo_prima": False, "ultima_pos": None}
    inneschi = []          # (angolo_gradi, distanza_px, varco_px) per ogni EVENTO di arresto

    traccia_originale = t._rileva_e_traccia
    limite_originale = sim.limita_accelerazione

    def _classifica(dati, continuazione):
        """Chiamata quando si scopre che il fotogramma precedente si era arrestato."""
        robot_x, robot_y, mobili, griglia = dati
        vicino, distanza = None, None
        for px, py in mobili:
            d = math.hypot(px - robot_x, py - robot_y)
            if d < raggio_sicurezza and (distanza is None or d < distanza):
                vicino, distanza = (px, py), d
        if vicino is None or continuazione:
            return  # nessun innesco, oppure e' lo stesso arresto che prosegue: un evento si conta una volta
        direzione = stato["direzione"]
        if direzione is None:
            return  # nessuna marcia ancora osservata: l'angolo non e' definito
        dx, dy = vicino[0] - robot_x, vicino[1] - robot_y
        norma = math.hypot(dx, dy)
        if norma == 0:
            return
        coseno = (dx * direzione[0] + dy * direzione[1]) / norma
        angolo = math.degrees(math.acos(max(-1.0, min(1.0, coseno))))
        # perpendicolari alla marcia, una per lato
        sinistra = _varco_libero(griglia, robot_x, robot_y, -direzione[1], direzione[0])
        destra = _varco_libero(griglia, robot_x, robot_y, direzione[1], -direzione[0])
        inneschi.append((angolo, distanza, sinistra + destra))

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
        # direzione di marcia dallo spostamento effettivo fra due fotogrammi: nei fotogrammi di
        # arresto il robot non si muove, quindi resta valida l'ultima direzione osservata
        if stato["ultima_pos"] is not None:
            dx, dy = robot_x - stato["ultima_pos"][0], robot_y - stato["ultima_pos"][1]
            norma = math.hypot(dx, dy)
            if norma > 1e-9:
                stato["direzione"] = (dx / norma, dy / norma)
        stato["ultima_pos"] = (robot_x, robot_y)

        # il fotogramma precedente era un arresto se limita_accelerazione non e' stata chiamata:
        # l'arresto di sicurezza fa `continue` prima di arrivarci (unico continue del ciclo)
        era_arresto = stato["scatto"] is not None and not stato["mosso"]
        if era_arresto:
            _classifica(stato["scatto"], stato["fermo_prima"])
        stato["fermo_prima"] = era_arresto
        stato["mosso"] = False

        # solo i corpi mobili, lo stesso insieme su cui lavora l'arresto: le persone ferme non
        # hanno la chiave "stato" (crea_persona_ferma restituisce solo x, y, r, c)
        mobili = [(p["x"], p["y"]) for p in tutte_le_persone if "stato" in p]
        stato["scatto"] = (robot_x, robot_y, mobili, griglia)
        return traccia_originale(tutte_le_persone, robot_x, robot_y, griglia,
                                 tracciamento_lidar, storico_posizioni)

    def _con_limite(*args, **kwargs):
        # segnale che il fotogramma NON si e' arrestato. Va intercettata questa e non
        # passo_movimento, che _muovi_e_gestisci_stato chiama per ogni persona a ogni fotogramma
        stato["mosso"] = True
        return limite_originale(*args, **kwargs)

    t._rileva_e_traccia = _con_misure
    sim.limita_accelerazione = _con_limite
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, condizione, popolazione, seed)
    finally:
        t._rileva_e_traccia = traccia_originale
        sim.limita_accelerazione = limite_originale

    frontali = sum(1 for a, _, _ in inneschi if a <= 45)
    laterali = sum(1 for a, _, _ in inneschi if 45 < a <= 135)
    posteriori = sum(1 for a, _, _ in inneschi if a > 135)
    varchi = [v for _, _, v in inneschi]
    stretti = sum(1 for v in varchi if v < sim.DIM_NODO)   # meno di 1 m totale: porta o strettoia
    return (etichetta, condizione, arrivato, len(inneschi), frontali, laterali, posteriori,
            statistics.mean(varchi) if varchi else 0.0, stretti,
            t._secondi(frame), t._secondi(frame_fermo))


def main():
    livelli = [(f"pop{round(sum(POPOLAZIONE_BASE.values()) * f)}", _popolazione_scalata(f))
               for f in FATTORI]
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(NUMERO_SEMI)]
    compiti = [(c, s, pop, et) for et, pop in livelli for s in semi for c in CONDIZIONI]

    print("Diagnosi degli arresti: chi li innesca, da che direzione, e con quanto spazio a lato")
    print(f"  raggio di sicurezza {sim.RAGGIO_SICUREZZA_DEFAULT} px "
          f"({sim.RAGGIO_SICUREZZA_DEFAULT * sim.CM_PER_PIXEL:.0f} cm), "
          f"orizzonte del controllo {sim.DISTANZA_LOOKAHEAD_VELOCITA_PX / PIXEL_PER_METRO:.0f} m")
    print(f"  {len(livelli)} livelli x {NUMERO_SEMI} semi x {len(CONDIZIONI)} condizioni "
          f"= {len(compiti)} corse\n")

    raccolta = {}
    n_worker = min(8, os.cpu_count() or 4)
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for i, r in enumerate(pool.imap_unordered(_una_corsa, compiti, chunksize=1), 1):
            raccolta.setdefault((r[0], r[1]), []).append(r)
            print(f"  [{i}/{len(compiti)}] {r[0]}/{r[1]}: {r[3]} arresti "
                  f"(front {r[4]}, lat {r[5]}, post {r[6]}), varco medio "
                  f"{r[7] * sim.CM_PER_PIXEL:.0f} cm")

    print("\n=== Direzione da cui arriva la persona che innesca l'arresto ===")
    print("(frontale = entro 45 gradi dalla marcia, cioe' dove il controllo di velocita' guarda)\n")
    print(f"{'livello':>9}{'condizione':>15}{'arresti':>9}{'frontali':>12}{'laterali':>12}"
          f"{'posteriori':>12}{'varco medio':>13}{'in stretto':>12}")
    print("-" * 96)
    for et, _ in livelli:
        for cond in CONDIZIONI:
            v = raccolta.get((et, cond), [])
            if not v:
                continue
            tot = sum(r[3] for r in v)
            if not tot:
                continue
            fr, la, po = sum(r[4] for r in v), sum(r[5] for r in v), sum(r[6] for r in v)
            varco = statistics.mean([r[7] for r in v if r[7] > 0]) if any(r[7] > 0 for r in v) else 0.0
            stretti = sum(r[8] for r in v)
            def q(n):
                return f"{n} ({n / tot * 100:.0f}%)"
            print(f"{et:>9}{cond:>15}{tot:>9}{q(fr):>12}{q(la):>12}{q(po):>12}"
                  f"{varco * sim.CM_PER_PIXEL:>10.0f} cm{q(stretti):>12}")

    tot = sum(r[3] for v in raccolta.values() for r in v)
    fr = sum(r[4] for v in raccolta.values() for r in v)
    stretti = sum(r[8] for v in raccolta.values() for r in v)
    if tot:
        quota_frontale = fr / tot * 100
        quota_stretta = stretti / tot * 100
        print(f"\nComplessivo: {tot} arresti, {fr} frontali ({quota_frontale:.0f}%), "
              f"{stretti} in spazi stretti ({quota_stretta:.0f}%).")
        # le conclusioni dipendono dai numeri: stamparle sempre le renderebbe decorazione
        if quota_frontale < 35:
            print("Frontali in netta minoranza -> il controllo di velocita' non poteva anticiparli:")
            print("guarda solo avanti lungo il percorso, e la correzione necessaria e' strutturale.")
        elif quota_frontale > 65:
            print("Frontali in netta maggioranza -> il controllo di velocita' li stava guardando e")
            print("non li ha evitati: il problema e' nella reazione, non in dove punta lo sguardo.")
        else:
            print("Frontali e non frontali si equivalgono -> due meta' con cause diverse. Su una il")
            print("controllo di velocita' non poteva vedere nulla; sull'altra guardava e non e'")
            print("bastato, perche' rallentare non crea distanza. Entrambe restano scoperte.")
        if quota_stretta > 25:
            print("Molti arresti in spazi stretti -> una traslazione laterale non ha dove andare")
            print("proprio dove servirebbe, e il controllo di posizione va limitato alle zone aperte.")
        else:
            print(f"Solo il {quota_stretta:.0f}% degli arresti avviene in spazi stretti: nella grande")
            print("maggioranza dei casi lo spazio per una traslazione laterale c'e'.")


if __name__ == "__main__":
    main()

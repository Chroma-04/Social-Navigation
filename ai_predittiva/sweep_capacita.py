"""La rete e' limitata dalla propria capacita' o dall'informazione che riceve? Si misura.

PERCHE'. Il modello impiegato e' una GRU a 96 unita' allenata per 150 epoche, e la tesi dichiara
come limite piu' stringente non la sua taglia ma il suo ingresso: la sola storia individuale
della persona, senza alcun descrittore di contesto. Aumentare capacita' e durata
dell'addestramento e' l'intervento piu' economico, ma agisce sul vincolo sbagliato se il modello
e' gia' al proprio pavimento informativo. Questo banco distingue i due casi PRIMA di spendere.

  SOTTO-ADDESTRAMENTO   perdita di verifica ancora in discesa a fine corsa, e vicina a quella di
                        addestramento: capacita' o epoche in piu' guadagnano ancora.
  PAVIMENTO INFORMATIVO perdita di verifica piatta da molte epoche mentre quella di addestramento
                        continua a scendere: la rete sta memorizzando rumore. Capacita' in piu'
                        peggiora. Cio' che manca e' informazione, non parametri.

L'INSIEME DI VERIFICA DELLA TESI NON VIENE TOCCATO. Il capitolo 5 afferma che architettura,
epoche e passo di apprendimento sono fissati a priori e che quei dati sono impiegati una sola
volta al termine dell'addestramento. Scegliere un'architettura confrontando i risultati su
quell'insieme renderebbe falsa quell'affermazione e ottimistico il 12,1% riportato. Gli scenari
sono percio' divisi in TRE parti: il test della tesi resta intatto e non viene nemmeno caricato
in memoria, gli scenari rimanenti sono ulteriormente divisi in addestramento e SELEZIONE, ed e'
solo su quest'ultima che i modelli vengono confrontati. Il vincitore andra' poi misurato una
sola volta sul test intatto e sull'insieme di Povo.

    py ai_predittiva/sweep_capacita.py --rapido    due configurazioni, 40 epoche
    py ai_predittiva/sweep_capacita.py             sei configurazioni, 150 epoche
"""
import os
import sys
import time
import json

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import torch
import torch.nn as nn

import allena_previsione as ap

DATASET = os.path.join(ap.CARTELLA_DATASET, "dataset_previsione_specializzato_5x.npz")
USCITA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweep_capacita.json")
FRAZIONE_SELEZIONE = 0.25   # degli scenari di addestramento, non del totale
SEME_SELEZIONE = 7          # diverso da quello del test della tesi, che usa 0

# (unita' nascoste, strati). Il primo e' la configurazione della tesi e fa da riferimento.
CONFIGURAZIONI = [(96, 1), (16, 1), (32, 1), (48, 1), (192, 1), (96, 2)]
CONFIGURAZIONI_RAPIDE = [(96, 1), (384, 1)]


class CorrezioneKalmanProfonda(nn.Module):
    """Come CorrezioneKalman, ma con numero di strati parametrico. A un solo strato e' identica,
    cosi' la configurazione della tesi resta riproducibile dentro questo banco."""

    def __init__(self, dimensione_input=4, dimensione_nascosta=96, strati=1):
        super().__init__()
        self.gru = nn.GRU(dimensione_input, dimensione_nascosta, num_layers=strati,
                          batch_first=True)
        self.testa = nn.Linear(dimensione_nascosta, 2)

    def forward(self, storico):
        _, h = self.gru(storico)
        return self.testa(h[-1])


def _tre_insiemi(scenario):
    """Test della tesi intatto; il resto diviso in addestramento e selezione.

    Il test riproduce esattamente la divisione di allena_previsione, seme compreso: e' lo stesso
    insieme su cui il capitolo 5 riporta il 12,1%, e qui serve solo a essere ESCLUSO.
    """
    maschera_train_tesi, maschera_test_tesi = ap.divide_train_test(scenario, ap.FRAZIONE_TEST)

    scenari_disponibili = sorted({s for s, dentro in zip(scenario.tolist(), maschera_train_tesi)
                                  if dentro})
    rng = np.random.RandomState(SEME_SELEZIONE)
    rng.shuffle(scenari_disponibili)
    n_selezione = max(1, int(len(scenari_disponibili) * FRAZIONE_SELEZIONE))
    scenari_selezione = set(scenari_disponibili[:n_selezione])

    maschera_selezione = np.array([s in scenari_selezione for s in scenario])
    maschera_addestramento = maschera_train_tesi & ~maschera_selezione
    return maschera_addestramento, maschera_selezione, maschera_test_tesi


def _errore_medio(previsioni, verita):
    return float(np.linalg.norm(previsioni - verita, axis=1).mean())


def allena(config, dati, numero_epoche, device, seme=0):
    """Una configurazione. Ritorna la curva della perdita di verifica, epoca per epoca."""
    nascoste, strati = config
    x_add, y_add, x_sel, kalman_sel, verita_sel, errore_kalman = dati

    torch.manual_seed(seme)   # stessa inizializzazione per tutte: le differenze sono dell'architettura
    modello = CorrezioneKalmanProfonda(dimensione_nascosta=nascoste, strati=strati).to(device)
    ottimizzatore = torch.optim.Adam(modello.parameters(), lr=ap.TASSO_APPRENDIMENTO)
    perdita_mse = nn.MSELoss()
    parametri = sum(p.numel() for p in modello.parameters())

    n = x_add.shape[0]
    curva_addestramento, curva_selezione, curva_guadagno = [], [], []
    avvio = time.time()
    for epoca in range(numero_epoche):
        modello.train()
        permutazione = torch.randperm(n, device=device)
        accumulata = 0.0
        for i in range(0, n, ap.DIMENSIONE_BATCH):
            indici = permutazione[i:i + ap.DIMENSIONE_BATCH]
            ottimizzatore.zero_grad()
            perdita = perdita_mse(modello(x_add[indici]), y_add[indici])
            perdita.backward()
            ottimizzatore.step()
            accumulata += perdita.item() * len(indici)

        modello.eval()
        with torch.no_grad():
            correzione = modello(x_sel).cpu().numpy()
        errore = _errore_medio(kalman_sel + correzione, verita_sel)
        curva_addestramento.append(accumulata / n)
        curva_selezione.append(errore)
        curva_guadagno.append(100.0 * (1.0 - errore / errore_kalman))

    epoca_migliore = int(np.argmin(curva_selezione))
    return {
        "unita": nascoste, "strati": strati, "parametri": parametri,
        "secondi": time.time() - avvio,
        "guadagno_finale": curva_guadagno[-1],
        "guadagno_migliore": curva_guadagno[epoca_migliore],
        "epoca_migliore": epoca_migliore + 1,
        "perdita_addestramento_finale": curva_addestramento[-1],
        "curva_selezione": curva_selezione,
        "curva_guadagno": curva_guadagno,
    }


def stabilita(dati, device, configurazioni=((96, 1), (96, 2)), semi=(0, 1, 2), epoche=30):
    """Le differenze fra configurazioni vicine sono reali o rumore di inizializzazione?

    Lo sweep confronta architetture a inizializzazione fissa, il che rende il confronto pulito ma
    non dice quanto ciascun numero ballerebbe cambiando seme. Se lo scarto fra due configurazioni
    e' minore della dispersione fra semi della stessa, sceglierne una sull'altra e' scegliere sul
    rumore - lo stesso errore che in questo lavoro ha prodotto quattro repliche fallite.
    """
    print("")
    print(f"  stabilita' rispetto al seme di inizializzazione, {len(semi)} semi, {epoche} epoche")
    for config in configurazioni:
        picchi = []
        for seme in semi:
            esito = allena(config, dati, epoche, device, seme=seme)
            picchi.append(esito["guadagno_migliore"])
        media = sum(picchi) / len(picchi)
        dispersione = max(picchi) - min(picchi)
        valori = "  ".join(f"{p:+.2f}%" for p in picchi)
        print(f"    GRU {config[0]:>3} x {config[1]}: {valori}   media {media:+.2f}%   "
              f"escursione {dispersione:.2f} punti", flush=True)


def principale():
    rapido = "--rapido" in sys.argv
    configurazioni = CONFIGURAZIONI_RAPIDE if rapido else CONFIGURAZIONI
    # 60 e non 150: la corsa rapida mostra il minimo della perdita di verifica attorno alla
    # ventesima epoca, quindi le epoche successive documentano solo il sovradattamento
    numero_epoche = 40 if rapido else 60
    for a in sys.argv[1:]:
        if a.startswith("--epoche="):
            numero_epoche = int(a.split("=", 1)[1])

    storico, pred_kalman, verita, scenario = ap.carica_dataset(DATASET)
    add, sel, test = _tre_insiemi(scenario)
    print(f"{len(verita)} campioni, {len(set(scenario.tolist()))} scenari")
    print(f"  addestramento {add.sum():>6}   selezione {sel.sum():>6}   "
          f"test della tesi {test.sum():>6} (INTATTO, non usato qui)")

    residuo = verita - pred_kalman
    ingresso = ap.prepara_input(storico)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    errore_kalman = _errore_medio(pred_kalman[sel], verita[sel])
    print(f"  device {device}   errore del Kalman sulla selezione: {errore_kalman:.2f} px\n")

    dati = (
        torch.tensor(ingresso[add], dtype=torch.float32, device=device),
        torch.tensor(residuo[add], dtype=torch.float32, device=device),
        torch.tensor(ingresso[sel], dtype=torch.float32, device=device),
        pred_kalman[sel], verita[sel], errore_kalman,
    )

    if "--stabilita" in sys.argv:
        # --configurazioni=96x2,96x3 per sondare architetture fuori dalla lista predefinita
        scelte = ((96, 1), (96, 2))
        for a in sys.argv[1:]:
            if a.startswith("--configurazioni="):
                scelte = tuple(tuple(int(v) for v in c.split("x"))
                               for c in a.split("=", 1)[1].split(","))
        stabilita(dati, device, configurazioni=scelte)
        return

    esiti = []
    for config in configurazioni:
        print(f"  GRU {config[0]:>3} unita', {config[1]} strat{'o' if config[1] == 1 else 'i'}, "
              f"{numero_epoche} epoche...", flush=True)
        esito = allena(config, dati, numero_epoche, device)
        esiti.append(esito)
        print(f"    {esito['parametri']:>7} parametri, {esito['secondi']:.0f}s   "
              f"guadagno finale {esito['guadagno_finale']:+.2f}%   "
              f"migliore {esito['guadagno_migliore']:+.2f}% all'epoca {esito['epoca_migliore']}",
              flush=True)

    print("\n" + "=" * 88)
    print("SELEZIONE (mai il test della tesi). Guadagno = riduzione dell'errore medio sul Kalman")
    print("=" * 88)
    print(f"  {'configurazione':<22}{'parametri':>10}{'finale':>10}{'migliore':>10}"
          f"{'epoca':>8}{'sovradatt.':>12}")
    for e in esiti:
        # se il meglio arriva molto prima della fine, le epoche successive peggiorano: e' la firma
        # del sovradattamento, e dice che il problema non e' la mancanza di capacita'
        sovra = "si'" if e["epoca_migliore"] < 0.7 * numero_epoche else "no"
        print(f"  GRU {e['unita']:>3} x {e['strati']}{'':<14}{e['parametri']:>10}"
              f"{e['guadagno_finale']:>9.2f}%{e['guadagno_migliore']:>9.2f}%"
              f"{e['epoca_migliore']:>8}{sovra:>12}")

    riferimento = esiti[0]
    migliore = max(esiti, key=lambda e: e["guadagno_migliore"])
    delta = migliore["guadagno_migliore"] - riferimento["guadagno_migliore"]
    print(f"\n  riferimento della tesi (96 x 1): {riferimento['guadagno_migliore']:+.2f}%")
    print(f"  migliore del banco ({migliore['unita']} x {migliore['strati']}): "
          f"{migliore['guadagno_migliore']:+.2f}%   differenza {delta:+.2f} punti")
    if delta < 0.5:
        print("\n  LETTURA: la capacita' non e' il vincolo. Cio' che manca alla rete e'")
        print("  informazione, non parametri: il passo successivo sono i descrittori di contesto,")
        print("  non una rete piu' grande.")
    else:
        print("\n  LETTURA: la capacita' guadagna ancora. Il vincitore va ora misurato UNA SOLA")
        print("  VOLTA sul test della tesi e sull'insieme di Povo, e i due numeri riportati in")
        print("  tesi vanno aggiornati insieme alla dichiarazione sulla selezione dell'architettura.")

    with open(USCITA, "w", encoding="utf-8") as fh:
        json.dump(esiti, fh, indent=1)
    print(f"\n  curve complete in {USCITA}")


if __name__ == "__main__":
    principale()

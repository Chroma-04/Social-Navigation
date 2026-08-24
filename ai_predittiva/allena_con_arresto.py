"""Addestramento con validazione per epoca, arresto anticipato e salvataggio dell'epoca migliore.

PERCHE' ESISTE. Il protocollo impiegato in tesi (allena_previsione.py) allena per un numero fisso
di epoche, valuta una sola volta alla fine e salva i pesi dell'ULTIMA epoca. La perdita di
addestramento scende sempre, quindi il ciclo non ha modo di accorgersi di quando la rete smette
di imparare struttura e comincia a memorizzare il rumore del lidar. La diagnosi condotta con
sweep_capacita.py mostra che accade presto: il minimo dell'errore di verifica cade attorno alla
ventesima epoca, mentre l'addestramento prosegue fino alla centocinquantesima.

Questo banco corregge i tre difetti insieme:
  - valuta ad ogni epoca sull'insieme di SELEZIONE, non solo alla fine;
  - conserva i pesi dell'epoca migliore invece di quelli dell'ultima;
  - si ferma quando la verifica non migliora piu' da un numero dato di epoche.

IL MODELLO DELLA TESI NON VIENE SOVRASCRITTO. Le campagne dei capitoli 6 e 7 sono state eseguite
con correzione_kalman_specializzato.pt: rimpiazzarlo renderebbe irriproducibili quei risultati.
Il modello prodotto qui e' un file distinto, e i due possono convivere.

L'INSIEME DI VERIFICA DELLA TESI RESTA INTATTO fino a esplicita richiesta. La divisione in tre
parti e' quella di sweep_capacita: il test della tesi non entra ne' nell'addestramento ne' nella
scelta dell'epoca. Misurarvi sopra il modello e' un atto deliberato e irripetibile, e richiede
--misura-finale: da quel momento quell'insieme ha partecipato a una decisione, e ogni ulteriore
confronto su di esso sarebbe ottimistico.

    py ai_predittiva/allena_con_arresto.py                    96 unita', arresto con pazienza 15
    py ai_predittiva/allena_con_arresto.py --unita=48
    py ai_predittiva/allena_con_arresto.py --misura-finale    misura UNA VOLTA su test e su Povo
"""
import os
import sys
import copy
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import torch
import torch.nn as nn

import allena_previsione as ap
from sweep_capacita import CorrezioneKalmanProfonda, _tre_insiemi, _errore_medio, DATASET

DATASET_POVO = os.path.join(ap.CARTELLA_DATASET, "dataset_previsione_povo_5x.npz")
FILE_USCITA = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt")
PAZIENZA = 15          # epoche senza miglioramento prima di fermarsi
EPOCHE_MASSIME = 150   # lo stesso tetto del protocollo originale, cosi' il confronto e' pulito


def _statistiche(previsioni, verita):
    errori = np.linalg.norm(previsioni - verita, axis=1)
    return errori.mean(), np.median(errori), np.percentile(errori, 90)


BLOCCO_INFERENZA = 8192   # l'insieme di Povo ha 160 mila campioni: in un colpo solo satura la GPU


def _valuta(modello, ingresso, pred_kalman, verita, device):
    """Errore medio, mediano e novantesimo percentile con e senza correzione.

    L'inferenza e' spezzata a blocchi perche' la GRU materializza gli stati nascosti di tutti i
    passi per l'intero lotto: su centosessantamila campioni sono decine di gigabyte, e la scheda
    ne ha otto. Il risultato e' identico a quello del lotto unico, la rete non ha stato fra
    campioni diversi.
    """
    modello.eval()
    pezzi = []
    with torch.no_grad():
        for i in range(0, len(ingresso), BLOCCO_INFERENZA):
            x = torch.tensor(ingresso[i:i + BLOCCO_INFERENZA], dtype=torch.float32, device=device)
            pezzi.append(modello(x).cpu().numpy())
    correzione = np.concatenate(pezzi)
    return _statistiche(pred_kalman, verita), _statistiche(pred_kalman + correzione, verita)


def principale():
    unita, strati, pazienza, epoche = 96, 1, PAZIENZA, EPOCHE_MASSIME
    for a in sys.argv[1:]:
        if a.startswith("--unita="):
            unita = int(a.split("=", 1)[1])
        elif a.startswith("--strati="):
            strati = int(a.split("=", 1)[1])
        elif a.startswith("--pazienza="):
            pazienza = int(a.split("=", 1)[1])
        elif a.startswith("--epoche="):
            epoche = int(a.split("=", 1)[1])

    storico, pred_kalman, verita, scenario = ap.carica_dataset(DATASET)
    add, sel, test = _tre_insiemi(scenario)
    residuo = verita - pred_kalman
    ingresso = ap.prepara_input(storico)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"{len(verita)} campioni, {len(set(scenario.tolist()))} scenari")
    print(f"  addestramento {add.sum():>6}   selezione {sel.sum():>6}   test {test.sum():>6}")
    print(f"  GRU {unita} x {strati}, fino a {epoche} epoche, arresto con pazienza {pazienza}")
    print(f"  device {device}\n")

    x_add = torch.tensor(ingresso[add], dtype=torch.float32, device=device)
    y_add = torch.tensor(residuo[add], dtype=torch.float32, device=device)
    x_sel = torch.tensor(ingresso[sel], dtype=torch.float32, device=device)
    kalman_sel, verita_sel = pred_kalman[sel], verita[sel]
    errore_kalman_sel = _errore_medio(kalman_sel, verita_sel)

    torch.manual_seed(0)
    modello = CorrezioneKalmanProfonda(dimensione_nascosta=unita, strati=strati).to(device)
    ottimizzatore = torch.optim.Adam(modello.parameters(), lr=ap.TASSO_APPRENDIMENTO)
    perdita_mse = nn.MSELoss()

    migliore_errore, migliori_pesi, migliore_epoca = float("inf"), None, 0
    n = x_add.shape[0]
    avvio = time.time()
    for epoca in range(1, epoche + 1):
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
            errore = _errore_medio(kalman_sel + modello(x_sel).cpu().numpy(), verita_sel)

        if errore < migliore_errore:
            # copia profonda: i pesi continueranno a cambiare nelle epoche successive
            migliore_errore, migliore_epoca = errore, epoca
            migliori_pesi = copy.deepcopy(modello.state_dict())
            marcatore = "  <-- migliore"
        else:
            marcatore = ""
        if epoca % 5 == 0 or marcatore:
            print(f"  epoca {epoca:>3}: addestramento {accumulata / n:>8.1f}   "
                  f"selezione {errore:>6.2f} px  ({100 * (1 - errore / errore_kalman_sel):+.2f}%)"
                  f"{marcatore}", flush=True)

        if epoca - migliore_epoca >= pazienza:
            print(f"\n  arresto anticipato: {pazienza} epoche senza miglioramento.")
            break

    print(f"\n  {time.time() - avvio:.0f}s, {epoca} epoche eseguite su {epoche} consentite")
    print(f"  epoca migliore: {migliore_epoca}   errore sulla selezione {migliore_errore:.2f} px "
          f"({100 * (1 - migliore_errore / errore_kalman_sel):+.2f}% sul Kalman)")

    modello.load_state_dict(migliori_pesi)
    os.makedirs(ap.CARTELLA_MODELLI, exist_ok=True)
    torch.save(migliori_pesi, FILE_USCITA)
    print(f"  salvati i pesi dell'epoca {migliore_epoca} in {FILE_USCITA}")
    if strati == 1:
        print("  (a uno strato lo state_dict e' compatibile con CorrezioneKalman: il modello e'")
        print("   caricabile dal simulatore senza modifiche)")

    if "--misura-finale" not in sys.argv:
        print("\n  Misura finale NON eseguita. Il test della tesi e l'insieme di Povo restano")
        print("  intatti: aggiungere --misura-finale quando la configurazione e' decisa.")
        return

    print("\n" + "=" * 78)
    print("MISURA FINALE - una volta sola, su insiemi mai impiegati in alcuna decisione")
    print("=" * 78)
    for nome, ing, kal, ver in (
        ("test della tesi", ingresso[test], pred_kalman[test], verita[test]),
        ("Povo", None, None, None),
    ):
        if nome == "Povo":
            if not os.path.exists(DATASET_POVO):
                print(f"\n  {DATASET_POVO} non trovato: misura su Povo saltata.")
                continue
            s_p, k_p, v_p, _ = ap.carica_dataset(DATASET_POVO)
            ing, kal, ver = ap.prepara_input(s_p), k_p, v_p
        base, corretto = _valuta(modello, ing, kal, ver, device)
        print(f"\n  {nome}  ({len(ver)} campioni)")
        print(f"    {'':14}{'medio':>9}{'mediano':>9}{'90-perc':>9}")
        print(f"    {'Kalman':14}{base[0]:>9.2f}{base[1]:>9.2f}{base[2]:>9.2f}")
        print(f"    {'Kalman + rete':14}{corretto[0]:>9.2f}{corretto[1]:>9.2f}{corretto[2]:>9.2f}")
        print(f"    riduzione dell'errore medio: {100 * (1 - corretto[0] / base[0]):+.2f}%")

    print("\n  Da questo momento i due insiemi hanno partecipato a una misura riportata: ogni")
    print("  ulteriore confronto su di essi va considerato ottimistico e va dichiarato tale.")


if __name__ == "__main__":
    principale()

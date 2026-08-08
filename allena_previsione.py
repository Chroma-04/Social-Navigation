"""FASE 2 (Livello 2 della tesi - previsione): allena il modello di correzione AI sul dataset generato da
genera_dataset_previsione.py.

Il modello impara il RESIDUO (posizione vera - previsione del Kalman) da un breve storico di stato
filtrato dal Kalman: la previsione finale resta "Kalman + correzione", non sostituisce il Kalman - vedi la
discussione sul perche' di questa scelta (l'AI rinforza il classico invece di ripartire da zero).

Split train/test per COMBINAZIONE di scenario (non per singolo campione): campioni della stessa traccia
hanno finestre di storico sovrapposte, mescolarli a livello di singolo campione farebbe trapelare
informazione fra train e test. Tenendo interi scenari fuori dal training da' anche una lettura onesta di
generalizzazione (il modello viene valutato su ambienti/densita' mai visti), non solo di overfitting.

Uso:
    py allena_previsione.py [percorso_dataset.npz]   default: dataset/dataset_previsione.npz, o quello
                                                       rapido se il completo non esiste ancora
"""
import os
import sys
import time
import numpy as np
import torch
import torch.nn as nn

CARTELLA_DATASET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")
CARTELLA_MODELLI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modelli")
FILE_MODELLO = os.path.join(CARTELLA_MODELLI, "correzione_kalman.pt")

# deve restare identica a FINESTRA_STORICO_FRAME in genera_dataset_previsione.py: e' la forma dell'input
# (numero di frame di storico) su cui il modello viene allenato, e va rispettata anche in inferenza
FINESTRA_STORICO_FRAME = 30
FRAZIONE_TEST = 0.2
DIMENSIONE_NASCOSTA = 32
NUMERO_EPOCHE = 60
DIMENSIONE_BATCH = 256
TASSO_APPRENDIMENTO = 1e-3


class CorrezioneKalman(nn.Module):
    """GRU leggera: legge lo storico (posizione relativa + velocita') e produce una correzione (dx, dy) da
    sommare alla previsione del Kalman. Volutamente piccola: il compito e' imparare solo lo scarto
    residuo (svolte, dinamiche di gruppo), non ripartire da zero come un modello puramente predittivo."""
    def __init__(self, dimensione_input=4, dimensione_nascosta=DIMENSIONE_NASCOSTA):
        super().__init__()
        self.gru = nn.GRU(dimensione_input, dimensione_nascosta, batch_first=True)
        self.testa = nn.Linear(dimensione_nascosta, 2)

    def forward(self, storico):
        _, h_finale = self.gru(storico)
        return self.testa(h_finale.squeeze(0))


def carica_dataset(path):
    d = np.load(path)
    return d["storico"], d["pred_kalman"], d["verita"], d["scenario"]


def prepara_input(storico):
    """Posizione relativa all'ultimo istante dello storico invece che coordinate assolute sullo schermo
    (che non contano nulla per il pattern di moto, solo lo spostamento relativo conta) + velocita' invariata."""
    storico = storico.copy()
    storico[:, :, 0:2] -= storico[:, -1:, 0:2]
    return storico


def divide_train_test(scenario, frazione_test, seed=0):
    scenari_unici = sorted(set(scenario.tolist()))
    rng = np.random.RandomState(seed)
    rng.shuffle(scenari_unici)
    n_test = max(1, int(len(scenari_unici) * frazione_test))
    scenari_test = set(scenari_unici[:n_test])
    maschera_test = np.array([s in scenari_test for s in scenario])
    return ~maschera_test, maschera_test


def errori_statistiche(previsioni, verita):
    errori = np.linalg.norm(previsioni - verita, axis=1)
    return errori.mean(), np.median(errori), np.percentile(errori, 90)


def main(path_dataset):
    print(f"Carico dataset: {path_dataset}")
    storico, pred_kalman, verita, scenario = carica_dataset(path_dataset)
    n = len(verita)
    print(f"{n} campioni, {len(set(scenario.tolist()))} combinazioni di scenario diverse")
    if n < 500:
        print("ATTENZIONE: pochi campioni per un training sensato (probabilmente il dataset rapido, non quello completo) - risultati solo indicativi.\n")

    maschera_train, maschera_test = divide_train_test(scenario, FRAZIONE_TEST)
    print(f"Train: {maschera_train.sum()} campioni  |  Test: {maschera_test.sum()} campioni (scenari mai visti in training)\n")

    residuo = verita - pred_kalman
    storico_norm = prepara_input(storico)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Device: {device}\n")

    x_train = torch.tensor(storico_norm[maschera_train], dtype=torch.float32, device=device)
    y_train = torch.tensor(residuo[maschera_train], dtype=torch.float32, device=device)
    x_test = torch.tensor(storico_norm[maschera_test], dtype=torch.float32, device=device)

    modello = CorrezioneKalman().to(device)
    ottimizzatore = torch.optim.Adam(modello.parameters(), lr=TASSO_APPRENDIMENTO)
    funzione_perdita = nn.MSELoss()

    n_train = x_train.shape[0]
    inizio = time.time()
    for epoca in range(NUMERO_EPOCHE):
        modello.train()
        permutazione = torch.randperm(n_train, device=device)
        perdita_totale = 0.0
        for i in range(0, n_train, DIMENSIONE_BATCH):
            indici = permutazione[i:i + DIMENSIONE_BATCH]
            batch_x, batch_y = x_train[indici], y_train[indici]
            ottimizzatore.zero_grad()
            previsione = modello(batch_x)
            perdita = funzione_perdita(previsione, batch_y)
            perdita.backward()
            ottimizzatore.step()
            perdita_totale += perdita.item() * len(indici)
        if (epoca + 1) % 10 == 0 or epoca == 0:
            print(f"  epoca {epoca + 1}/{NUMERO_EPOCHE}: loss train {perdita_totale / n_train:.1f}")

    print(f"\nTraining completato in {time.time() - inizio:.1f}s\n")

    modello.eval()
    with torch.no_grad():
        correzione_test = modello(x_test).cpu().numpy()
    pred_kalman_test = pred_kalman[maschera_test]
    verita_test = verita[maschera_test]
    previsione_corretta_test = pred_kalman_test + correzione_test

    err_kalman = errori_statistiche(pred_kalman_test, verita_test)
    err_corretto = errori_statistiche(previsione_corretta_test, verita_test)

    print("=== Confronto sul test set (scenari mai visti in training) ===")
    print(f"{'':20}{'medio':>10}{'mediano':>10}{'90-perc':>10}")
    print(f"{'Kalman solo':20}{err_kalman[0]:10.1f}{err_kalman[1]:10.1f}{err_kalman[2]:10.1f}")
    print(f"{'Kalman + AI':20}{err_corretto[0]:10.1f}{err_corretto[1]:10.1f}{err_corretto[2]:10.1f}")
    miglioramento = (1 - err_corretto[0] / err_kalman[0]) * 100
    print(f"\nMiglioramento errore medio: {miglioramento:+.1f}%")

    os.makedirs(CARTELLA_MODELLI, exist_ok=True)
    torch.save(modello.state_dict(), FILE_MODELLO)
    print(f"\nModello salvato in: {FILE_MODELLO}")


if __name__ == "__main__":
    path_dataset = sys.argv[1] if len(sys.argv) > 1 else None
    if path_dataset is None:
        candidato_completo = os.path.join(CARTELLA_DATASET, "dataset_previsione.npz")
        candidato_rapido = os.path.join(CARTELLA_DATASET, "dataset_previsione_rapido.npz")
        if os.path.exists(candidato_completo):
            path_dataset = candidato_completo
        elif os.path.exists(candidato_rapido):
            path_dataset = candidato_rapido
            print("Dataset completo non trovato, uso quello rapido (risultati solo indicativi).\n")
        else:
            print("Nessun dataset trovato: esegui prima genera_dataset_previsione.py")
            sys.exit(1)
    main(path_dataset)

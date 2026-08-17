"""FASE 4 (Livello 2 della tesi - previsione): analisi stratificata di QUANDO la correzione AI aiuta e
quanto, non solo QUANTO in media. Il numero aggregato (+22.3% sul test set, riportato da
allena_previsione.py) risponde a "funziona?" ma nasconde la domanda piu' interessante per la tesi: aiuta
allo stesso modo ovunque, o ci sono condizioni (densita' di folla, tipo di comportamento, complessita'
strutturale della mappa) in cui aiuta molto di piu' - o pochissimo, o addirittura peggiora la previsione
rispetto al solo Kalman?

Usa l'INTERO dataset (non solo il test set) per avere numeri per-strato statisticamente piu' solidi: molte
combinazioni mappa/densita'/mix hanno gia' pochi campioni sull'intero dataset, tenendo solo il ~20% di test
alcuni strati avrebbero troppo pochi campioni per dire qualcosa. Questo e' voluto e diverso dallo scopo di
allena_previsione.py: li' l'obiettivo era una misura ONESTA di generalizzazione (scenari mai visti), qui e'
DESCRITTIVO/diagnostico (dove si concentra il miglioramento) - il numero di generalizzazione onesto resta
quello gia' riportato in training, non viene ricalcolato qui.

Uso:
    py analisi_stratificata.py [percorso_dataset.npz]   default: dataset/dataset_previsione.npz
"""
import os
import sys
import numpy as np
import torch
from allena_previsione import (CorrezioneKalman, prepara_input, carica_dataset, FILE_MODELLO,
                               FILE_MODELLO_SPECIALIZZATO, CARTELLA_DATASET,
                               DIMENSIONE_NASCOSTA, DIMENSIONE_NASCOSTA_SPECIALIZZATO)

# mappe raggruppate per tipo strutturale: lo sweep di densita' (verifica_densita_massima.py) ha gia' mostrato
# che i colli di bottiglia (porta_stretta/doppia_porta/corridoio) saturano molto prima delle mappe aperte o
# con ostacoli sparsi - ha senso chiedersi se la correzione AI si comporta diversamente li'
TIPO_MAPPA = {
    "aperta": "aperta",
    "ostacoli_sparsi": "ostacoli",
    "ostacoli_incrociati": "ostacoli",
    "ostacoli_organici": "ostacoli",
    "corridoio": "collo_di_bottiglia",
    "porta_stretta": "collo_di_bottiglia",
    "doppia_porta": "collo_di_bottiglia",
    "incrocio": "incrocio",
}


def _tipo_mappa(nome_mappa):
    if nome_mappa.startswith("training_"):
        return "disegnata_a_mano"
    return TIPO_MAPPA.get(nome_mappa, "altro")


def carica_e_correggi(path_dataset, specializzato=False):
    """Carica il dataset e fa girare il modello gia' allenato su OGNI campione (nessun training qui, solo
    inferenza) per avere, per ciascuno, sia l'errore del solo Kalman sia quello di Kalman+AI da confrontare.

    'specializzato' sceglie il modello grande, quello che testing.py carica quando ACCELERAZIONE > 1: e'
    l'unico confronto lecito con le campagne sui tempi, che girano tutte a passo accelerato. Con il modello
    base i numeri di previsione non corrisponderebbero al robot effettivamente misurato."""
    storico, pred_kalman, verita, scenario = carica_dataset(path_dataset)
    # i dataset vecchi hanno 'mappa|densita|mix', quelli generati per Povo aggiungono la ripetizione
    # ('povo|pop50|specializzato|r1'): si leggono i primi tre campi e si ignora il resto
    campi = [s.split("|") for s in scenario]
    mappe = np.array([c[0] for c in campi])
    densita = np.array([c[1] for c in campi])
    mix = np.array([c[2] for c in campi])
    tipi_mappa = np.array([_tipo_mappa(m) for m in mappe])

    path_modello = FILE_MODELLO_SPECIALIZZATO if specializzato else FILE_MODELLO
    dimensione = DIMENSIONE_NASCOSTA_SPECIALIZZATO if specializzato else DIMENSIONE_NASCOSTA
    modello = CorrezioneKalman(dimensione_nascosta=dimensione)
    modello.load_state_dict(torch.load(path_modello, map_location="cpu"))
    modello.eval()
    with torch.no_grad():
        correzione = modello(torch.from_numpy(prepara_input(storico))).numpy()
    pred_ai = pred_kalman + correzione

    err_kalman = np.linalg.norm(pred_kalman - verita, axis=1)
    err_ai = np.linalg.norm(pred_ai - verita, axis=1)
    return err_kalman, err_ai, mappe, densita, mix, tipi_mappa


def _stampa_gruppo(titolo, chiavi, err_kalman, err_ai, n_minimo=20):
    print(f"\n=== {titolo} ===")
    print(f"{'':24}{'n':>7}{'Kalman':>10}{'Kalman+AI':>12}{'miglioramento':>15}")
    righe = []
    for chiave in sorted(set(chiavi)):
        maschera = chiavi == chiave
        n = int(maschera.sum())
        if n < n_minimo:
            continue
        m_kalman = err_kalman[maschera].mean()
        m_ai = err_ai[maschera].mean()
        miglioramento = (1 - m_ai / m_kalman) * 100 if m_kalman > 0 else 0.0
        righe.append((chiave, n, m_kalman, m_ai, miglioramento))
    for chiave, n, m_kalman, m_ai, miglioramento in sorted(righe, key=lambda r: -r[4]):
        segno = "" if miglioramento >= 0 else "  <-- l'AI PEGGIORA qui"
        print(f"{chiave:24}{n:7}{m_kalman:10.1f}{m_ai:12.1f}{miglioramento:14.1f}%{segno}")
    scartate = len(set(chiavi)) - len(righe)
    if scartate:
        print(f"({scartate} valori scartati per meno di {n_minimo} campioni)")


def main(path_dataset, specializzato=False):
    print(f"Carico dataset e modello: {path_dataset}"
          f" ({'modello specializzato' if specializzato else 'modello base'})")
    err_kalman, err_ai, mappe, densita, mix, tipi_mappa = carica_e_correggi(path_dataset, specializzato)
    print(f"{len(err_kalman)} campioni totali (intero dataset, non solo il test set - vedi nota in testa al file)")
    print(f"Miglioramento medio complessivo: {(1 - err_ai.mean() / err_kalman.mean()) * 100:.1f}%")

    _stampa_gruppo("Per livello di densita'", densita, err_kalman, err_ai)
    _stampa_gruppo("Per mix comportamentale", mix, err_kalman, err_ai)
    _stampa_gruppo("Per tipo strutturale di mappa", tipi_mappa, err_kalman, err_ai)
    _stampa_gruppo("Per mappa (dettaglio)", mappe, err_kalman, err_ai)

    # interazione densita' x mix: dove il miglioramento e' piu'/meno marcato incrociando le due dimensioni
    combinato = np.array([f"{d}/{x}" for d, x in zip(densita, mix)])
    _stampa_gruppo("Per densita' x mix (combinazioni)", combinato, err_kalman, err_ai)


if __name__ == "__main__":
    argomenti = [a for a in sys.argv[1:] if a != "--specializzato"]
    path_dataset = argomenti[0] if argomenti else os.path.join(CARTELLA_DATASET, "dataset_previsione.npz")
    main(path_dataset, specializzato="--specializzato" in sys.argv)

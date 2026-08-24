"""Quanto costa, in tempo, l'inferenza della rete rispetto al ciclo di controllo.

Il costo computazionale e' dichiarato in tesi fra le variabili in gioco ma non quantificato
per la componente appresa. Questo banco lo misura direttamente: una passata in avanti della
GRU 48x2 su un lotto pari al numero di persone contemporaneamente tracciate, che e' esattamente
come la rete viene interrogata dal vivo (un solo lotto per aggiornamento della macchia).

I termini di paragone sono tre, tutti gia' in tesi:
  - il fotogramma, 16,7 ms a 60 FPS;
  - l'intervallo di aggiornamento della macchia, 6 fotogrammi = 100 ms;
  - la ripianificazione del robot, 58 ms sulla griglia dell'ambiente e 244 ms su quella fine.

    py ai_predittiva/latenza_inferenza.py
"""
import os
import sys
import time
import statistics

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import torch
import allena_previsione as ap

FILE_V2 = os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt")
FINESTRA = 30           # istanti di storico per traccia
CANALI = 4              # posizione e velocita' filtrate
LOTTI = (1, 9, 14, 21, 30, 60)   # persone tracciate: 9,2 alla portata di esercizio, 30,3 a 16 m
RIPETIZIONI = 200
FOTOGRAMMA_MS = 1000.0 / 60


def _misura(modello, dispositivo, n):
    x = torch.randn(n, FINESTRA, CANALI, device=dispositivo)
    with torch.no_grad():
        for _ in range(20):          # riscaldamento: la prima passata alloca e compila
            modello(x)
        if dispositivo == "cuda":
            torch.cuda.synchronize()
        tempi = []
        for _ in range(RIPETIZIONI):
            avvio = time.perf_counter()
            modello(x)
            if dispositivo == "cuda":
                torch.cuda.synchronize()
            tempi.append((time.perf_counter() - avvio) * 1000)
    return statistics.median(tempi), statistics.quantiles(tempi, n=100)[94]


def principale():
    parametri = None
    print(f"fotogramma a 60 FPS: {FOTOGRAMMA_MS:.2f} ms   "
          f"aggiornamento macchia: ogni 6 fotogrammi = {6*FOTOGRAMMA_MS:.0f} ms\n")
    for dispositivo in ("cpu", "cuda"):
        if dispositivo == "cuda" and not torch.cuda.is_available():
            print("cuda non disponibile, saltata")
            continue
        modello = ap.modello_da_file(FILE_V2, device=dispositivo).to(dispositivo)
        modello.eval()
        if parametri is None:
            parametri = sum(p.numel() for p in modello.parameters())
            print(f"rete: GRU 48x2, {parametri} parametri\n")
        nome = "CPU" if dispositivo == "cpu" else torch.cuda.get_device_name(0)
        print(f"  {nome}")
        print(f"  {'persone':>9}{'mediana':>11}{'95-esimo':>11}{'% fotogramma':>15}")
        for n in LOTTI:
            mediana, novantacinque = _misura(modello, dispositivo, n)
            print(f"  {n:>9}{mediana:>10.3f}ms{novantacinque:>10.3f}ms"
                  f"{100*mediana/FOTOGRAMMA_MS:>14.2f}%")
        print()


if __name__ == "__main__":
    principale()

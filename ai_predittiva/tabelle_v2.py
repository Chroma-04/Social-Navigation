"""Ricalcola con la rete v2 tutte le grandezze di previsione riportate in tesi.

Nessuna simulazione: sono tutte misure sul dataset, quindi il confronto con i numeri della
tesi e' esatto e non soggetto a variabilita' di scena. Copre:

  Tabella 2 (5.2.8)   errore di previsione medio, mediano e 90-esimo percentile
  5.2.8 / 6.4         ampiezza delle correzioni e frazione che supera mezza cella e una cella
  6.2                 validita' su Povo: riduzione complessiva e per livello di densita'

Le due reti sono valutate sugli STESSI campioni, cosi' ogni riga e' un confronto appaiato.

    py ai_predittiva/tabelle_v2.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import torch

import allena_previsione as ap
from sweep_capacita import _tre_insiemi, DATASET
from allena_con_arresto import DATASET_POVO, BLOCCO_INFERENZA

PIXEL_PER_METRO = 30.0          # DIM_NODO = 30 px, METRI_PER_CELLA = 1,0
LATO_CELLA_M = 1.0              # celle di decisione da un metro (6.2)
MODELLI = (("tesi  (96x1)", "correzione_kalman_specializzato.pt"),
           ("nuovo (48x2)", "correzione_kalman_specializzato_v2.pt"))


def _correzioni(modello, ingresso, device):
    pezzi = []
    with torch.no_grad():
        for i in range(0, len(ingresso), BLOCCO_INFERENZA):
            x = torch.tensor(ingresso[i:i + BLOCCO_INFERENZA], dtype=torch.float32, device=device)
            pezzi.append(modello(x).cpu().numpy())
    return np.concatenate(pezzi)


def _statistiche_m(previsioni, verita):
    e = np.linalg.norm(previsioni - verita, axis=1) / PIXEL_PER_METRO
    return e.mean(), np.median(e), np.percentile(e, 90)


def principale():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    storico, kal, ver, scen = ap.carica_dataset(DATASET)
    _, _, test = _tre_insiemi(scen)
    ing = ap.prepara_input(storico)

    s_p, k_p, v_p, scen_p = ap.carica_dataset(DATASET_POVO)
    ing_p = ap.prepara_input(s_p)
    densita_p = np.array([s.split("|")[1] for s in scen_p])

    reti = {}
    for nome, file in MODELLI:
        perc = os.path.join(ap.CARTELLA_MODELLI, file)
        if not os.path.exists(perc):
            print(f"manca {perc}")
            return
        reti[nome] = ap.modello_da_file(perc, device).to(device)

    # ---------------- Tabella 2: errore di previsione sul test della tesi ----------------
    print("=" * 92)
    print(f"TABELLA 2 --- errore di previsione a un secondo, test della tesi ({test.sum()} campioni)")
    print("=" * 92)
    base = _statistiche_m(k_p[:0] if False else kal[test], ver[test])
    print(f"  {'':22}{'medio':>10}{'mediano':>10}{'90-esimo':>10}")
    print(f"  {'Kalman':22}{base[0]:>10.2f}{base[1]:>10.2f}{base[2]:>10.2f}")
    for nome in reti:
        c = _correzioni(reti[nome], ing[test], device)
        s = _statistiche_m(kal[test] + c, ver[test])
        print(f"  {'Kalman + rete ' + nome:22}{s[0]:>10.2f}{s[1]:>10.2f}{s[2]:>10.2f}")
        print(f"  {'  riduzione':22}{100*(1-s[0]/base[0]):>9.1f}%{100*(1-s[1]/base[1]):>9.1f}%"
              f"{100*(1-s[2]/base[2]):>9.1f}%")

    # ---------------- ampiezza delle correzioni ----------------
    print("\n" + "=" * 92)
    print("AMPIEZZA DELLE CORREZIONI --- quanto la rete sposta la previsione (5.2.8 e 6.4)")
    print("=" * 92)
    print(f"  {'rete':16}{'insieme':10}{'media':>9}{'mediana':>9}{'>mezza cella':>14}"
          f"{'>una cella':>12}{'avvicinamento':>15}")
    for nome in reti:
        for eti, i_, k_, v_ in (("test", ing[test], kal[test], ver[test]),
                                ("Povo", ing_p, k_p, v_p)):
            c = _correzioni(reti[nome], i_, device)
            amp = np.linalg.norm(c, axis=1) / PIXEL_PER_METRO
            e0 = np.linalg.norm(k_ - v_, axis=1) / PIXEL_PER_METRO
            e1 = np.linalg.norm(k_ + c - v_, axis=1) / PIXEL_PER_METRO
            print(f"  {nome:16}{eti:10}{amp.mean():>8.2f}m{np.median(amp):>8.2f}m"
                  f"{100*(amp > LATO_CELLA_M/2).mean():>13.1f}%{100*(amp > LATO_CELLA_M).mean():>11.1f}%"
                  f"{e0.mean()-e1.mean():>14.2f}m")
    print("\n  'avvicinamento' = di quanto l'errore medio si riduce: e' la frazione UTILE dello")
    print("  spostamento. Il rapporto fra media e avvicinamento e' il numero citato in 6.4.")

    # ---------------- validita' su Povo, per livello di densita' ----------------
    print("\n" + "=" * 92)
    print(f"VALIDITA' SU POVO (6.2) --- {len(v_p)} campioni, mai impiegati in addestramento")
    print("=" * 92)
    livelli = sorted(set(densita_p.tolist()), key=lambda s: int(s[3:]))
    corr = {nome: _correzioni(reti[nome], ing_p, device) for nome in reti}
    intestazione = f"  {'livello':>9}{'campioni':>10}{'Kalman':>10}"
    for nome in reti:
        intestazione += f"{nome:>16}"
    print(intestazione)
    riduzioni = {nome: [] for nome in reti}
    for liv in livelli:
        m = densita_p == liv
        e0 = np.linalg.norm(k_p[m] - v_p[m], axis=1).mean() / PIXEL_PER_METRO
        riga = f"  {liv:>9}{int(m.sum()):>10}{e0:>9.2f}m"
        for nome in reti:
            e1 = np.linalg.norm(k_p[m] + corr[nome][m] - v_p[m], axis=1).mean() / PIXEL_PER_METRO
            r = 100 * (1 - e1 / e0)
            riduzioni[nome].append(r)
            riga += f"{r:>15.1f}%"
        print(riga)
    e0 = np.linalg.norm(k_p - v_p, axis=1).mean() / PIXEL_PER_METRO
    riga = f"  {'TOTALE':>9}{len(v_p):>10}{e0:>9.2f}m"
    for nome in reti:
        e1 = np.linalg.norm(k_p + corr[nome] - v_p, axis=1).mean() / PIXEL_PER_METRO
        riga += f"{100*(1-e1/e0):>15.1f}%"
    print(riga)
    for nome in reti:
        print(f"  {nome}: intervallo fra i livelli {min(riduzioni[nome]):.1f}% e {max(riduzioni[nome]):.1f}%")


if __name__ == "__main__":
    principale()

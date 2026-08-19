"""Grafico: numero di fermate in funzione della densita' di popolazione, con e senza controllo.

Legge velocita_classico_povo_sweep.csv e traccia le due condizioni con barre di errore
(errore standard sui semi di ciascun livello). E' la lettura che il conteggio dei segni non
poteva dare: dove le due curve si separano, e in che verso.

Il pannello inferiore mostra la variazione percentuale delle fermate, cosi' la fascia in cui
il controllo le riduce si legge come area sotto lo zero invece che confrontando due curve
che crescono entrambe.

Uso:
    py ibridazione_velocita/grafico_fermate.py
"""
import os
import csv
import sys
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")  # nessuna finestra: si salva su file
import matplotlib.pyplot as plt

_QUI = os.path.dirname(os.path.abspath(__file__))
_RADICE = os.path.dirname(_QUI)

FILE_CSV = os.path.join(_RADICE, "risultati_test", "velocita_classico_povo_sweep.csv")
FILE_PNG = os.path.join(_QUI, "fermate_per_densita.png")

# superficie calpestabile della planimetria di Povo, alla scala di 1 cella = 1 m
AREA_LIBERA_M2 = 1997.0


def _media_errore(valori):
    """Media e errore standard. Con un solo valore l'errore non e' definito: si restituisce 0
    per non far esplodere il grafico, ma quel punto non ha barra."""
    media = statistics.mean(valori)
    if len(valori) < 2:
        return media, 0.0
    return media, statistics.stdev(valori) / len(valori) ** 0.5


def main():
    if not os.path.exists(FILE_CSV):
        print(f"Manca {FILE_CSV}: esegui prima performance_velocita_classico.py")
        return 1

    per_livello = defaultdict(lambda: {"base": [], "controllo": []})
    with open(FILE_CSV, encoding="utf-8") as f:
        for riga in csv.DictReader(f):
            livello = riga["livello"]
            per_livello[livello]["base"].append(float(riga["numero_fermate_base"]))
            per_livello[livello]["controllo"].append(float(riga["numero_fermate_controllo"]))

    # l'etichetta e' 'popNNN': il numero e' la popolazione totale, da cui la densita'
    livelli = sorted(per_livello, key=lambda e: int(e[3:]))
    popolazioni = [int(e[3:]) for e in livelli]
    densita = [p / AREA_LIBERA_M2 for p in popolazioni]

    media_b, errore_b, media_c, errore_c, variazione = [], [], [], [], []
    for livello in livelli:
        mb, eb = _media_errore(per_livello[livello]["base"])
        mc, ec = _media_errore(per_livello[livello]["controllo"])
        media_b.append(mb); errore_b.append(eb)
        media_c.append(mc); errore_c.append(ec)
        variazione.append((mc - mb) / mb * 100 if mb else 0.0)

    n_semi = len(per_livello[livelli[0]]["base"])
    figura, (asse, asse_var) = plt.subplots(
        2, 1, figsize=(9, 7.5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})

    asse.errorbar(densita, media_b, yerr=errore_b, marker="o", capsize=4, linewidth=2,
                  color="#3b6ea5", label="senza controllo di velocità")
    asse.errorbar(densita, media_c, yerr=errore_c, marker="s", capsize=4, linewidth=2,
                  color="#c2492f", label="con controllo di velocità")
    asse.set_ylabel("numero di fermate per corsa")
    asse.set_title(f"Fermate del robot in funzione della densità di folla\n"
                   f"Povo 1, {n_semi} semi appaiati per livello, barre = errore standard",
                   fontsize=11)
    asse.legend()
    asse.grid(alpha=0.3)

    colori = ["#2e7d32" if v < 0 else "#c2492f" for v in variazione]
    asse_var.bar(densita, variazione, width=0.006, color=colori, alpha=0.85)
    asse_var.axhline(0, color="black", linewidth=1)
    asse_var.set_ylabel("variazione fermate (%)")
    asse_var.set_xlabel("densità di folla (persone/m²)  —  superficie calpestabile 1997 m²")
    asse_var.grid(alpha=0.3, axis="y")
    asse_var.text(0.01, 0.06, "verde: il controllo riduce le fermate",
                  transform=asse_var.transAxes, fontsize=9, color="#2e7d32")

    # secondo asse in alto con la popolazione assoluta, piu' leggibile della densita'
    asse_alto = asse.secondary_xaxis("top", functions=(lambda d: d * AREA_LIBERA_M2,
                                                       lambda p: p / AREA_LIBERA_M2))
    asse_alto.set_xlabel("individui compresenti")

    figura.tight_layout()
    figura.savefig(FILE_PNG, dpi=150)
    print(f"Grafico salvato in: {FILE_PNG}\n")

    print(f"{'densita':>10}{'individui':>11}{'senza':>9}{'con':>9}{'variazione':>13}")
    for d, p, mb, mc, v in zip(densita, popolazioni, media_b, media_c, variazione):
        print(f"{d:>10.3f}{p:>11}{mb:>9.1f}{mc:>9.1f}{v:>+12.0f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Traccia le quattro configurazioni scena per scena, sulle 24 prove comuni a tutte.

Le medie aggregate nascondono quanto le quattro curve siano intrecciate: un grafico per campione
mostra a colpo d'occhio se una configurazione domina le altre o se le differenze sono minori
della variabilita' fra scene. Le scene sono ordinate per densita' crescente, cosi' l'andamento
di fondo (piu' persone, piu' fermate) resta leggibile sotto il confronto.

    py ibridazione_velocita/grafico_confronto.py                 fermate (predefinito)
    py ibridazione_velocita/grafico_confronto.py --grandezza=tempo_s
    py ibridazione_velocita/grafico_confronto.py --tutte         le quattro grandezze insieme
"""
import os
import sys
import csv

import matplotlib
matplotlib.use("Agg")   # nessuna finestra: si scrive un file e basta
import matplotlib.pyplot as plt

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CARTELLA = os.path.join(_RADICE, "risultati_test")

# ogni configurazione vive in un file diverso, con un suffisso di colonna diverso
SORGENTI = {
    "riferimento":            ("confronto_completo_velocita_libera.csv", "_base",      "velocita_libera"),
    "controllo di velocità":  ("confronto_completo_velocita_libera.csv", "_variante",  "velocita_libera"),
    "controllo + rete":       ("ai_velocita_povo.csv",                   "_ai",        None),
    "controllo oracolo":      ("oracolo_velocita_povo.csv",              "_oracolo",   None),
}
STILE = {   # colore, marcatore, spessore: il riferimento tratteggiato perche' e' il termine di paragone
    "riferimento":           ("#888888", "o", 1.6, "--"),
    "controllo di velocità": ("#1f77b4", "s", 2.0, "-"),
    "controllo + rete":      ("#d62728", "^", 2.0, "-"),
    "controllo oracolo":     ("#2ca02c", "D", 2.0, "-"),
}
GRANDEZZE = {
    "numero_fermate":     ("Numero di fermate", "arresti completi"),
    "tempo_s":            ("Tempo di percorrenza", "secondi"),
    "fermo_s":            ("Tempo trascorso fermo", "secondi"),
    "distanza_media_m":   ("Distanza dalle persone", "metri"),
}


def _indice(file_csv, condizione=None):
    d = {}
    with open(os.path.join(CARTELLA, file_csv), newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if condizione and r.get("condizione") != condizione:
                continue
            d[(r["livello"], int(r["seme"]))] = r
    return d


def carica():
    """Le 24 scene su cui tutte e quattro le configurazioni hanno effettivamente girato."""
    indici = {n: _indice(f, c) for n, (f, _, c) in SORGENTI.items()}
    chiavi = set.intersection(*(set(i) for i in indici.values()))
    chiavi = sorted(chiavi, key=lambda k: (int(k[0][3:]), k[1]))   # densita' crescente
    return indici, chiavi


def disegna(asse, indici, chiavi, grandezza):
    titolo, unita = GRANDEZZE[grandezza]
    x = range(1, len(chiavi) + 1)
    for nome, (_, suffisso, _) in SORGENTI.items():
        y = [float(indici[nome][k][grandezza + suffisso]) for k in chiavi]
        colore, marcatore, spessore, tratto = STILE[nome]
        asse.plot(x, y, tratto, color=colore, marker=marcatore, linewidth=spessore,
                  markersize=4, label=nome, alpha=0.85)
    # separatori fra i livelli di densita', per leggere il grafico a blocchi
    livelli = [k[0] for k in chiavi]
    for i in range(1, len(livelli)):
        if livelli[i] != livelli[i - 1]:
            asse.axvline(i + 0.5, color="#dddddd", linewidth=0.8, zorder=0)
    etichette = []
    for i, l in enumerate(livelli):
        etichette.append(l.replace("pop", "") if i == 0 or l != livelli[i - 1] else "")
    asse.set_xticks(list(x))
    asse.set_xticklabels(etichette, fontsize=8)
    asse.set_xlabel("scena (raggruppate per popolazione, densità crescente →)")
    asse.set_ylabel(unita)
    asse.set_title(titolo)
    asse.grid(alpha=0.25, linewidth=0.6)


def principale():
    grandezza = "numero_fermate"
    for a in sys.argv[1:]:
        if a.startswith("--grandezza="):
            grandezza = a.split("=", 1)[1]
    indici, chiavi = carica()
    print(f"{len(chiavi)} scene comuni a tutte e quattro le configurazioni")

    if "--tutte" in sys.argv:
        fig, assi = plt.subplots(2, 2, figsize=(14, 9))
        for asse, g in zip(assi.ravel(), GRANDEZZE):
            disegna(asse, indici, chiavi, g)
        assi[0][0].legend(fontsize=9)
        uscita = os.path.join(CARTELLA, "confronto_quattro_modelli.png")
    else:
        fig, asse = plt.subplots(figsize=(12, 6))
        disegna(asse, indici, chiavi, grandezza)
        asse.legend(fontsize=10)
        uscita = os.path.join(CARTELLA, f"confronto_quattro_modelli_{grandezza}.png")

    fig.tight_layout()
    fig.savefig(uscita, dpi=140)
    print("scritto", uscita)


if __name__ == "__main__":
    principale()

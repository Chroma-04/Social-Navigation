"""Robustezza dell'esito nullo sul canale velocita': la stessa conclusione sotto criteri diversi.

Un risultato nullo puo' essere un artefatto del trattamento dei dati: una coda pesante, un livello
di densita' anomalo, la media al posto della mediana. Questo banco rifa' la stessa stima sulla
distanza dalle persone sotto undici criteri di aggregazione differenti e riporta l'escursione.
Se la conclusione cambia da un criterio all'altro, non e' una conclusione.

Nessuna simulazione: legge il CSV della campagna a 82 coppie.

    py ai_controllo/robustezza_v2.py
"""
import os
import csv
import statistics as st

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILE_CSV = os.path.join(_RADICE, "risultati_test", "ai_velocita_v2_82coppie.csv")
GRANDEZZA = "distanza_media_m"
LIVELLI_BASSI = ("pop42", "pop84")   # le scene rarefatte, dove la distanza misura il vuoto


def _stima(differenze, mediana=False):
    n = len(differenze)
    centro = st.median(differenze) if mediana else st.mean(differenze)
    errore = st.stdev(differenze) / n ** 0.5
    return centro * 100, abs(centro / errore) if errore else float("inf"), n


def _winsorizza(valori, coda):
    """Riporta le code entro i percentili scelti: limita l'influenza dei singoli estremi."""
    ordinati = sorted(valori)
    k = int(len(ordinati) * coda)
    if k == 0:
        return list(valori)
    basso, alto = ordinati[k], ordinati[-k - 1]
    return [min(max(v, basso), alto) for v in valori]


def principale():
    righe = list(csv.DictReader(open(FILE_CSV, newline="", encoding="utf-8")))
    # la chiave e' (livello, seme): lo stesso seme ricorre su livelli diversi
    chiave = lambda r: (r["livello"], r["seme"])
    diff = {chiave(r): float(r[GRANDEZZA + "_v2"]) - float(r[GRANDEZZA + "_kalman"]) for r in righe}
    base = {chiave(r): float(r[GRANDEZZA + "_kalman"]) for r in righe}
    livello = {chiave(r): r["livello"] for r in righe}
    tutte = list(diff.values())

    criteri = [
        ("tutte le coppie, media", tutte, False),
        ("mediana in luogo della media", tutte, True),
        ("esclusi i due livelli piu' radi", [d for s, d in diff.items() if livello[s] not in LIVELLI_BASSI], False),
        ("escluso il livello piu' rado", [d for s, d in diff.items() if livello[s] != LIVELLI_BASSI[0]], False),
        ("winsorizzazione al 10%", _winsorizza(tutte, 0.10), False),
        ("winsorizzazione al 20%", _winsorizza(tutte, 0.20), False),
        ("solo scene con distanza < 2,0 m", [d for s, d in diff.items() if base[s] < 2.0], False),
        ("solo scene con distanza < 1,5 m", [d for s, d in diff.items() if base[s] < 1.5], False),
        ("solo scene con distanza < 1,2 m", [d for s, d in diff.items() if base[s] < 1.2], False),
        ("escluso il 5% di code (troncamento)", sorted(tutte)[4:-4], False),
        ("mediana sulle sole scene dense", [d for s, d in diff.items() if base[s] < 2.0], True),
    ]

    print("=" * 84)
    print(f"ROBUSTEZZA DELL'ESITO NULLO --- distanza dalle persone, rete v2, {len(tutte)} coppie")
    print("=" * 84)
    print("  segno + = la rete guadagna spazio\n")
    print(f"  {'criterio':<38}{'coppie':>8}{'stima':>10}{'rapporto':>11}")
    stime, rapporti = [], []
    for nome, valori, mediana in criteri:
        centro, rapporto, n = _stima(valori, mediana)
        stime.append(centro)
        rapporti.append(rapporto)
        print(f"  {nome:<38}{n:>8}{centro:>+9.1f}cm{rapporto:>11.2f}")
    print(f"\n  ESCURSIONE fra i {len(criteri)} criteri: da {min(stime):+.1f} a {max(stime):+.1f} cm")
    print(f"  rapporto massimo osservato: {max(rapporti):.2f}")
    concordi = sum(1 for s in stime if s > 0)
    print(f"  criteri con segno positivo: {concordi}/{len(stime)}")


if __name__ == "__main__":
    principale()

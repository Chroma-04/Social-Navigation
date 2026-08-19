"""Aggrega le tre campagne AI-contro-Kalman e produce la tabella finale.

Le tre campagne usano semi disgiunti fra loro e la stessa condizione (velocita_libera su povo),
quindi le coppie si sommano: sono repliche indipendenti dello stesso confronto.

  ai_velocita_povo.csv            prima campagna, 24 coppie   distanza +12,3 cm rapporto 2,30
  ai_velocita_replica_povo.csv    replica su semi disgiunti, 10 coppie   -8,5 cm rapporto 0,90
  ai_velocita_estensione_povo.csv estensione, 48 coppie

Il criterio e' quello usato in tutta la tesi: rapporto fra media della differenza appaiata e il suo
errore standard. Sotto 1 la grandezza e' indistinguibile da zero, sopra 2 e' stabilita. In piu' il
test dei segni sui livelli di densita', che sono blocchi indipendenti.

    py ai_controllo/aggrega_ai.py
"""
import os
import csv
import math
import statistics
from collections import defaultdict

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CARTELLA = os.path.join(_RADICE, "risultati_test")

CAMPAGNE = [
    ("prima", "ai_velocita_povo.csv"),
    ("replica", "ai_velocita_replica_povo.csv"),
    ("estensione", "ai_velocita_estensione_povo.csv"),
]

# nome interno -> (etichetta, unita', quante cifre, segno positivo = AI meglio)
GRANDEZZE = [
    ("tempo_s", "tempo di percorrenza", "s", 2, -1),
    ("fermo_s", "tempo trascorso fermo", "s", 2, -1),
    ("numero_fermate", "numero di fermate", "", 2, -1),
    ("distanza_media_m", "distanza dalle persone", "m", 3, +1),
    ("velocita_media_m_s", "velocita' media", "m/s", 3, +1),
]

AREA_M2 = 1997.0          # superficie calpestabile della mappa povo, contata sulle celle libere
# la replica arriva a due livelli piu alti degli altri: la densita si legge dall etichetta
def _popolazione(livello):
    return int(livello[3:])


def _rapporto(valori):
    """Media, errore standard e loro rapporto. Il rapporto e' il criterio di decisione."""
    if len(valori) < 2:
        return (valori[0] if valori else 0.0), float("nan"), float("nan")
    media = statistics.mean(valori)
    errore = statistics.stdev(valori) / math.sqrt(len(valori))
    return media, errore, (abs(media) / errore if errore > 0 else float("inf"))


def _giudizio(r):
    if math.isnan(r):
        return "?"
    if r >= 2.0:
        return "STABILITO"
    if r >= 1.3:
        return "tendenza"
    return "NULLO"


def _segni(p):
    """Test dei segni a due code su n livelli indipendenti, con p=0,5 sotto l'ipotesi nulla."""
    n, k = len(p), sum(1 for v in p if v > 0)
    estremo = max(k, n - k)
    coda = sum(math.comb(n, i) for i in range(estremo, n + 1)) / 2 ** n
    return k, n, min(1.0, 2 * coda)


def carica():
    """Le differenze AI meno Kalman, per campagna e per livello."""
    per_campagna = defaultdict(lambda: defaultdict(list))   # campagna -> grandezza -> [differenze]
    per_livello = defaultdict(lambda: defaultdict(list))    # livello  -> grandezza -> [differenze]
    assoluti = defaultdict(list)                            # grandezza+braccio -> valori
    mancanti = []
    for nome, file_csv in CAMPAGNE:
        percorso = os.path.join(CARTELLA, file_csv)
        if not os.path.exists(percorso):
            mancanti.append(file_csv)
            continue
        with open(percorso, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                for chiave, _, _, _, _ in GRANDEZZE:
                    k = float(r[chiave + "_kalman"])
                    a = float(r[chiave + "_ai"])
                    per_campagna[nome][chiave].append(a - k)
                    per_livello[r["livello"]][chiave].append(a - k)
                    assoluti[chiave + "_kalman"].append(k)
                    assoluti[chiave + "_ai"].append(a)
    return per_campagna, per_livello, assoluti, mancanti


def principale():
    per_campagna, per_livello, assoluti, mancanti = carica()
    if mancanti:
        print("MANCANO:", ", ".join(mancanti))
    if not per_campagna:
        return

    print("=" * 78)
    print("AI DENTRO IL CONTROLLO DI VELOCITA', CONTRO IL KALMAN PURO")
    print("segno + = l'AI fa meglio; su tempo, fermo e fermate il verso e' gia' invertito")
    print("=" * 78)

    print("\nPER CAMPAGNA, solo la distanza dalle persone")
    print(f"  {'campagna':<12} {'coppie':>7} {'media':>9} {'errore':>8} {'rapporto':>9}")
    for nome, _ in CAMPAGNE:
        d = per_campagna.get(nome, {}).get("distanza_media_m")
        if not d:
            continue
        m, e, r = _rapporto(d)
        print(f"  {nome:<12} {len(d):>7} {m:>+9.3f} {e:>8.3f} {r:>9.2f}")

    print("\nAGGREGATO SU TUTTE LE COPPIE")
    totale = defaultdict(list)
    for nome, _ in CAMPAGNE:
        for chiave, valori in per_campagna.get(nome, {}).items():
            totale[chiave].extend(valori)
    n_coppie = len(totale["tempo_s"])
    print(f"  {n_coppie} coppie appaiate, 8 livelli di densita'\n")
    print(f"  {'grandezza':<24} {'differenza':>12} {'errore':>8} {'rapporto':>9}  giudizio")
    for chiave, etichetta, unita, cifre, verso in GRANDEZZE:
        m, e, r = _rapporto(totale[chiave])
        m *= verso
        testo = f"{m:+.{cifre}f} {unita}".strip()
        print(f"  {etichetta:<24} {testo:>12} {e:>8.{cifre}f} {r:>9.2f}  {_giudizio(r)}")

    print("\n  test dei segni sui livelli (blocchi indipendenti)")
    for chiave, etichetta, _, _, verso in GRANDEZZE:
        medie = [verso * statistics.mean(per_livello[l][chiave]) for l in sorted(per_livello)]
        k, n, p = _segni(medie)
        print(f"    {etichetta:<24} {k}/{n} livelli favorevoli   p={p:.4f}")

    print("\nPER DENSITA' (persone per 100 m2): differenza di distanza, AI meno Kalman")
    print(f"  {'livello':>8} {'dens.':>7} {'coppie':>7} {'d distanza':>11} {'d tempo':>9}")
    for livello in sorted(per_livello, key=_popolazione):
        d = per_livello[livello]["distanza_media_m"]
        t = per_livello[livello]["tempo_s"]
        dens = _popolazione(livello) / AREA_M2 * 100
        print(f"  {livello:>8} {dens:>7.1f} {len(d):>7} {statistics.mean(d):>+11.3f} "
              f"{statistics.mean(t):>+9.1f}")

    print("\nVALORI ASSOLUTI MEDI (sulle stesse coppie)")
    for chiave, etichetta, unita, cifre, _ in GRANDEZZE:
        k = statistics.mean(assoluti[chiave + "_kalman"])
        a = statistics.mean(assoluti[chiave + "_ai"])
        print(f"  {etichetta:<24} kalman {k:>9.{cifre}f}   ai {a:>9.{cifre}f}  {unita}")


if __name__ == "__main__":
    principale()

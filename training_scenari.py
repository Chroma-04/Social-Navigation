"""Generazione di scenari per il training/valutazione del modulo AI di previsione (Livello 2 della tesi):
combina una "mappa" (struttura fissa dell'ambiente: muri) con un "livello di difficolta'" (densita' di
persone + mix comportamentale, che insieme determinano quanto il moto e' prevedibile) per costruire un
ambiente di simulazione pronto, riusando le funzioni di spawn/griglia gia' presenti in main.py.

Le mappe sono deterministiche (nessuna dipendenza dal seed dello scenario): lo stesso nome di mappa da'
sempre la stessa struttura di muri, cosi' la variabilita' fra run diverse viene solo da persone/gruppi
(seed passato a costruisci_scenario), non da un layout che cambia sotto i piedi.
"""
import collections
import os
import random
import main as sim

def _sigilla_tasche_isolate(griglia):
    """Ostacoli generati casualmente possono per caso richiudere piccole tasche di celle libere
    (raggiungibili da nessun'altra parte): un'entita' che ci nascesse dentro resterebbe bloccata per
    sempre (l'A* non trova mai un percorso verso il suo obiettivo). Trova la componente connessa piu'
    grande e trasforma in muro qualunque cella libera che ne resti fuori."""
    celle_libere = [(n.r, n.c) for riga in griglia for n in riga if n.tipo == "libero"]
    if not celle_libere:
        return
    non_visitate = set(celle_libere)
    componente_piu_grande = set()
    while non_visitate:
        partenza = next(iter(non_visitate))
        visitate = {partenza}
        coda = collections.deque([partenza])
        while coda:
            r, c = coda.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                vicino = (r + dr, c + dc)
                if vicino in non_visitate and vicino not in visitate:
                    visitate.add(vicino)
                    coda.append(vicino)
        non_visitate -= visitate
        if len(visitate) > len(componente_piu_grande):
            componente_piu_grande = visitate
    for r, c in celle_libere:
        if (r, c) not in componente_piu_grande:
            griglia[r][c].tipo = "muro"

# --- MAPPE: strutture d'ambiente a complessita' strutturale crescente ---

def mappa_aperta(griglia):
    """Arena vuota (solo i bordi, gia' presenti): nessun vincolo strutturale, riferimento a complessita' minima."""
    pass

def mappa_ostacoli_sparsi(griglia):
    """Arena aperta con colonne/ostacoli rettangolari sparsi: piu' percorsi alternativi possibili, nessun
    collo di bottiglia netto. Layout con seed fisso proprio, indipendente dal seed dello scenario."""
    rng = random.Random(12345)
    for _ in range(14):
        r0 = rng.randint(4, sim.Y_TOT - 6)
        c0 = rng.randint(4, sim.X_TOT - 6)
        h, w = rng.randint(2, 4), rng.randint(2, 4)
        for r in range(r0, min(r0 + h, sim.Y_TOT - 1)):
            for c in range(c0, min(c0 + w, sim.X_TOT - 1)):
                griglia[r][c].tipo = "muro"

def mappa_ostacoli_incrociati(griglia):
    """Arena aperta con ostacoli a croce (bracci orizzontale+verticale di lunghezza variabile): forme piu'
    articolate di un semplice blocco, con angoli concavi che offrono piu' nascondigli/svolte improvvise
    lungo il percorso. Seed fisso proprio."""
    rng = random.Random(777)
    for _ in range(22):
        rc = rng.randint(6, sim.Y_TOT - 7)
        cc = rng.randint(6, sim.X_TOT - 7)
        braccio_v = rng.randint(3, 6)
        braccio_h = rng.randint(3, 6)
        spessore = rng.randint(1, 2)
        for r in range(rc - braccio_v, rc + braccio_v + 1):
            for c in range(cc - spessore, cc + spessore + 1):
                if 1 <= r < sim.Y_TOT - 1 and 1 <= c < sim.X_TOT - 1:
                    griglia[r][c].tipo = "muro"
        for c in range(cc - braccio_h, cc + braccio_h + 1):
            for r in range(rc - spessore, rc + spessore + 1):
                if 1 <= r < sim.Y_TOT - 1 and 1 <= c < sim.X_TOT - 1:
                    griglia[r][c].tipo = "muro"
    _sigilla_tasche_isolate(griglia)

def mappa_ostacoli_organici(griglia):
    """Arena aperta con macchie irregolari (random walk a partire da un centro): forme grossolane e
    asimmetriche, ne' rettangoli ne' cerchi ne' croci - il tipo di ostacolo meno prevedibile fra quelli
    disponibili. Seed fisso proprio."""
    rng = random.Random(31415)
    for _ in range(13):
        r, c = rng.randint(6, sim.Y_TOT - 7), rng.randint(6, sim.X_TOT - 7)
        for _ in range(45):
            if 1 <= r < sim.Y_TOT - 1 and 1 <= c < sim.X_TOT - 1:
                griglia[r][c].tipo = "muro"
                if 1 <= r + 1 < sim.Y_TOT - 1:
                    griglia[r + 1][c].tipo = "muro"  # ispessisce un po' il tratto, altrimenti resta un filo di 1 cella
                if 1 <= c + 1 < sim.X_TOT - 1:
                    griglia[r][c + 1].tipo = "muro"
            dr, dc = rng.choice(((-1, 0), (1, 0), (0, -1), (0, 1)))
            r = max(2, min(sim.Y_TOT - 3, r + dr))
            c = max(2, min(sim.X_TOT - 3, c + dc))
    _sigilla_tasche_isolate(griglia)

def mappa_corridoio(griglia):
    """Un unico corridoio orizzontale che attraversa tutta la larghezza della mappa: tutte le persone sono
    forzate sulla stessa fascia stretta, alta densita' locale anche con pochi individui totali."""
    centro = sim.Y_TOT // 2
    meta_altezza = 5
    righe_corridoio = range(centro - meta_altezza, centro + meta_altezza)
    for r in range(1, sim.Y_TOT - 1):
        if r not in righe_corridoio:
            for c in range(1, sim.X_TOT - 1):
                griglia[r][c].tipo = "muro"

def mappa_porta_stretta(griglia):
    """Due stanze divise da un muro verticale con un solo varco: collo di bottiglia netto, massima densita'
    locale concentrata in un punto."""
    colonna = sim.X_TOT // 2
    riga_porta = sim.Y_TOT // 2
    for r in range(1, sim.Y_TOT - 1):
        if r != riga_porta:
            griglia[r][colonna].tipo = "muro"

def mappa_doppia_porta(griglia):
    """Due stanze divise da un muro verticale con due varchi (uno vicino, uno lontano): introduce una scelta
    di percorso reale, utile anche per osservare l'effetto della macchia di probabilita' sulla decisione."""
    colonna = sim.X_TOT // 2
    riga_vicina = sim.Y_TOT // 2 - 12
    riga_lontana = sim.Y_TOT // 2 + 12
    for r in range(1, sim.Y_TOT - 1):
        if r not in (riga_vicina, riga_lontana):
            griglia[r][colonna].tipo = "muro"

def mappa_incrocio(griglia):
    """Croce di corridoi che si incontrano al centro: piu' punti di decisione direzionale (4 direzioni)
    invece di un semplice passaggio, il tipo di complessita' strutturale piu' alto fra le mappe disponibili."""
    centro_r, centro_c = sim.Y_TOT // 2, sim.X_TOT // 2
    meta_larghezza = 4
    for r in range(1, sim.Y_TOT - 1):
        for c in range(1, sim.X_TOT - 1):
            in_orizzontale = abs(r - centro_r) <= meta_larghezza
            in_verticale = abs(c - centro_c) <= meta_larghezza
            if not (in_orizzontale or in_verticale):
                griglia[r][c].tipo = "muro"

MAPPE = {
    "aperta": mappa_aperta,
    "ostacoli_sparsi": mappa_ostacoli_sparsi,
    "ostacoli_incrociati": mappa_ostacoli_incrociati,
    "ostacoli_organici": mappa_ostacoli_organici,
    "corridoio": mappa_corridoio,
    "porta_stretta": mappa_porta_stretta,
    "doppia_porta": mappa_doppia_porta,
    "incrocio": mappa_incrocio,
}


def _costruttore_da_file(path):
    """Adatta una mappa salvata su disco (muri.json, incluso il bordo) all'interfaccia builder(griglia)."""
    def builder(griglia):
        sim.carica_mappa(griglia, path)
    return builder


def _carica_mappe_training():
    """Scandisce mappe_training/ (mappe disegnate a mano nel simulatore col tasto T) e le rende disponibili
    come mappe aggiuntive, col prefisso 'training_' per distinguerle da quelle procedurali qui sopra. Se la
    cartella non esiste ancora (nessuna mappa salvata finora), non trova nulla e non fa niente."""
    mappe_extra = {}
    if os.path.isdir(sim.CARTELLA_MAPPE_TRAINING):
        for nome_file in sorted(os.listdir(sim.CARTELLA_MAPPE_TRAINING)):
            if nome_file.endswith(".json"):
                nome_mappa = "training_" + os.path.splitext(nome_file)[0]
                mappe_extra[nome_mappa] = _costruttore_da_file(os.path.join(sim.CARTELLA_MAPPE_TRAINING, nome_file))
    return mappe_extra


MAPPE.update(_carica_mappe_training())

# --- LIVELLI DI DIFFICOLTA': densita' (scala) x mix comportamentale (prevedibilita') ---
# le mappe disegnate a mano hanno aree libere molto diverse fra loro (una piazza aperta contro una stanza
# piccola e articolata): usare un conteggio assoluto di persone renderebbe "denso" tutt'altro che
# comparabile da una mappa all'altra (40 persone in una piazza enorme sono niente, nella stessa cifra in una
# stanza piccola blocca tutto). I budget qui sotto sono percio' tarati su CELLE_LIBERE_RIFERIMENTO (l'area
# libera della mappa "aperta", la piu' grande possibile su questa griglia) e vengono scalati in
# costruisci_scenario in proporzione all'area libera reale della mappa richiesta, cosi' un dato livello di
# densita' rappresenta sempre la stessa pressione di folla (persone per cella libera), non lo stesso
# numero assoluto. "budget_individui" e' il totale approssimativo di persone/corridori/persone ferme (i
# gruppi si sommano a parte, essendo entita' composte da piu' membri). I valori di "medio" sono tarati per
# arrivare a circa 25 persone totali sulla mappa di riferimento, in linea con lo scenario discusso
# (corridoio, media 25).
CELLE_LIBERE_RIFERIMENTO = 4524  # area libera di "aperta" (misurata: solo i bordi sono muro)
LIVELLI_DENSITA = {
    "rado": {"budget_individui": 7, "numero_gruppi_base": 0},
    "medio": {"budget_individui": 22, "numero_gruppi_base": 1},
    "denso": {"budget_individui": 40, "numero_gruppi_base": 3},
}

# le proporzioni sono relative fra loro (vengono normalizzate), non devono sommare a 1. "moltiplicatore_gruppi"
# scala numero_gruppi_base: i gruppi (membri che si rincorrono/accelerano) e l'alta densita' sono i principali
# responsabili di moto non lineare (imprevedibile per un Kalman a velocita' costante), piu' dei semplici
# corridori (che vanno piu' veloci ma in modo altrettanto rettilineo se non devono schivare nessuno).
MIX_COMPORTAMENTALE = {
    "regolare": {"prop_normali": 0.85, "prop_corridori": 0.05, "prop_ferme": 0.10, "moltiplicatore_gruppi": 0.5},
    "misto": {"prop_normali": 0.60, "prop_corridori": 0.20, "prop_ferme": 0.10, "moltiplicatore_gruppi": 1.0},
    "caotico": {"prop_normali": 0.40, "prop_corridori": 0.35, "prop_ferme": 0.05, "moltiplicatore_gruppi": 2.0},
}


def costruisci_scenario(nome_mappa, nome_densita, nome_mix, seed=None):
    """Costruisce griglia + persone/corridori/persone_ferme/gruppi per la combinazione richiesta. Con lo
    stesso seed la composizione (chi/quanti) e' riproducibile; senza seed usa lo stato casuale corrente."""
    if seed is not None:
        random.seed(seed)

    griglia = [[sim.Nodo(r, c) for c in range(sim.X_TOT)] for r in range(sim.Y_TOT)]
    sim.crea_bordi(griglia)
    MAPPE[nome_mappa](griglia)

    celle_libere = sum(1 for riga in griglia for n in riga if n.tipo == "libero")
    fattore_area = celle_libere / CELLE_LIBERE_RIFERIMENTO

    densita = LIVELLI_DENSITA[nome_densita]
    mix = MIX_COMPORTAMENTALE[nome_mix]
    budget = densita["budget_individui"] * fattore_area
    totale_prop = mix["prop_normali"] + mix["prop_corridori"] + mix["prop_ferme"]
    numero_persone = max(0, round(budget * mix["prop_normali"] / totale_prop))
    numero_corridori = max(0, round(budget * mix["prop_corridori"] / totale_prop))
    numero_persone_ferme = max(0, round(budget * mix["prop_ferme"] / totale_prop))
    numero_gruppi = max(0, round(densita["numero_gruppi_base"] * mix["moltiplicatore_gruppi"] * fattore_area))

    persone, corridori, persone_ferme, gruppi = [], [], [], []
    sim.sincronizza_persone(persone, numero_persone, griglia, fabbrica=sim.crea_persona)
    sim.sincronizza_persone(corridori, numero_corridori, griglia, fabbrica=sim.crea_corridore)
    sim.sincronizza_persone_ferme(persone_ferme, numero_persone_ferme, griglia)
    sim.sincronizza_gruppi(gruppi, numero_gruppi, griglia)

    return {
        "griglia": griglia,
        "persone": persone,
        "corridori": corridori,
        "persone_ferme": persone_ferme,
        "gruppi": gruppi,
        "nome_mappa": nome_mappa,
        "nome_densita": nome_densita,
        "nome_mix": nome_mix,
    }


def tutte_le_combinazioni():
    """Elenca tutte le combinazioni (mappa, densita', mix) disponibili, nell'ordine in cui vale la pena
    generarle (dalla piu' semplice alla piu' complessa)."""
    for nome_mappa in MAPPE:
        for nome_densita in LIVELLI_DENSITA:
            for nome_mix in MIX_COMPORTAMENTALE:
                yield nome_mappa, nome_densita, nome_mix


if __name__ == "__main__":
    combinazioni = list(tutte_le_combinazioni())
    print(f"Mappe: {len(MAPPE)}, livelli densita': {len(LIVELLI_DENSITA)}, mix comportamentali: {len(MIX_COMPORTAMENTALE)}")
    print(f"Combinazioni totali: {len(combinazioni)}\n")
    for nome_mappa, nome_densita, nome_mix in combinazioni:
        scenario = costruisci_scenario(nome_mappa, nome_densita, nome_mix, seed=42)
        totale_gruppi = sum(len(g["membri"]) for g in scenario["gruppi"])
        totale = len(scenario["persone"]) + len(scenario["corridori"]) + len(scenario["persone_ferme"]) + totale_gruppi
        print(
            f"{nome_mappa:16} {nome_densita:6} {nome_mix:9} -> "
            f"persone={len(scenario['persone']):3} corridori={len(scenario['corridori']):3} "
            f"ferme={len(scenario['persone_ferme']):3} gruppi={len(scenario['gruppi']):2} "
            f"(membri={totale_gruppi:3}) totale~{totale:3}"
        )

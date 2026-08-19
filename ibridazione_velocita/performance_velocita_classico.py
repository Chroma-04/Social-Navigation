"""Caratterizzazione del controllo di velocita' CLASSICO, prima di ibridarlo.

E' la linea di base della serie: misura cosa fa oggi il componente descritto in tesi al
§3.2.3, su tutto l'intervallo di densita' invece che a una sola. Serve a stabilire dove
il controllo aiuta, dove danneggia, e quali grandezze reagiscono - perche' ibridare un
componente senza sapere cosa fa da solo produce un numero che non si sa a cosa attribuire.

COSA SI SAPEVA GIA'. Alla sola densita' 0,70 pers/m2 (vecchia scala) il controllo costa il
24,6% di tempo di percorrenza con rapporto 10,4, perdendo 47 ripetizioni su 50, e NON riduce
il tempo trascorso in arresto: 8,39 -> 8,58 s, rapporto 0,30. La causa e' un'asimmetria fra i
due regimi attivi - frenata nel 39,2% dei fotogrammi, accelerazione nel 3,8% - perche' lo
sprint richiede che il conflitto previsto scompaia interamente, condizione quasi mai
soddisfatta. Il componente si comporta di fatto come un freno.

COSA AGGIUNGE QUESTO BANCO. Quella misura sta a una densita' sola e guarda due grandezze.
Qui se ne guardano sei su dieci livelli, e tre non erano mai state rilevate:

  - NUMERO DI FERMATE, non solo il tempo complessivo passato fermi. Sono due cose diverse:
    dieci arresti brevi e uno lungo danno lo stesso tempo ma un'esperienza opposta per chi
    condivide il corridoio. E' inoltre la grandezza che il componente dichiara di voler
    ridurre, quindi la sua verifica diretta;
  - VELOCITA' MEDIA effettiva lungo il tragitto, che separa "ci mette di piu' perche' va
    piano" da "ci mette di piu' perche' fa piu' strada";
  - FREQUENZA DEI REGIMI: quanto spesso frena, quanto spesso accelera, e con che fattore
    medio. E' l'indicatore di esitazione, e a occhio e' la grandezza su cui l'ibridazione
    dovrebbe incidere di piu': il controllo frena sulla base di previsioni che sbagliano
    di circa un metro contro una soglia di conflitto di 62 cm, quindi molte frenate sono
    falsi allarmi che una previsione migliore potrebbe evitare.

Restano poi tempo di percorrenza, tempo fermo e distanza media dai corpi in movimento, per
continuita' con le campagne precedenti.

DISEGNO. Confronto appaiato a semi comuni fra `base` (nessun controllo) e `solo_velocita`,
su dieci livelli di popolazione ottenuti riscalando la stessa composizione - lo stesso
metodo con cui in tesi sono stati ricavati i livelli 0,51 e 0,64. Cinque semi per livello:
e' uno screening pensato per vedere dove l'effetto cambia segno o intensita' lungo la
densita', non per stabilire un singolo numero. Dove emergesse qualcosa, quel livello va
ripetuto con cinquanta semi prima di scriverlo.

Le persone ferme sono escluse dalle distanze, come in TEST/test_sicurezza_povo.py: il robot
le rasenta per costruzione e non sono cio' che la metrica vuole catturare.

Uso:
    py ibridazione_velocita/performance_velocita_classico.py           campagna completa
    py ibridazione_velocita/performance_velocita_classico.py --corto   2 semi x 3 livelli
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import csv
import time
import math
import zlib
import statistics
import multiprocessing as mp

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import main as sim

MAPPA = "povo"
CONDIZIONI = ("base", "solo_velocita")
ETICHETTA_SEMI = "ibridazione_velocita"  # semi propri: questa serie non si appaia a quelle vecchie
NUMERO_SEMI = 5

# Composizione derivata dalla serie d070 (~349 individui), riscalata per ottenere i livelli. I fattori
# coprono da un quinto a una volta e mezza quella popolazione: sotto restano troppe poche
# persone perche' un conflitto si presenti, sopra l'ambiente si avvicina alla saturazione.
# Composizione rivista il 18 agosto 2026: i corridori scendono dal 48% al 13% dei corpi mobili.
# Meta' folla che corre non descrive un corridoio universitario, e rendeva l'ambiente piu' veloce
# del robot. Il TOTALE resta 281, quindi le etichette pop* e le densita' non cambiano: cambia solo
# il mescolamento, e i risultati precedenti restano confrontabili in numerosita' ma non in cinematica.
POPOLAZIONE_BASE = {
    "numero_persone": 120,
    "numero_corridori": 18,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}
FATTORI = [0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20, 1.35, 1.50]

# scala fisica presa da main.py invece che riscritta: se cambia li', cambia anche qui
PIXEL_PER_METRO = 100.0 / sim.CM_PER_PIXEL

FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "velocita_classico_povo_sweep.csv")


def _popolazione_scalata(fattore):
    """Stessa composizione, numerosita' scalata. I gruppi sono entita' composte (ciascuno porta
    piu' membri), quindi vanno scalati anche loro e non tenuti fissi."""
    scalata = {chiave: max(0, round(valore * fattore)) for chiave, valore in POPOLAZIONE_BASE.items()}
    scalata.update(t._geometria_salvata())
    return scalata


def _una_corsa(argomenti):
    """Una corsa strumentata. Le due intercettazioni vanno installate QUI e non nel processo
    padre: con lo spawn di Windows il worker re-importa i moduli da zero e non vedrebbe le
    modifiche fatte altrove."""
    condizione, seed, popolazione, etichetta_livello = argomenti

    traiettoria = []   # posizione del robot a ogni frame, px
    distanze = []      # distanza dal corpo mobile piu' vicino, px
    fattori = []       # fattore di velocita' applicato, quando il controllo e' attivo

    traccia_originale = t._rileva_e_traccia

    def _con_misure(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar, storico_posizioni):
        minimo = None
        for p in tutte_le_persone:
            if p.get("stato", "attesa") == "attesa":
                continue  # persone ferme escluse: vedi nota in testa al file
            d = math.hypot(p["x"] - robot_x, p["y"] - robot_y)
            if minimo is None or d < minimo:
                minimo = d
        if minimo is not None:
            distanze.append(minimo)
        traiettoria.append((robot_x, robot_y))
        return traccia_originale(tutte_le_persone, robot_x, robot_y, griglia,
                                 tracciamento_lidar, storico_posizioni)

    velocita_originale = sim.fattore_velocita_da_conflitto

    def _con_conteggio(*args, **kwargs):
        fattore = velocita_originale(*args, **kwargs)
        fattori.append(fattore)
        return fattore

    t._rileva_e_traccia = _con_misure
    sim.fattore_velocita_da_conflitto = _con_conteggio
    try:
        arrivato, frame, frame_fermo = t.esegui_corsa(MAPPA, condizione, popolazione, seed)
    finally:
        t._rileva_e_traccia = traccia_originale
        sim.fattore_velocita_da_conflitto = velocita_originale

    tempo_s = t._secondi(frame)
    misure = _metriche(traiettoria, distanze, fattori, tempo_s)
    return etichetta_livello, condizione, seed, arrivato, tempo_s, t._secondi(frame_fermo), misure


def _metriche(traiettoria, distanze, fattori, tempo_s):
    """Dalla traiettoria grezza in pixel alle sei grandezze riportate."""
    vuoto = {"numero_fermate": 0, "distanza_media_m": 0.0, "velocita_media_m_s": 0.0,
             "fattore_medio": 1.0, "frenata_percento": 0.0, "sprint_percento": 0.0}
    if len(traiettoria) < 2:
        return vuoto

    # Un frame e' "fermo" se la posizione non e' cambiata: sia l'arresto di sicurezza sia un
    # percorso momentaneamente non percorribile lasciano il robot dov'era. Le FERMATE sono i
    # blocchi contigui di frame fermi, cioe' gli eventi, non i singoli frame.
    percorso_px = 0.0
    fermate = 0
    fermo_prima = False
    for (x0, y0), (x1, y1) in zip(traiettoria, traiettoria[1:]):
        passo = math.hypot(x1 - x0, y1 - y0)
        percorso_px += passo
        fermo_ora = passo < 1e-6
        if fermo_ora and not fermo_prima:
            fermate += 1  # transizione da moto a fermo: una fermata nuova
        fermo_prima = fermo_ora

    # velocita' media effettiva: strada percorsa / tempo impiegato. Distingue "va piano" da
    # "fa piu' strada", che sul solo tempo di percorrenza sono indistinguibili
    velocita = (percorso_px / PIXEL_PER_METRO / tempo_s) if tempo_s > 0 else 0.0

    # i fattori si raccolgono solo quando il controllo e' acceso; in condizione `base` la
    # funzione non viene mai chiamata e la lista resta vuota
    if fattori:
        fattore_medio = statistics.mean(fattori)
        frenata = sum(1 for f in fattori if f < 0.999) / len(fattori) * 100
        sprint = sum(1 for f in fattori if f > 1.001) / len(fattori) * 100
    else:
        fattore_medio, frenata, sprint = 1.0, 0.0, 0.0

    return {
        "numero_fermate": fermate,
        "distanza_media_m": statistics.mean(distanze) / PIXEL_PER_METRO if distanze else 0.0,
        "velocita_media_m_s": velocita,
        "fattore_medio": fattore_medio,
        "frenata_percento": frenata,
        "sprint_percento": sprint,
    }


METRICHE = ("numero_fermate", "distanza_media_m", "velocita_media_m_s",
            "fattore_medio", "frenata_percento", "sprint_percento")


def main():
    corto = "--corto" in sys.argv
    fattori = FATTORI[1::3] if corto else FATTORI      # 4 livelli distribuiti sull'intervallo
    numero_semi = 2 if corto else NUMERO_SEMI

    livelli = [(f"pop{round(sum(POPOLAZIONE_BASE.values()) * f)}", _popolazione_scalata(f))
               for f in fattori]
    semi = [zlib.crc32(f"{MAPPA}|{ETICHETTA_SEMI}|{i}".encode()) for i in range(numero_semi)]
    compiti = [(condizione, seme, popolazione, etichetta)
               for etichetta, popolazione in livelli
               for seme in semi
               for condizione in CONDIZIONI]

    n_worker = min(8, os.cpu_count() or 4)
    print(f"Controllo di velocita' classico su {MAPPA}: caratterizzazione lungo la densita'")
    print(f"  {len(livelli)} livelli x {numero_semi} semi x {len(CONDIZIONI)} condizioni = {len(compiti)} corse")
    print(f"  {n_worker} processi, accelerazione {t.ACCELERAZIONE}x, scala {PIXEL_PER_METRO:.0f} px/m")
    if corto:
        print("  MODALITA' CORTA: screening di validazione, non una misura da riportare")
    print()

    esiti = {}
    inizio = time.time()
    completate = 0
    with mp.Pool(n_worker, initializer=t._inizializza_worker) as pool:
        for etichetta, condizione, seed, arrivato, tempo_s, fermo_s, misure in pool.imap_unordered(
                _una_corsa, compiti, chunksize=1):
            completate += 1
            esiti[(etichetta, seed, condizione)] = (arrivato, tempo_s, fermo_s, misure)
            print(f"  [{completate}/{len(compiti)}] {etichetta} / {condizione}: {tempo_s:.1f}s, "
                  f"{misure['numero_fermate']} fermate, {misure['velocita_media_m_s']:.2f} m/s "
                  f"({time.time() - inizio:.0f}s)")

    print(f"\nCompletato in {time.time() - inizio:.1f}s\n")
    _riporta([e for e, _ in livelli], semi, esiti)


def _riporta(etichette_livelli, semi, esiti):
    righe = []
    print("=== Effetto del controllo di velocita', livello per livello ===")
    print("(differenze orientate come effetto del controllo: negativo = il controllo peggiora)\n")
    intestazione = (f"{'livello':>10}{'n':>4}{'tempo base':>12}{'tempo ctrl':>12}{'var. tempo':>12}"
                    f"{'ferm. b':>9}{'ferm. c':>9}{'vel. b':>9}{'vel. c':>9}"
                    f"{'frenata %':>11}{'sprint %':>10}")
    print(intestazione)
    print("-" * len(intestazione))

    for etichetta in etichette_livelli:
        coppie_tempo, coppie = [], {chiave: [] for chiave in METRICHE}
        for seme in semi:
            chiave_b = (etichetta, seme, "base")
            chiave_c = (etichetta, seme, "solo_velocita")
            if chiave_b not in esiti or chiave_c not in esiti:
                continue
            arrivato_b, tempo_b, fermo_b, mis_b = esiti[chiave_b]
            arrivato_c, tempo_c, fermo_c, mis_c = esiti[chiave_c]
            if not (arrivato_b and arrivato_c):
                print(f"  {etichetta} seme {seme}: coppia scartata, un robot non e' arrivato")
                continue
            coppie_tempo.append((tempo_b, tempo_c))
            for chiave in METRICHE:
                coppie[chiave].append((mis_b[chiave], mis_c[chiave]))
            riga = {"mappa": MAPPA, "livello": etichetta, "seme": seme,
                    "tempo_base_s": round(tempo_b, 2), "tempo_controllo_s": round(tempo_c, 2),
                    "fermo_base_s": round(fermo_b, 2), "fermo_controllo_s": round(fermo_c, 2)}
            for chiave in METRICHE:
                riga[f"{chiave}_base"] = round(mis_b[chiave], 4)
                riga[f"{chiave}_controllo"] = round(mis_c[chiave], 4)
            righe.append(riga)

        if not coppie_tempo:
            continue
        n = len(coppie_tempo)
        media_tb = statistics.mean(b for b, c in coppie_tempo)
        media_tc = statistics.mean(c for b, c in coppie_tempo)
        var_tempo = (media_tc - media_tb) / media_tb * 100 if media_tb else 0.0
        medie = {chiave: (statistics.mean(b for b, c in coppie[chiave]),
                          statistics.mean(c for b, c in coppie[chiave])) for chiave in METRICHE}
        print(f"{etichetta:>10}{n:>4}{media_tb:>11.1f}s{media_tc:>11.1f}s{var_tempo:>+11.1f}%"
              f"{medie['numero_fermate'][0]:>9.1f}{medie['numero_fermate'][1]:>9.1f}"
              f"{medie['velocita_media_m_s'][0]:>9.2f}{medie['velocita_media_m_s'][1]:>9.2f}"
              f"{medie['frenata_percento'][1]:>10.1f}%{medie['sprint_percento'][1]:>9.1f}%")

    if not righe:
        print("Nessuna coppia valida.")
        return

    print("\nCinque semi per livello sono uno screening: servono a vedere DOVE guardare, non a")
    print("stabilire un valore. Un livello interessante va ripetuto con cinquanta semi.")

    os.makedirs(t.CARTELLA_RISULTATI, exist_ok=True)
    with open(FILE_CSV, "w", newline="", encoding="utf-8") as f:
        scrittore = csv.DictWriter(f, fieldnames=list(righe[0].keys()))
        scrittore.writeheader()
        scrittore.writerows(righe)
    print(f"\nRighe per singola coppia: {FILE_CSV}")


if __name__ == "__main__":
    main()

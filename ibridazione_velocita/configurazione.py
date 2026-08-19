"""Impronta della configurazione che ha prodotto una misura.

PERCHE' SERVE. I banchi di misura scrivono CSV che si somigliano tutti, ma il simulatore
sotto cambia: il 18 agosto 2026 sono cambiate in poche ore la velocita' dei pedoni (1,2 ->
1,0 m/s), la quota di corridori (48% -> 13%) e la logica dello sprint. I file prodotti prima
e dopo hanno lo stesso formato e descrivono robot diversi, e nulla nel file lo dice.

C'e' un secondo pericolo, piu' silenzioso: t._geometria_salvata() legge config_pannello.json,
cioe' i cursori dell'interfaccia. Basta trascinare il raggio del lidar e una misura ripetuta
domani non riproduce quella di oggi, senza alcun segnale.

COSA FA. Accanto a ogni CSV scrive un <nome>.config.json con tutto cio' che influenza il
risultato: scala, velocita', banda di autorita', limiti dinamici, geometria dei corpi,
parametri dei due controlli, popolazione, livelli, semi, condizioni, riga di comando e commit
git. Con quel file una misura si rifa' identica, o si scopre perche' non torna.

Non e' documentazione: e' la differenza fra un numero verificabile e un numero da credere.
"""
import json
import os
import subprocess
import sys


def _commit_git(radice):
    """Hash del commit corrente, con marcatore se ci sono modifiche non committate: una misura
    prodotta con l'albero sporco non e' riproducibile dal solo hash."""
    try:
        hash_commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=radice,
                                     capture_output=True, text=True, timeout=10).stdout.strip()
        sporco = subprocess.run(["git", "status", "--porcelain"], cwd=radice,
                                capture_output=True, text=True, timeout=10).stdout.strip()
        return f"{hash_commit}+modifiche-non-committate" if sporco else hash_commit
    except Exception:
        return "sconosciuto"


def impronta(sim, testing, popolazione_base=None, fattori=None, semi=None, condizioni=None,
             geometria=None):
    """Raccoglie i parametri che determinano il risultato di una corsa."""
    return {
        "comando": " ".join(sys.argv),
        "commit_git": _commit_git(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "scala": {
            "dim_nodo_px": sim.DIM_NODO,
            "metri_per_cella": sim.METRI_PER_CELLA,
            "cm_per_pixel": round(sim.CM_PER_PIXEL, 4),
        },
        "velocita_m_s": {
            "robot_crociera": sim.VELOCITA_ROBOT * 2,
            "robot_minima": sim.VELOCITA_ROBOT * sim.FATTORE_VELOCITA_ROBOT_MIN * 2,
            "robot_massima": sim.VELOCITA_ROBOT * sim.FATTORE_VELOCITA_ROBOT_MAX * 2,
            "pedone": sim.VELOCITA_PERSONA * 2,
            "corridore": sim.VELOCITA_PERSONA * sim.FATTORE_VELOCITA_CORRIDORE * 2,
        },
        "limiti_dinamici_m_s2": {
            "accelerazione": sim.ACCELERAZIONE_ROBOT_M_S2,
            "decelerazione": sim.DECELERAZIONE_ROBOT_M_S2,
        },
        "geometria_px": {
            "raggio_robot": sim.RAGGIO_ROBOT_DEFAULT,
            "raggio_persona": sim.RAGGIO_PERSONA_DEFAULT,
            "raggio_sicurezza": sim.RAGGIO_SICUREZZA_DEFAULT,
            "raggio_lidar": sim.RAGGIO_LIDAR_DEFAULT,
            # cio' che il pannello puo' aver sovrascritto: e' la sorgente di irriproducibilita'
            # piu' insidiosa, perche' cambia senza toccare il codice
            "da_config_pannello": geometria or {},
        },
        "controllo_velocita": {
            "lookahead_px": sim.DISTANZA_LOOKAHEAD_VELOCITA_PX,
            "margine_conflitto_px": sim.MARGINE_SICUREZZA_CONFLITTO_PX,
            "frame_reazione": sim.FRAME_REAZIONE_VELOCITA_ROBOT,
            "fattore_min": sim.FATTORE_VELOCITA_ROBOT_MIN,
            "fattore_max": sim.FATTORE_VELOCITA_ROBOT_MAX,
        },
        "controllo_posizione": {
            "raggio_manovra_px": sim.RAGGIO_MANOVRA_PX,
            "previsione_frame": sim.PREVISIONE_EVITAMENTO_FRAME,
            "sigma_repulsione_px": sim.SIGMA_REPULSIONE_PX,
            "peso_ritorno_percorso": sim.PESO_RITORNO_PERCORSO,
            "velocita_laterale_max_m_s": sim.VELOCITA_LATERALE_MAX_M_S,
            "raggio_interesse_px": sim.RAGGIO_INTERESSE_EVITAMENTO_PX,
        },
        "simulazione": {
            "accelerazione_passo": testing.ACCELERAZIONE,
            "frame_max": testing.FRAME_MAX,
            "fattore_griglia_robot": testing.FATTORE_GRIGLIA_DEFAULT,
        },
        "esperimento": {
            "popolazione_base": popolazione_base,
            "fattori_densita": fattori,
            "semi": semi,
            "condizioni": condizioni,
        },
    }


def salva_accanto(percorso_csv, dati):
    """Scrive <percorso_csv senza estensione>.config.json. Ritorna il percorso scritto."""
    percorso = os.path.splitext(percorso_csv)[0] + ".config.json"
    with open(percorso, "w", encoding="utf-8") as f:
        json.dump(dati, f, indent=2, ensure_ascii=False)
    return percorso

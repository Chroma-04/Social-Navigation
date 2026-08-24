"""Verifica che la correzione appresa arrivi davvero al controllo, e con quale ampiezza.

Un esito nullo puo' nascere da due cause molto diverse: la rete non serve, oppure la rete non
viene applicata. Questo banco distingue i due casi misurando, su una corsa sola, che cosa
attraversa il punto di innesto:

  - a quali orizzonti il controllo interroga la previsione (la rete e' addestrata a uno fisso);
  - il fattore di riscalamento effettivamente applicato alla correzione;
  - l'ampiezza della correzione prima e dopo il riscalamento, in pixel e in metri;
  - quante interrogazioni ricevono una correzione e quante no.

Se il fattore medio fosse molto piccolo, la campagna avrebbe misurato una frazione della rete e
non la rete: sarebbe un difetto dell'innesto, non un risultato sul metodo.

    py ai_controllo/diagnosi_innesto.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys
import math
import statistics

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_RADICE, os.path.join(_RADICE, "ibridazione_velocita"),
           os.path.join(_RADICE, "TEST"), os.path.join(_RADICE, "ai_predittiva")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import testing as t
import main as sim
import confronto_controlli as cc
import ai_velocita as av

CM_PER_PIXEL = 100.0 / sim.DIM_NODO   # 1 cella = 1 m, quindi un pixel vale 1/DIM_NODO metri


def principale():
    t._inizializza_worker()
    if "--v2" in sys.argv:
        # stessa diagnosi sulla rete nuova: cambia solo il file dei pesi
        import allena_previsione as ap
        t._modello_ai_worker = ap.modello_da_file(
            os.path.join(ap.CARTELLA_MODELLI, "correzione_kalman_specializzato_v2.pt"))
        print("modello: v2 (GRU 48x2)")
    orizzonte = max(1, sim.PREVISIONE_BLOB_FRAME // t.ACCELERAZIONE)
    print(f"orizzonte di addestramento: {orizzonte} frame "
          f"(PREVISIONE_BLOB_FRAME={sim.PREVISIONE_BLOB_FRAME}, ACCELERAZIONE={t.ACCELERAZIONE})")

    ripristina = av.installa(t, sim, t._modello_ai_worker, orizzonte)
    corretta = sim.previsione_per_controllo   # la versione con innesto, gia' installata

    orizzonti, fattori, ampiezze_grezze, ampiezze_applicate = [], [], [], []
    senza_correzione = [0]

    def _spiata(traccia, frame_futuri):
        prima = av.previsione_originale_diagnostica(traccia, frame_futuri) \
            if hasattr(av, "previsione_originale_diagnostica") else None
        dopo = corretta(traccia, frame_futuri)
        c = av._CORREZIONI.get(traccia.get("pid"))
        orizzonti.append(frame_futuri)
        if c is None:
            senza_correzione[0] += 1
        else:
            scala = min(1.0, frame_futuri / orizzonte) if orizzonte else 1.0
            grezza = math.hypot(float(c[0]), float(c[1]))
            fattori.append(scala)
            ampiezze_grezze.append(grezza)
            ampiezze_applicate.append(grezza * scala)
        return dopo

    sim.previsione_per_controllo = _spiata
    try:
        popolazione = cc._popolazione_scalata(cc.FATTORI[4])   # livello centrale
        arrivato, frame, fermo = t.esegui_corsa("povo", "velocita_libera", popolazione, 925789145)
    finally:
        sim.previsione_per_controllo = corretta
        ripristina()

    n = len(orizzonti)
    print(f"\ncorsa conclusa: arrivato={arrivato}, {frame} frame")
    print(f"interrogazioni della previsione: {n}")
    if not n:
        print("NESSUNA interrogazione: il controllo non ha mai consultato la previsione.")
        return
    print(f"  senza correzione disponibile: {senza_correzione[0]} ({100*senza_correzione[0]/n:.1f}%)")
    print(f"\norizzonti richiesti dal controllo (frame)")
    print(f"  minimo {min(orizzonti)}   mediana {statistics.median(orizzonti):.0f}   "
          f"medio {statistics.mean(orizzonti):.1f}   massimo {max(orizzonti)}")
    if not fattori:
        print("\nnessuna correzione applicata: la rete non raggiunge mai il controllo.")
        return
    print(f"\nfattore di riscalamento applicato alla correzione")
    print(f"  medio {statistics.mean(fattori):.3f}   mediano {statistics.median(fattori):.3f}   "
          f"al massimo (1,0) nel {100*sum(1 for f in fattori if f >= 1.0)/len(fattori):.1f}% dei casi")
    print(f"\nampiezza della correzione")
    print(f"  grezza    media {statistics.mean(ampiezze_grezze):6.2f} px = "
          f"{statistics.mean(ampiezze_grezze)*CM_PER_PIXEL:5.1f} cm")
    print(f"  applicata media {statistics.mean(ampiezze_applicate):6.2f} px = "
          f"{statistics.mean(ampiezze_applicate)*CM_PER_PIXEL:5.1f} cm")
    quota = statistics.mean(ampiezze_applicate) / statistics.mean(ampiezze_grezze)
    print(f"\n  QUOTA DI RETE EFFETTIVAMENTE APPLICATA: {100*quota:.1f}%")


if __name__ == "__main__":
    principale()

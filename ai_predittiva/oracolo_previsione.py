# -*- coding: utf-8 -*-
"""Macchia di probabilita' ORACOLO: la stessa regione di rischio, costruita conoscendo il futuro.

A COSA SERVE. Il capitolo 5 misura una rete che riduce del 20,2% l'errore di previsione del Kalman
e non cambia il comportamento del robot. Resta pero' aperta la domanda che qualunque lettore pone
subito dopo: e se la rete fosse migliore? con il 50% di errore in meno? con il 100%?

Questo oracolo risponde una volta per tutte, perche' misura il caso limite. Sostituisce la
previsione che alimenta la macchia con la posizione futura VERA - errore nullo, cioe' la rete
perfetta che nessun addestramento potra' mai superare - e lascia identico tutto il resto: stessa
diffusione, stesso costo, stesso Theta*, stesso controllo.

  SE L'ORACOLO NON MIGLIORA I TEMPI, il risultato negativo del capitolo 5 non dipende dalla qualita'
  della rete: dipende dalla struttura della catena. Nessuna correzione della previsione, per quanto
  accurata, puo' cambiare il comportamento passando per la mappa di costo. E' un limite superiore
  DIMOSTRATO, che vale piu' di dieci tentativi di addestramento falliti.

  SE MIGLIORA, il tetto dice quanto margine resta e a quale accuratezza converrebbe puntare: la
  differenza fra il 20,2% ottenuto e il 100% dell'oracolo diventa una scala su cui collocarsi.

E' il gemello, sul canale del PIANIFICATORE, di quello che ai_controllo/oracolo.py fa sul canale
del CONTROLLO DI VELOCITA'. I tre canali di innesto di main.py - previsione_per_macchia,
previsione_per_controllo, previsione_per_evitamento - hanno ora ciascuno il proprio oracolo, e si
possono misurare uno alla volta.

COSA SIGNIFICA QUI "CONOSCERE IL FUTURO". La stessa cosa che significa per l'oracolo del controllo,
e si riusa la stessa funzione: la persona percorre il proprio percorso gia' pianificato, alla
propria velocita'. Cattura le SVOLTE, che sono cio' che il Kalman sbaglia di piu' - a velocita'
costante chi gira l'angolo viene proiettato dritto contro il muro.

LIMITE DA DICHIARARE, identico a quello dell'altro oracolo: non e' la traiettoria letterale.
Repulsione fra persone, repulsione del robot, attese e cambi di meta possono deviarla. E' quindi un
oracolo sull'INTENZIONE E IL PERCORSO, non sull'esito, e sottostima il vantaggio teorico massimo.
La direzione dell'errore e' quella prudente: se guadagna, il guadagno e' reale; se non guadagna,
resta il dubbio che un oracolo perfetto in senso stretto guadagnerebbe qualcosa - piccolo, perche'
le deviazioni da interazione sono di scala molto minore delle svolte.

SECONDO LIMITE: l'oracolo eredita i difetti della catena a valle. Misura quanto vale sapere il
futuro DENTRO QUESTO SISTEMA, non quanto varrebbe un sistema migliore. Se non guadagna nulla, la
conclusione corretta non e' "l'AI e' inutile" ma "migliorare la previsione e' inutile finche' il suo
unico canale d'azione e' il costo delle celle".
"""
import os
import sys

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "ai_controllo")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

# la traiettoria vera e' gia' definita una volta sola nell'oracolo del controllo: si riusa invece di
# riscriverla, cosi' i due oracoli non possono divergere su cosa intendono per "posizione futura"
from oracolo import posizione_futura_vera  # noqa: E402

# id(persona) -> persona. Il tracciamento usa id(p) come chiave di traccia, quindi da una traccia si
# risale all'oggetto persona e al suo percorso.
_REGISTRO = {}


def installa(testing, sim):
    """Sostituisce la sorgente di previsione della sola macchia di probabilita'.

    Va chiamata DENTRO il processo che esegue la corsa: con lo spawn di Windows i worker
    re-importano i moduli da zero e non vedrebbero una patch fatta nel processo padre.

    Ritorna una funzione che ripristina lo stato precedente.
    """
    traccia_originale = testing._rileva_e_traccia
    previsione_originale = sim.previsione_per_macchia

    def _con_registro(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                      storico_posizioni):
        _REGISTRO.clear()
        for p in tutte_le_persone:
            _REGISTRO[id(p)] = p
        risultato = traccia_originale(tutte_le_persone, robot_x, robot_y, griglia,
                                      tracciamento_lidar, storico_posizioni)
        # la traccia non porta con se' la propria chiave: la si scrive dentro, cosi' la previsione
        # oracolo puo' risalire alla persona che la traccia rappresenta
        for pid, traccia in tracciamento_lidar.items():
            traccia["pid"] = pid
        return risultato

    def _previsione_oracolo(traccia, frame_futuri):
        persona = _REGISTRO.get(traccia.get("pid"))
        if persona is None:
            # traccia senza persona nota: si ricade sul Kalman invece di inventare. Non dovrebbe
            # accadere, ma un oracolo che tace e' meglio di uno che mente
            return previsione_originale(traccia, frame_futuri)
        return posizione_futura_vera(sim, persona, frame_futuri)

    testing._rileva_e_traccia = _con_registro
    sim.previsione_per_macchia = _previsione_oracolo

    def ripristina():
        testing._rileva_e_traccia = traccia_originale
        sim.previsione_per_macchia = previsione_originale
        _REGISTRO.clear()

    return ripristina

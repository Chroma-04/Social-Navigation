"""Controllo di velocita' ORACOLO: la stessa decisione, presa conoscendo il futuro.

A COSA SERVE. Il controllo di velocita' proietta le persone col filtro di Kalman, che sbaglia
di circa un metro a un secondo di orizzonte, contro una soglia di conflitto di 62 cm. Non si
sa pero' quanto di cio' che il controllo NON ottiene dipenda da quell'errore e quanto dal modo
in cui il controllo e' fatto. L'oracolo separa le due cose: e' lo stesso identico codice
decisionale, con errore di previsione nullo.

I due usi, entrambi necessari prima di addestrare qualsiasi rete:

  TETTO. Se l'oracolo non migliora le prestazioni, nessuna rete potra' migliorarle passando
  per la previsione: il limite e' nella struttura del controllo, non nell'informazione. E' un
  limite superiore DIMOSTRATO, che in tesi vale piu' di un tentativo di addestramento fallito.

  MAESTRO. Se invece migliora, le sue decisioni sono etichette di addestramento esatte: per
  ogni fotogramma si ha "questa scena -> questo fattore di velocita' era quello giusto". E'
  apprendimento supervisionato con bersaglio noto, come il residuo del Kalman nel capitolo 5,
  ma spostato dal livello della PREVISIONE a quello della DECISIONE. La differenza conta: la
  rete produrrebbe direttamente la grandezza che agisce sul robot, senza intermediari che
  possano assorbire il guadagno lungo la strada - che e' esattamente cio' che e' successo alla
  correzione della previsione, migliore del 12,5% e ininfluente sui tempi.

COSA SIGNIFICA QUI "CONOSCERE IL FUTURO". La posizione futura vera si ottiene percorrendo il
percorso che la persona ha gia' pianificato, alla sua velocita'. Questo cattura le SVOLTE, che
sono cio' che il Kalman sbaglia di piu': a velocita' costante una persona che gira l'angolo
viene proiettata dritta contro il muro.

LIMITE DA DICHIARARE: non e' la traiettoria letterale. Le interazioni - repulsione fra persone,
repulsione del robot, attese, cambi di meta - possono deviarla, e questo oracolo non le prevede.
E' quindi un oracolo sull'INTENZIONE E IL PERCORSO, non sull'esito. Sottostima il vantaggio
teorico massimo, il che va nella direzione prudente: se guadagna, il guadagno e' reale.

SECONDO LIMITE, PIU' IMPORTANTE: l'oracolo non e' la decisione giusta in assoluto, e' la
decisione DELLO STESSO ALGORITMO con informazione perfetta. Se l'algoritmo e' mal concepito,
l'oracolo eredita i suoi difetti. Misura quanto vale sapere il futuro, non quanto varrebbe un
controllo migliore. Se non guadagna nulla, la conclusione corretta non e' "l'AI e' inutile" ma
"migliorare la previsione e' inutile per questo controllo, e semmai va cambiato il controllo".
"""
import math

# id(persona) -> persona. Il tracciamento usa id(p) come chiave di traccia (vedi
# _rileva_e_traccia in ai_predittiva/genera_dataset_previsione.py), quindi da una traccia si
# risale all'oggetto persona e al suo percorso.
_REGISTRO = {}


def posizione_futura_vera(sim, persona, frame_futuri):
    """Dove sara' la persona fra 'frame_futuri', percorrendo il proprio percorso alla propria
    velocita'. E' la traiettoria che seguirebbe se nessuno la deviasse."""
    if persona.get("stato") != "movimento":
        return persona["x"], persona["y"]
    percorso = persona.get("percorso")
    if not percorso:
        return persona["x"], persona["y"]

    # stessa velocita' di _muovi_e_gestisci_stato: l'accelerazione dei banchi di misura e' gia'
    # dentro fattore_velocita, quindi non va riapplicata qui
    da_percorrere = sim.VELOCITA_PERSONA * persona["fattore_velocita"] * frame_futuri
    x, y = persona["x"], persona["y"]
    for nodo in percorso:
        segmento = math.hypot(nodo.cx - x, nodo.cy - y)
        if segmento <= 0:
            continue
        if segmento >= da_percorrere:
            frazione = da_percorrere / segmento
            return x + (nodo.cx - x) * frazione, y + (nodo.cy - y) * frazione
        da_percorrere -= segmento
        x, y = nodo.cx, nodo.cy
    return x, y  # percorso piu' corto dell'orizzonte: si ferma alla meta'


def installa(testing, sim):
    """Sostituisce la sorgente di previsione del solo controllo di velocita'.

    Va chiamata DENTRO il processo che esegue la corsa: con lo spawn di Windows i worker
    re-importano i moduli da zero e non vedrebbero una patch fatta nel processo padre.

    Ritorna una funzione che ripristina lo stato precedente.
    """
    traccia_originale = testing._rileva_e_traccia
    previsione_originale = sim.previsione_per_controllo

    def _con_registro(tutte_le_persone, robot_x, robot_y, griglia, tracciamento_lidar,
                      storico_posizioni):
        _REGISTRO.clear()
        for p in tutte_le_persone:
            _REGISTRO[id(p)] = p
        risultato = traccia_originale(tutte_le_persone, robot_x, robot_y, griglia,
                                      tracciamento_lidar, storico_posizioni)
        # la traccia non porta con se' la propria chiave: la si scrive dentro, cosi' la
        # previsione oracolo puo' risalire alla persona che la traccia rappresenta
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
    sim.previsione_per_controllo = _previsione_oracolo

    def ripristina():
        testing._rileva_e_traccia = traccia_originale
        sim.previsione_per_controllo = previsione_originale
        _REGISTRO.clear()

    return ripristina

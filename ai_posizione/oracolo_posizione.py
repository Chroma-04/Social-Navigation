"""Controllo di POSIZIONE oracolo: la stessa manovra, scelta conoscendo il futuro.

A COSA SERVE. Il capitolo sull'ibridazione ha misurato il tetto dell'oracolo sul controllo di
VELOCITA' e lo ha trovato nullo: sapere il futuro non fa arrivare prima, perche' rallentare non
crea distanza. Quel risultato non si trasferisce al controllo di posizione, per due ragioni:

  - i due controlli hanno punti di innesto distinti (previsione_per_controllo contro
    previsione_per_evitamento), quindi in TUTTE le campagne dell'oracolo di velocita' il
    controllo di posizione, quando era acceso, ha continuato a girare su Kalman puro;

  - il canale e' diverso. Il controllo di velocita' sceglie uno scalare, e una previsione
    migliore serve solo a decidere QUANDO frenare. Il controllo di posizione sceglie una
    direzione in cui scansarsi, e la direzione e' esattamente cio' che la previsione codifica.
    E' quindi il canale su cui l'informazione ha la possibilita' strutturale di contare.

COSA MISURA. Il tetto: il massimo che una rete potra' mai ottenere passando per la previsione di
questo controllo. Se l'oracolo non guadagna, nessuna rete guadagnera' - e il limite e' nella
struttura del campionamento, non nell'informazione. Se guadagna, il suo margine e' il bersaglio.

COME. Sostituisce la sola sim.previsione_per_evitamento, lasciando intatti la macchia del
pianificatore e il controllo di velocita'. La posizione futura vera e' la stessa di
ai_controllo/oracolo.py - il percorso gia' pianificato della persona, percorso alla sua
velocita' - cosi' i due tetti sono confrontabili fra loro.

DUE LIMITI, gli stessi dell'oracolo di velocita' e vanno dichiarati insieme al risultato:
  - non e' la traiettoria letterale: repulsioni, attese e cambi di meta possono deviarla. E'
    un oracolo sull'INTENZIONE, e sottostima il vantaggio teorico. Prudente nel verso giusto.
  - non e' la manovra giusta in assoluto: e' la manovra DELLO STESSO algoritmo a campionamento
    con informazione perfetta. Se non guadagna, la conclusione e' "migliorare la previsione non
    serve a questo controllo", non "l'AI e' inutile".

NOTA SULL'ORIZZONTE. offset_evitamento_predittivo proietta a 0, 0,5 e 1,0 volte
PREVISIONE_EVITAMENTO_FRAME, contati in passi di simulazione. Sui banchi accelerati un passo
vale ACCELERAZIONE fotogrammi reali, e a differenza della macchia del pianificatore - che
divide l'orizzonte per ACCELERAZIONE in testing.py - questo controllo non lo fa: guarda quindi
piu' avanti di quanto farebbe dal vivo. L'oracolo eredita esattamente lo stesso orizzonte,
perche' fattore_velocita porta gia' dentro di se' l'accelerazione: il confronto resta appaiato.
Il primo dei tre istanti e' 0,0, dove la previsione e' esatta per costruzione: su tre termini
l'oracolo puo' cambiarne al massimo due.
"""
import os
import sys

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.join(_RADICE, "ai_controllo")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

from ai_controllo.oracolo import posizione_futura_vera

# id(persona) -> persona. Registro separato da quello di ai_controllo/oracolo.py: i due oracoli
# devono poter convivere nello stesso processo senza sovrascriversi il registro a vicenda.
_REGISTRO = {}


def installa(testing, sim):
    """Sostituisce la sorgente di previsione del solo controllo di posizione.

    Va chiamata DENTRO il processo che esegue la corsa: con lo spawn di Windows i worker
    re-importano i moduli da zero e non vedrebbero una patch fatta nel processo padre.

    Ritorna una funzione che ripristina lo stato precedente.
    """
    traccia_originale = testing._rileva_e_traccia
    previsione_originale = sim.previsione_per_evitamento

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
            # traccia senza persona nota: si ricade sul Kalman invece di inventare
            return previsione_originale(traccia, frame_futuri)
        return posizione_futura_vera(sim, persona, frame_futuri)

    testing._rileva_e_traccia = _con_registro
    sim.previsione_per_evitamento = _previsione_oracolo

    def ripristina():
        testing._rileva_e_traccia = traccia_originale
        sim.previsione_per_evitamento = previsione_originale
        _REGISTRO.clear()

    return ripristina

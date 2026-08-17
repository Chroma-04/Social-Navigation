"""Controllo predittivo di velocita' acceso / spento sulla planimetria reale di Povo 1.

Misura il componente che la tesi descrive ma che non era mai stato provato: `fattore_velocita_da_conflitto`,
che rallenta o accelera il robot in base ai conflitti previsti lungo il percorso. Tutte le campagne
precedenti confrontavano `base` con `solo_ai` tenendolo SPENTO in entrambe le condizioni, quindi delle
quattro condizioni definite in testing.CONDIZIONI ne erano state usate solo due.

Perche' proprio qui. Nei corridoi da 1,5 m di Povo la correzione AI non ha prodotto effetto, e
l'ipotesi e' che manchi lo spazio laterale per agire sulla previsione: prevedere meglio non serve se
l'unica mossa disponibile resta fermarsi. Il controllo di velocita' e' l'unico meccanismo del sistema
che **non ha bisogno di spazio laterale**: rallentare in anticipo si puo' fare anche in un corridoio
dove non ci si puo' scansare. Se un beneficio esiste da qualche parte, e' il posto dove cercarlo.

Disegno identico a confronto_ai.py, di cui riusa la macchina: stessi semi per le due condizioni, stesso
tragitto, stessa folla iniziale. Cambia solo quale componente viene acceso.

La popolazione e l'etichetta dei semi sono le stesse della serie `confronto_ai_povo_d070`, quindi:
  - i tempi della condizione `base` devono venire IDENTICI a quelli gia' misurati (stesso seme, stessa
    condizione, corsa deterministica): e' una verifica di riproducibilita' gratuita;
  - `solo_velocita` si confronta in modo appaiato con `solo_ai` sugli stessi 50 semi, quindi si puo'
    dire quale dei due componenti paga di piu'.

Uso:
    py TEST/test_velocita_povo.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # i moduli condivisi stanno nella radice
for _percorso in (_RADICE, os.path.dirname(os.path.abspath(__file__)),
                  os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import confronto_ai as ca

MAPPA = "povo"
NUMERO_SEMI = 50
CONDIZIONI = ("base", "solo_velocita")

# identica alla serie confronto_ai_povo_d070: ~349 individui su 499 m2 = 0,70 pers/m2
POPOLAZIONE = {
    "numero_persone": 72,
    "numero_corridori": 66,
    "numero_persone_ferme": 116,
    "numero_gruppi": 27,
}

ETICHETTA_SEMI = "confronto_ai_povo"  # invariata: stessi semi delle campagne sulla correzione AI
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "confronto_velocita_povo_d070.csv")


def main():
    ca.esegui_confronto(mappa=MAPPA, conteggi=POPOLAZIONE, numero_semi=NUMERO_SEMI,
                        file_csv=FILE_CSV, etichetta=ETICHETTA_SEMI, condizioni=CONDIZIONI)


if __name__ == "__main__":
    main()

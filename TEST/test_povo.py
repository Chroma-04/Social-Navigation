"""Confronto appaiato AI acceso / AI spento sulla planimetria reale del secondo piano di Povo 1.

Stesso disegno sperimentale di confronto_ai.py - stessi semi per le due condizioni, stesso tragitto
angolo->angolo, controllo di velocita' spento in entrambe - di cui riusa integralmente la macchina.
Cambiano due sole cose, ed e' il punto dell'esperimento:

  MAPPA. Non piu' una planimetria generata, ma quella ricalcata dall'edificio: 44x73 celle da 0,5 m,
  cioe' 22 x 36,5 m, con 499 m2 calpestabili contro gli 890 di training_mappa_01. La topologia e'
  diversa in modo sostanziale: un corridoio ad anello attorno a un nucleo centrale di servizi, quindi
  esistono due rotte alternative genuine fra due punti qualsiasi, ma i passaggi sono stretti - l'82%
  della superficie libera sta entro una cella da un muro, contro il 69% della mappa generata.

  DENSITA'. La popolazione e' riscalata sulla superficie calpestabile e alzata di poco rispetto alla
  proporzione esatta, per non misurare una scena piu' vuota di quella gia' misurata.

L'ipotesi che mette alla prova: la correzione predittiva puo' pagare solo se il robot ha spazio
laterale per agire sulla previsione. Corridoi da 1,5 m tolgono quello spazio, quindi qui il risultato
negativo gia' misurato dovrebbe semmai rafforzarsi. Se invece l'anello facesse pendere la scelta fra
le due rotte, si vedrebbe il contrario.

Uso:
    py TEST/test_povo.py
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

MAPPA = "povo"  # risolta da testing.MAPPE_SALVATE in mappe_salvate/mappa_povo.json
NUMERO_SEMI = 50

# Popolazione: ~254 individui su 499 m2 = 0,51 pers/m2, cioe' il 20% in MENO della serie di
# riferimento. Le proporzioni fra le quattro categorie restano quelle del pannello tecnico.
# Serie gia' misurate sulla stessa mappa, stessi semi, stesso tragitto:
#   0,64 pers/m2  65 / 60 / 105 / 25  (~318)  -> risultati_test/confronto_ai_povo_d064.md
#   0,70 pers/m2  72 / 66 / 116 / 27  (~349)  -> risultati_test/confronto_ai_povo_d070.md
POPOLAZIONE = {
    "numero_persone": 52,
    "numero_corridori": 48,
    "numero_persone_ferme": 84,
    "numero_gruppi": 20,
}

# etichetta INVARIATA fra le due serie di densita': i semi dipendono da mappa+etichetta+indice, quindi
# i primi 30 restano identici a quelli gia' misurati e le due densita' si confrontano in modo appaiato
ETICHETTA_SEMI = "confronto_ai_povo"
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "confronto_ai_povo_d051.csv")


def main():
    ca.esegui_confronto(mappa=MAPPA, conteggi=POPOLAZIONE, numero_semi=NUMERO_SEMI,
                        file_csv=FILE_CSV, etichetta=ETICHETTA_SEMI)


if __name__ == "__main__":
    main()

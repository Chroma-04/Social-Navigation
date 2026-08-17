"""L'ibridazione paga dove c'e' spazio per manovrare? Stessa densita' di Povo, ambiente senza vincoli.

L'ipotesi che mette alla prova: su Povo la correzione non produce effetto perche' l'ambiente non lascia
convertire una previsione migliore in una traiettoria diversa. Due indizi la sostengono, ed entrambi
vengono dai dati gia' raccolti:

  - il Kalman su Povo sbaglia MENO che sulla mappa generata (0,505 contro 0,599 m). Corridoi stretti
    significano traiettorie vincolate, e una traiettoria vincolata il moto rettilineo uniforme la
    indovina bene: resta poco da correggere. In campo aperto le persone svoltano liberamente, il Kalman
    degrada, e la rete ha piu' margine;
  - a Povo l'82% della superficie libera sta entro una cella da un muro. In arena vuota lo spazio
    laterale c'e' ovunque, quindi una previsione migliore puo' tradursi in una deviazione invece che
    in un arresto.

In arena aperta le due condizioni mancanti sono entrambe soddisfatte. Se l'effetto esiste, e' qui che
deve comparire; se non compare nemmeno qui, l'ultima variabile strutturale e' esclusa.

DENSITA' IDENTICA, NON POPOLAZIONE IDENTICA. L'arena 80x60 ha 78x58 celle libere dentro i bordi, cioe'
1131 m2 contro i 499 di Povo. Tenere gli stessi 349 individui darebbe 0,29 pers/m2: si cambierebbero
struttura E densita' insieme, senza poter attribuire il risultato a nessuna delle due. Con 0,70 pers/m2
servono ~792 individui, mantenendo le proporzioni fra le quattro classi della serie d070.

Nota a favore: 'aperta' fa parte della distribuzione di addestramento del modello, quindi qui non c'e'
alcun problema di generalizzazione - e' il terreno migliore possibile per la rete.

Uso:
    py TEST/test_aperta.py
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys

_RADICE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _percorso in (_RADICE, os.path.dirname(os.path.abspath(__file__)),
                  os.path.join(_RADICE, "ai_predittiva")):
    if _percorso not in sys.path:
        sys.path.insert(0, _percorso)

import testing as t
import confronto_ai as ca

MAPPA = "aperta"
NUMERO_SEMI = 20  # corse ~2,3x piu' care: qui la folla e' piu' che doppia

# stesse proporzioni della serie confronto_ai_povo_d070, scalate a 1131 m2 per restare a 0,70 pers/m2:
# 163 + 150 + 263 + 61x3,5 = ~790 individui
POPOLAZIONE = {
    "numero_persone": 163,
    "numero_corridori": 150,
    "numero_persone_ferme": 263,
    "numero_gruppi": 61,
}

ETICHETTA_SEMI = "confronto_ai_aperta"
FILE_CSV = os.path.join(t.CARTELLA_RISULTATI, "confronto_ai_aperta_d070.csv")


def main():
    ca.esegui_confronto(mappa=MAPPA, conteggi=POPOLAZIONE, numero_semi=NUMERO_SEMI,
                        file_csv=FILE_CSV, etichetta=ETICHETTA_SEMI,
                        condizioni=("base", "solo_ai"))


if __name__ == "__main__":
    main()

# Confronto AI acceso / AI spento — macchia a 1,3 celle

Misura del 12 agosto 2026, prodotta da `confronto_ai.py`. Dati grezzi in `confronto_ai_blob13.csv`.

Ripete **esattamente** l'esperimento di `confronto_ai_blob30.md` — stessi 20 semi, stessa popolazione,
stesso tragitto, stessa geometria — cambiando un solo parametro: il raggio della macchia di probabilità,
da 3,0 celle (1,5 m) a 1,3 celle (0,65 m).

Il raggio ridotto era stato dimensionato sull'incertezza misurata della previsione (0,60 m di errore medio
a un secondo, cioè 1,2 celle), contro i 3,0 celle precedenti scelti prima di disporre di quella misura.

## Risultati per seme

| seme | senza AI | con AI | differenza | variazione | fermo base | fermo AI |
|---|---|---|---|---|---|---|
| 758002065 | 76,2 s | 66,6 s | +9,7 s | +12,7% | 11,2 s | 7,2 s |
| 1512636679 | 62,1 s | 72,3 s | −10,2 s | −16,5% | 6,7 s | 10,3 s |
| 3273674941 | 63,3 s | 62,1 s | +1,2 s | +2,0% | 8,8 s | 6,6 s |
| 3022479403 | 62,1 s | 56,1 s | +6,0 s | +9,7% | 8,5 s | 3,4 s |
| 709096840 | 68,1 s | 68,8 s | −0,8 s | −1,1% | 5,2 s | 3,0 s |
| 1564787998 | 72,5 s | 68,4 s | +4,1 s | +5,6% | 12,0 s | 9,8 s |
| 3293418660 | 56,1 s | 54,9 s | +1,2 s | +2,1% | 5,5 s | 6,5 s |
| 3008013362 | 55,3 s | 67,2 s | −11,9 s | −21,5% | 5,1 s | 9,8 s |
| 603306403 | 63,5 s | 61,2 s | +2,3 s | +3,7% | 7,8 s | 7,4 s |
| 1425180981 | 64,1 s | 61,3 s | +2,7 s | +4,3% | 6,6 s | 5,8 s |
| 1793418115 | 59,7 s | 57,3 s | +2,3 s | +3,9% | 6,4 s | 7,8 s |
| 501371669 | 69,1 s | 66,1 s | +3,0 s | +4,3% | 7,6 s | 7,8 s |
| 2229994159 | 63,8 s | 60,9 s | +2,9 s | +4,6% | 3,7 s | 6,6 s |
| 4092342841 | 74,5 s | 71,2 s | +3,3 s | +4,5% | 6,5 s | 5,9 s |
| 1837672346 | 66,3 s | 67,2 s | −0,9 s | −1,4% | 9,7 s | 6,9 s |
| 445617932 | 63,0 s | 64,1 s | −1,1 s | −1,7% | 8,2 s | 8,5 s |
| 2206647990 | 64,8 s | 64,8 s | 0,0 s | 0,0% | 13,5 s | 12,9 s |
| 4102157856 | 74,0 s | 79,6 s | −5,6 s | −7,5% | 6,2 s | 7,3 s |
| 1681845169 | 64,7 s | 74,6 s | −9,9 s | −15,3% | 10,3 s | 8,9 s |
| 322558759 | 61,3 s | 55,3 s | +6,0 s | +9,8% | 6,9 s | 4,3 s |
| **media** | **65,23 s** | **65,00 s** | **+0,23 s** | **+0,1%** | **7,82 s** | **7,34 s** |

## Confronto fra le due configurazioni

| | macchia 3,0 (1,5 m) | macchia 1,3 (0,65 m) |
|---|---|---|
| tempo medio senza AI | 64,89 s | 65,23 s |
| tempo medio con AI | 63,37 s | 65,00 s |
| differenza media | +1,52 s (+2,1%) | +0,23 s (+0,1%) |
| coppie vinte dall'AI | 13/20 | 12/20 |
| rapporto media/errore | 1,26 | 0,2 |
| tempo fermo base | 8,50 s | 7,82 s |
| tempo fermo AI | 7,88 s | 7,34 s |

## Conclusione

Poiché i due giri condividono i medesimi semi, le due configurazioni si possono confrontare in modo
appaiato, seme per seme, prendendo per ciascuno la differenza fra il vantaggio dell'AI nell'una e
nell'altra. Il risultato è **1,30 s a favore della macchia larga, con errore standard 1,81 s**: rapporto
0,72, corrispondente a una probabilità intorno al 48% di osservare uno scarto simile per puro caso.

**Le due configurazioni non sono distinguibili fra loro.** La lettura corretta non è "restringere la
macchia peggiora l'AI", bensì: *in nessuna delle due configurazioni si rileva un effetto della correzione
sui tempi di percorrenza, e la conclusione è insensibile al raggio della macchia.*

Il risultato negativo è quindi **robusto** rispetto a un cambiamento che più che dimezza il parametro,
il che lo rende difficile da attribuire a una cattiva taratura di quel parametro.

La dispersione fra i due giri è però enorme sul singolo seme: `1681845169` passa da +11,1 s a −9,9 s,
`3008013362` da +0,3 s a −11,9 s, `1512636679` da +2,9 s a −10,2 s. Differenze minime di rotta all'avvio
mandano le due simulazioni su traiettorie divergenti, e il tempo finale ne risulta dominato.

## Configurazione mantenuta

Dopo questa misura il raggio è stato **riportato a 3,0 celle**: a parità di prestazioni misurate, si è
preferita la rappresentazione visiva più leggibile, coerente con le figure già prodotte per l'elaborato.

# Confronto AI acceso / AI spento — macchia a 3,0 celle

Misura del 12 agosto 2026, prodotta da `confronto_ai.py`. Dati grezzi in `confronto_ai_blob30.csv`.

## Configurazione

| parametro | valore |
|---|---|
| mappa | `training_mappa_01` |
| popolazione | 100 persone, 90 veloci, 170 ferme, 43 gruppi (~510 individui) |
| tragitto | fisso, angolo alto-sinistro → angolo basso-destro |
| condizioni | `base` (solo Kalman) contro `solo_ai` (Kalman + correzione neurale) |
| controllo di velocità | spento in entrambe |
| geometria | robot 5,7 px, persone 7,0 px, sicurezza 10,1 px, lidar 123 px (da `config_pannello.json`) |
| **raggio macchia** | **3,0 celle = 1,5 m** |
| accelerazione | 5× |
| ripetizioni | 20 semi appaiati (stessa folla per le due condizioni) |

## Risultati per seme

| seme | senza AI | con AI | differenza | variazione | fermo base | fermo AI |
|---|---|---|---|---|---|---|
| 758002065 | 82,58 s | 73,58 s | +9,00 s | +10,9% | 11,33 s | 12,17 s |
| 1512636679 | 64,75 s | 61,83 s | +2,92 s | +4,5% | 11,33 s | 10,67 s |
| 3273674941 | 66,08 s | 77,00 s | −10,92 s | −16,5% | 11,50 s | 8,75 s |
| 3022479403 | 70,42 s | 66,92 s | +3,50 s | +5,0% | 7,92 s | 6,75 s |
| 709096840 | 74,08 s | 78,08 s | −4,00 s | −5,4% | 8,17 s | 8,67 s |
| 1564787998 | 60,25 s | 65,42 s | −5,17 s | −8,6% | 7,42 s | 8,58 s |
| 3293418660 | 63,92 s | 64,08 s | −0,17 s | −0,3% | 7,17 s | 10,33 s |
| 3008013362 | 65,75 s | 65,50 s | +0,25 s | +0,4% | 9,33 s | 9,92 s |
| 603306403 | 59,83 s | 59,67 s | +0,17 s | +0,3% | 8,00 s | 8,00 s |
| 1425180981 | 59,75 s | 58,67 s | +1,08 s | +1,8% | 6,33 s | 4,83 s |
| 1793418115 | 57,08 s | 55,33 s | +1,75 s | +3,1% | 5,92 s | 5,67 s |
| 501371669 | 62,58 s | 62,58 s | 0,00 s | 0,0% | 6,50 s | 6,50 s |
| 2229994159 | 56,25 s | 54,42 s | +1,83 s | +3,3% | 4,75 s | 3,25 s |
| 4092342841 | 70,67 s | 56,42 s | +14,25 s | +20,2% | 7,58 s | 5,08 s |
| 1837672346 | 66,92 s | 65,00 s | +1,92 s | +2,9% | 8,17 s | 8,33 s |
| 445617932 | 62,75 s | 63,75 s | −1,00 s | −1,6% | 8,00 s | 9,75 s |
| 2206647990 | 60,33 s | 62,33 s | −2,00 s | −3,3% | 9,08 s | 10,75 s |
| 4102157856 | 66,08 s | 61,75 s | +4,33 s | +6,6% | 11,17 s | 9,08 s |
| 1681845169 | 69,00 s | 57,92 s | +11,08 s | +16,1% | 12,92 s | 4,17 s |
| 322558759 | 58,75 s | 57,25 s | +1,50 s | +2,6% | 7,33 s | 6,33 s |
| **media** | **64,89 s** | **63,37 s** | **+1,52 s** | **+2,1%** | **8,50 s** | **7,88 s** |

## Sintesi

- differenza media **+1,52 s (+2,1%)**, mediana +1,3 s: il segno non dipende dalle code;
- **13 coppie vinte su 20** dall'AI;
- errore standard 1,2 s, rapporto media/errore **1,26**: la differenza **non è distinguibile da zero**;
- intervallo di confidenza al 95%: da **−1,0 s a +4,0 s**, cioè da −1,5% a +6,2%;
- tempo fermo: 8,50 s contro 7,88 s, su ~65 s totali (**13,1%** del tempo nel caso base).

## Lettura

La distribuzione è asimmetrica: nove coppie stanno entro il ±3%, e i casi che contano sono le code — tre
guadagni grossi (+20,2%, +16,1%, +10,9%) contro due perdite grosse (−16,5%, −8,6%).

Due meccanismi opposti convivono. I guadagni maggiori vengono dagli arresti evitati: il seme `1681845169`
passa da 12,9 s a 4,2 s di tempo fermo, ed è da lì che nascono i suoi 11 secondi di vantaggio. La perdita
peggiore, il seme `3273674941`, avviene invece *pur stando fermo meno* (11,5 → 8,8 s): la mappa di costo
modificata lo ha spinto su una rotta più prudente ma più lunga.

Il risultato netto dipende da quale dei due meccanismi prevale nella scena specifica, il che spiega perché
la media sia piccola e la dispersione grande.

## Perché questa configurazione è stata poi cambiata

Il raggio di 1,5 m era stato scelto prima di misurare l'incertezza che la macchia rappresenta. Misurata
poi sul test set, quell'incertezza vale **0,60 m di errore medio** — la macchia era due volte e mezzo più
larga, e pari al 73% dell'intero raggio del lidar (2,05 m). Una sola persona rilevata rendeva cara più di
metà dell'area percepita, saturando la mappa di costo. Il confronto successivo usa un raggio di 1,3 celle
(0,65 m), dimensionato sull'errore misurato.

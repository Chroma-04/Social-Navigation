# Confronto AI acceso / AI spento - Povo 1 secondo piano, densita 0,51 pers/m2

Misura del 14 agosto 2026, prodotta da `TEST/test_povo.py`. Dati grezzi in `confronto_ai_povo_d051.csv`.

Terzo punto di densita sulla stessa planimetria, **20% sotto la serie di riferimento**. Etichetta dei
semi invariata, quindi tutti e 50 i semi coincidono con quelli della serie a 0,70 e i primi 30 anche
con quella a 0,64: le tre serie si confrontano in modo appaiato.

## Configurazione

| parametro | valore |
|---|---|
| mappa | `povo`, 44x73 celle da 0,5 m, 499 m2 calpestabili |
| popolazione | 52 persone, 48 veloci, 84 ferme, 20 gruppi (~254 individui) |
| densita | **0,51 pers/m2** |
| tragitto | fisso, angolo alto-sinistro -> angolo basso-destro |
| condizioni | `base` (solo Kalman) contro `solo_ai` (Kalman + correzione neurale) |
| geometria | robot 5,7 px, persone 7,0 px, sicurezza 10,1 px, lidar 123 px |
| ripetizioni | 50 semi appaiati, 100 corse |

## Risultati per seme

| seme | senza AI | con AI | differenza | variazione | fermo base | fermo AI |
|---|---|---|---|---|---|---|
| 2616969334 | 50.0 s | 49.8 s | +0.2 s | +0.5% | 4.5 s | 4.2 s |
| 3976001760 | 45.8 s | 46.1 s | -0.3 s | -0.7% | 3.2 s | 3.2 s |
| 1979033946 | 47.3 s | 48.0 s | -0.7 s | -1.4% | 3.3 s | 3.0 s |
| 49453516 | 46.3 s | 45.7 s | +0.7 s | +1.4% | 5.1 s | 4.0 s |
| 2627079279 | 45.6 s | 49.2 s | -3.6 s | -7.9% | 2.8 s | 3.6 s |
| 3952164089 | 42.3 s | 43.7 s | -1.3 s | -3.1% | 2.4 s | 1.9 s |
| 1922592067 | 47.1 s | 46.3 s | +0.8 s | +1.6% | 6.3 s | 5.0 s |
| 94330325 | 51.3 s | 49.7 s | +1.7 s | +3.2% | 7.6 s | 6.3 s |
| 2501918788 | 54.9 s | 47.2 s | +7.7 s | +14.0% | 9.2 s | 3.2 s |
| 3794235602 | 51.1 s | 46.8 s | +4.2 s | +8.3% | 7.3 s | 4.2 s |
| 1413333409 | 49.1 s | 53.2 s | -4.2 s | -8.5% | 1.7 s | 5.4 s |
| 591065399 | 48.7 s | 52.6 s | -3.9 s | -8.0% | 5.0 s | 8.3 s |
| 3123945613 | 44.6 s | 44.8 s | -0.2 s | -0.6% | 3.9 s | 4.3 s |
| 3442774043 | 46.1 s | 48.1 s | -2.0 s | -4.3% | 4.7 s | 5.9 s |
| 1397753272 | 50.3 s | 49.0 s | +1.3 s | +2.6% | 6.2 s | 3.9 s |
| 609695022 | 43.8 s | 44.8 s | -1.0 s | -2.3% | 1.8 s | 2.2 s |
| 3177079956 | 51.2 s | 47.9 s | +3.3 s | +6.5% | 4.9 s | 3.5 s |
| 3394851842 | 42.2 s | 48.1 s | -5.8 s | -13.8% | 2.3 s | 6.1 s |
| 1525041555 | 46.2 s | 46.0 s | +0.2 s | +0.5% | 3.8 s | 3.8 s |
| 769751301 | 45.3 s | 41.3 s | +4.0 s | +8.8% | 2.5 s | 0.6 s |
| 2131792482 | 49.4 s | 48.2 s | +1.2 s | +2.4% | 5.5 s | 5.6 s |
| 135766772 | 44.9 s | 44.8 s | +0.1 s | +0.2% | 2.8 s | 3.7 s |
| 2434724686 | 48.1 s | 57.1 s | -9.0 s | -18.7% | 5.4 s | 11.2 s |
| 3860448216 | 45.9 s | 46.2 s | -0.3 s | -0.7% | 2.8 s | 2.8 s |
| 2021480059 | 46.7 s | 48.9 s | -2.2 s | -4.8% | 4.4 s | 4.9 s |
| 259679981 | 48.3 s | 48.1 s | +0.2 s | +0.5% | 6.8 s | 5.4 s |
| 2524133207 | 44.3 s | 47.1 s | -2.8 s | -6.2% | 2.8 s | 5.5 s |
| 3782477761 | 49.8 s | 48.6 s | +1.2 s | +2.5% | 6.7 s | 3.4 s |
| 1909135952 | 48.7 s | 46.7 s | +2.0 s | +4.1% | 5.4 s | 3.2 s |
| 114043590 | 50.0 s | 49.1 s | +0.9 s | +1.8% | 5.2 s | 4.2 s |
| 1712038691 | 49.4 s | 49.1 s | +0.3 s | +0.7% | 3.6 s | 2.4 s |
| 286036917 | 49.1 s | 47.6 s | +1.5 s | +3.1% | 5.3 s | 3.9 s |
| 2282078735 | 46.2 s | 48.1 s | -1.9 s | -4.2% | 3.8 s | 4.5 s |
| 4278383257 | 43.0 s | 42.9 s | +0.1 s | +0.2% | 2.1 s | 2.7 s |
| 1634101050 | 45.1 s | 43.1 s | +2.0 s | +4.4% | 4.3 s | 2.2 s |
| 375478188 | 48.8 s | 47.7 s | +1.2 s | +2.4% | 6.8 s | 6.6 s |
| 2405959190 | 40.9 s | 43.7 s | -2.8 s | -6.7% | 1.6 s | 2.2 s |
| 4168038016 | 43.5 s | 42.9 s | +0.6 s | +1.3% | 3.1 s | 1.9 s |
| 1758472977 | 49.5 s | 46.9 s | +2.6 s | +5.2% | 5.8 s | 2.7 s |
| 534190983 | 48.8 s | 51.0 s | -2.2 s | -4.4% | 4.9 s | 6.1 s |
| 692729316 | 61.1 s | 58.8 s | +2.3 s | +3.8% | 5.8 s | 3.8 s |
| 1582105970 | 56.3 s | 55.9 s | +0.4 s | +0.7% | 5.9 s | 4.8 s |
| 3343144136 | 44.8 s | 45.3 s | -0.6 s | -1.3% | 2.2 s | 3.1 s |
| 2957206622 | 45.1 s | 45.3 s | -0.2 s | -0.6% | 2.7 s | 4.2 s |
| 774369789 | 48.1 s | 49.1 s | -1.0 s | -2.1% | 3.9 s | 5.0 s |
| 1495318891 | 49.8 s | 53.1 s | -3.3 s | -6.7% | 6.1 s | 7.2 s |
| 3223949521 | 46.9 s | 51.5 s | -4.6 s | -9.8% | 4.0 s | 7.1 s |
| 3073286215 | 46.3 s | 50.2 s | -3.8 s | -8.3% | 4.3 s | 8.2 s |
| 663862742 | 48.8 s | 46.6 s | +2.2 s | +4.4% | 6.5 s | 3.9 s |
| 1352043840 | 62.5 s | 55.7 s | +6.8 s | +10.9% | 6.8 s | 4.7 s |
| **media** | **47.99 s** | **48.15 s** | **-0.16 s** | **-0.6%** | **4.52 s** | **4.40 s** |

## Sintesi

- differenza media **-0.16 s (-0.6%)**, mediana +0.17 s;
- 27 coppie vinte su 50;
- errore standard 0.42 s, rapporto **0.38**;
- IC 95%: da **-2.0% a +1.4%** - il piu stretto dell'intera serie;
- tempo fermo **9.4%** del totale, contro 13,5% a 0,64 e 15,8% a 0,70.

## Le tre densita a confronto

| densita | semi | tempo base | differenza media | rapporto | coppie vinte | tempo fermo | IC 95% |
|---|---|---|---|---|---|---|---|
| 0,51 pers/m2 | 50 | 48,0 s | -0,16 s (-0,6%) | 0,38 | 27/50 | 9,4% | -2,0% .. +1,4% |
| 0,64 pers/m2 | 30 | 52,3 s | +0,68 s (+0,6%) | 0,63 | 17/30 | 13,5% | -2,7% .. +5,3% |
| 0,70 pers/m2 | 50 | 53,1 s | -0,16 s (-0,7%) | 0,22 | 24/50 | 15,8% | -2,9% .. +2,3% |

**Stima aggregata sui 130 semi: -0,07 s, cioe -0,14%, IC 95% da -1,46% a +1,17%.**

## Confronti appaiati fra densita

| confronto | semi | quantita | differenza | errore | rapporto |
|---|---|---|---|---|---|
| 0,70 meno 0,51 | 50 | tempo del robot base | +5,12 s | 0,86 s | **5,98** |
| 0,70 meno 0,51 | 50 | tempo fermo del robot base | +3,88 s | 0,60 s | **6,49** |
| 0,70 meno 0,51 | 50 | **vantaggio dell'AI** | **+0,00 s** | 0,90 s | **0,00** |
| 0,64 meno 0,51 | 30 | tempo del robot base | +4,74 s | 1,32 s | **3,59** |
| 0,64 meno 0,51 | 30 | tempo fermo del robot base | +2,49 s | 0,61 s | **4,07** |
| 0,64 meno 0,51 | 30 | vantaggio dell'AI | +0,93 s | 1,30 s | 0,71 |
| 0,70 meno 0,64 | 30 | tempo fermo del robot base | +1,62 s | 0,81 s | **2,00** |
| 0,70 meno 0,64 | 30 | vantaggio dell'AI | -0,63 s | 1,31 s | 0,48 |

## Lettura

Questa tabella e il risultato metodologicamente piu forte dell'intera serie.

La densita produce effetti **inequivocabili** e nella direzione attesa: passando da 0,51 a 0,70 pers/m2
il robot impiega 5,12 s in piu (rapporto 5,98) e ne passa 3,88 fermo in piu (rapporto 6,49). Sugli
stessi identici 50 semi, il vantaggio dell'AI cambia di **+0,00 s, rapporto 0,00**.

Il banco di prova rileva quindi effetti reali con rapporti fra 3,6 e 6,5, e per l'AI misura rapporti
fra 0,2 e 0,6: un ordine di grandezza di differenza. Non e un test cieco che non vede nulla - vede
benissimo cio' che c'e, e dell'AI non c'e nulla da vedere.

**Il punto decisivo per il capitolo sul quando conviene l'AI.** Il tempo fermo e il tetto di cio che
una qualunque miglioria predittiva puo recuperare, e fra 0,51 e 0,70 pers/m2 quel tetto cresce del
68% (dal 9,4% al 15,8% del tempo totale). Il margine disponibile quasi raddoppia, e la correzione
neurale continua a non estrarne niente. Non si tratta quindi di mancanza di spazio di manovra
statistico: lo spazio si allarga e resta inutilizzato.

## Conclusione

Cinque condizioni indipendenti - mappa generata, raggio della macchia dimezzato, planimetria reale e
tre livelli di densita - senza alcun effetto rilevabile sui tempi di percorrenza, a fronte di una
riduzione del 12,1% dell'errore di previsione.

Sui 130 semi appaiati della planimetria reale il limite dichiarabile e: **se un effetto esiste, e
compreso fra -1,5% e +1,2%.**

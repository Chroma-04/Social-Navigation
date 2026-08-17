# Confronto AI acceso / AI spento - planimetria reale di Povo 1, secondo piano

Misura del 14 agosto 2026, prodotta da `TEST/test_povo.py`. Dati grezzi in `confronto_ai_povo.csv`.

Replica del disegno di `confronto_ai_blob30.md` su una topologia completamente diversa: la planimetria
ricalcata a mano dall'edificio, invece di una mappa generata.

## Configurazione

| parametro | valore |
|---|---|
| mappa | `povo` (mappe_salvate/mappa_povo.json), 44x73 celle da 0,5 m = 22 x 36,5 m |
| superficie calpestabile | 499 m2 (contro 890 m2 di training_mappa_01) |
| topologia | corridoio ad anello attorno a un nucleo centrale; 82% della superficie libera entro una cella da un muro (69% sulla mappa generata) |
| popolazione | 65 persone, 60 veloci, 105 ferme, 25 gruppi (~318 individui) |
| densita | 0,64 pers/m2 (contro 0,57 pers/m2 della misura precedente) |
| tragitto | fisso, angolo alto-sinistro -> angolo basso-destro |
| condizioni | `base` (solo Kalman) contro `solo_ai` (Kalman + correzione neurale) |
| controllo di velocita | spento in entrambe |
| geometria | robot 5,7 px, persone 7,0 px, sicurezza 10,1 px, lidar 123 px (da `config_pannello.json`) |
| raggio macchia | 3,0 celle = 1,5 m |
| accelerazione | 5x |
| ripetizioni | 30 semi appaiati, 60 corse, completate in 8,5 minuti su 8 processi |

## Risultati per seme

| seme | senza AI | con AI | differenza | variazione | fermo base | fermo AI |
|---|---|---|---|---|---|---|
| 2616969334 | 47.6 s | 54.8 s | -7.2 s | -15.2% | 5.2 s | 10.4 s |
| 3976001760 | 45.1 s | 44.9 s | +0.2 s | +0.4% | 3.0 s | 2.8 s |
| 1979033946 | 48.0 s | 53.8 s | -5.8 s | -12.2% | 2.2 s | 6.6 s |
| 49453516 | 48.4 s | 47.0 s | +1.4 s | +2.9% | 5.2 s | 4.8 s |
| 2627079279 | 52.5 s | 49.0 s | +3.5 s | +6.7% | 7.3 s | 6.0 s |
| 3952164089 | 51.4 s | 49.8 s | +1.7 s | +3.2% | 6.2 s | 4.3 s |
| 1922592067 | 50.0 s | 50.4 s | -0.4 s | -0.8% | 8.2 s | 6.8 s |
| 94330325 | 51.4 s | 55.5 s | -4.1 s | -7.9% | 5.8 s | 11.2 s |
| 2501918788 | 51.8 s | 50.8 s | +1.1 s | +2.1% | 8.2 s | 8.8 s |
| 3794235602 | 57.2 s | 63.9 s | -6.8 s | -11.8% | 10.8 s | 14.1 s |
| 1413333409 | 51.8 s | 46.7 s | +5.1 s | +9.8% | 9.8 s | 6.2 s |
| 591065399 | 47.5 s | 52.0 s | -4.5 s | -9.5% | 6.2 s | 7.2 s |
| 3123945613 | 54.0 s | 47.8 s | +6.2 s | +11.4% | 8.8 s | 5.1 s |
| 3442774043 | 54.0 s | 52.8 s | +1.2 s | +2.2% | 3.7 s | 5.4 s |
| 1397753272 | 49.2 s | 48.8 s | +0.5 s | +1.0% | 3.8 s | 5.5 s |
| 609695022 | 76.6 s | 53.6 s | +23.0 s | +30.0% | 10.4 s | 7.6 s |
| 3177079956 | 47.1 s | 55.2 s | -8.2 s | -17.3% | 4.6 s | 9.1 s |
| 3394851842 | 54.8 s | 51.8 s | +3.1 s | +5.6% | 10.3 s | 5.7 s |
| 1525041555 | 48.4 s | 54.0 s | -5.6 s | -11.5% | 4.6 s | 10.0 s |
| 769751301 | 46.9 s | 47.2 s | -0.3 s | -0.7% | 6.2 s | 6.5 s |
| 2131792482 | 48.8 s | 49.0 s | -0.2 s | -0.3% | 5.2 s | 6.0 s |
| 135766772 | 52.8 s | 48.3 s | +4.5 s | +8.5% | 8.6 s | 6.9 s |
| 2434724686 | 64.0 s | 67.2 s | -3.2 s | -5.1% | 9.9 s | 14.8 s |
| 3860448216 | 49.0 s | 51.2 s | -2.2 s | -4.4% | 6.9 s | 8.0 s |
| 2021480059 | 57.2 s | 50.1 s | +7.1 s | +12.4% | 11.9 s | 3.8 s |
| 259679981 | 52.2 s | 49.3 s | +2.9 s | +5.6% | 8.6 s | 8.8 s |
| 2524133207 | 54.5 s | 53.8 s | +0.7 s | +1.2% | 10.6 s | 7.4 s |
| 3782477761 | 52.9 s | 49.2 s | +3.7 s | +6.9% | 10.1 s | 7.2 s |
| 1909135952 | 50.3 s | 50.3 s | +0.0 s | +0.0% | 4.5 s | 4.2 s |
| 114043590 | 52.1 s | 49.0 s | +3.1 s | +5.9% | 4.3 s | 5.9 s |
| **media** | **52.26 s** | **51.58 s** | **+0.68 s** | **+0.6%** | **7.05 s** | **7.23 s** |

## Sintesi

- differenza media **+0.68 s (+0.6%)**, mediana +0.58 s;
- **17 coppie vinte su 30** dall'AI;
- errore standard 1.07 s, rapporto media/errore **0.63**: la differenza **non e distinguibile da zero**;
- intervallo di confidenza al 95%: da **-1.42 s a +2.77 s**, cioe da -2.7% a +5.3%;
- tempo fermo: 7.05 s contro 7.23 s, cioe **13.5%** contro 14.0% del tempo totale;
- tutte e 30 le coppie sono valide: nessun robot ha superato il tetto di 150 s.

## Lettura

**La media positiva e interamente prodotta da un solo seme.** Il seme `609695022` vale +23,0 s perche
li il robot senza AI ha avuto una corsa patologica (76,6 s contro i ~52 s tipici), non perche quello
con AI sia andato particolarmente bene. Togliendo quella singola coppia la media crolla a
**-0.09 s** con errore 0.77 s, rapporto 0.12: esattamente zero.

Va quindi scritto come assenza di effetto, non come tendenza favorevole: il +0.6% non e un
guadagno piccolo, e rumore centrato su zero piu una coda.

**Il tempo fermo non migliora.** Sulla mappa generata l'AI lo riduceva (8,50 -> 7,88 s); qui resta
invariato (7.05 -> 7.23 s). Era da li che nascevano i guadagni maggiori nella misura
precedente, e su questa topologia quel meccanismo non si attiva.

**La dispersione resta il fatto dominante.** Deviazione standard delle differenze 5.85 s su
tempi medi di 52.3 s: 9 coppie su 30 si scostano di piu di 5 s in un verso o nell'altro.
Nell'anello di Povo esistono due rotte alternative genuine fra due punti qualsiasi, e basta una scelta
diversa all'avvio perche le due simulazioni divergano.

## Confronto con la misura sulla mappa generata

| | training_mappa_01 (20 semi) | povo (30 semi) |
|---|---|---|
| tempo medio senza AI | 64,89 s | 52.26 s |
| tempo medio con AI | 63,37 s | 51.58 s |
| differenza media | +1,52 s (+2,1%) | +0.68 s (+0.6%) |
| rapporto media/errore | 1,26 | 0.63 |
| coppie vinte dall'AI | 13/20 | 17/30 |
| tempo fermo (base) | 13,1% | 13.5% |

## Conclusione

Il risultato negativo **si replica su una topologia completamente diversa**: planimetria reale invece
che generata, superficie quasi dimezzata, corridoi molto piu stretti, densita leggermente superiore.
Insieme alla robustezza gia verificata rispetto al raggio della macchia (vedi `confronto_ai_blob13.md`),
sono tre condizioni indipendenti in cui l'effetto sui tempi di percorrenza non si manifesta.

Il limite superiore dichiarabile e **+5,3%** al 95% di confidenza.

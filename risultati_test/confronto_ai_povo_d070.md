# Confronto AI acceso / AI spento - Povo 1 secondo piano, densita 0,70 pers/m2

Misura del 14 agosto 2026, prodotta da `TEST/test_povo.py`. Dati grezzi in `confronto_ai_povo_d070.csv`.

Ripete l'esperimento di `confronto_ai_povo_d064.md` sulla stessa planimetria, con **popolazione
maggiorata del 10%** e **50 semi invece di 30**. L'etichetta dei semi e invariata, quindi i primi 30
sono gli stessi della serie precedente e le due densita si confrontano in modo appaiato.

## Configurazione

| parametro | valore |
|---|---|
| mappa | `povo`, 44x73 celle da 0,5 m = 22 x 36,5 m, 499 m2 calpestabili |
| popolazione | 72 persone, 66 veloci, 116 ferme, 27 gruppi (~349 individui) |
| densita | **0,70 pers/m2** (0,64 nella serie precedente) |
| tragitto | fisso, angolo alto-sinistro -> angolo basso-destro |
| condizioni | `base` (solo Kalman) contro `solo_ai` (Kalman + correzione neurale) |
| controllo di velocita | spento in entrambe |
| geometria | robot 5,7 px, persone 7,0 px, sicurezza 10,1 px, lidar 123 px |
| raggio macchia | 3,0 celle = 1,5 m |
| accelerazione | 5x |
| ripetizioni | 50 semi appaiati, 100 corse |

## Risultati per seme

| seme | senza AI | con AI | differenza | variazione | fermo base | fermo AI |
|---|---|---|---|---|---|---|
| 2616969334 | 46.9 s | 47.7 s | -0.8 s | -1.6% | 5.1 s | 5.5 s |
| 3976001760 | 53.7 s | 61.7 s | -8.0 s | -14.9% | 8.2 s | 10.2 s |
| 1979033946 | 57.0 s | 61.2 s | -4.2 s | -7.5% | 11.1 s | 9.0 s |
| 49453516 | 50.1 s | 57.8 s | -7.8 s | -15.5% | 7.4 s | 9.8 s |
| 2627079279 | 53.1 s | 50.2 s | +2.8 s | +5.3% | 7.4 s | 6.3 s |
| 3952164089 | 45.8 s | 44.7 s | +1.1 s | +2.4% | 5.0 s | 3.8 s |
| 1922592067 | 46.5 s | 47.3 s | -0.8 s | -1.8% | 4.0 s | 5.6 s |
| 94330325 | 52.0 s | 56.1 s | -4.1 s | -7.9% | 7.8 s | 12.5 s |
| 2501918788 | 55.1 s | 56.1 s | -1.0 s | -1.8% | 9.3 s | 9.2 s |
| 3794235602 | 47.1 s | 48.1 s | -1.0 s | -2.1% | 5.7 s | 5.6 s |
| 1413333409 | 54.2 s | 59.1 s | -4.8 s | -8.9% | 8.9 s | 12.8 s |
| 591065399 | 56.8 s | 55.4 s | +1.3 s | +2.3% | 8.8 s | 10.1 s |
| 3123945613 | 51.3 s | 59.8 s | -8.5 s | -16.6% | 8.2 s | 15.0 s |
| 3442774043 | 50.8 s | 51.2 s | -0.4 s | -0.8% | 7.7 s | 5.8 s |
| 1397753272 | 48.2 s | 51.1 s | -2.8 s | -5.9% | 5.2 s | 5.7 s |
| 609695022 | 56.8 s | 52.5 s | +4.3 s | +7.6% | 9.8 s | 7.8 s |
| 3177079956 | 52.1 s | 48.6 s | +3.5 s | +6.7% | 5.8 s | 3.6 s |
| 3394851842 | 56.2 s | 54.0 s | +2.2 s | +3.9% | 10.7 s | 9.2 s |
| 1525041555 | 51.9 s | 52.7 s | -0.8 s | -1.4% | 7.4 s | 8.0 s |
| 769751301 | 48.8 s | 48.4 s | +0.4 s | +0.9% | 4.8 s | 5.8 s |
| 2131792482 | 62.8 s | 56.1 s | +6.8 s | +10.7% | 17.3 s | 9.8 s |
| 135766772 | 47.7 s | 48.1 s | -0.4 s | -0.9% | 5.2 s | 4.2 s |
| 2434724686 | 60.9 s | 52.2 s | +8.7 s | +14.2% | 10.1 s | 6.8 s |
| 3860448216 | 51.0 s | 49.7 s | +1.3 s | +2.6% | 6.2 s | 5.5 s |
| 2021480059 | 60.1 s | 54.1 s | +6.0 s | +10.0% | 14.8 s | 10.2 s |
| 259679981 | 53.8 s | 52.1 s | +1.8 s | +3.3% | 10.5 s | 9.3 s |
| 2524133207 | 67.4 s | 55.9 s | +11.5 s | +17.1% | 22.9 s | 10.7 s |
| 3782477761 | 52.1 s | 52.1 s | +0.0 s | +0.0% | 4.6 s | 4.6 s |
| 1909135952 | 52.6 s | 57.4 s | -4.8 s | -9.2% | 5.9 s | 7.4 s |
| 114043590 | 61.6 s | 61.5 s | +0.1 s | +0.1% | 14.1 s | 13.9 s |
| 1712038691 | 54.8 s | 56.2 s | -1.3 s | -2.4% | 8.1 s | 7.2 s |
| 286036917 | 53.4 s | 49.4 s | +4.0 s | +7.5% | 9.5 s | 5.3 s |
| 2282078735 | 51.1 s | 50.3 s | +0.8 s | +1.5% | 7.6 s | 5.6 s |
| 4278383257 | 47.2 s | 47.6 s | -0.3 s | -0.7% | 5.4 s | 5.2 s |
| 1634101050 | 49.7 s | 46.3 s | +3.3 s | +6.7% | 6.5 s | 3.5 s |
| 375478188 | 53.0 s | 53.0 s | +0.0 s | +0.0% | 7.4 s | 7.2 s |
| 2405959190 | 58.5 s | 55.8 s | +2.8 s | +4.7% | 10.4 s | 7.8 s |
| 4168038016 | 54.1 s | 60.3 s | -6.2 s | -11.6% | 5.8 s | 8.2 s |
| 1758472977 | 52.5 s | 61.2 s | -8.8 s | -16.7% | 9.1 s | 19.3 s |
| 534190983 | 58.5 s | 52.5 s | +6.0 s | +10.3% | 14.7 s | 10.1 s |
| 692729316 | 59.9 s | 48.1 s | +11.8 s | +19.7% | 15.8 s | 7.8 s |
| 1582105970 | 48.3 s | 50.4 s | -2.1 s | -4.3% | 4.6 s | 4.8 s |
| 3343144136 | 53.0 s | 49.0 s | +4.0 s | +7.5% | 8.8 s | 5.6 s |
| 2957206622 | 49.0 s | 55.5 s | -6.5 s | -13.3% | 5.5 s | 12.0 s |
| 774369789 | 50.1 s | 63.4 s | -13.3 s | -26.6% | 4.2 s | 11.2 s |
| 1495318891 | 47.8 s | 47.7 s | +0.1 s | +0.2% | 3.7 s | 3.7 s |
| 3223949521 | 53.8 s | 53.1 s | +0.8 s | +1.4% | 9.0 s | 8.5 s |
| 3073286215 | 53.3 s | 56.4 s | -3.1 s | -5.8% | 7.1 s | 11.2 s |
| 663862742 | 50.3 s | 48.1 s | +2.2 s | +4.5% | 7.3 s | 6.6 s |
| 1352043840 | 52.9 s | 56.4 s | -3.5 s | -6.6% | 9.2 s | 12.8 s |
| **media** | **53.11 s** | **53.27 s** | **-0.16 s** | **-0.7%** | **8.39 s** | **8.15 s** |

## Sintesi

- differenza media **-0.16 s (-0.7%)**, mediana **+0.00 s**;
- **24 coppie vinte su 50** dall'AI: praticamente un lancio di moneta;
- errore standard 0.71 s, rapporto media/errore **0.22**;
- intervallo di confidenza al 95%: da **-1.54 s a +1.23 s**, cioe da **-2.9% a +2.3%**;
- tempo fermo: 8.39 s contro 8.15 s (15.8% contro 15.3% del tempo);
- tutte e 50 le coppie valide.

E la misura piu pulita della serie: mediana esattamente nulla, vittorie sotto la meta, e
l'intervallo di confidenza piu stretto ottenuto finora.

## Confronto appaiato fra le due densita (30 semi comuni)

| quantita | 0,70 meno 0,64 | errore | rapporto |
|---|---|---|---|
| vantaggio dell'AI | -0,63 s | 1,31 s | 0,48 |
| tempo del robot base | +1,22 s | 1,23 s | 0,99 |
| **tempo fermo del robot base** | **+1,62 s** | **0,81 s** | **2,00** |

Questa tabella e il controllo positivo dell'esperimento. Aumentare del 10% la densita **produce un
effetto misurabile e statisticamente solido** sul tempo che il robot passa fermo (rapporto 2,00):
la manipolazione ha davvero cambiato la scena, e il disegno sperimentale ha la sensibilita per
accorgersene. Sullo stesso campione, il vantaggio dell'AI **non cambia affatto** (rapporto 0,48).

Non si tratta quindi di un test troppo debole per vedere alcunche: vede l'effetto della densita e
non vede quello dell'AI.

## Cosa dice sulla misura precedente

Nella serie a 0,64 la differenza media era +0,68 s, interamente prodotta da un singolo seme
patologico. Qui, con densita maggiore e 50 semi, la media e **-0.16 s** e la mediana e **+0.00 s**:
quel +0,64% era rumore, come gia sospettato togliendo l'outlier.

## Confronto con l'intera serie

| | mappa generata | Povo 0,64 | Povo 0,70 |
|---|---|---|---|
| semi | 20 | 30 | **50** |
| differenza media | +1,52 s (+2,1%) | +0,68 s (+0,6%) | **-0.16 s (-0.7%)** |
| rapporto media/errore | 1,26 | 0,63 | **0.22** |
| coppie vinte | 13/20 | 17/30 | **24/50** |
| IC95% sulla variazione | -1,5% .. +6,2% | -2,7% .. +5,3% | **-2.9% .. +2.3%** |

## Conclusione

Quattro condizioni indipendenti - mappa generata, raggio della macchia dimezzato, planimetria reale,
densita maggiorata - e in nessuna la correzione neurale produce un effetto rilevabile sui tempi di
percorrenza, nonostante riduca del 12,1% l'errore di previsione.

Il limite superiore dichiarabile passa da +6,2% a **+2,3%**: se un effetto esiste, e piu piccolo di
questo.

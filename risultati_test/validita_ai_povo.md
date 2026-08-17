# Validita' della correzione AI sulla planimetria reale di Povo

Misura del 14 agosto 2026. Dataset in `ai_predittiva2/dataset/dataset_previsione_povo_5x.npz`.

## A cosa risponde

I test sui tempi su Povo sono stati fatti con il modello **specializzato su `training_mappa_01`**
(`testing.py` usa quello ogni volta che l'accelerazione e maggiore di 1). Su Povo quel modello e
fuori distribuzione, e due dei tre punti di densita provati stavano anche oltre il suo intervallo di
addestramento. Restava quindi aperta l'obiezione: *il risultato nullo sui tempi dipende dal fatto che
il modello non funzionava su quella mappa?*

## Il dataset di verifica

| parametro | valore |
|---|---|
| mappa | `povo`, 44x73 celle da 0,5 m, 499 m2 calpestabili |
| livelli di densita | 8, da 50 a 750 individui (0,10 - 1,50 pers/m2) |
| tetto | 650 individui = **1,30 pers/m2**, affollamento massimo da DM 26/8/1992 (25 aule x 26 persone); si campiona fino a 750 per non estrapolare al picco |
| composizione | 20% normali, 17% corridori, **34% ferme**, 29% membri di gruppo |
| scenari | 8 livelli x 10 ripetizioni = 80 |
| campioni | **160.189** |
| generazione | 36 minuti su 8 processi |

## Risultato

Modello valutato: `correzione_kalman_specializzato.pt`, addestrato su `training_mappa_01`
fra 50 e 550 individui (0,056 - 0,62 pers/m2).

| | campioni | Kalman | + AI | errore medio | errore mediano |
|---|---|---|---|---|---|
| mappa di training, **soli scenari di test** | 23.590 | 0,599 m | 0,527 m | **-12,1%** | -16,2% |
| **Povo, mappa mai vista** | 160.189 | 0,505 m | 0,442 m | **-12,5%** | -12,8% |

**La correzione non degrada.** Generalizza alla planimetria reale come generalizza a scenari mai visti
sulla propria mappa.

> Nota metodologica: valutando sull'INTERO dataset di addestramento si ottiene -37,6%, ma quel numero
> comprende gli scenari usati per allenare e non va mai citato. Il confronto onesto usa la sola
> partizione di test, separata per scenario.

## Stabilita rispetto alla densita

| pers/m2 | campioni | variazione dell'errore medio |
|---|---|---|
| 0,10 | 1.743 | -10,9% |
| 0,30 | 6.134 | -3,8% |
| 0,50 | 9.570 | -10,8% |
| 0,70 | 16.223 | -13,1% |
| 0,90 | 22.430 | -12,9% |
| 1,10 | 33.024 | -14,0% |
| **1,30** (picco normativo) | 33.998 | -11,2% |
| 1,50 | 37.067 | -13,7% |

Il modello e stato addestrato fino a 0,62 pers/m2. A 1,30 e 1,50 - oltre il doppio - la correzione
mantiene il vantaggio senza cedimenti: estrapola bene sulla densita.

## Conclusione

Il riaddestramento su Povo **non e necessario**: era motivato dall'ipotesi di un modello degradato, e
l'ipotesi e falsa.

L'obiezione che i test sui tempi fossero invalidati da un modello fuori posto non e piu disponibile:

> Sulla planimetria reale la correzione riduce del 12,5% l'errore di previsione, in linea con il 12,1%
> misurato sugli scenari di test della mappa di addestramento, e mantiene il vantaggio fino a
> 1,5 pers/m2. Nelle stesse condizioni l'effetto sui tempi di percorrenza resta compreso fra -1,5% e
> +1,2% (130 semi appaiati, vedi confronto_ai_povo_d051/d064/d070).

Previsione genuinamente migliore, tempi identici: e il trasferimento fra metrica di componente e
metrica di sistema a non avvenire.

## Dettaglio collaterale

Il Kalman su Povo sbaglia **meno** che sulla mappa generata (0,505 contro 0,599 m). Corridoi stretti
danno traiettorie piu vincolate, quindi piu prevedibili - e meno margine a un correttore.

# Analisi

Script diagnostici che hanno prodotto i numeri citati nella tesi. Non fanno parte del simulatore: si
lanciano a mano quando serve rifare una misura o verificarne una.

Si eseguono dalla cartella del progetto:

```
py analisi/misura_tetto.py
py analisi/verifica_ai_nel_loop.py
py analisi/profila_testing.py [popolazione] [condizione]
```

## `misura_tetto.py`

Quanto tempo il robot perde fermo per lo stop di sicurezza, sul percorso angolo→angolo con la
configurazione salvata nel pannello. È il **tetto** per qualunque controllo che voglia recuperare quel
tempo: non si può guadagnare più di ciò che si perde. Confronta anche il controllo predittivo di
velocità attuale contro il robot base, sugli stessi seed.

**Risultato (10 corse, ~490 individui, mappa universitaria):**

| condizione | tempo medio | tempo fermo | % fermo |
|---|---|---|---|
| base | 63.1 s | 7.9 s | **12.5%** |
| solo_velocita | 76.2 s | 6.4 s | 8.4% |

Il controllo di velocità riduce le fermate di 1.5 s ma allunga il percorso di 13.1 s: **paga circa 9
secondi per ogni secondo di fermata evitata**, e perde in 10 confronti appaiati su 10. Il baratto è in
perdita perché in questo modello fermarsi costa poco (il robot riparte istantaneamente a velocità piena)
mentre il rallentamento agisce su tutto il tragitto.

## `verifica_ai_nel_loop.py`

Verifica che la correzione AI funzioni **dentro** il ciclo di simulazione e non solo sul dataset. Il
+12.2% riportato dal training è una misura ad anello aperto su dati registrati; qui la stessa misura è
rifatta mentre il robot naviga davvero, confrontando previsione di Kalman e previsione corretta con la
posizione vera raggiunta dalla persona.

**Risultato (981 previsioni verificate):**

| | medio | mediano | 90° perc. |
|---|---|---|---|
| Kalman solo | 42.3 px | 32.7 px | 86.9 px |
| Kalman + AI | 38.4 px | 26.2 px | 85.0 px |

Miglioramento nel loop **+9.2%**, AI più vicina alla verità nel 57% dei casi. Il modello funziona: non
c'è nessun errore di applicazione. Ma il guadagno è concentrato nella **mediana** (+20%) e assente nella
**coda** (+2%) — e sono gli errori grandi a causare gli stop del robot.

## `profila_testing.py`

Dove va il tempo di calcolo. Il **97%** sta in `algoritmo_a_star`, e di quello il 96% in
`scorciatoia_libera` (il controllo di visibilità del Theta*, ~21 milioni di chiamate in 300 frame).

È il vincolo che ha determinato la risoluzione della griglia: a 30px per cella il pathfinding è già il
collo di bottiglia, e una griglia quattro volte più fine moltiplicherebbe per 16 le celle da esplorare.
Quella risoluzione grossolana è a sua volta ciò che assorbe il guadagno di precisione dell'AI di
predizione — vedi `ai_predittiva/__init__.py`.

"""AI PREDITTIVA (Livello 2): correzione neurale dell'errore del filtro di Kalman.

Raccoglie tutto il filone della previsione: generazione del dataset, allenamento del modello, modelli e
dataset salvati. Il modello impara il RESIDUO del Kalman (dove la persona si trovera' davvero meno dove il
Kalman prevede che sara') e la correzione viene sommata alla previsione, che alimenta la macchia di
probabilita' usata come costo extra dall'A* del robot.

STATO: implementato e misurato, ma DISATTIVATO di default nel simulatore (spunta "AI di predizione" nel
pannello tecnico). Il motivo e' documentato dalle misure, non e' un abbandono:

  - la previsione migliora davvero: -12.2% di errore medio sul dataset, -9.2% misurato dentro il ciclo di
    simulazione con il robot in movimento (quindi non e' un artefatto di valutazione offline);
  - ma il tempo di percorrenza NON migliora, e in alcune configurazioni peggiora.

Le cause misurate sono quattro, e valgono come criterio generale per decidere se conviene aggiungere un
modello predittivo a un sistema che gia' funziona:

  1. il guadagno viene assorbito dalla rappresentazione a valle: la macchia di probabilita' ha raggio 90px
     mentre l'incertezza reale della previsione e' ~38px, quindi spostarne il centro di pochi pixel non
     cambia quali celle l'A' considera rischiose;
  2. il miglioramento e' concentrato dove non serve: mediana -20%, ma 90-esimo percentile solo -2%. Sono
     gli errori grandi a causare gli stop, e quelli restano;
  3. il modello e' cieco al contesto: riceve solo (x, y, vx, vy) di UNA persona alla volta, mentre nella
     simulazione le deviazioni brusche nascono dalla repulsione fra vicini e dal robot stesso - sta
     prevedendo effetti di cui non vede le cause (nella letteratura la risposta e' il social pooling);
  4. il sistema base non paga il ritardo: il robot si ferma e riparte istantaneamente, quindi reagire
     tardi costa poco e anticipare vale poco. Il tempo perso fermo e' l'8.3%: quello e' il tetto massimo
     per qualunque miglioramento in questa direzione.

Uso (dalla cartella del progetto, come moduli):
    py -m ai_predittiva.genera_dataset_previsione --specializzato
    py -m ai_predittiva.allena_previsione --specializzato
"""

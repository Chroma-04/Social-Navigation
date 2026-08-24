# -*- coding: utf-8 -*-
"""Costruisce la presentazione della discussione a partire da questo file.

Lo scopo e' che la scaletta stia nel codice e non solo nel .pptx: se una slide va
ripensata si modifica qui e si rilancia, invece di ritoccare a mano e perdere la
struttura. Il .pptx prodotto resta comunque un file normale, da rifinire in
PowerPoint per tutto cio' che riguarda l'estetica fine.

    py presentazione/genera_presentazione.py

Impianto: cinque blocchi da dodici minuti complessivi, con il blocco del risultato
tenuto separato da quello dell'ibridazione — e' il contributo del lavoro e non deve
essere una coda del metodo. Le note di ciascuna slide riportano il tempo assegnato.
"""
import os
import copy

from pptx import Presentation
from pptx.util import Cm, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

CARTELLA = os.path.dirname(os.path.abspath(__file__))
FIGURE = os.path.join(CARTELLA, "figure")
USCITA = os.path.join(CARTELLA, "Discussione_Sabadas.pptx")

# --- misure della slide: 16:9 ---
L_SLIDE = Cm(33.867)
H_SLIDE = Cm(19.05)
MARGINE = Cm(2.0)
COLONNA_VIDEO_X = Cm(22.57)     # la colonna del video occupa il terzo destro
COLONNA_VIDEO_L = Cm(11.297)

# --- colori: fondo chiaro, un solo accento ---
TESTO = RGBColor(0x26, 0x26, 0x26)
TENUE = RGBColor(0x82, 0x82, 0x82)
ACCENTO = RGBColor(0x9B, 0x1E, 0x2D)
FILETTO = RGBColor(0xD7, 0xD7, 0xD7)
BIANCO = RGBColor(0xFF, 0xFF, 0xFF)
FONDINO = RGBColor(0xF4, 0xF2, 0xF0)

FONT_TITOLO = "Georgia"
FONT_CORPO = "Calibri"


# ----------------------------------------------------------------------------
# utilita' di disegno
# ----------------------------------------------------------------------------

def testo(slide, x, y, l, h, contenuto, dimensione=18, colore=TESTO, font=FONT_CORPO,
          grassetto=False, corsivo=False, allineamento=PP_ALIGN.LEFT, interlinea=1.15,
          spazio_dopo=6):
    """Una casella di testo semplice. `contenuto` puo' essere una stringa o una lista
    di righe, ciascuna delle quali diventa un paragrafo."""
    casella = slide.shapes.add_textbox(x, y, l, h)
    cornice = casella.text_frame
    cornice.word_wrap = True
    cornice.margin_left = cornice.margin_right = 0
    cornice.margin_top = cornice.margin_bottom = 0

    righe = [contenuto] if isinstance(contenuto, str) else list(contenuto)
    # un a capo esplicito diventa un paragrafo a se': PowerPoint non rende in modo
    # affidabile il carattere di nuova riga lasciato dentro un singolo tratto
    righe = [pezzo for riga in righe for pezzo in str(riga).split("\n")]
    for indice, riga in enumerate(righe):
        paragrafo = cornice.paragraphs[0] if indice == 0 else cornice.add_paragraph()
        paragrafo.alignment = allineamento
        paragrafo.line_spacing = interlinea
        paragrafo.space_after = Pt(spazio_dopo)
        tratto = paragrafo.add_run()
        tratto.text = riga
        tratto.font.size = Pt(dimensione)
        tratto.font.color.rgb = colore
        tratto.font.name = font
        tratto.font.bold = grassetto
        tratto.font.italic = corsivo
    return casella


def rettangolo(slide, x, y, l, h, riempimento=None, bordo=None, spessore=1.0):
    forma = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, x, y, l, h)
    if riempimento is None:
        forma.fill.background()
    else:
        forma.fill.solid()
        forma.fill.fore_color.rgb = riempimento
    if bordo is None:
        forma.line.fill.background()
    else:
        forma.line.color.rgb = bordo
        forma.line.width = Pt(spessore)
    forma.shadow.inherit = False
    return forma


def elenco(slide, x, y, l, voci, dimensione=19, passo=1.35):
    """Elenco senza pallini: un trattino d'accento e la riga. Poche voci, righe corte."""
    for indice, voce in enumerate(voci):
        cima = y + Cm(passo * indice)
        segno = rettangolo(slide, x, cima + Cm(0.28), Cm(0.42), Cm(0.07), riempimento=ACCENTO)
        segno.shadow.inherit = False
        testo(slide, x + Cm(0.85), cima, l - Cm(0.85), Cm(1.2), voce, dimensione=dimensione)


def note(slide, contenuto):
    slide.notes_slide.notes_text_frame.text = contenuto


def slide_vuota(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])   # layout "Vuota"


def intestazione(slide, titolo, blocco, numero, tempo):
    """Titolo della slide, filetto d'accento, piede. Il titolo enuncia l'affermazione:
    chi legge solo i titoli deve poter ricostruire la tesi."""
    testo(slide, MARGINE, Cm(1.25), L_SLIDE - 2 * MARGINE, Cm(2.2), titolo,
          dimensione=26, font=FONT_TITOLO, colore=TESTO, interlinea=1.05, spazio_dopo=0)
    rettangolo(slide, MARGINE, Cm(3.35), Cm(3.2), Cm(0.09), riempimento=ACCENTO)
    testo(slide, MARGINE, H_SLIDE - Cm(1.55), Cm(20), Cm(0.8), blocco,
          dimensione=10.5, colore=TENUE)
    testo(slide, L_SLIDE - MARGINE - Cm(4), H_SLIDE - Cm(1.55), Cm(4), Cm(0.8),
          "%d  ·  %s" % (numero, tempo), dimensione=10.5, colore=TENUE,
          allineamento=PP_ALIGN.RIGHT)


def figura(slide, nome, x, y, l=None, h=None):
    percorso = os.path.join(FIGURE, nome)
    if not os.path.exists(percorso):
        rettangolo(slide, x, y, l or Cm(10), h or Cm(8), riempimento=FONDINO, bordo=FILETTO)
        testo(slide, x, (y + (h or Cm(8)) / 2), l or Cm(10), Cm(1),
              "figura mancante: %s" % nome, dimensione=12, colore=TENUE,
              allineamento=PP_ALIGN.CENTER)
        return None
    return slide.shapes.add_picture(percorso, x, y, width=l, height=h)


def tabella(slide, x, y, l, righe, larghezze, dimensione=16, intestata=True):
    """Tabella minima: nessuna griglia, una riga d'accento sotto l'intestazione."""
    n_righe, n_colonne = len(righe), len(righe[0])
    altezza_riga = Cm(0.95)
    forma = slide.shapes.add_table(n_righe, n_colonne, x, y, l, altezza_riga * n_righe)
    tab = forma.table
    for indice, frazione in enumerate(larghezze):
        tab.columns[indice].width = Emu(int(l * frazione))

    for i, riga in enumerate(righe):
        tab.rows[i].height = altezza_riga
        for j, valore in enumerate(riga):
            cella = tab.cell(i, j)
            cella.text = ""
            cella.fill.solid()
            cella.fill.fore_color.rgb = BIANCO
            cella.margin_left = Cm(0.15)
            cella.margin_right = Cm(0.15)
            cella.margin_top = cella.margin_bottom = Cm(0.1)
            cella.vertical_anchor = MSO_ANCHOR.MIDDLE
            paragrafo = cella.text_frame.paragraphs[0]
            paragrafo.alignment = PP_ALIGN.LEFT if j == 0 else PP_ALIGN.RIGHT
            tratto = paragrafo.add_run()
            tratto.text = str(valore)
            tratto.font.size = Pt(dimensione)
            tratto.font.name = FONT_CORPO
            capofila = intestata and i == 0
            tratto.font.color.rgb = TENUE if capofila else TESTO
            tratto.font.bold = capofila
    return forma


# ----------------------------------------------------------------------------
# le slide
# ----------------------------------------------------------------------------

def slide_titolo(prs):
    slide = slide_vuota(prs)

    # colonna destra: qui va il video della simulazione, registrato con il tasto V.
    # Il segnaposto ha gia' misura e posizione definitive: si cancella e si inserisce
    # il video nello stesso riquadro
    rettangolo(slide, COLONNA_VIDEO_X, 0, COLONNA_VIDEO_L, H_SLIDE, riempimento=FONDINO)
    testo(slide, COLONNA_VIDEO_X + Cm(1), Cm(8.2), COLONNA_VIDEO_L - Cm(2), Cm(3),
          ["SEGNAPOSTO VIDEO",
           "Cancella questo riquadro e inserisci qui la registrazione della",
           "simulazione su Povo (Inserisci → Video → Questo dispositivo).",
           "Stessa posizione e misura: 22,57 / 0 / 11,3 × 19,05 cm."],
          dimensione=11, colore=TENUE, allineamento=PP_ALIGN.CENTER, spazio_dopo=3)
    rettangolo(slide, COLONNA_VIDEO_X, 0, Cm(0.035), H_SLIDE, riempimento=FILETTO)

    larghezza_testo = COLONNA_VIDEO_X - MARGINE - Cm(1.6)
    testo(slide, MARGINE, Cm(2.2), larghezza_testo, Cm(2),
          ["Università degli Studi di Trento",
           "Dipartimento di Ingegneria Industriale — Meccatronica"],
          dimensione=12, colore=TENUE, spazio_dopo=2)

    testo(slide, MARGINE, Cm(6.1), larghezza_testo, Cm(3.4),
          "Algoritmi di navigazione sociale",
          dimensione=38, font=FONT_TITOLO, colore=TESTO, interlinea=1.05, spazio_dopo=0)
    rettangolo(slide, MARGINE, Cm(9.85), Cm(3.2), Cm(0.09), riempimento=ACCENTO)
    testo(slide, MARGINE, Cm(10.5), larghezza_testo, Cm(2.4),
          ["Vantaggi, limiti e metodi dell'ibridazione",
           "di un sistema di pianificazione classico"],
          dimensione=18, colore=TESTO, corsivo=True, spazio_dopo=2)

    testo(slide, MARGINE, Cm(15.0), Cm(9), Cm(2),
          ["Relatore", "Prof. Daniele Fontanelli"], dimensione=13, colore=TESTO, spazio_dopo=1)
    testo(slide, MARGINE + Cm(9.5), Cm(15.0), Cm(9), Cm(2),
          ["Laureando", "Nicola Sabadas"], dimensione=13, colore=TESTO, spazio_dopo=1)

    note(slide, "0:15 — Non leggere la slide. Un saluto e passa oltre: il video "
                "lavora da solo mentre parli.")
    return slide


def slide_problema(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Un robot fra le persone non ha ostacoli fermi:\n"
                        "ha ostacoli che si muovono e che lui stesso influenza",
                 "1 · Il problema", 2, "1:00")
    elenco(slide, MARGINE, Cm(5.4), Cm(18.5), [
        "Il percorso ottimo di adesso può essere impercorribile fra mezzo secondo",
        "Il criterio smette di essere geometrico e diventa relazionale",
        "Tre strategie in letteratura: prevedere, rallentare, scansarsi",
    ])
    figura(slide, "povo.png", Cm(22.4), Cm(4.6), h=Cm(12.4))
    testo(slide, Cm(22.4), Cm(17.3), Cm(9.5), Cm(1),
          "Povo 1, piano secondo — l'ambiente misurato", dimensione=11, colore=TENUE)
    note(slide, "1:00 — Il punto da lasciare: l'ostacolo reagisce alla presenza del robot. "
                "Nomina le tre strategie perché tornano dopo come i tre punti di innesto.")
    return slide


def slide_domanda(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "La domanda, in forma verificabile",
                 "1 · Il problema", 3, "0:45")
    rettangolo(slide, MARGINE, Cm(5.2), L_SLIDE - 2 * MARGINE, Cm(3.4), riempimento=FONDINO)
    rettangolo(slide, MARGINE, Cm(5.2), Cm(0.12), Cm(3.4), riempimento=ACCENTO)
    testo(slide, MARGINE + Cm(0.9), Cm(5.9), L_SLIDE - 2 * MARGINE - Cm(1.8), Cm(2.4),
          "Una previsione più accurata del moto delle persone\n"
          "migliora il comportamento di un pianificatore già funzionante?",
          dimensione=23, font=FONT_TITOLO, interlinea=1.25)
    elenco(slide, MARGINE, Cm(10.2), Cm(29), [
        "Fra il modello e il comportamento stanno una mappa di costo, un pianificatore, "
        "una legge di controllo",
        "Ciascun passaggio può assorbire il guadagno prima che diventi azione",
        "L'ipotesi è presa come cosa da misurare, non come premessa",
    ])
    note(slide, "0:45 — Slide decisiva. Se la domanda è falsificabile, il risultato negativo "
                "è una risposta e non un fallimento. Insisti su «verificabile».")
    return slide


def slide_theta(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Pianificazione: Theta*, variante any-angle di A*",
                 "2 · Il sistema", 4, "0:45")
    elenco(slide, MARGINE, Cm(5.2), Cm(29), [
        "Un nodo si collega al nonno quando fra i due c'è linea di vista libera",
        "I percorsi non sono più vincolati agli otto multipli di 45°",
        "Il controllo di visibilità assorbe la quasi totalità del tempo di pianificazione",
        "Deterministico: requisito dei confronti appaiati usati in tutta la validazione",
    ])
    note(slide, "0:45 — Non spiegare A*: la commissione lo conosce. Di' cosa cambia Theta* "
                "e che il costo sta tutto nel controllo di visibilità, perché serve dopo.")
    return slide


def slide_rischio(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "La previsione diventa una regione di rischio,\n"
                        "non un punto",
                 "2 · Il sistema", 5, "0:50")
    elenco(slide, MARGINE, Cm(5.8), Cm(18.5), [
        "Kalman a velocità costante, proiezione a un secondo",
        "La macchia è propagata nello spazio libero: i muri la interrompono",
        "Anisotropa — si allunga lungo la direzione di marcia",
        "Convertita in costo additivo per il pianificatore",
    ])
    rettangolo(slide, Cm(21.8), Cm(5.6), Cm(10.1), Cm(8.4), riempimento=FONDINO, bordo=FILETTO)
    testo(slide, Cm(22.4), Cm(9.0), Cm(8.9), Cm(2),
          ["Qui una schermata della macchia",
           "(persona in movimento, Figura 2b della tesi)"],
          dimensione=12, colore=TENUE, allineamento=PP_ALIGN.CENTER, spazio_dopo=3)
    note(slide, "0:50 — La differenza dalle gaussiane della letteratura è che la macchia è "
                "propagata e non analitica. È il punto originale del capitolo 3: dillo.")
    return slide


def slide_percezione(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Il robot ragiona solo su ciò che vede,\n"
                        "e si ferma su un vincolo rigido",
                 "2 · Il sistema", 6, "0:40")
    elenco(slide, MARGINE, Cm(5.8), Cm(29), [
        "Lidar simulato: portata finita, occlusione dietro i muri, rumore sulla posizione",
        "Sicurezza e comfort separati per costruzione: un vincolo binario e un costo morbido",
        "Il vincolo rigido è ancorato a IEC 61496-1 e ISO 3691-4, il comfort alle zone di Hall",
    ])
    rettangolo(slide, MARGINE, Cm(11.4), L_SLIDE - 2 * MARGINE, Cm(2.9), riempimento=FONDINO)
    testo(slide, MARGINE + Cm(0.9), Cm(12.1), L_SLIDE - 2 * MARGINE - Cm(1.8), Cm(1.8),
          "I confini fra zone prossemiche distano decine di centimetri: un guadagno di pochi "
          "centimetri, per quanto misurabile, non sposta il robot da una zona all'altra.",
          dimensione=16, corsivo=True, interlinea=1.2)
    note(slide, "0:40 — Le normative in una riga sola, non svilupparle. Il riquadro in basso "
                "è il seme del risultato: la scala su cui un miglioramento sarebbe percepibile.")
    return slide


def slide_ambiente(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "L'ambiente e la folla sono misurati, non plausibili",
                 "2 · Il sistema", 7, "0:45")
    tabella(slide, MARGINE, Cm(5.2), Cm(18.0), [
        ["", "valore"],
        ["Planimetria", "Povo 1, piano secondo"],
        ["Superficie calpestabile", "1997 m² su 3212 lordi"],
        ["Densità di riferimento", "17,5 persone / 100 m²"],
        ["Ancoraggio normativo", "D.M. 26/8/1992 → 31,2 max"],
        ["Classi di pedoni", "4, proporzioni da picco reale"],
    ], [0.55, 0.45])
    figura(slide, "povo.png", Cm(22.4), Cm(4.6), h=Cm(12.4))
    note(slide, "0:45 — Il messaggio è che la densità non è inventata: viene da una norma "
                "antincendio, ed è poco più della metà del massimo ammesso.")
    return slide


def slide_innesti(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Tre punti di innesto indipendenti,\n"
                        "misurati uno alla volta",
                 "3 · L'ibridazione", 8, "0:55")
    colonne = [
        ("SPAZIO", "Regione di rischio", "dove il robot passa", "il pianificatore consuma la macchia come costo"),
        ("TEMPO", "Controllo di velocità", "quando vi passa", "confronta l'istante d'arrivo con la posizione prevista"),
        ("POSIZIONE", "Controllo di posizione", "dove sta lungo il percorso", "traslazione laterale entro un intorno"),
    ]
    larghezza = Cm(9.3)
    for indice, (etichetta, nome, sotto, riga) in enumerate(colonne):
        x = MARGINE + Cm(10.0) * indice
        rettangolo(slide, x, Cm(5.2), larghezza, Cm(0.12), riempimento=ACCENTO)
        testo(slide, x, Cm(5.7), larghezza, Cm(0.8), etichetta, dimensione=11, colore=ACCENTO,
              grassetto=True)
        testo(slide, x, Cm(6.6), larghezza, Cm(1.6), nome, dimensione=19, font=FONT_TITOLO)
        testo(slide, x, Cm(8.3), larghezza, Cm(1.0), sotto, dimensione=15, corsivo=True,
              colore=TENUE)
        testo(slide, x, Cm(9.5), larghezza, Cm(3.0), riga, dimensione=14, interlinea=1.2)
    testo(slide, MARGINE, Cm(14.2), L_SLIDE - 2 * MARGINE, Cm(2),
          "Consumano la stessa previsione attraverso punti distinti nel codice: è questa "
          "separazione a consentire di attribuire a ciascuno, separatamente, l'effetto misurato.",
          dimensione=15, colore=TENUE, interlinea=1.25)
    note(slide, "0:55 — Sono le tre strategie nominate nella slide 2. Chiudi il cerchio "
                "esplicitamente: «le tre strategie di prima, realizzate».")
    return slide


def slide_rete(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "La rete corregge il residuo del filtro,\n"
                        "non sostituisce nulla",
                 "3 · L'ibridazione", 9, "0:40")
    elenco(slide, MARGINE, Cm(5.8), Cm(18.5), [
        "GRU 48×2, poco meno di ventiduemila parametri",
        "Ingresso: trenta stime di stato filtrate",
        "Uscita: lo scarto fra previsione del Kalman e posizione vera",
        "Etichette lette a tempo debito dalla simulazione, non estrapolate",
    ])
    rettangolo(slide, Cm(21.8), Cm(5.6), Cm(10.1), Cm(6.2), riempimento=FONDINO)
    testo(slide, Cm(22.6), Cm(6.4), Cm(8.5), Cm(5),
          ["111.347 campioni", "110 esecuzioni indipendenti",
           "separate per scenario, non per campione"],
          dimensione=15, interlinea=1.3, spazio_dopo=8)
    note(slide, "0:40 — Insisti su «residuo»: alla rete non si chiede di prevedere il moto, "
                "solo di stimare quanto il modello analitico stia sbagliando.")
    return slide


def slide_metodo(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Come si misura: confronti appaiati sugli stessi semi",
                 "3 · L'ibridazione", 10, "0:55")
    elenco(slide, MARGINE, Cm(5.2), Cm(29), [
        "Ogni coppia gira sulla stessa scena: stessa folla, stesse traiettorie, stesso rumore",
        "Criterio: media della differenza diviso il suo errore standard — sotto 1 nulla, sopra 2 stabilito",
        "Test dei segni sui livelli di densità come blocchi indipendenti",
        "Ogni effetto che superava la soglia è stato replicato su semi disgiunti",
    ])
    rettangolo(slide, MARGINE, Cm(13.2), L_SLIDE - 2 * MARGINE, Cm(2.9), riempimento=FONDINO)
    rettangolo(slide, MARGINE, Cm(13.2), Cm(0.12), Cm(2.9), riempimento=ACCENTO)
    testo(slide, MARGINE + Cm(0.9), Cm(13.9), L_SLIDE - 2 * MARGINE - Cm(1.8), Cm(1.8),
          "Cinque effetti apparentemente stabiliti sono caduti alla replica: senza questa "
          "disciplina, cinque affermazioni false sarebbero finite nella tesi.",
          dimensione=16, corsivo=True, interlinea=1.2)
    note(slide, "0:55 — Il riquadro è la slide più forte del blocco metodo. Dillo con calma: "
                "è ciò che distingue una misura da un aneddoto.")
    return slide


def slide_previsione(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "La previsione migliora, e il guadagno tiene\n"
                        "fuori dall'ambiente di addestramento",
                 "4 · Il risultato", 11, "0:45")
    tabella(slide, MARGINE, Cm(5.6), Cm(19.0), [
        ["errore a un secondo", "medio", "mediano", "90° perc."],
        ["Kalman", "1,20 m", "0,93 m", "2,62 m"],
        ["Kalman + rete", "0,95 m", "0,73 m", "2,12 m"],
        ["riduzione", "20,7%", "21,7%", "19,0%"],
    ], [0.40, 0.20, 0.20, 0.20], dimensione=17)
    elenco(slide, MARGINE, Cm(11.0), Cm(29), [
        "−20,2% su Povo, planimetria mai vista in addestramento",
        "Il guadagno si conserva su sette livelli di densità su otto",
    ])
    note(slide, "0:45 — Qui il risultato è positivo e va detto senza cautele: la componente "
                "appresa fa esattamente ciò per cui è stata addestrata.")
    return slide


def slide_comportamento(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Il comportamento del robot non cambia:\n"
                        "nessuno dei canali misurati si muove",
                 "4 · Il risultato", 12, "1:15")
    figura(slide, "confronto_quattro_modelli.png", Cm(3.4), Cm(4.5), l=Cm(20.5))
    testo(slide, Cm(24.8), Cm(5.2), Cm(7.3), Cm(11),
          ["Tempo di percorrenza",
           "invariato entro ±1,7%",
           "",
           "Tempo in arresto",
           "nessuna variazione",
           "",
           "Margine spaziale",
           "meno di due centimetri",
           "",
           "Fluidità del moto",
           "nessuna variazione"],
          dimensione=14, interlinea=1.15, spazio_dopo=1)
    note(slide, "1:15 — Non dire «non è cambiato niente». Mostra i canali e lascia che siano "
                "loro a vedere che sono fermi. Quattro verifiche indipendenti escludono le "
                "spiegazioni alternative: portata triplicata, previsione soppressa, ambiente "
                "aperto, densità.")
    return slide


def slide_dove_si_arresta(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Dove si arresta il guadagno",
                 "4 · Il risultato", 13, "1:30")

    anelli = ["previsione", "centro del rischio", "mappa di costo", "percorso"]
    larghezza, passo = Cm(6.4), Cm(7.5)
    for indice, anello in enumerate(anelli):
        x = MARGINE + passo * indice
        rettangolo(slide, x, Cm(5.3), larghezza, Cm(1.9), riempimento=FONDINO, bordo=FILETTO)
        testo(slide, x, Cm(5.85), larghezza, Cm(1), anello, dimensione=15,
              allineamento=PP_ALIGN.CENTER)
        if indice < len(anelli) - 1:
            testo(slide, x + larghezza, Cm(5.85), passo - larghezza, Cm(1), "→",
                  dimensione=16, colore=TENUE, allineamento=PP_ALIGN.CENTER)
    testo(slide, MARGINE, Cm(7.6), Cm(29), Cm(1),
          "tutti e quattro gli anelli reggono — i percorsi differiscono davvero",
          dimensione=13, colore=TENUE, corsivo=True)

    rettangolo(slide, MARGINE, Cm(9.8), L_SLIDE - 2 * MARGINE, Cm(4.0), riempimento=FONDINO)
    rettangolo(slide, MARGINE, Cm(9.8), Cm(0.12), Cm(4.0), riempimento=ACCENTO)
    testo(slide, MARGINE + Cm(0.9), Cm(10.4), L_SLIDE - 2 * MARGINE - Cm(1.8), Cm(3.2),
          ["La correzione sposta la previsione di 76 cm, ma la avvicina al vero di 25.",
           "Le decisioni che determinano il tempo avvengono su celle da un metro."],
          dimensione=19, font=FONT_TITOLO, interlinea=1.35, spazio_dopo=8)

    testo(slide, MARGINE, Cm(14.6), L_SLIDE - 2 * MARGINE, Cm(2),
          "Il sistema si muove diversamente senza muoversi meglio: il guadagno si trasferisce "
          "alla decisione soltanto nella misura in cui supera la risoluzione alla quale la "
          "decisione è presa.",
          dimensione=16, colore=TESTO, interlinea=1.25)
    note(slide, "1:30 — La slide che giustifica la tesi. Novanta secondi, con calma. "
                "Lo spostamento è in larga parte trasversale rispetto all'errore: il rapporto "
                "fra i due è prossimo a tre.")
    return slide


def slide_oracolo(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Anche una previsione perfetta non aggiunge nulla",
                 "4 · Il risultato", 14, "1:00")
    elenco(slide, MARGINE, Cm(5.2), Cm(29), [
        "L'oracolo riceve la posizione futura vera: equivale ad abbattere il 100% dell'errore",
        "Nessuna rete, quale che sia l'architettura, può collocarsi al di sopra di esso",
        "Sul controllo di velocità: zero secondi, una ventina di centimetri di spazio",
        "Sul controllo di posizione: tre millimetri su venticinque centimetri di guadagno",
    ])
    rettangolo(slide, MARGINE, Cm(12.4), L_SLIDE - 2 * MARGINE, Cm(3.4), riempimento=FONDINO)
    rettangolo(slide, MARGINE, Cm(12.4), Cm(0.12), Cm(3.4), riempimento=ACCENTO)
    testo(slide, MARGINE + Cm(0.9), Cm(13.1), L_SLIDE - 2 * MARGINE - Cm(1.8), Cm(2.6),
          "I venticinque centimetri li produce l'attuatore, non il predittore.\n"
          "Su questo canale l'ibridazione è esclusa per dimostrazione, e la rete non vi è "
          "stata innestata: l'omissione è una conclusione, non una lacuna.",
          dimensione=17, interlinea=1.3)
    note(slide, "1:00 — Il valore di metodo: misurare il tetto costa ventiquattro corse, "
                "addestrare e misurare una rete costa settimane. L'oracolo ha chiuso una "
                "linea di lavoro prima che venisse percorsa.")
    return slide


def slide_conclusioni(prs):
    slide = slide_vuota(prs)
    intestazione(slide, "Il limite non è nell'informazione,\n"
                        "ma nella struttura con cui la si usa",
                 "5 · Conclusioni", 15, "1:30")
    elenco(slide, MARGINE, Cm(6.0), Cm(29), [
        "L'esito nullo si ripresenta identico sotto due modelli cinematici diversi: "
        "è una verifica di robustezza, non una disomogeneità",
        "La convenienza dell'ibridazione non è una proprietà del modello, ma della relazione "
        "fra scala del guadagno e risoluzione della decisione",
        "Il limite superiore si può misurare prima di addestrare: il costo di un "
        "addestramento inutile è evitabile per intero",
    ], passo=1.9)
    rettangolo(slide, MARGINE, Cm(13.6), L_SLIDE - 2 * MARGINE, Cm(2.9), riempimento=FONDINO)
    rettangolo(slide, MARGINE, Cm(13.6), Cm(0.12), Cm(2.9), riempimento=ACCENTO)
    testo(slide, MARGINE + Cm(0.9), Cm(14.3), L_SLIDE - 2 * MARGINE - Cm(1.8), Cm(1.9),
          "L'indifferenza, quando è misurata con il proprio intervallo di confidenza, "
          "è un'informazione utile quanto un guadagno.",
          dimensione=18, font=FONT_TITOLO, corsivo=True, interlinea=1.25)
    note(slide, "1:30 — Chiudi qui, non sui limiti. L'ultima frase è quella che resta.")
    return slide


# --- slide di riserva: non si mostrano, servono alle domande -----------------

def slide_riserva(prs, titolo, voci, numero):
    slide = slide_vuota(prs)
    intestazione(slide, titolo, "riserva — non esposta", numero, "domande")
    elenco(slide, MARGINE, Cm(5.4), Cm(29), voci, passo=1.6)
    return slide


def slide_separatore_riserva(prs):
    slide = slide_vuota(prs)
    rettangolo(slide, 0, 0, L_SLIDE, H_SLIDE, riempimento=FONDINO)
    testo(slide, MARGINE, Cm(8.4), L_SLIDE - 2 * MARGINE, Cm(2.5),
          "Slide di riserva", dimensione=30, font=FONT_TITOLO,
          allineamento=PP_ALIGN.CENTER)
    testo(slide, MARGINE, Cm(10.6), L_SLIDE - 2 * MARGINE, Cm(1.5),
          "Da qui in avanti non si espone: servono a rispondere alle domande.",
          dimensione=15, colore=TENUE, allineamento=PP_ALIGN.CENTER)
    return slide


def costruisci():
    prs = Presentation()
    prs.slide_width = L_SLIDE
    prs.slide_height = H_SLIDE

    slide_titolo(prs)
    slide_problema(prs)
    slide_domanda(prs)
    slide_theta(prs)
    slide_rischio(prs)
    slide_percezione(prs)
    slide_ambiente(prs)
    slide_innesti(prs)
    slide_rete(prs)
    slide_metodo(prs)
    slide_previsione(prs)
    slide_comportamento(prs)
    slide_dove_si_arresta(prs)
    slide_oracolo(prs)
    slide_conclusioni(prs)

    slide_separatore_riserva(prs)
    slide_riserva(prs, "Costo della risoluzione: perché non si è raffinato", [
        "Un dimezzamento porta la cella a 50 cm: il guadagno ne vale 25, la decisione "
        "resterebbe a scala doppia",
        "Latenza della singola ricerca: 95 ms → 401 ms → 1,68 s, contro un fotogramma da 17",
        "Diffusione delle macchie a 25 cm con trenta persone: 86 ms su 100",
        "Eseguire le campagne a 50 cm era possibile: costa l'1%, indistinguibile da zero",
        "Il vincolo non è il costo del banco, è la latenza a bordo",
    ], 17)
    slide_riserva(prs, "Scala spaziale del simulatore", [
        "Una cella vale un metro, ancorata all'ingombro dei corpi (spalle ~46 cm)",
        "A questa scala le velocità dei pedoni risultano superiori a quelle tipiche",
        "È un limite dichiarato in §9.4, non una svista",
        "I confronti sono appaiati e interni a ciascuna configurazione: la scala non li altera",
    ], 18)
    slide_riserva(prs, "Perché il guadagno della rete si ferma al 20,2%", [
        "L'ingresso contiene la sola storia individuale della persona osservata",
        "Nessun descrittore di contesto: né muri, né densità locale, né capofila del gruppo",
        "Deve inferire una svolta senza sapere che c'è un muro davanti",
        "È il limite più stringente dell'impostazione, ed è il primo sviluppo indicato",
    ], 19)
    slide_riserva(prs, "Come si affronterebbe su un robot reale", [
        "Risoluzione fine solo in una finestra attorno al robot, non su tutta la mappa",
        "Due livelli a frequenze diverse: globale lento, locale veloce",
        "Ripianificare solo quando le celle cambiate intersecano davvero il percorso",
        "Lo strato locale esiste già: è il controllo di posizione del Capitolo 8",
    ], 20)
    slide_riserva(prs, "I tempi assoluti dipendono dall'implementazione", [
        "Le misure sono tempo di parete di un'implementazione in Python",
        "Un'implementazione compilata sposterebbe gli assoluti di un fattore importante",
        "Il ×4 per dimezzamento è strutturale: discende dal quadruplicarsi dei nodi",
        "Sposterebbe la soglia, non la eliminerebbe: cambierebbe quanti dimezzamenti "
        "ci si può permettere",
    ], 21)

    prs.save(USCITA)
    return prs


if __name__ == "__main__":
    presentazione = costruisci()
    print("scritte %d slide in %s" % (len(presentazione.slides.__iter__.__self__._sldIdLst), USCITA))

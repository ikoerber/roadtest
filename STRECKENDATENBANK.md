# Streckendatenbank

Verbindliche Spezifikation des Projektziels. Festgelegt am 04.08.2026.

Alles, was die Firmware misst, dient diesem Ergebnis. Wo eine Entscheidung
zwischen Messgenauigkeit und diesem Ziel abzuwägen ist, gewinnt dieses
Dokument.

## Das Ziel

Aus mühelos aufgezeichneten Fahrten entsteht eine Datenbank der befahrenen
Straßen, aus der sich später neue Strecken zusammensetzen lassen, die mit
Freude zu fahren sind.

Zwei Stufen, in dieser Reihenfolge:

1. **Bewertete Karte** - welche befahrenen Straßen sind schön zu fahren.
   Kommt ohne Vorhersagemodell aus und erzeugt beim Bewerten zugleich die
   Referenzdaten für Stufe 2.
2. **Routengenerator** - "gib mir eine schöne Zwei-Stunden-Runde ab hier".
   Braucht ein Modell, das auch unbefahrene Straßen aus Kartengeometrie
   einschätzt, und damit die Urteile aus Stufe 1.

Stufe 2 wird nicht begonnen, bevor Stufe 1 die Abnahmekriterien erfüllt.

## Arbeitsteilung

Das Gerät erfasst und zeigt live an. Die Auswertung läuft auf dem Rechner.
Karten- und Straßeninformationen kommen aus offenen Daten und einer
Routenplanung auf dem Rechner, niemals aus dem Gerät.

Der Flash des LOLIN S3 Mini stand vor dem Aufräumen von 1.5.41 bei 93,6 und
liegt jetzt bei 88,3 Prozent. Für Kartendaten oder Bewertungsmodelle ist dort
auch so kein Platz, und das ist eher ein Glücksfall als eine Einschränkung: Die Trennung bleibt auch dann bestehen, wenn eine
Funktion auf dem Gerät kurzfristig bequemer wäre.

## Die härteste Anforderung: ohne Mühe

**Was während der Fahrt Aufmerksamkeit kostet, kommt nicht in die
Datenbank.** Alles, was nicht automatisch anfällt, muss nach der Fahrt am
Rechner nachholbar sein - oder es entfällt.

Der Grund ist keine Bequemlichkeit, sondern Statistik. Die Testfahrten bis zum
04.08.2026 liefen mit Ritual: im Stand auf Satelliten warten, über die
Weboberfläche starten, teils ein Beifahrer für die Urteile, am Ende stoppen,
danach die Karte ziehen. Das trägt fünf Fahrten. Die Datenbank entsteht erst
bei zweihundert.

**Mit 1.5.42 startet die Aufzeichnung selbsttätig**, sobald die SD-Karte
bereit ist. Ein eigenes Beenden braucht es nicht: Mit zündungsgeschalteter
Spannung endet die Fahrt durch Wegfall der Versorgung. Dabei geht höchstens ein
Flush-Umlauf von knapp fünf Sekunden verloren, und zwar genau der Abschnitt,
in dem das Fahrzeug bereits steht.

Der Start ist bewusst **nicht** an CAN gekoppelt. Ein Busfehler kostet dann die
OBD-Werte, nicht die ganze Fahrt.

Der zweite Engpass ist das Auslesen. Die Karte zu ziehen ist bei einer Fahrt
pro Woche gleichgültig und bei täglichen Fahrten der Grund, warum die Datenbank
nicht wächst - erst recht, wenn das Gerät hinter der Verkleidung sitzt.

**Der Zugriff erfolgt deshalb über das ROADTEST-WLAN vom Handy aus.** Seit
1.5.43 zeigt `/files` die Sitzungen und gibt jede als ein `.tar.gz` aus, mit
anschließendem Löschen. Damit kommt eine Funktion zurück, die einmal als zu
langsam entfernt wurde. Der Unterschied ist die Kompression: Eine Fahrstunde
erzeugt rund 12 MB, CSV komprimiert sich acht- bis zwölffach, und der
Kompressor stammt aus dem ROM des ESP32-S3 und kostet kein Byte Flash. Roh
übertragen war die alte Funktion zu Recht unbrauchbar; komprimiert ist ein
Fahrtag eine Sache von Sekunden.

Eine Sitzung bleibt dabei ein Tippvorgang. Sie besteht aus zehn Dateien -
einzeln geholt wären das bei zweihundert Fahrten zweitausend Bedienschritte,
und genau daran stirbt eine Datenbank.

Verbindlich bleibt der Grund, aus dem sie damals entfiel: **Unkomprimierte
Übertragung ganzer Logdateien wird nicht wieder eingeführt.**

## Die Einheit: das Wegstück zwischen zwei Kreuzungen

Die Datenbank besteht aus Segmenten des offenen Straßennetzes zwischen zwei
Kreuzungen, nicht aus den 200-Meter-Abschnitten der Firmware. Ein Urteil und
eine Routenentscheidung beziehen sich beide auf "diese Straße bis zur
Abzweigung". Die 200-Meter-Messungen hängen als Belege an den Segmenten.

**Segmente sind richtungsbehaftet.** Die Kurvengeometrie ist es nachweislich
nicht: Hin- und Rückfahrt derselben Strecke ergaben am 02.08.2026 über acht
gemeinsame Kurven 1,0 Grad Abweichung im Median. Fahrspaß dagegen schon -
dieselbe Straße kann bergab großartig und bergauf zäh sein. Geometrie wird
deshalb einmal je Straße geführt, Messung und Urteil je Richtung.

Je Segment stehen in der Datenbank:

- **Geometrie**, aus den Fahrten gemessen: Länge, Kurvenzahl, Winkelsumme,
  Radienverteilung, Höhendifferenz, Kurven je Kilometer.
- **Fahrbahn**: Median des Effektivwerts über alle Befahrungen, dazu deren
  Anzahl. Bei einer Befahrung ist der Wert eine Spurmessung, bei fünf eine
  Straßeneigenschaft - die Rauheit gehört der befahrenen Spur, nicht der
  Straßengeometrie.
- **Urteil**: die Noten mit Datum, einzeln erhalten, niemals zu einem Wert
  verdichtet gespeichert.
- **Herkunft**: welche Sitzungen, wann, in welcher Richtung.

## Der Bewertungsablauf

Nach dem Einlesen einer Fahrt zeigt der Rechner die befahrenen Segmente in
Fahrreihenfolge - Karte, Kurvenprofil, gefahrene Geschwindigkeit - und der
Fahrer vergibt Noten. Überspringen ist ausdrücklich erlaubt und der Normalfall:
Was nicht erinnert wird, bleibt unbewertet.

**Bei jeder Bewertungssitzung laufen einige bereits bewertete Segmente mit.**
Absolute Noten haben eine bekannte Schwäche, die bei einer über Monate
wachsenden Datenbank zuschlägt: Der Maßstab wandert, und das Modell lernt die
Stimmung statt die Straße. Weichen die Noten auf den mitgeführten Segmenten
systematisch ab, ist der Versatz messbar und rechnerisch abziehbar. Das ist
von Anfang an einzubauen - nachträglich lässt es sich nicht rekonstruieren.

Die Notenskala selbst ist **noch offen** und wird festgelegt, sobald die
Bewertungsoberfläche steht.

## Verbindliche Auswertungsregeln

Diese Regeln gelten aus dem bestehenden Projekt weiter und sind hier wichtiger
als bisher:

- **Die Bewertung wird bei der Auswertung neu gerechnet, nie aus einer Datei
  übernommen.** Urteile und Messungen sind die Daten, die Formel ist Auslegung.
  Nur so bleiben alte Fahrten nach einer Nachkalibrierung verwertbar.
- **Ein Urteil gilt für sein Segment, nicht bis zum nächsten.** Nicht bewertete
  Segmente bleiben ohne Note. Das ist der gewollte Zustand, keine Lücke, die zu
  füllen wäre.
- Nur nachgewiesen gültige Positionen werden verwendet. Über eine Qualitäts-
  oder Zeitlücke hinweg wird keine Linie gezogen.
- Unbekannte Werte erscheinen als `null`, niemals als 0.
- Die Fahrspaß-Formel wird **nicht** vor den ersten Urteilen festgelegt. Sonst
  wiederholt sich der Fehler der ersten Fahrbarkeitsklassen: eine plausible
  Benennung ohne Deckung in den Urteilen.

## Abnahme der Stufe 1

Prüfbare Kriterien, keine Absichtserklärungen:

1. Eine Fahrt wird eingelesen und ihren Segmenten zugeordnet, ohne dass jemand
   Hand anlegt.
2. **Dieselbe Straße zweimal gefahren ergibt dieselbe Geometrie.** Die
   Hin-und-Rück-Messung aus `tools/vergleich_hin_rueck.py` ist der bereits
   etablierte Prüfstand dafür und gilt unverändert für die Segmentdatenbank.
3. Eine Fahrt von 60 Minuten ist in wenigen Minuten bewertet.
4. Die Karte zeigt den Bestand über alle Fahrten hinweg, mit Befahrungszahl je
   Segment.

Die Fahrspaß-Formel gehört ausdrücklich nicht zu diesen Kriterien.

## Reihenfolge der Umsetzung

1. Zuordnung der Fahrten zum Straßennetz und Aufbau der Segmentdatenbank.
2. Bewertungsoberfläche auf dem Rechner.
3. Karte über den gesamten Bestand.
4. Erst danach: Modell und Routengenerator.

Am Gerät sind das selbsttätige Starten (1.5.42), der Datenzugriff übers Handy
(1.5.43) und der Rücksprung auf die vorherige Firmware (1.5.44) erledigt.
Offen ist dort nur noch die GPS-Antenne - der einzige nie erklärte Fehler des
Systems und damit das, was vor dem endgültigen Einbau zu klären bleibt.

**Die bereits vorhandenen Messdaten sind für die Schritte 1 bis 3 das richtige
Material.** Die 318 km aus 39 Sitzungen bis zum 04.08.2026 sind Testfahrten
zur Prüfung von Hardware und Messkette, keine Spaßfahrten - zum Lernen von
Fahrspaß taugen sie nicht. Zum Entwickeln und Abnehmen von Zuordnung,
Segmentbildung und Reproduzierbarkeit sind sie ideal: bekannte Strecken,
mehrfach befahren, teils hin und zurück. Die Rechnerkette lässt sich
vollständig bauen und abnehmen, bevor die erste Spaßfahrt stattfindet.

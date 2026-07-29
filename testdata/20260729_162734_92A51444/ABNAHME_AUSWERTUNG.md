# Abnahmeauswertung – Sitzung 20260729_162734_92A51444

## Gesamturteil

Die zweistufige Abnahmeführung von Firmware 1.5.20 funktioniert deutlich
besser. ECU-Wiedererkennung und Motorlauferkennung bestanden beide
Startzyklen innerhalb der Zehn-Sekunden-Grenze.

Urteil für OBD-/ECU-Recovery: **PASS mit Hinweisen**.

Die vollständige Datenqualitätsabnahme ist noch nicht erreicht, weil die
GPS-Streckenbildung im stehenden Fahrzeug 28 Meter aus Positions- und
Geschwindigkeitsrauschen erzeugte.

## Vollständigkeit

- zehn erwartete Sitzungsdateien vorhanden
- START- und END-Metadaten vorhanden
- Fahrzusammenfassung vorhanden
- `ACCEPTANCE_RESULT` vorhanden
- SD-Fehler: 0
- verworfene SD-Datensätze: 0
- OBD-Trace-Verluste: 0

## Abnahmezeiten

| Schritt | Zeit | Urteil |
|---|---:|---|
| Dateivorbereitung | 1,407 s | PASS |
| erste ECU-Antwort nach Zündungsmarker | 1,329 s | PASS* |
| erster Motorlauf über RPM ≥ 300 | 3,797 s | PASS |
| ECU-Ausfall nach Ausschaltmarker | 1,500 s | PASS |
| ECU-Antwort nach erneutem Zündungsmarker | 0,440 s | PASS* |
| Motorlauf nach Neustart | 5,227 s | PASS |

\* Die ECU war in beiden Zündungszyklen bereits kurz vor dem jeweiligen
Zündungsmarker erreichbar: 3,410 Sekunden vor dem ersten Marker und 0,619
Sekunden vor dem zweiten. Die Firmware maß anschließend korrekt eine weitere
Antwort nach dem Marker. Für einen streng kausalen Zündungstest muss der
Webbutton vor dem physischen Einschalten der Zündung gedrückt werden.

Die frühere Startverzögerung von ungefähr 40 Sekunden sank durch das
Sitzungsunterverzeichnis auf 1,407 Sekunden.

## OBD und CAN

- 134 OBD-Anfragen
- 109 Antworten
- 6 Sendefehler
- 31 Timeouts
- 3 nicht zugeordnete, verspätete Antworten
- 0 verlorene OBD-Trace-Ereignisse
- Antworten von `0x7E8` und `0x7E9`
- 2 RX0-Überläufe, 0 RX1-Überläufe
- 0 Controller-Recoveries

Die Timeouts und Sendefehler liegen überwiegend in den absichtlich
zündungslosen Abschnitten. Die zwei RX0-Überläufe entstanden während des
Unterstützungsblockscans mit nahezu gleichzeitigen Antworten mehrerer ECUs.
Sie verhinderten die Wiedererkennung nicht, bleiben aber ein
Optimierungshinweis.

## GPS

- Einbauort: im Fahrzeug unter dem Vordersitz
- Cabrioverdeck während der Messung geschlossen
- keine externe GPS-Antenne angeschlossen
- ländlicher Standort ohne hohe Bäume oder Gebäude, klarer Himmel
- 598 von 598 Snapshots mit gültigem Fix
- 197 neue Fixes
- 5 bis 11 Satelliten, Median 8
- HDOP Median 1,53, 95. Perzentil 2,33, Maximum 3,39
- maximales Positionsalter 827 ms
- 0 NMEA-Prüfsummenfehler
- 0 UART-Ringpufferüberläufe

Das Fahrzeug meldete über OBD durchgehend 0 km/h. GPS meldete dagegen
zeitweise bis 5,2 km/h und in 166 Snapshots mindestens 1,5 km/h. Die Position
streute ungefähr 7,2 Meter in Nord-Süd- und 16,7 Meter in Ost-West-Richtung.
Dadurch entstanden in der Fahrzusammenfassung fälschlich 0,028 km Strecke.

Dieser Befund bestätigt den geplanten nächsten Entwicklungsschritt:
feldweise GPS-Alters- und Qualitätsgrenzen sowie eine strengere
Stillstands-/Plausibilitätsprüfung.

Der Einbau unter dem Sitz und die fehlende externe Antenne schränken den
direkten Satellitenempfang trotz guter äußerer Bedingungen ein und können das
Rauschen begünstigen. Sie erklären aber nicht, warum dieses Rauschen als
gefahrene Strecke gewertet wurde.

Die anschließend entwickelte Filterung aus Firmware 1.5.21 wurde rechnerisch
gegen diese Sitzung sowie zwei echte Fahrten und zwei weitere Standtests
zurückgespielt. Für diese Sitzung und beide Standtests ergibt sie 0,000 km;
die echten Fahrten bleiben mit ungefähr 3,641 km und 1,471 km erhalten. Die
Bestätigung auf der Hardware steht noch aus.

## Fazit

Die Phase-2-Funktion „ECU nach Zündungswechsel ohne Firmware-Neustart wieder
finden“ ist praktisch bestätigt. Vor einer allgemeinen Datenqualitätsabnahme
sind noch zwei Punkte offen:

1. Zündungsmarker im nächsten Kontrolllauf nachweislich vor der physischen
   Betätigung setzen.
2. GPS-Stillstandsrauschen darf keine Strecke mehr erzeugen.

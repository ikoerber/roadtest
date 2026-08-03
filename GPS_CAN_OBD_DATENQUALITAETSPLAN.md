# Datenqualitätsplan für GPS und CAN/OBD

## Ziel

Die Hardware des ROADTEST-Funktionsprototyps bleibt grundsätzlich unverändert.
Der nächste Entwicklungsschwerpunkt ist die messbare Datenqualität der beiden
primären Fahrzeugdatenquellen:

1. GPS als unabhängige Referenz für Position und Geschwindigkeit
2. CAN/OBD als direkte Quelle für Fahrzeuggeschwindigkeit, Drehzahl und später
   weitere standardisierte Messwerte

Beide Quellen müssen in jedem Fahrzeugtest gleichzeitig aufgezeichnet,
diagnostiziert und miteinander verglichen werden. BNO055, SD, OLED und WLAN
laufen weiter mit, sind in dieser Phase aber vor allem Infrastruktur und
Stabilitätskontrolle.

## Ausgangslage

Der Fahrzeugtest vom 28.07.2026 ist in
`testdata/archiv/20260728_2300/TESTBERICHT_2026-07-28.md` dokumentiert.

Wesentliche Erkenntnisse:

- Der Porsche antwortete vor der Discovery grundsätzlich über CAN-ID `0x7E8`
  auf standardisierte OBD-Service-01-Anfragen.
- Die Discovery-Sitzung selbst enthielt keine OBD-Antworten, weil der
  Unterstützungsblockscan vor dem Motorstart ohne Antwort endete und danach
  nicht wiederholt wurde.
- Die aktuellen OBD-Zähler gelten seit dem Boot und erlauben keine eindeutige
  Bewertung einer einzelnen Sitzung.
- GPS lieferte gute Teilabschnitte, aber auch Fixverluste, wiederholte
  Positionen, unplausible Höhen und Positionssprünge.
- GPS-Felder können momentan unterschiedlich alt sein und trotzdem gemeinsam
  als Datensatz gespeichert werden.
- SD-Logging, BNO055, OLED, WLAN und die MCP2515-Grundfunktion waren stabil.

## Grundsätze

- Ungültig oder unbekannt ist nicht gleich null. Fehlende Werte erhalten ein
  Gültigkeitskennzeichen und einen Fehlergrund.
- Jeder Messwert benötigt Quelle, Erfassungszeit, Alter und Qualitätsstatus.
- Sitzungsstatistiken dürfen keine Werte aus der Zeit vor dem Sitzungsstart
  enthalten.
- Rohdaten bleiben erhalten; gefilterte beziehungsweise abgeleitete Werte
  werden zusätzlich gespeichert und eindeutig gekennzeichnet.
- Messqualität wird mit reproduzierbaren Kennzahlen bewertet, nicht anhand
  einzelner Bildschirmwerte.
- Standardisierte, nur lesende OBD-Service-01-Abfragen bleiben die
  Sicherheitsgrenze.
- GPS und OBD werden zuerst stabilisiert. Herstellerspezifische Porsche-PIDs
  folgen frühestens nach bestandener Standarddaten-Pipeline.

## Qualitätsziele

Die Grenzwerte sind erste Abnahmewerte und werden nach mindestens drei
vergleichbaren Testfahrten überprüft.

### GPS

| Kennzahl | Ziel | Warnung | Fehler |
|---|---:|---:|---:|
| gültiger Fix während freier Fahrt | mindestens 95 % | 90–95 % | unter 90 % |
| Alter der verwendeten Position | höchstens 2.000 ms | 2.001–5.000 ms | über 5.000 ms |
| Satelliten für gültigen Fahr-Fix | mindestens 4 | — | unter 4 |
| HDOP für gültigen Fahr-Fix | höchstens 5,0 | 5,1–10,0 | über 10,0 |
| unplausible Sprünge nach Filterung | 0 | 1 | mehr als 1 |
| unerkannte UART-Pufferüberläufe | 0 | — | größer 0 |
| verworfene NMEA-Sätze | vollständig gezählt | — | nicht messbar |

Ein GPS-Ausfall darf keine alte Position, Geschwindigkeit oder Höhe als
aktuellen Wert erscheinen lassen.

### CAN/OBD

| Kennzahl | Ziel | Warnung | Fehler |
|---|---:|---:|---:|
| ECU-Erkennung nach Motorstart | höchstens 10 s | 10–20 s | über 20 s |
| Antwortquote unterstützter PIDs bei aktiver ECU | mindestens 90 % | 75–90 % | unter 75 % |
| Sendefehler bei aktiver ECU | unter 1 % | 1–5 % | über 5 % |
| Alter von Geschwindigkeit und Drehzahl | höchstens 2.000 ms | 2.001–5.000 ms | über 5.000 ms |
| MCP2515 Bus-off | 0 | — | größer 0 |
| Empfangspufferüberläufe | 0 | 1 | mehr als 1 |
| nicht zuordenbare OBD-Antworten | 0 | einzeln dokumentiert | wiederholt |

Die Antwortquote wird erst nach bestätigter ECU-Erreichbarkeit berechnet.
Zündung aus oder ein schlafendes Gateway dürfen die Quote nicht künstlich
verschlechtern.

### Gemeinsamer GPS-/OBD-Vergleich

| Kennzahl | Ziel |
|---|---:|
| zeitlicher Versatz zugeordneter Geschwindigkeitswerte | höchstens 500 ms |
| Median der absoluten Abweichung bei Konstantfahrt | höchstens 3 km/h |
| 95-%-Wert der absoluten Abweichung bei Konstantfahrt | höchstens 5 km/h |
| GPS- und OBD-Geschwindigkeit im markierten Stillstand | jeweils 0 beziehungsweise innerhalb definierter Nulltoleranz |
| nicht zuordenbare Samples | weniger als 5 % eines gültigen Fahrabschnitts |

OBD-Geschwindigkeit basiert auf Fahrzeug- beziehungsweise Raddaten und muss
nicht exakt der GPS-Geschwindigkeit entsprechen. Bewertet werden Stabilität,
zeitliche Übereinstimmung und reproduzierbare Abweichung.

### Logging und Zeitbasis

- keine beschädigten CSV-Zeilen
- keine verworfenen SD-Datensätze
- monotone Sitzungs-Uptime
- eine gemeinsame Sitzungs-ID in allen Dateien
- Firmwareversion und relevante Konfiguration in den Sitzungsmetadaten
- Zeitstempel am Empfang der Daten, nicht erst beim späteren Schreiben
- eindeutige Unterscheidung zwischen Rohwert, gültigem Wert und abgeleitetem
  Wert

## Benötigte Datenfelder

### GPS-Datensatz

Zusätzlich zu den vorhandenen Werten werden benötigt:

- `LocationValid`
- `LocationAgeMs`
- `SpeedValid`
- `SpeedAgeMs`
- `AltitudeValid`
- `AltitudeAgeMs`
- `CourseValid`
- `CourseAgeMs`
- `SatellitesValid`
- `SatellitesAgeMs`
- `HDOPValid`
- `HDOPAgeMs`
- `NewFix`
- `FixSequence`
- `NMEAChars`
- `NMEASentencesValid`
- `NMEAChecksumFailures`
- `RXBufferOverflows`
- `Rejected`
- `RejectionReason`

### OBD-Datensatz

Zusätzlich zu den vorhandenen Werten werden benötigt:

- `SessionRequestSequence`
- `RequestUptimeMs`
- `RequestPID`
- `TransmitOK`
- `TransmitError`
- `ResponseUptimeMs`
- `ResponseLatencyMs`
- `ResponseECU`
- `ResponsePID`
- `ValueValid`
- `ValueAgeMs`
- `Timeout`
- `CANMode`
- `TEC`
- `REC`
- `EFLG`

### Sitzungsmetadaten

Eine Sitzung soll mindestens dokumentieren:

- Sitzungs-ID
- Firmwareversion
- Startzeit UTC und Start-Uptime
- Fahrzeugprofil
- CAN-Bitrate und MCP2515-Quarz
- aktive GPS- und OBD-Qualitätsgrenzen
- erkannte ECU-IDs
- unterstützte beziehungsweise unbekannte PIDs
- Anzahl GPS-, OBD-, CAN- und SD-Fehler während dieser Sitzung

## Umsetzungsphasen

### Phase 1 – Messbarkeit herstellen

Ziel: Fehler und Datenalter müssen sichtbar sein, bevor Filter oder neue
Messwerte ergänzt werden.

Status: **seit Firmware 1.5.15 umgesetzt und durch die Hardware- und
Fahrzeugtests bis `20260729_170946_05A4DB46` bestätigt.**

#### Firmware

- Beim Discovery-Start Startwerte aller OBD-Zähler erfassen oder echte
  Sitzungszähler zurücksetzen.
- Ergebnis jedes OBD-Sendeversuchs protokollieren.
- Anfrage und Antwort über Sequenz, PID, ECU-ID und Uptime zuordnen.
- MCP2515-Diagnosewerte beim Scan, bei Fehlern und regelmäßig während der
  Sitzung speichern.
- Für jedes GPS-Feld Gültigkeit und Alter bereitstellen.
- GPS-UART-Pufferüberläufe zählen.
- Neue GPS-Fixes von wiederholten Ausgaben unterscheiden.
- Firmwareversion und Qualitätsparameter in die Sitzung aufnehmen.

Umgesetzt werden die erweiterten GPS-Felder in `road_gps_...csv`, die
dekodierten Sitzungswerte in `road_obd_...csv`, einzelne Transaktionen in
`road_obd_trace_...csv` und Start-/Abschlusswerte in `road_meta_...csv`.
Die Metadatendatei enthält zusätzlich alle fünf Sekunden einen
Zwischenzustand.
`Rejected` und `RejectionReason` bleiben in 1.5.15 bewusst `0`; die
Plausibilitätsfilter gehören zu Phase 3.

#### Abnahme

- Ein kurzer Test ohne Fahrzeug erklärt jeden OBD-Sendefehler eindeutig.
- Ein Test mit aktiver ECU weist Antworten eindeutig der Sitzung zu.
- Ein statischer GPS-Test zeigt neue Fixes, Feldalter und Fixverluste.
- Alle Diagnosezähler beginnen für die Auswertung bei null oder besitzen einen
  dokumentierten Sitzungsstartwert.

### Phase 2 – OBD-Erkennung zuverlässig machen

Ziel: Ein später Motorstart oder ein vorübergehend schlafendes Gateway darf die
gesamte Messfahrt nicht ohne OBD-Daten lassen.

Status: **in Firmware 1.5.20 umgesetzt. Der Abnahmetest
`20260729_162734_92A51444` bestätigte die getrennte Erfassung von Zündung,
Motorlauf, ECU-Ausfall und Wiederanlauf am Fahrzeug.**

#### Firmware

- ECU-Erreichbarkeit als eigenen Zustand führen.
- Unterstützungsblockscan mit begrenztem Backoff wiederholen, solange keine
  ECU geantwortet hat.
- Zündung-Ein und Motorstart getrennt markieren: ECU-Erreichbarkeit anhand
  einer Antwort, tatsächlichen Motorlauf anhand frischer OBD-Drehzahl erkennen.
- Nach der ersten erkannten ECU-Antwort sofort neu scannen.
- `unbekannt` und `nicht unterstützt` strikt unterscheiden.
- Bei unbekannter Unterstützung die bereits am Fahrzeug bestätigten PIDs
  `0x0C`, `0x0D` und `0x11` in niedriger Rate als Rückfall testen.
- Erst nach positiven Antworten weitere bestätigte Standard-PIDs einplanen.
- OBD-Anfragerate weiterhin auf höchstens zwei Frames pro Sekunde begrenzen.
- Listen-Only-Phase unverändert ohne Senden und ohne ACK betreiben.

#### Abnahme

- Zündung aus: kein falscher ECU-Status.
- Zündung ein, Motor aus: vorhandene Antworten werden erkannt und protokolliert.
- Motorstart nach erfolglosem ersten Scan: ECU wird ohne Neustart der Firmware
  nachträglich erkannt.
- Kurzer ECU-Ausfall: automatische, begrenzte Wiederaufnahme ohne
  Anfrageflut.
- Alle drei Rückfall-PIDs werden getrennt als unterstützt, nicht unterstützt
  oder ohne Antwort bewertet.

### Phase 3 – GPS-Datensätze kohärent machen

Ziel: Ein gespeicherter GPS-Datensatz darf nur zeitlich zusammengehörige und
plausible Werte als gültig kennzeichnen.

Status: **Filterstufe in Firmware 1.5.21 umgesetzt und mit der Vergleichsfahrt
`20260729_170946_05A4DB46` im Einbau unter dem Vordersitz bestätigt.**
Positionen benötigen höchstens 1,5 Sekunden Alter,
mindestens fünf Satelliten und HDOP höchstens 3,5. GPS-Geschwindigkeit gilt
zwischen 6 und 300 km/h als belastbar. Höhe, Kurs und ihre jeweiligen Alter
werden getrennt geprüft. Ablehnungsgründe werden als Bitmaske protokolliert.
Bei der Streckenbildung bestätigt eine frische OBD-Geschwindigkeit bis
1 km/h den Fahrzeugstillstand; ohne OBD ist eine gültige GPS-Geschwindigkeit
der Bewegungsnachweis.

Firmware 1.5.25 reduziert `/acceptance` nach der bestandenen Vergleichsfahrt
auf den noch offenen ECU-Recovery-Kontrolltest. Die Seite überträgt während
der Messung nur kleine Statusantworten. Eine frische OBD-Drehzahl kann den
zweiten Motorstart unabhängig vom Browsermarker bestätigen. Laufzeit-, Web-
und alle relevanten SD-Zeiten werden mit Summe und Anzahl protokolliert;
verfehlte 10-Hz-Sensor- und 5-Hz-GPS-Termine besitzen eigene Sitzungszähler.
Ein nach dem Motor-Aus-Marker beobachteter ECU-Ausfall bleibt beim
anschließenden Wiederanlauf als abgeschlossener Meilenstein gespeichert.
Der Kontrolllauf `20260730_101000_5901D247` bestätigte diesen Wiederanlauf.
Firmware 1.5.26 entfernt zusätzlich die zweite Zeitbegrenzung in Sensor- und
GPS-Logger, damit jeder vom Hauptzeitplan erzeugte Datensatz gespeichert wird.
Firmware 1.5.27 sichert bei einem vorübergehenden SD-Ausfall noch im RAM
liegende Sensorzeilen in einer separaten Recovery-Datei und setzt die Messung
in einer per Ereignis verknüpften Sitzung fort. Außerdem ergänzt sie für
langgezogene Kurven einen kumulativen Erkennungspfad; die öffentliche
OSRM-Kartenzuordnung diente nur zur Offline-Kalibrierung der Schwellen und ist
keine Laufzeitabhängigkeit der Firmware.
Firmware 1.5.28 entfernt die inzwischen abgeschlossene `/acceptance`-Seite.
Der neue Beifahrer-Kurventest unter `/curve-test` startet eine normale
GPS-/OBD-Gesamtaufzeichnung und ergänzt ausschließlich zeitgenaue
Kurvenreferenzmarker; die bestehenden Datenqualitätsfelder und Grenzwerte
bleiben unverändert.

#### Firmware

- Positionsgültigkeit an Alter, Satellitenzahl und HDOP koppeln. **Umgesetzt
  in 1.5.21.**
- Geschwindigkeit, Höhe und Kurs nur übernehmen, wenn das jeweilige Feld
  frisch ist. **Umgesetzt in 1.5.21.**
- Alte Felder nicht aus `lastValidData` als aktuelle Messung weiterreichen.
- Letzten guten Wert nur als ausdrücklich historischen Referenzwert führen.
- Physikalisch unplausible Höhe, Geschwindigkeit und Positionssprünge
  zurückweisen und den Grund protokollieren. **Für Höhe, Geschwindigkeit und
  die Streckenbildung in 1.5.21 umgesetzt; eine allgemeine
  Positionssprungbewertung bleibt offen.**
- GPS vorzugsweise bei einem neuen Fix speichern; Status-Heartbeat unabhängig
  davon höchstens einmal pro Sekunde.
- UART kontinuierlich leeren oder nachweisen, dass der Ringpuffer unter
  maximaler Systemlast nicht überläuft.

#### Abnahme

- 15 Minuten statischer Außentest ohne Positionssprünge.
- Kontrollierter Fixverlust erzeugt ungültige, nicht eingefrorene Messwerte.
- Wiederkehrender Fix wird ohne Neustart sauber übernommen.
- Keine Position mit null Satelliten wird als gültiger aktueller Fix geführt.
- Ungültige Höhen beeinflussen weder Strecke noch spätere Auswertungen.

### Phase 4 – Zeitliche Korrelation

Ziel: GPS- und OBD-Geschwindigkeit müssen automatisch miteinander vergleichbar
sein.

#### Firmware und Auswertung

- Empfangs-Uptime beider Quellen verwenden.
- OBD-Antwortlatenz getrennt vom Messwertalter speichern.
- GPS- und OBD-Samples über den nächsten gültigen Zeitnachbarn zuordnen.
- Zuordnungen außerhalb des erlaubten Zeitfensters verwerfen.
- Keine künstliche Interpolation über Fix- oder ECU-Ausfälle.
- Pro markiertem Abschnitt Verfügbarkeit, Versatz und
  Geschwindigkeitsabweichung berechnen.

#### Abnahme

- Stillstand, Konstantfahrt und Beschleunigung sind im Bericht getrennt.
- Der geschätzte Zeitversatz ist reproduzierbar.
- Die Abweichungskennzahlen werden nur aus gültigen, zeitlich zugeordneten
  Samples berechnet.

### Phase 5 – Automatischer Qualitätsbericht

Ziel: Jede Testfahrt erhält ohne manuelle Tabellenkorrekturen eine einheitliche
PASS/WARN/FAIL-Auswertung.

Geplantes Auswertungswerkzeug:

```text
tools/analyze_gps_obd.py
```

Geplante Ausgaben:

```text
quality_summary_<SessionId>.json
quality_report_<SessionId>.md
```

Der Bericht soll mindestens enthalten:

- Sitzungs- und Firmwaredaten
- GPS-Verfügbarkeit und längste Ausfallzeit
- Satelliten- und HDOP-Verteilung
- Anzahl zurückgewiesener GPS-Werte nach Grund
- OBD-Anfragen, Sendeerfolge, Antworten und Timeouts je PID
- erkannte ECU-IDs
- CAN-Fehlerzustände
- GPS-/OBD-Geschwindigkeitsvergleich je Testmarkierung
- Datenlücken und SD-Fehler
- Gesamturteil mit den in diesem Dokument definierten Grenzwerten

## Standard-Testablauf im Fahrzeug

Tests nur durch eine zweite Person bedienen oder im sicheren Stillstand
markieren. Während der Fahrt keine Konsole bedienen.

### Vorbereitung

1. Mechanik, Versorgung, gemeinsame Masse und Steckverbindungen prüfen.
2. Seriellen Monitor öffnen.
3. System starten und `diag` ausführen.
4. Auf SD-, GPS-, CAN- und MCP2515-Bereitschaft prüfen.
5. Motor starten.
6. Prüfen, dass die ECU innerhalb von zehn Sekunden antwortet.
7. Erst dann die eigentliche Messsitzung starten.

### Pflichtmarkierungen

```text
discover mark leerlauf
discover mark fahrt_begonnen
discover mark konstant_30
discover mark konstant_50
discover mark konstant_70
discover mark beschleunigung
discover mark bremsung
discover mark wieder_im_stand
```

Eine Markierung für konstante Geschwindigkeit soll nur gesetzt werden, wenn
die Geschwindigkeit mindestens 15 bis 20 Sekunden sicher gehalten werden
kann. Nicht erreichbare oder unsichere Abschnitte werden ausgelassen.

### Abschluss

1. Im sicheren Stillstand `discover status` prüfen.
2. `discover end` ausführen.
3. Kontrollieren, dass alle Dateien geschlossen wurden.
4. Dateien über direkten SD-Kartenzugriff kopieren.
5. Automatischen Qualitätsbericht erzeugen.
6. PASS/WARN/FAIL und offene Auffälligkeiten im Testbericht festhalten.

## Testmatrix

| Test | Zweck | Erwartung |
|---|---|---|
| GPS statisch im Freien, 15 min | Grundrauschen und Fixstabilität | keine Sprünge, vollständige Diagnose |
| Zündung aus | OBD-Ruhezustand | keine falsche ECU-Verbindung |
| Zündung ein, Motor aus | Gateway-Wakeup | Antworten werden korrekt erkannt oder eindeutig als fehlend protokolliert |
| Motorstart nach fehlendem Scan | Wiederholungslogik | ECU wird ohne Firmware-Neustart erkannt |
| Leerlauf, 60 s | Nullgeschwindigkeit und Drehzahl | GPS/OBD nahe 0 km/h, plausible Drehzahl |
| Konstant 30 km/h | unterer Geschwindigkeitsvergleich | Differenz innerhalb der Grenzwerte |
| Konstant 50 km/h | mittlerer Geschwindigkeitsvergleich | Differenz innerhalb der Grenzwerte |
| Konstant 70 km/h | höherer Geschwindigkeitsvergleich | Differenz innerhalb der Grenzwerte |
| Beschleunigen und Bremsen | zeitlicher Versatz | keine langen Ausfälle, Versatz bestimmbar |
| kontrollierter GPS-Fixverlust | Recovery | alte Werte werden ungültig, Fix kehrt zurück |
| ECU-/Zündungsneustart | OBD-Recovery | begrenzte automatische Wiedererkennung |
| 30–60 min Dauerfahrt | Stabilität | keine SD-Verluste, Bus-off- oder Pufferfehler |

## Auswertungskennzahlen

Pro gesamter Sitzung und pro markiertem Abschnitt:

- Anzahl erwarteter und vorhandener Samples
- gültige Zeitabdeckung in Prozent
- längste Datenlücke
- Median, 95-%-Wert und Maximum des Datenalters
- GPS-Fixquote
- Verteilung von Satellitenzahl und HDOP
- Anzahl GPS-Ablehnungen je Grund
- OBD-Antwortquote je PID und ECU
- OBD-Antwortlatenz
- CAN-Sendefehlerquote
- TEC-, REC- und EFLG-Maximalwerte
- Median, 95-%-Wert und Maximum von `|GPS-Speed - OBD-Speed|`
- geschätzter Zeitversatz zwischen GPS und OBD
- SD-Schreibfehler, verworfene Datensätze und beschädigte Zeilen

Große Lücken werden nicht interpoliert. Ergebnisse müssen immer die Anzahl der
zugrunde liegenden gültigen Werte enthalten.

## Reihenfolge der Umsetzung

1. Sitzungsbezogene Diagnosezähler und neue Qualitätsfelder
2. Wiederholbare OBD-ECU-Erkennung mit sicheren Rückfall-PIDs
3. Kohärente GPS-Gültigkeits- und Altersprüfung
4. Gemeinsame Zeitzuordnung von GPS und OBD
5. Automatischer Auswertungsbericht
6. Drei aufeinanderfolgende Vergleichsfahrten
7. Grenzwerte anhand realer Daten bestätigen oder nachvollziehbar anpassen
8. Erst danach weitere PIDs wie Öltemperatur, Außentemperatur,
   Luftmassenstrom und Kraftstoffrate priorisieren

## Abschlusskriterien für die Datenqualitätsphase

Die GPS-/CAN-/OBD-Basis gilt als belastbar, wenn:

- drei aufeinanderfolgende Fahrzeugtests die Pflichtkriterien bestehen,
- Motorstart und ECU-Wiedererkennung keinen Firmware-Neustart benötigen,
- GPS-Fixverluste keine alten Werte als aktuell erscheinen lassen,
- die Geschwindigkeitsabweichung die definierten Grenzwerte erfüllt,
- keine Bus-off-, Empfangspuffer- oder SD-Verlustfehler auftreten,
- der Qualitätsbericht ohne manuelle Datenbereinigung erstellt werden kann,
- und alle verbleibenden Abweichungen automatisch markiert und begründet
  werden.

Hardwareänderungen erfolgen in dieser Phase nur bei einem reproduzierbaren,
messtechnisch belegten Hardwarefehler.

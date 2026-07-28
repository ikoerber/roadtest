# ROADTEST – Testbericht vom 28.07.2026

## Kurzfazit

Die Messfahrt mit Firmware 1.5.14 wurde vollständig und ohne SD- oder
I²C-Fehler aufgezeichnet. BNO055, SD-Karte, WLAN, optionales OLED und der
MCP2515 liefen stabil. Die Fahrbewegungsdaten sind grundsätzlich auswertbar.

Während der Discovery-Sitzung wurden jedoch keine OBD-Antworten aufgezeichnet.
Ursache ist nach dem derzeitigen Stand vor allem der Ablauf der
Fahrzeugdatenerkennung: Der einmalige PID-Unterstützungsscan fand statt, bevor
der Motor gestartet wurde. Nachdem alle Unterstützungsblöcke ohne Antwort
blieben, fragte die Firmware in der anschließenden Messphase keine PIDs mehr
ab. Der Motorstart löste keinen neuen Scan aus.

Die GPS-Daten enthalten zeitweise Fixverluste, veraltete oder nicht
zusammengehörige Felder sowie unplausible Höhen- und Positionswerte. Gute
Abschnitte der Strecke sind verwendbar, die von der Firmware ausgegebene
Gesamtdistanz sollte für diese Sitzung aber noch nicht als verlässlich
betrachtet werden.

## Testaufbau

- Fahrzeug: Porsche Carrera S, Baujahr 2012, PDK
- Diagnoseschnittstelle: OBD-II, ISO 15765-4 CAN
- CAN-Format: 11-Bit-ID, 500 kbit/s
- CAN-Controller: MCP2515, 16-MHz-Quarz
- MCP2515-Anschluss:
  - MISO/SO: GPIO 11
  - MOSI/SI: GPIO 13
  - SCK: GPIO 3
  - CS: GPIO 1
- BNO055 über I²C
- GPS: BN-880 über UART2
- SD-Karte über SPI, CS GPIO 4
- OLED: optional und steckbar
- Firmware: ROADTEST 1.5.14 „Vehicle Data Discovery“
- Discovery-Sitzung: `20260728_211454_F817A5B2`

## Vorbereitende Hardware- und Funktionstests

### BNO055

- Sensor wurde zuverlässig an seiner festgelegten Adresse und Chip-ID erkannt.
- Selbsttest war erfolgreich.
- Betriebsmodus NDOF und Systemstatus 5 wurden erreicht.
- Die gespeicherten Kalibrierungsdaten wurden aus NVS geladen.
- Während des dokumentierten seriellen Ausschnitts gab es keine
  I²C-Aussetzer.
- Der Gyrosensor war durchgehend vollständig kalibriert.
- Die Beschleunigungskalibrierung erreichte gegen Ende Stufe 3.
- System- und Magnetometerkalibrierung schwankten während der Fahrt. Dadurch
  sind relative Bewegungen gut nutzbar, ein absoluter NDOF-Kurs aber noch
  nicht durchgehend zuverlässig.

### Optionales OLED

- Das OLED wurde als optionale Komponente umgesetzt.
- Das System startet und misst auch ohne angeschlossenes Display.
- Im vorliegenden Test wurde das OLED als vorhanden und funktionsfähig
  gemeldet.
- Die serielle Diagnose kann während der Fahrzeugtests vollständig anstelle
  des OLED verwendet werden.

### SD-Logger

- SDHC-Karte wurde zuverlässig erkannt.
- Aufzeichnung wird nicht blockierend gestartet.
- Die zwischenzeitlich ergänzte Web-Downloadfunktion wurde wieder entfernt,
  weil die Übertragung über das Webinterface zu langsam war.
- Die Dateien wurden anschließend direkt von der SD-Karte in dieses
  Testverzeichnis kopiert.
- In den 127 ausgewerteten Diagnosemeldungen wurden keine Schreibfehler und
  keine verworfenen Datensätze gemeldet.
- Im seriell dokumentierten Zeitraum stieg die Zahl erfolgreicher
  Schreibvorgänge von 2.405 auf 10.772.

### MCP2515 und OBD-Grundfunktion

Vor der Discovery-Messfahrt konnte die grundsätzliche OBD-Kommunikation bereits
nachgewiesen werden. Bei eingeschalteter Zündung und stehendem Motor wurden
Antworten von CAN-ID `0x7E8` empfangen:

- PID `0x0D`: Geschwindigkeit 0 km/h
- PID `0x0C`: Motordrehzahl 0 1/min
- PID `0x11`: Drosselstellung, unter anderem etwa 14,1 %

Damit sind folgende Punkte grundsätzlich plausibel:

- MCP2515-Verkabelung
- gemeinsamer Massebezug
- 16-MHz-Quarzeinstellung
- CAN-Bitrate 500 kbit/s
- Senden einer funktionalen Anfrage über ID `0x7DF`
- Empfang von Antworten im Bereich `0x7E8` bis `0x7EF`

Die ECU antwortete beim Vergleich mit dem OBDLink MX+ erst zuverlässig, nachdem
das Fahrzeug beziehungsweise die ECU vollständig aktiv war. Eine ELM-Verbindung
allein bedeutet daher noch nicht, dass die ECU bereits erreichbar ist.

## Ablauf der Discovery-Messfahrt

Die aufgezeichnete Sitzung enthält folgende Phasen und Markierungen:

| Zeitpunkt ab Logstart | Ereignis |
|---:|---|
| ca. 0 s | Beginn der 60-sekündigen passiven Listen-Only-Phase |
| 26,5 s und 42,7 s | Markierung „Zündung ein, Motor aus“ |
| 60,1 s | Ende der passiven Phase |
| ca. 60–65 s | Zwei Scanrunden für PID-Blöcke 00/20/40/60 |
| ca. 64,8 s | Übergang zur Messwertphase |
| 91,3 s und 93,4 s | Markierung „Motor gestartet“ |
| ca. 207,7 s | Leerlaufmessung |
| 255,9–509,7 s | Erster markierter Fahrabschnitt |
| 509,7–564,1 s | Zwischenhalt |
| 564,1–813,8 s | Zweiter markierter Fahrabschnitt |
| ca. 846,5 s | Ende der aufgezeichneten Messfahrt |

Die serielle Discovery-Anzeige lief einschließlich Vorbereitung bis ungefähr
872 Sekunden. Die CSV-Sitzung selbst umfasst rund 846 Sekunden.

## Vorhandene Messdateien

Für die Sitzung `20260728_211454_F817A5B2` liegen vor:

| Dateiart | Datensätze | Bewertung |
|---|---:|---|
| Event | 70 | vollständig und strukturell korrekt |
| GPS | 2.773 | vollständig, aber mit Fix- und Plausibilitätsproblemen |
| Fahrbahn/Fahrqualität | 813 | vollständig |
| BNO055/Sensor | 7.160 | vollständig |
| Zusammenfassung | 1 | vorhanden |
| CAN-Rohdaten | 0 | keine Frames während der Discovery-Sitzung |
| OBD-Messwerte | 0 | keine Antworten während der Discovery-Sitzung |

Die vorhandenen CSV-Dateien enthalten keine beschädigten Zeilen und keine
unerwartet leeren Felder.

## CAN- und OBD-Auswertung

### Passive Phase

Nach 60 Sekunden Listen-Only wurden gemeldet:

- 0 CAN-Frames
- 0 eindeutige 11-Bit-IDs
- 0 Extended-Frames

Das ist am Porsche-OBD-Anschluss plausibel. Das Diagnose-Gateway leitet offenbar
keinen normalen, unaufgeforderten Fahrzeugverkehr an den OBD-Anschluss weiter.
Fahrzeugdaten müssen daher aktiv über OBD-Service 01 angefragt werden.

### PID-Unterstützungsscan

Keiner der Blöcke `00`, `20`, `40` oder `60` erhielt eine Antwort. Deshalb
blieben folgende PIDs in der Firmware als „unbekannt“ markiert:

- `0x0C` Motordrehzahl
- `0x0D` Geschwindigkeit
- `0x10` Luftmassenstrom
- `0x11` Drosselstellung
- `0x46` Außentemperatur
- `0x5C` Öltemperatur
- `0x5E` Kraftstoffrate

### Erklärung der seriellen Zähler

Die serielle Ausgabe zeigte wiederholt:

```text
OBD: 810 Anfragen, 39 Antworten, 0 PID-Blockantworten, 642 Sendefehler
SD-Sitzung: 20260728_211454_F817A5B2, CAN gesamt in Sitzung: 0
```

Die Werte `810/39/642` sind keine reinen Sitzungszähler. Beim Start einer
Discovery werden derzeit nur die erkannten Unterstützungsblöcke zurückgesetzt,
nicht aber Anfrage-, Antwort- und Sendefehlerzähler.

Aus dem Alter der letzten Antwort ergibt sich:

- letzte erfolgreiche OBD-Antwort bei ungefähr 398 Sekunden Systemlaufzeit
- Start der Discovery ungefähr bei 408 bis 413 Sekunden Systemlaufzeit
- damit lag die letzte Antwort etwa 10 bis 15 Sekunden vor der Discovery

Während der beiden markierten Fahrabschnitte blieben alle OBD-Zähler konstant.
Die 39 Antworten wurden folglich vor der Sitzung empfangen. Es fehlen keine
CAN- oder OBD-Dateien beim Kopieren; während der Sitzung wurde tatsächlich
keine Antwort empfangen.

### Zustand der CAN-Hardware

Im seriell aufgezeichneten Zeitraum von Systemsekunde 657 bis 1290 wurden
126 vollständige Statusmeldungen und 127 CAN-Hardwarediagnosen ausgewertet.
Die Hardwarediagnose war durchgehend:

```text
Modus=Normal, TEC=9, REC=0, EFLG=0x00
```

Damit war der MCP2515 zu diesem Zeitpunkt:

- im Normalmodus
- nicht Bus-off
- nicht Error-Passive
- ohne Empfangsüberlauf
- ohne aktuelles Fehlerflag

Die 642 Sendefehler sind aufsummierte Boot-Zähler. Die aktuelle Firmware
protokolliert nicht, ob genau die acht Discovery-Scan-Anfragen erfolgreich
gesendet wurden. Der Scan erhöht seinen Index auch nach einem fehlgeschlagenen
Sendeversuch und beendet sich anschließend ohne Wiederholung.

### Hauptursache für fehlende OBD-Fahrdaten

Die wahrscheinlichste Fehlerkette ist:

1. Vor der Discovery waren OBD-Antworten grundsätzlich möglich.
2. Die passive Phase empfing erwartungsgemäß keinen Gateway-Verkehr.
3. Der einmalige PID-Scan erhielt keine Antwort oder einzelne Anfragen konnten
   nicht erfolgreich gesendet werden.
4. Der Motor wurde erst etwa 26 Sekunden nach Abschluss des Scans gestartet.
5. Die Firmware wiederholte den Scan nach dem Motorstart nicht.
6. In der Messwertphase werden nur ausdrücklich bestätigte PIDs abgefragt.
7. Weil kein Unterstützungsblock bestätigt war, wurden während der Fahrt keine
   weiteren OBD-Anfragen gestellt.

## GPS-Auswertung

### Positive Beobachtungen

- In guten Abschnitten wurden meist 8 bis 12 Satelliten angezeigt.
- Der zweite markierte Fahrabschnitt ist weitgehend plausibel:
  - Strecke ungefähr 2,25 km
  - Median der Geschwindigkeit etwa 30,6 km/h
  - maximale Geschwindigkeit etwa 104,9 km/h
  - Median HDOP etwa 1,11
  - plausible Höhe überwiegend zwischen ungefähr 143 und 204 m

### Auffälligkeiten

- 16 von 126 seriellen GPS-Statusmeldungen zeigten „Kein Fix“.
- Zweimal wurde eine Koordinate mit 0 Satelliten als Position ausgegeben.
- Umgekehrt erschien „Kein Fix“ teilweise trotz 6 bis 10 angezeigter
  Satelliten.
- 77,4 % der aufeinanderfolgenden GPS-CSV-Zeilen wiederholen dieselbe Position.
- Die größte Lücke zwischen GPS-Datensätzen beträgt rund 103 Sekunden.
- Die Höhe fiel zeitweise bis ungefähr −1.091 m und ist dort eindeutig
  ungültig.
- Es gibt mehrere unplausible Positionssprünge.

Die unterschiedlichen Distanzberechnungen stimmen deshalb nicht ausreichend
überein:

- Firmware-Zusammenfassung: etwa 9,51 km
- Integration der GPS-Geschwindigkeit einschließlich großer Lücken: etwa
  8,03 km
- Integration ohne Lücken über 2 Sekunden: etwa 5,34 km
- Summe der rohen Koordinatenabstände: etwa 10,11 km

Die Gesamtdistanz dieser Sitzung ist daher nicht belastbar. Einzelne gute
Fahrtabschnitte, besonders der zweite Abschnitt, können separat verwendet
werden.

### Wahrscheinliche technische Ursache

Die Firmware betrachtet derzeit bereits eine weniger als fünf Sekunden alte
Position als gültigen Fix. Satellitenzahl, HDOP, Geschwindigkeit, Höhe und Kurs
werden unabhängig voneinander übernommen, sobald TinyGPS++ das jeweilige Feld
irgendwann als gültig markiert hat. Dadurch können neue und alte NMEA-Felder
miteinander kombiniert werden.

Zusätzlich besitzt der GPS-Empfangspuffer keinen sichtbaren Überlaufzähler.
Längere Schleifen- oder SD-Verzögerungen könnten deshalb unbemerkt NMEA-Zeichen
verwerfen. Die vorliegenden Daten beweisen einen solchen Überlauf noch nicht,
zeigen aber, dass dafür zusätzliche Diagnosewerte benötigt werden.

## BNO055- und Fahrbewegungsauswertung

### Abtastrate und Beschleunigung

- 7.160 Sensordatensätze in ungefähr 845 Sekunden
- effektive Rate etwa 8,47 Hz bei vorgesehenen 10 Hz
- Median des Zeitabstands etwa 101 ms
- 95-%-Wert etwa 200 ms
- größte Lücke etwa 1,37 Sekunden
- 180 Lücken größer als 250 ms

Beschleunigungsmagnitude:

- Median: ungefähr 0,29 m/s²
- 95-%-Wert: ungefähr 2,86 m/s²
- 99-%-Wert: ungefähr 4,48 m/s²
- Maximum: ungefähr 8,24 m/s²

Die negative Y-Achse korreliert am ehesten mit der aus GPS abgeleiteten
Längsbeschleunigung. Die explorative Korrelation lag bei ungefähr 0,59, wobei
der BNO055 dem geglätteten GPS-Wert etwa zwei Sekunden vorauslief. Das ist ein
Hinweis auf die Einbaurichtung, aber noch keine abschließende Achskalibrierung.

### Fahrbahn- und Kurvenerkennung

- 29 abgeschlossene Kurven erkannt
- 4 Schlaglochereignisse erkannt
- eines der vier Schlaglochereignisse lag während des markierten Zwischenhalts
  und ist wahrscheinlich eine Bewegung des Fahrzeugs beziehungsweise Geräts im
  Stand
- wahrscheinlich 3 reale Fahrbahnereignisse während der Fahrt
- Fahrqualitätswert:
  - erster Fahrabschnitt ungefähr 93,1
  - zweiter Fahrabschnitt ungefähr 92,4
  - Gesamtzusammenfassung 94,9

Der Gesamtwert ist durch Standzeiten etwas zu positiv. Ereignisse sollten
künftig mit einer Mindestgeschwindigkeit verknüpft werden.

## Stabilitätsbewertung

| Komponente | Ergebnis | Bemerkung |
|---|---|---|
| SD-Karte | Bestanden | keine Fehler, keine verworfenen Datensätze |
| BNO055-Kommunikation | Bestanden | keine I²C-Aussetzer, Selbsttest OK |
| OLED optional | Bestanden | erkannt; Betrieb ohne OLED ebenfalls möglich |
| WLAN/Webseite | Bestanden | während der Diagnose als OK gemeldet |
| MCP2515-Grundfunktion | Bestanden | frühere 0x7E8-Antworten und stabile Diagnose |
| Passive CAN-Erkennung am OBD-Port | Plausibel | Gateway liefert keine spontanen Frames |
| OBD-Discovery während der Fahrt | Nicht bestanden | kein erneuter Scan nach Motorstart |
| GPS-Grundfunktion | Teilweise bestanden | gute Abschnitte, aber Fix- und Plausibilitätsfehler |
| Fahrbewegungserkennung | Teilweise bestanden | grundsätzlich brauchbar, weitere Filter nötig |

## Empfohlene Firmwareänderungen

### Priorität 1: OBD-Discovery

1. OBD-Anfrage-, Antwort- und Fehlerzähler beim Discovery-Start als
   Sitzungszähler zurücksetzen oder einen Startwert speichern.
2. Für jede Scan-Anfrage protokollieren:
   - PID
   - Sendeversuch erfolgreich oder fehlgeschlagen
   - TEC, REC und EFLG
   - Wartezeit und Anzahl empfangener Antworten
3. Einen erfolglosen Unterstützungsblockscan regelmäßig wiederholen.
4. Nach einer Markierung wie `motor_gestartet` sofort einen neuen Scan
   auslösen.
5. „Unterstützung unbekannt“ nicht wie „nicht unterstützt“ behandeln.
6. Als sicheren Rückfall mindestens `0x0C`, `0x0D` und `0x11` in niedriger
   Rate direkt testen.
7. Erst dann in den dauerhaften Messmodus wechseln, wenn entweder eine ECU
   geantwortet hat oder der Benutzer den Modus ausdrücklich fortsetzt.

### Priorität 1: GPS-Qualität

1. Für einen gültigen Fix zusätzlich fordern:
   - frische Position
   - mindestens vier Satelliten
   - plausiblen HDOP, zum Beispiel kleiner oder gleich 5
2. Alter und Gültigkeit jedes NMEA-Feldes einzeln prüfen.
3. Unplausible Höhen, Geschwindigkeiten und Positionssprünge ablehnen.
4. GPS-Ringpufferüberläufe zählen und seriell sowie auf SD protokollieren.
5. Nur neue GPS-Fixes oder maximal etwa einen Datensatz pro Sekunde speichern,
   statt dieselbe Position mehrfach abzulegen.

### Priorität 2: Messqualität

1. Schlagloch- und Fahrbahnereignisse nur oberhalb einer
   Mindestgeschwindigkeit als Fahrereignis werten.
2. Standzeiten separat von der Fahrqualitätsbewertung behandeln.
3. Längere Schleifenpausen und die effektive Abtastrate pro Datenquelle
   überwachen.
4. Einbaurichtung und Vorzeichen der BNO055-Achsen in einem kurzen,
   kontrollierten Beschleunigungs- und Bremstest bestätigen.

## Empfohlener nächster Fahrzeugtest

1. Adapter anschließen und serielle Ausgabe öffnen.
2. Prüfen, dass SD, BNO055, GPS und MCP2515 bereit sind.
3. Zündung einschalten und anschließend den Motor vollständig starten.
4. Vor Beginn der Discovery mit wenigen Anfragen auf `0x0C`, `0x0D` und
   `0x11` prüfen, dass der ECU-Antwortzähler steigt.
5. Erst danach `discover begin` starten.
6. Nach der passiven Phase kontrollieren, dass mindestens ein
   PID-Unterstützungsblock oder ein Rückfall-PID antwortet.
7. Eine kurze Standmessung durchführen.
8. Eine kurze, sichere Testfahrt mit klaren Markierungen durchführen.
9. Prüfen, dass Anfrage- und Antwortzähler während der Fahrt weiter steigen.
10. `discover end` ausführen und kontrollieren, dass Sensor-, GPS-, Event-,
    CAN- und OBD-Dateien für dieselbe Sitzungs-ID vorhanden sind.

## Gesamtbewertung

Der Test war trotz fehlender OBD-Fahrdaten wertvoll. Er bestätigt, dass der
Roadtest-Adapter mechanisch und elektrisch grundsätzlich funktioniert, die
Aufzeichnung stabil läuft und der Porsche über standardisierte OBD-Anfragen
antworten kann. Die wichtigsten verbleibenden Probleme liegen in der
Discovery-Zustandslogik und in der GPS-Datenvalidierung, nicht im SD-Logger
oder in einem generellen Ausfall des MCP2515.

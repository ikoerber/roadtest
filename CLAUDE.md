# ROADTEST Firmware

ESP32-S3-Firmware zur Aufzeichnung von BNO055-, GPS-, Straßenqualitäts- und
standardisierten OBD-II-Daten. Aktueller Firmwarestand: **1.5.29**.

Der aktuelle Stand ist ein Hardware- und Fahrzeugteststand, nicht abschließend
produktionsreif. Bekannte Einschränkungen stehen weiter unten.

## Befehle

| Befehl | Zweck |
|---|---|
| `pio run` | Firmware für `lolin_s3_mini` bauen |
| `pio run -t upload` | Firmware über USB-CDC flashen |
| `pio device monitor` | Seriellen Monitor mit den Einstellungen aus `platformio.ini` öffnen |
| `pio run -t clean` | PlatformIO-Buildartefakte löschen |
| `pio test -e native` | Hosttests der hardwarefreien Auswertelogik ausführen |
| `python3 tools/export_geojson.py <Sitzung>` | Messsitzung als Karte für geojson.io exportieren |
| `c++ -std=gnu++17 -O1 -I src -o /tmp/vergleich tools/vergleich_kurvenwiedergabe.cpp src/curve_detector.cpp` | Wiedergabe gegen die Firmwareereignisse derselben Fahrt prüfen |

Die Web-OTA-Datei entsteht unter:

```text
.pio/build/lolin_s3_mini/roadtest_<Version>.bin
roadtest_<Version>.bin
```

Der Build liest `ROADTEST_FIRMWARE_VERSION` über
`scripts/version_firmware.py`, verwendet sie als Buildnamen und exportiert
dieselbe Datei zusätzlich ins Projektverzeichnis. Außerdem verwendet er
`scripts/patch_bno055_reset_timeout.py`. Diese Schritte nicht umgehen.

## Architektur

```text
src/main.cpp                    Systemstart, Hauptschleife und serielle Befehle
src/hardware_config.{h,cpp}     Verbindliche Pins, Intervalle und Grenzwerte
src/bno055_manager.{h,cpp}      NDOF-Sensorfusion, Status und NVS-Kalibrierung
src/gps_manager.{h,cpp}         BN-880-UART, TinyGPS++ und GPS-Zustand
src/MCP2515.{h,cpp}             Lokaler MCP2515-Treiber
src/CANController.{h,cpp}       Abstraktion des CAN-Controllers
src/can_reader.{h,cpp}          CAN-Empfang und erlaubte OBD-Service-01-PIDs
src/vehicle_data_discovery.*    Dreiphasige Fahrzeugdaten-Erkennung
src/sd_logger.{h,cpp}           Sitzungsbezogene CSV-Aufzeichnung
src/curve_detector.{h,cpp}      Kurvenerkennung, hardwarefrei und hosttestbar
src/road_metrics.{h,cpp}        Vibration, Straßenqualität, Schlaglöcher;
                                ebenfalls hardwarefrei und hosttestbar
src/road_quality.h              Datenstruktur RoadMetrics für Auswertungen
src/oled_manager.{h,cpp}        Optionale Statusanzeige
src/web_manager.{h,cpp}         ROADTEST-WLAN, Statusseite und Browser-OTA
src/runtime_diagnostics.{h,cpp} Laufzeit-, Web- und SD-Pausendiagnose
src/integration_tests.{h,cpp}   Serielle Hardware- und Integrationstests
```

Systemablauf:

1. WLAN und Webseite starten vor den externen Hardwareprüfungen.
2. BNO055, GPS, SD und optionales CAN werden nicht blockierend geprüft.
3. Das OLED ist reine Komfortausstattung und darf den Betrieb nie blockieren.
4. Die Hauptschleife bedient zuerst Web/OTA und anschließend Sensoren,
   Logging, CAN/OBD und Diagnose.
5. `VehicleDataDiscovery` übernimmt während einer Discovery-Sitzung die
   CAN-Modi, OBD-Anfragen und SD-Aufzeichnung.

## Verbindliche Hardware

Die vollständigen Anschluss- und Sicherheitshinweise stehen in
`HARDWARE.md`; `schematic.md` enthält den logischen Plan.

| Funktion | Pin |
|---|---:|
| I²C SDA für BNO055 und OLED | GPIO 8 |
| I²C SCL für BNO055 und OLED | GPIO 9 |
| GPS RX am ESP32 | GPIO 16 |
| GPS TX am ESP32 | GPIO 15 |
| SD CS | GPIO 4 |
| SD MOSI | GPIO 5 |
| SD MISO | GPIO 6 |
| SD SCK | GPIO 7 |
| MCP2515 CS | GPIO 1 |
| MCP2515 INT | GPIO 2 |
| MCP2515 SCK | GPIO 3 |
| MCP2515 MOSI/SI | GPIO 13 |
| MCP2515 MISO/SO | GPIO 11 |

Wichtige Invarianten:

- Zielboard ist ein LOLIN S3 Mini mit ESP32-S3 und 4 MB Flash.
- Der BNO055 läuft im Modus `NDOF`.
- Das MCP2515-Modul verwendet fest einen 16-MHz-Quarz und 500 kbit/s.
- SD und MCP2515 verwenden getrennte SPI-Pins.
- ESP32-S3-GPIOs sind nicht 5-V-tolerant.
- Am Fahrzeug sind gemeinsame Masse, kurze CAN-Stichleitung und die
  Versorgungshinweise aus `HARDWARE.md` zwingend.
- Der Terminierungsjumper P1 am CAN-Modul bleibt beim Fahrzeuganschluss
  entfernt.
- Hardware-Pins nicht aufgrund automatischer Erkennung oder Vermutungen
  ändern.

## CAN- und OBD-Sicherheit

- CAN ist für die allgemeine Systembereitschaft optional.
- Die Firmware sendet ausschließlich freigegebene, lesende OBD-Service-01-
  Anfragen über die funktionale ID `0x7DF`.
- Antworten werden nur im Bereich `0x7E8` bis `0x7EF` verarbeitet.
- Erlaubte PIDs stehen als Positivliste in `CANReader::requestOBDPid()`.
- Keine frei wählbaren CAN-Frames, UDS-Schreibdienste, Codierungen,
  Stellgliedtests oder Fehlerlöschfunktionen ergänzen.
- Die Gesamtrate aktiver OBD-Anfragen bleibt auf höchstens zwei Frames pro
  Sekunde begrenzt.
- Während Browser-OTA CAN-H und CAN-L vom Fahrzeug trennen.

## Fahrzeugdaten-Erkennung

Serielle Befehle:

```text
discover begin
discover status
discover mark <kurze Beschreibung>
discover end
```

Der Ablauf besteht aus:

1. 60 Sekunden echtem MCP2515-Listen-Only ohne ACKs oder Anfragen.
2. Zwei Scanrunden der OBD-Unterstützungsblöcke `00/20/40/60`.
3. Abfrage ausschließlich bestätigter Standard-PIDs bis `discover end`.

Eine Discovery-Sitzung erzeugt nach Bedarf Sensor-, Straßen-, GPS-, Event-,
CAN-, OBD- und Zusammenfassungsdateien mit gemeinsamer Sitzungs-ID.

## Serielle Diagnose und Tests

`test` zeigt alle verfügbaren seriellen Befehle. Für die normale Arbeit sind
besonders relevant:

```text
diag
hardware
quick
integration
stress
recovery
sdrecovery
buffer
memory
calibration
clear_cal
start
stop
obd on
obd off
test begin
test status
test end
```

Die Integrationstests sind Firmwaretests mit realer Hardware und teilweise
interaktiven Schritten. Es gibt derzeit keine belastbare automatisierte
Coverage-Zahl. Keine erfundenen Qualitäts- oder Abdeckungswerte dokumentieren.

## Hosttests

Die Auswertelogik der Kurvenerkennung liegt in `src/curve_detector.{h,cpp}`,
die der Fahrbahnbewertung in `src/road_metrics.{h,cpp}`. Beide Einheiten sind
frei von Arduino-, Sensor- und Zeitabhängigkeiten. `BNO055Manager` reicht nur
noch die Sensorwerte durch. Getestet wird über die Umgebung `native` auf dem
Entwicklungsrechner:

```bash
pio test -e native
```

Diese Tests belegen keinen Byte Firmware-Flash und laufen in unter zwei
Sekunden. `test/test_curve_detector/` prüft Kurvenerkennung,
`test/test_road_metrics/` Vibrationsanalyse, Straßenqualität und
Schlaglocherkennung. Beide enthalten Regressionsschutztests gegen Ereignisse
im Stillstand sowie Wiedergaben realer Fahrten aus `testdata/`.

Die Wiedergaben sind auf feste Kennzahlen festgeschrieben. Ändern sie sich,
ist das kein Testfehler, sondern eine Verhaltensänderung: prüfen, begründen
und den Festwert bewusst nachziehen.

`test_wiedergabe_referenzfahrten` misst die Kurvenerkennung gegen die
55 von Hand markierten Referenzkurven der fünf Beifahrerfahrten vom
31.07.2026. Die Geschwindigkeit wird dabei wie in `main.cpp` gewählt: zuerst
OBD, sonst GPS, sonst unbekannt. Diese Reihenfolge ist verbindlich, denn nur
mit ihr gibt die Wiedergabe die Firmwareläufe ereignisgenau wieder — 174 zu
174 Ereignissen bei 0,1 Grad mittlerer Winkelabweichung. Allein mit der
GPS-Geschwindigkeit waren es 128 statt 174. Neben der Trefferzahl ist die Zahl der Ereignisse mit einer
Zeitabdeckung unter 60 Prozent festgeschrieben: Ereignisse, deren Dauer nur
zu einem kleinen Teil aus echten Drehstichproben besteht, umfassen mehrere
Kurven, und ihr Radius beschreibt dann nichts Bestimmtes mehr. Dieser
Prüfstand ist die Messgrundlage für weitere Parameteränderungen an der
Kurvenerkennung.

Neue Schwellwerte gehören mit einem Fall knapp darüber und knapp darunter
abgesichert. Ein Mutationstest beim Aufbau der Suite zeigte, dass weit von
der Schwelle entfernte Testfälle eine Parameteränderung nicht bemerken.

Vor einem Commit mindestens ausführen:

```bash
pio run
pio test -e native
git diff --check
```

## Auswertung als Karte

`tools/export_geojson.py` wandelt eine Messsitzung in GeoJSON. Die Datei wird
per Drag-and-drop in <https://geojson.io> geöffnet; die Einfärbung folgt der
simplestyle-Spezifikation, alle übrigen Werte erscheinen beim Anklicken.

```bash
python3 tools/export_geojson.py testdata/20260730_095603_A41A2450
python3 tools/export_geojson.py testdata/20260730_095603_A41A2450 --modus geschwindigkeit
```

Ebenen: Strecke nach Straßenqualität oder nach der Abweichung zwischen GPS-
und OBD-Geschwindigkeit eingefärbt, Schlaglöcher als Marker nach Schwere,
Kurven als tatsächlicher Bogen (ab Schema 1.5.28 mit Anfangs- und Endzeit,
sonst als Punkt), Referenzintervalle des Beifahrer-Kurventests sowie ein
Sitzungskopf mit Firmware, Kennzahlen und Laufzeitdiagnose.

Verbindliche Regeln der Auswertung:

- Nur nachgewiesen gültige Positionen werden verwendet.
- Über eine Qualitäts- oder Zeitlücke hinweg wird keine Linie gezogen.
- Ereignisse ohne eigene Position werden über die Zeit aus der GPS-Spur
  verortet; die von der Firmware geschriebene Koordinate 0/0 gilt als
  unbekannt und wird nicht übernommen.
- Unbekannte Werte erscheinen als `null`, niemals als 0.

Die Breitengrade sind als `float32` gespeichert und dadurch in rund 0,33-m-
Stufen gerastert. Treppenstufen in der Karte bei langsamer Fahrt stammen aus
diesem Datentyp, nicht aus dem GPS-Empfang. Der Hinweis steht auch in den
Eigenschaften des Sitzungskopfes.

## Aktueller Teststand

- Firmware 1.5.29 baut erfolgreich für `lolin_s3_mini`.
- Letzter Build: 71.060 Byte RAM (21,7 %) und 1.249.082 Byte Flash (95,3 %).
- Am 31.07.2026 liefen fünf Beifahrer-Referenzfahrten mit 1.5.28 und je zwölf
  markierten Referenzintervallen. Sie sind der Prüfstand der Kurvenerkennung.
- 1.5.29 ist am Fahrzeug bestätigt: vier Fahrten am 01.08.2026 über 21 Minuten
  und 9,1 km, Zeitabdeckung der Ereignisse 97 % im Median gegenüber 67 % mit
  1.5.28, nur 1 von 66 Ereignissen unter 60 %, keine Ausreißer über die
  Höchstdauer, kein Ereignis im Stand. Der Abgleich der Wiedergabe gegen die
  Firmwareereignisse ordnete 65 von 66 zu, drei der vier Fahrten
  ereignisgenau bei 0,0 bis 0,1 Grad.
- Offen bleibt der Abschlussgrund `SESSION_END`: Er greift nur, wenn die
  Aufzeichnung gestoppt wird, während das Fahrzeug über 5 km/h fährt und eine
  Kurve läuft. Wird vorher angehalten, schließt bereits das
  Geschwindigkeitsgatter die Kurve als `QUIET` ab. Zum Prüfen einen Kreis bei
  etwa 15 km/h fahren und im Kreisen stoppen, ohne vorher auszurollen.
- Zwei SD-Abbrüche desselben Tages wurden verlustfrei aufgefangen:
  `20260731_142359_514EDDF4` mit 38 von 38 geretteten Pufferzeilen nach
  81 Sekunden Ausfall und `20260731_160032_3D732AD1` mit 17 von 17 nach
  11 Sekunden. Die Fortsetzung schloss zeitlich lückenlos und ohne Dublette
  an. Die abgebrochene Sitzung bleibt jedoch ohne END-Record und ohne
  Zusammenfassung; deren Kennzahlen sind nachträglich zu rekonstruieren.
- BNO055-Selbsttest, SD-Logging, WLAN, optionales OLED und MCP2515-
  Grundkommunikation wurden am System geprüft.
- Am Porsche Carrera S, Baujahr 2012, PDK wurden standardisierte Antworten von
  CAN-ID `0x7E8` für Drehzahl, Geschwindigkeit und Drosselstellung empfangen.
- Der ausführliche Fahrzeugtest steht in
  `testdata/20260728_2300/TESTBERICHT_2026-07-28.md`.
- Die aktuelle GPS-/OBD-Abnahme steht in
  `testdata/20260729_170946_05A4DB46/ABNAHME_AUSWERTUNG.md`.
- Der Recovery-Kontrolllauf `20260730_101000_5901D247` bestätigte mit 1.5.25
  beide Zündungs- und Motorstartschritte sowie ECU-Ausfall und Wiederanlauf.
  Gleichzeitig zeigte er, dass ein verbliebenes halbes Loggergatter weiterhin
  72 Sensor- und 21 GPS-Zeilen verwarf. 1.5.26 entfernt dieses zweite
  Zeitgatter vollständig; die Wirksamkeit ist noch am Gerät zu bestätigen.
- Die Überlandfahrt `20260730_095603_A41A2450` wurde mit ausdrücklicher
  Zustimmung über OSRM auf das Straßennetz abgeglichen. 13 von 37
  kartenseitigen Kurvenabschnitten besaßen kein zeitnahes Firmwareereignis;
  1.5.27 ergänzt dafür einen kumulativen Langkurvenpfad.
- Die Sitzung `20260730_093525_F94C8878` brach bei einem SD-Fehler ohne
  Abschlussmetadaten ab; 48 gepufferte Sensorzeilen gingen verloren. 1.5.27
  sichert solche Puffer nach dem Wiedereinbinden separat und setzt eine
  laufende Messung in einer verknüpften Folgesitzung fort.

## Verbindlicher Datenqualitätsfokus

Kurvenerkennung ist der primäre Fokus der nächsten Fahrzeugfahrt. Firmware
1.5.28 zeichnet dafür einbaulagenunabhängige Drehrate, vollständige
Kurvenintervalle, Radius, Querbeschleunigung und manuelle
Beifahrer-Referenzintervalle auf. GPS und CAN/OBD laufen gleichzeitig als
Zeit-, Geschwindigkeits- und Positionsreferenz mit.

Der verbindliche Umsetzungs-, Test- und Abnahmeplan steht in
`GPS_CAN_OBD_DATENQUALITAETSPLAN.md`. Neue Messfunktionen müssen dessen
Sitzungszähler, Gültigkeitsfelder, Zeitbasis und PASS/WARN/FAIL-Kriterien
berücksichtigen.

BNO055, SD, OLED und WLAN laufen bei diesen Tests als Stabilitätskontrolle mit.
Nach der Kurvenauswertung folgt die getrennte Verbesserung der
Straßenqualitätsbewertung anhand derselben Rohfahrt. Beide Bewertungsmodelle
werden nicht gleichzeitig neu parametriert.

## Bekannte Einschränkungen

### OBD-Discovery

- 1.5.25 führt einen eigenen ECU-Zustand und eine begrenzte Wiedererkennung
  nach erfolglosem Scan oder drei aufeinanderfolgenden tatsächlichen
  Anfragefehlern. Eine reine lokale Programmpause löst keinen ECU-Verlust aus.
- Die sichere Rückfallrunde verwendet ausschließlich `0x0C`, `0x0D`, `0x11`
  und `0x00`, bleibt bei höchstens zwei Anfragen pro Sekunde und scannt nach
  der ersten Antwort die Unterstützungsblöcke erneut.
- MCP2515-RX0-/RX1-Überläufe werden einzeln gezählt und nach dem Protokollieren
  kontrolliert zurückgesetzt.
- Wiederholter Bus-off oder ein falscher MCP2515-Betriebsmodus löst eine
  begrenzte Controller-Recovery aus; deren Sitzungsanzahl wird protokolliert.
- Der Abnahmetest `20260729_162734_92A51444` bestätigte die getrennte
  Zündungs- und Motorlauferkennung aus 1.5.20 einschließlich ECU-Ausfall und
  Wiederanlauf am Fahrzeug.
- Die Vergleichsfahrt `20260729_170946_05A4DB46` bestätigte GPS-/OBD-
  Geschwindigkeit und Strecke. Die dabei beobachteten, webseitig ausgelösten
  Messpausen werden mit 1.5.23 gezielt vermieden und protokolliert.
- Falls der zweite Browsermarker verloren geht, bestätigt 1.5.24 den
  Motorneustart automatisch aus einer frischen OBD-Drehzahl ab 300 U/min.
  Webaktionen werden mit ihrem Ergebnis protokolliert.
- Ein nach dem Motor-Aus-Marker beobachteter ECU-Ausfall bleibt in 1.5.25
  auch nach erfolgreicher Wiedererkennung als abgeschlossener Abnahmeschritt
  erhalten.

Die Beifahrerfahrten dieser Art sind am 31.07.2026 fünfmal gelaufen und
liegen als Prüfstand in `testdata/` vor. Die Bestätigungsfahrt mit 1.5.29 lief
am 01.08.2026 in vier Sitzungen und belegte die geschärften Ereignisgrenzen.
Referenzmarker braucht eine solche Fahrt nicht: Die Schwellwerte sind über die
Fahrten vom 31.07.2026 bestimmt und im Hosttest festgeschrieben; eine
Bestätigungsfahrt prüft die Übereinstimmung von Gerät und Wiedergabe, und dafür
genügt normales Fahren. Der vollständige ECU-Recovery-Ablauf muss dabei nicht
erneut durchgeführt werden.

Offen ist allein noch eine zweiminütige Parkplatzrunde für `SESSION_END`.

### GPS

- 1.5.21 koppelt die Positionsfreigabe an Feldalter, mindestens fünf
  Satelliten und HDOP höchstens 3,5; Geschwindigkeit, Höhe und Kurs besitzen
  eigene Alters- und Plausibilitätsgrenzen.
- GPS-Geschwindigkeit unter 6 km/h gilt nicht als zuverlässiger
  Bewegungsnachweis. Bei einer frischen OBD-Geschwindigkeit bis 1 km/h wird
  GPS-Drift nicht als Strecke aufsummiert.
- Rohwerte bleiben bewusst im Qualitäts-Snapshot erhalten. Nur die
  feldbezogenen Valid-Flags dürfen für Auswertungen verwendet werden;
  `RejectionReason` ist eine kombinierbare Bitmaske.
- Die Aufzeichnung schreibt bewusst alle 200 ms einen Qualitäts-Snapshot;
  `NewFix` zählt seit 1.5.24 eindeutige GNSS-Zeitepochen statt mehrfacher
  RMC-/GGA-Commits derselben Epoche.

Der Einbau unter dem Vordersitz wurde in `20260729_170946_05A4DB46` mit
10 bis 12 Satelliten, HDOP-Median 0,89 und vollständiger zeitlicher
GPS-/OBD-Zuordnung erfolgreich geprüft. Eine allgemeine
Positionssprungbewertung und ereignisorientiertes Logging bleiben offen.

### Messqualität

- Effektive BNO055-Abtastrate lag im Fahrzeugtest mit 1.5.22 bei ungefähr 8,5
  statt vorgesehenen 10 Hz. 1.5.24 vergrößert den Sensorpuffer, hält die
  Zeitplanphase stabil und zählt verfehlte Sensor- und GPS-Slots. 1.5.26
  entfernt die zweite Zeitbegrenzung im Logger; die vollständige
  Zeilenausgabe muss noch am Gerät bestätigt werden.
- Straßenqualität, Schlaglöcher und Kurven benötigen seit 1.5.26 eine
  nachgewiesene Fahrzeuggeschwindigkeit von mindestens 5 km/h.
- 1.5.27 erkennt zusätzlich langsame Kurven über mindestens 8 Grad
  Netto-Kursänderung und 20 Meter Fahrweg innerhalb von zehn Sekunden.
  1.5.28 verwendet dafür die auf die Schwerkraftrichtung projizierte
  Gyroskop-Drehrate, protokolliert vollständige Ereignisintervalle und nutzt
  das relative Heading als gekennzeichneten Rückfall und Qualitätsvergleich.
  Die fünf Beifahrerfahrten vom 31.07.2026 bestätigten den Gyro-Pfad: kein
  einziger Heading-Rückfall in 174 Ereignissen, 96 Prozent der markierten
  Referenzkurven erkannt.
- 1.5.29 schärft die Ereignisgrenzen. Das Ruhefenster bewertet die
  Netto-Kursänderung statt einzelner Stichproben, weil die Gierrate unter
  Fahrt über `CURVE_LONG_MIN_RATE_DPS` hinaus rauscht und Ereignisse dadurch
  über mehrere Kurven hinweg zusammenliefen. Eine beim Stoppen laufende
  Kurve wird als `SESSION_END` abgeschlossen statt verworfen, und der
  Detektor wird beim Messstart zurückgestellt. Wirksamkeit am Gerät noch
  nicht bestätigt.
- Ein Kurvenereignis benötigt zusätzlich mindestens 10 Meter Fahrweg und
  0,4 m/s² mittlere Querbeschleunigung. Die Geschwindigkeitsfreigabe allein
  reicht nicht, weil GPS-Drift im Stand bis 8 km/h als gültig ausgewiesen
  wird. Der Radius ist ausdrücklich kein Kriterium: Ein Bogen mit 500 Meter
  Radius bei 90 km/h ist eine markierte Referenzkurve, ein Autobahnbogen mit
  1450 Meter Radius bei 116 km/h nicht; unterscheiden lassen sie sich nur
  über die Querbeschleunigung.
- Schwellwerte der Kurvenerkennung nur gegen
  `test_wiedergabe_referenzfahrten` ändern, nie nach Augenschein. Die
  Referenzintervalle der Beifahrerseite sind gut, aber nicht fehlerfrei:
  einzelne Marker sitzen mehrere Sekunden versetzt, und drei markierte
  Kurven drehen netto weniger als 10 Grad.
- Ein vorübergehender SD-Ausfall wird in derselben Gerätesitzung automatisch
  durch eine separate Puffer-Recovery-Datei und eine neue, verknüpfte
  Messsitzung behandelt. Nach Reset oder Spannungsverlust bleibt ein
  manueller Messstart verbindlich.
- Absolute Kurswerte benötigen eine ausreichende Magnetometer- und
  Systemkalibrierung; relative Beschleunigungen bleiben davon weitgehend
  unabhängig.

## Entwicklungsregeln

- Bestehende, fest verdrahtete Pinbelegung beibehalten.
- Hardwarezugriffe nicht blockierend halten; WLAN, Webseite und OTA müssen
  erreichbar bleiben.
- OLED und CAN dürfen die allgemeine Bereitschaft nicht blockieren.
- SD-Aufzeichnung über die vorhandene nicht blockierende Startlogik öffnen.
- Die entfernte Web-Downloadfunktion für SD-Dateien nicht wieder einführen;
  sie war auf dem Gerät zu langsam.
- GPS- oder OBD-Werte nur als gültig ausgeben, wenn ihre Aktualität
  nachweisbar ist; unbekannte Werte nicht als Nullwert ausgeben.
- Firmwareversion zentral in `src/hardware_config.h` sowie `CHANGELOG.md` und
  `README.md` gemeinsam aktualisieren; Web, OLED und Bootmeldung verwenden
  dieses zentrale Makro. Der Firmwaredateiname wird beim Build automatisch
  daraus erzeugt und darf nicht manuell abweichend benannt werden.
- Rohmessungen, Fotos, `.sal`-Dateien, `.codex/`, `.claude/` und temporäre
  Dateien nur auf ausdrückliche Anforderung committen.
- Bestehende Nutzeränderungen im Arbeitsbaum nicht verwerfen oder
  überschreiben.

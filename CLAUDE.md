# ROADTEST Firmware

ESP32-S3-Firmware zur Aufzeichnung von BNO055-, GPS-, Straßenqualitäts- und
standardisierten OBD-II-Daten. Aktueller Firmwarestand: **1.5.43**.

Der aktuelle Stand ist ein Hardware- und Fahrzeugteststand, nicht abschließend
produktionsreif. Bekannte Einschränkungen stehen weiter unten.

## Projektziel

Die Firmware ist Mittel, nicht Zweck. Ziel ist eine **Streckendatenbank**: Aus
mühelos aufgezeichneten Fahrten entsteht auf dem Rechner eine Bewertung der
befahrenen Straßen, aus der sich später neue Strecken zusammensetzen lassen,
die mit Freude zu fahren sind.

**Die verbindliche Spezifikation steht in `STRECKENDATENBANK.md`.** Sie legt
Ergebnis, Einheit, Bewertungsablauf und Abnahmekriterien fest. Bei Zweifeln
über den Nutzen einer Änderung entscheidet dieses Dokument.

Der aktuelle Fokus ist **Stufe 1: die bewertete Karte**, und dort die
Zuordnung der Fahrten zum Straßennetz. Am Gerät ist das selbsttätige Starten
seit 1.5.42 erledigt, der Datenzugriff übers Handy seit 1.5.43; offen bleibt
der OTA-Rücksprung.

Diese Codebasis lädt zum Abschweifen ein - Flush-Zeiten, Kartenlatenz,
GPS-Aussetzer, OBD-Discovery, Kompression. Jedes dieser Themen ist für sich
interessant und keines bringt die Streckendatenbank näher. Verbindlich gilt
deshalb:

- **Messqualität ist kein Selbstzweck.** Eine Verbesserung ist zu begründen mit
  dem, was sie an der Streckendatenbank ändert. 2,1 Prozent verfehlte
  Sensorslots sind für eine Kurve über 200 Meter ohne Bedeutung.
- **Ein Fehler ohne zweiten Fall wird nicht weiterverfolgt.** Erst ein
  wiederholtes Auftreten eröffnet die Suche - so ist der stille SD-Verlust am
  04.08.2026 abgeschlossen worden.
- **Neue Funktionen am Gerät brauchen einen Eintrag in `STRECKENDATENBANK.md`.**
  Was dort nicht gefordert ist, gehört auf den Rechner oder gar nicht gebaut.

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
| `python3 tools/vergleich_hin_rueck.py <Sitzung>` | Kurvenerkennung einer hin und zurück gefahrenen Strecke gegen sich selbst prüfen |

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
src/sd_logger.{h,cpp}           Sitzungsbezogene CSV-Aufzeichnung
src/curve_detector.{h,cpp}      Kurvenerkennung, hardwarefrei und hosttestbar
src/road_metrics.{h,cpp}        Vibration, Straßenqualität, Schlaglöcher;
                                ebenfalls hardwarefrei und hosttestbar
src/road_quality.h              Datenstruktur RoadMetrics für Auswertungen
src/web_manager.{h,cpp}         ROADTEST-WLAN, Statusseite, Browser-OTA und
                                Datenzugriff /files
src/gzip_stream.{h,cpp}         gzip über den ROM-Kompressor, austauschbare Senke
src/tar_writer.{h,cpp}          tar-Rahmung, hardwarefrei und hosttestbar
src/runtime_diagnostics.{h,cpp} Laufzeit-, Web- und SD-Pausendiagnose
```

Systemablauf:

1. WLAN und Webseite starten vor den externen Hardwareprüfungen.
2. BNO055, GPS, SD und optionales CAN werden nicht blockierend geprüft.
3. Die Aufzeichnung startet seit 1.5.42 selbsttätig, sobald die SD-Karte
   bereit ist - einmal je Gerätestart, über die nicht blockierende
   Startlogik. Zündungsgeschaltete Spannung ist das Startsignal; `stop` und
   `/ride/stop` behalten das letzte Wort.
4. Das OLED ist seit dem 03.08.2026 ausgebaut. Der BNO055 ist damit der
   einzige I²C-Teilnehmer; Bus- und Sensorfehler sind über den Bus allein
   nicht mehr zu trennen.
5. Die Hauptschleife bedient zuerst Web/OTA und anschließend Sensoren,
   Logging, CAN/OBD und Diagnose.

## Verbindliche Hardware

Die vollständigen Anschluss- und Sicherheitshinweise stehen in
`HARDWARE.md`; `schematic.md` enthält den logischen Plan.

| Funktion | Pin |
|---|---:|
| I²C SDA für BNO055 | GPIO 8 |
| I²C SCL für BNO055 | GPIO 9 |
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

**Mit 1.5.41 entfernt.** Die dreiphasige Suche - Listen-Only-Mitschnitt, Scan
der Unterstützungsblöcke, Abfrage bestätigter PIDs - hat ihren Zweck erfüllt:
Die unterstützten Standard-PIDs des Fahrzeugs sind bekannt und am Porsche
Carrera S bestätigt. Die Befehle `discover begin`, `discover status`,
`discover mark` und `discover end` gibt es nicht mehr.

Die Firmware fragt im Betrieb zyklisch `0x0C`, `0x0D` und `0x11` ab. Wird
einmal ein anderes Fahrzeug angeschlossen, ist die Erkennung aus der Historie
zu holen, statt sie neu zu schreiben.

## Serielle Diagnose und Tests

`test` zeigt alle verfügbaren seriellen Befehle. Für die normale Arbeit sind
besonders relevant:

```text
diag
hardware
buffer
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

`hardware` fasst I²C-Scanner, BNO055-Prüfung, SD-Test mit sicheren Pins und
Puffersicherheit zusammen; diese Funktionen liegen in `main.cpp`. Die
interaktive Testsuite ist mit 1.5.41 entfernt - sie war mit 59,3 kByte das
größte Modul der Firmware und wartete teilweise auf Eingaben an der Konsole,
was zu einem eingebauten Gerät nicht mehr passt. Es gibt keine belastbare
automatisierte Coverage-Zahl. Keine erfundenen Qualitäts- oder
Abdeckungswerte dokumentieren.

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
Schlaglocherkennung, `test/test_tar_writer/` die tar-Rahmung des
Datenzugriffs gegen das echte `tar`. Beide enthalten Regressionsschutztests gegen Ereignisse
im Stillstand sowie Wiedergaben realer Fahrten aus `testdata/`.

Die Wiedergaben sind auf feste Kennzahlen festgeschrieben. Ändern sie sich,
ist das kein Testfehler, sondern eine Verhaltensänderung: prüfen, begründen
und den Festwert bewusst nachziehen.

**Ein Testlauf mit `skipped` ist nicht grün.** Die vier Wiedergabetests
überspringen sich selbst, wenn ihre Messdaten fehlen, denn die CSV-Dateien
sind per `.gitignore` bewusst nicht im Repository und fehlen in einem frischen
Clone. Am 02.08.2026 wanderten sieben Sitzungen nach `testdata/archiv/`, und
die Suite meldete weiterhin Erfolg — bei abgeschaltetem Prüfstand.
`findFixture()` sucht seitdem auch im Archiv. Vor jeder Schwellwertänderung an
der Kurvenerkennung gilt: Der Lauf muss **49 von 49** melden, nicht
„45 succeeded, 4 skipped“.

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

Daneben steht eine zweite, unabhängige Messung: `tools/vergleich_hin_rueck.py`
vergleicht die Kurven einer hin und zurück gefahrenen Strecke gegen sich
selbst. Jede Kurve wird zweimal durchfahren, mit anderer Richtung, Linie und
Geschwindigkeit; stimmen die Winkel überein, misst die Erkennung die Straße
und nicht die Fahrweise. Das braucht keine markierten Referenzintervalle. Die
Zuordnung läuft über die Kurvenfolge statt über die Position, weil Ereignisse
ohne GPS-Fix keine Koordinate tragen. Am 02.08.2026 ergab das über acht
gemeinsame Kurven 1,0 Grad Abweichung im Median.

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
python3 tools/export_geojson.py testdata/20260802_113909_C8F1C2F3 --nur-fahrbarkeit
python3 tools/export_geojson.py testdata/20260802_113909_C8F1C2F3 --ebenen fahrbarkeit,schlagloch
```

`--ebenen` wählt aus `strecke`, `referenz`, `fahrbarkeit`, `kurve`,
`schlagloch`, `beifahrerurteil`, `belagswechsel` und `sitzung`; ohne Angabe
erscheinen alle. `--nur-fahrbarkeit` ist die Kurzform für die
Fahrbarkeitsansicht samt Urteilen und Sitzungskopf. Das lohnt sich: Eine
52-Kilometer-Fahrt ergibt vollständig rund 2.500 Merkmale und 1,8 MB, womit
geojson.io träge wird; nur die Fahrbarkeit sind 395 Merkmale und 574 kB. Den
größten Anteil trägt die Streckenebene mit einem Merkmal je GPS-Segment.
Der Dateiname nennt die Auswahl, damit mehrere Ansichten derselben Sitzung
nebeneinander bestehen.

Ebenen: Strecke nach Straßenqualität oder nach der Abweichung zwischen GPS-
und OBD-Geschwindigkeit eingefärbt, Fahrbarkeit je Streckenabschnitt als
breite eingefärbte Linie (ab Schema 1.5.35), Schlaglöcher als Marker nach
Schwere, Kurven als tatsächlicher Bogen (ab Schema 1.5.28 mit Anfangs- und
Endzeit, sonst als Punkt), Beifahrerurteile und Belagswechsel als Marker,
Referenzintervalle des Beifahrer-Kurventests sowie ein Sitzungskopf mit
Firmware, Kennzahlen und Laufzeitdiagnose.

Verbindliche Regeln der Auswertung:

- **Die Fahrbarkeitsnote wird beim Export aus dem Effektivwert neu
  berechnet**, nicht aus der Logdatei übernommen. Der Effektivwert ist die
  Messung, die Note die Auslegung. Die Firmware schreibt die Note ihres
  eigenen Standes ins Log; nach einer Nachkalibrierung wären Aufzeichnungen
  verschiedener Versionen sonst nicht vergleichbar. Die Fahrt vom 02.08.2026
  trägt Noten der Skala von 1.5.35 und verteilte sich damit auf 181 zu 68 zu
  10 zu 0 Abschnitte je Klasse - die Karte wäre fast einfarbig gewesen. Mit
  der Skala von 1.5.36 sind es 84 zu 88 zu 45 zu 42. Der aufgezeichnete Wert
  bleibt als `noteAufzeichnung` erhalten.
- `tools/export_geojson.py` liest die Notengrenzen aus
  `src/hardware_config.h`, statt sie zu duplizieren. Diese Kopplung nicht
  auflösen.
- Beifahrerurteile werden als Einzelpunkte gezeichnet und nie zu Strecken
  verbunden: Ein Urteil gilt für seinen Abschnitt, nicht bis zum nächsten
  Marker.
- **Die Farbklassen der Fahrbarkeit tragen die Namen der Beifahrerstufen**
  und liegen in der Mitte zwischen deren Notenmedianen. Keine frei gewählten
  Klassennamen: Eine erste Fassung nannte alles unter Note 25 „nicht mehr
  zügig“, obwohl der Beifahrer dieselben Abschnitte 20 von 27 Mal mit
  „mäßig“ bewertet hatte. Die Messung war richtig, die Beschriftung
  übertrieben. Stufe 3 und 4 bleiben zusammengefasst, weil sie über den
  Effektivwert nicht trennbar sind.

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

- Firmware 1.5.43 baut erfolgreich für `lolin_s3_mini`.
- Letzter Build: 69.796 Byte RAM (21,3 %) und 1.166.174 Byte Flash (89,0 %).
  1.5.41 gab 70.708 Byte frei; verfügbar sind damit 153.982 Byte.
- Am 31.07.2026 liefen fünf Beifahrer-Referenzfahrten mit 1.5.28 und je zwölf
  markierten Referenzintervallen. Sie sind der Prüfstand der Kurvenerkennung.
- 1.5.29 ist am Fahrzeug bestätigt: vier Fahrten am 01.08.2026 über 21 Minuten
  und 9,1 km, Zeitabdeckung der Ereignisse 97 % im Median gegenüber 67 % mit
  1.5.28, nur 1 von 66 Ereignissen unter 60 %, keine Ausreißer über die
  Höchstdauer, kein Ereignis im Stand. Der Abgleich der Wiedergabe gegen die
  Firmwareereignisse ordnete 65 von 66 zu, drei der vier Fahrten
  ereignisgenau bei 0,0 bis 0,1 Grad.
- 1.5.34 ist am Fahrzeug bestätigt: drei Fahrten am 02.08.2026, alle
  Sitzungen vollständig, kein SD-Abbruch, keine Integritätsdatei, Wiedergabe
  33 zu 33 bei 0,1 Grad. Der Abschlussgrund `SESSION_END` ist dreimal belegt.
  Die vollständige Auswertung steht in
  `testdata/AUSWERTUNG_2026-08-02_1.5.34.md`.
- Die Hauptfahrt vom 02.08.2026 lief hin und zurück über dieselbe Strecke und
  belegt die Reproduzierbarkeit der Kurvenerkennung: acht gemeinsame Kurven,
  in beiden Richtungen erkannt, 1,0 Grad Winkelabweichung im Median und 3,4
  Grad im Maximum. Diese Messung braucht keine markierten Referenzintervalle
  und ist mit `tools/vergleich_hin_rueck.py` jederzeit zu wiederholen.
- In den **ersten drei** Fahrten vom 02.08.2026 hatte GPS keinen Fix: 1.887
  Zeilen mit exakt null Satelliten, am Vortag dagegen 12 bei HDOP 0,68. Ab
  11:38 Uhr desselben Tages kam der Empfang ohne Eingriff von selbst zurück
  und ist seitdem stabil: sechs Sitzungen bis zum 03.08.2026 mit 12
  Satelliten, HDOP-Median 0,74 bis 1,07 und 99,7 bis 100 Prozent Fixquote.
  Der Fehler ist damit nicht erklärt, sondern nur nicht wieder aufgetreten. Die Firmware ist als
  Ursache ausgeschlossen — sie sendet dem BN-880 nichts, `gps_manager.cpp` ist
  seit dem 30.07. unverändert, die NMEA-Kette war mit 2.440 gültigen Sätzen
  und null Prüfsummenfehlern intakt, und die Fix-LED des Moduls blieb dunkel.
  Null Satelliten über fünf Minuten schließen auch einen Kaltstart aus. Der
  Verdacht liegt auf einem Wackelkontakt an der aufgelöteten Keramikantenne
  oder der Versorgung. Vor jeder Fahrt im Stand abwarten, bis `diag`
  Satelliten meldet, und die Satellitenzahl im Sitzungskopf gegenprüfen.
- Zwei SD-Abbrüche desselben Tages wurden verlustfrei aufgefangen:
  `20260731_142359_514EDDF4` mit 38 von 38 geretteten Pufferzeilen nach
  81 Sekunden Ausfall und `20260731_160032_3D732AD1` mit 17 von 17 nach
  11 Sekunden. Die Fortsetzung schloss zeitlich lückenlos und ohne Dublette
  an. Die abgebrochene Sitzung bleibt jedoch ohne END-Record und ohne
  Zusammenfassung; deren Kennzahlen sind nachträglich zu rekonstruieren.
- BNO055-Selbsttest, SD-Logging, WLAN und MCP2515-Grundkommunikation
  wurden am System geprüft.
- Am Porsche Carrera S, Baujahr 2012, PDK wurden standardisierte Antworten von
  CAN-ID `0x7E8` für Drehzahl, Geschwindigkeit und Drosselstellung empfangen.
- Der ausführliche Fahrzeugtest steht in
  `testdata/archiv/20260728_2300/TESTBERICHT_2026-07-28.md`.
- Die aktuelle GPS-/OBD-Abnahme steht in
  `testdata/archiv/20260729_170946_05A4DB46/ABNAHME_AUSWERTUNG.md`.
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

**Die Erfassungsseite gilt seit dem 04.08.2026 als ausreichend.** Kurvenwinkel
sind über beide Fahrtrichtungen reproduzierbar, die Fahrbahnbewertung ist gegen
149 Beifahrerurteile kalibriert, GPS, CAN/OBD und SD liefen über 39 Sitzungen
und 318 km. Der Fokus liegt damit nicht mehr auf der Messung, sondern auf der
Auswertung nach `STRECKENDATENBANK.md`.

Weitere Arbeit an der Messgenauigkeit ist zu begründen mit dem, was sie an der
Streckendatenbank ändert - nicht mit der Zahl selbst. Wo eine Kennzahl nur
schöner wird, ohne eine Segmentbewertung zu verändern, bleibt sie wie sie ist.

Der Umsetzungs-, Test- und Abnahmeplan der Messkette steht weiterhin in
`GPS_CAN_OBD_DATENQUALITAETSPLAN.md`. Neue Messfunktionen müssen dessen
Sitzungszähler, Gültigkeitsfelder, Zeitbasis und PASS/WARN/FAIL-Kriterien
berücksichtigen.

BNO055, SD und WLAN laufen bei Fahrten als Stabilitätskontrolle mit.
Beide Bewertungsmodelle werden nicht gleichzeitig neu parametriert.

## Fahrbarkeit der Strecke

Seit 1.5.35 beantwortet die Fahrbahnbewertung eine andere Frage als zuvor.
Nicht "wo liegt ein Schlagloch", sondern **"kann man auf diesem Stück zügig
fahren"**. Schlechte Beläge - Pflaster auf Nebenstrecken, Kies, lange
vernachlässigter Asphalt - sind dabei nur insoweit interessant, als sie
sportliches Fahren verhindern. Die Ortsgenauigkeit einer einzelnen
Schadstelle ist ausdrücklich nachrangig.

Verbindliche Punkte:

- Bewertet wird die auf die Schwerkraftrichtung **projizierte**
  Vertikalbeschleunigung, niemals eine rohe Sensorachse. Bis 1.5.34 lief die
  Bewertung über `accelZ`; im Fahrzeug trug diese Achse 19 Prozent der
  Vertikalen und korrelierte mit ihr zu -0,06. Die Bewertung maß dadurch
  Quer- und Längsbeschleunigung, also die Fahrweise, und benotete gerade
  zügig gefahrene Kurven am schlechtesten.
- Ein Abschnitt umfasst einen festen **Weg** von `ROAD_SECTION_LENGTH_M`,
  keine feste Zeit. Nur so beschreibt er unabhängig vom Tempo dasselbe Stück
  Straße.
- Die Rauheit ist eine Eigenschaft der **befahrenen Spur**, nicht der
  Straßengeometrie. Hin- und Rückfahrt derselben Strecke sind deshalb kein
  Prüfstand für die Fahrbahnbewertung, obwohl sie einer für die
  Kurvenerkennung sind: Über kurze Abschnitte wichen sie am 02.08.2026 um
  27 Prozent voneinander ab, und der größte Ausreißer war genau der
  Abschnitt, in dem eine Richtung über zwei Schlaglöcher fuhr und die andere
  nicht.
- **Belagsarten sind bei 10 Hz Abtastung nicht unterscheidbar.**
  Kopfsteinpflaster regt bei 30 km/h mit rund 83 Hz an, Großpflaster mit
  42 Hz, Kies mit über 160 Hz; die Nyquist-Grenze liegt bei 5 Hz. Keine
  Frequenzschätzung einbauen, die das verschweigt - `VibrationMetrics`
  führt `frequency` bewusst als 0. Messbar bleibt die Aufbaubewegung bei 1
  bis 2 Hz, und die entscheidet über die Fahrbarkeit. Der Effektivwert
  bleibt auch bei Unterabtastung ein gültiges Energiemaß, weil Aliasing die
  Frequenz verfälscht und nicht die Gesamtenergie.
- Die Notengrenzen `ROAD_DRIVEABILITY_RMS_GOOD_MPS2` und
  `ROAD_DRIVEABILITY_RMS_BAD_MPS2` sind gegen 149 Beifahrerurteile aus zwei
  Fahrtagen bestimmt, darunter eine Kopfsteinpflasterfahrt. Belegt, aber
  aus einem Fahrzeug mit einem Beifahrer und nicht allgemeingültig. Änderungen nur
  gegen neue Urteilsdaten, nie nach Augenschein.
- Eine Geschwindigkeitsnormierung ist geprüft und verworfen: Der rohe
  Effektivwert erreicht 0,73 Rangkorrelation zur Wertungsstufe, die beste
  normierte Form 0,74.
- Die Skala ist über alle vier Stufen monoton: 0,701 / 0,843 / 1,420 / 1,583
  m/s² im Median über 149 Urteile zweier Fahrtage. Die Trennschwelle liegt
  bei 1,08 m/s² und trifft 85 Prozent. Stufe 4 ist mit sechs Abschnitten
  weiterhin dünn belegt.
- **Ab `ROAD_DRIVEABILITY_FAST_KMH` gilt die Fahrbahn als tragfähig** und die
  Note fällt nicht unter `ROAD_DRIVEABILITY_FAST_MIN_NOTE`. Wer schnell
  fährt, traut der Straße; die Anregung steigt mit der Geschwindigkeit, und
  der Effektivwert allein stufte solche Abschnitte zu schlecht ein.

  Diese Regel wirkt **ausschließlich nach oben** und darf nicht zu einer
  symmetrischen Umrechnung ausgebaut werden: Hohe Geschwindigkeit belegt eine
  tragfähige Fahrbahn, niedrige belegt nichts, weil sie ebenso am Verkehr
  liegen kann. Von 14 bewerteten Abschnitten über 100 km/h wurde keiner
  schlechter als „gut“ beurteilt, gegenüber 36 Prozent im Mittel.
- Die Fahrdynamik aus der Horizontalbeschleunigung ist geprüft und verworfen:
  nur 0,04 Rangkorrelation zur Wertungsstufe, als Korrekturfaktor
  verschlechtert sie das Ergebnis von 0,74 auf 0,48. Bremsen, Beschleunigen
  und Kurven finden unabhängig von der Fahrbahnqualität statt.

### Beifahrerurteile

Die Beifahrerseite `/fahrbahn` ist mit 1.5.41 entfernt. Urteile entstehen nach
`STRECKENDATENBANK.md` künftig nach der Fahrt am Rechner: Was während der Fahrt
Aufmerksamkeit kostet, kommt nicht in die Datenbank, und ein Beifahrer ist bei
einer alltäglichen Aufzeichnung nicht vorauszusetzen.

Die damit erhobenen 149 Urteile zweier Fahrtage bleiben die Grundlage der
Notengrenzen `ROAD_DRIVEABILITY_RMS_GOOD_MPS2` und `_BAD_MPS2`. Ihre
Auswertungsregeln gelten unverändert weiter, auch für die kommende Bewertung
am Rechner:

- **Ein Urteil gilt für seinen Abschnitt, nicht bis zum nächsten Marker.**
  Jedes Urteil trägt dafür die laufende Abschnittsnummer und den Weg im
  offenen Abschnitt mit. Andernfalls zöge eine Pause von zehn Minuten die
  letzte Wertung über zehn Minuten unbewertete Straße. Nicht bewertete
  Abschnitte bleiben ohne Urteil - das ist der gewollte Zustand, keine Lücke,
  die zu füllen wäre.
- Bewertet wird der **abgeschlossene** Abschnitt. Er liegt hinter dem Fahrzeug
  und kann das Urteil über die Strecke unter den Rädern nicht vorwegnehmen.
- Die Fahrbarkeit ist ein Urteil, kein Messwert. Ohne Urteilsdaten bleiben die
  beiden Notengrenzen Konvention; sie sind nie nach Augenschein zu ändern.

In Bestandsdaten heißen die Marker `FAHRBAHN_URTEIL` mit der Stufe 1 bis 4 in
`Schwere` sowie `FAHRBAHN_BELAGSWECHSEL`. Neue Aufzeichnungen enthalten sie
nicht mehr.

## Bekannte Einschränkungen

### CAN und OBD

- **Die Temperatur-PIDs `0x46` und `0x5C` rechnen `A - 40` über ein Byte;
  `0xFF` bedeutet dort „Wert nicht verfügbar" und nicht 215 °C.** Bis 1.5.36
  verbuchte die Firmware diesen Sentinel als gültige Messung: 28 von rund
  530 Antworten auf `0x46` in den Sitzungen vom 29./30.07.2026, allein
  45 von 2438 gültigen Zeilen in `20260729_170946_05A4DB46`. Die Antwort
  zählt weiterhin als Antwort, damit ECU-Erreichbarkeit und
  Timeoutbuchführung stimmen; nur das Gültigkeitsflag bleibt aus. Bei
  `0x0D` und `0x11` ist `0xFF` dagegen ein regulärer Wert und darf nicht
  verworfen werden. Bestandsdaten sind nicht korrigiert: Beim Auswerten der
  Außentemperatur älterer Sitzungen 215,0 herausfiltern.
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

Am 02.08.2026 liefen drei Fahrten mit 1.5.34: eine Strecke hin und zurück
sowie zwei Kreise auf dem Parkplatz. `SESSION_END` ist damit belegt, ebenso
die Reproduzierbarkeit der Kurvenerkennung über beide Fahrtrichtungen. Von den
Zielen aus 1.5.30 ist nur der gleichmäßige Fünf-Sekunden-Abstand der
Statuszeilen erreicht; `LoopMaxMs` blieb bei 251 ms.

Offen bleibt der Flush-Schritt. Über die Sitzungen vom 02. und 03.08.2026
liegt `SensorMissedSlots` bei 13 bis 16 je Minute, also 2,2 bis 2,7 Prozent
der Stichproben; die früher genannten 12,2 je Minute waren ein Einzelwert und
keine dauerhafte Verbesserung. `FlushMaxMs` erreicht 95 bis 119 ms und
überschreitet damit die Slotbreite von 100 ms.

Die Zerlegung aus `20260803_082231_D0606CF2` benennt die Ursache genau: Der
mittlere Flush-Schritt kostet 19,1 ms über 3.180 Schritte, aber ein einzelner
SD-Vorgang erreicht 79 ms und ein Flush-Schritt 111 ms. Bei 319 Umläufen
stehen 396 verfehlte Slots - also gut ein Slot je Umlauf. Genau ein Schritt
ist teuer, und das war Schritt 1: Er bündelte als einziger den Puffer-Write
und den Dateiflush der Sensordatei. Die restlichen rund 0,24 Slots je Umlauf
gehen auf die Schleifenausreißer mit `LoopMaxMs` bis 276 ms; der Flush
erklärt die Fehlrate also weitgehend, aber nicht vollständig.

**1.5.38 trennt beide Operationen.** Der Puffer-Write bleibt Schritt 1, der
Dateiflush wird Schritt 2; `SD_FLUSH_STEP_COUNT` steigt auf 11 und das
Schrittintervall sinkt auf 450 ms, damit der volle Umlauf bei 4,95 s bleibt.
Erwartet wird eine größte Blockade um 79 statt 111 ms. **Am Gerät ist das
noch nicht bestätigt** - die nächste Fahrt muss zeigen, ob `FlushMaxMs` unter
die Slotbreite fällt und `SensorMissedSlots` entsprechend sinkt.

Dafür führt die Metadatendatei seit 1.5.38 die Spalte `FlushMaxStep`: den
Index des Schritts, der den Höchstwert erzeugt hat. Ohne ihn war `FlushMaxMs`
nicht zuzuordnen, und die Ursache musste über die Zeitverteilung
erschlossen werden. Bleibt der Höchstwert über 100 ms, benennt das Feld
unmittelbar den nächsten Kandidaten.

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
  Detektor wird beim Messstart zurückgestellt. Am 01.08.2026 bestätigt,
  `SESSION_END` selbst am 02.08.2026 dreimal.
- `DetectionMode` ist kein Merkmal der Kurve, sondern des Erkennungspfads.
  In den Hin- und Rückfahrten vom 02.08.2026 stand für 4 von 8 gemeinsamen
  Kurven einmal `SHARP` und einmal `LONG`, bei auf ein Grad gleichem Winkel;
  entschieden hat die Anfahrgeschwindigkeit. Das Feld nicht als
  Kurveneigenschaft auswerten und nicht gegen einen Festwert prüfen.
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
  Messsitzung behandelt. Seit 1.5.30 bekommt die abgebrochene Sitzung
  zusätzlich ihre Zusammenfassung nachgetragen; das Fortsetzungsereignis
  führt dazu das Feld `ZusammenfassungNachgetragen`. Nach Reset oder
  Spannungsverlust bleibt ein manueller Messstart verbindlich, und es
  entsteht bewusst keine Zusammenfassung: Der Sitzungsmarker überlebt im NVS,
  die Kennzahlen nicht.
- Ein erfolgreich gemeldeter Schreibvorgang beweist nicht, dass die Zeile die
  Karte erreicht hat. In `20260801_114252_B9D1628B` fehlten 68 Prozent der
  GPS-Spur bei null Fehlerzählern und schlüssiger firmwareseitiger
  Buchführung. Seit 1.5.31 prüft die Firmware am Sitzungsende jede Logdatei
  gegen die tatsächliche Größe auf der Karte und legt bei Abweichung
  `road_integritaet_<Sitzung>.csv` an. Die Prüfung erkennt den Verlust, sie
  verhindert ihn nicht.

  **Die Ursachensuche ist am 04.08.2026 abgeschlossen, ohne den Fall zu
  erklären.** Der Verlust ist seit dem 01.08.2026 kein zweites Mal
  aufgetreten: zwölf Sitzungen liefen seither mit aktiver Integritätsprüfung,
  keine einzige erzeugte eine `road_integritaet`-Datei, und in allen deckten
  sich die Zeilenzahlen mit den Zählern der Metadatendatei. Eine neue Karte
  ist seit dem 04.08.2026 im Einsatz. Der damit mögliche Gegentest wurde
  bewusst nicht als Beweis geführt: Ein Fehler, der in rund 30 Sitzungen
  einmal auftrat, ist mit einer Fahrt weder zu bestätigen noch
  auszuschließen. Der Fall bleibt ein unerklärter Einzelfall.

  Verbindlich bleibt deshalb nicht ein Ergebnis, sondern ein Verfahren: Die
  Integritätsprüfung aus 1.5.31 ist der stehende Melder. Taucht wieder eine
  `road_integritaet`-Datei auf oder weicht eine Zeilenzahl von ihrem Zähler
  ab, beginnt die Suche neu - dann aber mit einem zweiten Fall statt mit
  einem. Ohne einen solchen Anlass keine weitere Ursachensuche in der
  Firmware: Ihre Buchführung war nachweislich in sich schlüssig.
- Bei Auswertungen jeder Sitzung zuerst die Zeilenzahl jeder Datei gegen die
  Zähler der Metadatendatei stellen. Vollständigkeit nicht aus dem Vorhandensein
  eines END-Records und einer Zusammenfassung schließen.
- Der SD-Flush läuft seit 1.5.30 in Einzelschritten. `flushStep()` gehört in
  die Hauptschleife und sichert je Aufruf höchstens eine Datei; `flush()`
  sichert alles in einem Zug und bleibt Sitzungsenden vorbehalten. Die
  Felder `FlushLastMs`, `FlushMaxMs` und `FlushCycles` beziehen sich dadurch
  auf den einzelnen Schritt statt auf den vollen Durchlauf.
- Absolute Kurswerte benötigen eine ausreichende Magnetometer- und
  Systemkalibrierung; relative Beschleunigungen bleiben davon weitgehend
  unabhängig.

## Entwicklungsregeln

- Bestehende, fest verdrahtete Pinbelegung beibehalten.
- Hardwarezugriffe nicht blockierend halten; WLAN, Webseite und OTA müssen
  erreichbar bleiben.
- CAN darf die allgemeine Bereitschaft nicht blockieren.
- Den OLED-Treiber nicht wieder einführen. Das Display ist seit dem
  03.08.2026 ausgebaut; Statusseite und serielle Ausgabe zeigen dieselben
  Informationen und genügen dafür. Mit ihm ist der zweite unabhängige
  I²C-Teilnehmer entfallen: Fällt der BNO055 aus, lässt sich am Bus nicht
  mehr feststellen, ob Sensor oder Bus die Ursache ist. Wer das für eine
  Fehlersuche braucht, hängt vorübergehend ein beliebiges I²C-Gerät an -
  der Scanner findet es ohne Codeänderung.
- SD-Aufzeichnung über die vorhandene nicht blockierende Startlogik öffnen.
- **Logdateien nicht unkomprimiert über das Netz ausgeben.** Genau daran
  scheiterte die früher entfernte Web-Downloadfunktion. Seit 1.5.43 gibt
  `/files` jede Sitzung als ein `.tar.gz` aus, komprimiert über
  `src/gzip_stream.{h,cpp}` und den ROM-Kompressor des ESP32-S3. Ein
  eingebautes Gerät ließe sich sonst nicht mehr auslesen, ohne die
  Verkleidung zu öffnen.
- Download und Löschen bleiben gesperrt, solange eine Messung läuft - wie das
  Firmware-Update und aus demselben Grund: Beides hält die Hauptschleife für
  seine ganze Dauer an.
- GPS- oder OBD-Werte nur als gültig ausgeben, wenn ihre Aktualität
  nachweisbar ist; unbekannte Werte nicht als Nullwert ausgeben.
- Firmwareversion zentral in `src/hardware_config.h` sowie `CHANGELOG.md` und
  `README.md` gemeinsam aktualisieren; Web und Bootmeldung verwenden
  dieses zentrale Makro. Der Firmwaredateiname wird beim Build automatisch
  daraus erzeugt und darf nicht manuell abweichend benannt werden.
- `AGENTS.md` und `CLAUDE.md` sind wortgleich und immer gemeinsam zu ändern.
  Sie richten sich an verschiedene Werkzeuge, beschreiben aber dasselbe
  Projekt; eine der beiden allein zu pflegen führt dazu, dass ein Werkzeug
  nach veralteten Regeln arbeitet. Bis zum 02.08.2026 stand `AGENTS.md` sechs
  Versionen zurück und nannte noch den ausgebauten OLED-Treiber sowie die
  entfernte Beifahrerseite. Nach einer Änderung prüfen:
  `diff AGENTS.md CLAUDE.md` muss leer bleiben.
- Rohmessungen, Fotos, `.sal`-Dateien, `.codex/`, `.claude/` und temporäre
  Dateien nur auf ausdrückliche Anforderung committen.
- Bestehende Nutzeränderungen im Arbeitsbaum nicht verwerfen oder
  überschreiben.

# ROADTEST Firmware

ESP32-S3-Firmware zur Aufzeichnung von BNO055-, GPS-, Straßenqualitäts- und
standardisierten OBD-II-Daten. Aktueller Firmwarestand: **1.5.25**.

Der aktuelle Stand ist ein Hardware- und Fahrzeugteststand, nicht abschließend
produktionsreif. Bekannte Einschränkungen stehen weiter unten.

## Befehle

| Befehl | Zweck |
|---|---|
| `pio run` | Firmware für `lolin_s3_mini` bauen |
| `pio run -t upload` | Firmware über USB-CDC flashen |
| `pio device monitor` | Seriellen Monitor mit den Einstellungen aus `platformio.ini` öffnen |
| `pio run -t clean` | PlatformIO-Buildartefakte löschen |

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
src/road_quality.h              Fahrbahn- und Kurvenmetriken
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

Vor einem Commit mindestens ausführen:

```bash
pio run
git diff --check
```

## Aktueller Teststand

- Firmware 1.5.25 baut erfolgreich für `lolin_s3_mini`.
- Letzter Build: 70.740 Byte RAM (21,6 %) und 1.236.006 Byte Flash (94,3 %).
- BNO055-Selbsttest, SD-Logging, WLAN, optionales OLED und MCP2515-
  Grundkommunikation wurden am System geprüft.
- Am Porsche Carrera S, Baujahr 2012, PDK wurden standardisierte Antworten von
  CAN-ID `0x7E8` für Drehzahl, Geschwindigkeit und Drosselstellung empfangen.
- Der ausführliche Fahrzeugtest steht in
  `testdata/20260728_2300/TESTBERICHT_2026-07-28.md`.
- Die aktuelle GPS-/OBD-Abnahme steht in
  `testdata/20260729_170946_05A4DB46/ABNAHME_AUSWERTUNG.md`.
- Der Recovery-Kontrolllauf `20260730_062734_543E6ED0` bestätigte stabile
  Web-, SD- und GPS-Aufzeichnung, zeigte aber einen verlorenen Browsermarker
  beim zweiten Motorstart sowie weiterhin verfehlte Abtastslots. Diese Punkte
  sind ab 1.5.24 korrigiert und noch am Gerät zu bestätigen.

## Verbindlicher Datenqualitätsfokus

GPS und CAN/OBD sind die beiden primären Datenquellen der nächsten
Entwicklungsphase. Jeder Fahrzeugtest muss beide Quellen gleichzeitig
aufzeichnen, ihre Eigenqualität bewerten und Geschwindigkeit zeitlich
miteinander vergleichen.

Der verbindliche Umsetzungs-, Test- und Abnahmeplan steht in
`GPS_CAN_OBD_DATENQUALITAETSPLAN.md`. Neue Messfunktionen müssen dessen
Sitzungszähler, Gültigkeitsfelder, Zeitbasis und PASS/WARN/FAIL-Kriterien
berücksichtigen.

BNO055, SD, OLED und WLAN laufen bei diesen Tests als Stabilitätskontrolle mit.
Weitere Fahrzeug-PIDs und nicht blockierende Nebenoptimierungen werden erst
priorisiert, wenn die GPS-/Standard-OBD-Basis reproduzierbar belastbar ist.

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

Nächster sinnvoller Schritt: kurzer ECU-Recovery-Kontrolltest im Stand über
`/acceptance` mit 1.5.25 und anschließende Prüfung der Abtast- und
`MissedSlots`-Zähler; eine weitere kurze Vergleichsfahrt ist dafür nicht nötig.

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
  Zeitplanphase stabil und zählt verfehlte Sensor- und GPS-Slots; die
  Wirksamkeit muss noch am Gerät bestätigt werden.
- Schlaglochereignisse sind noch nicht an eine Mindestgeschwindigkeit
  gekoppelt und können daher im Stand ausgelöst werden.
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

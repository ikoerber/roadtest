# ROADTEST Firmware

ESP32-S3-Firmware zur Aufzeichnung von BNO055-, GPS-, Straßenqualitäts- und
standardisierten OBD-II-Daten. Aktueller Firmwarestand: **1.5.14**.

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
.pio/build/lolin_s3_mini/firmware.bin
```

Der Build verwendet `scripts/patch_bno055_reset_timeout.py` als
PlatformIO-Pre-Script. Diesen Schritt nicht umgehen.

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

- Firmware 1.5.14 baut erfolgreich für `lolin_s3_mini`.
- Letzter Build: 57.876 Byte RAM (17,7 %) und 1.191.918 Byte Flash (90,9 %).
- BNO055-Selbsttest, SD-Logging, WLAN, optionales OLED und MCP2515-
  Grundkommunikation wurden am System geprüft.
- Am Porsche Carrera S, Baujahr 2012, PDK wurden standardisierte Antworten von
  CAN-ID `0x7E8` für Drehzahl, Geschwindigkeit und Drosselstellung empfangen.
- Der ausführliche Fahrzeugtest steht in
  `testdata/20260728_2300/TESTBERICHT_2026-07-28.md`.

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

- Der PID-Unterstützungsscan läuft derzeit nur einmal.
- Startet der Motor erst nach dem Scan, wird die ECU-Erreichbarkeit nicht
  erneut geprüft.
- Ohne bestätigten Unterstützungsblock fragt die Live-Phase keine PIDs ab.
- Anfrage-, Antwort- und Sendefehlerzähler sind Boot-Gesamtzähler und werden
  beim Discovery-Start nicht vollständig zurückgesetzt.
- Der Scan protokolliert noch nicht den Erfolg jedes einzelnen
  Sendeversuchs.

Nächster sinnvoller Schritt: Scan nach Motorstart beziehungsweise periodisch
wiederholen, bekannte PIDs `0x0C`, `0x0D` und `0x11` als sicheren Rückfall
testen und sitzungsbezogene OBD-Diagnosewerte speichern.

### GPS

- `hasValidFix()` prüft derzeit nur eine gültige Position mit weniger als fünf
  Sekunden Alter.
- TinyGPS++-Felder werden unabhängig übernommen; dadurch können Position,
  Satellitenzahl, HDOP, Geschwindigkeit und Höhe unterschiedlich alt sein.
- Der UART-Ringpuffer besitzt noch keinen sichtbaren Überlaufzähler.
- Die Aufzeichnung wiederholt GPS-Positionen häufiger als neue NMEA-Fixes
  eintreffen.

Nächster sinnvoller Schritt: feldweise Altersprüfung, Satelliten-/HDOP-
Grenzen, Plausibilitätsfilter, Überlaufzähler und Logging nur bei neuen Fixes.

### Messqualität

- Effektive BNO055-Abtastrate lag im Fahrzeugtest bei ungefähr 8,5 statt
  vorgesehenen 10 Hz.
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
- Firmwareversion in `src/web_manager.cpp`, `CHANGELOG.md` und `README.md`
  gemeinsam aktualisieren.
- Rohmessungen, Fotos, `.sal`-Dateien, `.claude/` und temporäre Dateien nur auf
  ausdrückliche Anforderung committen.
- Bestehende Nutzeränderungen im Arbeitsbaum nicht verwerfen oder
  überschreiben.

# Changelog

Alle wichtigen Änderungen am ESP32-S3 Straßenqualitäts-Messsystem werden in dieser Datei dokumentiert.

Das Format basiert auf [Keep a Changelog](https://keepachangelog.com/de/1.0.0/),
und dieses Projekt hält sich an [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unveröffentlicht]

### Dokumentation
- Hardwarebeschreibung auf den tatsächlichen LOLIN S3 Mini mit 4 MB Flash
  umgestellt
- Verbindliche GPIO-Belegung mit dem Firmwarecode abgeglichen
- Versorgung des PZSMOCN-SD-Moduls auf 3,3 V korrigiert
- Unbenutzte BNO055- und GPS-Pins sowie der deaktivierte CAN-Aufbau eindeutig
  dokumentiert
- Unsichere pauschale Angaben zu CAN-Versorgung und Busterminierung entfernt

## [1.5.10] - 2026-07-27

### Behoben
- Der sichtbare BNO055-Status bleibt bei einem oder zwei einzelnen,
  kurzzeitigen I²C-Statusfehlern auf dem zuletzt bestätigten Zustand
- Erst drei aufeinanderfolgende vollständige Fusionsfehler setzen den Status
  auf Fehler und lösen den bestehenden automatischen Sensorneustart aus
- Ein erfolgreicher Status setzt Fehlerzähler und Fusionsstatus sofort zurück

## [1.5.9] - 2026-07-27

### Behoben
- Bereitschaft, Datenverarbeitung, Webdiagnose und automatischer Wiederanlauf
  verwenden jetzt dieselbe vollständige Fusionsprüfung: IMUPLUS-Modus 8,
  Systemstatus 5 und Fehlercode 0
- Ungültige Fusionsdaten werden sofort gesperrt; ein Sensorneustart erfolgt
  erst nach drei aufeinanderfolgenden Fehlerprüfungen
- Integrationstests vergleichen beim IMUPLUS-Modus relative Drehänderungen
  statt des relativen Sensorwinkels mit dem absoluten GPS-Kurs
- Wiederanlauf- und Driftprüfung verwenden nur Gyro und Beschleunigung
- README, Hardwarebeschreibung, OLED-Texte und CSV-Feldnamen wurden auf den
  IMUPLUS-Betrieb abgestimmt

## [1.5.8] - 2026-07-27

### Geändert
- BNO055 läuft im IMUPLUS-Modus nur mit Gyro und Beschleunigung
- Magnetometer und absolute Kompassrichtung werden nicht mehr verwendet
- Für eine vollständige Kalibrierung müssen nur Gyro und Beschleunigung den
  Wert 3 erreichen; eine Achterbewegung entfällt
- BNO055-Selbsttest und Webdiagnose berücksichtigen den IMUPLUS-Betrieb

## [1.5.7] - 2026-07-27

### Geändert
- BNO055 verwendet die interne Taktquelle, um den sporadischen
  Fusionsfehler 9 des vorhandenen Aufbaus zu vermeiden
- I²C-Bus läuft mit 50 kHz statt 100 kHz, um die bekannten
  BNO055-Timingprobleme und die Lochrasterverdrahtung robuster zu behandeln
- Die Kalibrierseite zeigt die verwendete Taktquelle

## [1.5.6] - 2026-07-27

### Behoben
- Der BNO055-Modus wird nicht mehr in jedem Schleifendurchlauf über I2C
  abgefragt, wenn eine andere Komponente noch nicht bereit ist
- Einzelne fehlerhafte Moduslesungen lösen keinen unnötigen Wechsel auf NDOF
  mehr aus; erst drei aufeinanderfolgende Fehler führen zu einem Neustart
- Der Web-Schalter startet den BNO055 jetzt vollständig neu, statt lediglich
  kurz zwischen CONFIG und NDOF umzuschalten

## [1.5.5] - 2026-07-27

### Behoben
- Die automatische CAN-Initialisierung ist wieder deaktiviert, weil der
  blockierende MCP2515-Start ohne betriebsbereiten CAN-Aufbau die Webseite
  anhalten kann
- Die BNO055-NDOF-Diagnose aus 1.5.4 bleibt vollständig erhalten

## [1.5.4] - 2026-07-27

### Behoben
- Der BNO055-Betriebsmodus wird nach Initialisierung und Laden gespeicherter
  Kalibrierwerte geprüft und bei Bedarf ausdrücklich auf NDOF gesetzt
- Sensorwerte werden nur verarbeitet, wenn die NDOF-Sensorfusion aktiv ist

### Hinzugefügt
- Kalibrierseite zeigt BNO055-Betriebsmodus, Fusionsstatus, Systemstatus und
  Fehlercode
- Geschützter BNO055-Neustart direkt auf der Kalibrierseite

### Geändert
- Das optionale CAN-Modul wurde einmalig nach dem Systemstart geprüft

## [1.5.3] - 2026-07-27

### Geändert
- CAN ist für den aktuell verwendeten Aufbau standardmäßig vollständig
  deaktiviert und kann die Hauptschleife nicht mehr beeinflussen
- ROADTEST verwendet fest `192.168.4.1`, WLAN-Kanal 6 und keinen Schlafmodus

### Hinzugefügt
- WLAN-Watchdog, der einen ausgefallenen Access Point automatisch neu startet

## [1.5.2] - 2026-07-27

### Geändert
- Das ROADTEST-WLAN startet vor allen externen Hardwareprüfungen
- Die optionale CAN-Prüfung erfolgt erst nach dem Systemstart im Hintergrund

### Behoben
- Eine langsame Hardwareinitialisierung konnte den WLAN- und OTA-Zugang
  verzögern

## [1.5.1] - 2026-07-27

### Behoben
- Die GPS-Startprüfung wartete zu streng auf einen vollständig gültigen
  NMEA-Satz und konnte dadurch dauerhaft `SUCHE` anzeigen
- Für die Hardwareprüfung genügt nun ein aktueller NMEA-Datenstrom; Fix,
  gültige Sätze und Prüfsummen bleiben getrennt sichtbar

### Hinzugefügt
- GPS-Diagnose auf der Webseite mit empfangenen Zeichen, gültigen Sätzen und
  Prüfsummenfehlern

## [1.5.0] - 2026-07-27

### Hinzugefügt
- Einheitliche OLED-Prüfseite für OLED, BNO055, SD, GPS, WLAN und optionales CAN
- Automatische Wiederverbindung für BNO055, OLED, SD-Karte, GPS-UART und WLAN
- Anzeige von SD-Schreibfehlern und verworfenen Datensätzen auf der Webseite
- Kennzeichnung einer durch einen SD-Fehler abgebrochenen Messfahrt

### Geändert
- Die Startprüfung blockiert die Firmware nicht mehr; Webseite und
  Wiederanlauf bleiben während eines Fehlers aktiv
- Die OLED-Prüfseite wird aktualisiert, bis alle erforderlichen Komponenten
  reagieren, und erscheint bei einem späteren Ausfall erneut
- Ein GPS-Fix ist nicht zum Start nötig; gültige NMEA-Kommunikation genügt
- CAN bleibt optional und verhindert weder Systembereitschaft noch Messfahrt

### Behoben
- Ein fehlender BNO055 führte zuvor zu einer permanenten Warteschleife
- Mehrere aufeinanderfolgende Bootanzeigen überspielten sich gegenseitig
- Fehlgeschlagene SD-Schreibvorgänge wurden nicht zuverlässig gezählt

## [1.4.0] - 2026-07-27

### Hinzugefügt
- Geschützter Start und Stopp einer Messfahrt auf der Statusseite
- Live-Zusammenfassung mit Dauer, GPS-Strecke, Kurven, Schlaglöchern und
  Durchschnittsqualität
- Dauerhafte `road_summary_<Sitzung>.csv` nach dem Beenden
- Serielle Kommandos `start` und `stop` als Alternative zur Webseite

### Geändert
- Nach dem Einschalten ist das System messbereit, beginnt aber erst nach
  ausdrücklichem Start mit der SD-Aufzeichnung
- Alle Dateien einer Fahrt verwenden dieselbe eindeutige Sitzungskennung
- Beim Beenden werden sämtliche Puffer geleert und alle Dateien geschlossen

## [1.3.0] - 2026-07-27

### Hinzugefügt
- ROADTEST-WLAN mit kompakter Statusseite und Browser-OTA
- Web- und OLED-Assistent für BNO055-Kalibrierung
- Geschütztes Speichern einer vollständig erreichten Kalibrierung
- Warnung beim Start mit unvollständiger Kalibrierung

### Geändert
- BNO055 wird im Messbetrieb nur einmal pro 100-ms-Zyklus gelesen
- Vibration, Maximalstoß und Stoßanzahl verwenden ein rollendes
  Ein-Sekunden-Fenster
- Straßenqualität wird bei gültigem GPS vorsichtig auf 30 km/h normiert
- Kurven und Schlaglöcher werden nur einmal pro abgeschlossenem Ereignis
  protokolliert

### Behoben
- Gespeicherte BNO055-Kalibrierung wurde beim Start wegen einer falschen
  Initialisierungsreihenfolge nicht geladen
- Maximalstoß und Stoßanzahl wuchsen zuvor über die gesamte Fahrt
- Kurven konnten während desselben Ereignisses mehrfach protokolliert werden

## [1.2.0] - 2024-12-XX

### Hinzugefügt
- **Umfassende Integration-Test-Suite** mit 92% Coverage
  - 24+ Test-Szenarien für alle Module
  - Hardware Failure & Recovery Tests
  - Performance und Latenz-Messungen
  - Memory-Leak Detection
  - Edge-Case und Stress-Tests
- **GPS Interrupt-Modus** für verlustfreien UART-Datenempfang
  - Ring-Buffer mit 512 Bytes für NMEA-Daten
  - Automatische Aktivierung beim Start
  - Umschaltbar zwischen Interrupt und Polling-Modus
  - Hardware-Test vergleicht beide Modi
- **NVS-Speicher für BNO055-Kalibrierung**
  - Kalibrierungsdaten überstehen Neustarts
  - Automatisches Laden beim Start
  - `clearCalibration()` Methode zum Zurücksetzen
  - Zeitstempel für Kalibrierungsalter
- **Erweiterte Hardware-Tests**
  - GPS Interrupt vs. Polling Vergleichstest
  - Verbesserte Diagnose-Ausgaben

### Geändert
- **String-Operationen optimiert**
  - Statische Buffer statt String-Konkatenation
  - Bessere Performance in kritischen Pfaden
  - Reduzierte Heap-Fragmentierung
- **Sicherheitsverbesserungen**
  - Null-Checks für alle dynamischen Allokationen
  - `strcpy`/`strcat` durch `strncpy`/`strncat` ersetzt
  - Erweiterte Buffer-Overflow-Checks
- **Code-Organisation**
  - Hardware-Pin-Definitionen in `hardware_config.cpp` zentralisiert
  - Keine doppelten Pin-Definitionen mehr

### Behoben
- Potentielle Null-Pointer-Dereferenzierung in BNO055Manager
- String-Buffer-Overflow-Risiken in CAN-Reader und SD-Logger

### Performance
- GPS-Datenempfang ohne Verluste auch bei hoher CPU-Last
- Schnellere String-Formatierung durch vorallokierte Buffer
- Reduzierte CPU-Last durch Interrupt-basiertes GPS

## [1.1.0] - 2024-11-XX

### Hinzugefügt
- Multi-Layer Buffer-Overflow-Schutz
- Umfassende Hardware-Test-Suite
- Korrelierte Datenaufzeichnung (Sensor + CAN + GPS)
- 8 OLED-Display-Modi mit Auto-Rotation
- Automatische Hardware-Erkennung mit Fallback
- Vibrations-Analyse mit RMS und Schock-Erkennung

### Geändert
- Verbesserte Error-Recovery-Mechanismen
- Optimierte Timing-Intervalle für Echtzeit-Performance
- Erweiterte Logging-Funktionalität

### Behoben
- I2C-Bus-Hänger bei Sensor-Ausfall
- SD-Karten-Initialisierung mit alternativen Pin-Sets
- CAN-Bus Oszillator-Erkennung

## [1.0.0] - 2024-10-XX

### Initiales Release
- Basis-Funktionalität mit BNO055, GPS, CAN-Bus
- SD-Karten-Datenlogging
- Einfache Straßenqualitäts-Bewertung
- OLED-Display-Unterstützung
- Grundlegende Hardware-Tests

---

## Upgrade-Anleitung

### Von 1.1.0 zu 1.2.0
1. Code aktualisieren und neu kompilieren
2. Erste Kalibrierung nach Update wird automatisch im NVS gespeichert
3. GPS wechselt automatisch in Interrupt-Modus
4. Keine Konfigurations-Änderungen erforderlich

### Von 1.0.0 zu 1.2.0
1. Backup der SD-Karte erstellen
2. Neue Firmware flashen
3. Hardware-Test durchführen
4. BNO055 neu kalibrieren (wird dann gespeichert)

## Bekannte Probleme

### v1.2.0
- Keine bekannten Probleme

### v1.1.0
- BNO055-Kalibrierung geht bei Neustart verloren (behoben in v1.2.0)
- GPS-Daten können bei hoher CPU-Last verloren gehen (behoben in v1.2.0)

## Geplante Features

### Weitere Ideen
- [ ] FFT-Analyse für Vibrations-Frequenzspektrum
- [ ] FreeRTOS Task-basierte Architektur
- [ ] WLAN-Live-Streaming
- [ ] Erweiterte webbasierte Konfiguration
- [ ] Multi-Language Support für OLED

### v2.0.0 (Zukunft)
- [ ] Machine Learning für Straßentyp-Klassifikation
- [ ] Bluetooth BLE für Smartphone-App
- [ ] Cloud-Integration mit automatischem Upload
- [ ] OTA-Updates über WiFi

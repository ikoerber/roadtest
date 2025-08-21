# Changelog

Alle wichtigen Änderungen am ESP32-S3 Straßenqualitäts-Messsystem werden in dieser Datei dokumentiert.

Das Format basiert auf [Keep a Changelog](https://keepachangelog.com/de/1.0.0/),
und dieses Projekt hält sich an [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

### v1.3.0 (Geplant)
- [ ] FFT-Analyse für Vibrations-Frequenzspektrum
- [ ] FreeRTOS Task-basierte Architektur
- [ ] WiFi-Integration für Live-Streaming
- [ ] Web-basierte Konfiguration
- [ ] Multi-Language Support für OLED

### v2.0.0 (Zukunft)
- [ ] Machine Learning für Straßentyp-Klassifikation
- [ ] Bluetooth BLE für Smartphone-App
- [ ] Cloud-Integration mit automatischem Upload
- [ ] OTA-Updates über WiFi
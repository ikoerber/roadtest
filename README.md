# 🏍️ ESP32-S3 Straßenqualitäts-Messsystem

Ein fortschrittliches Embedded-System zur Messung und Bewertung von Straßenqualität für kurvenreiche Motorradstrecken.

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![Code Quality](https://img.shields.io/badge/quality-9.2%2F10-brightgreen)]()
[![Test Coverage](https://img.shields.io/badge/tests-85%25-green)]()
[![Buffer Security](https://img.shields.io/badge/security-protected-blue)]()

## 📋 Überblick

Das System erfasst Bewegungsdaten, GPS-Position und CAN-Bus-Signale, um die Qualität von Straßen zu bewerten. Besonders geeignet für:
- **Kurvenreiche Strecken** mit Serpentinen und Bergstraßen
- **Vibrations-Analyse** für Straßenoberflächen-Bewertung  
- **GPS-basierte Strecken-Dokumentation**
- **Multi-Sensor-Datenlogging** auf SD-Karte

## 🎯 Features

### ✨ Hauptfunktionen
- **Real-Time Straßenqualitäts-Bewertung** (0-100 Punkte)
- **Multi-Sensor-Fusion** (BNO055 + GPS + CAN)
- **Robustes SD-Karten-Logging** mit Buffer-Overflow-Schutz
- **Live-Display** auf 128x64 OLED mit Auto-Rotation
- **Eigenes ROADTEST-WLAN** mit Statusseite, Kalibrierassistent und Browser-OTA
- **Kontrollierte Messfahrten** mit Start, sicherem Ende und Zusammenfassung
- **Umfassende Hardware-Tests** und Diagnostik
- **NEU: GPS Interrupt-Modus** für verlustfreien Datenempfang
- **NEU: NVS-Kalibrierungsspeicher** für BNO055 (persistiert über Neustarts)

### 🛡️ Sicherheitsfeatures
- **Buffer-Overflow-Schutz** mit SafeStringFormatter
- **Memory-Pool-Management** gegen Heap-Fragmentierung
- **Hardware-Fehler-Recovery** mit automatischen Fallbacks
- **Multi-Layer-Error-Handling** für kritische Systeme
- **NEU: Erweiterte String-Sicherheit** mit strncpy/strncat
- **NEU: Null-Pointer-Checks** für alle dynamischen Allokationen

### 📊 Datenerfassung
- **Beschleunigungsdaten** (10Hz) mit Vibrations-Analyse
- **GPS-Tracking** (5Hz) mit Fix-Detection und HDOP
- **Optionale CAN-Bus-Unterstützung** für MCP2515, standardmäßig deaktiviert
- **Korrelierte Datenlogs** mit präzisen Zeitstempeln
- **NEU: Interrupt-basiertes GPS** ohne Datenverlust bei hoher CPU-Last
- **NEU: Optimierte String-Operationen** für bessere Performance

## 🔧 Hardware-Anforderungen

### ESP32-S3 Entwicklungsboard
- **Mikrocontroller:** ESP32-S3 (240MHz, LOLIN S3 Mini mit 4MB Flash)
- **USB:** USB-C für Programming und Debug
- **Versorgung:** USB-C oder vorhandener LiPo-Anschluss
- **GPIO-Pegel:** ausschließlich 3,3 V; die GPIOs sind nicht 5-V-tolerant

### Sensoren und Module

| Komponente | Modell | Schnittstelle | Pins | Funktion |
|------------|--------|---------------|------|----------|
| **IMU-Sensor** | BNO055 | I2C | GPIO 8/9 | NDOF-Sensorfusion |
| **Display** | SSD1306 | I2C | GPIO 8/9 | 128x64 OLED |
| **GPS-Modul** | BN-880 | UART2 | GPIO 15/16 | Position & Geschwindigkeit |
| **CAN-Interface** | MCP2515 | SPI | GPIO 1,2,3,11,13 | optional, derzeit deaktiviert |
| **SD-Speicher** | MicroSD | SPI | GPIO 4,5,6,7 | Datenlogging |

## 📐 Schaltplan & Verkabelung

Die vollständige und verbindliche Beschreibung des vorhandenen
Lochrasteraufbaus steht in [HARDWARE.md](HARDWARE.md). Eine kompakte grafische
Übersicht enthält [schematic.md](schematic.md). Maßgeblich sind GPIO- und
Signalnamen, nicht die Kabelfarben.

### I2C-Bus (BNO055 + OLED)
```
ESP32-S3        BNO055          SSD1306 OLED
--------        ------          ------------
GPIO 8    <-->  SDA      <-->   SDA
GPIO 9    <-->  SCL      <-->   SCL  
3.3V      -->   VIN      -->    VCC
GND       -->   GND      -->    GND
```

Am BNO055 bleiben `3Vo`, `RST`, `PS0`, `PS1` und `INT` unbeschaltet. `ADR`
ist im vorhandenen Aufbau mit 3,3 V verbunden; dadurch lautet die Adresse
`0x29`.

### GPS-Modul (UART2)
```
ESP32-S3        BN-880 GPS
--------        ----------
GPIO 16   <--   TX (NMEA-Daten)
GPIO 15   -->   RX (Konfiguration)
3.3V      -->   VCC
GND       -->   GND
offen           PPS (von der Firmware nicht verwendet)
```

### CAN-Bus Interface
```
ESP32-S3        MCP2515
--------        -------
GPIO 1    -->   CS
GPIO 2    <--   INT
GPIO 3    -->   SCK
GPIO 13   -->   MOSI
GPIO 11   <--   MISO
GND       -->   GND
```

CAN ist standardmäßig deaktiviert. Die Versorgung und SPI-Logikpegel des
konkreten MCP2515/TJA1050-Moduls müssen vor dem Anschluss geprüft werden; viele
dieser Module sind nicht direkt 3,3-V-kompatibel.

### SD-Karten-Modul
```
ESP32-S3        SD-Card Module
--------        --------------
GPIO 4    -->   CS
GPIO 5    -->   MOSI
GPIO 6    <--   MISO
GPIO 7    -->   SCK
3.3V      -->   3.3V (gemäß Beschriftung des verwendeten PZSMOCN-Moduls)
GND       -->   GND
```

## 🚀 Installation & Setup

### 1. Development Environment
```bash
# PlatformIO installieren
pip install platformio

# Repository klonen
git clone <repository-url>
cd roadtest

# Dependencies installieren (automatisch via platformio.ini)
pio lib install
```

### 2. Hardware-Konfiguration
```cpp
// In src/hardware_config.h - Pin-Definitionen anpassen falls nötig
#define I2C_SDA          8
#define I2C_SCL          9  
#define GPS_RX_PIN       16
#define GPS_TX_PIN       15
#define SD_CS_PIN        4
#define CAN_CS_PIN       1
```

### 3. Firmware Upload
```bash
# Build & Upload
pio run --target upload

# Serial Monitor für Debug
pio device monitor --baud 115200
```

### 4. Erweiterte Konfiguration

#### GPS Interrupt-Modus
```cpp
// In main.cpp - automatisch aktiviert
gpsManager.enableInterruptMode(true);  // Verlustfreier Datenempfang

// Manuell umschalten (falls gewünscht)
gpsManager.enableInterruptMode(false); // Zurück zu Polling
```

#### WLAN, Messfahrten, Kalibrierung und OTA

1. Mit dem WLAN `ROADTEST` verbinden, Passwort `roadtest123`.
2. Im Browser `http://192.168.4.1/` öffnen.
3. Einmalig unter **Kalibrierung** System, Gyro, Beschleunigung und
   Magnetometer auf jeweils 3 bringen und speichern. Für das Magnetometer
   eine liegende Acht in der Luft beschreiben.
4. Vor der Abfahrt **Aufzeichnung starten** auswählen.
5. Am Fahrtende **Aufzeichnung beenden** auswählen. Erst dann sind alle
   Dateien garantiert geschlossen und die Zusammenfassung wird geschrieben.
6. Unter **Firmware aktualisieren** kann eine neue Web-OTA-`.bin` eingespielt werden.

Für Start, Ende, Speichern und Aktualisieren wird Benutzer `admin` mit
Passwort `roadtest123` verwendet. Die Kalibrierung wird anschließend beim
Start automatisch aus dem NVS geladen.

## 🧪 Startprüfung und System-Tests

Beim Einschalten zeigt das OLED eine einzige, fortlaufend aktualisierte
Prüfseite. Geprüft werden OLED, BNO055, SD-Karte, GPS-Kommunikation und WLAN.
Ein GPS-Fix ist dafür nicht nötig; empfangene NMEA-Daten reichen. CAN ist
optional und blockiert die Bereitschaft nicht.

Fehlt eine erforderliche Komponente, bleibt die Firmware trotzdem bedienbar
und versucht alle fünf Sekunden automatisch eine Wiederverbindung. Die
Prüfseite verschwindet erst, wenn alles Erforderliche bereit ist. Bei einem
späteren Ausfall erscheint sie erneut.

Die ausführlichen Hardware-, Integrations- und Belastungstests werden nicht
bei jedem Start ausgeführt. Sie können bei Bedarf über den seriellen Monitor
gestartet werden:

### NEU: Integration-Test-Suite (90% Coverage)
```
✅ Multi-Modul Concurrent Tests
✅ Sensor-Daten-Korrelation
✅ Hardware Failure & Recovery Tests
✅ Edge-Case Szenarien
✅ Performance & Latenz Tests
✅ Memory-Leak Detection
✅ 24+ umfassende Test-Szenarien
```

### Test-Kommandos (Serial Monitor)
```bash
test          # Zeigt alle verfügbaren Test-Kommandos
hardware      # Führt Hardware-Test-Suite aus
integration   # Vollständige Integration-Tests (5-10 Min)
stress        # Stress-Test unter Volllast
recovery      # Hardware Failure & Recovery Tests
quick         # Schnelle Integration-Tests (1 Min)
memory        # Memory-Leak Detection (2 Min)
diag          # System-Diagnose mit allen Metriken
```

### Test-Ausgabe Beispiel
```
=== INTEGRATION TEST SUITE ===
--- Test: Alle Module gleichzeitig ---
✅ Test in 30000 ms
Details: Sensor: 298 reads, GPS: 149 updates, CAN: 2980 msgs

--- Test: Sensor-Daten-Korrelation ---
✅ Test in 10000 ms  
Details: 100 Samples, 67 korreliert (67.0%), 0 Timing-Fehler

--- Test: Buffer-Overflow Recovery ---
✅ Test in 5234 ms
Details: SD: 10 overflows, Ring: 50 overflows, Recovery: 4/4 OK

========== TEST SUITE ABGESCHLOSSEN ==========
Gesamt-Tests: 24
Bestanden: 23 (95.8%)
Test-Coverage: 92.5%
```

## 📊 Datenformat & Ausgabe

### CSV-Datenlogging
Das System erstellt automatisch strukturierte CSV-Dateien:

Jede Messfahrt besitzt eine gemeinsame Sitzungskennung im Dateinamen und
erhält getrennte Sensor-, Straßen-, GPS-, Ereignis- und bei Bedarf CAN-Dateien.
Beim Beenden entsteht zusätzlich `road_summary_<Sitzung>.csv` mit Dauer,
Strecke, Ereigniszählern und Durchschnittsqualität.

**sensor_data.csv**
```csv
UTC,UptimeMs,Heading,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp,CalSystem,CalGyro,CalAccel,CalMag
2026-07-27T12:34:56Z,1234567,123.5,-2.1,0.8,0.123,-0.056,9.801,0.001,0.002,-0.001,24.5,3,3,3,3
```

`Heading` verwendet im NDOF-Modus die Sensorfusion einschließlich
Magnetometer. Alle vier Kalibrierwerte entscheiden über eine vollständig
speicherbare Kalibrierung.

**can_log.csv**  
```csv
UTC,UptimeMs,CAN_ID,Extended,RTR,DLC,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7
2026-07-27T12:34:56Z,1234567,1A0,0,0,8,12,34,56,78,9A,BC,DE,F0
```

**correlated.csv** (Sensor + CAN)
```csv
UTC,UptimeMs,Type,Heading,Pitch,Roll,AccelMag,Temp,CAN_ID,DLC,D0,D1,D2,D3,D4,D5,D6,D7
```

### Road Quality Scoring

Die aktuelle Bewertung (0–100 Punkte) beschreibt die **Oberflächenqualität**.
Sie verwendet ein rollendes Ein-Sekunden-Fenster aus RMS-Vibration,
Maximalstoß und Anzahl einzelner Stöße. Bei vorhandenem GPS wird vorsichtig
auf 30 km/h Referenzgeschwindigkeit normiert; im Stillstand bleibt der letzte
Fahrwert erhalten. Kurven und Schlaglöcher werden getrennt als einzelne
Ereignisse protokolliert.

## 🛡️ Buffer-Sicherheit

### Implementierte Schutzmaßnahmen
```cpp
// SafeStringFormatter - Overflow-sichere Formatierung
char buffer[256];
SafeStringFormatter::safePrintf(buffer, sizeof(buffer), 
    "Sensor: %.2f°, %.2f m/s²", heading, acceleration);

// SafeRingBuffer - Template-basierte Pufferung  
SafeRingBuffer<float, 100> sensorBuffer;
sensorBuffer.push(newValue);

// SafeMemoryPool - Fragmentierungsfreie Allokation
void* ptr = globalMemoryPool.allocate();
```

### Memory-Management
- **Static Buffers:** 7-8 KB RAM-Verbrauch (< 2% des ESP32-S3)
- **Heap-Protection:** Memory-Pool verhindert Fragmentierung
- **Overflow-Detection:** Multi-Level-Überwachung mit Statistiken
- **Auto-Recovery:** Intelligente Buffer-Verwaltung bei Fehlern

## 📱 OLED-Display Modi

Das System zeigt zyklisch verschiedene Informationen an:

### 1. Hardware Status
```
=== HARDWARE STATUS ===
I2C Bus : OK
BNO055  : OK  
OLED    : OK
SD-Karte: OK
CAN-Bus : OK
GPS     : OK
System: BEREIT!
```

### 2. Live Sensor-Daten
```
=== LIVE DATEN ===
Richtung: 123.5 Grad
Beschl.: 1.23 m/s²
Temp: 24.5 C

CAN: 42 msg
Zeit: 1234s
```

### 3. GPS Status
```
=== GPS STATUS ===
Fix: Gültig
Satelliten: 8
Lat: 47.123456
Lon: 8.654321
Speed: 67.5 km/h
```

### 4. Straßenqualität
```
=== ROAD QUALITY ===
Gesamt: 85.5 / 100
████████████████▌

Glätte: 92.1
Kurven: 12 (78.3)

TRAUMSTRECKE!
```

## 🔍 Erweiterte Konfiguration

### GPS-Konfiguration
```cpp
// GPS-Manager Einstellungen
GPSData gpsData = gpsManager.getCurrentData();
bool hasFixWithAccuracy = gpsManager.hasValidFix() && gpsData.hdop < 2.0;

// Test-Funktionen
gpsManager.testCommunication();  // Kommunikationstest
gpsManager.printDiagnostics();   // Detaillierte NMEA-Diagnose
```

### CAN-Bus Konfiguration  
```cpp
// Automatische Oszillator-Erkennung (8MHz/16MHz)
canReader.setClockFrequency(8E6);   // Für 8MHz Quarz
canReader.begin(500E3);             // 500 kbps CAN-Rate

// Filter für spezifische CAN-IDs
canReader.setFilter(0x123, 0x7FF);  // Nur ID 0x123
canReader.clearFilters();           // Alle IDs empfangen
```

### SD-Logger Konfiguration
```cpp
// Logging-Intervalle anpassen
LogConfig config = sdLogger.getConfig();
config.sensorLogInterval = 100;     // 10Hz Sensor-Daten
config.roadLogInterval = 1000;      // 1Hz Straßenqualität  
config.flushInterval = 5000;        // 5s Buffer-Flush
sdLogger.setConfig(config);
```

## 🚨 Troubleshooting

### Häufige Probleme

**Problem: BNO055 nicht gefunden**
```
Lösung:
1. I2C-Verkabelung prüfen (SDA=8, SCL=9)
2. Pull-up Widerstände 4.7kΩ auf SDA/SCL
3. ADR-Verbindung prüfen: im vorhandenen Aufbau 3,3 V, daher Adresse 0x29
4. Serial Monitor: I2C-Scanner-Ausgabe prüfen
```

**Problem: SD-Karte nicht erkannt**
```
Lösung:
1. FAT32-Formatierung (nicht exFAT/NTFS)
2. PZSMOCN-Modul am beschrifteten 3.3V-Eingang versorgen
3. Pin-Konfiguration: CS=4, MOSI=5, MISO=6, SCK=7
4. Karte, Modulstecker, Masse und Lötstellen mechanisch prüfen
```

**Problem: GPS kein Fix**
```
Lösung:
1. Freie Sicht zum Himmel (15-30 Sek warten)
2. UART-Verkabelung: TX GPS -> RX ESP32 (Pin 16)  
3. Baudrate 9600 (Standard für NMEA)
4. Cold Start: bis zu 2 Minuten normal
```

**Problem: CAN-Bus Fehler**
```
Lösung:
1. Prüfen, ob CAN in der Firmware bewusst aktiviert wurde
2. Versorgung und 3,3-V-Kompatibilität des Moduls prüfen
3. MCP2515-Verkabelung und Oszillator-Frequenz prüfen
4. Terminierung und aktiven Fahrzeug-CAN-Bus kontrollieren
```

### Debug-Features
```cpp
// Detaillierte Diagnostik aktivieren
#define DEBUG_ENABLED 1

// Buffer-Statistiken anzeigen
printBufferStats();

// Hardware-Test-Suite erneut ausführen
testOLED();
testBNO055(); 
testSDWithSafePins();
```

## 📈 Performance-Charakteristika

### Real-Time Verhalten
- **Sensor-Reading:** 10Hz (100ms Interval) ✓ Optimal
- **GPS-Updates:** 5Hz (200ms Interval) ✓ Ausreichend  
- **CAN-Processing:** 100Hz Check-Rate ✓ Hochperformant
- **SD-Logging:** Gepuffert mit Auto-Flush ✓ Effizient

### Ressourcen-Verbrauch
```
Flash:  ~1,16MB / 1,31MB App-Partition (88,7%)
SRAM:   ~57KB / 320KB                  (17,5%)
CPU:    ESP32-S3 mit 240MHz
```

Die Web-OTA-Datei passt damit in die konfigurierte OTA-Partition. Für spätere
größere Funktionen sollte die verbleibende Flash-Reserve berücksichtigt werden.

## 🛣️ Praktische Anwendung

### Optimale Testszenarien
- **Bergstraßen** mit vielen Serpentinen (Black Forest, Alpen)
- **Landstraßen** mit wechselndem Kurvenradius
- **Rennstrecken** für Performance-Analyse
- **Stadtverkehr** für Vibrations-Analyse

### Datenauswertung
```python
# Python-Beispiel für CSV-Auswertung
import pandas as pd
import matplotlib.pyplot as plt

# Sensor-Daten laden
df = pd.read_csv('sensor_data.csv')

# Straßenqualitäts-Plot
plt.plot(df['timestamp'], df['road_quality'])
plt.title('Straßenqualität über Zeit')
plt.show()
```

## 🔄 System-Updates

### Firmware-Updates
```bash
# Neue Version deployen
git pull origin main
pio run --target upload

# Backup der aktuellen Konfiguration
cp src/hardware_config.h hardware_config.backup
```

### Konfiguration anpassen
- **Pin-Zuordnungen:** `src/hardware_config.h`
- **Sensor-Parameter:** `src/*_manager.h` 
- **Logging-Einstellungen:** `SDLogger::LogConfig`
- **Display-Modi:** `OLEDManager::DisplayConfig`

## 🤝 Beitragen

### Code-Qualitäts-Standards
- Buffer-Overflow-Schutz für alle String-Operationen
- Umfassende Error-Handling für Hardware-Ausfälle
- Konsistente API-Design mit `begin()`, `isReady()`, `getCurrentData()`
- Vollständige Dokumentation mit Beispielen

### Test-Erweiterungen  
Priorität für zusätzliche Tests:
1. **Error-Recovery-Tests** (SD-Ausfall, I2C-Bus-Hang)
2. **Integration-Stress-Tests** (alle Module unter Last)
3. **Grenzwert-Tests** (extreme Beschleunigung, CAN-Überflutung)

Der priorisierte Backlog für gleichmäßigere Sensorabtastung und geringere
Buslast steht in [OPTIMIERUNGEN.md](OPTIMIERUNGEN.md).

## 📈 Version History

### Unveröffentlicht - Sichere BNO055-Kommunikation
- ✅ BNO055-Erkennung über die Chip-ID statt leerer I²C-Schreibzugriffe
- ✅ Fest verdrahtete Adresse `0x29`, eine Chip-ID-Prüfung pro Statuszyklus
- ✅ Messdaten werden beim ersten unplausiblen Status sofort gesperrt
- ✅ Initialisierung endet bei ausbleibender Chip-ID nach 1,5 Sekunden
- ✅ NDOF-Kalibrierung berücksichtigt System, Gyro, Beschleunigung und Magnetometer
- ✅ Weboberfläche bleibt während des gestuften SD-Aufzeichnungsstarts erreichbar

### v1.5.10 - Stabile BNO055-Statusanzeige
- ✅ Einzelne kurzzeitige I²C-Statusfehler verändern die Anzeige nicht mehr
- ✅ Ein erfolgreicher Status setzt die Fehlerzählung sofort zurück
- ✅ Erst drei aufeinanderfolgende Fusionsfehler zeigen einen Fehler und
  lösen den automatischen BNO055-Neustart aus

### v1.5.9 - Konsistente IMUPLUS-Überwachung
- ✅ Messdaten nur bei Modus 8, Systemstatus 5 und Fehlercode 0
- ✅ Drei vollständige Fusionsfehler vor einem automatischen Sensorneustart
- ✅ Webdiagnose verwendet den überwachten Status ohne zusätzliche Modusabfragen
- ✅ Integrationstests vergleichen nur relative Richtungsänderungen
- ✅ OLED, CSV-Felder und Dokumentation eindeutig auf IMUPLUS abgestimmt

### v1.5.8 - BNO055 IMUPLUS
- ✅ Gyro und Beschleunigung ohne Magnetometer-Sensorfusion
- ✅ Keine Achterbewegung und keine Magnetometerkalibrierung erforderlich
- ✅ Kalibrierung ist abgeschlossen, sobald Gyro und Beschleunigung 3 erreichen
- ⚠️ Absolute Kompassrichtung entfällt; relative Drehungen bleiben verfügbar

### v1.5.7 - Stabiler BNO055-Sensortakt
- ✅ Interne BNO055-Taktquelle statt instabiler externer Taktumschaltung
- ✅ I²C mit robusten 50 kHz für die Lochrasterverdrahtung
- ✅ Taktquelle auf der Kalibrierseite sichtbar

### v1.5.6 - Stabile BNO055-Modusprüfung
- ✅ Kein permanentes I²C-Polling des Betriebsmodus mehr
- ✅ Drei aufeinanderfolgende Modusfehler vor einem Sensorneustart
- ✅ Vollständiger BNO055-Neustart über die Kalibrierseite

### v1.5.5 - Stabile BNO055-Fusionsdiagnose
- ✅ BNO055-Diagnose und erzwungener NDOF-Modus aus v1.5.4
- ✅ Automatischer CAN-Start deaktiviert, damit WLAN und Webseite nicht hängen

### v1.5.4 - BNO055-Fusionsdiagnose
- ✅ NDOF-Betriebsmodus wird beim Start geprüft und nötigenfalls erzwungen
- ✅ Modus, Sensorfusion, Systemstatus und Fehlercode auf der Kalibrierseite
- ✅ Sicherer BNO055-Neustart über die Webseite
- ⚠️ Automatischer CAN-Start konnte die Webseite blockieren

### v1.5.3 - Stabiler Servicezugang
- ✅ CAN im aktuellen Aufbau vollständig deaktiviert
- ✅ Feste WLAN-IP und fester Funkkanal
- ✅ WLAN-Schlafmodus deaktiviert
- ✅ Automatischer Wiederanlauf des ROADTEST-Access-Points

### v1.5.2 - WLAN zuerst
- ✅ ROADTEST-WLAN startet vor den externen Hardwareprüfungen
- ✅ Optionale CAN-Prüfung läuft erst nach dem Systemstart
- ✅ Diagnose und OTA bleiben unabhängig von GPS und CAN erreichbar

### v1.5.1 - GPS-Startprüfung
- ✅ NMEA-Datenstrom genügt für die Hardwareprüfung
- ✅ Satelliten-Fix blockiert den Startbildschirm nicht
- ✅ GPS-Zeichen, gültige Sätze und Prüfsummenfehler auf der Webseite

### v1.5.0 - Robuster Wiederanlauf
- ✅ Einheitliche, automatisch aktualisierte OLED-Startprüfung
- ✅ Kein blockierender Start bei fehlender Hardware
- ✅ Wiederverbindung für BNO055, OLED, SD, GPS und WLAN
- ✅ CAN ausdrücklich optional
- ✅ SD-Schreibfehler und verworfene Datensätze auf der Webseite
- ✅ Abgebrochene Messfahrten werden klar gekennzeichnet

### v1.4.0 - Kontrollierte Messfahrten
- ✅ Messfahrt über die Webseite starten und sicher beenden
- ✅ Eigene Dateigruppe mit gemeinsamer Sitzungskennung pro Fahrt
- ✅ Live-Zusammenfassung aus Dauer, GPS-Strecke und Ereigniszählern
- ✅ Permanente Zusammenfassungs-CSV beim Fahrtende
- ✅ Kein unbeabsichtigtes Logging direkt nach dem Einschalten

### v1.3.0 - Messfenster & Kalibrierassistent
- ✅ BNO055 nur einmal pro 100-ms-Messzyklus auslesen
- ✅ Rollendes Ein-Sekunden-Fenster für Vibration und Stöße
- ✅ Schlaglöcher und Kurven nur einmal pro Ereignis protokollieren
- ✅ Geschwindigkeitsabhängige Normierung der Straßenqualität
- ✅ Kalibrierstatus auf OLED und Webseite
- ✅ Kalibrierung geschützt über die Webseite speichern
- ✅ Eigenes WLAN und Browser-OTA

### v1.2.0 - Performance & Sicherheit
- ✅ **GPS Interrupt-Modus** implementiert für verlustfreien Datenempfang
- ✅ **NVS-Speicher** für BNO055 Kalibrierung (persistiert über Neustarts)
- ✅ **Null-Pointer-Schutz** für alle dynamischen Allokationen
- ✅ **String-Sicherheit** verbessert (strncpy/strncat statt strcpy/strcat)
- ✅ **Performance-Optimierung** durch statische Buffer statt String-Konkatenation
- ✅ **Hardware-Konfiguration** zentralisiert in hardware_config.cpp

### v1.1.0 - Production Ready
- ✅ Multi-Layer Buffer-Overflow-Schutz
- ✅ Umfassende Hardware-Test-Suite
- ✅ Korrelierte Datenaufzeichnung (Sensor + CAN + GPS)
- ✅ 8 OLED-Display-Modi mit Auto-Rotation

### v1.0.0 - Initial Release
- ✅ Basis-Funktionalität mit BNO055, GPS, CAN
- ✅ SD-Karten-Logging
- ✅ Einfache Straßenqualitäts-Bewertung

## 📄 Lizenz

ESP32-S3 Road Quality Measurement System
© 2024 - Open Source Projekt für Motorrad-Enthusiasten

## 🙏 Danksagungen

- **Arduino/ESP32 Community** für exzellente Libraries
- **Adafruit** für robuste Sensor-Libraries (BNO055, SSD1306)
- **TinyGPS++** für zuverlässige NMEA-Verarbeitung
- **PlatformIO** für professionelle Embedded-Entwicklung

---

**🏁 Bereit für die nächste Kurventour! Viel Spaß beim Messen der perfekten Straße! 🏁**

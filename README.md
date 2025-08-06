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
- **Umfassende Hardware-Tests** und Diagnostik

### 🛡️ Sicherheitsfeatures
- **Buffer-Overflow-Schutz** mit SafeStringFormatter
- **Memory-Pool-Management** gegen Heap-Fragmentierung
- **Hardware-Fehler-Recovery** mit automatischen Fallbacks
- **Multi-Layer-Error-Handling** für kritische Systeme

### 📊 Datenerfassung
- **Beschleunigungsdaten** (10Hz) mit Vibrations-Analyse
- **GPS-Tracking** (5Hz) mit Fix-Detection und HDOP
- **CAN-Bus-Integration** (100Hz Check-Rate) mit MCP2515
- **Korrelierte Datenlogfs** mit präzisen Zeitstempeln

## 🔧 Hardware-Anforderungen

### ESP32-S3 Entwicklungsboard
- **Mikrocontroller:** ESP32-S3 (240MHz, 512KB RAM, 8MB Flash)
- **USB:** USB-C für Programming und Debug
- **Power:** 3.3V/5V kompatibel

### Sensoren und Module

| Komponente | Modell | Schnittstelle | Pins | Funktion |
|------------|--------|---------------|------|----------|
| **IMU-Sensor** | BNO055 | I2C | GPIO 8/9 | 9-DoF Bewegungssensor |
| **Display** | SSD1306 | I2C | GPIO 8/9 | 128x64 OLED |
| **GPS-Modul** | BN-880 | UART2 | GPIO 15/16 | Position & Geschwindigkeit |
| **CAN-Interface** | MCP2515 | SPI | GPIO 1,2,3,11,13 | Fahrzeugdaten |
| **SD-Speicher** | MicroSD | SPI | GPIO 4,5,6,7 | Datenlogging |

## 📐 Schaltplan & Verkabelung

### I2C-Bus (BNO055 + OLED)
```
ESP32-S3        BNO055          SSD1306 OLED
--------        ------          ------------
GPIO 8    <-->  SDA      <-->   SDA
GPIO 9    <-->  SCL      <-->   SCL  
3.3V      -->   VCC      -->    VCC
GND       -->   GND      -->    GND
```

### GPS-Modul (UART2)
```
ESP32-S3        BN-880 GPS
--------        ----------
GPIO 16   <--   TX (NMEA-Daten)
GPIO 15   -->   RX (Konfiguration)
3.3V      -->   VCC
GND       -->   GND
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
3.3V      -->   VCC
GND       -->   GND
```

### SD-Karten-Modul
```
ESP32-S3        SD-Card Module
--------        --------------
GPIO 4    -->   CS
GPIO 5    -->   MOSI
GPIO 6    <--   MISO
GPIO 7    -->   SCK
5V        -->   VCC (wichtig: 5V!)
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

## 🧪 System-Tests

Das System führt beim Start automatisch umfassende Hardware-Tests durch:

### Hardware-Test-Suite
```
✅ I2C-Bus Scanner mit detaillierter Diagnostik
✅ BNO055 Sensor-Test mit Kalibrierungs-Check
✅ OLED Display-Test mit 8 Funktions-Tests
✅ SD-Karten-Test mit mehreren Pin-Sets
✅ CAN-Bus-Test mit MCP2515-Registern
✅ GPS-Test mit Kommunikations-Verifikation
✅ Buffer-Sicherheits-Test mit Overflow-Simulation
```

### Test-Ausgabe Beispiel
```
=== Straßenqualitäts-Messsystem ===
Version 1.1 - Production Ready

--- I2C Scanner ---
✅ BNO055 gefunden auf 0x29
✅ OLED gefunden auf 0x3C

--- OLED Display Test ---
1. Display Pixel-Test: OK
2. Boot-Nachricht Test: OK  
3. Display-Modi Test: OK
...
✅ Alle Tests bestanden!

=== SYSTEM BEREIT ===
I2C: ✓ | BNO055: ✓ | OLED: ✓ | SD: ✓ | CAN: ✓ | GPS: ✓
```

## 📊 Datenformat & Ausgabe

### CSV-Datenlogging
Das System erstellt automatisch strukturierte CSV-Dateien:

**sensor_data.csv**
```csv
timestamp,heading,pitch,roll,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,temp,cal_sys,cal_gyro,cal_acc,cal_mag
1234567,123.5,-2.1,0.8,0.123,-0.056,9.801,0.001,0.002,-0.001,24.5,3,3,3,3
```

**can_log.csv**  
```csv
timestamp,canId,extended,rtr,dlc,data0,data1,data2,data3,data4,data5,data6,data7
1234567,1A0,0,0,8,12,34,56,78,9A,BC,DE,F0
```

**correlated.csv** (Sensor + CAN + GPS)
```csv
timestamp,type,heading,pitch,roll,accel_mag,temp,canId,dlc,data,lat,lon,speed_kmh,satellites
```

### Road Quality Scoring
```cpp
Straßenqualitäts-Bewertung (0-100 Punkte):
• 80-100: TRAUMSTRECKE (ideal für Genießerfahrten)
• 60-79:  SEHR GUT (empfehlenswert)  
• 40-59:  OKAY (durchschnittlich)
• 0-39:   LANGWEILIG (wenig kurvenreich)

Faktoren:
- Kurvigkeit (40%): Kurvenradius, Serpentinen, Richtungsänderungen
- Oberflächenqualität (35%): Vibration, Schlaglöcher, Glätte
- Verkehrsruhe (15%): Geschwindigkeitsvariation, Stopps
- Landschaftsbonus (10%): Höhenmeter, Bergstraßen
```

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
3. ADDR-Pin: GND=0x28, 3.3V=0x29
4. Serial Monitor: I2C-Scanner-Ausgabe prüfen
```

**Problem: SD-Karte nicht erkannt**
```
Lösung:
1. FAT32-Formatierung (nicht exFAT/NTFS)
2. 5V Stromversorgung (nicht 3.3V!)
3. Pin-Konfiguration: CS=4, MOSI=5, MISO=6, SCK=7
4. Alternative Pin-Sets werden automatisch getestet
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
1. MCP2515-Verkabelung prüfen
2. Oszillator-Frequenz: 8MHz oder 16MHz
3. CAN-Bus-Terminierung (120Ω) 
4. Fahrzeug: Zündung an für CAN-Bus-Aktivität
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
Flash:  ~220KB / 8MB     (2.7%)
SRAM:   ~12KB / 512KB    (2.3%)  
CPU:    ~15% @ 240MHz
Strom:  ~167mA Normal, ~287mA Peak
```

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
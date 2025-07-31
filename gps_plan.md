# BN-880 GPS + HMC5883 Kompass Integration Plan

## Hardware-Spezifikationen

### Geekstory BN-880 GPS-Modul
- **GPS-Chip**: uBlox NEO-8M oder kompatibel
- **Kompass-Chip**: HMC5883L 3-Achsen Magnetometer
- **Genauigkeit**: 2.5m CEP (GPS), 1-2° (Kompass)
- **Update-Rate**: 10Hz GPS, 75Hz Kompass
- **Stromverbrauch**: ~45mA @ 3.3V
- **Abmessungen**: 38x38mm mit integrierter Keramik-Antenne

### Pin-Konfiguration ESP32-S3

#### GPS-Verbindung (UART):
```
BN-880 GPS  →  ESP32-S3
VCC         →  3.3V
GND         →  GND
TX          →  GPIO 10 (ESP32 RX)
RX          →  GPIO 12 (ESP32 TX)
```

#### Kompass-Verbindung (I2C):
```
BN-880 HMC5883  →  ESP32-S3
VCC             →  3.3V (bereits verbunden)
GND             →  GND (bereits verbunden)
SDA             →  GPIO 8 (geteilt mit BNO055/OLED)
SCL             →  GPIO 9 (geteilt mit BNO055/OLED)
```

### Erweiterte I2C-Bus-Konfiguration:
- **0x1E**: HMC5883L Kompass (neu)
- **0x29**: BNO055 Sensor (bestehend)
- **0x3C**: OLED Display (bestehend)
- **3 Geräte** auf GPIO 8/9 I2C-Bus

## Software-Architektur

### 1. GPS-Manager Klasse (`gps_manager.h/.cpp`)

#### Funktionalität:
- UART-Kommunikation mit HardwareSerial
- NMEA-Protokoll-Parsing (GPGGA, GPRMC, GPGSV)
- Koordinaten-Extraktion (Lat/Lon/Alt)
- GPS-Status-Tracking (Satelliten, Fix-Qualität)
- Speed/Heading aus GPS-Daten

#### Interface:
```cpp
class GPSManager {
public:
    bool begin(int rxPin = 10, int txPin = 12);
    void update();
    GPSData getCurrentData();
    bool hasValidFix();
    uint8_t getSatelliteCount();
    String getStatusString();
};
```

### 2. HMC5883-Kompass Klasse (`compass_hmc5883.h/.cpp`)

#### Funktionalität:
- I2C-Kommunikation (Adresse 0x1E)
- 3-Achsen Magnetometer-Daten
- Heading-Berechnung mit Deklinationskorrektur
- Kalibrierungs-Management
- Auto-spezifische Störungsfilterung

#### Interface:
```cpp
class CompassHMC5883 {
public:
    bool begin();
    void update();
    CompassData getCurrentData();
    void startCalibration();
    bool isCalibrated();
    void setDeclination(float declination);
};
```

### 3. Dreifach-Heading-System

#### Heading-Quellen:
1. **BNO055**: Sensor-Fusion Heading (primär)
   - Gyro + Accel + Mag fusion
   - Höchste Genauigkeit bei Bewegung
   - Drift-Kompensation

2. **HMC5883**: Reiner Magnetometer-Heading (backup)
   - Unabhängig vom BNO055
   - Redundanz bei BNO055-Ausfall
   - Kalibrierbar für Auto-Umgebung

3. **GPS**: Geschwindigkeits-basierter Heading (>5 km/h)
   - Ground Track Angle
   - Referenz für Validierung
   - Unabhängig von Magnetfeldern

#### Heading-Fusion-Algorithmus:
```cpp
float getFusedHeading() {
    if (speed_gps > 5.0) {
        // Bei Geschwindigkeit: GPS als Referenz
        return validateHeading(heading_gps, heading_bno, heading_hmc);
    } else {
        // Bei Stillstand: BNO055 primär, HMC5883 backup
        return (bno055.isCalibrated()) ? heading_bno : heading_hmc;
    }
}
```

## Datenstrukturen

### GPSData Struktur:
```cpp
struct GPSData {
    float latitude;          // Breitengrad (Dezimalgrad)
    float longitude;         // Längengrad (Dezimalgrad)
    float altitude;          // Höhe über Meeresspiegel (m)
    float speed_kmh;         // Geschwindigkeit (km/h)
    float heading_deg;       // Fahrtrichtung (0-360°)
    uint8_t satellites;      // Anzahl Satelliten
    uint8_t fix_quality;     // Fix-Qualität (0=kein Fix, 1=GPS, 2=DGPS)
    bool valid_fix;          // Gültiger GPS-Fix
    unsigned long timestamp; // Zeitstempel (ms)
    
    // HDOP/VDOP für Genauigkeitsbewertung
    float hdop;              // Horizontal Dilution of Precision
    float vdop;              // Vertical Dilution of Precision
};
```

### CompassData Struktur:
```cpp
struct CompassData {
    float heading_mag;       // Magnetischer Heading (0-360°)
    int16_t raw_x, raw_y, raw_z; // Rohe Magnetometer-Werte
    float calibrated_x, calibrated_y, calibrated_z; // Kalibrierte Werte
    bool calibrated;         // Kalibrierungs-Status
    float declination;       // Magnetische Deklination
    unsigned long timestamp; // Zeitstempel (ms)
    
    // Kalibrierungs-Parameter
    float offset_x, offset_y, offset_z;  // Hard-Iron Korrektur
    float scale_x, scale_y, scale_z;     // Soft-Iron Korrektur
};
```

### Erweiterte Log-Formate:

#### Geo-lokalisierte Sensor-Logs:
```csv
timestamp,lat,lon,alt,speed_gps,speed_can,heading_gps,heading_bno,heading_hmc,satellites,hdop,vibration_rms,road_quality
```

#### GPS-Status-Log:
```csv
timestamp,satellites,fix_quality,hdop,vdop,lat,lon,alt,speed_kmh,heading_deg
```

#### Kompass-Kalibrierungs-Log:
```csv
timestamp,raw_x,raw_y,raw_z,cal_x,cal_y,cal_z,heading_mag,declination,calibrated
```

## Integration-Steps

### 1. Library-Dependencies

#### PlatformIO Libraries hinzufügen:
```ini
lib_deps = 
    adafruit/Adafruit BNO055@^1.6.3
    adafruit/Adafruit Unified Sensor@^1.1.14
    adafruit/Adafruit GFX Library@^1.11.9  
    adafruit/Adafruit SSD1306@^2.5.9
    mikalhart/TinyGPSPlus@^1.0.3
    mprograms/QMC5883LCompass@^1.0.1
    Wire
    SPI
```

### 2. Hardware-Setup Sequenz

#### setup() Erweiterung:
```cpp
void setup() {
    // ... bestehende Initialisierung ...
    
    // GPS-Manager initialisieren
    Serial.println("\nInitialisiere GPS (BN-880)...");
    if (gpsManager.begin(10, 12)) {
        Serial.println("✅ GPS erfolgreich initialisiert");
    } else {
        Serial.println("❌ GPS Initialisierung fehlgeschlagen");
    }
    
    // HMC5883 Kompass initialisieren
    Serial.println("\nInitialisiere HMC5883 Kompass...");
    if (compassHMC.begin()) {
        Serial.println("✅ Kompass erfolgreich initialisiert");
    } else {
        Serial.println("❌ Kompass Initialisierung fehlgeschlagen");
    }
    
    // I2C-Scanner um 0x1E erweitern
    i2cScanner(); // Sollte jetzt 0x1E (HMC5883) finden
}
```

### 3. Loop-Integration

#### Timing-Schema:
```cpp
void loop() {
    static unsigned long lastGPSUpdate = 0;
    static unsigned long lastCompassUpdate = 0;
    
    unsigned long currentTime = millis();
    
    // GPS-Update alle 100ms
    if (currentTime - lastGPSUpdate >= 100) {
        gpsManager.update();
        lastGPSUpdate = currentTime;
    }
    
    // Kompass-Update alle 50ms
    if (currentTime - lastCompassUpdate >= 50) {
        compassHMC.update();
        lastCompassUpdate = currentTime;
    }
    
    // Korrelierte Logs alle 200ms
    if (currentTime - lastDataLog >= 200) {
        logCorrelatedGPSData();
        lastDataLog = currentTime;
    }
}
```

### 4. OLED-Display Erweiterung

#### GPS-Status-Anzeige:
```cpp
void displayGPSStatus() {
    display.clearDisplay();
    display.setTextSize(1);
    
    GPSData gps = gpsManager.getCurrentData();
    CompassData compass = compassHMC.getCurrentData();
    
    display.println("=== GPS STATUS ===");
    display.printf("Sat: %d  Fix: %d\n", gps.satellites, gps.fix_quality);
    display.printf("Lat: %.6f\n", gps.latitude);
    display.printf("Lon: %.6f\n", gps.longitude);
    display.printf("Speed: %.1f km/h\n", gps.speed_kmh);
    display.printf("Head: G%.0f B%.0f M%.0f\n", 
                  gps.heading_deg, lastSensorData.heading, compass.heading_mag);
    
    display.display();
}
```

## Kalibrierungs-Verfahren

### 1. Magnetometer-Kalibrierung für Auto-Umgebung

#### Problem:
- Auto-Elektronik erzeugt **Hard-Iron** und **Soft-Iron** Störungen
- Metallkarosserie verzerrt Magnetfeld
- 12V-Systeme (Lichtmaschine, Zündung) erzeugen EMI

#### Lösung - Figure-8 Kalibrierung:
```cpp
void performCompassCalibration() {
    Serial.println("Starte Kompass-Kalibrierung...");
    Serial.println("Fahre eine Figure-8 (Acht) für 60 Sekunden");
    
    compassHMC.startCalibration();
    
    float min_x = 32767, max_x = -32768;
    float min_y = 32767, max_y = -32768;
    float min_z = 32767, max_z = -32768;
    
    unsigned long startTime = millis();
    while (millis() - startTime < 60000) {  // 60 Sekunden
        CompassData data = compassHMC.getCurrentData();
        
        // Min/Max-Werte sammeln
        min_x = min(min_x, data.raw_x);
        max_x = max(max_x, data.raw_x);
        // ... für Y und Z
        
        delay(50);
    }
    
    // Hard-Iron Offset berechnen
    float offset_x = (max_x + min_x) / 2;
    float offset_y = (max_y + min_y) / 2;
    float offset_z = (max_z + min_z) / 2;
    
    compassHMC.setCalibration(offset_x, offset_y, offset_z);
    Serial.println("✅ Kalibrierung abgeschlossen");
}
```

### 2. GPS-Heading-Validierung

#### Automatische Offset-Korrektur:
```cpp
void validateHeadingOffset() {
    if (gps.speed_kmh > 10.0 && gps.valid_fix) {
        float gps_heading = gps.heading_deg;
        float bno_heading = sensorData.heading;
        float hmc_heading = compass.heading_mag;
        
        // Heading-Differenzen berechnen
        float bno_offset = normalizeAngle(gps_heading - bno_heading);
        float hmc_offset = normalizeAngle(gps_heading - hmc_heading);
        
        // Persistente Offsets speichern
        if (abs(bno_offset) > 5.0) {
            Serial.printf("BNO055 Heading-Offset erkannt: %.1f°\n", bno_offset);
        }
        
        if (abs(hmc_offset) > 5.0) {
            Serial.printf("HMC5883 Heading-Offset erkannt: %.1f°\n", hmc_offset);
        }
    }
}
```

### 3. Magnetische Deklination

#### Automatische Deklination basierend auf GPS-Position:
```cpp
float calculateMagneticDeclination(float lat, float lon) {
    // Vereinfachte Berechnung für Mitteleuropa
    // Für präzise Werte: NOAA Magnetic Declination API
    
    if (lat > 45.0 && lat < 55.0 && lon > 5.0 && lon < 15.0) {
        // Deutschland: ca. 1-3° East
        return 2.0;  // Östliche Deklination
    }
    
    return 0.0;  // Default: keine Korrektur
}
```

## Testing-Strategie

### 1. Bench-Tests

#### GPS-Modul Test:
```
1. Cold Start Test - Zeit bis zum ersten Fix
2. Satelliten-Tracking - Anzahl sichtbarer Satelliten
3. HDOP-Werte - Genauigkeitsbewertung
4. NMEA-Parsing - Vollständigkeit der Daten
```

#### Kompass-Test:
```
1. I2C-Kommunikation - Basis-Funktionalität
2. Raw-Werte - Magnetometer-Response
3. Heading-Berechnung - 360° Rotation
4. Störungstest - Metallgegenstände in der Nähe
```

### 2. Integrations-Tests

#### Dreifach-Heading-Vergleich:
```cpp
void testHeadingAccuracy() {
    Serial.println("=== HEADING-VERGLEICHSTEST ===");
    
    for (int angle = 0; angle < 360; angle += 45) {
        Serial.printf("Sollwert: %d°\n", angle);
        Serial.printf("BNO055:  %.1f°\n", sensorData.heading);
        Serial.printf("HMC5883: %.1f°\n", compass.heading_mag);
        Serial.printf("GPS:     %.1f°\n", gps.heading_deg);
        
        delay(5000);  // 5 Sekunden pro Messung
    }
}
```

### 3. Auto-Tests

#### Minimal-Test-Protokoll:
```
1. Statischer Test (Motor aus):
   - GPS-Fix-Zeit
   - Kompass-Genauigkeit
   - I2C-Bus-Stabilität

2. Motor-Lauf-Test (Stand):
   - EMI-Störungen durch Lichtmaschine
   - Kompass-Drift bei laufendem Motor

3. Kurze Testfahrt (5 Minuten):
   - GPS-Tracking-Qualität
   - Heading-Korrelation bei Kurven
   - Datenlogger-Performance

4. Kalibrierungs-Fahrt:
   - Figure-8 für Kompass-Kalibrierung
   - Verschiedene Geschwindigkeiten
   - Stadt vs. Autobahn
```

## Troubleshooting-Guide

### Häufige Probleme:

#### GPS bekommt keinen Fix:
```
- Antenne freie Sicht zum Himmel?
- Cold Start dauert bis zu 30 Sekunden
- Indoor-Test unmöglich
- NMEA-Messages kommen an? ($GPGGA check)
```

#### Kompass zeigt falsche Richtung:
```
- Kalibrierung notwendig (Figure-8)
- Metallgegenstände in der Nähe?
- Hard-Iron/Soft-Iron Störungen
- Magnetische Deklination eingestellt?
```

#### I2C-Bus Konflikte:
```
- i2cScanner zeigt alle 3 Geräte (0x1E, 0x29, 0x3C)?
- Pull-up Widerstände ausreichend?
- Bus-Speed zu hoch? (100kHz verwenden)
- Wire.begin() vor allen I2C-Zugriffen?
```

#### UART-Kommunikationsfehler:
```
- Baudrate korrekt? (meist 9600)
- TX/RX-Pins vertauscht?
- 3.3V Spannungsversorgung stabil?
- Serial2.begin() für Hardware-UART?
```

---

## Implementierungs-Prioritäten

### Phase 1: Basis-Funktionalität
1. GPS-Manager mit NMEA-Parsing
2. HMC5883-Kompass mit I2C
3. Basis-Logging mit Koordinaten

### Phase 2: Integration
1. Dreifach-Heading-System
2. Zeitkorrelierte Logs
3. OLED-GPS-Status

### Phase 3: Optimierung  
1. Kalibrierungs-Routinen
2. Auto-spezifische Störungsfilterung
3. Erweiterte Datenanalyse

### Phase 4: Validierung
1. Umfassende Tests
2. SavvyCAN-Integration
3. Performance-Optimierung

---

**Implementierung bereit für BN-880 GPS + HMC5883 Kompass Integration!** 🛰️🧭
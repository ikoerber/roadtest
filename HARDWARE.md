# 🔌 Hardware-Setup: ESP32-S3 Straßenqualitäts-Messsystem

## 📋 Einkaufsliste & Komponentenübersicht

### ESP32-S3 Entwicklungsboard
- **ESP32-S3-DevKitC-1** oder **WROOM-32S3** 
- **Preis:** ~15-25€
- **Spezifikation:** 240MHz, 512KB RAM, 8MB Flash, USB-C
- **Bezugsquelle:** AZ-Delivery, Espressif, Adafruit

### Sensoren & Module

| Komponente | Modell/Typ | Preis | Funktion | Bezugsquelle |
|------------|------------|-------|----------|--------------|
| **IMU-Sensor** | BNO055 Breakout | ~25€ | IMUPLUS: Gyro + Beschleunigung | Adafruit, Bosch |
| **Display** | SSD1306 OLED 0.96" | ~5€ | 128x64 Monochrom | AZ-Delivery, Waveshare |
| **GPS-Modul** | BN-880 GPS | ~15€ | GPS/GLONASS/Beidou | Beitian, AliExpress |
| **CAN-Interface** | MCP2515 + TJA1050 | ~8€ | CAN-Bus-Transceiver | CAN-Bus Triple |
| **SD-Karte-Modul** | MicroSD Breakout | ~3€ | Datenlogging | Standard SPI |

**Gesamtkosten Hardware:** ~70-80€

### Verkabelung & Zubehör
- **Breadboard** (830 Punkte) oder **Perfboard**
- **Jumperkabel** männlich-weiblich (40-Pack)
- **Lötdraht** 0.25mm² verschiedene Farben  
- **Pull-up Widerstände** 4.7kΩ (für I2C)
- **Kondensatoren** 100nF, 10µF (Entstörung)
- **MicroSD-Karte** 8-32GB, Class 10, FAT32

## 🔧 Detaillierte Verkabelung

### 1. I2C-Bus (BNO055 + OLED)

```
ESP32-S3 Pin    BNO055 Pin    SSD1306 Pin    Funktion
------------    ----------    -----------    --------
GPIO 8          SDA           SDA            I2C Data
GPIO 9          SCL           SCL            I2C Clock  
3.3V            VCC           VCC            Stromversorgung
GND             GND           GND            Masse
                              
Zusätzlich: 4.7kΩ Pull-up Widerstände auf SDA und SCL
```

#### BNO055 Besonderheiten:
- **ADDR-Pin:** GND = Adresse 0x28, 3.3V = Adresse 0x29
- **RST-Pin:** Optional an ESP32 GPIO für Hardware-Reset
- **PS0/PS1:** Beide auf GND für I2C-Modus

### 2. GPS-Modul BN-880 (UART2)

```
ESP32-S3 Pin    BN-880 Pin    Funktion
------------    ----------    --------
GPIO 16         TX            ESP32 empfängt NMEA-Daten
GPIO 15         RX            ESP32 sendet GPS-Befehle
3.3V            VCC           Stromversorgung (3.3V!)  
GND             GND           Masse

Baudrate: 9600 (Standard NMEA)
Format: 8N1 (8 Datenbit, keine Parität, 1 Stoppbit)
```

### 3. CAN-Bus Interface MCP2515

```
ESP32-S3 Pin    MCP2515 Pin   TJA1050 Pin   Funktion
------------    -----------   -----------   --------
GPIO 1          CS            -             Chip Select
GPIO 2          INT           -             Interrupt  
GPIO 3          SCK           -             SPI Clock
GPIO 13         MOSI (SI)     -             Master Out Slave In
GPIO 11         MISO (SO)     -             Master In Slave Out
3.3V            VCC           VCC           Stromversorgung
GND             GND           GND           Masse
                TXD           TXD           CAN High
                RXD           RXD           CAN Low
                              CANH          CAN-Bus High (zum Fahrzeug)
                              CANL          CAN-Bus Low (zum Fahrzeug)

Wichtig: 120Ω Terminierungswiderstand zwischen CANH und CANL
```

### 4. SD-Karten-Modul

```
ESP32-S3 Pin    SD-Modul Pin  Funktion
------------    ------------  --------
GPIO 4          CS            Chip Select
GPIO 5          MOSI          Master Out Slave In
GPIO 6          MISO          Master In Slave Out  
GPIO 7          SCK           SPI Clock
5V              VCC           Stromversorgung (5V wichtig!)
GND             GND           Masse

SD-Karte: FAT32 formatiert, Class 10 empfohlen
```

## ⚡ Stromversorgung & Spannungen

### Spannungsverteilung
```
Komponente      Spannung      Strom (typ.)   Strom (max.)
----------      --------      ------------   ------------
ESP32-S3        3.3V          80mA           200mA
BNO055          3.3V          12mA           50mA  
SSD1306 OLED    3.3V          20mA           40mA
BN-880 GPS      3.3V          25mA           45mA
MCP2515 CAN     3.3V          5mA            25mA
SD-Karte        5V            25mA           200mA (Schreiben)
---------------------------------------------------
Gesamt                        167mA          560mA
```

### Stromversorgungsoptionen
1. **USB-Versorgung (Entwicklung):** USB-C vom ESP32-S3
2. **Externe Versorgung (Auto):** 12V → 5V Buck-Converter → ESP32-S3
3. **Powerbank (Portable):** 5V USB-Powerbank mit 2A+ Kapazität

## 🛠️ Mechanischer Aufbau

### Gehäuse-Empfehlungen
- **Hammond 1591XXFLBK** (100x50x25mm) für Prototyping
- **IP65-Gehäuse** für Motorrad-Einsatz (wasserdicht)
- **Automotive-Grade** Gehäuse für permanente Installation

### Montage-Überlegungen
```
Sensor-Platzierung:
• BNO055: Starr mit dem Fahrzeug verbunden, möglichst zentral
• GPS-Antenne: Freie Himmelssicht, außen am Gehäuse  
• OLED-Display: Sichtbar für den Fahrer, blendungsfrei
• SD-Karte: Zugänglich für Wartung
```

### Vibrations-Dämpfung
- **Soft-Mounting** für das gesamte Gehäuse
- **Schaumstoff-Dämpfung** um das ESP32-S3  
- **Flexible Kabelführung** gegen Ermüdungsbrüche

## 🔍 Debugging & Testpunkte

### Hardware-Debug-Pins
```cpp
// Zusätzliche Test-Pins definieren
#define DEBUG_LED_PIN    2      // Status-LED
#define DEBUG_BUTTON_PIN 0      // Test-Taster  
#define DEBUG_PWM_PIN    4      // Oszilloskop-Ausgang

### Belegung OBD2 OBDII 16-poliger Stecker
Pin-Karte: 
1 – Schwarz, 
2 – Braun, 
3 – Rot, 
4 – Orange, 
5 – Gelb, 
6 – Grün     // CAN High (J-2284)
7 – Blau, 
8 – Lila, 
9 – Grau, 
10 – Weiß, 
11 – Rosa, 
12 – Hell Grün, 
13 – Schwarz und Weiß, 
14 – Braun und Weiß,  // CAN Low (J-2284)
15 – Rot und Weiß, 
16 – Grün und Weiß.

// Im Code verwenden:
digitalWrite(DEBUG_LED_PIN, HIGH);  // Status signalisieren
```

### Messungen & Validierung
1. **I2C-Bus:** Oszilloskop auf SDA/SCL für Timing-Analyse
2. **SPI-Bus:** Logic-Analyzer für SD-Karte und CAN-Bus
3. **UART-GPS:** Terminal-Programm für NMEA-Nachrichten
4. **Stromverbrauch:** Multimeter in Serienschaltung

## ⚠️ Häufige Hardware-Probleme

### I2C-Bus Probleme
```
Problem: BNO055 oder OLED nicht erkannt
Lösung:
1. Pull-up Widerstände 4.7kΩ auf SDA und SCL prüfen
2. Kabelverbindungen kontrollieren (max. 10cm für Tests)
3. I2C-Scanner laufen lassen: Wire.endTransmission()
4. Mehrere Geräte: Unterschiedliche Adressen sicherstellen
```

### SD-Karte Probleme  
```
Problem: SD-Karte nicht initialisiert
Lösung:
1. FAT32-Formatierung (nicht exFAT oder NTFS)
2. 5V Spannungsversorgung (ESP32 = 3.3V reicht nicht)
3. Kurze, stabile Kabelverbindungen (max. 15cm)
4. SD-Karte in PC testen (defekte Karten häufig)
```

### GPS-Empfang schwach
```
Problem: GPS kein Fix oder wenig Satelliten
Lösung:  
1. Antenne außerhalb des Metallgehäuses platzieren
2. Freie Sicht zum Himmel (nicht in Garagen testen)
3. Cold Start: 30-60 Sekunden Geduld
4. NMEA-Nachrichten mit Terminal-Programm kontrollieren
```

### CAN-Bus keine Daten
```
Problem: MCP2515 initialisiert, aber keine CAN-Nachrichten
Lösung:
1. Fahrzeug: Zündung einschalten (CAN-Bus aktiv)
2. Terminierung: 120Ω zwischen CANH und CANL  
3. Oszillator-Frequenz: 8MHz oder 16MHz konfigurieren
4. OBD-II-Port: Pin 6 = CANH, Pin 14 = CANL
```

## 🔧 Erweiterte Hardware-Optionen

### Performance-Upgrades
- **ESP32-S3 mit PSRAM:** Für komplexe Datenverarbeitung
- **Hochgeschwindigkeits-SD:** UHS-I/UHS-II für >100MB/s
- **RTK-GPS:** Zentimeter-genaue Positionierung
- **IMU-Upgrade:** ICM-20948 oder LSM9DS1 als Alternative

### Zusätzliche Sensoren
- **Umweltsensoren:** BME280 (Temperatur, Luftfeuchtigkeit, Luftdruck)
- **Lichtsensor:** TSL2561 für automatische Display-Helligkeit
- **Audio:** I2S-Mikrofon für Motorgeräusch-Analyse
- **Kamera:** ESP32-CAM für Strecken-Dokumentation

### Konnektivität-Erweiterungen  
- **WiFi-Modul:** ESP32 Built-in für Live-Datenübertragung
- **Bluetooth:** Für Smartphone-App-Kopplung
- **LoRaWAN:** Für weitreichende Datenübertragung
- **4G/LTE:** Für Cloud-Logging in Echtzeit

## 📐 PCB-Design (Fortgeschritten)

### Custom-PCB-Überlegungen
```
Vorteile:
+ Kompakter Aufbau (50x70mm möglich)
+ Bessere EMI-Abschirmung  
+ Professioneller Look
+ Industrielle Zuverlässigkeit

Layout-Tipps:
• Separate Analog/Digital-Grounds
• I2C-Leitungen kurz und parallel führen
• SPI-Signale mit Ground-Vias abschirmen  
• Power-Planes für stabile Versorgung
```

### Empfohlene PCB-Services
- **JLCPCB** (günstig, gute Qualität)
- **PCBWay** (professionelle Fertigung)  
- **Aisler** (europäischer Service)

## 🎯 Testing & Validierung

### Funktionstest-Checklist
```
Hardware-Aufbau:
☐ Alle Verbindungen laut Schaltplan
☐ Pull-up Widerstände auf I2C-Bus
☐ 5V für SD-Karte, 3.3V für Rest
☐ CAN-Terminierung 120Ω vorhanden

Software-Test:
☐ I2C-Scanner findet BNO055 und OLED
☐ GPS sendet NMEA-Nachrichten  
☐ SD-Karte wird erkannt und beschreibbar
☐ CAN-Bus zeigt Fahrzeugdaten
☐ OLED zeigt alle Test-Modi an

Integration:
☐ Alle Module gleichzeitig funktional
☐ Keine Timing-Konflikte  
☐ Buffer-Overflow-Tests bestanden
☐ Memory-Leaks über mehrere Stunden
```

---

**🔧 Mit diesem Hardware-Setup ist Ihr ESP32-S3 Straßenqualitäts-Messsystem bereit für professionelle Messfahrten! 🔧**

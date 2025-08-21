#include "oled_manager.h"
#include <Wire.h>

// Globale OLED-Manager Instanz
OLEDManager oledManager;

OLEDManager::OLEDManager() : 
    display(nullptr), initialized(false), currentMode(DISPLAY_MODE_STATUS),
    lastModeSwitch(0), lastUpdate(0), i2cAddress(0x3C), addressFound(false),
    rotated180(false) {
    
    // Standard-Konfiguration
    config = {
        true,   // autoRotate
        10000,  // rotateInterval (10 Sekunden)
        255,    // brightness (max)
        false   // showDebugInfo
    };
}

OLEDManager::~OLEDManager() {
    end();
}

bool OLEDManager::begin(uint8_t address) {
    if (initialized) {
        Serial.println("OLED-Manager bereits initialisiert");
        return true;
    }
    
    Serial.println("=== OLED-Manager Initialisierung ===");
    Serial.printf("Versuche Adresse 0x%02X\n", address);
    
    i2cAddress = address;
    
    // Display-Instanz erstellen
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    
    if (!display) {
        Serial.println("❌ Speicher-Allokation für Display fehlgeschlagen");
        return false;
    }
    
    // Initialisierung mit SSD1306_SWITCHCAPVCC
    if (!display->begin(SSD1306_SWITCHCAPVCC, i2cAddress)) {
        Serial.printf("❌ OLED-Init auf 0x%02X fehlgeschlagen\n", i2cAddress);
        
        // Alternative Adresse versuchen
        uint8_t altAddress = (i2cAddress == 0x3C) ? 0x3D : 0x3C;
        Serial.printf("Versuche alternative Adresse 0x%02X\n", altAddress);
        
        if (!display->begin(SSD1306_SWITCHCAPVCC, altAddress)) {
            Serial.printf("❌ OLED-Init auf 0x%02X ebenfalls fehlgeschlagen\n", altAddress);
            delete display;
            display = nullptr;
            return false;
        }
        
        i2cAddress = altAddress;
    }
    
    addressFound = true;
    initialized = true;
    
    Serial.printf("✅ OLED erfolgreich initialisiert auf Adresse 0x%02X\n", i2cAddress);
    Serial.printf("Display: %dx%d Pixel\n", SCREEN_WIDTH, SCREEN_HEIGHT);
    
    // Standard-Rotation setzen (normal)
    display->setRotation(0);
    
    // Initialer Display-Test
    showBootMessage("OLED Manager", 100);
    delay(1000);
    
    return true;
}

void OLEDManager::end() {
    if (display) {
        display->clearDisplay();
        display->display();
        delete display;
        display = nullptr;
    }
    initialized = false;
    addressFound = false;
    
    Serial.println("OLED-Manager beendet");
}

void OLEDManager::setMode(DisplayMode mode) {
    if (currentMode != mode) {
        currentMode = mode;
        lastModeSwitch = millis();
        
        // Sofortiges Update im neuen Modus
        lastUpdate = 0;
    }
}

void OLEDManager::update() {
    if (!initialized || !display) return;
    
    unsigned long currentTime = millis();
    
    // Automatischer Modus-Wechsel
    if (config.autoRotate && (currentTime - lastModeSwitch >= config.rotateInterval)) {
        int nextMode = (currentMode + 1) % 4; // Zwischen 4 Modi wechseln
        setMode((DisplayMode)nextMode);
    }
    
    // Display-Update (alle 500ms)
    if (currentTime - lastUpdate >= 500) {
        // Hier könnten wir basierend auf currentMode unterschiedliche Updates machen
        // Für jetzt: manuelles Update über spezifische show-Funktionen
        lastUpdate = currentTime;
    }
}

void OLEDManager::clear() {
    if (!initialized || !display) return;
    display->clearDisplay();
}

void OLEDManager::refresh() {
    if (!initialized || !display) return;
    display->display();
}

void OLEDManager::clearAndSetup() {
    if (!display) return;
    
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
}

void OLEDManager::drawHeader(const String& title) {
    if (!display) return;
    
    display->setTextSize(1);
    display->println("=== " + title + " ===");
    display->println();
}

void OLEDManager::drawStatusIndicator(const String& label, bool status, int line) {
    if (!display) return;
    
    display->setCursor(0, line * 8);
    display->print(label + ": ");
    display->println(status ? "OK" : "FEHLER");
}

void OLEDManager::drawProgressBar(int x, int y, int width, int height, float progress) {
    if (!display) return;
    
    // Rahmen zeichnen
    display->drawRect(x, y, width, height, SSD1306_WHITE);
    
    // Fortschritt füllen (0.0 - 1.0)
    int fillWidth = (int)(progress * (width - 2));
    if (fillWidth > 0) {
        display->fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
    }
}

String OLEDManager::formatFloat(float value, int decimals) {
    return String(value, decimals);
}

void OLEDManager::showHardwareStatus(bool i2c, bool bno055, bool oled, bool sd, bool can, bool gps) {
    clearAndSetup();
    drawHeader("HARDWARE STATUS");
    
    // Status für jede Komponente
    drawStatusIndicator("I2C Bus ", i2c, 0);
    drawStatusIndicator("BNO055  ", bno055, 1);
    drawStatusIndicator("OLED    ", oled, 2);
    drawStatusIndicator("SD-Karte", sd, 3);
    drawStatusIndicator("CAN-Bus ", can, 4);
    drawStatusIndicator("GPS     ", gps, 5);
    
    display->println();
    
    // Gesamtstatus (GPS optional)
    bool allCriticalOK = i2c && bno055 && oled && sd;
    display->print("System: ");
    if (allCriticalOK) {
        display->println("BEREIT!");
    } else {
        display->println("FEHLER");
    }
    
    refresh();
}

void OLEDManager::showTestResults(const String& testName, bool success, const String& details) {
    clearAndSetup();
    
    // Test-Name anzeigen
    display->println(testName);
    display->println("----------------");
    
    // Status anzeigen
    if (success) {
        display->println("STATUS: OK");
    } else {
        display->println("STATUS: FEHLER");
    }
    
    display->println();
    
    // Details anzeigen
    if (details.length() > 0) {
        // Lange Strings umbrechen
        int lineLength = 21; // Maximale Zeichen pro Zeile bei Textgröße 1
        
        for (int i = 0; i < details.length(); i += lineLength) {
            String line = details.substring(i, min(i + lineLength, (int)details.length()));
            display->println(line);
        }
    }
    
    refresh();
}

void OLEDManager::showSensorData(float heading, float accel, float temp, int canCount) {
    clearAndSetup();
    drawHeader("LIVE DATEN");
    
    // Heading/Richtung
    display->print("Richtung: ");
    display->print(formatFloat(heading, 1));
    display->println(" Grad");
    
    // Beschleunigung
    display->print("Beschl.: ");
    display->print(formatFloat(accel, 2));
    display->println(" m/s2");
    
    // Temperatur
    display->print("Temp: ");
    display->print(formatFloat(temp, 1));
    display->println(" C");
    
    display->println();
    
    // CAN-Nachrichten
    display->print("CAN: ");
    display->print(canCount);
    display->println(" msg");
    
    // Betriebszeit
    display->print("Zeit: ");
    display->print(millis() / 1000);
    display->println("s");
    
    refresh();
}

void OLEDManager::showGPSStatus(float lat, float lon, float speed, int satellites, bool fix) {
    clearAndSetup();
    drawHeader("GPS STATUS");
    
    // Fix Status
    display->print("Fix: ");
    display->println(fix ? "Gueltig" : "Kein Fix");
    
    display->print("Satelliten: ");
    display->println(satellites);
    
    // Position (falls gültiger Fix)
    if (fix) {
        display->printf("Lat: %.6f\n", lat);
        display->printf("Lon: %.6f\n", lon);
        display->printf("Speed: %.1f km/h\n", speed);
    } else {
        display->println("Warte auf Fix...");
        display->println("(15-30 Sekunden)");
    }
    
    refresh();
}

void OLEDManager::showRoadQuality(float quality, float smoothness, float curviness, int curves) {
    clearAndSetup();
    drawHeader("ROAD QUALITY");
    
    // Gesamt-Bewertung
    display->print("Gesamt: ");
    display->print(formatFloat(quality, 1));
    display->println(" / 100");
    
    // Fortschrittsbalken für Gesamtbewertung
    drawProgressBar(0, 16, 120, 8, quality / 100.0);
    
    display->setCursor(0, 28);
    
    // Einzelbewertungen
    display->print("Glatte: ");
    display->println(formatFloat(smoothness, 1));
    
    display->print("Kurven: ");
    display->print(curves);
    display->print(" (");
    display->print(formatFloat(curviness, 1));
    display->println(")");
    
    // Bewertungstext
    display->println();
    if (quality >= 80) {
        display->println("TRAUMSTRECKE!");
    } else if (quality >= 60) {
        display->println("SEHR GUT");
    } else if (quality >= 40) {
        display->println("OKAY");
    } else {
        display->println("LANGWEILIG");
    }
    
    refresh();
}

void OLEDManager::showSystemInfo(const String& version, unsigned long uptime, uint32_t freeHeap) {
    clearAndSetup();
    drawHeader("SYSTEM INFO");
    
    display->println("Version: " + version);
    
    display->print("Uptime: ");
    display->print(uptime / 1000);
    display->println("s");
    
    display->print("RAM frei: ");
    display->print(freeHeap / 1024);
    display->println(" KB");
    
    display->print("Flash: 35.3%");
    
    refresh();
}

void OLEDManager::showBootMessage(const String& message, int progress) {
    clearAndSetup();
    
    display->setTextSize(1);
    display->println("ESP32-S3 Road System");
    display->println();
    display->println(message);
    
    if (progress >= 0) {
        display->println();
        display->print("Progress: ");
        display->print(progress);
        display->println("%");
        
        // Fortschrittsbalken
        drawProgressBar(0, 40, 120, 10, progress / 100.0);
    }
    
    refresh();
}

void OLEDManager::showErrorMessage(const String& error, const String& details) {
    clearAndSetup();
    drawHeader("FEHLER");
    
    display->println(error);
    
    if (details.length() > 0) {
        display->println();
        display->println(details);
    }
    
    refresh();
}

void OLEDManager::showDebugInfo(const String& info) {
    if (!config.showDebugInfo) return;
    
    clearAndSetup();
    drawHeader("DEBUG");
    
    display->println(info);
    
    refresh();
}

bool OLEDManager::testDisplay() {
    if (!initialized || !display) return false;
    
    // Einfacher Display-Test
    clearAndSetup();
    display->println("Display Test");
    display->println("Pixel Test...");
    
    // Pixel-Test
    for (int i = 0; i < 10; i++) {
        display->drawPixel(i * 12, 30, SSD1306_WHITE);
    }
    
    refresh();
    delay(1000);
    
    return true;
}

String OLEDManager::getDisplayInfo() {
    String info = "OLED-Manager Status:\n";
    info += "Initialisiert: " + String(initialized ? "Ja" : "Nein") + "\n";
    if (addressFound) {
        info += "I2C-Adresse: 0x" + String(i2cAddress, HEX) + "\n";
    }
    info += "Modus: " + String(currentMode) + "\n";
    info += "Auto-Rotate: " + String(config.autoRotate ? "Ja" : "Nein");
    
    return info;
}

bool OLEDManager::scanI2CAddresses() {
    Serial.println("Scanne I2C-Adressen für OLED...");
    
    for (uint8_t addr = 0x3C; addr <= 0x3D; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("OLED gefunden auf Adresse 0x%02X\n", addr);
            return true;
        }
    }
    
    Serial.println("Kein OLED auf Standardadressen (0x3C, 0x3D) gefunden");
    return false;
}

String OLEDManager::getI2CDeviceInfo(uint8_t address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
        return "Gerät gefunden auf 0x" + String(address, HEX);
    } else {
        return "Kein Gerät auf 0x" + String(address, HEX);
    }
}

// Legacy-Kompatibilitätsfunktionen (deprecated)

bool initOLED() {
    return oledManager.begin();
}

void displayTestResults(const char* testName, bool success, const char* details) {
    String detailsStr = details ? String(details) : "";
    oledManager.showTestResults(String(testName), success, detailsStr);
}

void displayHardwareStatus(bool i2c, bool bno055, bool oled, bool sd, bool can, bool gps) {
    oledManager.showHardwareStatus(i2c, bno055, oled, sd, can, gps);
}

void displaySensorData(float heading, float accel, float temp, int canCount) {
    oledManager.showSensorData(heading, accel, temp, canCount);
}

void displayGPSStatus(float lat, float lon, float speed, int satellites, bool fix) {
    oledManager.showGPSStatus(lat, lon, speed, satellites, fix);
}

void OLEDManager::setRotation(bool rotate180) {
    if (!initialized || !display) return;
    
    rotated180 = rotate180;
    
    // Rotation setzen: 0 = normal, 2 = 180 Grad gedreht
    display->setRotation(rotated180 ? 2 : 0);
    
    Serial.printf("Display-Rotation: %s\n", rotated180 ? "180 Grad" : "Normal");
    
    // Display neu zeichnen
    refresh();
}
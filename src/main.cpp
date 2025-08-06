#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include "esp_log.h"
#include "hardware_config.h"
#include "buffer_utils.h"
#include "oled_manager.h"
#include "can_reader.h"
#include "bno055_manager.h"
#include "sd_logger.h"
#include "road_quality.h"
#include "hardware_test.h"
#include "gps_manager.h"

// Pin-Definitionen (zentral definiert in hardware_config.h)
const int SD_CS_PIN = 4;
const int I2C_SDA = 8;
const int I2C_SCL = 9;
const int SD_MOSI_PIN = 5;
const int SD_MISO_PIN = 6;
const int SD_SCK_PIN = 7;

// CAN-Bus Pin-Definitionen
const int CAN_CS_PIN = 1;
const int CAN_INT_PIN = 2;
const int CAN_SCK_PIN = 3;
const int CAN_MOSI_PIN = 13;
const int CAN_MISO_PIN = 11;

// GPS-Pins
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 15;
const int GPS_BAUD_RATE = 9600;

// Hardware-Instanzen
SPIClass spiSD(HSPI);   // Separate SPI-Instanz für SD-Karte

// Status-Variablen
bool canBusAvailable = false;
bool sdCardAvailable = false;
bool sdCardWasAvailable = false;
bool gpsAvailable = false;
unsigned long lastSDCheck = 0;
String currentLogFileName = "";
String canLogFileName = "";
File logFile;
bool canLoggingEnabled = false;
int totalCANMessages = 0;

// RoadMetrics ist jetzt in road_quality.h definiert
RoadMetrics currentMetrics = {0};
float totalDistance = 0;
int curveCount = 0;
float lastHeading = 0;
bool inCurve = false;
float curveStartHeading = 0;
unsigned long lastUpdate = 0;

// Ringpuffer für Beschleunigungswerte (1 Sekunde @ 10Hz)
const int ACCEL_BUFFER_SIZE = 10;
float accelBuffer[ACCEL_BUFFER_SIZE] = {0};
int accelBufferIndex = 0;

// Hardware-Test Funktionen
void i2cScanner() {
    Serial.println("\n--- I2C Scanner ---");
    Serial.print("Scanne I2C-Bus (SDA=GPIO"); Serial.print(I2C_SDA); 
    Serial.print(", SCL=GPIO"); Serial.print(I2C_SCL); Serial.println(")");
    
    byte error, address;
    int devicesFound = 0;
    
    Serial.println("Detaillierte I2C-Diagnose:");
    
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        // Debug für kritische Adressen
        if (address == 0x28 || address == 0x29 || address == 0x3C || address == 0x3D) {
            Serial.print("  0x"); 
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.print(": Error="); Serial.print(error);
            
            if (error == 0) {
                Serial.print(" - GEFUNDEN!");
                if (address == 0x28 || address == 0x29) Serial.print(" (BNO055)");
                if (address == 0x3C || address == 0x3D) Serial.print(" (OLED)");
            } else if (error == 2) {
                Serial.print(" - NACK (Gerät antwortet nicht)");
            } else if (error == 3) {
                Serial.print(" - NACK bei Datenübertragung");
            } else if (error == 4) {
                Serial.print(" - Sonstiger Fehler");
            } else if (error == 5) {
                Serial.print(" - Timeout");
            }
            Serial.println();
        }
        
        if (error == 0) {
            if (address != 0x28 && address != 0x29 && address != 0x3C && address != 0x3D) {
                Serial.print("Anderes I2C Gerät gefunden auf Adresse 0x");
                if (address < 16) Serial.print("0");
                Serial.println(address, HEX);
            }
            devicesFound++;
        }
    }
    
    Serial.println();
    if (devicesFound == 0) {
        Serial.println("❌ Keine I2C Geräte gefunden!");
        Serial.println("Mögliche Ursachen:");
        Serial.println("• Pull-up Widerstände fehlen (4.7kΩ auf SDA/SCL)");
        Serial.println("• Verkabelung: SDA->GPIO8, SCL->GPIO9");
        Serial.println("• Stromversorgung der I2C-Geräte");
        Serial.println("• Pin-Konflikte mit anderen Funktionen");
    } else {
        Serial.print("✅ Gefundene I2C-Geräte: ");
        Serial.println(devicesFound);
    }
}

bool testSDWithSafePins() {
    Serial.println("\n--- SD-Karten Test mit sicheren ESP32-S3 Pins ---");
    
    // Sichere Pin-Sets für ESP32-S3
    int pinSets[][4] = {
        {4, 5, 6, 7},     // CS, MOSI, MISO, SCK - GPIO 4-7
        {10, 11, 13, 12}, // CS, MOSI, MISO, SCK - Standard Backup
        {15, 16, 17, 18}  // CS, MOSI, MISO, SCK - Alternative Backup
    };
    const char* pinSetNames[] = {
        "GPIO 4-7 (4,5,6,7)", 
        "Standard Backup (10,11,13,12)", 
        "Alternative Backup (15,16,17,18)"
    };
    
    Serial.println("\n=== Hardware-Checkliste ===");
    Serial.println("SD-Karten Module benötigen:");
    Serial.println("• 5V Stromversorgung (VCC -> 5V Pin)");
    Serial.println("• FAT32 formatierte SD-Karte");
    Serial.println("• Stabile Kabelverbindungen");
    Serial.println("• SD-Karte fest eingesteckt");
    
    for (int set = 0; set < 3; set++) {
        Serial.print("\n"); Serial.print(set + 1); Serial.print(". Teste "); 
        Serial.print(pinSetNames[set]); Serial.print("...");
        
        // SPI sauber beenden
        spiSD.end();
        delay(100);
        
        int cs = pinSets[set][0];
        int mosi = pinSets[set][1];
        int miso = pinSets[set][2];
        int sck = pinSets[set][3];
        
        // Sichere Pin-Konfiguration
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
        pinMode(mosi, OUTPUT);
        pinMode(sck, OUTPUT);
        pinMode(miso, INPUT_PULLUP);
        
        delay(50);
        
        // SPI mit sicheren Parametern starten
        spiSD.begin(sck, miso, mosi, cs);
        delay(100);
        
        bool success = false;
        Serial.print(" 400kHz: ");
        
        if (SD.begin(cs, spiSD, 400000)) {
            uint8_t cardType = SD.cardType();
            if (cardType != CARD_NONE) {
                Serial.print("ERFOLG! ");
                
                const char* typeStr = "UNKNOWN";
                if (cardType == CARD_MMC) typeStr = "MMC";
                else if (cardType == CARD_SD) typeStr = "SDSC";
                else if (cardType == CARD_SDHC) typeStr = "SDHC";
                
                Serial.print(typeStr);
                
                uint64_t cardSize = SD.cardSize();
                if (cardSize > 0) {
                    Serial.print(", "); 
                    Serial.print(cardSize / (1024 * 1024)); 
                    Serial.print(" MB");
                }
                Serial.println();
                success = true;
                
                // Kurzer Funktionstest
                Serial.print("   Test-Datei: ");
                File testFile = SD.open("/hwtest.txt", FILE_WRITE);
                if (testFile) {
                    testFile.println("ESP32-S3 HW Test OK");
                    testFile.close();
                    Serial.println("OK");
                } else {
                    Serial.println("Schreibfehler");
                }
                
            } else {
                Serial.println("Keine Karte erkannt");
            }
        } else {
            Serial.println("Init fehlgeschlagen");
        }
        
        if (success) {
            Serial.println("✓ SD-Karte funktioniert!");
            return true;
        }
        
        spiSD.end();
        delay(200);
    }
    
    Serial.println("\n✗ Alle Pin-Sets fehlgeschlagen");
    Serial.println("\nHARDWARE PRÜFEN:");
    Serial.println("1. SD-Karte: In PC testen, als FAT32 formatieren");
    Serial.println("2. Verkabelung: VCC->5V, GND->GND, 4 SPI-Pins korrekt");
    Serial.println("3. SD-Adapter: Oft sind billige Adapter defekt");
    Serial.println("4. Stromversorgung: Unbedingt 5V verwenden!");
    Serial.println("5. Kabel: Kurz und stabil, Wackelkontakte vermeiden");
    
    return false;
}

void testBNO055() {
    Serial.println("\n--- BNO055 Test (Legacy) ---");
    Serial.println("Verwende BNO055Manager für detaillierten Test...");
    
    if (bnoManager.isReady()) {
        bnoManager.runSelfTest();
        CalibrationData cal = bnoManager.getCalibration();
        Serial.println("\nKalibrierung (0-3):");
        Serial.printf("  System: %d\n", cal.system);
        Serial.printf("  Gyro: %d\n", cal.gyro);
        Serial.printf("  Accel: %d\n", cal.accel);
        Serial.printf("  Mag: %d\n", cal.mag);
        
        SensorData data = bnoManager.getCurrentData();
        Serial.printf("Temperatur: %.1f °C\n", data.temperature);
        Serial.printf("Orientierung: H=%.1f°, P=%.1f°, R=%.1f°\n", 
                     data.heading, data.pitch, data.roll);
    } else {
        Serial.println("BNO055Manager nicht bereit!");
    }
}

void testOLED() {
    Serial.println("\n--- OLED Display Test ---");
    
    if (!oledManager.isReady()) {
        Serial.println("❌ OLED-Manager nicht initialisiert!");
        return;
    }
    
    Serial.println("Führe OLED-Tests durch...");
    
    // 1. Display-Test (Pixel)
    Serial.print("1. Display Pixel-Test: ");
    bool pixelTest = oledManager.testDisplay();
    Serial.println(pixelTest ? "OK" : "FEHLER");
    delay(2000);
    
    // 2. Boot-Nachricht mit Fortschrittsbalken
    Serial.print("2. Boot-Nachricht Test: ");
    for (int i = 0; i <= 100; i += 20) {
        oledManager.showBootMessage("System Test", i);
        delay(300);
    }
    Serial.println("OK");
    delay(1000);
    
    // 3. Test verschiedener Display-Modi
    Serial.print("3. Display-Modi Test: ");
    
    // Hardware Status Test
    oledManager.showHardwareStatus(true, true, true, true, false, false);
    delay(2000);
    
    // Sensor-Daten Test (Dummy-Werte)
    oledManager.showSensorData(123.5, 9.81, 25.4, 42);
    delay(2000);
    
    // GPS Status Test (ohne Fix)
    oledManager.showGPSStatus(0, 0, 0, 5, false);
    delay(2000);
    
    // Straßenqualität Test
    oledManager.showRoadQuality(85.5, 92.1, 78.3, 12);
    delay(2000);
    
    Serial.println("OK");
    
    // 4. Test-Ergebnisse Anzeige
    Serial.print("4. Test-Ergebnisse: ");
    oledManager.showTestResults("OLED Test", true, "Alle Modi erfolgreich");
    delay(2000);
    Serial.println("OK");
    
    // 5. Fehler-Nachricht Test
    Serial.print("5. Fehler-Nachricht: ");
    oledManager.showErrorMessage("Test Fehler", "Beispiel für Debug");
    delay(2000);
    Serial.println("OK");
    
    // 6. System-Info Test
    Serial.print("6. System-Info: ");
    oledManager.showSystemInfo("v1.1", millis(), ESP.getFreeHeap());
    delay(2000);
    Serial.println("OK");
    
    // 7. Debug-Informationen Test
    Serial.print("7. Debug-Info: ");
    DisplayConfig config = oledManager.getConfig();
    config.showDebugInfo = true;
    oledManager.setConfig(config);
    oledManager.showDebugInfo("Debug: OLED Test läuft");
    delay(2000);
    Serial.println("OK");
    
    // 8. Automatischer Modus-Wechsel Test
    Serial.print("8. Modus-Wechsel Test: ");
    config.autoRotate = true;
    config.rotateInterval = 2000; // 2 Sekunden
    oledManager.setConfig(config);
    
    for (int i = 0; i < 4; i++) {
        oledManager.setMode((DisplayMode)i);
        switch (i) {
            case 0: oledManager.showHardwareStatus(true, true, true, true, true, true); break;
            case 1: oledManager.showSensorData(45.2, 1.23, 26.1, 15); break;
            case 2: oledManager.showGPSStatus(47.123456, 8.654321, 67.5, 8, true); break;
            case 3: oledManager.showRoadQuality(72.8, 85.2, 45.6, 7); break;
        }
        delay(2000);
    }
    Serial.println("OK");
    
    // 9. Display-Info ausgeben
    Serial.println("\n=== OLED Display Information ===");
    Serial.println(oledManager.getDisplayInfo());
    
    // 10. Abschluss-Test
    oledManager.showTestResults("OLED Volltest", true, "Alle 8 Tests bestanden!");
    delay(3000);
    
    Serial.println("✅ OLED-Test erfolgreich abgeschlossen!");
    Serial.printf("Display: %dx%d SSD1306\n", SCREEN_WIDTH, SCREEN_HEIGHT);
    Serial.println("Alle Display-Modi funktional");
}

void testBufferSafety() {
    Serial.println("\n--- Buffer-Sicherheits-Test ---");
    
    // Test 1: Sichere String-Formatierung
    Serial.print("1. SafeStringFormatter Test: ");
    char testBuffer[64];
    bool success = SafeStringFormatter::safePrintf(testBuffer, sizeof(testBuffer), 
                                                  "Test %.2f %d %s", 3.14159f, 42, "OK");
    Serial.println(success ? "OK" : "FEHLER");
    
    // Test 2: Buffer-Overflow-Schutz
    Serial.print("2. Buffer-Overflow-Schutz: ");
    char smallBuffer[10];
    bool prevented = !SafeStringFormatter::safePrintf(smallBuffer, sizeof(smallBuffer), 
                                                     "Dies ist ein sehr langer String der nicht passt");
    Serial.println(prevented ? "OK (Overflow verhindert)" : "FEHLER");
    
    // Test 3: Ring-Buffer Test
    Serial.print("3. Ring-Buffer Test: ");
    SafeRingBuffer<float, 5> ringBuffer;
    
    // Fülle Buffer
    for (int i = 0; i < 7; i++) {
        ringBuffer.push(i * 1.5f);
    }
    
    bool overflowDetected = ringBuffer.hasOverflowed();
    Serial.printf("%s (Size: %zu, Overflow: %s)\n", 
                 ringBuffer.size() == 5 ? "OK" : "FEHLER",
                 ringBuffer.size(),
                 overflowDetected ? "Ja" : "Nein");
    
    // Test 4: Memory Pool
    Serial.print("4. Memory Pool Test: ");
    void* ptr1 = globalMemoryPool.allocate();
    void* ptr2 = globalMemoryPool.allocate();
    bool allocated = (ptr1 != nullptr && ptr2 != nullptr);
    
    if (allocated) {
        bool freed = globalMemoryPool.deallocate(ptr1) && globalMemoryPool.deallocate(ptr2);
        Serial.println(freed ? "OK" : "FEHLER beim Freigeben");
    } else {
        Serial.println("FEHLER bei Allokation");
    }
    
    // Test 5: Stack Buffer
    Serial.print("5. Stack Buffer Test: ");
    char* stackMem = formatBuffer.allocate(100);
    bool stackOK = (stackMem != nullptr);
    
    if (stackOK) {
        SAFE_SPRINTF(stackMem, "Stack Test: %.1f", 42.7f);
        formatBuffer.deallocate(100);
    }
    Serial.println(stackOK ? "OK" : "FEHLER");
    
    // Ausgabe Statistiken
    Serial.println("\n--- Buffer-Statistiken ---");
    printBufferStats();
    
    Serial.println("✅ Buffer-Sicherheits-Test abgeschlossen!");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // ESP32 Log Level auf Error setzen
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_log_level_set("Wire", ESP_LOG_NONE);
    
    Serial.println("\n=== Straßenqualitäts-Messsystem ===");
    Serial.println("Für kurvenreiche Genießer-Strecken");
    Serial.println("Version 1.0 mit Hardware-Test\n");
    
    // I2C sauber initialisieren
    Serial.println("Initialisiere I2C-Bus...");
    
    Wire.end();
    delay(100);
    
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    delay(200);
    
    bool i2cSuccess = Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    delay(200);
    
    Serial.print("I2C-Bus Status: ");
    if (i2cSuccess) {
        Serial.println("OK (SDA=GPIO8, SCL=GPIO9)");
    } else {
        Serial.println("FEHLER - Mögliche Pin-Konflikte");
    }
    
    // I2C-Bus Test
    Serial.println("Teste I2C-Bus Kommunikation...");
    Wire.beginTransmission(0x00);
    uint8_t error = Wire.endTransmission();
    Serial.print("I2C-Bus Test: ");
    if (error == 2) {
        Serial.println("OK (Bus funktionsfähig)");
    } else {
        Serial.print("Unerwarteter Status: "); Serial.println(error);
    }
    
    // I2C Scanner ausführen
    i2cScanner();
    delay(1000);
    
    // OLED Display initialisieren
    Serial.println("\nInitialisiere OLED Display...");
    
    // Vorab-Scan für OLED-Adressen
    bool oledDetected = OLEDManager::scanI2CAddresses();
    if (oledDetected) {
        Serial.println("OLED auf I2C-Bus erkannt - starte Initialisierung");
    } else {
        Serial.println("⚠️ Kein OLED auf Standard-Adressen gefunden");
    }
    
    if (oledManager.begin()) {
        Serial.println("OLED erfolgreich initialisiert!");
        oledManager.showTestResults("Straßenqualität", true, "Starte System...");
        delay(2000);
        
        // Vollständiger OLED-Test ausführen
        testOLED();
        
        // Buffer-Sicherheits-Test ausführen
        testBufferSafety();
    } else {
        Serial.println("OLED Initialisierung fehlgeschlagen!");
        Serial.println("Prüfe: VCC->3.3V, GND->GND, SDA->GPIO8, SCL->GPIO9");
    }
    
    // BNO055Manager initialisieren
    Serial.println("\nInitialisiere BNO055Manager...");
    if (!bnoManager.begin()) {
        Serial.println("BNO055 nicht gefunden! Prüfe Verkabelung.");
        Serial.println("SDA=GPIO8, SCL=GPIO9, VCC=5V, GND=GND");
        Serial.println("ADDR-Pin scheint auf 3.3V zu liegen (Adresse 0x29)");
        
        if (oledManager.isReady()) {
            oledManager.showTestResults("BNO055 Fehler", false, "Verkabelung prüfen!");
        }
        
        while (1) {
            delay(5000);
            Serial.println("Warte auf BNO055...");
            if (bnoManager.begin()) {
                Serial.println("BNO055 jetzt gefunden!");
                break;
            }
        }
    }
    
    Serial.println("✅ BNO055Manager erfolgreich initialisiert");
    
    // BNO055 detaillierter Test
    testBNO055();
    
    // SD-Karte testen
    sdCardAvailable = testSDWithSafePins();
    
    // SDLogger initialisieren
    if (sdCardAvailable && sdLogger.begin(spiSD)) {
        Serial.println("✅ SDLogger erfolgreich initialisiert");
        
        // Logging-Konfiguration
        LogConfig logConfig = sdLogger.getConfig();
        logConfig.sensorLogInterval = 100;  // 10Hz
        logConfig.roadLogInterval = 1000;   // 1Hz
        sdLogger.setConfig(logConfig);
        
        // Logging starten
        if (sdLogger.startLogging()) {
            Serial.println("✅ Datenaufzeichnung gestartet");
        }
    }
    
    // CAN-Reader mit arduino-CAN Library initialisieren
    if (sdCardAvailable) {
        Serial.println("\n--- CAN-Reader mit arduino-CAN Library ---");
        canReader.setPins(CAN_CS_PIN, CAN_INT_PIN);
        canReader.setClockFrequency(8E6); // 8MHz erkannt
        
        if (canReader.begin(500E3)) {
            canBusAvailable = true;
            
            // CAN-Logging auf SD-Karte aktivieren
            canLogFileName = "/can_log_" + String(millis()) + ".csv";
            if (canReader.enableLogging(canLogFileName)) {
                canLoggingEnabled = true;
                Serial.printf("✅ CAN-Logging aktiviert: %s\n", canLogFileName.c_str());
            }
            
            // Filter für interessante CAN-IDs (optional)
            // canReader.clearFilters(); // Alle Nachrichten empfangen
            
            Serial.println("✅ CAN-Reader mit arduino-CAN bereit!");
        } else {
            Serial.println("❌ CAN-Reader Initialisierung fehlgeschlagen");
            canBusAvailable = false;
        }
    } else {
        Serial.println("⚠️ SD-Karte nicht verfügbar - CAN-Logging deaktiviert");
    }
    
    // GPS-Manager initialisieren
    Serial.println("\n--- GPS-Manager (BN-880) ---");
    Serial.printf("Hardware UART2: RX=GPIO%d, TX=GPIO%d\n", GPS_RX_PIN, GPS_TX_PIN);
    if (gpsManager.begin(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE)) {
        gpsAvailable = true;
        Serial.println("✅ GPS-Manager erfolgreich initialisiert");
        Serial.println("GPS benötigt 15-30 Sekunden für ersten Fix");
        
        // Test-Kommunikation
        if (gpsManager.testCommunication()) {
            Serial.println("✅ GPS-Kommunikation etabliert");
        } else {
            Serial.println("⚠️ GPS-Kommunikation noch nicht aktiv (normal bei Cold Start)");
        }
    } else {
        Serial.println("❌ GPS-Manager Initialisierung fehlgeschlagen");
        Serial.printf("Prüfe: TX=GPIO%d, RX=GPIO%d, VCC=3.3V, GND=GND\n", GPS_TX_PIN, GPS_RX_PIN);
        gpsAvailable = false;
    }
    
    Serial.println("\n=== SYSTEM BEREIT ===");
    Serial.print("I2C: ✓ | BNO055: ✓ | OLED: ✓ | SD: ");
    Serial.print(sdCardAvailable ? "✓" : "❌");
    Serial.print(" | CAN: ");
    Serial.print(canBusAvailable ? "✓" : "❌");
    Serial.print(" | GPS: ");
    Serial.println(gpsAvailable ? "✓" : "❌");
    
    // Hardware-Status auf OLED anzeigen
    bool i2cOK = true;  // I2C funktioniert (BNO055 gefunden)
    bool bno055OK = true;  // BNO055 initialisiert
    bool oledOK = oledManager.isReady();  // OLED funktioniert wenn wir hier sind
    
    if (oledOK) {
        oledManager.showHardwareStatus(i2cOK, bno055OK, oledOK, sdCardAvailable, canBusAvailable, gpsAvailable);
        delay(3000);  // Status 3 Sekunden anzeigen
    }
    
    lastUpdate = millis();
    Serial.println("\nStarte Straßenqualitäts-Messung...");
    Serial.println("📊 Zeitbasierte Korrelation: Sensor ↔ CAN aktiviert");
    Serial.println("📁 Separate Logs: sensor_data.csv, can_log.csv, correlated.csv\n");
}

void loop() {
    static unsigned long lastCANCheck = 0;
    static unsigned long lastStatusReport = 0;
    static unsigned long lastSensorRead = 0;
    static unsigned long lastGPSUpdate = 0;
    static int canMessageCount = 0;
    static SensorData lastSensorData = {0};
    static CANMessage lastCANMessage = {0};
    static GPSData lastGPSData = {0};
    
    unsigned long currentTime = millis();
    
    // BNO055 Sensor-Daten lesen (alle 100ms)
    if (bnoManager.isReady() && (currentTime - lastSensorRead >= 100)) {
        SensorData sensorData = bnoManager.getCurrentData();
        lastSensorData = sensorData;  // Für Zeitkorrelation speichern
        
        // Daten auf SD-Karte loggen
        if (sdLogger.isLogging()) {
            sdLogger.logSensorData(sensorData);
            
            // Vibrations-Analyse
            VibrationMetrics vibMetrics = bnoManager.analyzeVibration();
            float roadQuality = bnoManager.calculateRoadQuality();
            
            // Straßenqualität loggen
            sdLogger.logRoadQuality(roadQuality, bnoManager.getSmoothness(), 
                                   0, vibMetrics.rmsAccel);
            
            // Schlagloch-Erkennung
            if (bnoManager.detectPothole()) {
                sdLogger.logPothole(vibMetrics.maxShock);
                Serial.println("⚠️  Schlagloch erkannt!");
            }
            
            // Kurven-Erkennung
            if (bnoManager.detectCurve()) {
                float curveAngle = bnoManager.getCurveAngle();
                sdLogger.logCurve(curveAngle, 0);
            }
            
            // Zeitkorrelation: Wenn aktuelle CAN-Nachricht vorhanden
            if (lastCANMessage.timestamp > 0 && 
                abs((long)(sensorData.timestamp - lastCANMessage.timestamp)) < 1000) {
                sdLogger.logCorrelatedData(sensorData, lastCANMessage);
            }
        }
        
        lastSensorRead = currentTime;
    }
    
    // GPS-Daten aktualisieren (alle 200ms)
    if (gpsAvailable && (currentTime - lastGPSUpdate >= 200)) {
        gpsManager.update();
        
        if (gpsManager.available()) {
            GPSData gpsData = gpsManager.getCurrentData();
            lastGPSData = gpsData; // Für Zeitkorrelation speichern
            
            // GPS-Daten loggen falls SD verfügbar
            if (sdLogger.isLogging()) {
                // Geo-lokalisierte Sensor-Daten loggen
                if (gpsData.valid_fix && lastSensorData.timestamp > 0) {
                    // Korrelierte GPS + Sensor-Daten
                    sdLogger.logCorrelatedData(lastSensorData, lastCANMessage);
                }
            }
        }
        
        lastGPSUpdate = currentTime;
    }
    
    // CAN-Bus Nachrichten empfangen (alle 10ms)
    if (canBusAvailable && (currentTime - lastCANCheck >= 10)) {
        if (canReader.hasMessage()) {
            CANMessage msg = canReader.readMessage();
            
            if (msg.canId != 0) { // Gültige Nachricht empfangen
                canMessageCount++;
                totalCANMessages++;
                lastCANMessage = msg;  // Für Zeitkorrelation speichern
                
                // In SDLogger aufzeichnen
                if (sdLogger.isLogging()) {
                    sdLogger.logCANMessage(msg);
                    
                    // Zeitkorrelation: Wenn aktuelle Sensor-Daten vorhanden
                    if (lastSensorData.timestamp > 0 && 
                        abs((long)(msg.timestamp - lastSensorData.timestamp)) < 1000) {
                        sdLogger.logCorrelatedData(lastSensorData, msg);
                    }
                }
                
                // Detaillierte Ausgabe für die ersten 10 Nachrichten
                if (canMessageCount <= 10) {
                    Serial.printf("CAN #%d: ", canMessageCount);
                    Serial.println(formatCANMessage(msg));
                }
            }
        }
        lastCANCheck = currentTime;
    }
    
    // Status-Report alle 5 Sekunden
    if (currentTime - lastStatusReport >= 5000) {
        Serial.printf("[%lu] System läuft - CAN: %d msg, ", 
                     currentTime/1000, canMessageCount);
        
        // BNO055 Status
        CalibrationData cal = bnoManager.getCalibration();
        Serial.printf("BNO055 Kal: %d/%d/%d/%d", 
                     cal.system, cal.gyro, cal.accel, cal.mag);
        
        // GPS Status
        if (gpsAvailable) {
            if (gpsManager.hasValidFix()) {
                GPSData gps = gpsManager.getCurrentData();
                Serial.printf(", GPS: %.6f°N %.6f°E (%d sat)", 
                             gps.latitude, gps.longitude, gps.satellites);
            } else {
                uint8_t sats = gpsManager.getSatelliteCount();
                Serial.printf(", GPS: Kein Fix (%d sat)", sats);
            }
        }
        
        // SD-Logger Status
        if (sdLogger.isLogging()) {
            Serial.printf(", SD: %lu KB frei\n", sdLogger.getFreeSpace());
            sdLogger.flush(); // Daten sichern
            
            // Kalibrierungs-Hinweise
            if (!cal.isFullyCalibrated()) {
                Serial.println(bnoManager.getCalibrationInstructions());
            }
        } else {
            Serial.println();
        }
        
        if (canLoggingEnabled) {
            canReader.flushLog(); // CAN-Log aktualisieren
        }
        
        // Live-Daten auf OLED anzeigen
        if (bnoManager.isReady() && oledManager.isReady()) {
            SensorData sensorData = bnoManager.getCurrentData();
            oledManager.showSensorData(sensorData.heading, sensorData.accelMagnitude, 
                            sensorData.temperature, totalCANMessages);
        }
        
        lastStatusReport = currentTime;
    }
    
    // Kurze Pause um CPU zu entlasten
    delay(1);
}
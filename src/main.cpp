#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "hardware_config.h"
#include "buffer_utils.h"
#include "oled_manager.h"
#include "can_reader.h"
#include "bno055_manager.h"
#include "sd_logger.h"
#include "road_quality.h"
#include "gps_manager.h"
#include "integration_tests.h"
#include "web_manager.h"

// Pin-Definitionen sind nun in hardware_config.cpp zentralisiert

// Forward declaration für Buffer-Statistik Funktion
void printBufferStats();

// Hardware-Instanzen
SPIClass spiSD(HSPI);   // Separate SPI-Instanz für SD-Karte
// CAN-Reader verwendet die globale SPI-Instanz

// Status-Variablen
bool canBusAvailable = false;
bool sdCardAvailable = false;
bool sdCardWasAvailable = false;
bool gpsAvailable = false;
bool gpsClockSynced = false;
bool webAvailable = false;
bool canInitializationAttempted = false;
constexpr bool ENABLE_OPTIONAL_CAN = false;
unsigned long lastSDCheck = 0;
String currentLogFileName = "";
String canLogFileName = "";
File logFile;
bool canLoggingEnabled = false;
int totalCANMessages = 0;

// Nicht blockierende Hardware-Überwachung
constexpr unsigned long HARDWARE_CHECK_INTERVAL = 5000;
constexpr unsigned long GPS_RESTART_INTERVAL = 30000;
constexpr unsigned long BOOT_STATUS_INTERVAL = 1000;
constexpr unsigned long BOOT_READY_HOLD_TIME = 2500;
unsigned long lastHardwareCheck = 0;
unsigned long lastGPSRestart = 0;
unsigned long lastBootStatusDisplay = 0;
unsigned long requiredHardwareReadySince = 0;
uint8_t bnoMissingChecks = 0;
uint8_t oledMissingChecks = 0;
bool bootStatusActive = true;

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
    
    // Debug: Zeige die konfigurierten Pins aus hardware_config.h
    Serial.println("\nKonfigurierte SD-Pins aus hardware_config.h:");
    Serial.printf("SD_CS_PIN=%d, SD_MOSI_PIN=%d, SD_MISO_PIN=%d, SD_SCK_PIN=%d\n", 
                  SD_CS_PIN, SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN);
    
    // Sichere Pin-Sets für ESP32-S3
    int pinSets[][4] = {
        {SD_CS_PIN, SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN},  // Primär aus hardware_config.h
        {10, 11, 13, 12}, // CS, MOSI, MISO, SCK - Standard Backup
        {15, 16, 17, 18}  // CS, MOSI, MISO, SCK - Alternative Backup
    };
    const char* pinSetNames[] = {
        "Hardware Config Pins", 
        "Standard Backup (10,11,13,12)", 
        "Alternative Backup (15,16,17,18)"
    };
    
    Serial.println("\n=== Hardware-Checkliste ===");
    Serial.println("SD-Karten Modul benötigt:");
    Serial.println("• 3.3V Stromversorgung (wie auf dem PZSMOCN-Modul beschriftet)");
    Serial.println("• FAT32 formatierte SD-Karte");
    Serial.println("• Stabile Kabelverbindungen");
    Serial.println("• SD-Karte fest eingesteckt");
    Serial.println("\nAktuelle Verkabelung prüfen:");
    Serial.println("• 3.3V   -> 3.3V");
    Serial.println("• GND    -> GND");
    Serial.printf("• CS     -> GPIO %d\n", SD_CS_PIN);
    Serial.printf("• MOSI   -> GPIO %d\n", SD_MOSI_PIN);
    Serial.printf("• MISO   -> GPIO %d (ggf. 10kΩ Pull-up nach 3.3V!)\n", SD_MISO_PIN);
    Serial.printf("• SCK    -> GPIO %d\n", SD_SCK_PIN);
    Serial.println("\n⚠️  WICHTIG: Falls externe Pull-ups nötig:");
    Serial.println("    - NUR gegen 3.3V, NIEMALS gegen 5V!");
    Serial.println("    - ESP32 GPIOs sind NICHT 5V-tolerant!");
    
    // Nur das primäre Pin-Set testen, da SD-Karte fest an GPIO 4-7 angeschlossen ist
    for (int set = 0; set < 1; set++) {  // Nur Set 0 testen
        Serial.print("\n"); Serial.print(set + 1); Serial.print(". Teste "); 
        Serial.print(pinSetNames[set]); Serial.print("...");
        
        // SPI sauber beenden
        spiSD.end();
        delay(100);
        
        int cs = pinSets[set][0];
        int mosi = pinSets[set][1];
        int miso = pinSets[set][2];
        int sck = pinSets[set][3];
        
        Serial.printf("   Pins: CS=%d, MOSI=%d, MISO=%d, SCK=%d\n", cs, mosi, miso, sck);
        
        // Sichere Pin-Konfiguration
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
        pinMode(mosi, OUTPUT);
        pinMode(sck, OUTPUT);
        pinMode(miso, INPUT_PULLUP);
        
        // Zusätzliche Pull-ups für Stabilität
        pinMode(mosi, INPUT_PULLUP);
        digitalWrite(mosi, HIGH);
        pinMode(sck, INPUT_PULLUP);
        digitalWrite(sck, LOW);
        
        delay(50);
        
        // Debug: Pin-Status vor SPI-Init
        Serial.printf("   Pin-Status vor SPI: CS=%d, MISO=%d\n", 
                      digitalRead(cs), digitalRead(miso));
        
        // SPI mit sicheren Parametern starten
        spiSD.begin(sck, miso, mosi, cs);
        spiSD.setFrequency(400000);  // Explizit niedrige Frequenz
        spiSD.setDataMode(SPI_MODE0); // SD-Karten verwenden Mode 0
        delay(100);
        
        // Debug: Pin-Status nach SPI-Init
        Serial.printf("   Pin-Status nach SPI: CS=%d, MISO=%d\n", 
                      digitalRead(cs), digitalRead(miso));
        
        // Teste SPI-Kommunikation direkt
        Serial.println("   Teste SPI-Kommunikation...");
        
        // Sende 80 Clock-Zyklen mit CS=HIGH zur Initialisierung
        digitalWrite(cs, HIGH);
        for (int i = 0; i < 10; i++) {
            spiSD.transfer(0xFF);
        }
        
        digitalWrite(cs, LOW);
        delay(1);
        
        // Sende CMD0 (GO_IDLE_STATE) - sollte 0x01 zurückgeben
        spiSD.transfer(0x40);  // CMD0
        spiSD.transfer(0x00);
        spiSD.transfer(0x00);
        spiSD.transfer(0x00);
        spiSD.transfer(0x00);
        spiSD.transfer(0x95);  // CRC für CMD0
        
        delay(1);
        
        // Lese Antwort (max 10 Versuche)
        uint8_t response = 0xFF;
        for (int i = 0; i < 10; i++) {
            response = spiSD.transfer(0xFF);
            if (response != 0xFF) break;
            delay(1);
        }
        
        digitalWrite(cs, HIGH);
        Serial.printf("   SPI-Antwort auf CMD0: 0x%02X (erwartet: 0x01)\n", response);
        
        // Wenn keine Antwort, Problem mit Hardware
        if (response == 0xFF) {
            Serial.println("   ⚠️  KEINE SPI-Antwort! Mögliche Ursachen:");
            Serial.println("      - SD-Karte nicht eingesteckt");
            Serial.println("      - MISO-Leitung nicht verbunden");
            Serial.println("      - Defektes SD-Modul");
            Serial.println("      - Fehlende Pull-up Widerstände (10kΩ an MISO gegen 3.3V!)");
            Serial.println("      - WICHTIG: Pull-ups IMMER gegen 3.3V, NICHT 5V!");
            
            // Zusätzliches Debugging für primäres Pin-Set
            if (set == 0) {
                Serial.println("\n   Erweiterte Diagnose für GPIO 4-7:");
                
                // Teste ob MISO überhaupt reagiert
                digitalWrite(cs, LOW);
                Serial.print("   MISO-Test: ");
                bool misoChanges = false;
                for (int i = 0; i < 20; i++) {
                    spiSD.transfer(0xFF);
                    int misoState = digitalRead(miso);
                    Serial.print(misoState);
                    if (i > 0 && misoState != 1) misoChanges = true;
                }
                digitalWrite(cs, HIGH);
                Serial.println(misoChanges ? " (MISO reagiert)" : " (MISO tot - Kabel prüfen!)");
                
                // Teste alle Pins einzeln
                Serial.println("   Pin-Kontinuität:");
                Serial.printf("   - CS (GPIO %d): %s\n", cs, digitalRead(cs) ? "HIGH" : "LOW");
                Serial.printf("   - MOSI (GPIO %d): Ausgang\n", mosi);
                Serial.printf("   - MISO (GPIO %d): %s\n", miso, digitalRead(miso) ? "HIGH" : "LOW");
                Serial.printf("   - SCK (GPIO %d): Ausgang\n", sck);
                
                // CS-Toggle-Test
                Serial.print("   CS-Toggle-Test: ");
                digitalWrite(cs, LOW);
                delay(1);
                Serial.print(digitalRead(cs) == LOW ? "LOW-OK " : "LOW-FEHLER ");
                digitalWrite(cs, HIGH);
                delay(1);
                Serial.println(digitalRead(cs) == HIGH ? "HIGH-OK" : "HIGH-FEHLER");
                
                // Empfehlung
                Serial.println("\n   ⚡ SOFORT-MAßNAHMEN:");
                Serial.println("   1. MISO-Kabel (GPIO 6 -> SD MISO) prüfen!");
                Serial.println("   2. SD-Karte herausnehmen und wieder einsetzen");
                Serial.println("   3. Mit Multimeter Durchgang MISO prüfen");
                Serial.println("   4. 10kΩ Pull-up von GPIO 6 nach 3.3V löten");
                Serial.println("   5. Anderes SD-Modul testen");
            }
        }
        
        bool success = false;
        Serial.print("   Teste SD.begin() mit 400kHz... ");
        
        // Mehrere Versuche mit verschiedenen Frequenzen
        uint32_t frequencies[] = {400000, 1000000, 4000000};
        const char* freqNames[] = {"400kHz", "1MHz", "4MHz"};
        
        for (int f = 0; f < 3; f++) {
            if (f > 0) Serial.printf("\n   Versuche %s... ", freqNames[f]);
            
            if (SD.begin(cs, spiSD, frequencies[f])) {
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
                
                break; // Erfolg, keine weiteren Frequenzen testen
            } else {
                Serial.println("Keine Karte erkannt");
            }
        } else {
            if (f == 0) {
                Serial.println("Init fehlgeschlagen");
            } else {
                Serial.println("Fehlgeschlagen");
            }
        }
        
        if (success) break; // Erfolg bei dieser Frequenz
        }
        
        if (success) {
            Serial.println("✓ SD-Karte funktioniert!");
            // SD-Karte ordnungsgemäß beenden für spätere Neuinitialisierung
            SD.end();
            spiSD.end();
            delay(100);
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
        Serial.println("\nIMUPLUS-Kalibrierung (0-3):");
        Serial.printf("  Gyro: %d\n", cal.gyro);
        Serial.printf("  Accel: %d\n", cal.accel);
        
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
    
    // Buffer-Statistiken manuell ausgeben (printBufferStats aus buffer_utils.cpp)
    Serial.println("=== Buffer Statistics ===");
    
    // Memory Pool Stats
    globalMemoryPool.printStats();
    
    // Format Buffer Stats  
    Serial.printf("Format Buffer: %zu/%zu bytes used (%.1f%%)\n",
                 formatBuffer.used(), 1024, 
                 (float)formatBuffer.used() / 1024.0f * 100.0f);
                 
    if (formatBuffer.hasOverflowed()) {
        Serial.println("⚠️ Format Buffer overflow detected!");
    }
    
    // ESP32 Heap Stats
    Serial.printf("ESP32 Heap: %lu Bytes frei, groesster Block %lu Bytes\n",
                 (unsigned long)ESP.getFreeHeap(),
                 (unsigned long)ESP.getMaxAllocHeap());
                 
    Serial.println("========================");
    
    Serial.println("✅ Buffer-Sicherheits-Test abgeschlossen!");
}

bool initializeGPSAndClock() {
    Serial.println("\n--- GPS-Manager (BN-880) ---");
    Serial.printf("Hardware UART2: RX=GPIO%d, TX=GPIO%d\n", GPS_RX_PIN, GPS_TX_PIN);

    if (!gpsManager.begin(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE)) {
        Serial.println("❌ GPS-Manager Initialisierung fehlgeschlagen");
        Serial.printf("Prüfe: GPS-TX->GPIO%d, GPS-RX->GPIO%d und gemeinsame Masse\n",
                      GPS_RX_PIN, GPS_TX_PIN);
        return false;
    }

    gpsManager.enableInterruptMode(true);
    lastGPSRestart = millis();
    Serial.println("✅ GPS läuft; Fix und UTC-Zeit werden im Hintergrund gesucht");
    return true;
}

bool i2cDeviceResponds(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

// Alle SPI-Slaves abwählen, bevor irgendein Bus initialisiert wird.
// Ein floatender CS kann ein Modul zufällig selektieren; beim CAN-Modul legt
// das im ungünstigen Fall dessen MISO-Ausgang auf GPIO 11 - bei einem mit 5 V
// versorgten Modul mit entsprechendem Pegel. Deshalb geschieht das
// unabhängig davon, ob CAN überhaupt aktiviert ist.
void deselectAllSPIDevices() {
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(CAN_CS_PIN, OUTPUT);
    digitalWrite(CAN_CS_PIN, HIGH);
}

// Hängt ein Slave den Bus fest (typisch nach einem Reset mitten in einer
// Übertragung), taktet diese Sequenz ihn frei und erzeugt eine Stop-Bedingung.
void recoverI2CBus() {
    Wire.end();

    pinMode(I2C_SCL, OUTPUT);
    pinMode(I2C_SDA, INPUT_PULLUP);
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL, LOW);
        delayMicroseconds(5);
    }

    pinMode(I2C_SDA, OUTPUT);
    digitalWrite(I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH);
    delayMicroseconds(5);

    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_CLOCK_SPEED);
}

// Die Testsuiten laufen minutenlang blockierend und warten teilweise auf
// Eingaben an der seriellen Konsole. Währenddessen darf der Watchdog nicht
// zuschlagen.
void suspendWatchdog() {
    esp_task_wdt_delete(nullptr);
}

void resumeWatchdog() {
    esp_task_wdt_add(nullptr);
    esp_task_wdt_reset();
}

void configureSDLogger() {
    LogConfig logConfig = sdLogger.getConfig();
    logConfig.sensorLogInterval = 100;
    logConfig.roadLogInterval = 1000;
    logConfig.gpsLogInterval = 200;
    sdLogger.setConfig(logConfig);
}

bool tryInitializeSDLogger() {
    if (sdLogger.isReady()) {
        return true;
    }

    SD.end();
    spiSD.end();
    spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!sdLogger.begin(spiSD)) {
        sdCardAvailable = false;
        return false;
    }

    configureSDLogger();
    sdCardAvailable = true;
    Serial.println("✅ SD-Logger bereit");
    return true;
}

bool initializeOptionalCAN() {
    SPI.end();
    SPI.begin(CAN_SCK_PIN, CAN_MISO_PIN, CAN_MOSI_PIN);
    SPI.setFrequency(1000000);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    canReader.setPins(CAN_CS_PIN, CAN_INT_PIN);
    // Der verbaute Joy-IT SBC-CAN01 hat einen 16-MHz-Quarz (Aufdruck 16.000).
    canReader.setClockFrequency(CAN_CLOCK_16MHZ);

    bool available = canReader.begin(CAN_BAUDRATE);
    if (available) {
        Serial.println("✅ Optionales CAN-Modul bereit");
    } else {
        Serial.println("ℹ️ CAN nicht verbunden (optional)");
    }
    return available;
}

bool requiredHardwareReady() {
    return oledManager.isReady() && bnoManager.isSelfTestPassed() &&
           bnoManager.isFusionModeActive() &&
           sdLogger.isReady() && gpsManager.isReceivingNMEA() &&
           webAvailable;
}

void updateBootStatusDisplay(unsigned long now, bool force = false) {
    bool allReady = requiredHardwareReady();
    if (!allReady) {
        bootStatusActive = true;
        requiredHardwareReadySince = 0;
    } else if (requiredHardwareReadySince == 0) {
        requiredHardwareReadySince = now;
    }

    if (oledManager.isReady() &&
        (force || (bootStatusActive &&
                   now - lastBootStatusDisplay >= BOOT_STATUS_INTERVAL))) {
        oledManager.showBootStatus(
            bnoManager.isSelfTestPassed() && bnoManager.isFusionModeActive(),
            sdLogger.isReady(),
            gpsManager.isReady(), gpsManager.isReceivingNMEA(),
            canBusAvailable, webAvailable, allReady);
        lastBootStatusDisplay = now;
    }

    if (allReady && bootStatusActive &&
        now - requiredHardwareReadySince >= BOOT_READY_HOLD_TIME) {
        bootStatusActive = false;
    }
}

void handleHardwareRecovery(unsigned long now) {
    if (now - lastHardwareCheck < HARDWARE_CHECK_INTERVAL) {
        return;
    }
    lastHardwareCheck = now;

    // BNO055 und OLED werden per I2C-Ping überwacht. Erst drei
    // aufeinanderfolgende Ausfälle gelten als echte Trennung.
    bool bnoResponding = bnoManager.responds();
    if (bnoManager.isReady()) {
        bnoMissingChecks = bnoResponding ? 0 : bnoMissingChecks + 1;
        if (bnoMissingChecks >= 3) {
            // Vor dem Abschalten des Sensors erst den Bus freitakten. Ein
            // festhängender Slave ist die häufigere Ursache als ein wirklich
            // abgezogenes Modul, und jeder Sensor-Neustart verwirft die
            // laufende Kalibrierung vollständig.
            Serial.println("⚠️ BNO055 antwortet nicht; I²C-Bus wird zurückgesetzt");
            recoverI2CBus();
            bnoMissingChecks = 0;
            if (bnoManager.responds()) {
                Serial.println("✅ BNO055 nach I²C-Reset wieder erreichbar");
            } else {
                Serial.println("⚠️ BNO055 weiterhin nicht erreichbar");
                bnoManager.end();
            }
        } else if (bnoResponding && !bnoManager.isSelfTestPassed()) {
            bnoManager.runSelfTest();
        } else if (bnoResponding && !bnoManager.verifyFusionMode()) {
            Serial.println("⚠️ BNO055-Fusion dreimal fehlerhaft; vollständiger Neustart");
            bnoManager.restartFusion();
        }
    } else if (bnoResponding && bnoManager.begin()) {
        if (bnoManager.runSelfTest()) {
            Serial.println("✅ BNO055 im Hintergrund wieder verbunden");
        } else {
            Serial.println("⚠️ BNO055 verbunden, Selbsttest noch fehlerhaft");
        }
    }

    bool oledResponding =
        i2cDeviceResponds(OLED_ADDRESS_A) ||
        i2cDeviceResponds(OLED_ADDRESS_B);
    if (oledManager.isReady()) {
        oledMissingChecks = oledResponding ? 0 : oledMissingChecks + 1;
        if (oledMissingChecks >= 3) {
            Serial.println("⚠️ OLED nicht mehr erreichbar");
            oledManager.end(false);
            oledMissingChecks = 0;
        }
    } else if (oledResponding && oledManager.begin(OLED_ADDRESS_A)) {
        oledManager.setRotation(true);
        bootStatusActive = true;
        Serial.println("✅ OLED im Hintergrund wieder verbunden");
    }

    if (sdLogger.isReady()) {
        sdCardAvailable = sdLogger.checkHealth();
    }
    if (!sdLogger.isReady()) {
        tryInitializeSDLogger();
    }

    if (!gpsManager.isReady()) {
        gpsAvailable = initializeGPSAndClock();
    } else {
        gpsAvailable = true;
        if (!gpsManager.isReceivingNMEA(15000) &&
            now - lastGPSRestart >= GPS_RESTART_INTERVAL) {
            Serial.println("⚠️ Keine GPS-NMEA-Daten; UART wird neu gestartet");
            gpsManager.end();
            gpsAvailable = initializeGPSAndClock();
        }
    }

    webAvailable = webManager.isReady();
    if (!webAvailable) {
        webAvailable = webManager.begin();
    }

    // CAN ist optional und wird erst nach dem vollständigen Systemstart
    // genau einmal geprüft. So kann es den WLAN-Start niemals verzögern.
    if (ENABLE_OPTIONAL_CAN && !canInitializationAttempted) {
        canInitializationAttempted = true;
        canBusAvailable = initializeOptionalCAN();
    }
}

void setup() {
    // Als Allererstes, noch vor jeder Businitialisierung: kein SPI-Slave darf
    // mit floatendem Chip Select im System hängen.
    deselectAllSPIDevices();

    Serial.begin(115200);
    delay(1000);

    // ESP32 Log Level auf Error setzen
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_log_level_set("Wire", ESP_LOG_NONE);

    Serial.println("\n=== Straßenqualitäts-Messsystem ===");
    Serial.println("Für kurvenreiche Genießer-Strecken");
    Serial.println("Version 1.5.10 mit entprellter BNO055-IMUPLUS-Prüfung\n");

    // Task-Watchdog aktivieren. Bleibt loop() hängen - etwa durch einen
    // abgebrochenen OTA-Upload oder einen blockierenden Peripheriezugriff -
    // startet das Gerät neu, statt auf der Strecke stumm zu bleiben.
    esp_task_wdt_config_t wdtConfig = {};
    wdtConfig.timeout_ms = WDT_TIMEOUT_MS;
    wdtConfig.idle_core_mask = 0;
    wdtConfig.trigger_panic = true;
    // Der Arduino-Kern initialisiert den Task-Watchdog bereits selbst. Der
    // zweite Aufruf ist deshalb erwartbar und wird unten sauber abgefangen -
    // die IDF protokolliert ihn aber als Fehler. Die Meldung sieht im Bootlog
    // wie ein echter Defekt aus und wird daher für diesen Aufruf unterdrückt.
    esp_log_level_set("task_wdt", ESP_LOG_NONE);
    if (esp_task_wdt_init(&wdtConfig) == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_reconfigure(&wdtConfig);
    }
    esp_log_level_set("task_wdt", ESP_LOG_ERROR);
    esp_task_wdt_add(nullptr);
    Serial.printf("Watchdog aktiv (%d ms)\n", WDT_TIMEOUT_MS);

    // Das Service-WLAN steht vor allen externen Hardwareprüfungen bereit.
    // Selbst ein langsames oder defektes Zusatzmodul verhindert so nicht,
    // dass Diagnose und OTA erreichbar werden.
    webAvailable = webManager.begin();

    // I2C-Bus einmal sauber aufsetzen.
    Wire.end();
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    bool i2cSuccess = Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_CLOCK_SPEED);
    Serial.printf("I2C: %s (SDA=%d, SCL=%d, %d Hz)\n",
                  i2cSuccess ? "OK" : "FEHLER", I2C_SDA, I2C_SCL, I2C_CLOCK_SPEED);

    // Ab hier wird immer dieselbe OLED-Statusseite aktualisiert.
    if (oledManager.begin(OLED_ADDRESS_A)) {
        oledManager.setRotation(true);
        Serial.println("✅ OLED geprüft");
    } else {
        Serial.println("⚠️ OLED fehlt; Wiederholungsprüfung läuft im Hintergrund");
    }
    updateBootStatusDisplay(millis(), true);

    if (bnoManager.begin()) {
        bool selfTestOK = bnoManager.runSelfTest();
        Serial.printf("BNO055 Selbsttest: %s\n", selfTestOK ? "OK" : "FEHLER");
    } else {
        Serial.println("⚠️ BNO055 fehlt; Firmware startet trotzdem");
    }
    updateBootStatusDisplay(millis(), true);

    if (bnoManager.isSelfTestPassed()) {
        CalibrationData startupCalibration = bnoManager.getCalibration();
        if (!startupCalibration.isFullyCalibrated()) {
            Serial.println("⚠️ BNO055-Kalibrierung noch nicht vollständig");
        }
    }

    gpsAvailable = initializeGPSAndClock();
    updateBootStatusDisplay(millis(), true);

    if (!tryInitializeSDLogger()) {
        Serial.println("⚠️ SD-Karte fehlt; Einstecken wird automatisch erkannt");
    }
    updateBootStatusDisplay(millis(), true);

    Serial.println("ℹ️ CAN ist deaktiviert; es kann WLAN und Webseite nicht blockieren");
    updateBootStatusDisplay(millis(), true);

    lastHardwareCheck = millis();
    lastUpdate = millis();
    Serial.println("\nBootprüfung läuft nicht blockierend weiter.");
    Serial.println("Webseite bleibt erreichbar; CAN ist für Messfahrten nicht erforderlich.");
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
    static CalibrationData lastDisplayedCalibration = {255, 255, 255, 255};
    static bool lastDisplayedCalibrationSaved = false;
    static unsigned long lastCalibrationDisplay = 0;
    static unsigned long lastCalibrationSaveAttempt = 0;

    esp_task_wdt_reset();

    // Web-Anfragen und OTA zuerst bedienen. Während eines Firmware-Uploads
    // bleibt die SD-Karte geschlossen und die übrige Messschleife pausiert.
    webManager.handleClient();
    if (webManager.isOTAInProgress()) {
        delay(1);
        return;
    }
    
    unsigned long currentTime = millis();
    handleHardwareRecovery(currentTime);
    gpsAvailable = gpsManager.isReady();
    
    // BNO055 Sensor-Daten lesen (alle 100ms)
    if (bnoManager.isSelfTestPassed() &&
        bnoManager.isFusionModeActive() &&
        (currentTime - lastSensorRead >= 100)) {
        SensorData sensorData = bnoManager.getCurrentData();
        lastSensorData = sensorData;  // Für Zeitkorrelation speichern
        bnoManager.processSample(sensorData);

        // Alle Auswertungen verwenden exakt denselben 10-Hz-Sensorwert.
        VibrationMetrics vibMetrics = bnoManager.analyzeVibration();
        float speedKmh = lastGPSData.valid_fix ? lastGPSData.speed_kmh : -1.0f;
        float roadQuality = bnoManager.calculateRoadQuality(speedKmh);
        bool potholeDetected = bnoManager.detectPothole(sensorData);
        bool curveCompleted = bnoManager.detectCurve(sensorData);

        // Daten auf SD-Karte loggen
        if (sdLogger.isLogging()) {
            sdLogger.logSensorData(sensorData);

            // Straßenqualität loggen
            sdLogger.logRoadQuality(roadQuality, bnoManager.getSmoothness(), 
                                   0, vibMetrics.rmsAccel);

            // Schlagloch-Erkennung
            if (potholeDetected) {
                sdLogger.logPothole(
                    vibMetrics.maxShock,
                    lastGPSData.valid_fix ? lastGPSData.latitude : 0.0f,
                    lastGPSData.valid_fix ? lastGPSData.longitude : 0.0f);
            }

            // Kurven-Erkennung
            if (curveCompleted) {
                float curveAngle = bnoManager.getCurveAngle();
                sdLogger.logCurve(
                    curveAngle, 0,
                    lastGPSData.valid_fix ? lastGPSData.latitude : 0.0f,
                    lastGPSData.valid_fix ? lastGPSData.longitude : 0.0f);
            }
        }

        if (potholeDetected) {
            Serial.println("⚠️  Schlagloch erkannt!");
        }
        if (curveCompleted) {
            Serial.printf("↪️  Kurve abgeschlossen: %.1f°\n",
                          bnoManager.getCurveAngle());
        }
        
        lastSensorRead = currentTime;
    }
    
    // GPS-Daten verarbeiten (alle 200ms)
    // Im Interrupt-Modus verarbeitet update() die gepufferten Daten
    if (gpsAvailable && (currentTime - lastGPSUpdate >= 200)) {
        gpsManager.update();  // Verarbeitet Interrupt-Buffer

        if (!gpsClockSynced && gpsManager.hasValidDateTime()) {
            gpsClockSynced = gpsManager.syncSystemClock();
            if (gpsClockSynced) {
                Serial.println("✅ Systemzeit nachträglich aus GPS synchronisiert");
            }
        }
        
        if (gpsManager.available()) {
            GPSData gpsData = gpsManager.getCurrentData();
            lastGPSData = gpsData; // Für Zeitkorrelation speichern
            
            // GPS-Daten loggen falls SD verfügbar
            if (sdLogger.isLogging()) {
                sdLogger.logGPSData(gpsData);
            }
        }
        
        lastGPSUpdate = currentTime;
    }

    if (!gpsManager.isCommunicating()) {
        // Keine alte Position oder Geschwindigkeit weiterverwenden, wenn
        // die GPS-Datenquelle ausgefallen ist.
        lastGPSData = {};
    }

    // Die gemeinsame Prüfseite bleibt aktiv, bis alle erforderlichen
    // Komponenten reagieren. Nach einem späteren Ausfall erscheint sie
    // automatisch erneut.
    updateBootStatusDisplay(currentTime);
    
    // CAN-Bus Nachrichten empfangen (alle 10ms)
    if (canBusAvailable && (currentTime - lastCANCheck >= 10)) {
        // hasMessage() puffert den Frame bereits; readMessage() liefert danach
        // genau diesen zurück. Eine zusätzliche Prüfung auf canId != 0 wäre
        // falsch, weil 0x000 eine gültige CAN-ID mit höchster Priorität ist.
        if (canReader.hasMessage()) {
            CANMessage msg = canReader.readMessage();

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
        lastCANCheck = currentTime;
    }
    
    // Status-Report alle 5 Sekunden
    if (currentTime - lastStatusReport >= 5000) {
        Serial.printf("[%lu] System läuft - CAN: %d msg, ", 
                     currentTime/1000, canMessageCount);
        
        // BNO055 Status
        CalibrationData cal = {0, 0, 0, 0};
        if (bnoManager.isSelfTestPassed() &&
            bnoManager.isFusionModeActive()) {
            cal = bnoManager.getCalibration();
            BNO055RuntimeStatus bnoStatus = bnoManager.getRuntimeStatus();
            Serial.printf("BNO055 %s/Sys:%d Kal:G%d/A%d/M%d",
                         ROADTEST_BNO_MODE_NAME, bnoStatus.systemStatus,
                         cal.gyro, cal.accel, cal.mag);
        } else {
            Serial.print("BNO055: Fusion nicht bereit");
        }

        // Eine vollständige Kalibrierung sofort dauerhaft sichern. Ohne das
        // geht sie beim nächsten Sensor-Neustart verloren, und genau solche
        // Neustarts löst die Hardware-Überwachung bei unruhigem I²C-Bus
        // regelmäßig aus. Der Abstand begrenzt NVS-Schreibzyklen, falls das
        // Speichern wiederholt fehlschlägt.
        if (cal.isFullyCalibrated() && !bnoManager.isCalibrationSaved() &&
            currentTime - lastCalibrationSaveAttempt >= 30000) {
            lastCalibrationSaveAttempt = currentTime;
            if (bnoManager.saveCalibration()) {
                Serial.println("✅ Kalibrierung automatisch dauerhaft gesichert");
            }
        }


        // GPS Status
        if (gpsAvailable && gpsManager.isReceivingNMEA()) {
            if (gpsManager.hasValidFix()) {
                GPSData gps = gpsManager.getCurrentData();
                Serial.printf(", GPS: %.6f°N %.6f°E (%d sat)", 
                             gps.latitude, gps.longitude, gps.satellites);
            } else {
                uint8_t sats = gpsManager.getSatelliteCount();
                Serial.printf(", GPS: Kein Fix (%d sat)", sats);
            }
        } else {
            Serial.print(", GPS: keine NMEA-Daten");
        }
        
        // SD-Logger Status
        if (sdLogger.isLogging()) {
            Serial.printf(", SD: %lu KB frei\n", sdLogger.getFreeSpace());
            sdLogger.flush(); // Daten sichern
            
            // Kalibrierungs-Hinweise
            if (!cal.isFullyCalibrated()) {
                Serial.println(bnoManager.getCalibrationInstructions());
            }
        } else if (sdLogger.isReady()) {
            Serial.println(", SD: bereit, Aufzeichnung gestoppt");
        } else {
            Serial.println(", SD: nicht bereit");
        }
        
        if (canLoggingEnabled) {
            canReader.flushLog(); // CAN-Log aktualisieren
        }
        
        // Bei Änderungen oder länger unvollständiger Kalibrierung zeigt das
        // OLED den Assistenten, ansonsten die Live-Daten.
        if (!bootStatusActive && bnoManager.isSelfTestPassed() &&
            oledManager.isReady()) {
            bool calibrationChanged =
                cal.gyro != lastDisplayedCalibration.gyro ||
                cal.accel != lastDisplayedCalibration.accel ||
                bnoManager.isCalibrationSaved() != lastDisplayedCalibrationSaved;
            bool calibrationReminder =
                !cal.isFullyCalibrated() &&
                currentTime - lastCalibrationDisplay >= 30000;

            if (calibrationChanged || calibrationReminder) {
                oledManager.showCalibrationStatus(
                    cal.gyro, cal.accel, cal.mag,
                    bnoManager.isCalibrationSaved());
                lastDisplayedCalibration = cal;
                lastDisplayedCalibrationSaved = bnoManager.isCalibrationSaved();
                lastCalibrationDisplay = currentTime;
            } else if (lastSensorData.timestamp > 0) {
                oledManager.showSensorData(
                    lastSensorData.heading, lastSensorData.accelMagnitude,
                    lastSensorData.temperature, totalCANMessages);
            }
        }
        
        lastStatusReport = currentTime;
    }
    
    // Serial-Kommandos für Tests
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "test") {
            Serial.println("\n=== Test-Kommandos ===");
            Serial.println("hardware - Hardware-Test-Suite");
            Serial.println("integration - Vollständige Integration-Tests");
            Serial.println("stress - Stress-Test-Suite");
            Serial.println("recovery - Failure-Recovery-Tests");
            Serial.println("quick - Schnelle Integration-Tests");
            Serial.println("buffer - Buffer-Sicherheits-Test");
            Serial.println("memory - Memory-Leak-Test (2 Min)");
            Serial.println("calibration - BNO055 Kalibrierung speichern");
            Serial.println("clear_cal - BNO055 Kalibrierung löschen");
            Serial.println("start - Messfahrt starten");
            Serial.println("stop - Messfahrt sicher beenden");
            Serial.println("gps_mode - GPS Interrupt/Polling umschalten");
            Serial.println("diag - System-Diagnose");
        }
        else if (command == "hardware") {
            suspendWatchdog();
            i2cScanner();
            testBNO055();
            testOLED();
            testSDWithSafePins();
            testBufferSafety();
            resumeWatchdog();
        }
        else if (command == "integration") {
            Serial.println("\n🚀 Starte umfassende Integration-Tests...");
            Serial.println("Dies dauert etwa 5-10 Minuten.");
            Serial.println("Achtung: einzelne Tests warten auf Eingaben an dieser Konsole.");
            suspendWatchdog();
            integrationTests.runAllTests();
            resumeWatchdog();
        }
        else if (command == "stress") {
            suspendWatchdog();
            runStressTestSuite();
            resumeWatchdog();
        }
        else if (command == "recovery") {
            suspendWatchdog();
            runFailureRecoveryTests();
            resumeWatchdog();
        }
        else if (command == "quick") {
            Serial.println("\n⚡ Starte schnelle Integration-Tests...");
            suspendWatchdog();
            integrationTests.testAllModulesConcurrent();
            integrationTests.testSensorDataCorrelation();
            integrationTests.testBufferOverflowRecovery();
            integrationTests.printResults();
            resumeWatchdog();
        }
        else if (command == "buffer") {
            testBufferSafety();
        }
        else if (command == "memory") {
            suspendWatchdog();
            integrationTests.testMemoryLeakDetection();
            resumeWatchdog();
        }
        else if (command == "calibration") {
            if (bnoManager.saveCalibration()) {
                Serial.println("✅ Kalibrierung gespeichert!");
            } else {
                Serial.println("❌ Kalibrierung konnte nicht gespeichert werden!");
            }
        }
        else if (command == "clear_cal") {
            if (bnoManager.clearCalibration()) {
                Serial.println("✅ Kalibrierung gelöscht!");
            }
        }
        else if (command == "start") {
            if (sdLogger.startLogging()) {
                Serial.println("✅ Messfahrt gestartet");
            } else {
                Serial.println("❌ Messfahrt konnte nicht gestartet werden");
            }
        }
        else if (command == "stop") {
            sdLogger.stopLogging();
        }
        else if (command == "gps_mode") {
            bool currentMode = gpsManager.isInterruptModeEnabled();
            gpsManager.enableInterruptMode(!currentMode);
            Serial.printf("GPS-Modus: %s\n", !currentMode ? "Interrupt" : "Polling");
        }
        else if (command == "diag") {
            Serial.println("\n=== System-Diagnose ===");
            Serial.printf("Free Heap: %lu Bytes\n",
                          (unsigned long)ESP.getFreeHeap());
            Serial.printf("Heap Size: %lu Bytes\n",
                          (unsigned long)ESP.getHeapSize());
            Serial.printf("Min Free Heap: %lu Bytes\n",
                          (unsigned long)ESP.getMinFreeHeap());

            // Steckbrief des Controllers. Das Board ist verlötet und trägt
            // keine auswertbare Typbezeichnung; für die Wahl des richtigen
            // Ziels in platformio.ini genügen aber Chip, Flash und PSRAM.
            Serial.println("\n=== Controller-Steckbrief ===");
            Serial.printf("Chip: %s, Revision %d, %d Kern(e), %lu MHz\n",
                          ESP.getChipModel(), ESP.getChipRevision(),
                          ESP.getChipCores(),
                          (unsigned long)ESP.getCpuFreqMHz());
            Serial.printf("Flash: %lu Bytes (%lu MB) bei %lu Hz\n",
                          (unsigned long)ESP.getFlashChipSize(),
                          (unsigned long)(ESP.getFlashChipSize() / (1024UL * 1024UL)),
                          (unsigned long)ESP.getFlashChipSpeed());
#ifdef BOARD_HAS_PSRAM
            Serial.println("Build: BOARD_HAS_PSRAM ist gesetzt");
#else
            Serial.println("Build: BOARD_HAS_PSRAM ist NICHT gesetzt");
#endif
            if (psramFound()) {
                Serial.printf("PSRAM: %lu Bytes vorhanden, %lu Bytes frei\n",
                              (unsigned long)ESP.getPsramSize(),
                              (unsigned long)ESP.getFreePsram());
            } else {
                Serial.println("PSRAM: nicht gefunden");
#ifdef BOARD_HAS_PSRAM
                Serial.println("  ⚠️ Der Build erwartet PSRAM, der Chip hat keinen.");
                Serial.println("     Board-Ziel in platformio.ini passt nicht.");
#endif
            }
            {
                const uint64_t mac = ESP.getEfuseMac();
                Serial.printf("eFuse-MAC: %04X%08X\n",
                              (unsigned)(uint16_t)(mac >> 32),
                              (unsigned)(uint32_t)mac);
            }
            Serial.printf("Sketch: %lu Bytes belegt, %lu Bytes frei in der Partition\n",
                          (unsigned long)ESP.getSketchSize(),
                          (unsigned long)ESP.getFreeSketchSpace());

            Serial.println("\nModule-Status:");
            Serial.printf("BNO055: %s (I2C 0x%02X)\n",
                          bnoManager.isSelfTestPassed() ? "OK" : "Fehler",
                          bnoManager.getAddress());
            Serial.printf("GPS: %s\n", gpsManager.isReady() ? "OK" : "Fehler");
            Serial.printf("CAN: %s\n", canReader.isReady() ? "OK" : "Fehler");
            Serial.printf("SD: %s\n", sdLogger.isReady() ? "OK" : "Fehler");
            Serial.printf("OLED: %s\n", oledManager.isReady() ? "OK" : "Fehler");
            
            // GPS-Details
            gpsManager.printDiagnostics();
            
            // Buffer-Statistiken
            printBufferStats();
        }
    }
    
    // Kurze Pause um CPU zu entlasten
    delay(1);
}

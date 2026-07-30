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
#include "runtime_diagnostics.h"
#include "web_manager.h"
#include "vehicle_data_discovery.h"

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
constexpr bool ENABLE_OPTIONAL_CAN = true;
unsigned long lastSDCheck = 0;
String currentLogFileName = "";
String canLogFileName = "";
File logFile;
bool canLoggingEnabled = false;
int totalCANMessages = 0;

// Nicht blockierende Hardware-Überwachung
constexpr unsigned long HARDWARE_CHECK_INTERVAL = 5000;
constexpr unsigned long GPS_RESTART_INTERVAL = 30000;
constexpr unsigned long CAN_RECOVERY_RETRY_INTERVAL = 10000;
constexpr uint8_t CAN_MODE_FAILURE_LIMIT = 2;
constexpr unsigned long BOOT_STATUS_INTERVAL = 1000;
constexpr unsigned long BOOT_READY_HOLD_TIME = 2500;
constexpr unsigned long SENSOR_SAMPLE_INTERVAL_MS = 100;
constexpr unsigned long GPS_SNAPSHOT_INTERVAL_MS = 200;
unsigned long lastHardwareCheck = 0;
unsigned long lastGPSRestart = 0;
unsigned long lastCANRecoveryAttempt = 0;
unsigned long lastBootStatusDisplay = 0;
unsigned long requiredHardwareReadySince = 0;
uint8_t bnoMissingChecks = 0;
uint8_t oledMissingChecks = 0;
uint32_t bnoProbeFailureCount = 0;
uint32_t oledProbeFailureCount = 0;
uint8_t canModeFailureCount = 0;
bool oledDetectedAtLeastOnce = false;
bool bootStatusActive = true;

struct VehicleTestSession {
    bool active = false;
    bool recordingObserved = false;
    unsigned long startedAt = 0;
    unsigned long initialOBDRequests = 0;
    unsigned long initialOBDResponses = 0;
    unsigned long initialOBDErrors = 0;
    uint32_t initialSDWrites = 0;
    uint32_t initialSDErrors = 0;
    uint32_t initialSDDropped = 0;
    uint32_t initialBNOProbeFailures = 0;
    uint32_t initialOLEDProbeFailures = 0;
    uint8_t peakTransmitErrorCount = 0;
    uint8_t peakReceiveErrorCount = 0;
    uint8_t latchedCANErrorFlags = 0;
};

VehicleTestSession vehicleTestSession;

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
        if (address == BNO055_ADDRESS_A || address == BNO055_ADDRESS_B) {
            error = BNO055Manager::probeAddress(address) ? 0 : 2;
        } else {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
        }
        
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
        Serial.printf("\n%s-Kalibrierung (0-3):\n", ROADTEST_BNO_MODE_NAME);
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
    oledManager.showSystemInfo(
        ROADTEST_FIRMWARE_VERSION, millis(), ESP.getFreeHeap());
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
    Wire.setTimeOut(I2C_TIMEOUT_MS);
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
    return (!OLED_REQUIRED || oledManager.isReady()) &&
           bnoManager.isSelfTestPassed() &&
           bnoManager.isFusionModeActive() && bnoManager.isDataValid() &&
           sdLogger.isReady() && gpsManager.isReceivingNMEA() &&
           webAvailable;
}

const char* canOperatingModeName(uint8_t mode) {
    switch (mode) {
        case 0: return "Normal";
        case 1: return "Sleep";
        case 2: return "Loopback";
        case 3: return "Listen-Only";
        case 4: return "Konfiguration";
        default: return "Unbekannt";
    }
}

void printCANErrorFlagNames(uint8_t flags) {
    if (flags == 0) {
        Serial.print("keine");
        return;
    }

    bool separator = false;
    const auto printFlag = [&separator](const char* name) {
        if (separator) {
            Serial.print(",");
        }
        Serial.print(name);
        separator = true;
    };
    if (flags & 0x01) printFlag("EWARN");
    if (flags & 0x02) printFlag("RXWAR");
    if (flags & 0x04) printFlag("TXWAR");
    if (flags & 0x08) printFlag("RXEP");
    if (flags & 0x10) printFlag("TXEP");
    if (flags & 0x20) printFlag("TXBO");
    if (flags & 0x40) printFlag("RX0OVR");
    if (flags & 0x80) printFlag("RX1OVR");
}

void updateVehicleTestCANLatches(const CANHardwareDiagnostics& diagnostics) {
    if (!vehicleTestSession.active || !diagnostics.valid) {
        return;
    }
    vehicleTestSession.peakTransmitErrorCount =
        max(vehicleTestSession.peakTransmitErrorCount,
            diagnostics.transmitErrorCount);
    vehicleTestSession.peakReceiveErrorCount =
        max(vehicleTestSession.peakReceiveErrorCount,
            diagnostics.receiveErrorCount);
    vehicleTestSession.latchedCANErrorFlags |= diagnostics.errorFlags;
}

void printVehicleTestStatus(bool finalReport) {
    if (!vehicleTestSession.active) {
        Serial.println("ℹ️ Kein Fahrzeug-Test aktiv; zuerst 'test begin' eingeben");
        return;
    }

    const unsigned long now = millis();
    const OBDLiveData obd = canReader.getOBDData();
    const LogStats sdStats = sdLogger.getStatistics();
    const CANHardwareDiagnostics canHardware =
        canReader.getHardwareDiagnostics();
    updateVehicleTestCANLatches(canHardware);

    const unsigned long requestDelta =
        obd.requestCount - vehicleTestSession.initialOBDRequests;
    const unsigned long responseDelta =
        obd.responseCount - vehicleTestSession.initialOBDResponses;
    const unsigned long obdErrorDelta =
        obd.requestErrors - vehicleTestSession.initialOBDErrors;
    const uint32_t sdWriteDelta =
        sdStats.totalWrites - vehicleTestSession.initialSDWrites;
    const uint32_t sdErrorDelta =
        sdStats.errorCount - vehicleTestSession.initialSDErrors;
    const uint32_t sdDroppedDelta =
        sdStats.droppedLogs - vehicleTestSession.initialSDDropped;
    const uint32_t bnoFailureDelta =
        bnoProbeFailureCount -
        vehicleTestSession.initialBNOProbeFailures;
    const uint32_t oledFailureDelta =
        oledProbeFailureCount -
        vehicleTestSession.initialOLEDProbeFailures;
    const unsigned long responseAge =
        obd.lastResponseMs > 0 ? now - obd.lastResponseMs : 0;
    const bool ecuResponding =
        obd.lastResponseMs > 0 &&
        responseAge <= CAN_OBD_VALUE_MAX_AGE_MS;

    vehicleTestSession.recordingObserved =
        vehicleTestSession.recordingObserved || sdLogger.isLogging() ||
        sdWriteDelta > 0;

    const bool hardCANFailure =
        vehicleTestSession.latchedCANErrorFlags & (0x20 | 0x40 | 0x80);
    const bool canWarning =
        vehicleTestSession.latchedCANErrorFlags & (0x01 | 0x02 | 0x04 |
                                                   0x08 | 0x10);
    const bool requiredModuleFailure =
        !bnoManager.isSelfTestPassed() || !bnoManager.isDataValid() ||
        !sdLogger.isReady() || !gpsManager.isReceivingNMEA() ||
        !webManager.isReady() || !canReader.isReady();
    const bool failed =
        requiredModuleFailure || hardCANFailure || bnoFailureDelta > 0 ||
        sdErrorDelta > 0 || sdDroppedDelta > 0;
    const bool incompleteSDTest =
        finalReport && (!vehicleTestSession.recordingObserved ||
                        sdWriteDelta == 0);
    const bool warned =
        !failed && (canWarning || oledFailureDelta > 0 ||
                    obdErrorDelta > 0 ||
                    (canReader.isOBDPollingEnabled() && !ecuResponding) ||
                    incompleteSDTest);

    Serial.printf("\n=== FAHRZEUG-TEST %s ===\n",
                  finalReport ? "ABSCHLUSS" : "STATUS");
    Serial.printf("Dauer: %lu s\n",
                  (now - vehicleTestSession.startedAt) / 1000);
    Serial.printf("OBD: +%lu Anfragen, +%lu Antworten, +%lu Sendefehler",
                  requestDelta, responseDelta, obdErrorDelta);
    if (requestDelta > 0) {
        Serial.printf(", Antwortrate %.1f %%",
                      responseDelta * 100.0f / requestDelta);
    }
    Serial.printf(", ECU: %s",
                  !canReader.isOBDPollingEnabled()
                      ? "Abfrage pausiert"
                      : (ecuResponding ? "verbunden" : "keine aktuelle Antwort"));
    if (obd.lastResponseMs > 0) {
        Serial.printf(" (%lu ms)", responseAge);
    }
    Serial.println();

    if (canHardware.valid) {
        Serial.printf("MCP2515: Modus=%s, TEC=%u (max %u), REC=%u (max %u), "
                      "EFLG=0x%02X, beobachtet=0x%02X [",
                      canOperatingModeName(canHardware.operatingMode),
                      canHardware.transmitErrorCount,
                      vehicleTestSession.peakTransmitErrorCount,
                      canHardware.receiveErrorCount,
                      vehicleTestSession.peakReceiveErrorCount,
                      canHardware.errorFlags,
                      vehicleTestSession.latchedCANErrorFlags);
        printCANErrorFlagNames(vehicleTestSession.latchedCANErrorFlags);
        Serial.println("]");
    } else {
        Serial.println("MCP2515: Diagnose nicht verfügbar");
    }

    Serial.printf("I2C-Aussetzer im Test: BNO055=%lu, OLED=%lu (optional)\n",
                  static_cast<unsigned long>(bnoFailureDelta),
                  static_cast<unsigned long>(oledFailureDelta));
    Serial.printf("SD im Test: +%lu Schreibvorgänge, +%lu Fehler, "
                  "+%lu verworfen, Aufzeichnung beobachtet=%s\n",
                  static_cast<unsigned long>(sdWriteDelta),
                  static_cast<unsigned long>(sdErrorDelta),
                  static_cast<unsigned long>(sdDroppedDelta),
                  vehicleTestSession.recordingObserved ? "ja" : "nein");
    Serial.printf("Module: BNO=%s, GPS-NMEA=%s, SD=%s, WLAN=%s, CAN=%s\n",
                  bnoManager.isSelfTestPassed() && bnoManager.isDataValid()
                      ? "OK" : "FEHLER",
                  gpsManager.isReceivingNMEA() ? "OK" : "FEHLER",
                  sdLogger.isReady() ? "OK" : "FEHLER",
                  webManager.isReady() ? "OK" : "FEHLER",
                  canReader.isReady() ? "OK" : "FEHLER");
    Serial.printf("ERGEBNIS: %s\n",
                  failed ? "FAIL" : (warned ? "WARN" : "PASS"));
    if (incompleteSDTest) {
        Serial.println("Hinweis: SD-Schreibtest fehlt; während des Tests "
                       "'start' und vor dem Abschluss 'stop' verwenden");
    }
}

void beginVehicleTest() {
    if (vehicleTestSession.active) {
        Serial.println("ℹ️ Fahrzeug-Test läuft bereits");
        printVehicleTestStatus(false);
        return;
    }

    const OBDLiveData obd = canReader.getOBDData();
    const LogStats sdStats = sdLogger.getStatistics();
    const CANHardwareDiagnostics canHardware =
        canReader.getHardwareDiagnostics();

    vehicleTestSession = VehicleTestSession{};
    vehicleTestSession.active = true;
    vehicleTestSession.startedAt = millis();
    vehicleTestSession.recordingObserved = sdLogger.isLogging();
    vehicleTestSession.initialOBDRequests = obd.requestCount;
    vehicleTestSession.initialOBDResponses = obd.responseCount;
    vehicleTestSession.initialOBDErrors = obd.requestErrors;
    vehicleTestSession.initialSDWrites = sdStats.totalWrites;
    vehicleTestSession.initialSDErrors = sdStats.errorCount;
    vehicleTestSession.initialSDDropped = sdStats.droppedLogs;
    vehicleTestSession.initialBNOProbeFailures = bnoProbeFailureCount;
    vehicleTestSession.initialOLEDProbeFailures = oledProbeFailureCount;
    if (canHardware.valid) {
        vehicleTestSession.peakTransmitErrorCount =
            canHardware.transmitErrorCount;
        vehicleTestSession.peakReceiveErrorCount =
            canHardware.receiveErrorCount;
        vehicleTestSession.latchedCANErrorFlags = canHardware.errorFlags;
    }

    Serial.println("\n✅ Fahrzeug-Test gestartet");
    Serial.println("Der Test beobachtet nur; Aufzeichnung und OBD-Zustand "
                   "werden nicht automatisch verändert");
    Serial.println("Ablauf: Zündung prüfen, 'start', Motorlauf prüfen, "
                   "'stop', danach 'test end'");
    printVehicleTestStatus(false);
}

void endVehicleTest() {
    if (!vehicleTestSession.active) {
        Serial.println("ℹ️ Kein Fahrzeug-Test aktiv");
        return;
    }
    printVehicleTestStatus(true);
    vehicleTestSession.active = false;
    Serial.println("=== FAHRZEUG-TEST BEENDET ===\n");
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
            bnoManager.isSelfTestPassed() && bnoManager.isFusionModeActive() &&
                bnoManager.isDataValid(),
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

    // Der BNO055 wird über seine Chip-ID, das OLED per I²C-Ping überwacht.
    // Erst drei aufeinanderfolgende Ausfälle gelten als echte Trennung.
    bool bnoResponding = bnoManager.responds();
    if (bnoManager.isReady()) {
        if (!bnoResponding) {
            bnoManager.invalidateMeasurements();
            bnoProbeFailureCount++;
            Serial.printf("⚠️ BNO055-I2C-Prüfung fehlgeschlagen (%u/3, gesamt %lu)\n",
                          bnoMissingChecks + 1,
                          static_cast<unsigned long>(bnoProbeFailureCount));
        } else if (bnoMissingChecks > 0) {
            Serial.printf("✅ BNO055-I2C wieder stabil nach %u Fehlprüfung(en)\n",
                          bnoMissingChecks);
        }
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
        } else if (bnoResponding && !bnoManager.verifyFusionMode(true)) {
            Serial.println("⚠️ BNO055-Fusion dreimal fehlerhaft; vollständiger Neustart");
            bnoManager.restartFusion();
        }
    } else if (bnoResponding && bnoManager.begin(true)) {
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
        if (!oledResponding) {
            oledProbeFailureCount++;
            Serial.printf("⚠️ OLED-I2C-Prüfung fehlgeschlagen (%u/3, gesamt %lu)\n",
                          oledMissingChecks + 1,
                          static_cast<unsigned long>(oledProbeFailureCount));
        } else if (oledMissingChecks > 0) {
            Serial.printf("✅ OLED-I2C wieder stabil nach %u Fehlprüfung(en)\n",
                          oledMissingChecks);
        }
        oledMissingChecks = oledResponding ? 0 : oledMissingChecks + 1;
        if (oledMissingChecks >= 3) {
            Serial.println("⚠️ Optionales OLED nicht mehr erreichbar; System läuft weiter");
            oledManager.end(false);
            oledMissingChecks = 0;
        }
    } else if (oledResponding && oledManager.begin(OLED_ADDRESS_A)) {
        oledManager.setRotation(true);
        oledDetectedAtLeastOnce = true;
        bootStatusActive = true;
        Serial.println("✅ Optionales OLED im Hintergrund wieder verbunden");
    }

    if (sdLogger.isReady() && !sdLogger.isLoggingStartPending()) {
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

    // CAN bleibt für die allgemeine Bereitschaft optional, wird nach einem
    // Startfehler oder einem dauerhaft falschen Controllerzustand aber mit
    // begrenztem Abstand erneut initialisiert. Eine laufende Listen-Only-Phase
    // wird dabei ausdrücklich wieder als Listen-Only hergestellt.
    if (ENABLE_OPTIONAL_CAN && !canReader.isReady()) {
        canBusAvailable = false;
        if (now - lastCANRecoveryAttempt >= CAN_RECOVERY_RETRY_INTERVAL) {
            lastCANRecoveryAttempt = now;
            canBusAvailable = initializeOptionalCAN();
            if (canBusAvailable && sdLogger.isLogging()) {
                sdLogger.logEvent(
                    "CAN_RECOVERY",
                    "MCP2515 nach Nichtverfügbarkeit initialisiert");
            }
        }
    } else if (ENABLE_OPTIONAL_CAN) {
        canBusAvailable = true;
        const CANHardwareDiagnostics diagnostics =
            canReader.getHardwareDiagnostics();
        const uint8_t expectedMode =
            vehicleDataDiscovery.isPassiveCaptureActive() ? 3 : 0;
        const bool controllerFault =
            !diagnostics.valid || diagnostics.transmitBusOff ||
            diagnostics.operatingMode != expectedMode;
        canModeFailureCount =
            controllerFault
                ? (canModeFailureCount < CAN_MODE_FAILURE_LIMIT
                       ? canModeFailureCount + 1
                       : CAN_MODE_FAILURE_LIMIT)
                : 0;

        if (canModeFailureCount >= CAN_MODE_FAILURE_LIMIT &&
            now - lastCANRecoveryAttempt >=
                CAN_RECOVERY_RETRY_INTERVAL) {
            lastCANRecoveryAttempt = now;
            const bool passiveMode =
                vehicleDataDiscovery.isPassiveCaptureActive();
            if (sdLogger.isLogging()) {
                sdLogger.logEvent(
                    "CAN_RECOVERY",
                    String("START;MODE_") +
                        String(diagnostics.operatingMode) +
                        ";EFLG_" + String(diagnostics.errorFlags, HEX));
            }
            canBusAvailable =
                canReader.restartController(CAN_BAUDRATE, passiveMode);
            canModeFailureCount = 0;
            if (sdLogger.isLogging()) {
                sdLogger.logEvent(
                    "CAN_RECOVERY",
                    canBusAvailable ? "SUCCESS" : "FAILED");
            }
        }
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
    Serial.printf(
        "Version %s mit Datenqualitätsdiagnose und BNO055-%s\n\n",
        ROADTEST_FIRMWARE_VERSION, ROADTEST_BNO_MODE_NAME);

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
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    Serial.printf("I2C: %s (SDA=%d, SCL=%d, %d Hz)\n",
                  i2cSuccess ? "OK" : "FEHLER", I2C_SDA, I2C_SCL, I2C_CLOCK_SPEED);

    // Das steckbare OLED ist eine reine Komfortanzeige. Nur initialisieren,
    // wenn eine seiner bekannten Adressen antwortet; sein Fehlen darf weder
    // Messung noch Bereitschaft blockieren.
    const bool oledPresent =
        i2cDeviceResponds(OLED_ADDRESS_A) ||
        i2cDeviceResponds(OLED_ADDRESS_B);
    if (oledPresent && oledManager.begin(OLED_ADDRESS_A)) {
        oledManager.setRotation(true);
        oledDetectedAtLeastOnce = true;
        Serial.println("✅ Optionales OLED geprüft");
    } else if (oledPresent) {
        oledDetectedAtLeastOnce = true;
        Serial.println("⚠️ Optionales OLED erkannt, Initialisierung fehlgeschlagen");
    } else {
        Serial.println("ℹ️ Optionales OLED nicht verbunden; System läuft ohne Display");
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

    if (ENABLE_OPTIONAL_CAN) {
        Serial.println(CAN_OBD_POLLING_ENABLED
                           ? "ℹ️ CAN startet mit begrenzter, nur lesender OBD-Abfrage"
                           : "ℹ️ CAN wird nach dem Systemstart einmalig geprüft");
    } else {
        Serial.println(
            "ℹ️ CAN ist deaktiviert; es kann WLAN und Webseite nicht blockieren");
    }
    updateBootStatusDisplay(millis(), true);

    lastHardwareCheck = millis();
    lastCANRecoveryAttempt =
        millis() - CAN_RECOVERY_RETRY_INTERVAL;
    lastUpdate = millis();
    Serial.println("\nBootprüfung läuft nicht blockierend weiter.");
    Serial.println("Webseite bleibt erreichbar; CAN ist für Messfahrten nicht erforderlich.");
}

void loop() {
    static unsigned long previousLoopStartedAt = 0;
    static unsigned long lastCANCheck = 0;
    static unsigned long lastOBDRequest = 0;
    static uint8_t obdPidIndex = 0;
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

    const unsigned long loopStartedAt = millis();
    if (previousLoopStartedAt > 0) {
        runtimeDiagnostics.recordLoopInterval(
            loopStartedAt - previousLoopStartedAt);
    }
    previousLoopStartedAt = loopStartedAt;

    esp_task_wdt_reset();

    // Web-Anfragen und OTA zuerst bedienen. Während eines Firmware-Uploads
    // bleibt die SD-Karte geschlossen und die übrige Messschleife pausiert.
    webManager.handleClient();
    if (webManager.isOTAInProgress()) {
        delay(1);
        return;
    }
    // Der Web-Handler quittiert den Start sofort. Pro loop()-Runde wird
    // anschließend höchstens eine SD-Startoperation ausgeführt, damit der
    // Webserver zwischen den Dateizugriffen weiter Anfragen bedienen kann.
    sdLogger.processLoggingStart();
    
    unsigned long currentTime = millis();
    handleHardwareRecovery(currentTime);
    gpsAvailable = gpsManager.isReady();
    // Den UART-Ringpuffer in jeder Schleifenrunde leeren. Die 200-ms-Grenze
    // gilt nur für CSV-Snapshots, nicht für den Empfang der NMEA-Zeichen.
    if (gpsAvailable) {
        gpsManager.update();
    }
    currentTime = millis();
    
    // BNO055 Sensor-Daten lesen (alle 100ms)
    if (bnoManager.isSelfTestPassed() &&
        bnoManager.isFusionModeActive() &&
        bnoManager.isDataValid() &&
        (currentTime - lastSensorRead >= SENSOR_SAMPLE_INTERVAL_MS)) {
        const bool firstSensorSample = lastSensorRead == 0;
        const unsigned long sensorElapsed =
            firstSensorSample
                ? SENSOR_SAMPLE_INTERVAL_MS
                : currentTime - lastSensorRead;
        runtimeDiagnostics.recordSensorSchedule(
            sensorElapsed, SENSOR_SAMPLE_INTERVAL_MS);
        SensorData sensorData = bnoManager.getCurrentData();
        lastSensorData = sensorData;  // Für Zeitkorrelation speichern
        bnoManager.processSample(sensorData);

        // Alle Auswertungen verwenden exakt denselben 10-Hz-Sensorwert.
        VibrationMetrics vibMetrics = bnoManager.analyzeVibration();
        float speedKmh =
            lastGPSData.speed_valid ? lastGPSData.speed_kmh : -1.0f;
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
        
        if (firstSensorSample) {
            lastSensorRead = currentTime;
        } else {
            lastSensorRead +=
                max(
                    sensorElapsed / SENSOR_SAMPLE_INTERVAL_MS,
                    1UL) *
                SENSOR_SAMPLE_INTERVAL_MS;
        }
        if (lastSensorRead > currentTime) {
            lastSensorRead = currentTime;
        }
    }
    
    // GPS-Daten verarbeiten (alle 200ms)
    // Im Interrupt-Modus verarbeitet update() die gepufferten Daten
    if (gpsAvailable &&
        (currentTime - lastGPSUpdate >= GPS_SNAPSHOT_INTERVAL_MS)) {
        const bool firstGPSSnapshot = lastGPSUpdate == 0;
        const unsigned long gpsElapsed =
            firstGPSSnapshot
                ? GPS_SNAPSHOT_INTERVAL_MS
                : currentTime - lastGPSUpdate;
        runtimeDiagnostics.recordGPSSchedule(
            gpsElapsed, GPS_SNAPSHOT_INTERVAL_MS);

        if (!gpsClockSynced && gpsManager.hasValidDateTime()) {
            gpsClockSynced = gpsManager.syncSystemClock();
            if (gpsClockSynced) {
                Serial.println("✅ Systemzeit nachträglich aus GPS synchronisiert");
            }
        }
        
        // Auch ein Fixverlust ist ein Messwert. Daher wird bei aktiver
        // Aufzeichnung alle 200 ms ein Qualitäts-Snapshot gespeichert und
        // nicht nur dann, wenn TinyGPS++ gerade einen gültigen Fix meldet.
        GPSData gpsData = gpsManager.getCurrentData(true);
        lastGPSData = gpsData;
        if (sdLogger.isLogging()) {
            sdLogger.logGPSData(gpsData);
        }
        
        if (firstGPSSnapshot) {
            lastGPSUpdate = currentTime;
        } else {
            lastGPSUpdate +=
                max(
                    gpsElapsed / GPS_SNAPSHOT_INTERVAL_MS,
                    1UL) *
                GPS_SNAPSHOT_INTERVAL_MS;
        }
        if (lastGPSUpdate > currentTime) {
            lastGPSUpdate = currentTime;
        }
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

    // Im normalen OBD-Betrieb genügt ein Frame pro 10-ms-Zyklus. Während
    // des passiven Erkennungsmitschnitts wird der MCP2515 dagegen in jeder
    // loop()-Runde bis zu 32-mal geleert, damit zyklischer Fahrzeugverkehr
    // seine beiden kleinen Empfangspuffer nicht sofort überläuft.
    if (canBusAvailable &&
        (vehicleDataDiscovery.isActive() ||
         currentTime - lastCANCheck >= 10)) {
        const uint8_t receiveBudget =
            vehicleDataDiscovery.isActive() ? 32 : 1;
        uint8_t receivedThisCycle = 0;

        // hasMessage() puffert den Frame bereits; readMessage() liefert danach
        // genau diesen zurück. Eine zusätzliche Prüfung auf canId != 0 wäre
        // falsch, weil 0x000 eine gültige CAN-ID mit höchster Priorität ist.
        while (receivedThisCycle < receiveBudget &&
               canReader.hasMessage()) {
            CANMessage msg = canReader.readMessage();
            const bool obdDecoded = canReader.processOBDResponse(msg);

            canMessageCount++;
            totalCANMessages++;
            lastCANMessage = msg;  // Für Zeitkorrelation speichern
            vehicleDataDiscovery.onCANMessage(msg);

            // In SDLogger aufzeichnen
            if (sdLogger.isLogging()) {
                sdLogger.logCANMessage(msg);
                if (obdDecoded) {
                    sdLogger.logOBDData(
                        canReader.getOBDData(),
                        canReader.getOBDSessionStats(),
                        canReader.getHardwareDiagnostics());
                }

                // Zeitkorrelation: Wenn aktuelle Sensor-Daten vorhanden
                if (!vehicleDataDiscovery.isActive() &&
                    lastSensorData.timestamp > 0 &&
                    abs((long)(msg.timestamp - lastSensorData.timestamp)) < 1000) {
                    sdLogger.logCorrelatedData(lastSensorData, msg);
                }
            }

            // Detaillierte Ausgabe für die ersten 10 Nachrichten
            if (canMessageCount <= 10) {
                Serial.printf("CAN #%d: ", canMessageCount);
                Serial.println(formatCANMessage(msg));
            }
            receivedThisCycle++;
        }
        lastCANCheck = currentTime;
    }

    // Empfang immer vor der Timeoutbewertung abarbeiten. So wird eine bereits
    // im MCP2515 wartende Antwort nach einem langsamen SD-Flush nicht zuerst
    // fälschlich als Timeout und anschließend als unzugeordnet verbucht.
    if (canBusAvailable) {
        canReader.updateOBDDiagnostics();
    }

    // Die Discovery darf erst nach dem Empfang entscheiden, ob eine ECU
    // erreichbar ist oder ob die nächste sichere Anfrage gesendet werden muss.
    vehicleDataDiscovery.update();

    // Drei standardisierte Live-PIDs zyklisch abfragen. Das gemeinsame
    // Intervall begrenzt die gesamte Senderate auf höchstens zwei Frames pro
    // Sekunde; ein fehlendes ACK endet zusätzlich im MCP2515-Treiber nach
    // CAN_TX_TIMEOUT_MS.
    if (canBusAvailable && canReader.isOBDPollingEnabled() &&
        !vehicleDataDiscovery.controlsOBDPolling() &&
        currentTime - lastOBDRequest >= CAN_OBD_REQUEST_INTERVAL_MS) {
        static const uint8_t obdPids[] = {0x0C, 0x0D, 0x11};
        canReader.requestOBDPid(obdPids[obdPidIndex]);
        obdPidIndex =
            (obdPidIndex + 1) % (sizeof(obdPids) / sizeof(obdPids[0]));
        lastOBDRequest = currentTime;
    }

    // Anfragen, Sendeprobleme, Antworten und Timeouts werden unabhängig von
    // der dekodierten Livewert-Datei als einzelne Transaktionen festgehalten.
    if (canBusAvailable && sdLogger.isLogging()) {
        OBDTraceEvent traceEvent;
        while (canReader.popOBDTraceEvent(traceEvent)) {
            sdLogger.logOBDTraceEvent(
                traceEvent, canReader.getHardwareDiagnostics());
        }
    }
    
    // Status-Report alle 5 Sekunden
    if (currentTime - lastStatusReport >= 5000) {
        Serial.printf("[%lu] System läuft - CAN: %d msg, ", 
                     currentTime/1000, canMessageCount);
        
        // BNO055 Status
        CalibrationData cal = {0, 0, 0, 0};
        if (bnoManager.isSelfTestPassed() &&
            bnoManager.isFusionModeActive() &&
            bnoManager.isDataValid()) {
            cal = bnoManager.getCalibration();
            BNO055RuntimeStatus bnoStatus = bnoManager.getRuntimeStatus();
            Serial.printf("BNO055 %s/Sys:%d Kal:S%d/G%d/A%d/M%d",
                         ROADTEST_BNO_MODE_NAME, bnoStatus.systemStatus,
                         cal.system, cal.gyro, cal.accel, cal.mag);
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
            const GPSStatus gpsStatus = gpsManager.getStatus();
            if (lastGPSData.valid_fix) {
                Serial.printf(", GPS: %.6f°N %.6f°E (%d sat)", 
                             lastGPSData.latitude, lastGPSData.longitude,
                             lastGPSData.satellites);
            } else {
                uint8_t sats = gpsManager.getSatelliteCount();
                Serial.printf(", GPS: Kein Fix (%d sat)", sats);
            }
            Serial.printf(
                " [Fix #%lu, Alter %lu ms, NMEA %lu/%lu, RX-Overflow %lu]",
                static_cast<unsigned long>(lastGPSData.fix_sequence),
                static_cast<unsigned long>(lastGPSData.location_age_ms),
                static_cast<unsigned long>(gpsStatus.sentences_received),
                static_cast<unsigned long>(gpsStatus.sentences_failed),
                static_cast<unsigned long>(gpsStatus.rx_buffer_overflows));
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

        if (canBusAvailable && CAN_OBD_POLLING_ENABLED) {
            OBDLiveData obd = canReader.getOBDData();
            const OBDSessionStats obdSession =
                canReader.getOBDSessionStats();
            const unsigned long obdResponseAge =
                obd.lastResponseMs > 0 ? currentTime - obd.lastResponseMs : 0;
            const bool ecuResponding =
                obd.lastResponseMs > 0 &&
                obdResponseAge <= CAN_OBD_VALUE_MAX_AGE_MS;
            const char* obdConnectionStatus =
                !canReader.isOBDPollingEnabled()
                    ? "Abfrage pausiert"
                    : (ecuResponding ? "verbunden" : "warte auf Antwort");
            if (obdSession.active) {
                Serial.printf(
                    "   OBD/ECU: %s, Sitzung %lu Anfrage(n), %lu "
                    "Antwort(en), %lu Sendefehler, %lu Timeouts",
                    obdConnectionStatus,
                    static_cast<unsigned long>(obdSession.requestCount),
                    static_cast<unsigned long>(obdSession.responseCount),
                    static_cast<unsigned long>(obdSession.requestErrors),
                    static_cast<unsigned long>(obdSession.timeoutCount));
                if (obdSession.lastResponseMs > 0) {
                    Serial.printf(
                        ", Antwort %03lX/PID %02X in %lu ms",
                        static_cast<unsigned long>(
                            obdSession.lastResponseCanId),
                        obdSession.lastResponsePid,
                        static_cast<unsigned long>(
                            obdSession.lastResponseLatencyMs));
                }
                Serial.printf(
                    " [Boot %lu/%lu/%lu]",
                    obd.requestCount, obd.responseCount, obd.requestErrors);
            } else {
                Serial.printf(
                    "   OBD/ECU: %s, Boot %lu Anfrage(n), %lu "
                    "Antwort(en), %lu Sendefehler",
                    obdConnectionStatus,
                    obd.requestCount, obd.responseCount,
                    obd.requestErrors);
            }
            if (obd.lastResponseMs > 0) {
                Serial.printf(", letzte Antwort vor %lu ms", obdResponseAge);
            }
            if (obd.rpmValid) {
                Serial.printf(", %.0f rpm", obd.rpm);
            }
            if (obd.speedValid) {
                Serial.printf(", %u km/h", obd.speedKmh);
            }
            if (obd.throttleValid) {
                Serial.printf(", %.1f %% Drossel", obd.throttlePercent);
            }
            if (obd.mafValid) {
                Serial.printf(", %.2f g/s MAF", obd.mafGramsPerSecond);
            }
            if (obd.fuelRateValid) {
                Serial.printf(
                    ", %.2f l/h Kraftstoff",
                    obd.fuelRateLitersPerHour);
            }
            if (obd.oilTemperatureValid) {
                Serial.printf(", %.1f C Oel", obd.oilTemperatureC);
            }
            if (obd.ambientTemperatureValid) {
                Serial.printf(
                    ", %.1f C Aussen",
                    obd.ambientTemperatureC);
            }
            Serial.println();
        }

        if (canReader.isReady()) {
            const CANHardwareDiagnostics canHardware =
                canReader.getHardwareDiagnostics();
            updateVehicleTestCANLatches(canHardware);
            if (canHardware.valid) {
                Serial.printf("   CAN-HW: Modus=%s, TEC=%u, REC=%u, EFLG=0x%02X [",
                              canOperatingModeName(canHardware.operatingMode),
                              canHardware.transmitErrorCount,
                              canHardware.receiveErrorCount,
                              canHardware.errorFlags);
                printCANErrorFlagNames(canHardware.errorFlags);
                Serial.print("]");
                if (vehicleTestSession.active) {
                    Serial.printf(", Fahrzeug-Test aktiv seit %lu s",
                                  (currentTime -
                                   vehicleTestSession.startedAt) / 1000);
                }
                if (vehicleDataDiscovery.isActive()) {
                    Serial.printf(", Datenerkennung: %s",
                                  vehicleDataDiscovery.getPhaseName());
                }
                Serial.println();
            }
        }

        {
            const LogStats logStats = sdLogger.getStatistics();
            Serial.printf("   TEST: OLED=%s%s, I2C-Aussetzer BNO/OLED=%lu/%lu, "
                          "SD-Schreibvorgänge=%lu Fehler=%lu Verworfen=%lu, WLAN=%s\n",
                          oledManager.isReady() ? "OK" : "nicht verbunden",
                          OLED_REQUIRED ? "" : " (optional)",
                          static_cast<unsigned long>(bnoProbeFailureCount),
                          static_cast<unsigned long>(oledProbeFailureCount),
                          static_cast<unsigned long>(logStats.totalWrites),
                          static_cast<unsigned long>(logStats.errorCount),
                          static_cast<unsigned long>(logStats.droppedLogs),
                          webManager.isReady() ? "OK" : "Fehler");
        }
        
        if (canLoggingEnabled) {
            canReader.flushLog(); // CAN-Log aktualisieren
        }
        
        // Bei Änderungen oder länger unvollständiger Kalibrierung zeigt das
        // OLED den Assistenten, ansonsten die Live-Daten.
        if (!bootStatusActive && bnoManager.isSelfTestPassed() &&
            bnoManager.isDataValid() &&
            oledManager.isReady()) {
            bool calibrationChanged =
                cal.system != lastDisplayedCalibration.system ||
                cal.gyro != lastDisplayedCalibration.gyro ||
                cal.accel != lastDisplayedCalibration.accel ||
                cal.mag != lastDisplayedCalibration.mag ||
                bnoManager.isCalibrationSaved() != lastDisplayedCalibrationSaved;
            bool calibrationReminder =
                !cal.isFullyCalibrated() &&
                currentTime - lastCalibrationDisplay >= 30000;

            if (calibrationChanged || calibrationReminder) {
                oledManager.showCalibrationStatus(
                    cal.system, cal.gyro, cal.accel, cal.mag,
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
            Serial.println("obd off - OBD-Abfragen für Werkstatttests pausieren");
            Serial.println("obd on - OBD-Abfragen für Fahrzeugtests aktivieren");
            Serial.println("test begin - passiven Fahrzeug-Test starten");
            Serial.println("test status - Zwischenbericht ausgeben");
            Serial.println("test end - Test mit PASS/WARN/FAIL abschließen");
            Serial.println("discover begin - Fahrzeugdaten-Erkennung und SD-Log starten");
            Serial.println("discover status - Phase und erkannte Daten anzeigen");
            Serial.println("discover mark <Text> - Zeitpunkt im SD-Log markieren");
            Serial.println("discover end - Erkennung sicher beenden");
            Serial.println("gps_mode - GPS Interrupt/Polling umschalten");
            Serial.println("diag - System-Diagnose");
        }
        else if (command == "test begin" || command == "test_begin") {
            beginVehicleTest();
        }
        else if (command == "test status" || command == "test_status") {
            printVehicleTestStatus(false);
        }
        else if (command == "test end" || command == "test_end") {
            endVehicleTest();
        }
        else if (command == "discover begin" ||
                 command == "discover_begin") {
            vehicleDataDiscovery.begin();
        }
        else if (command == "discover status" ||
                 command == "discover_status") {
            vehicleDataDiscovery.printStatus();
        }
        else if (command == "discover end" ||
                 command == "discover_end") {
            vehicleDataDiscovery.end();
        }
        else if (command.startsWith("discover mark ")) {
            vehicleDataDiscovery.addMarker(command.substring(14));
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
            if (vehicleDataDiscovery.isActive()) {
                Serial.println(
                    "ℹ️ Datenerkennung verwaltet die SD-Aufzeichnung bereits");
            } else if (sdLogger.startLogging()) {
                Serial.println("✅ Messfahrt gestartet");
            } else {
                Serial.println("❌ Messfahrt konnte nicht gestartet werden");
            }
        }
        else if (command == "stop") {
            if (vehicleDataDiscovery.isActive()) {
                Serial.println(
                    "ℹ️ Für diese Aufzeichnung 'discover end' verwenden");
            } else {
                sdLogger.stopLogging();
            }
        }
        else if (command == "obd off" || command == "obd_off") {
            if (vehicleDataDiscovery.isActive()) {
                Serial.println(
                    "ℹ️ Der aktuelle Erkennungsablauf steuert den OBD-Zustand");
            } else {
                canReader.setOBDPollingEnabled(false);
                Serial.println(
                    "ℹ️ OBD-Abfragen pausiert; CAN-Adapter bleibt initialisiert");
            }
        }
        else if (command == "obd on" || command == "obd_on") {
            if (vehicleDataDiscovery.isActive()) {
                Serial.println(
                    "ℹ️ Der aktuelle Erkennungsablauf steuert den OBD-Zustand");
            } else {
                canReader.setOBDPollingEnabled(true);
                Serial.println(
                    "✅ OBD-Abfragen aktiviert (maximal zwei pro Sekunde)");
            }
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
            OBDLiveData diagnosticOBD = canReader.getOBDData();
            const OBDSessionStats diagnosticOBDSession =
                canReader.getOBDSessionStats();
            const unsigned long diagnosticOBDAge =
                diagnosticOBD.lastResponseMs > 0
                    ? millis() - diagnosticOBD.lastResponseMs
                    : 0;
            Serial.printf("CAN: %s, OBD seit Boot: Anfragen=%lu "
                          "Antworten=%lu Sendefehler=%lu",
                          canReader.isReady() ? "OK" : "Fehler",
                          static_cast<unsigned long>(diagnosticOBD.requestCount),
                          static_cast<unsigned long>(diagnosticOBD.responseCount),
                          static_cast<unsigned long>(diagnosticOBD.requestErrors));
            if (diagnosticOBD.lastResponseMs > 0) {
                Serial.printf(", letzte ECU-Antwort vor %lu ms",
                              diagnosticOBDAge);
            }
            Serial.println();
            if (diagnosticOBDSession.active) {
                Serial.printf(
                    "OBD-Sitzung: Anfragen=%lu Antworten=%lu "
                    "Sendefehler=%lu Timeouts=%lu Nichtzugeordnet=%lu "
                    "Trace-Verworfen=%lu\n",
                    static_cast<unsigned long>(
                        diagnosticOBDSession.requestCount),
                    static_cast<unsigned long>(
                        diagnosticOBDSession.responseCount),
                    static_cast<unsigned long>(
                        diagnosticOBDSession.requestErrors),
                    static_cast<unsigned long>(
                        diagnosticOBDSession.timeoutCount),
                    static_cast<unsigned long>(
                        diagnosticOBDSession.unmatchedResponseCount),
                    static_cast<unsigned long>(
                        diagnosticOBDSession.traceDropped));
                Serial.printf(
                    "Letzte OBD-Transaktion: Anfrage #%lu PID %02X TX=%s, "
                    "Antwort %03lX/PID %02X in %lu ms\n",
                    static_cast<unsigned long>(
                        diagnosticOBDSession.lastRequestSequence),
                    diagnosticOBDSession.lastRequestPid,
                    diagnosticOBDSession.lastTransmitOK ? "OK" : "Fehler",
                    static_cast<unsigned long>(
                        diagnosticOBDSession.lastResponseCanId),
                    diagnosticOBDSession.lastResponsePid,
                    static_cast<unsigned long>(
                        diagnosticOBDSession.lastResponseLatencyMs));
            } else {
                Serial.println("OBD-Sitzung: keine aktive SD-Aufzeichnung");
            }
            CANHardwareDiagnostics diagnosticCANHardware =
                canReader.getHardwareDiagnostics();
            if (diagnosticCANHardware.valid) {
                Serial.printf("MCP2515: Modus=%s, TEC=%u, REC=%u, "
                              "EFLG=0x%02X [",
                              canOperatingModeName(
                                  diagnosticCANHardware.operatingMode),
                              diagnosticCANHardware.transmitErrorCount,
                              diagnosticCANHardware.receiveErrorCount,
                              diagnosticCANHardware.errorFlags);
                printCANErrorFlagNames(diagnosticCANHardware.errorFlags);
                Serial.println("]");
            } else {
                Serial.println("MCP2515: Diagnose nicht verfügbar");
            }
            LogStats diagnosticSD = sdLogger.getStatistics();
            Serial.printf("SD: %s, Schreiben=%lu Fehler=%lu Verworfen=%lu\n",
                          sdLogger.isReady() ? "OK" : "Fehler",
                          static_cast<unsigned long>(diagnosticSD.totalWrites),
                          static_cast<unsigned long>(diagnosticSD.errorCount),
                          static_cast<unsigned long>(diagnosticSD.droppedLogs));
            Serial.printf("OLED: %s (optional, jemals erkannt: %s, I2C-Aussetzer: %lu)\n",
                          oledManager.isReady() ? "OK" : "nicht verbunden",
                          oledDetectedAtLeastOnce ? "ja" : "nein",
                          static_cast<unsigned long>(oledProbeFailureCount));
            Serial.printf("BNO055-I2C-Aussetzer: %lu\n",
                          static_cast<unsigned long>(bnoProbeFailureCount));
            
            // GPS-Details
            gpsManager.printDiagnostics();
            
            // Buffer-Statistiken
            printBufferStats();
        }
    }
    
    // Kurze Pause um CPU zu entlasten
    delay(1);
}

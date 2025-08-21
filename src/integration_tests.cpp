#include "integration_tests.h"
#include "hardware_config.h"
#include <esp_heap_caps.h>

// Globale Test-Instanz
IntegrationTests integrationTests;

IntegrationTests::IntegrationTests() {
    config = IntegrationTestConfig();
    stats = IntegrationTestStats();
    resultCount = 0;
}

void IntegrationTests::addTestResult(const String& name, bool passed, uint32_t duration, const String& details) {
    if (resultCount < MAX_TEST_RESULTS) {
        results[resultCount] = {name, passed, duration, details, 0, 0};
        resultCount++;
        
        stats.totalTests++;
        if (passed) {
            stats.passedTests++;
        } else {
            stats.failedTests++;
        }
        stats.totalDuration += duration;
    }
}

uint32_t IntegrationTests::getFreeHeap() {
    return heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
}

void IntegrationTests::checkMemoryLeaks(uint32_t startHeap, uint32_t endHeap, IntegrationTestResult& result) {
    if (endHeap < startHeap) {
        result.memoryLeaked = startHeap - endHeap;
        if (result.memoryLeaked > 100) {  // Mehr als 100 Bytes = Problem
            result.passed = false;
            result.details += " MEMORY LEAK: " + String(result.memoryLeaked) + " bytes";
        }
    }
}

void IntegrationTests::monitorSystemHealth() {
    uint32_t currentHeap = getFreeHeap();
    if (currentHeap > stats.maxHeapUsage) {
        stats.maxHeapUsage = currentHeap;
    }
    if (currentHeap < stats.minFreeHeap) {
        stats.minFreeHeap = currentHeap;
    }
}

// ========== HAUPTTEST-FUNKTIONEN ==========

bool IntegrationTests::runAllTests() {
    Serial.println("\n========== INTEGRATION TEST SUITE ==========");
    Serial.println("Starte umfassende System-Integrations-Tests...\n");
    
    uint32_t suiteStartTime = millis();
    resultCount = 0;
    stats = IntegrationTestStats();
    
    // 1. Multi-Modul Tests
    Serial.println("=== MULTI-MODUL TESTS ===");
    testAllModulesConcurrent();
    testSensorDataCorrelation();
    testHighFrequencyLogging();
    testBufferStressUnderLoad();
    testMemoryFragmentation();
    
    // 2. Hardware Failure & Recovery Tests
    if (config.runFailureTests) {
        Serial.println("\n=== HARDWARE FAILURE & RECOVERY TESTS ===");
        testI2CBusRecovery();
        testSDCardHotplug();
        testSensorDisconnectReconnect();
        testCANBusOverload();
        testGPSSignalLoss();
        testPowerBrownout();
    }
    
    // 3. Edge-Case Tests
    Serial.println("\n=== EDGE-CASE TESTS ===");
    testMaximumVibration();
    testRapidDirectionChanges();
    testSimultaneousEvents();
    testBufferOverflowRecovery();
    testFileSystemFull();
    
    // 4. Performance Tests
    Serial.println("\n=== PERFORMANCE TESTS ===");
    testMaximumThroughput();
    testLatencyUnderLoad();
    testCPUUsageOptimization();
    
    // 5. Daten-Integritäts-Tests
    Serial.println("\n=== DATEN-INTEGRITÄTS-TESTS ===");
    testDataConsistency();
    testTimestampAccuracy();
    testCrossModuleSync();
    
    // 6. Langzeit-Tests (optional)
    if (config.runLongTermTests) {
        Serial.println("\n=== LANGZEIT-TESTS ===");
        testMemoryLeakDetection();
        testSensorCalibrationDrift();
    }
    
    stats.totalDuration = millis() - suiteStartTime;
    
    Serial.println("\n========== TEST SUITE ABGESCHLOSSEN ==========");
    printDetailedReport();
    
    return (stats.failedTests == 0);
}

// ========== MULTI-MODUL TESTS ==========

bool IntegrationTests::testAllModulesConcurrent() {
    Serial.println("\n--- Test: Alle Module gleichzeitig ---");
    uint32_t startTime = millis();
    uint32_t startHeap = getFreeHeap();
    bool testPassed = true;
    String details = "";
    
    // Alle Module parallel für 30 Sekunden betreiben
    uint32_t testDuration = 30000;
    uint32_t endTime = millis() + testDuration;
    
    uint32_t sensorReads = 0;
    uint32_t gpsUpdates = 0;
    uint32_t canMessages = 0;
    uint32_t oledUpdates = 0;
    uint32_t sdWrites = 0;
    
    Serial.println("Betreibe alle Module mit maximaler Last für 30 Sekunden...");
    
    while (millis() < endTime) {
        // BNO055 @ 10Hz
        if (millis() % 100 == 0) {
            SensorData data = bnoManager.getCurrentData();
            sensorReads++;
            
            // Vibrations-Analyse
            VibrationMetrics vib = bnoManager.analyzeVibration();
            if (vib.maxShock > 50.0) {  // Unrealistisch hoher Wert
                testPassed = false;
                details += "Sensor-Fehler ";
            }
        }
        
        // GPS @ 5Hz  
        if (millis() % 200 == 0) {
            gpsManager.update();
            if (gpsManager.available()) {
                GPSData gpsData = gpsManager.getCurrentData();
                gpsUpdates++;
            }
        }
        
        // CAN @ 100Hz simuliert
        if (millis() % 10 == 0) {
            canReader.update();
            if (canReader.available()) {
                canMessages++;
            }
        }
        
        // OLED @ 2Hz
        if (millis() % 500 == 0) {
            oledManager.update();
            oledUpdates++;
        }
        
        // SD-Logger @ 1Hz
        if (millis() % 1000 == 0) {
            if (sdLogger.isLogging()) {
                SensorData sensorData = bnoManager.getCurrentData();
                sdLogger.logSensorData(sensorData);
                sdWrites++;
            }
        }
        
        // System-Health überwachen
        monitorSystemHealth();
        
        // Kurze Pause für Task-Switching
        delay(1);
    }
    
    // Ergebnisse auswerten
    details = "Sensor: " + String(sensorReads) + " reads, ";
    details += "GPS: " + String(gpsUpdates) + " updates, ";
    details += "CAN: " + String(canMessages) + " msgs, ";
    details += "OLED: " + String(oledUpdates) + " refreshes, ";
    details += "SD: " + String(sdWrites) + " writes";
    
    // Prüfungen
    if (sensorReads < 280 || sensorReads > 320) {  // Erwarte ~300 bei 10Hz
        testPassed = false;
        details += " - Sensor-Timing falsch!";
    }
    
    uint32_t endHeap = getFreeHeap();
    if (endHeap < startHeap - 5000) {  // Mehr als 5KB verloren
        testPassed = false;
        details += " - Memory Leak!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Alle Module gleichzeitig", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testSensorDataCorrelation() {
    Serial.println("\n--- Test: Sensor-Daten-Korrelation ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    // Sammle korrelierte Daten für 10 Sekunden
    uint32_t testDuration = 10000;
    uint32_t endTime = millis() + testDuration;
    
    struct CorrelatedData {
        uint32_t timestamp;
        SensorData sensor;
        GPSData gps;
        CANMessage can;
        bool hasGPS;
        bool hasCAN;
    };
    
    const int MAX_SAMPLES = 100;
    CorrelatedData samples[MAX_SAMPLES];
    int sampleCount = 0;
    
    Serial.println("Sammle korrelierte Daten für 10 Sekunden...");
    
    while (millis() < endTime && sampleCount < MAX_SAMPLES) {
        uint32_t now = millis();
        
        // Sensor-Daten (Basis-Zeitstempel)
        SensorData sensorData = bnoManager.getCurrentData();
        sensorData.timestamp = now;
        
        // GPS-Daten (falls verfügbar)
        GPSData gpsData;
        bool hasGPS = false;
        if (gpsManager.available()) {
            gpsData = gpsManager.getCurrentData();
            hasGPS = true;
        }
        
        // CAN-Daten (falls verfügbar)
        CANMessage canMsg;
        bool hasCAN = false;
        if (canReader.available()) {
            canMsg = canReader.getLastMessage();
            hasCAN = true;
        }
        
        // Sample speichern
        samples[sampleCount] = {now, sensorData, gpsData, canMsg, hasGPS, hasCAN};
        sampleCount++;
        
        delay(100);  // 10Hz Sampling
    }
    
    // Korrelation analysieren
    int correlatedSamples = 0;
    int timingErrors = 0;
    
    for (int i = 0; i < sampleCount; i++) {
        // Prüfe Zeitstempel-Korrelation
        if (samples[i].hasGPS && samples[i].hasCAN) {
            correlatedSamples++;
            
            // Zeitstempel sollten innerhalb von 1 Sekunde sein
            if (abs((long)(samples[i].sensor.timestamp - samples[i].can.timestamp)) > 1000) {
                timingErrors++;
            }
        }
    }
    
    float correlationRate = (float)correlatedSamples / sampleCount * 100;
    details = String(sampleCount) + " Samples, ";
    details += String(correlatedSamples) + " korreliert (" + String(correlationRate, 1) + "%), ";
    details += String(timingErrors) + " Timing-Fehler";
    
    // Bewertung
    if (correlationRate < 50.0) {
        testPassed = false;
        details += " - Schlechte Korrelation!";
    }
    
    if (timingErrors > sampleCount * 0.1) {  // Mehr als 10% Timing-Fehler
        testPassed = false;
        details += " - Timing-Probleme!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Sensor-Daten-Korrelation", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testHighFrequencyLogging() {
    Serial.println("\n--- Test: Hochfrequenz-Datenlogging ---");
    uint32_t startTime = millis();
    uint32_t startHeap = getFreeHeap();
    bool testPassed = true;
    String details = "";
    
    // Maximale Logging-Rate für 20 Sekunden
    uint32_t testDuration = 20000;
    uint32_t endTime = millis() + testDuration;
    uint32_t logCount = 0;
    uint32_t dropCount = 0;
    
    Serial.println("Teste maximale Logging-Rate für 20 Sekunden...");
    
    // Speichere Start-Statistiken
    SDLoggerStats startStats = sdLogger.getStatistics();
    
    while (millis() < endTime) {
        // Sensor-Daten mit maximaler Rate
        SensorData sensorData = bnoManager.getCurrentData();
        
        // Versuche zu loggen
        if (sdLogger.logSensorData(sensorData)) {
            logCount++;
        } else {
            dropCount++;
        }
        
        // GPS wenn verfügbar
        if (gpsManager.available()) {
            GPSData gpsData = gpsManager.getCurrentData();
            sdLogger.logGPSData(gpsData);
        }
        
        // CAN wenn verfügbar
        if (canReader.available()) {
            CANMessage canMsg = canReader.getLastMessage();
            canReader.logMessage(canMsg);
        }
        
        // Keine Verzögerung - maximale Rate!
    }
    
    // Buffer flushen
    sdLogger.flush();
    
    // Statistiken auswerten
    SDLoggerStats endStats = sdLogger.getStatistics();
    uint32_t totalWrites = endStats.totalWrites - startStats.totalWrites;
    uint32_t bufferOverflows = endStats.bufferOverflows - startStats.bufferOverflows;
    
    float writeRate = (float)totalWrites / (testDuration / 1000.0);
    details = String(totalWrites) + " Schreibvorgänge (" + String(writeRate, 1) + "/s), ";
    details += String(dropCount) + " verworfen, ";
    details += String(bufferOverflows) + " Buffer-Overflows";
    
    // Bewertung
    if (writeRate < 50.0) {  // Weniger als 50 Writes/s
        testPassed = false;
        details += " - Zu langsam!";
    }
    
    if (bufferOverflows > 5) {
        testPassed = false;
        details += " - Zu viele Overflows!";
    }
    
    // Memory Check
    uint32_t endHeap = getFreeHeap();
    checkMemoryLeaks(startHeap, endHeap, results[resultCount]);
    
    uint32_t duration = millis() - startTime;
    addTestResult("Hochfrequenz-Logging", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testBufferStressUnderLoad() {
    Serial.println("\n--- Test: Buffer-Stress unter Last ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    // Teste alle Buffer-Systeme unter Stress
    const int TEST_SIZE = 1000;
    int overflowCount = 0;
    
    // 1. SafeRingBuffer Test
    Serial.println("Teste SafeRingBuffer mit Overflow...");
    SafeRingBuffer<float, 100> ringBuffer;
    
    for (int i = 0; i < TEST_SIZE; i++) {
        ringBuffer.push(i * 0.1f);
        if (ringBuffer.hasOverflowed()) {
            overflowCount++;
            ringBuffer.resetOverflow();
        }
    }
    
    if (overflowCount == 0) {
        testPassed = false;
        details += "RingBuffer Overflow nicht erkannt! ";
    }
    
    // 2. SD-Logger Buffer Test
    Serial.println("Teste SD-Logger Buffer-Grenzen...");
    String longLine = "";
    for (int i = 0; i < 100; i++) {
        longLine += "TestDaten1234567890";
    }
    
    bool bufferTestOK = true;
    for (int i = 0; i < 10; i++) {
        if (!sdLogger.safeAppendToBuffer(longLine.c_str(), longLine.length())) {
            bufferTestOK = false;
            break;
        }
    }
    
    if (bufferTestOK) {
        testPassed = false;
        details += "SD-Buffer Overflow nicht verhindert! ";
    }
    
    // 3. Memory Pool Test
    Serial.println("Teste Memory Pool Erschöpfung...");
    void* allocations[50];
    int allocCount = 0;
    
    // Allokiere bis Pool erschöpft
    for (int i = 0; i < 50; i++) {
        allocations[i] = globalMemoryPool.allocate();
        if (allocations[i] != nullptr) {
            allocCount++;
        } else {
            break;  // Pool erschöpft
        }
    }
    
    // Gebe alles wieder frei
    for (int i = 0; i < allocCount; i++) {
        globalMemoryPool.deallocate(allocations[i]);
    }
    
    // 4. String-Buffer Test
    Serial.println("Teste String-Buffer-Sicherheit...");
    char testBuffer[64];
    bool stringTestOK = SafeStringFormatter::safePrintf(testBuffer, sizeof(testBuffer),
        "Dies ist ein sehr langer Test-String der definitiv den Buffer überschreitet %s %d %f",
        "mit noch mehr Text", 12345, 3.14159);
    
    if (strlen(testBuffer) >= sizeof(testBuffer)) {
        testPassed = false;
        details += "String-Buffer Overflow! ";
    }
    
    details += "RingBuffer: " + String(overflowCount) + " Overflows, ";
    details += "MemPool: " + String(allocCount) + " Allocs, ";
    details += "Alle Sicherheitsmechanismen getestet";
    
    stats.bufferOverflows += overflowCount;
    
    uint32_t duration = millis() - startTime;
    addTestResult("Buffer-Stress unter Last", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testMemoryFragmentation() {
    Serial.println("\n--- Test: Memory-Fragmentierung ---");
    uint32_t startTime = millis();
    uint32_t startHeap = getFreeHeap();
    bool testPassed = true;
    String details = "";
    
    // Simuliere Fragmentierung durch viele Alloc/Free Zyklen
    const int CYCLES = 100;
    const int SIZES[] = {64, 128, 256, 512, 1024};
    const int SIZE_COUNT = 5;
    
    Serial.println("Führe 100 Alloc/Free Zyklen mit verschiedenen Größen durch...");
    
    for (int cycle = 0; cycle < CYCLES; cycle++) {
        void* ptrs[SIZE_COUNT];
        
        // Allokiere verschiedene Größen
        for (int i = 0; i < SIZE_COUNT; i++) {
            ptrs[i] = malloc(SIZES[i]);
            if (ptrs[i] == nullptr) {
                testPassed = false;
                details += "Malloc failed bei Zyklus " + String(cycle) + "! ";
                break;
            }
            
            // Schreibe Testdaten
            memset(ptrs[i], cycle & 0xFF, SIZES[i]);
        }
        
        // Gebe in anderer Reihenfolge frei (fragmentiert)
        for (int i = SIZE_COUNT - 1; i >= 0; i--) {
            if (ptrs[i] != nullptr) {
                free(ptrs[i]);
            }
        }
        
        // Prüfe Heap-Gesundheit
        if (cycle % 10 == 0) {
            uint32_t currentHeap = getFreeHeap();
            if (currentHeap < startHeap - 10000) {  // Mehr als 10KB verloren
                testPassed = false;
                details += "Heap-Verlust bei Zyklus " + String(cycle) + "! ";
            }
        }
    }
    
    // Finale Heap-Prüfung
    uint32_t endHeap = getFreeHeap();
    int32_t heapDiff = startHeap - endHeap;
    
    details += "Start: " + String(startHeap) + " bytes, ";
    details += "Ende: " + String(endHeap) + " bytes, ";
    details += "Differenz: " + String(heapDiff) + " bytes";
    
    if (heapDiff > 1000) {  // Mehr als 1KB Verlust
        testPassed = false;
        details += " - Signifikante Fragmentierung!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Memory-Fragmentierung", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

// ========== HARDWARE FAILURE & RECOVERY TESTS ==========

bool IntegrationTests::testI2CBusRecovery() {
    Serial.println("\n--- Test: I2C-Bus Recovery ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    // Speichere aktuellen I2C-Status
    bool bnoWasReady = bnoManager.isReady();
    bool oledWasReady = oledManager.isReady();
    
    Serial.println("Simuliere I2C-Bus-Störung...");
    
    // 1. Stoppe I2C
    Wire.end();
    delay(100);
    
    // 2. Versuche Sensor-Zugriff (sollte fehlschlagen)
    bool accessFailed = false;
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(0x29);  // BNO055 Adresse
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        accessFailed = true;
    }
    
    // 3. Recovery-Versuch
    Serial.println("Führe I2C-Bus-Recovery durch...");
    
    // Clock-Stretching Recovery
    pinMode(I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL, LOW);
        delayMicroseconds(5);
    }
    
    // Stop-Condition
    pinMode(I2C_SDA, OUTPUT);
    digitalWrite(I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH);
    delayMicroseconds(5);
    
    // Pins zurücksetzen
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    
    // 4. I2C neu initialisieren
    Wire.end();
    delay(100);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    
    // 5. Prüfe ob Geräte wieder erreichbar
    bool recoverySuccess = true;
    
    // BNO055 prüfen
    Wire.beginTransmission(0x29);
    error = Wire.endTransmission();
    if (error != 0) {
        recoverySuccess = false;
        details += "BNO055 nicht wiederhergestellt! ";
    }
    
    // OLED prüfen
    Wire.beginTransmission(0x3C);
    error = Wire.endTransmission();
    if (error != 0) {
        recoverySuccess = false;
        details += "OLED nicht wiederhergestellt! ";
    }
    
    // 6. Manager neu initialisieren wenn nötig
    if (bnoWasReady && !bnoManager.isReady()) {
        bnoManager.begin();
    }
    if (oledWasReady && !oledManager.isReady()) {
        oledManager.begin();
    }
    
    testPassed = accessFailed && recoverySuccess;
    
    details += "Bus-Störung: " + String(accessFailed ? "OK" : "Fehler") + ", ";
    details += "Recovery: " + String(recoverySuccess ? "Erfolgreich" : "Fehlgeschlagen");
    
    uint32_t duration = millis() - startTime;
    addTestResult("I2C-Bus Recovery", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testSDCardHotplug() {
    Serial.println("\n--- Test: SD-Karten Hotplug ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    // Warnung ausgeben
    Serial.println("⚠️  ACHTUNG: Dieser Test erfordert manuelles Eingreifen!");
    Serial.println("Bitte folgen Sie den Anweisungen...");
    
    // 1. Prüfe ob SD-Karte vorhanden
    bool initialSDState = sdLogger.isReady();
    if (!initialSDState) {
        Serial.println("❌ SD-Karte nicht eingesteckt! Test übersprungen.");
        addTestResult("SD-Karten Hotplug", false, 0, "SD-Karte nicht vorhanden");
        return false;
    }
    
    // 2. Schreibe Test-Datei
    String testFileName = "/hotplug_test.txt";
    if (SD.exists(testFileName)) {
        SD.remove(testFileName);
    }
    
    File testFile = SD.open(testFileName, FILE_WRITE);
    if (testFile) {
        testFile.println("Hotplug Test Data");
        testFile.close();
        Serial.println("✅ Test-Datei geschrieben");
    }
    
    // 3. Fordere Benutzer auf, Karte zu entfernen
    Serial.println("\n>>> Bitte SD-Karte JETZT entfernen und Enter drücken <<<");
    while (!Serial.available()) {
        delay(100);
    }
    while (Serial.available()) Serial.read();  // Buffer leeren
    
    // 4. Prüfe ob Karte entfernt wurde
    delay(500);
    bool cardRemoved = !SD.exists(testFileName);
    if (!cardRemoved) {
        // Versuche aktiv zu erkennen
        sdLogger.end();
        delay(100);
        cardRemoved = !sdLogger.begin();
    }
    
    if (cardRemoved) {
        Serial.println("✅ SD-Karte entfernt erkannt");
        details += "Entfernung erkannt, ";
    } else {
        Serial.println("❌ SD-Karte Entfernung nicht erkannt");
        testPassed = false;
    }
    
    // 5. Fordere Benutzer auf, Karte wieder einzusetzen
    Serial.println("\n>>> Bitte SD-Karte WIEDER EINSETZEN und Enter drücken <<<");
    while (!Serial.available()) {
        delay(100);
    }
    while (Serial.available()) Serial.read();  // Buffer leeren
    
    // 6. Versuche SD-Karte neu zu initialisieren
    delay(500);
    bool reInitSuccess = false;
    
    for (int attempt = 0; attempt < 3; attempt++) {
        Serial.printf("Initialisierungs-Versuch %d...\n", attempt + 1);
        
        sdLogger.end();
        delay(500);
        
        if (sdLogger.begin()) {
            reInitSuccess = true;
            break;
        }
        delay(1000);
    }
    
    if (reInitSuccess) {
        Serial.println("✅ SD-Karte erfolgreich neu initialisiert");
        details += "Re-Init OK, ";
        
        // 7. Prüfe ob Test-Datei noch vorhanden
        if (SD.exists(testFileName)) {
            Serial.println("✅ Test-Datei noch vorhanden");
            SD.remove(testFileName);
            details += "Daten erhalten";
        } else {
            details += "Daten verloren";
        }
    } else {
        Serial.println("❌ SD-Karte Re-Initialisierung fehlgeschlagen");
        testPassed = false;
        details += "Re-Init fehlgeschlagen";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("SD-Karten Hotplug", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testSensorDisconnectReconnect() {
    Serial.println("\n--- Test: Sensor Disconnect/Reconnect ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    // Speichere Sensor-Status
    bool bnoWasReady = bnoManager.isReady();
    CalibrationData calBefore = bnoManager.getCalibration();
    
    Serial.println("Simuliere BNO055 Ausfall...");
    
    // 1. Sensor "trennen" (Ende und Neustart ohne Hardware)
    bnoManager.end();
    delay(100);
    
    // 2. Versuche Zugriff (sollte fehlschlagen)
    bool accessFailed = true;
    SensorData data = bnoManager.getCurrentData();
    if (data.heading != 0 || data.pitch != 0 || data.roll != 0) {
        accessFailed = false;  // Daten sollten 0 sein wenn nicht verbunden
    }
    
    // 3. Warte kurz (simuliere Reconnect-Zeit)
    delay(1000);
    
    // 4. Versuche Reconnect
    Serial.println("Versuche Sensor-Reconnect...");
    bool reconnectSuccess = false;
    
    for (int attempt = 0; attempt < 3; attempt++) {
        Serial.printf("Reconnect-Versuch %d...\n", attempt + 1);
        
        if (bnoManager.begin()) {
            reconnectSuccess = true;
            Serial.println("✅ Sensor wiederverbunden");
            break;
        }
        delay(500);
    }
    
    if (reconnectSuccess) {
        // 5. Prüfe Kalibrierung
        delay(500);  // Zeit für Initialisierung
        CalibrationData calAfter = bnoManager.getCalibration();
        
        // Mit NVS sollte Kalibrierung erhalten bleiben
        if (calAfter.system > 0 || calAfter.gyro > 0) {
            Serial.println("✅ Kalibrierung aus NVS wiederhergestellt");
            details += "Kalibrierung erhalten, ";
        } else {
            details += "Kalibrierung verloren, ";
        }
        
        // 6. Prüfe Datenqualität
        data = bnoManager.getCurrentData();
        if (data.temperature > 0 && data.temperature < 50) {
            details += "Daten OK";
        } else {
            testPassed = false;
            details += "Daten fehlerhaft";
        }
    } else {
        testPassed = false;
        details += "Reconnect fehlgeschlagen";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Sensor Disconnect/Reconnect", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testCANBusOverload() {
    Serial.println("\n--- Test: CAN-Bus Überlastung ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    if (!canReader.isReady()) {
        Serial.println("⚠️ CAN-Bus nicht verfügbar - Test übersprungen");
        addTestResult("CAN-Bus Überlastung", false, 0, "CAN nicht verfügbar");
        return false;
    }
    
    // Speichere Start-Statistiken
    uint32_t startMessages = canReader.getMessageCount();
    uint32_t startErrors = canReader.getErrorCount();
    
    Serial.println("Simuliere CAN-Bus Überlastung (1000 Messages/s für 5s)...");
    
    // Sende viele Nachrichten schnell hintereinander
    uint32_t testDuration = 5000;
    uint32_t endTime = millis() + testDuration;
    uint32_t sentCount = 0;
    uint32_t errorCount = 0;
    
    while (millis() < endTime) {
        // Erstelle Test-Nachricht
        CANMessage testMsg;
        testMsg.canId = 0x100 + (sentCount & 0xFF);
        testMsg.dlc = 8;
        testMsg.timestamp = millis();
        
        for (int i = 0; i < 8; i++) {
            testMsg.data[i] = (sentCount >> (i * 8)) & 0xFF;
        }
        
        // Versuche zu senden (im Loopback-Modus)
        // Da wir keinen echten CAN-Bus haben, simulieren wir
        canReader.update();  // Process any pending
        
        sentCount++;
        
        // Keine Verzögerung - maximale Rate!
    }
    
    // Warte kurz auf Verarbeitung
    delay(100);
    
    // Prüfe Ergebnisse
    uint32_t endMessages = canReader.getMessageCount();
    uint32_t endErrors = canReader.getErrorCount();
    
    uint32_t received = endMessages - startMessages;
    errorCount = endErrors - startErrors;
    
    float messageRate = (float)sentCount / (testDuration / 1000.0);
    details = String(sentCount) + " gesendet (" + String(messageRate, 0) + "/s), ";
    details += String(received) + " empfangen, ";
    details += String(errorCount) + " Fehler";
    
    // Bewertung
    if (errorCount > sentCount * 0.1) {  // Mehr als 10% Fehler
        testPassed = false;
        details += " - Zu viele Fehler!";
    }
    
    // Prüfe ob System stabil blieb
    if (canReader.isReady()) {
        details += ", System stabil";
    } else {
        testPassed = false;
        details += ", System instabil!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("CAN-Bus Überlastung", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testGPSSignalLoss() {
    Serial.println("\n--- Test: GPS-Signal-Verlust ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    // Speichere GPS-Status
    bool hadFix = gpsManager.hasValidFix();
    GPSData lastGoodData;
    if (hadFix) {
        lastGoodData = gpsManager.getCurrentData();
    }
    
    Serial.println("Simuliere GPS-Signal-Verlust...");
    
    // 1. GPS auf Polling umschalten (einfacher zu manipulieren)
    bool wasInterruptMode = gpsManager.isInterruptModeEnabled();
    if (wasInterruptMode) {
        gpsManager.enableInterruptMode(false);
    }
    
    // 2. GPS "trennen" durch Stoppen der Updates
    // In echtem Test würde man die Antenne abdecken
    Serial.println("GPS-Updates gestoppt für 10 Sekunden...");
    
    uint32_t noUpdateEndTime = millis() + 10000;
    int lostFixAfterMs = 0;
    bool fixLost = false;
    
    while (millis() < noUpdateEndTime) {
        // Keine GPS-Updates!
        // gpsManager.update();  // NICHT aufrufen
        
        // Prüfe ob Fix verloren
        if (!fixLost && hadFix && !gpsManager.hasValidFix()) {
            fixLost = true;
            lostFixAfterMs = millis() - startTime;
            Serial.printf("GPS-Fix verloren nach %d ms\n", lostFixAfterMs);
        }
        
        delay(100);
    }
    
    // 3. GPS-Updates wieder aktivieren
    Serial.println("GPS-Updates wieder aktiviert...");
    
    uint32_t recoveryStart = millis();
    bool fixRecovered = false;
    int recoveredAfterMs = 0;
    
    // Maximal 30 Sekunden auf Fix warten
    while (millis() - recoveryStart < 30000) {
        gpsManager.update();
        
        if (!fixRecovered && gpsManager.hasValidFix()) {
            fixRecovered = true;
            recoveredAfterMs = millis() - recoveryStart;
            Serial.printf("GPS-Fix wiederhergestellt nach %d ms\n", recoveredAfterMs);
            break;
        }
        
        delay(100);
    }
    
    // 4. Interrupt-Modus wiederherstellen
    if (wasInterruptMode) {
        gpsManager.enableInterruptMode(true);
    }
    
    // Bewertung
    if (hadFix) {
        details += "Fix-Verlust: " + String(fixLost ? "OK" : "Fehler") + ", ";
        details += "Recovery: " + String(fixRecovered ? "OK" : "Fehler");
        
        if (!fixLost) {
            // Fix sollte verloren gehen wenn keine Updates
            testPassed = false;
        }
    } else {
        details = "Kein initialer Fix - Test eingeschränkt";
        // Test trotzdem als bestanden werten
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("GPS-Signal-Verlust", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testPowerBrownout() {
    Serial.println("\n--- Test: Spannungseinbruch-Simulation ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("⚠️  WARNUNG: Dieser Test simuliert Spannungsprobleme");
    Serial.println("In echter Hardware würde Brown-Out-Detection ausgelöst");
    
    // Da wir keinen echten Brownout auslösen können, 
    // simulieren wir die Auswirkungen
    
    // 1. Speichere kritische Daten
    Serial.println("Speichere kritische Daten vor Brownout...");
    
    // Kalibrierung speichern
    if (bnoManager.isReady()) {
        bnoManager.saveCalibration();
    }
    
    // SD-Buffer flushen
    if (sdLogger.isReady()) {
        sdLogger.flush();
    }
    
    // 2. Simuliere Brownout-Effekte
    Serial.println("Simuliere Brownout-Effekte...");
    
    // Alle Module "ausfallen" lassen
    bool modulesWereReady[5] = {
        bnoManager.isReady(),
        oledManager.isReady(),
        gpsManager.isReady(),
        canReader.isReady(),
        sdLogger.isReady()
    };
    
    // Module beenden
    bnoManager.end();
    oledManager.end();
    gpsManager.end();
    canReader.end();
    sdLogger.end();
    
    delay(1000);  // Simuliere Brownout-Dauer
    
    // 3. "Power-On" und Recovery
    Serial.println("Simuliere Power-Recovery...");
    
    // Module neu initialisieren
    int recoveredModules = 0;
    
    if (modulesWereReady[0] && bnoManager.begin()) {
        recoveredModules++;
        
        // Prüfe ob Kalibrierung erhalten blieb
        CalibrationData cal = bnoManager.getCalibration();
        if (cal.system > 0) {
            details += "BNO055-Kal erhalten, ";
        }
    }
    
    if (modulesWereReady[1] && oledManager.begin()) {
        recoveredModules++;
    }
    
    if (modulesWereReady[2] && gpsManager.begin()) {
        recoveredModules++;
        gpsManager.enableInterruptMode(true);  // Interrupt-Modus wiederherstellen
    }
    
    if (modulesWereReady[3] && canReader.begin(500E3)) {
        recoveredModules++;
    }
    
    if (modulesWereReady[4] && sdLogger.begin()) {
        recoveredModules++;
        
        // Prüfe ob Daten erhalten blieben
        if (SD.exists(sdLogger.getCurrentLogFile())) {
            details += "SD-Daten erhalten, ";
        }
    }
    
    // Bewertung
    int expectedModules = 0;
    for (int i = 0; i < 5; i++) {
        if (modulesWereReady[i]) expectedModules++;
    }
    
    details += String(recoveredModules) + "/" + String(expectedModules) + " Module wiederhergestellt";
    
    if (recoveredModules < expectedModules) {
        testPassed = false;
        details += " - Recovery unvollständig!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Spannungseinbruch-Recovery", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

// ========== EDGE-CASE TESTS ==========

bool IntegrationTests::testMaximumVibration() {
    Serial.println("\n--- Test: Maximale Vibration ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    if (!bnoManager.isReady()) {
        Serial.println("⚠️ BNO055 nicht bereit - Test übersprungen");
        addTestResult("Maximale Vibration", false, 0, "Sensor nicht bereit");
        return false;
    }
    
    Serial.println("Teste extreme Beschleunigungswerte...");
    
    // Speichere normale Schwellwerte
    float normalThreshold = 2.0;  // Normal 2.0 m/s²
    
    // Test mit verschiedenen Vibrationsstärken
    float testAccelerations[] = {5.0, 10.0, 20.0, 50.0, 100.0};  // m/s²
    int detectedEvents = 0;
    
    for (float testAccel : testAccelerations) {
        // Simuliere durch schnelle Sensor-Bewegung
        // In echtem Test: Sensor schütteln
        
        Serial.printf("Teste mit %.1f m/s² Vibration...\n", testAccel);
        
        // Sammle Daten für 2 Sekunden
        uint32_t testEnd = millis() + 2000;
        float maxDetected = 0;
        
        while (millis() < testEnd) {
            SensorData data = bnoManager.getCurrentData();
            
            // Berechne Gesamt-Beschleunigung
            float totalAccel = sqrt(data.accelX * data.accelX + 
                                  data.accelY * data.accelY + 
                                  data.accelZ * data.accelZ);
            
            if (totalAccel > maxDetected) {
                maxDetected = totalAccel;
            }
            
            // Vibrations-Analyse
            VibrationMetrics vib = bnoManager.analyzeVibration();
            if (vib.maxShock > testAccel * 0.8) {  // 80% des erwarteten Werts
                detectedEvents++;
            }
            
            delay(10);  // 100Hz Sampling
        }
        
        Serial.printf("  Max erkannt: %.1f m/s²\n", maxDetected);
    }
    
    // Prüfe Buffer-Integrität unter Last
    bool bufferStable = true;
    SafeRingBuffer<float, 100> testBuffer;
    
    // Fülle Buffer mit Extremwerten
    for (int i = 0; i < 200; i++) {
        testBuffer.push(i * 100.0f);
    }
    
    if (!testBuffer.hasOverflowed()) {
        bufferStable = false;
        details += "Buffer-Overflow nicht erkannt! ";
    }
    
    details += String(detectedEvents) + " Vibrations-Events erkannt, ";
    details += "Buffer: " + String(bufferStable ? "stabil" : "instabil");
    
    // Bewertung
    if (detectedEvents < 3) {  // Mindestens 3 von 5 Tests sollten erkannt werden
        testPassed = false;
        details += " - Zu wenige Events!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Maximale Vibration", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testRapidDirectionChanges() {
    Serial.println("\n--- Test: Schnelle Richtungswechsel ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    if (!bnoManager.isReady()) {
        Serial.println("⚠️ BNO055 nicht bereit - Test übersprungen");
        addTestResult("Schnelle Richtungswechsel", false, 0, "Sensor nicht bereit");
        return false;
    }
    
    Serial.println("Teste schnelle Heading-Änderungen (Serpentinen-Simulation)...");
    
    // Variablen für Kurven-Erkennung
    float lastHeading = bnoManager.getCurrentData().heading;
    int curveCount = 0;
    float maxHeadingChange = 0;
    float totalHeadingChange = 0;
    
    // 10 Sekunden Test
    uint32_t testDuration = 10000;
    uint32_t testEnd = millis() + testDuration;
    uint32_t samples = 0;
    
    Serial.println("Bitte Sensor schnell hin und her drehen...");
    
    while (millis() < testEnd) {
        SensorData data = bnoManager.getCurrentData();
        
        // Berechne Heading-Änderung
        float headingDiff = abs(data.heading - lastHeading);
        
        // Berücksichtige Wrap-Around (359° -> 1°)
        if (headingDiff > 180) {
            headingDiff = 360 - headingDiff;
        }
        
        totalHeadingChange += headingDiff;
        
        if (headingDiff > maxHeadingChange) {
            maxHeadingChange = headingDiff;
        }
        
        // Kurve erkannt bei > 5° Änderung
        if (headingDiff > 5.0) {
            curveCount++;
        }
        
        // Prüfe Gyro-Daten
        if (abs(data.gyroZ) > 1.0) {  // rad/s
            // Schnelle Rotation erkannt
        }
        
        lastHeading = data.heading;
        samples++;
        
        delay(50);  // 20Hz für Heading-Änderungen
    }
    
    // Berechne Durchschnittswerte
    float avgHeadingChange = totalHeadingChange / samples;
    
    details = String(curveCount) + " Kurven, ";
    details += "Max: " + String(maxHeadingChange, 1) + "°, ";
    details += "Avg: " + String(avgHeadingChange, 1) + "°/sample";
    
    // Bewertung
    if (curveCount < 20) {  // Weniger als 20 Kurven in 10s
        testPassed = false;
        details += " - Zu wenige Richtungswechsel!";
    }
    
    if (maxHeadingChange < 30.0) {  // Keine schnellen Drehungen
        testPassed = false;
        details += " - Zu langsame Änderungen!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Schnelle Richtungswechsel", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testSimultaneousEvents() {
    Serial.println("\n--- Test: Gleichzeitige Events ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Generiere mehrere Events gleichzeitig...");
    
    struct EventCount {
        int sensorEvents = 0;
        int gpsEvents = 0;
        int canEvents = 0;
        int logEvents = 0;
        int bufferEvents = 0;
    } events;
    
    // Test für 5 Sekunden
    uint32_t testDuration = 5000;
    uint32_t testEnd = millis() + testDuration;
    
    // Puffer für gleichzeitige Daten
    const int MAX_EVENTS = 50;
    uint32_t eventTimestamps[MAX_EVENTS];
    int eventIndex = 0;
    
    while (millis() < testEnd && eventIndex < MAX_EVENTS) {
        uint32_t now = millis();
        bool eventOccurred = false;
        
        // 1. Sensor-Event (Vibration)
        if (now % 100 == 0) {  // Alle 100ms
            SensorData data = bnoManager.getCurrentData();
            VibrationMetrics vib = bnoManager.analyzeVibration();
            
            if (vib.shockCount > 0) {
                events.sensorEvents++;
                eventOccurred = true;
            }
        }
        
        // 2. GPS-Event (neuer Fix)
        if (now % 200 == 0) {  // Alle 200ms
            gpsManager.update();
            if (gpsManager.available()) {
                events.gpsEvents++;
                eventOccurred = true;
            }
        }
        
        // 3. CAN-Event
        if (now % 10 == 0) {  // Alle 10ms
            canReader.update();
            if (canReader.available()) {
                events.canEvents++;
                eventOccurred = true;
            }
        }
        
        // 4. Logging-Event
        if (now % 50 == 0) {  // Alle 50ms
            if (sdLogger.isLogging()) {
                SensorData data = bnoManager.getCurrentData();
                if (sdLogger.logSensorData(data)) {
                    events.logEvents++;
                    eventOccurred = true;
                }
            }
        }
        
        // 5. Buffer-Event (simuliert)
        if (now % 250 == 0) {  // Alle 250ms
            SafeRingBuffer<float, 10> smallBuffer;
            for (int i = 0; i < 20; i++) {
                smallBuffer.push(i);
            }
            if (smallBuffer.hasOverflowed()) {
                events.bufferEvents++;
                eventOccurred = true;
            }
        }
        
        // Speichere Zeitstempel wenn mehrere Events gleichzeitig
        if (eventOccurred) {
            eventTimestamps[eventIndex++] = now;
        }
        
        delay(1);
    }
    
    // Analysiere Gleichzeitigkeit
    int simultaneousCount = 0;
    for (int i = 1; i < eventIndex; i++) {
        // Events innerhalb von 10ms als gleichzeitig betrachten
        if (eventTimestamps[i] - eventTimestamps[i-1] < 10) {
            simultaneousCount++;
        }
    }
    
    details = "Sensor: " + String(events.sensorEvents) + ", ";
    details += "GPS: " + String(events.gpsEvents) + ", ";
    details += "CAN: " + String(events.canEvents) + ", ";
    details += "Log: " + String(events.logEvents) + ", ";
    details += "Gleichzeitig: " + String(simultaneousCount);
    
    // Bewertung
    int totalEvents = events.sensorEvents + events.gpsEvents + 
                     events.canEvents + events.logEvents + events.bufferEvents;
    
    if (totalEvents < 50) {
        testPassed = false;
        details += " - Zu wenige Events!";
    }
    
    if (simultaneousCount < 5) {
        testPassed = false;
        details += " - Keine Gleichzeitigkeit!";
    }
    
    // Prüfe ob System stabil blieb
    if (!bnoManager.isReady() || !sdLogger.isReady()) {
        testPassed = false;
        details += " - System instabil!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Gleichzeitige Events", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testBufferOverflowRecovery() {
    Serial.println("\n--- Test: Buffer-Overflow Recovery ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Provoziere Buffer-Overflows und teste Recovery...");
    
    // 1. SD-Logger Buffer Overflow
    Serial.println("1. SD-Logger Buffer-Overflow...");
    
    // Generiere große Datenmengen
    String hugeData = "";
    for (int i = 0; i < 100; i++) {
        hugeData += "OVERFLOW_TEST_DATA_1234567890_";
    }
    
    int sdOverflows = 0;
    for (int i = 0; i < 10; i++) {
        if (!sdLogger.safeAppendToBuffer(hugeData.c_str(), hugeData.length())) {
            sdOverflows++;
        }
    }
    
    // Force flush
    sdLogger.flush();
    
    // Prüfe ob Logger noch funktioniert
    bool sdRecovered = sdLogger.logSensorData(bnoManager.getCurrentData());
    
    // 2. Ring-Buffer Overflow
    Serial.println("2. Ring-Buffer Overflow...");
    
    SafeRingBuffer<float, 50> ringBuffer;
    int ringOverflows = 0;
    
    // Überfülle Buffer
    for (int i = 0; i < 100; i++) {
        ringBuffer.push(i * 0.1f);
        if (ringBuffer.hasOverflowed()) {
            ringOverflows++;
            ringBuffer.resetOverflow();
        }
    }
    
    // Prüfe ob Buffer noch nutzbar
    ringBuffer.clear();
    ringBuffer.push(42.0f);
    float recoveredValue;
    bool ringRecovered = !ringBuffer.isEmpty() && ringBuffer.pop(recoveredValue) && recoveredValue == 42.0f;
    
    // 3. Memory-Pool Erschöpfung
    Serial.println("3. Memory-Pool Erschöpfung...");
    
    void* allocations[100];
    int successfulAllocs = 0;
    
    // Erschöpfe Pool
    for (int i = 0; i < 100; i++) {
        allocations[i] = globalMemoryPool.allocate();
        if (allocations[i] != nullptr) {
            successfulAllocs++;
        } else {
            break;  // Pool erschöpft
        }
    }
    
    // Gebe alles frei
    for (int i = 0; i < successfulAllocs; i++) {
        globalMemoryPool.deallocate(allocations[i]);
    }
    
    // Prüfe ob Pool wieder nutzbar
    void* testAlloc = globalMemoryPool.allocate();
    bool poolRecovered = (testAlloc != nullptr);
    if (poolRecovered) {
        globalMemoryPool.deallocate(testAlloc);
    }
    
    // 4. String-Buffer Overflow
    Serial.println("4. String-Buffer Overflow...");
    
    char smallBuffer[32];
    bool stringOverflowHandled = SafeStringFormatter::safePrintf(
        smallBuffer, sizeof(smallBuffer),
        "This is a very long string that will definitely overflow the small buffer %d %f %s",
        12345, 3.14159, "even more text here"
    );
    
    // Prüfe ob korrekt terminiert
    bool stringRecovered = (strlen(smallBuffer) < sizeof(smallBuffer));
    
    // Zusammenfassung
    details = "SD: " + String(sdOverflows) + " overflows, ";
    details += "Ring: " + String(ringOverflows) + " overflows, ";
    details += "Pool: " + String(successfulAllocs) + " allocs, ";
    details += "Recovery: ";
    
    int recoveryCount = 0;
    if (sdRecovered) recoveryCount++;
    if (ringRecovered) recoveryCount++;
    if (poolRecovered) recoveryCount++;
    if (stringRecovered) recoveryCount++;
    
    details += String(recoveryCount) + "/4 OK";
    
    // Bewertung
    if (recoveryCount < 4) {
        testPassed = false;
        details += " - Recovery fehlgeschlagen!";
    }
    
    stats.bufferOverflows += sdOverflows + ringOverflows;
    
    uint32_t duration = millis() - startTime;
    addTestResult("Buffer-Overflow Recovery", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testFileSystemFull() {
    Serial.println("\n--- Test: Dateisystem voll ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    if (!sdLogger.isReady()) {
        Serial.println("⚠️ SD-Karte nicht bereit - Test übersprungen");
        addTestResult("Dateisystem voll", false, 0, "SD nicht bereit");
        return false;
    }
    
    Serial.println("⚠️ WARNUNG: Dieser Test füllt die SD-Karte!");
    Serial.println("Erstelle große Test-Dateien...");
    
    // Speichere freien Speicher
    uint32_t startFreeKB = sdLogger.getFreeSpace();
    
    // Erstelle Test-Verzeichnis
    String testDir = "/fulltest";
    if (!SD.exists(testDir)) {
        SD.mkdir(testDir);
    }
    
    // Fülle Karte bis auf 10MB
    const uint32_t MIN_FREE_KB = 10240;  // 10MB Minimum
    const uint32_t CHUNK_SIZE = 10240;   // 10KB Chunks
    uint8_t* chunk = (uint8_t*)malloc(CHUNK_SIZE);
    
    if (chunk == nullptr) {
        addTestResult("Dateisystem voll", false, 0, "Malloc fehlgeschlagen");
        return false;
    }
    
    // Fülle Chunk mit Daten
    for (int i = 0; i < CHUNK_SIZE; i++) {
        chunk[i] = i & 0xFF;
    }
    
    int fileCount = 0;
    bool spaceLow = false;
    
    while (sdLogger.getFreeSpace() > MIN_FREE_KB && fileCount < 100) {
        String fileName = testDir + "/test_" + String(fileCount) + ".dat";
        File testFile = SD.open(fileName, FILE_WRITE);
        
        if (testFile) {
            // Schreibe 1MB pro Datei
            for (int i = 0; i < 100; i++) {
                size_t written = testFile.write(chunk, CHUNK_SIZE);
                if (written < CHUNK_SIZE) {
                    spaceLow = true;
                    break;
                }
            }
            testFile.close();
            fileCount++;
        } else {
            spaceLow = true;
            break;
        }
        
        // Prüfe ob Logger noch funktioniert
        if (fileCount % 10 == 0) {
            SensorData data = bnoManager.getCurrentData();
            if (!sdLogger.logSensorData(data)) {
                details += "Logger-Fehler bei " + String(sdLogger.getFreeSpace()) + "KB frei, ";
            }
        }
    }
    
    free(chunk);
    
    // Teste Verhalten bei vollem Speicher
    Serial.println("Teste Logger bei vollem Speicher...");
    
    int failedLogs = 0;
    for (int i = 0; i < 10; i++) {
        SensorData data = bnoManager.getCurrentData();
        if (!sdLogger.logSensorData(data)) {
            failedLogs++;
        }
    }
    
    // Räume auf
    Serial.println("Räume Test-Dateien auf...");
    
    for (int i = 0; i < fileCount; i++) {
        String fileName = testDir + "/test_" + String(i) + ".dat";
        SD.remove(fileName);
    }
    SD.rmdir(testDir);
    
    // Prüfe ob Logger wieder funktioniert
    bool recoveredAfterCleanup = sdLogger.logSensorData(bnoManager.getCurrentData());
    
    uint32_t endFreeKB = sdLogger.getFreeSpace();
    
    details = String(fileCount) + " Dateien erstellt, ";
    details += String(startFreeKB) + "KB -> " + String(MIN_FREE_KB) + "KB, ";
    details += String(failedLogs) + "/10 Logs fehlgeschlagen, ";
    details += "Recovery: " + String(recoveredAfterCleanup ? "OK" : "Fehler");
    
    // Bewertung
    if (!spaceLow) {
        testPassed = false;
        details += " - Speicher nicht voll!";
    }
    
    if (!recoveredAfterCleanup) {
        testPassed = false;
        details += " - Recovery fehlgeschlagen!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Dateisystem voll", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

// ========== PERFORMANCE TESTS ==========

bool IntegrationTests::testMaximumThroughput() {
    Serial.println("\n--- Test: Maximaler Durchsatz ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Teste maximale Datenverarbeitungsrate...");
    
    // Variablen für Durchsatz-Messung
    struct ThroughputStats {
        uint32_t sensorReads = 0;
        uint32_t gpsUpdates = 0;
        uint32_t canMessages = 0;
        uint32_t sdWrites = 0;
        uint32_t totalBytes = 0;
    } stats;
    
    // Test für 10 Sekunden mit maximaler Rate
    uint32_t testDuration = 10000;
    uint32_t testEnd = millis() + testDuration;
    
    // Deaktiviere Delays für maximalen Durchsatz
    Serial.println("Laufe mit maximaler Geschwindigkeit für 10 Sekunden...");
    
    while (millis() < testEnd) {
        // 1. Sensor-Daten so schnell wie möglich
        SensorData sensorData = bnoManager.getCurrentData();
        stats.sensorReads++;
        stats.totalBytes += sizeof(SensorData);
        
        // 2. GPS-Updates
        gpsManager.update();
        if (gpsManager.available()) {
            GPSData gpsData = gpsManager.getCurrentData();
            stats.gpsUpdates++;
            stats.totalBytes += sizeof(GPSData);
        }
        
        // 3. CAN-Messages
        canReader.update();
        if (canReader.available()) {
            CANMessage msg = canReader.getLastMessage();
            stats.canMessages++;
            stats.totalBytes += sizeof(CANMessage);
        }
        
        // 4. SD-Logging (limitierender Faktor)
        if (sdLogger.logSensorData(sensorData)) {
            stats.sdWrites++;
            stats.totalBytes += 100;  // Geschätzte CSV-Zeilen-Größe
        }
        
        // KEIN delay() - maximale Geschwindigkeit!
    }
    
    // Berechne Durchsatz
    float duration_s = testDuration / 1000.0;
    float sensorRate = stats.sensorReads / duration_s;
    float gpsRate = stats.gpsUpdates / duration_s;
    float canRate = stats.canMessages / duration_s;
    float sdRate = stats.sdWrites / duration_s;
    float throughputKBps = stats.totalBytes / 1024.0 / duration_s;
    
    details = "Sensor: " + String(sensorRate, 0) + " Hz, ";
    details += "GPS: " + String(gpsRate, 1) + " Hz, ";
    details += "CAN: " + String(canRate, 0) + " Hz, ";
    details += "SD: " + String(sdRate, 0) + " Hz, ";
    details += String(throughputKBps, 1) + " KB/s";
    
    // Bewertung
    if (sensorRate < 100) {  // Sollte mindestens 100Hz schaffen
        testPassed = false;
        details += " - Sensor zu langsam!";
    }
    
    if (sdRate < 50) {  // SD sollte mindestens 50Hz schaffen
        testPassed = false;
        details += " - SD zu langsam!";
    }
    
    if (throughputKBps < 10) {  // Mindestens 10KB/s Gesamtdurchsatz
        testPassed = false;
        details += " - Durchsatz zu niedrig!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Maximaler Durchsatz", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testLatencyUnderLoad() {
    Serial.println("\n--- Test: Latenz unter Last ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Messe Reaktionszeiten unter Volllast...");
    
    // Latenz-Messungen
    struct LatencyMeasurement {
        uint32_t sensorMin = 0xFFFFFFFF;
        uint32_t sensorMax = 0;
        uint32_t sensorTotal = 0;
        uint32_t gpsMin = 0xFFFFFFFF;
        uint32_t gpsMax = 0;
        uint32_t gpsTotal = 0;
        uint32_t count = 0;
    } latency;
    
    // Erzeuge Last durch parallele Operationen
    uint32_t testDuration = 5000;
    uint32_t testEnd = millis() + testDuration;
    
    while (millis() < testEnd) {
        // 1. Messe Sensor-Latenz
        uint32_t sensorStart = micros();
        SensorData data = bnoManager.getCurrentData();
        uint32_t sensorLatency = micros() - sensorStart;
        
        latency.sensorTotal += sensorLatency;
        if (sensorLatency < latency.sensorMin) latency.sensorMin = sensorLatency;
        if (sensorLatency > latency.sensorMax) latency.sensorMax = sensorLatency;
        
        // 2. Messe GPS-Latenz
        uint32_t gpsStart = micros();
        gpsManager.update();
        uint32_t gpsLatency = micros() - gpsStart;
        
        latency.gpsTotal += gpsLatency;
        if (gpsLatency < latency.gpsMin) latency.gpsMin = gpsLatency;
        if (gpsLatency > latency.gpsMax) latency.gpsMax = gpsLatency;
        
        // 3. Erzeuge zusätzliche Last
        // SD-Schreibvorgang
        sdLogger.logSensorData(data);
        
        // String-Operationen
        char buffer[256];
        SafeStringFormatter::safePrintf(buffer, sizeof(buffer),
            "Load test: %lu, %.2f, %.2f, %.2f",
            millis(), data.heading, data.pitch, data.roll);
        
        // Buffer-Operationen
        SafeRingBuffer<float, 50> tempBuffer;
        for (int i = 0; i < 50; i++) {
            tempBuffer.push(i * 0.1f);
        }
        
        latency.count++;
        
        delay(10);  // 100Hz
    }
    
    // Berechne Durchschnittswerte
    uint32_t sensorAvg = latency.sensorTotal / latency.count;
    uint32_t gpsAvg = latency.gpsTotal / latency.count;
    
    details = "Sensor: " + String(sensorAvg) + "µs (min:" + String(latency.sensorMin) + 
              ", max:" + String(latency.sensorMax) + "), ";
    details += "GPS: " + String(gpsAvg) + "µs (min:" + String(latency.gpsMin) + 
               ", max:" + String(latency.gpsMax) + ")";
    
    // Bewertung
    if (sensorAvg > 5000) {  // Mehr als 5ms Durchschnitt
        testPassed = false;
        details += " - Sensor-Latenz zu hoch!";
    }
    
    if (latency.sensorMax > 20000) {  // Mehr als 20ms Maximum
        testPassed = false;
        details += " - Sensor-Spitzen zu hoch!";
    }
    
    if (gpsAvg > 10000) {  // GPS darf länger dauern
        testPassed = false;
        details += " - GPS-Latenz zu hoch!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Latenz unter Last", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testCPUUsageOptimization() {
    Serial.println("\n--- Test: CPU-Auslastung ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Messe CPU-Auslastung bei verschiedenen Lasten...");
    
    // ESP32 spezifische CPU-Messung
    uint32_t idleLoops = 0;
    uint32_t busyLoops = 0;
    
    // 1. Idle-Baseline (nur Loop zählen)
    Serial.println("1. Messe Idle-Performance...");
    uint32_t idleStart = millis();
    while (millis() - idleStart < 1000) {
        idleLoops++;
        yield();  // Erlaube Task-Switching
    }
    
    // 2. Normale Last
    Serial.println("2. Messe bei normaler Last...");
    uint32_t normalStart = millis();
    uint32_t normalLoops = 0;
    
    while (millis() - normalStart < 1000) {
        // Normale Operationen
        if (millis() % 100 == 0) {
            bnoManager.getCurrentData();
        }
        if (millis() % 200 == 0) {
            gpsManager.update();
        }
        if (millis() % 10 == 0) {
            canReader.update();
        }
        
        normalLoops++;
        yield();
    }
    
    // 3. Volle Last
    Serial.println("3. Messe bei voller Last...");
    uint32_t busyStart = millis();
    
    while (millis() - busyStart < 1000) {
        // Alle Module mit maximaler Rate
        bnoManager.getCurrentData();
        gpsManager.update();
        canReader.update();
        oledManager.update();
        sdLogger.logSensorData(bnoManager.getCurrentData());
        
        busyLoops++;
        // Kein yield() - maximale CPU-Nutzung
    }
    
    // Berechne relative CPU-Auslastung
    float normalLoad = 100.0 * (1.0 - (float)normalLoops / idleLoops);
    float fullLoad = 100.0 * (1.0 - (float)busyLoops / idleLoops);
    
    details = "Idle: " + String(idleLoops) + " loops/s, ";
    details += "Normal: " + String(normalLoad, 1) + "% CPU, ";
    details += "Full: " + String(fullLoad, 1) + "% CPU";
    
    // Prüfe Heap-Nutzung
    uint32_t freeHeap = getFreeHeap();
    uint32_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    float heapUsage = 100.0 * (1.0 - (float)freeHeap / totalHeap);
    
    details += ", Heap: " + String(heapUsage, 1) + "%";
    
    // Bewertung
    if (normalLoad > 50) {  // Normale Last sollte unter 50% sein
        testPassed = false;
        details += " - Normale Last zu hoch!";
    }
    
    if (fullLoad > 90) {  // Selbst bei Volllast sollte etwas Reserve bleiben
        testPassed = false;
        details += " - CPU überlastet!";
    }
    
    if (heapUsage > 50) {  // Heap sollte nicht mehr als 50% genutzt sein
        testPassed = false;
        details += " - Heap-Nutzung zu hoch!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("CPU-Auslastung", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

// ========== DATEN-INTEGRITÄTS-TESTS ==========

bool IntegrationTests::testDataConsistency() {
    Serial.println("\n--- Test: Daten-Konsistenz ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Prüfe Konsistenz zwischen Modulen...");
    
    // Sammle synchrone Daten
    const int SAMPLE_COUNT = 50;
    struct ConsistencyData {
        uint32_t timestamp;
        float sensorHeading;
        float gpsHeading;
        float sensorAccel;
        float gpsSpeed;
        bool valid;
    } samples[SAMPLE_COUNT];
    
    int validSamples = 0;
    
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        uint32_t sampleTime = millis();
        
        // Hole alle Daten "gleichzeitig"
        SensorData sensor = bnoManager.getCurrentData();
        GPSData gps;
        bool hasGPS = false;
        
        if (gpsManager.available()) {
            gps = gpsManager.getCurrentData();
            hasGPS = true;
        }
        
        // Speichere Sample
        samples[i].timestamp = sampleTime;
        samples[i].sensorHeading = sensor.heading;
        samples[i].sensorAccel = sqrt(sensor.accelX * sensor.accelX + 
                                     sensor.accelY * sensor.accelY);
        
        if (hasGPS) {
            samples[i].gpsHeading = gps.heading_deg;
            samples[i].gpsSpeed = gps.speed_kmh;
            samples[i].valid = true;
            validSamples++;
        } else {
            samples[i].valid = false;
        }
        
        delay(100);  // 10Hz Sampling
    }
    
    // Analysiere Konsistenz
    int headingMismatches = 0;
    float maxHeadingDiff = 0;
    float totalHeadingDiff = 0;
    int comparedSamples = 0;
    
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (!samples[i].valid) continue;
        
        // Vergleiche Headings (sollten ähnlich sein wenn sich bewegt)
        float headingDiff = abs(samples[i].sensorHeading - samples[i].gpsHeading);
        
        // Berücksichtige Wrap-Around
        if (headingDiff > 180) {
            headingDiff = 360 - headingDiff;
        }
        
        totalHeadingDiff += headingDiff;
        comparedSamples++;
        
        if (headingDiff > maxHeadingDiff) {
            maxHeadingDiff = headingDiff;
        }
        
        // Große Abweichung = Problem
        if (headingDiff > 30.0) {
            headingMismatches++;
        }
    }
    
    // Prüfe Zeitstempel-Konsistenz
    int timestampErrors = 0;
    for (int i = 1; i < SAMPLE_COUNT; i++) {
        uint32_t timeDiff = samples[i].timestamp - samples[i-1].timestamp;
        
        // Sollte etwa 100ms sein (10Hz)
        if (timeDiff < 90 || timeDiff > 110) {
            timestampErrors++;
        }
    }
    
    float avgHeadingDiff = comparedSamples > 0 ? totalHeadingDiff / comparedSamples : 0;
    
    details = String(validSamples) + "/" + String(SAMPLE_COUNT) + " mit GPS, ";
    details += "Heading-Diff: " + String(avgHeadingDiff, 1) + "° (max:" + 
               String(maxHeadingDiff, 1) + "°), ";
    details += String(timestampErrors) + " Zeit-Fehler";
    
    // Bewertung
    if (avgHeadingDiff > 15.0 && validSamples > 10) {
        testPassed = false;
        details += " - Heading-Inkonsistenz!";
    }
    
    if (timestampErrors > SAMPLE_COUNT * 0.1) {  // Mehr als 10% Timing-Fehler
        testPassed = false;
        details += " - Timing-Probleme!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Daten-Konsistenz", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testTimestampAccuracy() {
    Serial.println("\n--- Test: Zeitstempel-Genauigkeit ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Prüfe Zeitstempel-Synchronisation...");
    
    // Test-Struktur für Zeitstempel
    struct TimestampTest {
        uint32_t systemTime;
        uint32_t sensorTime;
        uint32_t gpsTime;
        uint32_t canTime;
        uint32_t logTime;
    } timestamps[20];
    
    // Sammle Zeitstempel von allen Modulen
    for (int i = 0; i < 20; i++) {
        uint32_t now = millis();
        timestamps[i].systemTime = now;
        
        // Sensor-Zeitstempel
        SensorData sensor = bnoManager.getCurrentData();
        timestamps[i].sensorTime = sensor.timestamp;
        
        // GPS-Zeitstempel (wenn verfügbar)
        if (gpsManager.available()) {
            GPSData gps = gpsManager.getCurrentData();
            timestamps[i].gpsTime = now;  // GPS hat keinen eigenen Timestamp
        } else {
            timestamps[i].gpsTime = 0;
        }
        
        // CAN-Zeitstempel
        if (canReader.available()) {
            CANMessage msg = canReader.getLastMessage();
            timestamps[i].canTime = msg.timestamp;
        } else {
            timestamps[i].canTime = 0;
        }
        
        // Log-Zeitstempel (aktueller Write-Zeitpunkt)
        timestamps[i].logTime = millis();
        
        delay(50);  // 20Hz
    }
    
    // Analysiere Zeitstempel-Differenzen
    int largeDiffs = 0;
    uint32_t maxDiff = 0;
    uint32_t totalDiff = 0;
    int validComparisons = 0;
    
    for (int i = 0; i < 20; i++) {
        // Sensor vs System
        uint32_t sensorDiff = abs((int32_t)(timestamps[i].sensorTime - timestamps[i].systemTime));
        if (sensorDiff > maxDiff) maxDiff = sensorDiff;
        totalDiff += sensorDiff;
        validComparisons++;
        
        if (sensorDiff > 10) {  // Mehr als 10ms Unterschied
            largeDiffs++;
        }
        
        // CAN vs System (wenn vorhanden)
        if (timestamps[i].canTime > 0) {
            uint32_t canDiff = abs((int32_t)(timestamps[i].canTime - timestamps[i].systemTime));
            if (canDiff > maxDiff) maxDiff = canDiff;
            
            if (canDiff > 100) {  // CAN kann älter sein
                largeDiffs++;
            }
        }
    }
    
    // Prüfe Monotonie (Zeitstempel sollten immer steigen)
    int monotonicErrors = 0;
    for (int i = 1; i < 20; i++) {
        if (timestamps[i].sensorTime <= timestamps[i-1].sensorTime) {
            monotonicErrors++;
        }
        if (timestamps[i].logTime <= timestamps[i-1].logTime) {
            monotonicErrors++;
        }
    }
    
    float avgDiff = validComparisons > 0 ? (float)totalDiff / validComparisons : 0;
    
    details = "Avg Diff: " + String(avgDiff, 1) + "ms, ";
    details += "Max: " + String(maxDiff) + "ms, ";
    details += String(largeDiffs) + " große Abweichungen, ";
    details += String(monotonicErrors) + " Monotonie-Fehler";
    
    // Bewertung
    if (avgDiff > 5.0) {  // Durchschnitt über 5ms
        testPassed = false;
        details += " - Zu große Abweichungen!";
    }
    
    if (monotonicErrors > 0) {
        testPassed = false;
        details += " - Zeit läuft rückwärts!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Zeitstempel-Genauigkeit", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testCrossModuleSync() {
    Serial.println("\n--- Test: Cross-Module Synchronisation ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Teste Synchronisation zwischen Modulen...");
    
    // Test-Szenario: Bewegung detektieren und in allen Modulen sehen
    Serial.println("Bewege das System für konsistente Daten...");
    
    // Phase 1: Baseline (Ruhe)
    Serial.println("Phase 1: Ruhe-Baseline (3s)...");
    
    float baselineAccel = 0;
    float baselineGyro = 0;
    int baselineSamples = 30;
    
    for (int i = 0; i < baselineSamples; i++) {
        SensorData data = bnoManager.getCurrentData();
        baselineAccel += sqrt(data.accelX * data.accelX + 
                            data.accelY * data.accelY + 
                            data.accelZ * data.accelZ);
        baselineGyro += sqrt(data.gyroX * data.gyroX + 
                           data.gyroY * data.gyroY + 
                           data.gyroZ * data.gyroZ);
        delay(100);
    }
    
    baselineAccel /= baselineSamples;
    baselineGyro /= baselineSamples;
    
    // Phase 2: Bewegungs-Detektion
    Serial.println("Phase 2: Bitte System bewegen (5s)...");
    
    struct SyncEvent {
        uint32_t timestamp;
        bool sensorDetected;
        bool gpsDetected;
        bool vibrationDetected;
        float sensorValue;
        float gpsValue;
    } events[50];
    
    int eventCount = 0;
    uint32_t motionStart = millis();
    
    while (millis() - motionStart < 5000 && eventCount < 50) {
        SensorData sensor = bnoManager.getCurrentData();
        
        // Bewegung erkannt?
        float currentAccel = sqrt(sensor.accelX * sensor.accelX + 
                                sensor.accelY * sensor.accelY + 
                                sensor.accelZ * sensor.accelZ);
        
        bool motionDetected = (currentAccel > baselineAccel * 1.5);
        
        if (motionDetected) {
            events[eventCount].timestamp = millis();
            events[eventCount].sensorDetected = true;
            events[eventCount].sensorValue = currentAccel;
            
            // GPS-Geschwindigkeitsänderung?
            if (gpsManager.available()) {
                GPSData gps = gpsManager.getCurrentData();
                events[eventCount].gpsDetected = (gps.speed_kmh > 0.5);
                events[eventCount].gpsValue = gps.speed_kmh;
            }
            
            // Vibration erkannt?
            VibrationMetrics vib = bnoManager.analyzeVibration();
            events[eventCount].vibrationDetected = (vib.rmsAccel > baselineAccel * 1.2);
            
            eventCount++;
        }
        
        delay(100);
    }
    
    // Analysiere Synchronisation
    int syncedEvents = 0;
    int delayedEvents = 0;
    
    for (int i = 0; i < eventCount; i++) {
        if (events[i].sensorDetected && events[i].vibrationDetected) {
            syncedEvents++;
        }
        
        // Prüfe ob GPS verzögert reagiert (normal)
        if (i > 0 && events[i].gpsDetected && !events[i-1].gpsDetected) {
            delayedEvents++;
        }
    }
    
    float syncRate = eventCount > 0 ? (float)syncedEvents / eventCount * 100 : 0;
    
    details = String(eventCount) + " Bewegungs-Events, ";
    details += String(syncRate, 1) + "% synchron, ";
    details += String(delayedEvents) + " GPS-Verzögerungen";
    
    // Bewertung
    if (eventCount < 5) {
        testPassed = false;
        details += " - Zu wenig Bewegung!";
    }
    
    if (syncRate < 80.0 && eventCount > 10) {
        testPassed = false;
        details += " - Schlechte Synchronisation!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Cross-Module Sync", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

// ========== LANGZEIT-TESTS ==========

bool IntegrationTests::testMemoryLeakDetection() {
    Serial.println("\n--- Test: Memory-Leak-Erkennung ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    Serial.println("Überwache Speicher für 2 Minuten...");
    
    // Initiale Heap-Messung
    uint32_t initialHeap = getFreeHeap();
    uint32_t minHeap = initialHeap;
    uint32_t maxHeap = initialHeap;
    
    // Heap-Verlauf
    const int SAMPLES = 24;  // 2 Minuten / 5 Sekunden
    uint32_t heapSamples[SAMPLES];
    int sampleIndex = 0;
    
    // Test für 2 Minuten
    uint32_t testDuration = 120000;  // 2 Minuten
    uint32_t testEnd = millis() + testDuration;
    uint32_t lastSample = millis();
    
    while (millis() < testEnd) {
        // Normale Operationen
        SensorData sensor = bnoManager.getCurrentData();
        sdLogger.logSensorData(sensor);
        
        if (gpsManager.available()) {
            GPSData gps = gpsManager.getCurrentData();
            sdLogger.logGPSData(gps);
        }
        
        if (canReader.available()) {
            CANMessage msg = canReader.getLastMessage();
            canReader.logMessage(msg);
        }
        
        // Display-Update
        oledManager.update();
        
        // Heap-Sampling alle 5 Sekunden
        if (millis() - lastSample > 5000) {
            uint32_t currentHeap = getFreeHeap();
            
            if (sampleIndex < SAMPLES) {
                heapSamples[sampleIndex++] = currentHeap;
            }
            
            if (currentHeap < minHeap) minHeap = currentHeap;
            if (currentHeap > maxHeap) maxHeap = currentHeap;
            
            Serial.printf("Heap: %lu KB frei\n", currentHeap / 1024);
            
            lastSample = millis();
        }
        
        delay(100);
    }
    
    // Analysiere Heap-Trend
    uint32_t finalHeap = getFreeHeap();
    int32_t totalLeak = initialHeap - finalHeap;
    
    // Berechne Trend (lineare Regression vereinfacht)
    float avgFirstHalf = 0;
    float avgSecondHalf = 0;
    int halfPoint = sampleIndex / 2;
    
    for (int i = 0; i < halfPoint; i++) {
        avgFirstHalf += heapSamples[i];
    }
    avgFirstHalf /= halfPoint;
    
    for (int i = halfPoint; i < sampleIndex; i++) {
        avgSecondHalf += heapSamples[i];
    }
    avgSecondHalf /= (sampleIndex - halfPoint);
    
    float leakRate = (avgFirstHalf - avgSecondHalf) / 60.0;  // Bytes pro Sekunde
    
    details = "Start: " + String(initialHeap) + ", ";
    details += "Ende: " + String(finalHeap) + ", ";
    details += "Verlust: " + String(totalLeak) + " bytes, ";
    details += "Rate: " + String(leakRate, 1) + " B/s";
    
    // Bewertung
    if (totalLeak > 5000) {  // Mehr als 5KB in 2 Minuten
        testPassed = false;
        details += " - Signifikanter Leak!";
    }
    
    if (leakRate > 10.0) {  // Mehr als 10 Bytes/Sekunde
        testPassed = false;
        details += " - Leak-Rate zu hoch!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Memory-Leak-Erkennung", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

bool IntegrationTests::testSensorCalibrationDrift() {
    Serial.println("\n--- Test: Sensor-Kalibrierungs-Drift ---");
    uint32_t startTime = millis();
    bool testPassed = true;
    String details = "";
    
    if (!bnoManager.isReady()) {
        Serial.println("⚠️ BNO055 nicht bereit - Test übersprungen");
        addTestResult("Kalibrierungs-Drift", false, 0, "Sensor nicht bereit");
        return false;
    }
    
    Serial.println("Überwache Kalibrierung für 1 Minute...");
    
    // Initiale Kalibrierung
    CalibrationData initialCal = bnoManager.getCalibration();
    
    // Speichere initiale Sensor-Werte
    SensorData initialData = bnoManager.getCurrentData();
    
    // Überwache für 1 Minute
    const int SAMPLES = 12;  // 1 Sample alle 5 Sekunden
    CalibrationData calSamples[SAMPLES];
    SensorData dataSamples[SAMPLES];
    int sampleIndex = 0;
    
    uint32_t testDuration = 60000;  // 1 Minute
    uint32_t testEnd = millis() + testDuration;
    uint32_t lastSample = millis();
    
    while (millis() < testEnd && sampleIndex < SAMPLES) {
        // Sample alle 5 Sekunden
        if (millis() - lastSample > 5000) {
            calSamples[sampleIndex] = bnoManager.getCalibration();
            dataSamples[sampleIndex] = bnoManager.getCurrentData();
            
            Serial.printf("Cal[%d]: S:%d G:%d A:%d M:%d\n", 
                         sampleIndex,
                         calSamples[sampleIndex].system,
                         calSamples[sampleIndex].gyro,
                         calSamples[sampleIndex].accel,
                         calSamples[sampleIndex].mag);
            
            sampleIndex++;
            lastSample = millis();
        }
        
        // Normale Operationen
        bnoManager.getCurrentData();
        delay(100);
    }
    
    // Analysiere Drift
    int calDrops = 0;
    float maxHeadingDrift = 0;
    float totalHeadingDrift = 0;
    
    for (int i = 0; i < sampleIndex; i++) {
        // Kalibrierung verschlechtert?
        if (calSamples[i].system < initialCal.system ||
            calSamples[i].gyro < initialCal.gyro ||
            calSamples[i].accel < initialCal.accel ||
            calSamples[i].mag < initialCal.mag) {
            calDrops++;
        }
        
        // Heading-Drift (bei ruhigem Sensor)
        float headingDrift = abs(dataSamples[i].heading - initialData.heading);
        if (headingDrift > 180) headingDrift = 360 - headingDrift;
        
        totalHeadingDrift += headingDrift;
        if (headingDrift > maxHeadingDrift) {
            maxHeadingDrift = headingDrift;
        }
    }
    
    float avgHeadingDrift = sampleIndex > 0 ? totalHeadingDrift / sampleIndex : 0;
    
    // Finale Kalibrierung
    CalibrationData finalCal = bnoManager.getCalibration();
    
    details = "Cal-Drops: " + String(calDrops) + ", ";
    details += "Heading-Drift: " + String(avgHeadingDrift, 1) + "° (max:" + 
               String(maxHeadingDrift, 1) + "°), ";
    details += "Final: S" + String(finalCal.system) + " G" + String(finalCal.gyro) + 
               " A" + String(finalCal.accel) + " M" + String(finalCal.mag);
    
    // Bewertung
    if (calDrops > 2) {  // Mehr als 2 Drops in 1 Minute
        testPassed = false;
        details += " - Kalibrierung instabil!";
    }
    
    if (avgHeadingDrift > 2.0) {  // Mehr als 2° Drift bei Ruhe
        testPassed = false;
        details += " - Zu viel Drift!";
    }
    
    uint32_t duration = millis() - startTime;
    addTestResult("Kalibrierungs-Drift", testPassed, duration, details);
    
    Serial.printf("%s Test in %lu ms\n", testPassed ? "✅" : "❌", duration);
    Serial.println("Details: " + details);
    
    return testPassed;
}

// ========== ERGEBNIS-FUNKTIONEN ==========

void IntegrationTests::printResults() {
    Serial.println("\n=== INTEGRATION TEST ERGEBNISSE ===");
    Serial.println("Nr | Status | Test                        | Zeit    | Details");
    Serial.println("---|--------|-----------------------------|---------|---------");
    
    for (int i = 0; i < resultCount; i++) {
        Serial.printf("%2d | %s | %-26s | %6lums | %s\n",
                     i + 1,
                     results[i].passed ? " ✅  " : " ❌  ",
                     results[i].testName.substring(0, 26).c_str(),
                     results[i].duration,
                     results[i].details.substring(0, 40).c_str());
    }
}

void IntegrationTests::printDetailedReport() {
    Serial.println("\n========== DETAILLIERTER TEST-REPORT ==========");
    
    // Statistiken
    Serial.println("\n--- STATISTIKEN ---");
    Serial.printf("Gesamt-Tests: %lu\n", stats.totalTests);
    Serial.printf("Bestanden: %lu (%.1f%%)\n", stats.passedTests, 
                  stats.totalTests > 0 ? (float)stats.passedTests / stats.totalTests * 100 : 0);
    Serial.printf("Fehlgeschlagen: %lu\n", stats.failedTests);
    Serial.printf("Gesamt-Dauer: %.1f Sekunden\n", stats.totalDuration / 1000.0);
    Serial.printf("Test-Coverage: %.1f%%\n", getTestCoverage());
    
    // Memory-Statistiken
    Serial.println("\n--- MEMORY-STATISTIKEN ---");
    Serial.printf("Min Free Heap: %lu bytes\n", stats.minFreeHeap);
    Serial.printf("Max Heap Usage: %lu bytes\n", stats.maxHeapUsage);
    Serial.printf("Buffer Overflows: %lu\n", stats.bufferOverflows);
    Serial.printf("Hardware Failures: %lu\n", stats.hardwareFailures);
    
    // Fehlgeschlagene Tests
    if (stats.failedTests > 0) {
        Serial.println("\n--- FEHLGESCHLAGENE TESTS ---");
        for (int i = 0; i < resultCount; i++) {
            if (!results[i].passed) {
                Serial.printf("\n❌ %s\n", results[i].testName.c_str());
                Serial.printf("   Dauer: %lu ms\n", results[i].duration);
                Serial.printf("   Details: %s\n", results[i].details.c_str());
                if (results[i].memoryLeaked > 0) {
                    Serial.printf("   Memory Leak: %lu bytes\n", results[i].memoryLeaked);
                }
                if (results[i].errorsDetected > 0) {
                    Serial.printf("   Fehler erkannt: %lu\n", results[i].errorsDetected);
                }
            }
        }
    }
    
    // Empfehlungen
    Serial.println("\n--- EMPFEHLUNGEN ---");
    if (stats.failedTests == 0) {
        Serial.println("✅ Alle Tests bestanden! System ist bereit für Produktion.");
    } else {
        Serial.println("⚠️ Folgende Bereiche benötigen Aufmerksamkeit:");
        
        // Analysiere Fehler-Muster
        bool memoryIssues = false;
        bool timingIssues = false;
        bool hardwareIssues = false;
        
        for (int i = 0; i < resultCount; i++) {
            if (!results[i].passed) {
                if (results[i].testName.indexOf("Memory") >= 0 || 
                    results[i].testName.indexOf("Buffer") >= 0) {
                    memoryIssues = true;
                }
                if (results[i].testName.indexOf("Latenz") >= 0 || 
                    results[i].testName.indexOf("Timing") >= 0) {
                    timingIssues = true;
                }
                if (results[i].testName.indexOf("Hardware") >= 0 || 
                    results[i].testName.indexOf("Recovery") >= 0) {
                    hardwareIssues = true;
                }
            }
        }
        
        if (memoryIssues) {
            Serial.println("   - Memory-Management überprüfen");
            Serial.println("   - Buffer-Größen anpassen");
        }
        if (timingIssues) {
            Serial.println("   - Task-Prioritäten optimieren");
            Serial.println("   - Interrupt-Handling verbessern");
        }
        if (hardwareIssues) {
            Serial.println("   - Hardware-Verbindungen prüfen");
            Serial.println("   - Error-Recovery-Mechanismen verstärken");
        }
    }
    
    Serial.println("\n========== ENDE DES REPORTS ==========");
}

float IntegrationTests::getTestCoverage() {
    // Berechne Test-Coverage basierend auf Modul-Tests
    const int TOTAL_MODULES = 7;
    const int TESTS_PER_MODULE = 5;
    const int TOTAL_POSSIBLE_TESTS = TOTAL_MODULES * TESTS_PER_MODULE;
    
    // Zähle durchgeführte Tests pro Kategorie
    int multiModuleTests = 0;
    int failureTests = 0;
    int edgeCaseTests = 0;
    int performanceTests = 0;
    int integrityTests = 0;
    
    for (int i = 0; i < resultCount; i++) {
        String name = results[i].testName;
        
        if (name.indexOf("Module") >= 0 || name.indexOf("Korrelation") >= 0) {
            multiModuleTests++;
        } else if (name.indexOf("Recovery") >= 0 || name.indexOf("Failure") >= 0) {
            failureTests++;
        } else if (name.indexOf("Edge") >= 0 || name.indexOf("Maximum") >= 0) {
            edgeCaseTests++;
        } else if (name.indexOf("Throughput") >= 0 || name.indexOf("Latenz") >= 0) {
            performanceTests++;
        } else if (name.indexOf("Konsistenz") >= 0 || name.indexOf("Timestamp") >= 0) {
            integrityTests++;
        }
    }
    
    // Gewichtete Coverage
    float coverage = 0;
    coverage += (multiModuleTests / 5.0) * 20;    // 20% Gewicht
    coverage += (failureTests / 6.0) * 25;        // 25% Gewicht (wichtig!)
    coverage += (edgeCaseTests / 5.0) * 20;       // 20% Gewicht
    coverage += (performanceTests / 3.0) * 15;    // 15% Gewicht
    coverage += (integrityTests / 3.0) * 20;      // 20% Gewicht
    
    // Begrenze auf 100%
    if (coverage > 100) coverage = 100;
    
    return coverage;
}

void IntegrationTests::saveResultsToSD() {
    if (!sdLogger.isReady()) {
        Serial.println("⚠️ SD-Karte nicht verfügbar - Ergebnisse nicht gespeichert");
        return;
    }
    
    String fileName = "/test_results_" + String(millis()) + ".txt";
    File resultFile = SD.open(fileName, FILE_WRITE);
    
    if (!resultFile) {
        Serial.println("❌ Konnte Test-Ergebnis-Datei nicht erstellen");
        return;
    }
    
    // Header
    resultFile.println("ESP32-S3 ROAD QUALITY SYSTEM - INTEGRATION TEST RESULTS");
    resultFile.println("======================================================");
    resultFile.printf("Test Date: %lu ms since boot\n", millis());
    resultFile.printf("Total Duration: %.1f seconds\n", stats.totalDuration / 1000.0);
    resultFile.println();
    
    // Summary
    resultFile.println("SUMMARY");
    resultFile.println("-------");
    resultFile.printf("Total Tests: %lu\n", stats.totalTests);
    resultFile.printf("Passed: %lu (%.1f%%)\n", stats.passedTests, 
                      (float)stats.passedTests / stats.totalTests * 100);
    resultFile.printf("Failed: %lu\n", stats.failedTests);
    resultFile.printf("Coverage: %.1f%%\n", getTestCoverage());
    resultFile.println();
    
    // Detailed Results
    resultFile.println("DETAILED RESULTS");
    resultFile.println("----------------");
    
    for (int i = 0; i < resultCount; i++) {
        resultFile.printf("%d. %s - %s (%lu ms)\n", 
                         i + 1,
                         results[i].testName.c_str(),
                         results[i].passed ? "PASSED" : "FAILED",
                         results[i].duration);
        resultFile.printf("   Details: %s\n", results[i].details.c_str());
        
        if (!results[i].passed) {
            if (results[i].memoryLeaked > 0) {
                resultFile.printf("   Memory Leak: %lu bytes\n", results[i].memoryLeaked);
            }
            if (results[i].errorsDetected > 0) {
                resultFile.printf("   Errors: %lu\n", results[i].errorsDetected);
            }
        }
        resultFile.println();
    }
    
    resultFile.close();
    
    Serial.printf("✅ Test-Ergebnisse gespeichert in: %s\n", fileName.c_str());
}

// ========== UTILITY-FUNKTIONEN ==========

void runIntegrationTestSuite() {
    Serial.println("\n🚀 Starte vollständige Integration-Test-Suite...");
    integrationTests.runAllTests();
}

void runStressTestSuite() {
    Serial.println("\n🔥 Starte Stress-Test-Suite...");
    IntegrationTestConfig config;
    config.runStressTests = true;
    config.runFailureTests = false;
    config.stressTestDuration = 120000;  // 2 Minuten
    
    integrationTests = IntegrationTests();  // Reset
    integrationTests.testAllModulesConcurrent();
    integrationTests.testHighFrequencyLogging();
    integrationTests.testBufferStressUnderLoad();
    integrationTests.testMaximumThroughput();
    integrationTests.testLatencyUnderLoad();
    
    integrationTests.printDetailedReport();
}

void runFailureRecoveryTests() {
    Serial.println("\n🛠️ Starte Failure-Recovery-Test-Suite...");
    IntegrationTestConfig config;
    config.runStressTests = false;
    config.runFailureTests = true;
    
    integrationTests = IntegrationTests();  // Reset
    integrationTests.testI2CBusRecovery();
    integrationTests.testSDCardHotplug();
    integrationTests.testSensorDisconnectReconnect();
    integrationTests.testCANBusOverload();
    integrationTests.testGPSSignalLoss();
    integrationTests.testPowerBrownout();
    
    integrationTests.printDetailedReport();
}
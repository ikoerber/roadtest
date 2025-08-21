#ifndef INTEGRATION_TESTS_H
#define INTEGRATION_TESTS_H

#include <Arduino.h>
#include "hardware_test.h"
#include "bno055_manager.h"
#include "gps_manager.h"
#include "can_reader.h"
#include "sd_logger.h"
#include "oled_manager.h"
#include "buffer_utils.h"

// Test-Konfiguration
struct IntegrationTestConfig {
    bool runStressTests = true;
    bool runFailureTests = true;
    bool runLongTermTests = false;  // 24h Tests
    uint32_t stressTestDuration = 60000;  // 1 Minute Standard
    uint32_t moduleTestInterval = 100;    // 100ms zwischen Modul-Checks
};

// Test-Ergebnisse
struct IntegrationTestResult {
    String testName;
    bool passed;
    uint32_t duration;
    String details;
    uint32_t memoryLeaked = 0;
    uint32_t errorsDetected = 0;
};

// Test-Statistiken
struct IntegrationTestStats {
    uint32_t totalTests = 0;
    uint32_t passedTests = 0;
    uint32_t failedTests = 0;
    uint32_t totalDuration = 0;
    uint32_t maxHeapUsage = 0;
    uint32_t minFreeHeap = 0xFFFFFFFF;
    uint32_t bufferOverflows = 0;
    uint32_t hardwareFailures = 0;
};

class IntegrationTests {
private:
    IntegrationTestConfig config;
    IntegrationTestStats stats;
    static const int MAX_TEST_RESULTS = 50;
    IntegrationTestResult results[MAX_TEST_RESULTS];
    int resultCount = 0;
    
    // Hilfsfunktionen
    void addTestResult(const String& name, bool passed, uint32_t duration, const String& details);
    uint32_t getFreeHeap();
    void checkMemoryLeaks(uint32_t startHeap, uint32_t endHeap, IntegrationTestResult& result);
    bool simulateHardwareFailure(const String& component);
    bool recoverHardware(const String& component);
    void monitorSystemHealth();
    
public:
    IntegrationTests();
    
    // Haupttest-Funktionen
    bool runAllTests();
    bool runQuickTests();
    
    // Multi-Modul Tests
    bool testAllModulesConcurrent();          // Alle Module gleichzeitig
    bool testSensorDataCorrelation();         // Sensor + GPS + CAN Korrelation
    bool testHighFrequencyLogging();          // Maximum Logging-Rate
    bool testBufferStressUnderLoad();         // Buffer-Limits testen
    bool testMemoryFragmentation();           // Heap-Fragmentierung
    
    // Hardware Failure & Recovery Tests
    bool testI2CBusRecovery();               // I2C-Bus Hang & Recovery
    bool testSDCardHotplug();                 // SD-Karte entfernen/einsetzen
    bool testSensorDisconnectReconnect();     // Sensor-Ausfall simulieren
    bool testCANBusOverload();                // CAN-Bus Überflutung
    bool testGPSSignalLoss();                 // GPS-Signal-Verlust
    bool testPowerBrownout();                 // Spannungseinbruch
    
    // Edge-Case Tests
    bool testMaximumVibration();              // Extreme Beschleunigung
    bool testRapidDirectionChanges();         // Schnelle Richtungswechsel
    bool testSimultaneousEvents();            // Gleichzeitige Events
    bool testBufferOverflowRecovery();        // Buffer-Overflow Handling
    bool testFileSystemFull();                // SD-Karte voll
    
    // Langzeit-Tests
    bool test24HourStability();               // 24h Dauerlauf
    bool testMemoryLeakDetection();           // Memory-Leak über Zeit
    bool testSensorCalibrationDrift();        // Kalibrierungs-Drift
    
    // Performance Tests
    bool testMaximumThroughput();             // Maximale Datenrate
    bool testLatencyUnderLoad();              // Latenz bei Last
    bool testCPUUsageOptimization();          // CPU-Auslastung
    
    // Daten-Integritäts-Tests
    bool testDataConsistency();               // Daten-Konsistenz
    bool testTimestampAccuracy();             // Zeitstempel-Genauigkeit
    bool testCrossModuleSync();               // Modul-Synchronisation
    
    // Ergebnis-Funktionen
    void printResults();
    void printDetailedReport();
    IntegrationTestStats getStats() { return stats; }
    float getTestCoverage();
    void saveResultsToSD();
};

// Globale Test-Instanz
extern IntegrationTests integrationTests;

// Utility-Funktionen für Tests
void runIntegrationTestSuite();
void runStressTestSuite();
void runFailureRecoveryTests();

#endif // INTEGRATION_TESTS_H
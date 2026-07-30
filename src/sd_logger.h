#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include "bno055_manager.h"
#include "can_reader.h"
#include "gps_manager.h"
#include "hardware_config.h"
#include "road_quality.h"

// Log-Typen
enum LogType {
    LOG_TYPE_SENSOR,    // BNO055 Sensor-Daten
    LOG_TYPE_CAN,       // CAN-Bus Nachrichten
    LOG_TYPE_ROAD,      // Straßenqualitäts-Metriken
    LOG_TYPE_EVENT,     // Ereignisse (Schlaglöcher, Kurven)
    LOG_TYPE_SYSTEM,    // System-Status
    LOG_TYPE_GPS,       // GPS-Daten
    LOG_TYPE_OBD,       // Dekodierte OBD-II-Service-01-Daten
    LOG_TYPE_OBD_TRACE, // Einzelne OBD-Anfragen, Antworten und Timeouts
    LOG_TYPE_META,      // Sitzungsmetadaten und Abschlusszähler
    LOG_TYPE_CORRELATED // Zeitlich korrelierte Sensor-/CAN-Daten
};

// Log-Konfiguration
struct LogConfig {
    bool enableSensorLog;
    bool enableCANLog;
    bool enableRoadLog;
    bool enableEventLog;
    bool enableSystemLog;
    bool enableGPSLog;
    
    uint32_t sensorLogInterval;  // ms
    uint32_t roadLogInterval;    // ms
    uint32_t gpsLogInterval;     // ms
    uint32_t flushInterval;      // ms
    
    String filePrefix;
    bool useTimestamp;
    bool compressData;
};

// Log-Statistiken
struct LogStats {
    uint32_t totalWrites;
    uint32_t totalBytes;
    uint32_t droppedLogs;
    uint32_t fileCount;
    uint32_t errorCount;
    uint32_t bufferOverflows;  // Für Integration-Tests
    unsigned long startTime;
    
    LogStats() : totalWrites(0), totalBytes(0), droppedLogs(0), 
                 fileCount(0), errorCount(0), bufferOverflows(0),
                 startTime(millis()) {}
    
    float getWriteRate() const {
        unsigned long elapsed = millis() - startTime;
        if (elapsed == 0) return 0;
        return (float)totalWrites * 1000.0f / elapsed;
    }
    
    float getDataRate() const {
        unsigned long elapsed = millis() - startTime;
        if (elapsed == 0) return 0;
        return (float)totalBytes * 1000.0f / elapsed;
    }
};

// Alias für Integration-Tests
typedef LogStats SDLoggerStats;

// Zusammenfassung einer laufenden oder zuletzt beendeten Messfahrt
struct RideSummary {
    bool active;
    bool completed;
    bool interrupted;
    String sessionId;
    String startUTC;
    String endUTC;
    uint32_t durationSeconds;
    float distanceKm;
    uint32_t potholeCount;
    uint32_t curveCount;
    uint32_t qualitySamples;
    float averageQuality;

    RideSummary()
        : active(false), completed(false), interrupted(false), durationSeconds(0),
          distanceKm(0), potholeCount(0), curveCount(0),
          qualitySamples(0), averageQuality(0) {}
};

class SDLogger {
private:
    enum class LoggingStartState : uint8_t {
        IDLE,
        PREPARE,
        OPEN_SENSOR,
        OPEN_ROAD,
        OPEN_GPS,
        OPEN_EVENT,
        OPEN_META,
        OPEN_CAN,
        OPEN_OBD,
        OPEN_OBD_TRACE,
        OPEN_CORRELATED,
        FINALIZE
    };

    // Pin-Konfiguration
    int csPin;
    SPIClass* spiInstance;
    
    // Status
    bool initialized;
    bool cardAvailable;
    bool logging;
    bool stopping;
    LoggingStartState loggingStartState;
    String lastStartError;
    
    // Datei-Management
    File currentLogFile;
    String currentFileName;
    File roadLogFile;
    String roadFileName;
    File gpsLogFile;
    String gpsFileName;
    File canLogFile;
    String canFileName;
    File obdLogFile;
    String obdFileName;
    File obdTraceLogFile;
    String obdTraceFileName;
    File metaLogFile;
    String metaFileName;
    File eventLogFile;
    String eventFileName;
    File correlatedLogFile;
    String correlatedFileName;
    String sessionId;
    String sessionDirectory;
    
    // Konfiguration
    LogConfig config;
    LogStats stats;
    
    // Timing
    unsigned long lastRoadLog;
    unsigned long lastFlush;
    unsigned long lastMetadataLog;
    unsigned long sessionStartTime;
    GPSStatus gpsSessionStartStatus;
    uint32_t gpsSessionStartFixSequence;
    uint32_t gpsSessionLastLoggedFixSequence;
    CANHardwareDiagnostics canSessionStartDiagnostics;

    // Messfahrts-Zusammenfassung
    RideSummary rideSummary;
    double qualitySum;
    bool hasLastRideGPS;
    float lastRideLatitude;
    float lastRideLongitude;
    unsigned long lastRideGPSTime;
    
    // Puffer für Performance mit Overflow-Schutz
    // Fünf Sekunden 10-Hz-Sensordaten passen vollständig in den RAM-Puffer.
    // Der frühere 512-Byte-Puffer erzwang bereits nach wenigen CSV-Zeilen
    // einen synchronen SD-Schreibzugriff und verlor dadurch Abtastslots.
    static const int BUFFER_SIZE = 8192;
    static const int BUFFER_SAFETY_MARGIN = 128;
    char writeBuffer[BUFFER_SIZE];
    int bufferIndex;
    uint32_t bufferedRecordCount;

    // Wiederanlauf nach SD-Ausfall. Sensorzeilen bleiben bei einem
    // vorübergehenden Kartenfehler im RAM und werden nach dem erneuten
    // Einbinden in eine eindeutig gekennzeichnete Recovery-Datei geschrieben.
    // Eine neue Sitzung setzt die Messung anschließend automatisch fort und
    // verweist per Ereignis auf die unterbrochene Ursprungssitzung.
    bool recoveryResumePending;
    bool recoveryMarkerPending;
    bool recoveryBufferPending;
    bool recoveryStateLoaded;
    String recoveryOriginalSessionId;
    String recoveryFailureReason;
    String recoveryFailureUTC;
    unsigned long recoveryFailureTime;
    uint32_t recoveryBufferedRecordsAtFailure;
    uint32_t recoveryRecoveredRecords;
    String recoveryBufferFileName;
    String lastRecoveryStatus;
    
    // Buffer-Overflow Schutz Funktionen
    size_t getAvailableBufferSpace() const;
    
    // Hilfsfunktionen
    String generateFileName(LogType type);
    String generateSessionId();
    bool createLogFile(LogType type);
    bool openLogFile(File& file, String& fileName, LogType type);
    void closeLogFiles();
    void handleCardFailure(const char* reason, uint32_t droppedRecords = 0);
    bool writeHeader(File& file, LogType type);
    bool writeRideSummaryFile();
    bool recoverBufferedSensorData();
    bool flushBuffer();
    void failLoggingStart(const String& reason);
    void loadPersistentRecoveryState();
    void persistActiveSession();
    void persistFailureState(const String& reason);
    void clearPersistentRecoveryState();
    void finishRecoveryMarker();
    String formatTimestamp();
    String formatUTC();
    
public:
    // CS folgt der zentralen Pin-Konfiguration statt einem eigenen Literal.
    SDLogger(int cs = SD_CS_PIN);
    ~SDLogger();
    
    // Initialisierung
    bool begin(SPIClass& spi = SPI);
    void end();
    bool isReady() const { return initialized && cardAvailable; }
    bool checkHealth();
    
    // Konfiguration
    void setConfig(const LogConfig& cfg) { config = cfg; }
    LogConfig getConfig() const { return config; }
    void setLogInterval(LogType type, uint32_t interval);
    
    // Logging Control
    bool startLogging();
    bool requestLoggingStart();
    void processLoggingStart();
    void stopLogging();
    bool isLogging() const { return logging; }
    bool isLoggingStartPending() const {
        return loggingStartState != LoggingStartState::IDLE;
    }
    String getLastStartError() const { return lastStartError; }
    String getRecoveryStatus() const { return lastRecoveryStatus; }
    bool isRecoveryPending() const {
        return recoveryResumePending || recoveryMarkerPending ||
               recoveryBufferPending;
    }
    RideSummary getRideSummary() const;
    String getSessionId() const { return sessionId; }
    
    // Sensor-Daten loggen
    bool logSensorData(const SensorData& data);
    bool logVibrationMetrics(const VibrationMetrics& metrics);
    bool logCalibration(const CalibrationData& cal);
    
    // CAN-Daten loggen
    bool logCANMessage(const CANMessage& msg);
    bool logCANStatistics(uint32_t received, uint32_t errors);
    bool logOBDData(const OBDLiveData& obd,
                    const OBDSessionStats& session,
                    const CANHardwareDiagnostics& diagnostics);
    bool logOBDTraceEvent(const OBDTraceEvent& event,
                          const CANHardwareDiagnostics& diagnostics);
    bool logSessionMetadata(const char* record,
                            const GPSStatus& gpsStatus,
                            const OBDSessionStats& obdStatus,
                            const CANHardwareDiagnostics& canStatus);
    
    // GPS-Daten loggen
    bool logGPSData(const GPSData& gps);
    bool logGPSStatus(uint8_t satellites, bool fix, float hdop);
    
    // Straßenqualität loggen
    bool logRoadQuality(float quality, float smoothness, 
                       float curveFrequency, float vibrationRMS);
    bool logRoadMetrics(const RoadMetrics& metrics);
    
    // Ereignisse loggen
    bool logEvent(const String& eventType, const String& description,
                  float lat = 0, float lon = 0, float severity = 0);
    bool logPothole(float severity, float lat = 0, float lon = 0);
    bool logCurve(float angle, float radius, float lat = 0, float lon = 0);
    
    // System-Status loggen
    bool logSystemStatus(const String& status);
    bool logError(const String& error);
    bool logDebug(const String& message);
    
    // Daten-Management
    void flush();
    bool rotateLogFile();
    uint32_t getFileSize() const;
    uint32_t getFreeSpace();
    
    // Statistiken
    LogStats getStatistics() const { return stats; }
    void resetStatistics();
    void printStatistics();
    
    // Utility
    bool deleteOldLogs(uint32_t daysToKeep);
    bool compressLog(const String& fileName);
    bool uploadLog(const String& fileName); // Placeholder für Cloud-Upload
    
    // CSV-Export Funktionen
    bool exportSensorDataCSV(const String& fileName);
    bool exportRoadQualityCSV(const String& fileName);
    bool exportEventLogCSV(const String& fileName);
    
    // Zeitbasierte Synchronisation
    bool logCorrelatedData(const SensorData& sensorData, const CANMessage& canMsg);
    bool logGeolocatedData(const SensorData& sensorData, const GPSData& gpsData);
    bool exportCorrelatedCSV(const String& fileName);
    bool exportGeolocatedCSV(const String& fileName);
    
    // Dateisystem-Operationen
    bool listFiles(const String& path = "/");
    bool deleteFile(const String& fileName);
    bool renameFile(const String& oldName, const String& newName);
    bool copyFile(const String& source, const String& dest);
    
    // Für Integration-Tests
    bool safeAppendToBuffer(const char* data, size_t dataLen);
    bool safeAppendToBuffer(const String& data);
    String getCurrentLogFile() const { return currentFileName; }
};

// Globale Instanz
extern SDLogger sdLogger;

// Hilfsfunktionen für formatierte Ausgabe
String formatSensorDataCSV(const SensorData& data);
String formatCANMessageCSV(const CANMessage& msg);
String formatRoadMetricsCSV(const RoadMetrics& metrics);
String formatGPSDataCSV(const GPSData& gps);

#endif // SD_LOGGER_H

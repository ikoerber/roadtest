#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "bno055_manager.h"
#include "can_reader.h"
#include "road_quality.h"

// Forward declaration für GPS-Daten
struct GPSData;

// Log-Typen
enum LogType {
    LOG_TYPE_SENSOR,    // BNO055 Sensor-Daten
    LOG_TYPE_CAN,       // CAN-Bus Nachrichten
    LOG_TYPE_ROAD,      // Straßenqualitäts-Metriken
    LOG_TYPE_EVENT,     // Ereignisse (Schlaglöcher, Kurven)
    LOG_TYPE_SYSTEM,    // System-Status
    LOG_TYPE_GPS        // GPS-Daten
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
    unsigned long startTime;
    
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

class SDLogger {
private:
    // Pin-Konfiguration
    int csPin;
    SPIClass* spiInstance;
    
    // Status
    bool initialized;
    bool cardAvailable;
    bool logging;
    
    // Datei-Management
    File currentLogFile;
    String currentFileName;
    File eventLogFile;
    String eventFileName;
    
    // Konfiguration
    LogConfig config;
    LogStats stats;
    
    // Timing
    unsigned long lastSensorLog;
    unsigned long lastRoadLog;
    unsigned long lastFlush;
    unsigned long sessionStartTime;
    
    // Puffer für Performance mit Overflow-Schutz
    static const int BUFFER_SIZE = 512;
    static const int BUFFER_SAFETY_MARGIN = 32;  // Sicherheitspuffer
    char writeBuffer[BUFFER_SIZE];
    int bufferIndex;
    
    // Buffer-Overflow Schutz Funktionen
    bool safeAppendToBuffer(const char* data, size_t dataLen);
    bool safeAppendToBuffer(const String& data);
    size_t getAvailableBufferSpace() const;
    
    // Hilfsfunktionen
    String generateFileName(LogType type);
    bool createLogFile(LogType type);
    void writeHeader(File& file, LogType type);
    bool checkSDCard();
    void flushBuffer();
    String formatTimestamp();
    
public:
    SDLogger(int cs = 4);
    ~SDLogger();
    
    // Initialisierung
    bool begin(SPIClass& spi = SPI);
    void end();
    bool isReady() const { return initialized && cardAvailable; }
    
    // Konfiguration
    void setConfig(const LogConfig& cfg) { config = cfg; }
    LogConfig getConfig() const { return config; }
    void setLogInterval(LogType type, uint32_t interval);
    
    // Logging Control
    bool startLogging();
    void stopLogging();
    bool isLogging() const { return logging; }
    
    // Sensor-Daten loggen
    bool logSensorData(const SensorData& data);
    bool logVibrationMetrics(const VibrationMetrics& metrics);
    bool logCalibration(const CalibrationData& cal);
    
    // CAN-Daten loggen
    bool logCANMessage(const CANMessage& msg);
    bool logCANStatistics(uint32_t received, uint32_t errors);
    
    // GPS-Daten loggen
    bool logGPSData(const GPSData& gps);
    bool logGPSStatus(uint8_t satellites, bool fix, float hdop);
    
    // Straßenqualität loggen
    bool logRoadQuality(float quality, float smoothness, 
                       float curveFrequency, float vibrationRMS);
    bool logRoadMetrics(const RoadMetrics& metrics);
    
    // Ereignisse loggen
    bool logEvent(const String& eventType, const String& description, 
                  float lat = 0, float lon = 0);
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
};

// Globale Instanz
extern SDLogger sdLogger;

// Hilfsfunktionen für formatierte Ausgabe
String formatSensorDataCSV(const SensorData& data);
String formatCANMessageCSV(const CANMessage& msg);
String formatRoadMetricsCSV(const RoadMetrics& metrics);
String formatGPSDataCSV(const GPSData& gps);

#endif // SD_LOGGER_H
#include "sd_logger.h"
#include "road_quality.h"

// Forward-Deklaration
float calculateOverallQuality(const RoadMetrics& metrics);

// Globale Instanz
SDLogger sdLogger;

SDLogger::SDLogger(int cs) 
    : csPin(cs), spiInstance(nullptr), initialized(false), 
      cardAvailable(false), logging(false), bufferIndex(0),
      lastSensorLog(0), lastRoadLog(0), lastFlush(0),
      sessionStartTime(0) {
    
    // Standard-Konfiguration
    config = {
        true, true, true, true, true,  // Alle Logs aktiviert
        100,   // Sensor-Log alle 100ms
        1000,  // Road-Log alle 1s
        5000,  // Flush alle 5s
        "road", // Datei-Prefix
        true,   // Zeitstempel verwenden
        false   // Keine Kompression
    };
    
    stats = {0, 0, 0, 0, 0, 0};
}

SDLogger::~SDLogger() {
    end();
}

// Zeitbasierte Korrelation
bool SDLogger::logCorrelatedData(const SensorData& sensorData, const CANMessage& canMsg) {
    if (!logging || !isReady()) return false;
    
    // Kombinierte Log-Zeile erstellen
    String logLine = String(sensorData.timestamp) + ",CORR,";
    
    // Sensor-Daten
    logLine += String(sensorData.heading, 2) + ",";
    logLine += String(sensorData.pitch, 2) + ",";
    logLine += String(sensorData.roll, 2) + ",";
    logLine += String(sensorData.accelMagnitude, 3) + ",";
    logLine += String(sensorData.temperature, 1) + ",";
    
    // CAN-Daten
    logLine += "0x" + String(canMsg.canId, HEX) + ",";
    logLine += String(canMsg.dlc) + ",";
    
    // CAN-Data Bytes
    for (int i = 0; i < 8; i++) {
        if (i < canMsg.dlc) {
            logLine += "0x" + String(canMsg.data[i], HEX);
        }
        if (i < 7) logLine += ",";
    }
    
    logLine += "\n";
    
    // In separate korrelierte Datei schreiben
    String corrFileName = "/correlated_" + String(millis()) + ".csv";
    File corrFile = SD.open(corrFileName, FILE_APPEND);
    
    if (corrFile) {
        // Prüfen ob Datei leer ist (Header schreiben)
        if (corrFile.size() == 0) {
            corrFile.println("timestamp,type,heading,pitch,roll,accel_mag,temp,can_id,dlc,d0,d1,d2,d3,d4,d5,d6,d7");
        }
        
        corrFile.print(logLine);
        corrFile.close();
        
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }
    
    return false;
}

bool SDLogger::exportCorrelatedCSV(const String& fileName) {
    // Placeholder für korrelierte Daten Export
    Serial.println("Korrelierte Daten bereit für SavvyCAN Import!");
    return true;
}

bool SDLogger::begin(SPIClass& spi) {
    if (initialized) {
        return true;
    }
    
    Serial.println("=== SD Logger Initialisierung ===");
    Serial.printf("CS Pin: %d\n", csPin);
    
    spiInstance = &spi;
    
    // SD-Karte initialisieren
    if (!SD.begin(csPin, spi)) {
        Serial.println("❌ SD-Karte nicht gefunden!");
        return false;
    }
    
    cardAvailable = true;
    
    // Karteninfo ausgeben
    uint8_t cardType = SD.cardType();
    Serial.print("SD-Karten Typ: ");
    switch(cardType) {
        case CARD_MMC: Serial.println("MMC"); break;
        case CARD_SD: Serial.println("SDSC"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default: Serial.println("Unbekannt"); break;
    }
    
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("Kartengröße: %llu MB\n", cardSize);
    
    uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
    uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
    Serial.printf("Verwendet: %llu MB / %llu MB (%.1f%%)\n", 
                  usedBytes, totalBytes, 
                  (float)usedBytes * 100.0f / totalBytes);
    
    initialized = true;
    stats.startTime = millis();
    
    Serial.println("✅ SD Logger bereit!");
    return true;
}

void SDLogger::end() {
    if (logging) {
        stopLogging();
    }
    
    if (currentLogFile) {
        currentLogFile.close();
    }
    
    if (eventLogFile) {
        eventLogFile.close();
    }
    
    SD.end();
    initialized = false;
    cardAvailable = false;
}

bool SDLogger::startLogging() {
    if (!initialized || !cardAvailable) {
        Serial.println("SD Logger nicht bereit!");
        return false;
    }
    
    if (logging) {
        return true;
    }
    
    sessionStartTime = millis();
    
    // Haupt-Log-Datei erstellen
    if (!createLogFile(LOG_TYPE_SENSOR)) {
        return false;
    }
    
    // Event-Log-Datei erstellen
    eventFileName = generateFileName(LOG_TYPE_EVENT);
    eventLogFile = SD.open(eventFileName, FILE_WRITE);
    if (!eventLogFile) {
        currentLogFile.close();
        return false;
    }
    
    // Header schreiben
    writeHeader(currentLogFile, LOG_TYPE_SENSOR);
    writeHeader(eventLogFile, LOG_TYPE_EVENT);
    
    logging = true;
    Serial.printf("✅ Logging gestartet: %s\n", currentFileName.c_str());
    
    return true;
}

void SDLogger::stopLogging() {
    if (!logging) return;
    
    flush();
    
    if (currentLogFile) {
        currentLogFile.close();
    }
    
    if (eventLogFile) {
        eventLogFile.close();
    }
    
    logging = false;
    Serial.println("Logging gestoppt");
}

String SDLogger::generateFileName(LogType type) {
    String typeStr;
    switch(type) {
        case LOG_TYPE_SENSOR: typeStr = "sensor"; break;
        case LOG_TYPE_CAN: typeStr = "can"; break;
        case LOG_TYPE_ROAD: typeStr = "road"; break;
        case LOG_TYPE_EVENT: typeStr = "event"; break;
        case LOG_TYPE_SYSTEM: typeStr = "system"; break;
    }
    
    String fileName = "/" + config.filePrefix + "_" + typeStr;
    
    if (config.useTimestamp) {
        fileName += "_" + String(millis());
    }
    
    fileName += ".csv";
    return fileName;
}

bool SDLogger::createLogFile(LogType type) {
    currentFileName = generateFileName(type);
    currentLogFile = SD.open(currentFileName, FILE_WRITE);
    
    if (!currentLogFile) {
        Serial.printf("❌ Kann Datei nicht erstellen: %s\n", currentFileName.c_str());
        return false;
    }
    
    stats.fileCount++;
    return true;
}

void SDLogger::writeHeader(File& file, LogType type) {
    switch(type) {
        case LOG_TYPE_SENSOR:
            file.println("Zeit,Heading,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp,CalSys,CalGyro,CalAccel,CalMag");
            break;
            
        case LOG_TYPE_EVENT:
            file.println("Zeit,Ereignis,Beschreibung,Latitude,Longitude,Schwere");
            break;
            
        case LOG_TYPE_ROAD:
            file.println("Zeit,Qualität,Glätte,Kurven/km,VibrationRMS,MaxStoß");
            break;
            
        case LOG_TYPE_CAN:
            file.println("Zeit,CAN_ID,Extended,RTR,DLC,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7");
            break;
            
        case LOG_TYPE_SYSTEM:
            file.println("Zeit,Status,Nachricht");
            break;
    }
}

bool SDLogger::checkSDCard() {
    if (!cardAvailable) {
        return false;
    }
    
    // Karte noch vorhanden?
    if (SD.cardType() == CARD_NONE) {
        cardAvailable = false;
        Serial.println("⚠️ SD-Karte entfernt!");
        return false;
    }
    
    return true;
}

void SDLogger::flushBuffer() {
    if (bufferIndex > 0 && currentLogFile) {
        currentLogFile.write((const uint8_t*)writeBuffer, bufferIndex);
        bufferIndex = 0;
    }
}

String SDLogger::formatTimestamp() {
    unsigned long elapsed = millis() - sessionStartTime;
    return String(elapsed);
}

bool SDLogger::logSensorData(const SensorData& data) {
    if (!logging || !config.enableSensorLog) return false;
    
    unsigned long now = millis();
    if (now - lastSensorLog < config.sensorLogInterval) {
        return true; // Noch nicht Zeit für nächsten Log
    }
    
    String logLine = formatTimestamp() + "," +
                    String(data.heading, 1) + "," +
                    String(data.pitch, 1) + "," +
                    String(data.roll, 1) + "," +
                    String(data.accelX, 3) + "," +
                    String(data.accelY, 3) + "," +
                    String(data.accelZ, 3) + "," +
                    String(data.gyroX, 3) + "," +
                    String(data.gyroY, 3) + "," +
                    String(data.gyroZ, 3) + "," +
                    String(data.temperature, 1) + "," +
                    String(data.calibration.system) + "," +
                    String(data.calibration.gyro) + "," +
                    String(data.calibration.accel) + "," +
                    String(data.calibration.mag) + "\n";
    
    if (currentLogFile) {
        currentLogFile.print(logLine);
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        lastSensorLog = now;
        return true;
    }
    
    stats.droppedLogs++;
    return false;
}

bool SDLogger::logVibrationMetrics(const VibrationMetrics& metrics) {
    if (!logging || !config.enableRoadLog) return false;
    
    String logLine = formatTimestamp() + ",VIBRATION," +
                    String(metrics.rmsAccel, 3) + "," +
                    String(metrics.maxShock, 3) + "," +
                    String(metrics.frequency, 1) + "," +
                    String(metrics.shockCount) + "\n";
    
    if (eventLogFile) {
        eventLogFile.print(logLine);
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }
    
    return false;
}

bool SDLogger::logCalibration(const CalibrationData& cal) {
    if (!logging) return false;
    
    String status = cal.isFullyCalibrated() ? "VOLLSTÄNDIG" : "UNVOLLSTÄNDIG";
    return logEvent("KALIBRIERUNG", status + " Sys:" + String(cal.system) +
                    " Gyr:" + String(cal.gyro) + " Acc:" + String(cal.accel) +
                    " Mag:" + String(cal.mag));
}

bool SDLogger::logCANMessage(const CANMessage& msg) {
    if (!logging || !config.enableCANLog) return false;
    
    // Nutze can_reader's CSV-Funktionalität
    String logLine = String(msg.timestamp) + "," + 
                    String(msg.canId, HEX) + "," +
                    String(msg.extended) + "," +
                    String(msg.rtr) + "," +
                    String(msg.dlc);
    
    for (int i = 0; i < 8; i++) {
        logLine += ",";
        if (i < msg.dlc) {
            logLine += String(msg.data[i], HEX);
        }
    }
    logLine += "\n";
    
    if (currentLogFile) {
        currentLogFile.print(logLine);
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }
    
    return false;
}

bool SDLogger::logRoadQuality(float quality, float smoothness, 
                             float curveFrequency, float vibrationRMS) {
    if (!logging || !config.enableRoadLog) return false;
    
    unsigned long now = millis();
    if (now - lastRoadLog < config.roadLogInterval) {
        return true;
    }
    
    String logLine = formatTimestamp() + "," +
                    String(quality, 1) + "," +
                    String(smoothness, 3) + "," +
                    String(curveFrequency, 1) + "," +
                    String(vibrationRMS, 3) + "\n";
    
    if (currentLogFile) {
        currentLogFile.print(logLine);
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        lastRoadLog = now;
        return true;
    }
    
    return false;
}

bool SDLogger::logRoadMetrics(const RoadMetrics& metrics) {
    if (!logging || !config.enableRoadLog) return false;
    
    return logRoadQuality(
        calculateOverallQuality(metrics),
        metrics.surfaceSmoothness,
        metrics.curveFrequency,
        metrics.vibrationRMS
    );
}

bool SDLogger::logEvent(const String& eventType, const String& description, 
                       float lat, float lon) {
    if (!logging || !config.enableEventLog) return false;
    
    String logLine = formatTimestamp() + "," +
                    eventType + "," +
                    description + "," +
                    String(lat, 6) + "," +
                    String(lon, 6) + ",0\n";
    
    if (eventLogFile) {
        eventLogFile.print(logLine);
        eventLogFile.flush(); // Ereignisse sofort speichern
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }
    
    return false;
}

bool SDLogger::logPothole(float severity, float lat, float lon) {
    String sevStr;
    if (severity < 2.0) sevStr = "KLEIN";
    else if (severity < 5.0) sevStr = "MITTEL";
    else sevStr = "GROSS";
    
    return logEvent("SCHLAGLOCH", sevStr + " (" + String(severity, 1) + " m/s²)", lat, lon);
}

bool SDLogger::logCurve(float angle, float radius, float lat, float lon) {
    String desc = "Winkel: " + String(angle, 0) + "°";
    if (radius > 0) {
        desc += ", Radius: " + String(radius, 0) + "m";
    }
    
    return logEvent("KURVE", desc, lat, lon);
}

bool SDLogger::logSystemStatus(const String& status) {
    if (!logging || !config.enableSystemLog) return false;
    
    return logEvent("SYSTEM", status);
}

bool SDLogger::logError(const String& error) {
    stats.errorCount++;
    return logEvent("FEHLER", error);
}

bool SDLogger::logDebug(const String& message) {
    if (!logging) return false;
    
    return logEvent("DEBUG", message);
}

void SDLogger::flush() {
    flushBuffer();
    
    if (currentLogFile) {
        currentLogFile.flush();
    }
    
    if (eventLogFile) {
        eventLogFile.flush();
    }
    
    lastFlush = millis();
}

bool SDLogger::rotateLogFile() {
    if (!logging) return false;
    
    stopLogging();
    return startLogging();
}

uint32_t SDLogger::getFileSize() const {
    if (!currentLogFile) return 0;
    return currentLogFile.size();
}

uint32_t SDLogger::getFreeSpace() {
    if (!cardAvailable) return 0;
    
    uint64_t freeBytes = SD.totalBytes() - SD.usedBytes();
    return freeBytes / 1024; // KB zurückgeben
}

void SDLogger::resetStatistics() {
    stats = {0, 0, 0, 0, 0, millis()};
}

void SDLogger::printStatistics() {
    Serial.println("\n=== SD Logger Statistiken ===");
    Serial.printf("Schreibvorgänge: %lu\n", stats.totalWrites);
    Serial.printf("Geschriebene Bytes: %lu\n", stats.totalBytes);
    Serial.printf("Verworfene Logs: %lu\n", stats.droppedLogs);
    Serial.printf("Dateien erstellt: %lu\n", stats.fileCount);
    Serial.printf("Fehler: %lu\n", stats.errorCount);
    Serial.printf("Schreibrate: %.1f/s\n", stats.getWriteRate());
    Serial.printf("Datenrate: %.1f B/s\n", stats.getDataRate());
    Serial.printf("Freier Speicher: %lu KB\n", getFreeSpace());
}

bool SDLogger::listFiles(const String& path) {
    if (!cardAvailable) return false;
    
    Serial.printf("\n=== Dateien in %s ===\n", path.c_str());
    
    File root = SD.open(path);
    if (!root || !root.isDirectory()) {
        Serial.println("Verzeichnis nicht gefunden");
        return false;
    }
    
    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.printf("[DIR] %s\n", file.name());
        } else {
            Serial.printf("%s (%lu bytes)\n", file.name(), file.size());
        }
        file = root.openNextFile();
    }
    
    return true;
}

bool SDLogger::deleteFile(const String& fileName) {
    if (!cardAvailable) return false;
    
    if (SD.remove(fileName)) {
        Serial.printf("Datei gelöscht: %s\n", fileName.c_str());
        return true;
    }
    
    return false;
}

bool SDLogger::renameFile(const String& oldName, const String& newName) {
    if (!cardAvailable) return false;
    
    if (SD.rename(oldName, newName)) {
        Serial.printf("Datei umbenannt: %s -> %s\n", oldName.c_str(), newName.c_str());
        return true;
    }
    
    return false;
}

bool SDLogger::copyFile(const String& source, const String& dest) {
    if (!cardAvailable) return false;
    
    File srcFile = SD.open(source, FILE_READ);
    if (!srcFile) return false;
    
    File destFile = SD.open(dest, FILE_WRITE);
    if (!destFile) {
        srcFile.close();
        return false;
    }
    
    size_t size = srcFile.size();
    uint8_t buffer[512];
    
    while (size > 0) {
        size_t toRead = min(size, sizeof(buffer));
        srcFile.read(buffer, toRead);
        destFile.write(buffer, toRead);
        size -= toRead;
    }
    
    srcFile.close();
    destFile.close();
    
    Serial.printf("Datei kopiert: %s -> %s\n", source.c_str(), dest.c_str());
    return true;
}

bool SDLogger::deleteOldLogs(uint32_t daysToKeep) {
    // TODO: Implementierung mit Zeitstempel-Vergleich
    return false;
}

bool SDLogger::compressLog(const String& fileName) {
    // TODO: Implementierung mit Kompressionsalgorithmus
    return false;
}

bool SDLogger::uploadLog(const String& fileName) {
    // TODO: Implementierung für Cloud-Upload (WiFi/GSM)
    return false;
}

// Hilfsfunktionen für RoadMetrics
float calculateOverallQuality(const RoadMetrics& metrics) {
    float quality = 100;
    
    // Vibrationen bewerten
    quality -= min(metrics.vibrationRMS * 20, 35.0f);
    quality -= min(metrics.maxShock * 5, 25.0f);
    
    // Kurvigkeit bewerten (ideal: 15-25 Kurven/km)
    float idealCurves = 20.0;
    float curveDiff = fabs(metrics.curveFrequency - idealCurves);
    quality -= min(curveDiff * 1.5f, 20.0f);
    
    // Verkehrsruhe
    quality -= min(metrics.stopCount * 5, 20.0f);
    
    return max(quality, 0.0f);
}

// Globale Format-Funktionen
String formatSensorDataCSV(const SensorData& data) {
    return String(data.timestamp) + "," +
           String(data.heading, 1) + "," +
           String(data.pitch, 1) + "," +
           String(data.roll, 1) + "," +
           String(data.accelX, 3) + "," +
           String(data.accelY, 3) + "," +
           String(data.accelZ, 3);
}

String formatCANMessageCSV(const CANMessage& msg) {
    return formatCANMessage(msg); // Nutze can_reader Funktion
}

String formatRoadMetricsCSV(const RoadMetrics& metrics) {
    return String(metrics.curveFrequency, 1) + "," +
           String(metrics.vibrationRMS, 3) + "," +
           String(metrics.maxShock, 2) + "," +
           String(metrics.surfaceSmoothness, 3);
}
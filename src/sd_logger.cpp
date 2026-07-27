#include "sd_logger.h"
#include "road_quality.h"
#include "gps_manager.h"
#include <time.h>

// Forward-Deklaration
float calculateOverallQuality(const RoadMetrics& metrics);

// Globale Instanz
SDLogger sdLogger;

SDLogger::SDLogger(int cs) 
    : csPin(cs), spiInstance(nullptr), initialized(false), 
      cardAvailable(false), logging(false), bufferIndex(0),
      lastSensorLog(0), lastRoadLog(0), lastGPSLog(0), lastFlush(0),
      sessionStartTime(0), qualitySum(0), hasLastRideGPS(false),
      lastRideLatitude(0), lastRideLongitude(0), lastRideGPSTime(0) {
    
    // Standard-Konfiguration
    config = {
        true, true, true, true, true, true,  // Alle Logs aktiviert inkl. GPS
        100,   // Sensor-Log alle 100ms
        1000,  // Road-Log alle 1s
        200,   // GPS-Log alle 200ms
        5000,  // Flush alle 5s
        "road", // Datei-Prefix
        true,   // Zeitstempel verwenden
        false   // Keine Kompression
    };
    
    // Stats werden durch den Konstruktor von LogStats initialisiert
}

SDLogger::~SDLogger() {
    end();
}

// Zeitbasierte Korrelation
bool SDLogger::logCorrelatedData(const SensorData& sensorData, const CANMessage& canMsg) {
    if (!logging || !isReady() || canMsg.timestamp == 0) return false;

    if (!correlatedLogFile &&
        !openLogFile(correlatedLogFile, correlatedFileName, LOG_TYPE_CORRELATED)) {
        return false;
    }
    
    // Kombinierte Log-Zeile erstellen
    String logLine = formatTimestamp() + ",CORR,";
    
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
    
    if (correlatedLogFile &&
        correlatedLogFile.print(logLine) == logLine.length()) {
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }

    handleCardFailure("Korrelationsdaten konnten nicht geschrieben werden", 1);
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
    Serial.printf("CS Pin: %d, SPI-Takt: %d Hz\n", csPin, SD_SPI_SPEED);

    spiInstance = &spi;

    // SD-Karte initialisieren. Der Takt wird bewusst explizit übergeben; ohne
    // Argument nimmt die Bibliothek 4 MHz, was auf dem vorhandenen Aufbau zu
    // wenig Signalreserve lässt.
    if (!SD.begin(csPin, spi, SD_SPI_SPEED)) {
        Serial.println("❌ SD-Karte nicht gefunden!");
        return false;
    }

    // Karteninfo ausgeben
    uint8_t cardType = SD.cardType();
    uint64_t totalBytesRaw = SD.totalBytes();
    if (cardType == CARD_NONE || totalBytesRaw == 0) {
        Serial.println("❌ SD-Karte antwortet, liefert aber kein gültiges Dateisystem");
        SD.end();
        return false;
    }

    cardAvailable = true;
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
    uint64_t totalBytes = totalBytesRaw / (1024 * 1024);
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
    
    closeLogFiles();
    
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
    sessionId = generateSessionId();
    
    // Getrennte Dateien verhindern gemischte CSV-Zeilen.
    if (!createLogFile(LOG_TYPE_SENSOR)) {
        return false;
    }

    if ((config.enableRoadLog &&
         !openLogFile(roadLogFile, roadFileName, LOG_TYPE_ROAD)) ||
        (config.enableGPSLog &&
         !openLogFile(gpsLogFile, gpsFileName, LOG_TYPE_GPS)) ||
        (config.enableEventLog &&
         !openLogFile(eventLogFile, eventFileName, LOG_TYPE_EVENT))) {
        closeLogFiles();
        return false;
    }

    // Neue Fahrt beginnt immer mit einer leeren Zusammenfassung.
    rideSummary = RideSummary();
    rideSummary.active = true;
    rideSummary.sessionId = sessionId;
    rideSummary.startUTC = formatUTC();
    qualitySum = 0;
    hasLastRideGPS = false;
    lastRideLatitude = 0;
    lastRideLongitude = 0;
    lastRideGPSTime = 0;
    bufferIndex = 0;
    lastSensorLog = 0;
    lastRoadLog = 0;
    lastGPSLog = 0;

    logging = true;
    Serial.printf("✅ Sensor-Log: %s\n", currentFileName.c_str());
    if (roadLogFile) Serial.printf("✅ Straßen-Log: %s\n", roadFileName.c_str());
    if (gpsLogFile) Serial.printf("✅ GPS-Log: %s\n", gpsFileName.c_str());
    if (eventLogFile) Serial.printf("✅ Ereignis-Log: %s\n", eventFileName.c_str());
    
    return true;
}

void SDLogger::stopLogging() {
    if (!logging) return;

    rideSummary.durationSeconds = (millis() - sessionStartTime) / 1000;
    rideSummary.endUTC = formatUTC();
    rideSummary.active = false;
    rideSummary.completed = true;
    rideSummary.interrupted = false;
    if (rideSummary.qualitySamples > 0) {
        rideSummary.averageQuality =
            qualitySum / rideSummary.qualitySamples;
    }

    flush();
    if (!isReady()) {
        logging = false;
        Serial.println("⚠️ Messfahrt wegen SD-Schreibfehler abgebrochen");
        return;
    }

    writeRideSummaryFile();
    closeLogFiles();
    logging = false;

    Serial.println("✅ Messfahrt beendet und alle Dateien geschlossen");
    Serial.printf("   Dauer: %lu s, Strecke: %.2f km, Schlaglöcher: %lu, "
                  "Durchschnitt: %.1f\n",
                  rideSummary.durationSeconds, rideSummary.distanceKm,
                  rideSummary.potholeCount, rideSummary.averageQuality);
}

RideSummary SDLogger::getRideSummary() const {
    RideSummary result = rideSummary;
    if (logging) {
        result.active = true;
        result.durationSeconds = (millis() - sessionStartTime) / 1000;
        if (result.qualitySamples > 0) {
            result.averageQuality = qualitySum / result.qualitySamples;
        }
    }
    return result;
}

String SDLogger::generateFileName(LogType type) {
    String typeStr;
    switch(type) {
        case LOG_TYPE_SENSOR: typeStr = "sensor"; break;
        case LOG_TYPE_CAN: typeStr = "can"; break;
        case LOG_TYPE_ROAD: typeStr = "road"; break;
        case LOG_TYPE_EVENT: typeStr = "event"; break;
        case LOG_TYPE_SYSTEM: typeStr = "system"; break;
        case LOG_TYPE_GPS: typeStr = "gps"; break;
        case LOG_TYPE_CORRELATED: typeStr = "correlated"; break;
    }
    
    String fileName = "/" + config.filePrefix + "_" + typeStr;
    
    if (config.useTimestamp) {
        fileName += "_" + sessionId;
    }
    
    fileName += ".csv";
    return fileName;
}

String SDLogger::generateSessionId() {
    String baseId;
    time_t now = time(nullptr);
    struct tm utcTime = {};

    if (now > 0 && gmtime_r(&now, &utcTime) &&
        utcTime.tm_year + 1900 >= 2020) {
        char timestamp[24];
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &utcTime);
        baseId = timestamp;
    } else {
        baseId = "boot_" + String(millis());
    }

    String candidate = baseId;
    uint16_t suffix = 1;
    while (SD.exists("/" + config.filePrefix + "_sensor_" + candidate + ".csv") ||
           SD.exists("/" + config.filePrefix + "_road_" + candidate + ".csv") ||
           SD.exists("/" + config.filePrefix + "_gps_" + candidate + ".csv") ||
           SD.exists("/" + config.filePrefix + "_event_" + candidate + ".csv") ||
           SD.exists("/" + config.filePrefix + "_summary_" + candidate + ".csv") ||
           SD.exists("/" + config.filePrefix + "_can_" + candidate + ".csv") ||
           SD.exists("/" + config.filePrefix + "_correlated_" + candidate + ".csv")) {
        char suffixText[8];
        snprintf(suffixText, sizeof(suffixText), "_%02u", suffix++);
        candidate = baseId + suffixText;
    }
    return candidate;
}

bool SDLogger::createLogFile(LogType type) {
    currentFileName = generateFileName(type);
    currentLogFile = SD.open(currentFileName, FILE_WRITE);
    
    if (!currentLogFile) {
        Serial.printf("❌ Kann Datei nicht erstellen: %s\n", currentFileName.c_str());
        stats.errorCount++;
        return false;
    }

    if (!writeHeader(currentLogFile, type)) {
        handleCardFailure("CSV-Kopfzeile konnte nicht geschrieben werden");
        return false;
    }

    stats.fileCount++;
    return true;
}

bool SDLogger::openLogFile(File& file, String& fileName, LogType type) {
    fileName = generateFileName(type);
    file = SD.open(fileName, FILE_WRITE);
    if (!file) {
        Serial.printf("❌ Kann Datei nicht erstellen: %s\n", fileName.c_str());
        stats.errorCount++;
        return false;
    }

    if (!writeHeader(file, type)) {
        handleCardFailure("CSV-Kopfzeile konnte nicht geschrieben werden");
        return false;
    }

    stats.fileCount++;
    return true;
}

void SDLogger::closeLogFiles() {
    if (currentLogFile) currentLogFile.close();
    if (roadLogFile) roadLogFile.close();
    if (gpsLogFile) gpsLogFile.close();
    if (canLogFile) canLogFile.close();
    if (eventLogFile) eventLogFile.close();
    if (correlatedLogFile) correlatedLogFile.close();
}

void SDLogger::handleCardFailure(const char* reason, uint32_t droppedRecords) {
    if (!initialized && !cardAvailable) {
        return;
    }

    if (logging) {
        rideSummary.durationSeconds = (millis() - sessionStartTime) / 1000;
        rideSummary.endUTC = formatUTC();
        rideSummary.active = false;
        rideSummary.completed = false;
        rideSummary.interrupted = true;
    }

    stats.errorCount++;
    stats.droppedLogs += droppedRecords;
    closeLogFiles();
    logging = false;
    SD.end();
    initialized = false;
    cardAvailable = false;

    Serial.printf("⚠️ SD-Fehler: %s; Aufzeichnung beendet\n", reason);
}

bool SDLogger::writeHeader(File& file, LogType type) {
    const char* header = nullptr;
    switch(type) {
        case LOG_TYPE_SENSOR:
            header = "UTC,UptimeMs,RelativeHeading,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp,CalSystemInfo,CalGyro,CalAccel,CalMagUnused";
            break;
            
        case LOG_TYPE_EVENT:
            header = "UTC,UptimeMs,Ereignis,Beschreibung,Latitude,Longitude,Schwere";
            break;
            
        case LOG_TYPE_ROAD:
            header = "UTC,UptimeMs,Qualität,Glätte,KurvenProKm,VibrationRMS";
            break;
            
        case LOG_TYPE_CAN:
            header = "UTC,UptimeMs,CAN_ID,Extended,RTR,DLC,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7";
            break;
            
        case LOG_TYPE_SYSTEM:
            header = "UTC,UptimeMs,Status,Nachricht";
            break;

        case LOG_TYPE_GPS:
            header = "UTC,UptimeMs,Latitude,Longitude,AltitudeM,SpeedKmh,HeadingDeg,Satellites,ValidFix,HDOP";
            break;

        case LOG_TYPE_CORRELATED:
            header = "UTC,UptimeMs,Type,RelativeHeading,Pitch,Roll,AccelMag,Temp,CAN_ID,DLC,D0,D1,D2,D3,D4,D5,D6,D7";
            break;
    }

    return header != nullptr && file && file.println(header) > 0;
}

bool SDLogger::checkHealth() {
    if (!initialized || !cardAvailable) {
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    uint64_t totalBytes = SD.totalBytes();
    File root = SD.open("/");
    bool rootAvailable = static_cast<bool>(root);
    if (root) {
        root.close();
    }

    if (cardType == CARD_NONE || totalBytes == 0 || !rootAvailable) {
        handleCardFailure("SD-Karte nicht mehr erreichbar");
        return false;
    }
    
    return true;
}

bool SDLogger::writeRideSummaryFile() {
    if (!cardAvailable || sessionId.length() == 0) {
        return false;
    }

    String summaryFileName =
        "/" + config.filePrefix + "_summary_" + sessionId + ".csv";
    File summaryFile = SD.open(summaryFileName, FILE_WRITE);
    if (!summaryFile) {
        Serial.printf("❌ Fahrzusammenfassung konnte nicht erstellt werden: %s\n",
                      summaryFileName.c_str());
        stats.errorCount++;
        return false;
    }

    size_t headerWritten = summaryFile.println(
        "Session,StartUTC,EndUTC,DauerSekunden,StreckeKm,"
        "Schlagloecher,Kurven,Qualitaetswerte,Durchschnittsqualitaet");
    size_t summaryWritten = summaryFile.printf(
        "%s,%s,%s,%lu,%.3f,%lu,%lu,%lu,%.1f\n",
        rideSummary.sessionId.c_str(),
        rideSummary.startUTC.c_str(),
        rideSummary.endUTC.c_str(),
        static_cast<unsigned long>(rideSummary.durationSeconds),
        rideSummary.distanceKm,
        static_cast<unsigned long>(rideSummary.potholeCount),
        static_cast<unsigned long>(rideSummary.curveCount),
        static_cast<unsigned long>(rideSummary.qualitySamples),
        rideSummary.averageQuality);
    summaryFile.flush();
    summaryFile.close();

    if (headerWritten == 0 || summaryWritten == 0) {
        Serial.println("❌ Fahrzusammenfassung konnte nicht geschrieben werden");
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }

    stats.fileCount++;
    Serial.printf("✅ Fahrzusammenfassung: %s\n", summaryFileName.c_str());
    return true;
}

void SDLogger::flushBuffer() {
    if (bufferIndex > 0 && currentLogFile) {
        // Sicherheits-Check vor Buffer-Zugriff
        if (bufferIndex <= BUFFER_SIZE) {
            size_t bytesToWrite = bufferIndex;
            size_t bytesWritten =
                currentLogFile.write((const uint8_t*)writeBuffer, bytesToWrite);
            if (bytesWritten != bytesToWrite) {
                bufferIndex = 0;
                handleCardFailure(
                    "Sensorpuffer konnte nicht vollständig geschrieben werden", 1);
                return;
            }
        } else {
            // Kritischer Fehler: Buffer-Overflow erkannt
            Serial.printf("❌ KRITISCH: Buffer-Overflow erkannt! Index: %d, Max: %d\n", 
                         bufferIndex, BUFFER_SIZE);
            stats.errorCount++;
            
            // Notfall-Flush nur bis zur sicheren Grenze
            size_t emergencySize = BUFFER_SIZE - BUFFER_SAFETY_MARGIN;
            if (currentLogFile.write(
                    (const uint8_t*)writeBuffer, emergencySize) != emergencySize) {
                bufferIndex = 0;
                handleCardFailure(
                    "Notfallpuffer konnte nicht geschrieben werden", 1);
                return;
            }
        }
        bufferIndex = 0;
    }
}

bool SDLogger::safeAppendToBuffer(const char* data, size_t dataLen) {
    if (!data || dataLen == 0) return false;
    
    // Prüfe verfügbaren Platz mit Sicherheitspuffer
    size_t availableSpace = getAvailableBufferSpace();
    
    if (dataLen > availableSpace) {
        // Nicht genug Platz - Buffer leeren und erneut versuchen
        flushBuffer();
        availableSpace = getAvailableBufferSpace();
        
        // Wenn immer noch nicht genug Platz, Daten kürzen
        if (dataLen > availableSpace) {
            Serial.printf("⚠️ Datenzeile zu lang (%zu Bytes), kürze auf %zu\n", 
                         dataLen, availableSpace);
            dataLen = availableSpace;
        }
    }
    
    // Sichere Kopierung
    memcpy(writeBuffer + bufferIndex, data, dataLen);
    bufferIndex += dataLen;
    
    // Zusätzlicher Sicherheits-Check
    if (bufferIndex > BUFFER_SIZE) {
        Serial.println("❌ FATAL: Buffer-Index außerhalb der Grenzen!");
        bufferIndex = BUFFER_SIZE - BUFFER_SAFETY_MARGIN; // Notfall-Korrektur
        return false;
    }
    
    return true;
}

bool SDLogger::safeAppendToBuffer(const String& data) {
    return safeAppendToBuffer(data.c_str(), data.length());
}

size_t SDLogger::getAvailableBufferSpace() const {
    if (bufferIndex >= BUFFER_SIZE - BUFFER_SAFETY_MARGIN) {
        return 0;
    }
    return (BUFFER_SIZE - BUFFER_SAFETY_MARGIN) - bufferIndex;
}

String SDLogger::formatTimestamp() {
    unsigned long elapsed = millis() - sessionStartTime;
    return formatUTC() + "," + String(elapsed);
}

String SDLogger::formatUTC() {
    time_t now = time(nullptr);
    struct tm utcTime = {};
    if (now <= 0 || !gmtime_r(&now, &utcTime) ||
        utcTime.tm_year + 1900 < 2020) {
        return "";
    }

    char timestamp[24];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &utcTime);
    return String(timestamp);
}

bool SDLogger::logSensorData(const SensorData& data) {
    if (!logging || !config.enableSensorLog) return false;
    
    unsigned long now = millis();
    if (now - lastSensorLog < config.sensorLogInterval) {
        return true; // Noch nicht Zeit für nächsten Log
    }
    
    // Sichere Formatierung mit begrenzter Puffergröße
    char logBuffer[256]; // Maximale Zeilenlänge begrenzt
    int written = snprintf(logBuffer, sizeof(logBuffer), 
        "%s,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%d,%d,%d,%d\n",
        formatTimestamp().c_str(),
        data.heading, data.pitch, data.roll,
        data.accelX, data.accelY, data.accelZ,
        data.gyroX, data.gyroY, data.gyroZ,
        data.temperature,
        data.calibration.system, data.calibration.gyro, 
        data.calibration.accel, data.calibration.mag);
    
    // Überprüfe auf Truncation
    if (written >= sizeof(logBuffer)) {
        Serial.println("⚠️ Sensor-Log-Zeile wurde gekürzt!");
        stats.errorCount++;
    }
    
    // Verwende sichere Buffer-Append-Funktion
    bool success = safeAppendToBuffer(logBuffer, strlen(logBuffer));
    
    if (success) {
        stats.totalWrites++;
        stats.totalBytes += strlen(logBuffer);
        lastSensorLog = now;
        
        // Auto-Flush bei 80% Buffer-Auslastung
        if (getAvailableBufferSpace() < BUFFER_SIZE * 0.2) {
            flushBuffer();
        }
        return true;
    }
    
    stats.droppedLogs++;
    stats.errorCount++;
    return false;
}

bool SDLogger::logVibrationMetrics(const VibrationMetrics& metrics) {
    if (!logging || !config.enableRoadLog) return false;
    
    String logLine = formatTimestamp() + ",VIBRATION," +
                    String(metrics.rmsAccel, 3) + "," +
                    String(metrics.maxShock, 3) + "," +
                    String(metrics.frequency, 1) + "," +
                    String(metrics.shockCount) + "\n";
    
    if (eventLogFile &&
        eventLogFile.print(logLine) == logLine.length()) {
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }

    handleCardFailure("Vibrationsdaten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logCalibration(const CalibrationData& cal) {
    if (!logging) return false;
    
    String status = cal.isFullyCalibrated() ? "IMUPLUS_OK" : "IMUPLUS_UNVOLLSTÄNDIG";
    return logEvent("KALIBRIERUNG", status + " Gyr:" + String(cal.gyro) +
                    " Acc:" + String(cal.accel));
}

bool SDLogger::logCANMessage(const CANMessage& msg) {
    if (!logging || !config.enableCANLog) return false;

    if (!canLogFile &&
        !openLogFile(canLogFile, canFileName, LOG_TYPE_CAN)) {
        return false;
    }
    
    // Sichere CAN-Message-Formatierung mit begrenztem Buffer
    char logBuffer[128]; // Ausreichend für CAN-Message (max 64 Zeichen + Header)
    char dataBytes[32] = {0}; // Buffer für Daten-Bytes
    
    // Formatiere Daten-Bytes sicher
    for (int i = 0; i < 8; i++) {
        char byteStr[8];
        if (i < msg.dlc && i >= 0) {
            snprintf(byteStr, sizeof(byteStr), ",%02X", msg.data[i]);
        } else {
            snprintf(byteStr, sizeof(byteStr), ",");
        }
        
        // Sichere String-Verkettung
        size_t currentLen = strlen(dataBytes);
        size_t remainingSpace = sizeof(dataBytes) - currentLen - 1;
        if (strlen(byteStr) < remainingSpace) {
            strncat(dataBytes, byteStr, remainingSpace);
        } else {
            Serial.println("⚠️ CAN-Daten-Buffer-Overflow verhindert!");
            break;
        }
    }
    
    // Formatiere komplette CAN-Message
    int written = snprintf(logBuffer, sizeof(logBuffer), 
        "%s,%lX,%d,%d,%d%s\n",
        formatTimestamp().c_str(), msg.canId, msg.extended ? 1 : 0,
        msg.rtr ? 1 : 0, msg.dlc, dataBytes);
    
    // Überprüfe Truncation
    if (written >= sizeof(logBuffer)) {
        Serial.println("⚠️ CAN-Log-Zeile wurde gekürzt!");
        stats.errorCount++;
    }
    
    size_t logLength = strlen(logBuffer);
    if (canLogFile && canLogFile.print(logBuffer) == logLength) {
        stats.totalWrites++;
        stats.totalBytes += logLength;
        return true;
    }

    handleCardFailure("CAN-Daten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logGPSData(const GPSData& gps) {
    if (!logging || !config.enableGPSLog) return false;
    
    unsigned long now = millis();
    if (now - lastGPSLog < config.gpsLogInterval) {
        return true;
    }
    
    // Sichere GPS-Daten-Formatierung
    char logBuffer[256];
    String utc = gps.datetime_valid ? formatGPSDateTimeUTC(gps) : formatUTC();
    int written = snprintf(logBuffer, sizeof(logBuffer),
        "%s,%lu,%.6f,%.6f,%.2f,%.1f,%.1f,%d,%d,%.2f\n",
        utc.c_str(), now - sessionStartTime,
        gps.latitude, gps.longitude, gps.altitude,
        gps.speed_kmh, gps.heading_deg, gps.satellites, 
        gps.valid_fix ? 1 : 0, gps.hdop);
    
    // Überprüfe Truncation
    if (written >= sizeof(logBuffer)) {
        Serial.println("⚠️ GPS-Log-Zeile wurde gekürzt!");
        stats.errorCount++;
    }
    
    size_t logLength = strlen(logBuffer);
    if (gpsLogFile && gpsLogFile.print(logBuffer) == logLength) {
        stats.totalWrites++;
        stats.totalBytes += logLength;
        lastGPSLog = now;

        // Strecke nur aus plausiblen, bewegten GPS-Punkten bilden. Das
        // verhindert, dass Positionsrauschen im Stand als Weg gezählt wird.
        if (gps.valid_fix && gps.location_valid &&
            (gps.hdop <= 10.0f || gps.hdop == 0.0f)) {
            if (!hasLastRideGPS) {
                lastRideLatitude = gps.latitude;
                lastRideLongitude = gps.longitude;
                lastRideGPSTime = now;
                hasLastRideGPS = true;
            } else if (gps.speed_valid && gps.speed_kmh < 1.5f) {
                lastRideLatitude = gps.latitude;
                lastRideLongitude = gps.longitude;
                lastRideGPSTime = now;
            } else {
                float segmentMeters = calculateDistance(
                    lastRideLatitude, lastRideLongitude,
                    gps.latitude, gps.longitude);
                float elapsedSeconds =
                    max((now - lastRideGPSTime) / 1000.0f, 0.2f);
                float speedMetersPerSecond =
                    max(gps.speed_kmh, 5.0f) / 3.6f;
                float maxPlausibleSegment =
                    max(30.0f, speedMetersPerSecond * elapsedSeconds * 3.0f + 10.0f);

                if (segmentMeters >= 1.0f &&
                    segmentMeters <= maxPlausibleSegment) {
                    rideSummary.distanceKm += segmentMeters / 1000.0f;
                    lastRideLatitude = gps.latitude;
                    lastRideLongitude = gps.longitude;
                    lastRideGPSTime = now;
                }
            }
        }
        return true;
    }
    
    handleCardFailure("GPS-Daten konnten nicht geschrieben werden", 1);
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
    
    if (roadLogFile &&
        roadLogFile.print(logLine) == logLine.length()) {
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        lastRoadLog = now;
        qualitySum += quality;
        rideSummary.qualitySamples++;
        rideSummary.averageQuality =
            qualitySum / rideSummary.qualitySamples;
        return true;
    }

    handleCardFailure("Straßenqualitätsdaten konnten nicht geschrieben werden", 1);
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
    
    if (eventLogFile &&
        eventLogFile.print(logLine) == logLine.length()) {
        eventLogFile.flush(); // Ereignisse sofort speichern
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }

    handleCardFailure("Ereignisdaten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logPothole(float severity, float lat, float lon) {
    String sevStr;
    if (severity < 2.0) sevStr = "KLEIN";
    else if (severity < 5.0) sevStr = "MITTEL";
    else sevStr = "GROSS";
    
    bool logged = logEvent(
        "SCHLAGLOCH", sevStr + " (" + String(severity, 1) + " m/s²)",
        lat, lon);
    if (logged) {
        rideSummary.potholeCount++;
    }
    return logged;
}

bool SDLogger::logCurve(float angle, float radius, float lat, float lon) {
    String desc = "Winkel: " + String(angle, 0) + "°";
    if (radius > 0) {
        desc += ", Radius: " + String(radius, 0) + "m";
    }
    
    bool logged = logEvent("KURVE", desc, lat, lon);
    if (logged) {
        rideSummary.curveCount++;
    }
    return logged;
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
    if (roadLogFile) roadLogFile.flush();
    if (gpsLogFile) gpsLogFile.flush();
    if (canLogFile) canLogFile.flush();
    if (eventLogFile) eventLogFile.flush();
    if (correlatedLogFile) correlatedLogFile.flush();
    
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
    
    uint64_t totalBytes = SD.totalBytes();
    uint64_t usedBytes = SD.usedBytes();
    if (totalBytes == 0 || usedBytes > totalBytes) {
        return 0;
    }
    uint64_t freeBytes = totalBytes - usedBytes;
    return freeBytes / 1024; // KB zurückgeben
}

void SDLogger::resetStatistics() {
    stats.totalWrites = 0;
    stats.totalBytes = 0;
    stats.droppedLogs = 0;
    stats.fileCount = 0;
    stats.errorCount = 0;
    stats.bufferOverflows = 0;
    stats.startTime = millis();
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

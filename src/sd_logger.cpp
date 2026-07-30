#include "sd_logger.h"
#include "road_quality.h"
#include "gps_manager.h"
#include "runtime_diagnostics.h"
#include <esp_system.h>
#include <time.h>

// Forward-Deklaration
float calculateOverallQuality(const RoadMetrics& metrics);

namespace {
uint32_t sessionCounterDelta(uint32_t current, uint32_t start) {
    // Auch bei einem unerwarteten Zählerneustart oder 32-Bit-Überlauf niemals
    // einen unsigned-Unterlauf als riesigen Sitzungswert protokollieren.
    return current >= start ? current - start : current;
}

size_t writeFileTimed(File& file, const char* data, size_t length) {
    const unsigned long startedAt = millis();
    const size_t written = file.write(
        reinterpret_cast<const uint8_t*>(data), length);
    runtimeDiagnostics.recordSDDuration(millis() - startedAt);
    return written;
}

void flushFileTimed(File& file) {
    const unsigned long startedAt = millis();
    file.flush();
    runtimeDiagnostics.recordSDDuration(millis() - startedAt);
}

void advanceLogSchedule(
    unsigned long& lastLog, unsigned long now,
    uint32_t intervalMs) {
    if (lastLog == 0 || intervalMs == 0) {
        lastLog = now;
        return;
    }
    const unsigned long elapsed = now - lastLog;
    lastLog +=
        max(elapsed / intervalMs, 1UL) * intervalMs;
}
}

// Globale Instanz
SDLogger sdLogger;

SDLogger::SDLogger(int cs) 
    : csPin(cs), spiInstance(nullptr), initialized(false), 
      cardAvailable(false), logging(false),
      loggingStartState(LoggingStartState::IDLE), bufferIndex(0),
      lastSensorLog(0), lastRoadLog(0), lastGPSLog(0), lastFlush(0),
      lastMetadataLog(0), sessionStartTime(0),
      gpsSessionStartStatus{}, gpsSessionStartFixSequence(0),
      gpsSessionLastLoggedFixSequence(0),
      canSessionStartDiagnostics{}, qualitySum(0),
      hasLastRideGPS(false),
      lastRideLatitude(0), lastRideLongitude(0), lastRideGPSTime(0),
      bufferedRecordCount(0) {
    
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
        writeFileTimed(
            correlatedLogFile, logLine.c_str(), logLine.length()) ==
            logLine.length()) {
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
    // Die neun Sitzungslogs werden vor Messbeginn geöffnet; Root-Prüfung und
    // Abschlusszusammenfassung benötigen zusätzlich eigene Datei-Handles.
    if (!SD.begin(
            csPin, spi, SD_SPI_SPEED, "/sd", SD_MAX_OPEN_FILES, false)) {
        Serial.println("❌ SD-Karte nicht gefunden!");
        return false;
    }

    // Neue Fahrten liegen jeweils in einem eigenen Unterverzeichnis. Dadurch
    // muss FAT beim Anlegen einer Sitzung nicht für jede der neun Dateien das
    // mit alten Messungen gefüllte Wurzelverzeichnis erneut durchsuchen.
    if (!SD.exists("/sessions") && !SD.mkdir("/sessions")) {
        Serial.println("❌ SD-Sitzungsverzeichnis konnte nicht angelegt werden");
        SD.end();
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
    if (isLoggingStartPending()) {
        failLoggingStart("Start durch SD-Abschaltung abgebrochen");
    }

    if (logging) {
        stopLogging();
    }
    
    closeLogFiles();
    
    SD.end();
    initialized = false;
    cardAvailable = false;
}

bool SDLogger::startLogging() {
    if (logging) {
        return true;
    }

    if (!isLoggingStartPending() && !requestLoggingStart()) {
        return false;
    }

    // Kompatibler blockierender Einstieg für serielle Kommandos und Tests.
    // Die Weboberfläche verteilt dieselben Schritte über mehrere loop()-Runden.
    while (isLoggingStartPending()) {
        processLoggingStart();
        delay(0);
    }
    return logging;
}

bool SDLogger::requestLoggingStart() {
    if (!initialized || !cardAvailable) {
        Serial.println("SD Logger nicht bereit!");
        lastStartError = "SD-Karte ist nicht bereit";
        return false;
    }
    
    if (logging) {
        return true;
    }

    if (isLoggingStartPending()) {
        return true;
    }

    lastStartError = "";
    loggingStartState = LoggingStartState::PREPARE;
    Serial.println("SD-Aufzeichnung wird vorbereitet");
    return true;
}

void SDLogger::processLoggingStart() {
    switch (loggingStartState) {
        case LoggingStartState::IDLE:
            return;

        case LoggingStartState::PREPARE:
            closeLogFiles();
            sessionId = generateSessionId();
            sessionDirectory = "/sessions/" + sessionId;
            if (!SD.mkdir(sessionDirectory)) {
                failLoggingStart(
                    "Sitzungsverzeichnis konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_SENSOR;
            return;

        case LoggingStartState::OPEN_SENSOR:
            if (!createLogFile(LOG_TYPE_SENSOR)) {
                failLoggingStart("Sensor-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_ROAD;
            return;

        case LoggingStartState::OPEN_ROAD:
            if (config.enableRoadLog &&
                !openLogFile(roadLogFile, roadFileName, LOG_TYPE_ROAD)) {
                failLoggingStart("Straßen-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_GPS;
            return;

        case LoggingStartState::OPEN_GPS:
            if (config.enableGPSLog &&
                !openLogFile(gpsLogFile, gpsFileName, LOG_TYPE_GPS)) {
                failLoggingStart("GPS-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_EVENT;
            return;

        case LoggingStartState::OPEN_EVENT:
            if (config.enableEventLog &&
                !openLogFile(eventLogFile, eventFileName, LOG_TYPE_EVENT)) {
                failLoggingStart("Ereignis-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_META;
            return;

        case LoggingStartState::OPEN_META:
            if (!openLogFile(metaLogFile, metaFileName, LOG_TYPE_META)) {
                failLoggingStart("Metadaten-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_CAN;
            return;

        case LoggingStartState::OPEN_CAN:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(canLogFile, canFileName, LOG_TYPE_CAN)) {
                failLoggingStart("CAN-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_OBD;
            return;

        case LoggingStartState::OPEN_OBD:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(obdLogFile, obdFileName, LOG_TYPE_OBD)) {
                failLoggingStart("OBD-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_OBD_TRACE;
            return;

        case LoggingStartState::OPEN_OBD_TRACE:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(
                    obdTraceLogFile, obdTraceFileName, LOG_TYPE_OBD_TRACE)) {
                failLoggingStart("OBD-Trace konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_CORRELATED;
            return;

        case LoggingStartState::OPEN_CORRELATED:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(
                    correlatedLogFile, correlatedFileName,
                    LOG_TYPE_CORRELATED)) {
                failLoggingStart(
                    "Korrelations-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::FINALIZE;
            return;

        case LoggingStartState::FINALIZE:
            break;
    }

    sessionStartTime = millis();
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
    bufferedRecordCount = 0;
    lastSensorLog = 0;
    lastRoadLog = 0;
    lastGPSLog = 0;
    lastMetadataLog = 0;
    gpsSessionStartStatus = gpsManager.getStatus();
    gpsSessionStartFixSequence = gpsManager.getFixSequence();
    gpsSessionLastLoggedFixSequence = gpsSessionStartFixSequence;
    if (canReader.isReady()) {
        canSessionStartDiagnostics =
            canReader.getHardwareDiagnostics();
        canReader.beginOBDSession();
    } else {
        canSessionStartDiagnostics = CANHardwareDiagnostics{};
    }
    runtimeDiagnostics.resetSession();
    logging = true;
    loggingStartState = LoggingStartState::IDLE;
    const bool startMetadataWritten = logSessionMetadata(
        "START", gpsManager.getStatus(), canReader.getOBDSessionStats(),
        canReader.getHardwareDiagnostics());
    if (!startMetadataWritten) {
        if (logging) {
            failLoggingStart(
                "START-Metadaten konnten nicht gespeichert werden");
        }
        return;
    }
    Serial.printf("✅ Sensor-Log: %s\n", currentFileName.c_str());
    if (roadLogFile) Serial.printf("✅ Straßen-Log: %s\n", roadFileName.c_str());
    if (gpsLogFile) Serial.printf("✅ GPS-Log: %s\n", gpsFileName.c_str());
    if (eventLogFile) Serial.printf("✅ Ereignis-Log: %s\n", eventFileName.c_str());
    if (metaLogFile) Serial.printf("✅ Metadaten-Log: %s\n", metaFileName.c_str());
    if (canLogFile) Serial.printf("✅ CAN-Log: %s\n", canFileName.c_str());
    if (obdLogFile) Serial.printf("✅ OBD-Log: %s\n", obdFileName.c_str());
    if (obdTraceLogFile) {
        Serial.printf("✅ OBD-Trace: %s\n", obdTraceFileName.c_str());
    }
    if (correlatedLogFile) {
        Serial.printf("✅ Korrelations-Log: %s\n", correlatedFileName.c_str());
    }
}

void SDLogger::failLoggingStart(const String& reason) {
    if (logging) {
        rideSummary.durationSeconds =
            (millis() - sessionStartTime) / 1000;
        rideSummary.endUTC = formatUTC();
        rideSummary.active = false;
        rideSummary.completed = false;
        rideSummary.interrupted = true;
        if (canReader.isOBDSessionActive()) {
            canReader.endOBDSession();
        }
    }
    closeLogFiles();
    logging = false;
    loggingStartState = LoggingStartState::IDLE;
    lastStartError = reason;
    Serial.printf("❌ SD-Aufzeichnungsstart fehlgeschlagen: %s\n",
                  reason.c_str());
}

void SDLogger::stopLogging() {
    if (isLoggingStartPending()) {
        failLoggingStart("Start abgebrochen");
        return;
    }

    if (!logging) return;

    rideSummary.durationSeconds = (millis() - sessionStartTime) / 1000;
    rideSummary.endUTC = formatUTC();
    rideSummary.active = false;
    rideSummary.completed = false;
    rideSummary.interrupted = false;
    if (rideSummary.qualitySamples > 0) {
        rideSummary.averageQuality =
            qualitySum / rideSummary.qualitySamples;
    }

    canReader.updateOBDDiagnostics();
    const CANHardwareDiagnostics finalCANDiagnostics =
        canReader.getHardwareDiagnostics();
    OBDTraceEvent finalTraceEvent;
    while (canReader.popOBDTraceEvent(finalTraceEvent)) {
        logOBDTraceEvent(finalTraceEvent, finalCANDiagnostics);
    }
    const bool endMetadataWritten = logSessionMetadata(
        "END", gpsManager.getStatus(), canReader.getOBDSessionStats(),
        finalCANDiagnostics);
    canReader.endOBDSession();
    flush();
    if (!isReady()) {
        logging = false;
        Serial.println("⚠️ Messfahrt wegen SD-Schreibfehler abgebrochen");
        return;
    }

    const bool summaryWritten = writeRideSummaryFile();
    const bool sessionComplete = endMetadataWritten && summaryWritten;
    rideSummary.completed = sessionComplete;
    rideSummary.interrupted = !sessionComplete;
    if (!sessionComplete && logging && eventLogFile) {
        String incompleteReason;
        if (!endMetadataWritten) {
            incompleteReason = "END-Metadaten fehlen";
        }
        if (!summaryWritten) {
            if (incompleteReason.length() > 0) {
                incompleteReason += ";";
            }
            incompleteReason += "Fahrzusammenfassung fehlt";
        }
        logEvent(
            "SESSION_INCOMPLETE",
            incompleteReason);
    }
    closeLogFiles();
    logging = false;

    Serial.println(
        sessionComplete
            ? "✅ Messfahrt beendet und alle Dateien geschlossen"
            : "⚠️ Messfahrt beendet, Abschlussdaten unvollständig");
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
        case LOG_TYPE_OBD: typeStr = "obd"; break;
        case LOG_TYPE_OBD_TRACE: typeStr = "obd_trace"; break;
        case LOG_TYPE_META: typeStr = "meta"; break;
        case LOG_TYPE_CORRELATED: typeStr = "correlated"; break;
    }
    
    String fileName = sessionDirectory + "/" +
                      config.filePrefix + "_" + typeStr;
    
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

    // Der Zufallsanteil vermeidet Namenskollisionen ohne wiederholte,
    // synchrone FAT-Verzeichnissuchen. Diese wurden mit wachsender Anzahl
    // vorhandener Fahrten zunehmend teuer.
    char uniqueSuffix[12];
    snprintf(uniqueSuffix, sizeof(uniqueSuffix), "_%08lX",
             static_cast<unsigned long>(esp_random()));
    return baseId + uniqueSuffix;
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
    if (obdLogFile) obdLogFile.close();
    if (obdTraceLogFile) obdTraceLogFile.close();
    if (metaLogFile) metaLogFile.close();
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
    stats.droppedLogs += droppedRecords + bufferedRecordCount;
    bufferIndex = 0;
    bufferedRecordCount = 0;
    if (canReader.isOBDSessionActive()) {
        canReader.endOBDSession();
    }
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
            header = "UTC,UptimeMs,Heading,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp,CalSystem,CalGyro,CalAccel,CalMag";
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
            header =
                "UTC,UptimeMs,Latitude,Longitude,AltitudeM,SpeedKmh,"
                "HeadingDeg,Satellites,ValidFix,HDOP,"
                "LocationValid,LocationAgeMs,SpeedValid,SpeedAgeMs,"
                "AltitudeValid,AltitudeAgeMs,CourseValid,CourseAgeMs,"
                "SatellitesValid,SatellitesAgeMs,HDOPValid,HDOPAgeMs,"
                "NewFix,FixSequence,NMEAChars,NMEASentencesValid,"
                "NMEAChecksumFailures,RXBufferOverflows,"
                "Rejected,RejectionReason";
            break;

        case LOG_TYPE_OBD:
            header =
                "UTC,UptimeMs,LastPID,"
                "RPMValid,RPM,SpeedValid,SpeedKmh,"
                "ThrottleValid,ThrottlePercent,"
                "MAFValid,MAFGramsPerSecond,"
                "AmbientValid,AmbientC,OilValid,OilC,"
                "FuelRateValid,FuelRateLitersPerHour,"
                "Support00Valid,Support00,Support20Valid,Support20,"
                "Support40Valid,Support40,Support60Valid,Support60,"
                "Requests,Responses,SendErrors,"
                "SessionActive,SessionRequests,SessionResponses,"
                "SessionSendErrors,SessionTimeouts,UnmatchedResponses,"
                "TraceDropped,RequestSequence,RequestPID,TransmitOK,"
                "ResponsePID,ResponseECU,ResponseLatencyMs,"
                "CANMode,TEC,REC,EFLG,"
                "RX0OverflowCount,RX1OverflowCount,CANRecoveryCount";
            break;

        case LOG_TYPE_OBD_TRACE:
            header =
                "UTC,UptimeMs,Event,Sequence,PID,TransmitOK,"
                "MatchedRequest,RequestUptimeMs,ResponseUptimeMs,"
                "ResponseLatencyMs,ResponseECU,CANMode,TEC,REC,EFLG,"
                "RX0OverflowCount,RX1OverflowCount,CANRecoveryCount";
            break;

        case LOG_TYPE_META:
            header =
                "Record,UTC,UptimeMs,FirmwareVersion,SchemaVersion,"
                "SessionId,VehicleProfile,CANBitrate,CANClockHz,"
                "OBDRequestIntervalMs,OBDResponseTimeoutMs,GPSLogIntervalMs,"
                "GPSLocationMaxAgeMs,GPSSpeedMaxAgeMs,GPSQualityMaxAgeMs,"
                "GPSMinSatellites,GPSMaxHDOP,GPSMinReliableSpeedKmh,"
                "GPSOBDStationaryKmh,"
                "GPSChars,GPSSentencesValid,GPSChecksumFailures,"
                "GPSRXBufferOverflows,GPSBufferedBytes,"
                "OBDSessionActive,OBDRequests,OBDResponses,OBDSendErrors,"
                "OBDTimeouts,OBDUnmatchedResponses,OBDTraceDropped,"
                "CANMode,TEC,REC,EFLG,RX0OverflowCount,RX1OverflowCount,"
                "CANRecoveryCount,"
                "SDWrites,SDErrors,SDDropped,"
                "LoopLastMs,LoopMaxMs,LoopTotalMs,LoopSamples,LoopStalls,"
                "WebLastMs,WebMaxMs,WebTotalMs,WebSamples,WebStalls,"
                "SDLastMs,SDMaxMs,SDTotalMs,SDSamples,SDStalls,"
                "SensorSamples,SensorMissedSlots,"
                "GPSSnapshots,GPSMissedSlots";
            break;

        case LOG_TYPE_CORRELATED:
            header = "UTC,UptimeMs,Type,Heading,Pitch,Roll,AccelMag,Temp,CAN_ID,DLC,D0,D1,D2,D3,D4,D5,D6,D7";
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
        sessionDirectory + "/" + config.filePrefix +
        "_summary_" + sessionId + ".csv";
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

bool SDLogger::flushBuffer() {
    if (bufferIndex == 0) {
        return true;
    }

    if (!currentLogFile) {
        handleCardFailure(
            "Sensorpuffer ohne geöffnete Sensordatei");
        return false;
    }

    if (bufferIndex > 0) {
        // Sicherheits-Check vor Buffer-Zugriff
        if (bufferIndex <= BUFFER_SIZE) {
            size_t bytesToWrite = bufferIndex;
            const unsigned long writeStartedAt = millis();
            size_t bytesWritten =
                currentLogFile.write((const uint8_t*)writeBuffer, bytesToWrite);
            runtimeDiagnostics.recordSDDuration(
                millis() - writeStartedAt);
            if (bytesWritten != bytesToWrite) {
                bufferIndex = 0;
                handleCardFailure(
                    "Sensorpuffer konnte nicht vollständig geschrieben werden");
                return false;
            }
            stats.totalWrites += bufferedRecordCount;
            stats.totalBytes += bytesWritten;
        } else {
            // Niemals einen abgeschnittenen CSV-Puffer schreiben. Die Sitzung
            // wird sichtbar abgebrochen, damit keine scheinbar gültige,
            // tatsächlich beschädigte letzte Zeile entsteht.
            Serial.printf(
                "❌ KRITISCH: Buffer-Grenze verletzt! Index: %d, Max: %d\n",
                bufferIndex, BUFFER_SIZE);
            stats.bufferOverflows++;
            bufferIndex = 0;
            handleCardFailure("Sensorpuffer-Grenze verletzt");
            return false;
        }
        bufferIndex = 0;
        bufferedRecordCount = 0;
    }
    return true;
}

bool SDLogger::safeAppendToBuffer(const char* data, size_t dataLen) {
    if (!data || dataLen == 0) return false;
    
    // Prüfe verfügbaren Platz mit Sicherheitspuffer
    size_t availableSpace = getAvailableBufferSpace();
    
    if (dataLen > availableSpace) {
        // Nicht genug Platz - Buffer leeren und erneut versuchen
        if (!flushBuffer()) {
            return false;
        }
        availableSpace = getAvailableBufferSpace();
        
        // CSV-Zeilen niemals kürzen: Eine beschädigte Zeile wäre für die
        // spätere Auswertung gefährlicher als ein sichtbar verworfener Satz.
        if (dataLen > availableSpace) {
            Serial.printf(
                "⚠️ Datenzeile zu lang (%zu Bytes, verfügbar %zu)\n",
                dataLen, availableSpace);
            stats.bufferOverflows++;
            return false;
        }
    }
    
    // Sichere Kopierung
    memcpy(writeBuffer + bufferIndex, data, dataLen);
    bufferIndex += dataLen;
    
    // Zusätzlicher Sicherheits-Check
    if (bufferIndex > BUFFER_SIZE) {
        Serial.println("❌ FATAL: Buffer-Index außerhalb der Grenzen!");
        bufferIndex = 0;
        handleCardFailure("Sensorpuffer-Index außerhalb der Grenzen");
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
    if (lastSensorLog != 0 &&
        now - lastSensorLog < config.sensorLogInterval) {
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
    if (written < 0 || written >= static_cast<int>(sizeof(logBuffer))) {
        Serial.println("⚠️ Sensor-Log-Zeile wurde verworfen!");
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }
    
    // Verwende sichere Buffer-Append-Funktion
    bool success = safeAppendToBuffer(logBuffer, strlen(logBuffer));
    
    if (success) {
        bufferedRecordCount++;
        advanceLogSchedule(
            lastSensorLog, now, config.sensorLogInterval);
        
        // Auto-Flush bei 80% Buffer-Auslastung
        if (getAvailableBufferSpace() < BUFFER_SIZE * 0.2) {
            if (!flushBuffer()) {
                return false;
            }
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
        writeFileTimed(
            eventLogFile, logLine.c_str(), logLine.length()) ==
            logLine.length()) {
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        return true;
    }

    handleCardFailure("Vibrationsdaten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logCalibration(const CalibrationData& cal) {
    if (!logging) return false;
    
    String status = String(ROADTEST_BNO_MODE_NAME) +
                    (cal.isFullyCalibrated() ? "_OK" : "_UNVOLLSTÄNDIG");
    return logEvent("KALIBRIERUNG", status + " Sys:" + String(cal.system) +
                    " Gyr:" + String(cal.gyro) +
                    " Acc:" + String(cal.accel) +
                    " Mag:" + String(cal.mag));
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
    if (written < 0 || written >= static_cast<int>(sizeof(logBuffer))) {
        Serial.println("⚠️ CAN-Log-Zeile wurde verworfen!");
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }
    
    size_t logLength = strlen(logBuffer);
    if (canLogFile &&
        writeFileTimed(canLogFile, logBuffer, logLength) == logLength) {
        stats.totalWrites++;
        stats.totalBytes += logLength;
        return true;
    }

    handleCardFailure("CAN-Daten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logOBDData(
    const OBDLiveData& obd, const OBDSessionStats& session,
    const CANHardwareDiagnostics& diagnostics) {
    if (!logging || !config.enableCANLog) {
        return false;
    }

    if (!obdLogFile &&
        !openLogFile(obdLogFile, obdFileName, LOG_TYPE_OBD)) {
        return false;
    }

    char logBuffer[768];
    const int written = snprintf(
        logBuffer, sizeof(logBuffer),
        "%s,%02X,"
        "%d,%.2f,%d,%u,"
        "%d,%.3f,"
        "%d,%.3f,"
        "%d,%.1f,%d,%.1f,"
        "%d,%.3f,"
        "%d,%08lX,%d,%08lX,%d,%08lX,%d,%08lX,"
        "%lu,%lu,%lu,"
        "%d,%lu,%lu,%lu,%lu,%lu,%lu,"
        "%lu,%02X,%d,%02X,%03lX,%lu,"
        "%02X,%u,%u,%02X,%lu,%lu,%lu\n",
        formatTimestamp().c_str(), obd.lastPid,
        obd.rpmValid ? 1 : 0, obd.rpm,
        obd.speedValid ? 1 : 0, obd.speedKmh,
        obd.throttleValid ? 1 : 0, obd.throttlePercent,
        obd.mafValid ? 1 : 0, obd.mafGramsPerSecond,
        obd.ambientTemperatureValid ? 1 : 0, obd.ambientTemperatureC,
        obd.oilTemperatureValid ? 1 : 0, obd.oilTemperatureC,
        obd.fuelRateValid ? 1 : 0, obd.fuelRateLitersPerHour,
        obd.supportBlockValid[0] ? 1 : 0,
        static_cast<unsigned long>(obd.supportBitmap[0]),
        obd.supportBlockValid[1] ? 1 : 0,
        static_cast<unsigned long>(obd.supportBitmap[1]),
        obd.supportBlockValid[2] ? 1 : 0,
        static_cast<unsigned long>(obd.supportBitmap[2]),
        obd.supportBlockValid[3] ? 1 : 0,
        static_cast<unsigned long>(obd.supportBitmap[3]),
        obd.requestCount, obd.responseCount, obd.requestErrors,
        session.active ? 1 : 0,
        static_cast<unsigned long>(session.requestCount),
        static_cast<unsigned long>(session.responseCount),
        static_cast<unsigned long>(session.requestErrors),
        static_cast<unsigned long>(session.timeoutCount),
        static_cast<unsigned long>(session.unmatchedResponseCount),
        static_cast<unsigned long>(session.traceDropped),
        static_cast<unsigned long>(session.lastRequestSequence),
        session.lastRequestPid,
        session.lastTransmitOK ? 1 : 0,
        session.lastResponsePid,
        static_cast<unsigned long>(session.lastResponseCanId),
        static_cast<unsigned long>(session.lastResponseLatencyMs),
        diagnostics.operatingMode,
        diagnostics.transmitErrorCount,
        diagnostics.receiveErrorCount,
        diagnostics.errorFlags,
        static_cast<unsigned long>(sessionCounterDelta(
            diagnostics.receiveBuffer0OverflowCount,
            canSessionStartDiagnostics.receiveBuffer0OverflowCount)),
        static_cast<unsigned long>(sessionCounterDelta(
            diagnostics.receiveBuffer1OverflowCount,
            canSessionStartDiagnostics.receiveBuffer1OverflowCount)),
        static_cast<unsigned long>(sessionCounterDelta(
            diagnostics.controllerRecoveryCount,
            canSessionStartDiagnostics.controllerRecoveryCount)));

    if (written < 0 || written >= static_cast<int>(sizeof(logBuffer))) {
        Serial.println("⚠️ OBD-Log-Zeile konnte nicht vollständig formatiert werden");
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }

    const size_t logLength = static_cast<size_t>(written);
    if (obdLogFile &&
        writeFileTimed(obdLogFile, logBuffer, logLength) == logLength) {
        stats.totalWrites++;
        stats.totalBytes += logLength;
        return true;
    }

    handleCardFailure("OBD-Daten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logOBDTraceEvent(
    const OBDTraceEvent& event,
    const CANHardwareDiagnostics& diagnostics) {
    if (!logging || !config.enableCANLog) {
        return false;
    }

    if (!obdTraceLogFile &&
        !openLogFile(
            obdTraceLogFile, obdTraceFileName, LOG_TYPE_OBD_TRACE)) {
        return false;
    }

    const uint32_t eventSessionMs =
        event.eventUptimeMs >= sessionStartTime
            ? event.eventUptimeMs - sessionStartTime
            : 0;
    const uint32_t requestSessionMs =
        event.requestUptimeMs >= sessionStartTime
            ? event.requestUptimeMs - sessionStartTime
            : 0;
    const uint32_t responseSessionMs =
        event.responseUptimeMs >= sessionStartTime
            ? event.responseUptimeMs - sessionStartTime
            : 0;

    char logBuffer[256];
    const int written = snprintf(
        logBuffer, sizeof(logBuffer),
        "%s,%lu,%s,%lu,%02X,%d,%d,%lu,%lu,%lu,%03lX,"
        "%02X,%u,%u,%02X,%lu,%lu,%lu\n",
        formatUTC().c_str(),
        static_cast<unsigned long>(eventSessionMs),
        obdTraceEventName(event.type),
        static_cast<unsigned long>(event.sequence),
        event.pid,
        event.transmitOK ? 1 : 0,
        event.matchedRequest ? 1 : 0,
        static_cast<unsigned long>(requestSessionMs),
        static_cast<unsigned long>(responseSessionMs),
        static_cast<unsigned long>(event.responseLatencyMs),
        static_cast<unsigned long>(event.responseCanId),
        diagnostics.operatingMode,
        diagnostics.transmitErrorCount,
        diagnostics.receiveErrorCount,
        diagnostics.errorFlags,
        static_cast<unsigned long>(sessionCounterDelta(
            diagnostics.receiveBuffer0OverflowCount,
            canSessionStartDiagnostics.receiveBuffer0OverflowCount)),
        static_cast<unsigned long>(sessionCounterDelta(
            diagnostics.receiveBuffer1OverflowCount,
            canSessionStartDiagnostics.receiveBuffer1OverflowCount)),
        static_cast<unsigned long>(sessionCounterDelta(
            diagnostics.controllerRecoveryCount,
            canSessionStartDiagnostics.controllerRecoveryCount)));

    if (written < 0 || written >= static_cast<int>(sizeof(logBuffer))) {
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }

    const size_t length = static_cast<size_t>(written);
    if (writeFileTimed(obdTraceLogFile, logBuffer, length) == length) {
        stats.totalWrites++;
        stats.totalBytes += length;
        return true;
    }

    handleCardFailure("OBD-Trace konnte nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logSessionMetadata(
    const char* record, const GPSStatus& gpsStatus,
    const OBDSessionStats& obdStatus,
    const CANHardwareDiagnostics& canStatus) {
    if (!logging || !metaLogFile || record == nullptr) {
        return false;
    }

    const RuntimeTimingDiagnostics timing =
        runtimeDiagnostics.getTiming();
    char logBuffer[1280];
    const int written = snprintf(
        logBuffer, sizeof(logBuffer),
        "%s,%s,%lu,%s,%s,%s,%s,%ld,%ld,%d,%d,%lu,"
        "%lu,%lu,%lu,%u,%.1f,%.1f,%.1f,"
        "%lu,%lu,%lu,%lu,%u,"
        "%d,%lu,%lu,%lu,%lu,%lu,%lu,"
        "%02X,%u,%u,%02X,%lu,%lu,%lu,%lu,%lu,%lu,"
        "%lu,%lu,%lu,%lu,%lu,"
        "%lu,%lu,%lu,%lu,%lu,"
        "%lu,%lu,%lu,%lu,%lu,"
        "%lu,%lu,%lu,%lu\n",
        record,
        formatUTC().c_str(),
        static_cast<unsigned long>(millis() - sessionStartTime),
        ROADTEST_FIRMWARE_VERSION,
        ROADTEST_CSV_SCHEMA_VERSION,
        sessionId.c_str(),
        ROADTEST_VEHICLE_PROFILE,
        static_cast<long>(CAN_BAUDRATE),
        static_cast<long>(CAN_CLOCK_16MHZ),
        CAN_OBD_REQUEST_INTERVAL_MS,
        CAN_OBD_RESPONSE_TIMEOUT_MS,
        static_cast<unsigned long>(config.gpsLogInterval),
        static_cast<unsigned long>(GPS_LOCATION_MAX_AGE_MS),
        static_cast<unsigned long>(GPS_SPEED_MAX_AGE_MS),
        static_cast<unsigned long>(GPS_QUALITY_MAX_AGE_MS),
        static_cast<unsigned>(GPS_MIN_SATELLITES),
        static_cast<double>(GPS_MAX_HDOP),
        static_cast<double>(GPS_MIN_RELIABLE_SPEED_KMH),
        static_cast<double>(GPS_DISTANCE_OBD_STATIONARY_KMH),
        static_cast<unsigned long>(sessionCounterDelta(
            gpsStatus.chars_processed,
            gpsSessionStartStatus.chars_processed)),
        static_cast<unsigned long>(sessionCounterDelta(
            gpsStatus.sentences_received,
            gpsSessionStartStatus.sentences_received)),
        static_cast<unsigned long>(sessionCounterDelta(
            gpsStatus.sentences_failed,
            gpsSessionStartStatus.sentences_failed)),
        static_cast<unsigned long>(sessionCounterDelta(
            gpsStatus.rx_buffer_overflows,
            gpsSessionStartStatus.rx_buffer_overflows)),
        gpsStatus.buffered_bytes,
        obdStatus.active ? 1 : 0,
        static_cast<unsigned long>(obdStatus.requestCount),
        static_cast<unsigned long>(obdStatus.responseCount),
        static_cast<unsigned long>(obdStatus.requestErrors),
        static_cast<unsigned long>(obdStatus.timeoutCount),
        static_cast<unsigned long>(obdStatus.unmatchedResponseCount),
        static_cast<unsigned long>(obdStatus.traceDropped),
        canStatus.operatingMode,
        canStatus.transmitErrorCount,
        canStatus.receiveErrorCount,
        canStatus.errorFlags,
        static_cast<unsigned long>(sessionCounterDelta(
            canStatus.receiveBuffer0OverflowCount,
            canSessionStartDiagnostics.receiveBuffer0OverflowCount)),
        static_cast<unsigned long>(sessionCounterDelta(
            canStatus.receiveBuffer1OverflowCount,
            canSessionStartDiagnostics.receiveBuffer1OverflowCount)),
        static_cast<unsigned long>(sessionCounterDelta(
            canStatus.controllerRecoveryCount,
            canSessionStartDiagnostics.controllerRecoveryCount)),
        static_cast<unsigned long>(stats.totalWrites),
        static_cast<unsigned long>(stats.errorCount),
        static_cast<unsigned long>(stats.droppedLogs),
        static_cast<unsigned long>(timing.lastLoopIntervalMs),
        static_cast<unsigned long>(timing.maxLoopIntervalMs),
        static_cast<unsigned long>(timing.totalLoopIntervalMs),
        static_cast<unsigned long>(timing.loopIntervalCount),
        static_cast<unsigned long>(timing.loopStallCount),
        static_cast<unsigned long>(timing.lastWebDurationMs),
        static_cast<unsigned long>(timing.maxWebDurationMs),
        static_cast<unsigned long>(timing.totalWebDurationMs),
        static_cast<unsigned long>(timing.webDurationCount),
        static_cast<unsigned long>(timing.webStallCount),
        static_cast<unsigned long>(timing.lastSDDurationMs),
        static_cast<unsigned long>(timing.maxSDDurationMs),
        static_cast<unsigned long>(timing.totalSDDurationMs),
        static_cast<unsigned long>(timing.sdDurationCount),
        static_cast<unsigned long>(timing.sdStallCount),
        static_cast<unsigned long>(timing.sensorSampleCount),
        static_cast<unsigned long>(timing.sensorMissedSlots),
        static_cast<unsigned long>(timing.gpsSnapshotCount),
        static_cast<unsigned long>(timing.gpsMissedSlots));

    if (written < 0 || written >= static_cast<int>(sizeof(logBuffer))) {
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }

    const size_t length = static_cast<size_t>(written);
    if (writeFileTimed(metaLogFile, logBuffer, length) == length) {
        stats.totalWrites++;
        stats.totalBytes += length;
        lastMetadataLog = millis();
        return true;
    }

    handleCardFailure("Sitzungsmetadaten konnten nicht geschrieben werden", 1);
    return false;
}

bool SDLogger::logGPSData(const GPSData& gps) {
    if (!logging || !config.enableGPSLog) return false;
    
    unsigned long now = millis();
    if (lastGPSLog != 0 &&
        now - lastGPSLog < config.gpsLogInterval) {
        return true;
    }
    
    // Sichere GPS-Daten-Formatierung
    char logBuffer[640];
    String utc = gps.datetime_valid ? formatGPSDateTimeUTC(gps) : formatUTC();
    const bool newFixInSession =
        gps.fix_sequence > 0 &&
        gps.fix_sequence != gpsSessionLastLoggedFixSequence;
    int written = snprintf(logBuffer, sizeof(logBuffer),
        "%s,%lu,%.6f,%.6f,%.2f,%.1f,%.1f,%d,%d,%.2f,"
        "%d,%lu,%d,%lu,%d,%lu,%d,%lu,%d,%lu,%d,%lu,"
        "%d,%lu,%lu,%lu,%lu,%lu,%d,%u\n",
        utc.c_str(), now - sessionStartTime,
        gps.latitude, gps.longitude, gps.altitude,
        gps.speed_kmh, gps.heading_deg, gps.satellites, 
        gps.valid_fix ? 1 : 0, gps.hdop,
        gps.location_valid ? 1 : 0,
        static_cast<unsigned long>(gps.location_age_ms),
        gps.speed_valid ? 1 : 0,
        static_cast<unsigned long>(gps.speed_age_ms),
        gps.altitude_valid ? 1 : 0,
        static_cast<unsigned long>(gps.altitude_age_ms),
        gps.course_valid ? 1 : 0,
        static_cast<unsigned long>(gps.course_age_ms),
        gps.satellites_valid ? 1 : 0,
        static_cast<unsigned long>(gps.satellites_age_ms),
        gps.hdop_valid ? 1 : 0,
        static_cast<unsigned long>(gps.hdop_age_ms),
        newFixInSession ? 1 : 0,
        static_cast<unsigned long>(sessionCounterDelta(
            gps.fix_sequence, gpsSessionStartFixSequence)),
        static_cast<unsigned long>(sessionCounterDelta(
            gps.nmea_chars, gpsSessionStartStatus.chars_processed)),
        static_cast<unsigned long>(sessionCounterDelta(
            gps.nmea_sentences_valid,
            gpsSessionStartStatus.sentences_received)),
        static_cast<unsigned long>(sessionCounterDelta(
            gps.nmea_checksum_failures,
            gpsSessionStartStatus.sentences_failed)),
        static_cast<unsigned long>(sessionCounterDelta(
            gps.rx_buffer_overflows,
            gpsSessionStartStatus.rx_buffer_overflows)),
        gps.rejected ? 1 : 0,
        gps.rejection_reason);
    
    // Überprüfe Truncation
    if (written < 0 || written >= static_cast<int>(sizeof(logBuffer))) {
        Serial.println("⚠️ GPS-Log-Zeile wurde verworfen!");
        stats.errorCount++;
        stats.droppedLogs++;
        return false;
    }
    
    size_t logLength = strlen(logBuffer);
    if (gpsLogFile &&
        writeFileTimed(gpsLogFile, logBuffer, logLength) == logLength) {
        stats.totalWrites++;
        stats.totalBytes += logLength;
        advanceLogSchedule(
            lastGPSLog, now, config.gpsLogInterval);
        gpsSessionLastLoggedFixSequence = gps.fix_sequence;

        // Strecke nur einmal pro neuem Positionsfix bilden. Im Fahrzeug hat
        // eine frische OBD-Geschwindigkeit Vorrang: meldet das Steuergerät
        // Stillstand, darf GPS-Rauschen keine Strecke erzeugen. Ohne OBD wird
        // Bewegung erst ab der zentral definierten, zuverlässigen
        // GPS-Geschwindigkeit akzeptiert.
        if (newFixInSession) {
            const OBDLiveData obd = canReader.getOBDData();
            const bool obdSpeedAvailable = obd.speedValid;
            const bool obdStationary =
                obdSpeedAvailable &&
                obd.speedKmh <= GPS_DISTANCE_OBD_STATIONARY_KMH;
            const bool movementConfirmed =
                obdSpeedAvailable
                    ? !obdStationary
                    : gps.speed_valid;

            if (!gps.valid_fix || !gps.location_valid) {
                // Keine Segmente über eine Qualitätslücke hinweg verbinden.
                hasLastRideGPS = false;
            } else if (!movementConfirmed) {
                // Der aktuelle Standpunkt bleibt der Anker. Langsam wanderndes
                // Positionsrauschen kann sich dadurch nicht aufsummieren.
                lastRideLatitude = gps.latitude;
                lastRideLongitude = gps.longitude;
                lastRideGPSTime = now;
                hasLastRideGPS = true;
            } else if (!hasLastRideGPS) {
                lastRideLatitude = gps.latitude;
                lastRideLongitude = gps.longitude;
                lastRideGPSTime = now;
                hasLastRideGPS = true;
            } else {
                const float segmentMeters = calculateDistance(
                    lastRideLatitude, lastRideLongitude,
                    gps.latitude, gps.longitude);
                const float elapsedSeconds =
                    max((now - lastRideGPSTime) / 1000.0f, 0.2f);
                const float referenceSpeedKmh =
                    obdSpeedAvailable
                        ? static_cast<float>(obd.speedKmh)
                        : gps.speed_kmh;
                const float speedMetersPerSecond =
                    max(
                        referenceSpeedKmh,
                        GPS_MIN_RELIABLE_SPEED_KMH) /
                    3.6f;
                const float maxPlausibleSegment =
                    max(
                        30.0f,
                        speedMetersPerSecond * elapsedSeconds * 3.0f +
                            10.0f);
                const float minPlausibleSegment =
                    obdSpeedAvailable
                        ? GPS_DISTANCE_MIN_SEGMENT_OBD_M
                        : GPS_DISTANCE_MIN_SEGMENT_GPS_M;

                if (segmentMeters >= minPlausibleSegment &&
                    segmentMeters <= maxPlausibleSegment) {
                    rideSummary.distanceKm +=
                        segmentMeters / 1000.0f;
                    lastRideLatitude = gps.latitude;
                    lastRideLongitude = gps.longitude;
                    lastRideGPSTime = now;
                } else if (segmentMeters > maxPlausibleSegment) {
                    // Nach einem Sprung mit dem aktuellen Fix neu ankern,
                    // statt denselben Sprung in jeder Folgemessung zu prüfen.
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
    if (lastRoadLog != 0 &&
        now - lastRoadLog < config.roadLogInterval) {
        return true;
    }
    
    String logLine = formatTimestamp() + "," +
                    String(quality, 1) + "," +
                    String(smoothness, 3) + "," +
                    String(curveFrequency, 1) + "," +
                    String(vibrationRMS, 3) + "\n";
    
    if (roadLogFile &&
        writeFileTimed(
            roadLogFile, logLine.c_str(), logLine.length()) ==
            logLine.length()) {
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        advanceLogSchedule(
            lastRoadLog, now, config.roadLogInterval);
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
        writeFileTimed(
            eventLogFile, logLine.c_str(), logLine.length()) ==
            logLine.length()) {
        flushFileTimed(eventLogFile); // Ereignisse sofort speichern
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
    if (logging && metaLogFile &&
        millis() - lastMetadataLog >= 5000) {
        logSessionMetadata(
            "STATUS", gpsManager.getStatus(),
            canReader.getOBDSessionStats(),
            canReader.getHardwareDiagnostics());
    }

    if (!isReady() || !flushBuffer()) {
        return;
    }

    if (currentLogFile) {
        flushFileTimed(currentLogFile);
    }
    if (roadLogFile) flushFileTimed(roadLogFile);
    if (gpsLogFile) flushFileTimed(gpsLogFile);
    if (canLogFile) flushFileTimed(canLogFile);
    if (obdLogFile) flushFileTimed(obdLogFile);
    if (obdTraceLogFile) flushFileTimed(obdTraceLogFile);
    if (metaLogFile) flushFileTimed(metaLogFile);
    if (eventLogFile) flushFileTimed(eventLogFile);
    if (correlatedLogFile) flushFileTimed(correlatedLogFile);
    
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

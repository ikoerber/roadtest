#include "sd_logger.h"
#include "road_quality.h"
#include "bno055_manager.h"
#include "gps_manager.h"
#include "runtime_diagnostics.h"
#include <esp_system.h>
#include <time.h>

// Forward-Deklaration
float calculateOverallQuality(const RoadMetrics& metrics);

namespace {
constexpr const char* SD_RECOVERY_NVS_NAMESPACE = "rt_sd_rec";
constexpr const char* SENSOR_CSV_HEADER =
    "UTC,UptimeMs,Heading,Pitch,Roll,AccelX,AccelY,AccelZ,"
    "GyroX,GyroY,GyroZ,GravX,GravY,GravZ,YawRateDps,YawRateValid,"
    "Temp,CalSystem,CalGyro,CalAccel,CalMag";

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

// Wie writeFileTimed, führt zusätzlich die erwartete Dateigröße mit. Nur so
// lässt sich am Sitzungsende gegen die tatsächliche Größe auf der Karte
// prüfen.
size_t writeFileTimed(
    File& file, uint32_t& expectedBytes, const char* data, size_t length);

size_t writeFileTimed(
    File& file, uint32_t& expectedBytes, const char* data, size_t length) {
    const size_t written = writeFileTimed(file, data, length);
    expectedBytes += written;
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
      cardAvailable(false), logging(false), stopping(false),
      loggingStartState(LoggingStartState::IDLE), bufferIndex(0),
      lastRoadLog(0), lastFlush(0), lastFlushStep(0), flushCursor(0),
      lastMetadataLog(0), sessionStartTime(0),
      gpsSessionStartStatus{}, gpsSessionStartFixSequence(0),
      gpsSessionLastLoggedFixSequence(0),
      canSessionStartDiagnostics{}, qualitySum(0),
      hasLastRideGPS(false),
      lastRideLatitude(0), lastRideLongitude(0), lastRideGPSTime(0),
      bufferedRecordCount(0), recoveryResumePending(false),
      recoveryMarkerPending(false), recoveryBufferPending(false),
      recoveryStateLoaded(false), recoveryOriginalSessionId(""),
      recoveryFailureReason(""), recoveryFailureUTC(""),
      recoveryFailureTime(0), recoveryBufferedRecordsAtFailure(0),
      recoveryRecoveredRecords(0), recoveryBufferFileName(""),
      recoverySummaryWritten(false), expectedFileBytes{},
      lastRecoveryStatus("") {
    
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
        !openLogFile(correlatedLogFile, correlatedFileName, LOG_TYPE_CORRELATED, LOGFILE_CORRELATED)) {
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
            correlatedLogFile, expectedFileBytes[LOGFILE_CORRELATED],
            logLine.c_str(), logLine.length()) ==
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

    loadPersistentRecoveryState();
    // Die Zusammenfassung der abgebrochenen Sitzung zuerst nachtragen: Sie
    // hängt nur an Kennzahlen im Arbeitsspeicher und soll auch dann
    // entstehen, wenn gar keine Sensorzeilen zu retten waren.
    writeRecoveredRideSummary();
    if (recoveryBufferPending && !recoverBufferedSensorData()) {
        Serial.println(
            "⚠️ Gepufferte Sensordaten konnten noch nicht gesichert werden; "
            "SD-Wiederanlauf wird erneut versucht");
        SD.end();
        initialized = false;
        cardAvailable = false;
        return false;
    }

    // Nur ein im selben Gerätestart erkannter Kartenfehler wird automatisch
    // fortgesetzt. Nach Reset oder Spannungsverlust bleibt der alte
    // Sitzungsmarker erhalten, aber eine neue Messung beginnt aus
    // Sicherheitsgründen erst auf ausdrücklichen Start.
    if (recoveryResumePending) {
        lastRecoveryStatus =
            "SD wieder bereit; Fortsetzung der Messung wird vorbereitet";
        requestLoggingStart();
    }

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
            // Vor dem Öffnen zurückstellen. Stünde die Rückstellung im
            // gemeinsamen Rumpf hinter FINALIZE, würde sie die von
            // openLogFile() gesetzten Ausgangswerte samt Kopfzeilengröße
            // wieder verwerfen; die Größenprüfung verlöre ihre Grundlinie.
            for (uint8_t i = 0; i < LOGFILE_COUNT; ++i) {
                expectedFileBytes[i] = 0;
            }
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
                !openLogFile(roadLogFile, roadFileName, LOG_TYPE_ROAD, LOGFILE_ROAD)) {
                failLoggingStart("Straßen-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_GPS;
            return;

        case LoggingStartState::OPEN_GPS:
            if (config.enableGPSLog &&
                !openLogFile(gpsLogFile, gpsFileName, LOG_TYPE_GPS, LOGFILE_GPS)) {
                failLoggingStart("GPS-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_EVENT;
            return;

        case LoggingStartState::OPEN_EVENT:
            if (config.enableEventLog &&
                !openLogFile(eventLogFile, eventFileName, LOG_TYPE_EVENT, LOGFILE_EVENT)) {
                failLoggingStart("Ereignis-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_META;
            return;

        case LoggingStartState::OPEN_META:
            if (!openLogFile(metaLogFile, metaFileName, LOG_TYPE_META, LOGFILE_META)) {
                failLoggingStart("Metadaten-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_CAN;
            return;

        case LoggingStartState::OPEN_CAN:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(canLogFile, canFileName, LOG_TYPE_CAN, LOGFILE_CAN)) {
                failLoggingStart("CAN-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_OBD;
            return;

        case LoggingStartState::OPEN_OBD:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(obdLogFile, obdFileName, LOG_TYPE_OBD, LOGFILE_OBD)) {
                failLoggingStart("OBD-Log konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_OBD_TRACE;
            return;

        case LoggingStartState::OPEN_OBD_TRACE:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(
                    obdTraceLogFile, obdTraceFileName, LOG_TYPE_OBD_TRACE,
                    LOGFILE_OBD_TRACE)) {
                failLoggingStart("OBD-Trace konnte nicht angelegt werden");
                return;
            }
            loggingStartState = LoggingStartState::OPEN_CORRELATED;
            return;

        case LoggingStartState::OPEN_CORRELATED:
            if (config.enableCANLog && canReader.isReady() &&
                !openLogFile(
                    correlatedLogFile, correlatedFileName,
                    LOG_TYPE_CORRELATED, LOGFILE_CORRELATED)) {
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
    lastRoadLog = 0;
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
    // Ohne Rückstellung überlebt ein vor dem Messstart angefangener
    // Kurvenverlauf den Sitzungsbeginn. In 20260731_152148_452707FB begann
    // das erste Ereignis dadurch 2,2 s vor der Sitzung.
    bnoManager.resetCurveDetection();
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

    if (recoveryResumePending || recoveryMarkerPending) {
        finishRecoveryMarker();
        if (!logging) {
            return;
        }
    } else {
        persistActiveSession();
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
    const bool retryRecoveryStart = recoveryResumePending;
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
    stopping = false;
    loggingStartState = LoggingStartState::IDLE;
    lastStartError = reason;
    if (retryRecoveryStart) {
        // Eine noch instabile Karte kann unmittelbar nach dem ersten
        // erfolgreichen Mount beim Öffnen der Sitzungsdateien erneut
        // ausfallen. Den Wiederanlauf dann nicht still aufgeben, sondern über
        // denselben begrenzten Hardware-Recovery-Zyklus neu versuchen.
        SD.end();
        initialized = false;
        cardAvailable = false;
        lastRecoveryStatus =
            "Fortsetzung konnte noch nicht geöffnet werden; erneuter "
            "SD-Wiederanlauf folgt";
    }
    Serial.printf("❌ SD-Aufzeichnungsstart fehlgeschlagen: %s\n",
                  reason.c_str());
}

void SDLogger::stopLogging() {
    if (isLoggingStartPending()) {
        failLoggingStart("Start abgebrochen");
        return;
    }

    if (!logging) return;
    stopping = true;

    // Eine noch laufende Kurve zuerst abschließen, solange die Sitzungs-
    // dateien offen sind und bevor die Zusammenfassung berechnet wird. Das
    // Ruhefenster der Erkennung läuft nach dem Stoppen nicht mehr ab; ohne
    // diesen Schritt fehlt die letzte Kurve jeder Messfahrt.
    CurveEvent finalCurve;
    if (bnoManager.finishCurveDetection(finalCurve)) {
        const GPSData finalGPS = gpsManager.getCurrentData();
        logCurve(
            finalCurve,
            finalGPS.valid_fix ? finalGPS.latitude : 0.0f,
            finalGPS.valid_fix ? finalGPS.longitude : 0.0f);
    }

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
        stopping = false;
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
    stopping = false;

    // Erst nach dem Schließen liefert ein frisches Öffnen die Größe aus dem
    // Verzeichniseintrag der Karte. Eine Abweichung bedeutet, dass Zeilen
    // trotz erfolgreich gemeldeter Schreibvorgänge nicht angekommen sind.
    const uint8_t unvollstaendig =
        isReady() ? verifyLogFileSizes() : 0;
    if (unvollstaendig > 0) {
        rideSummary.completed = false;
        rideSummary.interrupted = true;
    }

    if (sessionComplete && unvollstaendig == 0) {
        clearPersistentRecoveryState();
    } else if (isReady()) {
        recoveryOriginalSessionId = sessionId;
        // Die beiden Fälle unterscheiden sich grundlegend: Bei fehlenden
        // Abschlussdateien hat die Firmware selbst nicht zu Ende geschrieben,
        // bei einer Größenabweichung sind gemeldete Schreibvorgänge unterhalb
        // der Firmware verloren gegangen.
        recoveryFailureReason =
            unvollstaendig > 0 ? "SESSION_FILES_TRUNCATED"
                               : "SESSION_FINALIZATION_INCOMPLETE";
        recoveryFailureUTC = rideSummary.endUTC;
        recoveryFailureTime = millis();
        // Ausdrücklich kein automatischer Wiederanlauf: Der Nutzer hat die
        // Messung beendet. Nur der Verweis bleibt bestehen.
        recoveryResumePending = false;
        recoveryBufferPending = false;
        recoveryBufferedRecordsAtFailure = 0;
        recoveryRecoveredRecords = 0;
        recoveryBufferFileName = "";
        recoveryMarkerPending = true;
        persistFailureState(recoveryFailureReason);
        if (unvollstaendig == 0) {
            lastRecoveryStatus =
                "Sitzungsabschluss unvollständig; Verweis wird bei der "
                "nächsten Messung protokolliert";
        }
    }

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

bool SDLogger::openLogFile(
    File& file, String& fileName, LogType type, LogFileIndex index) {
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

    // Ausgangswert einschließlich Kopfzeile. Alle weiteren Schreibvorgänge
    // addieren darauf, sodass am Sitzungsende eine erwartete Gesamtgröße
    // vorliegt.
    expectedFileBytes[index] = file.size();
    stats.fileCount++;
    return true;
}

uint8_t SDLogger::verifyLogFileSizes() {
    // Gegenprobe nach dem Schließen: Die Dateien werden frisch geöffnet,
    // sodass die Größe aus dem Verzeichniseintrag der Karte stammt und nicht
    // aus dem Arbeitsspeicher.
    //
    // Anlass ist 20260801_114252_B9D1628B: Dort verbuchte die Firmware 3534
    // erfolgreiche GPS-Schreibvorgänge, die Datei enthielt aber nur 1122
    // Zeilen und endete nach 227 von 709 Sekunden. Sämtliche Fehlerzähler
    // standen auf null. Ein solcher Verlust unterhalb der Firmware bleibt
    // ohne diese Prüfung unsichtbar.
    struct Eintrag {
        const String& name;
        uint32_t erwartet;
        const char* bezeichnung;
    };
    const Eintrag eintraege[] = {
        {currentFileName, expectedFileBytes[LOGFILE_SENSOR], "Sensor"},
        {roadFileName, expectedFileBytes[LOGFILE_ROAD], "Strasse"},
        {gpsFileName, expectedFileBytes[LOGFILE_GPS], "GPS"},
        {canFileName, expectedFileBytes[LOGFILE_CAN], "CAN"},
        {obdFileName, expectedFileBytes[LOGFILE_OBD], "OBD"},
        {obdTraceFileName, expectedFileBytes[LOGFILE_OBD_TRACE], "OBD-Trace"},
        {metaFileName, expectedFileBytes[LOGFILE_META], "Metadaten"},
        {eventFileName, expectedFileBytes[LOGFILE_EVENT], "Ereignis"},
        {correlatedFileName, expectedFileBytes[LOGFILE_CORRELATED],
         "Korrelation"},
    };

    String abweichungen;
    uint8_t betroffen = 0;
    for (const Eintrag& eintrag : eintraege) {
        if (eintrag.name.length() == 0 || eintrag.erwartet == 0) continue;
        File probe = SD.open(eintrag.name, FILE_READ);
        if (!probe) {
            ++betroffen;
            abweichungen += String(eintrag.bezeichnung) + "=nicht lesbar;";
            continue;
        }
        const uint32_t tatsaechlich = probe.size();
        probe.close();
        if (tatsaechlich >= eintrag.erwartet) continue;
        ++betroffen;
        abweichungen += String(eintrag.bezeichnung) + "=" +
                        String(tatsaechlich) + "/" +
                        String(eintrag.erwartet) + ";";
        Serial.printf(
            "❌ %s-Datei unvollständig: %lu von %lu Byte auf der Karte\n",
            eintrag.bezeichnung,
            static_cast<unsigned long>(tatsaechlich),
            static_cast<unsigned long>(eintrag.erwartet));
    }

    if (betroffen == 0) {
        return 0;
    }

    // Der Befund gehört in eine eigene Datei: Die Logdateien sind zu diesem
    // Zeitpunkt geschlossen, und genau ihre Vollständigkeit steht in Frage.
    const String berichtName =
        sessionDirectory + "/" + config.filePrefix + "_integritaet_" +
        sessionId + ".csv";
    File bericht = SD.open(berichtName, FILE_WRITE);
    if (bericht) {
        bericht.println("Session,Datei,ByteAufKarte,ByteErwartet");
        for (const Eintrag& eintrag : eintraege) {
            if (eintrag.name.length() == 0 || eintrag.erwartet == 0) continue;
            File probe = SD.open(eintrag.name, FILE_READ);
            const uint32_t tatsaechlich = probe ? probe.size() : 0;
            if (probe) probe.close();
            if (tatsaechlich >= eintrag.erwartet) continue;
            bericht.printf(
                "%s,%s,%lu,%lu\n", sessionId.c_str(), eintrag.bezeichnung,
                static_cast<unsigned long>(tatsaechlich),
                static_cast<unsigned long>(eintrag.erwartet));
        }
        bericht.flush();
        bericht.close();
    }

    stats.errorCount++;
    lastRecoveryStatus =
        "Sitzung " + sessionId + " unvollständig auf der Karte: " +
        abweichungen;
    Serial.println("❌ " + lastRecoveryStatus);
    return betroffen;
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

    const bool wasLogging = logging;
    if (wasLogging) {
        rideSummary.durationSeconds = (millis() - sessionStartTime) / 1000;
        rideSummary.endUTC = formatUTC();
        rideSummary.active = false;
        rideSummary.completed = false;
        rideSummary.interrupted = true;

        // Ein Fehler während eines ausdrücklich angeforderten Stopps darf
        // nicht überraschend eine neue Messung starten. Die Datenrettung und
        // der persistente Verweis bleiben trotzdem aktiv.
        recoveryResumePending = !stopping;
        recoveryMarkerPending = true;
        recoveryBufferPending =
            bufferIndex > 0 && bufferedRecordCount > 0;
        recoveryOriginalSessionId = sessionId;
        recoveryFailureReason = reason;
        recoveryFailureUTC = rideSummary.endUTC;
        recoveryFailureTime = millis();
        recoveryBufferedRecordsAtFailure = bufferedRecordCount;
        recoveryRecoveredRecords = 0;
        recoveryBufferFileName = "";
        // Sonst behauptete das naechste Fortsetzungsereignis faelschlich, die
        // Zusammenfassung sei bereits nachgetragen.
        recoverySummaryWritten = false;
        persistFailureState(recoveryFailureReason);
    }

    stats.errorCount++;
    stats.droppedLogs += droppedRecords;
    lastRecoveryStatus =
        wasLogging && recoveryResumePending
            ? String("SD-Fehler erkannt; ") +
                  String(bufferedRecordCount) +
                  " gepufferte Sensorzeilen werden gesichert und die "
                  "Messung danach fortgesetzt"
            : (wasLogging
                   ? String("SD-Fehler beim Beenden; ") +
                         String(bufferedRecordCount) +
                         " gepufferte Sensorzeilen werden nach dem "
                         "Wiedereinbinden gesichert"
                   : String(
                         "SD-Fehler erkannt; Karte wird erneut eingebunden"));
    if (canReader.isOBDSessionActive()) {
        canReader.endOBDSession();
    }
    closeLogFiles();
    logging = false;
    SD.end();
    initialized = false;
    cardAvailable = false;

    Serial.printf(
        "⚠️ SD-Fehler: %s; %s\n", reason,
        wasLogging && recoveryResumePending
            ? "automatischer Messungs-Wiederanlauf vorgemerkt"
            : "Karte wird im Hintergrund erneut geprüft");
}

void SDLogger::loadPersistentRecoveryState() {
    if (recoveryStateLoaded || recoveryResumePending ||
        recoveryMarkerPending) {
        return;
    }

    Preferences recoveryPreferences;
    if (!recoveryPreferences.begin(
            SD_RECOVERY_NVS_NAMESPACE, true)) {
        Serial.println(
            "⚠️ Persistenter SD-Sitzungsstatus konnte nicht gelesen werden");
        return;
    }

    const bool active =
        recoveryPreferences.getBool("active", false);
    if (active) {
        recoveryOriginalSessionId =
            recoveryPreferences.getString("sid", "");
        recoveryFailureReason =
            recoveryPreferences.getString("cause", "");
        recoveryFailureUTC =
            recoveryPreferences.getString("failutc", "");
        recoveryBufferedRecordsAtFailure =
            recoveryPreferences.getUInt("bufrows", 0);

        if (recoveryFailureReason.length() == 0) {
            recoveryFailureReason =
                "UNEXPECTED_RESET_OR_POWER_LOSS";
        }
        if (recoveryFailureUTC.length() == 0) {
            recoveryFailureUTC =
                recoveryPreferences.getString("startutc", "");
        }

        recoveryMarkerPending = true;
        lastRecoveryStatus =
            "Eine zuvor nicht sauber abgeschlossene Messung wurde erkannt; "
            "der Verweis wird beim nächsten Start protokolliert";
        Serial.printf(
            "⚠️ Unvollständige frühere Sitzung erkannt: %s (%s)\n",
            recoveryOriginalSessionId.c_str(),
            recoveryFailureReason.c_str());
    }

    recoveryPreferences.end();
    recoveryStateLoaded = true;
}

void SDLogger::persistActiveSession() {
    Preferences recoveryPreferences;
    if (!recoveryPreferences.begin(
            SD_RECOVERY_NVS_NAMESPACE, false)) {
        Serial.println(
            "⚠️ Aktiver SD-Sitzungsstatus konnte nicht gespeichert werden");
        return;
    }

    recoveryPreferences.putBool("active", true);
    recoveryPreferences.putString("sid", sessionId);
    recoveryPreferences.putString("startutc", rideSummary.startUTC);
    recoveryPreferences.putString("cause", "");
    recoveryPreferences.putString("failutc", "");
    recoveryPreferences.putUInt("bufrows", 0);
    recoveryPreferences.end();
}

void SDLogger::persistFailureState(const String& reason) {
    Preferences recoveryPreferences;
    if (!recoveryPreferences.begin(
            SD_RECOVERY_NVS_NAMESPACE, false)) {
        Serial.println(
            "⚠️ SD-Fehlerstatus konnte nicht persistent gespeichert werden");
        return;
    }

    recoveryPreferences.putBool("active", true);
    recoveryPreferences.putString(
        "sid", recoveryOriginalSessionId);
    recoveryPreferences.putString(
        "startutc", rideSummary.startUTC);
    recoveryPreferences.putString("cause", reason);
    recoveryPreferences.putString(
        "failutc", recoveryFailureUTC);
    recoveryPreferences.putUInt(
        "bufrows", recoveryBufferedRecordsAtFailure);
    recoveryPreferences.end();
}

void SDLogger::clearPersistentRecoveryState() {
    Preferences recoveryPreferences;
    if (!recoveryPreferences.begin(
            SD_RECOVERY_NVS_NAMESPACE, false)) {
        Serial.println(
            "⚠️ Abgeschlossener SD-Sitzungsstatus konnte nicht gelöscht werden");
        return;
    }

    recoveryPreferences.clear();
    recoveryPreferences.end();
}

bool SDLogger::recoverBufferedSensorData() {
    if (!recoveryBufferPending || bufferIndex <= 0 ||
        bufferedRecordCount == 0) {
        recoveryBufferPending = false;
        return true;
    }

    String recoveryDirectory =
        "/sessions/" + recoveryOriginalSessionId;
    if (!SD.exists(recoveryDirectory) &&
        !SD.mkdir(recoveryDirectory)) {
        Serial.printf(
            "❌ Recovery-Verzeichnis fehlt: %s\n",
            recoveryDirectory.c_str());
        return false;
    }

    char uniqueSuffix[12];
    snprintf(
        uniqueSuffix, sizeof(uniqueSuffix), "%08lX",
        static_cast<unsigned long>(esp_random()));
    const String recoveryFileName =
        recoveryDirectory + "/" + config.filePrefix +
        "_sensor_recovered_" + uniqueSuffix + ".csv";
    File recoveryFile =
        SD.open(recoveryFileName, FILE_WRITE);
    if (!recoveryFile) {
        Serial.printf(
            "❌ Recovery-Sensordatei konnte nicht angelegt werden: %s\n",
            recoveryFileName.c_str());
        return false;
    }

    const size_t headerBytes =
        recoveryFile.println(SENSOR_CSV_HEADER);
    const size_t sensorBytes =
        writeFileTimed(
            recoveryFile, writeBuffer,
            static_cast<size_t>(bufferIndex));
    flushFileTimed(recoveryFile);
    recoveryFile.close();

    if (headerBytes == 0 ||
        sensorBytes != static_cast<size_t>(bufferIndex)) {
        Serial.println(
            "❌ Gepufferte Sensorzeilen wurden nicht vollständig gesichert");
        return false;
    }

    recoveryRecoveredRecords = bufferedRecordCount;
    recoveryBufferFileName = recoveryFileName;
    stats.totalWrites += bufferedRecordCount;
    stats.totalBytes += sensorBytes;
    stats.fileCount++;
    bufferIndex = 0;
    bufferedRecordCount = 0;
    recoveryBufferPending = false;
    lastRecoveryStatus =
        String(recoveryRecoveredRecords) +
        " gepufferte Sensorzeilen gesichert; Fortsetzung wird vorbereitet";
    Serial.printf(
        "✅ %lu gepufferte Sensorzeilen gesichert: %s\n",
        static_cast<unsigned long>(recoveryRecoveredRecords),
        recoveryFileName.c_str());
    return true;
}

void SDLogger::finishRecoveryMarker() {
    String description =
        "Ursprungssitzung=" + recoveryOriginalSessionId +
        ";Ursache=" + recoveryFailureReason;
    if (recoveryFailureUTC.length() > 0) {
        description += ";FehlerUTC=" + recoveryFailureUTC;
    }
    if (recoveryFailureTime > 0) {
        description +=
            ";AusfallMs=" +
            String(millis() - recoveryFailureTime);
    }
    description +=
        ";PufferBeiFehler=" +
        String(recoveryBufferedRecordsAtFailure);
    description +=
        ";PufferGerettet=" +
        String(recoveryRecoveredRecords);
    if (recoveryBufferFileName.length() > 0) {
        description +=
            ";RecoveryDatei=" + recoveryBufferFileName;
    }
    // Damit sich in der Auswertung ohne Blick ins Verzeichnis erkennen lässt,
    // ob die Ursprungssitzung ihre Kennzahlen zurückbekommen hat.
    description +=
        String(";ZusammenfassungNachgetragen=") +
        (recoverySummaryWritten ? "ja" : "nein");

    if (!logEvent("SD_RECOVERY_CONTINUATION", description)) {
        return;
    }

    lastRecoveryStatus =
        "SD-Wiederanlauf erfolgreich; neue Sitzung " + sessionId +
        " setzt " + recoveryOriginalSessionId + " fort";
    Serial.println("✅ " + lastRecoveryStatus);

    recoveryResumePending = false;
    recoveryMarkerPending = false;
    recoveryBufferPending = false;
    recoveryOriginalSessionId = "";
    recoveryFailureReason = "";
    recoveryFailureUTC = "";
    recoveryFailureTime = 0;
    recoveryBufferedRecordsAtFailure = 0;
    recoveryRecoveredRecords = 0;
    recoveryBufferFileName = "";
    recoverySummaryWritten = false;
    persistActiveSession();
}

bool SDLogger::writeHeader(File& file, LogType type) {
    const char* header = nullptr;
    switch(type) {
        case LOG_TYPE_SENSOR:
            header = SENSOR_CSV_HEADER;
            break;
            
        case LOG_TYPE_EVENT:
            header =
                "UTC,UptimeMs,Ereignis,Beschreibung,Latitude,Longitude,Schwere,"
                "StartUptimeMs,EndUptimeMs,DurationMs,Direction,CurveGroupId,"
                "AngleDeg,HeadingAngleDeg,DistanceM,MeanSpeedKmh,MaxSpeedKmh,"
                "MeanYawRateDps,MaxYawRateDps,RadiusM,MeanLateralAccel,"
                "MaxLateralAccel,Samples,DetectionMode,CompletionReason,"
                "QualityFlags";
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
                "FlushLastMs,FlushMaxMs,FlushTotalMs,FlushCycles,"
                "FlushStalls,"
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
    return writeRideSummaryTo(sessionDirectory, sessionId);
}

bool SDLogger::writeRecoveredRideSummary() {
    // Nach einem Kartenfehler fehlte der abgebrochenen Sitzung bisher ihre
    // Zusammenfassung. Die Kennzahlen stehen zu diesem Zeitpunkt noch
    // vollständig im Arbeitsspeicher, nur die Karte war beim Abbruch nicht
    // mehr beschreibbar. In 20260731_160032_3D732AD1 gingen so Strecke,
    // Kurvenzahl und Durchschnittsqualität einer 2,5-km-Fahrt mit 16 Kurven
    // verloren, obwohl die Rohdateien vollständig vorlagen.
    //
    // Die Bedingung auf rideSummary.sessionId ist wesentlich: Nach einem
    // Reset oder Spannungsverlust wird der Sitzungsmarker zwar aus dem NVS
    // geladen, die Kennzahlen sind dann aber verloren. In diesem Fall darf
    // keine Zusammenfassung mit Nullwerten entstehen.
    if (!cardAvailable || recoveryOriginalSessionId.length() == 0 ||
        rideSummary.sessionId != recoveryOriginalSessionId ||
        !rideSummary.interrupted) {
        return false;
    }

    const String directory = "/sessions/" + recoveryOriginalSessionId;
    if (!SD.exists(directory)) {
        return false;
    }
    const String summaryFileName =
        directory + "/" + config.filePrefix + "_summary_" +
        recoveryOriginalSessionId + ".csv";
    if (SD.exists(summaryFileName)) {
        // Bereits vorhanden, etwa weil der Abbruch erst nach dem regulären
        // Schreiben auftrat. Eine bestehende Datei nicht überschreiben.
        return false;
    }

    if (rideSummary.qualitySamples > 0) {
        rideSummary.averageQuality =
            qualitySum / rideSummary.qualitySamples;
    }
    const bool written =
        writeRideSummaryTo(directory, recoveryOriginalSessionId);
    if (written) {
        recoverySummaryWritten = true;
        Serial.printf(
            "✅ Zusammenfassung der abgebrochenen Sitzung nachgetragen: %s\n",
            summaryFileName.c_str());
    }
    return written;
}

bool SDLogger::writeRideSummaryTo(
    const String& directory, const String& id) {
    String summaryFileName =
        directory + "/" + config.filePrefix + "_summary_" + id + ".csv";
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
    // Ohne einen einzigen gültigen Qualitätswert - etwa in einem reinen
    // Standlauf - bleibt das Mittelwertfeld leer. Eine 0,0 wäre hier keine
    // gemessene Straßenlage, sondern eine fehlende Messung.
    char averageQualityField[12] = "";
    if (rideSummary.qualitySamples > 0) {
        snprintf(
            averageQualityField, sizeof(averageQualityField), "%.1f",
            rideSummary.averageQuality);
    }
    size_t summaryWritten = summaryFile.printf(
        "%s,%s,%s,%lu,%.3f,%lu,%lu,%lu,%s\n",
        rideSummary.sessionId.c_str(),
        rideSummary.startUTC.c_str(),
        rideSummary.endUTC.c_str(),
        static_cast<unsigned long>(rideSummary.durationSeconds),
        rideSummary.distanceKm,
        static_cast<unsigned long>(rideSummary.potholeCount),
        static_cast<unsigned long>(rideSummary.curveCount),
        static_cast<unsigned long>(rideSummary.qualitySamples),
        averageQualityField);
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
                handleCardFailure(
                    "Sensorpuffer konnte nicht vollständig geschrieben werden");
                return false;
            }
            stats.totalWrites += bufferedRecordCount;
            stats.totalBytes += bytesWritten;
            expectedFileBytes[LOGFILE_SENSOR] += bytesWritten;
        } else {
            // Niemals einen abgeschnittenen CSV-Puffer schreiben. Die Sitzung
            // wird sichtbar abgebrochen, damit keine scheinbar gültige,
            // tatsächlich beschädigte letzte Zeile entsteht.
            Serial.printf(
                "❌ KRITISCH: Buffer-Grenze verletzt! Index: %d, Max: %d\n",
                bufferIndex, BUFFER_SIZE);
            stats.bufferOverflows++;
            const uint32_t invalidRecords = bufferedRecordCount;
            bufferIndex = 0;
            bufferedRecordCount = 0;
            handleCardFailure(
                "Sensorpuffer-Grenze verletzt", invalidRecords);
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
        const uint32_t invalidRecords = bufferedRecordCount;
        bufferIndex = 0;
        bufferedRecordCount = 0;
        handleCardFailure(
            "Sensorpuffer-Index außerhalb der Grenzen",
            invalidRecords);
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

    // Sichere Formatierung mit begrenzter Puffergröße
    char logBuffer[256]; // Maximale Zeilenlänge begrenzt
    int written = snprintf(logBuffer, sizeof(logBuffer),
        "%s,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,%d,%.1f,%d,%d,%d,%d\n",
        formatTimestamp().c_str(),
        data.heading, data.pitch, data.roll,
        data.accelX, data.accelY, data.accelZ,
        data.gyroX, data.gyroY, data.gyroZ,
        data.gravX, data.gravY, data.gravZ,
        data.yawRateDps, data.yawRateValid ? 1 : 0,
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
            eventLogFile, expectedFileBytes[LOGFILE_EVENT],
            logLine.c_str(), logLine.length()) ==
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
        !openLogFile(canLogFile, canFileName, LOG_TYPE_CAN, LOGFILE_CAN)) {
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
        writeFileTimed(canLogFile, expectedFileBytes[LOGFILE_CAN], logBuffer, logLength) == logLength) {
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
        !openLogFile(obdLogFile, obdFileName, LOG_TYPE_OBD, LOGFILE_OBD)) {
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
        writeFileTimed(obdLogFile, expectedFileBytes[LOGFILE_OBD], logBuffer, logLength) == logLength) {
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
            obdTraceLogFile, obdTraceFileName, LOG_TYPE_OBD_TRACE,
            LOGFILE_OBD_TRACE)) {
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
    if (writeFileTimed(obdTraceLogFile, expectedFileBytes[LOGFILE_OBD_TRACE], logBuffer, length) == length) {
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
        static_cast<unsigned long>(timing.lastFlushCycleMs),
        static_cast<unsigned long>(timing.maxFlushCycleMs),
        static_cast<unsigned long>(timing.totalFlushCycleMs),
        static_cast<unsigned long>(timing.flushCycleCount),
        static_cast<unsigned long>(timing.flushCycleStallCount),
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
    if (writeFileTimed(metaLogFile, expectedFileBytes[LOGFILE_META], logBuffer, length) == length) {
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

    // Sichere GPS-Daten-Formatierung
    const unsigned long now = millis();
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
        writeFileTimed(gpsLogFile, expectedFileBytes[LOGFILE_GPS], logBuffer, logLength) == logLength) {
        stats.totalWrites++;
        stats.totalBytes += logLength;
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
            roadLogFile, expectedFileBytes[LOGFILE_ROAD],
            logLine.c_str(), logLine.length()) ==
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
                       float lat, float lon, float severity) {
    if (!logging || !config.enableEventLog) return false;
    
    String logLine = formatTimestamp() + "," +
                    eventType + "," +
                    description + "," +
                    String(lat, 6) + "," +
                    String(lon, 6) + "," +
                    String(severity, 2) +
                    ",,,,,,,,,,,,,,,,,,,\n";
    
    if (eventLogFile &&
        writeFileTimed(
            eventLogFile, expectedFileBytes[LOGFILE_EVENT],
            logLine.c_str(), logLine.length()) ==
            logLine.length()) {
        flushFileTimed(eventLogFile);
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
        lat, lon, severity);
    if (logged) {
        rideSummary.potholeCount++;
    }
    return logged;
}

bool SDLogger::logCurve(
    const CurveEvent& event, float lat, float lon) {
    if (!logging || !config.enableEventLog || !event.valid) {
        return false;
    }

    const char* direction =
        event.direction > 0 ? "POSITIV"
                            : (event.direction < 0 ? "NEGATIV" : "UNBEKANNT");
    const char* detectionMode =
        event.detectionMode == CurveDetectionMode::SHARP ? "SHARP" : "LONG";
    const char* completionReason = "QUIET";
    if (event.completionReason == CurveCompletionReason::REVERSAL) {
        completionReason = "REVERSAL";
    } else if (event.completionReason == CurveCompletionReason::TIMEOUT) {
        completionReason = "TIMEOUT";
    } else if (event.completionReason == CurveCompletionReason::SESSION_END) {
        completionReason = "SESSION_END";
    }

    String description =
        String("Winkel=") + String(event.angleDeg, 1) +
        ";Radius=" + String(event.radiusM, 1) +
        ";Gruppe=" + String(event.groupId);
    String logLine = formatTimestamp() + ",KURVE," +
                     description + "," +
                     String(lat, 6) + "," +
                     String(lon, 6) + "," +
                     String(event.angleDeg, 2) + "," +
                     String(event.startTimeMs) + "," +
                     String(event.endTimeMs) + "," +
                     String(event.durationMs) + "," +
                     direction + "," +
                     String(event.groupId) + "," +
                     String(event.angleDeg, 2) + "," +
                     String(event.headingAngleDeg, 2) + "," +
                     String(event.distanceM, 2) + "," +
                     String(event.meanSpeedKmh, 2) + "," +
                     String(event.maxSpeedKmh, 2) + "," +
                     String(event.meanYawRateDps, 3) + "," +
                     String(event.maxYawRateDps, 3) + "," +
                     String(event.radiusM, 2) + "," +
                     String(event.meanLateralAccel, 3) + "," +
                     String(event.maxLateralAccel, 3) + "," +
                     String(event.sampleCount) + "," +
                     detectionMode + "," +
                     completionReason + "," +
                     String(event.qualityFlags) + "\n";

    bool logged =
        eventLogFile &&
        writeFileTimed(
            eventLogFile, expectedFileBytes[LOGFILE_EVENT],
            logLine.c_str(), logLine.length()) ==
            logLine.length();
    if (logged) {
        // Ein Ereignis wird sofort gesichert.
        flushFileTimed(eventLogFile);
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
        rideSummary.curveCount++;
    } else {
        handleCardFailure(
            "Kurvenereignis konnte nicht geschrieben werden", 1);
    }
    return logged;
}

// Fahrbarkeit eines Streckenabschnitts.
//
// Nutzt die Spalten der Ereignisdatei, die dieselbe Bedeutung haben wie beim
// Kurvenereignis: Start, Ende, Dauer, Weg, mittlere und höchste
// Geschwindigkeit sowie die Stichprobenzahl. Die Note steht in `Schwere`,
// damit die Auswertung sie ohne Sonderfall findet. Kurvenbezogene Spalten
// bleiben leer beziehungsweise null.
bool SDLogger::logRoadSection(
    const RoadSection& section, float lat, float lon) {
    if (!logging || !config.enableEventLog || section.samples == 0) {
        return false;
    }

    // Die Abschnittsnummer steht auch in der Beschreibung, damit sich ein
    // Beifahrerurteil eindeutig auf seinen Abschnitt beziehen lässt.
    String description =
        String("Abschnitt=") + String(section.number) +
        ";Note=" + String(section.driveability, 0) +
        ";RMS=" + String(section.rmsVertical, 3) +
        ";Stoesse=" + String(section.shockCount);
    String logLine = formatTimestamp() + ",ABSCHNITT," +
                     description + "," +
                     String(lat, 6) + "," +
                     String(lon, 6) + "," +
                     String(section.driveability, 2) + "," +
                     String(section.startMs) + "," +
                     String(section.endMs) + "," +
                     String(section.endMs - section.startMs) + "," +
                     "UNBEKANNT,0,0.00,0.00," +
                     String(section.distanceM, 2) + "," +
                     String(section.meanSpeedKmh, 2) + "," +
                     String(section.maxSpeedKmh, 2) + "," +
                     "0.000,0.000,0.00," +
                     String(section.rmsVertical, 3) + "," +
                     String(section.maxShock, 3) + "," +
                     String(section.samples) + "," +
                     "SECTION,LENGTH," +
                     String(section.shockCount) + "\n";

    bool logged =
        eventLogFile &&
        writeFileTimed(
            eventLogFile, expectedFileBytes[LOGFILE_EVENT],
            logLine.c_str(), logLine.length()) ==
            logLine.length();
    if (logged) {
        flushFileTimed(eventLogFile);
        stats.totalWrites++;
        stats.totalBytes += logLine.length();
    } else {
        handleCardFailure(
            "Streckenabschnitt konnte nicht geschrieben werden", 1);
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
    // Der gesamte Durchlauf wird zusätzlich zu den Einzelmessungen je Datei
    // als ein Vorgang erfasst. Nur damit lässt sich eine lange
    // Hauptschleifenpause einer Ursache zuordnen.
    const unsigned long flushCycleStartedAt = millis();

    if (logging && metaLogFile &&
        millis() - lastMetadataLog >= 5000) {
        logSessionMetadata(
            "STATUS", gpsManager.getStatus(),
            canReader.getOBDSessionStats(),
            canReader.getHardwareDiagnostics());
    }

    if (!isReady() || !flushBuffer()) {
        runtimeDiagnostics.recordFlushCycle(
            millis() - flushCycleStartedAt);
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
    flushCursor = 0;
    runtimeDiagnostics.recordFlushCycle(
        lastFlush - flushCycleStartedAt);
}

void SDLogger::flushStep() {
    // Ein vollständiger Flush aller neun Dateien blockierte die Hauptschleife
    // gemessen bis zu 235 ms am Stück und verschluckte dabei zwei bis drei
    // Sensorstichproben. Die Lücken lagen deterministisch bei 2290 und
    // 2590 ms modulo 5000, also genau im Takt dieses Durchlaufs.
    //
    // Der Durchlauf wird deshalb in Einzelschritte zerlegt: je Aufruf
    // höchstens eine Datei oder die Statuszeile. Ein voller Umlauf dauert
    // damit SD_FLUSH_STEP_INTERVAL_MS mal Schrittzahl und bleibt in derselben
    // Größenordnung wie zuvor, die einzelne Blockade aber sinkt auf die
    // Dauer eines Schritts.
    if (!logging) return;
    const unsigned long now = millis();
    if (now - lastFlushStep < SD_FLUSH_STEP_INTERVAL_MS) return;
    lastFlushStep = now;

    const unsigned long stepStartedAt = now;
    switch (flushCursor) {
        case 0:
            // Ohne zusätzliches Zeitgatter: Schritt 0 kehrt konstruktiv alle
            // SD_FLUSH_STEP_INTERVAL_MS mal SD_FLUSH_STEP_COUNT wieder. Die
            // frühere Kombination aus Fünf-Sekunden-Takt und einem separaten
            // 5000-ms-Gatter blockierte sich bei Jitter gegenseitig; in den
            // Messdaten vom 31.07.2026 erschienen Statuszeilen deshalb mal
            // nach fünf, mal erst nach zehn Sekunden.
            if (metaLogFile) {
                logSessionMetadata(
                    "STATUS", gpsManager.getStatus(),
                    canReader.getOBDSessionStats(),
                    canReader.getHardwareDiagnostics());
            }
            break;
        case 1:
            // Der Sensorpuffer zuerst: Er ist die einzige Stelle, an der
            // Messzeilen bei einem Kartenfehler verloren gehen könnten.
            if (isReady() && flushBuffer() && currentLogFile) {
                flushFileTimed(currentLogFile);
            }
            break;
        case 2: if (roadLogFile) flushFileTimed(roadLogFile); break;
        case 3: if (gpsLogFile) flushFileTimed(gpsLogFile); break;
        case 4: if (canLogFile) flushFileTimed(canLogFile); break;
        case 5: if (obdLogFile) flushFileTimed(obdLogFile); break;
        case 6: if (obdTraceLogFile) flushFileTimed(obdTraceLogFile); break;
        case 7: if (metaLogFile) flushFileTimed(metaLogFile); break;
        case 8: if (eventLogFile) flushFileTimed(eventLogFile); break;
        case 9: if (correlatedLogFile) flushFileTimed(correlatedLogFile); break;
        default: break;
    }

    if (++flushCursor >= SD_FLUSH_STEP_COUNT) {
        flushCursor = 0;
        lastFlush = millis();
    }
    // Erfasst wird jetzt der einzelne Schritt, nicht mehr der volle Umlauf.
    // Genau diese Zahl beschreibt die Blockade der Hauptschleife.
    runtimeDiagnostics.recordFlushCycle(millis() - stepStartedAt);
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

#include "vehicle_data_discovery.h"

#include <cstring>

#include "hardware_config.h"
#include "sd_logger.h"

VehicleDataDiscovery vehicleDataDiscovery;

VehicleDataDiscovery::VehicleDataDiscovery()
    : phase(VehicleDiscoveryPhase::IDLE),
      ecuState(ECUReachabilityState::UNKNOWN),
      startedAt(0),
      phaseStartedAt(0),
      lastRequestAt(0),
      scanFinishedAt(0),
      recoveryNextRoundAt(0),
      ecuFirstReachableAt(0),
      ecuLastReachableAt(0),
      ignitionOnMarkedAt(0),
      ignitionRestartMarkedAt(0),
      engineStartMarkedAt(0),
      engineStopMarkedAt(0),
      engineRestartMarkedAt(0),
      detectionAfterIgnitionOnMs(UINT32_MAX),
      detectionAfterIgnitionRestartMs(UINT32_MAX),
      detectionAfterEngineStartMs(UINT32_MAX),
      detectionAfterEngineRestartMs(UINT32_MAX),
      initialCANMessages(0),
      passiveFrames(0),
      passiveExtendedFrames(0),
      passiveUniqueStandardIds(0),
      scanSupportResponseBaseline(0),
      recoveryResponseBaseline(0),
      liveFailureBaseline(0),
      liveResponseBaseline(0),
      recoveryCount(0),
      ecuLossCount(0),
      ecuLossCountAtEngineStop(0),
      scanRequestIndex(0),
      liveRequestIndex(0),
      recoveryRequestIndex(0),
      recoveryRound(0),
      previousOBDPollingEnabled(false),
      loggingStartedByDiscovery(false),
      seenStandardIds{} {
}

const char* VehicleDataDiscovery::getPhaseName() const {
    switch (phase) {
        case VehicleDiscoveryPhase::IDLE:
            return "inaktiv";
        case VehicleDiscoveryPhase::PREPARING_LOG:
            return "SD-Aufzeichnung vorbereiten";
        case VehicleDiscoveryPhase::PASSIVE_CAPTURE:
            return "passiver Listen-Only-Mitschnitt";
        case VehicleDiscoveryPhase::PID_SCAN:
            return "Standard-PID-Erkennung";
        case VehicleDiscoveryPhase::ECU_RECOVERY:
            return "ECU-Wiedererkennung";
        case VehicleDiscoveryPhase::LIVE_SAMPLING:
            return "Messwerte sammeln";
    }
    return "unbekannt";
}

const char* VehicleDataDiscovery::getECUStateName() const {
    switch (ecuState) {
        case ECUReachabilityState::UNKNOWN:
            return "unbekannt";
        case ECUReachabilityState::SEARCHING:
            return "wird gesucht";
        case ECUReachabilityState::REACHABLE:
            return "erreichbar";
        case ECUReachabilityState::LOST:
            return "Verbindung verloren";
    }
    return "unbekannt";
}

bool VehicleDataDiscovery::begin() {
    if (isActive()) {
        Serial.println("ℹ️ Datenerkennung läuft bereits");
        printStatus();
        return true;
    }
    if (!canReader.isReady()) {
        Serial.println("❌ Datenerkennung benötigt einen bereiten CAN-Adapter");
        return false;
    }
    if (!sdLogger.isReady()) {
        Serial.println("❌ Datenerkennung benötigt eine bereite SD-Karte");
        return false;
    }
    if (sdLogger.isLogging() || sdLogger.isLoggingStartPending()) {
        Serial.println(
            "❌ Eine andere SD-Aufzeichnung läuft bereits; zuerst 'stop' verwenden");
        return false;
    }

    phase = VehicleDiscoveryPhase::PREPARING_LOG;
    startedAt = millis();
    phaseStartedAt = startedAt;
    initialCANMessages = canReader.getTotalMessages();
    passiveFrames = 0;
    passiveExtendedFrames = 0;
    passiveUniqueStandardIds = 0;
    scanRequestIndex = 0;
    liveRequestIndex = 0;
    recoveryRequestIndex = 0;
    recoveryRound = 0;
    scanFinishedAt = 0;
    recoveryNextRoundAt = 0;
    scanSupportResponseBaseline = 0;
    recoveryResponseBaseline = 0;
    liveFailureBaseline = 0;
    liveResponseBaseline = 0;
    recoveryCount = 0;
    ecuLossCount = 0;
    ecuLossCountAtEngineStop = 0;
    ecuState = ECUReachabilityState::UNKNOWN;
    ecuFirstReachableAt = 0;
    ecuLastReachableAt = 0;
    ignitionOnMarkedAt = 0;
    ignitionRestartMarkedAt = 0;
    engineStartMarkedAt = 0;
    engineStopMarkedAt = 0;
    engineRestartMarkedAt = 0;
    detectionAfterIgnitionOnMs = UINT32_MAX;
    detectionAfterIgnitionRestartMs = UINT32_MAX;
    detectionAfterEngineStartMs = UINT32_MAX;
    detectionAfterEngineRestartMs = UINT32_MAX;
    memset(seenStandardIds, 0, sizeof(seenStandardIds));

    previousOBDPollingEnabled = canReader.isOBDPollingEnabled();
    canReader.setOBDPollingEnabled(false);
    canReader.resetOBDDiscoveryData();
    loggingStartedByDiscovery = false;

    if (!sdLogger.requestLoggingStart()) {
        phase = VehicleDiscoveryPhase::IDLE;
        canReader.setOBDPollingEnabled(previousOBDPollingEnabled);
        return false;
    }

    Serial.println("\n✅ Fahrzeugdaten-Erkennung wird gestartet");
    Serial.println(
        "Phase 1 beginnt nach dem Öffnen der SD-Dateien und sendet 60 s lang nichts");
    Serial.println(
        "Danach folgen ausschließlich standardisierte OBD-Service-01-Abfragen");
    return true;
}

void VehicleDataDiscovery::startPassiveCapture() {
    // Ab diesem Zeitpunkt gehört die vom Erkennungsmodus angelegte
    // Aufzeichnung der Session und muss auch bei einem CAN-Umschaltfehler
    // wieder sauber geschlossen werden.
    loggingStartedByDiscovery = true;
    if (!canReader.configurePassiveCapture()) {
        Serial.println("❌ Passive CAN-Phase konnte nicht gestartet werden");
        end();
        return;
    }

    phase = VehicleDiscoveryPhase::PASSIVE_CAPTURE;
    phaseStartedAt = millis();
    sdLogger.logEvent(
        "DISCOVERY_PHASE",
        "PREPARATION_COMPLETE;DURATION_MS_" +
            String(phaseStartedAt - startedAt));
    sdLogger.logEvent(
        "DISCOVERY_PHASE",
        "PASSIVE_START;60s;MCP2515_LISTEN_ONLY;NO_REQUESTS");

    Serial.println("\n=== DATENERKENNUNG: PHASE 1/3 ===");
    Serial.println("60 s passiver CAN-Mitschnitt im echten Listen-Only-Modus");
    Serial.println("Es werden weder OBD-Anfragen noch CAN-ACKs gesendet");
}

void VehicleDataDiscovery::startPIDScan() {
    if (!canReader.configureOBDResponseMode()) {
        Serial.println("❌ OBD-Erkennungsphase konnte nicht gestartet werden");
        end();
        return;
    }

    canReader.setOBDPollingEnabled(true);
    phase = VehicleDiscoveryPhase::PID_SCAN;
    phaseStartedAt = millis();
    lastRequestAt = phaseStartedAt - CAN_OBD_REQUEST_INTERVAL_MS;
    scanRequestIndex = 0;
    scanFinishedAt = 0;
    scanSupportResponseBaseline =
        canReader.getOBDSessionStats().supportResponseCount;
    sdLogger.logEvent(
        "DISCOVERY_PHASE",
        "PID_SCAN_START;SERVICE_01;BLOCKS_00_20_40_60;TWO_ROUNDS");

    Serial.println("\n=== DATENERKENNUNG: PHASE 2/3 ===");
    Serial.printf(
        "Standard-PID-Blöcke 00/20/40/60, maximal alle %d ms\n",
        CAN_OBD_REQUEST_INTERVAL_MS);
}

void VehicleDataDiscovery::setECUReachable(
    unsigned long now, const char* reason) {
    const bool newlyReachable =
        ecuState != ECUReachabilityState::REACHABLE;
    ecuState = ECUReachabilityState::REACHABLE;
    ecuLastReachableAt = now;
    if (ecuFirstReachableAt == 0) {
        ecuFirstReachableAt = now;
    }

    if (newlyReachable && sdLogger.isLogging()) {
        sdLogger.logEvent(
            "ECU_STATE",
            String("REACHABLE;REASON_") + reason +
                ";RECOVERY_COUNT_" + String(recoveryCount));
    }
}

void VehicleDataDiscovery::updateAcceptanceEngineState(
    unsigned long now) {
    const OBDSessionStats session = canReader.getOBDSessionStats();

    unsigned long ignitionMarker = 0;
    unsigned long* ignitionDetection = nullptr;
    const char* ignitionEvent = nullptr;
    if (ignitionRestartMarkedAt > 0) {
        ignitionMarker = ignitionRestartMarkedAt;
        ignitionDetection = &detectionAfterIgnitionRestartMs;
        ignitionEvent = "ECU_NACH_ZUENDUNG_NEU_ERKANNT";
    } else if (ignitionOnMarkedAt > 0) {
        ignitionMarker = ignitionOnMarkedAt;
        ignitionDetection = &detectionAfterIgnitionOnMs;
        ignitionEvent = "ECU_NACH_ZUENDUNG_ERKANNT";
    }

    if (ignitionDetection != nullptr &&
        *ignitionDetection == UINT32_MAX &&
        session.lastResponseMs > ignitionMarker) {
        *ignitionDetection =
            session.lastResponseMs - ignitionMarker;
        if (sdLogger.isLogging()) {
            sdLogger.logEvent(
                "IGNITION_STATE",
                String(ignitionEvent) + ";DETECTION_MS_" +
                    String(*ignitionDetection));
        }
    }

    const OBDLiveData obd = canReader.getOBDData();
    if (!obd.rpmValid || obd.rpm < 300.0f) {
        return;
    }

    // Der Browser-Marker verbessert die Bediennachvollziehbarkeit, darf aber
    // nicht die reale OBD-Erkennung blockieren. Im Kontrolltest vom
    // 30.07.2026 ging genau der zweite POST verloren, obwohl anschließend
    // 99 gültige Drehzahlwerte vorlagen. Eine frische Drehzahl nach dem
    // Zündungsmarker ist der stärkere technische Nachweis.
    if (engineRestartMarkedAt == 0 &&
        ignitionRestartMarkedAt > 0 &&
        obd.rpmUpdatedMs > ignitionRestartMarkedAt &&
        obd.rpmUpdatedMs <= now) {
        if (!addMarker("ABNAHME_MOTOR_NEU_AUTOMATISCH_ERKANNT")) {
            return;
        }
        engineRestartMarkedAt = ignitionRestartMarkedAt;
        detectionAfterEngineRestartMs =
            obd.rpmUpdatedMs - ignitionRestartMarkedAt;
        if (sdLogger.isLogging()) {
            sdLogger.logEvent(
                "ENGINE_STATE",
                String("MOTOR_NEU_LAEUFT;RPM_") +
                    String(obd.rpm, 0) + ";DETECTION_MS_" +
                    String(detectionAfterEngineRestartMs) +
                    ";SOURCE_AUTO_RPM");
        }
        return;
    }
    if (engineStartMarkedAt == 0 &&
        ignitionOnMarkedAt > 0 &&
        engineStopMarkedAt == 0 &&
        obd.rpmUpdatedMs > ignitionOnMarkedAt &&
        obd.rpmUpdatedMs <= now) {
        if (!addMarker("ABNAHME_MOTOR_AUTOMATISCH_ERKANNT")) {
            return;
        }
        engineStartMarkedAt = ignitionOnMarkedAt;
        detectionAfterEngineStartMs =
            obd.rpmUpdatedMs - ignitionOnMarkedAt;
        if (sdLogger.isLogging()) {
            sdLogger.logEvent(
                "ENGINE_STATE",
                String("MOTOR_LAEUFT;RPM_") +
                    String(obd.rpm, 0) + ";DETECTION_MS_" +
                    String(detectionAfterEngineStartMs) +
                    ";SOURCE_AUTO_RPM");
        }
        return;
    }

    unsigned long engineMarker = 0;
    unsigned long* engineDetection = nullptr;
    const char* engineEvent = nullptr;
    if (engineRestartMarkedAt > 0) {
        engineMarker = engineRestartMarkedAt;
        engineDetection = &detectionAfterEngineRestartMs;
        engineEvent = "MOTOR_NEU_LAEUFT";
    } else if (engineStartMarkedAt > 0) {
        engineMarker = engineStartMarkedAt;
        engineDetection = &detectionAfterEngineStartMs;
        engineEvent = "MOTOR_LAEUFT";
    }

    if (engineDetection != nullptr &&
        *engineDetection == UINT32_MAX &&
        obd.rpmUpdatedMs > engineMarker &&
        obd.rpmUpdatedMs <= now) {
        *engineDetection = obd.rpmUpdatedMs - engineMarker;
        if (sdLogger.isLogging()) {
            sdLogger.logEvent(
                "ENGINE_STATE",
                String(engineEvent) + ";RPM_" +
                    String(obd.rpm, 0) + ";DETECTION_MS_" +
                    String(*engineDetection));
        }
    }
}

void VehicleDataDiscovery::enterECURecovery(const char* reason) {
    const unsigned long now = millis();
    if (!canReader.configureOBDResponseMode()) {
        Serial.println("❌ ECU-Wiedererkennung konnte nicht gestartet werden");
        end();
        return;
    }

    canReader.setOBDPollingEnabled(true);
    const bool wasReachable =
        ecuState == ECUReachabilityState::REACHABLE;
    if (wasReachable) {
        ecuLossCount++;
        ecuState = ECUReachabilityState::LOST;
    } else if (ecuState != ECUReachabilityState::LOST) {
        ecuState = ECUReachabilityState::SEARCHING;
    }

    phase = VehicleDiscoveryPhase::ECU_RECOVERY;
    phaseStartedAt = now;
    lastRequestAt = now - CAN_OBD_REQUEST_INTERVAL_MS;
    recoveryNextRoundAt = now;
    recoveryRequestIndex = 0;
    recoveryRound = 0;
    recoveryResponseBaseline =
        canReader.getOBDSessionStats().responseCount;
    recoveryCount++;

    if (sdLogger.isLogging()) {
        sdLogger.logEvent(
            "ECU_STATE",
            String(wasReachable ? "LOST" : "SEARCHING") +
                ";REASON_" + reason +
                ";RECOVERY_COUNT_" + String(recoveryCount));
    }
    Serial.printf(
        "⚠️ ECU-Wiedererkennung #%u: %s\n",
        recoveryCount, reason);
}

void VehicleDataDiscovery::printPIDSupport(
    uint8_t pid, const char* name, bool logToSD) {
    String result;
    if (!canReader.isOBDPidSupportKnown(pid)) {
        result = "unbekannt;Unterstuetzungsblock ohne Antwort";
    } else if (canReader.isOBDPidSupported(pid)) {
        result = "unterstuetzt";
    } else {
        result = "nicht_unterstuetzt";
    }

    Serial.printf("PID %02X %-20s: %s\n", pid, name, result.c_str());
    if (logToSD) {
        sdLogger.logEvent(
            "PID_SUPPORT",
            "PID_" + String(pid, HEX) + ";" + String(name) + ";" + result);
    }
}

void VehicleDataDiscovery::printSupportSummary(bool logToSD) {
    const OBDLiveData obd = canReader.getOBDData();
    Serial.println("\n--- Erkannte Standard-PID-Unterstützung ---");
    for (uint8_t block = 0; block < 4; block++) {
        Serial.printf(
            "Block %02X: %s",
            block * 0x20,
            obd.supportBlockValid[block] ? "Antwort" : "keine Antwort");
        if (obd.supportBlockValid[block]) {
            Serial.printf(
                ", Bitmap 0x%08lX",
                static_cast<unsigned long>(obd.supportBitmap[block]));
        }
        Serial.println();

        if (logToSD) {
            String description =
                "BLOCK_" + String(block * 0x20, HEX) + ";" +
                (obd.supportBlockValid[block] ? "VALID" : "NO_RESPONSE") +
                ";BITMAP_" + String(obd.supportBitmap[block], HEX);
            sdLogger.logEvent("PID_BITMAP", description);
        }
    }

    printPIDSupport(0x0C, "Motordrehzahl", logToSD);
    printPIDSupport(0x0D, "Geschwindigkeit", logToSD);
    printPIDSupport(0x10, "Luftmassenstrom", logToSD);
    printPIDSupport(0x11, "Drosselstellung", logToSD);
    printPIDSupport(0x46, "Aussentemperatur", logToSD);
    printPIDSupport(0x5C, "Oeltemperatur", logToSD);
    printPIDSupport(0x5E, "Kraftstoffrate", logToSD);
}

void VehicleDataDiscovery::startLiveSampling() {
    setECUReachable(millis(), "PID_SCAN_RESPONSE");
    phase = VehicleDiscoveryPhase::LIVE_SAMPLING;
    phaseStartedAt = millis();
    lastRequestAt = phaseStartedAt - CAN_OBD_REQUEST_INTERVAL_MS;
    liveRequestIndex = 0;
    const OBDSessionStats session = canReader.getOBDSessionStats();
    liveFailureBaseline =
        session.requestErrors + session.timeoutCount;
    liveResponseBaseline = session.responseCount;

    printSupportSummary(true);
    sdLogger.logEvent(
        "DISCOVERY_PHASE",
        "LIVE_SAMPLING_START;ONLY_CONFIRMED_STANDARD_PIDS");

    Serial.println("\n=== DATENERKENNUNG: PHASE 3/3 ===");
    Serial.println("Bestätigte Standardwerte werden bis 'discover end' gesammelt");
    Serial.println("Markierungen: discover mark <kurze Beschreibung>");
}

void VehicleDataDiscovery::update() {
    if (!isActive()) {
        return;
    }

    const unsigned long now = millis();
    updateAcceptanceEngineState(now);

    if (phase == VehicleDiscoveryPhase::PREPARING_LOG) {
        if (sdLogger.isLogging()) {
            startPassiveCapture();
        } else if (!sdLogger.isLoggingStartPending()) {
            Serial.printf(
                "❌ SD-Aufzeichnung nicht gestartet: %s\n",
                sdLogger.getLastStartError().c_str());
            restoreNormalOperation();
            phase = VehicleDiscoveryPhase::IDLE;
        }
        return;
    }

    if (!canReader.isReady() || !sdLogger.isReady() ||
        !sdLogger.isLogging()) {
        Serial.println(
            "❌ Datenerkennung wegen CAN- oder SD-Ausfall abgebrochen");
        end();
        return;
    }

    if (phase == VehicleDiscoveryPhase::PASSIVE_CAPTURE) {
        if (now - phaseStartedAt >= PASSIVE_DURATION_MS) {
            sdLogger.logEvent(
                "DISCOVERY_PHASE",
                "PASSIVE_END;FRAMES_" + String(passiveFrames) +
                    ";UNIQUE_11BIT_IDS_" +
                    String(passiveUniqueStandardIds) +
                    ";EXTENDED_FRAMES_" + String(passiveExtendedFrames));
            startPIDScan();
        }
        return;
    }

    if (phase == VehicleDiscoveryPhase::PID_SCAN) {
        static const uint8_t scanPids[] = {
            0x00, 0x20, 0x40, 0x60,
            0x00, 0x20, 0x40, 0x60
        };
        static_assert(
            sizeof(scanPids) == 4 * SCAN_ROUNDS,
            "PID-Scan-Runden und Anfrageliste muessen zusammenpassen");

        if (scanRequestIndex < sizeof(scanPids) &&
            now - lastRequestAt >= CAN_OBD_REQUEST_INTERVAL_MS) {
            canReader.requestOBDPid(scanPids[scanRequestIndex]);
            scanRequestIndex++;
            lastRequestAt = now;
            if (scanRequestIndex == sizeof(scanPids)) {
                scanFinishedAt = now;
            }
        } else if (scanFinishedAt > 0 &&
                   now - scanFinishedAt >= SCAN_RESPONSE_WAIT_MS) {
            const OBDSessionStats session =
                canReader.getOBDSessionStats();
            if (session.supportResponseCount >
                scanSupportResponseBaseline) {
                startLiveSampling();
            } else {
                enterECURecovery("PID_SCAN_NO_RESPONSE");
            }
        }
        return;
    }

    if (phase == VehicleDiscoveryPhase::ECU_RECOVERY) {
        const OBDSessionStats session =
            canReader.getOBDSessionStats();
        if (session.responseCount > recoveryResponseBaseline &&
            session.lastResponseMs >= phaseStartedAt) {
            setECUReachable(now, "SAFE_FALLBACK_RESPONSE");
            startPIDScan();
            return;
        }

        static const uint8_t recoveryPids[] = {
            0x0C, 0x0D, 0x11, 0x00
        };
        static_assert(
            sizeof(recoveryPids) == RECOVERY_PROBES_PER_ROUND,
            "Recovery-Proben und Rundengroesse muessen zusammenpassen");

        if (now >= recoveryNextRoundAt &&
            now - lastRequestAt >= CAN_OBD_REQUEST_INTERVAL_MS) {
            canReader.requestOBDPid(
                recoveryPids[recoveryRequestIndex]);
            recoveryRequestIndex++;
            lastRequestAt = now;

            if (recoveryRequestIndex >= sizeof(recoveryPids)) {
                recoveryRequestIndex = 0;
                recoveryRound++;
                const unsigned long backoff =
                    min(
                        RECOVERY_BACKOFF_BASE_MS *
                            static_cast<unsigned long>(recoveryRound),
                        RECOVERY_BACKOFF_MAX_MS);
                recoveryNextRoundAt = now + backoff;
            }
        }
        return;
    }

    if (phase == VehicleDiscoveryPhase::LIVE_SAMPLING) {
        OBDSessionStats session =
            canReader.getOBDSessionStats();
        const uint32_t failures =
            session.requestErrors + session.timeoutCount;
        if (session.responseCount > liveResponseBaseline) {
            liveResponseBaseline = session.responseCount;
            liveFailureBaseline = failures;
        }
        if (failures - liveFailureBaseline >=
                LIVE_FAILURES_BEFORE_LOST &&
            session.lastRequestMs > session.lastResponseMs) {
            enterECURecovery("CONSECUTIVE_REQUEST_FAILURES");
            return;
        }

        if (now - lastRequestAt < CAN_OBD_REQUEST_INTERVAL_MS) {
            return;
        }

        // Geschwindigkeit erscheint mehrfach, damit der Vergleich mit GPS
        // trotz der bewusst niedrigen Gesamtrate zeitlich brauchbar bleibt.
        static const uint8_t livePids[] = {
            0x0D, 0x0C, 0x0D, 0x10, 0x0D, 0x11,
            0x0D, 0x5E, 0x0D, 0x5C, 0x0D, 0x46
        };

        for (uint8_t attempt = 0; attempt < sizeof(livePids); attempt++) {
            const uint8_t pid = livePids[liveRequestIndex];
            liveRequestIndex =
                (liveRequestIndex + 1) % sizeof(livePids);
            if (canReader.isOBDPidSupported(pid)) {
                canReader.requestOBDPid(pid);
                break;
            }
        }
        lastRequestAt = now;
    }
}

bool VehicleDataDiscovery::wasStandardIdSeen(uint16_t id) const {
    return (seenStandardIds[id / 8] & (1U << (id % 8))) != 0;
}

void VehicleDataDiscovery::rememberStandardId(uint16_t id) {
    if (wasStandardIdSeen(id)) {
        return;
    }
    seenStandardIds[id / 8] |= 1U << (id % 8);
    passiveUniqueStandardIds++;
}

void VehicleDataDiscovery::onCANMessage(const CANMessage& message) {
    if (phase != VehicleDiscoveryPhase::PASSIVE_CAPTURE) {
        return;
    }

    passiveFrames++;
    if (message.extended) {
        passiveExtendedFrames++;
    } else if (message.canId >= 0 && message.canId <= 0x7FF) {
        rememberStandardId(static_cast<uint16_t>(message.canId));
    }
}

bool VehicleDataDiscovery::addMarker(String description) {
    if (!isActive() || !sdLogger.isLogging()) {
        Serial.println("ℹ️ Keine aktive Datenerkennung für eine Markierung");
        return false;
    }
    description.trim();
    description.replace(",", ";");
    description.replace("\n", " ");
    description.replace("\r", " ");
    if (description.length() == 0) {
        Serial.println("ℹ️ Markierung benötigt eine Beschreibung");
        return false;
    }

    const bool written = sdLogger.logEvent("DISCOVERY_MARK", description);
    Serial.printf(
        "%s Markierung: %s\n",
        written ? "✅" : "❌", description.c_str());
    return written;
}

bool VehicleDataDiscovery::markIgnitionOn() {
    if (!isActive() || !sdLogger.isLogging()) {
        return false;
    }

    const unsigned long now = millis();
    const bool restart =
        engineStopMarkedAt > 0 &&
        engineStopMarkedAt >= engineStartMarkedAt;
    if (restart) {
        if (ignitionRestartMarkedAt > 0) {
            return true;
        }
        if (!addMarker("ABNAHME_ZUENDUNG_NEU_AN")) {
            return false;
        }
        ignitionRestartMarkedAt = now;
        detectionAfterIgnitionRestartMs = UINT32_MAX;
        return true;
    }

    if (ignitionOnMarkedAt > 0) {
        return true;
    }
    if (!addMarker("ABNAHME_ZUENDUNG_AN")) {
        return false;
    }
    ignitionOnMarkedAt = now;
    detectionAfterIgnitionOnMs = UINT32_MAX;
    return true;
}

bool VehicleDataDiscovery::markEngineStarted() {
    if (!isActive() || !sdLogger.isLogging()) {
        return false;
    }

    const unsigned long now = millis();
    const bool restart =
        engineStopMarkedAt > 0 &&
        engineStopMarkedAt >= engineStartMarkedAt;
    if (restart) {
        if (ignitionRestartMarkedAt == 0) {
            Serial.println(
                "ℹ️ Vor dem Motorneustart zuerst Zündung-Ein markieren");
            return false;
        }
        if (engineRestartMarkedAt > 0) {
            return true;
        }
        if (!addMarker("ABNAHME_MOTOR_NEU_GESTARTET")) {
            return false;
        }
        engineRestartMarkedAt = now;
        detectionAfterEngineRestartMs = UINT32_MAX;
        return true;
    }

    if (ignitionOnMarkedAt == 0) {
        Serial.println(
            "ℹ️ Vor dem Motorstart zuerst Zündung-Ein markieren");
        return false;
    }
    if (engineStartMarkedAt > 0) {
        return true;
    }
    if (!addMarker("ABNAHME_MOTOR_GESTARTET")) {
        return false;
    }
    engineStartMarkedAt = now;
    detectionAfterEngineStartMs = UINT32_MAX;
    return true;
}

bool VehicleDataDiscovery::markEngineStopped() {
    if (!isActive() || !sdLogger.isLogging()) {
        return false;
    }

    if (engineStopMarkedAt > 0) {
        return true;
    }
    const OBDLiveData obd = canReader.getOBDData();
    if (engineStartMarkedAt == 0 ||
        detectionAfterEngineStartMs == UINT32_MAX ||
        millis() -
                (engineStartMarkedAt + detectionAfterEngineStartMs) <
            ACCEPTANCE_INITIAL_STAND_MS ||
        !obd.speedValid ||
        obd.speedKmh > GPS_DISTANCE_OBD_STATIONARY_KMH) {
        Serial.println(
            "ℹ️ Motor-Aus erst nach stabiler Laufphase bei OBD-Stillstand markieren");
        return false;
    }
    const unsigned long now = millis();
    if (!addMarker("ABNAHME_MOTOR_AUS")) {
        return false;
    }
    engineStopMarkedAt = now;
    ecuLossCountAtEngineStop = ecuLossCount;
    return true;
}

void VehicleDataDiscovery::printStatus() {
    if (!isActive()) {
        Serial.println("Fahrzeugdaten-Erkennung: inaktiv");
        return;
    }

    const unsigned long now = millis();
    const OBDLiveData obd = canReader.getOBDData();
    const OBDSessionStats session = canReader.getOBDSessionStats();
    Serial.println("\n=== FAHRZEUGDATEN-ERKENNUNG ===");
    Serial.printf(
        "Phase: %s, Gesamtdauer: %lu s\n",
        getPhaseName(), (now - startedAt) / 1000);
    Serial.printf(
        "ECU: %s, Wiedererkennungen: %u, Verbindungsverluste: %u\n",
        getECUStateName(), recoveryCount, ecuLossCount);
    if (phase == VehicleDiscoveryPhase::PASSIVE_CAPTURE) {
        const unsigned long elapsed = now - phaseStartedAt;
        const unsigned long remaining =
            elapsed >= PASSIVE_DURATION_MS
                ? 0
                : (PASSIVE_DURATION_MS - elapsed + 999) / 1000;
        Serial.printf(
            "Passiv: %lu Frames, %u eindeutige 11-Bit-IDs, "
            "%lu Extended-Frames, noch %lu s\n",
            passiveFrames, passiveUniqueStandardIds,
            passiveExtendedFrames, remaining);
    }
    Serial.printf(
        "OBD-Sitzung: %lu Anfragen, %lu Antworten, %lu "
        "PID-Blockantworten, %lu Sendefehler, %lu Timeouts, "
        "%lu nicht zugeordnet\n",
        static_cast<unsigned long>(session.requestCount),
        static_cast<unsigned long>(session.responseCount),
        static_cast<unsigned long>(session.supportResponseCount),
        static_cast<unsigned long>(session.requestErrors),
        static_cast<unsigned long>(session.timeoutCount),
        static_cast<unsigned long>(session.unmatchedResponseCount));
    Serial.printf(
        "OBD seit Boot: %lu Anfragen, %lu Antworten, %lu Sendefehler\n",
        obd.requestCount, obd.responseCount, obd.requestErrors);
    if (session.lastResponseMs > 0) {
        Serial.printf(
            "Letzte Transaktion: Anfrage #%lu PID %02X, "
            "Antwort %03lX/PID %02X nach %lu ms\n",
            static_cast<unsigned long>(session.lastRequestSequence),
            session.lastRequestPid,
            static_cast<unsigned long>(session.lastResponseCanId),
            session.lastResponsePid,
            static_cast<unsigned long>(session.lastResponseLatencyMs));
    }
    const bool hasLiveValue =
        obd.speedValid || obd.rpmValid || obd.throttleValid ||
        obd.mafValid || obd.fuelRateValid ||
        obd.oilTemperatureValid || obd.ambientTemperatureValid;
    if (hasLiveValue) {
        Serial.print("Messwerte:");
        if (obd.speedValid) {
            Serial.printf(" OBD-Speed=%u km/h", obd.speedKmh);
        }
        if (obd.rpmValid) {
            Serial.printf(", RPM=%.0f", obd.rpm);
        }
        if (obd.throttleValid) {
            Serial.printf(", Drossel=%.1f %%", obd.throttlePercent);
        }
        if (obd.mafValid) {
            Serial.printf(", MAF=%.2f g/s", obd.mafGramsPerSecond);
        }
        if (obd.fuelRateValid) {
            Serial.printf(
                ", Kraftstoffrate=%.2f l/h",
                obd.fuelRateLitersPerHour);
        }
        if (obd.oilTemperatureValid) {
            Serial.printf(", Oel=%.1f C", obd.oilTemperatureC);
        }
        if (obd.ambientTemperatureValid) {
            Serial.printf(
                ", Aussen=%.1f C",
                obd.ambientTemperatureC);
        }
        Serial.println();
    }
    Serial.printf(
        "SD-Sitzung: %s, CAN gesamt in Sitzung: %lu\n",
        sdLogger.getSessionId().c_str(),
        canReader.getTotalMessages() - initialCANMessages);
}

void VehicleDataDiscovery::restoreNormalOperation() {
    if (canReader.isReady()) {
        const bool restored = canReader.configureOBDResponseMode();
        canReader.setOBDPollingEnabled(
            restored && previousOBDPollingEnabled);
        if (!restored) {
            Serial.println(
                "⚠️ OBD-Abfragen bleiben nach CAN-Umschaltfehler pausiert");
        }
    }
}

void VehicleDataDiscovery::end() {
    if (!isActive()) {
        Serial.println("ℹ️ Fahrzeugdaten-Erkennung ist nicht aktiv");
        return;
    }

    const String finishedSessionId = sdLogger.getSessionId();
    const OBDSessionStats finalOBDSession =
        canReader.getOBDSessionStats();
    if (sdLogger.isLogging()) {
        printSupportSummary(true);
        if (ignitionOnMarkedAt > 0 || engineStartMarkedAt > 0) {
            const String ignitionFirst =
                detectionAfterIgnitionOnMs == UINT32_MAX
                    ? "NA"
                    : String(detectionAfterIgnitionOnMs);
            const String engineFirst =
                detectionAfterEngineStartMs == UINT32_MAX
                    ? "NA"
                    : String(detectionAfterEngineStartMs);
            const String ignitionAgain =
                detectionAfterIgnitionRestartMs == UINT32_MAX
                    ? "NA"
                    : String(detectionAfterIgnitionRestartMs);
            const String engineAgain =
                detectionAfterEngineRestartMs == UINT32_MAX
                    ? "NA"
                    : String(detectionAfterEngineRestartMs);
            sdLogger.logEvent(
                "ACCEPTANCE_RESULT",
                "IGNITION_FIRST_MS_" + ignitionFirst +
                    ";ENGINE_FIRST_MS_" + engineFirst +
                    ";IGNITION_RESTART_MS_" + ignitionAgain +
                    ";ENGINE_RESTART_MS_" + engineAgain);
        }
        sdLogger.logEvent(
            "DISCOVERY_PHASE",
            "DISCOVERY_END;DURATION_S_" +
                String((millis() - startedAt) / 1000) +
                ";PASSIVE_FRAMES_" + String(passiveFrames) +
                ";PASSIVE_UNIQUE_11BIT_IDS_" +
                String(passiveUniqueStandardIds) +
                ";OBD_REQUESTS_" + String(finalOBDSession.requestCount) +
                ";OBD_RESPONSES_" + String(finalOBDSession.responseCount) +
                ";OBD_TIMEOUTS_" + String(finalOBDSession.timeoutCount));
        sdLogger.flush();
    }

    restoreNormalOperation();
    if (sdLogger.isLoggingStartPending()) {
        sdLogger.stopLogging();
    } else if (loggingStartedByDiscovery && sdLogger.isLogging()) {
        sdLogger.stopLogging();
    }

    phase = VehicleDiscoveryPhase::IDLE;
    ecuState = ECUReachabilityState::UNKNOWN;
    loggingStartedByDiscovery = false;
    Serial.println("\n✅ Fahrzeugdaten-Erkennung beendet");
    Serial.printf("SD-Sitzung: %s\n", finishedSessionId.c_str());
    Serial.println(
        "Alle vorhandenen sensor-, gps-, can-, obd- und event-Dateien "
        "dieser Sitzung gehören zusammen");
}

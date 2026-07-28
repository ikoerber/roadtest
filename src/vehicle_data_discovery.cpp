#include "vehicle_data_discovery.h"

#include <cstring>

#include "hardware_config.h"
#include "sd_logger.h"

VehicleDataDiscovery vehicleDataDiscovery;

VehicleDataDiscovery::VehicleDataDiscovery()
    : phase(VehicleDiscoveryPhase::IDLE),
      startedAt(0),
      phaseStartedAt(0),
      lastRequestAt(0),
      scanFinishedAt(0),
      initialCANMessages(0),
      passiveFrames(0),
      passiveExtendedFrames(0),
      passiveUniqueStandardIds(0),
      scanRequestIndex(0),
      liveRequestIndex(0),
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
        case VehicleDiscoveryPhase::LIVE_SAMPLING:
            return "Messwerte sammeln";
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
    scanFinishedAt = 0;
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
    sdLogger.logEvent(
        "DISCOVERY_PHASE",
        "PID_SCAN_START;SERVICE_01;BLOCKS_00_20_40_60;TWO_ROUNDS");

    Serial.println("\n=== DATENERKENNUNG: PHASE 2/3 ===");
    Serial.printf(
        "Standard-PID-Blöcke 00/20/40/60, maximal alle %d ms\n",
        CAN_OBD_REQUEST_INTERVAL_MS);
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
    phase = VehicleDiscoveryPhase::LIVE_SAMPLING;
    phaseStartedAt = millis();
    lastRequestAt = phaseStartedAt - CAN_OBD_REQUEST_INTERVAL_MS;
    liveRequestIndex = 0;

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
            startLiveSampling();
        }
        return;
    }

    if (phase == VehicleDiscoveryPhase::LIVE_SAMPLING &&
        now - lastRequestAt >= CAN_OBD_REQUEST_INTERVAL_MS) {
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

void VehicleDataDiscovery::printStatus() {
    if (!isActive()) {
        Serial.println("Fahrzeugdaten-Erkennung: inaktiv");
        return;
    }

    const unsigned long now = millis();
    const OBDLiveData obd = canReader.getOBDData();
    Serial.println("\n=== FAHRZEUGDATEN-ERKENNUNG ===");
    Serial.printf(
        "Phase: %s, Gesamtdauer: %lu s\n",
        getPhaseName(), (now - startedAt) / 1000);
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
        "OBD: %lu Anfragen, %lu Antworten, %lu PID-Blockantworten, "
        "%lu Sendefehler\n",
        obd.requestCount, obd.responseCount, obd.supportResponseCount,
        obd.requestErrors);
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
    if (sdLogger.isLogging()) {
        printSupportSummary(true);
        sdLogger.logEvent(
            "DISCOVERY_PHASE",
            "DISCOVERY_END;DURATION_S_" +
                String((millis() - startedAt) / 1000) +
                ";PASSIVE_FRAMES_" + String(passiveFrames) +
                ";PASSIVE_UNIQUE_11BIT_IDS_" +
                String(passiveUniqueStandardIds));
        sdLogger.flush();
    }

    restoreNormalOperation();
    if (sdLogger.isLoggingStartPending()) {
        sdLogger.stopLogging();
    } else if (loggingStartedByDiscovery && sdLogger.isLogging()) {
        sdLogger.stopLogging();
    }

    phase = VehicleDiscoveryPhase::IDLE;
    loggingStartedByDiscovery = false;
    Serial.println("\n✅ Fahrzeugdaten-Erkennung beendet");
    Serial.printf("SD-Sitzung: %s\n", finishedSessionId.c_str());
    Serial.println(
        "Alle vorhandenen sensor-, gps-, can-, obd- und event-Dateien "
        "dieser Sitzung gehören zusammen");
}

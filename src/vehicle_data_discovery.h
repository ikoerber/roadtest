#ifndef VEHICLE_DATA_DISCOVERY_H
#define VEHICLE_DATA_DISCOVERY_H

#include <Arduino.h>

#include "can_reader.h"

enum class VehicleDiscoveryPhase : uint8_t {
    IDLE,
    PREPARING_LOG,
    PASSIVE_CAPTURE,
    PID_SCAN,
    ECU_RECOVERY,
    LIVE_SAMPLING
};

enum class ECUReachabilityState : uint8_t {
    UNKNOWN,
    SEARCHING,
    REACHABLE,
    LOST
};

class VehicleDataDiscovery {
private:
    static constexpr unsigned long PASSIVE_DURATION_MS = 60000;
    static constexpr unsigned long SCAN_RESPONSE_WAIT_MS = 1000;
    static constexpr unsigned long RECOVERY_BACKOFF_BASE_MS = 5000;
    static constexpr unsigned long RECOVERY_BACKOFF_MAX_MS = 5000;
    static constexpr uint8_t SCAN_ROUNDS = 2;
    static constexpr uint8_t RECOVERY_PROBES_PER_ROUND = 4;
    static constexpr uint8_t LIVE_FAILURES_BEFORE_LOST = 3;

    VehicleDiscoveryPhase phase;
    ECUReachabilityState ecuState;
    unsigned long startedAt;
    unsigned long phaseStartedAt;
    unsigned long lastRequestAt;
    unsigned long scanFinishedAt;
    unsigned long recoveryNextRoundAt;
    unsigned long ecuFirstReachableAt;
    unsigned long ecuLastReachableAt;
    unsigned long ignitionOnMarkedAt;
    unsigned long ignitionRestartMarkedAt;
    unsigned long engineStartMarkedAt;
    unsigned long engineStopMarkedAt;
    unsigned long engineRestartMarkedAt;
    unsigned long detectionAfterIgnitionOnMs;
    unsigned long detectionAfterIgnitionRestartMs;
    unsigned long detectionAfterEngineStartMs;
    unsigned long detectionAfterEngineRestartMs;
    unsigned long initialCANMessages;
    unsigned long passiveFrames;
    unsigned long passiveExtendedFrames;
    uint16_t passiveUniqueStandardIds;
    uint32_t scanSupportResponseBaseline;
    uint32_t recoveryResponseBaseline;
    uint32_t liveFailureBaseline;
    uint32_t liveResponseBaseline;
    uint16_t recoveryCount;
    uint16_t ecuLossCount;
    uint16_t ecuLossCountAtEngineStop;
    uint8_t scanRequestIndex;
    uint8_t liveRequestIndex;
    uint8_t recoveryRequestIndex;
    uint8_t recoveryRound;
    bool previousOBDPollingEnabled;
    bool loggingStartedByDiscovery;
    uint8_t seenStandardIds[256];

    void startPassiveCapture();
    void startPIDScan();
    void startLiveSampling();
    void enterECURecovery(const char* reason);
    void setECUReachable(unsigned long now, const char* reason);
    void updateAcceptanceEngineState(unsigned long now);
    void restoreNormalOperation();
    void printPIDSupport(uint8_t pid, const char* name, bool logToSD);
    void printSupportSummary(bool logToSD);
    bool wasStandardIdSeen(uint16_t id) const;
    void rememberStandardId(uint16_t id);

public:
    VehicleDataDiscovery();

    bool begin();
    void update();
    void end();
    void printStatus();
    bool addMarker(String description);
    bool markIgnitionOn();
    bool markEngineStarted();
    bool markEngineStopped();
    void onCANMessage(const CANMessage& message);

    bool isActive() const {
        return phase != VehicleDiscoveryPhase::IDLE;
    }
    bool controlsOBDPolling() const {
        return isActive();
    }
    bool isPassiveCaptureActive() const {
        return phase == VehicleDiscoveryPhase::PASSIVE_CAPTURE;
    }
    bool isECURecoveryActive() const {
        return phase == VehicleDiscoveryPhase::ECU_RECOVERY;
    }
    uint32_t getElapsedSeconds() const {
        return isActive() ? (millis() - startedAt) / 1000 : 0;
    }
    uint32_t getPhaseElapsedSeconds() const {
        return isActive() ? (millis() - phaseStartedAt) / 1000 : 0;
    }
    uint32_t getPassiveRemainingSeconds() const {
        if (phase != VehicleDiscoveryPhase::PASSIVE_CAPTURE) {
            return 0;
        }
        const unsigned long elapsed = millis() - phaseStartedAt;
        return elapsed >= PASSIVE_DURATION_MS
            ? 0
            : (PASSIVE_DURATION_MS - elapsed + 999) / 1000;
    }
    unsigned long getPassiveFrameCount() const {
        return passiveFrames;
    }
    ECUReachabilityState getECUState() const { return ecuState; }
    const char* getECUStateName() const;
    uint16_t getRecoveryCount() const { return recoveryCount; }
    uint16_t getECULossCount() const { return ecuLossCount; }
    bool hasObservedECULossAfterEngineStop() const {
        return engineStopMarkedAt > 0 &&
               ecuLossCount > ecuLossCountAtEngineStop;
    }
    unsigned long getEngineStartMarkedAt() const {
        return engineStartMarkedAt;
    }
    unsigned long getIgnitionOnMarkedAt() const {
        return ignitionOnMarkedAt;
    }
    unsigned long getIgnitionRestartMarkedAt() const {
        return ignitionRestartMarkedAt;
    }
    unsigned long getEngineStopMarkedAt() const {
        return engineStopMarkedAt;
    }
    unsigned long getEngineRestartMarkedAt() const {
        return engineRestartMarkedAt;
    }
    unsigned long getDetectionAfterEngineStartMs() const {
        return detectionAfterEngineStartMs;
    }
    unsigned long getDetectionAfterIgnitionOnMs() const {
        return detectionAfterIgnitionOnMs;
    }
    unsigned long getDetectionAfterIgnitionRestartMs() const {
        return detectionAfterIgnitionRestartMs;
    }
    unsigned long getDetectionAfterEngineRestartMs() const {
        return detectionAfterEngineRestartMs;
    }
    const char* getPhaseName() const;
};

extern VehicleDataDiscovery vehicleDataDiscovery;

#endif

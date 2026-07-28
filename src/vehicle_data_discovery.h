#ifndef VEHICLE_DATA_DISCOVERY_H
#define VEHICLE_DATA_DISCOVERY_H

#include <Arduino.h>

#include "can_reader.h"

enum class VehicleDiscoveryPhase : uint8_t {
    IDLE,
    PREPARING_LOG,
    PASSIVE_CAPTURE,
    PID_SCAN,
    LIVE_SAMPLING
};

class VehicleDataDiscovery {
private:
    static constexpr unsigned long PASSIVE_DURATION_MS = 60000;
    static constexpr unsigned long SCAN_RESPONSE_WAIT_MS = 1000;
    static constexpr uint8_t SCAN_ROUNDS = 2;

    VehicleDiscoveryPhase phase;
    unsigned long startedAt;
    unsigned long phaseStartedAt;
    unsigned long lastRequestAt;
    unsigned long scanFinishedAt;
    unsigned long initialCANMessages;
    unsigned long passiveFrames;
    unsigned long passiveExtendedFrames;
    uint16_t passiveUniqueStandardIds;
    uint8_t scanRequestIndex;
    uint8_t liveRequestIndex;
    bool previousOBDPollingEnabled;
    bool loggingStartedByDiscovery;
    uint8_t seenStandardIds[256];

    void startPassiveCapture();
    void startPIDScan();
    void startLiveSampling();
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
    const char* getPhaseName() const;
};

extern VehicleDataDiscovery vehicleDataDiscovery;

#endif

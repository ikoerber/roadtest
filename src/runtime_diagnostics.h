#ifndef RUNTIME_DIAGNOSTICS_H
#define RUNTIME_DIAGNOSTICS_H

#include <Arduino.h>

struct RuntimeTimingDiagnostics {
    uint32_t lastLoopIntervalMs = 0;
    uint32_t maxLoopIntervalMs = 0;
    uint32_t loopStallCount = 0;
    uint32_t lastWebDurationMs = 0;
    uint32_t maxWebDurationMs = 0;
    uint32_t webStallCount = 0;
    uint32_t lastSDDurationMs = 0;
    uint32_t maxSDDurationMs = 0;
    uint32_t sdStallCount = 0;
};

class RuntimeDiagnostics {
private:
    RuntimeTimingDiagnostics timing;

    void record(
        uint32_t durationMs, uint32_t& lastMs, uint32_t& maxMs,
        uint32_t& stallCount);

public:
    void resetSession();
    void recordLoopInterval(uint32_t durationMs);
    void recordWebDuration(uint32_t durationMs);
    void recordSDDuration(uint32_t durationMs);
    RuntimeTimingDiagnostics getTiming() const { return timing; }
};

extern RuntimeDiagnostics runtimeDiagnostics;

#endif

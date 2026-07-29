#include "runtime_diagnostics.h"

#include "hardware_config.h"

RuntimeDiagnostics runtimeDiagnostics;

void RuntimeDiagnostics::record(
    uint32_t durationMs, uint32_t& lastMs, uint32_t& maxMs,
    uint32_t& stallCount) {
    lastMs = durationMs;
    if (durationMs > maxMs) {
        maxMs = durationMs;
    }
    if (durationMs >= RUNTIME_STALL_THRESHOLD_MS) {
        stallCount++;
    }
}

void RuntimeDiagnostics::resetSession() {
    timing = RuntimeTimingDiagnostics{};
}

void RuntimeDiagnostics::recordLoopInterval(uint32_t durationMs) {
    record(
        durationMs, timing.lastLoopIntervalMs,
        timing.maxLoopIntervalMs, timing.loopStallCount);
}

void RuntimeDiagnostics::recordWebDuration(uint32_t durationMs) {
    record(
        durationMs, timing.lastWebDurationMs,
        timing.maxWebDurationMs, timing.webStallCount);
}

void RuntimeDiagnostics::recordSDDuration(uint32_t durationMs) {
    record(
        durationMs, timing.lastSDDurationMs,
        timing.maxSDDurationMs, timing.sdStallCount);
}

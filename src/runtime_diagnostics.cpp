#include "runtime_diagnostics.h"

#include "hardware_config.h"

RuntimeDiagnostics runtimeDiagnostics;

void RuntimeDiagnostics::record(
    uint32_t durationMs, uint32_t& lastMs, uint32_t& maxMs,
    uint32_t& totalMs, uint32_t& sampleCount,
    uint32_t& stallCount) {
    lastMs = durationMs;
    if (durationMs > maxMs) {
        maxMs = durationMs;
    }
    totalMs += durationMs;
    sampleCount++;
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
        timing.maxLoopIntervalMs, timing.totalLoopIntervalMs,
        timing.loopIntervalCount, timing.loopStallCount);
}

void RuntimeDiagnostics::recordWebDuration(uint32_t durationMs) {
    record(
        durationMs, timing.lastWebDurationMs,
        timing.maxWebDurationMs, timing.totalWebDurationMs,
        timing.webDurationCount, timing.webStallCount);
}

void RuntimeDiagnostics::recordSDDuration(uint32_t durationMs) {
    record(
        durationMs, timing.lastSDDurationMs,
        timing.maxSDDurationMs, timing.totalSDDurationMs,
        timing.sdDurationCount, timing.sdStallCount);
}

void RuntimeDiagnostics::recordFlushCycle(uint32_t durationMs) {
    record(
        durationMs, timing.lastFlushCycleMs,
        timing.maxFlushCycleMs, timing.totalFlushCycleMs,
        timing.flushCycleCount, timing.flushCycleStallCount);
}

void RuntimeDiagnostics::recordSensorSchedule(
    uint32_t elapsedMs, uint32_t intervalMs) {
    if (intervalMs == 0 || elapsedMs < intervalMs) {
        return;
    }
    timing.sensorSampleCount++;
    timing.sensorMissedSlots += elapsedMs / intervalMs - 1;
}

void RuntimeDiagnostics::recordGPSSchedule(
    uint32_t elapsedMs, uint32_t intervalMs) {
    if (intervalMs == 0 || elapsedMs < intervalMs) {
        return;
    }
    timing.gpsSnapshotCount++;
    timing.gpsMissedSlots += elapsedMs / intervalMs - 1;
}

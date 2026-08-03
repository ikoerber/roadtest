#ifndef RUNTIME_DIAGNOSTICS_H
#define RUNTIME_DIAGNOSTICS_H

#include <Arduino.h>

struct RuntimeTimingDiagnostics {
    uint32_t lastLoopIntervalMs = 0;
    uint32_t maxLoopIntervalMs = 0;
    uint32_t totalLoopIntervalMs = 0;
    uint32_t loopIntervalCount = 0;
    uint32_t loopStallCount = 0;
    uint32_t lastWebDurationMs = 0;
    uint32_t maxWebDurationMs = 0;
    uint32_t totalWebDurationMs = 0;
    uint32_t webDurationCount = 0;
    uint32_t webStallCount = 0;
    uint32_t lastSDDurationMs = 0;
    uint32_t maxSDDurationMs = 0;
    uint32_t totalSDDurationMs = 0;
    uint32_t sdDurationCount = 0;
    uint32_t sdStallCount = 0;
    // Der Sammelflush über alle offenen Dateien wird zusätzlich als ein
    // Vorgang gemessen. Die Einzelmessungen je Datei erklären eine lange
    // Hauptschleifenpause sonst nicht: 1.5.24 zeigte 225 ms Schleifenmaximum
    // bei nur 67 ms größter Einzeloperation.
    uint32_t lastFlushCycleMs = 0;
    uint32_t maxFlushCycleMs = 0;
    uint32_t totalFlushCycleMs = 0;
    uint32_t flushCycleCount = 0;
    uint32_t flushCycleStallCount = 0;
    // Welcher Schritt des aufgeteilten Durchlaufs den Höchstwert erzeugt hat.
    // Ohne diese Angabe ist `maxFlushCycleMs` nicht zuzuordnen: Über 3.180
    // Schritte einer Sitzung lag der Mittelwert bei 19 ms und das Maximum bei
    // 111 ms, ohne dass aus dem Log hervorging, welcher der Schritte das war.
    uint8_t maxFlushCycleStep = 0;
    uint32_t sensorSampleCount = 0;
    uint32_t sensorMissedSlots = 0;
    uint32_t gpsSnapshotCount = 0;
    uint32_t gpsMissedSlots = 0;
};

class RuntimeDiagnostics {
private:
    RuntimeTimingDiagnostics timing;

    void record(
        uint32_t durationMs, uint32_t& lastMs, uint32_t& maxMs,
        uint32_t& totalMs, uint32_t& sampleCount,
        uint32_t& stallCount);

public:
    void resetSession();
    void recordLoopInterval(uint32_t durationMs);
    void recordWebDuration(uint32_t durationMs);
    void recordSDDuration(uint32_t durationMs);
    void recordFlushCycle(uint32_t durationMs, uint8_t step = 0);
    void recordSensorSchedule(uint32_t elapsedMs, uint32_t intervalMs);
    void recordGPSSchedule(uint32_t elapsedMs, uint32_t intervalMs);
    RuntimeTimingDiagnostics getTiming() const { return timing; }
};

extern RuntimeDiagnostics runtimeDiagnostics;

#endif

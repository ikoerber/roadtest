#ifndef CURVE_DETECTOR_H
#define CURVE_DETECTOR_H

// Kurvenerkennung ohne Hardware- und Arduino-Abhängigkeiten.
//
// Diese Einheit enthält ausschließlich die Auswertelogik. Sie greift weder
// auf den Sensor noch auf millis(), Serial oder Wire zu; die Zeitbasis kommt
// mit jeder Stichprobe herein. Dadurch lässt sich die Logik auf dem
// Entwicklungsrechner testen, ohne Firmware zu flashen und ohne Flash zu
// belegen.
//
// Zeitangaben sind bewusst uint32_t und nicht unsigned long: Auf dem ESP32
// sind beide 32 Bit, auf einem 64-Bit-Host wäre unsigned long jedoch 64 Bit.
// Der feste Typ stellt sicher, dass Host- und Zieltest dasselbe
// Überlaufverhalten zeigen.

#include <stdint.h>

#include "hardware_config.h"

enum class CurveDetectionMode : uint8_t {
    SHARP,
    LONG
};

enum class CurveCompletionReason : uint8_t {
    QUIET,
    REVERSAL,
    TIMEOUT
};

enum CurveQualityFlag : uint16_t {
    CURVE_QUALITY_GYRO_PRIMARY = 1U << 0,
    CURVE_QUALITY_HEADING_FALLBACK = 1U << 1,
    CURVE_QUALITY_HEADING_MISMATCH = 1U << 2,
    CURVE_QUALITY_FEW_SAMPLES = 1U << 3
};

// Eingangsgröße der Kurvenerkennung. Bewusst nur die tatsächlich benötigten
// Felder, damit die Einheit nicht von SensorData und dessen
// Sensorbibliotheken abhängt.
struct CurveSample {
    uint32_t timestampMs = 0;
    float headingDeg = 0.0f;      // NDOF-Kurs, 0 bis 360 Grad
    float yawRateDps = 0.0f;      // einbaulagenunabhängige Gierrate
    bool yawRateValid = false;    // Schwerkraftbetrag im Plausibilitätsband
    bool gyroCalibrated = false;  // Kalibrierstufe des Gyroskops ist 3
    float speedKmh = -1.0f;       // negativ bedeutet unbekannt
};

struct CurveEvent {
    bool valid;
    uint32_t startTimeMs;
    uint32_t endTimeMs;
    uint32_t durationMs;
    uint32_t groupId;
    int8_t direction;
    float angleDeg;
    float headingAngleDeg;
    float distanceM;
    float meanSpeedKmh;
    float maxSpeedKmh;
    float meanYawRateDps;
    float maxYawRateDps;
    float radiusM;
    float meanLateralAccel;
    float maxLateralAccel;
    uint16_t sampleCount;
    uint16_t qualityFlags;
    CurveDetectionMode detectionMode;
    CurveCompletionReason completionReason;

    CurveEvent()
        : valid(false), startTimeMs(0), endTimeMs(0), durationMs(0),
          groupId(0), direction(0), angleDeg(0), headingAngleDeg(0),
          distanceM(0), meanSpeedKmh(0), maxSpeedKmh(0),
          meanYawRateDps(0), maxYawRateDps(0), radiusM(0),
          meanLateralAccel(0), maxLateralAccel(0), sampleCount(0),
          qualityFlags(0), detectionMode(CurveDetectionMode::LONG),
          completionReason(CurveCompletionReason::QUIET) {}
};

class CurveDetector {
public:
    CurveDetector() { reset(); }

    // Verarbeitet eine Stichprobe. Liefert true und füllt completedEvent,
    // wenn dabei eine Kurve abgeschlossen wurde.
    bool update(
        const CurveSample& sample, CurveEvent& completedEvent,
        float sharpStartRateDps = CURVE_SHARP_START_RATE_DPS);

    // Verwirft den gesamten Zustand. Ein angefangener Kurvenverlauf geht
    // dabei bewusst verloren.
    void reset();

    float getLastCompletedAngle() const { return lastCompletedCurveAngle; }

private:
    struct Accumulator {
        bool active;
        uint32_t startTimeMs;
        uint32_t lastTurnTimeMs;
        int8_t direction;
        float signedAngleDeg;
        float signedHeadingAngleDeg;
        float distanceM;
        float speedSumKmh;
        float maxSpeedKmh;
        float yawRateSumDps;
        float maxYawRateDps;
        float lateralAccelSum;
        float maxLateralAccel;
        uint16_t sampleCount;
        uint16_t gyroSampleCount;
        CurveDetectionMode detectionMode;

        void clear();
    };

    void addSample(
        Accumulator& accumulator, uint32_t intervalStartMs,
        uint32_t intervalEndMs, float signedTurnDeltaDeg,
        float headingDeltaDeg, float speedKmh, float yawRateDps,
        bool gyroPrimary);
    bool complete(
        const Accumulator& accumulator, uint32_t endTimeMs,
        uint32_t groupId, CurveCompletionReason reason,
        CurveEvent& completedEvent);

    // Vorbelegung im Kopf, damit reset() im Konstruktor keine
    // uninitialisierten Werte liest.
    float lastHeading = 0.0f;
    uint32_t lastHeadingTime = 0;
    bool headingInitialized = false;
    float lastCompletedCurveAngle = 0.0f;
    uint32_t curveQuietSince = 0;
    uint32_t lastCurveEvent = 0;
    Accumulator curveCandidate{};
    Accumulator activeCurve{};
    Accumulator curveReversal{};
    uint32_t nextCurveGroupId = 1;
    uint32_t activeCurveGroupId = 0;
};

#endif  // CURVE_DETECTOR_H

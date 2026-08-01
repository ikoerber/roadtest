#include "curve_detector.h"

#include <algorithm>
#include <cmath>

namespace {
// Arduino stellt DEG_TO_RAD bereit; diese Einheit soll aber ohne Arduino.h
// übersetzbar bleiben.
constexpr float kDegToRad = 0.017453292519943295f;
}  // namespace

void CurveDetector::Accumulator::clear() {
    active = false;
    startTimeMs = 0;
    lastTurnTimeMs = 0;
    direction = 0;
    signedAngleDeg = 0;
    signedHeadingAngleDeg = 0;
    distanceM = 0;
    speedSumKmh = 0;
    maxSpeedKmh = 0;
    yawRateSumDps = 0;
    maxYawRateDps = 0;
    lateralAccelSum = 0;
    maxLateralAccel = 0;
    sampleCount = 0;
    gyroSampleCount = 0;
    detectionMode = CurveDetectionMode::LONG;
}

bool CurveDetector::finish(CurveEvent& completedEvent) {
    completedEvent = CurveEvent();
    // Als Endzeit gilt die letzte tatsächliche Drehstichprobe, nicht der
    // Zeitpunkt des Stoppens. Ein bereits laufendes Ruhefenster ist
    // aussagekräftiger, weil dort die Drehung nachweislich endete.
    const uint32_t endTimeMs =
        curveQuietSince != 0 ? curveQuietSince : activeCurve.lastTurnTimeMs;
    const bool completed = complete(
        activeCurve, endTimeMs, activeCurveGroupId,
        CurveCompletionReason::SESSION_END, completedEvent);
    reset();
    if (completed) {
        lastCompletedCurveAngle = completedEvent.angleDeg;
    }
    return completed;
}

void CurveDetector::reset() {
    lastCompletedCurveAngle = 0;
    curveQuietSince = 0;
    quietNetAngleDeg = 0.0f;
    headingInitialized = false;
    curveCandidate.clear();
    activeCurve.clear();
    curveReversal.clear();
    activeCurveGroupId = 0;
    // lastHeading und lastHeadingTime folgen der nächsten Stichprobe, sobald
    // headingInitialized wieder gesetzt wird.
    lastHeading = 0;
    lastHeadingTime = 0;
    if (nextCurveGroupId == 0) {
        nextCurveGroupId = 1;
    }
}

void CurveDetector::addSample(
    Accumulator& accumulator, uint32_t intervalStartMs,
    uint32_t intervalEndMs, float signedTurnDeltaDeg,
    float headingDeltaDeg, float speedKmh, float yawRateDps,
    bool gyroPrimary) {
    if (!accumulator.active) {
        accumulator.clear();
        accumulator.active = true;
        accumulator.startTimeMs = intervalStartMs;
    }

    const float intervalSeconds =
        static_cast<float>(intervalEndMs - intervalStartMs) / 1000.0f;
    const float absoluteYawRate = std::fabs(yawRateDps);
    const float lateralAccel =
        speedKmh / 3.6f * absoluteYawRate * kDegToRad;

    accumulator.lastTurnTimeMs = intervalEndMs;
    accumulator.signedAngleDeg += signedTurnDeltaDeg;
    accumulator.signedHeadingAngleDeg += headingDeltaDeg;
    accumulator.distanceM += speedKmh / 3.6f * intervalSeconds;
    accumulator.speedSumKmh += speedKmh;
    accumulator.maxSpeedKmh = std::max(accumulator.maxSpeedKmh, speedKmh);
    accumulator.yawRateSumDps += absoluteYawRate;
    accumulator.maxYawRateDps =
        std::max(accumulator.maxYawRateDps, absoluteYawRate);
    accumulator.lateralAccelSum += lateralAccel;
    accumulator.maxLateralAccel =
        std::max(accumulator.maxLateralAccel, lateralAccel);
    accumulator.sampleCount++;
    if (gyroPrimary) {
        accumulator.gyroSampleCount++;
    }
    accumulator.direction =
        accumulator.signedAngleDeg > 0.0f
            ? 1
            : (accumulator.signedAngleDeg < 0.0f ? -1 : 0);
}

bool CurveDetector::complete(
    const Accumulator& accumulator, uint32_t endTimeMs,
    uint32_t groupId, CurveCompletionReason reason,
    CurveEvent& completedEvent) {
    const float angleDeg = std::fabs(accumulator.signedAngleDeg);
    if (!accumulator.active ||
        angleDeg < CURVE_MIN_EVENT_ANGLE_DEG ||
        accumulator.sampleCount == 0) {
        return false;
    }

    // Nachgewiesener Fahrweg statt Momentangeschwindigkeit. Die
    // Geschwindigkeitsfreigabe allein reicht nicht: GPS-Drift im Stand wird
    // mit bis zu 8 km/h als gültig ausgewiesen, und beim Rangieren entsteht
    // sonst aus wenigen Metern eine Kurve mit 3 m Radius. Der Streckenzähler
    // filtert dasselbe Rauschen seit jeher über eine Mindestsegmentlänge.
    if (accumulator.distanceM < CURVE_MIN_EVENT_DISTANCE_M) {
        return false;
    }

    // Eine Kurve, die niemand als Kurve fährt, ist keine. Maßgeblich ist die
    // Querbeschleunigung, nicht der Radius: Ein Bogen mit 500 m Radius bei
    // 90 km/h wurde von der Beifahrerseite ausdrücklich als Kurve markiert
    // und trägt 1,5 m/s², ein Autobahnbogen mit 1450 m Radius dagegen 0,4.
    const float meanLateralAccel =
        accumulator.lateralAccelSum / accumulator.sampleCount;
    if (meanLateralAccel < CURVE_MIN_EVENT_LATERAL_ACCEL) {
        return false;
    }

    completedEvent = CurveEvent();
    completedEvent.valid = true;
    completedEvent.startTimeMs = accumulator.startTimeMs;
    completedEvent.endTimeMs = std::max(endTimeMs, accumulator.startTimeMs);
    completedEvent.durationMs =
        completedEvent.endTimeMs - completedEvent.startTimeMs;
    completedEvent.groupId = groupId;
    completedEvent.direction = accumulator.direction;
    completedEvent.angleDeg = angleDeg;
    completedEvent.headingAngleDeg =
        std::fabs(accumulator.signedHeadingAngleDeg);
    completedEvent.distanceM = accumulator.distanceM;
    completedEvent.meanSpeedKmh =
        accumulator.speedSumKmh / accumulator.sampleCount;
    completedEvent.maxSpeedKmh = accumulator.maxSpeedKmh;
    completedEvent.meanYawRateDps =
        accumulator.yawRateSumDps / accumulator.sampleCount;
    completedEvent.maxYawRateDps = accumulator.maxYawRateDps;
    completedEvent.meanLateralAccel = meanLateralAccel;
    completedEvent.maxLateralAccel = accumulator.maxLateralAccel;
    completedEvent.sampleCount = accumulator.sampleCount;
    completedEvent.detectionMode = accumulator.detectionMode;
    completedEvent.completionReason = reason;
    if (angleDeg >= 1.0f) {
        completedEvent.radiusM =
            accumulator.distanceM / (angleDeg * kDegToRad);
    }

    if (accumulator.gyroSampleCount > 0) {
        completedEvent.qualityFlags |= CURVE_QUALITY_GYRO_PRIMARY;
    }
    if (accumulator.gyroSampleCount < accumulator.sampleCount) {
        completedEvent.qualityFlags |= CURVE_QUALITY_HEADING_FALLBACK;
    }
    const float headingDifference =
        std::fabs(completedEvent.headingAngleDeg - angleDeg);
    if (completedEvent.headingAngleDeg >= 5.0f &&
        headingDifference > std::max(8.0f, angleDeg * 0.35f)) {
        completedEvent.qualityFlags |= CURVE_QUALITY_HEADING_MISMATCH;
    }
    if (accumulator.sampleCount < CURVE_MIN_QUALITY_SAMPLES) {
        completedEvent.qualityFlags |= CURVE_QUALITY_FEW_SAMPLES;
    }

    lastCompletedCurveAngle = angleDeg;
    lastCurveEvent = completedEvent.endTimeMs;
    return true;
}

bool CurveDetector::update(
    const CurveSample& sample, CurveEvent& completedEvent,
    float sharpStartRateDps) {
    completedEvent = CurveEvent();
    if (sample.timestampMs == 0) return false;

    // Beim Ausrollen darf eine bereits qualifizierte Kurve nicht verloren
    // gehen. Der Zustand wird danach vollständig verworfen, damit beim
    // Wiederanfahren kein alter Verlauf fortgesetzt wird.
    if (!(sample.speedKmh >= ROAD_EVENT_MIN_SPEED_KMH)) {
        const bool completed = complete(
            activeCurve, activeCurve.lastTurnTimeMs, activeCurveGroupId,
            CurveCompletionReason::QUIET, completedEvent);
        reset();
        if (completed) {
            lastCompletedCurveAngle = completedEvent.angleDeg;
        }
        return completed;
    }

    if (!headingInitialized) {
        lastHeading = sample.headingDeg;
        lastHeadingTime = sample.timestampMs;
        headingInitialized = true;
        return false;
    }

    const uint32_t deltaTime = sample.timestampMs - lastHeadingTime;
    if (deltaTime == 0) return false;

    float headingDelta = sample.headingDeg - lastHeading;
    while (headingDelta > 180.0f) headingDelta -= 360.0f;
    while (headingDelta < -180.0f) headingDelta += 360.0f;

    if (deltaTime > CURVE_MAX_SAMPLE_GAP_MS) {
        const bool completed = complete(
            activeCurve, activeCurve.lastTurnTimeMs, activeCurveGroupId,
            CurveCompletionReason::TIMEOUT, completedEvent);
        reset();
        lastHeading = sample.headingDeg;
        lastHeadingTime = sample.timestampMs;
        headingInitialized = true;
        if (completed) {
            lastCompletedCurveAngle = completedEvent.angleDeg;
        }
        return completed;
    }

    const float intervalSeconds =
        static_cast<float>(deltaTime) / 1000.0f;
    const float headingRateDps = headingDelta / intervalSeconds;
    const bool gyroPrimary = sample.yawRateValid && sample.gyroCalibrated;
    const float selectedRateDps =
        gyroPrimary ? sample.yawRateDps : headingRateDps;
    const float selectedDeltaDeg = selectedRateDps * intervalSeconds;
    const float absoluteTurnRate = std::fabs(selectedRateDps);
    const int8_t turnDirection =
        selectedDeltaDeg > 0.0f
            ? 1
            : (selectedDeltaDeg < 0.0f ? -1 : 0);
    const bool meaningfulTurn =
        turnDirection != 0 &&
        absoluteTurnRate >= CURVE_LONG_MIN_RATE_DPS;

    if (!activeCurve.active) {
        const bool candidateExpired =
            curveCandidate.active &&
            (sample.timestampMs - curveCandidate.startTimeMs >
                 CURVE_LONG_MAX_WINDOW_MS ||
             sample.timestampMs - curveCandidate.lastTurnTimeMs >
                 CURVE_END_QUIET_MS);
        if (candidateExpired) {
            curveCandidate.clear();
        }

        if (meaningfulTurn) {
            addSample(
                curveCandidate, sample.timestampMs - deltaTime,
                sample.timestampMs, selectedDeltaDeg, headingDelta,
                sample.speedKmh, selectedRateDps, gyroPrimary);
        } else if (curveCandidate.active &&
                   sample.timestampMs - curveCandidate.lastTurnTimeMs >=
                       CURVE_END_QUIET_MS) {
            curveCandidate.clear();
        }

        const bool sharpStart =
            meaningfulTurn && absoluteTurnRate >= sharpStartRateDps &&
            sample.timestampMs - lastCurveEvent >= CURVE_EVENT_COOLDOWN_MS;
        const bool longStart =
            curveCandidate.active &&
            std::fabs(curveCandidate.signedAngleDeg) >=
                CURVE_LONG_START_ANGLE_DEG &&
            curveCandidate.distanceM >= CURVE_LONG_MIN_DISTANCE_M &&
            sample.timestampMs - curveCandidate.startTimeMs <=
                CURVE_LONG_MAX_WINDOW_MS;

        if (sharpStart || longStart) {
            activeCurve = curveCandidate;
            activeCurve.detectionMode =
                sharpStart ? CurveDetectionMode::SHARP
                           : CurveDetectionMode::LONG;
            curveCandidate.clear();
            curveQuietSince = 0;
            curveReversal.clear();
            activeCurveGroupId = 0;
        }
    } else {
        // Die Stichprobe zuerst einsortieren, danach das Ruhefenster
        // fortschreiben. Beides war früher verschränkt; dadurch setzte jede
        // einzelne Rauschspitze in Kurvenrichtung das Fenster zurück.
        if (meaningfulTurn && turnDirection == activeCurve.direction) {
            addSample(
                activeCurve, sample.timestampMs - deltaTime,
                sample.timestampMs, selectedDeltaDeg, headingDelta,
                sample.speedKmh, selectedRateDps, gyroPrimary);
            curveReversal.clear();
        } else if (meaningfulTurn) {
            addSample(
                curveReversal, sample.timestampMs - deltaTime,
                sample.timestampMs, selectedDeltaDeg, headingDelta,
                sample.speedKmh, selectedRateDps, gyroPrimary);
            if (std::fabs(curveReversal.signedAngleDeg) >=
                CURVE_REVERSAL_ANGLE_DEG) {
                if (activeCurveGroupId == 0) {
                    activeCurveGroupId = nextCurveGroupId++;
                    if (nextCurveGroupId == 0) {
                        nextCurveGroupId = 1;
                    }
                }
                const bool completed = complete(
                    activeCurve, curveReversal.startTimeMs,
                    activeCurveGroupId, CurveCompletionReason::REVERSAL,
                    completedEvent);
                activeCurve = curveReversal;
                activeCurve.detectionMode =
                    activeCurve.maxYawRateDps >= sharpStartRateDps
                        ? CurveDetectionMode::SHARP
                        : CurveDetectionMode::LONG;
                curveReversal.clear();
                curveQuietSince = 0;
                quietNetAngleDeg = 0.0f;
                lastHeading = sample.headingDeg;
                lastHeadingTime = sample.timestampMs;
                return completed;
            }
        } else {
            curveReversal.clear();
        }

        // Das Ruhefenster läuft durchgehend mit und wird nur von einer
        // tatsächlichen Netto-Kursänderung verworfen. Über die Fensterlänge
        // entspricht das einer mittleren Drehrate; symmetrisches Rauschen
        // hebt sich in der Summe auf, eine echte Kurve nicht.
        if (curveQuietSince == 0) {
            curveQuietSince = sample.timestampMs - deltaTime;
            quietNetAngleDeg = 0.0f;
        }
        quietNetAngleDeg += selectedDeltaDeg;
        if (std::fabs(quietNetAngleDeg) >= CURVE_QUIET_MIN_NET_ANGLE_DEG) {
            curveQuietSince = 0;
            quietNetAngleDeg = 0.0f;
        } else if (
            sample.timestampMs - curveQuietSince >= CURVE_END_QUIET_MS) {
            const bool completed = complete(
                activeCurve, curveQuietSince, activeCurveGroupId,
                CurveCompletionReason::QUIET, completedEvent);
            activeCurve.clear();
            curveQuietSince = 0;
            quietNetAngleDeg = 0.0f;
            activeCurveGroupId = 0;
            lastHeading = sample.headingDeg;
            lastHeadingTime = sample.timestampMs;
            return completed;
        }
    }

    if (activeCurve.active &&
        sample.timestampMs - activeCurve.startTimeMs >
            CURVE_MAX_DURATION_MS) {
        const bool completed = complete(
            activeCurve, sample.timestampMs, activeCurveGroupId,
            CurveCompletionReason::TIMEOUT, completedEvent);
        activeCurve.clear();
        curveQuietSince = 0;
        quietNetAngleDeg = 0.0f;
        curveReversal.clear();
        activeCurveGroupId = 0;
        lastHeading = sample.headingDeg;
        lastHeadingTime = sample.timestampMs;
        return completed;
    }

    lastHeading = sample.headingDeg;
    lastHeadingTime = sample.timestampMs;
    return false;
}

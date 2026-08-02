#include "road_metrics.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
// Arduino stellt constrain() als Makro bereit; diese Einheit soll ohne
// Arduino.h übersetzbar bleiben.
float clampf(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}
}  // namespace

void RoadMetricsAnalyzer::reset() {
    std::memset(accelBuffer, 0, sizeof(accelBuffer));
    bufferIndex = 0;
    bufferCount = 0;
    vibrationThreshold = VIBRATION_THRESHOLD;
    currentVibration = {0, 0, 0, 0};
    lastRoadQuality = 100.0f;
    hasRoadQuality = false;
    potholeArmed = false;
    potholeStartTime = 0;
    lastPotholeEvent = 0;
    resetSection();
    sectionCounter = 0;
}

void RoadMetricsAnalyzer::resetSection() {
    sectionStartMs = 0;
    sectionLastMs = 0;
    sectionDistanceM = 0;
    sectionSquareSum = 0;
    sectionMaxShock = 0;
    sectionShockCount = 0;
    sectionShockOpen = false;
    sectionSpeedSum = 0;
    sectionMaxSpeed = 0;
    sectionSamples = 0;
}

bool RoadMetricsAnalyzer::verticalFromGravity(
    float accelX, float accelY, float accelZ, float gravX, float gravY,
    float gravZ, float& verticalAccel) {
    const float magnitude =
        std::sqrt(gravX * gravX + gravY * gravY + gravZ * gravZ);
    if (!std::isfinite(magnitude) || magnitude < CURVE_GRAVITY_MIN_MPS2 ||
        magnitude > CURVE_GRAVITY_MAX_MPS2) {
        return false;
    }
    // Vorzeichen gedreht: Der Schwerkraftvektor zeigt nach unten, ein Stoß
    // von der Fahrbahn nach oben soll positiv erscheinen.
    const float value =
        -(accelX * gravX + accelY * gravY + accelZ * gravZ) / magnitude;
    if (!std::isfinite(value)) return false;
    verticalAccel = value;
    return true;
}

float RoadMetricsAnalyzer::driveabilityFromRms(float rmsVertical) {
    const float good = ROAD_DRIVEABILITY_RMS_GOOD_MPS2;
    const float bad = ROAD_DRIVEABILITY_RMS_BAD_MPS2;
    if (rmsVertical <= good) return 100.0f;
    if (rmsVertical >= bad) return 0.0f;
    return 100.0f * (bad - rmsVertical) / (bad - good);
}

float RoadMetricsAnalyzer::driveability(
    float rmsVertical, float maxSpeedKmh) {
    const float note = driveabilityFromRms(rmsVertical);
    if (maxSpeedKmh >= ROAD_DRIVEABILITY_FAST_KMH &&
        note < ROAD_DRIVEABILITY_FAST_MIN_NOTE) {
        return ROAD_DRIVEABILITY_FAST_MIN_NOTE;
    }
    return note;
}

bool RoadMetricsAnalyzer::updateSection(
    uint32_t timestampMs, float verticalAccel, float speedKmh,
    RoadSection& completed) {
    if (timestampMs == 0) return false;

    // Ohne nachgewiesene Bewegung wächst kein Abschnitt. Ein bereits
    // begonnener bleibt dabei erhalten: Eine kurze Verzögerung an einer
    // Kreuzung soll die Bewertung der Strecke nicht verwerfen.
    if (!(speedKmh >= ROAD_EVENT_MIN_SPEED_KMH)) {
        sectionLastMs = timestampMs;
        return false;
    }

    if (sectionSamples == 0) {
        sectionStartMs = timestampMs;
        sectionLastMs = timestampMs;
    }

    // Ein zu lange offener Abschnitt beschreibt keine zusammenhängende
    // Strecke mehr und wird verworfen statt bewertet.
    if (timestampMs - sectionStartMs > ROAD_SECTION_MAX_DURATION_MS) {
        resetSection();
        sectionStartMs = timestampMs;
        sectionLastMs = timestampMs;
    }

    // Eine Abtastlücke darf keinen Weg erfinden. Dieselbe Grenze wie im
    // CurveDetector: Über 500 ms ist die Zwischenzeit nicht mehr belegt.
    const uint32_t deltaMs = timestampMs - sectionLastMs;
    sectionLastMs = timestampMs;
    if (deltaMs <= CURVE_MAX_SAMPLE_GAP_MS) {
        sectionDistanceM += speedKmh / 3.6f * (deltaMs / 1000.0f);
    }

    sectionSquareSum += verticalAccel * verticalAccel;
    const float magnitude = verticalAccel < 0 ? -verticalAccel : verticalAccel;
    if (magnitude > sectionMaxShock) sectionMaxShock = magnitude;
    // Ein zusammenhängender Stoß zählt nur einmal, wie in analyzeVibration().
    if (magnitude > vibrationThreshold) {
        if (!sectionShockOpen) {
            sectionShockCount++;
            sectionShockOpen = true;
        }
    } else {
        sectionShockOpen = false;
    }
    sectionSpeedSum += speedKmh;
    if (speedKmh > sectionMaxSpeed) sectionMaxSpeed = speedKmh;
    sectionSamples++;

    if (sectionDistanceM < ROAD_SECTION_LENGTH_M ||
        sectionSamples < ROAD_SECTION_MIN_SAMPLES) {
        return false;
    }

    completed.startMs = sectionStartMs;
    completed.endMs = timestampMs;
    completed.distanceM = sectionDistanceM;
    completed.rmsVertical = std::sqrt(sectionSquareSum / sectionSamples);
    completed.maxShock = sectionMaxShock;
    completed.shockCount = sectionShockCount;
    completed.meanSpeedKmh = sectionSpeedSum / sectionSamples;
    completed.maxSpeedKmh = sectionMaxSpeed;
    completed.driveability =
        driveability(completed.rmsVertical, completed.maxSpeedKmh);
    completed.samples = sectionSamples;
    completed.number = ++sectionCounter;

    resetSection();
    return true;
}

void RoadMetricsAnalyzer::addSample(float verticalAccel) {
    accelBuffer[bufferIndex] = verticalAccel;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferCount < BUFFER_SIZE) {
        bufferCount++;
    }
}

VibrationMetrics RoadMetricsAnalyzer::analyzeVibration() {
    VibrationMetrics metrics = {0, 0, 0, 0};
    if (bufferCount == 0) return metrics;

    float sum = 0;
    float previousMagnitude = 0;
    int oldestIndex = (bufferIndex - bufferCount + BUFFER_SIZE) % BUFFER_SIZE;

    for (int i = 0; i < bufferCount; i++) {
        int index = (oldestIndex + i) % BUFFER_SIZE;
        float value = accelBuffer[index];
        float magnitude = std::fabs(value);

        sum += value * value;
        if (magnitude > metrics.maxShock) {
            metrics.maxShock = magnitude;
        }

        // Ein zusammenhängender Stoß zählt innerhalb des Fensters nur einmal.
        if (magnitude > vibrationThreshold &&
            (i == 0 || previousMagnitude <= vibrationThreshold)) {
            metrics.shockCount++;
        }
        previousMagnitude = magnitude;
    }

    metrics.rmsAccel = std::sqrt(sum / bufferCount);
    metrics.frequency = 0;
    currentVibration = metrics;
    return currentVibration;
}

float RoadMetricsAnalyzer::calculateRoadQuality(float speedKmh) {
    // Straßenqualität ohne nachgewiesene Fahrzeugbewegung ist kein Messwert.
    // Der frühere Zweig verglich gegen 3 km/h, war aber unerreichbar: Der
    // Aufrufer übergab bei ungültiger GPS-Geschwindigkeit -1, und
    // GPS-Geschwindigkeit gilt erst ab 6 km/h als gültig. Dadurch wurde im
    // belegten Stillstand Leerlaufvibration als Straßenlage bewertet
    // (20260730_071913_9B018397: 106 von 265 Werten unter 99,0, Minimum
    // 60,1). Ein negativer Rückgabewert bedeutet "kein Messwert" und darf
    // nicht als Zahl protokolliert werden.
    if (!(speedKmh >= ROAD_EVENT_MIN_SPEED_KMH)) {
        return -1.0f;
    }

    VibrationMetrics vib = analyzeVibration();

    // Messwerte vorsichtig auf eine Referenzgeschwindigkeit normieren. Die
    // Begrenzung vermeidet extreme Korrekturen - sie bewirkt aber auch, dass
    // oberhalb von 42,9 km/h gar keine Normierung mehr stattfindet.
    float speedFactor = 1.0f;
    if (speedKmh >= 5.0f) {
        speedFactor = clampf(
            ROAD_QUALITY_REFERENCE_SPEED_KMH / speedKmh,
            ROAD_QUALITY_SPEED_FACTOR_MIN,
            ROAD_QUALITY_SPEED_FACTOR_MAX);
    }

    float normalizedRms = vib.rmsAccel * speedFactor;
    float normalizedShock = vib.maxShock * std::sqrt(speedFactor);

    // Bewertung basierend auf Vibrationen (0-100 Punkte)
    float quality = 100;
    quality -= std::min(normalizedRms * 10.0f, 50.0f);
    quality -= std::min(normalizedShock * 2.0f, 30.0f);
    quality -= std::min(vib.shockCount * 5.0f, 20.0f);

    lastRoadQuality = std::max(quality, 0.0f);
    hasRoadQuality = true;
    return lastRoadQuality;
}

float RoadMetricsAnalyzer::getSmoothness() {
    VibrationMetrics vib = analyzeVibration();

    // Glattheit basierend auf RMS (invertiert, 0-1)
    return 1.0f / (1.0f + vib.rmsAccel);
}

bool RoadMetricsAnalyzer::detectPothole(
    uint32_t timestampMs, float accelZ, float speedKmh, float threshold) {
    if (timestampMs == 0) return false;

    // Ein Schlagloch setzt überfahrene Fahrbahn voraus. Ohne nachgewiesene
    // Bewegung wird die halbfertige Erkennung verworfen, damit ein im Stand
    // begonnener Ausschlag nicht beim Anfahren als Ereignis abschließt.
    if (!(speedKmh >= ROAD_EVENT_MIN_SPEED_KMH)) {
        potholeArmed = false;
        return false;
    }

    const uint32_t now = timestampMs;
    if (!potholeArmed && now - lastPotholeEvent >= ROAD_POTHOLE_REARM_MS &&
        accelZ <= -threshold) {
        potholeArmed = true;
        potholeStartTime = now;
    }

    if (potholeArmed) {
        if (accelZ >= threshold &&
            now - potholeStartTime <= ROAD_POTHOLE_WINDOW_MS) {
            potholeArmed = false;
            lastPotholeEvent = now;
            return true;
        }

        if (now - potholeStartTime > ROAD_POTHOLE_WINDOW_MS) {
            potholeArmed = false;
        }
    }

    return false;
}

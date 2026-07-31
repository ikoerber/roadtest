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
}

void RoadMetricsAnalyzer::addSample(float accelZ) {
    accelBuffer[bufferIndex] = accelZ;
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

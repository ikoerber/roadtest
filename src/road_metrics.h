#ifndef ROAD_METRICS_H
#define ROAD_METRICS_H

// Fahrbahnbewertung ohne Hardware- und Arduino-Abhängigkeiten.
//
// Enthält die Vibrationsanalyse, die Straßenqualitätszahl und die
// Schlaglocherkennung. Wie CurveDetector greift diese Einheit weder auf den
// Sensor noch auf millis(), Serial oder Wire zu; Zeit und Messwerte kommen
// mit dem Aufruf herein. Dadurch ist die Logik auf dem Entwicklungsrechner
// testbar, ohne Firmware-Flash zu belegen.

#include <stdint.h>

#include "hardware_config.h"

// Ergebnis der Vibrationsanalyse über das gleitende Fenster.
struct VibrationMetrics {
    float rmsAccel;       // Effektivwert der Vertikalbeschleunigung
    float maxShock;       // größter Betrag im Fenster
    float frequency;      // derzeit nicht bestimmt, bleibt 0
    uint32_t shockCount;  // zusammenhängende Überschreitungen der Schwelle
};

class RoadMetricsAnalyzer {
public:
    RoadMetricsAnalyzer() { reset(); }

    void reset();

    // Eine Vertikalbeschleunigung in das gleitende Fenster aufnehmen.
    void addSample(float accelZ);

    VibrationMetrics analyzeVibration();

    // Straßenqualität von 0 bis 100.
    //
    // Rückgabe kleiner null bedeutet ausdrücklich "kein Messwert": Ohne
    // nachgewiesene Fahrzeugbewegung ab ROAD_EVENT_MIN_SPEED_KMH entsteht
    // keine Bewertung. Dieser Fall darf nicht als Zahl protokolliert werden.
    float calculateRoadQuality(float speedKmh);

    // Glattheit von 0 bis 1, invertiert aus dem Effektivwert.
    float getSmoothness();

    // Schlagloch: Ausschlag nach unten, gefolgt von einem Ausschlag nach oben
    // innerhalb von ROAD_POTHOLE_WINDOW_MS. Liefert true genau im Moment des
    // Abschlusses. Ohne nachgewiesene Bewegung wird ein begonnener Ausschlag
    // verworfen.
    bool detectPothole(
        uint32_t timestampMs, float accelZ, float speedKmh,
        float threshold = VIBRATION_THRESHOLD);

    void setVibrationThreshold(float threshold) {
        vibrationThreshold = threshold;
    }
    float getVibrationThreshold() const { return vibrationThreshold; }
    VibrationMetrics getCurrentVibration() const { return currentVibration; }
    bool hasQuality() const { return hasRoadQuality; }
    float getLastQuality() const { return lastRoadQuality; }

private:
    static const int BUFFER_SIZE = 10;  // eine Sekunde bei 10 Hz

    float accelBuffer[BUFFER_SIZE];
    int bufferIndex;
    int bufferCount;

    float vibrationThreshold;
    VibrationMetrics currentVibration;
    float lastRoadQuality;
    bool hasRoadQuality;

    bool potholeArmed;
    uint32_t potholeStartTime;
    uint32_t lastPotholeEvent;
};

#endif  // ROAD_METRICS_H

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
    float frequency;      // nicht bestimmbar, siehe unten; bleibt 0
    uint32_t shockCount;  // zusammenhängende Überschreitungen der Schwelle
};

// Bewerteter Streckenabschnitt: "kann man hier zügig fahren?"
//
// Ein Abschnitt umfasst ROAD_SECTION_LENGTH_M gefahrenen Weg und wird als
// Ganzes bewertet. Die Note beschreibt die Fahrbahn, nicht die Fahrweise -
// Voraussetzung dafür ist, dass die Vertikalbeschleunigung projiziert
// übergeben wird und nicht als feste Sensorachse.
//
// Zur Frequenz: Belagsarten über ihre Anregungsfrequenz zu unterscheiden ist
// mit der Abtastung von 10 Hz ausgeschlossen. Kopfsteinpflaster regt bei
// 30 km/h mit rund 83 Hz an, Großpflaster mit 42 Hz, Kies mit über 160 Hz;
// die Nyquist-Grenze liegt bei 5 Hz. Messbar bleibt die Aufbaubewegung des
// Fahrzeugs bei 1 bis 2 Hz - und genau die entscheidet, ob zügiges Fahren
// angenehm bleibt. Der Effektivwert bleibt auch bei Unterabtastung ein
// brauchbares Energiemaß; Aliasing verfälscht die Frequenz, nicht die
// Gesamtenergie.
struct RoadSection {
    uint32_t startMs;
    uint32_t endMs;
    float distanceM;
    float rmsVertical;      // Effektivwert der Vertikalbeschleunigung, m/s²
    float maxShock;         // größter Einzelausschlag im Abschnitt, m/s²
    uint32_t shockCount;    // Überschreitungen der Stoßschwelle
    float meanSpeedKmh;
    float maxSpeedKmh;
    float driveability;     // 0 bis 100: 100 = zügig fahrbar
    uint32_t samples;
    uint32_t number;        // laufende Nummer in der Sitzung, ab 1
};

class RoadMetricsAnalyzer {
public:
    RoadMetricsAnalyzer() { reset(); }

    void reset();

    // Eine Vertikalbeschleunigung in das gleitende Fenster aufnehmen.
    //
    // Erwartet wird die auf die Schwerkraftrichtung projizierte lineare
    // Beschleunigung, nicht eine rohe Sensorachse. Siehe SensorData.
    void addSample(float verticalAccel);

    // Streckenabschnitt fortschreiben.
    //
    // Liefert true genau in dem Aufruf, in dem ein Abschnitt seine Länge
    // erreicht und in `completed` abgeschlossen wird. Ohne nachgewiesene
    // Bewegung ab ROAD_EVENT_MIN_SPEED_KMH wächst kein Abschnitt; steht einer
    // länger als ROAD_SECTION_MAX_DURATION_MS offen, wird er verworfen, damit
    // Fahrbahn vor und hinter einem Halt nicht zusammenfällt.
    bool updateSection(
        uint32_t timestampMs, float verticalAccel, float speedKmh,
        RoadSection& completed);

    // Note von 0 bis 100 aus dem vertikalen Effektivwert.
    static float driveabilityFromRms(float rmsVertical);

    // Note einschließlich der Plausibilitätsgrenze über die gefahrene
    // Höchstgeschwindigkeit. Wer schnell fährt, traut der Straße; der
    // Effektivwert allein stuft solche Abschnitte zu schlecht ein, weil die
    // Anregung mit der Geschwindigkeit steigt. Die Regel wirkt nur nach
    // oben: Niedrige Geschwindigkeit belegt nichts, sie kann am Verkehr
    // liegen.
    static float driveability(float rmsVertical, float maxSpeedKmh);

    // Lineare Beschleunigung auf die Schwerkraftrichtung projizieren.
    //
    // Ergebnis ist die Fahrbahnanregung, positiv entgegen der Schwerkraft.
    // Liefert false, wenn der Schwerkraftvektor unbrauchbar ist; dann
    // entsteht bewusst kein Messwert statt eines aus einer beliebigen
    // Sensorachse. Steht hier statt im BNO055Manager, damit die Projektion
    // ohne Hardware prüfbar bleibt - sie ist der Kern der Bewertung.
    static bool verticalFromGravity(
        float accelX, float accelY, float accelZ, float gravX, float gravY,
        float gravZ, float& verticalAccel);

    void resetSection();
    bool hasOpenSection() const { return sectionSamples > 0; }
    float openSectionDistanceM() const { return sectionDistanceM; }

    // Laufende Nummer des offenen Abschnitts, beginnend bei 1.
    //
    // Ein Beifahrerurteil trägt diese Nummer mit und gilt dadurch für genau
    // seinen Abschnitt. Ohne sie müsste die Auswertung annehmen, ein Urteil
    // gelte bis zum nächsten Marker - eine Pause von zehn Minuten zöge dann
    // die letzte Wertung über zehn Minuten unbewertete Straße.
    uint32_t currentSectionNumber() const { return sectionCounter + 1; }
    uint32_t completedSectionCount() const { return sectionCounter; }

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
        uint32_t timestampMs, float verticalAccel, float speedKmh,
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

    // Laufender Streckenabschnitt. Der Effektivwert entsteht aus der Summe
    // der Quadrate, damit kein Zwischenpuffer über die volle Abschnittslänge
    // nötig ist.
    uint32_t sectionStartMs;
    uint32_t sectionLastMs;
    float sectionDistanceM;
    float sectionSquareSum;
    float sectionMaxShock;
    uint32_t sectionShockCount;
    bool sectionShockOpen;
    float sectionSpeedSum;
    float sectionMaxSpeed;
    uint32_t sectionSamples;
    uint32_t sectionCounter;
};

#endif  // ROAD_METRICS_H

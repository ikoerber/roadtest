#ifndef BNO055_MANAGER_H
#define BNO055_MANAGER_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <Preferences.h>
#include "hardware_config.h"
#include "curve_detector.h"
#include "road_metrics.h"

// Betriebsmodus des BNO055.
//
// Ursprünglich IMUPLUS, weil für die Straßenmessung nur Gyro und
// Beschleunigung gebraucht werden und der Magnetometeranteil von NDOF als
// störanfällig galt.
//
// Nach wiederholten Fusionsfehlern im IMUPLUS-Betrieb wurde NDOF wieder
// aktiviert. Die spätere Logic-8-Aufnahme zeigte zusätzlich einen unabhängig
// davon zu behebenden I²C-Fehler durch leere BNO055-Prüftransaktionen.
//
// Zurückstellen auf IMUPLUS: OPERATION_MODE_IMUPLUS eintragen.
constexpr adafruit_bno055_opmode_t ROADTEST_BNO_MODE = OPERATION_MODE_NDOF;

// In NDOF gehört das Magnetometer zur Fusion. Der Kalibrierungsstatus gilt
// deshalb erst als vollständig, wenn auch mag den Wert 3 erreicht — das
// verlangt die bekannte Achterbewegung. In IMUPLUS bleibt mag unbeachtet.
constexpr bool ROADTEST_BNO_USES_MAG = (ROADTEST_BNO_MODE == OPERATION_MODE_NDOF);

// Klartextname für Log und Webseite, damit der Modus nur an einer
// Stelle steht und Ausgaben nicht auseinanderlaufen.
constexpr const char* ROADTEST_BNO_MODE_NAME = ROADTEST_BNO_USES_MAG ? "NDOF" : "IMUPLUS";

// Kalibrierungs-Datenstruktur
struct CalibrationData {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
    
    bool isFullyCalibrated() const {
        // getSensorOffsets() gibt im NDOF-Modus erst dann gültige Werte
        // zurück, wenn System, Gyro, Beschleunigung und Magnetometer jeweils
        // vollständig kalibriert sind.
        return gyro == 3 && accel == 3 &&
               (!ROADTEST_BNO_USES_MAG || (system == 3 && mag == 3));
    }
    
    uint8_t getMinimum() const {
        uint8_t minimum = gyro < accel ? gyro : accel;
        if (ROADTEST_BNO_USES_MAG) {
            minimum = minimum < mag ? minimum : mag;
            minimum = minimum < system ? minimum : system;
        }
        return minimum;
    }
};

// Laufzeitdiagnose des BNO055. Der konfigurierte Modus, Systemstatus 5 und
// Fehlercode 0 bedeuten zusammen, dass die Sensorfusion tatsächlich läuft.
struct BNO055RuntimeStatus {
    uint8_t operationMode;
    uint8_t systemStatus;
    uint8_t selfTestResult;
    uint8_t systemError;

    bool isExpectedModeActive() const {
        return operationMode == ROADTEST_BNO_MODE;
    }

    bool isFusionRunning() const {
        return isExpectedModeActive() && systemStatus == 5 && systemError == 0;
    }
};

// Erweiterte Sensor-Daten
struct SensorData {
    // Orientierung
    float heading;      // NDOF-Richtung 0-360° mit Magnetometerbezug
    float pitch;        // -180 bis +180°
    float roll;         // -90 bis +90°
    
    // Lineare Beschleunigung (ohne Gravitation)
    float accelX;       // m/s²
    float accelY;       // m/s²
    float accelZ;       // m/s²
    float accelMagnitude; // Gesamtbeschleunigung
    
    // Gyroskop. Adafruit_BNO055::getVector(VECTOR_GYROSCOPE) liefert °/s.
    float gyroX;        // °/s
    float gyroY;        // °/s
    float gyroZ;        // °/s
    
    // Gravitation
    float gravX;        // m/s²
    float gravY;        // m/s²
    float gravZ;        // m/s²

    // Einbaulagenunabhängige Fahrzeug-Drehrate. Der komplette
    // Gyroskopvektor wird auf die Schwerkraftrichtung projiziert; dadurch
    // ist keine feste Sensorachse als Fahrzeug-Hochachse vorausgesetzt.
    float yawRateDps;   // vorzeichenbehaftete Drehrate um die Vertikale
    bool yawRateValid;
    
    // Zusätzliche Metriken
    float temperature;  // °C
    CalibrationData calibration;
    unsigned long timestamp; // ms
};

// VibrationMetrics steht in road_metrics.h; CurveDetectionMode,
// CurveCompletionReason, CurveQualityFlag und CurveEvent stehen in
// curve_detector.h. Beide Einheiten sind frei von Arduino- und
// Sensorabhängigkeiten und dadurch auf dem Entwicklungsrechner testbar.

class BNO055Manager {
private:
    Adafruit_BNO055* sensor;
    bool initialized;
    bool selfTestPassed;
    bool fusionModeActive;
    bool dataValid;
    uint8_t fusionModeFailureCount;
    BNO055RuntimeStatus runtimeStatus;
    const uint8_t i2cAddress;

    BNO055RuntimeStatus readRuntimeStatus();
    
    // Kalibrierungs-Management
    bool calibrationSaved;
    adafruit_bno055_offsets_t calibrationOffsets;
    Preferences preferences;  // NVS für persistente Speicherung
    
    // Fahrbahnbewertung und Kurvenerkennung. Die gesamte Auswertelogik liegt
    // in RoadMetricsAnalyzer und CurveDetector und ist dort ohne Hardware
    // testbar; dieser Manager reicht nur noch die Sensorwerte durch.
    RoadMetricsAnalyzer roadMetrics;
    CurveDetector curveDetector;

public:
    // Der ADR-Pin ist im vorhandenen Aufbau fest mit 3,3 V verbunden.
    BNO055Manager();
    ~BNO055Manager();

    // Initialisierung
    // chipIdVerified darf nur unmittelbar nach erfolgreichem responds()
    // gesetzt werden und vermeidet dann eine doppelte Chip-ID-Abfrage.
    bool begin(bool chipIdVerified = false);
    void end();
    bool isReady() const { return initialized; }
    bool isSelfTestPassed() const { return initialized && selfTestPassed; }
    bool isDataValid() const {
        return initialized && fusionModeActive && dataValid;
    }
    void invalidateMeasurements() { dataValid = false; }

    // Fest verdrahtete I²C-Adresse.
    uint8_t getAddress() const { return i2cAddress; }

    // Antwortet auf der fest verdrahteten Adresse ein BNO055?
    bool responds() const;

    // Liest die BNO055-Chip-ID. Ein leerer I²C-Schreibzugriff wird bewusst
    // vermieden, weil er beim BNO055 den folgenden Transfer beschädigen kann.
    static bool probeAddress(uint8_t address);
    
    // Sensor-Konfiguration
    void setMode(adafruit_bno055_opmode_t mode);
    void setExtCrystal(bool useExternal);
    bool runSelfTest();
    bool ensureFusionMode();
    bool restartFusion();
    bool verifyFusionMode(bool chipIdVerified = false);
    bool isFusionModeActive() const {
        return initialized && fusionModeActive;
    }
    BNO055RuntimeStatus getRuntimeStatus() const { return runtimeStatus; }
    
    // Kalibrierung
    CalibrationData getCalibration();
    bool saveCalibration();
    bool loadCalibration();
    bool clearCalibration();  // Löscht gespeicherte Kalibrierung aus NVS
    bool isCalibrationSaved() const { return calibrationSaved; }
    void getCalibrationOffsets(adafruit_bno055_offsets_t* offsets);
    void setCalibrationOffsets(const adafruit_bno055_offsets_t* offsets);
    String getCalibrationInstructions();
    
    // Daten-Erfassung
    SensorData getCurrentData();
    imu::Vector<3> getVector(Adafruit_BNO055::adafruit_vector_type_t vectorType);
    float getTemperature();
    
    // Erweiterte Analyse
    VibrationMetrics analyzeVibration();
    void processSample(const SensorData& data);
    void updateVibrationBuffer(float accelZ);
    void setVibrationThreshold(float threshold) {
        roadMetrics.setVibrationThreshold(threshold);
    }
    
    // Kurven- und Fahrbahnereigniserkennung.
    //
    // speedKmh ist verbindlich und muss eine nachgewiesene
    // Fahrzeuggeschwindigkeit sein; ein negativer Wert bedeutet "unbekannt".
    // Unterhalb von ROAD_EVENT_MIN_SPEED_KMH und bei unbekannter
    // Geschwindigkeit entsteht kein Ereignis, und der interne Zustand wird
    // zurückgesetzt. Ohne diese Bindung lösten Leerlaufvibration und
    // Kursdrift im Stand Ereignisse aus.
    bool detectCurve(
        const SensorData& data, float speedKmh, CurveEvent& completedEvent,
        float turnRateThreshold = CURVE_SHARP_START_RATE_DPS);
    float getCurveAngle();

    // Schließt eine noch laufende Kurve ab. Beim Stoppen einer Messfahrt
    // verbindlich aufzurufen, sonst geht die letzte Kurve verloren.
    bool finishCurveDetection(CurveEvent& completedEvent);
    void resetCurveDetection();

    // Straßenqualitäts-Metriken. Rückgabe kleiner null bedeutet "kein
    // gültiger Messwert"; dieser Fall darf nicht als Zahl protokolliert
    // werden.
    float calculateRoadQuality(float speedKmh);
    float getSmoothness();
    bool detectPothole(
        const SensorData& data, float speedKmh, float threshold = 2.0);
    
    // Status und Diagnose
    void printSystemStatus();
    String getStatusString();
    String getErrorString();
    uint8_t getSystemError();
    
    // Utility
    static String vectorToString(const imu::Vector<3>& vec);
    static String quaternionToString(const imu::Quaternion& quat);
};

// Globale Instanz
extern BNO055Manager bnoManager;

#endif // BNO055_MANAGER_H

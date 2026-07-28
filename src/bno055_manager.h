#ifndef BNO055_MANAGER_H
#define BNO055_MANAGER_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <Preferences.h>

// Betriebsmodus des BNO055.
//
// Ursprünglich IMUPLUS, weil für die Straßenmessung nur Gyro und
// Beschleunigung gebraucht werden und der Magnetometeranteil von NDOF als
// störanfällig galt.
//
// Gegen IMUPLUS spricht der Messbefund vom 28. Juli 2026: Der Sensor meldet
// wiederholt SYS_ERR 9 ("Fusion algorithm configuration error") und startet
// sich rund alle 18 s neu. Die Logic-8-Aufnahme hat den I²C-Bus als Ursache
// ausgeschlossen — das OLED quittiert im selben Zeitraum jede Transaktion.
// Da NDOF nach Beobachtung früher über längere Zeit stabil lief, wird der
// Modus testweise zurückgestellt.
//
// Zurückstellen auf IMUPLUS: OPERATION_MODE_IMUPLUS eintragen.
constexpr adafruit_bno055_opmode_t ROADTEST_BNO_MODE = OPERATION_MODE_NDOF;

// In NDOF gehört das Magnetometer zur Fusion. Der Kalibrierungsstatus gilt
// deshalb erst als vollständig, wenn auch mag den Wert 3 erreicht — das
// verlangt die bekannte Achterbewegung. In IMUPLUS bleibt mag unbeachtet.
constexpr bool ROADTEST_BNO_USES_MAG = (ROADTEST_BNO_MODE == OPERATION_MODE_NDOF);

// Klartextname für Log, OLED und Webseite, damit der Modus nur an einer
// Stelle steht und Ausgaben nicht auseinanderlaufen.
constexpr const char* ROADTEST_BNO_MODE_NAME = ROADTEST_BNO_USES_MAG ? "NDOF" : "IMUPLUS";

// Kalibrierungs-Datenstruktur
struct CalibrationData {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
    
    bool isFullyCalibrated() const {
        // In NDOF zählt das Magnetometer zur Fusion und muss mitkalibriert
        // sein, sonst verweigert getSensorOffsets() die Herausgabe der Werte
        // und das automatische Sichern im NVS greift nicht.
        return gyro == 3 && accel == 3 &&
               (!ROADTEST_BNO_USES_MAG || mag == 3);
    }
    
    uint8_t getMinimum() const {
        return gyro < accel ? gyro : accel;
    }
};

// Laufzeitdiagnose des BNO055. IMUPLUS (0x08) und Systemstatus 5 bedeuten,
// dass die Sensorfusion tatsächlich läuft.
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
    float heading;      // Relative Richtung 0-360° (kein magnetischer Nordbezug)
    float pitch;        // -180 bis +180°
    float roll;         // -90 bis +90°
    
    // Lineare Beschleunigung (ohne Gravitation)
    float accelX;       // m/s²
    float accelY;       // m/s²
    float accelZ;       // m/s²
    float accelMagnitude; // Gesamtbeschleunigung
    
    // Gyroskop
    float gyroX;        // rad/s
    float gyroY;        // rad/s
    float gyroZ;        // rad/s
    
    // Gravitation
    float gravX;        // m/s²
    float gravY;        // m/s²
    float gravZ;        // m/s²
    
    // Zusätzliche Metriken
    float temperature;  // °C
    CalibrationData calibration;
    unsigned long timestamp; // ms
};

// Vibrations-Analyse
struct VibrationMetrics {
    float rmsAccel;     // RMS Beschleunigung
    float maxShock;     // Maximaler Stoß
    float frequency;    // Dominante Frequenz
    uint32_t shockCount; // Anzahl Stöße über Schwellwert
};

class BNO055Manager {
private:
    Adafruit_BNO055* sensor;
    bool initialized;
    bool selfTestPassed;
    bool fusionModeActive;
    uint8_t fusionModeFailureCount;
    BNO055RuntimeStatus runtimeStatus;
    uint8_t i2cAddress;

    BNO055RuntimeStatus readRuntimeStatus();
    bool detectAddress();
    static uint8_t alternateAddress(uint8_t address);
    
    // Kalibrierungs-Management
    bool calibrationSaved;
    adafruit_bno055_offsets_t calibrationOffsets;
    Preferences preferences;  // NVS für persistente Speicherung
    
    // Daten-Puffer für Analyse
    static const int BUFFER_SIZE = 10;  // 1 Sekunde bei 10 Hz
    float accelBuffer[BUFFER_SIZE];
    int bufferIndex;
    int bufferCount;
    
    // Vibrations-Analyse
    float vibrationThreshold;
    VibrationMetrics currentVibration;
    float lastRoadQuality;
    bool hasRoadQuality;
    
    // Orientierungs-Tracking
    float lastHeading;
    unsigned long lastHeadingTime;
    bool headingInitialized;
    bool inCurve;
    float curveStartHeading;
    float accumulatedCurveAngle;
    float lastCompletedCurveAngle;
    unsigned long curveStartTime;
    unsigned long curveQuietSince;
    unsigned long lastCurveEvent;

    // Schlagloch-Erkennung
    bool potholeArmed;
    unsigned long potholeStartTime;
    unsigned long lastPotholeEvent;
    
public:
    // Startadresse. Antwortet der Sensor dort nicht, wird die jeweils andere
    // dokumentierte BNO055-Adresse geprüft; Breakouts unterscheiden sich hier
    // je nach Beschaltung des ADR-Pins.
    BNO055Manager(uint8_t address = BNO055_ADDRESS_B);
    ~BNO055Manager();

    // Initialisierung
    bool begin();
    void end();
    bool isReady() const { return initialized; }
    bool isSelfTestPassed() const { return initialized && selfTestPassed; }

    // Tatsächlich verwendete I²C-Adresse (nach erfolgreicher Erkennung)
    uint8_t getAddress() const { return i2cAddress; }

    // Antwortet der Sensor auf einer der beiden möglichen Adressen?
    // Wird von der Hardware-Überwachung genutzt, damit dort keine Adresse
    // fest verdrahtet werden muss.
    bool responds() const;

    // Reiner I²C-Ping ohne Seiteneffekte
    static bool probeAddress(uint8_t address);
    
    // Sensor-Konfiguration
    void setMode(adafruit_bno055_opmode_t mode);
    void setExtCrystal(bool useExternal);
    bool runSelfTest();
    bool ensureFusionMode();
    bool restartFusion();
    bool verifyFusionMode();
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
    void setVibrationThreshold(float threshold) { vibrationThreshold = threshold; }
    
    // Kurven-Erkennung
    bool detectCurve(const SensorData& data, float turnRateThreshold = 8.0);
    float getCurveAngle();
    void resetCurveDetection();
    
    // Straßenqualitäts-Metriken
    float calculateRoadQuality(float speedKmh = -1.0f);
    float getSmoothness();
    bool detectPothole(const SensorData& data, float threshold = 2.0);
    
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

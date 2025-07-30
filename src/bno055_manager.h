#ifndef BNO055_MANAGER_H
#define BNO055_MANAGER_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Kalibrierungs-Datenstruktur
struct CalibrationData {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
    
    bool isFullyCalibrated() const {
        return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
    
    uint8_t getMinimum() const {
        uint8_t min = system;
        if (gyro < min) min = gyro;
        if (accel < min) min = accel;
        if (mag < min) min = mag;
        return min;
    }
};

// Erweiterte Sensor-Daten
struct SensorData {
    // Orientierung
    float heading;      // 0-360°
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
    
    // Magnetometer
    float magX;         // µT
    float magY;         // µT
    float magZ;         // µT
    
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
    uint8_t i2cAddress;
    
    // Kalibrierungs-Management
    bool calibrationSaved;
    adafruit_bno055_offsets_t calibrationOffsets;
    
    // Daten-Puffer für Analyse
    static const int BUFFER_SIZE = 100;
    float accelBuffer[BUFFER_SIZE];
    int bufferIndex;
    
    // Vibrations-Analyse
    float vibrationThreshold;
    VibrationMetrics currentVibration;
    
    // Orientierungs-Tracking
    float lastHeading;
    bool inCurve;
    float curveStartHeading;
    
public:
    BNO055Manager(uint8_t address = 0x29);
    ~BNO055Manager();
    
    // Initialisierung
    bool begin();
    void end();
    bool isReady() const { return initialized; }
    
    // Sensor-Konfiguration
    void setMode(adafruit_bno055_opmode_t mode);
    void setExtCrystal(bool useExternal);
    bool runSelfTest();
    
    // Kalibrierung
    CalibrationData getCalibration();
    bool saveCalibration();
    bool loadCalibration();
    void getCalibrationOffsets(adafruit_bno055_offsets_t* offsets);
    void setCalibrationOffsets(const adafruit_bno055_offsets_t* offsets);
    String getCalibrationInstructions();
    
    // Daten-Erfassung
    SensorData getCurrentData();
    imu::Vector<3> getVector(Adafruit_BNO055::adafruit_vector_type_t vectorType);
    float getTemperature();
    
    // Erweiterte Analyse
    VibrationMetrics analyzeVibration();
    void updateVibrationBuffer(float accelZ);
    void setVibrationThreshold(float threshold) { vibrationThreshold = threshold; }
    
    // Kurven-Erkennung
    bool detectCurve(float headingThreshold = 5.0);
    float getCurveAngle();
    void resetCurveDetection();
    
    // Straßenqualitäts-Metriken
    float calculateRoadQuality();
    float getSmoothness();
    bool detectPothole(float threshold = 2.0);
    
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
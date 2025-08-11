#include "bno055_manager.h"

// Globale Instanz
BNO055Manager bnoManager;

BNO055Manager::BNO055Manager(uint8_t address) 
    : sensor(nullptr), initialized(false), i2cAddress(address),
      calibrationSaved(false), bufferIndex(0), vibrationThreshold(2.0),
      lastHeading(0), inCurve(false), curveStartHeading(0) {
    
    currentVibration = {0, 0, 0, 0};
    memset(accelBuffer, 0, sizeof(accelBuffer));
}

BNO055Manager::~BNO055Manager() {
    end();
}

bool BNO055Manager::begin() {
    if (initialized) {
        return true;
    }
    
    // Sensor-Instanz erstellen mit Null-Check
    sensor = new Adafruit_BNO055(55, i2cAddress);
    
    if (!sensor) {
        Serial.println("FEHLER: Speicherallokation für BNO055 fehlgeschlagen!");
        return false;
    }
    
    if (!sensor->begin()) {
        Serial.printf("BNO055 nicht gefunden auf Adresse 0x%02X\n", i2cAddress);
        delete sensor;
        sensor = nullptr;
        return false;
    }
    
    Serial.println("BNO055 erfolgreich initialisiert!");
    
    // Externen Kristall verwenden für bessere Genauigkeit
    sensor->setExtCrystalUse(true);
    
    // Versuche gespeicherte Kalibrierung zu laden
    if (loadCalibration()) {
        Serial.println("Kalibrierung aus NVS geladen");
    }
    
    initialized = true;
    return true;
}

void BNO055Manager::end() {
    if (sensor) {
        delete sensor;
        sensor = nullptr;
    }
    initialized = false;
}

void BNO055Manager::setMode(adafruit_bno055_opmode_t mode) {
    if (!initialized) return;
    sensor->setMode(mode);
}

void BNO055Manager::setExtCrystal(bool useExternal) {
    if (!initialized) return;
    sensor->setExtCrystalUse(useExternal);
}

bool BNO055Manager::runSelfTest() {
    if (!initialized) return false;
    
    uint8_t system_status, self_test_results, system_error;
    sensor->getSystemStatus(&system_status, &self_test_results, &system_error);
    
    Serial.printf("Self-Test: 0x%02X\n", self_test_results);
    
    if (self_test_results == 0x0F) {
        Serial.println("Alle Tests bestanden!");
        return true;
    } else {
        Serial.println("Tests fehlgeschlagen:");
        if (!(self_test_results & 0x01)) Serial.println("  - Accelerometer");
        if (!(self_test_results & 0x02)) Serial.println("  - Magnetometer");
        if (!(self_test_results & 0x04)) Serial.println("  - Gyroscope");
        if (!(self_test_results & 0x08)) Serial.println("  - MCU");
        return false;
    }
}

CalibrationData BNO055Manager::getCalibration() {
    CalibrationData cal = {0, 0, 0, 0};
    
    if (!initialized) return cal;
    
    sensor->getCalibration(&cal.system, &cal.gyro, &cal.accel, &cal.mag);
    return cal;
}

bool BNO055Manager::saveCalibration() {
    if (!initialized) return false;
    
    // Kalibrierungsdaten vom Sensor holen
    sensor->getSensorOffsets(calibrationOffsets);
    
    // NVS öffnen
    preferences.begin("bno055_cal", false);  // false = Read/Write-Modus
    
    // Kalibrierungsdaten speichern
    preferences.putBytes("offsets", &calibrationOffsets, sizeof(calibrationOffsets));
    
    // Kalibrierungsstatus speichern
    CalibrationData cal = getCalibration();
    preferences.putUChar("cal_sys", cal.system);
    preferences.putUChar("cal_gyro", cal.gyro);
    preferences.putUChar("cal_accel", cal.accel);
    preferences.putUChar("cal_mag", cal.mag);
    
    // Zeitstempel speichern
    preferences.putULong("timestamp", millis());
    
    preferences.end();
    
    calibrationSaved = true;
    Serial.println("✅ Kalibrierung in NVS gespeichert!");
    Serial.printf("   System=%d, Gyro=%d, Accel=%d, Mag=%d\n", 
                  cal.system, cal.gyro, cal.accel, cal.mag);
    
    return true;
}

bool BNO055Manager::loadCalibration() {
    if (!initialized) return false;
    
    // NVS öffnen
    preferences.begin("bno055_cal", true);  // true = Read-Only-Modus
    
    // Prüfen ob Kalibrierungsdaten vorhanden
    size_t offsetsLen = preferences.getBytesLength("offsets");
    if (offsetsLen != sizeof(calibrationOffsets)) {
        preferences.end();
        Serial.println("⚠️ Keine gültigen Kalibrierungsdaten in NVS gefunden");
        return false;
    }
    
    // Kalibrierungsdaten laden
    preferences.getBytes("offsets", &calibrationOffsets, sizeof(calibrationOffsets));
    
    // Kalibrierungsstatus laden
    uint8_t savedSys = preferences.getUChar("cal_sys", 0);
    uint8_t savedGyro = preferences.getUChar("cal_gyro", 0);
    uint8_t savedAccel = preferences.getUChar("cal_accel", 0);
    uint8_t savedMag = preferences.getUChar("cal_mag", 0);
    unsigned long savedTime = preferences.getULong("timestamp", 0);
    
    preferences.end();
    
    // Kalibrierungsdaten auf Sensor anwenden
    sensor->setSensorOffsets(calibrationOffsets);
    
    // Alter der Kalibrierung berechnen
    unsigned long ageMinutes = (millis() - savedTime) / 60000;
    
    calibrationSaved = true;
    Serial.println("✅ Kalibrierung aus NVS geladen!");
    Serial.printf("   Gespeichert vor %lu Minuten\n", ageMinutes);
    Serial.printf("   Kalibrierungsstatus: System=%d, Gyro=%d, Accel=%d, Mag=%d\n", 
                  savedSys, savedGyro, savedAccel, savedMag);
    
    return true;
}

bool BNO055Manager::clearCalibration() {
    // NVS öffnen und alle Kalibrierungsdaten löschen
    preferences.begin("bno055_cal", false);
    preferences.clear();
    preferences.end();
    
    calibrationSaved = false;
    Serial.println("✅ Kalibrierungsdaten aus NVS gelöscht");
    
    return true;
}

void BNO055Manager::getCalibrationOffsets(adafruit_bno055_offsets_t* offsets) {
    if (!initialized || !offsets) return;
    sensor->getSensorOffsets(*offsets);
}

void BNO055Manager::setCalibrationOffsets(const adafruit_bno055_offsets_t* offsets) {
    if (!initialized || !offsets) return;
    sensor->setSensorOffsets(*offsets);
}

String BNO055Manager::getCalibrationInstructions() {
    CalibrationData cal = getCalibration();
    String instructions = "";
    
    if (cal.gyro < 3) {
        instructions += "Gyro: Gerät stillhalten\n";
    }
    if (cal.mag < 3) {
        instructions += "Mag: Figur-8 Bewegung\n";
    }
    if (cal.accel < 3) {
        instructions += "Accel: 6 Positionen (±X,±Y,±Z)\n";
    }
    if (cal.system < 3) {
        instructions += "System: Warte auf alle Sensoren\n";
    }
    
    if (cal.isFullyCalibrated()) {
        instructions = "Vollständig kalibriert!";
    }
    
    return instructions;
}

SensorData BNO055Manager::getCurrentData() {
    SensorData data = {0};
    
    if (!initialized) return data;
    
    // Zeitstempel
    data.timestamp = millis();
    
    // Orientierung (Euler-Winkel)
    imu::Vector<3> euler = sensor->getVector(Adafruit_BNO055::VECTOR_EULER);
    data.heading = euler.x();
    data.pitch = euler.y();
    data.roll = euler.z();
    
    // Lineare Beschleunigung (ohne Gravitation)
    imu::Vector<3> linAccel = sensor->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    data.accelX = linAccel.x();
    data.accelY = linAccel.y();
    data.accelZ = linAccel.z();
    data.accelMagnitude = sqrt(data.accelX*data.accelX + 
                              data.accelY*data.accelY + 
                              data.accelZ*data.accelZ);
    
    // Gyroskop
    imu::Vector<3> gyro = sensor->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    data.gyroX = gyro.x();
    data.gyroY = gyro.y();
    data.gyroZ = gyro.z();
    
    // Magnetometer
    imu::Vector<3> mag = sensor->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    data.magX = mag.x();
    data.magY = mag.y();
    data.magZ = mag.z();
    
    // Gravitation
    imu::Vector<3> grav = sensor->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    data.gravX = grav.x();
    data.gravY = grav.y();
    data.gravZ = grav.z();
    
    // Temperatur und Kalibrierung
    data.temperature = sensor->getTemp();
    data.calibration = getCalibration();
    
    // Vibrations-Buffer aktualisieren
    updateVibrationBuffer(data.accelZ);
    
    return data;
}

imu::Vector<3> BNO055Manager::getVector(Adafruit_BNO055::adafruit_vector_type_t vectorType) {
    if (!initialized) return imu::Vector<3>();
    return sensor->getVector(vectorType);
}

float BNO055Manager::getTemperature() {
    if (!initialized) return 0;
    return sensor->getTemp();
}

void BNO055Manager::updateVibrationBuffer(float accelZ) {
    accelBuffer[bufferIndex] = accelZ;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    
    // Stoß-Erkennung
    if (fabs(accelZ) > vibrationThreshold) {
        currentVibration.shockCount++;
        if (fabs(accelZ) > currentVibration.maxShock) {
            currentVibration.maxShock = fabs(accelZ);
        }
    }
}

VibrationMetrics BNO055Manager::analyzeVibration() {
    if (!initialized) return currentVibration;
    
    // RMS berechnen
    float sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += accelBuffer[i] * accelBuffer[i];
    }
    currentVibration.rmsAccel = sqrt(sum / BUFFER_SIZE);
    
    // TODO: FFT für Frequenz-Analyse implementieren
    currentVibration.frequency = 0;
    
    return currentVibration;
}

bool BNO055Manager::detectCurve(float headingThreshold) {
    if (!initialized) return false;
    
    SensorData data = getCurrentData();
    float headingDiff = fabs(data.heading - lastHeading);
    
    // Winkel-Wraparound berücksichtigen
    if (headingDiff > 180) {
        headingDiff = 360 - headingDiff;
    }
    
    if (!inCurve && headingDiff > headingThreshold) {
        inCurve = true;
        curveStartHeading = lastHeading;
    } else if (inCurve && headingDiff < headingThreshold / 2) {
        inCurve = false;
    }
    
    lastHeading = data.heading;
    return inCurve;
}

float BNO055Manager::getCurveAngle() {
    if (!initialized || !inCurve) return 0;
    
    float angle = fabs(lastHeading - curveStartHeading);
    if (angle > 180) {
        angle = 360 - angle;
    }
    
    return angle;
}

void BNO055Manager::resetCurveDetection() {
    inCurve = false;
    curveStartHeading = 0;
}

float BNO055Manager::calculateRoadQuality() {
    if (!initialized) return 0;
    
    VibrationMetrics vib = analyzeVibration();
    
    // Bewertung basierend auf Vibrationen (0-100 Punkte)
    float quality = 100;
    
    // RMS-Vibrationen (0-5 m/s² = perfekt bis schlecht)
    quality -= min(vib.rmsAccel * 10, 50.0f);
    
    // Maximale Stöße (>10 m/s² = sehr schlecht)
    quality -= min(vib.maxShock * 2, 30.0f);
    
    // Anzahl Stöße
    quality -= min(vib.shockCount * 0.5f, 20.0f);
    
    return max(quality, 0.0f);
}

float BNO055Manager::getSmoothness() {
    if (!initialized) return 0;
    
    VibrationMetrics vib = analyzeVibration();
    
    // Glattheit basierend auf RMS (invertiert, 0-1)
    return 1.0f / (1.0f + vib.rmsAccel);
}

bool BNO055Manager::detectPothole(float threshold) {
    if (!initialized) return false;
    
    SensorData data = getCurrentData();
    
    // Schlagloch = plötzliche negative Z-Beschleunigung gefolgt von positiver
    static float lastAccelZ = 0;
    static bool inPothole = false;
    
    if (!inPothole && data.accelZ < -threshold && lastAccelZ > 0) {
        inPothole = true;
    } else if (inPothole && data.accelZ > threshold && lastAccelZ < 0) {
        inPothole = false;
        lastAccelZ = data.accelZ;
        return true; // Schlagloch erkannt
    }
    
    lastAccelZ = data.accelZ;
    return false;
}

void BNO055Manager::printSystemStatus() {
    if (!initialized) {
        Serial.println("BNO055 nicht initialisiert");
        return;
    }
    
    Serial.println("\n=== BNO055 System Status ===");
    
    // System Status
    uint8_t system_status, self_test_results, system_error;
    sensor->getSystemStatus(&system_status, &self_test_results, &system_error);
    
    Serial.printf("System Status: 0x%02X\n", system_status);
    Serial.printf("Self Test: 0x%02X\n", self_test_results);
    Serial.printf("System Error: 0x%02X\n", system_error);
    
    // Kalibrierung
    CalibrationData cal = getCalibration();
    Serial.printf("Kalibrierung - Sys:%d Gyr:%d Acc:%d Mag:%d\n", 
                  cal.system, cal.gyro, cal.accel, cal.mag);
    
    // Temperatur
    Serial.printf("Temperatur: %.1f°C\n", getTemperature());
    
    // Aktuelle Daten
    SensorData data = getCurrentData();
    Serial.printf("Orientierung - Heading:%.1f° Pitch:%.1f° Roll:%.1f°\n",
                  data.heading, data.pitch, data.roll);
    Serial.printf("Lin.Beschl. - X:%.2f Y:%.2f Z:%.2f m/s²\n",
                  data.accelX, data.accelY, data.accelZ);
}

String BNO055Manager::getStatusString() {
    if (!initialized) return "Nicht initialisiert";
    
    CalibrationData cal = getCalibration();
    return String("BNO055 OK - Kal:") + String(cal.getMinimum()) + "/3";
}

String BNO055Manager::getErrorString() {
    if (!initialized) return "Nicht initialisiert";
    
    uint8_t error = getSystemError();
    switch(error) {
        case 0: return "Kein Fehler";
        case 1: return "Peripheral Init";
        case 2: return "System Init";
        case 3: return "Self Test Failed";
        case 4: return "Register Map Value";
        case 5: return "Register Map Address";
        case 6: return "Register Map Write";
        case 7: return "Low Power Mode";
        case 8: return "Accelerometer Power";
        case 9: return "Fusion Algorithm";
        case 10: return "Sensor Configuration";
        default: return "Unbekannter Fehler";
    }
}

uint8_t BNO055Manager::getSystemError() {
    if (!initialized) return 0xFF;
    
    uint8_t system_status, self_test_results, system_error;
    sensor->getSystemStatus(&system_status, &self_test_results, &system_error);
    return system_error;
}

String BNO055Manager::vectorToString(const imu::Vector<3>& vec) {
    return String("(") + String(vec.x(), 2) + ", " + 
           String(vec.y(), 2) + ", " + 
           String(vec.z(), 2) + ")";
}

String BNO055Manager::quaternionToString(const imu::Quaternion& quat) {
    return String("(") + String(quat.w(), 3) + ", " + 
           String(quat.x(), 3) + ", " + 
           String(quat.y(), 3) + ", " + 
           String(quat.z(), 3) + ")";
}
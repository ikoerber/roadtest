#include "bno055_manager.h"

// Globale Instanz
BNO055Manager bnoManager;

BNO055Manager::BNO055Manager(uint8_t address) 
    : sensor(nullptr), initialized(false), selfTestPassed(false),
      fusionModeActive(false), fusionModeFailureCount(0),
      runtimeStatus{0xFF, 0xFF, 0x00, 0xFF},
      i2cAddress(address),
      calibrationSaved(false), bufferIndex(0), bufferCount(0),
      vibrationThreshold(2.0), lastRoadQuality(100.0f),
      hasRoadQuality(false), lastHeading(0), lastHeadingTime(0),
      headingInitialized(false), inCurve(false), curveStartHeading(0),
      accumulatedCurveAngle(0), lastCompletedCurveAngle(0),
      curveStartTime(0), curveQuietSince(0), lastCurveEvent(0),
      potholeArmed(false), potholeStartTime(0), lastPotholeEvent(0) {
    
    currentVibration = {0, 0, 0, 0};
    memset(accelBuffer, 0, sizeof(accelBuffer));
}

BNO055Manager::~BNO055Manager() {
    end();
}

uint8_t BNO055Manager::alternateAddress(uint8_t address) {
    return address == BNO055_ADDRESS_B ? BNO055_ADDRESS_A : BNO055_ADDRESS_B;
}

bool BNO055Manager::probeAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool BNO055Manager::responds() const {
    return probeAddress(i2cAddress) || probeAddress(alternateAddress(i2cAddress));
}

bool BNO055Manager::detectAddress() {
    if (probeAddress(i2cAddress)) {
        return true;
    }

    const uint8_t fallback = alternateAddress(i2cAddress);
    if (probeAddress(fallback)) {
        Serial.printf("BNO055 antwortet auf 0x%02X statt 0x%02X; Adresse übernommen\n",
                      fallback, i2cAddress);
        i2cAddress = fallback;
        return true;
    }

    return false;
}

bool BNO055Manager::begin() {
    if (initialized) {
        return true;
    }

    // Erst prüfen, ob der Sensor überhaupt antwortet, und dabei die tatsächlich
    // belegte Adresse bestimmen. Das ist nicht nur Komfort:
    // Adafruit_BNO055::begin() wartet nach dem Software-Reset in einer Schleife
    // ohne Timeout auf die Chip-ID. Fehlt der Sensor oder ist der Bus gestört,
    // kehrt dieser Aufruf nie zurück und nimmt das gesamte Gerät mit.
    if (!detectAddress()) {
        Serial.printf("BNO055 antwortet weder auf 0x%02X noch auf 0x%02X\n",
                      i2cAddress, alternateAddress(i2cAddress));
        return false;
    }

    // Sensor-Instanz erstellen mit Null-Check
    sensor = new Adafruit_BNO055(55, i2cAddress);

    if (!sensor) {
        Serial.println("FEHLER: Speicherallokation für BNO055 fehlgeschlagen!");
        return false;
    }

    if (!sensor->begin(ROADTEST_BNO_MODE)) {
        Serial.printf("BNO055 nicht gefunden auf Adresse 0x%02X\n", i2cAddress);
        delete sensor;
        sensor = nullptr;
        return false;
    }
    
    Serial.println("BNO055 erfolgreich initialisiert!");
    
    // Der interne Takt ist bei diesem Aufbau stabiler. Die externe
    // 32-kHz-Taktquelle führte zeitweise zu SYS_ERR 9 und einem Rückfall in
    // CONFIG. Für 10-Hz-Straßenmessungen ist der interne Takt ausreichend.
    Serial.println("BNO055 verwendet die interne Taktquelle");
    delay(500);
    
    // loadCalibration() benötigt einen initialisierten Manager.
    initialized = true;

    // Versuche gespeicherte Kalibrierung zu laden
    if (loadCalibration()) {
        Serial.println("Kalibrierung aus NVS geladen");
    }

    if (!ensureFusionMode()) {
        Serial.println("⚠️ BNO055 konnte nicht in den IMUPLUS-Modus wechseln");
        end();
        return false;
    }

    return true;
}

void BNO055Manager::end() {
    if (sensor) {
        delete sensor;
        sensor = nullptr;
    }
    initialized = false;
    selfTestPassed = false;
    fusionModeActive = false;
    fusionModeFailureCount = 0;
    runtimeStatus = {0xFF, 0xFF, 0x00, 0xFF};
    calibrationSaved = false;
    bufferIndex = 0;
    bufferCount = 0;
    memset(accelBuffer, 0, sizeof(accelBuffer));
    currentVibration = {0, 0, 0, 0};
    lastRoadQuality = 100.0f;
    hasRoadQuality = false;
    headingInitialized = false;
    inCurve = false;
    potholeArmed = false;
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
    if (!initialized) {
        selfTestPassed = false;
        return false;
    }
    
    uint8_t system_status, self_test_results, system_error;
    sensor->getSystemStatus(&system_status, &self_test_results, &system_error);
    runtimeStatus.systemStatus = system_status;
    runtimeStatus.selfTestResult = self_test_results;
    runtimeStatus.systemError = system_error;
    fusionModeActive = runtimeStatus.isFusionRunning();
    
    Serial.printf("Self-Test: 0x%02X\n", self_test_results);
    
    // IMUPLUS benötigt Accelerometer, Gyro und MCU. Ein Fehler des nicht
    // verwendeten Magnetometers darf die Straßenmessung nicht blockieren.
    constexpr uint8_t REQUIRED_SELF_TESTS = 0x0D;
    if ((self_test_results & REQUIRED_SELF_TESTS) == REQUIRED_SELF_TESTS) {
        selfTestPassed = true;
        Serial.println("Alle für IMUPLUS benötigten Tests bestanden!");
        if (!(self_test_results & 0x02)) {
            Serial.println("Hinweis: Magnetometer-Test fehlgeschlagen (nicht benötigt)");
        }
        return true;
    } else {
        selfTestPassed = false;
        Serial.println("Tests fehlgeschlagen:");
        if (!(self_test_results & 0x01)) Serial.println("  - Accelerometer");
        if (!(self_test_results & 0x02)) Serial.println("  - Magnetometer");
        if (!(self_test_results & 0x04)) Serial.println("  - Gyroscope");
        if (!(self_test_results & 0x08)) Serial.println("  - MCU");
        return false;
    }
}

bool BNO055Manager::ensureFusionMode() {
    if (!initialized || !sensor) {
        return false;
    }

    adafruit_bno055_opmode_t currentMode = sensor->getMode();
    if (currentMode != ROADTEST_BNO_MODE) {
        Serial.printf("⚠️ BNO055 Modus 0x%02X; schalte auf IMUPLUS\n",
                      static_cast<uint8_t>(currentMode));
        sensor->setMode(ROADTEST_BNO_MODE);
    }

    runtimeStatus = readRuntimeStatus();
    const bool expectedMode = runtimeStatus.isExpectedModeActive();
    fusionModeActive = runtimeStatus.isFusionRunning();
    fusionModeFailureCount = 0;
    Serial.printf("BNO055 Betriebsmodus: 0x%02X (%s), System:%u, Fehler:%u\n",
                  runtimeStatus.operationMode,
                  expectedMode ? "IMUPLUS" : "FEHLER",
                  runtimeStatus.systemStatus, runtimeStatus.systemError);
    return expectedMode;
}

bool BNO055Manager::restartFusion() {
    Serial.println("BNO055 wird vollständig neu initialisiert");
    end();
    delay(100);
    return begin() && runSelfTest();
}

bool BNO055Manager::verifyFusionMode() {
    if (!initialized || !sensor) {
        fusionModeActive = false;
        return false;
    }

    runtimeStatus = readRuntimeStatus();
    if (runtimeStatus.isFusionRunning()) {
        fusionModeFailureCount = 0;
        fusionModeActive = true;
        return true;
    }

    // Einzelne Laufzeitstatus-Abfragen können trotz weiterlaufender Fusion
    // kurz fehlschlagen. Den zuletzt bestätigten aktiven Zustand deshalb bis
    // zum selben Grenzwert halten, der auch den Sensorneustart auslöst.
    if (fusionModeFailureCount < 3) {
        ++fusionModeFailureCount;
    }
    if (fusionModeFailureCount >= 3) {
        fusionModeActive = false;
    }
    Serial.printf("⚠️ BNO055 Fusionsprüfung %u/3: Modus=0x%02X, System=%u, Fehler=%u\n",
                  fusionModeFailureCount,
                  runtimeStatus.operationMode,
                  runtimeStatus.systemStatus,
                  runtimeStatus.systemError);

    // true bedeutet hier: noch keinen Neustart auslösen.
    return fusionModeFailureCount < 3;
}

BNO055RuntimeStatus BNO055Manager::readRuntimeStatus() {
    BNO055RuntimeStatus status = {0xFF, 0xFF, 0x00, 0xFF};
    if (!initialized || !sensor) {
        return status;
    }

    status.operationMode = static_cast<uint8_t>(sensor->getMode());
    sensor->getSystemStatus(&status.systemStatus, &status.selfTestResult,
                            &status.systemError);
    return status;
}

CalibrationData BNO055Manager::getCalibration() {
    CalibrationData cal = {0, 0, 0, 0};
    
    if (!initialized) return cal;
    
    sensor->getCalibration(&cal.system, &cal.gyro, &cal.accel, &cal.mag);
    return cal;
}

bool BNO055Manager::saveCalibration() {
    if (!initialized) return false;
    
    // Im IMUPLUS-Modus müssen Gyro und Beschleunigung kalibriert sein.
    // Ohne gültige Offsets wird nichts als "gespeichert" markiert.
    if (!sensor->getSensorOffsets(calibrationOffsets)) {
        Serial.println("❌ IMUPLUS-Kalibrierungswerte konnten nicht gelesen werden");
        return false;
    }
    
    // NVS öffnen
    preferences.begin("bno055_cal", false);  // false = Read/Write-Modus
    
    // Kalibrierungsdaten speichern
    preferences.putBytes("offsets", &calibrationOffsets, sizeof(calibrationOffsets));
    
    // Kalibrierungsstatus speichern
    CalibrationData cal = getCalibration();
    preferences.putUChar("cal_gyro", cal.gyro);
    preferences.putUChar("cal_accel", cal.accel);
    
    preferences.end();
    
    calibrationSaved = true;
    Serial.println("✅ IMUPLUS-Kalibrierung in NVS gespeichert!");
    Serial.printf("   Gyro=%d, Accel=%d\n", cal.gyro, cal.accel);
    
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
        calibrationSaved = false;
        Serial.println("⚠️ Keine gültigen Kalibrierungsdaten in NVS gefunden");
        return false;
    }
    
    // Kalibrierungsdaten laden
    preferences.getBytes("offsets", &calibrationOffsets, sizeof(calibrationOffsets));
    
    // Kalibrierungsstatus laden
    uint8_t savedGyro = preferences.getUChar("cal_gyro", 0);
    uint8_t savedAccel = preferences.getUChar("cal_accel", 0);
    preferences.end();
    
    // Kalibrierungsdaten auf Sensor anwenden
    sensor->setSensorOffsets(calibrationOffsets);
    
    calibrationSaved = true;
    Serial.println("✅ IMUPLUS-Kalibrierung aus NVS geladen!");
    Serial.printf("   Kalibrierungsstatus: Gyro=%d, Accel=%d\n",
                  savedGyro, savedAccel);
    
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
    if (cal.accel < 3) {
        instructions += "Accel: 6 Positionen (±X,±Y,±Z)\n";
    }
    
    if (cal.isFullyCalibrated()) {
        instructions = "Für IMUPLUS kalibriert!";
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
    
    // Gravitation
    imu::Vector<3> grav = sensor->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    data.gravX = grav.x();
    data.gravY = grav.y();
    data.gravZ = grav.z();
    
    // Temperatur und Kalibrierung
    data.temperature = sensor->getTemp();
    data.calibration = getCalibration();
    
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
    if (bufferCount < BUFFER_SIZE) {
        bufferCount++;
    }
}

void BNO055Manager::processSample(const SensorData& data) {
    if (!initialized || data.timestamp == 0) {
        return;
    }
    updateVibrationBuffer(data.accelZ);
}

VibrationMetrics BNO055Manager::analyzeVibration() {
    VibrationMetrics metrics = {0, 0, 0, 0};
    if (!initialized || bufferCount == 0) return metrics;

    float sum = 0;
    float previousMagnitude = 0;
    int oldestIndex = (bufferIndex - bufferCount + BUFFER_SIZE) % BUFFER_SIZE;

    for (int i = 0; i < bufferCount; i++) {
        int index = (oldestIndex + i) % BUFFER_SIZE;
        float value = accelBuffer[index];
        float magnitude = fabs(value);

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

    metrics.rmsAccel = sqrt(sum / bufferCount);
    metrics.frequency = 0;
    currentVibration = metrics;
    return currentVibration;
}

bool BNO055Manager::detectCurve(const SensorData& data, float turnRateThreshold) {
    if (!initialized || data.timestamp == 0) return false;

    if (!headingInitialized) {
        lastHeading = data.heading;
        lastHeadingTime = data.timestamp;
        headingInitialized = true;
        return false;
    }

    unsigned long deltaTime = data.timestamp - lastHeadingTime;
    if (deltaTime == 0) return false;

    float headingDelta = data.heading - lastHeading;
    while (headingDelta > 180.0f) headingDelta -= 360.0f;
    while (headingDelta < -180.0f) headingDelta += 360.0f;

    float turnRate = fabs(headingDelta) * 1000.0f / deltaTime;
    bool curveCompleted = false;

    if (!inCurve) {
        if (turnRate >= turnRateThreshold &&
            data.timestamp - lastCurveEvent >= 1000) {
            inCurve = true;
            curveStartHeading = lastHeading;
            accumulatedCurveAngle = headingDelta;
            curveStartTime = data.timestamp;
            curveQuietSince = 0;
        }
    } else {
        accumulatedCurveAngle += headingDelta;

        if (turnRate < turnRateThreshold * 0.4f) {
            if (curveQuietSince == 0) {
                curveQuietSince = data.timestamp;
            }

            if (data.timestamp - curveQuietSince >= 500) {
                lastCompletedCurveAngle = fabs(accumulatedCurveAngle);
                curveCompleted = lastCompletedCurveAngle >= 10.0f;
                inCurve = false;
                curveQuietSince = 0;
                if (curveCompleted) {
                    lastCurveEvent = data.timestamp;
                }
            }
        } else {
            curveQuietSince = 0;
        }

        // Verhindert einen dauerhaft offenen Kurvenzustand.
        if (inCurve && data.timestamp - curveStartTime > 30000) {
            lastCompletedCurveAngle = fabs(accumulatedCurveAngle);
            curveCompleted = lastCompletedCurveAngle >= 10.0f;
            inCurve = false;
            curveQuietSince = 0;
            if (curveCompleted) {
                lastCurveEvent = data.timestamp;
            }
        }
    }

    lastHeading = data.heading;
    lastHeadingTime = data.timestamp;
    return curveCompleted;
}

float BNO055Manager::getCurveAngle() {
    return initialized ? lastCompletedCurveAngle : 0;
}

void BNO055Manager::resetCurveDetection() {
    inCurve = false;
    curveStartHeading = 0;
    accumulatedCurveAngle = 0;
    lastCompletedCurveAngle = 0;
    curveQuietSince = 0;
    headingInitialized = false;
}

float BNO055Manager::calculateRoadQuality(float speedKmh) {
    if (!initialized) return 0;

    // Im Stillstand keine Fahrzeugbewegung als Straßenqualität bewerten.
    if (speedKmh >= 0.0f && speedKmh < 3.0f && hasRoadQuality) {
        return lastRoadQuality;
    }

    VibrationMetrics vib = analyzeVibration();

    // Messwerte vorsichtig auf eine Referenzgeschwindigkeit von 30 km/h
    // normieren. Die Begrenzung vermeidet extreme Korrekturen.
    float speedFactor = 1.0f;
    if (speedKmh >= 5.0f) {
        speedFactor = constrain(30.0f / speedKmh, 0.7f, 1.5f);
    }

    float normalizedRms = vib.rmsAccel * speedFactor;
    float normalizedShock = vib.maxShock * sqrt(speedFactor);

    // Bewertung basierend auf Vibrationen (0-100 Punkte)
    float quality = 100;
    quality -= min(normalizedRms * 10, 50.0f);
    quality -= min(normalizedShock * 2, 30.0f);
    quality -= min(vib.shockCount * 5.0f, 20.0f);

    lastRoadQuality = max(quality, 0.0f);
    hasRoadQuality = true;
    return lastRoadQuality;
}

float BNO055Manager::getSmoothness() {
    if (!initialized) return 0;
    
    VibrationMetrics vib = analyzeVibration();
    
    // Glattheit basierend auf RMS (invertiert, 0-1)
    return 1.0f / (1.0f + vib.rmsAccel);
}

bool BNO055Manager::detectPothole(const SensorData& data, float threshold) {
    if (!initialized || data.timestamp == 0) return false;

    const unsigned long now = data.timestamp;
    if (!potholeArmed && now - lastPotholeEvent >= 1200 &&
        data.accelZ <= -threshold) {
        potholeArmed = true;
        potholeStartTime = now;
    }

    if (potholeArmed) {
        if (data.accelZ >= threshold && now - potholeStartTime <= 600) {
            potholeArmed = false;
            lastPotholeEvent = now;
            return true;
        }

        if (now - potholeStartTime > 600) {
            potholeArmed = false;
        }
    }

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

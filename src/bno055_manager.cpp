#include "bno055_manager.h"

// Liefert ROAD_EVENT_MIN_SPEED_KMH.
#include "hardware_config.h"

// Globale Instanz
BNO055Manager bnoManager;

BNO055Manager::BNO055Manager()
    : sensor(nullptr), initialized(false), selfTestPassed(false),
      fusionModeActive(false), dataValid(false), fusionModeFailureCount(0),
      runtimeStatus{0xFF, 0xFF, 0x00, 0xFF},
      i2cAddress(BNO055_ADDRESS_B),
      calibrationSaved(false) {

    roadMetrics.reset();
    curveDetector.reset();
}

BNO055Manager::~BNO055Manager() {
    end();
}

bool BNO055Manager::probeAddress(uint8_t address) {
    while (Wire.available()) {
        Wire.read();
    }

    Wire.beginTransmission(address);
    Wire.write(static_cast<uint8_t>(Adafruit_BNO055::BNO055_CHIP_ID_ADDR));
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const size_t received =
        Wire.requestFrom(address, static_cast<size_t>(1), true);
    if (received != 1 || !Wire.available()) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }

    const uint8_t chipId = static_cast<uint8_t>(Wire.read());
    while (Wire.available()) {
        Wire.read();
    }
    return chipId == BNO055_ID;
}

bool BNO055Manager::responds() const {
    return probeAddress(i2cAddress);
}

bool BNO055Manager::begin(bool chipIdVerified) {
    if (initialized) {
        return true;
    }

    // Erst über die Chip-ID prüfen, ob auf der fest verdrahteten Adresse
    // wirklich ein BNO055 antwortet. Der Build begrenzt zusätzlich die
    // Wartezeit der Adafruit-Bibliothek nach ihrem Software-Reset.
    if (!chipIdVerified && !responds()) {
        Serial.printf("BNO055 antwortet nicht auf der festen Adresse 0x%02X\n",
                      i2cAddress);
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
        Serial.println("⚠️ BNO055 konnte nicht in den Fusionsmodus wechseln");
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
    dataValid = false;
    fusionModeFailureCount = 0;
    runtimeStatus = {0xFF, 0xFF, 0x00, 0xFF};
    calibrationSaved = false;
    roadMetrics.reset();
    resetCurveDetection();
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
        dataValid = false;
        return false;
    }
    
    uint8_t system_status, self_test_results, system_error;
    sensor->getSystemStatus(&system_status, &self_test_results, &system_error);
    runtimeStatus.systemStatus = system_status;
    runtimeStatus.selfTestResult = self_test_results;
    runtimeStatus.systemError = system_error;
    fusionModeActive = runtimeStatus.isFusionRunning();
    
    Serial.printf("Self-Test: 0x%02X\n", self_test_results);
    
    // Bit 0 Accelerometer, Bit 1 Magnetometer, Bit 2 Gyroskop, Bit 3 MCU.
    // IMUPLUS braucht Accel, Gyro und MCU; ein Fehler des dort ungenutzten
    // Magnetometers darf die Messung nicht blockieren. In NDOF gehört das
    // Magnetometer dagegen zur Fusion und muss mitgeprüft werden.
    constexpr uint8_t REQUIRED_SELF_TESTS = ROADTEST_BNO_USES_MAG ? 0x0F : 0x0D;
    if ((self_test_results & REQUIRED_SELF_TESTS) == REQUIRED_SELF_TESTS) {
        selfTestPassed = true;
        dataValid = fusionModeActive;
        Serial.println("Alle benötigten Selbsttests bestanden!");
        if (!(self_test_results & 0x02)) {
            Serial.println("Hinweis: Magnetometer-Test fehlgeschlagen (nicht benötigt)");
        }
        return true;
    } else {
        selfTestPassed = false;
        dataValid = false;
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
        Serial.printf("⚠️ BNO055 Modus 0x%02X; schalte auf %s\n",
                      static_cast<uint8_t>(currentMode), ROADTEST_BNO_MODE_NAME);
        sensor->setMode(ROADTEST_BNO_MODE);
    }

    runtimeStatus = readRuntimeStatus();
    const bool expectedMode = runtimeStatus.isExpectedModeActive();
    fusionModeActive = runtimeStatus.isFusionRunning();
    dataValid = fusionModeActive;
    fusionModeFailureCount = 0;
    Serial.printf("BNO055 Betriebsmodus: 0x%02X (%s), System:%u, Fehler:%u\n",
                  runtimeStatus.operationMode,
                  expectedMode ? ROADTEST_BNO_MODE_NAME : "FEHLER",
                  runtimeStatus.systemStatus, runtimeStatus.systemError);
    return expectedMode;
}

bool BNO055Manager::restartFusion() {
    Serial.println("BNO055 wird vollständig neu initialisiert");
    end();
    delay(100);
    return begin() && runSelfTest();
}

bool BNO055Manager::verifyFusionMode(bool chipIdVerified) {
    if (!initialized || !sensor) {
        fusionModeActive = false;
        dataValid = false;
        return false;
    }

    // Sofern der Aufrufer die Chip-ID nicht unmittelbar zuvor geprüft hat,
    // geschieht es hier. Ein fehlgeschlagener I²C-Lesevorgang liefert sonst
    // 0x00, was nicht sicher von einem echten CONFIG-Modus zu unterscheiden
    // wäre.
    if (!chipIdVerified && !responds()) {
        dataValid = false;
        if (fusionModeFailureCount < 3) {
            ++fusionModeFailureCount;
        }
        if (fusionModeFailureCount >= 3) {
            fusionModeActive = false;
        }
        Serial.printf("⚠️ BNO055 Fusionsprüfung %u/3: keine Antwort auf 0x%02X\n",
                      fusionModeFailureCount, i2cAddress);
        return fusionModeFailureCount < 3;
    }

    runtimeStatus = readRuntimeStatus();
    if (runtimeStatus.isFusionRunning()) {
        fusionModeFailureCount = 0;
        fusionModeActive = true;
        dataValid = true;
        return true;
    }

    // Schon der erste unplausible Status sperrt Messwerte. Nur die sichtbare
    // Störungsmeldung und der vollständige Neustart bleiben entprellt.
    dataValid = false;

    // Steht der Sensor lediglich im falschen Betriebsmodus, genügt es, den
    // Modus erneut zu setzen. Das dauert rund 7 ms und erhält die laufende
    // Kalibrierung. Ein vollständiger Neustart verwirft sie dagegen — genau
    // das führte bisher zur Dauerschleife aus Neustart und erneutem Ausfall.
    if (!runtimeStatus.isExpectedModeActive()) {
        Serial.printf("⚠️ BNO055 steht in Modus 0x%02X (System=%u, Fehler=%u); "
                      "setze %s erneut\n",
                      runtimeStatus.operationMode, runtimeStatus.systemStatus,
                      runtimeStatus.systemError, ROADTEST_BNO_MODE_NAME);
        sensor->setMode(ROADTEST_BNO_MODE);
        delay(30);
        runtimeStatus = readRuntimeStatus();
        if (runtimeStatus.isFusionRunning()) {
            fusionModeFailureCount = 0;
            fusionModeActive = true;
            dataValid = true;
            Serial.printf("✅ BNO055 ohne Neustart wieder in %s\n", ROADTEST_BNO_MODE_NAME);
            return true;
        }
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
    
    if (!isDataValid()) return cal;
    
    sensor->getCalibration(&cal.system, &cal.gyro, &cal.accel, &cal.mag);
    return cal;
}

bool BNO055Manager::saveCalibration() {
    if (!initialized) return false;

    CalibrationData cal = getCalibration();
    if (!cal.isFullyCalibrated()) {
        Serial.printf("❌ %s-Kalibrierung unvollständig (S%u/G%u/A%u/M%u)\n",
                      ROADTEST_BNO_MODE_NAME, cal.system, cal.gyro,
                      cal.accel, cal.mag);
        return false;
    }

    // Ohne gültige Offsets wird nichts als "gespeichert" markiert.
    if (!sensor->getSensorOffsets(calibrationOffsets)) {
        Serial.println("❌ Kalibrierungswerte konnten nicht gelesen werden");
        return false;
    }
    
    // NVS öffnen
    preferences.begin("bno055_cal", false);  // false = Read/Write-Modus
    
    // Kalibrierungsdaten speichern
    preferences.putBytes("offsets", &calibrationOffsets, sizeof(calibrationOffsets));
    
    // Kalibrierungsstatus speichern
    preferences.putUChar("cal_system", cal.system);
    preferences.putUChar("cal_gyro", cal.gyro);
    preferences.putUChar("cal_accel", cal.accel);
    preferences.putUChar("cal_mag", cal.mag);
    
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
        calibrationSaved = false;
        Serial.println("⚠️ Keine gültigen Kalibrierungsdaten in NVS gefunden");
        return false;
    }
    
    // Kalibrierungsdaten laden
    preferences.getBytes("offsets", &calibrationOffsets, sizeof(calibrationOffsets));
    
    // Kalibrierungsstatus laden
    uint8_t savedSystem = preferences.getUChar("cal_system", 0);
    uint8_t savedGyro = preferences.getUChar("cal_gyro", 0);
    uint8_t savedAccel = preferences.getUChar("cal_accel", 0);
    uint8_t savedMag = preferences.getUChar("cal_mag", 0);
    preferences.end();
    
    // Kalibrierungsdaten auf Sensor anwenden
    sensor->setSensorOffsets(calibrationOffsets);
    
    calibrationSaved = true;
    Serial.println("✅ Kalibrierung aus NVS geladen!");
    Serial.printf("   Kalibrierungsstatus: System=%d, Gyro=%d, Accel=%d, Mag=%d\n",
                  savedSystem, savedGyro, savedAccel, savedMag);
    
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
    if (ROADTEST_BNO_USES_MAG && cal.mag < 3) {
        instructions += "Mag: liegende Acht in der Luft beschreiben\n";
    }
    if (ROADTEST_BNO_USES_MAG && cal.system < 3) {
        instructions += "System: Bewegungen wiederholen, bis S=3\n";
    }

    if (cal.isFullyCalibrated()) {
        instructions = String("Für ") + ROADTEST_BNO_MODE_NAME + " kalibriert!";
    }
    
    return instructions;
}

SensorData BNO055Manager::getCurrentData() {
    SensorData data = {0};
    
    if (!isDataValid()) return data;
    
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

    const float gravityMagnitude =
        sqrt(data.gravX * data.gravX +
             data.gravY * data.gravY +
             data.gravZ * data.gravZ);
    data.yawRateValid =
        isfinite(gravityMagnitude) &&
        gravityMagnitude >= CURVE_GRAVITY_MIN_MPS2 &&
        gravityMagnitude <= CURVE_GRAVITY_MAX_MPS2;
    if (data.yawRateValid) {
        data.yawRateDps =
            (data.gyroX * data.gravX +
             data.gyroY * data.gravY +
             data.gyroZ * data.gravZ) /
            gravityMagnitude;
        data.yawRateValid = isfinite(data.yawRateDps);
    }

    // Vertikalbeschleunigung nach demselben Verfahren. accelX/Y/Z stammen aus
    // VECTOR_LINEARACCEL, die Schwerkraft ist also bereits abgezogen; die
    // Projektion liefert damit unmittelbar die Fahrbahnanregung. Die
    // Rechnung steht im hardwarefreien RoadMetricsAnalyzer, damit sie ohne
    // Gerät prüfbar bleibt.
    data.verticalAccelValid = RoadMetricsAnalyzer::verticalFromGravity(
        data.accelX, data.accelY, data.accelZ, data.gravX, data.gravY,
        data.gravZ, data.verticalAccel);
    
    // Temperatur und Kalibrierung
    data.temperature = sensor->getTemp();
    data.calibration = getCalibration();
    
    return data;
}

imu::Vector<3> BNO055Manager::getVector(Adafruit_BNO055::adafruit_vector_type_t vectorType) {
    if (!isDataValid()) return imu::Vector<3>();
    return sensor->getVector(vectorType);
}

float BNO055Manager::getTemperature() {
    if (!isDataValid()) return 0;
    return sensor->getTemp();
}

void BNO055Manager::processSample(const SensorData& data) {
    if (!initialized || data.timestamp == 0) {
        return;
    }
    // Nur die projizierte Vertikale beschreibt die Fahrbahn. Ist die
    // Schwerkraftrichtung unbrauchbar, entsteht lieber kein Messwert als
    // einer aus einer beliebigen Sensorachse.
    if (!data.verticalAccelValid) {
        return;
    }
    roadMetrics.addSample(data.verticalAccel);
}

VibrationMetrics BNO055Manager::analyzeVibration() {
    VibrationMetrics metrics = {0, 0, 0, 0};
    if (!initialized) return metrics;
    return roadMetrics.analyzeVibration();
}


bool BNO055Manager::detectCurve(
    const SensorData& data, float speedKmh, CurveEvent& completedEvent,
    float turnRateThreshold) {
    completedEvent = CurveEvent();
    if (!initialized) {
        return false;
    }

    // Die gesamte Auswertelogik liegt in CurveDetector. Hier werden nur die
    // Sensorwerte auf die hardwarefreie Eingangsstruktur abgebildet.
    CurveSample sample;
    sample.timestampMs = static_cast<uint32_t>(data.timestamp);
    sample.headingDeg = data.heading;
    sample.yawRateDps = data.yawRateDps;
    sample.yawRateValid = data.yawRateValid;
    sample.gyroCalibrated = data.calibration.gyro == 3;
    sample.speedKmh = speedKmh;

    return curveDetector.update(sample, completedEvent, turnRateThreshold);
}

float BNO055Manager::getCurveAngle() {
    return initialized ? curveDetector.getLastCompletedAngle() : 0;
}

bool BNO055Manager::finishCurveDetection(CurveEvent& completedEvent) {
    completedEvent = CurveEvent();
    if (!initialized) {
        return false;
    }
    return curveDetector.finish(completedEvent);
}

void BNO055Manager::resetCurveDetection() {
    curveDetector.reset();
}

float BNO055Manager::calculateRoadQuality(float speedKmh) {
    if (!initialized) return 0;
    return roadMetrics.calculateRoadQuality(speedKmh);
}

float BNO055Manager::getSmoothness() {
    if (!initialized) return 0;
    return roadMetrics.getSmoothness();
}

bool BNO055Manager::detectPothole(
    const SensorData& data, float speedKmh, float threshold) {
    if (!initialized || !data.verticalAccelValid) return false;
    return roadMetrics.detectPothole(
        static_cast<uint32_t>(data.timestamp), data.verticalAccel, speedKmh,
        threshold);
}

bool BNO055Manager::updateRoadSection(
    const SensorData& data, float speedKmh, RoadSection& completed) {
    if (!initialized || !data.verticalAccelValid) return false;
    const bool done = roadMetrics.updateSection(
        static_cast<uint32_t>(data.timestamp), data.verticalAccel, speedKmh,
        completed);
    if (done) {
        lastSection = completed;
        lastSectionValid = true;
    }
    return done;
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

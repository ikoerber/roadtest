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
      calibrationSaved(false), bufferIndex(0), bufferCount(0),
      vibrationThreshold(2.0), lastRoadQuality(100.0f),
      hasRoadQuality(false), lastHeading(0), lastHeadingTime(0),
      headingInitialized(false), inCurve(false), curveStartHeading(0),
      accumulatedCurveAngle(0), lastCompletedCurveAngle(0),
      curveStartTime(0), curveQuietSince(0), lastCurveEvent(0),
      curveDirection(0), curveCandidateActive(false),
      curveCandidateDirection(0), curveCandidateAngle(0),
      curveCandidateDistanceM(0), curveCandidateStartTime(0),
      curveCandidateLastTurnTime(0), curveReversalAngle(0),
      curveReversalStartTime(0),
      potholeArmed(false), potholeStartTime(0), lastPotholeEvent(0) {
    
    currentVibration = {0, 0, 0, 0};
    memset(accelBuffer, 0, sizeof(accelBuffer));
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
    bufferIndex = 0;
    bufferCount = 0;
    memset(accelBuffer, 0, sizeof(accelBuffer));
    currentVibration = {0, 0, 0, 0};
    lastRoadQuality = 100.0f;
    hasRoadQuality = false;
    headingInitialized = false;
    inCurve = false;
    curveDirection = 0;
    curveCandidateActive = false;
    curveCandidateDirection = 0;
    curveCandidateAngle = 0;
    curveCandidateDistanceM = 0;
    curveCandidateStartTime = 0;
    curveCandidateLastTurnTime = 0;
    curveReversalAngle = 0;
    curveReversalStartTime = 0;
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

bool BNO055Manager::detectCurve(
    const SensorData& data, float speedKmh, float turnRateThreshold) {
    if (!initialized || data.timestamp == 0) return false;

    // Ohne nachgewiesene Fahrzeugbewegung gibt es keine Kurve. Der Zustand
    // wird vollständig verworfen, damit beim Anfahren nicht der veraltete
    // Kurs der Standphase als Kursänderung erscheint.
    if (!(speedKmh >= ROAD_EVENT_MIN_SPEED_KMH)) {
        resetCurveDetection();
        return false;
    }

    if (!headingInitialized) {
        lastHeading = data.heading;
        lastHeadingTime = data.timestamp;
        headingInitialized = true;
        return false;
    }

    unsigned long deltaTime = data.timestamp - lastHeadingTime;
    if (deltaTime == 0) return false;
    if (deltaTime > CURVE_MAX_SAMPLE_GAP_MS) {
        // Nach einer längeren Abtastlücke ist nicht bekannt, auf welchem
        // Verlauf die Kursänderung entstand. Nicht aus zwei weit
        // auseinanderliegenden Punkten ein scheinbar präzises
        // Kurvenereignis ableiten.
        inCurve = false;
        curveDirection = 0;
        curveQuietSince = 0;
        accumulatedCurveAngle = 0;
        curveCandidateActive = false;
        curveCandidateAngle = 0;
        curveCandidateDistanceM = 0;
        curveReversalAngle = 0;
        lastHeading = data.heading;
        lastHeadingTime = data.timestamp;
        return false;
    }

    float headingDelta = data.heading - lastHeading;
    while (headingDelta > 180.0f) headingDelta -= 360.0f;
    while (headingDelta < -180.0f) headingDelta += 360.0f;

    const float absoluteHeadingDelta = fabs(headingDelta);
    const float turnRate =
        absoluteHeadingDelta * 1000.0f / deltaTime;
    const int8_t turnDirection =
        headingDelta > 0.0f ? 1 : (headingDelta < 0.0f ? -1 : 0);
    const float intervalDistanceM =
        speedKmh / 3.6f * (static_cast<float>(deltaTime) / 1000.0f);
    bool curveCompleted = false;

    if (!inCurve) {
        const bool meaningfulTurn =
            turnDirection != 0 && turnRate >= CURVE_LONG_MIN_RATE_DPS;

        if (meaningfulTurn) {
            const bool candidateExpired =
                curveCandidateActive &&
                (data.timestamp - curveCandidateStartTime >
                     CURVE_LONG_MAX_WINDOW_MS ||
                 data.timestamp - curveCandidateLastTurnTime >
                     CURVE_END_QUIET_MS);
            if (!curveCandidateActive || candidateExpired) {
                curveCandidateActive = true;
                curveCandidateDirection = turnDirection;
                curveCandidateAngle = absoluteHeadingDelta;
                curveCandidateDistanceM = intervalDistanceM;
                curveCandidateStartTime = data.timestamp - deltaTime;
            } else {
                // Vor dem Kurvenstart zählt die Netto-Richtungsänderung.
                // Einzelne kleine Gegenbewegungen des magnetischen Headings
                // löschen damit nicht mehr den gesamten Kandidaten.
                const float signedCandidateAngle =
                    curveCandidateDirection * curveCandidateAngle +
                    headingDelta;
                if (signedCandidateAngle > 0.0f) {
                    curveCandidateDirection = 1;
                    curveCandidateAngle = signedCandidateAngle;
                } else if (signedCandidateAngle < 0.0f) {
                    curveCandidateDirection = -1;
                    curveCandidateAngle = -signedCandidateAngle;
                } else {
                    curveCandidateAngle = 0;
                }
                curveCandidateDistanceM += intervalDistanceM;
            }
            curveCandidateLastTurnTime = data.timestamp;
        } else if (curveCandidateActive &&
                   data.timestamp - curveCandidateLastTurnTime >=
                       CURVE_END_QUIET_MS) {
            curveCandidateActive = false;
            curveCandidateAngle = 0;
            curveCandidateDistanceM = 0;
        }

        const bool sharpStart =
            turnRate >= turnRateThreshold &&
            data.timestamp - lastCurveEvent >= 1000;
        const bool longStart =
            curveCandidateActive &&
            curveCandidateAngle >= CURVE_LONG_START_ANGLE_DEG &&
            curveCandidateDistanceM >= CURVE_LONG_MIN_DISTANCE_M &&
            data.timestamp - curveCandidateStartTime <=
                CURVE_LONG_MAX_WINDOW_MS;

        if (sharpStart || longStart) {
            inCurve = true;
            curveDirection =
                curveCandidateActive
                    ? curveCandidateDirection
                    : turnDirection;
            curveStartHeading = data.heading -
                curveDirection * curveCandidateAngle;
            accumulatedCurveAngle =
                curveDirection *
                (curveCandidateActive
                     ? curveCandidateAngle
                     : absoluteHeadingDelta);
            curveStartTime =
                curveCandidateActive
                    ? curveCandidateStartTime
                    : data.timestamp - deltaTime;
            curveQuietSince = 0;
            curveReversalAngle = 0;
            curveReversalStartTime = 0;
            curveCandidateActive = false;
            curveCandidateAngle = 0;
            curveCandidateDistanceM = 0;
        }
    } else {
        if (turnRate >= CURVE_END_RATE_DPS && turnDirection != 0) {
            if (turnDirection == curveDirection) {
                accumulatedCurveAngle += headingDelta;
                curveQuietSince = 0;
                curveReversalAngle = 0;
                curveReversalStartTime = 0;
            } else {
                if (curveReversalAngle == 0) {
                    curveReversalStartTime = data.timestamp - deltaTime;
                }
                curveReversalAngle += absoluteHeadingDelta;

                // Eine ausreichend große Gegenkurve schließt die bisherige
                // Richtung ab und startet ohne blinden Zwischenraum die
                // nächste Hälfte einer S-Kurve.
                if (curveReversalAngle >= CURVE_REVERSAL_ANGLE_DEG) {
                    lastCompletedCurveAngle =
                        fabs(accumulatedCurveAngle);
                    curveCompleted =
                        lastCompletedCurveAngle >=
                        CURVE_MIN_EVENT_ANGLE_DEG;
                    if (curveCompleted) {
                        lastCurveEvent = data.timestamp;
                    }

                    curveDirection = turnDirection;
                    accumulatedCurveAngle =
                        curveDirection * curveReversalAngle;
                    curveStartHeading = data.heading -
                        accumulatedCurveAngle;
                    curveStartTime = curveReversalStartTime;
                    curveQuietSince = 0;
                    curveReversalAngle = 0;
                    curveReversalStartTime = 0;
                }
            }
        } else {
            curveReversalAngle = 0;
            curveReversalStartTime = 0;
            if (curveQuietSince == 0) {
                curveQuietSince = data.timestamp;
            }

            if (data.timestamp - curveQuietSince >= CURVE_END_QUIET_MS) {
                lastCompletedCurveAngle = fabs(accumulatedCurveAngle);
                curveCompleted =
                    lastCompletedCurveAngle >= CURVE_MIN_EVENT_ANGLE_DEG;
                inCurve = false;
                curveDirection = 0;
                curveQuietSince = 0;
                if (curveCompleted) {
                    lastCurveEvent = data.timestamp;
                }
            }
        }

        // Verhindert einen dauerhaft offenen Kurvenzustand.
        if (inCurve &&
            data.timestamp - curveStartTime > CURVE_MAX_DURATION_MS) {
            lastCompletedCurveAngle = fabs(accumulatedCurveAngle);
            curveCompleted =
                lastCompletedCurveAngle >= CURVE_MIN_EVENT_ANGLE_DEG;
            inCurve = false;
            curveDirection = 0;
            curveQuietSince = 0;
            curveReversalAngle = 0;
            curveReversalStartTime = 0;
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
    curveDirection = 0;
    curveCandidateActive = false;
    curveCandidateDirection = 0;
    curveCandidateAngle = 0;
    curveCandidateDistanceM = 0;
    curveCandidateStartTime = 0;
    curveCandidateLastTurnTime = 0;
    curveReversalAngle = 0;
    curveReversalStartTime = 0;
}

float BNO055Manager::calculateRoadQuality(float speedKmh) {
    if (!initialized) return 0;

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

bool BNO055Manager::detectPothole(
    const SensorData& data, float speedKmh, float threshold) {
    if (!initialized || data.timestamp == 0) return false;

    // Ein Schlagloch setzt überfahrene Fahrbahn voraus. Ohne nachgewiesene
    // Bewegung wird die halbfertige Erkennung verworfen, damit ein im Stand
    // begonnener Ausschlag nicht beim Anfahren als Ereignis abschließt.
    if (!(speedKmh >= ROAD_EVENT_MIN_SPEED_KMH)) {
        potholeArmed = false;
        return false;
    }

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

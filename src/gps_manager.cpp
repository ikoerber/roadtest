#include "gps_manager.h"
#include "hardware_config.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>

// Statische Member-Variablen
GPSManager* GPSManager::instance = nullptr;

// Globale GPS-Manager Instanz
GPSManager gpsManager;

static_assert(
    GPS_MIN_SATELLITES >= 4,
    "GPS-Positionsfreigabe benötigt mindestens vier Satelliten");
static_assert(
    GPS_LOCATION_MAX_AGE_MS <= 5000,
    "GPS-Positionsalter darf die bisherige Fixgrenze nicht überschreiten");
static_assert(
    GPS_MIN_RELIABLE_SPEED_KMH > GPS_DISTANCE_OBD_STATIONARY_KMH,
    "GPS-Bewegungsschwelle muss oberhalb der OBD-Stillstandsgrenze liegen");

GPSManager::GPSManager() : 
    serial(nullptr), initialized(false), rxPin(16), txPin(15), 
    baudRate(9600), lastDataUpdate(0), lastCharacterUpdate(0),
    lastNMEAStartUpdate(0), dataReady(false),
    interruptEnabled(false), rxIndex(0), rxProcessIndex(0),
    rxBufferOverflows(0), hasObservedLocationCommit(false),
    lastLocationCommitMs(0), fixSequence(0),
    lastDeliveredFixSequence(0) {
    
    // Status initialisieren
    status.initialized = false;
    status.communicating = false;
    status.buffered_bytes = 0;
    status.last_update = 0;
    
    // Leere GPS-Daten
    lastValidData = {};
    
    // Instance für Interrupt-Handler setzen
    instance = this;
}

GPSManager::~GPSManager() {
    end();
}

bool GPSManager::begin(int rx, int tx, uint32_t baud) {
    if (initialized) {
        Serial.println("GPS-Manager bereits initialisiert");
        return true;
    }
    
    rxPin = rx;
    txPin = tx;
    baudRate = baud;
    status.initialized = false;
    status.communicating = false;
    status.buffered_bytes = 0;
    status.last_update = 0;
    lastDataUpdate = 0;
    lastCharacterUpdate = 0;
    lastNMEAStartUpdate = 0;
    lastValidData = {};
    dataReady = false;
    rxIndex = 0;
    rxProcessIndex = 0;
    status.rx_buffer_overflows = rxBufferOverflows;
    
    Serial.println("=== GPS-Manager Initialisierung ===");
    Serial.printf("UART2: RX=%d, TX=%d, Baud=%lu\n", rxPin, txPin, baudRate);
    
    // Hardware UART2 initialisieren
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    serial = &Serial2;
    
    status.initialized = true;
    initialized = true;
    
    Serial.println("✅ GPS-UART initialisiert; NMEA-Prüfung läuft im Hintergrund");
    
    return true;
}

void GPSManager::end() {
    // Interrupt-Modus deaktivieren falls aktiv
    if (interruptEnabled) {
        enableInterruptMode(false);
    }
    
    if (serial) {
        serial->end();
        serial = nullptr;
    }
    initialized = false;
    status.initialized = false;
    status.communicating = false;
    status.buffered_bytes = 0;
    status.last_update = 0;
    lastDataUpdate = 0;
    lastCharacterUpdate = 0;
    lastNMEAStartUpdate = 0;
    lastValidData = {};
    dataReady = false;
    rxIndex = 0;
    rxProcessIndex = 0;
    status.rx_buffer_overflows = rxBufferOverflows;
    
    Serial.println("GPS-Manager beendet");
}

void GPSManager::update() {
    if (!initialized || !serial) {
        return;
    }
    
    if (interruptEnabled) {
        // Im Interrupt-Modus: Verarbeite gepufferte Daten
        processInterruptData();
    } else {
        // Polling-Modus: Direkt vom UART lesen
        while (serial->available() > 0) {
            char c = serial->read();
            unsigned long now = millis();
            lastCharacterUpdate = now;
            if (c == '$') {
                lastNMEAStartUpdate = now;
            }
            
            if (gps.encode(c)) {
                // Neuer kompletter NMEA-Satz verarbeitet
                status.sentences_received++;
                lastDataUpdate = millis();
                status.communicating = true;
                status.last_update = lastDataUpdate;
            }
            status.chars_processed++;
        }
    }
    
    // Status aktualisieren
    updateStatus();
}

// Interrupt-Handler (muss static und IRAM_ATTR sein)
void IRAM_ATTR GPSManager::onReceiveInterrupt() {
    if (!instance || !instance->serial) return;
    
    // Alle verfügbaren Bytes in den Ring-Buffer kopieren
    while (instance->serial->available() > 0) {
        uint8_t byte = instance->serial->read();
        
        // Ring-Buffer Index berechnen
        size_t nextIndex = (instance->rxIndex + 1) % RX_BUFFER_SIZE;
        
        // Prüfen ob Buffer voll ist
        if (nextIndex != instance->rxProcessIndex) {
            instance->rxBuffer[instance->rxIndex] = byte;
            instance->rxIndex = nextIndex;
            instance->dataReady = true;
        } else {
            instance->rxBufferOverflows =
                instance->rxBufferOverflows + 1;
        }
        // Der neue Byte wird bei vollem Puffer verworfen. Der Zähler macht
        // diesen Datenverlust ab 1.5.15 sichtbar.
    }
}

// Verarbeitet Daten aus dem Interrupt-Buffer
void GPSManager::processInterruptData() {
    if (!dataReady) return;
    
    // Interrupts temporär deaktivieren für Thread-Sicherheit
    noInterrupts();
    size_t currentRxIndex = rxIndex;
    interrupts();
    
    // Alle gepufferten Bytes verarbeiten
    while (rxProcessIndex != currentRxIndex) {
        char c = rxBuffer[rxProcessIndex];
        rxProcessIndex = (rxProcessIndex + 1) % RX_BUFFER_SIZE;
        unsigned long now = millis();
        lastCharacterUpdate = now;
        if (c == '$') {
            lastNMEAStartUpdate = now;
        }
        
        if (gps.encode(c)) {
            // Neuer kompletter NMEA-Satz verarbeitet
            status.sentences_received++;
            lastDataUpdate = millis();
            status.communicating = true;
            status.last_update = lastDataUpdate;
        }
        status.chars_processed++;
    }
    
    // Prüfen ob alle Daten verarbeitet wurden
    noInterrupts();
    if (rxProcessIndex == rxIndex) {
        dataReady = false;
    }
    interrupts();
}

// Interrupt-Modus aktivieren/deaktivieren
void GPSManager::enableInterruptMode(bool enable) {
    if (!initialized || !serial) {
        Serial.println("GPS nicht initialisiert!");
        return;
    }
    
    if (enable && !interruptEnabled) {
        // Buffer zurücksetzen
        rxIndex = 0;
        rxProcessIndex = 0;
        dataReady = false;
        // Interrupt-Handler registrieren
        serial->onReceive(onReceiveInterrupt);
        
        interruptEnabled = true;
        Serial.println("GPS Interrupt-Modus aktiviert");
        
    } else if (!enable && interruptEnabled) {
        // Interrupt-Handler entfernen
        serial->onReceive(nullptr);
        
        interruptEnabled = false;
        Serial.println("GPS Polling-Modus aktiviert");
    }
}

bool GPSManager::available() {
    if (!initialized) return false;
    return isDataFresh() && hasValidFix();
}

GPSData GPSManager::getCurrentData(bool consumeNewFix) {
    GPSData data;
    if (!initialized) {
        return data;
    }

    const uint32_t now = millis();

    uint8_t rejectionReason = GPS_REJECT_NONE;

    // Qualitätsfelder zuerst lesen, weil die Positionsfreigabe sowohl eine
    // frische Satellitenzahl als auch einen frischen HDOP benötigt.
    if (gps.satellites.isValid()) {
        data.satellites_age_ms = gps.satellites.age();
        data.satellites = gps.satellites.value();
        data.satellites_valid =
            data.satellites_age_ms <= GPS_QUALITY_MAX_AGE_MS;
    }
    if (!data.satellites_valid ||
        data.satellites < GPS_MIN_SATELLITES) {
        rejectionReason |= GPS_REJECT_SATELLITE_QUALITY;
    }

    if (gps.hdop.isValid()) {
        data.hdop_age_ms = gps.hdop.age();
        data.hdop = gps.hdop.hdop();
        data.hdop_valid =
            data.hdop_age_ms <= GPS_QUALITY_MAX_AGE_MS;
    }
    if (!data.hdop_valid || data.hdop > GPS_MAX_HDOP) {
        rejectionReason |= GPS_REJECT_HDOP_QUALITY;
    }

    // Aktuelle GPS-Daten von TinyGPS++ holen. Die Rohwerte bleiben für die
    // Diagnose erhalten, aber nur nachgewiesen frische und plausible Felder
    // bekommen ihr Valid-Flag.
    if (gps.location.isValid()) {
        data.location_age_ms = gps.location.age();
        data.latitude = gps.location.lat();
        data.longitude = gps.location.lng();
        data.location_valid =
            data.location_age_ms <= GPS_LOCATION_MAX_AGE_MS;
    }
    if (!data.location_valid) {
        rejectionReason |= GPS_REJECT_LOCATION_AGE;
    }
    if ((rejectionReason &
         (GPS_REJECT_SATELLITE_QUALITY | GPS_REJECT_HDOP_QUALITY)) != 0) {
        data.location_valid = false;
    }
    
    if (gps.altitude.isValid()) {
        data.altitude_age_ms = gps.altitude.age();
        data.altitude = gps.altitude.meters();
        data.altitude_valid =
            data.altitude_age_ms <= GPS_ALTITUDE_MAX_AGE_MS &&
            data.altitude >= GPS_MIN_ALTITUDE_M &&
            data.altitude <= GPS_MAX_ALTITUDE_M;
    }
    if (!data.altitude_valid) {
        rejectionReason |= GPS_REJECT_ALTITUDE;
    }
    
    if (gps.speed.isValid()) {
        data.speed_age_ms = gps.speed.age();
        data.speed_kmh = gps.speed.kmph();
        if (data.speed_age_ms > GPS_SPEED_MAX_AGE_MS) {
            rejectionReason |= GPS_REJECT_SPEED_AGE;
        } else if (
            data.speed_kmh < GPS_MIN_RELIABLE_SPEED_KMH ||
            data.speed_kmh > GPS_MAX_PLAUSIBLE_SPEED_KMH) {
            rejectionReason |= GPS_REJECT_SPEED_PLAUSIBILITY;
        } else {
            data.speed_valid = true;
        }
    } else {
        rejectionReason |= GPS_REJECT_SPEED_AGE;
    }
    
    if (gps.course.isValid()) {
        data.course_age_ms = gps.course.age();
        data.heading_deg = normalizeHeading(gps.course.deg());
        data.course_valid =
            data.course_age_ms <= GPS_COURSE_MAX_AGE_MS &&
            data.speed_valid;
    }
    if (!data.course_valid) {
        rejectionReason |= GPS_REJECT_COURSE;
    }

    if (hasValidDateTime()) {
        data.year = gps.date.year();
        data.month = gps.date.month();
        data.day = gps.date.day();
        data.hour = gps.time.hour();
        data.minute = gps.time.minute();
        data.second = gps.time.second();
        data.centisecond = gps.time.centisecond();
        data.datetime_valid = true;
    }
    
    data.valid_fix = data.location_valid;
    data.timestamp = now;
    data.fix_sequence = fixSequence;
    data.new_fix =
        fixSequence > 0 && fixSequence != lastDeliveredFixSequence;
    if (consumeNewFix) {
        lastDeliveredFixSequence = fixSequence;
    }
    data.nmea_chars = status.chars_processed;
    data.nmea_sentences_valid = status.sentences_received;
    data.nmea_checksum_failures = status.sentences_failed;
    data.rx_buffer_overflows = status.rx_buffer_overflows;
    data.rejection_reason = rejectionReason;
    data.rejected = rejectionReason != GPS_REJECT_NONE;
    
    // Gültige Daten für Backup speichern
    if (data.valid_fix) {
        lastValidData = data;
    }
    
    return data;
}

bool GPSManager::hasValidFix() const {
    if (!initialized ||
        !gps.location.isValid() ||
        !gps.satellites.isValid() ||
        !gps.hdop.isValid()) {
        return false;
    }
    TinyGPSPlus& current = const_cast<TinyGPSPlus&>(gps);
    return current.location.age() <= GPS_LOCATION_MAX_AGE_MS &&
           current.satellites.age() <= GPS_QUALITY_MAX_AGE_MS &&
           current.satellites.value() >= GPS_MIN_SATELLITES &&
           current.hdop.age() <= GPS_QUALITY_MAX_AGE_MS &&
           current.hdop.hdop() <= GPS_MAX_HDOP;
}

bool GPSManager::hasValidLocation() const {
    return hasValidFix();
}

bool GPSManager::hasValidSpeed() const {
    if (!initialized || !gps.speed.isValid()) {
        return false;
    }
    TinyGPSPlus& current = const_cast<TinyGPSPlus&>(gps);
    const float speedKmh = current.speed.kmph();
    return current.speed.age() <= GPS_SPEED_MAX_AGE_MS &&
           speedKmh >= GPS_MIN_RELIABLE_SPEED_KMH &&
           speedKmh <= GPS_MAX_PLAUSIBLE_SPEED_KMH;
}

bool GPSManager::hasValidDateTime() const {
    if (!initialized || !gps.date.isValid() || !gps.time.isValid()) {
        return false;
    }

    const uint16_t year = const_cast<TinyGPSPlus&>(gps).date.year();
    return year >= 2020 && year <= 2099 &&
           const_cast<TinyGPSPlus&>(gps).date.age() < 5000 &&
           const_cast<TinyGPSPlus&>(gps).time.age() < 5000;
}

bool GPSManager::isCommunicating(unsigned long maxAge) const {
    if (!initialized || !status.communicating || status.last_update == 0) {
        return false;
    }
    return millis() - status.last_update <= maxAge;
}

bool GPSManager::isReceivingNMEA(unsigned long maxAge) const {
    if (!initialized || lastCharacterUpdate == 0 ||
        lastNMEAStartUpdate == 0) {
        return false;
    }

    unsigned long now = millis();
    return now - lastCharacterUpdate <= maxAge &&
           now - lastNMEAStartUpdate <= maxAge;
}

bool GPSManager::syncSystemClock() {
    if (!hasValidDateTime()) {
        return false;
    }

    // mktime soll die GPS-Angaben unverändert als UTC interpretieren.
    setenv("TZ", "UTC0", 1);
    tzset();

    struct tm utcTime = {};
    utcTime.tm_year = gps.date.year() - 1900;
    utcTime.tm_mon = gps.date.month() - 1;
    utcTime.tm_mday = gps.date.day();
    utcTime.tm_hour = gps.time.hour();
    utcTime.tm_min = gps.time.minute();
    utcTime.tm_sec = gps.time.second();
    utcTime.tm_isdst = 0;

    time_t epoch = mktime(&utcTime);
    if (epoch <= 0) {
        return false;
    }

    struct timeval tv = {};
    tv.tv_sec = epoch;
    tv.tv_usec = gps.time.centisecond() * 10000L;
    return settimeofday(&tv, nullptr) == 0;
}

uint8_t GPSManager::getSatelliteCount() const {
    if (!initialized || !gps.satellites.isValid()) return 0;
    return const_cast<TinyGPSPlus&>(gps).satellites.value();
}

float GPSManager::getHDOP() const {
    if (!initialized || !gps.hdop.isValid()) return 99.9;
    return const_cast<TinyGPSPlus&>(gps).hdop.hdop();
}

String GPSManager::getLocationString() const {
    if (!hasValidLocation()) {
        return "Position unbekannt";
    }
    
    String lat = formatGPSCoordinate(const_cast<TinyGPSPlus&>(gps).location.lat(), true);
    String lon = formatGPSCoordinate(const_cast<TinyGPSPlus&>(gps).location.lng(), false);
    
    return lat + ", " + lon;
}

String GPSManager::getStatusString() const {
    if (!initialized) return "Nicht initialisiert";
    if (!status.communicating) return "Keine Kommunikation";
    if (!hasValidFix()) {
        static char searchBuffer[64];
        snprintf(searchBuffer, sizeof(searchBuffer), "Suche Satelliten (%d sichtbar)", getSatelliteCount());
        return String(searchBuffer);
    }
    
    static char fixBuffer[64];
    snprintf(fixBuffer, sizeof(fixBuffer), "Fix OK (%d Satelliten, HDOP: %.1f)", 
             getSatelliteCount(), getHDOP());
    return String(fixBuffer);
}

String GPSManager::getDiagnosticString() const {
    static char diagBuffer[1024];
    char* ptr = diagBuffer;
    size_t remaining = sizeof(diagBuffer);
    
    const uint32_t lastDataAge =
        status.last_update > 0 ? millis() - status.last_update : UINT32_MAX;
    int written = snprintf(ptr, remaining,
        "GPS-Diagnose:\n"
        "Initialisiert: %s\n"
        "Kommunikation: %s\n"
        "NMEA-Sätze: %lu OK, %lu Fehler\n"
        "Zeichen: %lu\n"
        "UART-Ringpuffer: %u/%u Bytes, Überläufe: %lu\n"
        "Letzte gültige NMEA-Daten: vor %lu ms\n",
        initialized ? "Ja" : "Nein",
        status.communicating ? "Aktiv" : "Inaktiv",
        status.sentences_received,
        status.sentences_failed,
        status.chars_processed,
        status.buffered_bytes,
        static_cast<unsigned>(RX_BUFFER_SIZE),
        status.rx_buffer_overflows,
        static_cast<unsigned long>(lastDataAge)
    );
    
    if (written > 0 && written < (int)remaining) {
        ptr += written;
        remaining -= written;
        
        GPSData data = const_cast<GPSManager*>(this)->getCurrentData();
        snprintf(
            ptr, remaining,
            "Fix: %s, Sequenz: %lu, neuer Fix: %s\n"
            "Position: %.6f/%.6f, gültig=%d, Alter=%lu ms\n"
            "Geschwindigkeit: %.1f km/h, gültig=%d, Alter=%lu ms\n"
            "Höhe: %.1f m, gültig=%d, Alter=%lu ms\n"
            "Kurs: %.1f Grad, gültig=%d, Alter=%lu ms\n"
            "Satelliten: %d, gültig=%d, Alter=%lu ms\n"
            "HDOP: %.1f, gültig=%d, Alter=%lu ms",
            data.valid_fix ? "gültig" : "ungültig",
            static_cast<unsigned long>(data.fix_sequence),
            data.new_fix ? "ja" : "nein",
            data.latitude, data.longitude,
            data.location_valid ? 1 : 0,
            static_cast<unsigned long>(data.location_age_ms),
            data.speed_kmh, data.speed_valid ? 1 : 0,
            static_cast<unsigned long>(data.speed_age_ms),
            data.altitude, data.altitude_valid ? 1 : 0,
            static_cast<unsigned long>(data.altitude_age_ms),
            data.heading_deg, data.course_valid ? 1 : 0,
            static_cast<unsigned long>(data.course_age_ms),
            data.satellites, data.satellites_valid ? 1 : 0,
            static_cast<unsigned long>(data.satellites_age_ms),
            data.hdop, data.hdop_valid ? 1 : 0,
            static_cast<unsigned long>(data.hdop_age_ms));
    }
    
    return String(diagBuffer);
}

bool GPSManager::testCommunication() {
    if (!serial) return false;
    
    Serial.print("Teste GPS-Kommunikation auf UART2... ");
    
    uint32_t startTime = millis();
    uint32_t charCount = 0;
    bool nmeaStartSeen = false;
    
    // 3 Sekunden auf Daten warten
    while (millis() - startTime < 3000) {
        if (serial->available()) {
            char c = serial->read();
            charCount++;

            if (c == '$') {
                nmeaStartSeen = true;
            }

            // Testdaten nicht verwerfen, sondern bereits an TinyGPS++ übergeben.
            if (gps.encode(c)) {
                status.sentences_received++;
                lastDataUpdate = millis();
                status.communicating = true;
                status.last_update = lastDataUpdate;
                Serial.println("OK");
                Serial.printf("NMEA-Satz empfangen (%lu Zeichen in %lu ms)\n",
                             charCount, millis() - startTime);
                return true;
            }
        }
        delay(10);
    }
    
    if (charCount > 0) {
        Serial.printf("Zeichen empfangen (%lu), NMEA-Start: %s\n",
                      charCount, nmeaStartSeen ? "ja" : "nein");
    } else {
        Serial.println("Keine Daten");
        Serial.println("Prüfe: TX/RX-Pins, Baudrate, GPS-Stromversorgung");
    }
    
    return false;
}

void GPSManager::printRawData() {
    if (!serial) {
        Serial.println("GPS nicht initialisiert");
        return;
    }
    
    Serial.println("=== GPS Raw Data (10 Sekunden) ===");
    uint32_t startTime = millis();
    
    while (millis() - startTime < 10000) {
        if (serial->available()) {
            Serial.write(serial->read());
        }
    }
    Serial.println("\n=== Ende Raw Data ===");
}

void GPSManager::printDiagnostics() {
    Serial.println("\n=== GPS-Diagnostics ===");
    Serial.printf("Modus: %s\n", interruptEnabled ? "Interrupt-basiert" : "Polling");
    
    if (interruptEnabled) {
        Serial.printf("Buffer-Auslastung: %zu/%zu Bytes\n", 
                     (rxIndex >= rxProcessIndex) ? (rxIndex - rxProcessIndex) : 
                     (RX_BUFFER_SIZE - rxProcessIndex + rxIndex), RX_BUFFER_SIZE);
        Serial.printf("Daten bereit: %s\n", dataReady ? "Ja" : "Nein");
    }
    
    Serial.println(getDiagnosticString());
    
    if (hasValidFix()) {
        GPSData data = getCurrentData();
        Serial.printf("Aktuelle Position: %.6f°N, %.6f°E\n", data.latitude, data.longitude);
        Serial.printf("Höhe: %.1fm, Geschwindigkeit: %.1f km/h\n", data.altitude, data.speed_kmh);
        Serial.printf("Kurs: %.1f°, HDOP: %.1f\n", data.heading_deg, data.hdop);
    }
    
    Serial.println("=========================");
}

// Private Hilfsfunktionen

void GPSManager::updateStatus() {
    status.chars_processed = gps.charsProcessed();
    status.sentences_received = gps.passedChecksum();
    status.sentences_failed = gps.failedChecksum();

    noInterrupts();
    const size_t writeIndex = rxIndex;
    const size_t readIndex = rxProcessIndex;
    status.rx_buffer_overflows = rxBufferOverflows;
    interrupts();
    status.buffered_bytes = static_cast<uint16_t>(
        writeIndex >= readIndex
            ? writeIndex - readIndex
            : RX_BUFFER_SIZE - readIndex + writeIndex);

    if (gps.location.isValid()) {
        const uint32_t now = millis();
        const uint32_t locationCommitMs =
            now - gps.location.age();
        if (!hasObservedLocationCommit ||
            locationCommitMs != lastLocationCommitMs) {
            hasObservedLocationCommit = true;
            lastLocationCommitMs = locationCommitMs;
            fixSequence++;
        }
    }

    // Kommunikations-Status basierend auf letzten Daten
    if (millis() - status.last_update > 10000) {
        status.communicating = false;
    }
}

bool GPSManager::isDataFresh(unsigned long maxAge) {
    if (!initialized) return false;
    return (millis() - lastDataUpdate) < maxAge;
}

float GPSManager::normalizeHeading(float heading) {
    while (heading < 0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;
    return heading;
}

// Utility-Funktionen

String formatGPSCoordinate(float coord, bool isLatitude) {
    if (coord == 0.0) return "0.000000°";
    
    char direction = isLatitude ? (coord > 0 ? 'N' : 'S') : (coord > 0 ? 'E' : 'W');
    
    return String(fabs(coord), 6) + "°" + direction;
}

String formatGPSData(const GPSData& data) {
    if (!data.valid_fix) {
        return "GPS: Kein Fix (" + String(data.satellites) + " Satelliten)";
    }
    
    String result =
        "GPS: " + formatGPSCoordinate(data.latitude, true) + ", " +
        formatGPSCoordinate(data.longitude, false) + " | ";
    result += data.speed_valid
        ? String(data.speed_kmh, 1) + " km/h"
        : String("Geschwindigkeit unbestätigt");
    result += " | " + String(data.satellites) + " Sat";
    return result;
}

String formatGPSDateTimeUTC(const GPSData& data) {
    if (!data.datetime_valid) {
        return "";
    }

    char buffer[32];
    snprintf(buffer, sizeof(buffer),
             "%04u-%02u-%02uT%02u:%02u:%02u.%02uZ",
             data.year, data.month, data.day,
             data.hour, data.minute, data.second, data.centisecond);
    return String(buffer);
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    // Haversine-Formel für Distanz zwischen zwei GPS-Punkten
    const float R = 6371000; // Erdradius in Metern
    
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(radians(lat1)) * cos(radians(lat2)) * 
              sin(dLon/2) * sin(dLon/2);
              
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c; // Distanz in Metern
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    // Bearing zwischen zwei GPS-Punkten
    float dLon = radians(lon2 - lon1);
    float y = sin(dLon) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) -
              sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    
    float bearing = degrees(atan2(y, x));
    
    // Normalisierung auf 0-360°
    while (bearing < 0) bearing += 360.0;
    while (bearing >= 360.0) bearing -= 360.0;
    
    return bearing;
}

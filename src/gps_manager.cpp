#include "gps_manager.h"
#include <math.h>

// Statische Member-Variablen
GPSManager* GPSManager::instance = nullptr;

// Globale GPS-Manager Instanz
GPSManager gpsManager;

GPSManager::GPSManager() : 
    serial(nullptr), initialized(false), rxPin(16), txPin(15), 
    baudRate(9600), lastDataUpdate(0), dataReady(false), 
    interruptEnabled(false), rxIndex(0), rxProcessIndex(0) {
    
    // Status initialisieren
    status = {false, false, 0, 0, 0, 0};
    
    // Leere GPS-Daten
    lastValidData = {0.0, 0.0, 0.0, 0.0, 0.0, 0, false, 0, 0.0, false, false, false};
    
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
    
    Serial.println("=== GPS-Manager Initialisierung ===");
    Serial.printf("UART2: RX=%d, TX=%d, Baud=%lu\n", rxPin, txPin, baudRate);
    
    // Hardware UART2 initialisieren
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    serial = &Serial2;
    
    delay(100);
    
    // Kommunikations-Test
    if (!testCommunication()) {
        Serial.println("⚠️ GPS-Kommunikation noch nicht aktiv (normal bei Cold Start)");
        // Trotzdem als initialisiert markieren - GPS braucht Zeit für Fix
    }
    
    status.initialized = true;
    initialized = true;
    
    Serial.println("✅ GPS-Manager erfolgreich initialisiert");
    Serial.println("GPS benötigt 15-30 Sekunden für ersten Fix");
    
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
        }
        // Bei Überlauf wird das älteste Byte überschrieben
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

GPSData GPSManager::getCurrentData() {
    GPSData data = lastValidData;
    
    if (!initialized || !isDataFresh()) {
        return data;
    }
    
    // Aktuelle GPS-Daten von TinyGPS++ holen
    if (gps.location.isValid()) {
        data.latitude = gps.location.lat();
        data.longitude = gps.location.lng();
        data.location_valid = true;
    }
    
    if (gps.altitude.isValid()) {
        data.altitude = gps.altitude.meters();
    }
    
    if (gps.speed.isValid()) {
        data.speed_kmh = gps.speed.kmph();
        data.speed_valid = true;
    }
    
    if (gps.course.isValid()) {
        data.heading_deg = normalizeHeading(gps.course.deg());
        data.course_valid = true;
    }
    
    if (gps.satellites.isValid()) {
        data.satellites = gps.satellites.value();
    }
    
    if (gps.hdop.isValid()) {
        data.hdop = gps.hdop.hdop();
    }
    
    data.valid_fix = hasValidFix();
    data.timestamp = millis();
    
    // Gültige Daten für Backup speichern
    if (data.valid_fix) {
        lastValidData = data;
    }
    
    return data;
}

bool GPSManager::hasValidFix() const {
    return initialized && gps.location.isValid() && gps.location.age() < 5000;
}

bool GPSManager::hasValidLocation() const {
    return initialized && gps.location.isValid();
}

bool GPSManager::hasValidSpeed() const {
    return initialized && gps.speed.isValid();
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
    static char diagBuffer[512];
    char* ptr = diagBuffer;
    size_t remaining = sizeof(diagBuffer);
    
    int written = snprintf(ptr, remaining,
        "GPS-Diagnose:\n"
        "Initialisiert: %s\n"
        "Kommunikation: %s\n"
        "NMEA-Sätze: %lu OK, %lu Fehler\n"
        "Zeichen: %lu\n"
        "Letzte Daten: vor %lus\n",
        initialized ? "Ja" : "Nein",
        status.communicating ? "Aktiv" : "Inaktiv",
        status.sentences_received,
        status.sentences_failed,
        status.chars_processed,
        (millis() - status.last_update) / 1000
    );
    
    if (written > 0 && written < (int)remaining) {
        ptr += written;
        remaining -= written;
        
        if (hasValidFix()) {
            GPSData data = const_cast<GPSManager*>(this)->getCurrentData();
            snprintf(ptr, remaining,
                "Position: %.6f°N, %.6f°E\n"
                "Geschwindigkeit: %.1f km/h\n"
                "Satelliten: %d, HDOP: %.1f",
                data.latitude, data.longitude,
                data.speed_kmh,
                data.satellites, data.hdop
            );
        }
    }
    
    return String(diagBuffer);
}

bool GPSManager::testCommunication() {
    if (!serial) return false;
    
    Serial.print("Teste GPS-Kommunikation auf UART2... ");
    
    uint32_t startTime = millis();
    uint32_t charCount = 0;
    
    // 3 Sekunden auf Daten warten
    while (millis() - startTime < 3000) {
        if (serial->available()) {
            char c = serial->read();
            charCount++;
            
            if (c == '$') {
                // NMEA-Satz-Start gefunden
                Serial.println("OK");
                Serial.printf("NMEA-Daten empfangen (%lu Zeichen in %lu ms)\n", 
                             charCount, millis() - startTime);
                return true;
            }
        }
        delay(10);
    }
    
    if (charCount > 0) {
        Serial.printf("Zeichen empfangen (%lu), aber kein NMEA-Format\n", charCount);
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
    // Kommunikations-Status basierend auf letzten Daten
    if (millis() - status.last_update > 10000) {
        status.communicating = false;
    }
    
    // Fehlgeschlagene Checksums zählen
    status.sentences_failed = gps.failedChecksum();
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
    
    return "GPS: " + 
           formatGPSCoordinate(data.latitude, true) + ", " +
           formatGPSCoordinate(data.longitude, false) + 
           " | " + String(data.speed_kmh, 1) + " km/h" +
           " | " + String(data.satellites) + " Sat";
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
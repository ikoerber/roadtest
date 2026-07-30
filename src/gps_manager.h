#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

enum GPSRejectionReason : uint8_t {
    GPS_REJECT_NONE = 0x00,
    GPS_REJECT_LOCATION_AGE = 0x01,
    GPS_REJECT_SATELLITE_QUALITY = 0x02,
    GPS_REJECT_HDOP_QUALITY = 0x04,
    GPS_REJECT_SPEED_AGE = 0x08,
    GPS_REJECT_SPEED_PLAUSIBILITY = 0x10,
    GPS_REJECT_ALTITUDE = 0x20,
    GPS_REJECT_COURSE = 0x40
};

// GPS-Datenstruktur
struct GPSData {
    float latitude = 0;          // Breitengrad (Dezimalgrad)
    float longitude = 0;         // Längengrad (Dezimalgrad)
    float altitude = 0;          // Höhe über Meeresspiegel (m)
    float speed_kmh = 0;         // Geschwindigkeit (km/h)
    float heading_deg = 0;       // Fahrtrichtung (0-360°, nur bei Bewegung gültig)
    uint8_t satellites = 0;      // Anzahl Satelliten
    bool valid_fix = false;      // Gültiger GPS-Fix
    unsigned long timestamp = 0; // Zeitstempel (ms)
    
    // Zusätzliche Genauigkeitsinformationen
    float hdop = 0;              // Horizontal Dilution of Precision
    bool location_valid = false; // Position gültig
    bool speed_valid = false;    // Geschwindigkeit gültig
    bool altitude_valid = false;
    bool course_valid = false;   // Fahrtrichtung gültig
    bool satellites_valid = false;
    bool hdop_valid = false;
    uint32_t location_age_ms = UINT32_MAX;
    uint32_t speed_age_ms = UINT32_MAX;
    uint32_t altitude_age_ms = UINT32_MAX;
    uint32_t course_age_ms = UINT32_MAX;
    uint32_t satellites_age_ms = UINT32_MAX;
    uint32_t hdop_age_ms = UINT32_MAX;
    bool new_fix = false;
    uint32_t fix_sequence = 0;
    uint32_t nmea_chars = 0;
    uint32_t nmea_sentences_valid = 0;
    uint32_t nmea_checksum_failures = 0;
    uint32_t rx_buffer_overflows = 0;
    bool rejected = false;
    // Bitmaske aus GPSRejectionReason. Der Datensatz bleibt als
    // Qualitäts-Snapshot erhalten; einzelne Felder werden über ihre
    // Valid-Flags für Auswertungen gesperrt.
    uint8_t rejection_reason = GPS_REJECT_NONE;

    // Datum und Uhrzeit direkt aus dem GPS (immer UTC)
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t centisecond = 0;
    bool datetime_valid = false;
};

// GPS-Status für Hardware-Tests
struct GPSStatus {
    bool initialized = false;
    bool communicating = false;
    uint32_t sentences_received = 0;
    uint32_t sentences_failed = 0;
    uint32_t chars_processed = 0;
    uint32_t rx_buffer_overflows = 0;
    uint16_t buffered_bytes = 0;
    unsigned long last_update = 0;
};

class GPSManager {
private:
    TinyGPSPlus gps;
    HardwareSerial* serial;
    bool initialized;
    
    // Hardware-Konfiguration
    int rxPin, txPin;
    uint32_t baudRate;
    
    // Status-Tracking
    GPSStatus status;
    GPSData lastValidData;
    unsigned long lastDataUpdate;
    unsigned long lastCharacterUpdate;
    unsigned long lastNMEAStartUpdate;
    
    // Interrupt-Support
    static GPSManager* instance;  // Für statische Interrupt-Handler
    volatile bool dataReady;
    volatile bool interruptEnabled;
    static const size_t UART_RX_BUFFER_SIZE = 2048;
    static const size_t RX_BUFFER_SIZE = 4096;
    volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
    volatile size_t rxIndex;
    volatile size_t rxProcessIndex;
    volatile uint32_t rxBufferOverflows;
    bool hasObservedFixEpoch;
    uint64_t lastFixEpochKey;
    uint32_t fixSequence;
    uint32_t lastDeliveredFixSequence;
    
    // Interrupt-Handler
    static void IRAM_ATTR onReceiveInterrupt();
    void processInterruptData();
    
    // Hilfsfunktionen
    void updateStatus();
    bool isDataFresh(unsigned long maxAge = 5000);
    float normalizeHeading(float heading);
    
public:
    GPSManager();
    ~GPSManager();
    
    // Initialisierung
    bool begin(int rx = 16, int tx = 15, uint32_t baud = 9600);
    void end();
    bool isReady() const { return initialized; }
    
    // Interrupt-Modus
    void enableInterruptMode(bool enable = true);
    bool isInterruptModeEnabled() const { return interruptEnabled; }
    
    // Daten-Updates
    void update();  // Für Polling oder Interrupt-Verarbeitung
    bool available();
    GPSData getCurrentData(bool consumeNewFix = false);
    GPSStatus getStatus() const { return status; }
    uint32_t getFixSequence() const { return fixSequence; }
    
    // GPS-Status-Abfragen
    bool hasValidFix() const;
    bool hasValidLocation() const;
    bool hasValidSpeed() const;
    bool hasValidDateTime() const;
    bool isCommunicating(unsigned long maxAge = 10000) const;
    bool isReceivingNMEA(unsigned long maxAge = 10000) const;
    bool syncSystemClock();
    uint8_t getSatelliteCount() const;
    float getHDOP() const;
    
    // Utility-Funktionen
    String getLocationString() const;
    String getStatusString() const;
    String getDiagnosticString() const;
    
    // Test-Funktionen
    bool testCommunication();
    void printRawData();
    void printDiagnostics();
};

// Globale GPS-Manager Instanz
extern GPSManager gpsManager;

// Utility-Funktionen
String formatGPSCoordinate(float coord, bool isLatitude);
String formatGPSData(const GPSData& data);
String formatGPSDateTimeUTC(const GPSData& data);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

#endif // GPS_MANAGER_H

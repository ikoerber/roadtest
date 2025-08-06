#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// GPS-Datenstruktur
struct GPSData {
    float latitude;          // Breitengrad (Dezimalgrad)
    float longitude;         // Längengrad (Dezimalgrad)  
    float altitude;          // Höhe über Meeresspiegel (m)
    float speed_kmh;         // Geschwindigkeit (km/h)
    float heading_deg;       // Fahrtrichtung (0-360°, nur bei Bewegung gültig)
    uint8_t satellites;      // Anzahl Satelliten
    bool valid_fix;          // Gültiger GPS-Fix
    unsigned long timestamp; // Zeitstempel (ms)
    
    // Zusätzliche Genauigkeitsinformationen
    float hdop;              // Horizontal Dilution of Precision
    bool location_valid;     // Position gültig
    bool speed_valid;        // Geschwindigkeit gültig
    bool course_valid;       // Fahrtrichtung gültig
};

// GPS-Status für Hardware-Tests
struct GPSStatus {
    bool initialized;
    bool communicating;
    uint32_t sentences_received;
    uint32_t sentences_failed;
    uint32_t chars_processed;
    unsigned long last_update;
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
    
    // Daten-Updates
    void update();
    bool available();
    GPSData getCurrentData();
    GPSStatus getStatus() const { return status; }
    
    // GPS-Status-Abfragen
    bool hasValidFix() const;
    bool hasValidLocation() const;
    bool hasValidSpeed() const;
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
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

#endif // GPS_MANAGER_H
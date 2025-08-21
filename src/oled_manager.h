#ifndef OLED_MANAGER_H
#define OLED_MANAGER_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Konfiguration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Reset pin nicht verwendet

// Display-Modi
enum DisplayMode {
    DISPLAY_MODE_STATUS,        // Hardware-Status
    DISPLAY_MODE_SENSOR_DATA,   // Live Sensor-Daten
    DISPLAY_MODE_GPS_STATUS,    // GPS-Status
    DISPLAY_MODE_ROAD_QUALITY,  // Straßenqualitäts-Anzeige
    DISPLAY_MODE_TEST_RESULT    // Test-Ergebnisse
};

// Display-Konfiguration
struct DisplayConfig {
    bool autoRotate;            // Automatischer Display-Wechsel
    uint32_t rotateInterval;    // Wechsel-Intervall in ms
    uint8_t brightness;         // Helligkeit (falls unterstützt)
    bool showDebugInfo;         // Debug-Informationen anzeigen
};

class OLEDManager {
private:
    Adafruit_SSD1306* display;
    bool initialized;
    
    // Display-Management
    DisplayMode currentMode;
    DisplayConfig config;
    unsigned long lastModeSwitch;
    unsigned long lastUpdate;
    
    // I2C-Konfiguration
    uint8_t i2cAddress;
    bool addressFound;
    
    // Display-Rotation
    bool rotated180;
    
    // Hilfsfunktionen
    void clearAndSetup();
    void drawHeader(const String& title);
    void drawStatusIndicator(const String& label, bool status, int line);
    void drawProgressBar(int x, int y, int width, int height, float progress);
    String formatFloat(float value, int decimals = 1);
    
public:
    OLEDManager();
    ~OLEDManager();
    
    // Initialisierung
    bool begin(uint8_t address = 0x3C);
    void end();
    bool isReady() const { return initialized; }
    
    // Konfiguration
    void setConfig(const DisplayConfig& cfg) { config = cfg; }
    DisplayConfig getConfig() const { return config; }
    void setMode(DisplayMode mode);
    DisplayMode getCurrentMode() const { return currentMode; }
    
    // Display-Funktionen
    void update(); // Automatisches Update basierend auf Modus
    void clear();
    void refresh();
    
    // Status-Anzeigen
    void showHardwareStatus(bool i2c, bool bno055, bool oled, bool sd, bool can, bool gps = false);
    void showTestResults(const String& testName, bool success, const String& details = "");
    
    // Live-Daten Anzeigen
    void showSensorData(float heading, float accel, float temp, int canCount);
    void showGPSStatus(float lat, float lon, float speed, int satellites, bool fix);
    void showRoadQuality(float quality, float smoothness, float curviness, int curves);
    
    // System-Informationen
    void showSystemInfo(const String& version, unsigned long uptime, uint32_t freeHeap);
    void showNetworkInfo(const String& ssid, const String& ip, int rssi);
    
    // Erweiterte Anzeigen
    void showBootMessage(const String& message, int progress = -1);
    void showErrorMessage(const String& error, const String& details = "");
    void showDebugInfo(const String& info);
    
    // Utility-Funktionen
    void setBrightness(uint8_t brightness);
    void setContrast(uint8_t contrast);
    void setRotation(bool rotate180);
    bool testDisplay();
    String getDisplayInfo();
    
    // Statische Utility-Funktionen
    static bool scanI2CAddresses();
    static String getI2CDeviceInfo(uint8_t address);
};

// Globale OLED-Manager Instanz
extern OLEDManager oledManager;

// Legacy-Kompatibilität (deprecated)
bool initOLED() __attribute__((deprecated("Use oledManager.begin() instead")));
void displayTestResults(const char* testName, bool success, const char* details = nullptr) __attribute__((deprecated("Use oledManager.showTestResults() instead")));
void displayHardwareStatus(bool i2c, bool bno055, bool oled, bool sd, bool can, bool gps = false) __attribute__((deprecated("Use oledManager.showHardwareStatus() instead")));
void displaySensorData(float heading, float accel, float temp, int canCount) __attribute__((deprecated("Use oledManager.showSensorData() instead")));
void displayGPSStatus(float lat, float lon, float speed, int satellites, bool fix) __attribute__((deprecated("Use oledManager.showGPSStatus() instead")));

#endif // OLED_MANAGER_H
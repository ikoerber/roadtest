#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// =============================================================================
// ESP32-S3 Hardware Pin-Definitionen
// =============================================================================
// Zentrale Pin-Konfiguration für das Straßenqualitäts-Messsystem
// Alle Hardware-spezifischen Pin-Zuordnungen sind hier definiert

// -----------------------------------------------------------------------------
// I2C-Bus Konfiguration 
// -----------------------------------------------------------------------------
extern const int I2C_SDA;          // GPIO 8  - I2C Data Line
extern const int I2C_SCL;          // GPIO 9  - I2C Clock Line

// I2C-Geräte-Adressen (BNO055_ADDRESS_* bereits in Adafruit_BNO055.h definiert)
#define OLED_ADDRESS_A      0x3C    // OLED Standard-Adresse
#define OLED_ADDRESS_B      0x3D    // OLED Alternative-Adresse

// -----------------------------------------------------------------------------
// SPI-Bus Konfiguration
// -----------------------------------------------------------------------------

// SD-Karte (HSPI)
extern const int SD_CS_PIN;        // GPIO 4  - SD Card Chip Select
extern const int SD_MOSI_PIN;      // GPIO 5  - SD Card Master Out Slave In
extern const int SD_MISO_PIN;      // GPIO 6  - SD Card Master In Slave Out  
extern const int SD_SCK_PIN;       // GPIO 7  - SD Card Serial Clock

// CAN-Bus (VSPI mit MCP2515)
extern const int CAN_CS_PIN;       // GPIO 1  - CAN Controller Chip Select
extern const int CAN_INT_PIN;      // GPIO 2  - CAN Controller Interrupt
extern const int CAN_SCK_PIN;      // GPIO 3  - CAN Controller Serial Clock
extern const int CAN_MOSI_PIN;     // GPIO 13 - CAN Controller Master Out Slave In
extern const int CAN_MISO_PIN;     // GPIO 11 - CAN Controller Master In Slave Out

// -----------------------------------------------------------------------------
// UART-Konfiguration (GPS)
// -----------------------------------------------------------------------------
extern const int GPS_RX_PIN;       // GPIO 16 - GPS UART Receive
extern const int GPS_TX_PIN;       // GPIO 15 - GPS UART Transmit
extern const int GPS_BAUD_RATE;    // 9600    - GPS Baudrate

// -----------------------------------------------------------------------------
// Hardware-Spezifikationen
// -----------------------------------------------------------------------------

// Timing-Konstanten
#define I2C_CLOCK_SPEED     100000  // 100kHz I2C-Bus
#define SD_SPI_SPEED        400000  // 400kHz für SD-Initialisierung
#define CAN_BAUDRATE        500000  // 500kbps CAN-Bus
#define CAN_CLOCK_8MHZ      8000000 // 8MHz MCP2515 Oszillator
#define CAN_CLOCK_16MHZ     16000000// 16MHz MCP2515 Oszillator

// Display-Spezifikationen
#define SCREEN_WIDTH        128     // OLED Display Breite
#define SCREEN_HEIGHT       64      // OLED Display Höhe
#define OLED_RESET          -1      // OLED Reset Pin (nicht verwendet)

// Sensor-Konfiguration  
#define VIBRATION_THRESHOLD 2.0     // Vibrations-Schwellwert (m/s²)
#define CURVE_THRESHOLD     5.0     // Kurven-Erkennungs-Schwellwert (°)

// System-Timing
#define SENSOR_READ_INTERVAL    100 // BNO055 Leseintervall (ms)
#define GPS_UPDATE_INTERVAL     200 // GPS Update-Intervall (ms) 
#define CAN_CHECK_INTERVAL      10  // CAN-Check Intervall (ms)
#define STATUS_REPORT_INTERVAL  5000// Status-Report Intervall (ms)

// Buffer-Sicherheit
#define MAX_LOG_LINE_LENGTH     256 // Maximale Länge einer Log-Zeile
#define BUFFER_SAFETY_FACTOR    0.8 // 80% Buffer-Auslastung vor Auto-Flush
#define MAX_STRING_LENGTH       512 // Maximale String-Länge für Sicherheit

// -----------------------------------------------------------------------------
// Hardware-Test Konfiguration
// -----------------------------------------------------------------------------

// I2C-Scanner Bereich
#define I2C_SCAN_START      0x08    // Erste zu scannende I2C-Adresse
#define I2C_SCAN_END        0x77    // Letzte zu scannende I2C-Adresse

// SD-Karten Test-Pin-Sets (CS, MOSI, MISO, SCK)
#define SD_PINSET_PRIMARY   {4, 5, 6, 7}       // Primäres Pin-Set
#define SD_PINSET_BACKUP1   {10, 11, 13, 12}   // Backup Pin-Set 1
#define SD_PINSET_BACKUP2   {15, 16, 17, 18}   // Backup Pin-Set 2

// Test-Timeouts
#define GPS_TEST_TIMEOUT    30000   // GPS Test-Timeout (ms)
#define CAN_TEST_TIMEOUT    5000    // CAN Test-Timeout (ms)
#define I2C_TEST_TIMEOUT    1000    // I2C Test-Timeout (ms)

#endif // HARDWARE_CONFIG_H
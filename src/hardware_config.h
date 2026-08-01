#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// =============================================================================
// ESP32-S3 Hardware Pin-Definitionen
// =============================================================================
// Zentrale Pin-Konfiguration für das Straßenqualitäts-Messsystem
// Alle Hardware-spezifischen Pin-Zuordnungen sind hier definiert

#define ROADTEST_FIRMWARE_VERSION "1.5.31"
#define ROADTEST_FIRMWARE_FILE_NAME \
    "roadtest_" ROADTEST_FIRMWARE_VERSION ".bin"
#define ROADTEST_CSV_SCHEMA_VERSION "1.5.28-quality-v10"
#define ROADTEST_VEHICLE_PROFILE "Porsche Carrera S 2012 PDK"

// -----------------------------------------------------------------------------
// I2C-Bus Konfiguration 
// -----------------------------------------------------------------------------
extern const int I2C_SDA;          // GPIO 8  - I2C Data Line
extern const int I2C_SCL;          // GPIO 9  - I2C Clock Line

// I2C-Geräte-Adressen (BNO055_ADDRESS_* bereits in Adafruit_BNO055.h definiert)
#define OLED_ADDRESS_A      0x3C    // OLED Standard-Adresse
#define OLED_ADDRESS_B      0x3D    // OLED Alternative-Adresse
#define OLED_REQUIRED       false   // Steckbares Display darf für Tests fehlen

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
//
// I²C: Der SSD1306-Treiber stellt den Bustakt bei jedem Transfer selbst ein
// (TRANSACTION_START/END in Adafruit_SSD1306.cpp) und lässt ihn danach auf
// seinem eigenen Default stehen. Ein Wire.setClock() in setup() wäre deshalb
// bereits nach dem ersten Bildaufbau wirkungslos. Beide Werte werden daher
// explizit an den Displaykonstruktor übergeben (oled_manager.cpp), erst damit
// steuern die Konstanten hier den Bus tatsächlich.
// Gemessen wurden 2,54 kOhm Pull-up. Eine Logic-8-Aufnahme zeigte, dass leere
// Schreibtransaktionen zur BNO055-Erkennung den folgenden Adresszugriff als
// Datenbyte an den Sensor anhängen konnten. Dadurch entstand SYS_ERR 5. Die
// Firmware liest deshalb zur Erkennung die Chip-ID und begrenzt außerdem jede
// I²C-Busoperation zeitlich.
//
// Beide Taktwerte bleiben gleich. Die gemessenen 100 kHz laufen mit BNO055 und
// OLED stabil; der Bildaufbau dauert bei einem Aktualisierungsintervall von
// fünf Sekunden nur unwesentlich länger.
#define I2C_CLOCK_SPEED     100000  // Basistakt für BNO055 zwischen den Transfers
#define I2C_DISPLAY_SPEED   100000  // Takt während der SSD1306-Übertragungen
#define I2C_TIMEOUT_MS          20  // Einzelne Busoperation spätestens abbrechen

// SD: Die Bibliothek verwendet ohne Argument 4 MHz. Auf Lochraster ohne
// Serienterminierung ist das die Schwelle, ab der sporadische Aussetzer
// beginnen. Bei 10 Hz Logging kostet 1 MHz keine nutzbare Bandbreite.
#define SD_SPI_SPEED       1000000  // Betriebstakt der SD-Karte
#define SD_MAX_OPEN_FILES       12  // 9 Logs plus Root-/Summary-Reserve

#define CAN_BAUDRATE        500000  // 500kbps CAN-Bus
#define CAN_CLOCK_8MHZ      8000000 // 8MHz MCP2515 Oszillator
#define CAN_CLOCK_16MHZ     16000000// 16MHz MCP2515 Oszillator
#define CAN_OBD_POLLING_ENABLED true // Nur Standard-OBD Service 01 aktiv abfragen
#define CAN_LISTEN_ONLY        false // Aktive OBD-Anfragen benötigen den Normalmodus
#define CAN_OBD_REQUEST_INTERVAL_MS 500 // Insgesamt höchstens zwei Anfragen pro Sekunde
#define CAN_OBD_RESPONSE_TIMEOUT_MS 400 // Antwort vor der nächsten Anfrage abschließen
#define CAN_OBD_VALUE_MAX_AGE_MS   5000 // Alte Livewerte nicht weiter anzeigen
#define CAN_OBD_SLOW_VALUE_MAX_AGE_MS 30000 // Langsame Temperatur-/Verbrauchswerte
#define CAN_TX_TIMEOUT_MS         25 // Fehlendes ACK darf die Hauptschleife nicht blockieren

// GPS-Feldqualität. Position und abgeleitete Werte dürfen nicht allein auf
// dem globalen TinyGPS++-Gültigkeitsbit beruhen, weil die NMEA-Felder
// unabhängig voneinander aktualisiert werden.
#define GPS_LOCATION_MAX_AGE_MS       1500
#define GPS_SPEED_MAX_AGE_MS          1500
#define GPS_ALTITUDE_MAX_AGE_MS       3000
#define GPS_COURSE_MAX_AGE_MS         1500
#define GPS_QUALITY_MAX_AGE_MS        3000
#define GPS_MIN_SATELLITES               5
#define GPS_MAX_HDOP                   3.5f
#define GPS_MIN_RELIABLE_SPEED_KMH     6.0f
#define GPS_MAX_PLAUSIBLE_SPEED_KMH  300.0f
#define GPS_MIN_ALTITUDE_M           -500.0f
#define GPS_MAX_ALTITUDE_M          10000.0f

// Streckenbildung: Eine frische OBD-Geschwindigkeit hat im Fahrzeug Vorrang
// vor GPS-Rauschen. Ohne OBD wird nur eine bereits als zuverlässig gefilterte
// GPS-Geschwindigkeit als Bewegungsnachweis akzeptiert.
#define GPS_DISTANCE_OBD_STATIONARY_KMH  1.0f
#define GPS_DISTANCE_MIN_SEGMENT_OBD_M   0.5f
#define GPS_DISTANCE_MIN_SEGMENT_GPS_M   3.0f

// Serielle Discovery-/Recovery-Zustandsmaschine. Der bisherige Webablauf
// wurde entfernt, die Mindestzeit bleibt für bestehende serielle Tests
// verbindlich.
#define ACCEPTANCE_INITIAL_STAND_MS         60000UL

// Laufzeitdiagnose: 250 ms verfehlen bereits zwei 10-Hz-Sensorzeitpunkte und
// mindestens einen 5-Hz-GPS-Snapshot. Verlorene Slots werden zusätzlich
// unabhängig von dieser Schwelle gezählt.
#define RUNTIME_STALL_THRESHOLD_MS            250UL

// Straßenqualität und Fahrbahnereignisse benötigen nachgewiesene
// Fahrzeugbewegung. Ohne belegte Geschwindigkeit entsteht kein Messwert und
// kein Ereignis; im Stand erzeugte Leerlaufvibration ist keine Straßenlage.
// Der Sitzungslauf 20260730_071913_9B018397 protokollierte im belegten
// Stillstand drei Schlaglöcher und eine Kurve mit 625 Grad.
#define ROAD_EVENT_MIN_SPEED_KMH               5.0f

// Schlaglocherkennung: Ein Ereignis besteht aus einem Ausschlag nach unten,
// gefolgt von einem Ausschlag nach oben innerhalb des Erkennungsfensters.
// Die Sperrzeit verhindert, dass ein Nachschwingen als zweites Schlagloch
// gezählt wird. Werte bisher als Zahlen im Erkennungscode.
#define ROAD_POTHOLE_WINDOW_MS               600UL
#define ROAD_POTHOLE_REARM_MS               1200UL

// Straßenqualität: Normierung auf eine Referenzgeschwindigkeit.
//
// Achtung, gemessene Einschränkung: Die Klemmung wirkt nur zwischen
// 30/1,5 = 20 km/h und 30/0,7 = 42,9 km/h. Oberhalb davon ist der Faktor
// konstant 0,7, die Normierung also wirkungslos. In der Fahrt
// 20260730_084245_54526FA7 mit Median 167 km/h betraf das die gesamte
// Strecke. Die Konstanten sind hier benannt, damit diese Grenze sichtbar und
// testbar ist; die Auslegung selbst ist unverändert.
#define ROAD_QUALITY_REFERENCE_SPEED_KMH      30.0f
#define ROAD_QUALITY_SPEED_FACTOR_MIN          0.7f
#define ROAD_QUALITY_SPEED_FACTOR_MAX          1.5f

// Zweistufige Kurvenerkennung:
// - schnelle Kurven starten weiterhin unmittelbar über die Drehrate;
// - langgezogene Kurven werden über kumulierten Winkel und gefahrenen Weg
//   erkannt. Die Werte wurden gegen die per OSRM auf das Straßennetz
//   abgeglichene Fahrt 20260730_095603_A41A2450 geprüft.
// Eine Richtungsumkehr von mindestens fünf Grad trennt S-Kurven zuverlässig,
// während kleinere Gegenbewegungen als Heading-Rauschen behandelt werden.
#define CURVE_SHARP_START_RATE_DPS             8.0f
#define CURVE_LONG_MIN_RATE_DPS                0.8f
#define CURVE_LONG_START_ANGLE_DEG             8.0f
#define CURVE_LONG_MIN_DISTANCE_M             20.0f
#define CURVE_LONG_MAX_WINDOW_MS           10000UL
#define CURVE_MAX_SAMPLE_GAP_MS               500UL
#define CURVE_END_RATE_DPS                     0.8f
#define CURVE_END_QUIET_MS                  2000UL
// Netto-Kursänderung, die das Ruhefenster als echte Kurvenfortsetzung
// wertet. CURVE_LONG_MIN_RATE_DPS allein reicht dafür nicht: Auf
// nachgewiesen gerader Strecke rauscht die Gierrate um bis zu 1,5 Grad/s
// (Sitzung 20260731_155106_3AEFA823, Kurs konstant 19,8 Grad). Jede
// Rauschspitze in Kurvenrichtung setzte das Ruhefenster zurück, sodass
// Ereignisse über mehrere Kurven hinweg zusammenliefen - 58 von 174
// Ereignissen des 31.07.2026 deckten unter 60 Prozent ihrer Dauer mit
// echten Drehstichproben ab. Über die Fensterlänge gemessen entspricht der
// Wert einer mittleren Drehrate von 1 Grad/s; Rauschen hebt sich dabei auf,
// eine echte langgezogene Kurve nicht.
#define CURVE_QUIET_MIN_NET_ANGLE_DEG          2.0f
#define CURVE_MIN_EVENT_ANGLE_DEG             10.0f
// Mindestfahrweg und Mindest-Querbeschleunigung eines Ereignisses. Beide
// Werte wurden gegen die 55 von Hand markierten Referenzkurven der fünf
// Beifahrerfahrten vom 31.07.2026 gewählt und lassen dort keine einzige
// Referenzkurve entfallen: Die schwächste markierte Kurve fuhr 10,8 m Weg
// und 0,45 m/s². Zusammen entfernen sie die beiden bekannten
// Fehlalarmklassen - Ereignisse aus GPS-Drift im Stand (1 bis 4 m Weg) und
// beim Rangieren (0,27 m/s²).
#define CURVE_MIN_EVENT_DISTANCE_M            10.0f
#define CURVE_MIN_EVENT_LATERAL_ACCEL          0.4f
#define CURVE_REVERSAL_ANGLE_DEG               5.0f
#define CURVE_MAX_DURATION_MS              60000UL
#define CURVE_EVENT_COOLDOWN_MS              1000UL
#define CURVE_GRAVITY_MIN_MPS2                  7.0f
#define CURVE_GRAVITY_MAX_MPS2                 12.0f
#define CURVE_MIN_QUALITY_SAMPLES                  5

// Task-Watchdog: Ein hängender loop() führt zum Neustart statt zu einem
// stummen Gerät auf der Strecke. Grosszügig bemessen, damit langsame
// SD-Schreibvorgänge und OTA-Uploads nicht fälschlich auslösen.
#define WDT_TIMEOUT_MS       30000

// WLAN-Sendeleistung. Die volle Leistung erzeugt Stromspitzen um 350 mA, die
// über die gemeinsame 3,3-V-Schiene direkt in SD-Karte und BNO055 koppeln.
// Für ein Endgerät in wenigen Metern Entfernung ist 11 dBm reichlich.
#define WIFI_TX_POWER       WIFI_POWER_11dBm

// Display-Spezifikationen
#define SCREEN_WIDTH        128     // OLED Display Breite
#define SCREEN_HEIGHT       64      // OLED Display Höhe
#define OLED_RESET          -1      // OLED Reset Pin (nicht verwendet)

// Sensor-Konfiguration  
#define VIBRATION_THRESHOLD 2.0     // Vibrations-Schwellwert (m/s²)
#define CURVE_THRESHOLD     CURVE_MIN_EVENT_ANGLE_DEG

// System-Timing
#define SENSOR_READ_INTERVAL    100 // BNO055 Leseintervall (ms)
#define GPS_UPDATE_INTERVAL     200 // GPS Update-Intervall (ms) 
#define CAN_CHECK_INTERVAL      10  // CAN-Check Intervall (ms)
#define STATUS_REPORT_INTERVAL  5000// Status-Report Intervall (ms)

// Aufgeteilter SD-Flush. Zehn Schritte im Abstand von 500 ms ergeben einen
// vollen Umlauf in fünf Sekunden - dieselbe Sicherungsfrequenz wie zuvor,
// aber ohne die gemessenen 235 ms Blockade am Stück.
#define SD_FLUSH_STEP_INTERVAL_MS            500UL
#define SD_FLUSH_STEP_COUNT                     10

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

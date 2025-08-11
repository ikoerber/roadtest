#include "hardware_config.h"

// =============================================================================
// Hardware Pin-Definitionen Implementation
// =============================================================================
// Diese Datei definiert die in hardware_config.h deklarierten Konstanten

// I2C-Bus Pins
const int I2C_SDA = 8;
const int I2C_SCL = 9;

// SD-Karten SPI Pins
const int SD_CS_PIN = 4;
const int SD_MOSI_PIN = 5;
const int SD_MISO_PIN = 6;
const int SD_SCK_PIN = 7;

// CAN-Bus SPI Pins
const int CAN_CS_PIN = 1;
const int CAN_INT_PIN = 2;
const int CAN_SCK_PIN = 3;
const int CAN_MOSI_PIN = 13;
const int CAN_MISO_PIN = 11;

// GPS UART Pins
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 15;
const int GPS_BAUD_RATE = 9600;
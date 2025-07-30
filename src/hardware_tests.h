#ifndef HARDWARE_TESTS_H
#define HARDWARE_TESTS_H

#include <Arduino.h>

// Pin-Definitionen
extern const int SD_CS_PIN;
extern const int I2C_SDA;
extern const int I2C_SCL;

// Hardware-Test Funktionen
void i2cScanner();
bool testSDWithSafePins();
void testBNO055();

// SD-Karte Funktionen
bool checkSDCardStatus();
bool initializeSDLogging();
void monitorSDCard();
bool writeToSDCard(const String& data);

#endif
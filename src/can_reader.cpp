#include "can_reader.h"
#include <SPI.h>

// Include arduino-CAN Library - Wir nutzen MCP2515 auf ESP32
#undef ARDUINO_ARCH_ESP32  // ESP32-internen CAN deaktivieren
#include "CANController.h"
#include "MCP2515.h"

// Globale CAN-Reader Instanz
CANReader canReader;

// MCP2515 Instanz erstellen
MCP2515Class canController;

CANReader::CANReader(int cs, int interrupt) 
    : initialized(false), messageCount(0), lastLogTime(0), 
      loggingEnabled(false), csPin(cs), intPin(interrupt),
      totalMessages(0), errorCount(0) {
}

CANReader::~CANReader() {
    end();
}

bool CANReader::begin(long baudRate) {
    if (initialized) {
        Serial.println("CAN-Reader bereits initialisiert");
        return true;
    }
    
    Serial.println("=== CAN-Bus Reader Initialisierung ===");
    Serial.printf("Pins: CS=%d, INT=%d\n", csPin, intPin);
    Serial.printf("Baudrate: %ld bps\n", baudRate);
    
    // ESP32-S3 spezifische Pin-Konfiguration
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    pinMode(intPin, INPUT_PULLUP);
    
    // SPI-Bus für CAN sauber initialisieren
    SPI.end();  // Beende eventuelle SD-SPI Nutzung
    delay(100);
    
    // ESP32-S3 sichere SPI-Pins nutzen
    SPI.begin(3, 11, 13, csPin);  // SCK=3, MISO=11, MOSI=13, CS=1
    delay(100);
    
    // Pin-Konfiguration setzen
    canController.setPins(csPin, intPin);
    
    // Debug: Teste SPI-Kommunikation direkt
    Serial.println("Teste SPI-Kommunikation...");
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
    uint8_t testResult = SPI.transfer(0x00);  // RESET command
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    Serial.printf("SPI Test-Response: 0x%02X\n", testResult);
    
    // MCP2515 Konfiguration - 16MHz Kristall versuchen (DEBO CON Standard)
    Serial.println("Teste 16MHz Kristall...");
    canController.setClockFrequency(16E6);
    canController.setSPIFrequency(1E6); // Niedrige SPI-Geschwindigkeit
    
    if (!canController.begin(baudRate)) {
        Serial.println("16MHz fehlgeschlagen, teste 8MHz...");
        
        // Fallback: 8MHz Kristall
        canController.setClockFrequency(8E6);
        
        if (!canController.begin(baudRate)) {
            Serial.println("❌ CAN-Bus Initialisierung fehlgeschlagen!");
            Serial.println("Mögliche Ursachen:");
            Serial.println("• Verkabelung: CS=GPIO1, INT=GPIO2, SCK=GPIO3, MOSI=GPIO13, MISO=GPIO11");
            Serial.println("• Stromversorgung: VCC1=5V, VCC=3.3V");
            Serial.println("• MCP2515 defekt oder nicht verbunden");
            Serial.println("• SPI-Pin-Konflikte");
            errorCount++;
            return false;
        } else {
            Serial.println("✅ 8MHz Kristall erkannt und konfiguriert");
        }
    } else {
        Serial.println("✅ 16MHz Kristall erkannt und konfiguriert");
    }
    
    initialized = true;
    messageCount = 0;
    totalMessages = 0;
    
    Serial.println("✅ CAN-Bus Reader erfolgreich gestartet!");
    Serial.printf("Bereit für Nachrichten auf %ld bps\n", baudRate);
    
    return true;
}

void CANReader::end() {
    if (!initialized) return;
    
    disableLogging();
    canController.end();
    initialized = false;
    
    Serial.println("CAN-Bus Reader beendet");
}

void CANReader::setPins(int cs, int interrupt) {
    if (initialized) {
        Serial.println("Warnung: Pin-Änderung nur vor begin() möglich");
        return;
    }
    
    csPin = cs;
    intPin = interrupt;
}

void CANReader::setClockFrequency(long clockFreq) {
    if (initialized) {
        Serial.println("Warnung: Clock-Frequenz nur vor begin() änderbar");
        return;
    }
    
    // MCP2515 nutzt externe Clock-Konfiguration
    canController.setClockFrequency(clockFreq);
}

bool CANReader::hasMessage() {
    if (!initialized) return false;
    
    return (canController.parsePacket() > 0 || canController.packetId() != -1);
}

CANMessage CANReader::readMessage() {
    CANMessage msg = {0};
    
    if (!initialized) {
        Serial.println("CAN-Reader nicht initialisiert");
        errorCount++;
        return msg;
    }
    
    int packetSize = canController.parsePacket();
    
    if (packetSize > 0 || canController.packetId() != -1) {
        // Nachricht empfangen
        msg.timestamp = millis();
        msg.canId = canController.packetId();
        msg.extended = canController.packetExtended();
        msg.rtr = canController.packetRtr();
        msg.dlc = canController.packetDlc();
        msg.rssi = 0.0; // Nicht verfügbar bei MCP2515
        
        // Daten lesen (falls nicht RTR)
        int dataIndex = 0;
        while (canController.available() && dataIndex < 8) {
            msg.data[dataIndex++] = canController.read();
        }
        
        messageCount++;
        totalMessages++;
        
        // Automatisches Logging
        if (loggingEnabled) {
            logMessage(msg);
        }
        
        return msg;
    }
    
    return msg; // Leere Nachricht
}

int CANReader::getAvailableMessages() {
    if (!initialized) return 0;
    
    return canController.available();
}

bool CANReader::enableLogging(const String& fileName) {
    if (loggingEnabled) {
        Serial.println("Logging bereits aktiviert");
        return true;
    }
    
    logFileName = fileName;
    logFile = SD.open(logFileName, FILE_WRITE);
    
    if (!logFile) {
        Serial.printf("❌ Kann Log-Datei nicht öffnen: %s\n", fileName.c_str());
        return false;
    }
    
    // CSV-Header schreiben
    logFile.println("Timestamp,CAN_ID,Extended,RTR,DLC,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7,DataHex");
    logFile.flush();
    
    loggingEnabled = true;
    lastLogTime = millis();
    
    Serial.printf("✅ CAN-Logging aktiviert: %s\n", fileName.c_str());
    return true;
}

void CANReader::disableLogging() {
    if (!loggingEnabled) return;
    
    if (logFile) {
        logFile.close();
    }
    
    loggingEnabled = false;
    Serial.println("CAN-Logging deaktiviert");
}

bool CANReader::logMessage(const CANMessage& msg) {
    if (!loggingEnabled || !logFile) return false;
    
    // CSV-Format: Timestamp,CAN_ID,Extended,RTR,DLC,Data0-7,DataHex
    logFile.printf("%lu,0x%lX,%d,%d,%d", 
                   msg.timestamp, msg.canId, msg.extended, msg.rtr, msg.dlc);
    
    // Einzelne Daten-Bytes
    for (int i = 0; i < 8; i++) {
        if (i < msg.dlc) {
            logFile.printf(",%02X", msg.data[i]);
        } else {
            logFile.print(",");
        }
    }
    
    // Hex-String der kompletten Daten
    logFile.print(",");
    for (int i = 0; i < msg.dlc; i++) {
        logFile.printf("%02X", msg.data[i]);
    }
    
    logFile.println();
    
    // Regelmäßig flushen (alle 5 Sekunden)
    if (millis() - lastLogTime > 5000) {
        logFile.flush();
        lastLogTime = millis();
    }
    
    return true;
}

void CANReader::flushLog() {
    if (loggingEnabled && logFile) {
        logFile.flush();
        lastLogTime = millis();
    }
}

void CANReader::setFilter(long id, long mask) {
    if (!initialized) {
        Serial.println("CAN-Reader nicht initialisiert für Filter");
        return;
    }
    
    if (canController.filter(id, mask)) {
        Serial.printf("✅ Standard-Filter gesetzt: ID=0x%lX, Mask=0x%lX\n", id, mask);
    } else {
        Serial.printf("❌ Filter-Fehler: ID=0x%lX, Mask=0x%lX\n", id, mask);
        errorCount++;
    }
}

void CANReader::setExtendedFilter(long id, long mask) {
    if (!initialized) {
        Serial.println("CAN-Reader nicht initialisiert für Extended-Filter");
        return;
    }
    
    if (canController.filterExtended(id, mask)) {
        Serial.printf("✅ Extended-Filter gesetzt: ID=0x%lX, Mask=0x%lX\n", id, mask);
    } else {
        Serial.printf("❌ Extended-Filter-Fehler: ID=0x%lX, Mask=0x%lX\n", id, mask);
        errorCount++;
    }
}

void CANReader::clearFilters() {
    // Alle Nachrichten empfangen (Standard-Verhalten)
    setFilter(0x000, 0x000);
}

void CANReader::dumpRegisters() {
    if (!initialized) {
        Serial.println("CAN-Reader nicht initialisiert für Register-Dump");
        return;
    }
    
    Serial.println("=== MCP2515 Register-Dump ===");
    canController.dumpRegisters(Serial);
}

String CANReader::getStatusString() {
    static char statusBuffer[512];
    
    int written = snprintf(statusBuffer, sizeof(statusBuffer),
        "CAN-Reader Status:\n"
        "  Initialisiert: %s\n"
        "  Nachrichten empfangen: %lu\n"
        "  Gesamt-Nachrichten: %lu\n"
        "  Fehler: %lu\n"
        "  Logging: %s\n",
        initialized ? "Ja" : "Nein",
        (unsigned long)messageCount,
        (unsigned long)totalMessages,
        (unsigned long)errorCount,
        loggingEnabled ? "Aktiv" : "Inaktiv"
    );
    
    if (loggingEnabled && written > 0 && written < (int)sizeof(statusBuffer) - 1) {
        snprintf(statusBuffer + written, sizeof(statusBuffer) - written,
                 "  Log-Datei: %s\n", logFileName.c_str());
    }
    
    return String(statusBuffer);
}

void CANReader::onReceive(void(*callback)(int)) {
    if (!initialized) {
        Serial.println("CAN-Reader nicht initialisiert für Callback");
        return;
    }
    
    canController.onReceive(callback);
    Serial.println("✅ CAN-Receive-Callback registriert");
}

// Hilfsfunktionen
String formatCANMessage(const CANMessage& msg) {
    static char msgBuffer[256];
    char* ptr = msgBuffer;
    size_t remaining = sizeof(msgBuffer);
    
    // Time und ID
    int written = snprintf(ptr, remaining, "Time: %lums, ID: %s, ", 
                          msg.timestamp, getCANIdString(msg.canId, msg.extended).c_str());
    if (written > 0 && written < (int)remaining) {
        ptr += written;
        remaining -= written;
    }
    
    // Flags
    if (msg.extended && remaining > 5) {
        strncpy(ptr, "EXT, ", remaining);
        ptr += 5;
        remaining -= 5;
    }
    if (msg.rtr && remaining > 5) {
        strncpy(ptr, "RTR, ", remaining);
        ptr += 5;
        remaining -= 5;
    }
    
    // DLC
    written = snprintf(ptr, remaining, "DLC: %d", msg.dlc);
    if (written > 0 && written < (int)remaining) {
        ptr += written;
        remaining -= written;
    }
    
    // Data bytes
    if (!msg.rtr && msg.dlc > 0 && remaining > 10) {
        strncpy(ptr, ", Data: ", remaining);
        ptr += 8;
        remaining -= 8;
        
        for (int i = 0; i < msg.dlc && remaining > 3; i++) {
            written = snprintf(ptr, remaining, "%02X", msg.data[i]);
            if (written > 0 && written < (int)remaining) {
                ptr += written;
                remaining -= written;
                
                if (i < msg.dlc - 1 && remaining > 1) {
                    *ptr++ = ' ';
                    remaining--;
                }
            }
        }
    }
    
    *ptr = '\0';
    return String(msgBuffer);
}

String getCANIdString(long id, bool extended) {
    static char idBuffer[32];
    
    if (extended) {
        snprintf(idBuffer, sizeof(idBuffer), "0x%lX (29-bit)", id);
    } else {
        snprintf(idBuffer, sizeof(idBuffer), "0x%lX (11-bit)", id);
    }
    
    return String(idBuffer);
}

void printCANMessage(const CANMessage& msg) {
    Serial.println("--- CAN-Nachricht ---");
    Serial.println(formatCANMessage(msg));
    
    if (!msg.rtr && msg.dlc > 0) {
        Serial.print("Raw Bytes: ");
        for (int i = 0; i < msg.dlc; i++) {
            Serial.printf("0x%02X ", msg.data[i]);
        }
        Serial.println();
        
        Serial.print("ASCII: ");
        for (int i = 0; i < msg.dlc; i++) {
            if (msg.data[i] >= 32 && msg.data[i] <= 126) {
                Serial.print((char)msg.data[i]);
            } else {
                Serial.print(".");
            }
        }
        Serial.println();
    }
    
    Serial.println("---------------------");
}

// Neue Methoden für Integration-Tests

bool CANReader::available() {
    // Alias für hasMessage
    return hasMessage();
}

void CANReader::update() {
    // Prozessiert anstehende Nachrichten
    if (!initialized) return;
    
    // Prüfe ob neue Nachrichten vorhanden sind
    if (hasMessage()) {
        // Lese und speichere die letzte Nachricht
        lastMessage = readMessage();
        lastMessage.timestamp = millis();
    }
}

CANMessage CANReader::getLastMessage() {
    // Gibt die zuletzt empfangene Nachricht zurück
    return lastMessage;
}
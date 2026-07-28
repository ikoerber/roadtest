#include "can_reader.h"
#include "hardware_config.h"
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
      clockFrequency(CAN_CLOCK_16MHZ),
      totalMessages(0), errorCount(0),
      lastMessage{}, pendingMessage{}, hasPendingMessage(false) {
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
    Serial.printf("Pins: CS=%d, INT=%d, SCK=%d, MOSI=%d, MISO=%d\n",
                  csPin, intPin, CAN_SCK_PIN, CAN_MOSI_PIN, CAN_MISO_PIN);

    // Der SPI-Bus wird vom Aufrufer einmalig eingerichtet, siehe
    // initializeOptionalCAN() in main.cpp.
    //
    // Frühere Fassungen riefen hier erneut SPI.end() und SPI.begin() auf, mit
    // dem Kommentar, damit werde "eventuelle SD-SPI Nutzung" beendet. Das war
    // sachlich falsch: Die SD-Karte hängt an der eigenen HSPI-Instanz, CAN am
    // globalen SPI-Objekt. Es sind getrennte Peripherien. Ein Busneustart zur
    // Laufzeit bringt hier nichts und kann eine laufende Übertragung abreißen.
    //
    // Ebenfalls entfernt: die Toggle-Tests, die SCK und MOSI per pinMode() und
    // digitalWrite() ansteuerten, nachdem SPI.begin() dieselben Pins bereits
    // über die GPIO-Matrix der SPI-Einheit zugewiesen hatte. Beide Treiber
    // arbeiteten dabei gegeneinander.
    pinMode(intPin, INPUT_PULLUP);
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    canController.setPins(csPin, intPin);

    // MCP2515 mit der fest hinterlegten Quarzfrequenz konfigurieren.
    // Der verbaute Joy-IT SBC-CAN01 trägt einen Quarz mit Aufdruck 16.000.
    // Früher wurde hier 16 MHz probiert und bei Misserfolg auf 8 MHz
    // umgeschaltet. Das ist gefährlich: Eine falsche Annahme wird von
    // begin() nicht erkannt, sondern ergibt eine um Faktor zwei falsche
    // Bitrate. Der Knoten quittiert dann jeden Frame als Fehler, geht in
    // Bus-Off und hält die INT-Leitung dauerhaft aktiv.
    Serial.printf("MCP2515: Quarz %ld Hz, Bitrate %ld bps\n",
                  clockFrequency, baudRate);
    canController.setClockFrequency(clockFrequency);
    canController.setSPIFrequency(1E6); // Niedrige SPI-Geschwindigkeit

    if (!canController.begin(baudRate)) {
        Serial.println("❌ CAN-Bus Initialisierung fehlgeschlagen!");
        Serial.println("Mögliche Ursachen:");
        Serial.printf("• Verkabelung: CS=GPIO%d, INT=GPIO%d, SCK=GPIO%d, MOSI=GPIO%d, MISO=GPIO%d\n",
                      csPin, intPin, CAN_SCK_PIN, CAN_MOSI_PIN, CAN_MISO_PIN);
        Serial.println("• Stromversorgung: VCC=3,3V (Logik), VCC1=5V (Bustreiber)");
        Serial.println("• Quarzaufdruck gegen CAN_CLOCK_16MHZ in hardware_config.h prüfen");
        Serial.println("• MCP2515 defekt oder nicht verbunden");
        errorCount++;
        return false;
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
    hasPendingMessage = false;
    pendingMessage = CANMessage{};

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

    // Nur merken. Angewendet wird der Wert in begin(), sonst würde ihn die
    // dortige Konfiguration wieder überschreiben.
    clockFrequency = clockFreq;
}

bool CANReader::fetchPacket() {
    if (hasPendingMessage) {
        return true;
    }
    if (!initialized) {
        return false;
    }

    // parsePacket() liest den Frame aus dem Empfangspuffer des MCP2515 und
    // quittiert dabei RXnIF. Ein zweiter Aufruf findet die Nachricht deshalb
    // nicht mehr vor. Genau das passierte bisher: hasMessage() parste den
    // Frame, readMessage() rief parsePacket() erneut auf, bekam nichts mehr
    // und lieferte eine leere Nachricht zurück - jeder Frame ging verloren.
    // Das Ergebnis wird deshalb hier zwischengespeichert.
    const int packetSize = canController.parsePacket();
    if (packetSize <= 0 && canController.packetId() == -1) {
        return false;
    }

    pendingMessage = CANMessage{};
    pendingMessage.timestamp = millis();
    pendingMessage.canId = canController.packetId();
    pendingMessage.extended = canController.packetExtended();
    pendingMessage.rtr = canController.packetRtr();
    pendingMessage.dlc = canController.packetDlc();
    pendingMessage.rssi = 0.0f; // Nicht verfügbar bei MCP2515

    int dataIndex = 0;
    while (canController.available() && dataIndex < 8) {
        pendingMessage.data[dataIndex++] = canController.read();
    }

    hasPendingMessage = true;
    messageCount++;
    totalMessages++;

    if (loggingEnabled) {
        logMessage(pendingMessage);
    }

    return true;
}

bool CANReader::hasMessage() {
    return fetchPacket();
}

CANMessage CANReader::readMessage() {
    if (!fetchPacket()) {
        // Keine Nachricht vorhanden. Der Aufrufer sollte vorher hasMessage()
        // prüfen; die CAN-ID 0x000 ist ein gültiger Bezeichner und taugt
        // deshalb nicht als Kennzeichen für "leer".
        return CANMessage{};
    }

    hasPendingMessage = false;
    lastMessage = pendingMessage;
    return pendingMessage;
}

int CANReader::getAvailableMessages() {
    // Der MCP2515 gibt keine Anzahl wartender Frames heraus; available() des
    // Treibers meldet lediglich die Restbytes des gerade geparsten Frames.
    // Hier zählt deshalb nur, ob ein abholbereiter Frame vorliegt.
    return fetchPacket() ? 1 : 0;
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
    // Bewusst nicht an den Treiber weitergereicht.
    //
    // MCP2515Class::onReceive() haengt den Callback an einen GPIO-Interrupt.
    // Der Handler ruft handleInterrupt() und darueber parsePacket() und
    // readRegister() auf - also SPI-Transfers. SPI.beginTransaction() nimmt
    // auf dem ESP32 einen FreeRTOS-Mutex; das im Interruptkontext zu tun
    // fuehrt zum Abbruch der Firmware. Zusaetzlich liegt der Handler nicht im
    // IRAM.
    //
    // Der unterstuetzte Weg ist das Polling in loop(): dort wird alle 10 ms
    // hasMessage() geprueft und readMessage() abgeholt.
    (void)callback;
    Serial.println("⚠️ CAN-Callback nicht unterstuetzt: Der Treiber wuerde SPI "
                   "im Interruptkontext ausfuehren.");
    Serial.println("   Nachrichten werden stattdessen in loop() abgefragt.");
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
    // Holt einen anstehenden Frame ab. readMessage() aktualisiert lastMessage
    // selbst; der Zeitstempel stammt aus dem Empfangszeitpunkt und wird hier
    // bewusst nicht überschrieben.
    if (hasMessage()) {
        readMessage();
    }
}

CANMessage CANReader::getLastMessage() {
    // Gibt die zuletzt empfangene Nachricht zurück
    return lastMessage;
}
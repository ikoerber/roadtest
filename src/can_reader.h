#ifndef CAN_READER_H
#define CAN_READER_H

#include <Arduino.h>
#include <SD.h>

// CAN-Bus Nachrichtenstruktur
struct CANMessage {
    unsigned long timestamp;    // Zeitstempel in ms
    long canId;                // CAN-ID (11 oder 29-bit)
    bool extended;             // Extended Frame?
    bool rtr;                  // Remote Transmission Request?
    int dlc;                   // Data Length Code
    uint8_t data[8];          // Nutzdaten (max 8 Bytes)
    float rssi;               // Signal-Stärke (falls verfügbar)
};

// CAN-Bus Reader Klasse
class CANReader {
private:
    bool initialized;
    unsigned long messageCount;
    unsigned long lastLogTime;
    String logFileName;
    File logFile;
    bool loggingEnabled;
    
    // Pin-Konfiguration
    int csPin;
    int intPin;
    
    // Statistiken
    unsigned long totalMessages;
    unsigned long errorCount;
    
    // Letzte empfangene Nachricht
    CANMessage lastMessage;
    
public:
    CANReader(int cs = 1, int interrupt = 2);
    ~CANReader();
    
    // Initialisierung und Konfiguration
    bool begin(long baudRate = 500E3);
    void end();
    void setPins(int cs, int interrupt);
    void setClockFrequency(long clockFreq = 16E6);
    
    // Nachrichtenempfang
    bool hasMessage();
    CANMessage readMessage();
    int getAvailableMessages();
    bool available();  // Alias für hasMessage
    void update();     // Prozessiert anstehende Nachrichten
    CANMessage getLastMessage();  // Gibt letzte empfangene Nachricht zurück
    
    // Logging-Funktionen
    bool enableLogging(const String& fileName);
    void disableLogging();
    bool logMessage(const CANMessage& msg);
    void flushLog();
    
    // Filter-Funktionen
    void setFilter(long id, long mask = 0x7FF);
    void setExtendedFilter(long id, long mask = 0x1FFFFFFF);
    void clearFilters();
    
    // Status und Statistiken
    bool isInitialized() const { return initialized; }
    bool isReady() const { return initialized; }  // Alias für isInitialized
    unsigned long getMessageCount() const { return messageCount; }
    unsigned long getTotalMessages() const { return totalMessages; }
    unsigned long getErrorCount() const { return errorCount; }
    
    // Debugging
    void dumpRegisters();
    String getStatusString();
    
    // Callback für Interrupt-basiertes Empfangen
    void onReceive(void(*callback)(int));
};

// Globale CAN-Reader Instanz
extern CANReader canReader;

// Hilfsfunktionen
String formatCANMessage(const CANMessage& msg);
String getCANIdString(long id, bool extended);
void printCANMessage(const CANMessage& msg);

#endif // CAN_READER_H
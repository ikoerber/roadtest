#ifndef CAN_READER_H
#define CAN_READER_H

#include <Arduino.h>

#include "hardware_config.h"

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

// Ausschließlich dekodierte Live-Werte aus standardisiertem OBD-II
// Service 01. Ungültige Werte werden nie als Null ausgegeben.
struct OBDLiveData {
    bool rpmValid = false;
    bool speedValid = false;
    bool throttleValid = false;
    bool mafValid = false;
    bool ambientTemperatureValid = false;
    bool oilTemperatureValid = false;
    bool fuelRateValid = false;
    float rpm = 0.0f;
    uint8_t speedKmh = 0;
    float throttlePercent = 0.0f;
    float mafGramsPerSecond = 0.0f;
    float ambientTemperatureC = 0.0f;
    float oilTemperatureC = 0.0f;
    float fuelRateLitersPerHour = 0.0f;
    unsigned long lastResponseMs = 0;
    unsigned long rpmUpdatedMs = 0;
    unsigned long speedUpdatedMs = 0;
    unsigned long throttleUpdatedMs = 0;
    unsigned long mafUpdatedMs = 0;
    unsigned long ambientTemperatureUpdatedMs = 0;
    unsigned long oilTemperatureUpdatedMs = 0;
    unsigned long fuelRateUpdatedMs = 0;
    uint8_t lastPid = 0;
    unsigned long requestCount = 0;
    unsigned long responseCount = 0;
    unsigned long requestErrors = 0;
    unsigned long supportResponseCount = 0;
    bool supportBlockValid[4] = {false, false, false, false};
    uint32_t supportBitmap[4] = {0, 0, 0, 0};
};

// Dekodierte, ausschließlich gelesene MCP2515-Fehlerdiagnose.
struct CANHardwareDiagnostics {
    bool valid = false;
    uint8_t operatingMode = 0;
    uint8_t transmitErrorCount = 0;
    uint8_t receiveErrorCount = 0;
    uint8_t errorFlags = 0;
    uint8_t txBuffer0Control = 0;
    bool errorWarning = false;
    bool receiveWarning = false;
    bool transmitWarning = false;
    bool receiveErrorPassive = false;
    bool transmitErrorPassive = false;
    bool transmitBusOff = false;
    bool receiveBuffer0Overflow = false;
    bool receiveBuffer1Overflow = false;
    uint32_t receiveBuffer0OverflowCount = 0;
    uint32_t receiveBuffer1OverflowCount = 0;
    uint32_t controllerRecoveryCount = 0;
};

enum class OBDTraceEventType : uint8_t {
    REQUEST_SENT,
    TRANSMIT_ERROR,
    RESPONSE,
    TIMEOUT
};

struct OBDTraceEvent {
    OBDTraceEventType type = OBDTraceEventType::REQUEST_SENT;
    uint32_t sequence = 0;
    uint32_t eventUptimeMs = 0;
    uint32_t requestUptimeMs = 0;
    uint32_t responseUptimeMs = 0;
    uint32_t responseLatencyMs = 0;
    uint32_t responseCanId = 0;
    uint8_t pid = 0;
    bool transmitOK = false;
    bool matchedRequest = false;
};

struct OBDSessionStats {
    bool active = false;
    uint32_t startedAt = 0;
    uint32_t requestCount = 0;
    uint32_t responseCount = 0;
    uint32_t requestErrors = 0;
    uint32_t timeoutCount = 0;
    uint32_t unmatchedResponseCount = 0;
    uint32_t supportResponseCount = 0;
    uint32_t traceDropped = 0;
    uint32_t lastRequestSequence = 0;
    uint32_t lastRequestMs = 0;
    uint32_t lastResponseMs = 0;
    uint32_t lastResponseLatencyMs = 0;
    uint32_t lastResponseCanId = 0;
    uint8_t lastRequestPid = 0;
    uint8_t lastResponsePid = 0;
    bool lastTransmitOK = false;
};

// CAN-Bus Reader Klasse
class CANReader {
private:
    static constexpr uint8_t OBD_TRACE_QUEUE_SIZE = 16;

    bool initialized;
    unsigned long messageCount;

    // Pin-Konfiguration
    int csPin;
    int intPin;

    // Quarzfrequenz des MCP2515-Moduls. Wird nicht mehr zur Laufzeit geraten:
    // Ein falscher Wert halbiert oder verdoppelt die Bitrate, der Knoten geht
    // in Bus-Off und die INT-Leitung bleibt dauerhaft aktiv.
    long clockFrequency;

    // Statistiken
    unsigned long totalMessages;
    unsigned long errorCount;
    uint32_t receiveBuffer0OverflowCount;
    uint32_t receiveBuffer1OverflowCount;
    uint32_t controllerRecoveryCount;
    
    // Letzte empfangene Nachricht
    CANMessage lastMessage;

    // Zwischengespeicherter Frame. parsePacket() entnimmt den Frame aus dem
    // Empfangspuffer des MCP2515 und löscht dabei das Interrupt-Flag; der
    // Aufruf darf deshalb pro Nachricht genau einmal erfolgen.
    CANMessage pendingMessage;
    bool hasPendingMessage;
    OBDLiveData obdData;
    bool obdPollingEnabled;
    OBDSessionStats obdSession;
    bool obdRequestPending;
    bool pendingOBDHadResponse;
    uint32_t pendingOBDSequence;
    uint32_t pendingOBDRequestMs;
    uint8_t pendingOBDPid;
    OBDTraceEvent obdTraceQueue[OBD_TRACE_QUEUE_SIZE];
    uint8_t obdTraceReadIndex;
    uint8_t obdTraceWriteIndex;

    // Holt höchstens einen Frame aus dem Controller in pendingMessage.
    bool fetchPacket();
    void enqueueOBDTrace(const OBDTraceEvent& event);
    void finishPendingOBDTimeout(uint32_t now);

public:
    CANReader(int cs = CAN_CS_PIN, int interrupt = CAN_INT_PIN);
    ~CANReader();

    // Initialisierung und Konfiguration
    bool begin(long baudRate = CAN_BAUDRATE);
    void end();
    bool restartController(
        long baudRate = CAN_BAUDRATE, bool passiveMode = false);
    void setPins(int cs, int interrupt);
    void setClockFrequency(long clockFreq = CAN_CLOCK_16MHZ);
    
    // Nachrichtenempfang
    bool hasMessage();
    CANMessage readMessage();
    int getAvailableMessages();
    bool available();  // Alias für hasMessage
    void update();     // Prozessiert anstehende Nachrichten
    CANMessage getLastMessage();  // Gibt letzte empfangene Nachricht zurück

    // Begrenzte, nur lesende OBD-II-Abfrage (Service 01).
    bool requestOBDPid(uint8_t pid);
    bool processOBDResponse(const CANMessage& msg);
    OBDLiveData getOBDData() const;
    void resetOBDDiscoveryData();
    void beginOBDSession();
    void endOBDSession();
    void updateOBDDiagnostics();
    OBDSessionStats getOBDSessionStats() const { return obdSession; }
    bool popOBDTraceEvent(OBDTraceEvent& event);
    bool isOBDSessionActive() const { return obdSession.active; }
    bool isOBDPidSupportKnown(uint8_t pid) const;
    bool isOBDPidSupported(uint8_t pid) const;
    void setOBDPollingEnabled(bool enabled) { obdPollingEnabled = enabled; }
    bool isOBDPollingEnabled() const { return obdPollingEnabled; }

    // Der Erkennungsmodus schaltet den MCP2515 wirklich auf Listen-Only und
    // öffnet den Standard-ID-Filter. Für OBD-Abfragen wird anschließend der
    // Normalmodus mit dem engen 0x7E8..0x7EF-Antwortfilter wiederhergestellt.
    bool configurePassiveCapture();
    bool configureOBDResponseMode();
    
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
    CANHardwareDiagnostics getHardwareDiagnostics();
    
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
const char* obdTraceEventName(OBDTraceEventType type);

#endif // CAN_READER_H

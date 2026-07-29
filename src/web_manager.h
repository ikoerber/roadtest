#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>

class WebManager {
private:
    WebServer server;
    bool initialized;
    bool otaAuthorized;
    bool otaInProgress;
    bool otaUploadRejected;
    bool otaBlockedByMeasurement;
    String otaUploadFileName;

    String buildStatusPage();
    String buildAcceptanceTestPage(const String& message = "");
    String buildAcceptanceStatusJSON();
    String buildCalibrationPage(const String& message = "");
    String buildUpdatePage();
    String buildUpdateResultPage(bool success);
    bool authenticateOTA();
    void handleUpdateUpload();
    void resetOTAState();
    void redirectAcceptance(bool blocked = false, bool ended = false);
    uint8_t getAcceptanceStage() const;
    uint32_t getAcceptanceCountdownSeconds(uint8_t stage) const;

public:
    WebManager();

    bool begin();
    void handleClient();
    bool isReady() const;
    bool isOTAInProgress() const { return otaInProgress; }
    String getIPAddress() const;
};

extern WebManager webManager;

#endif // WEB_MANAGER_H

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
    bool loggingWasActive;

    String buildStatusPage();
    String buildCalibrationPage(const String& message = "");
    String buildUpdatePage();
    bool authenticateOTA();
    void handleUpdateUpload();
    void resumeLoggingAfterFailedUpdate();

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

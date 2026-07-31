#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>

class WebManager {
private:
    enum class CurveReferenceType : uint8_t {
        NONE,
        STRAIGHT,
        LEFT_WIDE,
        RIGHT_WIDE,
        LEFT_NORMAL,
        RIGHT_NORMAL,
        S_CURVE
    };

    WebServer server;
    bool initialized;
    bool otaAuthorized;
    bool otaInProgress;
    bool otaUploadRejected;
    bool otaBlockedByMeasurement;
    String otaUploadFileName;
    bool curveTestActive;
    bool curveTestStartMarkerPending;
    bool curveReferenceActive;
    CurveReferenceType curveReferenceType;
    uint32_t curveReferenceId;
    unsigned long curveReferenceStartedAt;
    uint16_t curveStraightCount;
    uint16_t curveLeftWideCount;
    uint16_t curveRightWideCount;
    uint16_t curveLeftNormalCount;
    uint16_t curveRightNormalCount;
    uint16_t curveSCount;

    String buildStatusPage();
    String buildCurveTestPage(const String& message = "");
    String buildCurveTestStatusJSON();
    String buildCalibrationPage(const String& message = "");
    String buildUpdatePage();
    String buildUpdateResultPage(bool success);
    bool authenticateOTA();
    void handleUpdateUpload();
    void resetOTAState();
    void redirectCurveTest(const char* result = nullptr);
    const char* getCurveReferenceName(CurveReferenceType type) const;
    CurveReferenceType parseCurveReferenceType(const String& value) const;

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

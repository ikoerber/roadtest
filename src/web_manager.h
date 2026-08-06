#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>

// WebServer, der seinen chunked-Zustand herausgibt.
//
// Der Download rahmt seine HTTP-Chunks selbst, weil
// `WebServer::sendContent()` den Rückgabewert von `write()` verwirft: Bei
// einem Teilschreibvorgang stimmt die angekündigte Chunk-Länge dann nicht
// mehr mit den tatsächlich gesendeten Bytes überein, und die Gegenstelle
// wartet endlos auf den Rest. Ob überhaupt chunked geantwortet wird,
// entscheidet der WebServer anhand der HTTP-Version der Gegenstelle; dieser
// Zustand liegt `protected` und wird hier zugänglich gemacht. Ohne ihn müsste
// die Senke raten und würde einem HTTP/1.0-Client Chunk-Kopfzeilen mitten in
// die Nutzdaten schreiben.
class RoadtestWebServer : public WebServer {
public:
    using WebServer::WebServer;
    bool istChunked() const { return _chunked; }
};

class WebManager {
private:
    RoadtestWebServer server;
    bool initialized;
    bool otaAuthorized;
    bool otaInProgress;
    bool otaUploadRejected;
    bool otaBlockedByMeasurement;
    String otaUploadFileName;

    String buildStatusPage();
    String buildCalibrationPage(const String& message = "");
    String buildUpdatePage();
    String buildUpdateResultPage(bool success);
    String buildFilesPage(const String& message = "");
    bool authenticateOTA();
    void handleUpdateUpload();
    void handleSessionDownload();
    void handleSessionDelete();
    void handleSpeedtest();
    void resetOTAState();

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

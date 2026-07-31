#include "web_manager.h"

#include <WiFi.h>
#include <Update.h>

#include "bno055_manager.h"
#include "can_reader.h"
#include "gps_manager.h"
#include "hardware_config.h"
#include "oled_manager.h"
#include "runtime_diagnostics.h"
#include "sd_logger.h"
#include "vehicle_data_discovery.h"

namespace {
constexpr const char* WIFI_SSID = "ROADTEST";
constexpr const char* WIFI_PASSWORD = "roadtest123";
constexpr const char* OTA_USERNAME = "admin";
constexpr const char* OTA_PASSWORD = "roadtest123";

enum class CalibrationStepState : uint8_t {
    DONE,
    CURRENT,
    WAITING
};

String formatDuration(uint32_t totalSeconds) {
    char duration[16];
    snprintf(duration, sizeof(duration), "%02lu:%02lu:%02lu",
             static_cast<unsigned long>(totalSeconds / 3600),
             static_cast<unsigned long>((totalSeconds / 60) % 60),
             static_cast<unsigned long>(totalSeconds % 60));
    return String(duration);
}

String escapeHTML(String value) {
    value.replace("&", "&amp;");
    value.replace("<", "&lt;");
    value.replace(">", "&gt;");
    value.replace("\"", "&quot;");
    value.replace("'", "&#39;");
    return value;
}

String escapeJSON(String value) {
    value.replace("\\", "\\\\");
    value.replace("\"", "\\\"");
    value.replace("\n", "\\n");
    value.replace("\r", "");
    return value;
}

void appendCalibrationStep(
    String& page, uint8_t number, const char* title, const char* scoreLabel,
    uint8_t score, CalibrationStepState state, const char* instruction) {
    page += F("<section class='step ");
    switch (state) {
        case CalibrationStepState::DONE:
            page += F("done'><div class='step-number'>✓</div>");
            break;
        case CalibrationStepState::CURRENT:
            page += F("current'><div class='step-number'>");
            page += String(number);
            page += F("</div>");
            break;
        case CalibrationStepState::WAITING:
            page += F("waiting'><div class='step-number'>");
            page += String(number);
            page += F("</div>");
            break;
    }
    page += F("<div class='step-copy'><div class='eyebrow'>");
    page += state == CalibrationStepState::DONE
        ? F("ERLEDIGT")
        : (state == CalibrationStepState::CURRENT ? F("JETZT") : F("DANACH"));
    page += F("</div><h2>");
    page += title;
    page += F("</h2><p>");
    page += instruction;
    page += F("</p></div><div class='score'>");
    page += scoreLabel;
    page += F("<strong>");
    page += String(score);
    page += F("</strong><span>/3</span></div></section>");
}

}

WebManager webManager;

WebManager::WebManager()
    : server(80),
      initialized(false),
      otaAuthorized(false),
      otaInProgress(false),
      otaUploadRejected(false),
      otaBlockedByMeasurement(false),
      otaUploadFileName(""),
      curveTestActive(false),
      curveTestStartMarkerPending(false),
      curveReferenceActive(false),
      curveReferenceType(CurveReferenceType::NONE),
      curveReferenceId(0),
      curveReferenceStartedAt(0),
      curveStraightCount(0),
      curveLeftWideCount(0),
      curveRightWideCount(0),
      curveLeftNormalCount(0),
      curveRightNormalCount(0),
      curveSCount(0) {}

bool WebManager::begin() {
    if (initialized) {
        if (isReady()) {
            return true;
        }

        Serial.println("⚠️ ROADTEST-WLAN wird neu gestartet");
    }

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    IPAddress apAddress(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    if (!WiFi.softAPConfig(apAddress, gateway, subnet) ||
        !WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 6, false, 2)) {
        Serial.println("❌ ROADTEST-WLAN konnte nicht gestartet werden");
        return false;
    }

    // Sendeleistung begrenzen. Die Stromspitzen des Leistungsverstärkers
    // koppeln über die gemeinsame 3,3-V-Schiene in SD-Karte und BNO055; für
    // ein Endgerät in Reichweite weniger Meter ist die volle Leistung unnötig.
    WiFi.setTxPower(WIFI_TX_POWER);

    // Weicht die vergebene Adresse ab, ist das der einzige sichtbare Hinweis
    // darauf, dass softAPConfig() nicht wie erwartet gegriffen hat.
    const IPAddress actualAddress = WiFi.softAPIP();
    if (actualAddress != apAddress) {
        Serial.printf("⚠️ Access Point läuft auf %s statt %s\n",
                      actualAddress.toString().c_str(),
                      apAddress.toString().c_str());
    }

    if (initialized) {
        server.begin();
        Serial.println("✅ ROADTEST-WLAN wiederhergestellt");
        return true;
    }

    server.on("/", HTTP_GET, [this]() {
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "text/html; charset=utf-8", buildStatusPage());
    });

    server.on("/update", HTTP_GET, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "text/html; charset=utf-8", buildUpdatePage());
    });

    server.on("/calibration", HTTP_GET, [this]() {
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "text/html; charset=utf-8", buildCalibrationPage());
    });

    server.on("/curve-test", HTTP_GET, [this]() {
        String message;
        if (server.arg("result") == "blocked") {
            message = "Aktion nicht gespeichert. Bitte den angezeigten "
                      "Schritt prüfen und erneut versuchen.";
        } else if (server.arg("result") == "ended") {
            message = "Kurventest beendet und Messdateien geschlossen.";
        }
        server.sendHeader("Cache-Control", "no-store");
        server.send(
            200, "text/html; charset=utf-8",
            buildCurveTestPage(message));
    });

    server.on("/curve-test/status", HTTP_GET, [this]() {
        server.sendHeader("Cache-Control", "no-store");
        server.send(
            200, "application/json; charset=utf-8",
            buildCurveTestStatusJSON());
    });

    server.on("/curve-test/start", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (vehicleDataDiscovery.isActive() || curveTestActive ||
            !sdLogger.isReady() || sdLogger.isLogging() ||
            sdLogger.isLoggingStartPending()) {
            redirectCurveTest("blocked");
            return;
        }
        if (!sdLogger.requestLoggingStart()) {
            redirectCurveTest("blocked");
            return;
        }
        curveTestActive = true;
        curveTestStartMarkerPending = true;
        curveReferenceActive = false;
        curveReferenceType = CurveReferenceType::NONE;
        curveReferenceStartedAt = 0;
        curveStraightCount = 0;
        curveLeftWideCount = 0;
        curveRightWideCount = 0;
        curveLeftNormalCount = 0;
        curveRightNormalCount = 0;
        curveSCount = 0;
        redirectCurveTest();
    });

    server.on("/curve-test/reference-start", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        const CurveReferenceType type =
            parseCurveReferenceType(server.arg("type"));
        if (!curveTestActive || !sdLogger.isLogging() ||
            curveReferenceActive || type == CurveReferenceType::NONE) {
            redirectCurveTest("blocked");
            return;
        }
        const uint32_t referenceId = curveReferenceId + 1;
        const unsigned long startedAt = millis();
        const String description =
            String("Id=") + String(referenceId) +
            ";Type=" + getCurveReferenceName(type);
        if (!sdLogger.logEvent(
                "CURVE_REFERENCE_START", description,
                0, 0, 0, false)) {
            redirectCurveTest("blocked");
            return;
        }
        curveReferenceId = referenceId;
        curveReferenceType = type;
        curveReferenceStartedAt = startedAt;
        curveReferenceActive = true;
        redirectCurveTest();
    });

    server.on("/curve-test/reference-end", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (!curveTestActive || !sdLogger.isLogging() ||
            !curveReferenceActive) {
            redirectCurveTest("blocked");
            return;
        }
        const CurveReferenceType completedType = curveReferenceType;
        const unsigned long endedAt = millis();
        const String description =
            String("Id=") + String(curveReferenceId) +
            ";Type=" + getCurveReferenceName(completedType) +
            ";StartMs=" + String(curveReferenceStartedAt) +
            ";EndMs=" + String(endedAt) +
            ";DurationMs=" +
            String(endedAt - curveReferenceStartedAt);
        if (!sdLogger.logEvent(
                "CURVE_REFERENCE_END", description,
                0, 0, 0, false)) {
            redirectCurveTest("blocked");
            return;
        }
        if (completedType == CurveReferenceType::STRAIGHT) {
            curveStraightCount++;
        } else if (completedType == CurveReferenceType::LEFT_WIDE) {
            curveLeftWideCount++;
        } else if (completedType == CurveReferenceType::RIGHT_WIDE) {
            curveRightWideCount++;
        } else if (completedType == CurveReferenceType::LEFT_NORMAL) {
            curveLeftNormalCount++;
        } else if (completedType == CurveReferenceType::RIGHT_NORMAL) {
            curveRightNormalCount++;
        } else if (completedType == CurveReferenceType::S_CURVE) {
            curveSCount++;
        }
        curveReferenceActive = false;
        curveReferenceType = CurveReferenceType::NONE;
        curveReferenceStartedAt = 0;
        redirectCurveTest();
    });

    server.on("/curve-test/end", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (!curveTestActive || curveReferenceActive) {
            redirectCurveTest("blocked");
            return;
        }
        if (sdLogger.isLogging()) {
            sdLogger.logEvent(
                "CURVE_TEST_END",
                String("Straight=") + String(curveStraightCount) +
                    ";LeftWide=" + String(curveLeftWideCount) +
                    ";RightWide=" + String(curveRightWideCount) +
                    ";LeftNormal=" + String(curveLeftNormalCount) +
                    ";RightNormal=" + String(curveRightNormalCount) +
                    ";S=" + String(curveSCount),
                0, 0, 0, false);
            sdLogger.stopLogging();
        }
        curveTestActive = false;
        curveTestStartMarkerPending = false;
        curveReferenceType = CurveReferenceType::NONE;
        curveReferenceStartedAt = 0;
        redirectCurveTest("ended");
    });

    server.on("/calibration/save", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }

        CalibrationData calibration = bnoManager.getCalibration();
        if (!calibration.isFullyCalibrated()) {
            server.sendHeader("Cache-Control", "no-store");
            server.send(
                409, "text/html; charset=utf-8",
                buildCalibrationPage(
                    String("Noch nicht gespeichert: Alle für ") +
                    ROADTEST_BNO_MODE_NAME +
                    " benötigten Kalibrierwerte müssen 3 sein."));
            return;
        }

        const bool saved = bnoManager.saveCalibration();
        server.sendHeader("Cache-Control", "no-store");
        server.send(
            saved ? 200 : 500, "text/html; charset=utf-8",
            buildCalibrationPage(
                saved ? "Kalibrierung dauerhaft gespeichert."
                      : "Kalibrierung konnte nicht gespeichert werden."));
    });

    server.on("/ride/start", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (vehicleDataDiscovery.isActive()) {
            server.send(
                409, "text/html; charset=utf-8",
                "<!doctype html><meta charset='utf-8'><h1>Start nicht möglich</h1>"
                "<p>Der Fahrzeug- oder Kurventest verwaltet die laufende "
                "Aufzeichnung bereits.</p><p><a href='/'>Zurück</a></p>");
            return;
        }
        if (!sdLogger.isReady()) {
            server.send(
                503, "text/html; charset=utf-8",
                "<!doctype html><meta charset='utf-8'><h1>Start nicht möglich</h1>"
                "<p>Die SD-Karte ist nicht bereit.</p><p><a href='/'>Zurück</a></p>");
            return;
        }
        if (!sdLogger.requestLoggingStart()) {
            server.send(
                500, "text/html; charset=utf-8",
                "<!doctype html><meta charset='utf-8'><h1>Start fehlgeschlagen</h1>"
                "<p>Die Aufzeichnung konnte nicht vorbereitet werden.</p>"
                "<p><a href='/'>Zurück</a></p>");
            return;
        }

        server.sendHeader("Cache-Control", "no-store");
        server.send(
            202, "text/html; charset=utf-8",
            "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<meta http-equiv='refresh' content='1;url=/'>"
            "<title>ROADTEST startet</title></head><body>"
            "<h1>Aufzeichnung wird vorbereitet</h1>"
            "<p>Die Logdateien werden schrittweise geöffnet.</p>"
            "<p><a href='/'>Status anzeigen</a></p></body></html>");
    });

    server.on("/ride/stop", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (curveTestActive) {
            if (curveReferenceActive) {
                redirectCurveTest("blocked");
                return;
            }
            sdLogger.stopLogging();
            curveTestActive = false;
            curveTestStartMarkerPending = false;
            redirectCurveTest("ended");
            return;
        }
        if (vehicleDataDiscovery.isActive()) {
            vehicleDataDiscovery.end();
        } else {
            sdLogger.stopLogging();
        }
        server.sendHeader("Location", "/", true);
        server.send(303, "text/plain; charset=utf-8", "");
    });

    server.on(
        "/update", HTTP_POST,
        [this]() {
            if (!otaAuthorized) {
                server.requestAuthentication();
                return;
            }

            const bool success =
                !otaUploadRejected && !Update.hasError();
            server.sendHeader("Connection", "close");
            server.send(
                success ? 200 : 500,
                "text/html; charset=utf-8",
                buildUpdateResultPage(success));

            if (success) {
                delay(500);
                ESP.restart();
            } else {
                resetOTAState();
            }
        },
        [this]() { handleUpdateUpload(); });

    server.onNotFound([this]() {
        server.send(404, "text/plain; charset=utf-8", "Nicht gefunden");
    });

    server.begin();
    initialized = true;

    Serial.println("✅ ROADTEST-WLAN und Webseite gestartet");
    Serial.printf("   WLAN: %s\n", WIFI_SSID);
    Serial.printf("   Passwort: %s\n", WIFI_PASSWORD);
    Serial.printf("   Webseite: http://%s/\n", getIPAddress().c_str());
    Serial.printf("   OTA-Benutzer: %s\n", OTA_USERNAME);
    return true;
}

void WebManager::handleClient() {
    if (initialized) {
        const unsigned long startedAt = millis();
        server.handleClient();
        if (curveTestActive && curveTestStartMarkerPending &&
            sdLogger.isLogging() &&
            sdLogger.logEvent(
                "CURVE_TEST_START",
                "Targets=Straight1;Wide4;Normal4;S3",
                0, 0, 0, false)) {
            curveTestStartMarkerPending = false;
        } else if (curveTestActive && curveTestStartMarkerPending &&
                   !sdLogger.isLogging() &&
                   !sdLogger.isLoggingStartPending() &&
                   sdLogger.getLastStartError().length() > 0) {
            curveTestActive = false;
            curveTestStartMarkerPending = false;
        }
        runtimeDiagnostics.recordWebDuration(millis() - startedAt);
    }
}

bool WebManager::isReady() const {
    wifi_mode_t mode = WiFi.getMode();
    bool accessPointMode =
        mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA;
    // Bewusst keine Prüfung auf eine bestimmte IP-Adresse: Hätte softAPConfig()
    // einmal eine andere Adresse vergeben, bliebe die Bedingung dauerhaft
    // falsch. Die Hardware-Überwachung würde dann alle fünf Sekunden begin()
    // aufrufen und den Access Point neu aufsetzen - kein Client könnte sich
    // stabil verbinden. Eine abweichende Adresse wird in begin() protokolliert.
    return initialized && accessPointMode &&
           WiFi.softAPIP() != IPAddress(0, 0, 0, 0);
}

String WebManager::getIPAddress() const {
    return WiFi.softAPIP().toString();
}

String WebManager::buildStatusPage() {
    RideSummary ride = sdLogger.getRideSummary();
    LogStats logStats = sdLogger.getStatistics();
    bool gpsCommunication = gpsManager.isCommunicating();
    bool gpsNMEAStream = gpsManager.isReceivingNMEA();
    GPSStatus gpsStatus = gpsManager.getStatus();
    OBDLiveData obd = canReader.getOBDData();
    CANHardwareDiagnostics canHardware =
        canReader.getHardwareDiagnostics();
    const bool ecuResponding =
        obd.lastResponseMs > 0 &&
        millis() - obd.lastResponseMs <= CAN_OBD_VALUE_MAX_AGE_MS;
    const bool obdPollingEnabled = canReader.isOBDPollingEnabled();
    bool requiredHardwareReady =
        bnoManager.isSelfTestPassed() && bnoManager.isFusionModeActive() &&
        bnoManager.isDataValid() &&
        (!OLED_REQUIRED || oledManager.isReady()) &&
        sdLogger.isReady() && gpsNMEAStream && isReady();
    String page;
    page.reserve(4500);
    page += F("<!doctype html><html lang='de'><head><meta charset='utf-8'>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<meta http-equiv='refresh' content='3'>"
              "<title>ROADTEST</title>"
              "<style>body{font-family:system-ui;max-width:34rem;margin:2rem auto;padding:0 1rem}"
              "table{border-collapse:collapse;width:100%}td{padding:.45rem;border-bottom:1px solid #ddd}"
              "td:last-child{text-align:right}a{display:inline-block;margin-top:1.2rem}"
              "form{margin:1.2rem 0}button{font:inherit;font-weight:600;padding:.75rem 1.1rem;"
              "border:0;border-radius:.45rem;background:#087a30;color:white}"
              "button.stop{background:#b42318}.hint{color:#555}.ok{color:#087a30}"
              ".warn{color:#b42318;font-weight:600}</style>"
              "</head><body><h1>ROADTEST</h1><table>");

    page += F("<tr><td>Firmware</td><td>");
    page += ROADTEST_FIRMWARE_VERSION;
    page += F("</td></tr><tr><td>Laufzeit</td><td>");
    page += String(millis() / 1000);
    page += F(" s</td></tr><tr><td>System</td><td class='");
    page += requiredHardwareReady ? F("ok'>bereit") : F("warn'>Prüfung läuft");
    page += F("</td></tr><tr><td>BNO055</td><td>");
    page += bnoManager.isSelfTestPassed() && bnoManager.isFusionModeActive() &&
                    bnoManager.isDataValid()
                ? String("OK · ") + ROADTEST_BNO_MODE_NAME
                : String("Fehler");
    page += F("</td></tr><tr><td>OLED</td><td>");
    page += oledManager.isReady() ? F("OK · optional")
                                 : F("nicht verbunden · optional");
    page += F("</td></tr><tr><td>GPS</td><td>");
    if (gpsManager.hasValidFix()) {
        page += F("Fix, ");
        page += String(gpsManager.getSatelliteCount());
        page += F(" Satelliten");
    } else {
        if (gpsCommunication) {
            page += F("NMEA OK, noch kein Fix");
        } else if (gpsNMEAStream) {
            page += F("Datenstrom OK, warte auf gültigen NMEA-Satz");
        } else {
            page += gpsManager.isReady() ? F("Suche Kommunikation") : F("Fehler");
        }
    }
    page += F("</td></tr><tr><td>GPS-Diagnose</td><td>");
    page += String(gpsStatus.sentences_received);
    page += F(" Sätze OK, ");
    page += String(gpsStatus.sentences_failed);
    page += F(" Prüfsummenfehler, ");
    page += String(gpsStatus.chars_processed);
    page += F(" Zeichen");
    page += F("</td></tr><tr><td>SD-Karte</td><td>");
    page += sdLogger.isReady() ? F("OK") : F("Fehler");
    if (sdLogger.getRecoveryStatus().length() > 0) {
        page += F("</td></tr><tr><td>SD-Wiederanlauf</td><td>");
        page += escapeHTML(sdLogger.getRecoveryStatus());
    }
    page += F("</td></tr><tr><td>Aufzeichnung</td><td>");
    if (sdLogger.isLoggingStartPending()) {
        page += F("wird vorbereitet");
    } else {
        page += ride.active ? F("aktiv") : F("gestoppt");
    }
    page += F("</td></tr><tr><td>Freier SD-Speicher</td><td>");
    if (sdLogger.isLoggingStartPending()) {
        page += F("wird ermittelt");
    } else if (sdLogger.isReady()) {
        page += String(sdLogger.getFreeSpace() / 1024);
        page += F(" MB");
    } else {
        page += F("-");
    }
    page += F("</td></tr><tr><td>CAN-Adapter</td><td>");
    if (canReader.isReady()) {
        page += F("OK · ");
        if (CAN_OBD_POLLING_ENABLED) {
            page += obdPollingEnabled ? F("OBD nur lesend · ")
                                      : F("OBD pausiert · ");
        } else {
            page += CAN_LISTEN_ONLY ? F("Listen-Only · ") : F("Normal · ");
        }
        page += String(canReader.getTotalMessages());
        page += F(" Nachrichten");
    } else {
        page += F("optional, nicht verbunden");
    }
    page += F("</td></tr>");
    if (canHardware.valid) {
        const bool canHardwareOK = canHardware.errorFlags == 0;
        page += F("<tr><td>CAN-Hardware</td><td class='");
        page += canHardwareOK ? F("ok'>OK") : F("warn'>");
        if (!canHardwareOK) {
            if (canHardware.transmitBusOff) {
                page += F("Bus-Off");
            } else if (canHardware.receiveBuffer0Overflow ||
                       canHardware.receiveBuffer1Overflow) {
                page += F("Pufferüberlauf");
            } else if (canHardware.transmitErrorPassive ||
                       canHardware.receiveErrorPassive) {
                page += F("fehlerpassiv");
            } else {
                page += F("Warnung");
            }
        }
        page += F(" · TEC ");
        page += String(canHardware.transmitErrorCount);
        page += F(" · REC ");
        page += String(canHardware.receiveErrorCount);
        page += F(" · EFLG 0x");
        if (canHardware.errorFlags < 0x10) {
            page += F("0");
        }
        page += String(canHardware.errorFlags, HEX);
        page += F("</td></tr>");
    }
    if (CAN_OBD_POLLING_ENABLED) {
        page += F("<tr><td>ECU-Verbindung</td><td class='");
        if (!obdPollingEnabled) {
            page += F("'>Abfrage seriell pausiert");
        } else {
            page += ecuResponding ? F("ok'>verbunden")
                                  : F("warn'>warte auf Antwort");
        }
        if (obd.lastResponseMs > 0) {
            page += F(" · ");
            page += String((millis() - obd.lastResponseMs) / 1000.0f, 1);
            page += F(" s alt");
        }
        page += F("</td></tr><tr><td>OBD-Livedaten</td><td>");
        if (obd.rpmValid) {
            page += String(obd.rpm, 0);
            page += F(" rpm");
        } else {
            page += F("- rpm");
        }
        page += F(" · ");
        if (obd.speedValid) {
            page += String(obd.speedKmh);
            page += F(" km/h");
        } else {
            page += F("- km/h");
        }
        page += F(" · ");
        if (obd.throttleValid) {
            page += String(obd.throttlePercent, 1);
            page += F(" %");
        } else {
            page += F("- %");
        }
        page += F("<br><span class='hint'>");
        page += String(obd.requestCount);
        page += F(" Anfragen · ");
        page += String(obd.responseCount);
        page += F(" Antworten · ");
        page += String(obd.requestErrors);
        page += F(" Sendefehler</span></td></tr>");
    }
    page += F("<tr><td>Kalibrierung</td><td>");
    if (bnoManager.isSelfTestPassed()) {
        CalibrationData calibration = bnoManager.getCalibration();
        if (ROADTEST_BNO_USES_MAG) {
            page += F("S");
            page += String(calibration.system);
            page += F(" G");
            page += String(calibration.gyro);
            page += F(" A");
            page += String(calibration.accel);
            page += F(" M");
            page += String(calibration.mag);
        } else {
            page += F("G");
            page += String(calibration.gyro);
            page += F(" A");
            page += String(calibration.accel);
        }
        page += bnoManager.isCalibrationSaved() ? F(" · gespeichert")
                                                : F(" · nicht gespeichert");
    } else {
        page += F("-");
    }
    page += F("</td></tr><tr><td>WLAN-Geräte</td><td>");
    page += String(WiFi.softAPgetStationNum());
    page += F("</td></tr><tr><td>Schreibfehler</td><td class='");
    page += logStats.errorCount == 0 ? F("ok'>0") : F("warn'>");
    if (logStats.errorCount > 0) {
        page += String(logStats.errorCount);
    }
    page += F("</td></tr><tr><td>Verworfene Datensätze</td><td class='");
    page += logStats.droppedLogs == 0 ? F("ok'>0") : F("warn'>");
    if (logStats.droppedLogs > 0) {
        page += String(logStats.droppedLogs);
    }
    page += F("</td></tr></table><h2>");
    if (ride.active) {
        page += F("Aktuelle Messfahrt");
    } else if (ride.interrupted) {
        page += F("Abgebrochene Messfahrt");
    } else {
        page += F("Letzte Messfahrt");
    }
    page += F("</h2>");

    if (ride.active || ride.completed || ride.interrupted) {
        page += F("<table><tr><td>Sitzung</td><td>");
        page += ride.sessionId;
        page += F("</td></tr><tr><td>Dauer</td><td>");
        page += formatDuration(ride.durationSeconds);
        page += F("</td></tr><tr><td>Strecke</td><td>");
        page += String(ride.distanceKm, 2);
        page += F(" km</td></tr><tr><td>Schlaglöcher</td><td>");
        page += String(ride.potholeCount);
        page += F("</td></tr><tr><td>Kurven</td><td>");
        page += String(ride.curveCount);
        page += F("</td></tr><tr><td>Durchschnittsqualität</td><td>");
        if (ride.qualitySamples > 0) {
            page += String(ride.averageQuality, 1);
            page += F(" / 100");
        } else {
            page += F("-");
        }
        page += F("</td></tr></table>");
        if (ride.interrupted) {
            page += F("<p class='warn'>Die Aufzeichnung wurde wegen eines "
                      "SD-Fehlers unterbrochen. ROADTEST bindet die Karte "
                      "automatisch wieder ein und sichert gepufferte Daten. "
                      "Trat der Fehler während der Fahrt auf, folgt eine "
                      "verknüpfte Fortsetzungssitzung.</p>");
        }
    } else {
        page += F("<p class='hint'>Noch keine Messfahrt in dieser Sitzung.</p>");
    }

    if (!sdLogger.isReady()) {
        page += F("<p><strong>Keine Messfahrt möglich: SD-Karte nicht bereit.</strong></p>");
    } else if (sdLogger.isLoggingStartPending()) {
        page += F("<p><strong>Aufzeichnung wird vorbereitet …</strong></p>");
    } else if (ride.active) {
        page += F("<form method='POST' action='/ride/stop'>"
                  "<button class='stop' type='submit'>Aufzeichnung beenden</button></form>");
    } else {
        page += F("<form method='POST' action='/ride/start'>"
                  "<button type='submit'>Aufzeichnung starten</button></form>");
    }

    if (sdLogger.getLastStartError().length() > 0) {
        page += F("<p class='warn'>Letzter Startfehler: ");
        page += sdLogger.getLastStartError();
        page += F("</p>");
    }

    page += F("<p class='hint'>Start und Ende sind mit den Admin-Zugangsdaten geschützt.</p>"
              "<a href='/curve-test'>Beifahrer-Kurventest</a>"
              " &nbsp; <a href='/calibration'>Kalibrierung</a>"
              " &nbsp; <a href='/update'>Firmware aktualisieren</a>"
              "</body></html>");
    return page;
}

const char* WebManager::getCurveReferenceName(
    CurveReferenceType type) const {
    switch (type) {
        case CurveReferenceType::STRAIGHT:
            return "GERADE";
        case CurveReferenceType::LEFT_WIDE:
            return "LINKS_WEIT";
        case CurveReferenceType::RIGHT_WIDE:
            return "RECHTS_WEIT";
        case CurveReferenceType::LEFT_NORMAL:
            return "LINKS_NORMAL";
        case CurveReferenceType::RIGHT_NORMAL:
            return "RECHTS_NORMAL";
        case CurveReferenceType::S_CURVE:
            return "S_KURVE";
        default:
            return "KEINE";
    }
}

WebManager::CurveReferenceType WebManager::parseCurveReferenceType(
    const String& value) const {
    if (value == "straight") return CurveReferenceType::STRAIGHT;
    if (value == "left-wide") return CurveReferenceType::LEFT_WIDE;
    if (value == "right-wide") return CurveReferenceType::RIGHT_WIDE;
    if (value == "left-normal") return CurveReferenceType::LEFT_NORMAL;
    if (value == "right-normal") return CurveReferenceType::RIGHT_NORMAL;
    if (value == "s-curve") return CurveReferenceType::S_CURVE;
    return CurveReferenceType::NONE;
}

void WebManager::redirectCurveTest(const char* result) {
    String location = "/curve-test";
    if (result != nullptr) {
        location += "?result=";
        location += result;
    }
    server.sendHeader("Location", location, true);
    server.sendHeader("Cache-Control", "no-store");
    server.send(303, "text/plain; charset=utf-8", "");
}

String WebManager::buildCurveTestStatusJSON() {
    const OBDLiveData obd = canReader.getOBDData();
    const GPSData gps = gpsManager.getCurrentData(false);
    const RideSummary ride = sdLogger.getRideSummary();
    const LogStats sd = sdLogger.getStatistics();
    String json;
    json.reserve(360);
    json += F("{\"active\":");
    json += curveTestActive ? F("true") : F("false");
    json += F(",\"recording\":");
    json += sdLogger.isLogging() ? F("true") : F("false");
    json += F(",\"preparing\":");
    json += sdLogger.isLoggingStartPending() ? F("true") : F("false");
    json += F(",\"referenceActive\":");
    json += curveReferenceActive ? F("true") : F("false");
    json += F(",\"reference\":\"");
    json += getCurveReferenceName(curveReferenceType);
    json += F("\",\"obdSpeed\":");
    json += obd.speedValid ? String(obd.speedKmh) : String(-1);
    json += F(",\"gpsSpeed\":");
    json += gps.speed_valid ? String(gps.speed_kmh, 1) : String(-1);
    json += F(",\"distanceKm\":");
    json += String(ride.distanceKm, 3);
    json += F(",\"detectedCurves\":");
    json += String(ride.curveCount);
    json += F(",\"sdErrors\":");
    json += String(sd.errorCount);
    json += F("}");
    return json;
}

String WebManager::buildCurveTestPage(const String& message) {
    const bool recording = sdLogger.isLogging();
    const bool preparing = sdLogger.isLoggingStartPending();
    const RideSummary ride = sdLogger.getRideSummary();
    const OBDLiveData obd = canReader.getOBDData();
    const GPSData gps = gpsManager.getCurrentData(false);
    const uint16_t curveWideCount =
        curveLeftWideCount + curveRightWideCount;
    const uint16_t curveNormalCount =
        curveLeftNormalCount + curveRightNormalCount;
    const bool curveReferencesComplete =
        curveStraightCount >= 1 &&
        curveLeftWideCount >= 2 && curveRightWideCount >= 2 &&
        curveLeftNormalCount >= 2 && curveRightNormalCount >= 2 &&
        curveSCount >= 3;

    String page;
    page.reserve(8600);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1,"
        "viewport-fit=cover'><title>ROADTEST · Kurventest</title><style>"
        ":root{--road:#111713;--paper:#f0eddf;--acid:#dcf34f;"
        "--orange:#ff6b35;--mint:#33d69f;--muted:#777d75;--red:#c53b33}"
        "*{box-sizing:border-box}body{margin:0;background:var(--road);"
        "color:var(--paper);font-family:'Trebuchet MS','Avenir Next',sans-serif}"
        "body:before{content:'';position:fixed;inset:0;pointer-events:none;"
        "background:repeating-linear-gradient(115deg,transparent 0 56px,"
        "rgba(255,255,255,.025) 57px 59px)}main{width:min(48rem,100%);"
        "margin:auto;padding:1rem 1rem 3rem}header{padding:1rem 0 1.25rem;"
        "border-bottom:2px solid #485048}.brand{font-size:.72rem;font-weight:900;"
        "letter-spacing:.2em;color:var(--acid)}h1{font-size:clamp(2.8rem,13vw,"
        "5.7rem);line-height:.82;letter-spacing:-.065em;margin:1rem 0 .8rem}"
        ".lead{max-width:37rem;color:#b9c0b8;line-height:1.48;margin:0}"
        ".notice{margin:1rem 0;padding:1rem;border:2px solid var(--orange);"
        "background:#2a1c15}.dash{display:grid;grid-template-columns:repeat(2,"
        "1fr);gap:.55rem;margin:1rem 0}.metric{border:1px solid #465047;"
        "padding:.75rem;background:#18201b}.metric span{display:block;"
        "color:#8f9b91;font-size:.62rem;font-weight:900;letter-spacing:.12em;"
        "text-transform:uppercase}.metric b{display:block;font-size:1.12rem;"
        "margin-top:.3rem}.task{margin:1.4rem 0;padding:1.15rem;background:"
        "var(--paper);color:var(--road);box-shadow:8px 8px 0 var(--acid)}"
        ".step{font-size:.7rem;font-weight:900;letter-spacing:.17em;color:"
        "#596158}.task h2{font-size:clamp(1.65rem,7vw,2.5rem);line-height:1;"
        "margin:.45rem 0}.task p{line-height:1.5;margin:.5rem 0;color:#4e554f}"
        ".buttons{display:grid;gap:.75rem;margin-top:1.15rem}form{margin:0}"
        "button{width:100%;min-height:7rem;border:3px solid var(--road);"
        "background:var(--acid);color:var(--road);font:900 1.35rem "
        "'Trebuchet MS','Avenir Next',sans-serif;padding:1rem;"
        "box-shadow:0 7px 0 #000;touch-action:manipulation}"
        "button:active{transform:translateY(5px);box-shadow:0 2px 0 #000}"
        "button.end{min-height:9rem;background:var(--orange);font-size:1.7rem}"
        "button.finish{background:var(--mint)}button.stop{background:var(--red);"
        "color:white;min-height:5rem}.pair{display:grid;grid-template-columns:"
        "1fr 1fr;gap:.75rem}.progress{display:grid;grid-template-columns:"
        "repeat(4,1fr);gap:.35rem;margin:1rem 0}.progress div{text-align:center;"
        "padding:.55rem .2rem;border:1px solid #465047;color:#aab2aa}"
        ".progress b{display:block;color:white;font-size:1.2rem}.connection{"
        "font-size:.8rem;color:#8f9b91}.connection.bad{color:#ff867b;font-weight:"
        "900}.foot{border-top:1px solid #465047;margin-top:1.6rem;padding-top:"
        "1rem;color:#9aa39a;line-height:1.5;font-size:.85rem}a{color:var(--acid);"
        "font-weight:900}@media(min-width:42rem){.dash{grid-template-columns:"
        "repeat(4,1fr)}}@media(max-width:24rem){.pair{grid-template-columns:1fr}}"
        "</style></head><body><main><header><div class='brand'>ROADTEST / "
        "BEIFAHRER-MODUS</div><h1>Kurven<br>fangen.</h1><p class='lead'>"
        "Nur der Beifahrer bedient diese Seite. Markiert wird genau am "
        "physischen Beginn und Ende des Straßenverlaufs — nicht schon bei "
        "der Planung der nächsten Kurve.</p></header>");

    if (message.length() > 0) {
        page += F("<div class='notice'>");
        page += escapeHTML(message);
        page += F("</div>");
    }

    page += F("<section class='dash'><div class='metric'><span>Aufzeichnung"
              "</span><b id='recording'>");
    page += recording ? F("AKTIV") : (preparing ? F("STARTET") : F("AUS"));
    page += F("</b></div><div class='metric'><span>Geschwindigkeit</span><b "
              "id='speed'>");
    if (obd.speedValid) {
        page += String(obd.speedKmh) + " km/h";
    } else if (gps.speed_valid) {
        page += String(gps.speed_kmh, 1) + " km/h";
    } else {
        page += F("–");
    }
    page += F("</b></div><div class='metric'><span>Firmware-Kurven</span><b "
              "id='curves'>");
    page += String(ride.curveCount);
    page += F("</b></div><div class='metric'><span>Strecke</span><b "
              "id='distance'>");
    page += String(ride.distanceKm, 2);
    page += F(" km</b></div></section><section class='progress'>");
    page += F("<div>Gerade<b>");
    page += String(curveStraightCount);
    page += F("/1</b></div><div>Weit<b>");
    page += String(curveWideCount);
    page += F("/4</b></div><div>Normal<b>");
    page += String(curveNormalCount);
    page += F("/4</b></div><div>S-Kurve<b>");
    page += String(curveSCount);
    page += F("/3</b></div></section>");

    if (!curveTestActive) {
        page += F("<section class='task'><div class='step'>START</div>"
                  "<h2>Alles aus einer Hand</h2><p>Dieser Knopf startet die "
                  "Aufzeichnung. Eine vorher separat gestartete Messung ist "
                  "nicht nötig.</p><form method='POST' action='/curve-test/start'>"
                  "<button type='submit'>Kurventest & Aufzeichnung starten"
                  "</button></form></section>");
    } else if (!recording) {
        page += F("<section class='task'><div class='step'>VORBEREITUNG</div>"
                  "<h2>Dateien werden geöffnet</h2><p>Noch nicht losfahren. "
                  "Die Seite wechselt automatisch, sobald die Messung bereit "
                  "ist.</p></section>");
    } else if (curveReferenceActive) {
        page += F("<section class='task'><div class='step'>REFERENZ LÄUFT</div>"
                  "<h2>");
        page += getCurveReferenceName(curveReferenceType);
        page += F("</h2><p>Am tatsächlichen Ende des geraden Abschnitts oder "
                  "der Kurve drücken. Alle Startknöpfe bleiben bis dahin "
                  "ausgeblendet.</p><form method='POST' action="
                  "'/curve-test/reference-end'><button class='end' type="
                  "'submit'>JETZT ENDE MARKIEREN</button></form></section>");
    } else {
        if (!curveReferencesComplete) {
            page += F("<p class='lead'>Wähle immer die Kurve, die als Nächstes "
                      "tatsächlich kommt. Die Reihenfolge ist frei; erledigte "
                      "Richtungen verschwinden automatisch.</p>");
        }
        if (curveStraightCount < 1) {
            page += F("<section class='task'><div class='step'>OFFEN / GERADE "
                      "REFERENZ</div><h2>45–60 Sekunden gerade</h2><p>Erst am "
                      "Beginn eines wirklich geraden Abschnitts drücken. Die "
                      "Gerade kann zu jedem Zeitpunkt der Fahrt markiert "
                      "werden.</p><form method='POST' action="
                      "'/curve-test/reference-start'><input type='hidden' "
                      "name='type' value='straight'><button type='submit'>"
                      "GERADE BEGINNT JETZT</button></form></section>");
        }
        if (curveLeftWideCount < 2 || curveRightWideCount < 2) {
            page += F("<section class='task'><div class='step'>OFFEN / WEITE "
                      "KURVEN · LINKS ");
            page += String(curveLeftWideCount);
            page += F("/2 · RECHTS ");
            page += String(curveRightWideCount);
            page += F("/2</div><h2>Langgezogene Bögen</h2><p>Am wirklichen "
                      "Einlenkpunkt drücken; ideal sind ungefähr zehn Sekunden "
                      "oder länger.</p><div class='buttons pair'>");
            if (curveLeftWideCount < 2) {
                page += F("<form method='POST' action="
                          "'/curve-test/reference-start'><input type='hidden' "
                          "name='type' value='left-wide'><button type='submit'>"
                          "← WEIT LINKS BEGINNT</button></form>");
            }
            if (curveRightWideCount < 2) {
                page += F("<form method='POST' action="
                          "'/curve-test/reference-start'><input type='hidden' "
                          "name='type' value='right-wide'><button type='submit'>"
                          "WEIT RECHTS BEGINNT →</button></form>");
            }
            page += F("</div></section>");
        }
        if (curveLeftNormalCount < 2 || curveRightNormalCount < 2) {
            page += F("<section class='task'><div class='step'>OFFEN / NORMAL "
                      "ODER ENG · LINKS ");
            page += String(curveLeftNormalCount);
            page += F("/2 · RECHTS ");
            page += String(curveRightNormalCount);
            page += F("/2</div><h2>Deutliches Einlenken</h2><p>Normale oder "
                      "engere Straßenkurven markieren. Sicher und normal "
                      "fahren; keine besonderen Manöver provozieren.</p><div "
                      "class='buttons pair'>");
            if (curveLeftNormalCount < 2) {
                page += F("<form method='POST' action="
                          "'/curve-test/reference-start'><input type='hidden' "
                          "name='type' value='left-normal'><button type='submit'>"
                          "← NORMAL LINKS BEGINNT</button></form>");
            }
            if (curveRightNormalCount < 2) {
                page += F("<form method='POST' action="
                          "'/curve-test/reference-start'><input type='hidden' "
                          "name='type' value='right-normal'><button type='submit'>"
                          "NORMAL RECHTS BEGINNT →</button></form>");
            }
            page += F("</div></section>");
        }
        if (curveSCount < 3) {
            page += F("<section class='task'><div class='step'>OFFEN / S-KURVEN · ");
            page += String(curveSCount);
            page += F("/3</div><h2>Beide Hälften als ein Intervall</h2><p>Vor "
                      "der ersten Hälfte starten und erst nach der zweiten "
                      "Hälfte beenden.</p><form method='POST' action="
                      "'/curve-test/reference-start'><input type='hidden' "
                      "name='type' value='s-curve'><button type='submit'>"
                      "S-KURVE BEGINNT JETZT</button></form></section>");
        }
        if (curveReferencesComplete) {
            page += F("<section class='task'><div class='step'>VOLLSTÄNDIG</div>"
                      "<h2>Referenzen im Kasten</h2><p>Wenn es sich anbietet, "
                      "kann die Fahrt noch einige Minuten ohne Bedienung "
                      "weiterlaufen. Danach Dateien sauber schließen.</p>"
                      "<form method='POST' action='/curve-test/end'><button "
                      "class='finish' type='submit'>TEST BEENDEN & SPEICHERN"
                      "</button></form></section>");
        }
    }

    if (curveTestActive && !curveReferenceActive &&
        !curveReferencesComplete) {
        page += F("<details><summary>Test vorzeitig beenden</summary><form "
                  "method='POST' action='/curve-test/end'><button class='stop' "
                  "type='submit'>Vorzeitig beenden und Dateien schließen"
                  "</button></form></details>");
    }
    page += F("<p id='connection' class='connection'>Statusverbindung aktiv"
              "</p><p class='foot'>Keine Bedienung durch den Fahrer. Die "
              "Marker werden ohne sofortigen SD-Flush geschrieben, damit der "
              "Knopfdruck keine künstliche Messpause erzeugt.<br><br><a "
              "href='/'>← Systemstatus</a></p></main><script>const initial="
              "{active:");
    page += curveTestActive ? F("true") : F("false");
    page += F(",recording:");
    page += recording ? F("true") : F("false");
    page += F(",reference:");
    page += curveReferenceActive ? F("true") : F("false");
    page += F("};function t(id,v){const e=document.getElementById(id);if(e)e."
              "textContent=v}async function poll(){try{const r=await fetch("
              "'/curve-test/status',{cache:'no-store'});if(!r.ok)throw 0;const "
              "s=await r.json();t('recording',s.recording?'AKTIV':(s.preparing?"
              "'STARTET':'AUS'));const v=s.obdSpeed>=0?s.obdSpeed:s.gpsSpeed;"
              "t('speed',v>=0?v+' km/h':'–');t('curves',s.detectedCurves);"
              "t('distance',s.distanceKm."
              "toFixed(2)+' km');const c=document.getElementById('connection');"
              "c.textContent='Statusverbindung aktiv';c.className='connection';"
              "if(s.active!==initial.active||s.recording!==initial.recording||"
              "s.referenceActive!==initial.reference)location.reload()}catch(e){"
              "const c=document.getElementById('connection');c.textContent="
              "'Statusverbindung unterbrochen';c.className='connection bad'}}"
              "poll();setInterval(poll,2000);document.querySelectorAll('form')."
              "forEach(f=>f.addEventListener('submit',()=>{const b=f.querySelector"
              "('button');if(b){b.disabled=true;b.textContent='WIRD GESPEICHERT …'"
              "}}));</script></body></html>");
    return page;
}


String WebManager::buildCalibrationPage(const String& message) {
    CalibrationData calibration = bnoManager.getCalibration();
    const bool bnoReady = bnoManager.isSelfTestPassed();
    const BNO055RuntimeStatus runtimeStatus = bnoManager.getRuntimeStatus();
    const bool complete = calibration.isFullyCalibrated();
    const bool saved = bnoManager.isCalibrationSaved();
    const uint8_t activeStep =
        calibration.gyro < 3
            ? 1
            : (calibration.accel < 3
                   ? 2
                   : (ROADTEST_BNO_USES_MAG && calibration.mag < 3
                          ? 3
                          : (ROADTEST_BNO_USES_MAG &&
                                     calibration.system < 3
                                 ? 4
                                 : 5)));
    const uint8_t calibrationSum =
        calibration.gyro + calibration.accel +
        (ROADTEST_BNO_USES_MAG
             ? calibration.mag + calibration.system
             : 0);
    const uint8_t calibrationMaximum =
        ROADTEST_BNO_USES_MAG ? 12 : 6;
    const uint8_t progress =
        calibrationMaximum > 0
            ? calibrationSum * 100 / calibrationMaximum
            : 0;

    String page;
    page.reserve(6500);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='2;url=/calibration'>"
        "<title>ROADTEST Kalibrierung</title>"
        "<style>"
        ":root{--ink:#15221c;--muted:#66736c;--paper:#f4f1e8;"
        "--panel:#fffdf7;--line:#d8d5ca;--green:#17724a;--lime:#cce34d;"
        "--red:#a83d32}"
        "*{box-sizing:border-box}body{margin:0;background:var(--paper);"
        "color:var(--ink);font-family:'Avenir Next','Trebuchet MS',sans-serif}"
        "main{max-width:42rem;margin:auto;padding:1.2rem 1rem 2.5rem}"
        "header{border-bottom:3px solid var(--ink);padding:.6rem 0 1rem;"
        "margin-bottom:1rem}.brand{font-size:.72rem;font-weight:800;"
        "letter-spacing:.18em}.brand span{background:var(--lime);padding:.22rem .42rem}"
        "h1{font-size:clamp(2rem,9vw,3.6rem);line-height:.92;margin:1rem 0 .45rem;"
        "letter-spacing:-.055em}.lead{color:var(--muted);margin:0;max-width:34rem}"
        ".notice{border:1px solid var(--line);background:var(--panel);"
        "padding:.85rem 1rem;margin:1rem 0}.notice.ok{border-left:5px solid var(--green)}"
        ".notice.warn{border-left:5px solid var(--red)}"
        ".progress-head{display:flex;justify-content:space-between;align-items:end;"
        "margin:1.4rem 0 .45rem}.progress-head strong{font-size:1.6rem}"
        ".progress-track{height:.7rem;background:#dedbd0;overflow:hidden}"
        ".progress-bar{height:100%;background:var(--green)}"
        ".step{display:grid;grid-template-columns:2.7rem 1fr auto;gap:.8rem;"
        "align-items:start;border-top:1px solid var(--line);padding:1.05rem .2rem}"
        ".step.current{background:var(--ink);color:#fff;border:0;padding:1.15rem;"
        "margin:.55rem -.2rem;box-shadow:0 8px 0 var(--lime)}"
        ".step.waiting{color:#818a85}.step-number{width:2.35rem;height:2.35rem;"
        "border:2px solid currentColor;border-radius:50%;display:grid;place-items:center;"
        "font-weight:900}.step.done .step-number{background:var(--green);"
        "border-color:var(--green);color:#fff}.eyebrow{font-size:.65rem;font-weight:900;"
        "letter-spacing:.16em;color:var(--green)}.current .eyebrow{color:var(--lime)}"
        ".step h2{font-size:1.12rem;margin:.08rem 0 .25rem}.step p{font-size:.9rem;"
        "line-height:1.42;margin:0;max-width:28rem}.score{display:flex;align-items:baseline;"
        "gap:.16rem;font-size:.72rem;font-weight:800}.score strong{font-size:1.7rem}"
        ".score span{color:var(--muted)}.current .score span{color:#b7c0bb}"
        ".finish{background:var(--green);color:#fff;padding:1.25rem;margin:1.2rem 0}"
        ".finish h2{margin:0 0 .35rem}.finish p{margin:.25rem 0}"
        "button{width:100%;border:0;background:var(--ink);color:#fff;font:inherit;"
        "font-weight:800;padding:.9rem 1rem;margin:.6rem 0;cursor:pointer}"
        "button:active{transform:translateY(1px)}details{border-top:1px solid var(--line);"
        "border-bottom:1px solid var(--line);padding:.8rem 0;margin:1.2rem 0}"
        "summary{font-weight:800;cursor:pointer}.diagnostics{display:grid;"
        "grid-template-columns:1fr auto;gap:.45rem 1rem;font-size:.88rem;margin-top:.8rem}"
        ".diagnostics span:nth-child(even){font-weight:800;text-align:right}"
        "a{color:var(--ink);font-weight:800}.footer{margin-top:1.5rem;color:var(--muted);"
        "font-size:.85rem}@media(max-width:28rem){.step{grid-template-columns:2.4rem 1fr}"
        ".score{grid-column:2}.step.current{margin:.55rem 0}}"
        "</style></head><body><main><header><div class='brand'>"
        "<span>ROADTEST</span> · SENSOR-ASSISTENT</div>"
        "<h1>Kalibrierung</h1><p class='lead'>Die Anzeige aktualisiert sich "
        "alle zwei Sekunden und führt automatisch zum nächsten Schritt.</p></header>");

    if (message.length() > 0) {
        const bool messageIsWarning =
            message.startsWith("Noch nicht") ||
            message.indexOf("konnte nicht") >= 0;
        page += messageIsWarning
            ? F("<div class='notice warn'><strong>")
            : F("<div class='notice ok'><strong>");
        page += message;
        page += F("</strong></div>");
    }

    if (!bnoReady) {
        page += F(
            "<div class='notice warn'><strong>BNO055 ist nicht verbunden.</strong>"
            "<p>Die Firmware sucht den Sensor automatisch erneut. Sobald er "
            "antwortet, erscheint hier der erste Kalibrierungsschritt.</p></div>"
            "<p class='footer'><a href='/'>← Zur Statusseite</a></p>"
            "</main></body></html>");
        return page;
    }

    if (sdLogger.isLogging()) {
        page += F(
            "<div class='notice warn'><strong>Aufzeichnung läuft.</strong>"
            "<p>Bitte zuerst auf der Statusseite beenden. Die Bewegungen zur "
            "Kalibrierung würden sonst als Fahrbahndaten gespeichert.</p></div>");
    }

    if (!runtimeStatus.isFusionRunning()) {
        page += F(
            "<div class='notice warn'><strong>Sensorfusion noch nicht bereit.</strong>"
            "<p>Bitte das Gerät ruhig liegen lassen. Die Firmware stellt den "
            "korrekten Modus automatisch wieder her; ein manueller Neustart "
            "ist nicht erforderlich.</p></div>");
    }

    page += F("<div class='progress-head'><div><strong>");
    page += String(progress);
    page += F("%</strong><br><span>Gesamtfortschritt</span></div><span>");
    page += complete ? F("bereit") : F("in Arbeit");
    page += F("</span></div><div class='progress-track'><div class='progress-bar' style='width:");
    page += String(progress);
    page += F("%'></div></div>");

    appendCalibrationStep(
        page, 1, "Ruhig halten", "GYR", calibration.gyro,
        calibration.gyro == 3
            ? CalibrationStepState::DONE
            : (activeStep == 1 ? CalibrationStepState::CURRENT
                               : CalibrationStepState::WAITING),
        "Das ROADTEST-Gerät auf eine feste Fläche legen und einige Sekunden "
        "vollständig stillhalten. Nicht berühren, bis GYR 3 erreicht.");

    appendCalibrationStep(
        page, 2, "Sechs Seiten", "ACC", calibration.accel,
        calibration.accel == 3
            ? CalibrationStepState::DONE
            : (activeStep == 2 ? CalibrationStepState::CURRENT
                               : CalibrationStepState::WAITING),
        "Das Gerät nacheinander flach auf Oberseite, Unterseite und alle vier "
        "Seiten legen. Jede Lage ruhig halten, bis der Wert weitersteigt.");

    if (ROADTEST_BNO_USES_MAG) {
        appendCalibrationStep(
            page, 3, "Liegende Acht", "MAG", calibration.mag,
            calibration.mag == 3
                ? CalibrationStepState::DONE
                : (activeStep == 3 ? CalibrationStepState::CURRENT
                                   : CalibrationStepState::WAITING),
            "Abseits größerer Metallteile langsam eine liegende Acht in der "
            "Luft beschreiben und das Gerät dabei um alle Achsen drehen.");

        appendCalibrationStep(
            page, 4, "Sensorfusion bestätigen", "SYS",
            calibration.system,
            calibration.system == 3
                ? CalibrationStepState::DONE
                : (activeStep == 4 ? CalibrationStepState::CURRENT
                                   : CalibrationStepState::WAITING),
            "SYS ist kein eigener Sensor, sondern das Gesamtvertrauen der "
            "Fusion. Die ruhigen Lagen und die Acht langsam wiederholen, bis "
            "auch SYS 3 anzeigt.");
    }

    if (complete) {
        page += F("<div class='finish'><h2>Kalibrierung vollständig</h2><p>");
        page += String("Alle für ") + ROADTEST_BNO_MODE_NAME +
                " benötigten Werte stehen auf 3.</p><p>";
        page += saved
            ? F("Im Speicher liegen bereits Kalibrierungswerte. Mit dem "
                "Button wird der jetzt erreichte Stand übernommen.")
            : F("Jetzt dauerhaft speichern, bevor das Gerät ausgeschaltet wird.");
        page += F("</p></div><form method='POST' action='/calibration/save'>"
                  "<button type='submit'>");
        page += saved ? F("Aktuellen Stand speichern")
                      : F("Kalibrierung dauerhaft speichern");
        page += F("</button></form><p class='footer'>Das Speichern ist mit den "
                  "Admin-Zugangsdaten geschützt.</p>");
    }

    page += F(
        "<details><summary>Technische Sensordaten</summary>"
        "<div class='diagnostics'><span>Betriebsmodus</span><span>");
    page += runtimeStatus.isExpectedModeActive()
        ? String(ROADTEST_BNO_MODE_NAME)
        : String(runtimeStatus.operationMode);
    page += F("</span><span>Sensorfusion</span><span>");
    page += runtimeStatus.isFusionRunning() ? F("läuft") : F("läuft nicht");
    page += F("</span><span>Systemstatus</span><span>");
    page += String(runtimeStatus.systemStatus);
    page += F(" / erwartet 5</span><span>Fehlercode</span><span>");
    page += String(runtimeStatus.systemError);
    page += F(" / erwartet 0</span><span>Taktquelle</span><span>intern</span>"
              "<span>Gespeicherte Werte</span><span>");
    page += saved ? F("vorhanden") : F("noch keine");
    page += F(
        "</span></div></details>"
        "<p class='footer'>Die Seite aktualisiert sich automatisch. "
        "Ein Sensorneustart ist für die Kalibrierung nicht notwendig.</p>"
        "<p><a href='/'>← Zur Statusseite</a></p></main></body></html>");
    return page;
}

String WebManager::buildUpdatePage() {
    String page;
    page.reserve(3200);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ROADTEST Update</title>"
        "<style>:root{--ink:#15221c;--paper:#f4f1e8;--panel:#fffdf7;"
        "--line:#d8d5ca;--lime:#cce34d;--muted:#66736c}"
        "*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);"
        "font-family:'Avenir Next','Trebuchet MS',sans-serif}"
        "main{max-width:40rem;margin:auto;padding:1.4rem 1rem 2.5rem}"
        ".brand{font-size:.72rem;font-weight:900;letter-spacing:.18em;"
        "border-bottom:3px solid var(--ink);padding-bottom:.9rem}"
        ".brand span{background:var(--lime);padding:.22rem .42rem}"
        "h1{font-size:clamp(2rem,9vw,3.5rem);line-height:.95;letter-spacing:-.05em;"
        "margin:1.2rem 0}.version{display:grid;grid-template-columns:1fr auto;"
        "align-items:center;background:var(--ink);color:#fff;padding:1rem 1.1rem;"
        "box-shadow:0 7px 0 var(--lime)}.version small{letter-spacing:.12em;"
        "font-weight:800}.version strong{font-size:1.8rem}.file{border:1px solid var(--line);"
        "background:var(--panel);padding:1rem;margin:1.5rem 0}.file code{display:block;"
        "font-size:1rem;font-weight:800;margin-top:.35rem;overflow-wrap:anywhere}"
        "input{width:100%;font:inherit;border:1px solid var(--line);background:#fff;"
        "padding:.8rem;margin:.5rem 0 1rem}button{width:100%;border:0;"
        "background:var(--ink);color:#fff;font:inherit;font-weight:900;padding:.95rem}"
        ".hint{color:var(--muted);font-size:.9rem;line-height:1.5}"
        "a{color:var(--ink);font-weight:800}</style></head><body><main>"
        "<div class='brand'><span>ROADTEST</span> · FIRMWARE</div>"
        "<h1>Update</h1><div class='version'><small>AKTUELL INSTALLIERT</small><strong>");
    page += ROADTEST_FIRMWARE_VERSION;
    page += F(
        "</strong></div><div class='file'>Offizieller Dateiname dieses Builds:"
        "<code>");
    page += ROADTEST_FIRMWARE_FILE_NAME;
    page += F(
        "</code></div><p>Eine für das LOLIN S3 Mini erzeugte, versionierte "
        "<code>.bin</code>-Datei auswählen.</p>");
    const bool measurementActive =
        vehicleDataDiscovery.isActive() ||
        sdLogger.isLogging() ||
        sdLogger.isLoggingStartPending();
    if (measurementActive) {
        page += F(
            "<div class='file'><strong>Update gesperrt</strong><p>Bitte die "
            "laufende Messung zuerst sicher beenden. Dadurch bleiben Sitzung, "
            "Marker und Abschlussdateien vollständig.</p></div>");
    } else {
        page += F(
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='firmware' accept='.bin' required>"
            "<button type='submit'>Update starten</button></form>");
    }
    page += F(
        "<p class='hint'>Ein Update startet nur bei vollständig beendeter "
        "Aufzeichnung. Nach dem Neustart zeigt diese Seite die Version der "
        "neu installierten Firmware.</p>"
        "<p><a href='/'>← Abbrechen</a></p></main></body></html>");
    return page;
}

String WebManager::buildUpdateResultPage(bool success) {
    String page;
    page.reserve(1400);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ROADTEST Update</title><style>body{font-family:'Avenir Next',"
        "'Trebuchet MS',sans-serif;max-width:34rem;margin:2rem auto;"
        "padding:0 1rem;background:#f4f1e8;color:#15221c}"
        ".result{border-left:6px solid #17724a;background:#fffdf7;padding:1rem}"
        ".error{border-left-color:#a83d32}code{font-weight:800;"
        "overflow-wrap:anywhere}a{color:#15221c;font-weight:800}</style>"
        "</head><body><div class='result ");
    page += success ? F("'>") : F("error'>");
    page += success ? F("<h1>Update erfolgreich</h1>")
                    : F("<h1>Update fehlgeschlagen</h1>");
    page += F("<p>Hochgeladene Datei: <code>");
    page += escapeHTML(otaUploadFileName);
    page += F("</code></p>");
    if (success) {
        page += F(
            "<p>ROADTEST startet jetzt neu. Danach wieder mit dem WLAN "
            "verbinden und auf der Update-Seite die installierte "
            "Versionsnummer prüfen.</p>");
    } else {
        if (otaBlockedByMeasurement) {
            page += F(
                "<p>Das Update wurde nicht gestartet, weil noch eine Messung "
                "oder deren Vorbereitung aktiv ist. Bitte diese zuerst sicher "
                "beenden.</p>");
        }
        page += F("<p>Die bisherige Firmware ");
        page += ROADTEST_FIRMWARE_VERSION;
        page += F(
            " bleibt aktiv.</p><p><a href='/update'>← Zurück</a></p>");
    }
    page += F("</div></body></html>");
    return page;
}

bool WebManager::authenticateOTA() {
    if (server.authenticate(OTA_USERNAME, OTA_PASSWORD)) {
        return true;
    }
    server.requestAuthentication();
    return false;
}

void WebManager::handleUpdateUpload() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        otaAuthorized = server.authenticate(OTA_USERNAME, OTA_PASSWORD);
        if (!otaAuthorized) {
            return;
        }

        otaBlockedByMeasurement =
            vehicleDataDiscovery.isActive() ||
            sdLogger.isLogging() ||
            sdLogger.isLoggingStartPending();
        otaUploadRejected = otaBlockedByMeasurement;
        if (otaUploadRejected) {
            otaInProgress = false;
            otaUploadFileName = upload.filename;
            Serial.println(
                "⚠️ OTA abgelehnt: Eine Messung oder deren Vorbereitung läuft");
            return;
        }

        otaInProgress = true;
        otaUploadFileName = upload.filename;

        Serial.printf("OTA-Update gestartet: %s\n", upload.filename.c_str());
        const uint32_t maxSketchSpace =
            (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace, U_FLASH)) {
            Update.printError(Serial);
        }
    } else if (otaAuthorized && !otaUploadRejected &&
               upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (otaAuthorized && !otaUploadRejected &&
               upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("✅ OTA-Update vollständig: %u Bytes\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    } else if (otaAuthorized && !otaUploadRejected &&
               upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaUploadRejected = true;
        otaInProgress = false;
        Serial.println("⚠️ OTA-Update abgebrochen");
    }
}

void WebManager::resetOTAState() {
    otaInProgress = false;
    otaAuthorized = false;
    otaUploadRejected = false;
    otaBlockedByMeasurement = false;
}

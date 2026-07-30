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
      otaUploadFileName("") {}

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

    server.on("/acceptance", HTTP_GET, [this]() {
        String message;
        if (server.arg("blocked") == "1") {
            message =
                "Schritt nicht gespeichert. Bitte die angezeigte "
                "Voraussetzung prüfen.";
        } else if (server.arg("ended") == "1") {
            message = "Kontrolltest beendet und Dateien geschlossen.";
        }
        server.sendHeader("Cache-Control", "no-store");
        server.send(
            200, "text/html; charset=utf-8",
            buildAcceptanceTestPage(message));
    });

    server.on("/acceptance/status", HTTP_GET, [this]() {
        server.sendHeader("Cache-Control", "no-store");
        server.send(
            200, "application/json; charset=utf-8",
            buildAcceptanceStatusJSON());
    });

    server.on("/acceptance/start", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        const bool started = vehicleDataDiscovery.begin();
        redirectAcceptance(!started);
    });

    server.on("/acceptance/engine-start", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        const bool marked =
            vehicleDataDiscovery.markEngineStarted();
        if (vehicleDataDiscovery.isActive() && sdLogger.isLogging()) {
            sdLogger.logEvent(
                "WEB_ACTION",
                String("ENGINE_START;RESULT_") +
                    (marked ? "OK" : "BLOCKED"));
        }
        redirectAcceptance(!marked);
    });

    server.on("/acceptance/ignition-on", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        const bool marked =
            vehicleDataDiscovery.markIgnitionOn();
        if (vehicleDataDiscovery.isActive() && sdLogger.isLogging()) {
            sdLogger.logEvent(
                "WEB_ACTION",
                String("IGNITION_ON;RESULT_") +
                    (marked ? "OK" : "BLOCKED"));
        }
        redirectAcceptance(!marked);
    });

    server.on("/acceptance/engine-stop", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        const bool marked =
            vehicleDataDiscovery.markEngineStopped();
        if (vehicleDataDiscovery.isActive() && sdLogger.isLogging()) {
            sdLogger.logEvent(
                "WEB_ACTION",
                String("ENGINE_STOP;RESULT_") +
                    (marked ? "OK" : "BLOCKED"));
        }
        redirectAcceptance(!marked);
    });

    server.on("/acceptance/end", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (!vehicleDataDiscovery.isActive()) {
            redirectAcceptance(true);
            return;
        }
        if (sdLogger.isLogging()) {
            sdLogger.logEvent("WEB_ACTION", "END;RESULT_OK");
        }
        vehicleDataDiscovery.end();
        redirectAcceptance(false, true);
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
                "<p>Der Fahrzeug- oder Abnahmetest verwaltet die laufende "
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
              "<a href='/acceptance'>Recovery-Kontrolltest</a>"
              " &nbsp; <a href='/calibration'>Kalibrierung</a>"
              " &nbsp; <a href='/update'>Firmware aktualisieren</a>"
              "</body></html>");
    return page;
}


uint8_t WebManager::getAcceptanceStage() const {
    if (!vehicleDataDiscovery.isActive()) {
        return 0;
    }
    if (!sdLogger.isLogging() ||
        vehicleDataDiscovery.isPassiveCaptureActive()) {
        return 1;
    }

    const unsigned long ignitionOn =
        vehicleDataDiscovery.getIgnitionOnMarkedAt();
    if (ignitionOn == 0) {
        return 2;
    }
    const unsigned long ignitionDetection =
        vehicleDataDiscovery.getDetectionAfterIgnitionOnMs();
    if (ignitionDetection == UINT32_MAX) {
        return 3;
    }

    const unsigned long engineStart =
        vehicleDataDiscovery.getEngineStartMarkedAt();
    if (engineStart == 0) {
        return 4;
    }
    const unsigned long engineDetection =
        vehicleDataDiscovery.getDetectionAfterEngineStartMs();
    if (engineDetection == UINT32_MAX) {
        return 5;
    }
    const unsigned long engineDetectedAt =
        engineStart + engineDetection;
    if (millis() - engineDetectedAt <
        ACCEPTANCE_INITIAL_STAND_MS) {
        return 6;
    }

    if (vehicleDataDiscovery.getEngineStopMarkedAt() == 0) {
        const OBDLiveData obd = canReader.getOBDData();
        return obd.speedValid &&
                       obd.speedKmh <=
                           GPS_DISTANCE_OBD_STATIONARY_KMH
                   ? 8
                   : 7;
    }
    // ECU-Ausfall ist ein abgeschlossener Meilenstein. Nach erfolgreicher
    // Wiedererkennung darf die Seite nicht zurück zu "Ausfall abwarten"
    // springen, nur weil der aktuelle ECU-Zustand wieder REACHABLE ist.
    if (!vehicleDataDiscovery.hasObservedECULossAfterEngineStop()) {
        return 9;
    }

    const unsigned long ignitionRestart =
        vehicleDataDiscovery.getIgnitionRestartMarkedAt();
    if (ignitionRestart == 0) {
        return 10;
    }
    const unsigned long restartIgnitionDetection =
        vehicleDataDiscovery.getDetectionAfterIgnitionRestartMs();
    if (restartIgnitionDetection == UINT32_MAX) {
        return 11;
    }

    const unsigned long engineRestart =
        vehicleDataDiscovery.getEngineRestartMarkedAt();
    if (engineRestart == 0) {
        return 12;
    }
    const unsigned long restartEngineDetection =
        vehicleDataDiscovery.getDetectionAfterEngineRestartMs();
    if (restartEngineDetection == UINT32_MAX) {
        return 13;
    }
    const unsigned long restartDetectedAt =
        engineRestart + restartEngineDetection;
    return millis() - restartDetectedAt <
                   ACCEPTANCE_FINAL_STAND_MS
               ? 14
               : 15;
}

uint32_t WebManager::getAcceptanceCountdownSeconds(
    uint8_t stage) const {
    if (stage == 1) {
        return vehicleDataDiscovery.getPassiveRemainingSeconds();
    }
    if (stage == 6) {
        const unsigned long detectedAt =
            vehicleDataDiscovery.getEngineStartMarkedAt() +
            vehicleDataDiscovery.getDetectionAfterEngineStartMs();
        const unsigned long elapsed = millis() - detectedAt;
        return elapsed >= ACCEPTANCE_INITIAL_STAND_MS
                   ? 0
                   : (ACCEPTANCE_INITIAL_STAND_MS - elapsed + 999) /
                         1000;
    }
    if (stage == 14) {
        const unsigned long detectedAt =
            vehicleDataDiscovery.getEngineRestartMarkedAt() +
            vehicleDataDiscovery.getDetectionAfterEngineRestartMs();
        const unsigned long elapsed = millis() - detectedAt;
        return elapsed >= ACCEPTANCE_FINAL_STAND_MS
                   ? 0
                   : (ACCEPTANCE_FINAL_STAND_MS - elapsed + 999) /
                         1000;
    }
    return 0;
}

void WebManager::redirectAcceptance(bool blocked, bool ended) {
    String location = "/acceptance";
    if (blocked) {
        location += "?blocked=1";
    } else if (ended) {
        location += "?ended=1";
    }
    server.sendHeader("Location", location, true);
    server.sendHeader("Cache-Control", "no-store");
    server.send(303, "text/plain; charset=utf-8", "");
}

String WebManager::buildAcceptanceStatusJSON() {
    const uint8_t stage = getAcceptanceStage();
    const OBDLiveData obd = canReader.getOBDData();
    const GPSData gps = gpsManager.getCurrentData(false);
    const RideSummary ride = sdLogger.getRideSummary();
    const LogStats sd = sdLogger.getStatistics();
    const RuntimeTimingDiagnostics timing =
        runtimeDiagnostics.getTiming();

    String json;
    json.reserve(420);
    json += F("{\"stage\":");
    json += String(stage);
    json += F(",\"countdown\":");
    json += String(getAcceptanceCountdownSeconds(stage));
    json += F(",\"phase\":\"");
    json += escapeJSON(
        vehicleDataDiscovery.isActive()
            ? String(vehicleDataDiscovery.getPhaseName())
            : String("bereit"));
    json += F("\",\"ecu\":\"");
    json += escapeJSON(
        String(vehicleDataDiscovery.getECUStateName()));
    json += F("\",\"obdSpeed\":");
    json += obd.speedValid ? String(obd.speedKmh) : String(-1);
    json += F(",\"gpsSpeed\":");
    json += String(gps.speed_kmh, 1);
    json += F(",\"gpsSpeedValid\":");
    json += gps.speed_valid ? F("true") : F("false");
    json += F(",\"distanceKm\":");
    json += String(ride.distanceKm, 3);
    json += F(",\"sdErrors\":");
    json += String(sd.errorCount);
    json += F(",\"maxPauseMs\":");
    json += String(timing.maxLoopIntervalMs);
    json += F("}");
    return json;
}

String WebManager::buildAcceptanceTestPage(
    const String& message) {
    const uint8_t stage = getAcceptanceStage();
    const bool active = vehicleDataDiscovery.isActive();
    const OBDLiveData obd = canReader.getOBDData();
    const GPSData gps = gpsManager.getCurrentData(false);
    const RideSummary ride = sdLogger.getRideSummary();
    const RuntimeTimingDiagnostics timing =
        runtimeDiagnostics.getTiming();

    const char* eyebrow = "BEREIT";
    const char* title = "Recovery-Kontrolle starten";
    const char* instruction =
        "Zündung und Motor ausgeschaltet lassen. Der Test beginnt mit "
        "60 Sekunden echtem Listen-Only.";
    const char* action = "/acceptance/start";
    const char* button = "Kontrolltest starten";
    bool showAction = true;

    switch (stage) {
        case 1:
            eyebrow = "VORBEREITUNG";
            title = "Passive Phase abwarten";
            instruction =
                "Die Firmware bereitet die Dateien vor und sendet 60 "
                "Sekunden lang keine CAN-Anfrage.";
            showAction = false;
            break;
        case 2:
            eyebrow = "ZÜNDUNG";
            title = "Zündung einschalten";
            instruction =
                "Knopf zuerst drücken. Danach nur die Zündung "
                "einschalten, den Motor noch nicht starten.";
            action = "/acceptance/ignition-on";
            button = "Markieren · dann Zündung an";
            break;
        case 3:
            eyebrow = "ECU";
            title = "Erste ECU-Antwort abwarten";
            instruction =
                "Die Seite wechselt automatisch weiter, sobald eine "
                "Antwort nach dem Marker vorliegt.";
            showAction = false;
            break;
        case 4:
            eyebrow = "MOTORSTART";
            title = "Motor starten";
            instruction =
                "Knopf zuerst drücken und den Motor unmittelbar danach "
                "starten.";
            action = "/acceptance/engine-start";
            button = "Markieren · dann Motor starten";
            break;
        case 5:
            eyebrow = "MOTORLAUF";
            title = "Drehzahlbestätigung abwarten";
            instruction =
                "Mindestens 300 U/min müssen als frischer OBD-Wert "
                "erkannt werden.";
            showAction = false;
            break;
        case 6:
            eyebrow = "STABILITÄT";
            title = "Eine Minute laufen lassen";
            instruction =
                "Fahrzeug bleibt vollständig stehen. Die kleine "
                "Statusabfrage bleibt währenddessen aktiv.";
            showAction = false;
            break;
        case 7:
            eyebrow = "STILLSTAND";
            title = "Auf OBD 0 km/h warten";
            instruction =
                "Der Ausschaltknopf erscheint erst bei einer frischen "
                "Stillstandsbestätigung.";
            showAction = false;
            break;
        case 8:
            eyebrow = "AUSSCHALTEN";
            title = "Motor und Zündung ausschalten";
            instruction =
                "Knopf zuerst drücken und das Fahrzeug unmittelbar "
                "danach vollständig ausschalten.";
            action = "/acceptance/engine-stop";
            button = "Markieren · dann Motor und Zündung aus";
            break;
        case 9:
            eyebrow = "RECOVERY";
            title = "ECU-Verlust abwarten";
            instruction =
                "Der Zustand wechselt erst nach drei tatsächlich "
                "fehlgeschlagenen OBD-Anfragen.";
            showAction = false;
            break;
        case 10:
            eyebrow = "WIEDERANLAUF";
            title = "Zündung erneut einschalten";
            instruction =
                "Knopf zuerst drücken. Danach nur die Zündung "
                "einschalten.";
            action = "/acceptance/ignition-on";
            button = "Markieren · dann Zündung erneut an";
            break;
        case 11:
            eyebrow = "WIEDERVERBINDUNG";
            title = "ECU-Antwort abwarten";
            instruction =
                "Die Wiedererkennung läuft ohne Firmware-Neustart.";
            showAction = false;
            break;
        case 12:
            eyebrow = "NEUSTART";
            title = "Motor erneut starten";
            instruction =
                "Knopf zuerst drücken und den Motor unmittelbar danach "
                "starten.";
            action = "/acceptance/engine-start";
            button = "Markieren · dann Motor neu starten";
            break;
        case 13:
            eyebrow = "MOTORLAUF";
            title = "Neue Drehzahlbestätigung abwarten";
            instruction =
                "Die Firmware wartet auf eine frische Drehzahl von "
                "mindestens 300 U/min.";
            showAction = false;
            break;
        case 14:
            eyebrow = "ABSCHLUSSKONTROLLE";
            title = "Zwei Minuten stabil laufen lassen";
            instruction =
                "Die Statusabfrage bleibt aktiv. Es dürfen keine langen "
                "Messpausen oder GPS-Pufferverluste entstehen.";
            showAction = false;
            break;
        case 15:
            eyebrow = "FERTIG";
            title = "Kontrolltest abschließen";
            instruction =
                "Alle vorgesehenen Recovery-Schritte sind vollständig.";
            action = "/acceptance/end";
            button = "Test abschließen und Dateien schließen";
            break;
    }

    String page;
    page.reserve(7200);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1,"
        "viewport-fit=cover'><title>ROADTEST · Recovery</title><style>"
        ":root{--paper:#f1efe7;--ink:#17201b;--muted:#657069;"
        "--lime:#d9ed58;--green:#087a46;--amber:#d88716;--red:#b93830}"
        "*{box-sizing:border-box}body{margin:0;background:var(--paper);"
        "color:var(--ink);font-family:'Avenir Next','Trebuchet MS',sans-serif}"
        "main{width:min(42rem,100%);margin:auto;padding:1rem 1rem 3rem}"
        "header{border-bottom:4px solid var(--ink);padding:.8rem 0 1rem}"
        ".brand{font-size:.68rem;font-weight:900;letter-spacing:.18em}"
        ".brand span{background:var(--lime);padding:.2rem .45rem}"
        "h1{font-size:clamp(2.4rem,11vw,4.5rem);line-height:.88;"
        "letter-spacing:-.06em;margin:1rem 0 .5rem}.lead{color:var(--muted);"
        "line-height:1.45;margin:0}.notice{margin:1rem 0;padding:.8rem 1rem;"
        "border-left:5px solid var(--amber);background:#fff5d9}"
        ".metrics{display:grid;grid-template-columns:repeat(2,1fr);gap:.55rem;"
        "margin:1rem 0}.metric{background:var(--ink);color:white;padding:.75rem}"
        ".metric span{display:block;color:#aeb8b1;font-size:.62rem;"
        "font-weight:900;letter-spacing:.12em;text-transform:uppercase}"
        ".metric b{display:block;margin-top:.25rem;font-size:1.05rem}"
        ".task{background:var(--ink);color:white;padding:1.15rem;"
        "box-shadow:0 7px 0 var(--lime);margin:1.4rem 0}"
        ".eyebrow{color:var(--lime);font-size:.68rem;font-weight:900;"
        "letter-spacing:.16em}.task h2{font-size:1.55rem;margin:.3rem 0 .45rem}"
        ".task p{margin:0;color:#cbd3cd;line-height:1.48}.timer{font:900 2rem "
        "ui-monospace,monospace;color:var(--lime);margin-top:1rem}"
        "form{margin:0}button{width:100%;min-height:5.4rem;margin-top:1.1rem;"
        "border:3px solid var(--ink);background:var(--lime);color:var(--ink);"
        "font:900 1.15rem 'Avenir Next','Trebuchet MS',sans-serif;"
        "box-shadow:0 6px 0 #000;touch-action:manipulation;padding:1rem}"
        "button:active{transform:translateY(4px);box-shadow:0 2px 0 #000}"
        "button:disabled{background:#aaa;color:#666}.finish{background:var(--green);"
        "color:white}.connection{font-size:.78rem;color:var(--muted);margin-top:1rem}"
        ".connection.bad{color:var(--red);font-weight:900}details{margin-top:1.5rem;"
        "border-top:1px solid #c9cbc4;padding-top:1rem}summary{color:var(--muted);"
        "font-weight:800}.abort{background:var(--red);color:white;min-height:4.5rem}"
        ".foot{border-top:1px solid #c9cbc4;margin-top:1.5rem;padding-top:1rem;"
        "color:var(--muted);font-size:.84rem;line-height:1.45}a{color:#2468a2;"
        "font-weight:800}@media(min-width:38rem){.metrics{grid-template-columns:"
        "repeat(3,1fr)}}</style></head><body><main><header><div class='brand'>"
        "<span>ROADTEST</span> / RECOVERY</div><h1>Kurz&shy;kontrolle</h1>"
        "<p class='lead'>Kleine Statuspakete statt vollständiger "
        "Seitenaktualisierung. Keine Bedienung während der Fahrt erforderlich."
        "</p></header>");
    if (message.length() > 0) {
        page += F("<div class='notice'>");
        page += escapeHTML(message);
        page += F("</div>");
    }
    page += F("<section class='metrics'><div class='metric'><span>Phase</span>"
              "<b id='phase'>");
    page += active ? vehicleDataDiscovery.getPhaseName() : "bereit";
    page += F("</b></div><div class='metric'><span>ECU</span><b id='ecu'>");
    page += vehicleDataDiscovery.getECUStateName();
    page += F("</b></div><div class='metric'><span>OBD-Speed</span><b id='obd'>");
    page += obd.speedValid ? String(obd.speedKmh) + " km/h" : String("–");
    page += F("</b></div><div class='metric'><span>GPS-Speed</span><b id='gps'>");
    page += gps.speed_valid
        ? String(gps.speed_kmh, 1) + " km/h"
        : String(gps.speed_kmh, 1) + " roh";
    page += F("</b></div><div class='metric'><span>Strecke</span><b id='distance'>");
    page += String(ride.distanceKm, 3);
    page += F(" km</b></div><div class='metric'><span>Max. Pause</span><b id='pause'>");
    page += String(timing.maxLoopIntervalMs);
    page += F(" ms</b></div></section><section class='task'><div class='eyebrow'>");
    page += eyebrow;
    page += F("</div><h2>");
    page += title;
    page += F("</h2><p>");
    page += instruction;
    page += F("</p>");
    const uint32_t countdown =
        getAcceptanceCountdownSeconds(stage);
    if (stage == 1 || stage == 6 || stage == 14) {
        page += F("<div class='timer' id='timer'>");
        page += String(countdown / 60);
        page += F(":");
        if (countdown % 60 < 10) {
            page += F("0");
        }
        page += String(countdown % 60);
        page += F("</div>");
    }
    if (showAction) {
        page += F("<form method='POST' action='");
        page += action;
        page += F("'><button class='");
        page += stage == 15 ? F("finish") : F("action");
        page += F("' type='submit'>");
        page += button;
        page += F("</button></form>");
    }
    page += F("</section><div id='connection' class='connection'>"
              "Statusverbindung aktiv</div>");
    if (active && stage != 15) {
        page += F("<details><summary>Test vorzeitig abbrechen</summary>"
                  "<form method='POST' action='/acceptance/end'><button "
                  "class='abort' type='submit'>Abbruch bestätigen und Dateien "
                  "schließen</button></form></details>");
    }
    page += F("<p class='foot'>Die Seite überträgt im Hintergrund nur eine "
              "kleine Statusantwort. ECU-Recovery verwendet weiterhin "
              "ausschließlich freigegebene OBD-Service-01-Leseanfragen bei "
              "maximal zwei Frames pro Sekunde.<br><br><a href='/'>← "
              "Systemstatus</a></p></main><script>const initialStage=");
    page += String(stage);
    page += F(";function text(id,v){const e=document.getElementById(id);if(e)e."
              "textContent=v}function clock(s){return Math.floor(s/60)+':'"
              "+String(s%60).padStart(2,'0')}async function poll(){try{const r="
              "await fetch('/acceptance/status',{cache:'no-store'});if(!r.ok)"
              "throw 0;const s=await r.json();text('phase',s.phase);text('ecu',"
              "s.ecu);text('obd',s.obdSpeed<0?'–':s.obdSpeed+' km/h');text('gps',"
              "s.gpsSpeed+(s.gpsSpeedValid?' km/h':' roh'));text('distance',"
              "s.distanceKm.toFixed(3)+' km');text('pause',s.maxPauseMs+' ms');"
              "text('timer',clock(s.countdown));const c=document.getElementById("
              "'connection');if(c){c.textContent='Statusverbindung aktiv';c.className="
              "'connection'}if(s.stage!==initialStage)location.replace('/acceptance')"
              "}catch(e){const c=document.getElementById('connection');if(c){"
              "c.textContent='Statusverbindung unterbrochen';c.className='connection "
              "bad'}}}poll();setInterval(poll,2000);document.querySelectorAll('form')."
              "forEach(f=>f.addEventListener('submit',()=>{const b=f.querySelector("
              "'button');if(!b)return;const label=b.textContent;b.disabled=true;"
              "b.textContent='Wird gespeichert …';setTimeout(()=>{if(!b.disabled)"
              "return;b.disabled=false;b.textContent=label;const c=document."
              "getElementById('connection');if(c){c.textContent='Keine Bestätigung · "
              "Knopf erneut drücken';c.className='connection bad'}},5000)}));"
              "</script></body></html>");
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

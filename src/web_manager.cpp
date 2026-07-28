#include "web_manager.h"

#include <WiFi.h>
#include <Update.h>

#include "bno055_manager.h"
#include "can_reader.h"
#include "gps_manager.h"
#include "hardware_config.h"
#include "oled_manager.h"
#include "sd_logger.h"

namespace {
constexpr const char* WIFI_SSID = "ROADTEST";
constexpr const char* WIFI_PASSWORD = "roadtest123";
constexpr const char* OTA_USERNAME = "admin";
constexpr const char* OTA_PASSWORD = "roadtest123";
constexpr const char* FIRMWARE_VERSION = "1.5.10";

String formatDuration(uint32_t totalSeconds) {
    char duration[16];
    snprintf(duration, sizeof(duration), "%02lu:%02lu:%02lu",
             static_cast<unsigned long>(totalSeconds / 3600),
             static_cast<unsigned long>((totalSeconds / 60) % 60),
             static_cast<unsigned long>(totalSeconds % 60));
    return String(duration);
}
}

WebManager webManager;

WebManager::WebManager()
    : server(80),
      initialized(false),
      otaAuthorized(false),
      otaInProgress(false),
      loggingWasActive(false) {}

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
                    "Noch nicht gespeichert: Gyro und Beschleunigung müssen 3 sein."));
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

    server.on("/calibration/restart", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (sdLogger.isLogging()) {
            server.sendHeader("Cache-Control", "no-store");
            server.send(
                409, "text/html; charset=utf-8",
                buildCalibrationPage(
                    "BNO055-Neustart nicht möglich: Aufzeichnung zuerst stoppen."));
            return;
        }

        const bool restarted = bnoManager.restartFusion();
        BNO055RuntimeStatus status = bnoManager.getRuntimeStatus();
        const bool healthy = restarted && status.isFusionRunning();
        String message =
            healthy
                ? String("BNO055 neu gestartet: ") + ROADTEST_BNO_MODE_NAME + "-Sensorfusion läuft."
                : "BNO055-Neustart ohne Erfolg. Modus und Fehlercode prüfen.";
        server.sendHeader("Cache-Control", "no-store");
        server.send(healthy ? 200 : 500, "text/html; charset=utf-8",
                    buildCalibrationPage(message));
    });

    server.on("/ride/start", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        if (!sdLogger.isReady()) {
            server.send(
                503, "text/html; charset=utf-8",
                "<!doctype html><meta charset='utf-8'><h1>Start nicht möglich</h1>"
                "<p>Die SD-Karte ist nicht bereit.</p><p><a href='/'>Zurück</a></p>");
            return;
        }
        if (!sdLogger.startLogging()) {
            server.send(
                500, "text/html; charset=utf-8",
                "<!doctype html><meta charset='utf-8'><h1>Start fehlgeschlagen</h1>"
                "<p>Die Dateien konnten nicht angelegt werden.</p>"
                "<p><a href='/'>Zurück</a></p>");
            return;
        }

        server.sendHeader("Location", "/", true);
        server.send(303, "text/plain; charset=utf-8", "");
    });

    server.on("/ride/stop", HTTP_POST, [this]() {
        if (!authenticateOTA()) {
            return;
        }
        sdLogger.stopLogging();
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

            const bool success = !Update.hasError();
            server.sendHeader("Connection", "close");
            server.send(
                success ? 200 : 500,
                "text/html; charset=utf-8",
                success
                    ? "<!doctype html><meta charset='utf-8'><h1>Update erfolgreich</h1>"
                      "<p>ROADTEST startet neu. Danach wieder mit dem WLAN verbinden.</p>"
                    : "<!doctype html><meta charset='utf-8'><h1>Update fehlgeschlagen</h1>"
                      "<p>Die bisherige Firmware bleibt aktiv.</p><p><a href='/update'>Zurück</a></p>");

            if (success) {
                delay(500);
                ESP.restart();
            } else {
                resumeLoggingAfterFailedUpdate();
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
        server.handleClient();
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
    bool requiredHardwareReady =
        bnoManager.isSelfTestPassed() && bnoManager.isFusionModeActive() &&
        oledManager.isReady() &&
        sdLogger.isReady() && gpsNMEAStream && isReady();
    String page;
    page.reserve(3800);
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
    page += FIRMWARE_VERSION;
    page += F("</td></tr><tr><td>Laufzeit</td><td>");
    page += String(millis() / 1000);
    page += F(" s</td></tr><tr><td>System</td><td class='");
    page += requiredHardwareReady ? F("ok'>bereit") : F("warn'>Prüfung läuft");
    page += F("</td></tr><tr><td>BNO055</td><td>");
    page += bnoManager.isSelfTestPassed() && bnoManager.isFusionModeActive()
                ? String("OK · ") + ROADTEST_BNO_MODE_NAME
                : String("Fehler");
    page += F("</td></tr><tr><td>OLED</td><td>");
    page += oledManager.isReady() ? F("OK") : F("Fehler");
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
    page += F("</td></tr><tr><td>Aufzeichnung</td><td>");
    page += ride.active ? F("aktiv") : F("gestoppt");
    page += F("</td></tr><tr><td>Freier SD-Speicher</td><td>");
    if (sdLogger.isReady()) {
        page += String(sdLogger.getFreeSpace() / 1024);
        page += F(" MB");
    } else {
        page += F("-");
    }
    page += F("</td></tr><tr><td>CAN</td><td>");
    page += canReader.isReady() ? F("OK") : F("optional, nicht verbunden");
    page += F("</td></tr><tr><td>Kalibrierung</td><td>");
    if (bnoManager.isSelfTestPassed()) {
        CalibrationData calibration = bnoManager.getCalibration();
        page += F("G");
        page += String(calibration.gyro);
        page += F(" A");
        page += String(calibration.accel);
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
                      "SD-Fehlers abgebrochen.</p>");
        }
    } else {
        page += F("<p class='hint'>Noch keine Messfahrt in dieser Sitzung.</p>");
    }

    if (!sdLogger.isReady()) {
        page += F("<p><strong>Keine Messfahrt möglich: SD-Karte nicht bereit.</strong></p>");
    } else if (ride.active) {
        page += F("<form method='POST' action='/ride/stop'>"
                  "<button class='stop' type='submit'>Aufzeichnung beenden</button></form>");
    } else {
        page += F("<form method='POST' action='/ride/start'>"
                  "<button type='submit'>Aufzeichnung starten</button></form>");
    }

    page += F("<p class='hint'>Start und Ende sind mit den Admin-Zugangsdaten geschützt.</p>"
              "<a href='/calibration'>Kalibrierung</a>"
              " &nbsp; <a href='/update'>Firmware aktualisieren</a>"
              "</body></html>");
    return page;
}

String WebManager::buildCalibrationPage(const String& message) {
    CalibrationData calibration = bnoManager.getCalibration();
    bool bnoReady = bnoManager.isSelfTestPassed();
    BNO055RuntimeStatus runtimeStatus = bnoManager.getRuntimeStatus();
    String page;
    page.reserve(2800);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='2;url=/calibration'>"
        "<title>ROADTEST Kalibrierung</title>"
        "<style>body{font-family:system-ui;max-width:34rem;margin:2rem auto;padding:0 1rem}"
        "table{border-collapse:collapse;width:100%;font-size:1.15rem}"
        "td{padding:.5rem;border-bottom:1px solid #ddd}td:last-child{text-align:right}"
        "button{font:inherit;padding:.6rem 1rem}.ok{color:#087a30}.warn{color:#a33}"
        "li{margin:.5rem 0}</style></head><body><h1>BNO055-Kalibrierung</h1>");

    if (message.length() > 0) {
        page += F("<p><strong>");
        page += message;
        page += F("</strong></p>");
    }

    if (!bnoReady) {
        page += F(
            "<p class='warn'><strong>BNO055 ist nicht verbunden.</strong></p>"
            "<p>Die Firmware prüft den Sensor automatisch erneut. Diese Seite "
            "aktualisiert sich alle zwei Sekunden.</p>"
            "<p><a href='/'>Zur Statusseite</a></p></body></html>");
        return page;
    }

    page += F("<table><tr><td>Betriebsmodus</td><td class='");
    if (runtimeStatus.isExpectedModeActive()) {
        page += "ok'>";
        page += ROADTEST_BNO_MODE_NAME;
    } else {
        page += F("warn'>");
    }
    if (!runtimeStatus.isExpectedModeActive()) {
        page += String(runtimeStatus.operationMode);
    }
    page += F("</td></tr><tr><td>Taktquelle</td><td>intern</td></tr>"
              "<tr><td>Sensorfusion</td><td class='");
    page += runtimeStatus.isFusionRunning() ? F("ok'>läuft")
                                            : F("warn'>läuft nicht");
    page += F("</td></tr><tr><td>Systemstatus</td><td>");
    page += String(runtimeStatus.systemStatus);
    page += F(" (erwartet: 5)</td></tr><tr><td>Fehlercode</td><td>");
    page += String(runtimeStatus.systemError);
    page += F(" (erwartet: 0)</td></tr><tr><td>Gyro</td><td>");
    page += String(calibration.gyro);
    page += F(" / 3</td></tr><tr><td>Beschleunigung</td><td>");
    page += String(calibration.accel);
    page += F(" / 3</td></tr><tr><td>System</td><td>nicht erforderlich</td></tr>"
              "<tr><td>Magnetometer</td><td>");
    if (ROADTEST_BNO_USES_MAG) {
        page += String(calibration.mag);
        page += F(" / 3");
    } else {
        page += F("nicht verwendet");
    }
    page += F("</td></tr>"
              "<tr><td>Dauerhaft gespeichert</td><td>");
    page += bnoManager.isCalibrationSaved() ? F("Ja") : F("Nein");
    page += F("</td></tr></table>");

    if (calibration.isFullyCalibrated()) {
        page += String("<p class='ok'><strong>Für ") + ROADTEST_BNO_MODE_NAME +
                " ausreichend kalibriert.</strong></p>";
    } else {
        page += F(
            "<p class='warn'><strong>Gyro oder Beschleunigung noch nicht kalibriert.</strong></p>");
    }

    page += F(
        "<ol><li>Gyro: Gerät einige Sekunden völlig stillhalten.</li>"
        "<li>Beschleunigung: nacheinander auf alle sechs Seiten legen.</li>"
        "<li>Warten, bis Gyro und Beschleunigung 3 anzeigen.</li></ol>"
        "<li>Magnetometer: liegende Acht in der Luft beschreiben.</li>"
        "<form method='POST' action='/calibration/save'>"
        "<button type='submit'>Kalibrierung speichern</button></form>"
        "<form method='POST' action='/calibration/restart'>"
        "<button type='submit'>BNO055 neu starten</button></form>"
        "<p>Neustart und Speichern verwenden dieselben Zugangsdaten wie das "
        "Firmware-Update. Vor einem Sensorneustart die Aufzeichnung stoppen.</p>"
        "<p><a href='/'>Zur Statusseite</a></p></body></html>");
    return page;
}

String WebManager::buildUpdatePage() {
    return F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ROADTEST Update</title>"
        "<style>body{font-family:system-ui;max-width:34rem;margin:2rem auto;padding:0 1rem}"
        "input,button{font:inherit;margin:.5rem 0}button{padding:.55rem 1rem}</style>"
        "</head><body><h1>Firmware aktualisieren</h1>"
        "<p>Nur eine für das LOLIN S3 Mini erzeugte <code>.bin</code>-Datei auswählen.</p>"
        "<p>Während des Updates wird die SD-Aufzeichnung sauber beendet.</p>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin' required><br>"
        "<button type='submit'>Update starten</button></form>"
        "<p><a href='/'>Abbrechen</a></p></body></html>");
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

        otaInProgress = true;
        loggingWasActive = sdLogger.isLogging();
        if (loggingWasActive) {
            sdLogger.stopLogging();
        }

        Serial.printf("OTA-Update gestartet: %s\n", upload.filename.c_str());
        const uint32_t maxSketchSpace =
            (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace, U_FLASH)) {
            Update.printError(Serial);
        }
    } else if (otaAuthorized && upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (otaAuthorized && upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("✅ OTA-Update vollständig: %u Bytes\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    } else if (otaAuthorized && upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        Serial.println("⚠️ OTA-Update abgebrochen");
        resumeLoggingAfterFailedUpdate();
    }
}

void WebManager::resumeLoggingAfterFailedUpdate() {
    otaInProgress = false;
    otaAuthorized = false;

    if (loggingWasActive && sdLogger.isReady() && !sdLogger.isLogging()) {
        if (sdLogger.startLogging()) {
            Serial.println("✅ SD-Aufzeichnung nach fehlgeschlagenem Update fortgesetzt");
        } else {
            Serial.println("❌ SD-Aufzeichnung konnte nicht erneut gestartet werden");
        }
    }
    loggingWasActive = false;
}

#include "web_manager.h"

#include <WiFi.h>
#include <Update.h>
#include <SD.h>
#include <esp_task_wdt.h>

// FATFS direkt, nicht über die Arduino-FS-Schicht.
//
// `File::openNextFile()` öffnet jede Datei einzeln: Für jeden Eintrag läuft
// `f_open()` die Verzeichniskette neu ab. Am Gerät kostete das am 06.08.2026
// **83 ms je Datei**; 1.103 Dateien auf der Karte ergaben 91 Sekunden für die
// Dateiliste. `f_readdir()` liefert Name und Größe aus dem Verzeichniseintrag
// selbst, in einem sequenziellen Durchlauf und ohne eine Datei zu öffnen.
extern "C" {
#include "ff.h"
#include <miniz.h>
}

#include "gzip_stream.h"
#include "tar_writer.h"

#include "bno055_manager.h"
#include "can_reader.h"
#include "gps_manager.h"
#include "hardware_config.h"
#include "runtime_diagnostics.h"
#include "sd_logger.h"

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


// Sitzungsverzeichnisse liegen unter /sessions/<Id> auf der Karte.
constexpr const char* SESSION_ROOT = "/sessions";

// FATFS-Pfad des Sitzungsverzeichnisses, also mit Laufwerkspräfix.
//
// Die Arduino-SD-Bibliothek mountet die Karte unter `"<pdrv>:"`, hält `_pdrv`
// aber privat und bietet keinen Zugriff darauf. Die Nummer wird deshalb durch
// Probieren bestimmt. `FF_FS_RPATH` ist 0, ein Präfix ist also zwingend -
// ohne es findet FATFS das Verzeichnis nicht.
//
// Bewusst ohne Zwischenspeicher: Der Aufruf kostet ein `f_opendir()` und
// bliebe nach einem Wiedereinbinden der Karte sonst womöglich falsch.
const char* sitzungsWurzelFatfs() {
    static char pfad[16];
    for (int laufwerk = 0; laufwerk < FF_VOLUMES; ++laufwerk) {
        snprintf(pfad, sizeof(pfad), "%d:%s", laufwerk, SESSION_ROOT);
        FF_DIR probe;
        if (f_opendir(&probe, pfad) == FR_OK) {
            f_closedir(&probe);
            return pfad;
        }
    }
    return nullptr;
}

// Nur Zeichen, die die Firmware selbst in eine Sitzungs-ID schreibt. Ohne
// diese Prüfung wäre "../" ein gültiger Parameter, und der Download gäbe
// beliebige Dateien der Karte heraus.
bool istGueltigeSitzungsId(const String& id) {
    if (id.length() == 0 || id.length() > 40) {
        return false;
    }
    for (size_t i = 0; i < id.length(); ++i) {
        const char c = id[i];
        const bool erlaubt = (c >= '0' && c <= '9') ||
                             (c >= 'A' && c <= 'Z') ||
                             (c >= 'a' && c <= 'z') || c == '_';
        if (!erlaubt) {
            return false;
        }
    }
    return true;
}

// Senke, die den komprimierten Strom als HTTP-Chunks ausgibt.
//
// Die Länge steht erst nach der Kompression fest; die Antwort läuft deshalb
// mit CONTENT_LENGTH_UNKNOWN und damit chunked. Ein Abbruch der Verbindung
// bleibt hier unbemerkt - sendContent() meldet ihn nicht -, kostet aber nur
// eine unvollständige Datei, die sich erneut holen lässt.
// Größe eines HTTP-Chunks beim Download.
//
// Aus der Messung am Gerät am 04.08.2026: Ein Sendeaufruf kostet rund 4,57 ms
// Fixaufwand plus etwa 2 µs je Byte. 4 kB je Chunk liegt im flachen Teil der
// Kurve; darüber bringt Vergrößern kaum noch etwas.
constexpr size_t HTTP_CHUNK_BYTES = 4096;

// Platz für die Chunk-Kopfzeile ("1000\r\n") unmittelbar vor der Nutzlast.
// Kopf und Nutzlast gehen dadurch in einem Schreibvorgang hinaus.
constexpr size_t HTTP_KOPF_RESERVE = 12;

// Kein Fortschritt über diese Zeit hinweg gilt als tote Verbindung.
constexpr unsigned long HTTP_SENDE_TIMEOUT_MS = 10000;

// **Der Puffer muss einen vollständigen Kompressorrückruf aufnehmen.**
//
// Der ROM-Kompressor sammelt intern und ruft seine Senke mit bis zu
// `TDEFL_OUT_BUF_SIZE` = 31.948 Byte am Stück auf - nicht in der Größe der
// hineingereichten Nutzdaten. An echten Sitzungen sind die ersten Rückrufe
// 9,9 bis 16,7 kB groß. Ein Puffer, der auf die Eingabegröße baut, läuft
// über; ein 8-kB-Versuch am 04.08.2026 brach deshalb bei jeder Sitzung schon
// beim ersten Rückruf ab.
constexpr size_t HTTP_SINK_PUFFER_BYTES =
    TDEFL_OUT_BUF_SIZE + HTTP_CHUNK_BYTES + HTTP_KOPF_RESERVE;

class HttpChunkSink : public GzipSink {
public:
    HttpChunkSink(RoadtestWebServer& server, uint8_t* puffer)
        : server(server), puffer(puffer) {}

    // Nimmt ausschließlich entgegen und sendet **nicht**.
    //
    // Diese Methode läuft im Rückruf des Kompressors, tief im Aufrufstapel von
    // `tdefl_compress_buffer()`. Von dort aus zu senden sprengt den Stapel des
    // Loop-Tasks: Am 06.08.2026 endete jeder Download mit
    // `Stack canary watchpoint triggered (loopTask)`, im Backtrace
    // `GzipStream::write()` → ROM-Kompressor → Senke → lwIP. Der Loop-Task hat
    // 8 kB, und der Sendepfad mit `select()`, lwIP-Nachrichten und dem
    // Interrupt-Spielraum passt darunter nicht mehr hinein.
    //
    // Eine frühere Host-Messung hatte die Kette von `tdefl_compress_buffer()`
    // bis in den Rückruf mit nur 320 Byte beziffert und daraus geschlossen,
    // Senden aus dem Rückruf sei unbedenklich. Sie maß die falsche Hälfte -
    // die tdefl-Rahmen, nicht den lwIP-Pfad darunter.
    //
    // Gesendet wird deshalb ausschließlich von der Downloadschleife über
    // `spuelenAlles()`, auf oberster Ebene.
    size_t write(const uint8_t* data, size_t length) override {
        if (abgebrochen) {
            return 0;
        }
        if (fuellstand + length > HTTP_SINK_PUFFER_BYTES - HTTP_KOPF_RESERVE) {
            // Kann nur eintreten, wenn die Schleife das Spülen ausgelassen
            // hat. Dann geht nichts verloren, sondern der Strom wird als
            // gescheitert gemeldet.
            Serial.printf(
                "  ! Senkenpuffer zu klein: %u + %u > %u\n",
                static_cast<unsigned>(fuellstand),
                static_cast<unsigned>(length),
                static_cast<unsigned>(HTTP_SINK_PUFFER_BYTES -
                                      HTTP_KOPF_RESERVE));
            abgebrochen = true;
            return 0;
        }
        memcpy(nutzlast() + fuellstand, data, length);
        fuellstand += length;
        return length;
    }

    // Alles Gepufferte hinausschreiben, in Scheiben von HTTP_CHUNK_BYTES.
    //
    // Gehört auf die oberste Ebene der Downloadschleife, niemals in den
    // Rückruf. Muss auch nach `finish()` laufen, sonst fehlt das Ende des
    // gzip-Stroms.
    void spuelenAlles() {
        while (fuellstand > 0 && !abgebrochen) {
            const size_t menge =
                fuellstand < HTTP_CHUNK_BYTES ? fuellstand : HTTP_CHUNK_BYTES;
            const unsigned long begonnen = millis();
            const bool ok = sendeScheibe(sendeOffset, menge);
            sendeMs += millis() - begonnen;
            if (!ok) {
                abgebrochen = true;
                break;
            }
            bloecke++;
            gesendet += menge;
            sendeOffset += menge;
            fuellstand -= menge;
            // Ein langer Download hält die Hauptschleife an. Ohne diesen Reset
            // greift der Task-Watchdog mitten in der Übertragung.
            esp_task_wdt_reset();
        }
        sendeOffset = 0;
        fuellstand = 0;
    }

    bool wurdeAbgebrochen() const { return abgebrochen; }
    unsigned long sendedauerMs() const { return sendeMs; }
    uint32_t blockzahl() const { return bloecke; }
    uint32_t gesendeteBytes() const { return gesendet; }

private:
    uint8_t* nutzlast() const { return puffer + HTTP_KOPF_RESERVE; }

    // Eine Scheibe als HTTP-Chunk hinausschreiben.
    //
    // Die Kopfzeile wird in die Bytes unmittelbar davor geschrieben. Für die
    // erste Scheibe ist das die Reserve am Pufferanfang, für jede weitere
    // bereits gesendete Nutzlast - beides gefahrlos überschreibbar.
    bool sendeScheibe(size_t offset, size_t menge) {
        const uint8_t* start = nutzlast() + offset;
        size_t laenge = menge;

        // Die Rahmung übernimmt bewusst nicht sendContent(): Nur hier lässt
        // sich ein Teilschreibvorgang überhaupt bemerken.
        if (server.istChunked()) {
            char kopf[HTTP_KOPF_RESERVE];
            const int kopflaenge = snprintf(kopf, sizeof(kopf), "%x\r\n",
                                            static_cast<unsigned>(menge));
            if (kopflaenge <= 0 ||
                static_cast<size_t>(kopflaenge) > HTTP_KOPF_RESERVE) {
                return false;
            }
            start -= kopflaenge;
            memcpy(const_cast<uint8_t*>(start), kopf,
                   static_cast<size_t>(kopflaenge));
            laenge += static_cast<size_t>(kopflaenge);
        }
        if (!schreibeVollstaendig(start, laenge)) {
            return false;
        }
        if (server.istChunked()) {
            static const uint8_t abschluss[2] = {'\r', '\n'};
            return schreibeVollstaendig(abschluss, sizeof(abschluss));
        }
        return true;
    }

    // Schreibt vollständig oder meldet Fehlschlag.
    //
    // `WebServer::sendContent()` verwirft den Rückgabewert von `write()`; ein
    // Teilschreibvorgang verfälschte damit die angekündigte Chunk-Länge. Hier
    // wird jeder Rückgabewert ausgewertet und ab der erreichten Stelle
    // weitergeschrieben.
    bool schreibeVollstaendig(const uint8_t* daten, size_t laenge) {
        size_t geschrieben = 0;
        unsigned long letzterFortschritt = millis();
        while (geschrieben < laenge) {
            if (!server.client().connected()) {
                return false;
            }
            const size_t n = server.client().write(daten + geschrieben,
                                                   laenge - geschrieben);
            esp_task_wdt_reset();
            if (n > 0) {
                geschrieben += n;
                letzterFortschritt = millis();
                continue;
            }
            if (millis() - letzterFortschritt > HTTP_SENDE_TIMEOUT_MS) {
                return false;
            }
            // Dem TCP-Stack Zeit geben, den Sendepuffer zu leeren.
            delay(1);
        }
        return true;
    }

    RoadtestWebServer& server;
    uint8_t* puffer = nullptr;
    size_t fuellstand = 0;
    size_t sendeOffset = 0;
    unsigned long sendeMs = 0;
    uint32_t bloecke = 0;
    uint32_t gesendet = 0;
    bool abgebrochen = false;
};

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

    // Datenzugriff übers Handy. Die Sitzung wird als ein einziges .tar.gz
    // ausgegeben, damit eine Fahrt ein Tippvorgang bleibt und nicht zehn.
    server.on("/files", HTTP_GET, [this]() {
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "text/html; charset=utf-8", buildFilesPage());
    });

    server.on("/files/download", HTTP_GET, [this]() {
        handleSessionDownload();
    });

    server.on("/files/delete", HTTP_POST, [this]() {
        handleSessionDelete();
    });

    server.on("/speedtest", HTTP_GET, [this]() {
        handleSpeedtest();
    });

    // Die Beifahrerseite /fahrbahn ist mit STRECKENDATENBANK.md entfallen.
    // Urteile entstehen jetzt nach der Fahrt am Rechner, nicht mehr während
    // der Fahrt am Gerät: Was Aufmerksamkeit kostet, kommt nicht in die
    // Datenbank. Die Notengrenzen ROAD_DRIVEABILITY_RMS_GOOD_MPS2 und
    // _BAD_MPS2 sind mit den 149 Urteilen zweier Fahrtage bestimmt und
    // bleiben davon unberührt.

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
              "<a href='/files'>Daten holen</a>"
              " &nbsp; <a href='/calibration'>Kalibrierung</a>"
              " &nbsp; <a href='/update'>Firmware aktualisieren</a>"
              "</body></html>");
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

// Übersicht der Sitzungen auf der Karte.
//
// Bewusst ohne Vorschau und ohne Einzeldateien: Die Seite wird am Handy im
// Auto bedient. Je Sitzung eine Zeile, eine Schaltfläche zum Holen, eine zum
// Löschen.
String WebManager::buildFilesPage(const String& message) {
    String page;
    page.reserve(2400);
    page += F(
        "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ROADTEST Daten</title>"
        "<style>body{margin:0;padding:14px;background:#15221c;color:#f4f1e8;"
        "font-family:system-ui,sans-serif}h1{font-size:1.15rem;margin:0 0 12px}"
        ".s{background:#22332b;border-radius:12px;padding:12px;margin-bottom:10px}"
        ".id{font-family:ui-monospace,monospace;font-size:0.95rem}"
        ".m{color:#9db3a6;font-size:0.85rem;margin:4px 0 10px}"
        "a.btn,button{display:inline-block;border:0;border-radius:10px;"
        "padding:12px 16px;font-size:1rem;font-weight:600;text-decoration:none;"
        "margin-right:8px}a.btn{background:#8fd14f;color:#15221c}"
        "button{background:#e3734d;color:#15221c}"
        ".hinweis{background:#3d3320;border-radius:10px;padding:10px;"
        "margin-bottom:12px}</style></head><body><h1>Aufgezeichnete Fahrten</h1>");

    if (message.length() > 0) {
        page += F("<div class='hinweis'>");
        page += escapeHTML(message);
        page += F("</div>");
    }

    if (sdLogger.isLogging() || sdLogger.isLoggingStartPending()) {
        page += F(
            "<div class='hinweis'><strong>Aufzeichnung läuft</strong>"
            "<p>Herunterladen und Löschen sind gesperrt, solange gemessen "
            "wird - genau wie beim Firmware-Update. Ein Download hält die "
            "Hauptschleife für seine ganze Dauer an; die laufende Messung "
            "verlöre dabei Zeilen.</p>"
            "<p>Erst auf der Statusseite beenden.</p></div>");
    }

    // Anzahl und Größe je Sitzung stammen aus den Verzeichniseinträgen selbst.
    // Keine Datei wird dafür geöffnet: `f_readdir()` liest die Einträge
    // sequenziell, während `openNextFile()` für jeden ein `f_open()` mit
    // vollständiger Pfadauflösung ausführte. Am 06.08.2026 kostete das am
    // Gerät 83 ms je Datei und 91 Sekunden für 1.103 Dateien.
    //
    // Der Watchdog-Reset bleibt: Er verhinderte den Reboot der Dateiliste und
    // ist auch hier die Absicherung, falls die Karte langsam antwortet.
    const unsigned long listeBegonnen = millis();
    uint16_t dateienGesamt = 0;

    // FILINFO fasst mit aktivem LFN rund 270 Byte. Zwei davon auf dem Stapel
    // des Loop-Tasks wären unnötig knapp; die Seite wird nie nebenläufig
    // gebaut, weil die Hauptschleife einsträngig im Handler steckt.
    static FILINFO eintrag;
    static FILINFO datei;
    static char sitzungsPfad[300];

    const char* wurzelPfad = sitzungsWurzelFatfs();
    FF_DIR wurzel;
    if (wurzelPfad == nullptr || f_opendir(&wurzel, wurzelPfad) != FR_OK) {
        page += F("<p>Keine Sitzungen auf der Karte.</p>");
    } else {
        uint16_t anzahl = 0;
        while (f_readdir(&wurzel, &eintrag) == FR_OK && eintrag.fname[0] != '\0') {
            esp_task_wdt_reset();
            if (!(eintrag.fattrib & AM_DIR)) {
                continue;
            }
            const String id(eintrag.fname);

            uint32_t summe = 0;
            uint16_t dateien = 0;
            snprintf(sitzungsPfad, sizeof(sitzungsPfad), "%s/%s", wurzelPfad,
                     eintrag.fname);
            FF_DIR sitzung;
            if (f_opendir(&sitzung, sitzungsPfad) == FR_OK) {
                while (f_readdir(&sitzung, &datei) == FR_OK &&
                       datei.fname[0] != '\0') {
                    if (!(datei.fattrib & AM_DIR)) {
                        summe += static_cast<uint32_t>(datei.fsize);
                        dateien++;
                    }
                }
                f_closedir(&sitzung);
                esp_task_wdt_reset();
            }
            dateienGesamt += dateien;

            anzahl++;
            page += F("<div class='s'><div class='id'>");
            page += escapeHTML(id);
            page += F("</div><div class='m'>");
            page += String(dateien);
            page += F(" Dateien &middot; ");
            page += String(summe / 1024);
            page += F(" kB</div><a class='btn' href='/files/download?s=");
            page += escapeHTML(id);
            page += F("'>Holen</a>"
                      "<form method='POST' action='/files/delete' "
                      "style='display:inline'>"
                      "<input type='hidden' name='s' value='");
            page += escapeHTML(id);
            page += F("'><button type='submit'>Löschen</button>"
                      "</form></div>");
        }
        f_closedir(&wurzel);
        if (anzahl == 0) {
            page += F("<p>Keine Sitzungen auf der Karte.</p>");
        }
    }

    // Messen statt schätzen: Die Dauer wächst mit der Zahl der Dateien auf der
    // Karte. Bleibt sie im Sekundenbereich, genügt der Watchdog-Reset; wird
    // sie zweistellig, muss das Öffnen jeder einzelnen Datei entfallen.
    const unsigned long listeMs = millis() - listeBegonnen;
    Serial.printf("Dateiliste: %u Dateien in %lu ms (%lu ms je Datei)\n",
                  static_cast<unsigned>(dateienGesamt), listeMs,
                  dateienGesamt > 0 ? listeMs / dateienGesamt : 0);

    page += F(
        "<p class='m'>Das Archiv enthält die Sitzung als Verzeichnis. Am "
        "Rechner mit <code>tar xzf</code> auspacken.</p>"
        "<p class='m'>Dauert ein Download auffällig lange: "
        "<a href='/speedtest?kb=1024&amp;bs=1460'>WLAN messen</a> - "
        "1 MB ohne Karte und ohne Kompression, das Ergebnis steht am Ende "
        "der Antwort.</p>"
        "<p><a class='btn' href='/'>Zur Statusseite</a></p></body></html>");
    return page;
}

// Reine WLAN-Messung ohne SD und ohne Kompression.
//
// Der Download durchläuft drei Stufen - von der Karte lesen, komprimieren,
// über WLAN senden -, und die Gesamtdauer sagt nicht, welche davon bremst.
// Dieser Endpunkt misst die dritte allein: Nutzdaten aus dem Arbeitsspeicher,
// sonst derselbe Weg. Ergebnis ist die Obergrenze, die der echte Download
// niemals überschreiten kann.
//
// `kb` wählt die Datenmenge, `bs` die Blockgröße je sendContent(). Der zweite
// Parameter ist der eigentliche Verdacht: Der Kompressor gibt seine Blöcke in
// der Größe aus, die ihm passt, und jeder Aufruf erzeugt einen eigenen
// HTTP-Chunk mit eigenem Kopf. Viele kleine Blöcke kosten Durchsatz.
//
//   http://192.168.4.1/speedtest?kb=1024&bs=4096
void WebManager::handleSpeedtest() {
    long kb = server.arg("kb").toInt();
    if (kb <= 0 || kb > 8192) {
        kb = 1024;
    }
    long bs = server.arg("bs").toInt();
    if (bs <= 0 || bs > 8192) {
        bs = 1460;  // eine TCP-Nutzlast bei 1500 Byte MTU
    }

    static uint8_t muster[8192];
    for (size_t i = 0; i < sizeof(muster); ++i) {
        muster[i] = static_cast<uint8_t>('A' + (i % 26));
    }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain; charset=utf-8", "");

    const uint32_t gesamt = static_cast<uint32_t>(kb) * 1024;
    uint32_t gesendet = 0;
    const unsigned long begonnen = millis();
    while (gesendet < gesamt && server.client().connected()) {
        const size_t block = static_cast<size_t>(
            (gesamt - gesendet) < static_cast<uint32_t>(bs)
                ? (gesamt - gesendet)
                : static_cast<uint32_t>(bs));
        server.sendContent(reinterpret_cast<const char*>(muster), block);
        gesendet += block;
        esp_task_wdt_reset();
    }
    const unsigned long dauer = millis() - begonnen;

    // Die Messung selbst hängt hinten an der Antwort. Damit braucht es weder
    // serielle Konsole noch Stoppuhr am Handy.
    const float rate = dauer > 0 ? (gesendet / 1024.0f) / (dauer / 1000.0f) : 0;
    String bericht = String("\n--- WLAN-Messung ---\n") +
                     String(gesendet / 1024) + " kB in " + String(dauer) +
                     " ms = " + String(rate, 1) + " kB/s\n" +
                     "Blockgroesse " + String(bs) + " Byte, " +
                     String(gesendet / (bs > 0 ? bs : 1)) + " Bloecke\n";
    server.sendContent(bericht);
    server.sendContent("");

    Serial.printf("WLAN-Messung: %lu kB in %lu ms = %.1f kB/s (Block %ld B)\n",
                  static_cast<unsigned long>(gesendet / 1024), dauer, rate, bs);
}

// Eine Sitzung als tar.gz ausliefern.
//
// Komprimiert wird im Fluss: Die Länge steht erst am Ende fest, die Antwort
// läuft deshalb chunked. Roh übertragen wären es rund 12 MB je Fahrtstunde -
// daran scheiterte die früher entfernte Downloadfunktion. Mit dem
// ROM-Kompressor bleibt davon etwa ein Achtel bis Zwölftel, ohne ein Byte
// Flash zu kosten.
void WebManager::handleSessionDownload() {
    // Jeder Ausgang meldet sich. Die frühere Fassung schrieb ausschließlich am
    // Ende; ein vorzeitiger Ausstieg oder ein Hänger erzeugte damit gar keine
    // Ausgabe, und "keine Zeile" war nicht von "langsam" zu unterscheiden.
    const String id = server.arg("s");
    Serial.printf("Download angefragt: '%s'\n", id.c_str());

    if (!istGueltigeSitzungsId(id)) {
        Serial.println("  -> abgelehnt: ungültige Sitzungs-ID");
        server.send(400, "text/plain; charset=utf-8", "Ungültige Sitzung");
        return;
    }
    if (sdLogger.isLogging() || sdLogger.isLoggingStartPending()) {
        Serial.println("  -> abgelehnt: Aufzeichnung läuft");
        server.send(409, "text/plain; charset=utf-8",
                    "Aufzeichnung läuft - zuerst beenden");
        return;
    }

    const String pfad = String(SESSION_ROOT) + "/" + id;
    File verzeichnis = SD.open(pfad);
    if (!verzeichnis || !verzeichnis.isDirectory()) {
        if (verzeichnis) {
            verzeichnis.close();
        }
        Serial.printf("  -> abgelehnt: %s ist kein Verzeichnis\n",
                      pfad.c_str());
        server.send(404, "text/plain; charset=utf-8", "Sitzung nicht gefunden");
        return;
    }

    // Der Senkenpuffer liegt im PSRAM, nicht im knappen internen RAM: Er fasst
    // einen vollstaendigen Kompressorrueckruf und waere dort mit 36 kB ein
    // dauerhafter Posten. Nur waehrend des Downloads belegt.
    uint8_t* senkenPuffer = static_cast<uint8_t*>(
        heap_caps_malloc(HTTP_SINK_PUFFER_BYTES, MALLOC_CAP_SPIRAM));
    if (senkenPuffer == nullptr) {
        verzeichnis.close();
        Serial.println("  -> abgelehnt: kein PSRAM fuer den Senkenpuffer");
        server.send(503, "text/plain; charset=utf-8",
                    "Speicher nicht verfuegbar - Karte auslesen");
        return;
    }

    HttpChunkSink senke(server, senkenPuffer);
    GzipStream strom;
    if (!strom.begin(senke)) {
        heap_caps_free(senkenPuffer);
        verzeichnis.close();
        // Ohne PSRAM gibt es keinen Kompressor. Unkomprimiert auszuliefern ist
        // ausdrücklich ausgeschlossen; die Karte bleibt dann der Weg.
        Serial.println("  -> abgelehnt: Kompressor nicht verfügbar");
        server.send(503, "text/plain; charset=utf-8",
                    "Kompression nicht verfügbar - Karte auslesen");
        return;
    }
    Serial.println("  -> Übertragung beginnt");

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.sendHeader("Content-Disposition",
                      "attachment; filename=\"" + id + ".tar.gz\"");
    server.send(200, "application/gzip", "");

    static uint8_t block[512];
    static uint8_t puffer[1024];

    // Zeit je Stufe getrennt mitführen. Die Gesamtdauer allein sagt nicht, ob
    // die Karte, der Kompressor oder das WLAN bremst.
    const unsigned long downloadBegonnen = millis();
    unsigned long leseMs = 0;
    unsigned long letzteMeldung = downloadBegonnen;
    uint32_t rohBytes = 0;

    File datei = verzeichnis.openNextFile();
    while (datei && !senke.wurdeAbgebrochen()) {
        if (!datei.isDirectory()) {
            const String name = String(datei.name());
            const String kurz = name.substring(name.lastIndexOf('/') + 1);
            const uint32_t groesse = datei.size();

            tarWriteHeader(block, (id + "/" + kurz).c_str(), groesse);
            strom.write(block, sizeof(block));
            senke.spuelenAlles();

            Serial.printf("  Datei %s (%lu Byte)\n", kurz.c_str(),
                          static_cast<unsigned long>(groesse));

            uint32_t uebertragen = 0;
            while (uebertragen < groesse && !senke.wurdeAbgebrochen()) {
                const unsigned long leseBegonnen = millis();
                const size_t gelesen = datei.read(puffer, sizeof(puffer));
                leseMs += millis() - leseBegonnen;
                if (gelesen == 0) {
                    Serial.printf("  ! read() lieferte 0 nach %lu von %lu Byte\n",
                                  static_cast<unsigned long>(uebertragen),
                                  static_cast<unsigned long>(groesse));
                    break;
                }
                strom.write(puffer, gelesen);
                // Senden gehoert auf diese Ebene, nicht in den Rueckruf des
                // Kompressors: Dort reicht der Stapel des Loop-Tasks nicht.
                senke.spuelenAlles();
                uebertragen += gelesen;
                rohBytes += gelesen;
                esp_task_wdt_reset();

                // Fortschritt im Sekundentakt. Bleibt der Download stehen,
                // zeigt die letzte Zeile, an welcher Stelle - ohne sie war
                // ein Hänger von langsamer Übertragung nicht zu trennen.
                const unsigned long jetzt = millis();
                if (jetzt - letzteMeldung >= 1000) {
                    letzteMeldung = jetzt;
                    const unsigned long lauf = jetzt - downloadBegonnen;
                    Serial.printf(
                        "    %lu kB gelesen, %lu kB gesendet, %lu ms "
                        "(lesen %lu, senden %lu)\n",
                        static_cast<unsigned long>(rohBytes / 1024),
                        static_cast<unsigned long>(senke.gesendeteBytes() / 1024),
                        lauf, leseMs, senke.sendedauerMs());
                }
            }

            // tar füllt jede Datei auf ein Vielfaches von 512 auf.
            const size_t fuellbytes = tarPaddingBytes(uebertragen);
            if (fuellbytes > 0) {
                memset(block, 0, fuellbytes);
                strom.write(block, fuellbytes);
                senke.spuelenAlles();
            }
        }
        datei.close();
        datei = verzeichnis.openNextFile();
    }
    verzeichnis.close();

    // Zwei Nullblöcke schließen ein tar-Archiv ab.
    memset(block, 0, sizeof(block));
    strom.write(block, sizeof(block));
    senke.spuelenAlles();
    strom.write(block, sizeof(block));
    senke.spuelenAlles();

    strom.finish();
    strom.end();
    // Erst jetzt den Rest ausgeben: finish() schreibt den gzip-Nachspann noch
    // durch die Senke, und ohne ihn meldet jedes Werkzeug beim Auspacken einen
    // unvollständigen Strom.
    senke.spuelenAlles();

    if (senke.wurdeAbgebrochen()) {
        // Verbindung hart schließen, statt mit dem Null-Chunk eine
        // vollständige Antwort vorzutäuschen. Nur so erkennt die Gegenstelle
        // das Archiv als abgebrochen und legt keine halbe Datei als
        // vollständig ab.
        server.client().stop();
    } else {
        server.sendContent("");
    }

    // Der Puffer wird nur für die Dauer der Übertragung gehalten. Erst nach
    // dem letzten Spülen freigeben - die Senke zeigt bis hierher hinein.
    heap_caps_free(senkenPuffer);

    // Aufschlüsselung auf die serielle Konsole. Ohne sie bliebe nur die
    // Gesamtdauer, und die benennt keine Ursache.
    const unsigned long gesamtMs = millis() - downloadBegonnen;
    const unsigned long sendeMs = senke.sendedauerMs();
    const unsigned long rechenMs =
        gesamtMs > (leseMs + sendeMs) ? gesamtMs - leseMs - sendeMs : 0;
    const float rate =
        gesamtMs > 0 ? (rohBytes / 1024.0f) / (gesamtMs / 1000.0f) : 0;
    Serial.printf(
        "Download %s: %lu kB roh -> %lu kB gepackt in %lu ms = %.1f kB/s\n",
        id.c_str(), static_cast<unsigned long>(rohBytes / 1024),
        static_cast<unsigned long>(senke.gesendeteBytes() / 1024), gesamtMs,
        rate);
    Serial.printf(
        "  SD-Lesen %lu ms, Komprimieren %lu ms, Senden %lu ms in %lu Bloecken"
        " (%lu Byte je Block)%s\n",
        leseMs, rechenMs, sendeMs,
        static_cast<unsigned long>(senke.blockzahl()),
        static_cast<unsigned long>(
            senke.blockzahl() > 0 ? senke.gesendeteBytes() / senke.blockzahl()
                                  : 0),
        senke.wurdeAbgebrochen() ? " - ABGEBROCHEN" : "");
}

// Eine Sitzung von der Karte entfernen.
//
// Nur nach Anmeldung und nur innerhalb von /sessions mit geprüfter ID. Die
// laufende Aufzeichnung ist ausgenommen.
void WebManager::handleSessionDelete() {
    if (!authenticateOTA()) {
        return;
    }
    const String id = server.arg("s");
    if (!istGueltigeSitzungsId(id)) {
        server.send(400, "text/html; charset=utf-8",
                    buildFilesPage("Ungültige Sitzung."));
        return;
    }
    if (sdLogger.isLogging() || sdLogger.isLoggingStartPending()) {
        server.send(409, "text/html; charset=utf-8",
                    buildFilesPage("Aufzeichnung läuft - zuerst beenden."));
        return;
    }

    const String pfad = String(SESSION_ROOT) + "/" + id;
    File verzeichnis = SD.open(pfad);
    if (!verzeichnis || !verzeichnis.isDirectory()) {
        if (verzeichnis) {
            verzeichnis.close();
        }
        server.send(404, "text/html; charset=utf-8",
                    buildFilesPage("Sitzung nicht gefunden."));
        return;
    }

    uint16_t geloescht = 0;
    File datei = verzeichnis.openNextFile();
    while (datei) {
        // name() liefert je nach Version der SD-Bibliothek den vollen Pfad
        // oder nur den Dateinamen. SD.remove() braucht den vollen; mit dem
        // blossen Namen schlaegt es still fehl, und das Verzeichnis bliebe
        // mitsamt Dateien stehen. Deshalb wird der Pfad hier gebaut.
        const String roh = String(datei.name());
        const String kurz = roh.substring(roh.lastIndexOf('/') + 1);
        const bool istVerzeichnis = datei.isDirectory();
        datei.close();
        if (!istVerzeichnis && SD.remove(pfad + "/" + kurz)) {
            geloescht++;
        }
        datei = verzeichnis.openNextFile();
        // Löschen läuft über dieselbe langsame Karte wie die Dateiliste und
        // braucht denselben Schutz, sonst greift der Task-Watchdog.
        esp_task_wdt_reset();
    }
    verzeichnis.close();
    SD.rmdir(pfad);

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/html; charset=utf-8",
                buildFilesPage(String("Sitzung ") + id + " gelöscht (" +
                               geloescht + " Dateien)."));
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

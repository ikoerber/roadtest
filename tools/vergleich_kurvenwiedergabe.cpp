// Vergleicht die Wiedergabe auf dem Entwicklungsrechner mit den Ereignissen,
// die die Firmware waehrend derselben Fahrt selbst geschrieben hat.
//
// Zweck: nach einer Fahrt pruefen, ob die Kurvenerkennung auf dem Geraet
// dasselbe liefert wie dieselbe Logik auf dem Rechner. Stimmen beide
// Ereignislisten ueberein, ist die Umsetzung auf dem ESP32 bestaetigt und
// nicht nur plausibel.
//
// Uebersetzen und ausfuehren aus dem Projektverzeichnis:
//
//   c++ -std=gnu++17 -O1 -I src -o /tmp/vergleich \
//       tools/vergleich_kurvenwiedergabe.cpp src/curve_detector.cpp
//   /tmp/vergleich testdata/
//
// Die auszuwertenden Sitzungen stehen in der Liste in main(). Eine Sitzung
// braucht road_sensor, road_gps und road_event; road_obd wird verwendet,
// wenn vorhanden.
//
// WICHTIG: Der Vergleich ist nur aussagekraeftig, wenn die Sitzung mit
// derselben Firmwareversion aufgezeichnet wurde, die hier uebersetzt wird.
// Gegen eine aeltere Aufzeichnung misst das Werkzeug die beabsichtigte
// Verhaltensaenderung, nicht die Umsetzungstreue. Mit den Quellen von 1.5.28
// gegen die Aufzeichnungen vom 31.07.2026 lautet das Ergebnis 174 zu 174
// Ereignissen, 0,1 Grad mittlere Winkelabweichung und 7 bis 21 Millisekunden
// Startversatz in den Fahrten mit belastbarer Zeitbasis. So sieht ein
// bestandener Abgleich aus. Mit den Quellen von 1.5.34 gegen die drei Fahrten
// vom 02.08.2026 lautet es 33 zu 33 bei ebenfalls 0,1 Grad.
//
// Die Geschwindigkeit wird wie in main.cpp gewaehlt: zuerst OBD, sonst GPS,
// sonst unbekannt. Nur so misst die Wiedergabe dasselbe System - mit der
// GPS-Geschwindigkeit allein wich die Wiedergabe der Fahrten vom 31.07.2026
// um 46 von 174 Ereignissen ab, mit OBD stimmte sie exakt.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "curve_detector.h"

namespace {

struct Csv {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    int col(const std::string& n) const {
        for (size_t i = 0; i < header.size(); ++i) {
            if (header[i] == n) return static_cast<int>(i);
        }
        return -1;
    }
};

bool load(const std::string& path, Csv& csv) {
    std::ifstream in(path);
    if (!in.good()) return false;
    std::string line;
    if (!std::getline(in, line)) return false;
    std::stringstream hs(line);
    std::string cell;
    while (std::getline(hs, cell, ',')) csv.header.push_back(cell);
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> cells;
        std::stringstream ss(line);
        while (std::getline(ss, cell, ',')) cells.push_back(cell);
        while (cells.size() < csv.header.size()) cells.push_back("");
        csv.rows.push_back(cells);
    }
    return true;
}

// Zeitlich sortierte Geschwindigkeitsquelle mit Gueltigkeitsflag.
struct Quelle {
    std::vector<uint32_t> t;
    std::vector<float> v;
    std::vector<bool> gueltig;

    void add(uint32_t zeit, float wert, bool ok) {
        t.push_back(zeit);
        v.push_back(wert);
        gueltig.push_back(ok);
    }

    // Juengster Eintrag bis zum Zeitpunkt, hoechstens maxAgeMs alt.
    bool at(uint32_t zeit, uint32_t maxAgeMs, float& out) const {
        size_t lo = 0, hi = t.size();
        while (lo < hi) {
            const size_t mid = (lo + hi) / 2;
            if (t[mid] <= zeit) lo = mid + 1; else hi = mid;
        }
        if (lo == 0) return false;
        const size_t idx = lo - 1;
        if (!gueltig[idx]) return false;
        if (zeit - t[idx] > maxAgeMs) return false;
        out = v[idx];
        return true;
    }
};

struct Firmwareereignis {
    uint32_t start;
    uint32_t ende;
    float winkel;
    bool zugeordnet = false;
};

}  // namespace

int main(int argc, char** argv) {
    const std::string basis = argc > 1 ? argv[1] : "testdata/";
    const char* sitzungen[] = {
        "20260802_095708_84FD688D", "20260802_100225_B067E95B",
        "20260802_100356_CF34940C"};

    size_t gesamtFirmware = 0, gesamtWiedergabe = 0, gesamtPaare = 0;
    double winkelAbwSumme = 0;
    double startAbwSumme = 0;

    for (const char* s : sitzungen) {
        const std::string dir = basis + s + "/";
        Csv sensor, gps, obd, ereignis;
        if (!load(dir + "road_sensor_" + s + ".csv", sensor) ||
            !load(dir + "road_gps_" + s + ".csv", gps) ||
            !load(dir + "road_event_" + s + ".csv", ereignis)) {
            std::fprintf(stderr, "Sitzung %s nicht lesbar\n", s);
            return 1;
        }
        const bool hatOBD = load(dir + "road_obd_" + s + ".csv", obd);

        // Zeitversatz zwischen Geraete- und Sitzungszeit aus einem beliebigen
        // Kurvenereignis: EndUptimeMs ist Geraetezeit, UptimeMs Sitzungszeit.
        const int eK = ereignis.col("Ereignis");
        const int eStart = ereignis.col("StartUptimeMs");
        const int eEnde = ereignis.col("EndUptimeMs");
        const int eWinkel = ereignis.col("AngleDeg");
        const int eUptime = ereignis.col("UptimeMs");
        const int eBeschr = ereignis.col("Beschreibung");
        std::vector<Firmwareereignis> firmware;

        // Zeitbasisversatz bevorzugt aus einem Referenzmarker: der wird ohne
        // Verzoegerung geschrieben. Ein Kurvenereignis entsteht dagegen erst
        // nach dem Ruhefenster, sein UptimeMs liegt also spaeter als sein
        // EndUptimeMs; daraus geschaetzt waere der Versatz um genau diese
        // Wartezeit falsch.
        uint32_t versatz = 0;
        bool versatzBekannt = false;
        for (const auto& r : ereignis.rows) {
            if (r[eK] != "CURVE_REFERENCE_END") continue;
            const std::string& d = r[eBeschr];
            const size_t p = d.find("EndMs=");
            if (p == std::string::npos) continue;
            versatz =
                static_cast<uint32_t>(
                    std::strtoul(d.c_str() + p + 6, nullptr, 10)) -
                static_cast<uint32_t>(
                    std::strtoul(r[eUptime].c_str(), nullptr, 10));
            versatzBekannt = true;
            break;
        }
        // Ohne Referenzmarker bleibt nur der Rueckschluss aus den
        // Kurvenereignissen. EndUptimeMs - UptimeMs ergibt den Versatz
        // abzueglich des Schreibverzugs, und der haengt vom Abschlussgrund ab:
        // nach QUIET vergeht das volle Ruhefenster, nach REVERSAL fast nichts.
        // Das Maximum ueber alle Ereignisse trifft deshalb den Versatz am
        // besten; der erste Wert lag je nach Fahrt bis zu zwei Sekunden daneben.
        if (!versatzBekannt) {
            for (const auto& r : ereignis.rows) {
                if (r[eK] != "KURVE" || r[eEnde].empty()) continue;
                const uint32_t kandidat =
                    static_cast<uint32_t>(
                        std::strtoul(r[eEnde].c_str(), nullptr, 10)) -
                    static_cast<uint32_t>(
                        std::strtoul(r[eUptime].c_str(), nullptr, 10));
                if (!versatzBekannt || kandidat > versatz) {
                    versatz = kandidat;
                    versatzBekannt = true;
                }
            }
        }
        for (const auto& r : ereignis.rows) {
            if (r[eK] != "KURVE" || r[eEnde].empty()) continue;
            const uint32_t ende =
                static_cast<uint32_t>(std::strtoul(r[eEnde].c_str(), nullptr, 10));
            firmware.push_back(
                {static_cast<uint32_t>(
                     std::strtoul(r[eStart].c_str(), nullptr, 10)) - versatz,
                 ende - versatz,
                 static_cast<float>(std::atof(r[eWinkel].c_str())), false});
        }

        Quelle obdQuelle, gpsQuelle;
        if (hatOBD) {
            const int t = obd.col("UptimeMs");
            const int ok = obd.col("SpeedValid");
            const int v = obd.col("SpeedKmh");
            if (t >= 0 && ok >= 0 && v >= 0) {
                for (const auto& r : obd.rows) {
                    obdQuelle.add(
                        static_cast<uint32_t>(
                            std::strtoul(r[t].c_str(), nullptr, 10)),
                        static_cast<float>(std::atof(r[v].c_str())),
                        r[ok] == "1");
                }
            }
        }
        {
            const int t = gps.col("UptimeMs");
            const int ok = gps.col("SpeedValid");
            const int v = gps.col("SpeedKmh");
            for (const auto& r : gps.rows) {
                gpsQuelle.add(
                    static_cast<uint32_t>(std::strtoul(r[t].c_str(), nullptr, 10)),
                    static_cast<float>(std::atof(r[v].c_str())), r[ok] == "1");
            }
        }

        const int sT = sensor.col("UptimeMs");
        const int sH = sensor.col("Heading");
        const int sY = sensor.col("YawRateDps");
        const int sV = sensor.col("YawRateValid");
        const int sC = sensor.col("CalGyro");

        CurveDetector det;
        std::vector<CurveEvent> events;
        for (const auto& r : sensor.rows) {
            const uint32_t t =
                static_cast<uint32_t>(std::strtoul(r[sT].c_str(), nullptr, 10));
            // Reihenfolge wie in main.cpp: erst OBD, dann GPS.
            float speed = -1.0f;
            float wert = 0.0f;
            if (obdQuelle.at(t, 1000, wert)) {
                speed = wert;
            } else if (gpsQuelle.at(t, 1000, wert)) {
                speed = wert;
            }

            CurveSample sm;
            sm.timestampMs = t;
            sm.headingDeg = static_cast<float>(std::atof(r[sH].c_str()));
            sm.yawRateDps = static_cast<float>(std::atof(r[sY].c_str()));
            sm.yawRateValid = r[sV] == "1";
            sm.gyroCalibrated = std::atoi(r[sC].c_str()) == 3;
            sm.speedKmh = speed;
            CurveEvent ev;
            if (det.update(sm, ev)) events.push_back(ev);
        }
        CurveEvent letzte;
        if (det.finish(letzte)) events.push_back(letzte);

        // Zuordnung ueber Ueberlappung der Intervalle.
        size_t paare = 0;
        double winkelAbw = 0;
        double startAbw = 0;
        for (const CurveEvent& e : events) {
            Firmwareereignis* beste = nullptr;
            uint32_t besteUeberlappung = 0;
            for (Firmwareereignis& fe : firmware) {
                if (fe.zugeordnet) continue;
                if (e.startTimeMs >= fe.ende || fe.start >= e.endTimeMs) continue;
                const uint32_t a = std::max(e.startTimeMs, fe.start);
                const uint32_t b = std::min(e.endTimeMs, fe.ende);
                if (b - a >= besteUeberlappung) {
                    besteUeberlappung = b - a;
                    beste = &fe;
                }
            }
            if (beste) {
                beste->zugeordnet = true;
                ++paare;
                winkelAbw += std::fabs(e.angleDeg - beste->winkel);
                startAbw += std::fabs(
                    static_cast<double>(e.startTimeMs) - beste->start);
            }
        }
        size_t ohnePartner = 0;
        for (const Firmwareereignis& fe : firmware) {
            if (!fe.zugeordnet) ++ohnePartner;
        }

        std::printf(
            "%s  Firmware %3zu  Wiedergabe %3zu  Paare %3zu  "
            "nur Firmware %2zu  nur Wiedergabe %2zu",
            s + 9, firmware.size(), events.size(), paare, ohnePartner,
            events.size() - paare);
        if (paare) {
            std::printf("  mittlere Winkelabweichung %.1f Grad  Startversatz %.0f ms",
                        winkelAbw / paare, startAbw / paare);
        }
        std::printf("\n");

        gesamtFirmware += firmware.size();
        gesamtWiedergabe += events.size();
        gesamtPaare += paare;
        winkelAbwSumme += winkelAbw;
        startAbwSumme += startAbw;
    }

    std::printf(
        "\nGESAMT  Firmware %zu  Wiedergabe %zu  Paare %zu (%.0f %% der "
        "Firmwareereignisse)\n",
        gesamtFirmware, gesamtWiedergabe, gesamtPaare,
        100.0 * gesamtPaare / gesamtFirmware);
    if (gesamtPaare) {
        std::printf(
            "        mittlere Winkelabweichung %.1f Grad, mittlerer "
            "Startversatz %.0f ms\n",
            winkelAbwSumme / gesamtPaare, startAbwSumme / gesamtPaare);
    }
    return 0;
}

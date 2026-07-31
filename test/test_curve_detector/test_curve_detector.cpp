// Hosttests der Kurvenerkennung.
//
// Diese Tests laufen auf dem Entwicklungsrechner und benötigen weder Hardware
// noch Firmware-Flash. Sie decken die Pfade ab, die bisher ausschließlich
// durch Ausfahrten prüfbar waren.
//
//   pio test -e native

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "curve_detector.h"

namespace {

constexpr uint32_t kStepMs = 100;  // 10 Hz wie in der Firmware

// Speist eine Folge gleichförmiger Stichproben ein und sammelt die dabei
// abgeschlossenen Kurven. Der Kurs wird konsistent zur Gierrate integriert,
// damit die Mismatch-Erkennung nicht anschlägt.
struct Feeder {
    CurveDetector detector;
    std::vector<CurveEvent> events;
    uint32_t timeMs = 1000;  // nicht bei 0 starten: 0 ist der Ungültigwert
    float headingDeg = 90.0f;

    void feed(
        uint32_t count, float yawRateDps, float speedKmh,
        bool gyroValid = true) {
        for (uint32_t i = 0; i < count; ++i) {
            CurveSample s;
            s.timestampMs = timeMs;
            s.headingDeg = headingDeg;
            s.yawRateDps = yawRateDps;
            s.yawRateValid = gyroValid;
            s.gyroCalibrated = gyroValid;
            s.speedKmh = speedKmh;

            CurveEvent ev;
            if (detector.update(s, ev)) {
                events.push_back(ev);
            }

            timeMs += kStepMs;
            headingDeg += yawRateDps * (static_cast<float>(kStepMs) / 1000.0f);
            while (headingDeg >= 360.0f) headingDeg -= 360.0f;
            while (headingDeg < 0.0f) headingDeg += 360.0f;
        }
    }
};

}  // namespace

void setUp() {}
void tearDown() {}

// --- Regressionsschutz -----------------------------------------------------

// Im Stillstand darf keinerlei Ereignis entstehen. In der Sitzung
// 20260730_071913_9B018397 erzeugte die Firmware bei belegtem Stillstand
// eine Kurve mit 625 Grad. Der Test hält genau diesen Fall fest.
void test_stillstand_erzeugt_kein_ereignis() {
    Feeder f;
    // Kräftige Drehrate, aber Geschwindigkeit unter der Freigabeschwelle.
    f.feed(600, 25.0f, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

void test_unbekannte_geschwindigkeit_erzeugt_kein_ereignis() {
    Feeder f;
    f.feed(600, 25.0f, -1.0f);  // negativ bedeutet unbekannt
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

// --- Scharfe Kurve ---------------------------------------------------------

void test_scharfe_kurve_wird_erkannt() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);      // Geradeaus, Kurs einschwingen
    f.feed(60, 15.0f, 50.0f);    // 6 s mit 15 Grad/s = 90 Grad
    f.feed(30, 0.0f, 50.0f);     // 3 s ruhig -> Abschluss ueber QUIET

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    const CurveEvent& e = f.events[0];
    TEST_ASSERT_TRUE(e.valid);
    TEST_ASSERT_EQUAL_INT(1, e.direction);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 90.0f, e.angleDeg);
    TEST_ASSERT_EQUAL(CurveDetectionMode::SHARP, e.detectionMode);
    TEST_ASSERT_EQUAL(CurveCompletionReason::QUIET, e.completionReason);
    TEST_ASSERT_TRUE(e.sampleCount > 0);
    TEST_ASSERT_TRUE(e.durationMs > 0);
}

void test_linkskurve_hat_negative_richtung() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(60, -15.0f, 50.0f);
    f.feed(30, 0.0f, 50.0f);

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_EQUAL_INT(-1, f.events[0].direction);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 90.0f, f.events[0].angleDeg);
}

// --- Langgezogene Kurve ----------------------------------------------------

// Genau der Fall, den die zweistufige Erkennung ergaenzen sollte: eine
// Drehrate weit unter der Schwelle fuer scharfe Kurven, dafuer ueber Winkel
// und Weg qualifiziert.
void test_lange_kurve_wird_erkannt() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);      // 72 km/h = 20 m/s
    f.feed(90, 3.0f, 72.0f);     // 9 s mit 3 Grad/s = 27 Grad, 180 m
    f.feed(30, 0.0f, 72.0f);

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    const CurveEvent& e = f.events[0];
    TEST_ASSERT_EQUAL(CurveDetectionMode::LONG, e.detectionMode);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 27.0f, e.angleDeg);
    TEST_ASSERT_TRUE(e.distanceM >= CURVE_LONG_MIN_DISTANCE_M);
    TEST_ASSERT_TRUE(e.radiusM > 0.0f);
}

// Unterhalb der Mindestdrehrate darf nichts entstehen, egal wie lange.
void test_sehr_flache_kurve_erzeugt_kein_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(300, 0.3f, 72.0f);    // 0,3 Grad/s liegt unter CURVE_LONG_MIN_RATE_DPS
    f.feed(30, 0.0f, 72.0f);
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

// Ein zu kleiner Gesamtwinkel wird nicht protokolliert.
void test_kleiner_winkel_erzeugt_kein_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(20, 2.0f, 50.0f);     // 2 s mit 2 Grad/s = 4 Grad
    f.feed(30, 0.0f, 50.0f);
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

// --- Schwellwertgrenzen ----------------------------------------------------
//
// Die beiden folgenden Tests liegen bewusst dicht an
// CURVE_MIN_EVENT_ANGLE_DEG. Ohne sie bleibt eine Aenderung dieser Schwelle
// unbemerkt - genau das zeigte ein Mutationstest beim Aufbau dieser Suite.

// Knapp ueber der Mindestgroesse: muss ein Ereignis liefern.
void test_winkel_knapp_ueber_mindestgroesse_liefert_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(40, 3.5f, 72.0f);     // 4 s mit 3,5 Grad/s = 14 Grad
    f.feed(30, 0.0f, 72.0f);
    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 14.0f, f.events[0].angleDeg);
}

// Knapp darunter: startet zwar als Kandidat, darf aber nicht abschliessen.
void test_winkel_knapp_unter_mindestgroesse_liefert_kein_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(30, 2.9f, 72.0f);     // 3 s mit 2,9 Grad/s = 8,7 Grad
    f.feed(30, 0.0f, 72.0f);
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

// --- S-Kurve ---------------------------------------------------------------

void test_s_kurve_liefert_zwei_ereignisse_einer_gruppe() {
    Feeder f;
    f.feed(5, 0.0f, 60.0f);
    f.feed(60, 15.0f, 60.0f);    // rechts, 90 Grad
    f.feed(60, -15.0f, 60.0f);   // links, 90 Grad
    f.feed(30, 0.0f, 60.0f);

    TEST_ASSERT_TRUE(f.events.size() >= 2);
    const CurveEvent& a = f.events[0];
    const CurveEvent& b = f.events[1];
    TEST_ASSERT_EQUAL(CurveCompletionReason::REVERSAL, a.completionReason);
    TEST_ASSERT_TRUE(a.groupId != 0);
    TEST_ASSERT_EQUAL_UINT32(a.groupId, b.groupId);
    TEST_ASSERT_EQUAL_INT(1, a.direction);
    TEST_ASSERT_EQUAL_INT(-1, b.direction);
}

// --- Quellenwahl und Qualitaetsflags ---------------------------------------

void test_gyro_wird_bevorzugt_und_markiert() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(60, 15.0f, 50.0f, true);
    f.feed(30, 0.0f, 50.0f, true);

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_TRUE(
        f.events[0].qualityFlags & CURVE_QUALITY_GYRO_PRIMARY);
    TEST_ASSERT_FALSE(
        f.events[0].qualityFlags & CURVE_QUALITY_HEADING_FALLBACK);
}

// Ohne gueltige Gierrate faellt die Erkennung auf den Kurs zurueck und weist
// das aus. Genau dieser Pfad lief in allen bisherigen Fahrten, weil das
// Magnetometer unkalibriert war und der Gyro-Pfad noch nicht existierte.
void test_ohne_gyro_faellt_auf_kurs_zurueck() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f, false);
    f.feed(60, 15.0f, 50.0f, false);
    f.feed(30, 0.0f, 50.0f, false);

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_TRUE(
        f.events[0].qualityFlags & CURVE_QUALITY_HEADING_FALLBACK);
    TEST_ASSERT_FALSE(
        f.events[0].qualityFlags & CURVE_QUALITY_GYRO_PRIMARY);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 90.0f, f.events[0].angleDeg);
}

// --- Zeitluecken -----------------------------------------------------------

// Eine Luecke ueber CURVE_MAX_SAMPLE_GAP_MS darf keinen Verlauf ueber die
// Luecke hinweg fortsetzen. Die Firmware verliert im Fahrbetrieb regelmaessig
// Abtastzeitpunkte; die groesste gemessene Luecke lag bei 471 ms.
void test_grosse_zeitluecke_setzt_kurve_nicht_fort() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(30, 15.0f, 50.0f);    // 45 Grad
    f.timeMs += 3000;            // Luecke weit ueber CURVE_MAX_SAMPLE_GAP_MS
    f.feed(30, 15.0f, 50.0f);    // weitere 45 Grad nach der Luecke
    f.feed(30, 0.0f, 50.0f);

    // Beide Teilstuecke duerfen nicht zu einer 90-Grad-Kurve verschmelzen.
    for (const CurveEvent& e : f.events) {
        TEST_ASSERT_TRUE(e.angleDeg < 80.0f);
    }
}

// --- Rueckstellung ---------------------------------------------------------

void test_reset_verwirft_laufenden_verlauf() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(40, 15.0f, 50.0f);
    f.detector.reset();
    f.feed(30, 0.0f, 50.0f);
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

// --- Wiedergabe echter Messdaten -------------------------------------------

namespace {

struct ReplayRow {
    uint32_t t;
    float heading;
};

bool loadCsv(
    const std::string& path, std::vector<std::string>& header,
    std::vector<std::vector<std::string>>& rows) {
    std::ifstream in(path);
    if (!in.good()) return false;
    std::string line;
    if (!std::getline(in, line)) return false;
    {
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) header.push_back(cell);
    }
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> cells;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) cells.push_back(cell);
        if (cells.size() == header.size()) rows.push_back(cells);
    }
    return !rows.empty();
}

int columnIndex(
    const std::vector<std::string>& header, const std::string& name) {
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) return static_cast<int>(i);
    }
    return -1;
}

std::string findFixture(const std::string& relative) {
    const char* prefixes[] = {"", "../", "../../", "../../../"};
    for (const char* p : prefixes) {
        std::string candidate = std::string(p) + relative;
        std::ifstream in(candidate);
        if (in.good()) return candidate;
    }
    return std::string();
}

}  // namespace

// Spielt die reale Fahrt 20260730_095603_A41A2450 durch die Erkennung.
//
// Die Sitzung stammt aus Firmware 1.5.25 und enthaelt weder Gravitation noch
// Gierrate; der Kurs-Rueckfallpfad wird also nachgestellt. Die Geschwindigkeit
// kommt aus der OBD-Datei ueber den zeitlich naechsten Wert innerhalb von
// 1500 ms. Der Test sichert Reproduzierbarkeit und Plausibilitaet, nicht die
// Gleichheit mit dem damaligen Firmwarelauf.
void test_wiedergabe_echter_fahrt() {
    const std::string base = "testdata/20260730_095603_A41A2450/";
    const std::string sensorPath =
        findFixture(base + "road_sensor_20260730_095603_A41A2450.csv");
    const std::string obdPath =
        findFixture(base + "road_obd_20260730_095603_A41A2450.csv");
    if (sensorPath.empty() || obdPath.empty()) {
        TEST_IGNORE_MESSAGE("Messdaten nicht gefunden - Wiedergabe uebersprungen");
        return;
    }

    std::vector<std::string> sh, oh;
    std::vector<std::vector<std::string>> sr, orow;
    TEST_ASSERT_TRUE(loadCsv(sensorPath, sh, sr));
    TEST_ASSERT_TRUE(loadCsv(obdPath, oh, orow));

    const int sT = columnIndex(sh, "UptimeMs");
    const int sH = columnIndex(sh, "Heading");
    const int oT = columnIndex(oh, "UptimeMs");
    const int oV = columnIndex(oh, "SpeedValid");
    const int oS = columnIndex(oh, "SpeedKmh");
    TEST_ASSERT_TRUE(sT >= 0 && sH >= 0 && oT >= 0 && oV >= 0 && oS >= 0);

    std::vector<std::pair<uint32_t, float>> speeds;
    for (const auto& r : orow) {
        if (r[oV] != "1") continue;
        speeds.push_back(
            {static_cast<uint32_t>(std::strtoul(r[oT].c_str(), nullptr, 10)),
             static_cast<float>(std::atof(r[oS].c_str()))});
    }
    TEST_ASSERT_TRUE(speeds.size() > 100);

    CurveDetector detector;
    std::vector<CurveEvent> events;
    size_t cursor = 0;
    for (const auto& r : sr) {
        const uint32_t t =
            static_cast<uint32_t>(std::strtoul(r[sT].c_str(), nullptr, 10));
        while (cursor + 1 < speeds.size() && speeds[cursor + 1].first <= t) {
            ++cursor;
        }
        float speed = -1.0f;
        if (!speeds.empty()) {
            const uint32_t st = speeds[cursor].first;
            const uint32_t diff = t > st ? t - st : st - t;
            if (diff <= 1500) speed = speeds[cursor].second;
        }

        CurveSample s;
        s.timestampMs = t;
        s.headingDeg = static_cast<float>(std::atof(r[sH].c_str()));
        s.yawRateValid = false;   // 1.5.25 lieferte keine Gierrate
        s.gyroCalibrated = false;
        s.speedKmh = speed;

        CurveEvent ev;
        if (detector.update(s, ev)) events.push_back(ev);
    }

    std::printf("\nWiedergabe 095603: %zu Stichproben, %zu Kurven\n",
                sr.size(), events.size());

    // Plausibilitaet jedes einzelnen Ereignisses.
    uint32_t previousEnd = 0;
    for (const CurveEvent& e : events) {
        TEST_ASSERT_TRUE(e.valid);
        TEST_ASSERT_TRUE(e.angleDeg >= CURVE_MIN_EVENT_ANGLE_DEG);
        TEST_ASSERT_TRUE(e.angleDeg <= 180.0f);   // Regressionsschutz 625 Grad
        TEST_ASSERT_TRUE(e.sampleCount > 0);
        TEST_ASSERT_TRUE(e.durationMs > 0);
        TEST_ASSERT_TRUE(e.radiusM > 0.0f);
        TEST_ASSERT_TRUE(e.meanSpeedKmh >= ROAD_EVENT_MIN_SPEED_KMH);
        TEST_ASSERT_TRUE(e.endTimeMs >= previousEnd);
        previousEnd = e.endTimeMs;
    }

    // Festwert statt Korridor.
    //
    // Ein weiter Korridor faengt keine Verhaltensaenderung: Beim Aufbau
    // dieser Suite blieb eine Verdopplung von CURVE_MIN_EVENT_ANGLE_DEG
    // unbemerkt. Der Wert ist deshalb exakt festgeschrieben.
    //
    // Zum Vergleich: Die geometrische Auswertung der GPS-Spur ergab fuer diese
    // Fahrt 48 Kurven ab 10 Grad, die Firmware 1.5.25 protokollierte damals
    // 24. Diese Wiedergabe nutzt den Kurs-Rueckfallpfad und die
    // OBD-Geschwindigkeit ueber den naechsten Nachbarn; sie ist deshalb nicht
    // deckungsgleich mit dem Firmwarelauf.
    //
    // Aendert sich der Wert, ist das kein Testfehler, sondern eine
    // Verhaltensaenderung: Sie ist zu pruefen, zu begruenden und der Festwert
    // hier bewusst nachzuziehen.
    const size_t kErwarteteKurven = 32;
    TEST_ASSERT_EQUAL_UINT32(kErwarteteKurven, events.size());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_stillstand_erzeugt_kein_ereignis);
    RUN_TEST(test_unbekannte_geschwindigkeit_erzeugt_kein_ereignis);
    RUN_TEST(test_scharfe_kurve_wird_erkannt);
    RUN_TEST(test_linkskurve_hat_negative_richtung);
    RUN_TEST(test_lange_kurve_wird_erkannt);
    RUN_TEST(test_sehr_flache_kurve_erzeugt_kein_ereignis);
    RUN_TEST(test_kleiner_winkel_erzeugt_kein_ereignis);
    RUN_TEST(test_winkel_knapp_ueber_mindestgroesse_liefert_ereignis);
    RUN_TEST(test_winkel_knapp_unter_mindestgroesse_liefert_kein_ereignis);
    RUN_TEST(test_s_kurve_liefert_zwei_ereignisse_einer_gruppe);
    RUN_TEST(test_gyro_wird_bevorzugt_und_markiert);
    RUN_TEST(test_ohne_gyro_faellt_auf_kurs_zurueck);
    RUN_TEST(test_grosse_zeitluecke_setzt_kurve_nicht_fort);
    RUN_TEST(test_reset_verwirft_laufenden_verlauf);
    RUN_TEST(test_wiedergabe_echter_fahrt);
    return UNITY_END();
}

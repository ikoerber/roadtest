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

// --- Mindestfahrweg --------------------------------------------------------
//
// Die Geschwindigkeitsfreigabe allein reicht nicht: GPS-Drift im Stand wird
// mit bis zu 8 km/h als gueltig ausgewiesen. In 20260731_160627_085A6E1C
// entstanden daraus zwei Kurven mit 2,5 und 3,4 m Radius ueber 1,6 und 1,8 m
// Fahrweg, waehrend das Geraet in der Hand gedreht wurde.

// 7,2 km/h sind 2 m/s; 6 s ergeben 12 m und damit knapp ueber der Grenze.
void test_fahrweg_knapp_ueber_mindestwert_liefert_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 7.2f);
    f.feed(60, 15.0f, 7.2f);     // 90 Grad ueber 12 m
    f.feed(30, 0.0f, 7.2f);
    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_TRUE(f.events[0].distanceM >= CURVE_MIN_EVENT_DISTANCE_M);
}

// Dieselbe Drehung ueber 8 m Fahrweg bleibt unter der Grenze.
void test_fahrweg_knapp_unter_mindestwert_liefert_kein_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 7.2f);
    f.feed(40, 22.5f, 7.2f);     // ebenfalls 90 Grad, aber nur 8 m
    f.feed(30, 0.0f, 7.2f);
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());
}

// --- Mindest-Querbeschleunigung --------------------------------------------
//
// Massgeblich ist die Querbeschleunigung, nicht der Radius. Ein Bogen mit
// 500 m Radius bei 90 km/h wurde von der Beifahrerseite ausdruecklich als
// Kurve markiert und traegt 1,5 m/s²; ein Autobahnbogen mit 1450 m Radius
// bei 116 km/h dagegen 0,4 und ist keine gefahrene Kurve.

// 72 km/h sind 20 m/s; 1,3 Grad/s ergeben 0,45 m/s².
void test_querbeschleunigung_knapp_ueber_mindestwert_liefert_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(120, 1.3f, 72.0f);    // 12 s, 15,6 Grad, 240 m
    f.feed(30, 0.0f, 72.0f);
    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_TRUE(
        f.events[0].meanLateralAccel >= CURVE_MIN_EVENT_LATERAL_ACCEL);
}

// 1,0 Grad/s bei gleicher Geschwindigkeit sind 0,35 m/s². Der Winkel
// qualifiziert weiterhin, die Kurve wird aber nicht gefahren.
void test_querbeschleunigung_knapp_unter_mindestwert_liefert_kein_ereignis() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(150, 1.0f, 72.0f);    // 15 s, 15 Grad, 300 m
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

// --- Ruhefenster gegen Gierratenrauschen -----------------------------------
//
// Auf nachgewiesen gerader Strecke rauscht die Gierrate um mehr als
// CURVE_LONG_MIN_RATE_DPS. Frueher setzte jede Spitze in Kurvenrichtung das
// Ruhefenster zurueck; Ereignisse liefen dadurch ueber mehrere Kurven hinweg
// zusammen. Entscheidend ist die Netto-Kursaenderung im Fenster, nicht die
// einzelne Stichprobe.

namespace {

// Speist abwechselnd Werte ein, deren Betrag ueber CURVE_LONG_MIN_RATE_DPS
// liegt, deren Summe sich aber aufhebt.
void feedNoise(Feeder& f, uint32_t count, float amplitudeDps, float speedKmh) {
    for (uint32_t i = 0; i < count; ++i) {
        f.feed(1, (i % 2 == 0) ? amplitudeDps : -amplitudeDps, speedKmh);
    }
}

}  // namespace

void test_rauschen_haelt_kurve_nicht_offen() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(60, 15.0f, 50.0f);    // 90 Grad
    feedNoise(f, 40, 1.5f, 50.0f);  // 4 s Rauschen ueber der Mindestdrehrate

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    TEST_ASSERT_EQUAL(
        CurveCompletionReason::QUIET, f.events[0].completionReason);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 90.0f, f.events[0].angleDeg);
}

// Zwei durch eine Rauschstrecke getrennte Kurven duerfen nicht zu einem
// Ereignis verschmelzen. Genau das trat in den Fahrten des 31.07.2026 auf:
// 47 von 128 Ereignissen deckten unter 60 Prozent ihrer Dauer mit echten
// Drehstichproben ab.
void test_rauschen_trennt_zwei_kurven() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(40, 15.0f, 50.0f);       // 60 Grad
    feedNoise(f, 60, 1.5f, 50.0f);  // 6 s Rauschen
    f.feed(40, 15.0f, 50.0f);       // erneut 60 Grad, gleiche Richtung
    f.feed(30, 0.0f, 50.0f);

    TEST_ASSERT_EQUAL_UINT32(2, f.events.size());
    for (const CurveEvent& e : f.events) {
        TEST_ASSERT_FLOAT_WITHIN(6.0f, 60.0f, e.angleDeg);
    }
}

// Knapp ueber der Fortsetzungsschwelle: eine echte langgezogene Kurve haelt
// das Ruhefenster offen. CURVE_QUIET_MIN_NET_ANGLE_DEG ueber
// CURVE_END_QUIET_MS entspricht 1,0 Grad/s.
void test_langsame_kurve_knapp_ueber_fortsetzungsschwelle_bleibt_offen() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(40, 4.0f, 72.0f);     // 16 Grad, qualifiziert die Kurve
    f.feed(100, 1.4f, 72.0f);    // 10 s knapp ueber 1,0 Grad/s
    f.feed(30, 0.0f, 72.0f);

    TEST_ASSERT_EQUAL_UINT32(1, f.events.size());
    // Der langsame Teil steckt mit im Ereignis, es wurde also nicht getrennt.
    TEST_ASSERT_TRUE(f.events[0].angleDeg > 25.0f);
}

// Knapp darunter: der langsame Teil gilt als Ruhe, die Kurve schliesst ab.
void test_langsame_kurve_knapp_unter_fortsetzungsschwelle_schliesst_ab() {
    Feeder f;
    f.feed(5, 0.0f, 72.0f);
    f.feed(40, 4.0f, 72.0f);     // 16 Grad
    f.feed(100, 0.9f, 72.0f);    // 10 s knapp unter 1,0 Grad/s
    f.feed(30, 0.0f, 72.0f);

    TEST_ASSERT_TRUE(f.events.size() >= 1);
    TEST_ASSERT_EQUAL(
        CurveCompletionReason::QUIET, f.events[0].completionReason);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 16.0f, f.events[0].angleDeg);
}

// --- Abschluss beim Sitzungsende -------------------------------------------

// In 20260731_152148_452707FB endete die Messfahrt zwei Sekunden nach einer
// Kurve mit 44 Grad Kursaenderung. Das Ruhefenster lief nicht mehr ab, das
// Ereignis fehlte vollstaendig.
void test_finish_schliesst_laufende_kurve_ab() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(60, 15.0f, 50.0f);    // 90 Grad, danach sofort Sitzungsende
    TEST_ASSERT_EQUAL_UINT32(0, f.events.size());

    CurveEvent letzte;
    TEST_ASSERT_TRUE(f.detector.finish(letzte));
    TEST_ASSERT_TRUE(letzte.valid);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 90.0f, letzte.angleDeg);
    TEST_ASSERT_EQUAL(
        CurveCompletionReason::SESSION_END, letzte.completionReason);
    TEST_ASSERT_TRUE(letzte.durationMs > 0);
}

// Ohne laufende Kurve entsteht kein Ereignis, und ein zu kleiner Winkel
// bleibt auch beim Abschluss unter der Meldeschwelle.
void test_finish_ohne_kurve_liefert_nichts() {
    Feeder f;
    f.feed(30, 0.0f, 50.0f);
    CurveEvent letzte;
    TEST_ASSERT_FALSE(f.detector.finish(letzte));

    Feeder g;
    g.feed(5, 0.0f, 50.0f);
    g.feed(20, 2.0f, 50.0f);     // 4 Grad, unter CURVE_MIN_EVENT_ANGLE_DEG
    CurveEvent klein;
    TEST_ASSERT_FALSE(g.detector.finish(klein));
}

// finish() verwirft den Zustand, damit die naechste Messfahrt nicht auf einem
// alten Verlauf aufsetzt.
void test_finish_verwirft_den_zustand() {
    Feeder f;
    f.feed(5, 0.0f, 50.0f);
    f.feed(60, 15.0f, 50.0f);
    CurveEvent letzte;
    TEST_ASSERT_TRUE(f.detector.finish(letzte));
    CurveEvent nochmal;
    TEST_ASSERT_FALSE(f.detector.finish(nochmal));
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
    // Der Wert stieg mit 1.5.29 von 32 auf 42, weil sich das Ruhefenster
    // nicht mehr von Rauschen offen halten laesst und zusammengelaufene
    // Ereignisse sich wieder auftrennen. Mindestfahrweg und
    // Mindest-Querbeschleunigung nehmen davon zwei wieder heraus.
    const size_t kErwarteteKurven = 40;
    TEST_ASSERT_EQUAL_UINT32(kErwarteteKurven, events.size());
}

// --- Wiedergabe der Referenzfahrten ----------------------------------------

namespace {

struct Referenzintervall {
    uint32_t startMs;
    uint32_t endeMs;
    std::string typ;
};

// Zerlegt "Id=27;Type=RECHTS_NORMAL;StartMs=...;EndMs=..." in seine Felder.
std::string descriptionField(const std::string& description,
                             const std::string& key) {
    std::stringstream ss(description);
    std::string part;
    while (std::getline(ss, part, ';')) {
        const size_t pos = part.find('=');
        if (pos != std::string::npos && part.substr(0, pos) == key) {
            return part.substr(pos + 1);
        }
    }
    return std::string();
}

// Wie loadCsv, ergaenzt aber leere Felder am Zeilenende. Die Ereignisdatei
// laesst die Kurvenspalten bei Referenzmarkern leer; ohne Auffuellen fallen
// genau diese Zeilen heraus.
bool loadEventCsv(
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
        while (cells.size() < header.size()) cells.push_back("");
        if (cells.size() == header.size()) rows.push_back(cells);
    }
    return !rows.empty();
}

struct Referenzergebnis {
    size_t referenzen = 0;
    size_t treffer = 0;
    size_t ereignisse = 0;
    size_t geringeAbdeckung = 0;
};

// Spielt eine Fahrt mit Beifahrer-Kurventest durch die Erkennung und misst
// sie gegen die von Hand gesetzten Referenzintervalle.
bool replayReferenceDrive(const std::string& session, Referenzergebnis& out) {
    const std::string base = "testdata/" + session + "/";
    const std::string sensorPath =
        findFixture(base + "road_sensor_" + session + ".csv");
    const std::string gpsPath =
        findFixture(base + "road_gps_" + session + ".csv");
    const std::string eventPath =
        findFixture(base + "road_event_" + session + ".csv");
    if (sensorPath.empty() || gpsPath.empty() || eventPath.empty()) {
        return false;
    }

    std::vector<std::string> sh, gh, eh;
    std::vector<std::vector<std::string>> sr, gr, er;
    if (!loadCsv(sensorPath, sh, sr)) return false;
    if (!loadCsv(gpsPath, gh, gr)) return false;
    if (!loadEventCsv(eventPath, eh, er)) return false;

    // Referenzmarker tragen Geraetezeit, die Ereignisspalte Sitzungszeit.
    // Die Differenz ist der Versatz zwischen beiden Zeitbasen.
    const int eKind = columnIndex(eh, "Ereignis");
    const int eDesc = columnIndex(eh, "Beschreibung");
    const int eTime = columnIndex(eh, "UptimeMs");
    if (eKind < 0 || eDesc < 0 || eTime < 0) return false;

    std::vector<Referenzintervall> referenzen;
    for (const auto& r : er) {
        if (r[eKind] != "CURVE_REFERENCE_END") continue;
        const std::string typ = descriptionField(r[eDesc], "Type");
        const std::string startText = descriptionField(r[eDesc], "StartMs");
        const std::string endText = descriptionField(r[eDesc], "EndMs");
        if (typ.empty() || startText.empty() || endText.empty()) continue;
        const uint32_t endeGeraet =
            static_cast<uint32_t>(std::strtoul(endText.c_str(), nullptr, 10));
        const uint32_t versatz =
            endeGeraet -
            static_cast<uint32_t>(std::strtoul(r[eTime].c_str(), nullptr, 10));
        if (typ == "GERADE") continue;  // Geraden sind keine Trefferpruefung
        referenzen.push_back(
            {static_cast<uint32_t>(
                 std::strtoul(startText.c_str(), nullptr, 10)) - versatz,
             endeGeraet - versatz, typ});
    }
    if (referenzen.empty()) return false;

    const int gT = columnIndex(gh, "UptimeMs");
    const int gV = columnIndex(gh, "SpeedValid");
    const int gS = columnIndex(gh, "SpeedKmh");
    if (gT < 0 || gV < 0 || gS < 0) return false;
    std::vector<std::pair<uint32_t, float>> speeds;
    for (const auto& r : gr) {
        if (r[gV] != "1") continue;
        speeds.push_back(
            {static_cast<uint32_t>(std::strtoul(r[gT].c_str(), nullptr, 10)),
             static_cast<float>(std::atof(r[gS].c_str()))});
    }
    if (speeds.empty()) return false;

    const int sT = columnIndex(sh, "UptimeMs");
    const int sH = columnIndex(sh, "Heading");
    const int sY = columnIndex(sh, "YawRateDps");
    const int sV = columnIndex(sh, "YawRateValid");
    const int sC = columnIndex(sh, "CalGyro");
    if (sT < 0 || sH < 0 || sY < 0 || sV < 0 || sC < 0) return false;

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
        const uint32_t st = speeds[cursor].first;
        const uint32_t diff = t > st ? t - st : st - t;
        if (diff <= 1500) speed = speeds[cursor].second;

        CurveSample s;
        s.timestampMs = t;
        s.headingDeg = static_cast<float>(std::atof(r[sH].c_str()));
        s.yawRateDps = static_cast<float>(std::atof(r[sY].c_str()));
        s.yawRateValid = r[sV] == "1";
        s.gyroCalibrated = std::atoi(r[sC].c_str()) == 3;
        s.speedKmh = speed;

        CurveEvent ev;
        if (detector.update(s, ev)) events.push_back(ev);
    }
    CurveEvent letzte;
    if (detector.finish(letzte)) events.push_back(letzte);

    out.referenzen = referenzen.size();
    out.ereignisse = events.size();
    for (const Referenzintervall& ref : referenzen) {
        for (const CurveEvent& e : events) {
            if (e.startTimeMs < ref.endeMs && ref.startMs < e.endTimeMs) {
                ++out.treffer;
                break;
            }
        }
    }
    for (const CurveEvent& e : events) {
        if (e.durationMs == 0) continue;
        // Anteil der Ereignisdauer, der von echten Drehstichproben belegt ist.
        const double abdeckung =
            static_cast<double>(e.sampleCount) * kStepMs / e.durationMs;
        if (abdeckung < 0.6) ++out.geringeAbdeckung;
    }
    return true;
}

}  // namespace

// Misst die Erkennung gegen die von Hand gesetzten Referenzintervalle der
// Beifahrerseite vom 31.07.2026.
//
// Zwei Kennzahlen sind festgeschrieben:
//
//   Treffer          - wieviele Referenzkurven ein Ereignis ueberlappt.
//   geringeAbdeckung - Ereignisse, deren Dauer zu weniger als 60 Prozent von
//                      echten Drehstichproben belegt ist. Solche Ereignisse
//                      umfassen mehrere Kurven; Radius, Dauer und Mittelwerte
//                      beschreiben dann keine einzelne Kurve mehr.
//
// Aendern sich die Werte, ist das kein Testfehler, sondern eine
// Verhaltensaenderung: pruefen, begruenden und den Festwert nachziehen.
void test_wiedergabe_referenzfahrten() {
    const char* sessions[] = {
        "20260731_151337_DAB245DC", "20260731_152148_452707FB",
        "20260731_153224_503BD642", "20260731_153850_6C7F041E",
        "20260731_155106_3AEFA823"};

    Referenzergebnis gesamt;
    size_t gefunden = 0;
    for (const char* session : sessions) {
        Referenzergebnis einzeln;
        if (!replayReferenceDrive(session, einzeln)) continue;
        ++gefunden;
        gesamt.referenzen += einzeln.referenzen;
        gesamt.treffer += einzeln.treffer;
        gesamt.ereignisse += einzeln.ereignisse;
        gesamt.geringeAbdeckung += einzeln.geringeAbdeckung;
    }
    if (gefunden == 0) {
        TEST_IGNORE_MESSAGE("Messdaten nicht gefunden - Wiedergabe uebersprungen");
        return;
    }
    TEST_ASSERT_EQUAL_UINT32(5, gefunden);

    std::printf(
        "\nReferenzfahrten: %zu Referenzkurven, %zu erkannt, %zu Ereignisse, "
        "%zu mit Abdeckung unter 60 %%\n",
        gesamt.referenzen, gesamt.treffer, gesamt.ereignisse,
        gesamt.geringeAbdeckung);

    TEST_ASSERT_EQUAL_UINT32(55, gesamt.referenzen);

    // 52 von 55. Die drei fehlenden Referenzen drehen netto 5,7, 6,4 und
    // 10,0 Grad und liegen damit unter CURVE_MIN_EVENT_ANGLE_DEG; sie sind
    // von der Beifahrerseite als Kurve markiert, qualifizieren aber nach der
    // geltenden Regel nicht. Vor 1.5.29 wurden zwei davon nur deshalb
    // getroffen, weil zusammengelaufene Nachbarereignisse ihr Zeitfenster
    // zufaellig ueberlappten.
    TEST_ASSERT_EQUAL_UINT32(52, gesamt.treffer);

    // Vor 1.5.29 lagen hier 47 von 128 Ereignissen.
    TEST_ASSERT_EQUAL_UINT32(7, gesamt.geringeAbdeckung);

    // 128 vor 1.5.29, dann 158 durch die aufgetrennten Ereignisse, davon
    // nehmen Mindestfahrweg und Mindest-Querbeschleunigung fuenf wieder
    // heraus. Ueber alle neun Fahrten des Tages entfernen die beiden Regeln
    // 25 Ereignisse, darunter alle vier bekannten Fehlalarme aus GPS-Drift
    // im Stand und beim Rangieren, ohne eine Referenzkurve zu verlieren.
    TEST_ASSERT_EQUAL_UINT32(153, gesamt.ereignisse);
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
    RUN_TEST(test_fahrweg_knapp_ueber_mindestwert_liefert_ereignis);
    RUN_TEST(test_fahrweg_knapp_unter_mindestwert_liefert_kein_ereignis);
    RUN_TEST(test_querbeschleunigung_knapp_ueber_mindestwert_liefert_ereignis);
    RUN_TEST(
        test_querbeschleunigung_knapp_unter_mindestwert_liefert_kein_ereignis);
    RUN_TEST(test_s_kurve_liefert_zwei_ereignisse_einer_gruppe);
    RUN_TEST(test_gyro_wird_bevorzugt_und_markiert);
    RUN_TEST(test_ohne_gyro_faellt_auf_kurs_zurueck);
    RUN_TEST(test_grosse_zeitluecke_setzt_kurve_nicht_fort);
    RUN_TEST(test_rauschen_haelt_kurve_nicht_offen);
    RUN_TEST(test_rauschen_trennt_zwei_kurven);
    RUN_TEST(test_langsame_kurve_knapp_ueber_fortsetzungsschwelle_bleibt_offen);
    RUN_TEST(test_langsame_kurve_knapp_unter_fortsetzungsschwelle_schliesst_ab);
    RUN_TEST(test_finish_schliesst_laufende_kurve_ab);
    RUN_TEST(test_finish_ohne_kurve_liefert_nichts);
    RUN_TEST(test_finish_verwirft_den_zustand);
    RUN_TEST(test_reset_verwirft_laufenden_verlauf);
    RUN_TEST(test_wiedergabe_echter_fahrt);
    RUN_TEST(test_wiedergabe_referenzfahrten);
    return UNITY_END();
}

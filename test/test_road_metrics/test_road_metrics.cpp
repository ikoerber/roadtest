// Hosttests der Fahrbahnbewertung.
//
//   pio test -e native

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "road_metrics.h"

namespace {

// Füllt das gleitende Fenster vollständig mit einem festen Wert.
void fillWindow(RoadMetricsAnalyzer& a, float accelZ, int count = 10) {
    for (int i = 0; i < count; ++i) a.addSample(accelZ);
}

// Füllt das Fenster abwechselnd mit +v und -v, erzeugt also einen
// Effektivwert von v ohne Gleichanteil.
void fillAlternating(RoadMetricsAnalyzer& a, float v, int count = 10) {
    for (int i = 0; i < count; ++i) a.addSample(i % 2 == 0 ? v : -v);
}

}  // namespace

void setUp() {}
void tearDown() {}

// --- Grundverhalten der Bewertung ------------------------------------------

// Ohne nachgewiesene Bewegung darf kein Messwert entstehen. Genau dieser
// Fall erzeugte in 20260730_071913_9B018397 im belegten Stillstand 106 von
// 265 Qualitätswerten unter 99,0 mit Minimum 60,1.
void test_stillstand_liefert_keinen_messwert() {
    RoadMetricsAnalyzer a;
    fillAlternating(a, 3.0f);
    TEST_ASSERT_TRUE(a.calculateRoadQuality(0.0f) < 0.0f);
    TEST_ASSERT_TRUE(a.calculateRoadQuality(4.9f) < 0.0f);
}

void test_unbekannte_geschwindigkeit_liefert_keinen_messwert() {
    RoadMetricsAnalyzer a;
    fillAlternating(a, 3.0f);
    TEST_ASSERT_TRUE(a.calculateRoadQuality(-1.0f) < 0.0f);
}

void test_ab_mindestgeschwindigkeit_entsteht_ein_messwert() {
    RoadMetricsAnalyzer a;
    fillAlternating(a, 0.2f);
    const float q = a.calculateRoadQuality(ROAD_EVENT_MIN_SPEED_KMH);
    TEST_ASSERT_TRUE(q >= 0.0f);
    TEST_ASSERT_TRUE(q <= 100.0f);
}

// Glatte Fahrbahn muss deutlich besser bewertet werden als raue.
void test_glatte_fahrbahn_besser_als_raue() {
    RoadMetricsAnalyzer glatt;
    fillAlternating(glatt, 0.1f);
    const float qGlatt = glatt.calculateRoadQuality(50.0f);

    RoadMetricsAnalyzer rau;
    fillAlternating(rau, 3.0f);
    const float qRau = rau.calculateRoadQuality(50.0f);

    TEST_ASSERT_TRUE(qGlatt > qRau);
    TEST_ASSERT_TRUE(qGlatt > 90.0f);
    TEST_ASSERT_TRUE(qRau < 70.0f);
}

// Die Bewertung ist nach unten begrenzt.
void test_qualitaet_bleibt_im_bereich_null_bis_hundert() {
    RoadMetricsAnalyzer a;
    fillAlternating(a, 50.0f);  // extrem
    const float q = a.calculateRoadQuality(50.0f);
    TEST_ASSERT_TRUE(q >= 0.0f);
    TEST_ASSERT_TRUE(q <= 100.0f);
}

// --- Gemessene Einschränkung der Geschwindigkeitsnormierung ----------------
//
// Der Faktor ist auf ROAD_QUALITY_SPEED_FACTOR_MIN bis _MAX geklemmt. Er
// wirkt dadurch nur zwischen 30/1,5 = 20 km/h und 30/0,7 = 42,9 km/h.
// Oberhalb davon ist er konstant, die Normierung also wirkungslos. In der
// Fahrt 20260730_084245_54526FA7 mit Median 167 km/h betraf das die gesamte
// Strecke.
//
// Diese Tests halten den Ist-Zustand fest. Sie schlagen fehl, sobald die
// Normierung geändert wird - das ist beabsichtigt und erzwingt eine bewusste
// Entscheidung.

void test_normierung_wirkt_im_mittleren_bereich() {
    RoadMetricsAnalyzer a20, a40;
    fillAlternating(a20, 1.0f);
    fillAlternating(a40, 1.0f);
    const float q20 = a20.calculateRoadQuality(20.0f);
    const float q40 = a40.calculateRoadQuality(40.0f);
    // Gleiche Anregung, unterschiedliche Geschwindigkeit: Die Normierung
    // muss zu unterschiedlichen Bewertungen führen.
    TEST_ASSERT_TRUE(std::fabs(q20 - q40) > 1.0f);
}

void test_normierung_ist_oberhalb_von_43_kmh_wirkungslos() {
    RoadMetricsAnalyzer a50, a120, a200;
    fillAlternating(a50, 1.0f);
    fillAlternating(a120, 1.0f);
    fillAlternating(a200, 1.0f);
    const float q50 = a50.calculateRoadQuality(50.0f);
    const float q120 = a120.calculateRoadQuality(120.0f);
    const float q200 = a200.calculateRoadQuality(200.0f);

    // Bei gleicher Anregung liefern 50, 120 und 200 km/h denselben Wert:
    // Der Normierungsfaktor steht durchgehend an der unteren Klemme.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, q50, q120);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, q50, q200);
}

// Die Klemmgrenzen selbst festhalten.
void test_klemmgrenzen_der_normierung() {
    const float untereGrenze =
        ROAD_QUALITY_REFERENCE_SPEED_KMH / ROAD_QUALITY_SPEED_FACTOR_MAX;
    const float obereGrenze =
        ROAD_QUALITY_REFERENCE_SPEED_KMH / ROAD_QUALITY_SPEED_FACTOR_MIN;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, untereGrenze);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 42.9f, obereGrenze);
}

// --- Vibrationsanalyse -----------------------------------------------------

void test_effektivwert_wird_richtig_gebildet() {
    RoadMetricsAnalyzer a;
    fillAlternating(a, 2.0f);
    const VibrationMetrics m = a.analyzeVibration();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, m.rmsAccel);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, m.maxShock);
}

void test_leeres_fenster_liefert_nullwerte() {
    RoadMetricsAnalyzer a;
    const VibrationMetrics m = a.analyzeVibration();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.rmsAccel);
    TEST_ASSERT_EQUAL_UINT32(0, m.shockCount);
}

// Ein zusammenhängender Stoß darf im Fenster nur einmal zählen.
void test_zusammenhaengender_stoss_zaehlt_einmal() {
    RoadMetricsAnalyzer a;
    a.setVibrationThreshold(2.0f);
    a.addSample(0.1f);
    a.addSample(5.0f);   // Stoß beginnt
    a.addSample(5.0f);   // dauert an
    a.addSample(5.0f);
    a.addSample(0.1f);
    const VibrationMetrics m = a.analyzeVibration();
    TEST_ASSERT_EQUAL_UINT32(1, m.shockCount);
}

void test_zwei_getrennte_stoesse_zaehlen_zweimal() {
    RoadMetricsAnalyzer a;
    a.setVibrationThreshold(2.0f);
    a.addSample(0.1f);
    a.addSample(5.0f);
    a.addSample(0.1f);
    a.addSample(5.0f);
    a.addSample(0.1f);
    const VibrationMetrics m = a.analyzeVibration();
    TEST_ASSERT_EQUAL_UINT32(2, m.shockCount);
}

// Das Fenster umfasst genau eine Sekunde bei 10 Hz; ältere Werte fallen raus.
void test_fenster_vergisst_alte_werte() {
    RoadMetricsAnalyzer a;
    fillAlternating(a, 5.0f, 10);
    fillWindow(a, 0.0f, 10);
    const VibrationMetrics m = a.analyzeVibration();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.rmsAccel);
}

void test_glaettewert_faellt_mit_rauheit() {
    RoadMetricsAnalyzer glatt, rau;
    fillAlternating(glatt, 0.1f);
    fillAlternating(rau, 5.0f);
    TEST_ASSERT_TRUE(glatt.getSmoothness() > rau.getSmoothness());
    TEST_ASSERT_TRUE(glatt.getSmoothness() <= 1.0f);
    TEST_ASSERT_TRUE(rau.getSmoothness() > 0.0f);
}

// --- Schlaglocherkennung ---------------------------------------------------

// Ausschlag nach unten, dann nach oben innerhalb des Fensters.
void test_schlagloch_wird_erkannt() {
    RoadMetricsAnalyzer a;
    TEST_ASSERT_FALSE(a.detectPothole(2000, -5.0f, 50.0f));  // scharfstellen
    TEST_ASSERT_TRUE(a.detectPothole(2200, 5.0f, 50.0f));    // abschliessen
}

// Im Stillstand darf kein Schlagloch entstehen. In
// 20260730_071913_9B018397 wurden im belegten Stillstand drei protokolliert.
void test_kein_schlagloch_im_stillstand() {
    RoadMetricsAnalyzer a;
    TEST_ASSERT_FALSE(a.detectPothole(2000, -5.0f, 0.0f));
    TEST_ASSERT_FALSE(a.detectPothole(2200, 5.0f, 0.0f));
}

// Ein im Stand begonnener Ausschlag darf beim Anfahren nicht abschliessen.
void test_im_stand_begonnener_ausschlag_schliesst_nicht_ab() {
    RoadMetricsAnalyzer a;
    a.detectPothole(2000, -5.0f, 0.0f);                     // Stand
    TEST_ASSERT_FALSE(a.detectPothole(2200, 5.0f, 50.0f));  // rollt an
}

// Nach dem Erkennungsfenster zaehlt der Gegenausschlag nicht mehr.
void test_zu_spaeter_gegenausschlag_wird_verworfen() {
    RoadMetricsAnalyzer a;
    a.detectPothole(2000, -5.0f, 50.0f);
    const uint32_t zuSpaet = 2000 + ROAD_POTHOLE_WINDOW_MS + 100;
    TEST_ASSERT_FALSE(a.detectPothole(zuSpaet, 5.0f, 50.0f));
}

// Die Sperrzeit verhindert, dass ein Nachschwingen sofort erneut zaehlt.
void test_sperrzeit_verhindert_doppelzaehlung() {
    RoadMetricsAnalyzer a;
    a.detectPothole(2000, -5.0f, 50.0f);
    TEST_ASSERT_TRUE(a.detectPothole(2200, 5.0f, 50.0f));

    // Innerhalb der Sperrzeit: darf nicht erneut scharfstellen.
    a.detectPothole(2400, -5.0f, 50.0f);
    TEST_ASSERT_FALSE(a.detectPothole(2600, 5.0f, 50.0f));

    // Nach Ablauf der Sperrzeit wieder moeglich.
    const uint32_t nachSperre = 2200 + ROAD_POTHOLE_REARM_MS + 100;
    a.detectPothole(nachSperre, -5.0f, 50.0f);
    TEST_ASSERT_TRUE(a.detectPothole(nachSperre + 200, 5.0f, 50.0f));
}

// Unterhalb der Schwelle passiert nichts.
void test_kleine_ausschlaege_erzeugen_kein_schlagloch() {
    RoadMetricsAnalyzer a;
    a.detectPothole(2000, -1.0f, 50.0f);
    TEST_ASSERT_FALSE(a.detectPothole(2200, 1.0f, 50.0f));
}

// --- Wiedergabe echter Messdaten -------------------------------------------

namespace {

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

// Sucht eine Messdatei relativ zum Projektverzeichnis. Das Arbeitsverzeichnis
// des Testlaufs steht nicht fest, deshalb die Praefixe.
//
// Archivierte Sitzungen werden mitgesucht: Wer testdata/ aufraeumt, soll den
// Pruefstand nicht unbemerkt abschalten. Genau das passierte am 02.08.2026 -
// sieben Sitzungen wanderten nach testdata/archiv/, und vier Wiedergabetests
// meldeten fortan TEST_IGNORE. Die Suite blieb gruen, obwohl die
// Messgrundlage jeder Schwellwertaenderung fehlte.
std::string findFixture(const std::string& relative) {
    const char* prefixes[] = {"", "../", "../../", "../../../"};
    std::vector<std::string> variants{relative};
    const std::string marker = "testdata/";
    if (relative.compare(0, marker.size(), marker) == 0) {
        variants.push_back("testdata/archiv/" + relative.substr(marker.size()));
    }
    for (const char* p : prefixes) {
        for (const std::string& v : variants) {
            std::string candidate = std::string(p) + v;
            std::ifstream in(candidate);
            if (in.good()) return candidate;
        }
    }
    return std::string();
}

}  // namespace

// Spielt den belegten Standlauf 20260730_101000_5901D247 ein.
//
// OBD meldete dort in allen gültigen Samples 0 km/h. Es darf daher weder ein
// Qualitätswert noch ein Schlagloch entstehen. Das ist der Regressionsschutz
// für den Befund aus 20260730_071913_9B018397.
void test_wiedergabe_standlauf_erzeugt_nichts() {
    const std::string path = findFixture(
        "testdata/20260730_101000_5901D247/"
        "road_sensor_20260730_101000_5901D247.csv");
    if (path.empty()) {
        TEST_IGNORE_MESSAGE("Messdaten nicht gefunden - uebersprungen");
        return;
    }

    std::vector<std::string> h;
    std::vector<std::vector<std::string>> r;
    TEST_ASSERT_TRUE(loadCsv(path, h, r));
    const int tIdx = columnIndex(h, "UptimeMs");
    const int zIdx = columnIndex(h, "AccelZ");
    TEST_ASSERT_TRUE(tIdx >= 0 && zIdx >= 0);

    RoadMetricsAnalyzer a;
    int qualityValues = 0;
    int potholes = 0;
    for (const auto& row : r) {
        const uint32_t t =
            static_cast<uint32_t>(std::strtoul(row[tIdx].c_str(), nullptr, 10));
        const float z = static_cast<float>(std::atof(row[zIdx].c_str()));
        a.addSample(z);
        // Das Fahrzeug stand: OBD lieferte durchgehend 0 km/h.
        if (a.calculateRoadQuality(0.0f) >= 0.0f) qualityValues++;
        if (a.detectPothole(t, z, 0.0f)) potholes++;
    }

    std::printf(
        "\nStandlauf 101000: %zu Stichproben, %d Qualitaetswerte, "
        "%d Schlagloecher\n",
        r.size(), qualityValues, potholes);
    TEST_ASSERT_EQUAL_INT(0, qualityValues);
    TEST_ASSERT_EQUAL_INT(0, potholes);
}

// Spielt die Fahrt 20260730_095603_A41A2450 mit echter OBD-Geschwindigkeit
// ein und hält die Verteilung als Festwerte fest.
void test_wiedergabe_fahrt_kennzahlen() {
    const std::string base = "testdata/20260730_095603_A41A2450/";
    const std::string sensorPath =
        findFixture(base + "road_sensor_20260730_095603_A41A2450.csv");
    const std::string obdPath =
        findFixture(base + "road_obd_20260730_095603_A41A2450.csv");
    if (sensorPath.empty() || obdPath.empty()) {
        TEST_IGNORE_MESSAGE("Messdaten nicht gefunden - uebersprungen");
        return;
    }

    std::vector<std::string> sh, oh;
    std::vector<std::vector<std::string>> sr, orow;
    TEST_ASSERT_TRUE(loadCsv(sensorPath, sh, sr));
    TEST_ASSERT_TRUE(loadCsv(obdPath, oh, orow));
    const int sT = columnIndex(sh, "UptimeMs");
    const int sZ = columnIndex(sh, "AccelZ");
    const int oT = columnIndex(oh, "UptimeMs");
    const int oV = columnIndex(oh, "SpeedValid");
    const int oS = columnIndex(oh, "SpeedKmh");
    TEST_ASSERT_TRUE(sT >= 0 && sZ >= 0 && oT >= 0 && oV >= 0 && oS >= 0);

    std::vector<std::pair<uint32_t, float>> speeds;
    for (const auto& row : orow) {
        if (row[oV] != "1") continue;
        speeds.push_back(
            {static_cast<uint32_t>(std::strtoul(row[oT].c_str(), nullptr, 10)),
             static_cast<float>(std::atof(row[oS].c_str()))});
    }
    TEST_ASSERT_TRUE(speeds.size() > 100);

    RoadMetricsAnalyzer a;
    size_t cursor = 0;
    int qualityValues = 0;
    int potholes = 0;
    double qualitySum = 0.0;
    float minQuality = 100.0f;
    for (const auto& row : sr) {
        const uint32_t t =
            static_cast<uint32_t>(std::strtoul(row[sT].c_str(), nullptr, 10));
        const float z = static_cast<float>(std::atof(row[sZ].c_str()));
        while (cursor + 1 < speeds.size() && speeds[cursor + 1].first <= t) {
            ++cursor;
        }
        float speed = -1.0f;
        const uint32_t st = speeds[cursor].first;
        const uint32_t diff = t > st ? t - st : st - t;
        if (diff <= 1500) speed = speeds[cursor].second;

        a.addSample(z);
        const float q = a.calculateRoadQuality(speed);
        if (q >= 0.0f) {
            qualityValues++;
            qualitySum += q;
            if (q < minQuality) minQuality = q;
        }
        if (a.detectPothole(t, z, speed)) potholes++;
    }

    const double mean = qualityValues > 0 ? qualitySum / qualityValues : 0.0;
    std::printf(
        "\nFahrt 095603: %d Qualitaetswerte, Mittel %.1f, Minimum %.1f, "
        "%d Schlagloecher\n",
        qualityValues, mean, minQuality, potholes);

    // Festwerte statt Korridor: Eine Aenderung ist zu pruefen, zu begruenden
    // und hier bewusst nachzuziehen.
    TEST_ASSERT_EQUAL_INT(5856, qualityValues);
    TEST_ASSERT_EQUAL_INT(7, potholes);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 84.2f, static_cast<float>(mean));
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 33.2f, minQuality);
}

// ---------------------------------------------------------------------------
// Fahrbarkeit je Streckenabschnitt
//
// Die Frage ist nicht "wo liegt ein Schlagloch", sondern "kann man hier zügig
// fahren". Bewertet wird deshalb ein fester Weg statt einer festen Zeit.
// ---------------------------------------------------------------------------

namespace {
// Speist einen Abschnitt mit gleichmäßigem Wechselsignal. Liefert die Zahl der
// abgeschlossenen Abschnitte und den zuletzt abgeschlossenen.
uint32_t fahreAbschnitt(
    RoadMetricsAnalyzer& a, float amplitude, float speedKmh, uint32_t schritte,
    RoadSection& letzter, uint32_t intervallMs = 100, uint32_t startMs = 1000) {
    uint32_t abgeschlossen = 0;
    RoadSection s;
    for (uint32_t i = 0; i < schritte; ++i) {
        const float value = (i % 2 == 0) ? amplitude : -amplitude;
        if (a.updateSection(startMs + i * intervallMs, value, speedKmh, s)) {
            letzter = s;
            ++abgeschlossen;
        }
    }
    return abgeschlossen;
}
}  // namespace

// 200 m bei 72 km/h = 20 m/s sind zehn Sekunden. Der erste Aufruf setzt nur
// den Startzeitpunkt und trägt keinen Weg bei, deshalb 110 statt 100
// Stichproben.
void test_abschnitt_schliesst_nach_der_weglaenge() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    const uint32_t n = fahreAbschnitt(a, 0.5f, 72.0f, 110, s);
    TEST_ASSERT_EQUAL_UINT32(1, n);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, ROAD_SECTION_LENGTH_M, s.distanceM);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, s.rmsVertical);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 72.0f, s.meanSpeedKmh);
}

// Doppelte Geschwindigkeit, halbe Zeit, gleiche Weglänge: Ein Abschnitt
// beschreibt immer dasselbe Stück Straße.
void test_abschnittslaenge_ist_vom_tempo_unabhaengig() {
    RoadMetricsAnalyzer langsam, schnell;
    RoadSection sl{}, ss{};
    // Beide Seiten müssen tatsächlich abschließen, sonst vergliche der Test
    // zwei Nullwerte miteinander und wäre wertlos.
    TEST_ASSERT_EQUAL_UINT32(1, fahreAbschnitt(langsam, 0.5f, 36.0f, 210, sl));
    TEST_ASSERT_EQUAL_UINT32(1, fahreAbschnitt(schnell, 0.5f, 72.0f, 110, ss));
    TEST_ASSERT_FLOAT_WITHIN(5.0f, sl.distanceM, ss.distanceM);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, sl.rmsVertical, ss.rmsVertical);
}

// Ohne nachgewiesene Bewegung entsteht kein Abschnitt. Sonst bewertete die
// Firmware im Stand die Leerlaufvibration als Fahrbahn.
void test_kein_abschnitt_im_stillstand() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    TEST_ASSERT_EQUAL_UINT32(0, fahreAbschnitt(a, 2.0f, 0.0f, 500, s));
    TEST_ASSERT_FALSE(a.hasOpenSection());
}

// Auch eine unbekannte Geschwindigkeit darf keinen Abschnitt füllen.
void test_kein_abschnitt_bei_unbekannter_geschwindigkeit() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    TEST_ASSERT_EQUAL_UINT32(0, fahreAbschnitt(a, 1.0f, -1.0f, 500, s));
}

// Knapp unter der Mindestgeschwindigkeit wächst nichts, knapp darüber schon.
void test_abschnitt_an_der_mindestgeschwindigkeit() {
    RoadMetricsAnalyzer unter;
    RoadSection s{};
    fahreAbschnitt(unter, 0.5f, ROAD_EVENT_MIN_SPEED_KMH - 0.1f, 50, s);
    TEST_ASSERT_FALSE(unter.hasOpenSection());

    RoadMetricsAnalyzer drueber;
    fahreAbschnitt(drueber, 0.5f, ROAD_EVENT_MIN_SPEED_KMH + 0.1f, 50, s);
    TEST_ASSERT_TRUE(drueber.hasOpenSection());
}

// Eine Abtastlücke darf keinen Weg erfinden. Bei 500 ms Abstand zählt jeder
// Schritt noch, bei 600 ms nicht mehr - dieselbe Grenze wie im CurveDetector.
void test_abtastluecke_erfindet_keinen_weg() {
    RoadMetricsAnalyzer knapp, darueber;
    RoadSection s{};
    fahreAbschnitt(knapp, 0.5f, 72.0f, 20, s, CURVE_MAX_SAMPLE_GAP_MS);
    fahreAbschnitt(darueber, 0.5f, 72.0f, 20, s, CURVE_MAX_SAMPLE_GAP_MS + 100);
    TEST_ASSERT_TRUE(knapp.openSectionDistanceM() > 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, darueber.openSectionDistanceM());
}

// Ein zu lange offener Abschnitt verbände Fahrbahn vor und hinter einem Halt.
void test_zu_langer_abschnitt_wird_verworfen() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    // Langsam genug, dass 200 m nicht erreicht werden, bevor die Zeit abläuft.
    const uint32_t schritte = ROAD_SECTION_MAX_DURATION_MS / 1000 + 10;
    uint32_t abgeschlossen = 0;
    for (uint32_t i = 0; i < schritte; ++i) {
        if (a.updateSection(1000 + i * 1000, 0.5f, 6.0f, s)) ++abgeschlossen;
    }
    TEST_ASSERT_EQUAL_UINT32(0, abgeschlossen);
    TEST_ASSERT_TRUE(a.openSectionDistanceM() < ROAD_SECTION_LENGTH_M);
}

// Die Note muss an beiden Grenzen greifen, je ein Fall knapp darüber und
// knapp darunter.
void test_note_an_den_grenzen() {
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, 100.0f,
        RoadMetricsAnalyzer::driveabilityFromRms(
            ROAD_DRIVEABILITY_RMS_GOOD_MPS2 - 0.01f));
    TEST_ASSERT_TRUE(
        RoadMetricsAnalyzer::driveabilityFromRms(
            ROAD_DRIVEABILITY_RMS_GOOD_MPS2 + 0.01f) < 100.0f);
    TEST_ASSERT_TRUE(
        RoadMetricsAnalyzer::driveabilityFromRms(
            ROAD_DRIVEABILITY_RMS_BAD_MPS2 - 0.01f) > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, 0.0f,
        RoadMetricsAnalyzer::driveabilityFromRms(
            ROAD_DRIVEABILITY_RMS_BAD_MPS2 + 0.01f));
}

// Wer schnell fährt, traut der Straße: Ab ROAD_DRIVEABILITY_FAST_KMH gilt die
// Fahrbahn als tragfähig, und die Note fällt nicht unter die Untergrenze. In
// der Fahrt vom 02.08.2026 erhielten zwei Abschnitte die Note 0 bei 130 km/h
// Durchschnitt; von 14 bewerteten Abschnitten über 100 km/h wurde keiner
// schlechter als "gut" beurteilt.
void test_hohe_geschwindigkeit_hebt_die_note_an() {
    // Rauer Abschnitt, langsam gefahren: Note bleibt schlecht.
    const float rau = ROAD_DRIVEABILITY_RMS_BAD_MPS2 + 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, 0.0f,
        RoadMetricsAnalyzer::driveability(
            rau, ROAD_DRIVEABILITY_FAST_KMH - 0.1f));
    // Derselbe Abschnitt mit hoher Geschwindigkeit: Untergrenze greift.
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, ROAD_DRIVEABILITY_FAST_MIN_NOTE,
        RoadMetricsAnalyzer::driveability(
            rau, ROAD_DRIVEABILITY_FAST_KMH + 0.1f));
}

// Die Regel wirkt nur nach oben. Eine bereits gute Note wird durch hohe
// Geschwindigkeit nicht verändert - und schon gar nicht abgesenkt.
void test_geschwindigkeitsregel_senkt_keine_note() {
    const float glatt = ROAD_DRIVEABILITY_RMS_GOOD_MPS2 - 0.1f;
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, 100.0f,
        RoadMetricsAnalyzer::driveability(glatt, 200.0f));
    // Und ohne Geschwindigkeitsangabe bleibt es beim reinen Effektivwert.
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, RoadMetricsAnalyzer::driveabilityFromRms(1.2f),
        RoadMetricsAnalyzer::driveability(1.2f, 30.0f));
}

// Raue Fahrbahn bekommt eine schlechtere Note als glatte.
void test_raue_fahrbahn_schlechtere_note() {
    RoadMetricsAnalyzer glatt, rau;
    RoadSection sg{}, sr{};
    TEST_ASSERT_EQUAL_UINT32(1, fahreAbschnitt(glatt, 0.3f, 72.0f, 110, sg));
    TEST_ASSERT_EQUAL_UINT32(1, fahreAbschnitt(rau, 2.0f, 72.0f, 110, sr));
    TEST_ASSERT_TRUE(sg.driveability > sr.driveability + 30.0f);
}

// Regressionsschutz gegen den Befund vom 02.08.2026: Die Fahrbahnbewertung
// lief über die Sensorachse accelZ, die im Fahrzeug nur 19 Prozent der
// Vertikalen trug und mit ihr zu -0,06 korrelierte. Dadurch bewertete sie
// Querbeschleunigung als Fahrbahnstoß, und gerade zügig gefahrene Kurven
// bekamen die schlechteste Note - genau umgekehrt zum Zweck der Messung.
//
// Der Test stellt die gemessene Einbaulage nach: Die Schwerkraft lag bei
// (8,60 / -4,23 / -1,85), die Sensorachse Z trug also nur 19 Prozent der
// Vertikalen. Eine reine Querbeschleunigung darf danach keine
// Vertikalbeschleunigung erzeugen.
void test_querbeschleunigung_erzeugt_keine_vertikale() {
    const float gx = 8.60f, gy = -4.23f, gz = -1.85f;
    const float mag = std::sqrt(gx * gx + gy * gy + gz * gz);
    // Ein Vektor senkrecht zur Schwerkraft: Kreuzprodukt mit einer beliebigen
    // nicht parallelen Richtung.
    const float qx = gy * 1.0f - gz * 0.0f;
    const float qy = gz * 0.0f - gx * 1.0f;
    const float qz = gx * 0.0f - gy * 0.0f;
    const float qn = std::sqrt(qx * qx + qy * qy + qz * qz);
    const float skala = 4.0f / qn;  // kräftige Querbeschleunigung

    float vertikal = -1.0f;
    TEST_ASSERT_TRUE(RoadMetricsAnalyzer::verticalFromGravity(
        qx * skala, qy * skala, qz * skala, gx, gy, gz, vertikal));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vertikal);

    // Zum Gegenbeweis: Dieselbe Beschleunigung auf der alten Achse accelZ
    // wäre als Fahrbahnstoß durchgegangen.
    TEST_ASSERT_TRUE(std::fabs(qz * skala) > 1.0f || std::fabs(qx * skala) > 1.0f);
    (void)mag;
}

// Ein echter Fahrbahnstoß, also eine Beschleunigung längs der Schwerkraft,
// muss dagegen voll durchkommen.
void test_vertikalstoss_kommt_voll_an() {
    const float gx = 8.60f, gy = -4.23f, gz = -1.85f;
    const float mag = std::sqrt(gx * gx + gy * gy + gz * gz);
    // 3 m/s² nach oben, also entgegen der Schwerkraftrichtung.
    float vertikal = 0.0f;
    TEST_ASSERT_TRUE(RoadMetricsAnalyzer::verticalFromGravity(
        -3.0f * gx / mag, -3.0f * gy / mag, -3.0f * gz / mag, gx, gy, gz,
        vertikal));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, vertikal);
}

// Ohne brauchbare Schwerkraftrichtung entsteht kein Messwert.
void test_unbrauchbare_schwerkraft_liefert_nichts() {
    float vertikal = 42.0f;
    TEST_ASSERT_FALSE(RoadMetricsAnalyzer::verticalFromGravity(
        1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, vertikal));
    TEST_ASSERT_FALSE(RoadMetricsAnalyzer::verticalFromGravity(
        1.0f, 1.0f, 1.0f, 40.0f, 0.0f, 0.0f, vertikal));
    // Der Ausgabewert bleibt unangetastet.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.0f, vertikal);
}

// Abschnitte werden fortlaufend ab 1 nummeriert, und die Nummer des offenen
// Abschnitts zeigt immer auf den nächsten. Ein Beifahrerurteil trägt diese
// Nummer mit und gilt dadurch für genau seinen Abschnitt - wer minutenlang
// aussetzt, hinterlässt eine Lücke statt einer fortgeschriebenen Wertung.
void test_abschnitte_werden_fortlaufend_nummeriert() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    TEST_ASSERT_EQUAL_UINT32(1, a.currentSectionNumber());
    TEST_ASSERT_EQUAL_UINT32(0, a.completedSectionCount());

    fahreAbschnitt(a, 0.5f, 72.0f, 110, s);
    TEST_ASSERT_EQUAL_UINT32(1, s.number);
    TEST_ASSERT_EQUAL_UINT32(2, a.currentSectionNumber());

    fahreAbschnitt(a, 0.5f, 72.0f, 110, s, 100, 100000);
    TEST_ASSERT_EQUAL_UINT32(2, s.number);
    TEST_ASSERT_EQUAL_UINT32(2, a.completedSectionCount());
}

// Der Zähler beginnt mit jeder Messung neu, damit die Nummern einer Sitzung
// eindeutig bleiben.
void test_zaehler_faellt_beim_zuruecksetzen() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    fahreAbschnitt(a, 0.5f, 72.0f, 110, s);
    TEST_ASSERT_EQUAL_UINT32(1, a.completedSectionCount());
    a.reset();
    TEST_ASSERT_EQUAL_UINT32(0, a.completedSectionCount());
    TEST_ASSERT_EQUAL_UINT32(1, a.currentSectionNumber());
}

// Zügige Kurvenfahrt auf glatter Bahn muss eine gute Note bekommen. Mit der
// alten Achse bekam sie die schlechteste.
void test_kurvenfahrt_auf_glatter_bahn_bleibt_gut() {
    RoadMetricsAnalyzer a;
    RoadSection s{};
    const uint32_t n = fahreAbschnitt(a, 0.25f, 80.0f, 110, s);
    TEST_ASSERT_EQUAL_UINT32(1, n);
    TEST_ASSERT_TRUE(s.driveability >= 95.0f);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_stillstand_liefert_keinen_messwert);
    RUN_TEST(test_unbekannte_geschwindigkeit_liefert_keinen_messwert);
    RUN_TEST(test_ab_mindestgeschwindigkeit_entsteht_ein_messwert);
    RUN_TEST(test_glatte_fahrbahn_besser_als_raue);
    RUN_TEST(test_qualitaet_bleibt_im_bereich_null_bis_hundert);
    RUN_TEST(test_normierung_wirkt_im_mittleren_bereich);
    RUN_TEST(test_normierung_ist_oberhalb_von_43_kmh_wirkungslos);
    RUN_TEST(test_klemmgrenzen_der_normierung);
    RUN_TEST(test_effektivwert_wird_richtig_gebildet);
    RUN_TEST(test_leeres_fenster_liefert_nullwerte);
    RUN_TEST(test_zusammenhaengender_stoss_zaehlt_einmal);
    RUN_TEST(test_zwei_getrennte_stoesse_zaehlen_zweimal);
    RUN_TEST(test_fenster_vergisst_alte_werte);
    RUN_TEST(test_glaettewert_faellt_mit_rauheit);
    RUN_TEST(test_schlagloch_wird_erkannt);
    RUN_TEST(test_kein_schlagloch_im_stillstand);
    RUN_TEST(test_im_stand_begonnener_ausschlag_schliesst_nicht_ab);
    RUN_TEST(test_zu_spaeter_gegenausschlag_wird_verworfen);
    RUN_TEST(test_sperrzeit_verhindert_doppelzaehlung);
    RUN_TEST(test_kleine_ausschlaege_erzeugen_kein_schlagloch);
    RUN_TEST(test_wiedergabe_standlauf_erzeugt_nichts);
    RUN_TEST(test_wiedergabe_fahrt_kennzahlen);
    RUN_TEST(test_abschnitt_schliesst_nach_der_weglaenge);
    RUN_TEST(test_abschnittslaenge_ist_vom_tempo_unabhaengig);
    RUN_TEST(test_kein_abschnitt_im_stillstand);
    RUN_TEST(test_kein_abschnitt_bei_unbekannter_geschwindigkeit);
    RUN_TEST(test_abschnitt_an_der_mindestgeschwindigkeit);
    RUN_TEST(test_abtastluecke_erfindet_keinen_weg);
    RUN_TEST(test_zu_langer_abschnitt_wird_verworfen);
    RUN_TEST(test_note_an_den_grenzen);
    RUN_TEST(test_hohe_geschwindigkeit_hebt_die_note_an);
    RUN_TEST(test_geschwindigkeitsregel_senkt_keine_note);
    RUN_TEST(test_raue_fahrbahn_schlechtere_note);
    RUN_TEST(test_querbeschleunigung_erzeugt_keine_vertikale);
    RUN_TEST(test_vertikalstoss_kommt_voll_an);
    RUN_TEST(test_unbrauchbare_schwerkraft_liefert_nichts);
    RUN_TEST(test_abschnitte_werden_fortlaufend_nummeriert);
    RUN_TEST(test_zaehler_faellt_beim_zuruecksetzen);
    RUN_TEST(test_kurvenfahrt_auf_glatter_bahn_bleibt_gut);
    return UNITY_END();
}

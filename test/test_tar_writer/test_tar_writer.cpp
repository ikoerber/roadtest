// Prüft die tar-Rahmung des Datenzugriffs.
//
// Ein falscher Kopf macht jedes heruntergeladene Archiv unbrauchbar, und das
// fiele erst am Handy im Auto auf - an einem Gerät, an das man nach dem Einbau
// nicht mehr herankommt. Der Test erzeugt deshalb ein vollständiges Archiv und
// lässt es vom echten `tar` des Entwicklungsrechners lesen. Die Feldprüfungen
// darunter sagen im Fehlerfall, welches Feld es war.

#include <unity.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "tar_writer.h"

namespace {

// Baut ein Archiv aus benannten Inhalten, wie es handleSessionDownload() tut:
// je Datei ein Kopf, die Nutzdaten, Füllbytes bis zur Blockgrenze, am Ende
// zwei Nullblöcke.
std::vector<uint8_t> baueArchiv(
    const std::vector<std::pair<std::string, std::string>>& dateien) {
    std::vector<uint8_t> archiv;
    uint8_t block[TAR_BLOCK_SIZE];

    for (const auto& datei : dateien) {
        tarWriteHeader(block, datei.first.c_str(),
                       static_cast<uint32_t>(datei.second.size()));
        archiv.insert(archiv.end(), block, block + TAR_BLOCK_SIZE);
        archiv.insert(archiv.end(), datei.second.begin(), datei.second.end());

        const size_t fuell = tarPaddingBytes(
            static_cast<uint32_t>(datei.second.size()));
        archiv.insert(archiv.end(), fuell, 0);
    }

    archiv.insert(archiv.end(), 2 * TAR_BLOCK_SIZE, 0);
    return archiv;
}

std::string feld(const uint8_t* block, size_t offset, size_t laenge) {
    std::string wert;
    for (size_t i = 0; i < laenge && block[offset + i] != '\0'; ++i) {
        wert += static_cast<char>(block[offset + i]);
    }
    return wert;
}

}  // namespace

void test_kopf_traegt_namen_und_groesse() {
    uint8_t block[TAR_BLOCK_SIZE];
    tarWriteHeader(block, "20260804_080744_8C5DBE75/road_gps.csv", 519415);

    TEST_ASSERT_EQUAL_STRING("20260804_080744_8C5DBE75/road_gps.csv",
                             feld(block, 0, 100).c_str());
    // 519415 dezimal sind 1766367 oktal, auf elf Stellen aufgefüllt.
    TEST_ASSERT_EQUAL_STRING("00001766367", feld(block, 124, 12).c_str());
    TEST_ASSERT_EQUAL_CHAR('0', static_cast<char>(block[156]));
    TEST_ASSERT_EQUAL_STRING("ustar", feld(block, 257, 6).c_str());
}

// Die Prüfsumme ist der Grund für diesen Test: Sie zählt ihr eigenes Feld als
// acht Leerzeichen. Wer sie über den fertigen Kopf bildet, bekommt eine
// Summe, die kein tar akzeptiert.
void test_pruefsumme_rechnet_ihr_eigenes_feld_als_leerzeichen() {
    uint8_t block[TAR_BLOCK_SIZE];
    tarWriteHeader(block, "a.csv", 1);

    uint32_t erwartet = 0;
    for (size_t i = 0; i < TAR_BLOCK_SIZE; ++i) {
        erwartet += (i >= 148 && i < 156) ? ' ' : block[i];
    }

    const uint32_t gelesen =
        static_cast<uint32_t>(strtoul(feld(block, 148, 7).c_str(), nullptr, 8));
    TEST_ASSERT_EQUAL_UINT32(erwartet, gelesen);
    TEST_ASSERT_EQUAL_CHAR('\0', static_cast<char>(block[154]));
    TEST_ASSERT_EQUAL_CHAR(' ', static_cast<char>(block[155]));
}

void test_fuellbytes_runden_auf_blockgrenze() {
    TEST_ASSERT_EQUAL_UINT32(0, tarPaddingBytes(0));
    TEST_ASSERT_EQUAL_UINT32(511, tarPaddingBytes(1));
    TEST_ASSERT_EQUAL_UINT32(0, tarPaddingBytes(512));
    TEST_ASSERT_EQUAL_UINT32(511, tarPaddingBytes(513));
    TEST_ASSERT_EQUAL_UINT32(0, tarPaddingBytes(1024));
}

// Der eigentliche Nachweis: Das echte tar muss das Archiv lesen und die
// Inhalte unverändert zurückgeben.
void test_echtes_tar_liest_das_archiv() {
    const std::vector<std::pair<std::string, std::string>> dateien = {
        {"20260804_080744_8C5DBE75/road_meta.csv",
         "Record,UTC\nSTART,2026-08-04T08:07:45Z\n"},
        {"20260804_080744_8C5DBE75/road_gps.csv", std::string(1500, 'x')},
        {"20260804_080744_8C5DBE75/road_summary.csv", "Session,Strecke\n"}};

    const std::vector<uint8_t> archiv = baueArchiv(dateien);

    const char* pfad = "/tmp/roadtest_test.tar";
    FILE* f = fopen(pfad, "wb");
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL_UINT32(
        archiv.size(), fwrite(archiv.data(), 1, archiv.size(), f));
    fclose(f);

    // tar meldet einen Fehlercode, wenn Prüfsumme, Feldformat oder
    // Blockstruktur nicht stimmen.
    TEST_ASSERT_EQUAL_INT(
        0, system("tar tf /tmp/roadtest_test.tar > /tmp/roadtest_test.liste"));

    FILE* liste = fopen("/tmp/roadtest_test.liste", "rb");
    TEST_ASSERT_NOT_NULL(liste);
    char inhalt[512] = {0};
    fread(inhalt, 1, sizeof(inhalt) - 1, liste);
    fclose(liste);
    TEST_ASSERT_NOT_NULL(strstr(inhalt, "road_meta.csv"));
    TEST_ASSERT_NOT_NULL(strstr(inhalt, "road_gps.csv"));
    TEST_ASSERT_NOT_NULL(strstr(inhalt, "road_summary.csv"));

    // Auspacken und den Inhalt der größten Datei gegen das Original stellen.
    TEST_ASSERT_EQUAL_INT(
        0, system("rm -rf /tmp/roadtest_test_aus && mkdir /tmp/roadtest_test_aus"
                  " && tar xf /tmp/roadtest_test.tar -C /tmp/roadtest_test_aus"));
    FILE* ausgepackt = fopen(
        "/tmp/roadtest_test_aus/20260804_080744_8C5DBE75/road_gps.csv", "rb");
    TEST_ASSERT_NOT_NULL(ausgepackt);
    fseek(ausgepackt, 0, SEEK_END);
    const long groesse = ftell(ausgepackt);
    fclose(ausgepackt);
    TEST_ASSERT_EQUAL_INT32(1500, groesse);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_kopf_traegt_namen_und_groesse);
    RUN_TEST(test_pruefsumme_rechnet_ihr_eigenes_feld_als_leerzeichen);
    RUN_TEST(test_fuellbytes_runden_auf_blockgrenze);
    RUN_TEST(test_echtes_tar_liest_das_archiv);
    return UNITY_END();
}

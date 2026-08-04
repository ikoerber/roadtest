#ifndef GZIP_STREAM_H
#define GZIP_STREAM_H

// Schreibt einen gzip-Datenstrom in eine geöffnete Datei.
//
// Zweck: Die Logdateien belegen roh rund 186 kB je Fahrtminute. Eine
// 52-Kilometer-Fahrt ergab 10,9 MB. Gemessen an denselben Daten komprimiert
// gzip um den Faktor 5,4; das senkt die Schreiblast auf der SD-Karte ebenso
// wie die Zeit, eine Sitzung später wieder auszulesen.
//
// Warum gzip und kein eigenes Binärformat: Eine `.csv.gz` bleibt eine
// gewöhnliche CSV in einer Verpackung, die jedes Werkzeug öffnet - `zcat`,
// Pandas, Excel nach dem Auspacken. Es entsteht keine Formatbeschreibung,
// die verlorengehen kann, und kein Konverter, der zur einzigen Brücke zu den
// Messdaten wird. Ein handgeschriebenes Binärformat erreichte in der Messung
// nur Faktor 4,6 und hätte die Lesbarkeit gekostet.
//
// Der Kompressor stammt aus dem ROM des ESP32-S3 (miniz, `tdefl_*`). Er
// belegt dadurch **kein Byte Flash**. Der Arbeitsspeicher von rund 133 kB je
// Datenstrom liegt im PSRAM, damit der knappe Heap unangetastet bleibt.
//
// Diese Einheit kapselt ausschließlich die Kompression und die
// gzip-Rahmung. Sie kennt weder Sitzungen noch Logtypen und ist deshalb für
// sich prüfbar.

#include <Arduino.h>
#include <FS.h>

// Senke des komprimierten Stroms.
//
// Ursprünglich schrieb die Einheit ausschließlich in eine geöffnete Datei.
// Seit der Datenzugriff übers Handy dieselbe Kompression braucht, ist das Ziel
// austauschbar: dieselbe Rahmung, dieselbe Prüfsumme, einmal in eine Datei auf
// der Karte und einmal in eine HTTP-Antwort. Die Alternative wäre ein zweiter
// Kompressionspfad gewesen - und damit zwei Stellen, an denen ein
// gzip-Nachspann fehlen kann.
class GzipSink {
public:
    virtual ~GzipSink() = default;
    // Rückgabe ist die Zahl tatsächlich abgenommener Bytes. Alles andere als
    // `length` gilt als Fehler und macht den Strom unbrauchbar.
    virtual size_t write(const uint8_t* data, size_t length) = 0;
};

// Senke, die in eine geöffnete Datei schreibt.
class GzipFileSink : public GzipSink {
public:
    GzipFileSink() = default;
    void setFile(fs::File& target) { file = &target; }
    size_t write(const uint8_t* data, size_t length) override {
        return file != nullptr ? file->write(data, length) : 0;
    }

private:
    fs::File* file = nullptr;
};

class GzipStream {
public:
    GzipStream() = default;
    ~GzipStream() { end(); }

    GzipStream(const GzipStream&) = delete;
    GzipStream& operator=(const GzipStream&) = delete;

    // Kompressor anlegen und den gzip-Kopf schreiben.
    //
    // Die Datei muss geöffnet sein und bleibt im Besitz des Aufrufers. Der
    // Strom merkt sich ihre Adresse: Die übergebene File-Instanz muss bis
    // end() am selben Ort weiterleben. Ein dauerhaftes Member wie im
    // SDLogger erfüllt das; eine lokale Variable oder Kopie nicht. Ohne
    // verfügbares PSRAM scheitert der Aufruf; der Aufrufer schreibt dann
    // unkomprimiert weiter, statt die Messung zu verlieren.
    bool begin(fs::File& file);

    // Wie begin(fs::File&), aber mit beliebiger Senke. Die Senke muss bis
    // end() weiterleben.
    bool begin(GzipSink& sink);

    // Nutzdaten übernehmen. Rückgabe ist die Zahl der Bytes, die dabei
    // tatsächlich in die Datei geflossen sind - das können null sein, wenn
    // der Kompressor die Daten noch im Fenster hält.
    //
    // Bei einem Schreibfehler wird der Strom als gescheitert markiert und
    // liefert dauerhaft false aus `healthy()`.
    size_t write(const uint8_t* data, size_t length);

    // Ausstehende Daten in die Datei drücken, ohne den Strom zu beenden.
    //
    // Entspricht Z_SYNC_FLUSH. Der Kompressionsfaktor sinkt dadurch
    // messbar kaum: Bei Synchronisationspunkten alle 4 kB wurden 4,4 statt
    // 4,5 erreicht. Nach dem Aufruf ist alles bis hierher in der Datei, was
    // die Sitzung nach einem Spannungsverlust auswertbar hält.
    size_t flush();

    // Strom abschließen: letzte Daten, Prüfsumme und Länge.
    //
    // Ohne diesen Aufruf fehlt der gzip-Nachspann, und Werkzeuge melden beim
    // Auspacken einen unvollständigen Strom - die Nutzdaten bleiben zwar
    // lesbar, aber die Prüfung schlägt fehl.
    size_t finish();

    // Kompressor freigeben. Die Datei wird nicht geschlossen.
    void end();

    bool active() const { return compressor != nullptr; }
    // Ein Fehlerzustand bleibt bewusst über end() hinaus lesbar, damit der
    // Aufrufer nach dem Abschluss noch entscheiden kann, ob die Datei
    // vollständig ist. Erst das nächste begin() setzt ihn zurück.
    bool healthy() const { return !failed; }

    // Byte, die in die Datei geschrieben wurden. Grundlage für die
    // Größenprüfung am Sitzungsende.
    uint32_t compressedBytes() const { return writtenBytes; }
    uint32_t rawBytes() const { return sourceBytes; }

private:
    // Rückruf für den ROM-Kompressor. Schreibt in die Datei und zählt mit.
    static int putBuffer(const void* buffer, int length, void* user);

    void* compressor = nullptr;   // tdefl_compressor im PSRAM
    GzipSink* target = nullptr;
    // Trägt die Senke für begin(fs::File&). Sie muss als Member leben, weil
    // target auf sie zeigt.
    GzipFileSink fileSink;
    uint32_t crc = 0;             // CRC32 über die Nutzdaten, gzip-Nachspann
    uint32_t sourceBytes = 0;
    uint32_t writtenBytes = 0;
    bool failed = false;
    bool finished = false;
};

#endif  // GZIP_STREAM_H

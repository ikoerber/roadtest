#include "gzip_stream.h"

#include <esp_heap_caps.h>
#include <string.h>

// miniz aus dem ROM des ESP32-S3. Die Symbole stellt esp32s3.rom.ld bereit;
// der Code belegt deshalb keinen Flash. Der Header liegt im IDF unter
// esp_rom/include/miniz.h.
extern "C" {
#include <miniz.h>
}

namespace {

// gzip-Kopf nach RFC 1952: Kennung, Deflate, keine Zusatzfelder, kein
// Zeitstempel (die Sitzungszeit steht in den Daten selbst und im Dateinamen),
// unbekanntes Betriebssystem.
const uint8_t GZIP_HEADER[10] = {
    0x1F, 0x8B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};

// Die unteren zwoelf Bit des Flags-Parameters von tdefl_init() geben die Zahl
// der Hash-Sondierungen je Woerterbuchzugriff an. TDEFL_DEFAULT_MAX_PROBES
// betraegt 128 und entspricht der Einstellung, mit der an echten Logdateien
// Faktor 5,4 gemessen wurde.
//
// Sollte sich die Rechenzeit in der Hauptschleife als zu hoch erweisen -
// sie hat mit einem Flush-Schritt von 119 ms bei 100 ms Slotbreite keine
// Reserve -, ist dieser Wert der erste Stellhebel. Weniger Sondierungen
// bedeuten schneller und etwas schlechter.
const int DEFLATE_FLAGS = TDEFL_DEFAULT_MAX_PROBES;

}  // namespace

int GzipStream::putBuffer(const void* buffer, int length, void* user) {
    GzipStream* self = static_cast<GzipStream*>(user);
    if (self == nullptr || self->target == nullptr || self->failed) {
        return 0;
    }
    if (length <= 0) {
        return 1;
    }
    const size_t written = self->target->write(
        static_cast<const uint8_t*>(buffer), static_cast<size_t>(length));
    self->writtenBytes += written;
    if (written != static_cast<size_t>(length)) {
        // Ein Teilschreibvorgang macht den Strom unbrauchbar: Im Deflate-Strom
        // fehlten Bytes, und alles Folgende wäre nicht mehr auspackbar.
        self->failed = true;
        return 0;
    }
    return 1;
}

bool GzipStream::begin(fs::File& file) {
    end();

    if (!file) {
        return false;
    }

    // Der Kompressor gehört in den PSRAM. Er belegt rund 133 kB; im Heap
    // wären das bei mehreren Datenströmen mehr, als überhaupt frei ist.
    compressor = heap_caps_malloc(sizeof(tdefl_compressor), MALLOC_CAP_SPIRAM);
    if (compressor == nullptr) {
        // Kein PSRAM: Der Aufrufer schreibt unkomprimiert weiter. Eine
        // Messung ohne Kompression ist einer verlorenen Messung vorzuziehen.
        return false;
    }

    target = &file;
    crc = MZ_CRC32_INIT;
    sourceBytes = 0;
    writtenBytes = 0;
    failed = false;
    finished = false;

    // Rohes Deflate ohne zlib-Kopf; die Rahmung übernimmt gzip.
    const tdefl_status status = tdefl_init(
        static_cast<tdefl_compressor*>(compressor), putBuffer, this,
        DEFLATE_FLAGS);
    if (status != TDEFL_STATUS_OKAY) {
        end();
        return false;
    }

    const size_t headerWritten = file.write(GZIP_HEADER, sizeof(GZIP_HEADER));
    writtenBytes += headerWritten;
    if (headerWritten != sizeof(GZIP_HEADER)) {
        failed = true;
        end();
        return false;
    }
    return true;
}

size_t GzipStream::write(const uint8_t* data, size_t length) {
    if (compressor == nullptr || failed || finished || length == 0) {
        return 0;
    }

    const uint32_t before = writtenBytes;
    // tdefl_compress_buffer() verbraucht den Eingabepuffer stets vollstaendig.
    const tdefl_status status = tdefl_compress_buffer(
        static_cast<tdefl_compressor*>(compressor), data, length,
        TDEFL_NO_FLUSH);
    if (status != TDEFL_STATUS_OKAY) {
        failed = true;
        return writtenBytes - before;
    }

    crc = mz_crc32(crc, data, length);
    sourceBytes += length;
    return writtenBytes - before;
}

size_t GzipStream::flush() {
    if (compressor == nullptr || failed || finished) {
        return 0;
    }
    const uint32_t before = writtenBytes;
    const tdefl_status status = tdefl_compress_buffer(
        static_cast<tdefl_compressor*>(compressor), nullptr, 0,
        TDEFL_SYNC_FLUSH);
    if (status != TDEFL_STATUS_OKAY) {
        failed = true;
    }
    return writtenBytes - before;
}

size_t GzipStream::finish() {
    if (compressor == nullptr || finished) {
        return 0;
    }
    const uint32_t before = writtenBytes;

    if (!failed) {
        const tdefl_status status = tdefl_compress_buffer(
            static_cast<tdefl_compressor*>(compressor), nullptr, 0,
            TDEFL_FINISH);
        if (status != TDEFL_STATUS_DONE) {
            failed = true;
        }
    }

    // gzip-Nachspann: CRC32 und Länge der Nutzdaten, je vier Byte in
    // Little-Endian. Ohne ihn meldet jedes Auspackwerkzeug einen
    // unvollständigen Strom.
    if (!failed && target != nullptr) {
        uint8_t trailer[8];
        trailer[0] = static_cast<uint8_t>(crc & 0xFF);
        trailer[1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        trailer[2] = static_cast<uint8_t>((crc >> 16) & 0xFF);
        trailer[3] = static_cast<uint8_t>((crc >> 24) & 0xFF);
        trailer[4] = static_cast<uint8_t>(sourceBytes & 0xFF);
        trailer[5] = static_cast<uint8_t>((sourceBytes >> 8) & 0xFF);
        trailer[6] = static_cast<uint8_t>((sourceBytes >> 16) & 0xFF);
        trailer[7] = static_cast<uint8_t>((sourceBytes >> 24) & 0xFF);
        const size_t written = target->write(trailer, sizeof(trailer));
        writtenBytes += written;
        if (written != sizeof(trailer)) {
            failed = true;
        }
    }

    finished = true;
    return writtenBytes - before;
}

void GzipStream::end() {
    if (compressor != nullptr) {
        heap_caps_free(compressor);
        compressor = nullptr;
    }
    target = nullptr;
    finished = false;
}

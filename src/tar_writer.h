#ifndef TAR_WRITER_H
#define TAR_WRITER_H

// Erzeugt tar-Köpfe im ustar-Format.
//
// Bewusst frei von Arduino, SD und Netzwerk: Ein falscher Kopf macht jedes
// heruntergeladene Archiv unbrauchbar, und das fiele erst am Handy im Auto
// auf. Als eigene Einheit lässt sich die Rahmung auf dem Entwicklungsrechner
// gegen das echte `tar` prüfen - dieselbe Trennung wie bei curve_detector und
// road_metrics.
//
// Warum tar: Eine Sitzung besteht aus zehn Dateien. Einzeln geholt sind das
// zehn Tippvorgänge je Fahrt, und die Streckendatenbank entsteht erst bei
// zweihundert Fahrten. `tar xzf` liefert am Rechner genau die
// Verzeichnisstruktur, die die Auswertewerkzeuge erwarten.

#include <stddef.h>
#include <stdint.h>

// Größe eines tar-Blocks. Köpfe belegen genau einen, Nutzdaten werden auf ein
// Vielfaches aufgefüllt, und zwei Nullblöcke schließen das Archiv ab.
constexpr size_t TAR_BLOCK_SIZE = 512;

// Schreibt den Kopf einer gewöhnlichen Datei in einen 512-Byte-Block.
//
// `name` ist der Pfad im Archiv und wird auf 99 Zeichen begrenzt; längere
// Namen entstehen bei Sitzungsverzeichnis plus Dateiname nicht. `size` ist die
// Länge der folgenden Nutzdaten in Byte.
void tarWriteHeader(uint8_t* block, const char* name, uint32_t size);

// Zahl der Füllbytes, die nach `size` Nutzdaten bis zur Blockgrenze fehlen.
size_t tarPaddingBytes(uint32_t size);

#endif  // TAR_WRITER_H

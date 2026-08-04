#include "tar_writer.h"

#include <stdio.h>
#include <string.h>

void tarWriteHeader(uint8_t* block, const char* name, uint32_t size) {
    memset(block, 0, TAR_BLOCK_SIZE);

    // Der Name belegt 100 Byte und braucht keinen Abschluss, wenn er sie
    // vollständig füllt. strncpy mit 99 lässt das Nullbyte in jedem Fall
    // stehen.
    strncpy(reinterpret_cast<char*>(block), name, 99);

    // Mode, uid und gid als oktale Zeichenketten. snprintf hängt jeweils ein
    // Nullbyte an; das ist im tar-Format zulässig und üblich.
    snprintf(reinterpret_cast<char*>(block + 100), 8, "0000644");
    snprintf(reinterpret_cast<char*>(block + 108), 8, "0000000");
    snprintf(reinterpret_cast<char*>(block + 116), 8, "0000000");

    // Größe und Änderungszeit oktal mit elf Stellen plus Nullbyte. Die Zeit
    // bleibt null: Die Sitzungszeit steht im Verzeichnisnamen und in den Daten
    // selbst, und eine falsche Zeit wäre schlechter als keine.
    snprintf(reinterpret_cast<char*>(block + 124), 12, "%011lo",
             static_cast<unsigned long>(size));
    snprintf(reinterpret_cast<char*>(block + 136), 12, "%011lo", 0UL);

    block[156] = '0';  // Typ: gewöhnliche Datei
    memcpy(block + 257, "ustar", 5);
    memcpy(block + 263, "00", 2);

    // Die Prüfsumme ist die Bytesumme des gesamten Kopfes, wobei ihr eigenes
    // Feld dabei als acht Leerzeichen zählt. Danach steht dort die Summe
    // oktal, gefolgt von Nullbyte und Leerzeichen - so schreiben es die
    // gängigen Implementierungen, und darauf prüfen sie auch.
    memset(block + 148, ' ', 8);
    uint32_t summe = 0;
    for (size_t i = 0; i < TAR_BLOCK_SIZE; ++i) {
        summe += block[i];
    }
    snprintf(reinterpret_cast<char*>(block + 148), 8, "%06lo",
             static_cast<unsigned long>(summe));
    block[154] = '\0';
    block[155] = ' ';
}

size_t tarPaddingBytes(uint32_t size) {
    const size_t rest = static_cast<size_t>(size % TAR_BLOCK_SIZE);
    return rest == 0 ? 0 : TAR_BLOCK_SIZE - rest;
}

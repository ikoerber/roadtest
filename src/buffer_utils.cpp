#include "buffer_utils.h"
#include <stdarg.h>

// Globale Instanzen
SafeMemoryPool<2048, 64> globalMemoryPool;
SafeStackBuffer<1024> formatBuffer;

// =============================================================================
// SafeStringFormatter Implementation
// =============================================================================

int SafeStringFormatter::safePrintf(char* buffer, size_t bufferSize, const char* format, ...) {
    if (!buffer || bufferSize == 0 || !format) {
        return -1;
    }
    
    va_list args;
    va_start(args, format);
    
    int result = vsnprintf(buffer, bufferSize, format, args);
    
    va_end(args);
    
    // Prüfe auf Truncation
    if (result >= (int)bufferSize) {
        Serial.printf("⚠️ String truncated: needed %d, got %zu\n", result, bufferSize);
        buffer[bufferSize - 1] = '\0'; // Stelle null-termination sicher
        return -1;
    }
    
    return result;
}

bool SafeStringFormatter::safeStrcat(char* dest, size_t destSize, const char* src) {
    if (!dest || !src || destSize == 0) {
        return false;
    }
    
    size_t destLen = strlen(dest);
    size_t srcLen = strlen(src);
    
    // Prüfe verfügbaren Platz (1 Byte für null-terminator)
    if (destLen + srcLen >= destSize) {
        Serial.printf("⚠️ strcat overflow verhindert: %zu + %zu >= %zu\n", 
                     destLen, srcLen, destSize);
        return false;
    }
    
    // Sichere Verkettung
    strncat(dest, src, destSize - destLen - 1);
    dest[destSize - 1] = '\0'; // Sicherheits-null-termination
    
    return true;
}

bool SafeStringFormatter::safeStrcpy(char* dest, size_t destSize, const char* src) {
    if (!dest || !src || destSize == 0) {
        return false;
    }
    
    size_t srcLen = strlen(src);
    
    if (srcLen >= destSize) {
        Serial.printf("⚠️ strcpy overflow verhindert: %zu >= %zu\n", srcLen, destSize);
        return false;
    }
    
    // Sichere Kopierung
    strncpy(dest, src, destSize - 1);
    dest[destSize - 1] = '\0'; // Sicherheits-null-termination
    
    return true;
}

bool SafeStringFormatter::formatFloat(char* buffer, size_t bufferSize, float value, int decimals) {
    if (!buffer || bufferSize == 0) {
        return false;
    }
    
    // Begrenze Dezimalstellen auf sinnvollen Bereich
    if (decimals < 0) decimals = 0;
    if (decimals > 6) decimals = 6;
    
    int result = snprintf(buffer, bufferSize, "%.*f", decimals, value);
    
    if (result >= (int)bufferSize) {
        Serial.printf("⚠️ Float formatting truncated: %.3f\n", value);
        return false;
    }
    
    return true;
}

bool SafeStringFormatter::formatSensorCSV(char* buffer, size_t bufferSize, 
                                         unsigned long timestamp, float* data, int dataCount) {
    if (!buffer || !data || bufferSize == 0 || dataCount <= 0) {
        return false;
    }
    
    // Beginne mit Timestamp
    int written = snprintf(buffer, bufferSize, "%lu", timestamp);
    
    if (written >= (int)bufferSize) {
        return false;
    }
    
    // Füge Daten-Werte hinzu
    for (int i = 0; i < dataCount && written < (int)bufferSize; i++) {
        int additional = snprintf(buffer + written, bufferSize - written, ",%.3f", data[i]);
        
        if (additional < 0 || written + additional >= (int)bufferSize) {
            Serial.printf("⚠️ CSV formatting truncated at field %d\n", i);
            return false;
        }
        
        written += additional;
    }
    
    // Füge Newline hinzu
    if (written + 1 < (int)bufferSize) {
        buffer[written] = '\n';
        buffer[written + 1] = '\0';
        return true;
    }
    
    return false;
}

// =============================================================================
// Buffer-Overflow Detection für Arduino String-Klasse
// =============================================================================

/**
 * Wrapper für Arduino String mit Längen-Überwachung
 */
class SafeString {
private:
    String str;
    size_t maxLength;
    bool overflowDetected;
    
    void checkOverflow() {
        if (str.length() > maxLength) {
            Serial.printf("⚠️ SafeString overflow: %zu > %zu\n", str.length(), maxLength);
            overflowDetected = true;
            
            // Kürze String auf maximale Länge
            str = str.substring(0, maxLength);
        }
    }
    
public:
    SafeString(size_t maxLen = 512) : maxLength(maxLen), overflowDetected(false) {}
    
    SafeString& operator+=(const String& other) {
        if (str.length() + other.length() <= maxLength) {
            str += other;
        } else {
            Serial.printf("⚠️ SafeString += overflow verhindert\n");
            overflowDetected = true;
        }
        return *this;
    }
    
    SafeString& operator+=(const char* other) {
        return *this += String(other);
    }
    
    SafeString& operator=(const String& other) {
        str = other;
        checkOverflow();
        return *this;
    }
    
    const char* c_str() const { return str.c_str(); }
    size_t length() const { return str.length(); }
    bool hasOverflowed() const { return overflowDetected; }
    void clearOverflowFlag() { overflowDetected = false; }
    
    // Sichere printf-style Formatierung
    bool printf(const char* format, ...) {
        char buffer[maxLength + 1];
        va_list args;
        va_start(args, format);
        
        int result = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        if (result >= (int)sizeof(buffer)) {
            overflowDetected = true;
            return false;
        }
        
        str = String(buffer);
        return true;
    }
    
    String toString() const { return str; }
};

// =============================================================================
// Debug-Funktionen für Buffer-Monitoring
// =============================================================================

void printBufferStats() {
    Serial.println("=== Buffer Statistics ===");
    
    // Memory Pool Stats
    globalMemoryPool.printStats();
    
    // Format Buffer Stats  
    Serial.printf("Format Buffer: %zu/%zu bytes used (%.1f%%)\n",
                 formatBuffer.used(), 1024, 
                 (float)formatBuffer.used() / 1024.0f * 100.0f);
                 
    if (formatBuffer.hasOverflowed()) {
        Serial.println("⚠️ Format Buffer overflow detected!");
    }
    
    // ESP32 Heap Stats
    Serial.printf("ESP32 Heap: %zu bytes free, %zu largest block\n",
                 ESP.getFreeHeap(), ESP.getMaxAllocHeap());
                 
    Serial.println("========================");
}
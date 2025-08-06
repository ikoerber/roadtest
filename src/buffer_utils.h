#ifndef BUFFER_UTILS_H
#define BUFFER_UTILS_H

#include <Arduino.h>

// =============================================================================
// Sichere Buffer-Utility-Funktionen
// =============================================================================
// Zentrale Buffer-Overflow-Schutz Funktionen für das gesamte System

/**
 * Sichere String-Formatierung mit Buffer-Schutz
 * Ersetzt unsichere sprintf/String-Konkatenationen
 */
class SafeStringFormatter {
private:
    static const size_t MAX_FORMAT_LENGTH = 512;
    
public:
    /**
     * Sichere printf-Style Formatierung
     * @param buffer Ziel-Buffer
     * @param bufferSize Größe des Ziel-Buffers
     * @param format Format-String
     * @param ... Variable Argumente
     * @return Anzahl geschriebener Zeichen oder -1 bei Fehler
     */
    static int safePrintf(char* buffer, size_t bufferSize, const char* format, ...);
    
    /**
     * Sichere String-Verkettung
     * @param dest Ziel-String
     * @param destSize Maximale Größe des Ziel-Strings
     * @param src Quell-String
     * @return true bei Erfolg, false bei Overflow
     */
    static bool safeStrcat(char* dest, size_t destSize, const char* src);
    
    /**
     * Sichere String-Kopierung
     * @param dest Ziel-Buffer
     * @param destSize Größe des Ziel-Buffers
     * @param src Quell-String
     * @return true bei Erfolg, false bei Overflow
     */
    static bool safeStrcpy(char* dest, size_t destSize, const char* src);
    
    /**
     * Formatiere Float-Wert sicher
     * @param buffer Ziel-Buffer
     * @param bufferSize Größe des Ziel-Buffers
     * @param value Float-Wert
     * @param decimals Anzahl Dezimalstellen
     * @return true bei Erfolg, false bei Overflow
     */
    static bool formatFloat(char* buffer, size_t bufferSize, float value, int decimals = 2);
    
    /**
     * Formatiere Sensor-Daten sicher für CSV
     * @param buffer Ziel-Buffer (min 256 Bytes empfohlen)
     * @param bufferSize Größe des Ziel-Buffers
     * @param timestamp Zeitstempel
     * @param data Sensor-Daten Array
     * @param dataCount Anzahl Werte
     * @return true bei Erfolg, false bei Overflow
     */
    static bool formatSensorCSV(char* buffer, size_t bufferSize, 
                               unsigned long timestamp, float* data, int dataCount);
};

/**
 * Ring-Buffer mit Overflow-Schutz
 * Template-basierte sichere Ring-Buffer-Implementierung
 */
template<typename T, size_t SIZE>
class SafeRingBuffer {
private:
    T buffer[SIZE];
    size_t head;
    size_t tail;
    size_t count;
    bool overflow_occurred;
    
public:
    SafeRingBuffer() : head(0), tail(0), count(0), overflow_occurred(false) {}
    
    /**
     * Füge Element zum Buffer hinzu
     * @param item Element zum Hinzufügen
     * @return true bei Erfolg, false bei Overflow
     */
    bool push(const T& item) {
        if (count >= SIZE) {
            overflow_occurred = true;
            // Überschreibe ältestes Element
            tail = (tail + 1) % SIZE;
            count--;
        }
        
        buffer[head] = item;
        head = (head + 1) % SIZE;
        count++;
        
        return !overflow_occurred;
    }
    
    /**
     * Hole Element vom Buffer
     * @param item Referenz zum Element
     * @return true wenn Element verfügbar, false wenn leer
     */
    bool pop(T& item) {
        if (count == 0) return false;
        
        item = buffer[tail];
        tail = (tail + 1) % SIZE;
        count--;
        
        return true;
    }
    
    /**
     * Prüfe ob Buffer voll ist
     */
    bool isFull() const { return count >= SIZE; }
    
    /**
     * Prüfe ob Buffer leer ist
     */
    bool isEmpty() const { return count == 0; }
    
    /**
     * Aktuelle Anzahl Elemente
     */
    size_t size() const { return count; }
    
    /**
     * Maximale Kapazität
     */
    size_t capacity() const { return SIZE; }
    
    /**
     * Prüfe ob Overflow aufgetreten ist
     */
    bool hasOverflowed() const { return overflow_occurred; }
    
    /**
     * Reset Overflow-Flag
     */
    void clearOverflowFlag() { overflow_occurred = false; }
    
    /**
     * Buffer leeren
     */
    void clear() {
        head = tail = count = 0;
        overflow_occurred = false;
    }
};

/**
 * Stack-Buffer mit automatischem Overflow-Schutz
 */
template<size_t STACK_SIZE>
class SafeStackBuffer {
private:
    char stack[STACK_SIZE];
    size_t stackPointer;
    bool overflow_detected;
    
public:
    SafeStackBuffer() : stackPointer(0), overflow_detected(false) {
        memset(stack, 0, STACK_SIZE);
    }
    
    /**
     * Allokiere Speicher vom Stack
     * @param size Benötigte Bytes
     * @return Pointer auf Speicher oder nullptr bei Overflow
     */
    char* allocate(size_t size) {
        if (stackPointer + size >= STACK_SIZE) {
            overflow_detected = true;
            return nullptr;
        }
        
        char* ptr = &stack[stackPointer];
        stackPointer += size;
        return ptr;
    }
    
    /**
     * Gebe Speicher frei (einfache Implementierung)
     * @param size Anzahl Bytes freizugeben
     */
    void deallocate(size_t size) {
        if (size <= stackPointer) {
            stackPointer -= size;
        } else {
            stackPointer = 0; // Reset bei Fehler
        }
    }
    
    /**
     * Verfügbarer Speicher
     */
    size_t available() const {
        return STACK_SIZE - stackPointer;
    }
    
    /**
     * Verwendeter Speicher
     */
    size_t used() const {
        return stackPointer;
    }
    
    /**
     * Overflow-Status
     */
    bool hasOverflowed() const {
        return overflow_detected;
    }
    
    /**
     * Reset Stack
     */
    void reset() {
        stackPointer = 0;
        overflow_detected = false;
        memset(stack, 0, STACK_SIZE);
    }
};

// =============================================================================
// Memory-Pool für sichere dynamische Allokation
// =============================================================================

/**
 * Einfacher Memory-Pool für ESP32
 * Verhindert Heap-Fragmentierung und Out-of-Memory
 */
template<size_t POOL_SIZE, size_t BLOCK_SIZE>
class SafeMemoryPool {
private:
    static const size_t NUM_BLOCKS = POOL_SIZE / BLOCK_SIZE;
    char pool[POOL_SIZE];
    bool blockUsed[NUM_BLOCKS];
    size_t allocatedBlocks;
    
public:
    SafeMemoryPool() : allocatedBlocks(0) {
        memset(pool, 0, POOL_SIZE);
        memset(blockUsed, false, sizeof(blockUsed));
    }
    
    /**
     * Allokiere Block
     * @return Pointer auf Block oder nullptr wenn voll
     */
    void* allocate() {
        for (size_t i = 0; i < NUM_BLOCKS; i++) {
            if (!blockUsed[i]) {
                blockUsed[i] = true;
                allocatedBlocks++;
                return &pool[i * BLOCK_SIZE];
            }
        }
        return nullptr; // Pool voll
    }
    
    /**
     * Gebe Block frei
     * @param ptr Pointer auf Block
     * @return true bei Erfolg
     */
    bool deallocate(void* ptr) {
        if (ptr < pool || ptr >= pool + POOL_SIZE) {
            return false; // Ungültiger Pointer
        }
        
        size_t index = ((char*)ptr - pool) / BLOCK_SIZE;
        if (index < NUM_BLOCKS && blockUsed[index]) {
            blockUsed[index] = false;
            allocatedBlocks--;
            return true;
        }
        
        return false;
    }
    
    /**
     * Verfügbare Blöcke
     */
    size_t availableBlocks() const {
        return NUM_BLOCKS - allocatedBlocks;
    }
    
    /**
     * Verwendete Blöcke
     */
    size_t usedBlocks() const {
        return allocatedBlocks;
    }
    
    /**
     * Pool-Statistiken
     */
    void printStats() {
        Serial.printf("Memory Pool: %zu/%zu blocks used (%.1f%%)\n", 
                     allocatedBlocks, NUM_BLOCKS, 
                     (float)allocatedBlocks / NUM_BLOCKS * 100.0f);
    }
};

// =============================================================================
// Globale sichere Buffer-Instanzen
// =============================================================================

// Globaler sicherer Memory Pool für kleine Allokationen
extern SafeMemoryPool<2048, 64> globalMemoryPool;

// Sicherer Formatierungs-Buffer für temporäre String-Operationen
extern SafeStackBuffer<1024> formatBuffer;

// Makros für sichere Buffer-Operationen
#define SAFE_SPRINTF(buf, fmt, ...) SafeStringFormatter::safePrintf(buf, sizeof(buf), fmt, ##__VA_ARGS__)
#define SAFE_STRCAT(dest, src) SafeStringFormatter::safeStrcat(dest, sizeof(dest), src)
#define SAFE_STRCPY(dest, src) SafeStringFormatter::safeStrcpy(dest, sizeof(dest), src)

#endif // BUFFER_UTILS_H
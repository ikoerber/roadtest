#ifndef ROAD_QUALITY_H
#define ROAD_QUALITY_H

#include <Arduino.h>

// Bewertungsparameter Struktur
struct RoadMetrics {
    // Kurvigkeit (40%)
    float curveFrequency;      // Kurven pro km
    float curveVariation;      // Standardabweichung der Kurvenradien
    float curveFlow;           // Harmonische Übergänge (0-1)
    
    // Oberflächenqualität (35%)
    float vibrationRMS;        // RMS der Z-Beschleunigung
    float maxShock;            // Maximaler Stoß in g
    float surfaceSmoothness;   // 0-1 (1 = perfekt glatt)
    
    // Verkehrsruhe (15%)
    float speedVariation;      // Standardabweichung der Geschwindigkeit
    float stopCount;           // Anzahl Stopps pro km
    
    // Landschaftsbonus (10%)
    float elevationChange;     // Höhenmeter pro km
    float serpentineScore;     // Serpentinen-Erkennung
};

// Globale Variablen (extern)
extern RoadMetrics currentMetrics;
extern float totalDistance;
extern int curveCount;
extern float lastHeading;
extern bool inCurve;
extern float curveStartHeading;
extern unsigned long lastUpdate;

// Ringpuffer für Beschleunigungswerte
extern const int ACCEL_BUFFER_SIZE;
extern float accelBuffer[];
extern int accelBufferIndex;

// Funktionsdeklarationen - nicht mehr benötigt, da in Manager-Klassen implementiert
// void updateCurveDetection();        // -> BNO055Manager::detectCurve()
// void updateSurfaceQuality();        // -> BNO055Manager::analyzeVibration() 
// float calculateOverallScore();      // -> BNO055Manager::calculateRoadQuality()
// void displayResults();              // -> OLEDManager::showRoadQuality()
// void displaySensorData();           // -> OLEDManager::showSensorData()
// void readCANMessages();             // -> CANReader::readMessage()

#endif
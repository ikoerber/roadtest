# 🏍️ ESP32-S3 Straßenqualitäts-Messsystem

Ein fortschrittliches Embedded-System zur Messung und Bewertung von Straßenqualität für kurvenreiche Motorradstrecken.

Aktueller Firmwarestand: **1.5.32**. Der Stand ist ein Hardware- und
Fahrzeugteststand. 1.5.28 misst Kurven über eine einbaulagenunabhängige
Gyroskopprojektion, protokolliert Radius und vollständige Ereignisintervalle
und stellt dafür einen geführten Beifahrertest bereit. 1.5.29 schärft die
Ereignisgrenzen anhand der fünf Referenzfahrten vom 31.07.2026.

## 📋 Überblick

Das System erfasst Bewegungsdaten, GPS-Position und CAN-Bus-Signale, um die Qualität von Straßen zu bewerten. Besonders geeignet für:
- **Kurvenreiche Strecken** mit Serpentinen und Bergstraßen
- **Vibrations-Analyse** für Straßenoberflächen-Bewertung  
- **GPS-basierte Strecken-Dokumentation**
- **Multi-Sensor-Datenlogging** auf SD-Karte

## 🎯 Features

### ✨ Hauptfunktionen
- **Real-Time Straßenqualitäts-Bewertung** (0-100 Punkte)
- **Multi-Sensor-Fusion** (BNO055 + GPS + CAN)
- **Robustes SD-Karten-Logging** mit Buffer-Overflow-Schutz
- **SD-Wiederanlauf** mit Recovery-Datei und verknüpfter Fortsetzungssitzung
- **Eigenes ROADTEST-WLAN** mit Statusseite, Kalibrierassistent und Browser-OTA
- **Kontrollierte Messfahrten** mit Start, sicherem Ende und Zusammenfassung
- **Umfassende Hardware-Tests** und Diagnostik
- **Gepufferter GPS-Empfang** mit sichtbaren Überlauf- und Prüfsummenzählern
- **NVS-Kalibrierungsspeicher** für BNO055 (persistiert über Neustarts)

### 🛡️ Sicherheitsfeatures
- **Buffer-Overflow-Schutz** mit SafeStringFormatter
- **Memory-Pool-Management** gegen Heap-Fragmentierung
- **Hardware-Fehler-Recovery** mit automatischen Fallbacks
- **Multi-Layer-Error-Handling** für kritische Systeme
- **NEU: Erweiterte String-Sicherheit** mit strncpy/strncat
- **NEU: Null-Pointer-Checks** für alle dynamischen Allokationen

### 📊 Datenerfassung
- **Beschleunigungsdaten** mit konfiguriertem 10-Hz-Messintervall
- **GPS-Qualitätssnapshots** (5 Hz) mit neuen Fixes, Feldalter und HDOP
- **Begrenzte OBD-II-Liveabfrage** über MCP2515 (nur lesender Service 01)
- **Korrelierte Datenlogs** mit präzisen Zeitstempeln
- **Interrupt-basierter GPS-Ringpuffer** mit automatischer UART-Recovery
- **Begrenzte String-Operationen** für bessere Robustheit

## 🔧 Hardware-Anforderungen

### ESP32-S3 Entwicklungsboard
- **Mikrocontroller:** ESP32-S3 (240MHz, LOLIN S3 Mini mit 4MB Flash)
- **USB:** USB-C für Programming und Debug
- **Versorgung:** USB-C oder vorhandener LiPo-Anschluss
- **GPIO-Pegel:** ausschließlich 3,3 V; die GPIOs sind nicht 5-V-tolerant

### Sensoren und Module

| Komponente | Modell | Schnittstelle | Pins | Funktion |
|------------|--------|---------------|------|----------|
| **IMU-Sensor** | BNO055 | I2C | GPIO 8/9 | NDOF-Sensorfusion |
| **Display** | SSD1306 | I2C | GPIO 8/9 | verbaut, wird nicht angesteuert; dient der I²C-Diagnose |
| **GPS-Modul** | BN-880 | UART2 | GPIO 15/16 | Position & Geschwindigkeit |
| **CAN-Interface** | Joy-IT SBC-CAN01, MCP2515/MCP2562 | SPI | GPIO 1,2,3,11,13 | OBD-II, 11 Bit, 500 kbit/s |
| **SD-Speicher** | MicroSD | SPI | GPIO 4,5,6,7 | Datenlogging |

## 📐 Schaltplan & Verkabelung

Die vollständige und verbindliche Beschreibung des vorhandenen
Lochrasteraufbaus steht in [HARDWARE.md](HARDWARE.md). Eine kompakte grafische
Übersicht enthält [schematic.md](schematic.md). Maßgeblich sind GPIO- und
Signalnamen, nicht die Kabelfarben.

### I2C-Bus (BNO055 + OLED)
```
ESP32-S3        BNO055          SSD1306 OLED
--------        ------          ------------
GPIO 8    <-->  SDA      <-->   SDA
GPIO 9    <-->  SCL      <-->   SCL  
3.3V      -->   VIN      -->    VCC
GND       -->   GND      -->    GND
```

Am BNO055 bleiben `3Vo`, `RST`, `PS0`, `PS1` und `INT` unbeschaltet. `ADR`
ist im vorhandenen Aufbau mit 3,3 V verbunden; dadurch lautet die Adresse
`0x29`.

### GPS-Modul (UART2)
```
ESP32-S3        BN-880 GPS
--------        ----------
GPIO 16   <--   TX (NMEA-Daten)
GPIO 15   -->   RX (Konfiguration)
3.3V      -->   VCC
GND       -->   GND
offen           PPS (von der Firmware nicht verwendet)
```

### CAN-Bus Interface
```
ESP32-S3        MCP2515
--------        -------
GPIO 1    -->   CS
GPIO 2    <--   INT
GPIO 3    -->   SCK
GPIO 13   -->   MOSI
GPIO 11   <--   MISO
GND       -->   GND
```

Die Firmware fragt am bestätigten ISO-15765-4-Anschluss des Porsche 991.1
höchstens zweimal pro Sekunde standardisierte OBD-Livedaten ab: Motordrehzahl
(PID `0x0C`), Geschwindigkeit (`0x0D`) und Drosselklappenstellung (`0x11`).
Sie implementiert weder Fehlerlöschen noch Codierung oder Stellgliedtests.
Versorgung, gemeinsame Masse und SPI-Logikpegel müssen vor dem Anschluss
gemäß [HARDWARE.md](HARDWARE.md) geprüft sein.

### SD-Karten-Modul
```
ESP32-S3        SD-Card Module
--------        --------------
GPIO 4    -->   CS
GPIO 5    -->   MOSI
GPIO 6    <--   MISO
GPIO 7    -->   SCK
3.3V      -->   3.3V (gemäß Beschriftung des verwendeten PZSMOCN-Moduls)
GND       -->   GND
```

## 🚀 Installation & Setup

### 1. Development Environment
```bash
# PlatformIO installieren
pip install platformio

# Repository klonen
git clone <repository-url>
cd roadtest

# Dependencies installieren (automatisch via platformio.ini)
pio lib install
```

### 2. Verbindliche Hardware-Konfiguration
```cpp
// Die Pins sind für den vorhandenen Aufbau in
// src/hardware_config.cpp festgelegt.
const int I2C_SDA = 8;
const int I2C_SCL = 9;
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 15;
const int SD_CS_PIN = 4;
const int CAN_CS_PIN = 1;
```

Die Pinbelegung nicht ohne entsprechende Änderung der realen Verdrahtung
ändern. Maßgeblich sind [HARDWARE.md](HARDWARE.md) und
[schematic.md](schematic.md).

### 3. Firmware Upload
```bash
# Build & Upload
pio run --target upload

# Serial Monitor für Debug
pio device monitor --baud 115200
```

### 4. Erweiterte Konfiguration

#### GPS Interrupt-Modus
```cpp
// In main.cpp - automatisch aktiviert
gpsManager.enableInterruptMode(true);  // Gepufferter UART-Empfang

// Manuell umschalten (falls gewünscht)
gpsManager.enableInterruptMode(false); // Zurück zu Polling
```

Der Ringpuffer entkoppelt UART-Empfang und Auswertung, garantiert aber bei
langen Programmpausen keine Verlustfreiheit. Überläufe und NMEA-
Prüfsummenfehler werden deshalb pro Sitzung gezählt.

#### WLAN, Messfahrten, Kalibrierung und OTA

1. Mit dem WLAN `ROADTEST` verbinden, Passwort `roadtest123`.
2. Im Browser `http://192.168.4.1/` öffnen.
3. Unter **Kalibrierung** dem automatisch aktualisierten Assistenten folgen.
   Er zeigt nacheinander Ruhigstellen, sechs Gerätelagen, die liegende Acht
   und die abschließende Systemfusion an. Sobald alle erforderlichen Werte 3
   erreichen, den aktuellen Stand speichern.
4. Vor der Abfahrt **Aufzeichnung starten** auswählen.
5. Am Fahrtende **Aufzeichnung beenden** auswählen. Erst dann sind alle
   Dateien garantiert geschlossen und die Zusammenfassung wird geschrieben.
6. Unter **Firmware aktualisieren** kann eine neue Web-OTA-`.bin` eingespielt
   werden. Während Browser-OTA müssen CAN-H und CAN-L vom Fahrzeug getrennt
   sein.

Für Start, Ende, Speichern und Aktualisieren wird Benutzer `admin` mit
Passwort `roadtest123` verwendet. Die Kalibrierung wird anschließend beim
Start automatisch aus dem NVS geladen.

## 🧪 Startprüfung und System-Tests

Der Prüfstand meldet sich über die serielle Ausgabe und die Statusseite im
ROADTEST-WLAN. Erforderlich sind BNO055, SD-Karte, GPS-NMEA-Daten und WLAN;
ein GPS-Fix ist nicht nötig. CAN bleibt für die allgemeine Bereitschaft
optional. Ob alles Erforderliche bereit ist, zeigt der serielle Befehl `diag`
in der Zeile "Pflichthardware bereit".

Fehlt eine erforderliche Komponente, bleibt die Firmware trotzdem bedienbar
und versucht alle fünf Sekunden automatisch eine Wiederverbindung.

Das OLED wird seit 1.5.32 nicht mehr angesteuert. Es bleibt verbaut, weil die
Firmware seine I²C-Adresse weiter anpingt und Aussetzer zählt: Als zweiter
unabhängiger Busteilnehmer trennt es Sensorfehler von Bus- und
Versorgungsfehlern. An- und abgesteckt wird nur bei ausgeschaltetem System.

Der serielle Fünf-Sekunden-Status enthält für mechanische Tests zusätzlich:

- kumulierte I²C-Aussetzer von BNO055 und OLED
- getrennten CAN-Adapter- und ECU-Verbindungsstatus samt Antwortalter
- SD-Schreibvorgänge, Schreibfehler und verworfene Datensätze
- WLAN-Zustand; `diag` gibt dieselben Diagnosezähler auf Anforderung aus

Ohne Fahrzeug können die aktiven OBD-Anfragen mit `obd off` pausiert werden;
`obd on` aktiviert sie vor dem nächsten Fahrzeugtest wieder. Der MCP2515
bleibt dabei initialisiert, und Webseite sowie serieller Status zeigen die
Pause ausdrücklich an.

### Fahrzeugdaten für die spätere Auswertung erkennen

Seit Version 1.5.14 besitzt die Firmware eine eigene Diagnoseaufzeichnung.
Seit Version 1.5.16 ist die GPS-/OBD-Datenqualität messbar. Version 1.5.20
ergänzt die automatische ECU- und Controller-Wiederherstellung sowie einen
geführten Abnahmetest mit getrennten Schritten für Zündung und Motorstart.
Version 1.5.21 aktiviert feldweise GPS-Alters- und Qualitätsgrenzen und
verhindert, dass GPS-Drift bei bestätigtem Fahrzeugstillstand als Strecke
gezählt wird.
Die Discovery startet und beendet ihre SD-Sitzung selbst:

```text
discover begin
discover status
discover mark motor_gestartet
discover mark fahrt_begonnen
discover mark gleichmaessig_50_kmh
discover end
```

`discover begin` führt automatisch drei Phasen aus:

1. **60 Sekunden passiv:** Der MCP2515 arbeitet im echten Listen-Only-Modus
   und sendet weder Diagnoseanfragen noch CAN-Bestätigungen. Alle am
   OBD-Anschluss sichtbaren 11-Bit-Telegramme werden aufgezeichnet.
2. **Standard-PID-Scan:** Die Blöcke `0100`, `0120`, `0140` und `0160` werden
   zweimal und mit insgesamt höchstens zwei Anfragen pro Sekunde gelesen.
3. **Messphase:** Nur bestätigte Standard-PIDs werden bis `discover end`
   abgefragt. Geschwindigkeit erhält für den späteren GPS-Vergleich die
   höchste Gewichtung; langsame Temperaturwerte werden seltener gelesen.

Die Dateien mit demselben Sitzungsnamen gehören zusammen:

- `road_sensor_...csv`: Orientierung, lineare Beschleunigung, Gyro und
  BNO055-Temperatur
- `road_gps_...csv`: Position, GPS-Geschwindigkeit, Kurs, Satelliten und HDOP
- `road_can_...csv`: unveränderte CAN-Rohframes mit ECU-ID
- `road_obd_...csv`: dekodierte Standardwerte und PID-Unterstützungsbitmaps
- `road_obd_trace_...csv`: einzelne Anfragen, Antworten, Timeouts,
  Zuordnung, Latenz und MCP2515-Diagnose
- `road_meta_...csv`: Firmware-/Schemaversion, Konfiguration sowie Start-,
  Fünf-Sekunden- und Abschlusszähler der Sitzung einschließlich maximaler
  Hauptschleifen-, Web- und SD-Pausen
- `road_event_...csv`: Phasenwechsel und manuelle `discover mark`-Zeitpunkte

GPS-Zeichen-, Satz-, Prüfsummen-, Ringpuffer- und Fixsequenzzähler beginnen
für jede Aufzeichnung bei null. Dadurch lassen sich Sitzungen direkt auswerten,
ohne vor dem Start aufgelaufene Boot-Gesamtwerte abzuziehen.
Die GPS-Rohwerte bleiben für die Diagnose sichtbar. Nur Felder mit gesetztem
Gültigkeitsflag dürfen als Messwert verwendet werden. `RejectionReason`
enthält eine Bitmaske, sodass mehrere gleichzeitig verletzte Alters- oder
Qualitätsgrenzen nachvollziehbar bleiben.

Wenn das Porsche-Gateway keine zyklischen Fahrzeugtelegramme an den
OBD-Anschluss weiterleitet, bleibt die passive CAN-Datei in der ersten Phase
leer. Das ist ein verwertbares Testergebnis und kein Loggerfehler. Der
PID-Scan erkennt ausschließlich standardisierte Service-01-Werte; unbekannte
Porsche-UDS-Kennungen werden nicht durchprobiert.

Während dieses Ablaufs sind `start`, `stop` und `obd on/off` absichtlich
gesperrt. `discover end` stellt den
vorherigen OBD-Zustand wieder her und schließt alle SD-Dateien.

### Geführter Beifahrer-Kurventest

Firmware 1.5.28 stellt unter `/curve-test` einen eigenen Kurventest bereit.
Die Seite startet und beendet die Aufzeichnung selbst; vorheriges manuelles
Starten ist nicht nötig. Die Referenzen können in jeder Reihenfolge markiert
werden, so wie sie sich aus der gefahrenen Strecke ergeben:

1. 45 bis 60 Sekunden möglichst gerade Fahrt
2. vier langgezogene Kurven, möglichst je zwei links und rechts
3. vier normale oder enge Kurven, möglichst je zwei links und rechts
4. drei vollständige S-Kurven

Der Beifahrer drückt den Startknopf genau am physischen Beginn der Kurve und
den danach allein sichtbaren Endknopf am tatsächlichen Ende. Bei einer
S-Kurve umfasst ein Referenzintervall beide Hälften. Vollständig erledigte
Kurvenarten beziehungsweise Richtungen verschwinden; alle noch offenen
Referenzen bleiben auswählbar. Der Fahrer bedient die Seite nicht und soll
keine ungewöhnlichen oder fahrdynamisch riskanten Manöver ausführen.

Die Ereignisdatei enthält neben den manuellen Referenzintervallen automatisch
erkannte Kurven mit Start- und Endzeit, Radius, Drehrate,
Querbeschleunigung, Erkennungsmodus und Qualitätsflags. Die Sensor-CSV
protokolliert außerdem Schwerkraft und projizierte Drehrate für eine
unabhängige Nachrechnung. Referenzmarker werden erst beim regulären
Sammelflush gesichert, damit ein Knopfdruck keine künstliche Messpause
verursacht.

### Abnahme von Firmware 1.5.28

Die Probefahrt sollte 20 bis 30 Minuten dauern. Ein geeigneter Abschnitt
enthält zunächst eine gut erkennbare Gerade und danach bekannte weite,
normale und wechselnde Kurven. Nach der Fahrt werden geprüft:

- Anteil automatisch erkannter Kurven an den Referenzintervallen
- automatische Kurven außerhalb aller Referenzen als mögliche Fehlalarme
- Start-/Endabweichung, Richtung und S-Kurven-Gruppierung
- Radius und Querbeschleunigung auf Plausibilität
- Übereinstimmung von projiziertem Gyro-Winkel und relativem Heading-Winkel
- `SensorMissedSlots`, `GPSMissedSlots`, SD-Fehler und Datenvollständigkeit

Die Straßenqualitätsbewertung bleibt in dieser Version absichtlich
unverändert. Ihre nächste Parametrierung folgt getrennt aus denselben
Rohdaten, damit Änderungen an Kurven- und Oberflächenmodell eindeutig
bewertbar bleiben.

Der SD-Ausfalltest ist bewusst ein kontrollierter Hardwaretest. Er darf nicht
während einer Fahrt, eines OTA-Uploads oder bei angeschlossenem Fahrzeug-CAN
durchgeführt werden. GPS- und OBD-Daten, die genau während einer entfernten
Karte entstehen, können nicht nachträglich rekonstruiert werden; das
Recovery-Ereignis dokumentiert deshalb zusätzlich die Ausfalldauer.

### Testberichte

- [Erster ausführlicher Fahrzeugtest](testdata/20260728_2300/TESTBERICHT_2026-07-28.md)
- [Geführte Abnahmeläufe](testdata/20260729_1800/TESTBERICHT_2026-07-29_ABNAHME.md)
- [Zündungs-, Motorlauf- und Recovery-Abnahme](testdata/20260729_162734_92A51444/ABNAHME_AUSWERTUNG.md)
- [GPS-/OBD-Vergleichsfahrt](testdata/20260729_170946_05A4DB46/ABNAHME_AUSWERTUNG.md)

Die Berichte sind versioniert; die zugehörigen Roh-CSV-Dateien bleiben wegen
Größe und Messdatencharakter bewusst außerhalb des Repositorys.

### Geführter Fahrzeug-Test

Version 1.5.13 besitzt einen rein beobachtenden Testlauf. Er verändert weder
den OBD-Zustand noch startet er selbst eine Aufzeichnung:

```text
test begin
start
test status
stop
test end
```

Der Abschlussbericht bewertet neue SD-Fehler, verworfene Datensätze,
BNO055-I²C-Aussetzer, CAN-Bus-Off und Empfangspufferüberläufe als `FAIL`.
Vorübergehende CAN-Warnungen, Sendefehler, ein I²C-Aussetzer des OLED
oder eine aktuell fehlende ECU-Antwort ergeben `WARN`. Ohne durchgeführten
SD-Schreibtest bleibt das Ergebnis ebenfalls `WARN`. `PASS` bedeutet, dass
alle erforderlichen Module stabil blieben und der SD-Schreibtest erfolgreich
war.

Die MCP2515-Zeile zeigt zusätzlich:

- `TEC`: Transmit Error Counter
- `REC`: Receive Error Counter
- `EFLG`: dekodierte Warn-, Fehlerpassiv-, Bus-Off- und Pufferüberlaufbits
- aktuellen Betriebsmodus sowie die während des Tests beobachteten
  TEC-/REC-Spitzen und EFLG-Bits

Die ausführlichen Hardware-, Integrations- und Belastungstests werden nicht
bei jedem Start ausgeführt. Sie können bei Bedarf über den seriellen Monitor
gestartet werden:

Die Integration-Test-Suite enthält Mehrmodul-, Korrelations-, Recovery-,
Belastungs- und Speichertests. Für sie existiert derzeit keine belastbare
automatisierte Coverage-Zahl.

### Hosttests ohne Hardware

Die Auswertelogik der Kurvenerkennung (`src/curve_detector.{h,cpp}`) und der
Fahrbahnbewertung (`src/road_metrics.{h,cpp}`) ist frei von Arduino-, Sensor-
und Zeitabhängigkeiten. Sie wird auf dem Entwicklungsrechner getestet:

```bash
pio test -e native
```

37 Tests, Laufzeit unter zwei Sekunden, ohne Firmware-Flash zu belegen.
Geprüft werden unter anderem scharfe und langgezogene Kurven,
S-Kurvengruppierung, Gyro-Vorrang und Kurs-Rückfall, Schwellwertgrenzen,
Stoßzählung und Sperrzeiten. Dazu kommen Wiedergaben realer Fahrten aus
`testdata/`: Der belegte Standlauf muss null Ereignisse erzeugen, die
Überlandfahrt ist auf feste Kennzahlen festgeschrieben.

Ändert sich eine dieser Kennzahlen, ist das kein Testfehler, sondern eine
Verhaltensänderung — prüfen, begründen und den Festwert bewusst nachziehen.

### Test-Kommandos (Serial Monitor)
```bash
test          # Zeigt alle verfügbaren Test-Kommandos
hardware      # Führt Hardware-Test-Suite aus
integration   # Vollständige Integration-Tests (5-10 Min)
stress        # Stress-Test unter Volllast
recovery      # Hardware Failure & Recovery Tests
quick         # Schnelle Integration-Tests (1 Min)
memory        # Memory-Leak Detection (2 Min)
diag          # System-Diagnose mit allen Metriken
```

## 🗺️ Kartenauswertung

`tools/export_geojson.py` wandelt eine Messsitzung in GeoJSON zum Hineinziehen
in [geojson.io](https://geojson.io):

```bash
python3 tools/export_geojson.py testdata/20260730_095603_A41A2450
python3 tools/export_geojson.py testdata/20260730_095603_A41A2450 --modus geschwindigkeit
```

Die Strecke wird nach Straßenqualität eingefärbt — oder wahlweise nach der
Abweichung zwischen GPS- und OBD-Geschwindigkeit. Dazu kommen Schlaglöcher als
Marker nach Schwere, Kurven als tatsächlich gefahrener Bogen, die
Referenzintervalle des Beifahrer-Kurventests und ein Sitzungskopf mit allen
Kennzahlen.

Damit wird sichtbar, **wo** die Fahrbahn schlecht war und welche markierte
Kurve die Firmware erkannt hat — beides steht in den CSV-Dateien nur als Zahl.

## 📊 Datenformat & Ausgabe

### CSV-Datenlogging
Das System erstellt automatisch strukturierte CSV-Dateien:

Jede Messfahrt besitzt eine gemeinsame Sitzungskennung im Dateinamen und
erhält getrennte Sensor-, Straßen-, GPS-, Ereignis- und bei Bedarf CAN-Dateien.
Beim Beenden entsteht zusätzlich `road_summary_<Sitzung>.csv` mit Dauer,
Strecke, Ereigniszählern und Durchschnittsqualität.

**sensor_data.csv**
```csv
UTC,UptimeMs,Heading,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp,CalSystem,CalGyro,CalAccel,CalMag
2026-07-27T12:34:56Z,1234567,123.5,-2.1,0.8,0.123,-0.056,9.801,0.001,0.002,-0.001,24.5,3,3,3,3
```

`Heading` verwendet im NDOF-Modus die Sensorfusion einschließlich
Magnetometer. Alle vier Kalibrierwerte entscheiden über eine vollständig
speicherbare Kalibrierung.

**can_log.csv**  
```csv
UTC,UptimeMs,CAN_ID,Extended,RTR,DLC,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7
2026-07-27T12:34:56Z,1234567,1A0,0,0,8,12,34,56,78,9A,BC,DE,F0
```

**correlated.csv** (Sensor + CAN)
```csv
UTC,UptimeMs,Type,Heading,Pitch,Roll,AccelMag,Temp,CAN_ID,DLC,D0,D1,D2,D3,D4,D5,D6,D7
```

### Road Quality Scoring

Die aktuelle Bewertung (0–100 Punkte) beschreibt die **Oberflächenqualität**.
Sie verwendet ein rollendes Ein-Sekunden-Fenster aus RMS-Vibration,
Maximalstoß und Anzahl einzelner Stöße. Bei vorhandenem GPS wird vorsichtig
auf 30 km/h Referenzgeschwindigkeit normiert; im Stillstand bleibt der letzte
Fahrwert erhalten. Kurven und Schlaglöcher werden getrennt als einzelne
Ereignisse protokolliert.

## 🛡️ Buffer-Sicherheit

### Implementierte Schutzmaßnahmen
```cpp
// SafeStringFormatter - Overflow-sichere Formatierung
char buffer[256];
SafeStringFormatter::safePrintf(buffer, sizeof(buffer), 
    "Sensor: %.2f°, %.2f m/s²", heading, acceleration);

// SafeRingBuffer - Template-basierte Pufferung  
SafeRingBuffer<float, 100> sensorBuffer;
sensorBuffer.push(newValue);

// SafeMemoryPool - Fragmentierungsfreie Allokation
void* ptr = globalMemoryPool.allocate();
```

### Memory-Management
- **Static Buffers:** 7-8 KB RAM-Verbrauch (< 2% des ESP32-S3)
- **Heap-Protection:** Memory-Pool verhindert Fragmentierung
- **Overflow-Detection:** Multi-Level-Überwachung mit Statistiken
- **Auto-Recovery:** Intelligente Buffer-Verwaltung bei Fehlern

## 🔍 Erweiterte Konfiguration

### GPS-Konfiguration
```cpp
// GPS-Manager Einstellungen
GPSData gpsData = gpsManager.getCurrentData();
bool hasFixWithAccuracy = gpsManager.hasValidFix() && gpsData.hdop < 2.0;

// Test-Funktionen
gpsManager.testCommunication();  // Kommunikationstest
gpsManager.printDiagnostics();   // Detaillierte NMEA-Diagnose
```

### CAN-/OBD-Konfiguration
```cpp
// Fest für den vorhandenen Aufbau:
#define CAN_BAUDRATE                 500000
#define CAN_CLOCK_16MHZ            16000000
#define CAN_OBD_REQUEST_INTERVAL_MS      500
#define CAN_OBD_VALUE_MAX_AGE_MS        5000
#define CAN_TX_TIMEOUT_MS                 25
```

Gesendet werden ausschließlich funktionale OBD-Anfragen auf `0x7DF` mit
Service `01`. Die feste Positivliste enthält die vier Unterstützungsblöcke
`00/20/40/60` sowie Drehzahl (`0x0C`), Geschwindigkeit (`0x0D`),
Luftmassenstrom (`0x10`), Drosselstellung (`0x11`), Außentemperatur (`0x46`),
Öltemperatur (`0x5C`) und Kraftstoffrate (`0x5E`). In der Live-Phase werden
nur zuvor bestätigte PIDs abgefragt. Antworten werden ausschließlich von
`0x7E8` bis `0x7EF` verarbeitet; der MCP2515-Hardwarefilter verwirft andere
Identifier. Die Webseite zeigt Anfragen, Antworten, Sendefehler und nur
frische Werte.
### SD-Logger Konfiguration
```cpp
// Logging-Intervalle anpassen
LogConfig config = sdLogger.getConfig();
config.sensorLogInterval = 100;     // 10Hz Sensor-Daten
config.roadLogInterval = 1000;      // 1Hz Straßenqualität  
config.flushInterval = 5000;        // 5s Buffer-Flush
sdLogger.setConfig(config);
```

## 🚨 Troubleshooting

### Häufige Probleme

**Problem: BNO055 nicht gefunden**
```
Lösung:
1. I2C-Verkabelung prüfen (SDA=8, SCL=9)
2. I²C-Pull-ups und Leitungen gemäß `HARDWARE.md` prüfen
3. ADR-Verbindung prüfen: im vorhandenen Aufbau 3,3 V, daher Adresse 0x29
4. Serial Monitor: I2C-Scanner-Ausgabe prüfen
```

**Problem: SD-Karte nicht erkannt**
```
Lösung:
1. FAT32-Formatierung (nicht exFAT/NTFS)
2. PZSMOCN-Modul am beschrifteten 3.3V-Eingang versorgen
3. Pin-Konfiguration: CS=4, MOSI=5, MISO=6, SCK=7
4. Karte, Modulstecker, Masse und Lötstellen mechanisch prüfen
```

**Problem: GPS kein Fix**
```
Lösung:
1. Freie Sicht zum Himmel (15-30 Sek warten)
2. UART-Verkabelung: TX GPS -> RX ESP32 (Pin 16)  
3. Baudrate 9600 (Standard für NMEA)
4. Cold Start: bis zu 2 Minuten normal
```

**Problem: CAN-Bus Fehler**
```
Lösung:
1. Fahrzeug abstellen, Zündung einschalten und gemeinsame Masse prüfen
2. VCC=3,3 V, VCC1=5 V und P1/Jumper offen kontrollieren
3. OBD Pin 6=CAN-H und Pin 14=CAN-L kontrollieren
4. Auf der Webseite Anfragen, Antworten und Sendefehler vergleichen
5. Bei Sendefehlern MCP2515-Verkabelung und 16-MHz-Quarz prüfen
```

### Debug-Features
```cpp
// Detaillierte Diagnostik aktivieren
#define DEBUG_ENABLED 1

// Buffer-Statistiken anzeigen
printBufferStats();

// Hardware-Test-Suite erneut ausführen
testBNO055(); 
testSDWithSafePins();
```

## 📈 Performance-Charakteristika

### Real-Time Verhalten
- **Sensor-Reading:** Ziel 10 Hz; zuletzt im Fahrzeug effektiv etwa 8,5 Hz
- **GPS-Snapshots:** 5 Hz; neue NMEA-Fixes werden getrennt gekennzeichnet
- **CAN-Processing:** 100Hz Check-Rate ✓ Hochperformant
- **SD-Logging:** Gepuffert mit Auto-Flush ✓ Effizient

### Ressourcen-Verbrauch
```
Flash:  1.232.962 Byte / 1.310.720 Byte App-Partition (94,1%)
SRAM:   60.956 Byte / 327.680 Byte                    (18,6%)
CPU:    ESP32-S3 mit 240MHz
```

Die Web-OTA-Datei passt damit in die konfigurierte OTA-Partition. Für spätere
größere Funktionen sollte die verbleibende Flash-Reserve berücksichtigt werden.

## 🛣️ Praktische Anwendung

### Optimale Testszenarien
- **Bergstraßen** mit vielen Serpentinen (Black Forest, Alpen)
- **Landstraßen** mit wechselndem Kurvenradius
- **Rennstrecken** für Performance-Analyse
- **Stadtverkehr** für Vibrations-Analyse

### Datenauswertung
```python
# Python-Beispiel für CSV-Auswertung
import pandas as pd
import matplotlib.pyplot as plt

# Sensor-Daten laden
df = pd.read_csv('sensor_data.csv')

# Straßenqualitäts-Plot
plt.plot(df['timestamp'], df['road_quality'])
plt.title('Straßenqualität über Zeit')
plt.show()
```

## 🔄 System-Updates

### Firmware-Updates
```bash
# Projektstand aktualisieren
git pull --ff-only

# Neue Version bauen; die Versionsnummer stammt aus hardware_config.h
pio run
# Ergebnis: roadtest_<Version>.bin im Projektverzeichnis und Buildordner

# Über USB deployen
pio run --target upload

# Backup der aktuellen Konfiguration
cp src/hardware_config.h hardware_config.backup
```

### Konfiguration anpassen
- **Pin-Zuordnungen:** verbindlich in `src/hardware_config.cpp`; nur zusammen
  mit der realen Verdrahtung ändern
- **Sensor-Parameter:** `src/*_manager.h` 
- **Logging-Einstellungen:** `SDLogger::LogConfig`

## 🤝 Beitragen

### Code-Qualitäts-Standards
- Buffer-Overflow-Schutz für alle String-Operationen
- Umfassende Error-Handling für Hardware-Ausfälle
- Konsistente API-Design mit `begin()`, `isReady()`, `getCurrentData()`
- Vollständige Dokumentation mit Beispielen

### Test-Erweiterungen  
Priorität für zusätzliche Tests:
1. **Error-Recovery-Tests** (SD-Ausfall, I2C-Bus-Hang)
2. **Integration-Stress-Tests** (alle Module unter Last)
3. **Grenzwert-Tests** (extreme Beschleunigung, CAN-Überflutung)

Der priorisierte Backlog für gleichmäßigere Sensorabtastung und geringere
Buslast steht in [OPTIMIERUNGEN.md](OPTIMIERUNGEN.md).

## 📈 Version History

### v1.5.32 - OLED-Treiber entfernt
- ✅ Displaytreiber und beide Adafruit-Bibliotheken raus; 19.592 Byte Flash
  frei, 94,1 statt 95,6 Prozent
- ✅ Display bleibt verbaut und wird als zweiter I²C-Teilnehmer weiter
  angepingt; die Buszustandsdiagnose bleibt vollständig erhalten
- ✅ `requiredHardwareReady()` jetzt im seriellen `diag` sichtbar

### v1.5.31 - Größenprüfung der Logdateien
- ✅ Jede Logdatei wird am Sitzungsende gegen die tatsächliche Größe auf der
  Karte geprüft; Abweichungen landen in `road_integritaet_<Sitzung>.csv`
- ✅ Anlass: In einer Fahrt fehlten 68 % der GPS-Spur, ohne dass ein einziger
  Fehlerzähler ansprach
- ⚠️ Erkennt den Verlust, verhindert ihn nicht

### v1.5.30 - Aufgeteilter SD-Flush und nachgetragene Zusammenfassung
- ✅ Flush je Aufruf höchstens eine Datei statt aller neun am Stück; ein
  voller Umlauf dauert unverändert fünf Sekunden
- ✅ Abgebrochene Sitzung bekommt ihre Zusammenfassung nachgetragen, sobald
  die Karte wieder beschreibbar ist
- ✅ Statuszeilen wieder in gleichmäßigem Fünf-Sekunden-Abstand
- ⚠️ Wirksamkeit am Gerät noch nicht bestätigt

### v1.5.29 - Scharfe Kurvenintervalle und Abschluss beim Messende
- ✅ Ruhefenster bewertet die Netto-Kursänderung statt einzelner Stichproben;
  Gierratenrauschen hält eine Kurve nicht mehr offen
- ✅ Zeitabdeckung der Ereignisse steigt im Wiedergabestand von 67 auf
  90 Prozent, Ereignisse unter 60 Prozent Abdeckung fallen von 47 auf 7
- ✅ Eine beim Stoppen laufende Kurve wird als `SESSION_END` abgeschlossen
  statt verworfen
- ✅ Kurvenerkennung wird beim Messstart zurückgestellt
- ✅ Mindestfahrweg von 10 m und Mindest-Querbeschleunigung von 0,4 m/s² je
  Ereignis; entfernt Fehlalarme aus GPS-Drift im Stand und beim Rangieren,
  ohne eine der 55 markierten Referenzkurven zu verlieren
- ✅ Wiedergabe aller fünf Referenzfahrten als Hosttest mit festgeschriebener
  Trefferzahl gegen 55 von Hand markierte Referenzkurven
- ✅ Am Fahrzeug bestätigt (vier Fahrten am 01.08.2026, 21 Minuten, 9,1 km):
  Zeitabdeckung 97 Prozent im Median, nur 1 von 66 Ereignissen unter
  60 Prozent, keine Ausreißer über die Höchstdauer, kein Ereignis im Stand
- ✅ Wiedergabe gibt die Firmwareläufe wieder: 65 von 66 Ereignissen
  zugeordnet, drei von vier Fahrten ereignisgenau bei 0,0 bis 0,1 Grad
- ⚠️ Abschlussgrund `SESSION_END` am Gerät noch nicht beobachtet

### v1.5.28 - Einbaulagenunabhängige Drehrate und Hosttests
- ✅ Fahrzeug-Drehrate aus der Projektion des Gyroskopvektors auf die
  Schwerkraftrichtung; keine feste Sensorachse als Hochachse vorausgesetzt
- ✅ Strukturierte Kurvenereignisse mit Beginn, Ende, Richtung, Radius,
  Querbeschleunigung, S-Kurven-Gruppierung und Qualitätsflags
- ✅ Geführte Beifahrerseite `/curve-test` für Referenzintervalle
- ✅ `KurvenProKm` wird aus tatsächlichen Kurven und geprüfter GPS-Strecke gefüllt
- ✅ Auswertelogik von Kurven und Fahrbahn hardwarefrei ausgelagert und mit
  37 Hosttests abgedeckt (`pio test -e native`, ohne Firmware-Flash)
- ⚠️ Wirksamkeit am Gerät noch nicht bestätigt

### v1.5.27 - Langkurven und SD-Messungswiederanlauf
- ✅ Zweistufige Kurvenerkennung: unmittelbarer Start bei hoher Drehrate und
  kumulativer Start über Netto-Kursänderung und gefahrenen Weg
- ✅ Zwei Sekunden Abschlussruhe sowie Richtungswechseltrennung für S-Kurven
- ✅ Abtastlücken über 500 ms erzeugen keine scheinbar präzisen Kurven
- ✅ Gepufferte Sensorzeilen bleiben bei einem vorübergehenden SD-Fehler im
  RAM und werden in einer eigenen Recovery-Datei der Ursprungssitzung gesichert
- ✅ Nach erfolgreichem Wiedereinbinden startet automatisch eine neue,
  per `SD_RECOVERY_CONTINUATION` verknüpfte Sitzung
- ✅ Nicht sauber abgeschlossene Sitzungen werden im NVS erkannt; nach einem
  Neustart beginnt eine Messung weiterhin nur auf ausdrücklichen Start
- ✅ Der serielle SD-Hotplug-Test prüft nun eine laufende Messung,
  Pufferrettung und automatische Folgesitzung gemeinsam
- ⚠️ Langkurven- und SD-Wiederanlaufverhalten noch am Gerät zu bestätigen

### v1.5.26 - Vollständige Aufzeichnung und gültige Fahrbahnereignisse
- ✅ Sensor- und GPS-Logger besitzen kein zweites Zeitgatter mehr; die
  phasenerhaltende Hauptschleife ist alleiniger Taktgeber
- ✅ Sammelflush zusätzlich als Gesamtvorgang gemessen
  (`FlushLastMs`/`FlushMaxMs`/`FlushTotalMs`/`FlushCycles`/`FlushStalls`)
- ✅ Straßenqualität und Fahrbahnereignisse an nachgewiesene
  Fahrzeuggeschwindigkeit gebunden; im Stillstand entsteht kein Messwert
- ✅ Ereignisschwere enthält Stoßbeschleunigung beziehungsweise Kurvenwinkel
- ✅ Keine starre 180°-Grenze, die reale Kreisverkehre oder Wendemanöver
  verwirft
- ⚠️ Vollständigkeit der gespeicherten Zeilen noch am Gerät zu bestätigen

### v1.5.25 - Stabiler Recovery-Meilenstein
- ✅ Ein erkannter ECU-Ausfall bleibt auch nach erfolgreicher Wiederverbindung
  als abgeschlossener Abnahmeschritt erhalten
- ✅ Keine Rückkehr zu „ECU-Ausfall abwarten“ nach erneutem Einschalten
- ✅ Ausfallbewertung beginnt exakt beim Motor-Aus-Marker
- ✅ Wiederanlauf mit beiden Zündungs- und Motorstartschritten im Test
  `20260730_101000_5901D247` bestätigt

### v1.5.24 - Kritische Datenqualitätskorrekturen
- ✅ `NewFix` zählt GNSS-Zeitepochen statt mehrfacher NMEA-Positionscommits
- ✅ Phasentreue 10-Hz-/5-Hz-Zeitplanung mit Zählern für verlorene Slots
- ✅ 8-KiB-Sensorpuffer und kontinuierliches Leeren des GPS-UART-Puffers
- ✅ Vollständige Laufzeit- und SD-Zeitdiagnose mit Summe und Messanzahl
- ✅ Automatische OBD-Bestätigung des zweiten Motorstarts

### v1.5.23 - Kurzer Recovery-Kontrolltest
- ✅ Nur noch der offene ECU-Ausfall-/Wiederanlauf-Test unter `/acceptance`
- ✅ Kleine Statusabfrage statt vollständiger Seitenaktualisierung
- ✅ ECU-Verlust erst nach drei tatsächlichen Anfragefehlern
- ✅ Laufzeit-, Web- und SD-Pausen in den Sitzungsmetadaten
- ✅ Alte `/test`-Webseite aus Navigation und Routing entfernt

### v1.5.22 - Geführte GPS-/OBD-Datenabnahme
- ✅ Große, eindeutige Smartphone-Knöpfe nur für den aktuellen Schritt
- ✅ Erledigte Aktionsknöpfe verschwinden dauerhaft aus dem laufenden Test
- ✅ Serverseitige Mindestzeiten und OBD-Stillstandsprüfung vor Fahrmarkern
- ✅ Zwei Fahrphasen und drei Stillstandsabschnitte in einer Sitzung
- ✅ GPS-, OBD- und Streckenwerte direkt auf der Testseite

### v1.5.21 - GPS-Stillstands- und Feldqualitätsfilter
- ✅ Position nur mit frischem Fix, mindestens fünf Satelliten und HDOP ≤ 3,5
- ✅ Feldweise Alters- und Plausibilitätsgrenzen für Geschwindigkeit, Höhe und Kurs
- ✅ OBD-Geschwindigkeit hat bei der Streckenbildung im Fahrzeug Vorrang
- ✅ GPS-Drift im bestätigten Stillstand wird nicht mehr als Strecke summiert
- ✅ Ablehnungsgründe werden als kombinierbare Bitmaske protokolliert

### v1.5.20 - Zweistufige Porsche-Abnahme
- ✅ Zündung-Ein und Motorstart werden getrennt markiert und ausgewertet
- ✅ Motorlauf wird erst durch frische OBD-Drehzahl ab 300 U/min bestätigt
- ✅ ECU- und Motorlauf-Recovery besitzen getrennte PASS/WARN/FAIL-Zeiten
- ✅ Sitzungsdateien liegen gemeinsam unter `/sessions/<SessionId>/`
- ✅ Dateivorbereitung durchsucht das volle FAT-Wurzelverzeichnis nur noch einmal

### v1.5.19 - Stabile Messungs-Recovery
- ✅ SD-Schreibfehler können nicht mehr als erfolgreiche Datensätze erscheinen
- ✅ GPS-Sitzungszähler bleiben über UART-Neustarts hinweg monoton
- ✅ Begrenzte MCP2515-Recovery mit eigenem Sitzungszähler
- ✅ Empfang wird vor OBD-Timeouts ausgewertet
- ✅ Abnahmemarker werden erst nach bestätigtem SD-Schreiben übernommen
- ✅ OTA ist während laufender oder vorbereiteter Messungen gesperrt

### v1.5.18 - ECU-Wiedererkennung und geführte Abnahme
- ✅ Später Motorstart wird mit begrenztem Backoff ohne Firmware-Neustart erkannt
- ✅ Sichere Rückfall-PIDs `0x0C`, `0x0D`, `0x11` und `0x00`
- ✅ ECU-Ausfall nach fünf Sekunden und automatische Wiederaufnahme
- ✅ Geführte Abnahmeseite unter `/acceptance`
- ✅ Einzelzähler für MCP2515-RX0-/RX1-Überläufe
- ✅ GPS-UART-Ringpuffer auf 2.048 Byte vergrößert

### v1.5.17 - Mobile Fahrzeugtest-Steuerung
- ✅ Große Schaltflächen für Discovery-Start, Status, Marker und Abschluss
- ✅ Live-Anzeige von Phase, GPS, HDOP, OBD-Sitzung und SD-Fehlern
- ✅ Bedienung verwendet ausschließlich die vorhandenen sicheren Discovery-Funktionen

### v1.5.16 - Störungsfreier Start der Qualitätsaufzeichnung
- ✅ CAN-, OBD-, Trace- und Korrelationsdateien vor Sitzungsbeginn geöffnet
- ✅ Keine verzögerte Erstöffnung mehr während der GPS-Aufzeichnung
- ✅ GPS-Sitzungszähler beginnen erst nach abgeschlossener Dateivorbereitung

### v1.5.15 - Messbare GPS-/OBD-Datenqualität
- ✅ OBD-Sitzungszähler getrennt von Boot-Gesamtzählern
- ✅ Transaktions-CSV für Sendeversuche, Antworten, Timeouts und Latenz
- ✅ GPS-Feldalter, Gültigkeit, Fixsequenz und UART-Überlaufzähler
- ✅ Sitzungsmetadaten mit Firmware-, Schema- und Diagnosewerten
- ✅ Fixverluste werden als ungültige Qualitäts-Snapshots aufgezeichnet
- ✅ Geführter Web-Kalibrierassistent ohne manuellen Sensor-Neustart
- ✅ Versionierter Firmwaredateiname sowie Versionsanzeige auf OLED und OTA-Seite

### v1.5.14 - Fahrzeugdaten-Erkennung
- ✅ 60 Sekunden echter Listen-Only-Mitschnitt ohne Sendungen
- ✅ Standard-PID-Erkennung über `00`, `20`, `40` und `60`
- ✅ Synchronisierte CAN-, OBD-, GPS-, BNO055- und Ereignisdaten
- ✅ Markierungen über `discover mark <Text>` für die spätere Auswertung
- ✅ Nur bestätigte, standardisierte Service-01-PIDs in der Messphase

### v1.5.13 - Geführter Fahrzeug-Test
- ✅ `test begin`, `test status` und `test end`
- ✅ PASS/WARN/FAIL anhand der während des Tests neu aufgetretenen Fehler
- ✅ MCP2515-Diagnose mit Modus, TEC, REC und dekodiertem EFLG
- ✅ Spitzenwerte und Fehlerbits werden während des Testlaufs festgehalten
- ✅ CAN-Hardwarestatus zusätzlich auf der Webseite

### v1.5.12 - Optionales OLED und Testdiagnose
- ✅ OLED blockiert weder Bereitschaft noch Aufzeichnung
- ✅ Automatische Erkennung beim Start und Wiederverbindung bleiben aktiv
- ✅ Serielle Aussetzerzähler für BNO055 und OLED
- ✅ ECU-Verbindungsstatus, Antwortalter und SD-Fehler im Teststatus
- ✅ Webseite unterscheidet CAN-Adapter und ECU-Verbindung
- ✅ Serielle Befehle `obd off`/`obd on` für Tests ohne Fahrzeug

### v1.5.11 - Begrenzte OBD-Liveabfrage
- ✅ ISO 15765-4 CAN mit 11-Bit-ID und 500 kbit/s für den Porsche 991.1
- ✅ Nur lesender Service 01 für Drehzahl, Geschwindigkeit und Drosselklappe
- ✅ Höchstens zwei Anfragen pro Sekunde und 25-ms-Sende-Timeout
- ✅ OBD-Werte und Anfrage-/Antwortstatistik auf der Statusseite
- ✅ Keine Funktionen zum Fehlerlöschen, Codieren oder Ansteuern

### Unveröffentlicht - Sichere BNO055-Kommunikation
- ✅ BNO055-Erkennung über die Chip-ID statt leerer I²C-Schreibzugriffe
- ✅ Fest verdrahtete Adresse `0x29`, eine Chip-ID-Prüfung pro Statuszyklus
- ✅ Messdaten werden beim ersten unplausiblen Status sofort gesperrt
- ✅ Initialisierung endet bei ausbleibender Chip-ID nach 1,5 Sekunden
- ✅ NDOF-Kalibrierung berücksichtigt System, Gyro, Beschleunigung und Magnetometer
- ✅ Weboberfläche bleibt während des gestuften SD-Aufzeichnungsstarts erreichbar

### v1.5.10 - Stabile BNO055-Statusanzeige
- ✅ Einzelne kurzzeitige I²C-Statusfehler verändern die Anzeige nicht mehr
- ✅ Ein erfolgreicher Status setzt die Fehlerzählung sofort zurück
- ✅ Erst drei aufeinanderfolgende Fusionsfehler zeigen einen Fehler und
  lösen den automatischen BNO055-Neustart aus

### v1.5.9 - Konsistente IMUPLUS-Überwachung
- ✅ Messdaten nur bei Modus 8, Systemstatus 5 und Fehlercode 0
- ✅ Drei vollständige Fusionsfehler vor einem automatischen Sensorneustart
- ✅ Webdiagnose verwendet den überwachten Status ohne zusätzliche Modusabfragen
- ✅ Integrationstests vergleichen nur relative Richtungsänderungen
- ✅ OLED, CSV-Felder und Dokumentation eindeutig auf IMUPLUS abgestimmt

### v1.5.8 - BNO055 IMUPLUS
- ✅ Gyro und Beschleunigung ohne Magnetometer-Sensorfusion
- ✅ Keine Achterbewegung und keine Magnetometerkalibrierung erforderlich
- ✅ Kalibrierung ist abgeschlossen, sobald Gyro und Beschleunigung 3 erreichen
- ⚠️ Absolute Kompassrichtung entfällt; relative Drehungen bleiben verfügbar

### v1.5.7 - Stabiler BNO055-Sensortakt
- ✅ Interne BNO055-Taktquelle statt instabiler externer Taktumschaltung
- ✅ I²C mit robusten 50 kHz für die Lochrasterverdrahtung
- ✅ Taktquelle auf der Kalibrierseite sichtbar

### v1.5.6 - Stabile BNO055-Modusprüfung
- ✅ Kein permanentes I²C-Polling des Betriebsmodus mehr
- ✅ Drei aufeinanderfolgende Modusfehler vor einem Sensorneustart
- ✅ Vollständiger BNO055-Neustart über die Kalibrierseite

### v1.5.5 - Stabile BNO055-Fusionsdiagnose
- ✅ BNO055-Diagnose und erzwungener NDOF-Modus aus v1.5.4
- ✅ Automatischer CAN-Start deaktiviert, damit WLAN und Webseite nicht hängen

### v1.5.4 - BNO055-Fusionsdiagnose
- ✅ NDOF-Betriebsmodus wird beim Start geprüft und nötigenfalls erzwungen
- ✅ Modus, Sensorfusion, Systemstatus und Fehlercode auf der Kalibrierseite
- ✅ Sicherer BNO055-Neustart über die Webseite
- ⚠️ Automatischer CAN-Start konnte die Webseite blockieren

### v1.5.3 - Stabiler Servicezugang
- ✅ CAN im aktuellen Aufbau vollständig deaktiviert
- ✅ Feste WLAN-IP und fester Funkkanal
- ✅ WLAN-Schlafmodus deaktiviert
- ✅ Automatischer Wiederanlauf des ROADTEST-Access-Points

### v1.5.2 - WLAN zuerst
- ✅ ROADTEST-WLAN startet vor den externen Hardwareprüfungen
- ✅ Optionale CAN-Prüfung läuft erst nach dem Systemstart
- ✅ Diagnose und OTA bleiben unabhängig von GPS und CAN erreichbar

### v1.5.1 - GPS-Startprüfung
- ✅ NMEA-Datenstrom genügt für die Hardwareprüfung
- ✅ Satelliten-Fix blockiert den Startbildschirm nicht
- ✅ GPS-Zeichen, gültige Sätze und Prüfsummenfehler auf der Webseite

### v1.5.0 - Robuster Wiederanlauf
- ✅ Einheitliche, automatisch aktualisierte OLED-Startprüfung
- ✅ Kein blockierender Start bei fehlender Hardware
- ✅ Wiederverbindung für BNO055, OLED, SD, GPS und WLAN
- ✅ CAN ausdrücklich optional
- ✅ SD-Schreibfehler und verworfene Datensätze auf der Webseite
- ✅ Abgebrochene Messfahrten werden klar gekennzeichnet

### v1.4.0 - Kontrollierte Messfahrten
- ✅ Messfahrt über die Webseite starten und sicher beenden
- ✅ Eigene Dateigruppe mit gemeinsamer Sitzungskennung pro Fahrt
- ✅ Live-Zusammenfassung aus Dauer, GPS-Strecke und Ereigniszählern
- ✅ Permanente Zusammenfassungs-CSV beim Fahrtende
- ✅ Kein unbeabsichtigtes Logging direkt nach dem Einschalten

### v1.3.0 - Messfenster & Kalibrierassistent
- ✅ BNO055 nur einmal pro 100-ms-Messzyklus auslesen
- ✅ Rollendes Ein-Sekunden-Fenster für Vibration und Stöße
- ✅ Schlaglöcher und Kurven nur einmal pro Ereignis protokollieren
- ✅ Geschwindigkeitsabhängige Normierung der Straßenqualität
- ✅ Kalibrierstatus auf OLED und Webseite
- ✅ Kalibrierung geschützt über die Webseite speichern
- ✅ Eigenes WLAN und Browser-OTA

### v1.2.0 - Performance & Sicherheit
- ✅ **GPS Interrupt-Modus** zur Entkopplung von UART-Empfang und Auswertung
- ✅ **NVS-Speicher** für BNO055 Kalibrierung (persistiert über Neustarts)
- ✅ **Null-Pointer-Schutz** für alle dynamischen Allokationen
- ✅ **String-Sicherheit** verbessert (strncpy/strncat statt strcpy/strcat)
- ✅ **Performance-Optimierung** durch statische Buffer statt String-Konkatenation
- ✅ **Hardware-Konfiguration** zentralisiert in hardware_config.cpp

### v1.1.0 - Grundlegende Robustheit
- ✅ Multi-Layer Buffer-Overflow-Schutz
- ✅ Umfassende Hardware-Test-Suite
- ✅ Korrelierte Datenaufzeichnung (Sensor + CAN + GPS)
- ✅ 8 OLED-Display-Modi mit Auto-Rotation

### v1.0.0 - Initial Release
- ✅ Basis-Funktionalität mit BNO055, GPS, CAN
- ✅ SD-Karten-Logging
- ✅ Einfache Straßenqualitäts-Bewertung

## 📄 Lizenz

Für dieses Repository ist derzeit keine separate Lizenzdatei hinterlegt.

## 🙏 Danksagungen

- **Arduino/ESP32 Community** für exzellente Libraries
- **Adafruit** für robuste Sensor-Libraries (BNO055, SSD1306)
- **TinyGPS++** für zuverlässige NMEA-Verarbeitung
- **PlatformIO** für professionelle Embedded-Entwicklung

---

**🏁 Bereit für die nächste Kurventour! Viel Spaß beim Messen der perfekten Straße! 🏁**

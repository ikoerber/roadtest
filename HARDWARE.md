# Hardware und Verdrahtung

Diese Beschreibung gilt für den tatsächlich verwendeten ROADTEST-Aufbau auf
Lochraster. Maßgeblich sind die GPIO-Nummern des ESP32-S3 und die
Signalbezeichnungen auf den Modulen. Kabelfarben sind nicht verbindlich.

## Verwendete Baugruppen

| Baugruppe | Ausführung | Verwendung |
|---|---|---|
| Controller | LOLIN S3 Mini, ESP32-S3, 4 MB Flash | Firmware, WLAN und OTA |
| Bewegungssensor | Adafruit BNO055 Breakout | IMUPLUS: Gyro und Beschleunigung |
| Display | SSD1306 OLED, 128 × 64 | I²C-Statusanzeige |
| GPS | Beitian BN-880 | NMEA über UART, 9600 Baud |
| Speicher | PZSMOCN Micro-SD-Modul | SPI-Datenaufzeichnung |
| CAN, optional | Joy-IT SBC-CAN01: MCP2515 + MCP2562, 16-MHz-Quarz | Derzeit softwareseitig deaktiviert |

> ⚠️ **Ungeklärt: der tatsächliche Controllertyp.**
> Auf den Aufbaufotos vom 28. Juli 2026 trägt das Board die Pinreihe
> `BAT · 5V · GND · 3V3 · 13 · 12 · 11 · 10 · 9 · 8`, dazu BOOT- und
> RST-Taster sowie eine RGB-LED an GPIO 48. **Ein LOLIN S3 Mini hat keinen
> `BAT`-Pin.** Das in `platformio.ini` gewählte Ziel `lolin_s3_mini` passt
> also vermutlich nicht.
>
> Praktische Folge: Das Board-Ziel setzt `-DBOARD_HAS_PSRAM`. Hat der
> verbaute Controller keinen PSRAM, scheitert dessen Initialisierung bei
> jedem Start. Vor einer Korrektur von `platformio.ini` muss der reale
> Boardtyp bestimmt werden — Beschriftung auf der Platinenunterseite und
> Bootlog auswerten.

## Verbindliche GPIO-Belegung

Diese Tabelle entspricht `src/hardware_config.cpp`:

| LOLIN S3 Mini | Richtung | Modulanschluss | Funktion |
|---|---:|---|---|
| GPIO 8 | ↔ | BNO055 SDA und OLED SDA | I²C-Daten |
| GPIO 9 | → | BNO055 SCL und OLED SCL | I²C-Takt, 50 kHz |
| GPIO 16 | ← | BN-880 TX | GPS-NMEA zum ESP32 |
| GPIO 15 | → | BN-880 RX | ESP32 zum GPS |
| GPIO 4 | → | SD CS | SD Chip Select |
| GPIO 5 | → | SD MOSI/DI | SD-Daten zum Modul |
| GPIO 6 | ← | SD MISO/DO | SD-Daten zum ESP32 |
| GPIO 7 | → | SD SCLK/CLK | SD-Takt |
| GPIO 1 | → | MCP2515 CS | CAN Chip Select, optional |
| GPIO 2 | ← | MCP2515 INT | CAN Interrupt, optional |
| GPIO 3 | → | MCP2515 SCK | CAN SPI-Takt, optional |
| GPIO 13 | → | MCP2515 SI/MOSI | CAN-Daten zum Modul, optional |
| GPIO 11 | ← | MCP2515 SO/MISO | CAN-Daten zum ESP32, optional |

TX und RX werden beim GPS gekreuzt: **GPS-TX geht an ESP32-RX (GPIO 16)** und
**GPS-RX an ESP32-TX (GPIO 15)**.

## Stromversorgung

| Baugruppe | Anschluss im aktuellen Aufbau |
|---|---|
| LOLIN S3 Mini | USB-C oder vorhandener LiPo-Akkuanschluss des Boards |
| BNO055 | geregelte 3,3 V an `VIN`, GND an GND |
| OLED | geregelte 3,3 V an `VCC`, GND an GND |
| BN-880 | geregelte 3,3 V an `VCC`, GND an GND |
| PZSMOCN SD-Modul | **3,3 V** an den mit `3.3V` beschrifteten Eingang |
| MCP2515/CAN | noch nicht freigegeben; siehe CAN-Hinweis |

Alle angeschlossenen Module benötigen eine gemeinsame Masse. Die
ESP32-S3-GPIOs sind nicht 5-V-tolerant. Das PZSMOCN-Modul darf in diesem Aufbau
nicht nach generischen Anleitungen für andere SD-Module an 5 V angeschlossen
werden.

Peripherie wird aus der geregelten 3,3-V-Schiene versorgt, nicht direkt aus der
variablen LiPo-Zellenspannung.

## BNO055 und OLED am gemeinsamen I²C-Bus

```text
LOLIN S3 Mini       BNO055                 SSD1306 OLED
-------------       ------                 ------------
GPIO 8              SDA                    SDA
GPIO 9              SCL                    SCL
3,3 V               VIN                    VCC
GND                  GND                    GND
```

### Weitere BNO055-Pins

| BNO055-Pin | Aktueller Aufbau |
|---|---|
| `3Vo` | nicht anschließen; dies ist ein Ausgang des Breakout-Reglers |
| `ADR` | nicht angeschlossen, dadurch Standardadresse `0x28` |
| `RST` | nicht angeschlossen; die Firmware initialisiert den Sensor über I²C neu |
| `PS0`, `PS1` | nicht angeschlossen; beim Adafruit-Breakout standardmäßig I²C |
| `INT` | nicht verwendet |

Der BNO055 läuft im IMUPLUS-Modus 8. Das Magnetometer wird nicht verwendet,
deshalb gibt es keinen magnetischen Nordbezug und keine
Magnetometerkalibrierung.

Die erwarteten I²C-Adressen sind:

- BNO055: `0x28` bei unbeschaltetem `ADR` (Adafruit-Breakout, wie verbaut).
  Die Firmware prüft seit 1.5.10 beide dokumentierten Adressen `0x29` und
  `0x28` und übernimmt die antwortende. `diag` gibt die tatsächlich
  verwendete Adresse aus.
- OLED: `0x3C`, alternativ wird `0x3D` geprüft

Die Breakout-Module besitzen gewöhnlich bereits I²C-Pull-ups. Zusätzliche
4,7-kΩ-Pull-ups sollten nur eingebaut werden, wenn sie elektrisch erforderlich
sind; mehrere parallel geschaltete Pull-ups können den Bus unnötig stark
belasten.

## GPS BN-880

```text
LOLIN S3 Mini       BN-880
-------------       ------
GPIO 16 (RX)        TX
GPIO 15 (TX)        RX
3,3 V               VCC
GND                  GND
nicht verbunden     PPS
```

Die Firmware verarbeitet NMEA-Daten mit 9600 Baud und benötigt keinen GPS-Fix
für die Systembereitschaft. Der PPS-Ausgang des BN-880 wird von der aktuellen
Firmware nicht ausgewertet und bleibt unbeschaltet.

## PZSMOCN Micro-SD-Modul

```text
LOLIN S3 Mini       PZSMOCN SD-Modul
-------------       -----------------
GPIO 4              CS
GPIO 5              MOSI / DI
GPIO 6              MISO / DO
GPIO 7              SCLK / CLK
3,3 V               3.3V
GND                  GND
```

Die normalen Start- und Wiederanlaufversuche verwenden ausschließlich diese
fest verdrahteten GPIOs. Alternative Pin-Sätze sind für den realen Aufbau
nicht vorgesehen. Die SD-Karte sollte FAT32 formatiert und mechanisch sicher
im Sockel sowie im Steckverbinder sitzen.

## Optionales CAN-Modul: Joy-IT SBC-CAN01

Verbaut ist ein **Joy-IT SBC-CAN01** mit MCP2515-Controller und
**MCP2562**-Transceiver. Der Quarz trägt den Aufdruck **16.000**, also 16 MHz.
Dieser Wert steht als `CAN_CLOCK_16MHZ` in `hardware_config.h` und wird von
`CANReader::begin()` fest verwendet. Er wird **nicht** mehr zur Laufzeit
geraten: Eine falsche Annahme wird von `begin()` nicht erkannt, ergibt eine um
Faktor zwei falsche Bitrate, treibt den Knoten in Bus-Off und hält die
INT-Leitung dauerhaft aktiv.

Die Signalleitungen sind in der Firmware vorbereitet:

```text
Controller          SBC-CAN01
----------          ---------
GPIO 1              CS
GPIO 2              INT
GPIO 3              SCK
GPIO 13             SI / MOSI
GPIO 11             SO / MISO
GND                  GND
```

CAN ist in Version 1.5.10 mit `ENABLE_OPTIONAL_CAN = false` deaktiviert und
blockiert weder Start, WLAN noch Aufzeichnung.

### Versorgung: VCC und VCC1 sind nicht dasselbe

Der MCP2562 ist ein Zweispannungs-Transceiver. Das Modul hat deshalb zwei
getrennte Versorgungseingänge:

| Anschluss | Spannung | Versorgt |
|---|---|---|
| `VCC`  | **3,3 V** | Logik und MCP2515 — an einem ESP32 also 3,3 V |
| `VCC1` | **5 V**   | Bustreiber zum Fahrzeug |

Richtig beschaltet ist **kein Pegelwandler** in den SPI-Leitungen nötig. Genau
so wird das Modul auch am Raspberry Pi betrieben.

Liegt dagegen `VCC` auf 5 V (Arduino-Beschaltung), treibt der `SO`-Ausgang
5-V-Pegel auf GPIO 11. Der Strom fließt dann über die ESD-Schutzdiode des
ESP32-S3 in die 3,3-V-Schiene und hebt die Versorgung des gesamten Systems an.
Der absolute Grenzwert des ESP32-S3 beträgt 3,6 V.

**Vor der Inbetriebnahme `VCC` und `VCC1` einzeln gegen GND messen.**

Zu beachten: Hängt `VCC1` am `5V`-Pad des Controllerboards, ist der Bustreiber
im reinen Akkubetrieb unversorgt. CAN liefe dann nur am USB-Kabel. Für den
Fahrbetrieb ist ein 5-V-Boost aus der Zelle erforderlich.

Der 120-Ω-Abschlusswiderstand des Moduls ist über den Jumper `ON`/`OFF`
schaltbar. Am Fahrzeug bleibt er auf `OFF` — siehe unten.

Für einen späteren Fahrzeuganschluss:

- OBD-II Pin 6 ist üblicherweise CAN-H.
- OBD-II Pin 14 ist üblicherweise CAN-L.
- Eine 120-Ω-Terminierung darf nicht pauschal ergänzt werden; zunächst den
  Widerstand zwischen CAN-H und CAN-L am ausgeschalteten Fahrzeug prüfen.

## Mechanischer Aufbau

- BNO055 starr und mit bekannter Achsrichtung am Fahrzeug befestigen.
- GPS-Antenne mit möglichst freier Sicht zum Himmel montieren.
- SD-Modul und Steckverbinder zugänglich und zugentlastet montieren.
- I²C- und SPI-Leitungen kurz halten.
- Lötbrücken und kalte Lötstellen besonders an SD-Stecker und Masse prüfen.
- Kabelfarben nur als Montagehilfe verwenden; die Modulbeschriftung bleibt
  maßgeblich.

## Inbetriebnahme-Checkliste

- [ ] LOLIN S3 Mini wird über USB-C oder den vorhandenen Akkuanschluss versorgt.
- [ ] An der 3,3-V-Schiene liegen ungefähr 3,3 V gegen GND an.
- [ ] Alle Module teilen dieselbe Masse.
- [ ] I²C: GPIO 8 = SDA und GPIO 9 = SCL.
- [ ] BNO055 antwortet auf `0x28`.
- [ ] OLED antwortet auf `0x3C` oder `0x3D`.
- [ ] GPS-TX ist mit GPIO 16 verbunden und liefert NMEA-Daten.
- [ ] GPS-RX ist mit GPIO 15 verbunden; PPS bleibt frei.
- [ ] SD: CS 4, MOSI 5, MISO 6 und SCLK 7.
- [ ] PZSMOCN SD-Modul wird mit 3,3 V versorgt.
- [ ] CAN bleibt unverbunden, bis Versorgung und Pegel geprüft wurden.
- [ ] SD-Karte ist FAT32 formatiert und mechanisch sicher eingesteckt.

## Fehlersuche

### BNO055 oder OLED wird nicht erkannt

1. Gemeinsame Masse und 3,3-V-Versorgung prüfen.
2. SDA an GPIO 8 und SCL an GPIO 9 kontrollieren.
3. Im seriellen I²C-Scan nach `0x28` und `0x3C`/`0x3D` suchen.
4. Steckverbindungen und Lötstellen bewegen beziehungsweise auf Durchgang
   prüfen.
5. Erst danach zusätzliche Pull-ups in Betracht ziehen.

### SD-Karte wird nicht erkannt

1. Karte und Modulstecker vollständig einsetzen.
2. Am Modul zwischen `3.3V` und `GND` die Versorgung prüfen.
3. CS 4, MOSI 5, MISO 6 und SCLK 7 kontrollieren.
4. Karte als FAT32 formatieren und mit einer zweiten Karte gegenprüfen.
5. Bei sporadischen Fehlern zuerst Steckkontakt, Masse und Lötstellen prüfen.

### GPS liefert keine Daten

1. Prüfen, dass GPS-TX an GPIO 16 liegt; TX und RX nicht gleichnamig verbinden.
2. Gemeinsame Masse und 3,3-V-Versorgung kontrollieren.
3. Für einen Fix freie Sicht zum Himmel schaffen und nach einem Kaltstart bis
   zu einigen Minuten warten.
4. Ein fehlender PPS-Impuls verhindert den NMEA-Empfang nicht.

### CAN

CAN erst diagnostizieren, nachdem die elektrische Kompatibilität des konkreten
Moduls geklärt und die Firmwareoption bewusst aktiviert wurde.

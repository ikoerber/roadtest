# Hardware und Verdrahtung

Diese Beschreibung gilt für den tatsächlich verwendeten ROADTEST-Aufbau auf
Lochraster. Maßgeblich sind die GPIO-Nummern des ESP32-S3 und die
Signalbezeichnungen auf den Modulen. Kabelfarben sind nicht verbindlich.

## Verwendete Baugruppen

| Baugruppe | Ausführung | Verwendung |
|---|---|---|
| Controller | ESP32-S3-Board, genaue Variante per `diag` prüfen | Firmware, WLAN und OTA |
| Bewegungssensor | Adafruit BNO055 Breakout | NDOF-Sensorfusion |
| GPS | Beitian BN-880 | NMEA über UART, 9600 Baud |
| Speicher | PZSMOCN Micro-SD-Modul | SPI-Datenaufzeichnung |
| CAN/OBD | Joy-IT SBC-CAN01: MCP2515 + MCP2562, 16-MHz-Quarz | Nur lesende OBD-II-Liveabfrage |

> ⚠️ **Ungeklärt: der tatsächliche Controllertyp.**
> Auf den Aufbaufotos vom 28. Juli 2026 trägt das Board die Pinreihe
> `BAT · 5V · GND · 3V3 · 13 · 12 · 11 · 10 · 9 · 8`, dazu BOOT- und
> RST-Taster sowie eine RGB-LED an GPIO 48. **Ein LOLIN S3 Mini hat keinen
> `BAT`-Pin.** Das in `platformio.ini` gewählte Ziel `lolin_s3_mini` passt
> also vermutlich nicht.
>
> Praktische Folge: Das Board-Ziel setzt `-DBOARD_HAS_PSRAM`. Hat der
> verbaute Controller keinen PSRAM, scheitert dessen Initialisierung bei
> jedem Start.
>
> Der Boardtyp lässt sich nicht mehr ablesen, weil das Modul verlötet ist.
> Für die Wahl des richtigen Ziels ist das aber auch nicht nötig — es zählen
> Chip, Flash und PSRAM, und die meldet der Controller selbst. Das Kommando
> `diag` gibt dafür einen **Controller-Steckbrief** aus:
>
> ```text
> === Controller-Steckbrief ===
> Chip: ESP32-S3, Revision 0, 2 Kern(e), 240 MHz
> Flash: 4194304 Bytes (4 MB) bei 80000000 Hz
> Build: BOARD_HAS_PSRAM ist gesetzt
> PSRAM: 2097152 Bytes vorhanden, 2093964 Bytes frei
> eFuse-MAC: ...
> ```
>
> Meldet die Zeile `PSRAM: nicht gefunden`, obwohl `BOARD_HAS_PSRAM` gesetzt
> ist, passt das Board-Ziel nicht und gehört in `platformio.ini` auf eine
> Variante ohne PSRAM geändert. Stimmen dagegen Flashgröße und PSRAM mit den
> Werten oben überein, ist `lolin_s3_mini` trotz des abweichenden Boardlayouts
> eine tragfähige Wahl — die Pinbelegung setzt die Firmware ohnehin selbst.

## Verbindliche GPIO-Belegung

Diese Tabelle entspricht `src/hardware_config.cpp`:

| ESP32-S3 GPIO | Richtung | Modulanschluss | Funktion |
|---|---:|---|---|
| GPIO 8 | ↔ | BNO055 SDA | I²C-Daten |
| GPIO 9 | → | BNO055 SCL | I²C-Takt, 100 kHz |
| GPIO 16 | ← | BN-880 TX | GPS-NMEA zum ESP32 |
| GPIO 15 | → | BN-880 RX | ESP32 zum GPS |
| GPIO 4 | → | SD CS | SD Chip Select |
| GPIO 5 | → | SD MOSI/DI | SD-Daten zum Modul |
| GPIO 6 | ← | SD MISO/DO | SD-Daten zum ESP32 |
| GPIO 7 | → | SD SCLK/CLK | SD-Takt |
| GPIO 1 | → | MCP2515 CS | CAN Chip Select |
| GPIO 2 | ← | MCP2515 INT | CAN Interrupt |
| GPIO 3 | → | MCP2515 SCK | CAN SPI-Takt; GPIO 3 ist ein Strapping-Pin |
| GPIO 13 | → | MCP2515 SI/MOSI | CAN-Daten zum Modul |
| GPIO 11 | ← | MCP2515 SO/MISO | CAN-Daten zum ESP32 |

TX und RX werden beim GPS gekreuzt: **GPS-TX geht an ESP32-RX (GPIO 16)** und
**GPS-RX an ESP32-TX (GPIO 15)**.

## Stromversorgung

| Baugruppe | Anschluss im aktuellen Aufbau |
|---|---|
| ESP32-S3-Controller | Zündungsgeschaltete 12 V über Abwärtswandler; am Arbeitsplatz USB-C |
| BNO055 | geregelte 3,3 V an `VIN`, GND an GND |
| BN-880 | geregelte 3,3 V an `VCC`, GND an GND |
| PZSMOCN SD-Modul | **3,3 V** an den mit `3.3V` beschrifteten Eingang |
| MCP2515 `VCC` | geregelte 3,3 V für Logik und MCP2515 |
| MCP2515 `VCC1` | geregelte 5 V für den MCP2562-Bustreiber |

Alle angeschlossenen Module benötigen eine gemeinsame Masse. Die
ESP32-S3-GPIOs sind nicht 5-V-tolerant. Das PZSMOCN-Modul darf in diesem Aufbau
nicht nach generischen Anleitungen für andere SD-Module an 5 V angeschlossen
werden.

Peripherie wird aus der geregelten 3,3-V-Schiene versorgt, nicht direkt aus der
variablen LiPo-Zellenspannung.

### Einbau im Fahrzeug: zündungsgeschaltet aus dem Sicherungskasten

Für den festen Einbau kommt die Versorgung aus einer **zündungsgeschalteten
Sicherung**, CAN weiterhin aus der OBD-Buchse. Damit ist die Versorgung selbst
das Startsignal: Strom da bedeutet fahren, Strom weg bedeutet Fahrtende. Die
Firmware startet die Aufzeichnung seit 1.5.42 von allein und braucht dafür
weder Bewegungserkennung noch Zündungslogik.

Die Alternative - alles aus der OBD-Buchse mit Dauerplus und Tiefschlaf -
wäre ein Stecker weniger und vollständig rückbaubar. Sie ist verworfen, weil
ein falsch umgesetzter Tiefschlaf über Wochen Standzeit die Fahrzeugbatterie
entlädt und der Fehler erst auffällt, wenn das Auto nicht anspringt.

Verbindlich beim Einbau:

- **Eigene Absicherung im Abzweig.** Der Abgriff bekommt seine eigene
  Sicherung, unabhängig vom angezapften Stromkreis.
- **Abwärtswandler mit automotive-tauglichem Eingang.** 12 V im Fahrzeug sind
  keine sauberen 12 V; ein Wandler ohne Transientenschutz ist die häufigste
  Ausfallursache bei solchen Einbauten. Der Bedarf liegt bei geschätzt 250 bis
  400 mA auf der 5-V-Seite, ein 1-A-Wandler ist reichlich.
- **Eine einzige Masse.** Das Gerät bekommt Masse an der Karosserie beim
  Sicherungskasten, und der CAN-Transceiver bezieht sich auf genau diese
  Masse. Von der OBD-Buchse kommen dann **nur CAN-H und CAN-L**, nicht
  zusätzlich Masse - sonst entsteht eine zweite Masseschleife über den dünnen
  OBD-Massepfad.
- **Kein LiPo im Fahrzeug.** Ein Innenraum erreicht im Sommer 60 bis 70 Grad;
  der Akkuanschluss des Boards bleibt beim Einbau leer.
- **Steif befestigen.** Die Einbaulage ist dank der Projektion auf die
  Schwerkraftrichtung frei, die Steifigkeit nicht: Die Rauheitsmessung ist das
  Herz der Fahrbahnbewertung, und ein lose in einer Ablage liegendes Gerät
  misst seine eigene Resonanz mit. Fest verschraubt oder verklebt an Struktur.

Ein hartes Abschalten ist ausdrücklich zulässig. Es kostet höchstens einen
Flush-Umlauf von knapp fünf Sekunden, und zwar genau den Abschnitt, in dem das
Fahrzeug bereits steht. Abschlussdatensatz und Zusammenfassung fehlen dann;
die Auswertung rechnet ohnehin alles aus den Rohzeilen neu.

### GPS-Antenne vor dem endgültigen Einbau

Der BN-880 trägt seine Keramikantenne fest aufgelötet. Am 02.08.2026 fiel der
Empfang über drei Fahrten vollständig aus - 1.887 Zeilen mit exakt null
Satelliten bei intakter NMEA-Kette - und kam ohne Eingriff von selbst zurück.
Die Firmware ist als Ursache ausgeschlossen; der Verdacht liegt auf einem
Wackelkontakt an der Antenne oder ihrer Versorgung. Erklärt ist der Fehler
nicht, nur nicht wieder aufgetreten.

**Das ist der einzige nie erklärte Fehler des Systems, und hinter der
Verkleidung wird er teuer.** Vor dem festen Einbau gehört deshalb eine
externe aktive GNSS-Antenne mit ordentlichem Stecker und freier Sicht an das
Modul. Das behebt den Verdachtsfall und verbessert nebenbei den Empfang.

## BNO055 am I²C-Bus

Der BNO055 ist seit dem 03.08.2026 der **einzige I²C-Teilnehmer**. Das
SSD1306-OLED war bis dahin gesteckt und trug als zweiter, unabhängiger
Teilnehmer die Diagnose des Busses; es ist ausgebaut, seine Ansteuerung mit
1.5.32 und seine Überwachung mit 1.5.39 entfernt.

**Damit ist eine Fehlerunterscheidung verlorengegangen:** Fällt der BNO055
aus, lässt sich am Bus allein nicht mehr feststellen, ob Sensor, Bus oder
Versorgung die Ursache ist. Wer das für eine Fehlersuche braucht, hängt
vorübergehend ein beliebiges I²C-Gerät an — der Scanner in `main.cpp` findet
es ohne Codeänderung. Wegen des gemeinsamen Busses darf ein solcher Stecker
nur bei ausgeschaltetem System betätigt werden.

```text
ESP32-S3 GPIO       BNO055
-------------       ------
GPIO 8              SDA
GPIO 9              SCL
3,3 V               VIN
GND                 GND
```

### Weitere BNO055-Pins

| BNO055-Pin | Aktueller Aufbau |
|---|---|
| `3Vo` | nicht anschließen; dies ist ein Ausgang des Breakout-Reglers |
| `ADR` | **auf 3,3 V gelegt, dadurch Adresse `0x29`** (nicht die Adafruit-Standardadresse 0x28) |
| `RST` | extern nicht angeschlossen; der auf dem Breakout vorhandene 10-kΩ-Pull-up hält Reset inaktiv |
| `PS0`, `PS1` | nicht angeschlossen; beim Adafruit-Breakout standardmäßig I²C |
| `INT` | nicht verwendet |

Der BNO055 läuft im NDOF-Modus 12. System, Gyro, Beschleunigung und
Magnetometer müssen für eine vollständig speicherbare Kalibrierung jeweils
den Wert 3 erreichen.

Die erwarteten I²C-Adressen sind:

- BNO055: **`0x29`**. Der `ADR`-Pad des Adafruit-Breakouts liegt in diesem
  Aufbau auf 3,3 V; ohne diese Beschaltung wäre es die Standardadresse `0x28`.
  Frühere Fassungen dieser Datei nannten fälschlich 0x28.
  Die Firmware verwendet ausschließlich `0x29` und bestätigt den Sensor über
  die Chip-ID `0xA0`. `diag` gibt die feste Adresse aus.

Das Adafruit-BNO055-Breakout besitzt bereits je einen **10-kΩ-Pull-up an SDA
und SCL** sowie einen **10-kΩ-Pull-up an RST**. Die parallelen Pull-ups der
angeschlossenen Breakout-Module ergeben im fertigen Aufbau effektiv
**2,54 kΩ** gegen 3,3 V, an SDA wie an SCL (gemessen am 28. Juli 2026,
spannungsfrei). Der Wert liegt im empfohlenen Fenster von 2,2 bis 4,7 kΩ; im
gezogenen Zustand fließen rund 1,3 mA, deutlich unter den 3 mA der
I²C-Spezifikation. **Zusätzliche externe Pull-ups sind nicht erforderlich.**

Zur Flankenreserve: Die Anstiegszeit beträgt näherungsweise 2,2 · R · C. Bei
2,54 kΩ bleibt der Bus bis etwa 180 pF innerhalb der 1000 ns, die der Basistakt
von 100 kHz zulässt — reichlich Reserve. Der Bus läuft ausschließlich mit
diesem Basistakt aus `I2C_CLOCK_SPEED`; das frühere `I2C_DISPLAY_SPEED` mit
400 kHz für die Bildtransfers des SSD1306 ist mit dem Display entfallen. Damit
ist die knappste Anforderung an den Lochrasteraufbau weggefallen — 400 kHz
hätten 300 ns und damit unter etwa 54 pF verlangt.

## GPS BN-880

```text
ESP32-S3 GPIO       BN-880, 6-Pin
-------------       ---------------
GPIO 16 (RX)        TX
GPIO 15 (TX)        RX
3,3 V               VCC
GND                  GND
nicht verbunden     SCL
nicht verbunden     SDA
```

Die Firmware verarbeitet NMEA-Daten mit 9600 Baud und benötigt keinen GPS-Fix
für die Systembereitschaft. Die am fotografierten 6-poligen Anschluss
zusätzlich vorhandenen Leitungen `SCL` und `SDA` gehören zur optionalen
Kompass-Schnittstelle des Moduls. Sie werden von der aktuellen Firmware nicht
ausgewertet und bleiben unbeschaltet.

## PZSMOCN Micro-SD-Modul

```text
ESP32-S3 GPIO       PZSMOCN SD-Modul
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

## CAN-/OBD-Modul: Joy-IT SBC-CAN01

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

Version 1.5.23 verwendet das Modul mit ISO 15765-4 CAN, 11-Bit-Identifiern und
500 kbit/s. Sie sendet höchstens zweimal pro Sekunde eine funktionale,
standardisierte OBD-Anfrage auf `0x7DF`: Service 01, PID `0x0C` (Drehzahl),
`0x0D` (Geschwindigkeit) oder `0x11` (Drosselklappe). Akzeptiert werden
Antworten auf `0x7E8` bis `0x7EF`.

Das ist eine eng begrenzte Leseimplementierung. Fehlerlöschen, Codierung,
Stellgliedtests, UDS-Schreibdienste und frei wählbare CAN-Telegramme sind nicht
implementiert. Ein fehlendes ACK wird nach 25 ms abgebrochen, damit weder
Webseite noch Watchdog hängen.

Ab Version 1.5.13 liest die Diagnose zusätzlich die MCP2515-Register `TEC`,
`REC` und `EFLG`. Diese Zugriffe verändern den Buszustand nicht. Der geführte
Testlauf hält Spitzenwerte und zwischenzeitlich beobachtete Warn-, Bus-Off-
und Empfangspufferüberlaufbits bis zu `test end` fest.

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

> **Genau das war hier der Fall.** `VCC` lag ursprünglich auf 5 V. Das erklärt
> die Beobachtung, dass sich beim Aktivieren des CAN-Moduls andere Komponenten
> aufhängen. Die Beschaltung ist am 28. Juli 2026 korrigiert worden:
> `VCC` auf 3,3 V, `VCC1` auf 5 V.
>
> **Folge im Auge behalten:** GPIO 11 hat über längere Zeit Klemmstrom geführt.
> Der zulässige Klemmstrom liegt bei etwa 20 mA; ein Transceiver-Ausgang ohne
> Serienwiderstand liefert mehr. ESP32-Pins überstehen das häufig, aber nicht
> immer. Läuft die SPI-Kommunikation zum MCP2515 nach der Umverdrahtung
> zuverlässig, ist der Pin in Ordnung. Bleiben sporadische Fehler, ist ein
> beschädigter Eingang der nächste Verdacht.

**Vor der Inbetriebnahme `VCC` und `VCC1` einzeln gegen GND messen.**

Zu beachten: Hängt `VCC1` am `5V`-Pad des Controllerboards, ist der Bustreiber
im reinen Akkubetrieb unversorgt. CAN liefe dann nur am USB-Kabel. Für den
Fahrbetrieb ist ein 5-V-Boost aus der Zelle erforderlich.

### Abschlusswiderstand: am Fahrzeug entfernt

Der 120-Ω-Abschluss (R1, Aufdruck `1200`) wird über den Jumper `P1` geschaltet.

> **Ist-Zustand, Stand 28. Juli 2026: Jumper `P1` ist abgezogen; der
> 120-Ω-Widerstand des Moduls ist damit nicht zugeschaltet.**

**Am Fahrzeug muss der Jumper abgezogen werden.** Zwei Gründe:

1. Der Fahrzeugbus ist bereits an beiden Enden mit je 120 Ω abgeschlossen, was
   zusammen 60 Ω ergibt. Ein dritter Widerstand parallel führt auf rund 40 Ω.
   Der Transceiver muss für einen dominanten Pegel entsprechend mehr Strom
   treiben und erreicht die geforderte Differenzspannung unter Umständen nicht
   mehr.
2. ROADTEST hängt als Stichleitung am OBD-Stecker und ist damit kein Busende.
   Stichleitungen werden grundsätzlich nicht abgeschlossen.

Für einen späteren Fahrzeuganschluss:

- OBD-II Pin 6 ist üblicherweise CAN-H.
- OBD-II Pin 14 ist üblicherweise CAN-L.
- Eine 120-Ω-Terminierung darf nicht pauschal ergänzt werden; zunächst den
  Widerstand zwischen CAN-H und CAN-L am ausgeschalteten Fahrzeug prüfen.
  Rund 60 Ω bedeuten: Bus ist beidseitig abgeschlossen, `P1` bleibt offen.
- Die Stichleitung vom OBD-Stecker zum Modul kurz halten, etwa unter 30 cm.

## Mechanischer Aufbau

- BNO055 starr und mit bekannter Achsrichtung am Fahrzeug befestigen.
- GPS-Antenne mit möglichst freier Sicht zum Himmel montieren.
- SD-Modul und Steckverbinder zugänglich und zugentlastet montieren.
- I²C- und SPI-Leitungen kurz halten.
- Lötbrücken und kalte Lötstellen besonders an SD-Stecker und Masse prüfen.
- Kabelfarben nur als Montagehilfe verwenden; die Modulbeschriftung bleibt
  maßgeblich.

## Inbetriebnahme-Checkliste

- [ ] Am Arbeitsplatz: ESP32-S3-Controller wird über USB-C versorgt.
- [ ] Im Fahrzeug: zündungsgeschalteter Abgriff mit eigener Sicherung, Wandler
      mit Transientenschutz, Masse nur an einer Stelle, LiPo-Anschluss leer.
- [ ] An der 3,3-V-Schiene liegen ungefähr 3,3 V gegen GND an.
- [ ] Alle Module teilen dieselbe Masse.
- [ ] I²C: GPIO 8 = SDA und GPIO 9 = SCL.
- [ ] BNO055 antwortet auf `0x29` (ADR liegt auf 3,3 V).
- [ ] GPS-TX ist mit GPIO 16 verbunden und liefert NMEA-Daten.
- [ ] GPS-RX ist mit GPIO 15 verbunden; die BN-880-Leitungen SCL und SDA bleiben frei.
- [ ] SD: CS 4, MOSI 5, MISO 6 und SCLK 7.
- [ ] PZSMOCN SD-Modul wird mit 3,3 V versorgt.
- [ ] CAN-Modul: `VCC` führt 3,3 V, `VCC1` führt 5 V (einzeln gegen GND messen).
- [ ] CAN-Modul und Fahrzeug teilen eine gemeinsame Masse.
- [ ] CAN-Modul: Jumper `P1` ist vor dem Fahrzeuganschluss abgezogen.
- [ ] OBD: Pin 6 führt CAN-H, Pin 14 CAN-L; Stichleitung bleibt unter etwa 30 cm.
- [ ] SD-Karte ist FAT32 formatiert und mechanisch sicher eingesteckt.

## Fehlersuche

### BNO055 wird nicht erkannt

1. Gemeinsame Masse und 3,3-V-Versorgung prüfen.
2. SDA an GPIO 8 und SCL an GPIO 9 kontrollieren.
3. Im seriellen I²C-Scan nach `0x29` suchen.
   Antwortet stattdessen `0x28`, ist die ADR-Brücke nach 3,3 V offen.
4. Steckverbindungen und Lötstellen bewegen beziehungsweise auf Durchgang
   prüfen.
5. Spannungsfrei den Widerstand von SDA und SCL gegen 3,3 V prüfen. Erwartet
   werden im fertigen Aufbau etwa 2,54 kΩ; keine weiteren Pull-ups ergänzen.

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
4. Die BN-880-Leitungen `SCL` und `SDA` bleiben in diesem Aufbau frei; für
   NMEA werden nur `TX`, `RX`, `VCC` und `GND` benötigt.

### CAN

1. Fahrzeug abstellen und Zündung einschalten.
2. Auf der Webseite `OBD-Livedaten` prüfen. Steigende Anfragen bei null
   Antworten und steigenden Sendefehlern deuten zuerst auf Versorgung,
   gemeinsame Masse, vertauschte CAN-Leitungen oder fehlende Busverbindung.
3. Steigende Anfragen ohne Sendefehler, aber ohne Antworten bedeuten, dass der
   Frame bestätigt wurde, jedoch kein Steuergerät auf die freigegebenen
   Service-01-PIDs antwortet.
4. Während eines Firmware-Uploads CAN-H und CAN-L vom Fahrzeug trennen.

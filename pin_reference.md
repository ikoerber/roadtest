# Pin-Referenz der ROADTEST-Hardware

Diese Datei trennt bewusst zwischen den **beschrifteten Anschlüssen der
fotografierten Module** und den Pins der darauf verbauten ICs. Für Verdrahtung
und Fehlersuche sind immer zuerst die Modulbeschriftungen maßgeblich. Die
Reihenfolge eines Steckers darf nicht aus einer generischen Internetabbildung
oder aus Kabelfarben abgeleitet werden.

## ESP32-S3-Controller

Der genaue Boardtyp des verlöteten Controllers ist noch nicht abschließend
bestätigt. Die Firmware setzt die benötigten GPIOs explizit; die Zuordnung gilt
daher unabhängig vom aufgedruckten Boardnamen:

| GPIO | Richtung | Signal |
|---:|:---:|---|
| 1 | → | CAN `CS` |
| 2 | ← | CAN `INT` |
| 3 | → | CAN `SCK` |
| 4 | → | MicroSD `CS` |
| 5 | → | MicroSD `DI` / `MOSI` |
| 6 | ← | MicroSD `DO` / `MISO` |
| 7 | → | MicroSD `SCLK` |
| 8 | ↔ | I²C `SDA` für BNO055 und OLED |
| 9 | → | I²C `SCL` für BNO055 und OLED |
| 11 | ← | CAN `MISO` / `SO` |
| 13 | → | CAN `MOSI` / `SI` |
| 15 | → | UART `TX` zum BN-880-`RX` |
| 16 | ← | UART `RX` vom BN-880-`TX` |

> GPIO 3 ist beim ESP32-S3 ein Strapping-Pin. Das CAN-Modul darf während des
> Resets keinen störenden Pegel auf diese Leitung zwingen.

## Adafruit BNO055 Breakout

### Verwendete Modulanschlüsse

| Breakout-Aufdruck | ROADTEST-Verbindung |
|---|---|
| `VIN` | geregelte 3,3 V |
| `GND` | gemeinsame Masse |
| `SDA` | GPIO 8 |
| `SCL` | GPIO 9 |
| `ADR` | fest an 3,3 V, dadurch Adresse `0x29` |
| `RST` | extern nicht angeschlossen |
| `PS0`, `PS1`, `INT` | nicht angeschlossen |

Auf dem Adafruit-Breakout sind bereits je **10 kΩ Pull-up an SDA und SCL**
sowie **10 kΩ Pull-up an RST** vorhanden. Der RST-Anschluss darf deshalb
offenbleiben. Am fertig aufgebauten gemeinsamen I²C-Bus wurden effektiv
2,54 kΩ gemessen; zusätzliche externe SDA/SCL-Pull-ups sind nicht vorgesehen.

### BNO055-LGA-Pins – nur zur IC-Referenz

Die frühere Zuordnung „Pin 4 = SDA, Pin 5 = SCL“ war falsch. Laut
Bosch-Datenblatt gilt für das 28-Pin-LGA:

| IC-Pin | Name | I²C-Funktion |
|---:|---|---|
| 2 | `GND` | Versorgung |
| 3 | `VDD` | Versorgung |
| 4 | `nBOOT_LOAD_PIN` | Bootloader-Auswahl, **nicht SDA** |
| 5 | `PS1` | Protokollauswahl, **nicht SCL** |
| 11 | `nRESET` | aktiver Low-Reset |
| 14 | `INT` | Interrupt-Ausgang |
| 17 | `COM3` | I²C-Adressauswahl |
| 19 | `COM1` | `SCL` im I²C-Modus |
| 20 | `COM0` | `SDA` im I²C-Modus |
| 25 | `GNDIO` | Logikmasse |
| 28 | `VDDIO` | Logikversorgung |

## SSD1306-OLED

Das fotografierte Modul besitzt die beschrifteten Anschlüsse `GND`, `VCC`,
`SCL` und `SDA`. Die aufgedruckten Namen sind verbindlich:

| Modulanschluss | ROADTEST-Verbindung |
|---|---|
| `GND` | gemeinsame Masse |
| `VCC` | geregelte 3,3 V |
| `SCL` | GPIO 9 |
| `SDA` | GPIO 8 |

Das Display ist optional und antwortet je nach Variante auf `0x3C` oder
`0x3D`.

## Beitian BN-880

Das fotografierte BN-880 hat einen **6-poligen Anschluss**. Für ROADTEST wird
nur UART plus Versorgung benutzt:

| Modulanschluss | ROADTEST-Verbindung |
|---|---|
| `VCC` | geregelte 3,3 V |
| `GND` | gemeinsame Masse |
| `TX` | GPIO 16 / ESP32-RX |
| `RX` | GPIO 15 / ESP32-TX |
| `SCL` | nicht angeschlossen |
| `SDA` | nicht angeschlossen |

`SCL` und `SDA` gehören zur optionalen Kompass-Schnittstelle des Moduls und
werden von der aktuellen Firmware nicht verwendet. Kabelfarben oder eine
vermutete Nummerierung des Steckers sind nicht verbindlich.

## PZSMOCN MicroSD-Modul

| Modulanschluss | ROADTEST-Verbindung |
|---|---|
| `CS` | GPIO 4 |
| `DI` / `MOSI` | GPIO 5 |
| `DO` / `MISO` | GPIO 6 |
| `SCLK` / `SCK` | GPIO 7 |
| `VCC` | geregelte 3,3 V |
| `GND` | gemeinsame Masse |

Auch hier gelten die Beschriftungen des fotografierten Boards, nicht eine
generische Pinreihenfolge.

## Joy-IT SBC-CAN01

Das Modul kombiniert einen MCP2515 mit einem MCP2562 und besitzt zwei
verschiedene Versorgungseingänge:

| Modulanschluss | ROADTEST-Verbindung |
|---|---|
| `VCC` | 3,3 V für MCP2515 und SPI-Logik |
| `VCC1` | 5 V für den MCP2562-Bustreiber |
| `GND` | gemeinsame Masse |
| `CS` | GPIO 1 |
| `INT` | GPIO 2 |
| `SCK` | GPIO 3 |
| `SI` / `MOSI` | GPIO 13 |
| `SO` / `MISO` | GPIO 11 |
| `CANH` | OBD-II Pin 6 |
| `CANL` | OBD-II Pin 14 |

`P1` schaltet den 120-Ω-Abschluss des Moduls. Am Fahrzeug bleibt der Jumper
offen beziehungsweise abgezogen.

### MCP2515 – 18-Pin-SOIC/PDIP, nur zur IC-Referenz

| IC-Pin | Name | Funktion |
|---:|---|---|
| 1 | `TXCAN` | Ausgang zum CAN-Transceiver |
| 2 | `RXCAN` | Eingang vom CAN-Transceiver |
| 3 | `CLKOUT/SOF` | Taktausgang / Start-of-Frame |
| 7 | `OSC2` | Quarz |
| 8 | `OSC1` | Quarz |
| 9 | `VSS` | Masse |
| 12 | `INT` | Interrupt-Ausgang |
| 13 | `SCK` | SPI-Takt |
| 14 | `SI` | SPI-MOSI |
| 15 | `SO` | SPI-MISO |
| 16 | `CS` | SPI-Chip-Select |
| 17 | `RESET` | aktiver Low-Reset |
| 18 | `VDD` | Logikversorgung |

Die frühere Referenz „MCP2515 Pin 6 = CAN-L, Pin 7 = CAN-H“ war falsch.
**CAN-H und CAN-L liegen am MCP2562-Transceiver**, nicht direkt am MCP2515.

## Primärquellen

- Bosch Sensortec: BNO055 Data Sheet, Rev. 1.8, Tabelle 5-1
- Adafruit: BNO055 Breakout Pinouts und Schaltplan
- Microchip: MCP2515 Family Data Sheet, Tabelle 1-1

# ROADTEST – logischer Verdrahtungsplan

Dieses Dokument zeigt die Signalverbindungen des vorhandenen
Lochrasteraufbaus. Es ist kein maßstäblicher Bestückungsplan und bildet keine
einzelnen Lochraster-Lötpunkte oder Kabelfarben ab. Die ausführlichen
elektrischen Hinweise stehen in [HARDWARE.md](HARDWARE.md).

## Aktiver Aufbau

```mermaid
flowchart TB
    MCU["LOLIN S3 Mini<br/>ESP32-S3, 4 MB"]
    BNO["BNO055<br/>NDOF, Adresse 0x29"]
    OLED["SSD1306 OLED<br/>steckbar, optional<br/>Adresse 0x3C/0x3D"]
    GPS["BN-880 GPS<br/>UART, 9600 Baud"]
    SD["PZSMOCN Micro-SD<br/>SPI, 3,3 V"]
    CAN["SBC-CAN01<br/>MCP2515 + MCP2562<br/>OBD nur lesend"]
    V33["geregelte 3,3 V"]
    V5["geregelte 5 V"]
    GND["gemeinsame Masse"]

    MCU -->|"GPIO 8 · SDA"| BNO
    MCU -->|"GPIO 9 · SCL"| BNO
    MCU -->|"GPIO 8 · SDA"| OLED
    MCU -->|"GPIO 9 · SCL"| OLED

    GPS -->|"TX → GPIO 16 · RX"| MCU
    MCU -->|"GPIO 15 · TX → RX"| GPS

    MCU -->|"GPIO 4 · CS"| SD
    MCU -->|"GPIO 5 · MOSI"| SD
    SD -->|"MISO → GPIO 6"| MCU
    MCU -->|"GPIO 7 · SCLK"| SD

    MCU -->|"GPIO 1 · CS"| CAN
    CAN -->|"INT · GPIO 2"| MCU
    MCU -->|"GPIO 3 · SCK"| CAN
    MCU -->|"GPIO 13 · MOSI"| CAN
    CAN -->|"MISO · GPIO 11"| MCU

    V33 --> BNO
    V33 --> OLED
    V33 --> GPS
    V33 --> SD
    V33 -->|"VCC"| CAN
    V5 -->|"VCC1"| CAN

    GND --> MCU
    GND --> BNO
    GND --> OLED
    GND --> GPS
    GND --> SD
    GND --> CAN
```

Firmware 1.5.11 fragt über ISO 15765-4 CAN mit 11-Bit-Identifiern und
500 kbit/s ausschließlich OBD-Service-01-Livewerte ab. `P1` bleibt am
Fahrzeug offen; CAN-H geht an OBD-Pin 6, CAN-L an OBD-Pin 14.

## Pinliste

| GPIO | Signal | Ziel |
|---:|---|---|
| 1 | CAN CS | MCP2515, optional |
| 2 | CAN INT | MCP2515, optional |
| 3 | CAN SCK | MCP2515, optional |
| 4 | SD CS | PZSMOCN SD-Modul |
| 5 | SD MOSI | PZSMOCN SD-Modul |
| 6 | SD MISO | PZSMOCN SD-Modul |
| 7 | SD SCLK | PZSMOCN SD-Modul |
| 8 | I²C SDA | BNO055 und OLED |
| 9 | I²C SCL | BNO055 und OLED |
| 11 | CAN MISO | MCP2515, optional |
| 13 | CAN MOSI | MCP2515, optional |
| 15 | UART TX | BN-880 RX |
| 16 | UART RX | BN-880 TX |

## Nicht verwendete Signale

- BNO055: `3Vo`, `RST`, `PS0`, `PS1` und `INT`
- BN-880: `PPS`
- MCP2515/CAN: keine DTC-Löschung, Codierung oder Stellgliedansteuerung

## Versorgung

- LOLIN S3 Mini: USB-C oder vorhandener LiPo-Akkuanschluss
- BNO055, OLED, BN-880 und PZSMOCN SD-Modul: geregelte 3,3 V
- Alle Baugruppen: gemeinsame Masse
- Keine 5-V-Signale an ESP32-S3-GPIOs
- CAN-Modul: `VCC` 3,3 V, `VCC1` 5 V, gemeinsame Masse

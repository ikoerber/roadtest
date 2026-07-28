# Messkampagne mit dem Saleae Logic 8

Ergänzung zu [MESSPLAN.html](MESSPLAN.html). Dort stehen die Messungen mit
Multimeter und Oszilloskop; hier die, die der Logikanalysator besser kann.

## Ausstattung

- Saleae Logic 8, Geräte-ID `73FB0A8E263260E3`, Logic-2-App Version 2.4.45
- MCP-Server der App unter `http://127.0.0.1:10530`, in `~/.claude.json` für
  dieses Projekt als `logic2` registriert
- 8 Kanäle, jeder digital und/oder analog nutzbar; die Abtastrate sinkt mit
  der Zahl aktiver Kanäle

**Der MCP-Server ist nur erreichbar, solange die Logic-2-App läuft.** Startet
Claude Code, während die App zu ist, fehlen die Werkzeuge für die ganze
Sitzung — App zuerst starten, dann Claude Code.

## Grenzen der Analogkanäle

DC-gekoppelt, ±10 V, 10 Bit — rund 20 mV pro Stufe. Damit lässt sich

- **nicht** die Welligkeit von 50 mV charakterisieren,
- **wohl aber** die Frage beantworten, ob die Schiene unter 3,0 V einbricht.

Genau das ist die Ja/Nein-Entscheidung aus Messpunkt 2. Für eine genaue
Ripple-Messung bleibt das Oszilloskop nötig.

## Grundregeln für alle Aufnahmen

- Masseklemme des Logic an **GND des jeweiligen Moduls**, nicht irgendwo am
  Aufbau. Kurze Masseverbindung.
- **Beim Akkubetrieb muss das USB-Kabel des ROADTEST abgezogen sein.** Sonst
  läuft das Board am Mac, und die Messung trifft genau den Fall, der ohnehin
  funktioniert. Der Logic selbst hängt am Mac; seine Masseleitung verbindet
  die Massen, speist die 3,3-V-Schiene aber nicht.
- Die Tastspitzen sind für 3,3-V-Logik unkritisch.

---

## Kampagne 1 — Schiene und SD gleichzeitig

Beantwortet Messpunkt 2 und liefert zusätzlich den Kausalitätsnachweis:
**Fällt ein Spannungseinbruch zeitlich mit einer fehlschlagenden
SPI-Transaktion zusammen?** Das leistet weder Multimeter noch Oszilloskop
allein.

| Kanal | Art | Signal | Abgriff |
|---|---|---|---|
| 0 | analog | 3,3-V-Schiene | `3.3V`-Pin **am SD-Modul** |
| 1 | digital | SD CS | GPIO 4 |
| 2 | digital | SD CLK | GPIO 7 |
| 3 | digital | SD MOSI | GPIO 5 |
| 4 | digital | SD MISO | GPIO 6 |
| GND | — | Masse | `GND`-Pin am SD-Modul |

- Analyzer: **SPI** auf Kanal 1–4, Mode 0, MSB first
- Betriebsfall: nur Akku, WLAN aktiv, Aufzeichnung über die Weboberfläche
  gestartet, mindestens zwei Minuten
- Auswertung: Analogverlauf auf Einbrüche absuchen, an den Fundstellen die
  SPI-Dekodierung danebenlegen

Erwartung bei bestätigter These: Einbruch unter 3,0 V, unmittelbar gefolgt
von einer abbrechenden oder fehlerhaften SPI-Antwort und der Meldung
`⚠️ SD-Fehler` auf der Konsole.

---

## Kampagne 2 — I²C

Klärt zwei offene Punkte auf einmal: ob der BNO055 sauber antwortet, und wie
die 400-kHz-Displaybursts tatsächlich aussehen. Die Pull-up-Messung ergab
2,54 kΩ, was für 100 kHz reichlich Reserve lässt, für 400 kHz aber knapp ist.

| Kanal | Art | Signal | Abgriff |
|---|---|---|---|
| 0 | digital | SDA | GPIO 8 |
| 1 | digital | SCL | GPIO 9 |
| 2 | analog | SDA | derselbe Punkt, für die Flankenform |

- Analyzer: **I2C**
- Betriebsfall: normaler Betrieb, keine Aufzeichnung nötig

Worauf achten:

- Adresse `0x29` muss mit ACK beantwortet werden. NACKs oder ausbleibende
  Antworten deuten auf ein Kontaktproblem am Sensor.
- Verstümmelte Transaktionen zwischen zwei Displayaufbauten
- Der Analogkanal auf SDA zeigt die Flankenform. Sind die Anstiege bei den
  400-kHz-Bursts sichtbar verschliffen, ist `I2C_DISPLAY_SPEED` in
  `hardware_config.h` der Stellhebel — nicht `I2C_CLOCK_SPEED`.

---

## Kampagne 3 — CAN-SPI

Erst nach `ENABLE_OPTIONAL_CAN = true`. Zugleich die Funktionsprobe für
GPIO 11, der durch die frühere 5-V-Beschaltung von `VCC` über längere Zeit
Klemmstrom geführt hat.

| Kanal | Art | Signal | Abgriff |
|---|---|---|---|
| 0 | digital | CAN CS | GPIO 1 |
| 1 | digital | CAN SCK | GPIO 3 |
| 2 | digital | CAN MOSI | GPIO 13 |
| 3 | digital | CAN MISO | GPIO 11 |
| 4 | digital | CAN INT | GPIO 2 |

- Analyzer: **SPI** auf Kanal 0–3, Mode 0, MSB first

Worauf achten:

- Antwortet der MCP2515 auf `READ CANSTAT` (Instruktion `0x03`, Adresse
  `0x0E`) mit plausiblen Werten statt mit `0xFF`?
- Bleibt MISO sauber, oder zeigt der Pegel Auffälligkeiten? Dauerhaft
  verschliffene oder unvollständige Pegel wären der Hinweis auf einen
  beschädigten Eingang.
- Die INT-Leitung darf nicht dauerhaft low liegen. Passiert das, stimmt die
  Bitrate nicht und der Knoten ist in Bus-Off.

---

## Ablauf über die MCP-Werkzeuge

Verfügbar sind unter anderem `get_devices`, `start_capture`, `stop_capture`,
`wait_capture`, `add_analyzer`, `export_data_table_csv` und `save_capture`.

Sinnvolle Reihenfolge je Kampagne:

1. `get_devices` — Gerät bestätigen
2. `start_capture` mit der Kanalkonfiguration aus der jeweiligen Tabelle
3. Betriebsfall am Gerät herstellen, `wait_capture` oder `stop_capture`
4. `add_analyzer` für SPI beziehungsweise I2C
5. `export_data_table_csv` — die dekodierten Ergebnisse auswerten
6. `save_capture` als `.sal`, damit die Aufnahme später erneut betrachtet
   werden kann

`wait_capture` blockiert bis zum Ende der Aufnahme und kann Minuten dauern.

---

## Ergebnis Kampagne 2 (I²C), 28. Juli 2026

Aufnahme: 60 s, 25 MS/s, Kanal 0 = SDA, Kanal 1 = SCL, Kanal 2 = RST.
Rohdaten in `i2c_bno055_ausfall.sal`.

| Adresse | Transaktionen | davon NACK |
|---|---:|---:|
| `0x29` BNO055 | 3670 | **94** |
| `0x3C` OLED | 183 | **0** |
| `0x28` Probe | 10 | 10 (erwartet) |
| `0x3D` Probe | 8 | 8 (erwartet) |

**Der Bus ist elektrisch in Ordnung.** Das OLED quittiert über die gesamte
Aufnahme jede einzelne Transaktion. Es traten ausserdem **keine unerwarteten
Adressen** auf — Bitfehler auf SDA scheiden damit aus.

Die 94 NACKs liegen in genau **zwei Clustern**, beide mit identischer Signatur:

```
t=26,687 s bis 27,147 s   47 NACKs   Dauer 0,460 s, danach 0,552 s Stille
t=44,372 s bis 44,832 s   47 NACKs   Dauer 0,459 s, danach 0,552 s Stille
```

Rund **1 s Totalausfall, etwa alle 18 s**, deterministisch und identisch
wiederholt. Das entspricht der Bootzeit eines BNO055 nach einem Power-on-Reset
(Datenblatt: rund 650 ms). Unmittelbar vor jedem Cluster laufen die
Transaktionen normal und werden alle quittiert.

**Schlussfolgerung: Der BNO055 startet sich selbst neu.** Nicht der Bus, nicht
die Adressierung, nicht die Firmware.

Damit sind widerlegt:

- **Bitfehler durch 400-kHz-Displaytransfers.** Keine Fremdadressen, OLED
  fehlerfrei. Die Absenkung auf 100 kHz bleibt trotzdem richtig, weil die
  Flankenreserve rechnerisch zu knapp war.
- **Floatender RST-Pin.** Der 10-kΩ-Pull-up ist gesetzt, die Ausfälle bleiben.

Offen bleiben zwei Möglichkeiten:

1. **Versorgungseinbruch am Sensor.** Nächste Messung: Analogkanal auf `3Vo`
   des Breakouts, also der Ausgang des bordeigenen Reglers und die tatsächliche
   VDD des BNO055. Digitaltrigger auf SDA, damit die Aufnahme den Ausfall
   einfängt.
2. **Defekter Baustein.** Bleibt die Versorgung stabil, ist der Sensor zu
   tauschen.

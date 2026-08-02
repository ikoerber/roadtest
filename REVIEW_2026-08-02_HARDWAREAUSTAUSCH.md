# Code-Review: Wie austauschbar ist die Hardware?

| Feld | Angabe |
|---|---|
| Stand | Firmware 1.5.36, Commit `78d3e77`, 02.08.2026 |
| Anlass | Frage, ob die Struktur einen Hardwaretausch trägt - vor allem Ersatz der mechanischen SD-Karte |
| Umfang | 16.222 Zeilen in `src/`, Schwerpunkt Speicheranbindung |

## Gesamturteil

**Die Struktur trägt den Tausch, und zwar besser als erwartet.** Der
entscheidende Grund ist keine bewusste Abstraktion, sondern eine glückliche
Eigenschaft des Arduino-Frameworks: Der Typ `File`, der in
`sd_logger.h` neunmal als Member steht, **ist bereits `fs::File`** - der
generische Dateityp aller ESP32-Dateisysteme. `SD` ist lediglich eine globale
Instanz von `SDFS : public FS`.

Damit funktioniert der weitaus größte Teil des Codes unverändert mit jedem
anderen Backend. Die Naht ist klein und liegt an einer klar benennbaren
Stelle.

## Die Naht im Detail

Von 46 Aufrufen der SD-Bibliothek in `sd_logger.cpp` und `main.cpp`:

| Art | Anzahl | Betroffen vom Tausch? |
|---|---:|---|
| `open`, `mkdir`, `exists`, `remove` | 19 | **Nein.** Alle auf der Basisklasse `FS` vorhanden |
| `begin`, `end`, `cardType`, `cardSize`, `totalBytes`, `usedBytes` | 27 | **Ja.** Backendspezifisch |

Dazu kommt eine bereits vorhandene Engstelle, die viel wert ist: **Jeder
Schreibvorgang läuft durch `writeFileTimed()`, jeder Flush durch
`flushFileTimed()`** (`sd_logger.cpp:25` und `:46`). Diese beiden Funktionen
nehmen `File&` entgegen und messen die Dauer. Ein Backendwechsel muss sie
nicht anfassen - und die gesamte Laufzeitdiagnose bleibt gültig.

Ein Umbau bestünde also aus:

1. `fs::FS& dateisystem` als Member statt der globalen `SD`-Instanz,
   im Konstruktor oder in `begin()` übergeben.
2. Eine schmale Backend-Schnittstelle für die sechs
   nicht-generischen Aufrufe: `initialisieren()`, `beenden()`,
   `gesamtBytes()`, `benutzteBytes()`, `typBeschreibung()`.
3. Die rund 200 Zeilen rohe SPI-Kartenprüfung in `main.cpp:208-398`
   (CMD0-Sequenz, Frequenzabstufung) sind vollständig SD-spezifisch und
   müssten je Backend neu geschrieben oder entfallen.

**Aufwand geschätzt: ein Arbeitstag**, überwiegend mechanisch. Keine
Änderung an der Sitzungsverwaltung, der Integritätsprüfung, der Recovery
oder am Datenformat.

## Was der Speicher leisten muss

Gemessen an den Fahrten vom 02.08.2026, beide unabhängig voneinander:

```
20260802_113909_C8F1C2F3   10.888 kB in 3.507 s   ->  186,3 kB/min
20260802_125851_99D63730    5.480 kB in 1.770 s   ->  185,8 kB/min
```

Die Rate ist bemerkenswert konstant, weil sie vom Zeittakt bestimmt wird und
nicht von der Strecke. Verteilung in der langen Fahrt: `road_sensor`
4.204 kB, `road_gps` 2.440 kB, `road_obd` 1.332 kB, `road_obd_trace`
1.184 kB.

Daraus die Reichweite je Speichergröße:

| Speicher | Reichweite |
|---|---:|
| 16 MB SPI-NOR-Flash | 1,5 Stunden |
| 128 MB SPI-NOR-Flash | 11,7 Stunden |
| 1 GB eMMC | 91,8 Stunden |
| 8 GB SD-Karte (heute) | 734 Stunden |

**Das schließt SPI-NOR-Flash als Ersatz praktisch aus.** Ein W25Q128 mit
16 MB wäre nach anderthalb Stunden voll - die Fahrt vom 02.08. hätte gerade
so hineingepasst. Selbst 128 MB wären knapp, wenn mehrere Fahrten zwischen
zwei Auslesevorgängen liegen.

### Realistische Kandidaten

**eMMC über SDMMC**, aufgelötet, 4 bis 8 GB. Löst das mechanische Problem
vollständig, behält die Kapazität. Der Haken liegt nicht in der Software,
sondern in den Pins: `SD_MMC` benötigt die SDMMC-Peripherie des ESP32-S3,
also andere GPIOs als die heutigen SPI-Pins 4/5/6/7. Das berührt die
verbindliche Pinbelegung und `HARDWARE.md`.

**SD-Karte im Push-Push-Halter mit Verriegelung**, also derselbe Speicher in
besserer Fassung. Keine Softwareänderung, kein Pinwechsel. Wenn das Problem
der Kontakt ist und nicht die Karte, ist das die ehrlichste Lösung.

Vor beidem steht allerdings der ungeklärte Befund aus 1.5.31: In
`20260801_114252_B9D1628B` fehlten 68 Prozent der GPS-Spur bei null
Fehlerzählern und schlüssiger firmwareseitiger Buchführung. **Solange
unklar ist, ob die Ursache im Kartenslot, in der Karte oder in der
SD-Bibliothek liegt, kann ein Hardwaretausch das Problem ebenso gut
mitnehmen wie beheben.** Der Gegentest mit einer zweiten Karte ist die
billigere Diagnose und steht weiter aus.

## Befunde

### 1. Toter Codepfad koppelt `CANReader` unnötig an die SD-Bibliothek

`can_reader.h:5` bindet `<SD.h>` ein und `can_reader.h:124` hält einen
`File logFile`. Beides existiert allein für `CANReader::enableLogging()`
(`can_reader.cpp:709-760`, rund 50 Zeilen).

**Diese Funktion wird nirgends aufgerufen.** Die CAN-Aufzeichnung läuft
vollständig über `SDLogger::logCANMessage()`. Der Pfad ist toter Code, der
eine zweite Datei an die Speicherhardware bindet und bei einem Backendwechsel
mit angefasst werden müsste.

*Behoben:* 76 Zeilen in `can_reader.cpp` und der `<SD.h>`-Einbindung samt
`File`-Member im Header entfernt, dazu die globale `canLoggingEnabled` in
`main.cpp`, die ebenfalls nie auf `true` stand. Gewinn 644 Byte Flash und
48 Byte RAM; wichtiger ist, dass `CANReader` die Speicherhardware nicht mehr
kennt.

### 2. Der Hardwaretest prüfte den Betriebstakt nicht

`SD.begin()` steht an zwei Stellen: `main.cpp` im Hardwaretest mit
Frequenzabstufung, `sd_logger.cpp:175` im Produktivpfad mit `SD_SPI_SPEED`.

Die Abstufung selbst ist Absicht und diagnostisch wertvoll - der erste
Verdacht, sie durch die Produktivinitialisierung zu ersetzen, war falsch.
Der Defekt lag im Ablauf: Die Reihenfolge war 400 kHz, 1 MHz, 4 MHz, **und
die Schleife brach beim ersten Erfolg ab.** Trug die Karte 400 kHz, meldete
der Test "SD-Karte funktioniert!" und prüfte den Betriebstakt nie. Ein
bestandener Test belegte damit nicht, dass die Aufzeichnung läuft.

*Behoben:* `SD_SPI_SPEED` steht jetzt an erster Stelle, die übrigen Stufen
bleiben als Diagnose. Trägt nur eine langsamere, meldet der Test das
ausdrücklich als Warnung statt als Erfolg.

**Offen bleibt** ein zweiter Unterschied: Der Test verwendet die
Standardwerte der Bibliothek, der Produktivpfad dagegen den Mountpunkt
`/sd` und `SD_MAX_OPEN_FILES` = 12. Eine Karte, die mit zwölf offenen
Dateien überfordert ist, fällt im Test nicht auf. Das zu prüfen hieße, die
neun Sitzungslogs im Test nachzubilden - dafür ist der Nutzen zu gering.

### 3. `sd_logger.cpp` ist mit 2.534 Zeilen das größte Modul

Es vereint Sitzungsverwaltung, neun Dateiformate, Pufferung,
Integritätsprüfung, Recovery und Zusammenfassung. Das ist gewachsen und
funktioniert, aber es ist die Stelle, an der ein Backendwechsel am meisten
Aufmerksamkeit braucht.

*Keine Empfehlung zum Aufteilen* - der Nutzen stünde in keinem Verhältnis
zum Risiko an einem funktionierenden, am Fahrzeug bestätigten Modul. Der
Hinweis dient der Aufwandsschätzung.

## Die übrige Hardware

| Modul | Kopplung | Austauschbarkeit |
|---|---|---|
| `bno055_manager` | `Adafruit_BNO055`, `Wire`, `Preferences` im Header | Mittel. Die **Auswertung** ist bereits sauber getrennt: `curve_detector` und `road_metrics` sind hardwarefrei und hosttestbar. Ein anderer IMU müsste nur `SensorData` füllen. |
| `gps_manager` | `TinyGPS++`, `HardwareSerial` | Gut. NMEA ist ein Standard; ein anderer Empfänger spricht dasselbe Protokoll. |
| `can_reader` | Eigener `MCP2515`-Treiber im Projekt | Gut. Der Treiber liegt als `src/MCP2515.cpp` vor, `CANController` ist bereits eine Abstraktionsschicht. |

**Der wertvollste Strukturentscheid des Projekts** ist die Auslagerung der
Auswertelogik nach `curve_detector` und `road_metrics`: 651 Zeilen ohne jede
Hardware-, Arduino- oder Zeitabhängigkeit, abgesichert durch 66 Hosttests,
und identisch übersetzbar für Gerät und PC. Diese Trennung macht die Frage
nach der Hardware für die eigentliche Messaufgabe weitgehend gleichgültig.

## Empfehlung

1. ~~Befund 1 beheben~~ - **erledigt.**
2. ~~Befund 2 beheben~~ - **erledigt.**
3. **Den Kartentausch erst nach dem Gegentest entscheiden.** Ohne zu wissen,
   ob Slot, Karte oder Bibliothek den stillen Verlust verursacht, wäre ein
   eMMC-Umbau eine teure Wette. Eine zweite SD-Karte kostet wenige Euro und
   beantwortet die Frage.
4. **Falls der Tausch kommt: eMMC, nicht SPI-Flash.** 186 kB/min sind für
   NOR-Flash zu viel.

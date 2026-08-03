# Prüfbericht – Sitzung 20260730_071913_9B018397

## Prüfkopf

| Feld | Angabe |
|---|---|
| Prüfgegenstand | ROADTEST-Firmware **1.5.24**, CSV-Schema `1.5.24-quality-v7` |
| Sitzung | `20260730_071913_9B018397`, 30.07.2026, 07:19:14–07:23:40 UTC |
| Sitzungsdauer | 265,4 s |
| Prüfgrundlage | `GPS_CAN_OBD_DATENQUALITAETSPLAN.md`, `CLAUDE.md`, Prüfanweisung aus der Vorprüfung vom 30.07.2026 |
| Vergleichsläufe | `20260730_062734_543E6ED0` (FW 1.5.23), `20260729_170946_05A4DB46` (FW 1.5.22) |
| Prüfart | Auswertung der Rohdaten mit begleitender Quelltextprüfung |
| Berichtsdatum | 30.07.2026 |

## Datenbasis und Randbedingungen

Ausgewertet wurden alle zehn CSV-Dateien der Sitzung. Alle Zeitvergleiche
verwenden `UptimeMs`. Es wurde nicht interpoliert.

**Der Stillstand des Fahrzeugs ist nachgewiesen** und damit die Voraussetzung
für alle Aussagen zum Standverhalten:

- OBD-Geschwindigkeit 0 km/h in allen 256 gültigen Samples
- GPS-Rohgeschwindigkeit maximal 4,4 km/h
- `SpeedValid` in 0 von 1.138 GPS-Snapshots
- Summenstrecke 0,000 km
- Motor zeitweise laufend, OBD-Drehzahl bis 763 U/min

## Gesamturteil

**Keine Freigabe der Messkette.**

Die Zeitplanung auf Ebene der Hauptschleife ist nachweislich repariert. Der
Datenverlust ist dadurch jedoch nicht beseitigt, sondern in eine Ebene
gewandert, die die neu eingeführte Laufzeitdiagnose nicht erfasst: Es fehlen
**13,9 % der GPS-Datensätze**, gemeldet werden **0,30 %**.

Zusätzlich hat eine zentrale Kennzahl zwischen 1.5.23 und 1.5.24 ihre
Definition gewechselt, wodurch eine scheinbare Verbesserung entsteht, die
messtechnisch nicht belegt ist.

## 1. Nachweislich behoben

| Befund | Nachweis |
|---|---|
| **B1** Fixdefinition | `NewFix` = 265 in 265 s = **1,002 Hz** bei 1-Hz-Empfänger. Vorher 1,374 Hz (1.5.23) und 1,486 Hz (1.5.22). Der Epochenschlüssel aus der GNSS-Zeit arbeitet korrekt. |
| **B3** Zeitplanung der Hauptschleife | `GPSMissedSlots` = **4 von 1.325 = 0,30 %** gegen Sollwert ≤ 2 %. `SensorMissedSlots` = 84 von 2.651 = 3,17 %. |
| **B2** Messbarkeit | Mittelwerte erstmals verfügbar: Loop 2,432 ms über 108.985 Messungen, SD 3,605 ms über 3.352, Web 0,996 ms über 108.984. |
| Risiko aus der Vorprüfung | Die Bindung des Epochenschlüssels an `gps.time.isValid()` führte im Standlauf nicht zum Stillstand des Fixzählers: 265 Fixes in 265 s. Entwarnung gilt für den Standfall; der Fahrfall bleibt ungeprüft. |

## 2. Befund B12 – doppelte, gegeneinander laufende Taktung

**Beweislage: belegt (Daten und Quelltext).**

Metadaten und CSV widersprechen sich. Der Widerspruch ist der eigentliche
Datenverlust:

| Kanal | Metadaten (abgetastet) | CSV (geschrieben) | Differenz |
|---|---:|---:|---:|
| GPS | 1.321 | 1.138 | **183 = 13,9 %** |
| Sensor | 2.567 | 2.526 | 41 = 1,6 % |

Gegenprobe der Plausibilität: GPS 1.321 + 4 verfehlt = 1.325 Slots gegenüber
1.327 rechnerisch erwarteten bei 5 Hz; Sensor 2.567 + 84 = 2.651 gegenüber
2.654 erwarteten bei 10 Hz. Die Abtastung auf Schleifenebene ist damit
vollständig, der Verlust entsteht erst beim Schreiben.

### Ursache im Quelltext

Zwei voneinander unabhängige Taktgeber mit identischer Periode:

- `src/main.cpp:1392` taktet den GPS-Snapshot mit
  `GPS_SNAPSHOT_INTERVAL_MS = 200` und meldet verfehlte Slots an
  `recordGPSSchedule`.
- `src/sd_logger.cpp:1331-1334` besitzt ein **eigenes** Intervallgatter mit
  `config.gpsLogInterval = 200` (gesetzt in `src/main.cpp:707`) und liefert bei
  Ablehnung **`return true`** zurück. Der Aufrufer wertet die verworfene Zeile
  als erfolgreich geschrieben.

Der Sensorpfad ist identisch aufgebaut: `src/sd_logger.cpp:912-915` mit
`config.sensorLogInterval = 100` aus `src/main.cpp:705`.

Folge: Der Verlust ist für die Diagnose unsichtbar. `GPSMissedSlots`
unterschätzt ihn um Faktor 46.

Damit erklären sich auch die trotz reparierter Schleife weiterhin zu niedrigen
Raten in den Dateien: **GPS 4,301 Hz, Sensor 9,534 Hz.**

### Mechanismus

**Beweislage: Indiz, nicht belegt.** Der Ankerzeitpunkt des Loggers stammt aus
einem `millis()`-Aufruf innerhalb der Logfunktion, die Taktung dagegen vom
Aufrufer. Schwankt die Verarbeitungsdauer zwischen beiden Zeitpunkten, fällt
ein Aufruf unter die Intervallschwelle und wird verworfen. Die größere
Streuung beim Verarbeiten der NMEA-Bursts erklärt, warum GPS mit 13,9 %
deutlich stärker betroffen ist als der Sensorpfad mit 1,6 %.

### Maßnahme und Abnahmekriterium

Das Intervallgatter in `logGPSData` und `logSensorData` entfernen; der
Aufrufer taktet bereits und zählt bereits verfehlte Slots.

> **Abnahmekriterium B12:** `GPSSnapshots` aus den Sitzungsmetadaten und die
> Zeilenzahl der GPS-CSV dürfen um höchstens 1 voneinander abweichen. Analog
> für `SensorSamples` und die Sensor-CSV.

## 3. Messtechnischer Vorbehalt – gewechselte Definition von `SDMaxMs`

**Beweislage: belegt (Quelltext).**

`SDMaxMs` ist von 193 ms (1.5.23) auf 67 ms (1.5.24) gefallen. **Das ist kein
belegter Fortschritt.**

- In 1.5.23 maß `flush()` den gesamten Block über neun Dateien als eine
  Operation.
- In 1.5.24 misst `flushFileTimed()` jede Datei einzeln; eine Messung des
  Gesamtdurchlaufs existiert nicht mehr (`src/sd_logger.cpp`, `SDLogger::flush()`).

Die beiden Werte sind nicht vergleichbar. Der belastbare Indikator ist
unverändert geblieben:

| Kennzahl | 1.5.23 | 1.5.24 |
|---|---:|---:|
| `LoopMaxMs` | 222 ms | **225 ms** |
| `SDMaxMs` | 193 ms (Block) | 67 ms (Einzeldatei) |
| `WebMaxMs` | 39 ms | 38 ms |

Die größte Blockade besteht unverändert fort und wird von keinem SD- oder
Web-Einzelwert erklärt. Der Sammelflush ist aus der Messung herausgefallen.

> **Abnahmekriterium:** Zusätzlich zur Einzelmessung die Gesamtdauer eines
> `flush()`-Durchlaufs erfassen. Die Summe der Einzelmessungen eines
> Durchlaufs muss zur Gesamtmessung passen, und `LoopMaxMs` muss durch eine
> der beiden Größen erklärbar sein.

## 4. Befund B4 – Falschereignisse im nachgewiesenen Stillstand

**Beweislage: belegt (Daten und Quelltext). Gegenüber der Vorprüfung
verschärft.**

Aus `road_event_20260730_071913_9B018397.csv` bei belegtem Stillstand:

```
233916,SCHLAGLOCH,MITTEL (3.9 m/s²),51.063587,12.080621,0
239719,SCHLAGLOCH,MITTEL (3.5 m/s²),51.063560,12.080624,0
255817,SCHLAGLOCH,GROSS (5.4 m/s²),51.063568,12.080563,0
256417,KURVE,Winkel: 625°,51.063568,12.080560,0
```

Zusätzlich 106 von 265 Straßenqualitätswerten unter 99,0, Minimum **60,1**.

Ursache unverändert `src/main.cpp:1336-1339`: Übergeben wird
`speed_valid ? speed_kmh : -1.0f`. Da `speed_valid` unterhalb von 6 km/h
definitionsgemäß falsch ist, erreicht im Stillstand immer `-1` die
Auswertung, und der Stillstandszweig in `calculateRoadQuality()` bleibt
unerreichbar. `detectPothole()` erhält weiterhin überhaupt keine
Geschwindigkeit.

> **Abnahmekriterium B4:** In einem Standlauf von 5 Minuten mit laufendem
> Motor: null Zeilen mit Straßenqualität unter 99,0 und null Schlagloch- oder
> Kurvenereignisse.

## 5. Neue Befunde aus dieser Sitzung

### B13 – Kurvenwinkel physikalisch unmöglich

**Beweislage: belegt.** Das protokollierte Ereignis `Winkel: 625°` entspricht
1,7 vollen Umdrehungen und trat bei stehendem Fahrzeug auf. Die
Kursintegration in `detectCurve`/`getCurveAngle` besitzt keine
Plausibilitätsgrenze; im Stillstand driftet der BNO055-Kurs frei.

> **Abnahmekriterium B13:** Kein Kurvenereignis mit einem Betrag über 180°;
> keine Kurvenerkennung unterhalb einer definierten Mindestgeschwindigkeit.

### B14 – Spalte `Schwere` konstant null

**Beweislage: belegt.** In `src/sd_logger.cpp:1529` endet die Ereigniszeile
auf das Literal `",0\n"`. Die tatsächliche Schwere steht ausschließlich im
Freitextfeld der Beschreibung. Das maschinenlesbare Zahlenfeld ist damit
unbrauchbar und erzwingt Textparsing bei jeder Auswertung.

> **Abnahmekriterium B14:** `Schwere` trägt den numerischen Wert; eine
> Auswertung der Ereignisdatei ist ohne Parsen des Beschreibungstextes
> möglich.

## 6. Weiterhin offene Befunde aus der Vorprüfung

| Befund | Stand in dieser Sitzung | Beweislage |
|---|---|---|
| **B5** Lat/Lon als `float` | `src/gps_manager.h:21-22` unverändert; Quantisierung 0,334 m | belegt |
| **B7** Feldalter | **verschlechtert**: Anteil \|SpeedAge − LocationAge\| > 100 ms von 29,3 % auf **77,0 %**, p95 218 ms, max 465 ms | Zahl belegt, Ursache offen |
| **B6** UART-Reserve | `RXBufferOverflows` = 0, größte GPS-Lücke jedoch nur 809 ms — die auslösende Bedingung trat nicht auf | **ungeprüft** |
| **B8** Statusseite `/` | `src/web_manager.cpp:414` unverändert `refresh 3` bei 4,5 KB; `WebMaxMs` 38 ms nur für `/acceptance` aussagekräftig | unverändert |
| **B9** Schwelle 6 km/h | `src/hardware_config.h:103` unverändert | unverändert |

Die Verschlechterung bei **B7** ist vor der nächsten Freigabe zu erklären. Ein
unerklärter Rückschritt in einer Qualitätskennzahl darf nicht mitlaufen.

## 7. OBD und CAN

- 335 Anfragen, 284 Antworten
- 27 Sendefehler = 8,1 %, Zielwert unter 1 %
- 33 Timeouts, 2 nicht zugeordnete Antworten
- RX0-Überläufe **0** (vorher 9), RX1-Überläufe 0, keine Controller-Recovery
- keine verlorenen Trace-Ereignisse

Die Sendefehlerquote überschreitet den Zielwert. Sie ist jedoch **nicht
bewertbar**, weil die Metadaten Sendefehler nicht nach Zündungsphase trennen
und Fehler bei ausgeschalteter Zündung erwartungsgemäß auftreten.

> **Abnahmekriterium:** Sendefehler und Timeouts getrennt nach ECU-Zustand
> ausweisen, damit die Quote gegen den Zielwert geprüft werden kann.

## 8. Ergebnis der acht Prüfpunkte

| Nr. | Prüfpunkt | Ergebnis | Urteil |
|---|---|---|---|
| 1 | SD-Blockade erkennbar | SDMax 67 ms, LoopMax 225 ms unerklärt | nicht erfüllt |
| 2 | Mittelwerte verfügbar | SD 3,605 ms × 3.352 = 12,1 s = 4,6 % der Sitzung | erfüllt |
| 3 | Verfehlte Slots ≤ 2 % | GPS 0,30 %, Sensor 3,17 % | teilweise |
| 4 | Raten ≥ 4,90 / ≥ 9,80 Hz | 4,301 / 9,534 Hz | nicht erfüllt, Ursache B12 |
| 5 | `NewFix`-Rate | 1,002 Hz | erfüllt |
| 6 | Strecke im Stand 0,000 km | 0,000 km | erfüllt |
| 7 | `RXBufferOverflows` mit Lücke > 2 s | 0, größte Lücke 809 ms | ungeprüft |
| 8 | B4 dokumentiert | 106 Werte < 99,0; 3 Schlaglöcher; Kurve 625° | bestätigt, verschärft |

## 9. Korrekturen an den eigenen Prüfvorgaben

Zwei Kriterien der Vorprüfung waren fehlerhaft spezifiziert und werden hiermit
berichtigt:

1. **Prüfpunkt 5 war für einen Standlauf ungeeignet.** Gefordert waren null
   `NewFix`-Datensätze mit identischer Position. Bei stehendem Fahrzeug sind
   identische Folgepositionen physikalisch zu erwarten, zusätzlich verstärkt
   durch die Quantisierung aus B5. Die gemessenen 27 von 265 sind **kein
   Mangel**. Gültig ist allein das Ratenkriterium. Das Duplikatkriterium gilt
   ausschließlich für Fahrtabschnitte.
2. **Prüfpunkt 1 griff zu kurz.** Gefordert war `SDStalls > 0` als Nachweis
   funktionierender Instrumentierung. `SDStalls = 0` ist hier korrekt, weil
   die größte Einzeloperation 67 ms dauerte und die Schwelle bei 250 ms liegt.
   Lückenhaft ist nicht der Zähler, sondern die entfallene Aggregatmessung des
   Sammelflush nach Abschnitt 3.

## 10. Freigabeempfehlung und nächste Schritte

**Keine Freigabe der Messkette.**

Vorrangige Maßnahmen in dieser Reihenfolge:

1. **B12** – Doppelgatter in `logGPSData` und `logSensorData` entfernen.
2. **Abschnitt 3** – Aggregatmessung des `flush()`-Durchlaufs
   wiederherstellen, damit `LoopMaxMs` zuordenbar wird.
3. **B4 und B13** – Geschwindigkeit im Loop aus frischer OBD-Geschwindigkeit,
   dann gültiger GPS-Geschwindigkeit, dann `-1` auflösen und diesen Wert an
   Straßenqualität, Schlagloch- und Kurvenerkennung übergeben.
4. **B7** – Ursache der Verschlechterung klären.

Danach genügt erneut ein Standlauf von zehn Minuten. Die entscheidende
Einzelkontrolle ist der Abgleich von `GPSSnapshots` aus den Metadaten gegen
die Zeilenzahl der GPS-CSV.

## 11. Prüfumfang und Grenzen

Geprüft wurden `gps_manager`, `sd_logger`, `runtime_diagnostics`,
`vehicle_data_discovery`, die Hauptschleife in `main.cpp`, die Routen und
Seitengrößen in `web_manager` sowie `hardware_config.h`.

Nicht geprüft wurden `integration_tests.cpp`, `MCP2515.cpp` und
`CANController.cpp` im Detail, der OTA-Uploadpfad, `oled_manager` sowie
`bno055_manager` über die in B4 und B13 genannten Funktionen hinaus.

Die Prüfung erfolgte am Rohdatenbestand und am Quelltext. Ein eigener
Hardwaretest wurde nicht durchgeführt. Der Fahrbetrieb ist mit 1.5.24 nicht
gemessen; alle Aussagen gelten für den Standlauf.

## Anlage – Reproduktion der Kennzahlen

Alle Zahlen dieses Berichts sind aus den CSV-Dateien der Sitzung mit
`python3` und der Standardbibliothek nachrechenbar.

Abgetastete gegen geschriebene Datensätze (Befund B12):

```python
import csv, os
meta = list(csv.DictReader(open([f for f in os.listdir('.')
                                 if f.startswith('road_meta')][0])))[-1]
gps  = list(csv.DictReader(open([f for f in os.listdir('.')
                                 if f.startswith('road_gps')][0])))
print(meta['GPSSnapshots'], len(gps), meta['GPSMissedSlots'])
```

Abtastraten und Lücken:

```python
t = [int(r['UptimeMs']) for r in gps]
d = sorted(t[i+1] - t[i] for i in range(len(t)-1))
print(len(gps) / ((t[-1]-t[0]) / 1000), d[len(d)//2], d[-1])
```

Nachweis des Stillstands:

```python
obd = list(csv.DictReader(open([f for f in os.listdir('.')
                                if f.startswith('road_obd') and 'trace' not in f][0])))
print({int(r['SpeedKmh']) for r in obd if r['SpeedValid'] == '1'})
```

Falschereignisse (Befunde B4, B13, B14):

```bash
grep -E "SCHLAGLOCH|KURVE" road_event_20260730_071913_9B018397.csv
```

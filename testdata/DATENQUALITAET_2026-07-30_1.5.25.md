# Datenqualitätsbericht – vier Sitzungen vom 30.07.2026 (Firmware 1.5.25)

## Prüfkopf

| Feld | Angabe |
|---|---|
| Prüfgegenstand | Messdaten aus Firmware **1.5.25**, CSV-Schema `1.5.25-quality-v8` |
| Sitzungen | `20260730_084245_54526FA7`, `20260730_093525_F94C8878`, `20260730_095603_A41A2450`, `20260730_101000_5901D247` |
| Prüfgrundlage | `GPS_CAN_OBD_DATENQUALITAETSPLAN.md`, Prüfanweisung aus `20260730_071913_9B018397/PRUEFBERICHT_2026-07-30.md` |
| Prüfart | reine Datenauswertung, begleitende Quelltextprüfung; **keine Codeänderung** |
| Berichtsdatum | 30.07.2026 |

**Wichtiger Hinweis zum Firmwarestand:** Alle vier Sitzungen wurden mit 1.5.25
aufgezeichnet. Die zweistufige Kurvenerkennung aus 1.5.27 ist in diesen Daten
**nicht** enthalten. Die Aussagen zur Kurvenerkennung beschreiben den Stand vor
dieser Erweiterung und liefern die Vergleichsbasis, an der 1.5.27 zu messen
sein wird.

## Übersicht der Sitzungen

| Sitzung | Dauer | Art | Strecke | Schlaglöcher | Kurven | Ø-Qualität |
|---|---:|---|---:|---:|---:|---:|
| `084245_54526FA7` | 330 s | schnelle Fahrt, Median 167 km/h, max 218 | 14,208 km | 23 | 3 | 71,7 |
| `093525_F94C8878` | 915 s | gemischt, Median 53 km/h, max 172 | — | 20 | 31 | — |
| `095603_A41A2450` | 667 s | Landstraße, Median 57 km/h, max 130 | 10,384 km | 6 | 24 | 84,3 |
| `101000_5901D247` | 286 s | Standlauf, OBD 0 km/h durchgehend | 0,000 km | 0 | 0 | leer |

## Gesamturteil

**Die Datenqualität der Aufzeichnung ist gut.** GPS, SD-Integrität und
Zeitbasis erfüllen die Vorgaben des Datenqualitätsplans in allen vier
Sitzungen. Die vier Korrekturen aus 1.5.25 sind durch den Standlauf
`101000_5901D247` bestätigt.

Offen bleiben drei Punkte, die die **Auswertbarkeit** betreffen, nicht die
Rohdaten: ein systematisch untertreibender Diagnosezähler, nicht
sitzungsbezogene SD-Zähler und zwei tote Spalten. Dazu kommt ein Befund, der
die Kurvenauswertung direkt betrifft: Der BNO055 lief in allen Fahrten mit
unkalibriertem Magnetometer.

## 1. Verifikation der 1.5.25-Korrekturen

Der Standlauf `101000_5901D247` ist ein gültiger Nachweis: OBD meldete in
allen 399 gültigen Samples 0 km/h, GPS-Rohgeschwindigkeit blieb unter der
Freigabeschwelle, `SpeedValid` war in 0 von 1.409 Snapshots gesetzt.

| Prüfpunkt | Erwartung | Ergebnis | Urteil |
|---|---|---|---|
| B4 Straßenqualität im Stand | 0 Zeilen | **0 Zeilen** | bestanden |
| B4/B13 Fahrbahnereignisse im Stand | 0 Ereignisse | **0 Schlaglöcher, 0 Kurven** | bestanden |
| Mittelwertfeld ohne Messwerte | leer, nicht `0.0` | `...,286,0.000,0,0,0,` – **leer** | bestanden |
| Strecke im Stand | 0,000 km | 0,000 km | bestanden |
| B12 abgetastet gegen geschrieben | Differenz ≤ 1 | siehe Abschnitt 2 | siehe dort |

Zusätzlich in allen vier Sitzungen: keine Kurvenwinkel über 180°
(Maximum 124°). Der in `20260730_071913_9B018397` protokollierte Wert von
625° tritt nicht mehr auf.

## 2. B12 – behoben, aber der Zähler untertreibt

### Der Datenverlust ist erklärt

Die zuvor unsichtbare Lücke zwischen abgetasteten und geschriebenen
Datensätzen ist geschlossen. Alle fehlenden Datensätze sind jetzt als Lücken
in der Zeitreihe sichtbar:

| Sitzung | Kanal | geschrieben | erwartet | fehlend | davon als Lücke sichtbar | ungeklärt |
|---|---|---:|---:|---:|---:|---:|
| `084245` | GPS | 1.559 | 1.652 | 93 | 95 | −2 |
| `084245` | Sensor | 3.120 | 3.304 | 184 | 186 | −2 |
| `093525` | GPS | 4.380 | 4.600 | 220 | 222 | −2 |
| `093525` | Sensor | 8.618 | 9.149 | 531 | 534 | −3 |
| `095603` | GPS | 3.194 | 3.335 | 141 | 142 | −1 |
| `095603` | Sensor | 6.313 | 6.671 | 358 | 356 | +2 |
| `101000` | GPS | 1.409 | 1.429 | 20 | 21 | −1 |
| `101000` | Sensor | 2.714 | 2.860 | 146 | 145 | +1 |

Die Restabweichung von ±3 ist Rundung an den Rändern. **Es gibt keinen
stillen Verlust mehr.** Der Median des Schreibabstands liegt exakt bei 200
beziehungsweise 100 ms; die Zeitplanphase ist stabil.

### Befund N1 – `MissedSlots` zählt nur ab doppeltem Intervall

**Beweislage: belegt (Daten und Quelltext).**

| Sitzung | Kanal | tatsächliche Lücken | Lücken ≥ 2× Intervall | gemeldete `MissedSlots` |
|---|---|---:|---:|---:|
| `084245` | GPS | 95 | 19 | **17** |
| `093525` | GPS | 222 | 13 | **12** |
| `095603` | GPS | 142 | 3 | **2** |
| `101000` | GPS | 21 | 0 | **0** |
| `084245` | Sensor | 186 | 139 | **141** |
| `093525` | Sensor | 534 | 342 | **344** |
| `095603` | Sensor | 356 | 248 | **248** |
| `101000` | Sensor | 145 | 74 | **74** |

Die gemeldeten Werte stimmen mit der Spalte „≥ 2× Intervall“ überein, nicht
mit den tatsächlichen Lücken. Ursache ist die ganzzahlige Division in
`RuntimeDiagnostics::recordGPSSchedule` und `recordSensorSchedule`
(`elapsedMs / intervalMs - 1`): Eine GPS-Lücke von 350 ms ergibt
`350 / 200 = 1`, also null verfehlte Slots, obwohl ein Slot fehlt.

Weil beim GPS die Lücken typischerweise zwischen 300 und 400 ms liegen,
untertreibt der Zähler dort um **Faktor 5,6** (95 gegen 17). Beim Sensor mit
100-ms-Raster fallen die Lücken meist über 200 ms und werden erfasst; dort
beträgt die Untertreibung nur rund 25 %.

Wirkung: Ein Abnahmekriterium „`GPSMissedSlots` ≤ 2 %“ ist mit dem aktuellen
Zähler nicht aussagekräftig. `095603` meldet zwei verfehlte GPS-Slots, real
sind es 142 von 3.335 (4,3 %).

## 3. Befund N2 – der Sammelflush ist der dominante Blocker

**Beweislage: belegt.** Die in 1.5.25 ergänzte Aggregatmessung liefert genau
die zuvor fehlende Zuordnung:

| Sitzung | Flush-Zyklen | Mittel | Maximum | `LoopMaxMs` | `SDMaxMs` (Einzeldatei) | `WebMaxMs` |
|---|---:|---:|---:|---:|---:|---:|
| `084245` | 66 | **161,9 ms** | 220 ms | 236 ms | 55 ms | 10 ms |
| `093525` | 182 | **163,9 ms** | 288 ms | 295 ms | 141 ms | 14 ms |
| `095603` | 134 | **162,3 ms** | 210 ms | 245 ms | 69 ms | 10 ms |
| `101000` | 59 | **123,8 ms** | 226 ms | 238 ms | 48 ms | 38 ms |

Damit ist die offene Frage aus dem Vorbericht beantwortet: Die
Hauptschleifenpause von 236 bis 295 ms wird vom Sammelflush erklärt, nicht
vom Webhandler (Mittel 1,0 ms) und nicht von einer einzelnen Dateioperation.

Der Flush läuft alle fünf Sekunden und blockiert dabei im Mittel 162 ms über
neun Dateien, also rund 18 ms je Datei. Bezogen auf die Sitzungsdauer sind
das 3,2 % Blockadezeit — genau in der Größenordnung, die die fehlenden
GPS- und Sensordatensätze aus Abschnitt 2 erklärt.

`FlushStalls` steht dennoch fast überall auf 0, weil die Schwelle bei 250 ms
liegt und der typische Flush 162 ms dauert. Der Zähler ist damit für diesen
Zweck zu grob eingestellt.

## 4. Befund N3 – SD-Zähler sind nicht sitzungsbezogen

**Beweislage: belegt.** In `095603_A41A2450` und `101000_5901D247` steht
bereits im `START`-Datensatz bei Uptime 0:

```
SDErrors = 1, SDDropped = 48
```

Die Werte ändern sich über die gesamte Sitzung nicht mehr. Es handelt sich um
Übernahmen aus einer früheren Sitzung, nicht um Fehler dieser Läufe.

Das widerspricht dem Grundsatz des Datenqualitätsplans: „Sitzungsstatistiken
dürfen keine Werte aus der Zeit vor dem Sitzungsstart enthalten.“ Eine
Auswertung, die `SDErrors > 0` als Fehlerkriterium verwendet, bewertet diese
beiden Sitzungen fälschlich als fehlerhaft. Anders als GPS-, OBD- und
CAN-Zähler besitzen die SD-Zähler keinen Sitzungsstartwert.

## 5. Befund N4 – `093525_F94C8878` wurde nicht sauber geschlossen

**Beweislage: belegt.**

- Letzter Metadatensatz ist `STATUS`, nicht `END`
- Keine Zusammenfassungsdatei vorhanden
- Die letzte GPS-Zeile liegt bei 919.892 ms, also **4.964 ms nach** dem
  letzten Metadatensatz bei 914.928 ms

Die Sitzung wurde während des Betriebs unterbrochen, ohne dass die Dateien
geschlossen wurden. Die Daten selbst sind konsistent: keine rückwärts
laufenden Zeitstempel, keine beschädigten Zeilen. Für eine Abnahme ist die
Sitzung dennoch nicht verwendbar, weil Abschlusszähler und Strecke fehlen.

## 6. Befund N5 – zwei tote Spalten

**Beweislage: belegt (Daten und Quelltext).**

- `KurvenProKm` in `road_road_*.csv` ist in **allen 1.843 Zeilen aller vier
  Sitzungen konstant 0,0**. Der Aufrufer übergibt in `main.cpp` ein Literal
  `0` als dritten Parameter von `logRoadQuality`.
- `Schwere` in `road_event_*.csv` ist weiterhin konstant `0`; der Zahlenwert
  steht nur im Freitextfeld `Beschreibung` (`sd_logger.cpp`, Literal `",0\n"`
  in `logEvent`). Der Befund aus dem Vorbericht besteht unverändert fort.

Beide Spalten belegen Speicherplatz und suggerieren Messwerte, wo keine sind.
Für jede Auswertung muss die Schwere aus dem Text geparst werden.

## 7. Befund N6 – BNO055 ohne Magnetometer- und Systemkalibrierung

**Beweislage: belegt.** In allen drei Fahrsitzungen:

| Sitzung | CalSystem | CalGyro | CalAccel | CalMag |
|---|---:|---:|---:|---:|
| `084245` | 0 | 3 | **1** | **0** |
| `093525` | 0 | 3 | 3 | **0–1** |
| `095603` | 0 | 3 | 3 | **0** |

Die Werte sind über die gesamte Sitzung konstant. Der Sensor läuft im
NDOF-Modus, der das Magnetometer für den absoluten Kurs benötigt.

### Was das konkret bedeutet

Ich habe den BNO-Kurs gegen den GPS-Kurs geprüft, ausschließlich bei
Geschwindigkeiten ab 50 km/h und mit **Kreisstatistik**, weil der Versatz bei
zwei der drei Sitzungen nahe ±180° liegt und lineare Statistik dort
Scheinwerte erzeugt:

| Sitzung | mittlerer Versatz | Kreisstreuung | Drift über die Sitzung |
|---|---:|---:|---:|
| `084245` | −86,7° | **5,3°** | −11,7° in 5,5 min = **−2,1 °/min** |
| `093525` | −159,1° | **7,4°** | −14,8° in 15 min = **−1,0 °/min** |
| `095603` | +168,0° | **8,5°** | −18,4° in 11 min = **−1,7 °/min** |

Daraus folgt differenziert:

- **Der absolute Kurs ist unbrauchbar.** Der Versatz zum wahren Fahrtkurs
  beträgt je nach Sitzung −87° bis +168° und ist nicht reproduzierbar.
- **Der relative Kurs ist brauchbar.** Eine Kreisstreuung von 5,3 bis 8,5°
  gegenüber dem GPS-Kurs ist für die Kurvenerkennung ausreichend.
- **Die Drift ist gering, aber nicht null.** 1,0 bis 2,1 °/min. Für schnelle
  Kurven ist das bedeutungslos. Für eine langgezogene Kurve, die über 60
  Sekunden aufsummiert wird, kommen 1 bis 2° Scheinwinkel hinzu — gegenüber
  einer Startschwelle von 8° sind das bis zu 25 %.

Die Drift ist in allen drei Sitzungen **gleichgerichtet negativ**, wirkt also
systematisch und nicht als Rauschen. Linkskurven werden dadurch geringfügig
über-, Rechtskurven unterbewertet, je nach Vorzeichenkonvention.

## 8. Was nachweislich gut ist

Diese Punkte erfüllen die Vorgaben des Datenqualitätsplans in allen vier
Sitzungen und brauchen keine Maßnahme:

| Kennzahl | Ergebnis | Zielwert |
|---|---|---|
| Gültige GPS-Position | **100,0 %** in allen vier Sitzungen | ≥ 95 % |
| Satelliten | Median 11–12, Minimum 8 | ≥ 5 |
| HDOP | Median 0,82–0,93, Maximum 1,59 | ≤ 3,5 |
| Alter der Position | p95 ≤ 869 ms, Maximum 910 ms | ≤ 1.500 ms |
| GPS-UART-Überläufe | **0** in allen Sitzungen | 0 |
| NMEA-Prüfsummenfehler | **0** in allen Sitzungen | 0 |
| SD-Schreibfehler während der Sitzungen | **0** (siehe N3 zu den Startwerten) | 0 |
| Zeitstempel | monoton, keine Dubletten | monoton |
| Web-Handler | Mittel 1,0 ms, Maximum 38 ms | unkritisch |

Der GPS-Einbau ist damit über vier weitere Sitzungen bestätigt. Die
Empfangsqualität ist besser als in der Referenzfahrt vom Vortag.

## 9. Korrekturen an meinen eigenen Zwischenergebnissen

Zwei meiner Rechnungen waren zunächst falsch. Beide sind hier bereits
korrigiert, werden aber offengelegt, weil sie die Methodik betreffen:

1. **Lückenzählung mit ganzzahliger Division.** Mein erster Ansatz zählte eine
   GPS-Lücke von 350 ms als null verfehlte Slots und ergab dadurch einen
   scheinbar ungeklärten Restverlust. Mit korrekter Rundung stimmt die Bilanz
   auf ±3 Datensätze. Dieselbe Rechenweise steckt in der Firmware und ist dort
   als Befund N1 dokumentiert — der eigene Fehler hat den Firmwarefehler
   sichtbar gemacht.
2. **Kursversatz mit linearer Statistik.** Für `095603` ergab sich zunächst
   eine Streuung von 100°, was auf einen unbrauchbaren Kurs hingedeutet hätte.
   Der Wert war ein Artefakt des Sprungs bei ±180°. Mit Kreisstatistik beträgt
   die Streuung 8,5°, der Kurs ist relativ brauchbar. Die ursprüngliche
   Bewertung wäre eine Fehlbeurteilung der Kurvenerkennung gewesen.

---

# Teil 2 – Analyseideen

## A. Kurvenerkennung

### A.1 Referenzverfahren aus der GPS-Spur

Für `095603_A41A2450` habe ich aus der GPS-Spur eine unabhängige
Kurvenreferenz gerechnet: Kurs aus aufeinanderfolgenden Fixpaaren mit
mindestens 3 m Abstand, Kursänderung je Segment, zusammenhängende Läufe
gleicher Drehrichtung ab 10° Gesamtwinkel.

| Größe | Wert |
|---|---:|
| Verwendete Fixes | 667 |
| Streckensegmente ≥ 3 m | 603, zusammen 10,342 km |
| Kursänderung je Segment | Median 2,19°, p90 7,79°, Maximum 25,32° |
| Summe der absoluten Kursänderung | 2.051° |
| Geometrisch erkannte Kurven ≥ 10° | **48** |
| Von der Firmware protokollierte Kurven | **24** |
| Firmware-Ereignisse mit geometrischer Entsprechung | **22 von 24** |
| Geometrische Kurven ohne Firmware-Ereignis | **26** |

Daraus zwei belastbare Kennzahlen für 1.5.25:

- **Genauigkeit rund 92 %** – die protokollierten Kurven sind fast immer echt.
- **Vollständigkeit rund 46 %** – mehr als die Hälfte der Kurven fehlt.

Das ist genau das Verhalten, das die Erweiterung in 1.5.27 adressiert. Die
beiden Zahlen sind der Maßstab, an dem 1.5.27 zu messen ist.

*Einschränkung:* Das Referenzverfahren ist selbst eine Heuristik mit vier
Parametern (3 m, 1°, zwei Segmente Lückentoleranz, 10° Mindestwinkel). Es ist
keine Grundwahrheit. Die per OSRM auf das Straßennetz abgeglichene Spur, die
laut `hardware_config.h` für die Parametrierung von 1.5.27 verwendet wurde,
ist die bessere Referenz.

### A.2 Drehrate aus dem Gyroskop statt aus dem Kurs

Der stärkste Vorschlag aus dieser Analyse: **die Drehrate für die
Kurvenerkennung aus `GyroZ` ableiten statt aus dem fusionierten Kurs.**

Belege:

| Argument | Messwert |
|---|---|
| `CalGyro` ist in allen Sitzungen **3 von 3** | gegen `CalMag` = 0 |
| Gyro benötigt kein Magnetometer | keine Abhängigkeit von N6 |
| Skalenfaktor Drehrate(Kurs) zu `GyroZ` | Median −0,95 bis −1,14, also ±1 mit invertierter Achse |
| Auflösung `GyroZ` in der CSV | **0,062 °/s** (BNO055-LSB = 1/16 °/s) |
| Auflösung `Heading` in der CSV | 0,1°, bei 10 Hz also **1,0 °/s** abgeleitete Drehrate |

Der letzte Punkt ist entscheidend für jede nachträgliche Auswertung: Die
Schwelle für langgezogene Kurven liegt bei 0,8 °/s. Aus der Spalte `Heading`
lässt sich diese Schwelle **grundsätzlich nicht auflösen**, weil die
Quantisierung mit 1,0 °/s darüber liegt. `GyroZ` ist 16-mal feiner.

Die Firmware rechnet intern mit dem ungerundeten Kurs und ist davon nicht
betroffen — jede Auswertung aus der CSV aber schon.

Gemessene Drehratenverteilung:

| Sitzung | Median \|GyroZ\| | p90 | Maximum | Anteil > 8 °/s | Anteil > 0,8 °/s |
|---|---:|---:|---:|---:|---:|
| `084245` | 0,88 °/s | 2,38 | 8,0 | **0,0 %** | 53,5 % |
| `095603` | 0,81 °/s | 2,88 | 14,1 | **0,6 %** | 51,5 % |

Das erklärt die Untererkennung quantitativ: Die Startschwelle für schnelle
Kurven von 8 °/s wird auf realen Straßen praktisch nie erreicht. Umgekehrt
ist die Schwelle für lange Kurven mit 0,8 °/s so niedrig, dass sie die halbe
Fahrzeit überschreitet — die Trennung muss dort vollständig über Mindestwinkel
und Mindestweg erfolgen.

### A.3 Kurven über den Radius statt über den Winkel klassifizieren

Der Winkel allein beschreibt eine Kurve unzureichend: 90° mit 200 m Radius
und 90° mit 20 m Radius sind fahrdynamisch völlig verschieden. Aus den
vorhandenen Daten ist der Radius direkt berechenbar:

```
R = v / ω          v aus OBD-Geschwindigkeit, ω aus GyroZ
a_quer = v · ω     Querbeschleunigung als Gegenprobe
```

Beide Größen liegen im Datensatz vor. `a_quer` lässt sich zusätzlich gegen
`AccelY` prüfen — stimmen die beiden überein, ist die Kurve bestätigt, weichen
sie ab, deutet das auf Kursdrift oder Einbaulage hin. Das ist eine
Plausibilitätsprüfung, die ohne zusätzliche Hardware auskommt.

Vorschlag für eine Kurvenklassifikation, die für kurvenreiche
Motorradstrecken aussagekräftiger ist als der reine Winkel:

| Klasse | Radius | typische Querbeschleunigung |
|---|---|---|
| weit | > 200 m | < 2 m/s² |
| mittel | 80–200 m | 2–4 m/s² |
| eng | 30–80 m | 4–6 m/s² |
| sehr eng | < 30 m | > 6 m/s² |

### A.4 Weitere Vorschläge

- **S-Kurven getrennt ausweisen.** Die Richtungsumkehrschwelle von 5° aus
  1.5.27 ist plausibel; ob sie trifft, lässt sich gegen die 48 geometrischen
  Kurven aus A.1 prüfen, indem man dort Vorzeichenwechsel zählt.
- **Kurvenereignisse mit Anfangs- und Endzeit protokollieren**, nicht nur mit
  dem Abschlusszeitpunkt. Ohne Anfangszeit ist eine saubere Zuordnung zur
  GPS-Spur nur über ein Zeitfenster möglich, was die Genauigkeitsangabe aus
  A.1 unnötig unscharf macht.
- **`KurvenProKm` tatsächlich füllen** (Befund N5). Die Kennzahl ist für
  „kurvenreiche Strecke“ die eigentlich interessante und derzeit konstant 0.

## B. Straßenqualitätsprüfung

### B.1 Die Kennzahl misst praktisch nur den Vibrations-Effektivwert

| Sitzung | Korrelation Qualität ~ VibrationRMS | Qualität ~ Geschwindigkeit | RMS ~ Geschwindigkeit |
|---|---:|---:|---:|
| `084245` | **−0,868** | +0,183 | −0,383 |
| `093525` | **−0,938** | −0,196 | +0,244 |
| `095603` | **−0,933** | −0,183 | +0,310 |

Zwei Schlüsse:

- Die Qualitätszahl ist zu 87 bis 94 % durch `VibrationRMS` bestimmt. Die
  beiden weiteren Terme der Formel — Maximalstoß und Stoßanzahl — tragen wenig
  bei. Wer nur `VibrationRMS` auswertet, verliert kaum Information.
- Die Entkopplung von der Geschwindigkeit ist mit \|r\| ≤ 0,20 **besser als
  erwartet**. Die Kennzahl misst nicht bloß, wie schnell gefahren wurde.

### B.2 Die Geschwindigkeitsnormierung wirkt nur zwischen 20 und 43 km/h

**Beweislage: belegt (Quelltext).** Die Normierung lautet

```cpp
speedFactor = constrain(30.0f / speedKmh, 0.7f, 1.5f);
```

Die untere Klemme wird bei 30/0,7 = **42,9 km/h** erreicht, die obere bei
30/1,5 = **20 km/h**. Außerhalb dieses Fensters ist der Faktor konstant.

In den ausgewerteten Fahrten liegt die Geschwindigkeit fast durchgehend
darüber: Median 53, 57 und 167 km/h. Für `084245` mit Median 167 km/h ist der
Faktor über die gesamte Fahrt konstant 0,7 — die Normierung ist dort **wirkungslos**.

Damit sind Qualitätswerte verschiedener Fahrten bei unterschiedlichem
Geschwindigkeitsniveau nicht vergleichbar. Sichtbar wird das in den
Klassenmedianen:

| Sitzung | 5–50 km/h | 50–90 km/h | 90–130 km/h | 130–260 km/h |
|---|---:|---:|---:|---:|
| `084245` Qualität | 63,5 | 62,1 | 64,5 | 73,1 |
| `084245` RMS | 2,363 | 2,653 | 2,407 | **1,664** |
| `095603` Qualität | 89,8 | 91,0 | 82,8 | — |
| `095603` RMS | 0,726 | 0,902 | **1,261** | — |

In `095603` steigt der Effektivwert mit der Geschwindigkeit, in `084245`
fällt er. Das ist kein Widerspruch, sondern ein Hinweis darauf, dass in
`084245` die langsamen Abschnitte auf schlechterem Belag lagen. Genau diese
Verwechslung von Belag und Fahrprofil kann die aktuelle Kennzahl nicht
auflösen.

### B.3 Vorschläge für eine belastbare Straßenqualität

**Vorschlag 1 – Auswertung über den Weg statt über die Zeit.**
Aktuell wird einmal pro Sekunde ein Wert gebildet. Bei 50 km/h entspricht das
14 m, bei 200 km/h 56 m. Ein und dieselbe Fahrbahn erhält je nach
Geschwindigkeit unterschiedlich viele Stützstellen. Eine Aggregation über
feste Wegabschnitte, etwa 100 m, macht Fahrten unmittelbar vergleichbar und
ist aus den vorhandenen Daten nachträglich berechenbar.

**Vorschlag 2 – Anregung normieren, nicht die Bewertung.**
Die Vertikalanregung einer Fahrbahnwelligkeit wächst näherungsweise mit der
Geschwindigkeit. Statt eines auf 0,7 bis 1,5 geklemmten Faktors bietet sich
eine Normierung über den gesamten genutzten Bereich an, verankert an einer
Referenzgeschwindigkeit und ohne Klemmung nach oben. Prüfbar ist das direkt:
Nach der Änderung muss die Korrelation zwischen Qualität und Geschwindigkeit
innerhalb einer Fahrt gegen null gehen, während die Unterschiede zwischen
Belagsarten erhalten bleiben.

**Vorschlag 3 – Wellenlängen trennen statt Effektivwert bilden.**
Der Effektivwert vermischt kurzwellige Rauheit und langwellige Welligkeit.
Beide werden fahrdynamisch völlig verschieden erlebt. Bei 10 Hz Abtastung
liegt die Nyquist-Grenze bei 5 Hz, was bei 50 km/h einer Wellenlänge von
2,8 m entspricht. Eine Aufteilung in zwei bis drei Bänder ist damit möglich
und näher an der etablierten Praxis (ISO 8608, IRI), die ebenfalls über die
Wegfrequenz und nicht über die Zeit arbeitet.

**Vorschlag 4 – Kurveneinfluss aus der Bewertung herausrechnen.**
In einer Kurve erzeugt die Querbeschleunigung eine Aufbaubewegung, die als
Vibration in die Bewertung eingeht. Mit dem Radius aus A.3 ist der erwartete
Anteil berechenbar und abziehbar. Prüfbar: Die Qualitätswerte innerhalb
erkannter Kurven dürfen sich statistisch nicht mehr von denen auf geraden
Abschnitten gleicher Fahrbahn unterscheiden.

**Vorschlag 5 – Schlaglochschwelle je Plattform belegen.**
Die Ereignisdichte schwankt erheblich:

| Sitzung | Schlaglöcher | Strecke | Dichte |
|---|---:|---:|---:|
| `084245` | 23 | 14,208 km | **1,62/km** |
| `095603` | 6 | 10,384 km | **0,58/km** |

Faktor 2,8 zwischen zwei Fahrten am selben Tag mit demselben Aufbau. Die
Schwere reicht von 2,2 bis 15,2 m/s². Ohne einen definierten Referenzabschnitt
lässt sich nicht entscheiden, ob das die Fahrbahn oder das Fahrprofil abbildet.
Vorschlag: eine kurze, mehrfach befahrene Referenzstrecke festlegen und
verlangen, dass drei Wiederholungen die Ereignisdichte auf ±20 % reproduzieren.

**Vorschlag 6 – die dritte Kennzahl `Glätte` prüfen oder streichen.**
Neben `Qualität` und `VibrationRMS` wird `Glätte` protokolliert. Ob sie
gegenüber dem Effektivwert eigenständige Information trägt, ist offen und mit
einer Korrelationsrechnung über die vorhandenen 1.843 Zeilen sofort zu klären.

## 10. Empfehlungen

Nach Dringlichkeit, alle ohne Codeänderung in diesem Bericht:

1. **N1 zuerst.** Solange `MissedSlots` um Faktor 5,6 untertreibt, ist jedes
   darauf gestützte Abnahmekriterium wertlos. Rundung statt Abschneiden.
2. **N2 als Maßnahme umsetzen.** Der Sammelflush über neun Dateien ist mit
   162 ms im Mittel die messbare Hauptursache der verbliebenen Datenlücken.
   Naheliegend ist, die Dateien nicht mehr alle im selben Durchlauf zu leeren.
3. **N3 und N5** sind kleine Korrekturen mit unmittelbarem Nutzen für jede
   Auswertung.
4. **N6 vor der nächsten Kurvenmessung klären.** Entweder das Magnetometer
   kalibrieren oder — deutlich robuster — die Kurvenerkennung nach A.2 auf
   `GyroZ` stützen und damit ganz von der Magnetometerkalibrierung lösen.
5. **1.5.27 gegen die Kennzahlen aus A.1 messen:** Genauigkeit 92 % und
   Vollständigkeit 46 % sind der Ausgangswert. Eine Verbesserung der
   Vollständigkeit darf die Genauigkeit nicht unter 85 % drücken.

## 11. Prüfumfang und Grenzen

Ausgewertet wurden alle CSV-Dateien der vier genannten Sitzungen sowie die
zugehörigen Stellen in `main.cpp`, `sd_logger.cpp`, `runtime_diagnostics.cpp`,
`bno055_manager.cpp` und `hardware_config.h`.

Nicht Gegenstand dieses Berichts: die Firmware 1.5.27, für die keine Messdaten
vorliegen; die OBD-/CAN-Qualität über die für den Geschwindigkeitsabgleich
benötigten Felder hinaus; die Sitzungen vor dem 30.07.2026.

Alle Zahlen stammen aus den CSV-Dateien und sind mit `python3` und der
Standardbibliothek reproduzierbar. Angaben zur Beweislage sind je Befund
ausgewiesen; wo nur ein Indiz vorliegt, ist das gekennzeichnet.

# Auswertung der Fahrten vom 02.08.2026 mit Firmware 1.5.34

| Feld | Angabe |
|---|---|
| Firmware | 1.5.34, Schema `1.5.28-quality-v10` |
| Fahrzeug | Porsche Carrera S 2012 PDK |
| Sitzungen | `20260802_095708_84FD688D`, `20260802_100225_B067E95B`, `20260802_100356_CF34940C` |
| Zweck | `SESSION_END` belegen, 1.5.30 bis 1.5.34 am Gerät prüfen, Kurvenerkennung auf derselben Strecke hin und zurück vergleichen |
| Berichtsdatum | 02.08.2026 |

## Gesamturteil

**Die Kurvenerkennung ist reproduzierbar.** Auf einer hin und zurück
gefahrenen Strecke stimmen die Kurvenwinkel beider Richtungen im Median auf
1,0 Grad überein, obwohl Richtung, Linie und Geschwindigkeit sich
unterschieden. Der offene Abschlussgrund `SESSION_END` ist dreifach belegt.
Der Abgleich von Gerät und Wiedergabe ergab 33 zu 33 Ereignissen.

Zwei Einschränkungen: GPS hatte in keiner der drei Sitzungen einen Fix, und
das Ziel von 1.5.30 für `LoopMaxMs` wurde nicht erreicht.

## GPS ohne Fix

Über alle 1.887 GPS-Zeilen der drei Sitzungen gilt `Satellites=0`,
`ValidFix=0`, `HDOP=99.99` und `LocationAgeMs=4294967295`.
`RejectionReason=111` entspricht der Bitmaske
`LOCATION_AGE | SATELLITE_QUALITY | HDOP | SPEED_AGE | ALTITUDE | COURSE`.

Der Empfänger arbeitete: 90.364 NMEA-Zeichen, 2.440 gültige Sätze, null
Prüfsummenfehler und null Pufferüberläufe in der Hauptsitzung. Er sah nur
keine Satelliten. Zum Vergleich lieferte derselbe Einbau in
`20260729_170946_05A4DB46` 10 bis 12 Satelliten bei HDOP-Median 0,89.

Folgen:

- `StreckeKm` steht in allen drei Zusammenfassungen auf `0.000`.
- Jedes Ereignis trägt die Koordinate `0.000000/0.000000`, also unbekannt.
- Eine Kartenauswertung dieser Fahrten ist nicht möglich.

Die Kurvenerkennung ist davon **nicht** betroffen. Die Geschwindigkeit kam
wie in `main.cpp` vorgesehen aus OBD, und OBD lief fehlerfrei: 609 Anfragen,
609 Antworten, null Timeouts, null Sendefehler, null CAN-Recovery.

## Vollständigkeit

Zeilenzahl je Datei gegen die Zähler der Metadatendatei, wie für jede
Auswertung verbindlich:

| Sitzung | Dauer | Sensorzeilen / Slots | GPS-Zeilen / Slots | Qualitätswerte | Ereignisse |
|---|---:|---|---|---|---|
| `095708_84FD688D` | 304 s | 2.985 / 3.047 | 1.524 / 1.524 | 292 = Zähler | 31 = 26 Kurven + 5 Schlaglöcher |
| `100225_B067E95B` | 60 s | 591 / 606 | 300 / 300 | 61 = Zähler | 6 = 6 Kurven |
| `100356_CF34940C` | 12 s | 120 / 123 | 62 / 62 | 12 = Zähler | 1 = 1 Kurve |

Alle neun Dateien jeder Sitzung decken die volle Dauer ab. `SDErrors=0`,
`SDDropped=0`, kein `road_integritaet_*.csv`. Der stille Verlust aus
`20260801_114252_B9D1628B` hat sich nicht wiederholt; das erklärt ihn nicht.

## `SESSION_END` bestätigt

Der Abschlussgrund greift nur, wenn die Aufzeichnung gestoppt wird, während
das Fahrzeug über 5 km/h fährt und eine Kurve läuft. Er wurde dreimal
protokolliert:

| Sitzung | Winkel | Dauer | Weg | v mittel | ay mittel |
|---|---:|---:|---:|---:|---:|
| `100356_CF34940C` | 356,9° | 11,2 s | 57,9 m | 18,7 km/h | 2,89 m/s² |
| `100225_B067E95B` | 327,2° | 18,2 s | 91,4 m | 23,0 km/h | 2,02 m/s² |
| `095708_84FD688D` | 34,1° | 3,1 s | 35,5 m | 36,4 km/h | 1,78 m/s² |

Die Werte sind in sich stimmig: 57,9 m Bogenlänge auf 356,9 Grad ergeben
9,3 m Radius, und v²/r beträgt bei 5,18 m/s genau die protokollierten
2,89 m/s².

Damit ist der letzte offene Punkt aus 1.5.29 erledigt.

## Gerät gegen Wiedergabe

`tools/vergleich_kurvenwiedergabe.cpp`, übersetzt aus den Quellen von 1.5.34
gegen die Aufzeichnungen derselben Firmware:

```text
095708_84FD688D  Firmware 26  Wiedergabe 26  Paare 26  Winkelabweichung 0,1 Grad
100225_B067E95B  Firmware  6  Wiedergabe  6  Paare  6  Winkelabweichung 0,0 Grad
100356_CF34940C  Firmware  1  Wiedergabe  1  Paare  1  Winkelabweichung 0,0 Grad

GESAMT  33 zu 33, 100 Prozent zugeordnet, 0,1 Grad mittlere Winkelabweichung
```

Keine Abweichung. Die Umsetzung auf dem ESP32 ist belegt, nicht nur
plausibel.

Der ausgewiesene mittlere Startversatz von 559 ms in der Hauptsitzung ist
kein Messfehler der Erkennung, sondern die Unschärfe der Zeitbasisschätzung.
Ohne Referenzmarker schätzt das Werkzeug den Versatz aus
`max(EndUptimeMs - UptimeMs)`, und das trifft nur, wenn mindestens ein
Ereignis genau nach dem vollen Ruhefenster geschrieben wurde.

## Kurvenerkennung hin gegen zurück

Die Hauptsitzung fuhr eine Strecke hin und zurück, mit einem Wendemanöver in
der Mitte und einem Stopp der Aufzeichnung vor Erreichen des Ausgangspunkts.
Das ergibt eine Reproduzierbarkeitsmessung, die von den Referenzintervallen
vom 31.07.2026 unabhängig ist: dieselbe Kurve, andere Richtung, andere Linie,
andere Geschwindigkeit.

### Zuordnung

Ohne GPS gibt es keine Position. Eine Koppelnavigation aus Gierrate und
OBD-Geschwindigkeit trägt für die Zuordnung nicht: Der Rückweg endete
vorzeitig, und über 300 Sekunden summiert sich die Gyrodrift.

Tragfähig ist die Kurvenfolge selbst. Der Rückweg muss die Hinwegfolge
rückwärts und spiegelverkehrt wiederholen; das Vorzeichen jeder Kurve kippt.
Das Wendemanöver ist eindeutig: Kurve 17 mit +100 Grad bei 18 km/h und 13,6 m
Radius, die engste und langsamste Kurve der Fahrt. Die Ausrichtung des
Rückwegs gegen den umgekehrten Hinweg ist ebenfalls eindeutig; sie beginnt
bei Hinkurve 14.

Auswertung mit `tools/vergleich_hin_rueck.py`.

### Ergebnis

| Hin | Rück | Winkel hin | Winkel zurück | Δ | v hin | v zurück | R hin | R zurück | Modus hin | Modus zurück |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 14 | 18 | −118,5° | +121,0° | +2,5° | 84 | 77 | 171 m | 182 m | SHARP | LONG |
| 13 | 19 | +104,0° | −102,9° | −1,1° | 67 | 65 | 97 m | 86 m | SHARP | LONG |
| 12 | 20 | −37,9° | +38,8° | +1,0° | 64 | 57 | 222 m | 219 m | LONG | LONG |
| 11 | 21 | +53,8° | −53,4° | −0,5° | 54 | 52 | 210 m | 185 m | LONG | LONG |
| 10 | 22 | −95,2° | +95,6° | +0,3° | 46 | 48 | 100 m | 110 m | SHARP | LONG |
| 9 | 23 | +36,1° | −36,8° | +0,7° | 53 | 55 | 144 m | 144 m | SHARP | SHARP |
| 8 | 24 | −32,3° | +35,7° | +3,4° | 52 | 58 | 199 m | 208 m | LONG | LONG |
| 7 | 25 | +35,5° | −37,8° | +2,3° | 49 | 54 | 134 m | 124 m | SHARP | LONG |
| 6 | 26 | −55,9° | +34,1° | −21,8° | 43 | 36 | 87 m | 60 m | LONG | SHARP |

Acht von neun Paaren treffen. Das Vorzeichen kippt in jedem Paar korrekt.

- **Winkel: 1,0 Grad Abweichung im Median, 3,4 Grad im Maximum**, relativ
  2,0 Prozent im Median und 10,5 Prozent im Maximum.
- Radius: 7 Prozent im Median, 12 Prozent im Maximum.
- Mittlere Querbeschleunigung: 16 Prozent im Median, 33 Prozent im Maximum.
- Geschwindigkeit: 6 Prozent im Median, 13 Prozent im Maximum.

Das letzte Paar zählt nicht: Kurve 26 ist die beim Stopp abgeschnittene
`SESSION_END`-Kurve. Sie **soll** zu kurz sein.

Dass Radius und Querbeschleunigung stärker streuen als der Winkel, ist
erwartet: Beide hängen von der gefahrenen Linie und der Geschwindigkeit ab,
die Netto-Kursänderung nicht.

Die Hinkurven 1 bis 5 haben keinen Partner. Der Rückweg wurde vor ihrem
Streckenabschnitt gestoppt.

### `DetectionMode` ist keine Kurveneigenschaft

**In 4 von 8 Paaren weicht `DetectionMode` ab** — dieselbe Kurve, Winkel auf
1 Grad genau gleich, aber `SHARP` hin und `LONG` zurück. Das Feld beschreibt,
welches Tor die Kurve geöffnet hat, nicht die Kurve. Es hängt an der
Anfahrgeschwindigkeit und darf in keiner Auswertung als Kurveneigenschaft
verwendet oder gegen einen Festwert geprüft werden.

Für die Erkennung selbst ist der Befund beruhigend: Beide Pfade führen auf
dasselbe Ergebnis.

### Schlaglöcher

Drei auf dem Hinweg, zwei zurück. Im Abschnitt zwischen den Kurven 10 und 11
lagen hin zwei Treffer mit 3,4 und 6,4 m/s², zurück einer mit 6,8 m/s².
Für den großen Treffer ist das eine auffällige Übereinstimmung, mehr als ein
Hinweis ist es nicht: Der Abschnitt ist mehrere hundert Meter lang, und ohne
GPS gibt es keine Position. Die Straßenqualität steht ohnehin erst nach der
Kurvenerkennung an.

## Laufzeitdiagnose: 1.5.30 zur Hälfte bestätigt

| Kriterium | Erwartet | Gemessen | Urteil |
|---|---|---|---|
| Statuszeilen | gleichmäßig 5 s | 5,00 bis 5,04 s, ein Ausreißer 5,22 s | PASS |
| `SensorMissedSlots` | deutlich unter 19 je Minute | 12,2 je Minute | WARN |
| `LoopMaxMs` | deutlich unter 247 ms | 251 ms | FAIL |

Der Verlauf über die Sitzung erklärt beide Zahlen.

`LoopMaxMs` steht bereits fünf Sekunden nach dem Messstart bei 222 ms, zu
einem Zeitpunkt, an dem `SDMaxMs` erst 44 ms und `WebMaxMs` 10 ms beträgt.
Das ist der Sitzungsstart mit neun geöffneten Dateien, nicht der Flush.
Danach bleibt der Wert bis Sekunde 231 flach auf 232 ms; bei etwa 291 s kommt
ein einzelner Ausreißer auf 251 ms bei `LoopStalls=1`. Der Dauerbetrieb liegt
bei `LoopLastMs=2`.

Die verfehlten Sensorstichproben wachsen streng linear mit rund **einer je
fünf Sekunden** — genau dem Takt eines vollen Flush-Umlaufs. Dazu passt
`FlushMaxMs=101` für einen **einzelnen** Schritt: Ein Schritt von 101 ms
überschreitet die Slotbreite von 100 ms und kostet zwangsläufig eine
Stichprobe.

Die Aufteilung aus 1.5.30 hat den 235-ms-Block also beseitigt. Übrig bleibt,
dass die größte Einzeldatei knapp über der Slotbreite liegt. Das erklärt die
verbliebenen 12 verfehlten Slots je Minute vollständig und ist der nächste
Ansatzpunkt, falls die Rate weiter sinken soll.

## Offene Punkte

1. **GPS-Fix.** Ursache unbekannt. Vor der nächsten Fahrt im Stand abwarten,
   bis `diag` Satelliten meldet, und den Einbau gegen
   `20260729_170946_05A4DB46` prüfen.
2. **Flushschritt über der Slotbreite.** `FlushMaxMs=101` gegen 100 ms Slot.
3. **Stiller SD-Datenverlust.** Unverändert; der Gegentest wartet auf eine
   zweite Karte.

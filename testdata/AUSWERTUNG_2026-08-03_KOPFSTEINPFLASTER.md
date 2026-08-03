# Kopfsteinpflasterfahrt 03.08.2026: die Notenskala bestätigt

| Feld | Angabe |
|---|---|
| Firmware | 1.5.35 |
| Sitzungen | `20260803_082231_D0606CF2` (27,6 km), `20260803_085616_42D61C40` (1,8 km) |
| Zweck | Den unteren Bereich der Fahrbarkeitsskala mit echten Urteilen belegen und den Belagswechselmarker prüfen |

## Gesamturteil

**Die Kalibrierung vom 02.08.2026 bestätigt sich, und die dort dokumentierte
Grenze bei Stufe 4 war kein Methodenproblem, sondern fehlende Daten.**

Beide Sitzungen sind vollständig: Zeilenzahlen decken sich mit den Zählern,
`SDErrors=0`, `SDDropped=0`, keine Integritätsdatei.

## Die Grenzen bleiben unverändert

Über beide Fahrtage zusammen liegen 149 Abschnitte mit Urteil vor, nach 119
am 02.08.2026.

| Stufe | Abschnitte | Median | Bereich | 02.08. allein |
|---:|---:|---:|---|---:|
| 1 sehr gut | 31 | 0,701 | 0,50–1,41 | 0,663 |
| 2 gut | 69 | 0,843 | 0,55–2,57 | 0,843 |
| 3 mäßig | 43 | 1,420 | 0,88–1,89 | 1,373 |
| 4 schlecht | 6 | **1,583** | 1,05–2,29 | 1,316 |

Die beste Trennung zwischen "geht noch" und "geht nicht mehr" liegt
**weiterhin bei 1,08 m/s²** und ordnet 126 von 149 Abschnitten richtig zu -
85 statt zuvor 84 Prozent.

`ROAD_DRIVEABILITY_RMS_GOOD_MPS2` = 0,55 und `_BAD_MPS2` = 1,60 bleiben
damit unverändert. Sie sind jetzt gegen zwei unabhängige Fahrtage belegt,
nicht mehr gegen einen.

## Die Grenze bei Stufe 4 ist keine

Am 02.08.2026 stand im Bericht, Stufe 4 habe keinen höheren Effektivwert als
Stufe 3, weil auf sehr schlechter Straße so langsam gefahren werde, dass die
Anregung wieder sinke. Das mittlere Tempo fiel monoton von 78 auf 39 km/h.

Diese Erklärung war falsch. Der Fahrt fehlte schlicht schlechter Belag - sie
enthielt vier Abschnitte der Stufe 4, und der Beifahrer hielt ausdrücklich
fest, es habe an diesem Tag keine wirklich schlechten Straßen gegeben.

Auf echtem Kopfsteinpflaster steigt der Effektivwert **trotz** niedrigen
Tempos: bis 2,29 m/s² bei einem Median von 2,126 in der langen Sitzung. Stufe
4 liegt jetzt klar über Stufe 3.

Mit sechs Abschnitten bleibt die Datenlage dünn. Die Richtung stimmt aber,
und die Skala ist über alle vier Stufen monoton.

## Der Belagswechselmarker funktioniert

Erstmals benutzt, zehn Marker in beiden Sitzungen. An den Übergängen aufs
Pflaster springt der gemessene Effektivwert deutlich:

| Abschnitt | davor | danach | Sprung |
|---|---:|---:|---:|
| 520 | 0,83 | 1,35 | +63 % |
| 553 | 0,84 | 1,47 | +76 % |
| 599 | 0,85 | 1,53 | +79 % |
| 556 | 2,04 | 1,71 | −16 % |
| 600 | 1,53 | 1,36 | −11 % |

Die drei positiven Sprünge sind die Auffahrten aufs Pflaster, die negativen
die Rückkehr auf Asphalt. **Die 200-Meter-Abschnitte trennen die Beläge
sauber** und vermischen sie nicht - das war die offene Frage, für die der
Knopf gebaut wurde.

Nebenbeobachtung: Bei Abschnitt 553 wurde dreimal innerhalb von 16 Metern
gedrückt, bei 602 zweimal. Das schadet nichts, weil jedes Ereignis seine
Abschnittsnummer trägt. Es zeigt nur, dass der Knopf während der Fahrt schwer
punktgenau zu treffen ist.

## Laufzeitdiagnose

| Sitzung | Dauer | LoopMaxMs | FlushMaxMs | SensorMissed |
|---|---:|---:|---:|---:|
| `082231_D0606CF2` | 1.597 s | 251 | 111 | 396 (14,9/min) |
| `085616_42D61C40` | 188 s | **276** | 95 | 50 (16,0/min) |

`LoopMaxMs` erreicht in der kurzen Sitzung 276 ms und damit den höchsten
bisher gemessenen Wert; bisher lagen die Spitzen bei 251 bis 254 ms. Die
verfehlten Sensorstichproben bleiben mit 15 bis 16 je Minute im gewohnten
Bereich.

Ob das Pflaster den Sensorpfad tatsächlich stärker belastet oder es sich um
einen Einzelwert beim Sitzungsstart handelt, ist offen. Der Wert stammt
bisher stets aus dem Start mit neun geöffneten Dateien.

## Offen

1. `LoopMaxMs` von 276 ms einordnen.
2. Stufe 4 bleibt mit sechs Abschnitten dünn belegt.
3. Der Kartenexport zeigt die Belagswechsel bereits als eigene Ebene; eine
   Sichtprüfung auf der Karte steht aus.

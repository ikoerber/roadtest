# Kalibrierung der Fahrbarkeitsnote, 02.08.2026

| Feld | Angabe |
|---|---|
| Firmware | 1.5.35 aufgezeichnet, Kalibrierung ergibt 1.5.36 |
| Sitzungen | `20260802_113846_4F82C674` (Fehlstart), `20260802_113909_C8F1C2F3`, `20260802_123757_C1B1796E`, `20260802_125851_99D63730` |
| Umfang | 1 h 50 min, 81,3 km, 402 Streckenabschnitte, 135 Beifahrerurteile |
| Zweck | Die geschätzten Notengrenzen gegen menschliches Urteil bestimmen |

## Gesamturteil

**Die Fahrbarkeitsnote lässt sich kalibrieren.** 119 Abschnitte mit
Beifahrerurteil ergeben eine klare Zuordnung von Effektivwert zu Wertung. Die
bisher geschätzten Grenzen waren deutlich zu weit gefasst; die Skala nutzte
nur ihr oberes Drittel.

GPS lief in allen drei Messfahrten. Alle Dateien sind vollständig, keine
Fehler, keine verworfenen Zeilen.

## Vollständigkeit

| Sitzung | Dauer | Strecke | Sensorzeilen | GPS-Zeilen | SDErrors | SDDropped |
|---|---:|---:|---:|---:|---:|---:|
| `113909_C8F1C2F3` | 3.508 s | 52,19 km | 34.222 | 17.481 | 0 | 0 |
| `123757_C1B1796E` | 1.242 s | 1,72 km | 12.150 | 6.201 | 0 | 0 |
| `125851_99D63730` | 1.770 s | 27,42 km | 17.234 | 8.799 | 0 | 0 |

Jede Zeilenzahl stimmt mit dem Zähler der Metadatendatei überein. Keine
`road_integritaet_*.csv`. `20260802_113846_4F82C674` lief 14 Sekunden und
enthält keine Messwerte; offenbar ein Fehlstart.

**GPS war zurück.** Nach dem Totalausfall am Vormittag lieferte der Empfänger
wieder Positionen; die Streckenwerte sind belegt. Der Verdacht auf einen
Wackelkontakt bleibt damit bestehen, siehe
`AUSWERTUNG_2026-08-02_1.5.34.md`.

## Kalibrierung

135 Urteile verteilen sich auf 119 Abschnitte; Mehrfachurteile je Abschnitt
wurden gemittelt.

| Stufe | Wertung | Abschnitte | Effektivwert im Median | Bereich | mittleres Tempo |
|---:|---|---:|---:|---|---:|
| 1 | sehr gut | 19 | **0,663** m/s² | 0,50 – 0,96 | 78 km/h |
| 2 | gut | 57 | **0,843** m/s² | 0,55 – 1,78 | 59 km/h |
| 3 | mäßig | 39 | **1,373** m/s² | 0,88 – 1,89 | 51 km/h |
| 4 | schlecht | 4 | **1,316** m/s² | 1,05 – 1,64 | 39 km/h |

Die Stufen 1 bis 3 trennen sich sauber. Die beste Trennung zwischen
"geht noch" (1 und 2) und "geht nicht mehr" (3 und 4) liegt bei
**1,08 m/s²** und ordnet 100 von 119 Abschnitten richtig zu, also
84 Prozent. Für eine subjektive Zielgröße ist das ein belastbarer Wert.

### Neue Grenzen

Die Grenzen sind so gelegt, dass die Note 50 genau auf der Trennschwelle
liegt: (0,55 + 1,60) / 2 = 1,075.

```c
#define ROAD_DRIVEABILITY_RMS_GOOD_MPS2  0.55f   // vorher 0,5
#define ROAD_DRIVEABILITY_RMS_BAD_MPS2   1.60f   // vorher 3,0
```

| Stufe | Note bisher | Note neu |
|---:|---:|---:|
| 1 | 93 | **89** |
| 2 | 86 | **72** |
| 3 | 65 | **22** |
| 4 | 68 | **28** |

Mit den geschätzten Grenzen lagen alle vier Stufen zwischen 65 und 93 - die
Skala unterschied kaum. Über die 259 Abschnitte der Hauptfahrt ergibt sich
jetzt ein Median von 64 bei voller Ausnutzung des Bereichs; 7 Prozent der
Abschnitte erreichen die Note 0, 2 Prozent die Note 100. Es gibt also weder
Sättigung nach oben noch nach unten.

### Geschwindigkeitsnormierung bringt nichts

Geprüft wurde die Rangkorrelation zur Wertungsstufe:

| Größe | Spearman |
|---|---:|
| Effektivwert roh | +0,73 |
| geteilt durch Wurzel der Geschwindigkeit | +0,74 |
| mal Wurzel des Referenzverhältnisses | +0,74 |
| geteilt durch die Geschwindigkeit | +0,67 |
| geteilt durch das Quadrat | +0,57 |
| Geschwindigkeit allein | −0,34 |

Der Unterschied zwischen roh und der besten normierten Form beträgt 0,01 und
ist keine Grundlage für zusätzliche Komplexität. **Es bleibt beim Rohwert.**

## Grenze der Methode

**Stufe 4 hat keinen höheren Effektivwert als Stufe 3** (1,316 gegen 1,373).
Die Ursache steht in derselben Tabelle: Das mittlere Tempo fällt monoton mit
der Wertung, von 78 auf 39 km/h. Auf sehr schlechter Straße wird so langsam
gefahren, dass die Anregung wieder sinkt, obwohl die Fahrbahn schlechter ist.

Damit ist die Note im oberen Bereich der Skala unschärfer als im mittleren.
Vier Abschnitte sind zu wenig für eine belastbare Aussage; die Stufe 4 sollte
auf weiteren Fahrten gezielt häufiger vergeben werden.

Ein Ansatz wäre, die gefahrene Geschwindigkeit als eigenständiges Merkmal
aufzunehmen statt als Normierungsfaktor - wer langsam fährt, obwohl die
Strecke es nicht verlangt, sagt damit etwas über die Fahrbahn aus. Das ist
aber Spekulation und mit vier Datenpunkten nicht zu belegen.

## Laufzeitdiagnose

| Sitzung | LoopMaxMs | LoopStalls | FlushMaxMs | SensorMissed je Minute |
|---|---:|---:|---:|---:|
| `113909_C8F1C2F3` | 245 | 0 | 119 | 14,6 |
| `123757_C1B1796E` | 242 | 0 | 112 | 13,1 |
| `125851_99D63730` | 254 | 1 | 119 | 15,9 |

Gegenüber dem Vormittag (12,2 je Minute) sind die verfehlten Sensorslots
leicht gestiegen, ebenso `FlushMaxMs` von 101 auf 119. Beides passt zum
gewachsenen Umfang: Die Ereignisdatei nimmt jetzt zusätzlich Abschnitte und
Urteile auf, und die Hauptfahrt dauerte mit 58 Minuten fast zwölfmal so lange
wie die bisherigen Messungen. Der Anteil bleibt mit 2,2 bis 2,7 Prozent der
Stichproben klein.

`LoopMaxMs` liegt unverändert bei rund 250 ms und stammt weiterhin aus dem
Sitzungsstart, nicht aus dem laufenden Betrieb.

## Offen

1. Stufe 4 ist mit vier Abschnitten unterbelegt.
2. Der Belagswechsel-Knopf wurde nicht benutzt; die Abschnittsgrenzen sind
   deshalb noch nicht gegen echte Übergänge geprüft.
3. Der GeoJSON-Export kennt weder `ABSCHNITT` noch die Beifahrermarker.
4. Die Notengrenzen stammen aus einer einzigen Fahrt eines Fahrzeugs mit einem
   Beifahrer. Sie sind belegt, aber nicht allgemeingültig.

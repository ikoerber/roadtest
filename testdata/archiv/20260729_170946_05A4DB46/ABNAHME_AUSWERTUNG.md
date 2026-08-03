# Abnahmeauswertung – Sitzung 20260729_170946_05A4DB46

## Gesamturteil

Die GPS-/OBD-Messqualität während der beiden Fahrphasen ist **PASS**. Der
GPS-Stillstandsfilter, die Streckenbildung und der Geschwindigkeitsvergleich
arbeiten überzeugend.

Die vollständige Stabilitätsabnahme ist nach den strengen Kriterien des
Datenqualitätsplans noch **nicht bestanden**. Nach dem Wiederstart traten drei
systemweite Messpausen von 6,2 bis 8,2 Sekunden auf. Gleichzeitig wurden
9.874 GPS-UART-Bytes verworfen; eine der Pausen führte zu einem kurzzeitigen,
vom Gerät selbst verursachten ECU-Verlustzustand.

## Datenbasis und Methodik

Ausgewertet wurden alle zehn CSV-Dateien der Sitzung vom 29. Juli 2026. Alle
Zeitvergleiche verwenden `UptimeMs`; UTC dient nur der Lesbarkeit. GPS und OBD
wurden über den zeitlich nächsten Messwert zugeordnet. Große Lücken wurden
nicht interpoliert.

- Firmware: 1.5.22
- CSV-Schema: `1.5.21-quality-v5`
- Sitzungsdauer: 1.383 Sekunden
- Strecke laut Zusammenfassung: 9,997 km
- zehn erwartete Sitzungsdateien vorhanden
- alle CSV-Zeilen besitzen die erwartete Spaltenzahl
- keine doppelten oder rückwärts laufenden Uptime-Werte
- START-, STATUS- und END-Metadaten vorhanden
- `ACCEPTANCE_RESULT` vorhanden

## Geführter Ablauf

| Schritt | Ergebnis | Urteil |
|---|---:|---|
| Dateivorbereitung | 1,633 s | PASS |
| passive Listen-Only-Phase | 60 s, 0 Frames | PASS |
| erste ECU-Antwort nach Zündungsmarker | 0,439 s | PASS mit Hinweis |
| erster Motorlauf über RPM ≥ 300 | 3,315 s | PASS |
| erster Stillstand | 302,9 s | PASS |
| erste Fahrphase | 348,834 s | PASS |
| Zwischenstopp | 63,038 s | PASS |
| zweite Fahrphase | 368,867 s | PASS |
| ECU-Verlust nach Ausschaltmarker | 8,983 s | PASS |
| ECU-Antwort nach erneutem Zündungsmarker | 0,589 s | PASS |
| Motorlauf nach Neustart | 5,963 s | PASS |
| Abschlussstillstand | 125,3 s | PASS |

Hinweis zum ersten Zündungsschritt: Die ECU wurde bereits 0,101 Sekunden vor
dem Zündungsmarker als erreichbar protokolliert. Die gemessenen 0,439 Sekunden
sind die Zeit bis zur nächsten Antwort nach dem Marker, aber kein streng
kausaler Nachweis des ersten Einschaltvorgangs. Der Wiederanlaufmarker ist
zeitlich sauber.

## GPS-Qualität

- 5.777 GPS-Qualitätssnapshots
- 2.056 neue Positionsfixes
- 100 % gültige Positionen
- 10 bis 12 Satelliten, Median 12
- HDOP Median 0,89, 95. Perzentil 1,15, Maximum 1,63
- maximales Positionsalter 862 ms
- zwei NMEA-Prüfsummenfehler
- 9.874 verworfene UART-Bytes

Die 2.857 Datensätze mit `RejectionReason=80` entsprechen der Bitmaske
`0x50`: GPS-Geschwindigkeit unter 6 km/h und deshalb ebenfalls ungültiger
Kurs. Das ist in den Stillstandsabschnitten das beabsichtigte Verhalten; die
Position blieb dabei gültig.

### Stillstandsfilter

| Abschnitt | Dauer | maximale GPS-Rohgeschwindigkeit | gültige GPS-Speed-Werte | rohe Fix-Strecke |
|---|---:|---:|---:|---:|
| erster Stillstand | 302,9 s | 1,7 km/h | 0 | 31,7 m |
| Zwischenstopp | 63,0 s | 0,2 km/h | 0 | 5,0 m |
| Stand vor Ausschalten | 9,1 s | 0,4 km/h | 0 | 1,4 m |
| Abschlussstillstand | 125,3 s | 1,9 km/h | 0 | 22,7 m |

Die neuen Fixes wanderten in diesen Standabschnitten roh insgesamt ungefähr
61 Meter. Die beiden Fahrphasen ergeben aus neuen GPS-Fixes ungefähr
9,9997 km; die Fahrzusammenfassung enthält 9,997 km. Daraus folgt, dass die
Standdrift praktisch vollständig aus der Strecke ausgeschlossen wurde. Die
Firmware protokolliert die laufende Teilstrecke nicht separat, daher ist dies
eine Rückrechnung aus Rohpfad, Markern und Abschlusswert.

## GPS-/OBD-Geschwindigkeitsvergleich

Für den Vergleich wurden neue GPS-Fixes mit gültiger Geschwindigkeit und der
zeitlich nächsten gültigen OBD-Geschwindigkeit verwendet.

- 1.048 von 1.048 geeigneten GPS-Fixes innerhalb von 500 ms zugeordnet
- medianer absoluter Zeitabstand: 109 ms
- maximaler Zeitabstand: 355 ms
- Median der absoluten Geschwindigkeitsabweichung: 0,7 km/h
- 95. Perzentil: 3,5 km/h
- Maximum bei dynamischer Fahrt: 9,5 km/h
- mittlere GPS-Abweichung gegenüber OBD: −0,17 km/h
- Pearson-Korrelation: 0,9986
- aus OBD-Geschwindigkeit integrierte Fahrstrecke: ungefähr 10,033 km
- Abweichung der Firmwarestrecke zu OBD: ungefähr −0,36 %

Auch die gehaltenen Geschwindigkeitsbereiche bestanden die Zielwerte:

| OBD-Bereich | Wertepaare | Median absolut | 95. Perzentil |
|---|---:|---:|---:|
| 30 ± 2 km/h | 79 | 0,6 km/h | 2,9 km/h |
| 50 ± 2 km/h | 55 | 0,6 km/h | 3,25 km/h |
| 70 ± 2 km/h | 13 | 0,6 km/h | 2,12 km/h |
| 80 ± 2 km/h | 104 | 0,3 km/h | 1,4 km/h |

Damit werden die Grenzwerte von höchstens 3 km/h Median, höchstens 5 km/h
beim 95. Perzentil und höchstens 500 ms Zeitzuordnung eingehalten.

## OBD und CAN

- 2.529 OBD-Anfragen
- 2.486 Antworten
- sieben Sendefehler
- 55 Timeouts
- zwei nicht zugeordnete Antworten
- keine verlorenen OBD-Trace-Ereignisse
- Antwortlatenz Median 10 ms, 95. Perzentil 17 ms
- sieben RX0-Überläufe, keine RX1-Überläufe
- keine Controller-Recovery und kein Bus-off

In beiden Fahrphasen und im Zwischenstopp antwortete die ECU vollständig:

- Fahrphase 1: 693 Anfragen, 693 Antworten
- Zwischenstopp: 125 Anfragen, 125 Antworten
- Fahrphase 2: 733 Anfragen, 733 Antworten

Die meisten Timeouts und alle Sendefehler liegen erwartungsgemäß in den
Abschnitten mit ausgeschalteter Zündung beziehungsweise im Wiederanlauf. Alle
sieben RX0-Überläufe entstanden während Unterstützungsblockscans mit
gleichzeitigen ECU-Antworten, keiner während der Vergleichsfahrt.

Nach dem Neustart wechselte der ECU-Zustand mehrfach zwischen verloren und
erreichbar. Vor dem Motorstart kann dies teilweise vom schlafenden
Fahrzeuggateway verursacht sein. Der Verlust bei laufendem Motor bei
1.276,353 Sekunden fiel jedoch genau mit einer 7,8-sekündigen systemweiten
Messpause zusammen und ist deshalb kein belastbarer Fahrzeugausfall.

## SD- und Systemstabilität

- 29.268 erfolgreiche SD-Schreibvorgänge
- 0 SD-Fehler
- 0 verworfene SD-Datensätze
- Sensorrate über die gesamte Sitzung: ungefähr 8,5 Hz
- GPS-Snapshotrate über die gesamte Sitzung: ungefähr 4,2 Hz

Die drei größten Lücken waren in allen unabhängigen Logdateien gleichzeitig
sichtbar:

| Sitzungszeit | GPS-Lücke | verworfene GPS-Bytes |
|---:|---:|---:|
| etwa 1.252 s | 8,178 s | 3.860 |
| etwa 1.276 s | 7,783 s | 3.535 |
| etwa 1.314 s | 6,238 s | 2.479 |

Alle drei Lücken lagen nach der Testfahrt, als die Weboberfläche wieder aktiv
bedient beziehungsweise automatisch aktualisiert wurde. Die stärkste
Arbeitshypothese ist deshalb die synchrone Übertragung der großen
Abnahmeseite im Zwei-Sekunden-Rhythmus über die WLAN-Verbindung zum unter dem
Sitz montierten Gerät. Während dieser Übertragung wird die Hauptschleife
offenbar mehrere Sekunden nicht bedient. Ein blockierender SD-Zugriff ist
weniger wahrscheinlich, aber ohne eigene Laufzeitmessung noch nicht
vollständig ausgeschlossen.

## Fazit und nächster Schritt

Bestanden:

- GPS-Feldfilter und Stillstandserkennung
- GPS-Streckenbildung
- GPS-/OBD-Geschwindigkeitsvergleich
- beide Fahrphasen und die geführten Mindestzeiten
- SD-Integrität während der gesamten Sitzung
- ECU- und Motorlauferkennung ohne Firmware-Neustart

Vor der nächsten Abnahme zu beheben:

1. Die Abnahmeseite darf nicht alle zwei Sekunden als vollständige große
   HTML-Seite übertragen werden. Statuswerte sollten über eine kleine
   Statusantwort aktualisiert werden.
2. Laufzeit von Webhandlern, SD-Flush und Hauptschleife muss gemessen und in
   den Sitzungsmetadaten protokolliert werden.
3. Ein ECU-Verlust darf nicht allein aus verstrichener Wandzeit entstehen,
   wenn die Firmware in dieser Zeit selbst keine neue OBD-Anfrage senden
   konnte.
4. Danach genügt zunächst ein kurzer Wiederholungstest von Wiederstart und
   zweiminütigem Abschlussstillstand; die ausgezeichnete Vergleichsfahrt muss
   nicht vollständig wiederholt werden.

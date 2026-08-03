# ROADTEST-Abnahmetest vom 29.07.2026

## Ergebnis

Die ECU-Wiedererkennung der Firmware 1.5.19 funktioniert grundsätzlich ohne
Firmware-Neustart. Der zweite und letzte Durchlauf
`20260729_160020_5818ACB6` bildet den beabsichtigten Ablauf am besten ab.

Gesamturteil für die Recovery-Funktion: **PASS mit Hinweisen**.

Die Abnahmeoberfläche bildete den Porsche-Ablauf noch nicht korrekt ab:
Zündung-Ein und tatsächlicher Motorstart waren ein gemeinsamer Schritt. Die
Daten zeigen jedoch zwei klar getrennte Zeitpunkte. Dieser Bedien- und
Auswertungsfehler wird mit Firmware 1.5.20 korrigiert.

## Datenbasis

Ausgewertet wurden alle CSV-Dateien der zwei vollständigen Sitzungen:

- `20260729_155442_88648234`
- `20260729_160020_5818ACB6`

Beide Sitzungen besitzen START- und END-Metadaten sowie eine
Fahrzusammenfassung. Es gab keine SD-Schreibfehler und keine verworfenen
SD-Datensätze.

## Letzter Durchlauf

Sitzung: `20260729_160020_5818ACB6`

| Prüfung | Ergebnis |
|---|---:|
| SD-Sitzung vollständig | PASS |
| Listen-Only-Phase | 60,1 s |
| ECU nach erstem kombinierten Marker erkannt | 1,9 s |
| Tatsächlicher Motorlauf über RPM > 300 erkannt | 7,1 s |
| ECU-Ausfall nach Ausschaltmarker erkannt | 11,4 s |
| ECU nach Wiederstartmarker erkannt | 1,1 s |
| Tatsächlicher Motorlauf nach Wiederstart | 6,4 s |
| Firmware-Neustart erforderlich | nein |
| SD-Fehler / verworfene Datensätze | 0 / 0 |
| GPS-Fix-Verfügbarkeit | 100 % |
| GPS-UART-Überläufe | 0 |
| CAN-RX0-/RX1-Überläufe | 2 / 0 |

Die ECU antwortete über `0x7E8` und bei mehreren Unterstützungsblockabfragen
zusätzlich über `0x7E9`. Die OBD-Sitzung enthält 87 Anfragen, 66 Antworten,
4 Sendeprobleme, 28 Timeouts und eine verspätete beziehungsweise nicht mehr
zuordenbare Antwort. Die Timeouts liegen überwiegend in den absichtlich
zündungslosen Abschnitten.

GPS war während aller 480 Snapshots gültig. Es wurden 169 neue Fixes, keine
UART-Überläufe und eine NMEA-Prüfsummenabweichung gezählt. Die Satellitenzahl
lag zwischen 7 und 12, der mediane HDOP bei 0,91 und der maximale HDOP bei
2,21.

Die zwei CAN-RX0-Überläufe bleiben ein Hinweis für weitere Fahrzeugtests,
haben die Wiedererkennung in diesem Durchlauf aber nicht verhindert.

## Erster Durchlauf

Sitzung: `20260729_155442_88648234`

Dieser Durchlauf enthält mehrere Bedien- beziehungsweise Fahrzeugzustände:
Die ECU und eine laufende Motordrehzahl waren bereits deutlich vor dem ersten
Abnahmemarker sichtbar. Danach wurden mehrere ECU-Verluste und
Wiedererkennungen aufgezeichnet. Er eignet sich deshalb als Belastungs- und
Recovery-Nachweis, aber nicht als sauberer einzelner Abnahmeablauf.

Trotzdem blieben SD und GPS stabil:

- 0 SD-Fehler und 0 verworfene Datensätze
- 100 % gültige GPS-Snapshots
- 0 GPS-UART-Überläufe
- 5 CAN-RX0-Überläufe
- fünf protokollierte ECU-Wiedererkennungsphasen

## Ursache des Hakens in der Weboberfläche

Zwischen der beim Start erzeugten Sitzungs-ID und dem Beginn der
Listen-Only-Aufzeichnung liegen in beiden Sitzungen ziemlich genau 40
Sekunden. In dieser Zeit wurden neun Dateien einzeln im bereits stark
gefüllten FAT-Wurzelverzeichnis angelegt. Jeder Zugriff blockierte den
Webserver zeitweise, obwohl die Startzustandsmaschine grundsätzlich
nichtblockierend aufgebaut war.

Firmware 1.5.20 legt neue Fahrten daher unter
`/sessions/<SessionId>/` ab. Das Wurzelverzeichnis wird pro Sitzung nur noch
für einen Ordner statt für neun einzelne Dateien erweitert. Das Ereignis
`PREPARATION_COMPLETE` protokolliert künftig die tatsächlich benötigte
Vorbereitungszeit.

## Korrigierter Abnahmeablauf ab 1.5.20

1. Zündung und Motor aus, Abnahmetest starten.
2. 60 Sekunden Listen-Only abwarten.
3. Zündung-Ein markieren und nur die Zündung einschalten.
4. ECU-Antwort abwarten.
5. Motorstart markieren und Motor starten.
6. Motorlauf über frische Drehzahl ab 300 U/min bestätigen lassen.
7. Motor und Zündung ausschalten und ECU-Ausfall abwarten.
8. Zündung erneut separat einschalten und ECU-Wiederverbindung abwarten.
9. Motor erneut starten und Drehzahlnachweis abwarten.
10. Abnahmetest sicher beenden.

## Offene Abnahme

Firmware 1.5.20 muss mit einem weiteren kurzen Standtest bestätigt werden.
Dabei sind besonders die neue Vorbereitungsdauer, die getrennten
Zündungs-/Motorzeiten und die CAN-RX-Überlaufzähler zu prüfen.

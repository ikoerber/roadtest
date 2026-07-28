# Technischer Optimierungs-Backlog

Stand: 28. Juli 2026

## Ausgangslage

Die 30-Sekunden-Aufzeichnung mit Saleae Logic 8 (Capture 13, Kanäle 0/1,
10 MS/s) bestätigt eine fehlerfreie BNO055-Kommunikation:

- feste Adresse `0x29`
- Chip-ID `0xA0`
- NDOF-Modus `0x0C`
- Systemstatus `0x05`
- Selbsttest `0x0F`
- Systemfehler `0x00`
- keine Adress-NACKs und keine fehlerhaft verketteten Transaktionen

Die typische Sensorperiode lag bei 100 bis 102 ms. Periodische Arbeiten
erzeugten jedoch elf Lücken von 195 bis 278 ms. Dadurch wurden in knapp
30 Sekunden nur 283 Sensorzyklen erreicht, entsprechend etwa 9,43 Hz.

Die folgenden Arbeiten sind bewusst zurückgestellt und sollen später
einzeln umgesetzt und jeweils mit einer neuen Saleae-Aufzeichnung geprüft
werden.

## Priorität 1: Sensorabfrage zeitlich priorisieren

- BNO055-Abfrage im Hauptprogramm vor Hardwareprüfung, OLED-Aktualisierung
  und SD-Wartung ausführen.
- Statt `lastSensorRead = currentTime` einen festen nächsten Fälligkeitstermin
  verwenden, damit Verzögerungen den Messtakt nicht dauerhaft verschieben.
- Verpasste Zeitpunkte nicht in einer schnellen Leseschleife nachholen,
  sondern sauber zählen und diagnostizieren.

Ziel: stabiler 100-ms-Takt ohne langfristige Drift.

## Priorität 1: BNO055-Transaktionen reduzieren

- Die Gravitationswerte nicht mehr mit 10 Hz lesen, solange sie weder
  ausgewertet noch protokolliert werden.
- Temperatur und Kalibrierstatus zwischenspeichern und höchstens einmal pro
  Sekunde aktualisieren.
- Prüfen, ob Gyro und Euler-Winkel in einem gemeinsamen Registerblock gelesen
  werden können.
- Lineare Beschleunigung weiterhin in jedem Sensorzyklus erfassen.

Ziel: die regelmäßigen BNO055-Transaktionen ungefähr zu halbieren, ohne
Messfelder im CSV unbeabsichtigt zu verändern.

## Priorität 2: OLED-Blockierzeit reduzieren

- Zuerst die Displayaktualisierung unmittelbar nach einer Sensorprobe
  einplanen.
- Danach wahlweise partielle Displayupdates untersuchen.
- Eine Erhöhung des OLED-Takts auf 400 kHz nur separat testen. Der BNO055 soll
  anschließend wieder mit 100 kHz angesprochen werden.

Ziel: Kein einzelnes OLED-Update darf den nächsten 100-ms-Sensorzeitpunkt
verzögern. Ein 400-kHz-Test gilt nur dann als bestanden, wenn Saleae weiterhin
keine NACKs, beschädigten Transaktionen oder BNO055-Systemfehler zeigt.

## Priorität 2: SD-Schreibzugriffe entkoppeln

Zwischenstand: Der Start über die Weboberfläche ist bereits gestuft. Der
Browser erhält sofort eine Antwort und pro Hauptschleife wird höchstens eine
Logdatei geöffnet. Wiederholte FAT-Suchen nach einem freien Sitzungsnamen
wurden entfernt.

- Dauer und Zeitpunkt der fünfsekündlichen `flush()`-Aufrufe messen.
- Größeren Schreibpuffer, längeres Flush-Intervall oder einen separaten
  Hintergrundtask vergleichen.
- Den Zielkonflikt dokumentieren: Weniger Blockierzeit bedeutet bei einem
  plötzlichen Stromausfall potenziell mehr noch nicht gespeicherte Daten.

Ziel: SD-Latenzen dürfen den Sensorzyklus nicht blockieren.

## Priorität 3: Erfassungs- und Lograte trennen

- Für Schlagloch- und Vibrationsanalyse eine höhere interne Erfassungsrate
  prüfen.
- CSV- und GPS-Korrelation können unabhängig davon bei 10 Hz bleiben.
- Ringpuffer, Schwellwerte und Frequenzanalyse müssen vor einer Erhöhung der
  Erfassungsrate entsprechend angepasst werden.

## Abnahmemessung

Nach jedem Optimierungsschritt:

1. Firmware per OTA einspielen.
2. Saleae-Aufzeichnung für mindestens 30 Sekunden mit Kanal 0 als SDA und
   Kanal 1 als SCL bei 10 MS/s durchführen.
3. I²C mit Adresse `0x29` dekodieren.
4. Prüfen:
   - keine Adress-NACKs oder fehlerhaften Schreibdaten
   - Chip-ID stets `0xA0`
   - Modus `0x0C`, Systemstatus `0x05`, Selbsttest `0x0F`, Fehler `0x00`
   - nur eine Chip-ID-Abfrage pro fünf Sekunden
   - mindestens 295 Sensorzyklen in knapp 30 Sekunden
   - Median und 95. Perzentil der Sensorperiode höchstens 105 ms
   - keine ungeplante Sensorlücke über 120 ms

Jede Änderung soll einzeln gemessen werden, damit ihr tatsächlicher Effekt
und mögliche Nebenwirkungen eindeutig zugeordnet werden können.

# Changelog

Alle wichtigen Änderungen am ESP32-S3 Straßenqualitäts-Messsystem werden in dieser Datei dokumentiert.

Das Format basiert auf [Keep a Changelog](https://keepachangelog.com/de/1.0.0/),
und dieses Projekt hält sich an [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unveröffentlicht]

### Behoben
- BNO055-Erkennung liest jetzt die Chip-ID `0xA0`, statt leere
  I²C-Schreibtransaktionen zu senden, die in der Logic-8-Aufnahme den
  Folgezugriff beschädigten und `SYS_ERR 5` auslösten
- Messwerte werden bereits beim ersten Kommunikations- oder Fusionsfehler
  gesperrt; nur Statusanzeige und automatischer Neustart bleiben entprellt
- Die unbegrenzte Wartezeit nach dem BNO055-Software-Reset endet nach
  1,5 Sekunden mit einem kontrollierten Fehler
- Vollständige NDOF-Kalibrierung verlangt nun konsistent System, Gyro,
  Beschleunigung und Magnetometer mit Status 3
- Der Webbutton zum Starten einer Aufzeichnung wartet nicht mehr auf alle
  SD-Dateioperationen. Die Dateien werden über mehrere Hauptschleifen
  vorbereitet und Startfehler erscheinen auf der Statusseite

### Geändert
- Weboberfläche, OLED, NVS-Metadaten und CSV-Spalten sind auf den aktiven
  NDOF-Modus abgestimmt
- I²C-Operationen werden nach 20 ms abgebrochen
- Die fest verdrahtete BNO055-Adresse ist verbindlich `0x29`; der automatische
  Fallback auf `0x28` entfällt
- Die periodische Gesundheitsprüfung liest die Chip-ID nur noch einmal pro
  Fünf-Sekunden-Zyklus
- Sitzungsnamen erhalten einen Zufallsanteil; wiederholte FAT-Verzeichnissuchen
  nach freien Dateinamen entfallen

### Dokumentation
- Hardwarebeschreibung auf den tatsächlichen LOLIN S3 Mini mit 4 MB Flash
  umgestellt
- Verbindliche GPIO-Belegung mit dem Firmwarecode abgeglichen
- Versorgung des PZSMOCN-SD-Moduls auf 3,3 V korrigiert
- Unbenutzte BNO055- und GPS-Pins sowie CAN-Aufbau und Versorgung eindeutig
  dokumentiert
- Unsichere pauschale Angaben zu CAN-Versorgung und Busterminierung entfernt

## [1.5.23] - 2026-07-29

### Behoben
- Die ECU gilt nicht mehr allein wegen einer langen lokalen Programmpause als
  verloren. Der Wechsel in die Wiedererkennung erfolgt erst nach drei
  tatsächlich unbeantworteten oder fehlgeschlagenen OBD-Anfragen.
- Die Abnahmeseite lädt nicht länger alle zwei Sekunden das vollständige
  HTML-Dokument neu. Dadurch werden die in der Vergleichsfahrt beobachteten
  systemweiten Messpausen und GPS-UART-Pufferverluste vermieden.

### Geändert
- Die bereits bestandene GPS-/OBD-Vergleichsfahrt wurde aus dem Webablauf
  entfernt. Unter `/acceptance` bleibt nur der kurze, geführte
  ECU-Recovery-Kontrolltest mit großen, schrittweise verschwindenden Knöpfen.
- Zustandsänderungen antworten mit einer kleinen Weiterleitung; laufende
  Werte kommen über eine kompakte JSON-Statusantwort. Die Seite wird nur bei
  einem echten Schrittwechsel neu aufgebaut.
- Die frühere allgemeine Fahrzeugtest-Webseite unter `/test` ist nicht mehr
  registriert oder auf der Statusseite verlinkt.
- Das CSV-Schema wurde auf `1.5.23-quality-v6` angehoben.

### Diagnose
- `road_meta_...csv` protokolliert jetzt letzte und maximale Dauer sowie
  Stallzähler für Hauptschleife, Webbehandlung und SD-Flush. Eine Pause ab
  einer Sekunde zählt als Stall.

## [1.5.22] - 2026-07-29

### Hinzugefügt
- Die Abnahmeseite führt durch einen kombinierten Datenqualitätstest mit
  fünf Minuten Anfangsstillstand, zwei mindestens fünfminütigen Fahrphasen,
  einminütigem Zwischenstopp, ECU-Ausfall/Wiederanlauf und zweiminütiger
  Abschlusskontrolle
- Eigene SD-Marker für Beginn, Zwischenstopp, Weiterfahrt und Ende der
  Vergleichsfahrt
- Live-Anzeige von OBD-Geschwindigkeit, GPS-Roh- beziehungsweise
  gültiger Geschwindigkeit und gefilterter Strecke

### Geändert
- Aktionsknöpfe sind auf mindestens 5,4 rem Höhe vergrößert,
  erscheinen nur im jeweils zulässigen Schritt und werden nach erfolgreicher
  Speicherung nicht mehr ausgegeben
- Fahrmarker werden serverseitig erst nach der vorgesehenen Mindestdauer und
  bei einer frischen OBD-Geschwindigkeit bis 1 km/h akzeptiert
- Doppeltippen wird nach dem ersten Absenden im Browser gesperrt; ein
  vorzeitiger Abbruch liegt getrennt hinter einer aufklappbaren Bestätigung

## [1.5.21] - 2026-07-29

### Behoben
- GPS-Geschwindigkeiten unter 6 km/h gelten nicht mehr automatisch als
  zuverlässiger Bewegungsnachweis; das verhindert die im stehenden Fahrzeug
  beobachtete Scheingeschwindigkeit bis 5,2 km/h
- Die Fahrstrecke wird nur noch aus neuen, qualitätsgeprüften Positionsfixes
  gebildet. Eine frische OBD-Geschwindigkeit bis 1 km/h bestätigt Stillstand
  und verhindert, dass GPS-Drift als Weg aufsummiert wird
- Qualitätslücken und unplausibel große Positionssprünge werden nicht durch
  Streckensegmente überbrückt

### Geändert
- Position, Geschwindigkeit, Höhe und Kurs besitzen aktive Alters- und
  Plausibilitätsgrenzen; eine Position benötigt mindestens fünf Satelliten
  und einen HDOP von höchstens 3,5
- `RejectionReason` ist eine Bitmaske der abgelehnten GPS-Felder. Rohwerte
  bleiben zur Diagnose im Qualitäts-Snapshot erhalten, Auswertungen müssen
  die jeweiligen Gültigkeitsfelder beachten
- CSV-Schema auf `1.5.21-quality-v5` angehoben

## [1.5.20] - 2026-07-29

### Behoben
- Der Porsche-Abnahmetest trennt Zündung-Ein und Motorstart in zwei
  eigenständige, quittierte Schritte; ein Motorlauf gilt erst ab einer
  frischen OBD-Drehzahl von mindestens 300 U/min als erkannt
- Doppelte Browserübermittlungen überschreiben die Zeitpunkte von
  Zündungs-, Motorstart- und Motor-Aus-Markern nicht mehr
- Das Anlegen von neun Sitzungsdateien blockiert nicht mehr wiederholt das mit
  alten Messungen gefüllte FAT-Wurzelverzeichnis

### Hinzugefügt
- Getrennte PASS/WARN/FAIL-Zeitmessung für ECU-Erkennung nach Zündung-Ein und
  Motorlauferkennung nach dem Start, jeweils auch beim Wiederanlauf
- Ereignisse `IGNITION_STATE`, `ENGINE_STATE`, `ACCEPTANCE_RESULT` und
  `PREPARATION_COMPLETE` für eine reproduzierbare Abnahmeauswertung

### Geändert
- Neue Messungen werden unter `/sessions/<SessionId>/` abgelegt; die
  bisherigen versionierten Dateinamen bleiben innerhalb des Ordners erhalten
- CSV-Schema auf `1.5.20-quality-v4` angehoben

## [1.5.19] - 2026-07-29

### Behoben
- SD-Pufferfehler werden bis zum aufrufenden Messpfad weitergegeben; nach
  einem fehlgeschlagenen Flush kann kein Datensatz mehr fälschlich als
  gespeichert gezählt werden
- Überlange oder intern beschädigte Sensorzeilen werden vollständig verworfen,
  statt als abgeschnittene CSV-Zeile geschrieben zu werden
- GPS-Fixsequenz und UART-Überlaufzähler bleiben über einen automatischen
  UART-Neustart hinweg monoton; ein alter TinyGPS++-Stand wird dabei nicht
  erneut als neuer Fix ausgegeben
- OBD-Empfangspuffer werden vor der Timeoutbewertung geleert, sodass bereits
  wartende Antworten nach einer langsamen Hauptschleife nicht als Timeout und
  anschließend als unzugeordnet gelten
- Motorstart-, Motor-Aus- und Neustartmarker werden erst nach erfolgreichem
  SD-Schreiben in den Abnahmezustand übernommen
- Eine fehlgeschlagene Fahrzusammenfassung führt nicht mehr zu einem
  fälschlich erfolgreichen Sitzungsabschluss

### Hinzugefügt
- Begrenzte MCP2515-Controller-Recovery bei wiederholtem Bus-off oder falschem
  Betriebsmodus; eine aktive Listen-Only-Phase wird sicher wiederhergestellt
- `CANRecoveryCount` in OBD-, Trace- und Metadatendateien

### Geändert
- Browser-OTA ist während einer laufenden oder vorbereiteten Messung gesperrt,
  damit keine Sitzung geteilt oder mit einem laufenden Discovery-Zustand
  vermischt werden kann
- CSV-Schema auf `1.5.19-quality-v3` angehoben

### Sicherheit
- Controller-Recovery verändert weder die Service-01-Positivliste noch das
  Limit von höchstens zwei OBD-Anfragen pro Sekunde
- Während der passiven 60-Sekunden-Phase startet auch eine Controller-Recovery
  ausschließlich im echten Listen-Only-Modus

## [1.5.18] - 2026-07-29

### Hinzugefügt
- Eigener ECU-Erreichbarkeitszustand `unbekannt`, `wird gesucht`,
  `erreichbar` und `Verbindung verloren`
- Begrenzte Wiedererkennung nach unbeantwortetem PID-Scan oder fünf Sekunden
  ohne ECU-Antwort; sichere Rückfallrunde mit `0x0C`, `0x0D`, `0x11` und
  `0x00`, anschließend erneuter Unterstützungsblockscan
- Geführte mobile Abnahmeseite unter `/acceptance` für Motor aus, späten
  Motorstart, erkannten ECU-Ausfall und Wiederstart einschließlich
  PASS/WARN/FAIL-Zeitmessung
- Einzelzähler für MCP2515-Empfangspufferüberläufe in OBD-, Trace- und
  Metadatendateien

### Geändert
- GPS-UART-Ringpuffer von 512 auf 2.048 Byte vergrößert
- CSV-Schema auf `1.5.18-quality-v2` angehoben

### Sicherheit
- Wiedererkennung verwendet nur die vorhandene Service-01-Positivliste,
  sendet insgesamt höchstens zwei Frames pro Sekunde und bleibt während der
  60-sekündigen Listen-Only-Phase vollständig passiv
- MCP2515-Überlaufflags werden erst nach dem Zählen kontrolliert gelöscht;
  Bus-off- oder andere Fehlerflags bleiben unverändert

## [1.5.17] - 2026-07-29

### Hinzugefügt
- Mobile Fahrzeugtest-Seite unter `/test` mit großen Schaltflächen für
  Discovery-Start, Statusaktualisierung, die sechs festgelegten
  Fahrabschnittsmarker und den sicheren Abschluss
- Live-Anzeige von Discovery-Phase und -Dauer, verbleibender Listen-Only-Zeit,
  GPS-Fix, Satelliten, HDOP, OBD-Sitzungszählern und SD-Schreibfehlern
- Direkter Link von der Systemstatusseite zur Fahrzeugtest-Steuerung

### Sicherheit
- Alle zustandsändernden Testaktionen bleiben mit den Admin-Zugangsdaten
  geschützt und rufen ausschließlich die vorhandenen Discovery-Funktionen auf
- Die Seite ergänzt keine frei wählbaren CAN-Frames oder schreibenden
  Fahrzeugdienste; Listen-Only, Service-01-Positivliste und Senderatenlimit
  bleiben unverändert
- Das CSV-Schema bleibt `1.5.16-quality-v1`, da sich keine Spalten ändern

## [1.5.16] - 2026-07-29

### Geändert
- CAN-, OBD-, OBD-Trace- und Korrelationsdateien werden bei verfügbarem
  CAN-Adapter schrittweise während der Startvorbereitung geöffnet, bevor die
  eigentliche Messsitzung und ihre GPS-Zähler beginnen
- GPS-Verarbeitung und 200-ms-Snapshots werden dadurch nicht mehr von der
  mehrsekündigen ersten FAT-Dateiöffnung innerhalb der Sitzung unterbrochen
- Firmware- und CSV-Schemaversion wurden für das geänderte Binär- und
  Datenformat gemeinsam auf 1.5.16 angehoben

### Behoben
- Der erste OBD-Sendeversuch erzeugt nicht länger einen GPS-Ringpufferüberlauf
  durch das verzögerte Erstellen von `road_obd_trace_...csv`
- Später eintreffende CAN- und OBD-Daten lösen während einer laufenden
  Messsitzung keine bedarfsgesteuerte Erstöffnung ihrer Logdateien mehr aus

## [1.5.15] - 2026-07-29

### Hinzugefügt
- Echte OBD-Sitzungszähler ab Beginn jeder SD-Aufzeichnung, getrennt von den
  weiterhin sichtbaren Boot-Gesamtzählern
- Transaktionsdatei `road_obd_trace_<Sitzung>.csv` für jeden Sendeversuch,
  jede zugeordnete Antwort und jeden 400-ms-Antworttimeout einschließlich
  Sequenz, PID, ECU-ID, Latenz, MCP2515-Modus, TEC, REC und EFLG
- Metadatendatei `road_meta_<Sitzung>.csv` mit Start-, Fünf-Sekunden- und
  Abschlusswerten, Firmware- und CSV-Schemaversion, Fahrzeugprofil,
  CAN-Konfiguration sowie GPS-, OBD-, CAN- und SD-Diagnosezählern
- Feldweise GPS-Gültigkeit und Alter für Position, Geschwindigkeit, Höhe,
  Kurs, Satelliten und HDOP
- GPS-Fixsequenz, Kennzeichnung neuer Fixes, exakte NMEA-Prüfsummenzähler und
  sichtbarer UART-Ringpuffer-Überlaufzähler

### Geändert
- GPS-CSV-Dateien enthalten auch bei einem Fixverlust weiterhin
  Qualitäts-Snapshots; alte gültige Datensätze werden nicht mehr als aktuelle
  Messung zurückgegeben
- GPS-Diagnosezähler und Fixsequenz beginnen in jeder Aufzeichnung bei null;
  automatische GPS-Neustarts erzeugen dabei keinen Zählerunterlauf
- Das SD-Dateilimit berücksichtigt alle Grund-, CAN-, OBD-, Trace- und
  Korrelationsdateien sowie die Abschlusszusammenfassung; die erste
  OBD-Trace-Datei blockiert dadurch nicht länger die GPS-Verarbeitung
- Die Web-Kalibrierung führt auf Mobilgeräten automatisch durch den jeweils
  nächsten notwendigen Gyro-, Beschleunigungs-, Magnetometer- und
  Systemschritt; Fortschritt und aktuelle Rohstatuswerte bleiben sichtbar
- Der manuelle BNO055-Neustart und seine Webroute wurden aus der
  Kalibrierungsseite entfernt; die vorhandene automatische
  Sensorwiederherstellung bleibt aktiv
- PlatformIO übernimmt die zentrale Firmwareversion automatisch in den
  Buildnamen und exportiert `roadtest_<Version>.bin` ins Projektverzeichnis
- OLED-Start-, Live-, GPS-, Fahrbahn-, Kalibrierungs- und Diagnoseansichten
  zeigen konsistent die zentrale Firmwareversion
- Die OTA-Seite zeigt die installierte Version, den offiziellen
  Firmwaredateinamen und nach dem Upload den tatsächlich gewählten Dateinamen
- OBD-Antworten werden innerhalb eines Anfragefensters auch dann korrekt
  derselben Sequenz zugeordnet, wenn mehrere Steuergeräte antworten
- Serieller Fünf-Sekunden-Status, `diag`, OLED, Weboberfläche und
  Bootmeldung verwenden konsistent Firmwareversion 1.5.15

### Sicherheit
- OBD-Positivliste, ausschließlich lesender Service 01, maximale Senderate
  von zwei Anfragen pro Sekunde und echte Listen-Only-Phase bleiben
  unverändert
- 1.5.15 protokolliert Datenqualität, verändert aber noch nicht den
  einmaligen PID-Scan oder die GPS-Plausibilitätsfilter; diese Schritte folgen
  getrennt in späteren Versionen

## [1.5.14] - 2026-07-28

### Hinzugefügt
- Geführte Fahrzeugdaten-Erkennung über `discover begin`, `discover status`,
  `discover mark <Text>` und `discover end`
- Automatische 60-Sekunden-Phase im echten MCP2515-Listen-Only-Modus ohne
  OBD-Anfragen, ACK- oder Error-Frames
- Zweifacher, auf insgesamt zwei Anfragen pro Sekunde begrenzter Scan der
  standardisierten Service-01-Unterstützungsblöcke `00`, `20`, `40` und `60`
- Fortlaufende Abfrage ausschließlich der vom Fahrzeug bestätigten PIDs für
  Drehzahl, Geschwindigkeit, Luftmassenstrom, Drosselstellung,
  Außentemperatur, Öltemperatur und Kraftstoffrate
- Eigenes OBD-CSV mit dekodierten Messwerten, Gültigkeitskennzeichen,
  Unterstützungsbitmaps und Diagnosezählern
- Zeitmarken und Phasenwechsel im Ereignis-CSV zur späteren gemeinsamen
  Auswertung von CAN-, GPS-, BNO055- und OBD-Dateien

### Sicherheit
- Der MCP2515 empfängt während des passiven Mitschnitts ausschließlich und
  wird erst vor den standardisierten Service-01-Abfragen in den Normalmodus
  zurückgeschaltet
- Frei wählbare CAN-Telegramme, herstellerspezifische Dienste, PID-Bruteforce,
  UDS-Schreibdienste und Fehlerlöschen bleiben ausgeschlossen
- Die Datenerkennung übernimmt ihre SD-Aufzeichnung und verhindert
  konkurrierende `start`-, `stop`- und `obd on/off`-Kommandos

### Leistung
- Während der Erkennung werden CAN-Empfangspuffer in jeder Hauptschleife mit
  begrenztem Budget geleert
- Das redundante Korrelations-CSV bleibt in diesem Modus aus; Sensor-, GPS-
  und CAN-Dateien sind bereits über UTC und Sitzungs-Uptime synchronisiert

## [1.5.13] - 2026-07-28

### Hinzugefügt
- Rein beobachtender Fahrzeug-Testlauf über `test begin`, `test status` und
  `test end`; Aufzeichnung und OBD-Abfragen werden nicht automatisch verändert
- PASS-/WARN-/FAIL-Abschluss anhand der während des Testzeitraums neu
  aufgetretenen OBD-, I²C-, SD- und CAN-Hardwarefehler
- Dekodierte MCP2515-Diagnose mit Betriebsmodus, Transmit Error Counter
  (`TEC`), Receive Error Counter (`REC`) und Error Flag Register (`EFLG`)
- TEC-/REC-Spitzenwerte und EFLG-Bits werden während eines Testlaufs
  festgehalten, damit zwischenzeitliche Fehler im Abschlussbericht sichtbar
  bleiben
- CAN-Hardwarezustand mit TEC, REC und EFLG auf der Web-Statusseite

### Sicherheit
- Die neue Testfunktion führt selbst keine CAN- oder OBD-Anfrage aus und
  startet oder beendet keine SD-Aufzeichnung
- MCP2515-Diagnoseregister werden ausschließlich gelesen

## [1.5.12] - 2026-07-28

### Geändert
- Das steckbare OLED ist optional und blockiert weder Systembereitschaft noch
  SD-Aufzeichnung
- Die automatische Erkennung beim Start und Wiederverbindung eines später
  angeschlossenen OLED bleiben aktiv
- Die Weboberfläche unterscheidet zwischen bereitem CAN-Adapter und einer
  aktuell antwortenden ECU

### Diagnose
- Der serielle Fünf-Sekunden-Status meldet kumulierte BNO055-/OLED-I²C-
  Aussetzer, ECU-Antwortalter sowie SD-Schreibvorgänge, Fehler und verworfene
  Datensätze
- Bereits die erste fehlgeschlagene BNO055- oder OLED-I²C-Prüfung wird seriell
  gemeldet; die bestehende Drei-Prüfungen-Entprellung bleibt erhalten
- Das serielle Kommando `diag` enthält die neuen OBD-, SD- und I²C-Zähler
- `obd off` pausiert aktive OBD-Anfragen bei Tests ohne Fahrzeug; `obd on`
  aktiviert sie wieder, ohne den MCP2515 neu zu initialisieren

## [1.5.11] - 2026-07-28

### Hinzugefügt
- Begrenzte, nur lesende OBD-II-Service-01-Abfrage für Drehzahl (PID `0x0C`),
  Geschwindigkeit (`0x0D`) und Drosselklappenstellung (`0x11`)
- OBD-Livewerte sowie Anfrage-, Antwort- und Sendefehlerzähler auf der
  Web-Statusseite
- Hardwarefilter für die OBD-Antwort-IDs `0x7E8` bis `0x7EF`; Livewerte
  verfallen nach fünf Sekunden ohne neue Antwort

### Sicherheit
- Die gesamte OBD-Senderate ist auf zwei Anfragen pro Sekunde begrenzt
- Der MCP2515 bricht eine nicht bestätigte Übertragung nach 25 ms ab, damit
  Webseite und Watchdog nicht blockieren
- Fehlerlöschen, Codierung, Stellgliedtests und frei wählbare CAN-Nachrichten
  sind nicht implementiert

## [1.5.10] - 2026-07-27

### Behoben
- Der sichtbare BNO055-Status bleibt bei einem oder zwei einzelnen,
  kurzzeitigen I²C-Statusfehlern auf dem zuletzt bestätigten Zustand
- Erst drei aufeinanderfolgende vollständige Fusionsfehler setzen den Status
  auf Fehler und lösen den bestehenden automatischen Sensorneustart aus
- Ein erfolgreicher Status setzt Fehlerzähler und Fusionsstatus sofort zurück

## [1.5.9] - 2026-07-27

### Behoben
- Bereitschaft, Datenverarbeitung, Webdiagnose und automatischer Wiederanlauf
  verwenden jetzt dieselbe vollständige Fusionsprüfung: IMUPLUS-Modus 8,
  Systemstatus 5 und Fehlercode 0
- Ungültige Fusionsdaten werden sofort gesperrt; ein Sensorneustart erfolgt
  erst nach drei aufeinanderfolgenden Fehlerprüfungen
- Integrationstests vergleichen beim IMUPLUS-Modus relative Drehänderungen
  statt des relativen Sensorwinkels mit dem absoluten GPS-Kurs
- Wiederanlauf- und Driftprüfung verwenden nur Gyro und Beschleunigung
- README, Hardwarebeschreibung, OLED-Texte und CSV-Feldnamen wurden auf den
  IMUPLUS-Betrieb abgestimmt

## [1.5.8] - 2026-07-27

### Geändert
- BNO055 läuft im IMUPLUS-Modus nur mit Gyro und Beschleunigung
- Magnetometer und absolute Kompassrichtung werden nicht mehr verwendet
- Für eine vollständige Kalibrierung müssen nur Gyro und Beschleunigung den
  Wert 3 erreichen; eine Achterbewegung entfällt
- BNO055-Selbsttest und Webdiagnose berücksichtigen den IMUPLUS-Betrieb

## [1.5.7] - 2026-07-27

### Geändert
- BNO055 verwendet die interne Taktquelle, um den sporadischen
  Fusionsfehler 9 des vorhandenen Aufbaus zu vermeiden
- I²C-Bus läuft mit 50 kHz statt 100 kHz, um die bekannten
  BNO055-Timingprobleme und die Lochrasterverdrahtung robuster zu behandeln
- Die Kalibrierseite zeigt die verwendete Taktquelle

## [1.5.6] - 2026-07-27

### Behoben
- Der BNO055-Modus wird nicht mehr in jedem Schleifendurchlauf über I2C
  abgefragt, wenn eine andere Komponente noch nicht bereit ist
- Einzelne fehlerhafte Moduslesungen lösen keinen unnötigen Wechsel auf NDOF
  mehr aus; erst drei aufeinanderfolgende Fehler führen zu einem Neustart
- Der Web-Schalter startet den BNO055 jetzt vollständig neu, statt lediglich
  kurz zwischen CONFIG und NDOF umzuschalten

## [1.5.5] - 2026-07-27

### Behoben
- Die automatische CAN-Initialisierung ist wieder deaktiviert, weil der
  blockierende MCP2515-Start ohne betriebsbereiten CAN-Aufbau die Webseite
  anhalten kann
- Die BNO055-NDOF-Diagnose aus 1.5.4 bleibt vollständig erhalten

## [1.5.4] - 2026-07-27

### Behoben
- Der BNO055-Betriebsmodus wird nach Initialisierung und Laden gespeicherter
  Kalibrierwerte geprüft und bei Bedarf ausdrücklich auf NDOF gesetzt
- Sensorwerte werden nur verarbeitet, wenn die NDOF-Sensorfusion aktiv ist

### Hinzugefügt
- Kalibrierseite zeigt BNO055-Betriebsmodus, Fusionsstatus, Systemstatus und
  Fehlercode
- Geschützter BNO055-Neustart direkt auf der Kalibrierseite

### Geändert
- Das optionale CAN-Modul wurde einmalig nach dem Systemstart geprüft

## [1.5.3] - 2026-07-27

### Geändert
- CAN ist für den aktuell verwendeten Aufbau standardmäßig vollständig
  deaktiviert und kann die Hauptschleife nicht mehr beeinflussen
- ROADTEST verwendet fest `192.168.4.1`, WLAN-Kanal 6 und keinen Schlafmodus

### Hinzugefügt
- WLAN-Watchdog, der einen ausgefallenen Access Point automatisch neu startet

## [1.5.2] - 2026-07-27

### Geändert
- Das ROADTEST-WLAN startet vor allen externen Hardwareprüfungen
- Die optionale CAN-Prüfung erfolgt erst nach dem Systemstart im Hintergrund

### Behoben
- Eine langsame Hardwareinitialisierung konnte den WLAN- und OTA-Zugang
  verzögern

## [1.5.1] - 2026-07-27

### Behoben
- Die GPS-Startprüfung wartete zu streng auf einen vollständig gültigen
  NMEA-Satz und konnte dadurch dauerhaft `SUCHE` anzeigen
- Für die Hardwareprüfung genügt nun ein aktueller NMEA-Datenstrom; Fix,
  gültige Sätze und Prüfsummen bleiben getrennt sichtbar

### Hinzugefügt
- GPS-Diagnose auf der Webseite mit empfangenen Zeichen, gültigen Sätzen und
  Prüfsummenfehlern

## [1.5.0] - 2026-07-27

### Hinzugefügt
- Einheitliche OLED-Prüfseite für OLED, BNO055, SD, GPS, WLAN und optionales CAN
- Automatische Wiederverbindung für BNO055, OLED, SD-Karte, GPS-UART und WLAN
- Anzeige von SD-Schreibfehlern und verworfenen Datensätzen auf der Webseite
- Kennzeichnung einer durch einen SD-Fehler abgebrochenen Messfahrt

### Geändert
- Die Startprüfung blockiert die Firmware nicht mehr; Webseite und
  Wiederanlauf bleiben während eines Fehlers aktiv
- Die OLED-Prüfseite wird aktualisiert, bis alle erforderlichen Komponenten
  reagieren, und erscheint bei einem späteren Ausfall erneut
- Ein GPS-Fix ist nicht zum Start nötig; gültige NMEA-Kommunikation genügt
- CAN bleibt optional und verhindert weder Systembereitschaft noch Messfahrt

### Behoben
- Ein fehlender BNO055 führte zuvor zu einer permanenten Warteschleife
- Mehrere aufeinanderfolgende Bootanzeigen überspielten sich gegenseitig
- Fehlgeschlagene SD-Schreibvorgänge wurden nicht zuverlässig gezählt

## [1.4.0] - 2026-07-27

### Hinzugefügt
- Geschützter Start und Stopp einer Messfahrt auf der Statusseite
- Live-Zusammenfassung mit Dauer, GPS-Strecke, Kurven, Schlaglöchern und
  Durchschnittsqualität
- Dauerhafte `road_summary_<Sitzung>.csv` nach dem Beenden
- Serielle Kommandos `start` und `stop` als Alternative zur Webseite

### Geändert
- Nach dem Einschalten ist das System messbereit, beginnt aber erst nach
  ausdrücklichem Start mit der SD-Aufzeichnung
- Alle Dateien einer Fahrt verwenden dieselbe eindeutige Sitzungskennung
- Beim Beenden werden sämtliche Puffer geleert und alle Dateien geschlossen

## [1.3.0] - 2026-07-27

### Hinzugefügt
- ROADTEST-WLAN mit kompakter Statusseite und Browser-OTA
- Web- und OLED-Assistent für BNO055-Kalibrierung
- Geschütztes Speichern einer vollständig erreichten Kalibrierung
- Warnung beim Start mit unvollständiger Kalibrierung

### Geändert
- BNO055 wird im Messbetrieb nur einmal pro 100-ms-Zyklus gelesen
- Vibration, Maximalstoß und Stoßanzahl verwenden ein rollendes
  Ein-Sekunden-Fenster
- Straßenqualität wird bei gültigem GPS vorsichtig auf 30 km/h normiert
- Kurven und Schlaglöcher werden nur einmal pro abgeschlossenem Ereignis
  protokolliert

### Behoben
- Gespeicherte BNO055-Kalibrierung wurde beim Start wegen einer falschen
  Initialisierungsreihenfolge nicht geladen
- Maximalstoß und Stoßanzahl wuchsen zuvor über die gesamte Fahrt
- Kurven konnten während desselben Ereignisses mehrfach protokolliert werden

## [1.2.0] - 2024-12-XX

### Hinzugefügt
- **Umfassende Integration-Test-Suite** mit 92% Coverage
  - 24+ Test-Szenarien für alle Module
  - Hardware Failure & Recovery Tests
  - Performance und Latenz-Messungen
  - Memory-Leak Detection
  - Edge-Case und Stress-Tests
- **GPS Interrupt-Modus** zur Entkopplung von UART-Empfang und Auswertung
  - Ring-Buffer mit 512 Bytes für NMEA-Daten
  - Automatische Aktivierung beim Start
  - Umschaltbar zwischen Interrupt und Polling-Modus
  - Hardware-Test vergleicht beide Modi
- **NVS-Speicher für BNO055-Kalibrierung**
  - Kalibrierungsdaten überstehen Neustarts
  - Automatisches Laden beim Start
  - `clearCalibration()` Methode zum Zurücksetzen
  - Zeitstempel für Kalibrierungsalter
- **Erweiterte Hardware-Tests**
  - GPS Interrupt vs. Polling Vergleichstest
  - Verbesserte Diagnose-Ausgaben

### Geändert
- **String-Operationen optimiert**
  - Statische Buffer statt String-Konkatenation
  - Bessere Performance in kritischen Pfaden
  - Reduzierte Heap-Fragmentierung
- **Sicherheitsverbesserungen**
  - Null-Checks für alle dynamischen Allokationen
  - `strcpy`/`strcat` durch `strncpy`/`strncat` ersetzt
  - Erweiterte Buffer-Overflow-Checks
- **Code-Organisation**
  - Hardware-Pin-Definitionen in `hardware_config.cpp` zentralisiert
  - Keine doppelten Pin-Definitionen mehr

### Behoben
- Potentielle Null-Pointer-Dereferenzierung in BNO055Manager
- String-Buffer-Overflow-Risiken in CAN-Reader und SD-Logger

### Performance
- GPS-Datenempfang ohne Verluste auch bei hoher CPU-Last
- Schnellere String-Formatierung durch vorallokierte Buffer
- Reduzierte CPU-Last durch Interrupt-basiertes GPS

## [1.1.0] - 2024-11-XX

### Hinzugefügt
- Multi-Layer Buffer-Overflow-Schutz
- Umfassende Hardware-Test-Suite
- Korrelierte Datenaufzeichnung (Sensor + CAN + GPS)
- 8 OLED-Display-Modi mit Auto-Rotation
- Automatische Hardware-Erkennung mit Fallback
- Vibrations-Analyse mit RMS und Schock-Erkennung

### Geändert
- Verbesserte Error-Recovery-Mechanismen
- Optimierte Timing-Intervalle für Echtzeit-Performance
- Erweiterte Logging-Funktionalität

### Behoben
- I2C-Bus-Hänger bei Sensor-Ausfall
- SD-Karten-Initialisierung mit alternativen Pin-Sets
- CAN-Bus Oszillator-Erkennung

## [1.0.0] - 2024-10-XX

### Initiales Release
- Basis-Funktionalität mit BNO055, GPS, CAN-Bus
- SD-Karten-Datenlogging
- Einfache Straßenqualitäts-Bewertung
- OLED-Display-Unterstützung
- Grundlegende Hardware-Tests

---

## Upgrade-Anleitung

### Von 1.1.0 zu 1.2.0
1. Code aktualisieren und neu kompilieren
2. Erste Kalibrierung nach Update wird automatisch im NVS gespeichert
3. GPS wechselt automatisch in Interrupt-Modus
4. Keine Konfigurations-Änderungen erforderlich

### Von 1.0.0 zu 1.2.0
1. Backup der SD-Karte erstellen
2. Neue Firmware flashen
3. Hardware-Test durchführen
4. BNO055 neu kalibrieren (wird dann gespeichert)

## Bekannte Probleme

### v1.2.0
- Keine bekannten Probleme

### v1.1.0
- BNO055-Kalibrierung geht bei Neustart verloren (behoben in v1.2.0)
- GPS-Daten können bei hoher CPU-Last verloren gehen (behoben in v1.2.0)

## Geplante Features

### Weitere Ideen
- [ ] FFT-Analyse für Vibrations-Frequenzspektrum
- [ ] FreeRTOS Task-basierte Architektur
- [ ] WLAN-Live-Streaming
- [ ] Erweiterte webbasierte Konfiguration
- [ ] Multi-Language Support für OLED

### v2.0.0 (Zukunft)
- [ ] Machine Learning für Straßentyp-Klassifikation
- [ ] Bluetooth BLE für Smartphone-App
- [ ] Cloud-Integration mit automatischem Upload
- [ ] OTA-Updates über WiFi

# Changelog

Alle wichtigen Änderungen am ESP32-S3 Straßenqualitäts-Messsystem werden in dieser Datei dokumentiert.

Das Format basiert auf [Keep a Changelog](https://keepachangelog.com/de/1.0.0/),
und dieses Projekt hält sich an [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.5.44] - 2026-08-04

### Hinzugefügt: Rücksprung auf die vorherige Firmware

Ein fehlerhaftes Abbild macht ein eingebautes Gerät zum Ausbauteil. Der
Bootloader des ESP32-S3 kann das auffangen -
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` ist im Arduino-Unterbau gesetzt -,
aber der Mechanismus lief bisher ins Leere: `initArduino()` markiert jedes
Abbild als gültig, **bevor `setup()` überhaupt läuft**. Damit galt alles als
gut, was bis zur Arduino-Initialisierung kam, auch eine Firmware, die danach
abstürzt oder deren WLAN nicht mehr hochkommt.

Die Firmware übernimmt die Bestätigung jetzt selbst. Über den vom Kern
vorgesehenen Haken `verifyRollbackLater()` bleibt das Abbild im Zustand
`PENDING_VERIFY`, bis es sich als lauffähig erwiesen hat. Startet es vorher
neu, verwirft der Bootloader es und bootet die vorherige Partition.

**Geprüft wird ausdrücklich nur die Erreichbarkeit**, nicht die Messkette.
Eine fehlende SD-Karte oder ein stummer BNO055 sind Hardwarezustände; ein
Rücksprung würde daran nichts ändern und nur eine gute Version verwerfen.
Entscheidend ist allein, ob sich das Gerät noch erreichen lässt, um erneut zu
flashen - also `webManager.isReady()` und `FIRMWARE_CONFIRM_UPTIME_MS`
Laufzeit.

Das Fenster ist mit 15 Sekunden bewusst kurz. Mit zündungsgeschalteter
Spannung startet das Gerät bei jeder Fahrt neu, und ein Neustart vor der
Bestätigung löst den Rücksprung aus. Ein langes Fenster verwürfe irgendwann
eine einwandfreie Firmware, nur weil jemand kurz umgeparkt hat.

**Nach einem OTA-Update deshalb 15 Sekunden warten**, bevor die Zündung
ausgeht. `diag` zeigt den Zustand: „bestätigt" oder „noch unbestätigt -
Neustart springt zurück".

## [1.5.43] - 2026-08-04

### Hinzugefügt: Daten übers Handy holen

Die Karte zu ziehen ist bei einer Fahrt pro Woche gleichgültig und bei
täglichen Fahrten der Grund, warum die Streckendatenbank nicht wächst - erst
recht, wenn das Gerät hinter der Verkleidung sitzt. `/files` zeigt jetzt die
Sitzungen auf der Karte und gibt jede als **ein `.tar.gz`** aus.

- **Eine Sitzung ist ein Tippvorgang.** Sie besteht aus zehn Dateien; einzeln
  geholt wären das zehn Bedienschritte je Fahrt, bei zweihundert Fahrten
  zweitausend. `tar xzf` liefert am Rechner genau die Verzeichnisstruktur, die
  `tools/export_geojson.py` und die Wiedergabetests erwarten.

- **Komprimiert, nicht roh.** Genau daran scheiterte die früher entfernte
  Downloadfunktion: rund 12 MB je Fahrtstunde. Der Kompressor stammt aus dem
  ROM des ESP32-S3 und kostet kein Byte Flash; CSV komprimiert sich acht- bis
  zwölffach. `GzipStream` konnte bisher nur in Dateien schreiben und hat dafür
  eine austauschbare Senke bekommen - dieselbe Rahmung und dieselbe Prüfsumme
  für Karte und HTTP-Antwort, statt eines zweiten Kompressionspfades.

- **Gesperrt, solange gemessen wird**, wie beim Firmware-Update und aus
  demselben Grund: Ein Download hält die Hauptschleife für seine ganze Dauer
  an, die laufende Messung verlöre dabei Zeilen. Die Seite sagt das und
  verweist auf die Statusseite.

- **Löschen nur nach Anmeldung**, nur innerhalb von `/sessions` und nur mit
  geprüfter Sitzungs-ID. Ohne diese Prüfung wäre `../` ein gültiger Parameter
  gewesen.

Der Task-Watchdog wird während der Übertragung bedient. Ohne das greift er
mitten im Download, weil die Hauptschleife währenddessen nicht weiterläuft.

### Hinzugefügt: Hosttest der tar-Rahmung

`src/tar_writer.{h,cpp}` ist frei von Arduino, SD und Netzwerk und damit auf
dem Entwicklungsrechner prüfbar - dieselbe Trennung wie bei `curve_detector`
und `road_metrics`. `test/test_tar_writer/` erzeugt ein vollständiges Archiv
und lässt es vom echten `tar` lesen und auspacken.

Der Anlass ist die Prüfsumme im tar-Kopf: Sie zählt ihr eigenes Feld als acht
Leerzeichen. Wer sie über den fertigen Kopf bildet, bekommt eine Summe, die
kein `tar` annimmt - und das fiele erst am Handy im Auto auf, an einem Gerät,
an das man nach dem Einbau nicht mehr herankommt. Die Hosttests laufen damit
bei **70 von 70**.

## [1.5.42] - 2026-08-04

### Hinzugefügt: Aufzeichnung startet ohne Handgriff

Das Gerät hängt nach `STRECKENDATENBANK.md` an zündungsgeschalteter Spannung.
Damit ist die Versorgung selbst das Startsignal: Strom da bedeutet fahren,
Strom weg bedeutet Fahrtende. Sobald die SD-Karte bereit ist, beginnt die
Aufzeichnung von allein - keine Bewegungserkennung, keine Zündungslogik, kein
Zustandsautomat.

Der Start läuft über `requestLoggingStart()` und damit über dieselbe nicht
blockierende Startlogik wie der Webbefehl; pro Schleifenrunde wird höchstens
eine Datei geöffnet, die Webseite bleibt erreichbar. Er erfolgt genau einmal je
Gerätestart, sonst ließe sich eine laufende Aufzeichnung nicht mehr beenden:
`stop` und `/ride/stop` behalten das letzte Wort.

**Bewusst nicht an CAN gekoppelt.** Die OBD-Geschwindigkeit ist zwar die
verbindliche erste Quelle aller Auswertungen - über den Prüfstand belegt mit
174 gegenüber 128 Ereignissen allein mit GPS -, aber CAN ist für die
Systembereitschaft ausdrücklich optional. Hinge der Start an einer OBD-Antwort,
entstünde bei einem Busfehler gar keine Aufzeichnung statt einer ohne
OBD-Werte. Für ein eingebautes Gerät ist das der schlechteste Fall: ein
stiller Totalausfall statt einer unvollständigen Fahrt. GPS und OBD melden sich
an, sobald sie können; bis dahin tragen ihre Felder ihre eigenen
Gültigkeitsflags.

### Nicht umgesetzt: Sitzungen ohne Bewegung verwerfen

Geplant war, eine Sitzung beim Beenden zu verwerfen, wenn sie nie 5 km/h
gesehen hat - gegen Kleinstsitzungen vom Rangieren. Beim Umsetzen zeigte sich,
dass die Regel im Zielaufbau fast nie greifen würde: Mit
zündungsgeschalteter Spannung endet eine Fahrt durch Wegfall der Versorgung,
und dann läuft `stopLogging()` überhaupt nicht. Genau die Sitzungen, die
verworfen werden sollten, enden ebenso.

Der Filter gehört damit auf den Rechner, wo die Auswertung ohnehin liegt.
Kleinstsitzungen kosten dort nichts: Ohne 5 km/h entstehen weder Kurven noch
Schlaglöcher noch Abschnittsbewertungen, eine solche Sitzung enthält also
Rohzeilen und keine Ergebnisse.

## [1.5.41] - 2026-08-04

### Entfernt: Ballast für den Einbau ins Fahrzeug

Das Gerät soll künftig einmal eingebaut und nicht mehr angefasst werden. Drei
Baugruppen sind damit gegenstandslos geworden und fallen weg. Der Flashbedarf
sinkt von 93,6 auf **88,2 Prozent**, der freie Platz wächst von 83.486 auf
154.194 Byte - Raum für den Datenzugriff übers Handy, der als Nächstes kommt.

- **Die interaktive Testsuite `integration_tests`** war mit 59,3 kByte das
  größte Modul der ganzen Firmware, größer als der SD-Logger. Ihre Tests
  warten teilweise auf Eingaben an der seriellen Konsole - an einem Gerät
  hinter der Verkleidung ist niemand mehr, der etwas eingibt. Entfallen sind
  die Befehle `integration`, `stress`, `recovery`, `sdrecovery`, `quick` und
  `memory`.

  **`hardware` und `buffer` bleiben erhalten.** Ihre Funktionen - I²C-Scanner,
  BNO055-Prüfung, SD-Test mit sicheren Pins, Puffersicherheit - liegen in
  `main.cpp` und hingen nie an der Testsuite. Damit bleibt auch die Korrektur
  aus 1.5.40 wirksam, die dem SD-Test die Karte wieder freigibt.

- **Die Fahrzeugdaten-Erkennung `vehicle_data_discovery`** (16,6 kByte) hat
  ihren Zweck erfüllt: Die unterstützten PIDs des Fahrzeugs sind bekannt und
  bestätigt. Die dreiphasige Suche mit Listen-Only-Mitschnitt und
  Unterstützungsblock-Scan ist damit Ballast. Entfallen sind die Befehle
  `discover begin`, `discover status`, `discover mark` und `discover end`.

  Der CAN-Empfangspfad wird dadurch einfacher: Das Empfangsbudget von 32
  Frames je Schleifendurchlauf galt allein dem passiven Mitschnitt, der
  Betriebsmodus des MCP2515 ist nur noch der normale, und die Sonderfälle in
  `start`, `stop`, `obd on`, `obd off` sowie `/ride/start` und `/ride/stop`
  fallen weg.

- **Die Beifahrerseite `/fahrbahn`** ist mit `STRECKENDATENBANK.md` entfallen.
  Urteile entstehen jetzt nach der Fahrt am Rechner: Was während der Fahrt
  Aufmerksamkeit kostet, kommt nicht in die Datenbank. Die Notengrenzen
  `ROAD_DRIVEABILITY_RMS_GOOD_MPS2` und `_BAD_MPS2` sind mit den 149 Urteilen
  zweier Fahrtage bereits bestimmt und bleiben unberührt. Die Ereignisse
  `FAHRBAHN_URTEIL` und `FAHRBAHN_BELAGSWECHSEL` entstehen nicht mehr;
  Bestandsdaten behalten sie.

Die Abschnittsbewertung selbst, das Ereignis `ABSCHNITT` und alle Messgrößen
bleiben unverändert. Entfernt ist die Eingabe des Urteils, nicht die Messung.

## [1.5.40] - 2026-08-03

### Behoben
- **Der Hardware-Test machte die SD-Karte bis zum Neustart unbrauchbar.**
  `testSDWithSafePins()` greift roh auf den SPI-Bus zu und schaltet MOSI und
  SCK zwischenzeitlich auf `INPUT_PULLUP`. Hielt der `SDLogger` die Karte
  parallel eingebunden, zog ihm das den Bus unter den Füßen weg: CMD0
  meldete danach `0xFF`, die automatische Wiedererkennung kam nicht mehr
  durch, und der Test riet zum Prüfen des MISO-Kabels - während die Karte in
  Wahrheit einwandfrei war und nach einem Neustart sofort wieder lief.

  Der Test gibt die Karte jetzt ausdrücklich frei (`sdLogger.end()`) und
  bindet sie danach wieder ein, mit sichtbarer Rückmeldung. Eine laufende
  Messung hat Vorrang: Dann findet der SD-Test gar nicht erst statt, statt
  sie stillschweigend zu unterbrechen.

  Damit entfällt auch der Doppel-Mount, der zuvor „Test-Datei:
  Schreibfehler" erzeugte. Die in 1.5.39 dafür eingeführte Sonderbehandlung
  ist wieder entfernt - ein Schreibfehler an dieser Stelle ist jetzt wieder
  ein echter Befund.

  Am Gerät bestätigt: CMD0 `0x01` statt `0xFF`, „Test-Datei: OK" statt
  Schreibfehler, und `SD: bereit` in allen Statuszeilen nach dem Test.

## [1.5.39] - 2026-08-03

### Behoben: irreführende Meldungen
- **„SD-Karte nicht gefunden" behauptete zu viel.** `SD.begin()` erledigt
  Karteninitialisierung und Mount in einem Aufruf und unterscheidet die Fälle
  nicht. Am 03.08.2026 kostete das Zeit: Eine Karte antwortete auf CMD0
  nachweislich mit `0x01`, war also elektrisch erreichbar, scheiterte aber an
  der Initialisierungssequenz - und die Firmware meldete, es stecke keine
  Karte. Die Meldung nennt jetzt alle drei möglichen Ursachen und verweist
  auf `hardware`, das die Fälle über die CMD0-Rohantwort trennt.
- **Der Buffer-Test meldete einen Fehler, den es nie gab.**
  `safePrintf()` liefert die Zahl geschriebener Zeichen und `-1` bei
  drohender Truncation. Test 2 prüfte `!safePrintf(...)`; weil `-1` wahr ist,
  meldete er genau dann FEHLER, wenn der Überlaufschutz korrekt gegriffen
  hatte. Beide Tests werten den Rückgabewert jetzt als `int` aus, Test 2
  prüft zusätzlich die Nullterminierung.
- **`SAFE_SPRINTF` maß die Größe eines Zeigers.** Das Makro verwendete
  `sizeof(buf)`; mit einem `char*` sind das vier Byte statt der tatsächlichen
  Puffergröße. Im Stack-Buffer-Test erschien deshalb „needed 16, got 4",
  obwohl 100 Byte bereitstanden. Die Makros nehmen jetzt über
  `safeBufferSize()` ausschließlich Arrays an - ein Zeiger führt zum
  Übersetzungsfehler statt zu einer stillschweigend falschen Grenze.
- **„Test-Datei: Schreibfehler" im Hardware-Test** entstand durch doppelte
  Einbindung: Der Test bindet die Karte mit eigenen Parametern ein, während
  der `SDLogger` sie unter `/sd` hält. Der Test erkennt das jetzt und meldet
  es als übersprungen statt als Kartenfehler. Betroffen war eine Karte, die
  zuvor 70 Minuten fehlerfrei aufgezeichnet hatte.

### Entfernt: OLED
- **Das Display ist ausgebaut, die Firmware überwacht es nicht mehr.** Der
  Ping auf 0x3C/0x3D, die Aussetzerzählung, die Statusausgaben und die
  Adressmakros sind entfallen, ebenso die ungenutzte Hilfsfunktion
  `i2cDeviceResponds()`. Ohne diese Änderung hätte die Firmware den
  Sollzustand dauerhaft als Fehler gemeldet.
- **Der Integrationstest „I2C-Bus Recovery" wäre ab sofort immer
  fehlgeschlagen.** Er verlangte nach einer provozierten Busstörung, dass
  BNO055 *und* OLED wieder antworten. Der Nachweis stützt sich jetzt allein
  auf den BNO055.
- Damit entfällt der zweite unabhängige I²C-Teilnehmer. Fällt der BNO055
  aus, ist am Bus nicht mehr zu unterscheiden, ob Sensor oder Bus die
  Ursache ist - genau diese Unterscheidung hatte die Fehlersuche im Juli 2026
  vorangebracht. Für eine Fehlersuche genügt es, vorübergehend ein
  beliebiges I²C-Gerät anzuhängen; der Scanner findet es ohne Codeänderung.

## [1.5.38] - 2026-08-03

### Geändert
- **Der teuerste Flush-Schritt ist in zwei Schritte geteilt.** Schritt 1
  erledigte bislang als einziger zwei teure Operationen: den Puffer-Write
  der Sensordatei und deren Dateiflush. Er war damit der einzige Schritt
  oberhalb der 100-ms-Slotbreite und kostete je Umlauf eine Sensorstichprobe.

  Messgrundlage ist `20260803_082231_D0606CF2`: Über 3.180 Schritte lag der
  Mittelwert bei 19,1 ms, das Maximum aber bei 111 ms, und ein einzelner
  SD-Vorgang darin erreichte 79 ms. Bei 319 Umläufen standen 396 verfehlte
  Sensorslots - gut einer je Umlauf. Über die Sitzungen vom 02. und
  03.08.2026 entspricht das 13 bis 16 verfehlten Slots je Minute oder 2,2
  bis 2,7 Prozent der Stichproben.

  Der Puffer-Write bleibt Schritt 1, der Dateiflush wird Schritt 2.
  `SD_FLUSH_STEP_COUNT` steigt auf 11, `SD_FLUSH_STEP_INTERVAL_MS` sinkt auf
  450 ms; der volle Umlauf bleibt damit bei 4,95 s und der gleichmäßige
  Abstand der Statuszeilen erhalten. Die Sicherungslücke wächst nur um ein
  Schrittintervall, weil die Zeilen bereits nach dem Write in der Datei
  stehen.

  **Am Gerät noch nicht bestätigt.** Erwartet wird eine größte Blockade um
  79 statt 111 ms; die nächste Fahrt muss zeigen, ob `FlushMaxMs` unter die
  Slotbreite fällt und `SensorMissedSlots` entsprechend sinkt.

### Hinzugefügt
- **Die Metadatendatei führt `FlushMaxStep`**, den Index des Flush-Schritts,
  der den Höchstwert erzeugt hat. Bisher war `FlushMaxMs` keinem Schritt
  zuzuordnen; welcher Schritt die 111 ms verursachte, musste über die
  Zeitverteilung erschlossen werden. Bleibt der Höchstwert über 100 ms,
  benennt das Feld unmittelbar den nächsten Kandidaten.

  Das Schema steigt dadurch auf `1.5.38-quality-v12`. Die Spalte steht hinter
  `FlushStalls`, alle übrigen bleiben unverändert; `tools/export_geojson.py`
  zeigt sie als `flushMaxSchritt` im Sitzungskopf und liefert für ältere
  Sitzungen `null`.

## [1.5.37] - 2026-08-03

### Behoben
- **Die Temperatur-PIDs 0x46 und 0x5C erkennen den Sentinel 0xFF.** Beide
  rechnen `A - 40` über ein einzelnes Byte; 0xFF bedeutet dort „Wert derzeit
  nicht verfügbar“ und ergab umgerechnet 215 °C. Die Firmware verbuchte
  diesen Wert bis 1.5.36 als gültige Messung mit `AmbientValid = 1`.

  Beleg: In den sieben Discovery-Sitzungen vom 29./30.07.2026 trugen 28 von
  rund 530 Antworten auf 0x46 genau dieses Byte. In
  `20260729_170946_05A4DB46` sind das 45 von 2438 als gültig markierten
  Zeilen, also 1,8 Prozent; die echten Werte lagen dort zwischen 31 und
  34 °C. Ein Mittelwert über die Außentemperatur wurde dadurch um mehrere
  Grad zu hoch. Im Rohframe ist der Unterschied unmittelbar sichtbar:
  `03 41 46 49` sind 33 °C, `03 41 46 FF` waren die 215.

  Die Antwort zählt weiterhin als Antwort - `decoded` bleibt true, damit
  ECU-Erreichbarkeit, Antwortzähler und Timeoutbuchführung unverändert
  stimmen. Nur das Gültigkeitsflag bleibt aus, und ohne frischen Zeitstempel
  altert ein zuvor gemessener Wert über `CAN_OBD_SLOW_VALUE_MAX_AGE_MS`
  regulär aus.

  Die Prüfung gilt ausdrücklich nur für diese beiden PIDs. Bei
  Geschwindigkeit (0x0D) und Drosselstellung (0x11) ist 0xFF ein regulärer
  Wert - 255 km/h beziehungsweise 100 Prozent - und lässt sich nicht von
  einer Fehlanzeige unterscheiden.

  **Bestandsdaten sind nicht korrigiert.** Wer die Außentemperatur aus
  Sitzungen bis 1.5.36 auswertet, filtert 215,0 vorher heraus. Kein
  Auswertewerkzeug im Repository verwendet diese Spalten bisher.

## [1.5.36] - 2026-08-02

### Geändert
- **Die Fahrbarkeitsnote ist gegen menschliches Urteil kalibriert.** Grundlage
  sind 135 Beifahrerurteile auf 119 Abschnitten einer 52-Kilometer-Fahrt
  (`20260802_113909_C8F1C2F3`). Vollständige Auswertung in
  `testdata/AUSWERTUNG_2026-08-02_FAHRBARKEIT.md`.

  Gemessene Effektivwerte je Wertungsstufe, im Median: sehr gut 0,663,
  gut 0,843, mäßig 1,373, schlecht 1,316 m/s². Die beste Trennung zwischen
  „geht noch“ und „geht nicht mehr“ liegt bei 1,08 m/s² und ordnet 100 von
  119 Abschnitten richtig zu, also 84 Prozent.

  `ROAD_DRIVEABILITY_RMS_GOOD_MPS2` steht deshalb auf 0,55 statt 0,5 und
  `ROAD_DRIVEABILITY_RMS_BAD_MPS2` auf 1,60 statt 3,0. Die Note 50 liegt
  damit genau auf der Trennschwelle. Die vier Stufen erhalten die Noten
  89 / 72 / 22 / 28; mit den geschätzten Grenzen lagen sie zwischen 65 und 93,
  die Skala unterschied also kaum. Über die 259 Abschnitte der Hauptfahrt
  ergibt sich ein Median von 64 bei 7 Prozent Note 0 und 2 Prozent Note 100 -
  weder Sättigung nach oben noch nach unten.
- Eine Geschwindigkeitsnormierung entfällt endgültig. Der rohe Effektivwert
  erreicht eine Rangkorrelation von 0,73 zur Wertungsstufe, die beste
  normierte Form 0,74; das rechtfertigt keine zusätzliche Rechnung.

### Hinzugefügt: Fahrbarkeit auf der Karte
- `tools/export_geojson.py` kennt die Ereignisse `ABSCHNITT`,
  `FAHRBAHN_URTEIL` und `FAHRBAHN_BELAGSWECHSEL`. Abschnitte erscheinen als
  breite eingefärbte Linie entlang der tatsächlich gefahrenen Spur, Urteile
  und Belagswechsel als Marker. Die Urteilsfarben sind bewusst andere
  Farbtöne als die der gemessenen Fahrbarkeit, damit auf der Karte
  unterscheidbar bleibt, was gemessen und was geurteilt wurde.
- **Die Note wird beim Export aus dem Effektivwert neu berechnet.** Die
  Firmware schreibt die Note ihres eigenen Standes ins Log; ohne
  Neuberechnung wären Aufzeichnungen vor und nach einer Nachkalibrierung
  nicht vergleichbar. Die Fahrt vom 02.08.2026 verteilte sich mit ihren
  aufgezeichneten Noten auf 181 zu 68 zu 10 zu 0 Abschnitte je Klasse, die
  Karte wäre also fast einfarbig gewesen; mit der Skala von 1.5.36 sind es
  84 zu 88 zu 45 zu 42. Der aufgezeichnete Wert bleibt als
  `noteAufzeichnung` erhalten. Die Grenzen liest das Skript aus
  `src/hardware_config.h`, statt sie zu duplizieren.
- Die Farbklassen der Fahrbarkeit tragen die Namen der Beifahrerstufen und
  liegen in der Mitte zwischen deren Notenmedianen: über 81 „sehr gut“, über
  47 „gut“, darunter „mäßig bis schlecht“. Die erste Fassung nannte alles
  unter 25 „nicht mehr zügig“ - dieselben Abschnitte hatte der Beifahrer
  jedoch 20 von 27 Mal mit „mäßig“ bewertet und nur 2 Mal mit „schlecht“.
  Die Messung war also richtig, ihre Beschriftung übertrieben.

  Stufe 3 und 4 bleiben zusammengefasst, weil sie über den Effektivwert nicht
  trennbar sind. Über die 52-Kilometer-Fahrt ergibt sich damit 26 Prozent
  „sehr gut“, 43 Prozent „gut“ und 31 Prozent „mäßig bis schlecht“.
- Der Export lässt sich auf einzelne Ebenen beschränken. `--ebenen` wählt aus
  `strecke`, `referenz`, `fahrbarkeit`, `kurve`, `schlagloch`,
  `beifahrerurteil`, `belagswechsel` und `sitzung`; `--nur-fahrbarkeit` ist
  die Kurzform für die Fahrbarkeitsansicht samt Urteilen und Sitzungskopf.

  Eine 52-Kilometer-Fahrt ergibt vollständig rund 2.500 Merkmale und 1,8 MB,
  womit geojson.io träge wird. Nur die Fahrbarkeit sind 395 Merkmale und
  574 kB, nur deren Linien 259 Merkmale und 529 kB. Den größten Anteil trägt
  die Streckenebene mit einem Merkmal je GPS-Segment. Der Dateiname nennt die
  Auswahl, damit mehrere Ansichten derselben Sitzung nebeneinander bestehen.
- Der Zeitversatz zwischen Geräte- und Sitzungszeit wird bevorzugt aus einem
  Abschnittsereignis bestimmt. Es entsteht im selben Schleifendurchlauf, in
  dem der Abschnitt seine Länge erreicht, und kennt daher keinen
  Schreibverzug - anders als ein Kurvenereignis, das erst nach bis zu zwei
  Sekunden Ruhefenster geschrieben wird. Das verbessert nebenbei die Lage der
  Kurvenbögen.

### Hinzugefügt: Plausibilitätsgrenze über die gefahrene Geschwindigkeit
- Wer schnell fährt, traut der Straße. Weil die Anregung mit der
  Geschwindigkeit steigt, stufte der Effektivwert allein schnell gefahrene
  Abschnitte zu schlecht ein: Zwei Abschnitte der Fahrt
  `20260802_113909_C8F1C2F3` erhielten die Note 0 bei 130 km/h im
  Durchschnitt.

  Ab `ROAD_DRIVEABILITY_FAST_KMH` = 100 km/h fällt die Note deshalb nicht
  unter `ROAD_DRIVEABILITY_FAST_MIN_NOTE` = 47, die Klassengrenze zwischen
  „gut“ und „mäßig“. Über die 52-Kilometer-Fahrt betrifft das 8 von 259
  Abschnitten; der Anteil „mäßig bis schlecht“ sinkt von 31 auf 27 Prozent.

- **Die Regel wirkt bewusst nur in einer Richtung.** Hohe Geschwindigkeit
  belegt eine tragfähige Fahrbahn, niedrige belegt nichts - sie kann ebenso
  am Verkehr liegen. Von 14 bewerteten Abschnitten über 100 km/h erhielt kein
  einziger das Urteil „mäßig“ oder schlechter, gegenüber 36 Prozent im Mittel
  aller Abschnitte.

- Geprüft und verworfen wurde die naheliegendere Variante, die Fahrdynamik
  aus der Horizontalbeschleunigung zu bewerten. Sie erreicht nur 0,04
  Rangkorrelation zur Wertungsstufe, und als Korrekturfaktor verschlechtert
  sie das Ergebnis von 0,74 auf 0,48. Bremsen, Beschleunigen und Kurven
  finden unabhängig von der Fahrbahnqualität statt. Auch die
  Querbeschleunigung trennt kaum: Über 3,0 m/s² sinkt der Anteil schlechter
  Urteile nur von 36 auf 26 Prozent.

- Datenlage: 14 Abschnitte. Die Grenze ist belegt, aber dünn.

### Am 03.08.2026 auf Kopfsteinpflaster bestätigt

Zwei Fahrten über 29,4 Kilometer mit 34 Urteilen und erstmals zehn
Belagswechselmarkern. Beide Sitzungen vollständig, `SDErrors=0`,
`SDDropped=0`, keine Integritätsdatei.

- **Die Grenzen bleiben unverändert.** Mit 149 statt 119 Urteilen liegt die
  Trennung zwischen „geht noch“ und „geht nicht mehr“ weiterhin bei
  1,08 m/s² und trifft 85 statt 84 Prozent. `ROAD_DRIVEABILITY_RMS_GOOD_MPS2`
  = 0,55 und `_BAD_MPS2` = 1,60 sind damit gegen zwei unabhängige Fahrtage
  belegt.
- **Die Skala ist jetzt durchgängig monoton:** Stufe 1 bei 0,701, Stufe 2 bei
  0,843, Stufe 3 bei 1,420 und Stufe 4 bei 1,583 m/s², jeweils im Median.
- **Der Belagswechselmarker funktioniert.** An den Übergängen aufs Pflaster
  springt der Effektivwert um 63 bis 79 Prozent (0,83 auf 1,35, 0,84 auf
  1,47, 0,85 auf 1,53). Die 200-Meter-Abschnitte trennen die Beläge sauber
  und vermischen sie nicht.

### Damit erledigt: die vermeintliche Grenze bei Stufe 4

Bis zum 02.08.2026 stand hier, Stufe 4 habe keinen höheren Effektivwert als
Stufe 3 (1,316 gegen 1,373), weil auf sehr schlechter Straße so langsam
gefahren werde, dass die Anregung wieder sinke.

**Das war kein Methodenproblem, sondern fehlende Daten.** Der Fahrt vom
02.08.2026 fehlte schlechter Belag; sie enthielt vier Abschnitte der Stufe 4.
Auf echtem Kopfsteinpflaster steigt der Effektivwert trotz niedrigen Tempos
bis 2,29 m/s², und Stufe 4 liegt mit 1,583 im Median klar über Stufe 3.

Die Datenlage bleibt mit sechs Abschnitten dünn, aber die Richtung stimmt.

### Am Gerät bestätigt
- 1.5.35 lief über 1 Stunde 50 Minuten und 81,3 Kilometer in drei Sitzungen.
  Alle Dateien vollständig, jede Zeilenzahl deckt sich mit dem Zähler der
  Metadatendatei, `SDErrors` und `SDDropped` bleiben null, keine
  Integritätsdatei. 402 Streckenabschnitte entstanden, in der Hauptfahrt 259
  bei 52,19 Kilometern - genau die erwartete Zahl.
- GPS lieferte wieder Positionen; die Streckenwerte sind belegt. Der Verdacht
  auf einen Wackelkontakt bleibt bestehen.
- `SensorMissedSlots` liegt bei 13,1 bis 15,9 je Minute gegenüber 12,2 am
  Vormittag, `FlushMaxMs` bei 112 bis 119 gegenüber 101. Beides passt zum
  gewachsenen Umfang: Die Ereignisdatei nimmt jetzt zusätzlich Abschnitte und
  Urteile auf, und die Hauptfahrt dauerte fast zwölfmal so lange wie die
  bisherigen Messungen. Der Anteil bleibt mit 2,2 bis 2,7 Prozent klein.

## [1.5.35] - 2026-08-02

### Behoben
- **Die Fahrbahnbewertung maß die Fahrweise statt der Straße.** Sie lief über
  die rohe Sensorachse `accelZ`. Im Fahrzeug liegt die Vertikale dort aber
  nicht: Gemessen an der Fahrt `20260802_095708_84FD688D` verteilt sich die
  Schwerkraft auf (8,60 / -4,23 / -1,85) m/s², die Z-Achse trägt also
  19 Prozent der Vertikalen. Die Korrelation zwischen `accelZ` und der
  tatsächlichen Vertikalen betrug **-0,06**.

  Folge: Quer- und Längsbeschleunigung gingen als Fahrbahnstöße durch. Die
  Straßenqualität lag in Kurven im Median bei 72,6 und außerhalb bei 92,1,
  und die Zahl der Stichproben über der Stoßschwelle war mit 838 gegenüber
  127 um das 6,6-fache überhöht. Drei von fünf Schlaglöchern lagen innerhalb
  eines Kurvenereignisses. Für den Zweck der Messung war das Ergebnis genau
  verkehrt herum: Je zügiger gefahren wurde, desto schlechter die Note.

  Die lineare Beschleunigung wird jetzt auf die Schwerkraftrichtung
  projiziert, nach demselben Verfahren wie die Drehrate seit 1.5.28. Das
  Übersprechen der Querbeschleunigung fällt damit von Faktor 2,59 auf 1,23.
  `SensorData` führt dafür `verticalAccel` und `verticalAccelValid`; ohne
  brauchbare Schwerkraftrichtung entsteht bewusst kein Messwert.

### Hinzugefügt
- **Fahrbarkeit je Streckenabschnitt.** Alle `ROAD_SECTION_LENGTH_M` = 200
  Meter entsteht eine Note von 0 bis 100 für die Frage, ob sich auf diesem
  Stück zügig fahren lässt. Bewertet wird ein fester Weg statt einer festen
  Zeit, damit ein Abschnitt unabhängig vom Tempo dasselbe Stück Straße
  beschreibt.

  200 Meter sind kurz genug für einen Belagswechsel und lang genug, dass die
  Spurwahl sich herausmittelt. Über kürzere Abschnitte wichen Hin- und
  Rückfahrt derselben Strecke um 27 Prozent voneinander ab, weil beide
  Richtungen verschiedene Fahrspuren benutzen. Anders als der Kurvenwinkel
  ist die Rauheit keine Eigenschaft der Straßengeometrie, sondern der
  befahrenen Spur.

  Der Abschnitt erscheint als Ereignis `ABSCHNITT` in der vorhandenen
  Ereignisdatei, nicht in einer zehnten Logdatei: Ein Flush-Schritt kostet
  bereits bis zu 101 ms und damit einen vollen Sensorslot. Die Note steht in
  `Schwere`, Weg und Geschwindigkeiten in den Spalten des Kurvenereignisses.
  `DetectionMode` trägt `SECTION`, `CompletionReason` trägt `LENGTH`.

  In der Wiedergabe der Fahrt vom 02.08.2026 entstehen 21 Abschnitte mit
  Noten zwischen 58 und 96. Die Korrelation der Note zum Kurvenanteil sinkt
  gegenüber der bisherigen Qualitätszahl von -0,71 auf -0,46.

### Geändert
- `ROADTEST_CSV_SCHEMA_VERSION` steht auf `1.5.35-quality-v11`. Die
  Spaltenbelegung ist unverändert; hinzu kommt allein der Ereignistyp
  `ABSCHNITT`.

### Bekannte Grenze
- **Belagsarten sind bei 10 Hz Abtastung nicht unterscheidbar.** Kopfstein-
  pflaster regt bei 30 km/h mit rund 83 Hz an, Großpflaster mit 42 Hz, Kies
  mit über 160 Hz; die Nyquist-Grenze liegt bei 5 Hz. `VibrationMetrics`
  führt `frequency` deshalb weiterhin als 0 und nicht etwa als Schätzwert.

  Messbar bleibt die Aufbaubewegung des Fahrzeugs bei 1 bis 2 Hz - und genau
  die entscheidet, ob zügiges Fahren angenehm bleibt. Der Effektivwert ist
  auch bei Unterabtastung ein brauchbares Energiemaß, weil Aliasing die
  Frequenz verfälscht, nicht die Gesamtenergie. Die Note beantwortet damit
  "wie gut lässt sich hier fahren", nicht "welcher Belag ist das".

### Hinzugefügt: Beifahrerseite `/fahrbahn`
- Vier Wertungsstufen von „sehr gut“ bis „schlecht“ und ein Knopf
  „Belagswechsel hier“. Jeder Druck schreibt ein Ereignis
  `FAHRBAHN_URTEIL` beziehungsweise `FAHRBAHN_BELAGSWECHSEL`; die Stufe steht
  zusätzlich in `Schwere`.

  Anlass ist die Natur der Zielgröße: Die Fahrbarkeit ist ein Urteil, keine
  physikalische Größe. Ob 1,42 m/s² noch zügiges Fahren zulassen, kann keine
  Messung beantworten. Erst die Wertepaare aus Beifahrerurteil und gemessenem
  Effektivwert machen aus den beiden Notengrenzen eine Messung statt einer
  Konvention. Das unterscheidet den Fall von der Kurvenerkennung, deren
  Referenzmarker nur bestätigten, was ohnehin objektiv vorlag.
- Die Seite zeigt den **zuletzt abgeschlossenen** Abschnitt mit Note,
  Effektivwert und Tempo sowie den Weg im laufenden Abschnitt. Bewusst nicht
  den laufenden Wert: Der abgeschlossene liegt bereits hinter dem Fahrzeug
  und kann das Urteil über die Strecke unter den Rädern nicht vorwegnehmen.
- Kein Seitenwechsel beim Drücken, damit ein Funkloch keine Wertung
  verschluckt; jeder Druck wird einzeln quittiert. Ohne laufende Aufzeichnung
  antworten die Endpunkte mit 409 statt stillschweigend zu verwerfen.
- Die Seite ist absichtlich nicht passwortgeschützt - der Beifahrer soll
  während der Fahrt keine Zugangsdaten eingeben. Sie kann ausschließlich
  Marker setzen.
- **Bewerten ist jederzeit aussetzbar.** Die Aufzeichnung hängt in keiner
  Weise an der Bewertung: `/fahrbahn` ruft ausschließlich `logEvent()`,
  Start und Ende bleiben allein bei `/ride/start` und `/ride/stop`.

  Damit eine Pause auch in der Auswertung folgenlos bleibt, trägt jedes
  Urteil die laufende Abschnittsnummer und den Weg im offenen Abschnitt mit.
  Ein Urteil gilt dadurch für **genau seinen Abschnitt** und nicht bis zum
  nächsten Marker. Ohne diese Zuordnung zöge eine Pause von zehn Minuten die
  letzte Wertung über zehn Minuten unbewertete Straße. Wer aussetzt,
  hinterlässt jetzt eine Lücke - und eine Lücke ist ehrlich, eine
  fortgeschriebene Wertung wäre es nicht.

  Das Ereignis `ABSCHNITT` führt dieselbe Nummer in seiner Beschreibung,
  `RoadSection` das Feld `number`. Der Zähler beginnt mit jeder Messung neu.
- Kosten: 5.028 Byte Flash, 93,6 statt 93,2 Prozent. Die 2026 in 1.5.33
  entfernte Seite `/curve-test` brauchte 13.868 Byte; ihr Muster mit Start-
  und Endmarkern passt hier nicht, weil die Abschnittsgrenzen bereits von der
  Firmware gesetzt werden.

### Noch offen
- Die Grenzen `ROAD_DRIVEABILITY_RMS_GOOD_MPS2` = 0,5 und
  `ROAD_DRIVEABILITY_RMS_BAD_MPS2` = 3,0 stammen aus einer einzigen Fahrt und
  sind gegen weitere nachzuziehen. Genau dafür ist `/fahrbahn` gedacht.
- Der GeoJSON-Export kennt den Ereignistyp `ABSCHNITT` noch nicht.
- Wirksamkeit am Gerät nicht bestätigt.

## [1.5.34] - 2026-08-01

### Am Gerät bestätigt

Drei Fahrten am 02.08.2026: eine Strecke hin und zurück über 304 Sekunden
sowie zwei Kreise auf dem Parkplatz. Alle drei Sitzungen vollständig, kein
SD-Abbruch, keine Integritätsdatei. Vollständige Auswertung in
`testdata/AUSWERTUNG_2026-08-02_1.5.34.md`.

- **`SESSION_END` ist belegt**, dreimal: 356,9 Grad bei 18,7 km/h, 327,2 Grad
  bei 23,0 km/h und 34,1 Grad bei 36,4 km/h. Die Werte sind in sich stimmig -
  57,9 Meter Bogenlänge auf 356,9 Grad ergeben 9,3 Meter Radius, und v²/r
  trifft die protokollierte Querbeschleunigung von 2,89 m/s² genau. Damit ist
  der letzte offene Punkt aus 1.5.29 erledigt.
- Abgleich der Wiedergabe gegen die Firmwareereignisse: **33 zu 33**, alle
  zugeordnet, 0,1 Grad mittlere Winkelabweichung.
- **Die Kurvenerkennung ist reproduzierbar.** Die Hauptfahrt lief hin und
  zurück über dieselbe Strecke. Acht gemeinsame Kurven, in beiden Richtungen
  erkannt, mit korrekt kippendem Vorzeichen: **1,0 Grad Winkelabweichung im
  Median, 3,4 Grad im Maximum**, relativ 2,0 Prozent. Radius weicht 7 Prozent
  ab, mittlere Querbeschleunigung 16 Prozent, Geschwindigkeit 6 Prozent. Dass
  die beiden letzteren stärker streuen, ist erwartet: Sie hängen an der
  gefahrenen Linie, die Netto-Kursänderung nicht. Diese Messung braucht keine
  von Hand markierten Referenzintervalle; Werkzeug ist
  `tools/vergleich_hin_rueck.py`.
- **`DetectionMode` ist keine Kurveneigenschaft.** In 4 von 8 Paaren steht
  `SHARP` für die Hin- und `LONG` für die Rückfahrt derselben Kurve, bei auf
  ein Grad gleichem Winkel. Das Feld beschreibt, welches Tor die Kurve
  geöffnet hat, und hängt an der Anfahrgeschwindigkeit. Es darf nicht als
  Kurveneigenschaft ausgewertet oder gegen einen Festwert geprüft werden.

### Nicht bestätigt

- **GPS hatte in keiner der drei Sitzungen einen Fix.** Über alle 1.887
  GPS-Zeilen gilt `Satellites=0`, `ValidFix=0`, `HDOP=99.99` und
  `RejectionReason=111`. Der Empfänger arbeitete - 90.364 NMEA-Zeichen, 2.440
  gültige Sätze, null Prüfsummenfehler -, er sah nur nichts. Folge:
  `StreckeKm 0.000`, alle Ereignisse ohne Position, keine Kartenauswertung
  möglich. Die Kurvenerkennung ist davon nicht betroffen, weil die
  Geschwindigkeit wie vorgesehen aus OBD kam; OBD lief mit 609 Anfragen und
  609 Antworten fehlerfrei. Noch am selben Tag kehrte der Empfang ohne
  Eingriff zurück. Die Firmware scheidet als Ursache aus: Sie sendet dem
  BN-880 nichts, `gps_manager.cpp` ist seit dem 30.07.2026 unverändert, und
  die Fix-LED des Moduls blieb dunkel. Verdacht ist ein Wackelkontakt an der
  aufgelöteten Keramikantenne oder der Versorgung; Einzelheiten im
  Auswertungsbericht.
- **Das Ziel von 1.5.30 für `LoopMaxMs` wurde verfehlt**: 251 statt deutlich
  unter 247 Millisekunden. Der Wert steht aber schon fünf Sekunden nach dem
  Messstart bei 222 ms, als `SDMaxMs` erst 44 ms beträgt - das ist der
  Sitzungsstart mit neun geöffneten Dateien, nicht der Flush. Der
  Dauerbetrieb liegt bei `LoopLastMs=2`.
- `SensorMissedSlots` sank von 19 auf 12,2 je Minute, wächst aber streng
  linear mit rund einer je fünf Sekunden - dem Takt eines vollen
  Flush-Umlaufs. Dazu passt `FlushMaxMs=101` für einen **einzelnen** Schritt:
  Er überschreitet die Slotbreite von 100 ms und kostet damit zwangsläufig
  eine Stichprobe. Die Aufteilung aus 1.5.30 hat den 235-ms-Block beseitigt;
  übrig bleibt die größte Einzeldatei knapp über der Slotbreite.
- Die Statuszeilen standen in gleichmäßigem Abstand von 5,00 bis 5,04
  Sekunden mit einem Ausreißer bei 5,22 Sekunden. Dieses Ziel aus 1.5.30 ist
  erfüllt.

### Behoben
- Die Größenprüfung aus 1.5.31 verlor ihre Grundlinie. Die Rückstellung der
  Byte-Zähler stand im gemeinsamen Rumpf hinter `FINALIZE` und lief damit
  erst, nachdem `openLogFile()` die Ausgangswerte einschließlich Kopfzeile
  gesetzt hatte. Die erwartete Größe fiel dadurch je Datei um die Kopfzeile
  zu niedrig aus, und ein Verlust in genau dieser Größenordnung wäre
  unentdeckt geblieben. Die Rückstellung steht jetzt in `PREPARE`, also vor
  dem Öffnen.
- Ein dauerhaft fehlendes OLED erzeugte seit 1.5.32 endlos Meldungen. Die
  Verzweigung hing an `oledDetectedAtLeastOnce`, das nie zurückfällt; zuvor
  schaltete `oledManager.end()` die Überwachung nach drei Fehlversuchen ab.
  Ohne Displaytreiber lief sie weiter: alle fünf Sekunden eine Warnung und
  ein im Prüfintervall hochlaufender Aussetzerzähler, der damit keine
  Aussetzer mehr zählte, sondern Sekunden. Die Meldung wird jetzt einmal
  abgesetzt und bis zur Rückkehr des Displays unterdrückt.
- `recoverySummaryWritten` wird beim Kartenfehler zurückgestellt. Sonst hätte
  ein zweiter Ausfall vor dem Schreiben des Fortsetzungsereignisses
  fälschlich `ZusammenfassungNachgetragen=ja` gemeldet.

### Entfernt
- Sechs verwaiste Zählvariablen des Kurventests in `WebManager` sowie ein
  leeres Formatargument in der seriellen Teststatuszeile.

## [1.5.33] - 2026-08-01

### Entfernt
- Die geführte Beifahrerseite `/curve-test` samt ihren sechs Routen, dem
  Seitenaufbau, der Zustandsverwaltung und dem Status-JSON. Ihr Zweck ist
  erfüllt: Am 31.07.2026 entstanden damit fünf Referenzfahrten mit zusammen
  55 markierten Kurven, die in `testdata/` liegen und als Festwerte in
  `test_wiedergabe_referenzfahrten` verankert sind. Diese Daten bleiben die
  Messgrundlage für Schwellwertänderungen der Kurvenerkennung.
- Der Parameter `flushImmediately` von `SDLogger::logEvent()` entfällt. Er
  existierte allein für die gepufferten Referenzmarker; alle verbliebenen
  Aufrufer sichern das Ereignis sofort.
- Der Gewinn beträgt **13.868 Byte Flash**: 1.219.138 statt 1.233.006 Byte,
  93,0 statt 94,1 Prozent der App-Partition.

### Hinweis
- Wird je wieder ein frischer Referenzsatz gebraucht, ist die Seite über die
  Versionsgeschichte wiederherstellbar. Die Ereignisse hießen
  `CURVE_TEST_START`, `CURVE_TEST_END`, `CURVE_REFERENCE_START` und
  `CURVE_REFERENCE_END`; der GeoJSON-Export wertet sie unverändert aus, damit
  die vorhandenen Fahrten weiter vollständig darstellbar bleiben.

## [1.5.32] - 2026-08-01

### Entfernt
- Der OLED-Treiber und die beiden Adafruit-Displaybibliotheken. Das Display
  wird nicht mehr genutzt; die Hardware bleibt verbaut. Entfallen sind
  `src/oled_manager.{h,cpp}` mit 674 Zeilen, 64 Aufrufstellen in `main.cpp`,
  die OLED-Schritte der Integrationstests, die Displayzeile der Statusseite
  und der serielle Befehlszweig des Displaytests.
- Damit entfiel auch die Bootstatus-Anzeige samt ihrer Zustandsverwaltung
  (`updateBootStatusDisplay`, `bootStatusActive`, `BOOT_READY_HOLD_TIME`).
  Sie hatte nach dem Ausbau nur noch Variablen gesetzt, die niemand mehr
  liest. Die Prüfung `requiredHardwareReady()` bleibt erhalten und erscheint
  jetzt im seriellen `diag` als "Pflichthardware bereit"; ohne das wäre mit
  dem Display auch diese Information verschwunden.
- Der Gewinn beträgt **19.592 Byte Flash**: 1.233.006 statt 1.252.598 Byte,
  94,1 statt 95,6 Prozent der App-Partition. Der RAM-Bedarf sinkt um 48 Byte.

### Geändert
- Das Display bleibt bewusst gesteckt und wird weiterhin per I²C angepingt.
  Als zweiter unabhängiger Busteilnehmer ist es das wertvollste Mittel zur
  Unterscheidung von Sensor- und Busfehlern: Fällt es zusammen mit dem BNO055
  aus, liegt die Ursache am Bus oder an der Versorgung. Genau diese
  Unterscheidung brachte die Fehlersuche im Juli 2026 weiter. Der Zähler der
  I²C-Aussetzer, der Scanner und die Prüfung im Bus-Recovery-Test bleiben
  deshalb unverändert.
- `OLED_ADDRESS_A` und `OLED_ADDRESS_B` bleiben in `hardware_config.h`, jetzt
  als Beschreibung dessen, was am Bus zu erwarten ist. `SCREEN_WIDTH`,
  `SCREEN_HEIGHT`, `OLED_REQUIRED` und `I2C_DISPLAY_SPEED` entfallen.

## [1.5.31] - 2026-08-01

### Hinzugefügt
- Größenprüfung aller Logdateien am Sitzungsende. Nach dem Schließen wird
  jede Datei frisch geöffnet, sodass ihre Größe aus dem Verzeichniseintrag
  der Karte stammt und nicht aus dem Arbeitsspeicher, und gegen die Summe der
  geschriebenen Bytes gestellt.

  Anlass ist `20260801_114252_B9D1628B`: Dort verbuchte die Firmware 3534
  erfolgreiche GPS-Schreibvorgänge, die Datei enthielt aber nur 1122 Zeilen
  und endete nach 227 von 709 Sekunden. 68 Prozent der GPS-Spur fehlen. Alle
  acht übrigen Dateien der Sitzung decken die volle Dauer ab, sämtliche
  Fehlerzähler standen auf null, und `SDWrites` von 18.125 entspricht der
  Annahme, dass alle 3534 Zeilen geschrieben wurden - gegenüber 15.734 bei
  nur den vorhandenen 1122. Der Verlust liegt damit unterhalb der Firmware;
  ihre eigene Buchführung war in sich schlüssig.

  Bei einer Abweichung entsteht `road_integritaet_<Sitzung>.csv` mit einer
  Zeile je betroffener Datei aus Sitzung, Dateiart, Byte auf der Karte und
  Byte erwartet. Die Sitzung gilt als unterbrochen, und der persistente
  Verweis trägt die Ursache `SESSION_FILES_TRUNCATED`, abgegrenzt vom
  bisherigen `SESSION_FINALIZATION_INCOMPLETE` für fehlende Abschlussdateien.
  Ein automatischer Wiederanlauf entsteht bewusst nicht: Der Nutzer hat die
  Messung beendet.

  Die Prüfung erkennt den Verlust, sie verhindert ihn nicht. Sie macht aus
  einem stillen Datenverlust einen protokollierten Befund.

## [1.5.30] - 2026-08-01

### Behoben
- Nach einem Kartenfehler bekommt die abgebrochene Sitzung ihre
  Zusammenfassung nachgetragen, sobald die Karte wieder beschreibbar ist. Die
  Kennzahlen stehen zu diesem Zeitpunkt noch vollständig im Arbeitsspeicher,
  nur die Karte war beim Abbruch nicht mehr erreichbar. In
  `20260731_160032_3D732AD1` fehlten dadurch Strecke, Kurvenzahl und
  Durchschnittsqualität einer 2,5-km-Fahrt mit 16 Kurven, obwohl alle
  Rohdateien vollständig vorlagen. Eine bereits vorhandene Datei wird nicht
  überschrieben. Nach Reset oder Spannungsverlust entsteht bewusst keine
  Zusammenfassung: Der Sitzungsmarker überlebt zwar im NVS, die Kennzahlen
  nicht, und eine Datei voller Nullwerte wäre keine Messung. Das
  Fortsetzungsereignis `SD_RECOVERY_CONTINUATION` führt dafür das neue Feld
  `ZusammenfassungNachgetragen` mit `ja` oder `nein`.
- Statuszeilen erscheinen wieder in gleichmäßigem Fünf-Sekunden-Abstand. Die
  Kombination aus Fünf-Sekunden-Takt und einem zusätzlichen 5000-ms-Gatter
  blockierte sich bei Jitter gegenseitig; in den Messdaten vom 31.07.2026
  standen deshalb Statuszeilen mal nach fünf, mal erst nach zehn Sekunden.
  Den Takt gibt jetzt allein der Schrittzähler des Flush-Durchlaufs vor.

### Geändert
- Der SD-Flush wird in Einzelschritte zerlegt. Bisher sicherte ein Durchlauf
  alle neun Dateien am Stück und blockierte die Hauptschleife dabei gemessen
  bis zu 235 Millisekunden; die verfehlten Sensorstichproben lagen
  deterministisch bei 2290 und 2590 Millisekunden modulo 5000, also genau im
  Takt dieses Durchlaufs. Neu sichert `SDLogger::flushStep()` je Aufruf
  höchstens eine Datei oder die Statuszeile. Zehn Schritte im Abstand von
  500 Millisekunden ergeben einen vollen Umlauf in fünf Sekunden, also
  dieselbe Sicherungsfrequenz wie zuvor. `SDLogger::flush()` sichert
  weiterhin alles in einem Zug und bleibt den Sitzungsenden vorbehalten.
- Der Aufruf steht bewusst hinter der Sensorabtastung, damit ein Schritt in
  die Lücke zwischen zwei Abtastzeitpunkten fällt statt auf einen.
- `FlushLastMs`, `FlushMaxMs` und `FlushCycles` in der Metadatendatei
  beziehen sich jetzt auf den einzelnen Schritt statt auf den vollen
  Durchlauf. Genau diese Zahl beschreibt die Blockade der Hauptschleife;
  `LoopMaxMs` und `SensorMissedSlots` behalten ihre Bedeutung unverändert.
- Noch am Gerät zu bestätigen: erwartet werden ein deutlicher Rückgang von
  `LoopMaxMs` gegenüber den zuletzt gemessenen 247 Millisekunden und von
  `SensorMissedSlots` gegenüber 19 je Minute.

## [1.5.29] - 2026-07-31

Auswertung der fünf Beifahrer-Referenzfahrten vom 31.07.2026 mit zusammen
55 von Hand markierten Referenzkurven.

### Am Gerät bestätigt

Vier Fahrten am 01.08.2026 über zusammen 21 Minuten und 9,1 Kilometer,
ohne Beifahrer-Referenzmarker. Alle vier Sitzungen vollständig, kein
SD-Abbruch.

- Zeitabdeckung der Ereignisse: 97 Prozent im Median, 1 von 66 Ereignissen
  unter 60 Prozent. Mit 1.5.28 waren es 67 Prozent im Median und 37 Prozent
  darunter. Kein einziges Ereignis lief mehr in die Höchstdauer; mit 1.5.28
  gab es zwei solche Ausreißer über volle 60 Sekunden.
- Kein Kurvenereignis im Stand. In der Hauptfahrt entstand während der
  einminütigen Standphase nach dem Messstart nichts; das erste Ereignis kam
  bei 116,6 Sekunden.
- Abgleich der Wiedergabe gegen die Firmwareereignisse mit
  `tools/vergleich_kurvenwiedergabe.cpp`: 65 von 66 Ereignissen zugeordnet,
  drei der vier Fahrten ereignisgenau bei 0,0 bis 0,1 Grad Winkelabweichung.
  Die Umsetzung auf dem ESP32 ist damit belegt und nicht nur plausibel.
- Die einzige Abweichung ist ein Grenzfall der Ruhefensterschwelle: Der
  Netto-Winkel erreichte 2,020 Grad bei einer Fensterlaufzeit von 1999
  Millisekunden, gegen eine Schwelle von 2,0 Grad und 2000 Millisekunden.
  Die Firmware trennte dort in zwei Kurven mit 92,8 und 98,2 Grad, die
  Wiedergabe führte eine mit 191,4 Grad durch. Die Ursache liegt in der
  Prüfmethode: die Gierrate wird mit drei Nachkommastellen protokolliert,
  und über die rund zwanzig Stichproben des Fensters summiert sich diese
  Rundung auf etwa 0,01 Grad.
- Noch offen: `SESSION_END` wurde nicht beobachtet. Der Abschlussgrund greift
  nur, wenn die Aufzeichnung gestoppt wird, während das Fahrzeug über
  5 km/h fährt und eine Kurve läuft. Auf dem Parkplatz wurde vor dem Stopp
  angehalten; das Geschwindigkeitsgatter schloss die Kurve dann bereits als
  `QUIET` ab. Der Pfad bleibt durch drei Hosttests abgedeckt.

### Behoben
- Eine beim Stoppen noch laufende Kurve wird abgeschlossen und protokolliert,
  statt verloren zu gehen. Das Ruhefenster über `CURVE_END_QUIET_MS` läuft
  nach dem Messende nicht mehr ab; in `20260731_152148_452707FB` fehlte
  dadurch eine eindeutige Kurve mit 44 Grad Kursänderung bei 55 bis 72 km/h
  und Drehraten bis 18 Grad/s vollständig. Der neue Abschlussgrund heißt
  `SESSION_END`. Die Spaltenbelegung der CSV-Dateien bleibt unverändert;
  `ROADTEST_CSV_SCHEMA_VERSION` steht deshalb weiter auf
  `1.5.28-quality-v10`. In der Spalte `CompletionReason` kann jetzt der
  zusätzliche Wert `SESSION_END` stehen.
- Die Kurvenerkennung wird beim Messstart zurückgestellt. Ein vor dem Start
  begonnener Verlauf lief bisher in die Sitzung hinein; in
  `20260731_152148_452707FB` begann das erste Ereignis 2,2 Sekunden vor dem
  Sitzungsbeginn.

### Geändert
- Das Ruhefenster der Kurvenerkennung bewertet die Netto-Kursänderung im
  Fenster statt einzelner Stichproben. `CURVE_LONG_MIN_RATE_DPS` von
  0,8 Grad/s liegt unter dem Rauschband der Gierrate unter Fahrt - auf
  nachgewiesen gerader Strecke wurden bis zu 1,5 Grad/s gemessen. Jede
  Rauschspitze in Kurvenrichtung setzte das Fenster zurück, sodass Ereignisse
  über mehrere Kurven hinweg zusammenliefen. Neu gilt eine Kurve nur dann als
  fortgesetzt, wenn sie im Ruhefenster mindestens
  `CURVE_QUIET_MIN_NET_ANGLE_DEG` netto weiterdreht; über die Fensterlänge
  entspricht das einer mittleren Drehrate von 1,0 Grad/s. Symmetrisches
  Rauschen hebt sich dabei auf, eine echte langgezogene Kurve nicht.
- Wirkung auf den Wiedergabestand der fünf Referenzfahrten: Der Anteil der
  Ereignisdauer, der von echten Drehstichproben belegt ist, steigt von 67 auf
  91 Prozent. Ereignisse unter 60 Prozent Abdeckung fallen von 47 auf 7. Die
  Zahl der Ereignisse steigt von 130 auf 154, weil zusammengelaufene
  Ereignisse sich wieder auftrennen. Radius, Dauer und Mittelwerte beschreiben
  damit wieder eine einzelne Kurve.
- Trefferquote gegen die 55 Referenzkurven: 52 statt 53. Die drei fehlenden
  Referenzen drehen netto 5,7, 6,4 und 10,0 Grad und liegen damit unter
  `CURVE_MIN_EVENT_ANGLE_DEG`; zwei davon wurden zuvor nur getroffen, weil
  zusammengelaufene Nachbarereignisse ihr Zeitfenster zufällig überlappten.
  Der Rückgang ist keine Verschlechterung der Erkennung, sondern der Wegfall
  zufälliger Überlappungen.

### Hinzugefügt
- Mindestfahrweg `CURVE_MIN_EVENT_DISTANCE_M` von 10 Metern je Ereignis. Die
  Geschwindigkeitsfreigabe allein reichte nicht: GPS-Drift im Stand wird mit
  bis zu 8 km/h als gültig ausgewiesen. In `20260731_160627_085A6E1C`
  entstanden daraus zwei Kurven mit 2,5 und 3,4 Meter Radius über 1,6 und
  1,8 Meter Fahrweg, während das Gerät in der Hand gedreht wurde. Der
  Streckenzähler filtert dasselbe Rauschen seit jeher über eine
  Mindestsegmentlänge.
- Mindest-Querbeschleunigung `CURVE_MIN_EVENT_LATERAL_ACCEL` von 0,4 m/s² je
  Ereignis. Maßgeblich ist die Querbeschleunigung, nicht der Radius: Ein
  Bogen mit 500 Meter Radius bei 90 km/h wurde von der Beifahrerseite
  ausdrücklich als Kurve markiert und trägt 1,5 m/s²; ein Autobahnbogen mit
  1450 Meter Radius bei 116 km/h dagegen 0,4 und ist keine gefahrene Kurve.
- Beide Werte wurden gegen die 55 markierten Referenzkurven gewählt und
  lassen dort keine einzige entfallen; die schwächste markierte Kurve fuhr
  10,8 Meter Weg und 0,45 m/s². Über alle neun Fahrten des 31.07.2026 fallen
  die Ereignisse unter 12 km/h von 13 auf 2. Alle vier bekannten Fehlalarme
  aus GPS-Drift im Stand und beim Rangieren entfallen; das Wendemanöver mit
  190,7 Grad über 27 Meter bei 14 km/h bleibt korrekt erhalten.
- Hosttest `test_wiedergabe_referenzfahrten`: spielt alle fünf
  Referenzfahrten durch die Erkennung und schreibt Trefferzahl,
  Ereigniszahl und die Zahl der Ereignisse mit geringer Zeitabdeckung als
  Festwerte fest. Die Geschwindigkeit wird dabei wie in `main.cpp` gewählt -
  zuerst OBD, sonst GPS, sonst unbekannt. Mit dieser Reihenfolge gibt die
  Wiedergabe die Firmwareläufe des 31.07.2026 ereignisgenau wieder: 174 zu
  174 Ereignissen bei 0,1 Grad mittlerer Winkelabweichung und einem
  Startversatz von 7 bis 21 Millisekunden in den Fahrten mit belastbarer
  Zeitbasis. Allein mit der GPS-Geschwindigkeit waren es 128 statt 174. Der
  Prüfstand bildet die Firmware damit belastbar ab; Parameteränderungen
  lassen sich vorab am Entwicklungsrechner messen, und die Ereignisse einer
  neuen Fahrt lassen sich gegen die Wiedergabe derselben Rohdaten stellen.
- Hosttests für das entrauschte Ruhefenster, den Mindestfahrweg und die
  Mindest-Querbeschleunigung mit je einem Fall knapp über und knapp unter der
  Schwelle sowie für `CurveDetector::finish()`. 49 Hosttests insgesamt.

## [Unveröffentlicht]

### Geändert
- Beifahrer-Kurventest auf eine Bildschirmseite ohne Scrollen umgestellt. Der
  Rahmen ist `100dvh` hoch, die Knopffläche nimmt den Rest ein und die Knöpfe
  wachsen mit der verfügbaren Höhe. Selbst im Querformat bleiben rund 79 Pixel
  je Knopfreihe, deutlich über der Mindestgröße für Tippziele.
- Alle sechs Referenzarten stehen in fester 2x3-Anordnung: linke Spalte
  Linkskurven, rechte Spalte Rechtskurven. Erledigte Arten verschwinden nicht
  mehr, sondern werden nur deaktiviert. Zuvor verschoben sich dadurch die
  Positionen der übrigen Knöpfe während der Fahrt - der Beifahrer greift aber
  nach Position, nicht nach Text.
- Jeder Knopf trägt seinen eigenen Fortschritt, der Kopf die Gesamtzahl
  erfasster Referenzen.
- Während eines laufenden Intervalls bleibt unverändert nur der Endknopf
  sichtbar; ein versehentlicher zweiter Start ist ausgeschlossen.
- Erklärtexte auf das Nötige gekürzt. Die Seite belegt dadurch 3.660 Byte
  weniger Flash: 1.248.338 statt 1.251.998 Byte, 95,2 statt 95,5 Prozent.

## [1.5.28] - 2026-07-31

### Hinzugefügt
- Einbaulagenunabhängige Fahrzeug-Drehrate aus der Projektion des vollständigen
  Gyroskopvektors auf die gemessene Schwerkraftrichtung. Die rohe Sensor-Z-Achse
  wird nicht als Fahrzeug-Hochachse vorausgesetzt.
- Strukturierte Kurvenereignisse mit Beginn, Ende, Dauer, Richtung,
  S-Kurven-Gruppierung, Gyro- und Heading-Winkel, Weg, mittlerer und maximaler
  Geschwindigkeit, Drehrate, Radius, Querbeschleunigung, Erkennungsmodus,
  Abschlussgrund und Qualitätsflags.
- Geführte Beifahrerseite unter `/curve-test` für eine gerade Referenz, vier
  weite Kurven, vier normale beziehungsweise enge Kurven und drei S-Kurven.
  Alle noch offenen Kurvenarten sind unabhängig von der Streckenreihenfolge
  auswählbar. Erledigte Richtungen verschwinden; nach dem Start eines
  Referenzintervalls bleibt nur dessen großer Endknopf sichtbar.
- Sensor-CSV mit Schwerkraftvektor sowie projizierter Drehrate und
  Gültigkeitsflag, damit das neue Verfahren unabhängig nachgerechnet werden
  kann.

### Geändert
- Die Kurvenerkennung verwendet bei vollständig kalibriertem Gyroskop die
  projizierte Drehrate; bei fehlender Projektion bleibt der relative
  Heading-Verlauf als gekennzeichneter Rückfall verfügbar.
- Langgezogene und schnelle Kurven nutzen denselben kontinuierlichen
  Ereignisakkumulator. Ein Richtungswechsel ab fünf Grad erzeugt getrennte
  Kurvenhälften mit gemeinsamer Gruppennummer.
- `KurvenProKm` wird aus den tatsächlich protokollierten Kurven und der
  qualitätsgeprüften GPS-Strecke gefüllt.
- Referenzmarker der Beifahrerseite werden ohne sofortigen Einzel-Flush
  geschrieben und beim regulären Sammelflush gesichert. Dadurch erzeugt der
  Knopfdruck selbst keine zusätzliche SD-Pause.
- Die abgeschlossene Recovery-Webseite `/acceptance` einschließlich ihrer
  Zustandslogik wurde entfernt. Firmwareversion und CSV-Schema steigen auf
  1.5.28 beziehungsweise `1.5.28-quality-v10`.

### Testfokus
- Die heutige Vergleichsfahrt misst zuerst Erkennungsgenauigkeit und
  Vollständigkeit gegen explizite Beifahrerintervalle. Die
  Straßenqualitätsbewertung wird erst anhand dieser neuen Fahrt weiter
  parametriert, damit Kurven- und Oberflächenmodell nicht gleichzeitig
  verändert werden.

### Testbarkeit
- Die Auswertelogik der Kurvenerkennung liegt jetzt in
  `src/curve_detector.{h,cpp}` und ist frei von Arduino-, Sensor- und
  Zeitabhängigkeiten. `BNO055Manager` bildet nur noch die Sensorwerte auf die
  Eingangsstruktur ab; das Verhalten ist unverändert. Zeitangaben verwenden
  `uint32_t` statt `unsigned long`, damit Host und Ziel dasselbe
  Überlaufverhalten zeigen.
- Neue PlatformIO-Umgebung `native` mit 15 Hosttests
  (`pio test -e native`, Laufzeit unter einer Sekunde). Sie belegen keinen
  Firmware-Flash — relevant, weil der Messbuild bei 95,5 Prozent steht und die
  On-Device-Suite allein 55,6 kByte belegt.
- Abgedeckt sind scharfe und langgezogene Kurven, S-Kurvengruppierung,
  Gyro-Vorrang und Kurs-Rückfall, Zeitlücken, Rückstellung, die Grenzen von
  `CURVE_MIN_EVENT_ANGLE_DEG` sowie zwei Regressionsschutztests gegen
  Ereignisse im Stillstand — der Fall, der in `20260730_071913_9B018397`
  eine Kurve mit 625 Grad erzeugte.
- Enthalten ist eine Wiedergabe der realen Fahrt `20260730_095603_A41A2450`
  aus `testdata/`. Damit sind Parameteränderungen der Kurvenerkennung ohne
  Ausfahrt bewertbar.
- Fünf Mutationstests belegen, dass die Suite Parameteränderungen tatsächlich
  erkennt. Ein erster Durchlauf tat das nicht; Schwellwertgrenzfälle und ein
  fester Erwartungswert für die Wiedergabe wurden daraufhin ergänzt.
- Ebenso ausgelagert: Vibrationsanalyse, Straßenqualität und
  Schlaglocherkennung nach `src/road_metrics.{h,cpp}`. Die bisherigen
  Zahlenliterale der Schlaglocherkennung und der Geschwindigkeitsnormierung
  stehen jetzt als benannte Konstanten in `hardware_config.h`
  (`ROAD_POTHOLE_WINDOW_MS`, `ROAD_POTHOLE_REARM_MS`,
  `ROAD_QUALITY_REFERENCE_SPEED_KMH`, `ROAD_QUALITY_SPEED_FACTOR_MIN/MAX`).
- 22 weitere Hosttests in `test/test_road_metrics/`. Zwei davon halten die
  gemessene Einschränkung fest, dass die Geschwindigkeitsnormierung nur
  zwischen 20 und 42,9 km/h wirkt und oberhalb davon konstant ist. Die
  Auslegung selbst bleibt unverändert; die Grenze ist damit sichtbar und
  eine Änderung erzwingt eine bewusste Entscheidung.
- Wiedergaben aus `testdata/`: der belegte Standlauf
  `20260730_101000_5901D247` muss null Qualitätswerte und null Schlaglöcher
  erzeugen, die Fahrt `20260730_095603_A41A2450` ist auf 5.856
  Qualitätswerte, Mittel 84,2, Minimum 33,2 und sieben Schlaglöcher
  festgeschrieben.
- Elf Mutationstests über beide Einheiten belegen, dass die Suite
  Parameteränderungen erkennt.
- Die Portierung ist als wortgetreu nachgewiesen: Die Rümpfe von
  `detectCurve`, `addCurveSample`, `completeCurve`, `analyzeVibration`,
  `calculateRoadQuality`, `getSmoothness` und `detectPothole` stimmen nach
  Normalisierung von Namen und Typen mit dem Stand vor der Auslagerung
  überein.
- `[platformio] default_envs = lolin_s3_mini` ergänzt, damit `pio run` nicht
  versucht, die Testumgebung zu linken.
- Kosten: 580 Byte Flash für die zusätzliche Indirektion, RAM um 8 Byte
  geringer.

## [1.5.27] - 2026-07-30

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
- Langgezogene Kurven benötigen nicht mehr zwingend eine momentane Drehrate
  von mindestens 8 Grad pro Sekunde. Zusätzlich zum schnellen Einstieg
  startet ein Kurvenkandidat nach mindestens 8 Grad Netto-Kursänderung und
  20 Metern Fahrt innerhalb von zehn Sekunden. Die Schwellen wurden gegen die
  per OSRM auf das Straßennetz abgeglichene Überlandfahrt
  `20260730_095603_A41A2450` festgelegt.
- Einzelne kleine Gegenbewegungen des NDOF-Headings verwerfen einen
  Langkurvenkandidaten nicht mehr. Ein Richtungswechsel ab fünf Grad trennt
  dagegen die beiden Hälften einer S-Kurve.
- Kurven enden erst nach zwei Sekunden unter 0,8 Grad pro Sekunde. Eine
  Abtastlücke über 500 ms verwirft den offenen Zustand, statt aus zeitlich
  weit getrennten Headings ein Ereignis abzuleiten.
- Ein SD-Schreibfehler verwirft den noch nicht geschriebenen Sensorpuffer
  nicht mehr sofort. Nach erfolgreichem Wiedereinbinden wird der vollständige
  Puffer in einer separaten `road_sensor_recovered_*.csv` innerhalb der
  Ursprungssitzung gesichert.
- Nach einem vorübergehenden SD-Ausfall während einer laufenden Messung startet
  automatisch eine neue Sitzung. Das Ereignis
  `SD_RECOVERY_CONTINUATION` verknüpft sie mit Ursprungssitzung, Fehlergrund,
  Ausfalldauer und Anzahl geretteter Pufferzeilen.
- Aktive und fehlerhaft beendete Sitzungen werden im NVS markiert. Nach Reset
  oder Spannungsverlust wird der Verweis beim nächsten manuellen Messstart
  protokolliert; eine Messung startet nach einem Neustart nicht selbsttätig.
- Ein SD-Fehler während eines ausdrücklich angeforderten Stopps löst keine
  unerwartete automatische Folgemessung aus.

### Geändert
- Weboberfläche, OLED, NVS-Metadaten und CSV-Spalten sind auf den aktiven
  NDOF-Modus abgestimmt.
- I²C-Operationen werden nach 20 ms abgebrochen.
- Die fest verdrahtete BNO055-Adresse ist verbindlich `0x29`; der automatische
  Fallback auf `0x28` entfällt.
- Die periodische Gesundheitsprüfung liest die Chip-ID nur noch einmal pro
  Fünf-Sekunden-Zyklus.
- Sitzungsnamen erhalten einen Zufallsanteil; wiederholte FAT-
  Verzeichnissuchen nach freien Dateinamen entfallen.
- Firmwareversion auf 1.5.27 und CSV-Schemakennung auf
  `1.5.27-quality-v9` angehoben.
- Die Statusseite zeigt Sicherung und Fortsetzung nach einem SD-Fehler
  ausdrücklich an.
- Der interaktive SD-Hotplug-Test startet eine echte Messung, hält mindestens
  eine Sensorzeile im RAM und bestätigt nach Entfernen und Wiedereinsetzen der
  Karte sowohl Pufferrettung als auch automatische Folgesitzung. Er ist
  einzeln über `sdrecovery` aufrufbar.

### Dokumentation
- Hardwarebeschreibung auf den tatsächlichen LOLIN S3 Mini mit 4 MB Flash
  umgestellt.
- Verbindliche GPIO-Belegung mit dem Firmwarecode abgeglichen.
- Versorgung des PZSMOCN-SD-Moduls auf 3,3 V korrigiert.
- Unbenutzte BNO055- und GPS-Pins sowie CAN-Aufbau und Versorgung eindeutig
  dokumentiert.
- Unsichere pauschale Angaben zu CAN-Versorgung und Busterminierung entfernt.

### Grundlage
- Die Kartenzuordnung der Fahrt `20260730_095603_A41A2450` erreichte über elf
  überlappende OSRM-Abschnitte eine mittlere Matching-Konfidenz von 0,978.
  Von 37 kartenseitig erkannten Kurvenabschnitten lagen 13 ohne zeitnahes
  Firmwareereignis; besonders betroffen waren 9 bis 23 Sekunden lange Kurven
  mit nur etwa 1,3 bis 4,0 Grad pro Sekunde.
- Die abgebrochene Sitzung `20260730_093525_F94C8878` endete ohne
  `END`-Metadaten. Die folgende Sitzung startete ohne Neustart bereits mit
  einem SD-Fehler und 48 verworfenen Zeilen. Damit ist ein SD-Schreibfehler
  beim Fünf-Sekunden-Sensorflush als Abbruchursache belegt.

## [1.5.26] - 2026-07-30

### Behoben
- Doppelte Taktung von Sensor- und GPS-Aufzeichnung beseitigt. Hauptschleife
  und `SDLogger` besaßen zwei unabhängige Gatter derselben Periode, die
  gegeneinander schwebten. Verworfene Datensätze wurden dem Aufrufer als
  Erfolg gemeldet und blieben in der Diagnose unsichtbar: In
  `20260730_071913_9B018397` fehlten 183 GPS- und 41 Sensorzeilen, gemeldet
  wurden vier verfehlte GPS-Slots. Das zunächst halbierte Schutzgatter
  verwarf im Kontrolllauf `20260730_101000_5901D247` weiterhin 21 GPS- und
  72 Sensorzeilen. Sensor- und GPS-Logger besitzen deshalb kein eigenes
  Zeitgatter mehr; die phasenerhaltende Hauptschleife ist alleiniger
  Taktgeber.
- Straßenqualität und Fahrbahnereignisse sind an eine nachgewiesene
  Fahrzeuggeschwindigkeit gebunden. Der bisherige Stillstandszweig war
  unerreichbar, weil der Aufrufer bei ungültiger GPS-Geschwindigkeit `-1`
  übergab, GPS-Geschwindigkeit aber erst ab 6 km/h als gültig gilt. Im
  belegten Stillstand entstanden dadurch 106 von 265 Qualitätswerten unter
  99,0 sowie drei Schlagloch- und ein Kurvenereignis.
- Die Spalte `Schwere` enthält bei Schlaglöchern die gemessene
  Stoßbeschleunigung und bei Kurven den aufsummierten Winkel. Allgemeine
  Systemereignisse behalten den neutralen Wert null.
- Die starre 180-Grad-Grenze für Kurven entfällt. Sie hätte reale
  Kreisverkehre und Wendemanöver verworfen; Kursdrift im Stand wird bereits
  durch den verbindlichen Bewegungsnachweis verhindert.

### Geändert
- Firmwareversion auf 1.5.26 und CSV-Schemakennung auf
  `1.5.26-quality-v8` angehoben. Die Metadatendatei
  enthält zusätzlich `FlushLastMs`, `FlushMaxMs`, `FlushTotalMs`,
  `FlushCycles` und `FlushStalls`.
- Der Sammelflush über alle offenen Dateien wird zusätzlich zu den
  Einzelmessungen je Datei als ein Vorgang gemessen. Ohne diese
  Aggregatmessung war eine Hauptschleifenpause nicht zuordenbar: 1.5.24
  zeigte 225 ms Schleifenmaximum bei nur 67 ms größter Einzeloperation.
- Straßenqualität wird im Stillstand nicht mehr geschrieben. Ein
  Rückgabewert kleiner null bedeutet „kein Messwert“ und erzeugt keine
  CSV-Zeile.
- Neue verbindliche Geschwindigkeitsauflösung in der Hauptschleife: frische
  OBD-Geschwindigkeit, dann gefilterte GPS-Geschwindigkeit, sonst unbekannt.
  Unbekannt wird als negativer Wert weitergegeben, nicht als null.

### Bestätigt
- Der Kontrolllauf `20260730_101000_5901D247` bestätigte den vollständigen
  ECU-Wiederanlauf mit 1.5.25 sowie die Geschwindigkeitsfreigabe: Im Stand
  entstanden keine Straßenqualitäts- oder Fahrbahnereigniszeilen.
- Die vollständige Sensor- und GPS-Aufzeichnung ohne zweites Loggergatter
  muss noch mit 1.5.26 am Gerät bestätigt werden.

## [1.5.25] - 2026-07-30

### Behoben
- Die Abnahmeseite behandelt den nach dem Ausschaltmarker beobachteten
  ECU-Ausfall jetzt als dauerhaft erreichten Meilenstein. Nach dem erneuten
  Einschalten und erfolgreicher Wiedererkennung springt die Anzeige nicht
  länger fälschlich zu „ECU-Ausfall abwarten“ zurück.
- Der ECU-Ausfall wird relativ zum Zählerstand beim Motor-Aus-Marker
  ausgewertet. Ein früherer transienter ECU-Ausfall kann den Schritt daher
  nicht vorzeitig erfüllen.

### Bestätigt
- Der Wiederanlauflauf `20260730_101000_5901D247` erkannte erste Zündung,
  ersten Motorstart, ECU-Ausfall, erneute Zündung und zweiten Motorstart
  vollständig.

## [1.5.24] - 2026-07-30

### Behoben
- `NewFix` und `FixSequence` folgen jetzt der eindeutigen GNSS-Zeitepoche.
  Mehrere RMC-/GGA-Commits derselben Empfängerepoche erzeugen keinen
  zusätzlichen Positionsfix mehr.
- Der zweite Motorstart des Recovery-Kontrolltests wird aus einer frischen
  OBD-Drehzahl ab 300 U/min automatisch bestätigt, falls der zugehörige
  Browser-POST ausbleibt.
- Sensor- und GPS-Zeitpläne behalten nach einer kurzen Verzögerung ihre
  Sollphase, statt die Verzögerung dauerhaft in alle Folgeintervalle zu
  übernehmen.

### Geändert
- Der Sensor-Schreibpuffer umfasst 8 KiB und wird regulär über das vorhandene
  Fünf-Sekunden-Sicherungsintervall geleert. Der frühere 512-Byte-Puffer
  verursachte bereits nach wenigen 10-Hz-Zeilen einen synchronen SD-Zugriff.
- GPS-UART-Zeichen werden in jeder Hauptschleife verarbeitet; nur der
  Qualitäts-Snapshot bleibt auf 200 ms begrenzt. Hardware- und
  Software-RX-Puffer wurden auf 2 beziehungsweise 4 KiB vergrößert.
- Ein Stall zählt ab 250 statt 1.000 ms. Das CSV-Schema wurde auf
  `1.5.24-quality-v7` angehoben.

### Diagnose
- Laufzeit-, Web- und SD-Diagnose protokollieren neben Letzt- und Maximalwert
  nun auch Messanzahl und aufsummierte Dauer.
- Jeder relevante SD-Schreib- und Flush-Zugriff wird zeitlich erfasst.
- `SensorMissedSlots` und `GPSMissedSlots` zählen verfehlte Solltermine
  unabhängig von der allgemeinen Stallgrenze.
- Die Abnahmeseite protokolliert angenommene und abgelehnte Webaktionen. Nach
  fünf Sekunden ohne HTTP-Bestätigung wird der Aktionsknopf wieder bedienbar.

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

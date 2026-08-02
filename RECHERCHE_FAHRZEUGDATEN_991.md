# Recherche: Fahrdynamikdaten aus dem 991 Carrera S

| Feld | Angabe |
|---|---|
| Fahrzeug | Porsche 911 Carrera S, Typ 991.1, Baujahr 2012, PDK |
| Anlass | Der verbaute Sport Chrono zeigt Fahrdynamikwerte an, die ROADTEST bisher nicht erfasst |
| Stand | 02.08.2026, reine Schreibtischrecherche, nichts am Fahrzeug erprobt |

## Ergebnis in einem Satz

Der Sport Chrono selbst gibt nichts heraus, aber die Sensordaten dahinter -
**Lenkwinkel, vier Raddrehzahlen, vier Bremsdrücke, Querbeschleunigung,
Gang** - liegen am **Gateway-Stecker unter der Lenksäule** an, und der
Zugang ist durch AiM Technologies für genau dieses Fahrzeug dokumentiert.

## 1. Der Sport Chrono ist keine Datenquelle

Der Sport Chrono Plus im 991.1 ist Anzeige und Stoppuhr, kein Logger. Er
speichert nichts Exportierbares und besitzt keine Schnittstelle. Die
App-gestützte Aufzeichnung (Porsche Track Precision) kam erst mit dem 991.2
ab 2016.

Er ist selbst nur Mithörer der Fahrwerkssensorik. Wer an dieselben Werte
will, muss dieselbe Quelle anzapfen - nicht den Chrono.

## 2. Der OBD-Stecker scheidet für passives Mithören aus (belegt)

Das mussten wir nicht vermuten. Jede Discovery-Sitzung beginnt mit
60 Sekunden echtem MCP2515-Listen-Only bei offenem Filter. Ergebnis an zehn
Terminen zwischen dem 28.07. und 30.07.2026, ausnahmslos:

```
PASSIVE_END;FRAMES_0;UNIQUE_11BIT_IDS_0;EXTENDED_FRAMES_0
```

Über alle 34 aufgezeichneten Sitzungen erscheinen ausschließlich `7E8` und
`7E9`, also Antworten auf unsere eigenen Anfragen. Das Zentralgateway leitet
keinerlei Broadcast-Verkehr an die Diagnosebuchse weiter.

## 3. Der Gateway-Stecker führt die vollständigen Daten

AiM Technologies dokumentiert für das Protokoll `991-911` den Anschluss am
Gateway-Stecker. Die Angaben stammen aus deren offizieller
Anschlussanleitung (Release 1.05).

**Lage:** Fahrerseite, **unter der Lenksäule, aber oberhalb des
Sicherungskastens**. Der Sicherungskasten selbst liegt im Fußraum links.

**Anschluss:**

| Steckerpin | Aderfarbe | Funktion |
|---|---|---|
| **A20** | orange/rot, verdrillt | CAN High |
| **A10** | orange/braun, verdrillt | CAN Low |

**Ausdrückliche Warnung von AiM:** Am Gateway-Stecker liegen **mehrere
orange/braune Adern** an. Die richtige ist nur über das verdrillte Paar mit
der orange/roten Ader zu identifizieren.

Bestätigend nennen mehrere Foren als Alternativstelle das verdrillte Paar an
den Sicherungskästen in den Fußräumen; für den 992 wird dort von einem
erfolgreichen VBOX-Einbau mit Abgriffklemme ohne Aufschneiden berichtet.

### Verfügbare Kanäle über den Gateway-Stecker

Aus der AiM-Kanalliste des Protokolls `991-911`, gekürzt auf das für uns
Interessante:

| Kanal | Bedeutung |
|---|---|
| `PSH_WHEEL_FL/FR/RL/RR` | **vier einzelne Raddrehzahlen** |
| `PSH_STEER_POS` | **Lenkwinkel** |
| `PSH_STEER_SPD` | **Lenkgeschwindigkeit** |
| `PSH_ACC_LAT` | **Querbeschleunigung** |
| `PSH_BRAKE_PR1..4` | **vier Bremsdrücke**, dazu `PSH_BRAKE_SW` und `PSH_BRAKE_P` |
| `PSH_GEAR`, `PSH_GEAR_INFO` | eingelegter Gang |
| `PSH_MODE_TYPE` | gewählter Fahrmodus (Sport, Sport Plus) |
| `PSH_SW_SUSP` | Fahrwerksschalter |
| `PSH_PPS` | Pedalstellung |
| `PSH_RPM`, `PSH_VEH_SPEED` | Drehzahl, Fahrzeuggeschwindigkeit |

Dazu Drehmoment-, Öl-, Getriebetemperatur- und Leistungskanäle.

**Radschlupf** ist damit rechnerisch zugänglich (Differenz der vier
Raddrehzahlen) - der einzige Kanal, der für die Belagserkennung
möglicherweise mehr trägt als unser BNO055.

## 4. Der OBD-Weg mit herstellerspezifischen Anfragen

AiM listet für das Protokoll `991-981_OBDII` ebenfalls `STEER_ANGLE`,
`BRAKE_PRESS` und die vier Raddrehzahlen - **und dieser Weg gilt
ausdrücklich nur für Fahrzeuge mit Automatik oder PDK**, was auf unser
Fahrzeug zutrifft.

Da am OBD-Stecker nachweislich nichts gesendet wird, können diese Werte nur
über **aktive, herstellerspezifische Anfragen** kommen, vermutlich UDS
Service 22. Zwei harte Einwände:

1. Die Firmware erlaubt bewusst nur die Positivliste aus Service 01
   (`CANReader::requestOBDPid()`). Proprietäre Anfragen an ein
   Porsche-Gateway wären eine bewusste Aufweichung dieser Sicherheitsregel.
2. Bei höchstens zwei Anfragen pro Sekunde - ebenfalls eine bewusste
   Grenze - bekämen wir Lenkwinkel in Zeitlupe. Für Fahrdynamik wertlos.

Die konkreten Anfrage-PIDs sind zudem nirgends öffentlich dokumentiert; AiM
gibt sie nicht preis.

## 5. Dekodierung: Anhaltspunkte aus der Nachbargeneration

Für den 991.1 sind keine Frame-Belegungen öffentlich. Für den **997.2**, die
unmittelbare Vorgängergeneration mit verwandter Elektrik, sind sie
reverse-engineert:

| Größe | CAN-ID | Belegung |
|---|---|---|
| Lenkwinkel | 194 (`0xC2`) | Byte 0-1 little endian, vorzeichenbehaftet, Faktor 0,045; Byte 2-3 Änderungsrate |
| Raddrehzahlen | 586 (`0x24A`) | je 2 Byte little endian: VL, VR, HL, HR, Faktor 1/100 |
| Drehzahl | 578 (`0x242`) | `((Byte3 * 256) + Byte2) / 4`, Drosselklappe in Byte 5 / 255 |
| Fahrzeuggeschwindigkeit | 330 (`0x14A`) | `((Byte3 * 256) + Byte2) / 100` |

Für den noch älteren 996 sind zusätzlich Querbeschleunigung (`0x1A0`) und
Gierrate (`0x4A1`, 9 Bit, Faktor 0,0021326 rad/s) dokumentiert.

**Diese Werte sind Ausgangspunkte, keine Wahrheit.** Sie müssen am 991 neu
verifiziert werden.

## 6. Passt das zu unserer Hardware?

| Frage | Antwort |
|---|---|
| Bitrate | Der Antriebs-CAN läuft mit **500 kbit/s** (997-Quelle). Unser MCP2515 ist fest darauf verdrahtet - passt. |
| Betriebsart | `CANReader::configurePassiveCapture()` schaltet echtes Listen-Only mit offenem Filter. **Existiert bereits und ist erprobt.** Kein ACK, keine Rückwirkung auf den Bus. |
| Aufzeichnung | `road_can_*.csv` schreibt ID, DLC und acht Datenbytes. **Existiert bereits.** |
| Datenrate | Ein voller Fahrwerks-CAN sendet mehrere Tausend Frames pro Sekunde. Unsere Kette ist auf gut 2 Frames/s ausgelegt. **Das ist der eigentliche Engpass.** |

## 7. Bewertung und Empfehlung

**Machbar, aber der Nutzen ist begrenzt.** Querbeschleunigung und Gierrate -
die Kerngrößen des Chrono - misst der BNO055 bereits selbst, mit 10 Hz und
gegen 135 Beifahrerurteile kalibriert. Exklusiv wären Lenkwinkel, Bremsdruck
und Radschlupf.

Für die aktuelle Frage "wo lässt sich zügig fahren" trägt davon wenig bei.
Radschlupf wäre für die Belagserkennung interessant, aber genau dort ist
ohnehin die Abtastrate der Flaschenhals, nicht die Datenquelle.

Dem steht gegenüber: Eingriff in den Kabelbaum eines 991 hinter der
Verkleidung, Reverse Engineering unbeschrifteter Frames, und ein
Datenratenproblem, das unsere Aufzeichnungskette so nicht bewältigt.

**Empfehlung: nicht jetzt.** Die Recherche ist damit abgeschlossen und
dokumentiert; der Weg ist bekannt, falls sich die Frage später anders
stellt.

### Falls doch, dann in dieser Reihenfolge

1. **Erst messen, nicht löten.** Gateway-Stecker aufsuchen, orange/rot und
   orange/braun am verdrillten Paar identifizieren, mit Abgriffklemme ohne
   Aufschneiden anschließen. `discover begin` liefert 60 Sekunden
   Listen-Only - wenn dort Frames erscheinen, ist der Zugang bestätigt.
2. **Frame-Inventur:** Welche IDs mit welcher Rate? Erst danach entscheidet
   sich, ob ein Filter nötig ist, um die Aufzeichnungskette nicht zu
   überfahren.
3. **Dekodierung als PC-Arbeit.** Unsere Logs sind dafür das ideale
   Werkzeug: Der Lenkwinkel muss mit der BNO-Gierrate korrelieren, die
   Raddrehzahlen mit der OBD-Geschwindigkeit. Damit lassen sich Kandidaten
   ohne Rätselraten bestätigen.
4. **Erst dann** entscheiden, ob überhaupt etwas dauerhaft in die Firmware
   wandert.

## Quellen

- [AiM Infotech: Porsche 991-981 OBDII und 991-911 ECU, Release 1.05](https://www.aimtechnologies.com/aim-support/stockecu/5_Porsche_991-981_OBDII+991-911_105_eng.pdf) - Gateway-Pinbelegung und vollständige Kanalliste
- [Rennlist: CAN PIDs for 997.2](https://rennlist.com/forums/997-forum/1206692-can-pids-for-997-2-a.html) - Dekodierung Lenkwinkel, Raddrehzahlen, Drehzahl
- [Rennlist: CAN bus tap in location](https://rennlist.com/forums/992-gt3-and-gt2rs-forum/1278338-can-bus-tap-in-location.html) - Abgriff am Fußraum-Sicherungskasten, verdrilltes Paar
- [911uk: CAN bus Message data](https://911uk.com/porsche/can-bus-message-data.122053/) - 996-Belegungen für Querbeschleunigung und Gierrate
- [Autosport Labs: Porsche 987.2 / 997.2 CAN bus data](https://forum.autosportlabs.com/viewtopic.php?t=5616) - Bitrate des Antriebs-CAN
- [planetkris: Porsche 718 Cayman CAN BUS Track Day Data](https://planetkris.com/porsche-718-cayman-can-bus-track-day-data/) - Vorgehen beim Reverse Engineering mit SavvyCAN
- Eigene Messungen: zehn Listen-Only-Mitschnitte am OBD-Stecker, alle mit
  null Frames, in `testdata/archiv/*/road_event_*.csv`

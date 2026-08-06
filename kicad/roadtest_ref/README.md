# ROADTEST – Referenzschaltplan

Erzeugter KiCad-Schaltplan der **logischen Verdrahtung** des aktuellen
Aufbaus. Quellen sind `HARDWARE.md`, `schematic.md` und
`src/hardware_config.h` – nicht der Lochrasteraufbau selbst.

```text
roadtest.kicad_sch   Schaltplan, A3, Symbole zusätzlich eingebettet
ROADTEST.kicad_sym   Symbolbibliothek der Modulsymbole
sym-lib-table        bindet sie projektlokal ein
roadtest.kicad_pro   Projektdatei
gen_roadtest_sch.py  erzeugt alles davon neu
```

Geprüft mit KiCad 10.0.5:

- `kicad-cli sch erc` meldet **0 Fehler und 0 Warnungen**.
- Die exportierte Netzliste deckt sich Pin für Pin mit der verbindlichen
  Pinliste aus `schematic.md`. Unverbunden bleiben genau die dort als
  unbenutzt geführten Anschlüsse: BNO055 `3Vo`, `RST`, `INT`, `PS0`, `PS1`
  sowie BN-880 `SCL` und `SDA`.

## Verhältnis zu den anderen KiCad-Ordnern

`Roadtest_v2/` und `Roadtest_v3/` sind eigenständige, von Hand gepflegte
Projekte mit Leiterplatte. Dieser Ordner ist davon unabhängig und ersetzt sie
nicht: Er hält den dokumentierten Sollzustand der Verdrahtung fest und lässt
sich nach jeder Änderung an der Pinbelegung neu erzeugen.

## Warum Kastensymbole

Die Baugruppen sind fertige Module, keine diskreten ICs. Ein Symbol mit
beschrifteten Modulpins bildet ab, was tatsächlich verdrahtet wird. Ein
nachgezeichneter MCP2515 mit Quarz und Transceiver wäre eine Erfindung – das
Modul wird als Ganzes gekauft und angeschlossen.

## Abweichung von `schematic.md`

`schematic.md` zeigt weiterhin das SSD1306-OLED. Es ist seit dem 03.08.2026
ausgebaut und hier **nicht** gezeichnet. Der BNO055 ist damit der einzige
I²C-Teilnehmer.

## Neu erzeugen

```bash
python3 kicad/roadtest_ref/gen_roadtest_sch.py   # aus dem Projektwurzelverzeichnis
```

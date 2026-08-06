#!/usr/bin/env python3
"""Erzeugt kicad/roadtest_ref/ aus der verbindlichen Pinbelegung.

Quellen: HARDWARE.md, schematic.md, src/hardware_config.h.

Die Modulsymbole sind bewusst Kastensymbole mit beschrifteten Pins - die
Baugruppen sind fertige Module, keine diskreten ICs. Es entstehen:

    ROADTEST.kicad_sym   Symbolbibliothek
    sym-lib-table        bindet sie projektlokal ein
    roadtest.kicad_sch   Schaltplan (Symbole zusaetzlich eingebettet)
    roadtest.kicad_pro   Projektdatei

Alles liegt auf dem 1,27-mm-Raster; Breiten sind Vielfache von 2,54 mm und
Ursprungspunkte Vielfache von 2,54 mm, damit KiCad keine Endpunkte neben dem
Verbindungsraster meldet.
"""

import uuid, datetime, os, textwrap

U = lambda: str(uuid.uuid4())
ROOT = U()
LIB = "ROADTEST"
OUT = "kicad/roadtest_ref"
FONT = "(effects (font (size 1.27 1.27)))"
G = 2.54

SYMS = {
    "ESP32S3": dict(ref="U", val="LOLIN S3 Mini (ESP32-S3, 4 MB Flash, 2 MB PSRAM)",
                    w=15*G, pins=[
        ("5V",   "1", "power_in",  "L"), ("3V3", "2", "power_out", "L"),
        ("GND",  "3", "power_in",  "L"),
        ("GPIO1","4","bidirectional","R"), ("GPIO2","5","input","R"),
        ("GPIO3","6","output","R"),        ("GPIO4","7","output","R"),
        ("GPIO5","8","output","R"),        ("GPIO6","9","input","R"),
        ("GPIO7","10","output","R"),       ("GPIO8","11","bidirectional","R"),
        ("GPIO9","12","output","R"),       ("GPIO11","13","input","R"),
        ("GPIO13","14","output","R"),      ("GPIO15","15","output","R"),
        ("GPIO16","16","input","R"),
    ]),
    "BNO055": dict(ref="U", val="Adafruit BNO055 Breakout (NDOF, Adresse 0x29)",
                   w=15*G, pins=[
        ("VIN","1","power_in","L"), ("GND","2","power_in","L"),
        ("SDA","3","bidirectional","R"), ("SCL","4","input","R"),
        ("3Vo","5","power_out","R"), ("RST","6","input","R"),
        ("INT","7","output","R"), ("PS0","8","input","R"), ("PS1","9","input","R"),
    ]),
    "BN880": dict(ref="U", val="Beitian BN-880 GPS (NMEA, 9600 Baud)", w=15*G, pins=[
        ("VCC","1","power_in","L"), ("GND","2","power_in","L"),
        ("TX","3","output","R"), ("RX","4","input","R"),
        ("SCL","5","bidirectional","R"), ("SDA","6","bidirectional","R"),
    ]),
    "SDCARD": dict(ref="U", val="PZSMOCN Micro-SD-Modul (SPI, 3,3 V)", w=15*G, pins=[
        ("3.3V","1","power_in","L"), ("GND","2","power_in","L"),
        ("CS","3","input","R"), ("MOSI","4","input","R"),
        ("MISO","5","output","R"), ("SCK","6","input","R"),
    ]),
    "SBCCAN01": dict(ref="U", val="Joy-IT SBC-CAN01 (MCP2515 + MCP2562, 16 MHz)",
                     w=16*G, pins=[
        ("VCC","1","power_in","L"), ("VCC1","2","power_in","L"),
        ("GND","3","power_in","L"),
        ("CS","4","input","R"), ("SI","5","input","R"), ("SO","6","output","R"),
        ("SCK","7","input","R"), ("INT","8","output","R"),
        ("CANH","9","bidirectional","R"), ("CANL","10","bidirectional","R"),
    ]),
    "BUCK": dict(ref="U", val="Abwaertswandler 12V->5V, 1 A, automotive-tauglich",
                 w=14*G, pins=[
        ("VIN","1","power_in","L"), ("GND_IN","2","power_in","L"),
        ("VOUT","3","power_out","R"), ("GND_OUT","4","passive","R"),
    ]),
    "FUSE": dict(ref="F", val="Sicherung im Abzweig (eigene Absicherung)", w=9*G, pins=[
        ("1","1","passive","L"), ("2","2","passive","R"),
    ]),
    "OBD": dict(ref="J", val="OBD-II Buchse - NUR CAN-H und CAN-L, keine Masse",
                w=14*G, pins=[
        ("6_CANH","6","passive","L"), ("14_CANL","14","passive","L"),
    ]),
    "FUSEBOX": dict(ref="J", val="Sicherungskasten, zuendungsgeschaltet", w=15*G, pins=[
        ("12V_ZUEND","1","power_out","R"), ("KAROSSERIE_GND","2","passive","R"),
    ]),
}


def lib_symbol(name, d, indent, prefix):
    """Ein Symbol als S-Ausdruck. prefix=True fuer die eingebettete Fassung."""
    pins_l = [p for p in d["pins"] if p[3] == "L"]
    pins_r = [p for p in d["pins"] if p[3] == "R"]
    rows = max(len(pins_l), len(pins_r))
    half_h = (rows * G) / 2 + G
    half_w = d["w"] / 2
    sn = f"{LIB}:{name}" if prefix else name
    i = " " * indent
    o = [f'{i}(symbol "{sn}" (pin_names (offset 0.508)) (in_bom yes) (on_board yes)']
    o.append(f'{i}  (property "Reference" "{d["ref"]}" (at {-half_w} {half_h+1.27} 0) '
             f'(effects (font (size 1.27 1.27)) (justify left)))')
    o.append(f'{i}  (property "Value" "{d["val"]}" (at {-half_w} {-half_h-2.54} 0) '
             f'(effects (font (size 1.27 1.27)) (justify left)))')
    o.append(f'{i}  (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    o.append(f'{i}  (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    o.append(f'{i}  (symbol "{name}_0_1"')
    o.append(f'{i}    (rectangle (start {-half_w} {half_h}) (end {half_w} {-half_h}) '
             f'(stroke (width 0.254) (type default)) (fill (type background)))')
    o.append(f'{i}  )')
    o.append(f'{i}  (symbol "{name}_1_1"')
    for plist, side in ((pins_l, "L"), (pins_r, "R")):
        top = (len(plist) - 1) * G / 2
        for k, (pn, num, typ, _) in enumerate(plist):
            y = top - k * G
            x, ang = (-half_w - G, 0) if side == "L" else (half_w + G, 180)
            o.append(f'{i}    (pin {typ} line (at {x} {y} {ang}) (length {G})')
            o.append(f'{i}      (name "{pn}" {FONT}) (number "{num}" {FONT}))')
    o.append(f'{i}  )')
    o.append(f'{i})')
    return "\n".join(o)


def pwr_flag(indent, prefix):
    sn = f"{LIB}:PWR_FLAG" if prefix else "PWR_FLAG"
    i = " " * indent
    return "\n".join([
        f'{i}(symbol "{sn}" (power) (pin_numbers hide) (pin_names (offset 0) hide) '
        f'(in_bom no) (on_board no)',
        f'{i}  (property "Reference" "#FLG" (at 0 2.54 0) '
        f'(effects (font (size 1.27 1.27)) hide))',
        f'{i}  (property "Value" "PWR_FLAG" (at 0 4.06 0) (effects (font (size 1.27 1.27))))',
        f'{i}  (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
        f'{i}  (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
        f'{i}  (symbol "PWR_FLAG_0_0"',
        f'{i}    (pin power_out line (at 0 0 0) (length 0)',
        f'{i}      (name "~" {FONT}) (number "1" {FONT}))',
        f'{i}  )',
        f'{i}  (symbol "PWR_FLAG_0_1"',
        f'{i}    (polyline (pts (xy 0 0) (xy 0 1.27) (xy -1.016 1.905) (xy 0 2.54) '
        f'(xy 1.016 1.905) (xy 0 1.27)) (stroke (width 0.254) (type default)) '
        f'(fill (type none)))',
        f'{i}  )',
        f'{i})',
    ])


# (symbol, ref, x, y, {pin: netz | None = no-connect})  x,y Vielfache von 2,54
PLACE = [
    ("FUSEBOX", "J2", 38.10, 38.10, {"12V_ZUEND": "+12V_ZUEND",
                                     "KAROSSERIE_GND": "GND"}),
    ("FUSE",    "F1", 129.54, 33.02, {"1": "+12V_ZUEND", "2": "+12V_ABGESICHERT"}),
    ("BUCK",    "U6", 210.82, 38.10, {"VIN": "+12V_ABGESICHERT", "GND_IN": "GND",
                                      "VOUT": "+5V", "GND_OUT": "GND"}),
    ("ESP32S3", "U1", 119.38, 132.08, {
        "5V": "+5V", "3V3": "+3V3", "GND": "GND",
        "GPIO1": "CAN_CS", "GPIO2": "CAN_INT", "GPIO3": "CAN_SCK",
        "GPIO4": "SD_CS", "GPIO5": "SD_MOSI", "GPIO6": "SD_MISO", "GPIO7": "SD_SCK",
        "GPIO8": "I2C_SDA", "GPIO9": "I2C_SCL",
        "GPIO11": "CAN_MISO", "GPIO13": "CAN_MOSI",
        "GPIO15": "GPS_MCU_TX", "GPIO16": "GPS_MCU_RX"}),
    ("BNO055",  "U2", 248.92, 71.12, {"VIN": "+3V3", "GND": "GND",
                                      "SDA": "I2C_SDA", "SCL": "I2C_SCL",
                                      "3Vo": None, "RST": None, "INT": None,
                                      "PS0": None, "PS1": None}),
    ("BN880",   "U3", 248.92, 119.38, {"VCC": "+3V3", "GND": "GND",
                                       "TX": "GPS_MCU_RX", "RX": "GPS_MCU_TX",
                                       "SCL": None, "SDA": None}),
    ("SDCARD",  "U4", 248.92, 160.02, {"3.3V": "+3V3", "GND": "GND",
                                       "CS": "SD_CS", "MOSI": "SD_MOSI",
                                       "MISO": "SD_MISO", "SCK": "SD_SCK"}),
    ("SBCCAN01","U5", 248.92, 205.74, {"VCC": "+3V3", "VCC1": "+5V", "GND": "GND",
                                       "CS": "CAN_CS", "SI": "CAN_MOSI",
                                       "SO": "CAN_MISO", "SCK": "CAN_SCK",
                                       "INT": "CAN_INT",
                                       "CANH": "CAN_H", "CANL": "CAN_L"}),
    ("OBD",     "J1", 355.60, 210.82, {"6_CANH": "CAN_H", "14_CANL": "CAN_L"}),
]

# PWR_FLAG auf Netze ohne treibenden Ausgang
FLAGS = [("GND", 63.50, 76.20), ("+12V_ABGESICHERT", 165.10, 55.88)]

NOTES = [
    (20.32, 93.98, textwrap.dedent("""\
        ROADTEST - logische Verdrahtung des aktuellen Aufbaus
        Erzeugt aus HARDWARE.md, schematic.md und src/hardware_config.h.
        Kastensymbole: die Baugruppen sind fertige Module, keine diskreten ICs.
        Kein Bestueckungsplan - keine Lochraster-Loetpunkte, keine Kabelfarben.""")),
    (20.32, 175.26, textwrap.dedent("""\
        VERBINDLICH:
        - ESP32-S3-GPIOs sind NICHT 5-V-tolerant. MCP2515 VCC = 3,3 V, VCC1 = 5 V.
          Lag VCC auf 5 V, trieb SO 5-V-Pegel auf GPIO 11 (korrigiert am 28.07.2026).
        - Terminierungsjumper P1 am CAN-Modul bleibt am Fahrzeug ENTFERNT.
        - Von der OBD-Buchse kommen NUR CAN-H und CAN-L. Masse ausschliesslich an
          der Karosserie beim Sicherungskasten - sonst zweite Masseschleife.
        - Eigene Absicherung im Abzweig, unabhaengig vom angezapften Stromkreis.
        - Kein LiPo im Fahrzeug: der Innenraum erreicht im Sommer 60 bis 70 Grad.
        - Das SSD1306-OLED ist seit dem 03.08.2026 AUSGEBAUT und hier nicht mehr
          gezeichnet; schematic.md zeigt es noch. Der BNO055 ist damit der einzige
          I2C-Teilnehmer - Bus- und Sensorfehler sind nicht mehr trennbar.
        - Vor dem endgueltigen Einbau gehoert eine externe aktive GNSS-Antenne an
          den BN-880; die aufgeloetete Keramikantenne steht im Verdacht.
        - Das BNO055-Breakout traegt je 10 kOhm von SDA, SCL und RST nach 3Vo.
          Keine zusaetzlichen externen Pull-ups bestuecken (gemessen 2,54 kOhm).
        - Der Controllertyp ist verloetet und nicht ablesbar; massgeblich ist der
          Controller-Steckbrief aus dem seriellen Kommando 'diag'."""))
]


def geometry(sym):
    if sym == "PWR_FLAG":
        return [], [], 2.54, 0.0
    d = SYMS[sym]
    pl = [p for p in d["pins"] if p[3] == "L"]
    pr = [p for p in d["pins"] if p[3] == "R"]
    rows = max(len(pl), len(pr))
    return pl, pr, (rows * G) / 2 + G, d["w"] / 2


def main():
    os.makedirs(OUT, exist_ok=True)

    # --- Symbolbibliothek ---
    lib = ["(kicad_symbol_lib (version 20231120) (generator kicad_symbol_editor)"]
    lib.append(pwr_flag(2, False))
    for n, d in SYMS.items():
        lib.append(lib_symbol(n, d, 2, False))
    lib.append(")")
    open(f"{OUT}/{LIB}.kicad_sym", "w", encoding="utf-8").write("\n".join(lib) + "\n")

    open(f"{OUT}/sym-lib-table", "w", encoding="utf-8").write(
        '(sym_lib_table\n  (version 7)\n'
        f'  (lib (name "{LIB}")(type "KiCad")'
        f'(uri "${{KIPRJMOD}}/{LIB}.kicad_sym")(options "")'
        '(descr "ROADTEST Modulsymbole"))\n)\n')

    # --- Schaltplan ---
    parts = [pwr_flag(4, True)] + [lib_symbol(n, d, 4, True) for n, d in SYMS.items()]
    body = []

    def place(sym, ref, ox, oy, val=None, hide_ref=False):
        _, _, half_h, half_w = geometry(sym)
        v = val if val else SYMS[sym]["val"]
        body.append(f'  (symbol (lib_id "{LIB}:{sym}") (at {ox} {oy} 0) (unit 1)')
        body.append(f'    (in_bom yes) (on_board yes) (dnp no) (uuid "{U()}")')
        h = " hide" if hide_ref else ""
        body.append(f'    (property "Reference" "{ref}" (at {ox-half_w} {oy-half_h-1.27} 0) '
                    f'(effects (font (size 1.27 1.27)) (justify left){h}))')
        body.append(f'    (property "Value" "{v}" (at {ox-half_w} {oy+half_h+3.81} 0) '
                    f'(effects (font (size 1.27 1.27)) (justify left)))')
        body.append(f'    (property "Footprint" "" (at {ox} {oy} 0) '
                    f'(effects (font (size 1.27 1.27)) hide))')
        body.append(f'    (property "Datasheet" "" (at {ox} {oy} 0) '
                    f'(effects (font (size 1.27 1.27)) hide))')
        body.append(f'    (instances (project "roadtest" '
                    f'(path "/{ROOT}" (reference "{ref}") (unit 1))))')
        body.append('  )')

    def stub(px, py, ex, just, net):
        body.append(f'  (wire (pts (xy {px} {py}) (xy {ex} {py})) '
                    f'(stroke (width 0) (type default)) (uuid "{U()}"))')
        body.append(f'  (label "{net}" (at {ex} {py} 0) '
                    f'(effects (font (size 1.27 1.27)) (justify {just} bottom)) '
                    f'(uuid "{U()}"))')

    for sym, ref, ox, oy, nets in PLACE:
        pl, pr, half_h, half_w = geometry(sym)
        place(sym, ref, ox, oy)
        for plist, side in ((pl, "L"), (pr, "R")):
            top = (len(plist) - 1) * G / 2
            for k, (pn, _num, _t, _s) in enumerate(plist):
                y = oy - (top - k * G)          # KiCad-Y zeigt nach unten
                if side == "L":
                    px, ex, just = ox - half_w - G, ox - half_w - G - 3*G, "right"
                else:
                    px, ex, just = ox + half_w + G, ox + half_w + G + 3*G, "left"
                net = nets.get(pn)
                if net is None:
                    body.append(f'  (no_connect (at {px} {y}) (uuid "{U()}"))')
                else:
                    stub(px, y, ex, just, net)

    for i, (net, fx, fy) in enumerate(FLAGS):
        place("PWR_FLAG", f"#FLG{i+1}", fx, fy, val="PWR_FLAG", hide_ref=True)
        stub(fx, fy, fx + 3*G, "left", net)

    for x, y, txt in NOTES:
        esc = txt.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
        body.append(f'  (text "{esc}" (at {x} {y} 0) '
                    f'(effects (font (size 1.27 1.27)) (justify left top)) (uuid "{U()}"))')

    today = datetime.date.today().isoformat()
    sch = "\n".join([
        '(kicad_sch (version 20230121) (generator eeschema)',
        f'  (uuid "{ROOT}")',
        '  (paper "A3")',
        '  (title_block',
        '    (title "ROADTEST - Strassenqualitaets-Messsystem")',
        f'    (date "{today}") (rev "1.5.49")',
        '    (comment 1 "Erzeugt aus HARDWARE.md, schematic.md und hardware_config.h")',
        '    (comment 2 "Logische Verdrahtung, kein Bestueckungsplan")',
        '  )',
        '  (lib_symbols',
        "\n".join(parts),
        '  )',
        "\n".join(body),
        '  (sheet_instances (path "/" (page "1")))',
        ')',
    ])
    open(f"{OUT}/roadtest.kicad_sch", "w", encoding="utf-8").write(sch + "\n")

    open(f"{OUT}/roadtest.kicad_pro", "w", encoding="utf-8").write(
        '{\n  "board": {"design_settings": {}},\n'
        '  "meta": {"filename": "roadtest.kicad_pro", "version": 1},\n'
        '  "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},\n'
        f'  "sheets": [["{ROOT}", "Root"]],\n'
        '  "text_variables": {}\n}\n')
    print(f"geschrieben nach {OUT}/")


if __name__ == "__main__":
    main()

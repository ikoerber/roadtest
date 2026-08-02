#!/usr/bin/env python3
"""Wandelt eine ROADTEST-Messsitzung in GeoJSON für geojson.io um.

Aufruf:

    python3 tools/export_geojson.py testdata/20260730_095603_A41A2450

Die Ausgabedatei wird per Drag-and-drop in https://geojson.io geöffnet. Die
Einfärbung folgt der simplestyle-Spezifikation, die geojson.io auswertet;
alle übrigen Werte stehen als Eigenschaften am Feature und erscheinen beim
Anklicken.

Ebenen:

    1. Strecke, eingefärbt nach Straßenqualität oder nach der Abweichung
       zwischen GPS- und OBD-Geschwindigkeit (--modus)
    2. Schlaglöcher als Marker, Größe und Farbe nach Schwere
    3. Kurven als tatsächlicher Bogen, sofern die Sitzung Anfangs- und
       Endzeit liefert (ab Schema 1.5.28), sonst als Punkt
    4. Referenzintervalle des Beifahrer-Kurventests
    5. Sitzungskopf mit Firmware, Kennzahlen und Laufzeitdiagnose

Grundsätze der Auswertung:

    - Nur Positionen mit ``LocationValid=1`` werden verwendet.
    - Über eine Qualitäts- oder Zeitlücke hinweg wird keine Linie gezogen.
      Der Datenqualitätsplan verlangt das ausdrücklich.
    - Unbekannte Werte erscheinen als ``null``, niemals als 0.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from bisect import bisect_left

# Eine Lücke über dieser Dauer trennt die Linie, statt sie quer durch die
# Landschaft zu ziehen.
GAP_SPLIT_MS = 5000

# Zuordnungsfenster für zeitlich benachbarte Messwerte.
MATCH_WINDOW_MS = 1500

# Die Breitengrade werden in der Firmware als float32 gespeichert. Daraus
# folgt eine Rasterung von rund 0,33 m, die in der Karte bei Schrittgeschwin-
# digkeit als Treppe sichtbar wird. Das ist kein GPS-Rauschen.
QUANTISIERUNG_HINWEIS = (
    "Breiten- und Längengrad sind in der Firmware als float32 gespeichert. "
    "Daraus folgt eine Rasterung von rund 0,33 m in der Breite. Sichtbare "
    "Treppenstufen bei langsamer Fahrt stammen aus diesem Datentyp, nicht "
    "aus dem GPS-Empfang."
)

QUALITAET_FARBEN = [
    (95.0, "#1a9850", "sehr gut"),
    (85.0, "#a6d96a", "gut"),
    (70.0, "#fdae61", "maessig"),
    (0.0, "#d73027", "schlecht"),
]
FARBE_UNBEKANNT = "#9e9e9e"

ABWEICHUNG_FARBEN = [
    (2.0, "#1a9850", "unter 2 km/h"),
    (5.0, "#a6d96a", "2 bis 5 km/h"),
    (10.0, "#fdae61", "5 bis 10 km/h"),
    (float("inf"), "#d73027", "ueber 10 km/h"),
]

SCHLAGLOCH_STUFEN = [
    (5.0, "#d73027", "large", "GROSS"),
    (2.0, "#fc8d59", "medium", "MITTEL"),
    (0.0, "#fee08b", "small", "KLEIN"),
]

KURVEN_FARBEN = [
    (200.0, "#4575b4", "weit"),
    (80.0, "#74add1", "mittel"),
    (30.0, "#f46d43", "eng"),
    (0.0, "#a50026", "sehr eng"),
]

# Fahrbarkeit je Streckenabschnitt.
#
# Die Klassen tragen die Namen der Beifahrerstufen und liegen jeweils in der
# Mitte zwischen deren Notenmedianen aus der Fahrt vom 02.08.2026: Stufe 1
# erreichte 89, Stufe 2 dann 72, Stufe 3 noch 22. Frei gewählte Grenzen wären
# irreführend - eine erste Fassung nannte alles unter 25 "nicht mehr zuegig",
# obwohl der Beifahrer dieselben Abschnitte 20 von 27 Mal mit "maessig"
# bewertet hatte.
#
# Stufe 3 und 4 bleiben zusammengefasst. Sie sind über den Effektivwert nicht
# trennbar: Auf sehr schlechter Straße wird so langsam gefahren, dass die
# Anregung wieder sinkt, und Stufe 4 lag im Median unter Stufe 3.
FAHRBARKEIT_FARBEN = [
    (81.0, "#1a9850", "sehr gut"),
    (47.0, "#a6d96a", "gut"),
    (0.0, "#fdae61", "maessig bis schlecht"),
]

# Beifahrerurteile. Vier Stufen, absichtlich andere Farbtöne als die
# gemessene Fahrbarkeit: Auf der Karte muss unterscheidbar bleiben, was
# gemessen und was geurteilt wurde.
URTEIL_STUFEN = {
    1: ("#08519c", "SEHR_GUT"),
    2: ("#4292c6", "GUT"),
    3: ("#9e3ea1", "MAESSIG"),
    4: ("#67000d", "SCHLECHT"),
}

REFERENZ_FARBEN = {
    "STRAIGHT": "#6a51a3",
    "LEFT_WIDE": "#2171b5",
    "RIGHT_WIDE": "#2171b5",
    "LEFT_NORMAL": "#238b45",
    "RIGHT_NORMAL": "#238b45",
    "S_CURVE": "#d94801",
}


# ---------------------------------------------------------------------------
# Einlesen
# ---------------------------------------------------------------------------


def lade_csv(verzeichnis: str, praefix: str) -> list[dict]:
    """Liest die erste passende CSV. Fehlt sie, wird eine leere Liste
    geliefert - eine Sitzung muss nicht alle Dateien enthalten."""
    treffer = sorted(
        f
        for f in os.listdir(verzeichnis)
        if f.startswith(praefix) and f.endswith(".csv")
    )
    if not treffer:
        return []
    with open(os.path.join(verzeichnis, treffer[0]), newline="") as fh:
        return list(csv.DictReader(fh))


def als_zahl(text: str | None):
    """Wandelt in float um. Leer oder unlesbar ergibt None, nicht 0."""
    if text is None:
        return None
    text = text.strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def als_ganzzahl(text: str | None):
    zahl = als_zahl(text)
    return int(zahl) if zahl is not None else None


def beschreibung_zerlegen(text: str) -> dict:
    """Zerlegt Beschreibungen der Form ``Winkel=90.0;Radius=45.2;Gruppe=3``.

    Ältere Sitzungen verwenden ``Winkel: 90°`` beziehungsweise
    ``MITTEL (3.9 m/s²)``; beides wird ebenfalls erkannt."""
    werte: dict = {}
    for teil in text.split(";"):
        if "=" in teil:
            schluessel, wert = teil.split("=", 1)
            werte[schluessel.strip()] = wert.strip()
    if not werte:
        treffer = re.search(r"Winkel:\s*(-?[\d.]+)", text)
        if treffer:
            werte["Winkel"] = treffer.group(1)
        treffer = re.search(r"\(([\d.]+)\s*m/s", text)
        if treffer:
            werte["Schwere"] = treffer.group(1)
    return werte


# ---------------------------------------------------------------------------
# Zeitliche Zuordnung
# ---------------------------------------------------------------------------


class Zeitreihe:
    """Nachschlagen des zeitlich nächsten Wertes mit Höchstabstand."""

    def __init__(self, paare: list[tuple[int, object]]):
        paare = sorted(paare, key=lambda p: p[0])
        self.zeiten = [p[0] for p in paare]
        self.werte = [p[1] for p in paare]

    def bei(self, zeit: int, fenster: int = MATCH_WINDOW_MS):
        if not self.zeiten:
            return None
        i = bisect_left(self.zeiten, zeit)
        kandidaten = []
        if i < len(self.zeiten):
            kandidaten.append(i)
        if i > 0:
            kandidaten.append(i - 1)
        bester, bester_abstand = None, None
        for k in kandidaten:
            abstand = abs(self.zeiten[k] - zeit)
            if bester_abstand is None or abstand < bester_abstand:
                bester, bester_abstand = k, abstand
        if bester is None or bester_abstand > fenster:
            return None
        return self.werte[bester]


def gps_punkte(gps_zeilen: list[dict]) -> list[tuple[int, float, float, dict]]:
    """Liefert nur nachgewiesen gültige Positionen als (Zeit, lon, lat, Zeile).

    GeoJSON erwartet die Reihenfolge Länge vor Breite."""
    # Sitzungen vor Schema 1.5.15 kennen LocationValid noch nicht. Dort ist
    # ValidFix das einzige Gültigkeitsmerkmal; es ist schwächer, weil es die
    # feldbezogene Alters- und Qualitätsprüfung nicht enthält.
    spalte = "LocationValid" if gps_zeilen and "LocationValid" in gps_zeilen[0] \
        else "ValidFix"

    punkte = []
    for zeile in gps_zeilen:
        if zeile.get(spalte) != "1":
            continue
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        lat = als_zahl(zeile.get("Latitude"))
        lon = als_zahl(zeile.get("Longitude"))
        if zeit is None or lat is None or lon is None:
            continue
        punkte.append((zeit, lon, lat, zeile))
    return punkte


# ---------------------------------------------------------------------------
# Einfärbung
# ---------------------------------------------------------------------------


def qualitaet_farbe(wert):
    if wert is None:
        return FARBE_UNBEKANNT, "unbekannt"
    for schwelle, farbe, name in QUALITAET_FARBEN:
        if wert >= schwelle:
            return farbe, name
    return FARBE_UNBEKANNT, "unbekannt"


def abweichung_farbe(wert):
    if wert is None:
        return FARBE_UNBEKANNT, "unbekannt"
    for schwelle, farbe, name in ABWEICHUNG_FARBEN:
        if wert < schwelle:
            return farbe, name
    return FARBE_UNBEKANNT, "unbekannt"


def kurven_farbe(radius):
    if radius is None or radius <= 0:
        return FARBE_UNBEKANNT, "unbekannt"
    for schwelle, farbe, name in KURVEN_FARBEN:
        if radius >= schwelle:
            return farbe, name
    return FARBE_UNBEKANNT, "unbekannt"


# ---------------------------------------------------------------------------
# Ebenen
# ---------------------------------------------------------------------------


def ebene_strecke(punkte, qualitaet, gps_speed, obd_speed, modus):
    """Strecke als eingefärbte Linienzüge.

    Aufeinanderfolgende Abschnitte gleicher Farbe werden zu einem Feature
    zusammengefasst. Das hält die Datei klein, ohne Information zu verlieren:
    Ein Farbwechsel trennt weiterhin."""
    features = []
    laufend: list[list[float]] = []
    laufende_farbe = None
    laufende_stufe = None
    laufender_start = None
    letzte_zeit = None

    def abschluss(ende_zeit):
        if len(laufend) >= 2:
            features.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "LineString", "coordinates": list(laufend)},
                    "properties": {
                        "ebene": "strecke",
                        "bewertung": laufende_stufe,
                        "modus": modus,
                        "vonUptimeMs": laufender_start,
                        "bisUptimeMs": ende_zeit,
                        "stroke": laufende_farbe,
                        "stroke-width": 4,
                        "stroke-opacity": 0.9,
                    },
                }
            )

    for zeit, lon, lat, _zeile in punkte:
        if modus == "geschwindigkeit":
            g = gps_speed.bei(zeit)
            o = obd_speed.bei(zeit)
            wert = abs(g - o) if (g is not None and o is not None) else None
            farbe, stufe = abweichung_farbe(wert)
        else:
            wert = qualitaet.bei(zeit)
            farbe, stufe = qualitaet_farbe(wert)

        luecke = letzte_zeit is not None and zeit - letzte_zeit > GAP_SPLIT_MS
        if luecke or (laufende_farbe is not None and farbe != laufende_farbe):
            abschluss(letzte_zeit)
            # Bei Farbwechsel ohne Lücke bleibt der letzte Punkt als
            # Anschluss erhalten, damit die Linie nicht aufreisst.
            laufend = [] if luecke else laufend[-1:]
            laufender_start = None if luecke else letzte_zeit

        if not laufend:
            laufender_start = zeit
        laufend.append([lon, lat])
        laufende_farbe, laufende_stufe = farbe, stufe
        letzte_zeit = zeit

    abschluss(letzte_zeit)
    return features


def ebene_schlagloecher(ereignisse, position):
    features = []
    for zeile in ereignisse:
        if zeile.get("Ereignis") != "SCHLAGLOCH":
            continue
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        werte = beschreibung_zerlegen(zeile.get("Beschreibung", ""))
        schwere = als_zahl(zeile.get("Schwere")) or als_zahl(werte.get("Schwere"))
        koordinate = position_fuer(zeile, position, zeit)
        if koordinate is None:
            continue
        _, farbe, groesse, stufe = SCHLAGLOCH_STUFEN[-1]
        for schwelle, f, g, s in SCHLAGLOCH_STUFEN:
            if schwere is not None and schwere >= schwelle:
                farbe, groesse, stufe = f, g, s
                break
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": koordinate},
                "properties": {
                    "ebene": "schlagloch",
                    "stufe": stufe,
                    "schwereMps2": schwere,
                    "utc": zeile.get("UTC"),
                    "uptimeMs": zeit,
                    "beschreibung": zeile.get("Beschreibung"),
                    "marker-color": farbe,
                    "marker-size": groesse,
                    "marker-symbol": "circle",
                },
            }
        )
    return features


def position_fuer(zeile, position: Zeitreihe, zeit):
    """Position eines Ereignisses.

    Die Firmware schreibt bei unbekannter Position 0/0 in die Ereignisdatei.
    Das ist eine gültige Koordinate im Golf von Guinea und darf nicht
    übernommen werden; in diesem Fall wird über die Zeit aus der GPS-Spur
    nachgeschlagen."""
    lat = als_zahl(zeile.get("Latitude"))
    lon = als_zahl(zeile.get("Longitude"))
    if lat is not None and lon is not None and (abs(lat) > 1e-6 or abs(lon) > 1e-6):
        return [lon, lat]
    if zeit is None:
        return None
    return position.bei(zeit, fenster=3000)


def kurven_zeitversatz(ereignisse):
    """Versatz zwischen Geräte- und Sitzungszeit in Millisekunden.

    StartUptimeMs und EndUptimeMs eines Kurvenereignisses zählen seit dem
    Gerätestart, die Spalte UptimeMs dagegen seit dem Sitzungsbeginn. Ohne
    Umrechnung liegen beide Zeitbasen um die gesamte bisherige Gerätelaufzeit
    auseinander, und kein einziger GPS-Punkt fällt in ein Kurvenintervall.

    Ein Referenzmarker der Beifahrerseite wird ohne Verzögerung geschrieben
    und liefert den Versatz exakt. Ebenso ein Abschnittsereignis ab Schema
    1.5.35: Es entsteht im selben Schleifendurchlauf, in dem der Abschnitt
    seine Länge erreicht, und kennt daher keinen Schreibverzug.

    Sonst bleibt der Rückschluss aus den Kurvenereignissen: EndUptimeMs minus
    UptimeMs ergibt den Versatz abzüglich des Schreibverzugs, und der hängt
    vom Abschlussgrund ab. Nach QUIET vergeht das volle Ruhefenster von zwei
    Sekunden, nach REVERSAL fast nichts. Das Maximum trifft ihn deshalb am
    besten.
    """
    for zeile in ereignisse:
        if zeile.get("Ereignis") != "CURVE_REFERENCE_END":
            continue
        werte = beschreibung_zerlegen(zeile.get("Beschreibung", ""))
        ende = als_ganzzahl(werte.get("EndMs"))
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        if ende is not None and zeit is not None:
            return ende - zeit

    for zeile in ereignisse:
        if zeile.get("Ereignis") != "ABSCHNITT":
            continue
        ende = als_ganzzahl(zeile.get("EndUptimeMs"))
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        if ende is not None and zeit is not None:
            return ende - zeit

    versatz = None
    for zeile in ereignisse:
        if zeile.get("Ereignis") != "KURVE":
            continue
        ende = als_ganzzahl(zeile.get("EndUptimeMs"))
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        if ende is None or zeit is None:
            continue
        kandidat = ende - zeit
        if versatz is None or kandidat > versatz:
            versatz = kandidat
    return versatz


def ebene_kurven(ereignisse, punkte, position):
    """Kurven als Bogen, wenn Anfangs- und Endzeit vorliegen.

    Ab Schema 1.5.28 enthält die Ereignisdatei StartUptimeMs und
    EndUptimeMs. Dann wird der tatsächlich gefahrene Bogen aus der GPS-Spur
    gezeichnet. Ältere Sitzungen liefern nur den Abschlusszeitpunkt; dort
    bleibt es bei einem Punkt."""
    features = []
    versatz = kurven_zeitversatz(ereignisse)
    for zeile in ereignisse:
        if zeile.get("Ereignis") != "KURVE":
            continue
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        werte = beschreibung_zerlegen(zeile.get("Beschreibung", ""))
        start = als_ganzzahl(zeile.get("StartUptimeMs"))
        ende = als_ganzzahl(zeile.get("EndUptimeMs"))
        radius = als_zahl(zeile.get("RadiusM")) or als_zahl(werte.get("Radius"))
        winkel = als_zahl(zeile.get("AngleDeg")) or als_zahl(werte.get("Winkel"))
        farbe, klasse = kurven_farbe(radius)

        eigenschaften = {
            "ebene": "kurve",
            "klasse": klasse,
            "winkelGrad": winkel,
            "radiusM": radius,
            "utc": zeile.get("UTC"),
            "uptimeMs": zeit,
            "richtung": zeile.get("Direction") or None,
            "gruppe": als_ganzzahl(zeile.get("CurveGroupId"))
            or als_ganzzahl(werte.get("Gruppe")),
            "dauerMs": als_ganzzahl(zeile.get("DurationMs")),
            "wegM": als_zahl(zeile.get("DistanceM")),
            "mittlereGeschwindigkeitKmh": als_zahl(zeile.get("MeanSpeedKmh")),
            "maxGierrateDps": als_zahl(zeile.get("MaxYawRateDps")),
            "mittlereQuerbeschleunigung": als_zahl(zeile.get("MeanLateralAccel")),
            "maxQuerbeschleunigung": als_zahl(zeile.get("MaxLateralAccel")),
            "erkennungsmodus": zeile.get("DetectionMode") or None,
            "abschlussgrund": zeile.get("CompletionReason") or None,
            "qualitaetsflags": als_ganzzahl(zeile.get("QualityFlags")),
            "beschreibung": zeile.get("Beschreibung"),
        }

        bogen = None
        if (
            start is not None
            and ende is not None
            and ende > start
            and versatz is not None
        ):
            # Beide Grenzen auf die Sitzungszeit der GPS-Spur umrechnen.
            von = start - versatz
            bis = ende - versatz
            eigenschaften["vonUptimeMs"] = von
            eigenschaften["bisUptimeMs"] = bis
            bogen = [[lon, lat] for t, lon, lat, _ in punkte if von <= t <= bis]
        if bogen and len(bogen) >= 2:
            eigenschaften.update(
                {"stroke": farbe, "stroke-width": 6, "stroke-opacity": 1.0}
            )
            features.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "LineString", "coordinates": bogen},
                    "properties": eigenschaften,
                }
            )
            continue

        koordinate = position_fuer(zeile, position, zeit)
        if koordinate is None:
            continue
        eigenschaften.update(
            {
                "marker-color": farbe,
                "marker-size": "medium",
                "marker-symbol": "triangle",
                "hinweis": (
                    "Kein Bogen: Sitzung ohne Anfangs- und Endzeit"
                    if start is None or ende is None
                    else "Kein Bogen: keine GPS-Punkte im Kurvenintervall"
                ),
            }
        )
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": koordinate},
                "properties": eigenschaften,
            }
        )
    return features


def notengrenzen() -> tuple[float, float]:
    """Liest die Notengrenzen aus src/hardware_config.h.

    Die Firmware schreibt die fertig berechnete Note in die Logdatei. Wird
    die Skala nachkalibriert, tragen ältere Aufzeichnungen weiterhin die
    Noten ihrer Aufzeichnungsversion und wären untereinander nicht
    vergleichbar. Der Export rechnet die Note deshalb aus dem gespeicherten
    Effektivwert neu: Der Effektivwert ist die Messung, die Note die
    Auslegung.

    Gelesen statt dupliziert, damit Skript und Firmware nicht auseinander
    laufen. Fehlt der Header, gilt die Auslegung von 1.5.36.
    """
    pfad = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src",
        "hardware_config.h",
    )
    werte = {}
    try:
        with open(pfad, encoding="utf-8") as datei:
            for zeile in datei:
                treffer = re.match(
                    r"\s*#define\s+(ROAD_DRIVEABILITY_RMS_\w+)\s+([\d.]+)f?",
                    zeile,
                )
                if treffer:
                    werte[treffer.group(1)] = float(treffer.group(2))
    except OSError:
        pass
    return (
        werte.get("ROAD_DRIVEABILITY_RMS_GOOD_MPS2", 0.55),
        werte.get("ROAD_DRIVEABILITY_RMS_BAD_MPS2", 1.60),
    )


def note_aus_effektivwert(rms, grenzen):
    if rms is None:
        return None
    gut, schlecht = grenzen
    if rms <= gut:
        return 100.0
    if rms >= schlecht:
        return 0.0
    return 100.0 * (schlecht - rms) / (schlecht - gut)


def fahrbarkeit_farbe(note):
    if note is None:
        return FARBE_UNBEKANNT, "unbekannt"
    for schwelle, farbe, name in FAHRBARKEIT_FARBEN:
        if note >= schwelle:
            return farbe, name
    return FARBE_UNBEKANNT, "unbekannt"


def ebene_abschnitte(ereignisse, punkte, position):
    """Fahrbarkeit je Streckenabschnitt als eingefärbte Linie.

    Ein Abschnitt umfasst rund 200 Meter Fahrweg und beantwortet die Frage,
    ob sich dort zügig fahren lässt. Er wird über sein Zeitintervall aus der
    GPS-Spur gezeichnet, genau wie ein Kurvenbogen.
    """
    features = []
    versatz = kurven_zeitversatz(ereignisse)
    grenzen = notengrenzen()
    for zeile in ereignisse:
        if zeile.get("Ereignis") != "ABSCHNITT":
            continue
        werte = beschreibung_zerlegen(zeile.get("Beschreibung", ""))
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        start = als_ganzzahl(zeile.get("StartUptimeMs"))
        ende = als_ganzzahl(zeile.get("EndUptimeMs"))
        effektivwert = als_zahl(werte.get("RMS")) or als_zahl(
            zeile.get("MeanYawRateDps")
        )
        # Aus dem Effektivwert neu berechnet, damit Aufzeichnungen
        # verschiedener Firmwarestände dieselbe Skala verwenden.
        note = note_aus_effektivwert(effektivwert, grenzen)
        note_geraet = als_zahl(zeile.get("Schwere")) or als_zahl(
            werte.get("Note")
        )
        farbe, klasse = fahrbarkeit_farbe(note)

        eigenschaften = {
            "ebene": "fahrbarkeit",
            "klasse": klasse,
            "note": round(note, 1) if note is not None else None,
            "noteAufzeichnung": note_geraet,
            "abschnitt": als_ganzzahl(werte.get("Abschnitt")),
            "effektivwertMps2": effektivwert,
            "groessterStossMps2": als_zahl(zeile.get("MaxYawRateDps")),
            "stoesse": als_ganzzahl(werte.get("Stoesse")),
            "wegM": als_zahl(zeile.get("DistanceM")),
            "mittlereGeschwindigkeitKmh": als_zahl(zeile.get("MeanSpeedKmh")),
            "maxGeschwindigkeitKmh": als_zahl(zeile.get("MaxSpeedKmh")),
            "stichproben": als_ganzzahl(zeile.get("Samples")),
            "utc": zeile.get("UTC"),
            "uptimeMs": zeit,
        }

        linie = None
        if start is not None and ende is not None and ende > start and versatz is not None:
            von = start - versatz
            bis = ende - versatz
            eigenschaften["vonUptimeMs"] = von
            eigenschaften["bisUptimeMs"] = bis
            linie = [[lon, lat] for t, lon, lat, _ in punkte if von <= t <= bis]
        if linie and len(linie) >= 2:
            eigenschaften.update(
                {"stroke": farbe, "stroke-width": 9, "stroke-opacity": 0.85}
            )
            features.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "LineString", "coordinates": linie},
                    "properties": eigenschaften,
                }
            )
            continue

        koordinate = position_fuer(zeile, position, zeit)
        if koordinate is None:
            continue
        eigenschaften.update(
            {
                "marker-color": farbe,
                "marker-size": "small",
                "marker-symbol": "square",
                "hinweis": "Keine GPS-Punkte im Abschnittsintervall",
            }
        )
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": koordinate},
                "properties": eigenschaften,
            }
        )
    return features


def ebene_beifahrerurteile(ereignisse, position):
    """Urteile und Belagswechsel des Beifahrers.

    Ein Urteil gilt ausdrücklich für seinen Abschnitt und nicht bis zum
    nächsten Marker; wer eine Weile aussetzt, hinterlässt eine Lücke. Die
    Punkte werden deshalb einzeln gezeichnet und nicht zu Strecken verbunden.
    """
    features = []
    for zeile in ereignisse:
        art = zeile.get("Ereignis")
        if art not in ("FAHRBAHN_URTEIL", "FAHRBAHN_BELAGSWECHSEL"):
            continue
        werte = beschreibung_zerlegen(zeile.get("Beschreibung", ""))
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        koordinate = position_fuer(zeile, position, zeit)
        if koordinate is None:
            continue

        gemeinsam = {
            "utc": zeile.get("UTC"),
            "uptimeMs": zeit,
            "abschnitt": als_ganzzahl(werte.get("Abschnitt")),
            "abschnittWegM": als_zahl(werte.get("AbschnittWegM")),
        }
        if art == "FAHRBAHN_URTEIL":
            stufe = als_ganzzahl(werte.get("Stufe"))
            farbe, name = URTEIL_STUFEN.get(stufe, (FARBE_UNBEKANNT, "unbekannt"))
            gemeinsam.update(
                {
                    "ebene": "beifahrerurteil",
                    "stufe": stufe,
                    "wertung": werte.get("Wertung") or name,
                    "marker-color": farbe,
                    "marker-size": "medium",
                    "marker-symbol": str(stufe) if stufe else "circle",
                }
            )
        else:
            gemeinsam.update(
                {
                    "ebene": "belagswechsel",
                    "marker-color": "#000000",
                    "marker-size": "medium",
                    "marker-symbol": "roadblock",
                }
            )
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": koordinate},
                "properties": gemeinsam,
            }
        )
    return features


def ebene_referenzen(ereignisse, punkte):
    """Referenzintervalle des Beifahrer-Kurventests.

    Die Firmware schreibt für diese Marker keine Position; sie wird über die
    Zeit aus der GPS-Spur bestimmt."""
    features = []
    offen: dict[str, dict] = {}
    for zeile in ereignisse:
        art = zeile.get("Ereignis")
        if art not in ("CURVE_REFERENCE_START", "CURVE_REFERENCE_END"):
            continue
        zeit = als_ganzzahl(zeile.get("UptimeMs"))
        werte = beschreibung_zerlegen(zeile.get("Beschreibung", ""))
        kennung = werte.get("Id")
        if kennung is None or zeit is None:
            continue
        if art == "CURVE_REFERENCE_START":
            offen[kennung] = {"start": zeit, "typ": werte.get("Type", "UNBEKANNT")}
            continue

        eintrag = offen.pop(kennung, None)
        if eintrag is None:
            continue
        start, typ = eintrag["start"], eintrag["typ"]
        bogen = [[lon, lat] for t, lon, lat, _ in punkte if start <= t <= zeit]
        if len(bogen) < 2:
            continue
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": bogen},
                "properties": {
                    "ebene": "referenz",
                    "typ": typ,
                    "id": als_ganzzahl(kennung),
                    "vonUptimeMs": start,
                    "bisUptimeMs": zeit,
                    "dauerMs": zeit - start,
                    "stroke": REFERENZ_FARBEN.get(typ, "#000000"),
                    "stroke-width": 10,
                    "stroke-opacity": 0.45,
                },
            }
        )
    return features


def vollstaendigkeit(ende, gps_zeilen):
    """Vergleicht die vorhandenen GPS-Zeilen mit dem Zähler der Firmware.

    Die Firmware zählt in GPSSnapshots jeden geschriebenen Qualitäts-Snapshot.
    Weicht die Zeilenzahl der Datei davon ab, fehlen Zeilen, und die Strecke
    auf der Karte bricht entsprechend früh ab. In 20260801_114252_B9D1628B
    standen 1122 Zeilen gegen 3534 gezählte Snapshots; die Linie endete nach
    227 von 709 Sekunden. Ohne diesen Hinweis wirkt das wie ein Fehler der
    Auswertung statt wie fehlende Daten.
    """
    gezaehlt = als_ganzzahl(ende.get("GPSSnapshots"))
    vorhanden = len(gps_zeilen)
    if gezaehlt is None or gezaehlt <= vorhanden:
        return {}
    fehlend = gezaehlt - vorhanden
    return {
        "gpsZeilenErwartet": gezaehlt,
        "gpsZeilenFehlend": fehlend,
        "hinweisDatenverlust": (
            f"Unvollständig: {fehlend} von {gezaehlt} GPS-Zeilen fehlen in der "
            "Datei. Die Firmware hat sie als geschrieben gezählt. Strecke und "
            "Kurvenbögen enden dort, wo die GPS-Spur abbricht; Ereignisse mit "
            "eigener Koordinate bleiben darüber hinaus sichtbar."
        ),
    }


def ebene_sitzungskopf(verzeichnis, meta, summary, punkte, gps_zeilen):
    if not punkte:
        return []
    start, lon, lat, _ = punkte[0]
    ende = meta[-1] if meta else {}
    zusammenfassung = summary[0] if summary else {}

    def wert(quelle, schluessel):
        roh = quelle.get(schluessel)
        if roh is None or str(roh).strip() == "":
            return None
        zahl = als_zahl(roh)
        return zahl if zahl is not None else roh

    gueltig = sum(1 for z in gps_zeilen if z.get("LocationValid") == "1")
    eigenschaften = {
        "ebene": "sitzung",
        "sitzung": os.path.basename(os.path.normpath(verzeichnis)),
        "firmware": ende.get("FirmwareVersion"),
        "schema": ende.get("SchemaVersion"),
        "startUTC": zusammenfassung.get("StartUTC"),
        "endeUTC": zusammenfassung.get("EndUTC"),
        "dauerSekunden": wert(zusammenfassung, "DauerSekunden"),
        "streckeKm": wert(zusammenfassung, "StreckeKm"),
        "schlagloecher": wert(zusammenfassung, "Schlagloecher"),
        "kurven": wert(zusammenfassung, "Kurven"),
        "durchschnittsqualitaet": wert(zusammenfassung, "Durchschnittsqualitaet"),
        "gpsSnapshots": len(gps_zeilen),
        "gpsPositionGueltig": gueltig,
        "gpsRXUeberlaeufe": wert(ende, "GPSRXBufferOverflows"),
        "sdSchreibvorgaenge": wert(ende, "SDWrites"),
        "sdFehler": wert(ende, "SDErrors"),
        "loopMaxMs": wert(ende, "LoopMaxMs"),
        "flushMaxMs": wert(ende, "FlushMaxMs"),
        "gpsVerfehlteSlots": wert(ende, "GPSMissedSlots"),
        "sensorVerfehlteSlots": wert(ende, "SensorMissedSlots"),
        "hinweisQuantisierung": QUANTISIERUNG_HINWEIS,
        **vollstaendigkeit(ende, gps_zeilen),
        "marker-color": "#000000",
        "marker-size": "large",
        "marker-symbol": "star",
    }
    return [
        {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [lon, lat]},
            "properties": eigenschaften,
        }
    ]


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------


# Alle Ebenen in der Reihenfolge, in der sie erzeugt werden. Wer nur eine
# Frage beantworten will, blendet den Rest aus: Eine 52-km-Fahrt ergibt
# vollständig rund 2500 Merkmale, und geojson.io wird damit träge.
EBENEN = (
    "strecke",
    "referenz",
    "fahrbarkeit",
    "kurve",
    "schlagloch",
    "beifahrerurteil",
    "belagswechsel",
    "sitzung",
)


def exportieren(verzeichnis: str, modus: str, ebenen=None) -> dict:
    gps_zeilen = lade_csv(verzeichnis, "road_gps")
    if not gps_zeilen:
        raise SystemExit(f"Keine GPS-Datei in {verzeichnis}")

    road_zeilen = lade_csv(verzeichnis, "road_road")
    ereignisse = lade_csv(verzeichnis, "road_event")
    meta = lade_csv(verzeichnis, "road_meta")
    summary = lade_csv(verzeichnis, "road_summary")
    obd_zeilen = [
        z for z in lade_csv(verzeichnis, "road_obd") if "SpeedValid" in z
    ]

    punkte = gps_punkte(gps_zeilen)
    if not punkte:
        raise SystemExit("Keine gültige Position in der Sitzung")

    position = Zeitreihe([(t, [lon, lat]) for t, lon, lat, _ in punkte])
    qualitaet = Zeitreihe(
        [
            (als_ganzzahl(z["UptimeMs"]), als_zahl(z.get("Qualität")))
            for z in road_zeilen
            if als_ganzzahl(z.get("UptimeMs")) is not None
        ]
    )
    gps_speed = Zeitreihe(
        [
            (t, als_zahl(z.get("SpeedKmh")))
            for t, _lon, _lat, z in punkte
            if z.get("SpeedValid") == "1"
        ]
    )
    obd_speed = Zeitreihe(
        [
            (als_ganzzahl(z["UptimeMs"]), als_zahl(z.get("SpeedKmh")))
            for z in obd_zeilen
            if z.get("SpeedValid") == "1"
            and als_ganzzahl(z.get("UptimeMs")) is not None
        ]
    )

    gewaehlt = set(ebenen) if ebenen else set(EBENEN)

    features = []
    if "strecke" in gewaehlt:
        features += ebene_strecke(
            punkte, qualitaet, gps_speed, obd_speed, modus
        )
    if "referenz" in gewaehlt:
        features += ebene_referenzen(ereignisse, punkte)
    # Die Abschnitte liegen vor den Kurven, damit deren schmalere Bögen
    # darüber sichtbar bleiben.
    if "fahrbarkeit" in gewaehlt:
        features += ebene_abschnitte(ereignisse, punkte, position)
    if "kurve" in gewaehlt:
        features += ebene_kurven(ereignisse, punkte, position)
    if "schlagloch" in gewaehlt:
        features += ebene_schlagloecher(ereignisse, position)
    if gewaehlt & {"beifahrerurteil", "belagswechsel"}:
        features += [
            merkmal
            for merkmal in ebene_beifahrerurteile(ereignisse, position)
            if merkmal["properties"]["ebene"] in gewaehlt
        ]
    if "sitzung" in gewaehlt:
        features += ebene_sitzungskopf(
            verzeichnis, meta, summary, punkte, gps_zeilen
        )

    return {"type": "FeatureCollection", "features": features}


def main() -> int:
    zerleger = argparse.ArgumentParser(
        description="ROADTEST-Messsitzung nach GeoJSON für geojson.io wandeln"
    )
    zerleger.add_argument("sitzung", help="Verzeichnis der Sitzung in testdata/")
    zerleger.add_argument(
        "-o", "--ausgabe", help="Zieldatei (Vorgabe: <Sitzung>.geojson)"
    )
    zerleger.add_argument(
        "--modus",
        choices=("qualitaet", "geschwindigkeit"),
        default="qualitaet",
        help="Einfärbung der Strecke: Straßenqualität oder Abweichung "
        "zwischen GPS- und OBD-Geschwindigkeit",
    )
    zerleger.add_argument(
        "--ebenen",
        help="Kommaliste der auszugebenden Ebenen; Vorgabe sind alle. "
        "Möglich sind " + ", ".join(EBENEN) + ".",
    )
    zerleger.add_argument(
        "--nur-fahrbarkeit",
        action="store_true",
        help="Kurzform für --ebenen fahrbarkeit,beifahrerurteil,"
        "belagswechsel,sitzung",
    )
    argumente = zerleger.parse_args()

    if argumente.nur_fahrbarkeit and argumente.ebenen:
        zerleger.error("--nur-fahrbarkeit und --ebenen schließen sich aus")
    if argumente.nur_fahrbarkeit:
        ebenen = ["fahrbarkeit", "beifahrerurteil", "belagswechsel", "sitzung"]
    elif argumente.ebenen:
        ebenen = [
            teil.strip() for teil in argumente.ebenen.split(",") if teil.strip()
        ]
        unbekannt = [teil for teil in ebenen if teil not in EBENEN]
        if unbekannt:
            zerleger.error(
                "Unbekannte Ebene: "
                + ", ".join(unbekannt)
                + ". Möglich sind "
                + ", ".join(EBENEN)
                + "."
            )
    else:
        ebenen = None

    sammlung = exportieren(argumente.sitzung, argumente.modus, ebenen)
    name = os.path.basename(os.path.normpath(argumente.sitzung))
    # Der Dateiname nennt die Auswahl, damit mehrere Exporte derselben
    # Sitzung nebeneinander bestehen. Das Präfix "ebenen-" trennt die freie
    # Auswahl von der Kurzform, die sonst denselben Namen erzeugen könnte.
    kennung = argumente.modus
    if argumente.nur_fahrbarkeit:
        kennung = "fahrbarkeit"
    elif ebenen:
        kennung = "ebenen-" + "-".join(ebenen)
    ziel = argumente.ausgabe or f"{name}_{kennung}.geojson"
    with open(ziel, "w") as fh:
        # allow_nan=False: Python schriebe sonst NaN oder Infinity, was kein
        # gültiges JSON ist und von geojson.io abgelehnt würde. Ein Fehler
        # hier ist einem stillschweigend unbrauchbaren Ergebnis vorzuziehen.
        json.dump(sammlung, fh, ensure_ascii=False, allow_nan=False)

    nach_ebene: dict[str, int] = {}
    for merkmal in sammlung["features"]:
        ebene = merkmal["properties"].get("ebene", "?")
        nach_ebene[ebene] = nach_ebene.get(ebene, 0) + 1
    groesse = os.path.getsize(ziel) / 1024.0

    print(f"{ziel}  ({groesse:.0f} kB, {len(sammlung['features'])} Features)")
    for ebene in sorted(nach_ebene):
        print(f"   {ebene:12s} {nach_ebene[ebene]}")
    print("\nDatei in https://geojson.io hineinziehen.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

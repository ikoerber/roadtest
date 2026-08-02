#!/usr/bin/env python3
"""Vergleicht die Kurvenerkennung einer Hin- und Rueckfahrt derselben Strecke.

Zweck: eine Reproduzierbarkeitsmessung, die ohne von Hand markierte
Referenzintervalle auskommt. Wer dieselbe Strecke hin und zurueck faehrt,
durchfaehrt jede Kurve zweimal - mit anderer Richtung, anderer Linie und
anderer Geschwindigkeit. Stimmen die Winkel beider Durchgaenge ueberein,
misst die Erkennung die Strasse und nicht die Fahrweise.

Aufruf aus dem Projektverzeichnis:

    python3 tools/vergleich_hin_rueck.py testdata/20260802_095708_84FD688D

Die Zuordnung laeuft ueber die Kurvenfolge, nicht ueber die Position. Das ist
Absicht: Ereignisse tragen nur dann eine Koordinate, wenn GPS einen Fix hatte,
und eine Koppelnavigation aus Gierrate und Geschwindigkeit driftet ueber
mehrere Minuten zu stark. Die Folge der Kurven ist dagegen ein Fingerabdruck
der Strecke. Der Rueckweg wiederholt sie rueckwaerts und spiegelverkehrt, das
Vorzeichen jeder Kurve kippt.

Als Wendepunkt gilt die engste Kurve im mittleren Drittel der Fahrt; ein
Wendemanoever faellt durch kleinen Radius bei niedriger Geschwindigkeit auf.
Die Ausrichtung sucht anschliessend den Versatz in der umgekehrten
Hinwegfolge, der die meisten Paare im Toleranzband trifft.

Eine beim Messende abgeschnittene Kurve (`CompletionReason=SESSION_END`)
zaehlt nicht in die Statistik: Sie ist absichtlich zu kurz.
"""

import csv
import glob
import os
import statistics
import sys

# Ein Paar gilt als Treffer, wenn das Vorzeichen kippt und der Winkelbetrag
# innerhalb dieses Bandes liegt. Absolutwert fuer kleine Kurven, sonst relativ.
TOLERANZ_GRAD = 12.0
TOLERANZ_ANTEIL = 0.35


def lade(verzeichnis, muster):
    treffer = glob.glob(os.path.join(verzeichnis, muster))
    if not treffer:
        raise SystemExit(f"Datei {muster} in {verzeichnis} nicht gefunden")
    with open(treffer[0], newline="") as datei:
        return list(csv.DictReader(datei))


def kurven(verzeichnis):
    zeilen = [z for z in lade(verzeichnis, "road_event_*.csv")
              if z["Ereignis"] == "KURVE"]
    if not zeilen:
        raise SystemExit("Keine Kurvenereignisse in der Sitzung")
    # Zeitbasisversatz wie in tools/vergleich_kurvenwiedergabe.cpp: das
    # Ereignis wird erst nach dem Ruhefenster geschrieben, sein UptimeMs liegt
    # also spaeter als sein EndUptimeMs. Das Maximum trifft den Versatz.
    versatz = max(int(z["EndUptimeMs"]) - int(z["UptimeMs"]) for z in zeilen)
    ergebnis = []
    for nummer, z in enumerate(zeilen, start=1):
        vorzeichen = -1 if z["Direction"] == "NEGATIV" else 1
        ergebnis.append({
            "nr": nummer,
            "t0": (int(z["StartUptimeMs"]) - versatz) / 1000.0,
            "t1": (int(z["EndUptimeMs"]) - versatz) / 1000.0,
            "winkel": float(z["AngleDeg"]) * vorzeichen,
            "v": float(z["MeanSpeedKmh"]),
            "ay": float(z["MeanLateralAccel"]),
            "radius": float(z["RadiusM"]),
            "weg": float(z["DistanceM"]),
            "modus": z["DetectionMode"],
            "ende": z["CompletionReason"],
        })
    return ergebnis


def wendepunkt(alle):
    dauer = alle[-1]["t1"]
    mitte = [k for k in alle if dauer / 4 < k["t0"] < dauer * 3 / 4]
    if not mitte:
        raise SystemExit("Kein Wendemanoever im mittleren Fahrtabschnitt")
    return min(mitte, key=lambda k: k["radius"])


def ausrichten(hin, rueck):
    """Sucht den Versatz in der umgekehrten Hinwegfolge mit den meisten Treffern."""
    gespiegelt = list(reversed(hin))
    bestes = (0, [], 0)
    for versatz in range(len(gespiegelt)):
        anzahl = min(len(rueck), len(gespiegelt) - versatz)
        if anzahl < 3:
            break
        treffer = []
        for j in range(anzahl):
            a, b = gespiegelt[versatz + j], rueck[j]
            if (a["winkel"] > 0) == (b["winkel"] > 0):
                continue
            grenze = max(TOLERANZ_GRAD, TOLERANZ_ANTEIL * abs(a["winkel"]))
            if abs(abs(b["winkel"]) - abs(a["winkel"])) < grenze:
                treffer.append(j)
        if len(treffer) > len(bestes[1]):
            bestes = (versatz, treffer, anzahl)
    return gespiegelt, bestes


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f"Aufruf: {sys.argv[0]} <Sitzungsverzeichnis>")
    verzeichnis = sys.argv[1]
    alle = kurven(verzeichnis)
    wende = wendepunkt(alle)
    hin = [k for k in alle if k["t1"] <= wende["t0"]]
    rueck = [k for k in alle if k["t0"] >= wende["t1"]]
    if not hin or not rueck:
        raise SystemExit("Hin- oder Rueckweg enthaelt keine Kurven")

    print(f"{os.path.basename(verzeichnis.rstrip('/'))}: {len(alle)} Kurven")
    print(f"Wendemanoever: Kurve {wende['nr']} bei t={wende['t0']:.0f} s, "
          f"{wende['winkel']:+.0f} Grad, R={wende['radius']:.1f} m, "
          f"v={wende['v']:.0f} km/h")
    print(f"Hinweg {len(hin)} Kurven, Rueckweg {len(rueck)} Kurven")

    gespiegelt, (versatz, treffer, anzahl) = ausrichten(hin, rueck)
    print(f"Ausrichtung: Rueckkurve 1 entspricht Hinkurve "
          f"{gespiegelt[versatz]['nr']}, {len(treffer)} von {anzahl} Paaren "
          f"im Toleranzband\n")

    kopf = (f"{'Hin':>3} {'Rue':>3} | {'Wnk_h':>7} {'Wnk_r':>7} {'dWnk':>6} | "
            f"{'v_h':>4} {'v_r':>4} | {'ay_h':>5} {'ay_r':>5} | "
            f"{'R_h':>6} {'R_r':>6} | {'Mod_h':>5} {'Mod_r':>5} | {'Ende_r':>11}")
    print(kopf)
    print("-" * len(kopf))

    winkel, radius, quer, tempo, modus = [], [], [], [], 0
    for j in range(anzahl):
        a, b = gespiegelt[versatz + j], rueck[j]
        differenz = abs(b["winkel"]) - abs(a["winkel"])
        gewertet = b["ende"] != "SESSION_END"
        if gewertet:
            winkel.append(differenz)
            radius.append(100 * (b["radius"] - a["radius"]) / a["radius"])
            quer.append(100 * (b["ay"] - a["ay"]) / a["ay"])
            tempo.append(100 * (b["v"] - a["v"]) / a["v"])
            modus += a["modus"] != b["modus"]
        hinweis = "" if j in treffer else "  <-- kein Treffer"
        if not gewertet:
            hinweis = "  <-- beim Stopp abgeschnitten, nicht gewertet"
        print(f"{a['nr']:>3} {b['nr']:>3} | {a['winkel']:>+7.1f} "
              f"{b['winkel']:>+7.1f} {differenz:>+6.1f} | {a['v']:>4.0f} "
              f"{b['v']:>4.0f} | {a['ay']:>5.2f} {b['ay']:>5.2f} | "
              f"{a['radius']:>6.1f} {b['radius']:>6.1f} | {a['modus'][:5]:>5} "
              f"{b['modus'][:5]:>5} | {b['ende']:>11}{hinweis}")

    if not winkel:
        raise SystemExit("\nKein wertbares Paar")

    def streuung(name, werte, einheit):
        median = statistics.median(abs(w) for w in werte)
        groesst = max(werte, key=abs)
        print(f"{name:<22} Median {median:>5.1f} {einheit}, "
              f"groesste {groesst:>+6.1f} {einheit}")

    print(f"\n{len(winkel)} gewertete Paare")
    streuung("Winkel", winkel, "Grad")
    streuung("Winkel relativ", [100 * w / abs(a["winkel"]) for w, a in
                                zip(winkel, gespiegelt[versatz:])], "%")
    streuung("Radius", radius, "%")
    streuung("Querbeschleunigung", quer, "%")
    streuung("Geschwindigkeit", tempo, "%")
    print(f"\nDetectionMode weicht in {modus} von {len(winkel)} Paaren ab. "
          f"Das Feld beschreibt den Erkennungspfad, nicht die Kurve.")


if __name__ == "__main__":
    main()

# RoboTwin-Programmformat

Version **1**

Ein Programm ist eine Liste von Punkten, die der Roboter nacheinander anfährt
(Teach-Pendant-Prinzip). Programme werden als JSON-Dateien gespeichert, im Editor im Ordner
[`Programs/`](../Programs) im Repo – so lassen sie sich versionieren, im Diff lesen und auch
von Hand oder per Skript erzeugen.

## Aufbau

```json
{
  "format": "robotwin-program",
  "version": 1,
  "name": "Pick & Place",
  "points": [
    {"name":"Bereitschaft","joints":[0.0,-90.0,90.0,-90.0,-90.0,0.0],"speed":1.0,"wait":0.0},
    {"name":"Greifen","joints":[120.091,-80.529,131.806,-141.277,-90.0,210.091],"speed":0.2,"wait":0.5}
  ]
}
```

| Feld | Typ | Pflicht | Bedeutung |
|---|---|---|---|
| `format` | Text | ja | immer `"robotwin-program"` – kennzeichnet die Datei |
| `version` | Zahl | ja | Formatversion, derzeit `1` |
| `name` | Text | ja | Anzeigename des Programms |
| `points` | Liste | ja | die Punkte in Abfahrreihenfolge (darf leer sein) |

### Punkt

| Feld | Typ | Pflicht | Bedeutung |
|---|---|---|---|
| `name` | Text | ja | Anzeigename, z. B. `"Über dem Teil"` |
| `joints` | Zahl[6] | ja | Gelenkwinkel in **Grad**, URDF-Konvention, Reihenfolge Basis → Handgelenk 3 |
| `speed` | Zahl | nein (1.0) | Anteil der Maximalgeschwindigkeit, `0 < speed ≤ 1` |
| `wait` | Zahl | nein (0.0) | Wartezeit in Sekunden **nach** Ankunft (z. B. „Greifer schließen“) |

Einheiten und Wertebereiche sind dieselben wie bei `move_joints` im [Protokoll](PROTOCOL.md):
Ein Programmpunkt ist im Kern ein gespeicherter `move_joints`-Befehl plus Name und Wartezeit.

## Regeln

- **Gelenkwinkel, keine TCP-Pose.** Ein Punkt speichert die Armhaltung exakt; beim Abspielen ist
  keine inverse Kinematik nötig, jeder Punkt ist genau so erreichbar, wie er geteacht wurde.
- **Keine abgeleiteten Werte.** Die TCP-Position wird in der Bedienoberfläche angezeigt, aber
  nicht gespeichert – sie folgt aus den Winkeln (Vorwärtskinematik). So können Datei und Anzeige
  nie widersprüchlich sein.
- **Eine Zeile pro Punkt.** So zeigt ein Git-Diff genau, welcher Punkt sich geändert hat.
- Zahlen mit **Punkt** als Dezimaltrennzeichen, UTF-8 ohne BOM.
- Liegen Winkel außerhalb der Achsgrenzen, lehnt der Controller den Punkt beim Abspielen ab
  (`joint_limit`) und das Programm bricht ab.

## Abspielen

Unity sendet die Punkte nacheinander als `move_joints` und wartet jeweils, bis der Roboter
am Ziel steht (`idle` und Ist-Winkel = Punkt) und die Wartezeit vorbei ist.
Das Programm bricht ab bei: abgelehntem Befehl, Not-Halt, Verbindungsverlust oder Verlust der
Steuerung. **Pause** hält an und fährt beim Fortsetzen den unterbrochenen Punkt erneut an;
**Stop** hält an und beginnt beim nächsten Start von vorn.

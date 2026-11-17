# RoboTwin-Controller

Simulierter Robotercontroller in Python 3 (≥ 3.10), nur Standardbibliothek.
Spricht das Protokoll aus [`docs/PROTOCOL.md`](../docs/PROTOCOL.md).


## Aufbau

| Datei | Inhalt |
|---|---|
| `robotwin_controller/motion.py` | synchrones Trapezprofil (Python-Fassung des Unity-`JointMotionSimulator`) |
| `robotwin_controller/robot.py` | Achsgrenzen aus `ur5.urdf`, Befehle prüfen (ablehnen statt begrenzen) und ausführen |
| `robotwin_controller/protocol.py` | JSON-Zeilen ↔ Nachrichten, prüft die Form (Felder, Typen) |
| `robotwin_controller/server.py` | asyncio-TCP-Server: Clients, Rollen, 50-Hz-Zustandstakt |
| `robotwin_controller/__main__.py` | Start über die Kommandozeile |
| `client.py` | Test-Client: fährt den Roboter ohne Unity (interaktiv oder `--demo`); kennt nur das Protokoll |
| `tests/` | Unit-Tests (`unittest`) |

## Starten

Im Ordner `controller/`:

```bash
python -m robotwin_controller
```

Optionen (`--help` zeigt alle): `--host`, `--port` (Standard 5005), `--urdf`, `--acceleration`,
`--rate` (Standard 50 Hz), `--verbose`. Beenden mit Strg+C.

## Test-Client

Bei laufendem Controller, in einem **zweiten** Konsolenfenster im Ordner `controller/`:

```bash
python client.py --demo
```

spielt eine feste Abfolge ab (Bereitschaftsstellung, Testpose, abgelehntes Ziel, Override, Stop, Home).

```bash
python client.py
```

startet den interaktiven Modus; `help` zeigt die Befehle (`move`, `home`, `stop`, `override`,
`state`, `watch`, `raw`, `quit`).

## Tests ausführen

Im Ordner `controller/`:

```bash
python -m unittest -v
```

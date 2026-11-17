# RoboTwin-Controller

Simulierter Robotercontroller in Python 3 (≥ 3.10), nur Standardbibliothek.
Spricht das Protokoll aus [`docs/PROTOCOL.md`](../docs/PROTOCOL.md).

> In Arbeit (Meilenstein M5). Vorhanden: Bewegungssimulation, Roboter und Protokoll mit Tests.

## Aufbau

| Datei | Inhalt |
|---|---|
| `robotwin_controller/motion.py` | synchrones Trapezprofil (Python-Fassung des Unity-`JointMotionSimulator`) |
| `robotwin_controller/robot.py` | Achsgrenzen aus `ur5.urdf`, Befehle prüfen (ablehnen statt begrenzen) und ausführen |
| `robotwin_controller/protocol.py` | JSON-Zeilen ↔ Nachrichten, prüft die Form (Felder, Typen) |
| `tests/` | Unit-Tests (`unittest`) |

## Tests ausführen

Im Ordner `controller/`:

```bash
python -m unittest -v
```

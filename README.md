# RoboTwin

Digitaler Zwilling eines 6-Achs-Roboterarms (UR5) in Unity, gesteuert über eine
austauschbare Netzwerkschicht durch einen simulierten Robotercontroller in Python.

## Zweck

<!-- TODO (Yannick): 2–3 Sätze in eigenen Worten: Was ist RoboTwin, für wen, warum? -->

## Repo-Struktur

| Ordner | Inhalt |
|---|---|
| `RoboTwin-Unity/` | Unity-Projekt (Unity 6000.6, URP, UI Toolkit) |
| `controller/` | Python-Controller (asyncio-TCP-Server) und Test-Client |
| `docs/` | Protokollbeschreibung (`PROTOCOL.md`), Architekturdiagramm |

## Architektur

<!-- M4/M8: Diagramm Modell / Kinematik / Verbindung / UI, IRobotConnection -->

## Setup

<!-- M8: Schritt-für-Schritt-Anleitung, Ziel: in 10 Minuten lauffähig -->

## Protokoll

Siehe [`docs/PROTOCOL.md`](docs/PROTOCOL.md).

## Grenzen

<!-- M8 -->

## Ausblick

<!-- M8: z. B. ROS 2 oder echter Servo-Arm als weitere IRobotConnection -->

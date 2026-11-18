# RoboTwin

Digitaler Zwilling eines 6-Achs-Roboterarms (UR5) in Unity, gesteuert über eine
austauschbare Netzwerkschicht durch einen simulierten Robotercontroller in Python.

## Zweck

RoboTwin ist ein digitaler Zwilling eines 6-Achs-Industrieroboters in Unity. Der Arm lässt sich über eine Bedienoberfläche achsweise oder über Zielkoordinaten steuern und Bewegungsprogramme abfahren. Über eine austauschbare Netzwerkschicht spricht die Simulation per TCP mit einem Robotercontroller in Python und zeigt dabei immer den gemeldeten Ist-Zustand an.

Das Projekt zeigt, wie sich Game-Engine-Technologie für Robotik und Automatisierung einsetzen lässt: zum Testen von Abläufen ohne echte Hardware, für Schulungen und als Grundlage, um später einen realen Roboter anzubinden.

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

<!-- M8: ausformulieren. Notizen aus der Entwicklung:
  - Bewegungssimulation (JointMotionSimulator): Ankunft durch diskrete Frames 2–5 % früher
    als theoretisch; im letzten Frame springt die Geschwindigkeit von wenigen °/s auf 0.
  - Zielwechsel während der Fahrt: aktuelle Geschwindigkeit wird auf die neue Richtung
    projiziert; bei Richtungsumkehr (Projektion < 0) bremst die Simulation abrupt auf 0.
  - Override begrenzt nur die Geschwindigkeit, nicht die Beschleunigung (sonst kann der Arm
    bei Override-Reduktion während der Fahrt nicht rechtzeitig bremsen).
  - Beschleunigung (180 °/s²) ist frei gewählt, die URDF enthält keine Werte.
  - Python-Controller unter Windows: Timer-Auflösung ~15,6 ms, daher schwankt der Abstand
    zwischen zwei state-Nachrichten (15–31 ms); im Mittel 50 Hz (feste Zielzeitpunkte).
  - Protokoll ohne Anmeldung/Verschlüsselung: Controller lauscht standardmäßig nur auf 127.0.0.1.
  - Not-Halt ist Stopp-Kategorie 0 (sofort stehen); echte Roboter bremsen oft kontrolliert (Kat. 1).
  - Programme laufen in Unity (ProgramRunner) – trennt sich Unity, endet das Programm
    (sicher: Controller hält beim Verlust des Besitzers an).
  - Kollisionsprüfung (Kapselmodell): Kapseln liegen auf der kinematischen Kette (Gelenkursprünge);
    die seitlichen Versätze der echten UR-Glieder (Schulter ~13 cm, Ellbogen ~12 cm) sind nur über
    die Radien abgedeckt. Keine Selbstkollision, kein Werkzeug/Greifer. Hindernisse werden beim
    Start einmal eingelesen (statisch). Geprüft wird nur in Unity – der Python-Controller kennt den
    Tisch nicht; die laufende Überwachung greift nur, solange Unity verbunden ist.
  - URDF-Grenzen vorsichtiger als UR5-Datenblatt: Handgelenke 180 statt 360 °/s,
    Ellbogen ±180° statt ±360°.
-->

<!-- M8 -->

## Ausblick

<!-- M8: z. B. ROS 2 oder echter Servo-Arm als weitere IRobotConnection
  - Programme im Controller ausführen (Entscheidung A2 in M7): Unity lädt das Programm hoch
    ("run_program"), Python spielt es ab und meldet den Fortschritt – läuft dann auch ohne UI weiter.
  - "take_control": Steuerung übernehmen statt Rollen nach Verbindungsreihenfolge (R2 in M6).
  - move_linear: lineare TCP-Bahnen (IK in Python oder Bahn in Unity vorberechnen).
-->

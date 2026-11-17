"""Der simulierte Roboter: Achsgrenzen aus der URDF, Befehle prüfen und ausführen.

Python-Gegenstück zur SimulatedConnection in Unity (Assets/Scripts/Connection/SimulatedConnection.cs):
Befehle werden ABGELEHNT statt still begrenzt – der Roboter fährt genau wie befohlen oder gar nicht.

Aufgabenteilung:
* protocol.py prüft die FORM einer Nachricht (Felder vorhanden, richtige Typen),
* robot.py prüft die BEDEUTUNG (Wertebereiche, Achsgrenzen) und führt aus.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional, Sequence

from .motion import JointMotionSimulator

# robot.py → robotwin_controller → controller → Repo-Wurzel
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = REPO_ROOT / "RoboTwin-Unity" / "Assets" / "Robots" / "UR5" / "ur5.urdf"

# Die URDF enthält keine Beschleunigungen – derselbe Wert wie in Unity (RobotConnectionHost).
DEFAULT_ACCELERATION = 180.0   # Grad/s²


class CommandError(Exception):
    """Ein Befehl wurde abgelehnt. 'code' ist maschinenlesbar (PROTOCOL.md, Abschnitt 7)."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class RobotStatus(str, Enum):
    """Status für die state-Nachricht. Erbt von str, damit der Wert direkt ins JSON passt."""

    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"


@dataclass(frozen=True)
class JointLimits:
    """Grenzen eines Drehgelenks, in Grad bzw. Grad/s und Grad/s²."""

    name: str
    min: float
    max: float
    max_speed: float
    max_acceleration: float


@dataclass(frozen=True)
class RobotState:
    """Ist-Zustand für die state-Nachricht."""

    joints: tuple[float, ...]
    status: RobotStatus
    override: float
    timestamp: float


def load_joint_limits(urdf_path: Path | str, acceleration: float = DEFAULT_ACCELERATION) -> list[JointLimits]:
    """Liest alle Drehgelenke (type="revolute") aus einer URDF, geordnet von der Basis zur Hand.

    Die URDF speichert Winkel in Radiant – das Protokoll verwendet Grad, daher die Umrechnung hier.
    """
    root = ET.parse(urdf_path).getroot()

    # Elternbeziehungen: Kind-Link → Eltern-Link (jedes Gelenk verbindet genau zwei Links).
    parent_of = {}
    for joint in root.findall("joint"):
        parent_of[joint.find("child").get("link")] = joint.find("parent").get("link")

    def depth(link: str) -> int:
        """Abstand eines Links von der Wurzel der Kette."""
        steps = 0
        while link in parent_of:
            link = parent_of[link]
            steps += 1
        return steps

    revolute = []
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"Gelenk {name}: <limit> fehlt in der URDF")

        # float() liest in Python IMMER mit Punkt – anders als double.Parse in C# ohne InvariantCulture.
        limits = JointLimits(
            name=name,
            min=math.degrees(float(limit.get("lower"))),
            max=math.degrees(float(limit.get("upper"))),
            max_speed=math.degrees(float(limit.get("velocity"))),
            max_acceleration=acceleration,
        )
        revolute.append((depth(joint.find("child").get("link")), limits))

    # Nach Tiefe sortieren: Reihenfolge entlang der Kette, unabhängig von der Reihenfolge in der Datei.
    revolute.sort(key=lambda entry: entry[0])
    depths = [d for d, _ in revolute]
    if len(set(depths)) != len(depths):
        raise ValueError("URDF ist keine einfache Kette (zwei Drehgelenke auf gleicher Tiefe)")
    return [limits for _, limits in revolute]


class Robot:
    """Simulierter Roboter: prüft Befehle und führt sie auf der Bewegungssimulation aus."""

    def __init__(self, limits: Sequence[JointLimits], start_angles: Optional[Sequence[float]] = None) -> None:
        self.limits = tuple(limits)
        count = len(self.limits)

        start = list(start_angles) if start_angles is not None else [0.0] * count
        if len(start) != count:
            raise ValueError(f"Erwartet {count} Startwinkel, erhalten {len(start)}")
        for i, (angle, joint) in enumerate(zip(start, self.limits)):
            if not joint.min <= angle <= joint.max:
                raise ValueError(f"Startwinkel von Gelenk {i + 1} außerhalb der Grenzen")

        self._motion = JointMotionSimulator(
            [j.min for j in self.limits],
            [j.max for j in self.limits],
            [j.max_speed for j in self.limits],
            [j.max_acceleration for j in self.limits],
            start,
        )
        self._command_speed = 1.0    # Geschwindigkeit des laufenden Befehls (0..1)
        self._override = 100.0       # globaler Override in Prozent
        self._time = 0.0             # "Uhr des Controllers" in Sekunden

    @classmethod
    def from_urdf(cls, urdf_path: Path | str = DEFAULT_URDF, acceleration: float = DEFAULT_ACCELERATION) -> "Robot":
        return cls(load_joint_limits(urdf_path, acceleration))

    @property
    def joint_count(self) -> int:
        return len(self.limits)

    # ---- Befehle ----

    def move_joints(self, joints: Sequence[float], speed: float = 1.0) -> None:
        """Alle Gelenke synchron zu den Zielwinkeln fahren. Wirft CommandError bei ungültigen Werten."""
        if len(joints) != self.joint_count:
            raise CommandError("invalid_command", f"Erwartet {self.joint_count} Gelenkwinkel, erhalten {len(joints)}.")

        # So formuliert, dass auch NaN abgelehnt wird (jeder Vergleich mit NaN ist False).
        if not 0.0 < speed <= 1.0:
            raise CommandError("invalid_command", f"Geschwindigkeit {speed} außerhalb (0, 1].")

        for i, (angle, joint) in enumerate(zip(joints, self.limits)):
            if math.isnan(angle) or math.isinf(angle):
                raise CommandError("invalid_command", f"Gelenk {i + 1}: ungültiger Wert {angle}.")
            if not joint.min <= angle <= joint.max:
                raise CommandError(
                    "joint_limit",
                    f"Gelenk {i + 1}: {angle:.1f}° außerhalb [{joint.min:.0f}°, {joint.max:.0f}°].",
                )

        # Erst ALLES prüfen, dann ausführen: Ein abgelehnter Befehl darf nichts verändert haben.
        self._command_speed = speed
        self._apply_speed()
        self._motion.set_targets(joints)

    def stop(self) -> None:
        """Sofort anhalten."""
        self._motion.stop()

    def set_override(self, percent: float) -> None:
        """Override 1 … 100 %, wirkt auch auf eine laufende Bewegung."""
        if not 1.0 <= percent <= 100.0:
            raise CommandError("invalid_command", f"Override {percent} außerhalb [1, 100] %.")
        self._override = float(percent)
        self._apply_speed()

    # ---- Simulation ----

    def step(self, dt: float) -> None:
        """Simuliert dt Sekunden."""
        self._motion.step(dt)
        self._time += dt

    def state(self) -> RobotState:
        status = RobotStatus.MOVING if self._motion.is_moving else RobotStatus.IDLE
        return RobotState(self._motion.current, status, self._override, self._time)

    def _apply_speed(self) -> None:
        self._motion.speed_factor = self._command_speed * self._override / 100.0

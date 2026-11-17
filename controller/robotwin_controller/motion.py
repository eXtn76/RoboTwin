"""Bewegungssimulation für einen Roboterarm mit Drehgelenken.

Python-Fassung des JointMotionSimulator aus Unity (Assets/Scripts/Model/JointMotionSimulator.cs):

* synchron: alle Gelenke liegen auf EINER geraden Bahn im Gelenkraum, beschrieben durch den
  Fortschritt s (0 = Start, 1 = Ziel) – daher starten und stoppen sie gleichzeitig,
* Trapezprofil: beschleunigen – konstant fahren – bremsen (bzw. Dreieck bei kurzen Wegen),
* Achsgrenzen: Ziele werden auf die Grenzen begrenzt (die Prüfung mit Ablehnung macht robot.py).

Reine Logik ohne Uhr und ohne Netzwerk: Die Zeit kommt von außen über step(dt) – daher testbar.
Winkel in Grad, Geschwindigkeiten in Grad/s, Beschleunigungen in Grad/s².
"""

from __future__ import annotations

import math
from typing import Sequence

MIN_DISTANCE = 1e-4      # kürzere Wege (Grad) gelten als "schon da"
ARRIVAL_EPSILON = 1e-6   # Toleranz für s = 1 (Rundungsfehler)


class JointMotionSimulator:
    """Simuliert die Gelenkbewegung: synchron, Trapezprofil, Achsgrenzen."""

    def __init__(
        self,
        min_angles: Sequence[float],
        max_angles: Sequence[float],
        max_speeds: Sequence[float],
        max_accelerations: Sequence[float],
        start_angles: Sequence[float],
    ) -> None:
        count = len(start_angles)
        for name, values in (
            ("min_angles", min_angles),
            ("max_angles", max_angles),
            ("max_speeds", max_speeds),
            ("max_accelerations", max_accelerations),
        ):
            if len(values) != count:
                raise ValueError(f"{name}: erwartet {count} Werte, erhalten {len(values)}")

        # list(...) kopiert: Ändert der Aufrufer später seine Liste, bleibt unser Zustand unberührt.
        self._min = list(min_angles)
        self._max = list(max_angles)
        self._max_speeds = list(max_speeds)
        self._max_accelerations = list(max_accelerations)

        self._current = list(start_angles)   # Ist-Winkel
        self._velocity = [0.0] * count       # Ist-Geschwindigkeit
        self._target = list(start_angles)    # Soll-Winkel
        self._start = list(start_angles)     # Startpunkt der aktuellen Bahn

        # Zustand der Bahn – EIN Wert für alle Gelenke
        self._s = 1.0           # Fortschritt: 0 = Start, 1 = Ziel
        self._s_dot = 0.0       # Fortschrittsgeschwindigkeit (1/s)
        self._s_dot_max = 0.0   # erlaubte Fortschrittsgeschwindigkeit
        self._s_ddot_max = 0.0  # erlaubte Fortschrittsbeschleunigung

        self._speed_factor = 1.0

    # ---- Abfragen ----

    @property
    def joint_count(self) -> int:
        return len(self._current)

    @property
    def current(self) -> tuple[float, ...]:
        """Ist-Winkel (als Tupel: Aufrufer können unseren Zustand nicht verändern)."""
        return tuple(self._current)

    @property
    def velocity(self) -> tuple[float, ...]:
        return tuple(self._velocity)

    @property
    def target(self) -> tuple[float, ...]:
        return tuple(self._target)

    @property
    def is_moving(self) -> bool:
        return self._s < 1.0

    @property
    def speed_factor(self) -> float:
        """Override 0.01 … 1. Begrenzt nur die Geschwindigkeit, nicht die Beschleunigung."""
        return self._speed_factor

    @speed_factor.setter
    def speed_factor(self, value: float) -> None:
        self._speed_factor = min(max(value, 0.01), 1.0)
        self._replan()   # wirkt auch auf eine laufende Bewegung

    # ---- Befehle ----

    def set_target(self, joint: int, angle: float) -> None:
        """Neues Ziel für ein einzelnes Gelenk."""
        self._target[joint] = self._clamp(joint, angle)
        self._replan()

    def set_targets(self, angles: Sequence[float]) -> None:
        """Neue Ziele für alle Gelenke – EIN Replan, also eine gemeinsame Bahn."""
        if len(angles) != self.joint_count:
            raise ValueError(f"Erwartet {self.joint_count} Winkel, erhalten {len(angles)}")
        for i, angle in enumerate(angles):
            self._target[i] = self._clamp(i, angle)
        self._replan()

    def stop(self) -> None:
        """Sofort anhalten: Ziel = aktuelle Position."""
        self._target = list(self._current)
        self._replan()   # kein Weg mehr → s = 1, Geschwindigkeiten 0

    def step(self, dt: float) -> None:
        """Simuliert dt Sekunden Bewegung."""
        if dt < 0:
            raise ValueError("dt darf nicht negativ sein")
        if self._s >= 1.0:
            return

        remaining = 1.0 - self._s
        # Höchste Geschwindigkeit, bei der wir noch genau bei s = 1 zum Stehen kommen (v² = 2·a·d).
        brake_speed = math.sqrt(2.0 * self._s_ddot_max * remaining)

        if self._s_dot > brake_speed:
            # 1. Zu schnell, um rechtzeitig zu stoppen → auf der Bremskurve bleiben.
            self._s_dot = brake_speed
        elif self._s_dot < self._s_dot_max:
            # 2. Beschleunigen – nie über die Höchstgeschwindigkeit, nie über die Bremskurve.
            self._s_dot = min(self._s_dot + self._s_ddot_max * dt, self._s_dot_max, brake_speed)
        else:
            # 3. Schneller als erlaubt (Override wurde gesenkt) → sanft herunterbremsen.
            self._s_dot = max(self._s_dot - self._s_ddot_max * dt, self._s_dot_max)

        self._s += self._s_dot * dt
        if self._s >= 1.0 - ARRIVAL_EPSILON:
            self._s = 1.0
            self._s_dot = 0.0
        self._update_joints()

    # ---- intern ----

    def _clamp(self, joint: int, angle: float) -> float:
        return min(max(angle, self._min[joint]), self._max[joint])

    def _replan(self) -> None:
        """Plant eine neue Bahn vom aktuellen Ist-Zustand zum Ziel."""
        # 1. Neue Bahn beginnt an der aktuellen Position. Nebenbei: Skalarprodukt aus
        #    aktueller Geschwindigkeit und neuer Richtung (für Schritt 4).
        dot = 0.0
        length_sq = 0.0
        for i in range(self.joint_count):
            self._start[i] = self._current[i]
            delta = self._target[i] - self._start[i]
            dot += self._velocity[i] * delta
            length_sq += delta * delta

        # 2. Gelenkgrenzen in Grenzen für s umrechnen: Gelenk i bewegt sich mit |Δi|·ṡ.
        #    Das muss für ALLE Gelenke gelten → jeweils das Minimum.
        self._s_dot_max = math.inf
        self._s_ddot_max = math.inf
        has_motion = False
        for i in range(self.joint_count):
            distance = abs(self._target[i] - self._start[i])
            if distance < MIN_DISTANCE:
                continue   # dieses Gelenk bewegt sich nicht (und 0 im Nenner wäre fatal)
            has_motion = True
            self._s_dot_max = min(self._s_dot_max, self._max_speeds[i] * self._speed_factor / distance)
            self._s_ddot_max = min(self._s_ddot_max, self._max_accelerations[i] / distance)

        # 3. Kein Weg mehr: exakt aufs Ziel setzen und stehen bleiben.
        if not has_motion:
            self._current = list(self._target)
            self._velocity = [0.0] * self.joint_count
            self._s = 1.0
            self._s_dot = 0.0
            return

        # 4. Neue Bahn starten. Anfangsgeschwindigkeit = Anteil der aktuellen Bewegung,
        #    der in die neue Richtung zeigt (Projektion). Zeigt sie rückwärts: 0.
        self._s = 0.0
        self._s_dot = max(0.0, dot / length_sq)

    def _update_joints(self) -> None:
        """Überträgt den Bahnzustand (s, ṡ) auf die einzelnen Gelenke."""
        for i in range(self.joint_count):
            delta = self._target[i] - self._start[i]
            self._current[i] = self._start[i] + delta * self._s
            self._velocity[i] = delta * self._s_dot

"""Programmdateien lesen und prüfen – siehe docs/PROGRAM_FORMAT.md.

Wird derzeit nur von den Tests benutzt (Unity spielt Programme ab). Für den Ausblick
"Programm im Controller ausführen" könnte der Controller damit Programme direkt laden.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .robot import JointLimits

FORMAT = "robotwin-program"
VERSION = 1


class ProgramFormatError(ValueError):
    """Datei entspricht nicht dem Programmformat."""


@dataclass(frozen=True)
class ProgramPoint:
    name: str
    joints: tuple[float, ...]
    speed: float = 1.0
    wait: float = 0.0


@dataclass(frozen=True)
class Program:
    name: str
    points: tuple[ProgramPoint, ...]


def load_program(path: Path | str, limits: Sequence[JointLimits]) -> Program:
    return parse_program(Path(path).read_text(encoding="utf-8"), limits)


def parse_program(text: str, limits: Sequence[JointLimits]) -> Program:
    """Liest ein Programm und prüft Form UND Achsgrenzen. Wirft ProgramFormatError mit klarer Meldung."""
    try:
        data = json.loads(text, parse_constant=_reject_constant)
    except ValueError as error:
        raise ProgramFormatError(f"Kein gültiges JSON: {error}") from None

    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ProgramFormatError(f"Feld 'format' muss \"{FORMAT}\" sein.")
    if data.get("version") != VERSION:
        raise ProgramFormatError(f"Formatversion {data.get('version')!r} wird nicht unterstützt (erwartet {VERSION}).")
    if not isinstance(data.get("name"), str):
        raise ProgramFormatError("Feld 'name' fehlt oder ist kein Text.")
    if not isinstance(data.get("points"), list):
        raise ProgramFormatError("Feld 'points' fehlt oder ist keine Liste.")

    points = tuple(_parse_point(number, raw, limits) for number, raw in enumerate(data["points"], start=1))
    return Program(data["name"], points)


def _parse_point(number: int, raw: Any, limits: Sequence[JointLimits]) -> ProgramPoint:
    where = f"Punkt {number}"
    if not isinstance(raw, dict):
        raise ProgramFormatError(f"{where}: muss ein Objekt sein.")

    name = raw.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ProgramFormatError(f"{where}: 'name' fehlt oder ist leer.")

    joints = raw.get("joints")
    if not isinstance(joints, list) or len(joints) != len(limits) or not all(_is_number(v) for v in joints):
        raise ProgramFormatError(f"{where} ({name}): 'joints' braucht {len(limits)} Zahlen.")
    for i, (angle, joint) in enumerate(zip(joints, limits), start=1):
        if not joint.min <= angle <= joint.max:
            raise ProgramFormatError(
                f"{where} ({name}): Gelenk {i} mit {angle}° außerhalb [{joint.min:.0f}°, {joint.max:.0f}°].")

    speed = raw.get("speed", 1.0)
    if not _is_number(speed) or not 0.0 < speed <= 1.0:
        raise ProgramFormatError(f"{where} ({name}): 'speed' muss in (0, 1] liegen.")

    wait = raw.get("wait", 0.0)
    if not _is_number(wait) or not 0.0 <= wait < math.inf:
        raise ProgramFormatError(f"{where} ({name}): 'wait' muss ≥ 0 sein.")

    return ProgramPoint(name, tuple(float(v) for v in joints), float(speed), float(wait))


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _reject_constant(name: str) -> None:
    raise ValueError(f"{name} ist in JSON nicht erlaubt")

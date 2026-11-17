"""Übersetzung zwischen JSON-Zeilen und Nachrichten – siehe docs/PROTOCOL.md.

Prüft die FORM einer Nachricht (gültiges JSON, Pflichtfelder, Typen).
Die BEDEUTUNG (Wertebereiche, Achsgrenzen) prüft robot.py.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Union

from .robot import JointLimits, RobotState

PROTOCOL_VERSION = 1
CONTROLLER_NAME = "RoboTwin Python Controller"
MAX_LINE_BYTES = 64 * 1024


# ---- Nachrichten vom Client (nach dem Parsen) ----

@dataclass(frozen=True)
class MoveJoints:
    id: int
    joints: tuple[float, ...]
    speed: float = 1.0


@dataclass(frozen=True)
class Stop:
    id: int


@dataclass(frozen=True)
class SetOverride:
    id: int
    percent: float


@dataclass(frozen=True)
class EmergencyStop:
    id: int


@dataclass(frozen=True)
class Reset:
    id: int


Command = Union[MoveJoints, Stop, SetOverride, EmergencyStop, Reset]


class ProtocolError(Exception):
    """Nachricht ungültig. command_id ist None, wenn keine gültige Id gelesen werden konnte."""

    def __init__(self, code: str, message: str, command_id: Optional[int] = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.command_id = command_id


# ---- Lesen: JSON-Zeile → Command ----

def parse_command(line: Union[str, bytes]) -> Command:
    """Wandelt eine empfangene Zeile in einen Befehl um oder wirft ProtocolError."""
    if isinstance(line, bytes):
        try:
            line = line.decode("utf-8")
        except UnicodeDecodeError:
            raise ProtocolError("invalid_json", "Zeile ist kein gültiges UTF-8.") from None

    try:
        # parse_constant: Pythons json akzeptiert sonst NaN/Infinity – das ist aber kein gültiges JSON.
        data = json.loads(line, parse_constant=_reject_constant)
    except (json.JSONDecodeError, ValueError) as error:
        raise ProtocolError("invalid_json", f"Ungültiges JSON: {error}") from None

    if not isinstance(data, dict):
        raise ProtocolError("invalid_json", "Nachricht muss ein JSON-Objekt sein.")

    command_id = _read_id(data)   # ab hier kennen wir die Id und geben sie bei Fehlern mit zurück

    message_type = data.get("type")
    if not isinstance(message_type, str):
        raise ProtocolError("invalid_command", "Feld 'type' fehlt oder ist kein Text.", command_id)

    if message_type == "move_joints":
        joints = data.get("joints")
        if not isinstance(joints, list) or not all(_is_number(value) for value in joints):
            raise ProtocolError("invalid_command", "Feld 'joints' muss eine Liste von Zahlen sein.", command_id)
        speed = data.get("speed", 1.0)
        if not _is_number(speed):
            raise ProtocolError("invalid_command", "Feld 'speed' muss eine Zahl sein.", command_id)
        return MoveJoints(command_id, tuple(float(value) for value in joints), float(speed))

    if message_type == "stop":
        return Stop(command_id)

    if message_type == "set_override":
        percent = data.get("percent")
        if not _is_number(percent):
            raise ProtocolError("invalid_command", "Feld 'percent' fehlt oder ist keine Zahl.", command_id)
        return SetOverride(command_id, float(percent))

    if message_type == "emergency_stop":
        return EmergencyStop(command_id)

    if message_type == "reset":
        return Reset(command_id)

    if message_type == "move_linear":
        raise ProtocolError("not_implemented", "move_linear ist in Protokollversion 1 nicht umgesetzt.", command_id)

    raise ProtocolError("unknown_command", f"Unbekannter Befehl '{message_type}'.", command_id)


def _read_id(data: dict) -> int:
    command_id = data.get("id")
    if not _is_integer(command_id) or command_id < 1:
        raise ProtocolError("invalid_command", "Feld 'id' fehlt oder ist keine ganze Zahl ≥ 1.")
    return command_id


def _is_number(value: Any) -> bool:
    # bool ist in Python eine Unterklasse von int (True == 1) – ausdrücklich ausschließen.
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _reject_constant(name: str) -> None:
    raise ValueError(f"{name} ist in JSON nicht erlaubt")


# ---- Schreiben: Nachricht → JSON-Zeile ----

def encode(message: dict) -> bytes:
    """Eine Nachricht als kompakte JSON-Zeile (UTF-8, abgeschlossen mit \\n)."""
    text = json.dumps(
        message,
        separators=(",", ":"),   # kompakt: keine Leerzeichen
        ensure_ascii=False,      # "°" direkt als UTF-8 statt °
        allow_nan=False,         # NaN/Infinity → ValueError statt ungültigem JSON
    )
    return (text + "\n").encode("utf-8")


def hello(role: str, limits: Sequence[JointLimits], state_rate: int) -> dict:
    return {
        "type": "hello",
        "protocol": PROTOCOL_VERSION,
        "controller": CONTROLLER_NAME,
        "role": role,
        "state_rate": state_rate,
        "joints": [
            {
                "name": joint.name,
                "min": joint.min,
                "max": joint.max,
                "max_speed": joint.max_speed,
                "max_acceleration": joint.max_acceleration,
            }
            for joint in limits
        ],
    }


def role_changed(role: str) -> dict:
    return {"type": "role", "role": role}


def ack(command_id: int) -> dict:
    return {"type": "ack", "id": command_id}


def error(command_id: Optional[int], code: str, message: str) -> dict:
    return {"type": "error", "id": command_id, "code": code, "message": message}


def state(seq: int, robot_state: RobotState) -> dict:
    return {
        "type": "state",
        "seq": seq,
        # Gerundet: 0,0001° bzw. 1 ms genügen völlig und halten die Zeilen kurz (50 pro Sekunde!).
        "timestamp": round(robot_state.timestamp, 3),
        "status": robot_state.status.value,
        "override": robot_state.override,
        "joints": [round(angle, 4) for angle in robot_state.joints],
        "error": robot_state.error,
    }

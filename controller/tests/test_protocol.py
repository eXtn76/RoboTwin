"""Tests für robotwin_controller.protocol (JSON-Zeilen ↔ Nachrichten).

Ausführen im Ordner controller/:   python -m unittest
"""

import json
import math
import re
import unittest
from pathlib import Path

from robotwin_controller import protocol
from robotwin_controller.protocol import (
    EmergencyStop,
    MoveJoints,
    ProtocolError,
    Reset,
    SetOverride,
    Stop,
    parse_command,
)
from robotwin_controller.robot import CommandError, JointLimits, Robot, RobotState, RobotStatus

PROTOCOL_MD = Path(__file__).resolve().parents[2] / "docs" / "PROTOCOL.md"


def assert_protocol_error(test, line, code, command_id=None):
    with test.assertRaises(ProtocolError) as caught:
        parse_command(line)
    test.assertEqual(caught.exception.code, code)
    test.assertEqual(caught.exception.command_id, command_id)


class ParseValidCommandsTests(unittest.TestCase):
    def test_move_joints(self):
        command = parse_command('{"type":"move_joints","id":1,"joints":[0,-90,90,-90,-90,0],"speed":0.5}')
        self.assertEqual(command, MoveJoints(1, (0.0, -90.0, 90.0, -90.0, -90.0, 0.0), 0.5))

    def test_move_joints_speed_defaults_to_one(self):
        command = parse_command('{"type":"move_joints","id":7,"joints":[1,2,3,4,5,6]}')
        self.assertEqual(command.speed, 1.0)

    def test_stop(self):
        self.assertEqual(parse_command('{"type":"stop","id":2}'), Stop(2))

    def test_set_override(self):
        self.assertEqual(parse_command('{"type":"set_override","id":3,"percent":50}'), SetOverride(3, 50.0))

    def test_emergency_stop_and_reset(self):
        self.assertEqual(parse_command('{"type":"emergency_stop","id":8}'), EmergencyStop(8))
        self.assertEqual(parse_command('{"type":"reset","id":9}'), Reset(9))

    def test_unknown_fields_are_ignored(self):
        self.assertEqual(parse_command('{"type":"stop","id":4,"comment":"neu in Version 2"}'), Stop(4))

    def test_accepts_bytes_with_newline(self):
        self.assertEqual(parse_command(b'{"type":"stop","id":5}\n'), Stop(5))


class ParseInvalidMessagesTests(unittest.TestCase):
    def test_not_json(self):
        assert_protocol_error(self, "hallo", "invalid_json")

    def test_not_an_object(self):
        assert_protocol_error(self, "[1,2,3]", "invalid_json")

    def test_nan_literal_is_rejected(self):
        assert_protocol_error(self, '{"type":"move_joints","id":1,"joints":[NaN,0,0,0,0,0]}', "invalid_json")

    def test_invalid_utf8(self):
        assert_protocol_error(self, b'{"type":"stop","id":1}\xff', "invalid_json")

    def test_missing_or_invalid_id(self):
        for line in ('{"type":"stop"}', '{"type":"stop","id":0}', '{"type":"stop","id":"5"}',
                     '{"type":"stop","id":true}', '{"type":"stop","id":1.5}'):
            assert_protocol_error(self, line, "invalid_command", None)

    def test_missing_type_keeps_id(self):
        assert_protocol_error(self, '{"id":9}', "invalid_command", 9)

    def test_unknown_type(self):
        assert_protocol_error(self, '{"type":"dance","id":10}', "unknown_command", 10)

    def test_move_linear_is_reserved(self):
        assert_protocol_error(self, '{"type":"move_linear","id":11,"pose":{}}', "not_implemented", 11)

    def test_joints_must_be_list_of_numbers(self):
        for joints in ('"90"', '[0,"90",0,0,0,0]', '[0,true,0,0,0,0]', '{"a":1}'):
            line = '{"type":"move_joints","id":12,"joints":%s}' % joints
            assert_protocol_error(self, line, "invalid_command", 12)

    def test_speed_must_be_number(self):
        assert_protocol_error(self, '{"type":"move_joints","id":13,"joints":[0,0,0,0,0,0],"speed":"fast"}',
                              "invalid_command", 13)

    def test_override_needs_percent(self):
        assert_protocol_error(self, '{"type":"set_override","id":14}', "invalid_command", 14)


class EncodeTests(unittest.TestCase):
    def test_one_compact_line(self):
        data = protocol.encode(protocol.ack(1))
        self.assertEqual(data, b'{"type":"ack","id":1}\n')

    def test_utf8_without_escapes(self):
        data = protocol.encode(protocol.error(1, "joint_limit", "200.0°"))
        self.assertIn("°".encode("utf-8"), data)

    def test_nan_is_never_sent(self):
        with self.assertRaises(ValueError):
            protocol.encode({"type": "state", "joints": [math.nan]})

    def test_error_without_id_sends_null(self):
        self.assertIn(b'"id":null', protocol.encode(protocol.error(None, "invalid_json", "x")))

    def test_hello_contains_version_role_and_limits(self):
        limits = [JointLimits("j1", -360, 360, 180, 180)]
        message = protocol.hello("observer", limits, 50)
        self.assertEqual(message["protocol"], 1)
        self.assertEqual(message["role"], "observer")
        self.assertEqual(message["joints"][0]["name"], "j1")
        self.assertEqual(message["joints"][0]["max_speed"], 180)

    def test_state_is_rounded(self):
        robot_state = RobotState((1.123456789, 0, 0, 0, 0, 0), RobotStatus.MOVING, 50.0, 1.23456789)
        message = protocol.state(42, robot_state)
        self.assertEqual(message["joints"][0], 1.1235)
        self.assertEqual(message["timestamp"], 1.235)
        self.assertEqual(message["status"], "moving")
        self.assertIsNone(message["error"])


class ProtocolDocumentTests(unittest.TestCase):
    """Die Beispiele aus docs/PROTOCOL.md müssen zum Code passen – sonst lügt die Dokumentation."""

    @classmethod
    def setUpClass(cls):
        text = PROTOCOL_MD.read_text(encoding="utf-8")
        example = text.split("## 8. Beispiel einer Sitzung", 1)[1]
        cls.client_lines = re.findall(r"^→ (\{.*\})$", example, flags=re.MULTILINE)
        cls.controller_lines = re.findall(r"^← (\{.*\})$", example, flags=re.MULTILINE)

    def test_document_contains_examples(self):
        self.assertGreaterEqual(len(self.client_lines), 3)

    def test_all_client_examples_parse(self):
        for line in self.client_lines:
            with self.subTest(line=line):
                parse_command(line)

    def test_ack_example_matches_encoder(self):
        self.assertIn(protocol.encode(protocol.ack(1)).decode().strip(), self.controller_lines)

    def test_joint_limit_example_matches_robot_and_encoder(self):
        limits = [JointLimits(f"j{i + 1}", -360, 360, 180, 180) for i in range(6)]
        limits[2] = JointLimits("elbow", -180, 180, 180, 180)   # wie beim UR5
        robot = Robot(limits)
        command = parse_command('{"type":"move_joints","id":2,"joints":[0,0,200,0,0,0]}')
        with self.assertRaises(CommandError) as caught:
            robot.move_joints(command.joints, command.speed)
        line = protocol.encode(protocol.error(command.id, caught.exception.code, caught.exception.message))
        self.assertIn(line.decode("utf-8").strip(), self.controller_lines)

    def test_first_state_example_matches(self):
        robot = Robot([JointLimits(f"j{i}", -360, 360, 180, 180) for i in range(6)])
        robot.step(0.02)
        generated = protocol.state(1, robot.state())
        documented = [json.loads(line) for line in self.controller_lines if '"seq":1,' in line][0]
        self.assertEqual(generated, documented)


if __name__ == "__main__":
    unittest.main()

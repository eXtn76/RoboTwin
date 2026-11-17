"""Tests für robotwin_controller.robot (Grenzen aus der URDF, Befehle prüfen und ausführen).

Ausführen im Ordner controller/:   python -m unittest
"""

import math
import tempfile
import unittest
from pathlib import Path

from robotwin_controller.robot import (
    DEFAULT_ACCELERATION,
    DEFAULT_URDF,
    CommandError,
    JointLimits,
    Robot,
    RobotStatus,
    load_joint_limits,
)

DT = 1 / 50   # Takt des Controllers (50 Hz)

UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def make_robot():
    """UR5-ähnlicher Roboter ohne Dateizugriff: ±360°, Ellbogen ±180°, 180 °/s, 180 °/s²."""
    limits = [JointLimits(name, -360, 360, 180, 180) for name in UR5_JOINTS]
    limits[2] = JointLimits("elbow_joint", -180, 180, 180, 180)
    return Robot(limits)


def run_until_idle(robot, max_seconds=30.0):
    peak = [0.0] * robot.joint_count
    previous = robot.state().joints
    elapsed = 0.0
    while robot.state().status == RobotStatus.MOVING:
        robot.step(DT)
        elapsed += DT
        current = robot.state().joints
        for i in range(robot.joint_count):
            peak[i] = max(peak[i], abs(current[i] - previous[i]) / DT)
        previous = current
        if elapsed > max_seconds:
            raise AssertionError("Bewegung endet nicht")
    return peak


class LoadUrdfTests(unittest.TestCase):
    """Liest die ECHTE ur5.urdf aus dem Unity-Projekt – dieselbe Quelle wie Unity."""

    def setUp(self):
        self.limits = load_joint_limits(DEFAULT_URDF)

    def test_finds_six_joints_in_chain_order(self):
        self.assertEqual([j.name for j in self.limits], UR5_JOINTS)

    def test_elbow_is_limited_to_180_degrees(self):
        elbow = self.limits[2]
        self.assertAlmostEqual(elbow.min, -180)
        self.assertAlmostEqual(elbow.max, 180)

    def test_other_joints_allow_360_degrees(self):
        for joint in self.limits[:2] + self.limits[3:]:
            self.assertAlmostEqual(joint.min, -360, msg=joint.name)
            self.assertAlmostEqual(joint.max, 360, msg=joint.name)

    def test_speed_is_converted_to_degrees(self):
        for joint in self.limits:
            self.assertAlmostEqual(joint.max_speed, 180, msg=joint.name)   # π rad/s

    def test_acceleration_uses_default(self):
        for joint in self.limits:
            self.assertEqual(joint.max_acceleration, DEFAULT_ACCELERATION)


class LoadUrdfStructureTests(unittest.TestCase):
    def _load(self, urdf_text):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "test.urdf"
            path.write_text(urdf_text, encoding="utf-8")
            return load_joint_limits(path)

    def test_order_follows_chain_not_file(self):
        urdf = """<robot name="t">
          <joint name="second" type="revolute"><parent link="a"/><child link="b"/>
            <limit lower="-1" upper="1" velocity="1"/></joint>
          <joint name="first" type="revolute"><parent link="root"/><child link="a"/>
            <limit lower="-1" upper="1" velocity="1"/></joint>
          <joint name="fixed" type="fixed"><parent link="b"/><child link="tool"/></joint>
        </robot>"""
        self.assertEqual([j.name for j in self._load(urdf)], ["first", "second"])

    def test_missing_limit_is_reported(self):
        urdf = """<robot name="t">
          <joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>
        </robot>"""
        with self.assertRaises(ValueError):
            self._load(urdf)


class MoveJointsTests(unittest.TestCase):
    def test_accepted_command_moves_and_finishes_idle(self):
        robot = make_robot()
        robot.move_joints([0, -90, 90, -90, -90, 0])
        self.assertEqual(robot.state().status, RobotStatus.MOVING)
        run_until_idle(robot)
        self.assertEqual(robot.state().status, RobotStatus.IDLE)
        for actual, expected in zip(robot.state().joints, [0, -90, 90, -90, -90, 0]):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_joint_limit_is_rejected_not_clamped(self):
        robot = make_robot()
        with self.assertRaises(CommandError) as caught:
            robot.move_joints([0, 0, 200, 0, 0, 0])
        self.assertEqual(caught.exception.code, "joint_limit")
        self.assertIn("Gelenk 3", caught.exception.message)
        # Abgelehnt heißt: nichts hat sich verändert.
        self.assertEqual(robot.state().status, RobotStatus.IDLE)
        robot.step(DT)
        self.assertEqual(robot.state().joints, (0, 0, 0, 0, 0, 0))

    def test_limit_values_themselves_are_allowed(self):
        robot = make_robot()
        robot.move_joints([360, -360, 180, 0, 0, 0])   # genau auf der Grenze

    def test_wrong_joint_count_is_rejected(self):
        with self.assertRaises(CommandError) as caught:
            make_robot().move_joints([1, 2, 3])
        self.assertEqual(caught.exception.code, "invalid_command")

    def test_nan_and_infinity_are_rejected(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.assertRaises(CommandError) as caught:
                make_robot().move_joints([0, 0, bad, 0, 0, 0])
            self.assertEqual(caught.exception.code, "invalid_command")

    def test_speed_must_be_in_range(self):
        for bad in (0, -0.5, 1.5, math.nan):
            with self.assertRaises(CommandError) as caught:
                make_robot().move_joints([10, 0, 0, 0, 0, 0], speed=bad)
            self.assertEqual(caught.exception.code, "invalid_command", msg=f"speed={bad}")

    def test_speed_limits_joint_velocity(self):
        robot = make_robot()
        robot.move_joints([300, 0, 0, 0, 0, 0], speed=0.5)
        peak = run_until_idle(robot)
        self.assertLessEqual(peak[0], 90 + 1e-6)


class OverrideAndStopTests(unittest.TestCase):
    def test_override_is_reported_in_state(self):
        robot = make_robot()
        robot.set_override(50)
        self.assertEqual(robot.state().override, 50)

    def test_override_must_be_in_range(self):
        for bad in (0, 0.5, 101, math.nan):
            with self.assertRaises(CommandError) as caught:
                make_robot().set_override(bad)
            self.assertEqual(caught.exception.code, "invalid_command", msg=f"percent={bad}")

    def test_override_and_speed_multiply(self):
        robot = make_robot()
        robot.set_override(50)
        robot.move_joints([300, 0, 0, 0, 0, 0], speed=0.5)
        peak = run_until_idle(robot)
        self.assertLessEqual(peak[0], 45 + 1e-6)   # 180 · 0,5 · 0,5

    def test_stop_halts(self):
        robot = make_robot()
        robot.move_joints([300, 0, 0, 0, 0, 0])
        for _ in range(25):
            robot.step(DT)
        robot.stop()
        position = robot.state().joints
        robot.step(DT)
        self.assertEqual(robot.state().status, RobotStatus.IDLE)
        self.assertEqual(robot.state().joints, position)


class EmergencyStopTests(unittest.TestCase):
    def test_emergency_stop_halts_and_reports_error(self):
        robot = make_robot()
        robot.move_joints([300, 0, 0, 0, 0, 0])
        for _ in range(25):
            robot.step(DT)
        robot.emergency_stop("Test")
        position = robot.state().joints
        robot.step(DT)
        state = robot.state()
        self.assertEqual(state.status, RobotStatus.ERROR)
        self.assertEqual(state.error, "Test")
        self.assertEqual(state.joints, position)

    def test_moves_are_rejected_until_reset(self):
        robot = make_robot()
        robot.emergency_stop()
        with self.assertRaises(CommandError) as caught:
            robot.move_joints([10, 0, 0, 0, 0, 0])
        self.assertEqual(caught.exception.code, "emergency_stop_active")

        robot.reset()
        robot.move_joints([10, 0, 0, 0, 0, 0])   # jetzt wieder erlaubt
        self.assertEqual(robot.state().status, RobotStatus.MOVING)
        self.assertIsNone(robot.state().error)

    def test_stop_and_override_stay_allowed(self):
        robot = make_robot()
        robot.emergency_stop()
        robot.stop()
        robot.set_override(50)
        self.assertEqual(robot.state().override, 50)

    def test_first_reason_is_kept(self):
        robot = make_robot()
        robot.emergency_stop("erster")
        robot.emergency_stop("zweiter")
        self.assertEqual(robot.state().error, "erster")

    def test_reset_without_emergency_stop_is_harmless(self):
        robot = make_robot()
        robot.reset()
        self.assertEqual(robot.state().status, RobotStatus.IDLE)


class StateTests(unittest.TestCase):
    def test_timestamp_advances_with_simulation_time(self):
        robot = make_robot()
        for _ in range(50):
            robot.step(DT)
        self.assertAlmostEqual(robot.state().timestamp, 1.0, places=9)

    def test_start_angles_outside_limits_are_rejected(self):
        limits = [JointLimits("j", -10, 10, 1, 1)]
        with self.assertRaises(ValueError):
            Robot(limits, start_angles=[20])

    def test_status_value_fits_protocol(self):
        self.assertEqual(RobotStatus.MOVING.value, "moving")


if __name__ == "__main__":
    unittest.main()

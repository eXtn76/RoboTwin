"""Tests für robotwin_controller.motion (synchrones Trapezprofil).

Ausführen im Ordner controller/:   python -m unittest
"""

import math
import unittest

from robotwin_controller.motion import JointMotionSimulator

DT = 1 / 60          # Simulationsschritt wie bei 60 FPS
SPEED = 180.0        # Grad/s (UR5-Werte aus der URDF)
ACCELERATION = 180.0 # Grad/s²


def make_simulator(joint_count=6, start=None, lower=-360.0, upper=360.0):
    start = start if start is not None else [0.0] * joint_count
    return JointMotionSimulator(
        [lower] * joint_count,
        [upper] * joint_count,
        [SPEED] * joint_count,
        [ACCELERATION] * joint_count,
        start,
    )


def run_until_idle(sim, max_seconds=30.0):
    """Lässt die Simulation laufen und zeichnet pro Schritt die Geschwindigkeiten auf."""
    velocities = []
    steps = 0
    while sim.is_moving:
        sim.step(DT)
        velocities.append(sim.velocity)
        steps += 1
        if steps * DT > max_seconds:
            raise AssertionError("Bewegung endet nicht")
    return steps * DT, velocities


class StartStateTests(unittest.TestCase):
    def test_starts_idle_at_start_angles(self):
        sim = make_simulator(start=[10, 20, 30, 40, 50, 60])
        self.assertFalse(sim.is_moving)
        self.assertEqual(sim.current, (10, 20, 30, 40, 50, 60))
        self.assertEqual(sim.velocity, (0, 0, 0, 0, 0, 0))

    def test_rejects_mismatched_lengths(self):
        with self.assertRaises(ValueError):
            JointMotionSimulator([0] * 6, [1] * 6, [1] * 5, [1] * 6, [0] * 6)


class ReachTargetTests(unittest.TestCase):
    def test_reaches_target_exactly(self):
        sim = make_simulator()
        target = [90, -45, 60, -30, 90, 45]
        sim.set_targets(target)
        run_until_idle(sim)
        for actual, expected in zip(sim.current, target):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_all_joints_arrive_at_the_same_time(self):
        sim = make_simulator()
        target = [90, -10, 60, 0, 5, 45]   # sehr unterschiedliche Wege
        sim.set_targets(target)

        arrival_step = [None] * 6
        step = 0
        while sim.is_moving:
            sim.step(DT)
            step += 1
            for i in range(6):
                if arrival_step[i] is None and abs(sim.current[i] - target[i]) < 1e-9 and target[i] != 0:
                    arrival_step[i] = step

        moving_joints = [s for i, s in enumerate(arrival_step) if target[i] != 0]
        self.assertEqual(len(set(moving_joints)), 1, f"Ankunftsschritte: {arrival_step}")

    def test_joints_stay_on_a_straight_line_in_joint_space(self):
        """Synchron heißt auch: Alle Gelenke haben jederzeit denselben Anteil ihres Weges geschafft."""
        sim = make_simulator()
        target = [90, -30, 60, 0, 0, 0]
        sim.set_targets(target)
        for _ in range(30):
            sim.step(DT)
            fractions = [sim.current[i] / target[i] for i in range(3)]
            self.assertAlmostEqual(fractions[0], fractions[1], places=9)
            self.assertAlmostEqual(fractions[0], fractions[2], places=9)


class ProfileTests(unittest.TestCase):
    def test_long_move_has_trapezoid_profile(self):
        # 300°: beschleunigen (1 s, 90°), konstant 180°/s (120°), bremsen (1 s, 90°) → 2,667 s
        sim = make_simulator()
        sim.set_targets([300, 0, 0, 0, 0, 0])
        duration, velocities = run_until_idle(sim)

        peak = max(abs(v[0]) for v in velocities)
        self.assertAlmostEqual(peak, SPEED, places=6)
        self.assertAlmostEqual(duration, 300 / SPEED + SPEED / ACCELERATION, delta=0.1)

    def test_short_move_has_triangle_profile(self):
        # 90° reicht nicht, um 180°/s zu erreichen: Spitze √(d·a) = 127,3°/s, Dauer 2·√(d/a) = 1,414 s
        sim = make_simulator()
        sim.set_targets([90, 0, 0, 0, 0, 0])
        duration, velocities = run_until_idle(sim)

        peak = max(abs(v[0]) for v in velocities)
        self.assertAlmostEqual(peak, math.sqrt(90 * ACCELERATION), delta=2.0)
        self.assertAlmostEqual(duration, 2 * math.sqrt(90 / ACCELERATION), delta=0.1)

    def test_speed_never_exceeds_limit_times_override(self):
        sim = make_simulator()
        sim.speed_factor = 0.5
        sim.set_targets([300, -200, 100, 50, -300, 10])
        _, velocities = run_until_idle(sim)
        for v in velocities:
            for joint_speed in v:
                self.assertLessEqual(abs(joint_speed), SPEED * 0.5 + 1e-9)

    def test_acceleration_stays_near_limit(self):
        """Beschleunigung ≤ Grenze – mit 30 % Toleranz und ohne den letzten Schritt.

        Durch die diskreten Schritte springt die Geschwindigkeit im letzten Schritt von wenigen °/s
        auf 0, und die Bremskurve wird nur stückweise verfolgt (bekannte Grenze, siehe README).
        """
        sim = make_simulator()
        sim.set_targets([300, 0, 0, 0, 0, 0])
        _, velocities = run_until_idle(sim)
        speeds = [0.0] + [v[0] for v in velocities]
        accelerations = [abs(b - a) / DT for a, b in zip(speeds, speeds[1:])]
        self.assertLessEqual(max(accelerations[:-1]), ACCELERATION * 1.3)


class LimitTests(unittest.TestCase):
    def test_targets_are_clamped_to_joint_limits(self):
        sim = make_simulator(lower=-180, upper=180)
        sim.set_targets([500, -500, 0, 0, 0, 0])
        run_until_idle(sim)
        self.assertEqual(sim.current[0], 180)
        self.assertEqual(sim.current[1], -180)

    def test_set_targets_rejects_wrong_count(self):
        sim = make_simulator()
        with self.assertRaises(ValueError):
            sim.set_targets([1, 2, 3])


class CommandTests(unittest.TestCase):
    def test_stop_halts_immediately(self):
        sim = make_simulator()
        sim.set_targets([300, 0, 0, 0, 0, 0])
        for _ in range(30):
            sim.step(DT)
        position = sim.current

        sim.stop()
        self.assertFalse(sim.is_moving)
        self.assertEqual(sim.velocity, (0, 0, 0, 0, 0, 0))
        sim.step(DT)
        self.assertEqual(sim.current, position)

    def test_override_change_during_motion_still_reaches_target(self):
        sim = make_simulator()
        sim.set_targets([300, 0, 0, 0, 0, 0])
        for _ in range(90):   # 1,5 s: volle Fahrt
            sim.step(DT)
        sim.speed_factor = 0.1
        run_until_idle(sim, max_seconds=60)
        self.assertAlmostEqual(sim.current[0], 300, places=9)

    def test_override_reduction_slows_down_gradually(self):
        sim = make_simulator()
        sim.set_targets([300, 0, 0, 0, 0, 0])
        for _ in range(90):
            sim.step(DT)
        before = abs(sim.velocity[0])
        sim.speed_factor = 0.1
        sim.step(DT)
        after = abs(sim.velocity[0])
        # Nicht schlagartig auf 18 °/s, sondern um höchstens a·dt pro Schritt langsamer.
        self.assertGreater(after, before - ACCELERATION * DT - 1e-6)

    def test_new_target_in_same_direction_keeps_velocity(self):
        """Slider ziehen: Neues Ziel in dieselbe Richtung darf die Fahrt nicht abbremsen."""
        sim = make_simulator()
        sim.set_targets([100, 0, 0, 0, 0, 0])
        for _ in range(20):
            sim.step(DT)
        before = sim.velocity[0]
        sim.set_targets([200, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(sim.velocity[0], before, places=6)
        self.assertTrue(sim.is_moving)

    def test_same_input_gives_same_result(self):
        results = []
        for _ in range(2):
            sim = make_simulator()
            sim.set_targets([90, -45, 60, -30, 90, 45])
            for _ in range(50):
                sim.step(DT)
            results.append(sim.current)
        self.assertEqual(results[0], results[1])


if __name__ == "__main__":
    unittest.main()

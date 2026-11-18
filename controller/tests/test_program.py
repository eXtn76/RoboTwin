"""Tests für das Programmformat (docs/PROGRAM_FORMAT.md) und die Programme im Repo.

Ausführen im Ordner controller/:   python -m unittest
"""

import re
import unittest
from pathlib import Path

from robotwin_controller.program import ProgramFormatError, load_program, parse_program
from robotwin_controller.robot import DEFAULT_URDF, load_joint_limits

REPO_ROOT = Path(__file__).resolve().parents[2]
PROGRAMS = REPO_ROOT / "Programs"
FORMAT_DOC = REPO_ROOT / "docs" / "PROGRAM_FORMAT.md"

LIMITS = load_joint_limits(DEFAULT_URDF)
POINT = '{"name":"P1","joints":[0,-90,90,-90,-90,0]}'


def program_text(points=POINT, header='"format":"robotwin-program","version":1,"name":"Test"'):
    return '{%s,"points":[%s]}' % (header, points)


class RepositoryProgramsTests(unittest.TestCase):
    """Alle Programme im Ordner Programs/ müssen gültig sein und in die UR5-Achsgrenzen passen."""

    def test_programs_folder_contains_files(self):
        self.assertTrue(list(PROGRAMS.glob("*.json")))

    def test_all_programs_are_valid(self):
        for path in PROGRAMS.glob("*.json"):
            with self.subTest(program=path.name):
                program = load_program(path, LIMITS)
                self.assertTrue(program.points)

    def test_demo_has_at_least_five_points(self):
        # Anforderung: Demo-Programm mit mindestens 5 Punkten
        program = load_program(PROGRAMS / "demo_pick_and_place.json", LIMITS)
        self.assertGreaterEqual(len(program.points), 5)

    def test_document_example_is_valid(self):
        text = FORMAT_DOC.read_text(encoding="utf-8")
        example = re.search(r"```json\n(.*?)```", text, flags=re.DOTALL).group(1)
        program = parse_program(example, LIMITS)
        self.assertEqual(program.points[1].wait, 0.5)


class ParseTests(unittest.TestCase):
    def test_defaults_for_speed_and_wait(self):
        point = parse_program(program_text(), LIMITS).points[0]
        self.assertEqual(point.speed, 1.0)
        self.assertEqual(point.wait, 0.0)

    def test_empty_program_is_allowed(self):
        self.assertEqual(parse_program(program_text(points=""), LIMITS).points, ())

    def test_invalid_programs_are_rejected_with_message(self):
        cases = {
            "kein JSON": "hallo",
            "falsches Format": program_text(header='"format":"etwas","version":1,"name":"T"'),
            "falsche Version": program_text(header='"format":"robotwin-program","version":2,"name":"T"'),
            "ohne Name": program_text(header='"format":"robotwin-program","version":1'),
            "Punkt ohne Name": program_text('{"joints":[0,0,0,0,0,0]}'),
            "5 Winkel": program_text('{"name":"P","joints":[0,0,0,0,0]}'),
            "Text als Winkel": program_text('{"name":"P","joints":[0,"0",0,0,0,0]}'),
            "Ellbogen 200°": program_text('{"name":"P","joints":[0,0,200,0,0,0]}'),
            "speed 0": program_text('{"name":"P","joints":[0,0,0,0,0,0],"speed":0}'),
            "negative Wartezeit": program_text('{"name":"P","joints":[0,0,0,0,0,0],"wait":-1}'),
            "NaN": program_text('{"name":"P","joints":[NaN,0,0,0,0,0]}'),
        }
        for label, text in cases.items():
            with self.subTest(label):
                with self.assertRaises(ProgramFormatError):
                    parse_program(text, LIMITS)

    def test_error_message_names_the_point(self):
        with self.assertRaises(ProgramFormatError) as caught:
            parse_program(program_text(POINT + ',{"name":"Greifen","joints":[0,0,200,0,0,0]}'), LIMITS)
        self.assertIn("Punkt 2 (Greifen)", str(caught.exception))
        self.assertIn("Gelenk 3", str(caught.exception))


if __name__ == "__main__":
    unittest.main()

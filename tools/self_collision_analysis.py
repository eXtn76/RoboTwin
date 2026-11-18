"""Selbstkollisions-Analyse für einen Roboter aus einer URDF – mit den echten Kollisions-Meshes.

Beantwortet zwei Fragen per Zufallsstichprobe:
1. Welche Gliederpaare berühren sich NIE?            (wie der "MoveIt Setup Assistant")
2. Gibt es Selbstkollisionen, wenn Gelenke auf ±X° begrenzt werden?
   Damit sind die Sicherheitsgrenzen in RoboTwin begründet (SafetyMonitor: Ellbogen ±145°,
   Handgelenk 2 ±160°). Stellungen, in denen der Arm unter die Montageebene greift (z < 0), werden
   getrennt gezählt: Dort steckt er im Tisch, und das meldet schon die Tisch-/Bodenprüfung.

Benachbarte Glieder werden nicht geprüft – sie stecken im Gelenk ineinander und berühren sich immer.

Benötigt numpy (nur dieses Werkzeug; der Controller selbst braucht nur die Standardbibliothek):
    pip install numpy
    python tools/self_collision_analysis.py --samples 2000 --limit elbow_joint=145 --limit wrist_2_joint=160
"""

from __future__ import annotations

import argparse
import math
import struct
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "RoboTwin-Unity" / "Assets" / "Robots" / "UR5" / "ur5.urdf"
CONTACT = 0.003   # m – kleiner gilt als Berührung


# ---- URDF und Vorwärtskinematik ----

def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(roll), math.sin(roll), math.cos(pitch),
                                              math.sin(pitch), math.cos(yaw), math.sin(yaw)))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_matrix(element) -> np.ndarray:
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, :3] = rpy_matrix(*map(float, element.get("rpy", "0 0 0").split()))
        matrix[:3, 3] = list(map(float, element.get("xyz", "0 0 0").split()))
    return matrix


class Robot:
    def __init__(self, urdf_path: Path) -> None:
        root = ET.parse(urdf_path).getroot()
        self.joint_of_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        self.revolute = [j.get("name") for j in self._chain_to_tip(root) if j.get("type") == "revolute"]
        self.meshes = {}
        for link in root.findall("link"):
            collision = link.find("collision")
            if collision is None or collision.find("geometry/mesh") is None:
                continue
            filename = collision.find("geometry/mesh").get("filename").replace("package://", "")
            vertices = read_stl(urdf_path.parent / filename)
            self.meshes[link.get("name")] = (origin_matrix(collision.find("origin")), vertices)
        # Glieder mit Mesh, entlang der Kette sortiert (Basis zuerst)
        self.links = sorted(self.meshes, key=self._depth)

    def _depth(self, link: str) -> int:
        depth = 0
        while link in self.joint_of_child:
            link = self.joint_of_child[link].find("parent").get("link")
            depth += 1
        return depth

    def _chain_to_tip(self, root) -> list:
        deepest = max((j.find("child").get("link") for j in root.findall("joint")), key=self._depth)
        chain, link = [], deepest
        while link in self.joint_of_child:
            joint = self.joint_of_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        return chain[::-1]

    def link_pose(self, link: str, angles_deg: dict) -> np.ndarray:
        chain, current = [], link
        while current in self.joint_of_child:
            joint = self.joint_of_child[current]
            chain.append(joint)
            current = joint.find("parent").get("link")
        pose = np.eye(4)
        for joint in reversed(chain):
            pose = pose @ origin_matrix(joint.find("origin"))
            if joint.get("type") == "revolute":
                axis = np.array(list(map(float, joint.find("axis").get("xyz").split())))
                pose = pose @ rotation(axis, math.radians(angles_deg[joint.get("name")]))
        return pose

    def vertices(self, link: str, angles_deg: dict) -> np.ndarray:
        offset, local = self.meshes[link]
        pose = self.link_pose(link, angles_deg) @ offset
        return local @ pose[:3, :3].T + pose[:3, 3]


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    matrix = np.eye(4)
    matrix[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
    return matrix


def read_stl(path: Path) -> np.ndarray:
    """Binäres STL lesen; doppelte Eckpunkte entfernen."""
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    records = np.frombuffer(data[84:84 + count * 50],
                            dtype=np.dtype([("normal", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]))
    return np.unique(records["v"].reshape(-1, 3).astype(float), axis=0)


def min_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Kleinster Abstand zweier Punktwolken (Mesh-Eckpunkte), blockweise."""
    best = math.inf
    for start in range(0, len(a), 400):
        block = a[start:start + 400, None, :] - b[None, :, :]
        best = min(best, float(np.sqrt((block ** 2).sum(-1)).min()))
    return best


# ---- Analyse ----

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--limit", action="append", default=[], metavar="GELENK=GRAD",
                        help="Sicherheitsgrenze ±GRAD für ein Gelenk, mehrfach möglich (z. B. elbow_joint=145)")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    robot = Robot(args.urdf)
    links = robot.links
    pairs = [(i, j) for i in range(len(links)) for j in range(i + 2, len(links))]
    rng = np.random.default_rng(args.seed)
    limits = {}
    for entry in args.limit:
        name, degrees = entry.split("=")
        if name not in robot.revolute:
            parser.error(f"unbekanntes Gelenk {name}; vorhanden: {', '.join(robot.revolute)}")
        limits[name] = float(degrees)

    def random_pose(limited: bool) -> dict:
        pose = {name: rng.uniform(-360, 360) for name in robot.revolute}
        if limited:
            for name, degrees in limits.items():
                pose[name] = rng.uniform(-degrees, degrees)
        return pose

    print(f"Roboter: {args.urdf.name}, Glieder mit Mesh: {', '.join(links)}")
    started = time.time()

    # 1. Paare, die sich nie berühren (Gelenke frei bis ±360°)
    contacts = {pair: 0 for pair in pairs}
    closest = {pair: math.inf for pair in pairs}
    for _ in range(args.samples):
        pose = random_pose(False)
        clouds = {link: robot.vertices(link, pose) for link in links}
        for i, j in pairs:
            distance = min_distance(clouds[links[i]], clouds[links[j]])
            closest[(i, j)] = min(closest[(i, j)], distance)
            contacts[(i, j)] += distance < CONTACT
    never = [pair for pair in pairs if contacts[pair] == 0]
    print(f"\n1. Nicht benachbarte Paare in {args.samples} Zufallsstellungen:")
    for i, j in pairs:
        mark = "NIE" if (i, j) in never else f"{contacts[(i, j)]:4}×"
        print(f"   {links[i]:18} ↔ {links[j]:18} {mark}   kleinster Abstand {closest[(i, j)] * 1000:6.1f} mm")

    # 2. Selbstkollisionen mit Sicherheitsgrenzen (alle nicht benachbarten Paare, auch die "nie"-Paare)
    above, below = [], 0
    for _ in range(args.samples):
        pose = random_pose(True)
        clouds = {link: robot.vertices(link, pose) for link in links}
        for i, j in pairs:
            if min_distance(clouds[links[i]], clouds[links[j]]) < CONTACT:
                # Greift ein bewegliches Glied (ab dem Oberarm) unter die Montageebene?
                if min(clouds[link][:, 2].min() for link in links[2:]) < 0.0:
                    below += 1
                else:
                    above.append((links[i], links[j], [round(v) for v in pose.values()]))
                break
    limit_text = ", ".join(f"{name} ±{degrees:.0f}°" for name, degrees in limits.items()) or "keine"
    print(f"\n2. Sicherheitsgrenzen: {limit_text} – {args.samples} Stellungen:")
    print(f"   {below:4} Selbstkollisionen mit Arm unter der Montageebene (meldet die Tischprüfung)")
    print(f"   {len(above):4} Selbstkollisionen über der Montageebene (müssen 0 sein)")
    for first, second, pose in above[:10]:
        print(f"        {first} ↔ {second}   {pose}")
    print(f"\nDauer: {time.time() - started:.0f} s")


if __name__ == "__main__":
    main()

# UR5-Robotermodell

## Quelle

- Repository: <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description>
- Version: 4.4.1 (Commit `0eb1a86`)
- Lizenz: BSD-3-Clause, siehe `ur_description/LICENSE.txt`

`ur5.urdf` ist aus dem Xacro-Makro erzeugt (Xacro = XML-Makrosprache, aus der ROS
die eigentliche URDF generiert). Meshes unverändert übernommen.

## Ordnerstruktur

```
UR5/
├── ur5.urdf
└── ur_description/            ← Name des ROS-Pakets
    └── meshes/ur5/
        ├── visual/*.dae       ← Optik (detailliert)
        └── collision/*.stl    ← Kollision (vereinfacht)
```

Die URDF verweist auf Meshes mit `package://ur_description/meshes/...`. Der Unity
URDF Importer löst `package://<name>/` relativ zum Ordner der URDF-Datei auf,
deshalb liegt der Ordner `ur_description/` direkt neben `ur5.urdf`.

## Gelenke

| Gelenk | Grenzen | Max. Geschw. |
|---|---|---|
| `shoulder_pan_joint`  | ±360° | 180°/s |
| `shoulder_lift_joint` | ±360° | 180°/s |
| `elbow_joint`         | ±180° | 180°/s |
| `wrist_1_joint`       | ±360° | 180°/s |
| `wrist_2_joint`       | ±360° | 180°/s |
| `wrist_3_joint`       | ±360° | 180°/s |

## Neu erzeugen

```bash
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
pip install xacro pyyaml
# In urdf/*.xacro und urdf/inc/*.xacro "$(find ur_description)" durch den
# absoluten Pfad des geklonten Ordners ersetzen (ohne ROS gibt es kein "find").
xacro urdf/ur.urdf.xacro ur_type:=ur5 name:=ur5 -o ur5.urdf
```

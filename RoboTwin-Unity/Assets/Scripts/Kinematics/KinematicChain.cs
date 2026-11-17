using System.Collections.Generic;
using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Kinematics
{
    /// <summary>
    /// Rechenmodell der Gelenkkette – unabhängig von der Szene.
    /// Berechnet die TCP-Lage für BELIEBIGE Winkel, ohne den sichtbaren Roboter zu bewegen.
    /// Alle Frames relativ zur Roboter-Wurzel, in Unity-Achsen und Metern.
    /// </summary>
    public class KinematicChain
    {
        readonly Frame baseOffset;      // Wurzel → Elternteil von Gelenk 1
        readonly Frame[] jointOffsets;  // Elternteil → Gelenk i bei 0° (Position + Grundrotation)
        readonly Frame toolOffset;      // letztes Gelenk → TCP
        readonly Frame robotBase;       // Wurzel → URDF-Frame "base" (Bezug für mm-Koordinaten)

        public int JointCount => jointOffsets.Length;

        KinematicChain(Frame baseOffset, Frame[] jointOffsets, Frame toolOffset, Frame robotBase)
        {
            this.baseOffset = baseOffset;
            this.jointOffsets = jointOffsets;
            this.toolOffset = toolOffset;
            this.robotBase = robotBase;
        }

        /// <summary>Liest die festen Teile der Kette einmal aus der Szene.</summary>
        public static KinematicChain FromScene(Transform root, RobotModel model, Transform tool, Transform robotBaseFrame)
        {
            int count = model.JointCount;
            var offsets = new Frame[count];

            for (int i = 0; i < count; i++)
            {
                JointDriver joint = model.GetJoint(i);

                // Annahme: Gelenk i hängt direkt unter Gelenk i-1. Sonst stimmt die Kette nicht.
                if (i > 0 && joint.transform.parent != model.GetJoint(i - 1).transform)
                    throw new System.InvalidOperationException(
                        $"{joint.name} hängt nicht direkt unter {model.GetJoint(i - 1).name}.");

                // ZeroRotation statt localRotation: unabhängig davon, wie das Gelenk gerade steht.
                offsets[i] = new Frame(joint.transform.localPosition, joint.ZeroRotation);
            }

            Transform firstParent = model.GetJoint(0).transform.parent;
            Transform lastJoint = model.GetJoint(count - 1).transform;

            return new KinematicChain(
                Frame.RelativeTo(root, firstParent),
                offsets,
                Frame.RelativeTo(lastJoint, tool),
                Frame.RelativeTo(root, robotBaseFrame));
        }

        /// <summary>TCP-Lage relativ zur Wurzel für die gegebenen Winkel (Grad, URDF-Konvention).</summary>
        public Frame ForwardKinematics(IReadOnlyList<float> angles) => ForwardKinematics(angles, null, null);

        /// <summary>
        /// Wie oben, liefert zusätzlich Position und Drehachse jedes Gelenks (relativ zur Wurzel).
        /// Die Achse zeigt so, dass ein POSITIVER URDF-Winkel eine positive Drehung um sie ist –
        /// genau das braucht die Jacobi-Matrix.
        /// </summary>
        public Frame ForwardKinematics(IReadOnlyList<float> angles, Vector3[] jointPositions, Vector3[] jointAxes)
        {
            Frame frame = baseOffset;
            for (int i = 0; i < jointOffsets.Length; i++)
            {
                frame = frame * jointOffsets[i];   // jetzt sitzen wir im Gelenk (vor seiner Drehung)

                if (jointPositions != null)
                {
                    jointPositions[i] = frame.Position;
                    // AngleAxis(−θ, up) ist dieselbe Drehung wie AngleAxis(+θ, down).
                    jointAxes[i] = frame.Rotation * Vector3.down;
                }

                // Drehung um die eigene Achse – genau wie im JointDriver
                // (Minus: URDF rechtshändig → Unity linkshändig).
                frame = frame * new Frame(Vector3.zero, Quaternion.AngleAxis(-angles[i], Vector3.up));
            }
            return frame * toolOffset;
        }

        /// <summary>Punkt (relativ zur Wurzel, Unity-Achsen, m) → Roboter-Basis (mm, Z oben, rechtshändig).</summary>
        public Vector3 ToRobotBase(Vector3 rootPoint)
        {
            Vector3 u = robotBase.Inverse().TransformPoint(rootPoint);   // im Frame "base", noch Unity-Achsen
            return new Vector3(u.z, -u.x, u.y) * 1000f;                  // Unity → ROS-Achsen, m → mm
        }

        /// <summary>Roboter-Basis (mm) → Punkt relativ zur Wurzel (Unity-Achsen, m).</summary>
        public Vector3 FromRobotBase(Vector3 robotPointMm)
        {
            Vector3 u = new Vector3(-robotPointMm.y, robotPointMm.z, robotPointMm.x) / 1000f;   // ROS → Unity-Achsen, mm → m
            return robotBase.TransformPoint(u);
        }

        /// <summary>Richtung im Roboter-Basis-System → Richtung relativ zur Wurzel (Unity-Achsen).</summary>
        public Vector3 DirectionFromRobotBase(Vector3 robotDirection) =>
            robotBase.Rotation * new Vector3(-robotDirection.y, robotDirection.z, robotDirection.x);

        /// <summary>
        /// Werkzeug-Orientierung aus zwei Achsen im Roboter-Basis-System.
        /// Der Importer bildet URDF-X auf Unity-forward und URDF-Z auf Unity-up ab –
        /// daher passt LookRotation(forward, up) direkt.
        /// </summary>
        public Quaternion ToolRotationFromRobotBase(Vector3 toolX, Vector3 toolZ) =>
            Quaternion.LookRotation(DirectionFromRobotBase(toolX), DirectionFromRobotBase(toolZ));
    }
}
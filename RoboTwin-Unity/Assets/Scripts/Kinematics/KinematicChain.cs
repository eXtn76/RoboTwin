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
        public Frame ForwardKinematics(IReadOnlyList<float> angles)
        {
            Frame frame = baseOffset;
            for (int i = 0; i < jointOffsets.Length; i++)
            {
                // Erst zum Gelenk (fester Versatz + Grundrotation), dann um die eigene Achse drehen –
                // genau wie im JointDriver (Minus: URDF rechtshändig → Unity linkshändig).
                Frame rotation = new Frame(Vector3.zero, Quaternion.AngleAxis(-angles[i], Vector3.up));
                frame = frame * jointOffsets[i] * rotation;
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
    }
}
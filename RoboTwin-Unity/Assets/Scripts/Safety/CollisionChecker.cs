using System.Collections.Generic;
using RoboTwin.Kinematics;
using UnityEngine;

namespace RoboTwin.Safety
{
    /// <summary>Quader-Hindernis (z. B. Tischplatte), Lage relativ zur Roboter-Wurzel.</summary>
    public readonly struct BoxObstacle
    {
        public readonly string Name;
        public readonly Frame Pose;            // Mittelpunkt + Rotation, relativ zur Roboter-Wurzel
        public readonly Vector3 HalfExtents;   // halbe Kantenlängen in Metern

        public BoxObstacle(string name, Frame pose, Vector3 halfExtents)
        {
            Name = name;
            Pose = pose;
            HalfExtents = halfExtents;
        }
    }

    /// <summary>Ergebnis einer Prüfung.</summary>
    public readonly struct CollisionHit
    {
        public readonly bool Hit;
        public readonly string Segment;       // z. B. "Unterarm"
        public readonly string Obstacle;      // z. B. "Tischplatte"
        public readonly float PathFraction;   // 0 … 1: wo auf der Bahn (bei Pfadprüfung)

        public CollisionHit(string segment, string obstacle, float pathFraction)
        {
            Hit = true;
            Segment = segment;
            Obstacle = obstacle;
            PathFraction = pathFraction;
        }

        public static readonly CollisionHit None = default;   // Hit = false
    }

    /// <summary>
    /// Kollisionsprüfung mit Kapselmodell: Der Arm ist eine Kette von Kapseln (Strecke + Radius) zwischen
    /// den Gelenkpunkten, Hindernisse sind Quader und eine Bodenebene.
    /// Eine Kapsel wird als Reihe von Kugeln geprüft (Abstand ≤ halber Radius): Kugel trifft Quader,
    /// wenn der Abstand Mittelpunkt ↔ nächster Punkt des Quaders kleiner als der Radius ist.
    /// Reine Rechnung auf der KinematicChain – bewegt nichts in der Szene, daher auch für Vorab-Prüfungen.
    /// Alles relativ zur Roboter-Wurzel, Unity-Achsen, Meter.
    /// </summary>
    public class CollisionChecker
    {
        // Punkte: 0–5 = Gelenke J1–J6 (Ursprünge aus der Vorwärtskinematik), 6 = TCP.
        // Der Fuß (Basis bis J2) fehlt absichtlich: Er ist am Tisch festgeschraubt und berührt ihn immer.
        // Die Radien sind etwas größer als der echte Arm – das ist gewollt (Sicherheitsabstand).
        public static readonly (string Name, int From, int To, float Radius)[] Segments =
        {
            ("Oberarm", 1, 2, 0.060f),
            ("Unterarm", 2, 3, 0.050f),
            ("Handgelenk 1", 3, 4, 0.045f),
            ("Handgelenk 2", 4, 5, 0.045f),
            ("Handgelenk 3", 5, 6, 0.045f),
        };

        const float PathStepDegrees = 2f;   // Bahnprüfung: eine Zwischenstellung pro 2° des größten Gelenkwegs
        const int MaxPathSamples = 200;

        readonly KinematicChain chain;
        readonly BoxObstacle[] boxes;
        readonly Vector3 floorPoint;    // ein Punkt der Bodenebene
        readonly Vector3 floorNormal;   // zeigt nach oben
        readonly float margin;          // zusätzlicher Sicherheitsabstand auf jeden Radius

        // Arbeitsspeicher, wiederverwendet
        readonly Vector3[] jointPositions;
        readonly Vector3[] jointAxes;
        readonly Vector3[] points;
        readonly float[] sample;

        public CollisionChecker(KinematicChain chain, BoxObstacle[] boxes, Vector3 floorPoint, Vector3 floorNormal, float margin)
        {
            this.chain = chain;
            this.boxes = boxes;
            this.floorPoint = floorPoint;
            this.floorNormal = floorNormal.normalized;
            this.margin = margin;

            jointPositions = new Vector3[chain.JointCount];
            jointAxes = new Vector3[chain.JointCount];
            points = new Vector3[chain.JointCount + 1];
            sample = new float[chain.JointCount];
        }

        public IReadOnlyList<BoxObstacle> Boxes => boxes;
        public float Margin => margin;

        /// <summary>Gelenkpunkte + TCP für diese Winkel (relativ zur Wurzel) – z. B. zum Zeichnen.</summary>
        public Vector3[] ComputePoints(IReadOnlyList<float> angles)
        {
            Frame tcp = chain.ForwardKinematics(angles, jointPositions, jointAxes);
            for (int i = 0; i < jointPositions.Length; i++)
                points[i] = jointPositions[i];
            points[jointPositions.Length] = tcp.Position;
            return points;
        }

        /// <summary>Kollidiert der Arm in DIESER Stellung?</summary>
        public CollisionHit CheckPose(IReadOnlyList<float> angles)
        {
            ComputePoints(angles);

            foreach (var segment in Segments)
            {
                Vector3 start = points[segment.From];
                Vector3 end = points[segment.To];
                float radius = segment.Radius + margin;

                // Kapsel als Kugelreihe: Abstand zwischen zwei Kugeln höchstens der halbe Radius.
                int count = Mathf.Max(2, Mathf.CeilToInt(Vector3.Distance(start, end) / (radius * 0.5f)) + 1);
                for (int k = 0; k < count; k++)
                {
                    Vector3 center = Vector3.Lerp(start, end, k / (float)(count - 1));

                    if (Vector3.Dot(center - floorPoint, floorNormal) < radius)
                        return new CollisionHit(segment.Name, "Boden", 0f);

                    foreach (BoxObstacle box in boxes)
                        if (DistanceToBox(center, box) < radius)
                            return new CollisionHit(segment.Name, box.Name, 0f);
                }
            }
            return CollisionHit.None;
        }

        /// <summary>
        /// Prüft die Bahn von 'from' nach 'to'. Unsere PTP-Bewegung ist synchron, also eine GERADE Linie im
        /// Gelenkraum – die Zwischenstellungen sind deshalb schon vorher bekannt.
        /// Steht der Roboter schon in Kollision, zählen erst Treffer NACH dem ersten freien Punkt:
        /// So kann er aus einer Kollision herausfahren, statt dauerhaft blockiert zu sein.
        /// </summary>
        public CollisionHit CheckPath(IReadOnlyList<float> from, IReadOnlyList<float> to)
        {
            float largestStep = 0f;
            for (int i = 0; i < sample.Length; i++)
                largestStep = Mathf.Max(largestStep, Mathf.Abs(to[i] - from[i]));
            int count = Mathf.Clamp(Mathf.CeilToInt(largestStep / PathStepDegrees) + 1, 2, MaxPathSamples);

            bool freeSeen = false;
            for (int k = 0; k < count; k++)
            {
                float t = k / (float)(count - 1);
                for (int i = 0; i < sample.Length; i++)
                    sample[i] = Mathf.Lerp(from[i], to[i], t);

                CollisionHit hit = CheckPose(sample);
                if (!hit.Hit)
                    freeSeen = true;
                else if (freeSeen)
                    return new CollisionHit(hit.Segment, hit.Obstacle, t);
            }
            return CollisionHit.None;
        }

        // Abstand eines Punkts zum Quader: in das Quader-System umrechnen, auf den Quader "klemmen"
        // (nächster Punkt des Quaders), Abstand messen. Liegt der Punkt innen, ist der Abstand 0.
        static float DistanceToBox(Vector3 point, BoxObstacle box)
        {
            Vector3 local = box.Pose.Inverse().TransformPoint(point);
            Vector3 h = box.HalfExtents;
            var closest = new Vector3(
                Mathf.Clamp(local.x, -h.x, h.x),
                Mathf.Clamp(local.y, -h.y, h.y),
                Mathf.Clamp(local.z, -h.z, h.z));
            return Vector3.Distance(local, closest);
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using RoboTwin.Connection;
using RoboTwin.Kinematics;
using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Safety
{
    /// <summary>
    /// Kollisionsschutz in der Szene:
    /// 1. Vorab-Prüfung – meldet sich beim Host als IMotionGuard an. Jeder Fahrbefehl über host.SendMove()
    ///    wird geprüft (Sicherheitsgrenzen der Achsen, dann Tisch und Boden); ist er nicht sicher, wird
    ///    nicht gesendet.
    /// 2. Laufende Überwachung – prüft jeden Ist-Zustand. Gerät der Arm trotzdem in Kollision (z. B. weil der
    ///    Python-Client steuert, der den Tisch nicht kennt), löst Unity den Not-Halt aus.
    /// 3. Gizmos – zeichnet Kapseln und Hindernisse in die Scene-Ansicht (grün frei, rot Kollision).
    /// </summary>
    public class SafetyMonitor : MonoBehaviour, IMotionGuard
    {
        /// <summary>Sicherheitsgrenze: Gelenk darf nur im Bereich ±Limit fahren (enger als mechanisch möglich).</summary>
        [Serializable]
        public class JointSafetyLimit
        {
            [Tooltip("Gelenknummer, 1 = Basis … 6 = Handgelenk 3")]
            public int joint;
            [Tooltip("Erlaubter Bereich in Grad (±)")]
            public float limit;
        }
        [SerializeField] RobotModel model;
        [SerializeField] RobotKinematics kinematics;
        [SerializeField] RobotConnectionHost host;

        [Header("Hindernisse")]
        [SerializeField] BoxCollider[] obstacles;      // Quader, z. B. Tischplatte und Tischbeine
        [SerializeField] Transform floor;              // Bodenebene (Position + Oberseite)
        [SerializeField] float safetyMargin = 0.01f;   // m, zusätzlich zu den Kapselradien

        // Selbstkollision wird über Achsgrenzen verhindert – wie mit den "Joint Position Limits" in den
        // Sicherheitseinstellungen echter UR-Steuerungen. Begründung (Stichproben mit den echten Kollisions-Meshes):
        //   python tools/self_collision_analysis.py --limit elbow_joint=145 --limit wrist_2_joint=160
        // Ellbogen: erste Selbstkollision bei ±150°. Handgelenk 2: Werkzeug klappt bei ~165–195° auf den Unterarm.
        [Header("Sicherheitsgrenzen gegen Selbstkollision")]
        [SerializeField] JointSafetyLimit[] jointLimits =
        {
            new JointSafetyLimit { joint = 3, limit = 145f },   // Ellbogen
            new JointSafetyLimit { joint = 5, limit = 160f },   // Handgelenk 2
        };

        [Header("Überwachung")]
        [SerializeField] bool monitorActualState = true;
        [SerializeField] bool drawGizmos = true;

        CollisionChecker checker;
        CollisionHit currentHit;
        bool wasColliding;

        // Lazy: Die Kette baut RobotKinematics erst in seinem Awake.
        CollisionChecker Checker => checker ??= CreateChecker();

        void Reset()
        {
            model = GetComponent<RobotModel>();
            kinematics = GetComponent<RobotKinematics>();
            host = GetComponent<RobotConnectionHost>();

            // Vorschlag: alle Quader-Kollider der Szene, die nicht zum Roboter selbst gehören (Tisch, Beine …).
            obstacles = FindObjectsByType<BoxCollider>(FindObjectsSortMode.None)
                .Where(c => !c.transform.IsChildOf(transform))
                .ToArray();
            GameObject floorObject = GameObject.Find("Floor");
            if (floorObject != null)
                floor = floorObject.transform;
        }

        // Awake: beim Host anmelden, bevor irgendein Panel (in Start) etwas senden kann.
        void Awake() => host.MotionGuard = this;

        void Start() => model.StateChanged += OnStateChanged;

        void OnDestroy()
        {
            if (model != null)
                model.StateChanged -= OnStateChanged;
            if (host != null && ReferenceEquals(host.MotionGuard, this))
                host.MotionGuard = null;
        }

        // ---- 1. Vorab-Prüfung (IMotionGuard) ----

        public bool CheckMove(IReadOnlyList<float> from, IReadOnlyList<float> to, out string reason)
        {
            // Zuerst die Sicherheitsgrenzen. Die Bahn ist eine gerade Linie im Gelenkraum – liegt das ZIEL im
            // erlaubten Bereich (und der Start auch), dann auch jeder Punkt dazwischen. Liegt nur der Start
            // außerhalb, ist die Fahrt erlaubt: Sie führt zurück in den sicheren Bereich.
            if (ViolatedLimit(to, 0f, out reason))
            {
                reason += " – Selbstkollision droht, nicht gefahren.";
                return false;
            }

            CollisionHit hit = Checker.CheckPath(from, to);
            reason = hit.Hit
                ? $"Kollision bei {hit.PathFraction:P0} der Bahn: {hit.Segment} ↔ {hit.Obstacle} – nicht gefahren."
                : null;
            return !hit.Hit;
        }

        public void RestrictLimits(float[] minAngles, float[] maxAngles)
        {
            foreach (JointSafetyLimit safety in jointLimits)
            {
                int i = safety.joint - 1;
                if (i < 0 || i >= minAngles.Length)
                    continue;
                minAngles[i] = Mathf.Max(minAngles[i], -safety.limit);
                maxAngles[i] = Mathf.Min(maxAngles[i], safety.limit);
            }
        }

        // Liegt ein Winkel außerhalb einer Sicherheitsgrenze (plus Toleranz)?
        bool ViolatedLimit(IReadOnlyList<float> angles, float tolerance, out string reason)
        {
            foreach (JointSafetyLimit safety in jointLimits)
            {
                int i = safety.joint - 1;
                if (i < 0 || i >= angles.Count)
                    continue;
                if (Mathf.Abs(angles[i]) > safety.limit + tolerance)
                {
                    string name = model.GetJoint(i).JointName.Replace("_joint", "");
                    reason = $"Sicherheitsgrenze: Gelenk {safety.joint} ({name}) bei {angles[i]:F1}°, erlaubt ±{safety.limit:F0}°";
                    return true;
                }
            }
            reason = null;
            return false;
        }

        // ---- 2. Laufende Überwachung ----

        void OnStateChanged()
        {
            if (!monitorActualState)
                return;

            currentHit = Checker.CheckPose(model.ActualAngles);
            // 0,5° Toleranz: Rundung der Zustandsmeldungen soll keinen Not-Halt auslösen.
            bool limitViolated = ViolatedLimit(model.ActualAngles, 0.5f, out string limitReason);
            bool unsafeNow = currentHit.Hit || limitViolated;

            // Nur beim ÜBERGANG sicher → unsicher auslösen. Steht der Arm nach dem Quittieren noch so,
            // würde er sonst sofort wieder gesperrt – und könnte nie herausfahren.
            if (unsafeNow && !wasColliding && host.IsConnected && model.Status != RobotStatus.Error)
            {
                host.Connection.SendCommand(new EmergencyStopCommand());   // darf auch ein Beobachter
                string cause = currentHit.Hit ? $"Kollision erkannt: {currentHit.Segment} ↔ {currentHit.Obstacle}" : limitReason;
                host.ReportWarning($"{cause} – Not-Halt ausgelöst.");
            }
            wasColliding = unsafeNow;
        }

        // ---- Aufbau ----

        // Hindernisse aus der Szene einmal in das System der Roboter-Wurzel umrechnen.
        CollisionChecker CreateChecker()
        {
            Transform root = kinematics.transform;
            Quaternion toRoot = Quaternion.Inverse(root.rotation);

            var boxes = new List<BoxObstacle>();
            foreach (BoxCollider box in obstacles)
            {
                if (box == null)
                    continue;
                Transform t = box.transform;
                Vector3 centerWorld = t.TransformPoint(box.center);
                // Die Skalierung steckt in der Größe – der Frame selbst hat nur Position und Rotation.
                Vector3 size = Vector3.Scale(box.size, t.lossyScale);
                var halfExtents = new Vector3(Mathf.Abs(size.x), Mathf.Abs(size.y), Mathf.Abs(size.z)) * 0.5f;
                var pose = new Frame(root.InverseTransformPoint(centerWorld), toRoot * t.rotation);
                boxes.Add(new BoxObstacle(box.name, pose, halfExtents));
            }

            // Ohne Boden-Objekt: Ebene 10 m tiefer (praktisch keine Prüfung).
            Vector3 floorPoint = floor != null ? floor.position : root.position + Vector3.down * 10f;
            Vector3 floorNormal = floor != null ? floor.up : Vector3.up;

            return new CollisionChecker(kinematics.Chain, boxes.ToArray(),
                root.InverseTransformPoint(floorPoint), toRoot * floorNormal, safetyMargin);
        }

        // ---- 3. Gizmos (Scene-Ansicht; im Game-Fenster über den Button "Gizmos") ----

        void OnDrawGizmos()
        {
            if (!drawGizmos || !Application.isPlaying || checker == null)
                return;

            Matrix4x4 rootMatrix = kinematics.transform.localToWorldMatrix;

            Gizmos.color = new Color(1f, 0.6f, 0f);
            foreach (BoxObstacle box in checker.Boxes)
            {
                Gizmos.matrix = rootMatrix * Matrix4x4.TRS(box.Pose.Position, box.Pose.Rotation, Vector3.one);
                Gizmos.DrawWireCube(Vector3.zero, box.HalfExtents * 2f);
            }

            Gizmos.matrix = rootMatrix;
            Vector3[] points = checker.ComputePoints(model.ActualAngles);
            foreach (var segment in CollisionChecker.Segments)
            {
                bool hit = currentHit.Hit && currentHit.Segment == segment.Name;
                Gizmos.color = hit ? Color.red : Color.green;

                Vector3 start = points[segment.From];
                Vector3 end = points[segment.To];
                float radius = segment.Radius + checker.Margin;
                Gizmos.DrawWireSphere(start, radius);
                Gizmos.DrawWireSphere(end, radius);
                Gizmos.DrawLine(start, end);
            }
            Gizmos.matrix = Matrix4x4.identity;
        }
    }
}

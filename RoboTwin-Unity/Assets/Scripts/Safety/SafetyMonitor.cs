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
    ///    wird geprüft; ist die Bahn nicht frei, wird nicht gesendet.
    /// 2. Laufende Überwachung – prüft jeden Ist-Zustand. Gerät der Arm trotzdem in Kollision (z. B. weil der
    ///    Python-Client steuert, der den Tisch nicht kennt), löst Unity den Not-Halt aus.
    /// 3. Gizmos – zeichnet Kapseln und Hindernisse in die Scene-Ansicht (grün frei, rot Kollision).
    /// </summary>
    public class SafetyMonitor : MonoBehaviour, IMotionGuard
    {
        [SerializeField] RobotModel model;
        [SerializeField] RobotKinematics kinematics;
        [SerializeField] RobotConnectionHost host;

        [Header("Hindernisse")]
        [SerializeField] BoxCollider[] obstacles;      // Quader, z. B. Tischplatte und Tischbeine
        [SerializeField] Transform floor;              // Bodenebene (Position + Oberseite)
        [SerializeField] float safetyMargin = 0.01f;   // m, zusätzlich zu den Kapselradien

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
            CollisionHit hit = Checker.CheckPath(from, to);
            reason = hit.Hit
                ? $"Kollision bei {hit.PathFraction:P0} der Bahn: {hit.Segment} ↔ {hit.Obstacle} – nicht gefahren."
                : null;
            return !hit.Hit;
        }

        // ---- 2. Laufende Überwachung ----

        void OnStateChanged()
        {
            if (!monitorActualState)
                return;

            currentHit = Checker.CheckPose(model.ActualAngles);
            bool colliding = currentHit.Hit;

            // Nur beim ÜBERGANG frei → Kollision auslösen. Steht der Arm nach dem Quittieren noch in Kollision,
            // würde er sonst sofort wieder gesperrt – und könnte nie herausfahren.
            if (colliding && !wasColliding && host.IsConnected && model.Status != RobotStatus.Error)
            {
                host.Connection.SendCommand(new EmergencyStopCommand());   // darf auch ein Beobachter
                host.ReportWarning($"Kollision erkannt: {currentHit.Segment} ↔ {currentHit.Obstacle} – Not-Halt ausgelöst.");
            }
            wasColliding = colliding;
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

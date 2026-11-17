using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Kinematics
{
    public enum ToolOrientation
    {
        Down,   // Werkzeug zeigt senkrecht nach unten
        Keep    // aktuelle Werkzeug-Orientierung beibehalten
    }

    /// <summary>
    /// Kartesische Steuerung: Zielposition (Roboter-Basis, mm) → IK → Fahrbefehl an die Simulation.
    /// Ist das Ziel unerreichbar, bewegt sich der Roboter NICHT.
    /// </summary>
    public class CartesianController : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] RobotKinematics kinematics;
        [SerializeField] LocalRobotSimulation simulation;

        [Header("Test ohne UI")]
        [SerializeField] Vector3 testTargetMm = new Vector3(400f, -200f, 300f);
        [SerializeField] ToolOrientation testOrientation = ToolOrientation.Down;

        IkSolver solver;

        // Lazy Initialization: Der Solver wird beim ERSTEN Gebrauch gebaut, nicht in Start.
        // Grund: Das Kartesisch-Panel ruft Plan() schon in SEINEM Start auf – und Unity garantiert
        // keine Reihenfolge zwischen den Start-Methoden verschiedener Objekte. Beim ersten Gebrauch
        // sind aber alle Awakes gelaufen, also existiert die Kette (RobotKinematics.Awake).
        IkSolver Solver => solver ??= CreateSolver();

        void Reset()
        {
            model = GetComponent<RobotModel>();
            kinematics = GetComponent<RobotKinematics>();
            simulation = GetComponent<LocalRobotSimulation>();
        }

        IkSolver CreateSolver()
        {
            int n = model.JointCount;
            var minAngles = new float[n];
            var maxAngles = new float[n];
            for (int i = 0; i < n; i++)
            {
                minAngles[i] = model.GetJoint(i).MinAngle;
                maxAngles[i] = model.GetJoint(i).MaxAngle;
            }
            return new IkSolver(kinematics.Chain, minAngles, maxAngles);
        }

        /// <summary>Ziel-Lage relativ zur Wurzel aus Position (mm, Basis) und Orientierungswahl.</summary>
        public Frame TargetFrame(Vector3 targetMm, ToolOrientation orientation)
        {
            KinematicChain chain = kinematics.Chain;
            Quaternion rotation = orientation == ToolOrientation.Down
                // ACHTUNG: Basis-Koordinaten, nicht Unity! "Unten" ist hier −Z, also (0, 0, −1).
                ? chain.ToolRotationFromRobotBase(new Vector3(1f, 0f, 0f), new Vector3(0f, 0f, -1f))
                : chain.ForwardKinematics(model.ActualAngles).Rotation;

            return new Frame(chain.FromRobotBase(targetMm), rotation);
        }

        /// <summary>
        /// Berechnet nur die IK (Start: aktuelle Ist-Winkel), ohne zu fahren – z. B. für die Vorschau.
        /// </summary>
        public IkResult Plan(Vector3 targetMm, ToolOrientation orientation) =>
            Solver.Solve(model.ActualAngles, TargetFrame(targetMm, orientation));

        /// <summary>
        /// Berechnet die IK (Start: aktuelle Ist-Winkel) und fährt bei Erfolg los.
        /// Gibt false zurück, wenn das Ziel unerreichbar ist – dann bewegt sich nichts.
        /// </summary>
        public bool MoveTo(Vector3 targetMm, ToolOrientation orientation, out IkResult result)
        {
            result = Plan(targetMm, orientation);
            if (result.Success)
                simulation.SetTargets(result.Angles);
            return result.Success;
        }

        [ContextMenu("Move to Test Target")]
        void MoveToTestTarget()
        {
            if (MoveTo(testTargetMm, testOrientation, out IkResult r))
                Debug.Log($"IK erfolgreich nach {r.Iterations} Iterationen, Restfehler {r.PositionErrorMm:F2} mm.");
            else
                Debug.LogWarning($"Ziel unerreichbar: Restfehler {r.PositionErrorMm:F1} mm / {r.OrientationErrorDeg:F1}°.");
        }
    }
}
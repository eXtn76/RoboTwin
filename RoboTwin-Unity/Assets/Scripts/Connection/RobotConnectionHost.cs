using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Connection
{
    /// <summary>
    /// Hält die aktive Roboterverbindung, "tickt" sie jedes Frame im Haupt-Thread und gibt
    /// empfangene Ist-Zustände an das RobotModel weiter. Die UI holt sich die Verbindung hier –
    /// und kennt dabei nur das Interface IRobotConnection.
    /// </summary>
    public class RobotConnectionHost : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] float acceleration = 180f;   // Grad/s², für die Simulation
        [SerializeField] bool connectOnStart = true;

        public IRobotConnection Connection { get; private set; }

        void Reset() => model = GetComponent<RobotModel>();

        // Awake statt Start: Die Verbindung muss existieren, bevor irgendein Start() der UI sie abfragt.
        // Gebraucht werden nur die (serialisierten) Daten der JointDriver – nicht RobotModel.Awake.
        void Awake()
        {
            Connection = CreateSimulatedConnection();
            Connection.StateReceived += model.ApplyState;
        }

        void Start()
        {
            if (connectOnStart)
                Connection.Connect();
        }

        // Der einzige Ort, an dem die Verbindung "arbeitet" – immer im Haupt-Thread.
        void Update() => Connection.Tick(Time.deltaTime);

        void OnDestroy()
        {
            if (Connection == null)
                return;
            Connection.StateReceived -= model.ApplyState;
            Connection.Disconnect();
        }

        // Test für "ablehnen statt begrenzen": Über Slider und IK kommt man nie über die Grenzen,
        // also schicken wir hier absichtlich einen ungültigen Befehl (Ellbogen auf 200°).
        [ContextMenu("Test: Befehl außerhalb der Grenzen senden")]
        void SendOutOfLimitTestCommand()
        {
            var joints = new float[model.JointCount];
            joints[2] = 200f;
            int id = Connection.SendCommand(new MoveJointsCommand(joints));
            Debug.Log($"Testbefehl #{id} gesendet (Ellbogen 200°) – erwartet: Ablehnung im Panel.");
        }

        SimulatedConnection CreateSimulatedConnection()
        {
            int count = model.JointCount;
            var minAngles = new float[count];
            var maxAngles = new float[count];
            var maxSpeeds = new float[count];
            var maxAccelerations = new float[count];
            var startAngles = new float[count];

            for (int i = 0; i < count; i++)
            {
                JointDriver joint = model.GetJoint(i);
                minAngles[i] = joint.MinAngle;
                maxAngles[i] = joint.MaxAngle;
                maxSpeeds[i] = joint.MaxSpeed;
                maxAccelerations[i] = acceleration;
                startAngles[i] = joint.Angle;
            }

            return new SimulatedConnection(minAngles, maxAngles, maxSpeeds, maxAccelerations, startAngles);
        }
    }
}

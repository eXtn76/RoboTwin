using System;
using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Connection
{
    public enum ConnectionMode
    {
        Simulation,   // Controller läuft in Unity (SimulatedConnection)
        Network       // Python-Controller über TCP (TcpConnection)
    }

    /// <summary>
    /// Hält die aktive Roboterverbindung, "tickt" sie jedes Frame im Haupt-Thread und gibt
    /// empfangene Ist-Zustände an das RobotModel weiter. Schaltet zwischen Simulation und Netzwerk um.
    ///
    /// Die UI abonniert die Events des HOSTS, nicht die der Verbindung: Der Host leitet sie von der
    /// jeweils aktiven Verbindung weiter. So muss sich die UI beim Umschalten nicht neu anmelden.
    /// </summary>
    public class RobotConnectionHost : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] float acceleration = 180f;   // Grad/s², für die Simulation

        [Header("Verbindung")]
        [SerializeField] ConnectionMode startMode = ConnectionMode.Simulation;
        [SerializeField] string networkHost = "127.0.0.1";
        [SerializeField] int networkPort = 5005;
        [SerializeField] bool connectOnStart = true;

        public IRobotConnection Connection { get; private set; }
        public ConnectionMode Mode { get; private set; }
        public string NetworkHost => networkHost;
        public int NetworkPort => networkPort;

        public bool IsConnected => Connection.Status == ConnectionStatus.Connected;

        /// <summary>Darf Unity gerade steuern? Verbunden UND Besitzer.</summary>
        public bool CanControl => IsConnected && Connection.Role == ControlRole.Owner;

        /// <summary>Darf Unity gerade FAHREN? Steuern dürfen UND kein Not-Halt aktiv.</summary>
        public bool CanMove => CanControl && model.Status != RobotStatus.Error;

        RobotStatus lastRobotStatus;

        // ---- Weitergeleitete Events der aktiven Verbindung ----
        public event Action<ConnectionStatus> StatusChanged;
        public event Action<ControlRole> RoleChanged;
        public event Action<CommandResult> CommandResultReceived;
        public event Action<string> Warning;

        /// <summary>Verbindung wurde ausgetauscht (Simulation ↔ Netzwerk, neue Adresse).</summary>
        public event Action ConnectionSwitched;

        /// <summary>Etwas hat sich geändert, das CanControl/CanMove/IsConnected beeinflusst
        /// (Verbindungsstatus, Rolle, Umschalten, Not-Halt).</summary>
        public event Action ControlChanged;

        void Reset() => model = GetComponent<RobotModel>();

        // Awake statt Start: Die Verbindung muss existieren, bevor irgendein Start() der UI sie abfragt.
        void Awake()
        {
            Mode = startMode;
            Attach(CreateConnection(Mode));
            model.StateChanged += OnModelStateChanged;
        }

        // Not-Halt gesetzt oder quittiert → CanMove hat sich geändert.
        void OnModelStateChanged()
        {
            bool wasError = lastRobotStatus == RobotStatus.Error;
            bool isError = model.Status == RobotStatus.Error;
            lastRobotStatus = model.Status;
            if (wasError != isError)
                ControlChanged?.Invoke();
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
            if (model != null)
                model.StateChanged -= OnModelStateChanged;
            if (Connection != null)
                Detach();   // trennt auch die TCP-Verbindung → der Netzwerk-Thread endet
        }

        /// <summary>Alte Verbindung trennen, neue erzeugen und verbinden.</summary>
        public void SwitchTo(ConnectionMode mode, string host, int port)
        {
            Mode = mode;
            networkHost = host;
            networkPort = port;

            Detach();
            Attach(CreateConnection(mode));
            Connection.Connect();

            ConnectionSwitched?.Invoke();
            ControlChanged?.Invoke();
        }

        void Attach(IRobotConnection connection)
        {
            Connection = connection;
            connection.StateReceived += model.ApplyState;
            connection.StatusChanged += OnStatusChanged;
            connection.RoleChanged += OnRoleChanged;
            connection.CommandResultReceived += OnCommandResult;
            connection.Warning += OnWarning;
        }

        void Detach()
        {
            Connection.StateReceived -= model.ApplyState;
            Connection.StatusChanged -= OnStatusChanged;
            Connection.RoleChanged -= OnRoleChanged;
            Connection.CommandResultReceived -= OnCommandResult;
            Connection.Warning -= OnWarning;
            Connection.Disconnect();
        }

        void OnStatusChanged(ConnectionStatus status)
        {
            StatusChanged?.Invoke(status);
            ControlChanged?.Invoke();
        }

        void OnRoleChanged(ControlRole role)
        {
            RoleChanged?.Invoke(role);
            ControlChanged?.Invoke();
        }

        void OnCommandResult(CommandResult result) => CommandResultReceived?.Invoke(result);
        void OnWarning(string message) => Warning?.Invoke(message);

        // ---- Verbindungen erzeugen ----

        IRobotConnection CreateConnection(ConnectionMode mode) =>
            mode == ConnectionMode.Network ? CreateTcpConnection() : CreateSimulatedConnection();

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
                startAngles[i] = joint.Angle;   // dort weitermachen, wo der Arm gerade steht
            }

            return new SimulatedConnection(minAngles, maxAngles, maxSpeeds, maxAccelerations, startAngles);
        }

        TcpConnection CreateTcpConnection()
        {
            int count = model.JointCount;
            var names = new string[count];
            var minAngles = new float[count];
            var maxAngles = new float[count];

            for (int i = 0; i < count; i++)
            {
                JointDriver joint = model.GetJoint(i);
                names[i] = joint.JointName;
                minAngles[i] = joint.MinAngle;
                maxAngles[i] = joint.MaxAngle;
            }

            return new TcpConnection(networkHost, networkPort, names, minAngles, maxAngles);
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
    }
}

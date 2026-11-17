using System;
using System.Collections.Generic;
using RoboTwin.Model;

namespace RoboTwin.Connection
{
    /// <summary>
    /// Roboter-Controller, der komplett in Unity läuft: der JointMotionSimulator hinter dem Interface.
    /// Verhält sich bewusst wie eine Netzwerkverbindung: Befehle werden geprüft (und ggf. abgelehnt),
    /// Antworten und Zustände kommen erst im nächsten Tick() – nie direkt aus SendCommand().
    /// </summary>
    public class SimulatedConnection : IRobotConnection
    {
        readonly JointMotionSimulator simulator;
        readonly float[] minAngles;
        readonly float[] maxAngles;

        // Antworten warten hier, bis Tick() sie ausliefert.
        readonly Queue<CommandResult> pendingResults = new Queue<CommandResult>();

        ConnectionStatus status = ConnectionStatus.Disconnected;
        bool statusChangePending;   // StatusChanged wird erst im nächsten Tick() ausgelöst
        int nextCommandId = 1;
        float commandSpeed = 1f;        // Geschwindigkeit des laufenden Befehls (0..1)
        float overridePercent = 100f;   // globaler Override (1..100)
        double time;                    // "Uhr des Controllers" in Sekunden

        public SimulatedConnection(float[] minAngles, float[] maxAngles,
                                   float[] maxSpeeds, float[] maxAccelerations, float[] startAngles)
        {
            this.minAngles = (float[])minAngles.Clone();
            this.maxAngles = (float[])maxAngles.Clone();
            simulator = new JointMotionSimulator(minAngles, maxAngles, maxSpeeds, maxAccelerations, startAngles);
        }

        public string Name => "Simulation";
        public ConnectionStatus Status => status;
        public ControlRole Role => ControlRole.Owner;   // die Simulation gehört immer Unity

        public event Action<ConnectionStatus> StatusChanged;
        public event Action<RobotState> StateReceived;
        public event Action<CommandResult> CommandResultReceived;

        // Rollenwechsel und Warnungen gibt es in der Simulation nicht. Leere add/remove-Blöcke
        // erfüllen das Interface, ohne dass der Compiler "Event wird nie benutzt" meldet.
        public event Action<ControlRole> RoleChanged { add { } remove { } }
        public event Action<string> Warning { add { } remove { } }

        // Die Simulation ist sofort "verbunden" – beim Netzwerk gibt es dazwischen "Connecting".
        public void Connect() => SetStatus(ConnectionStatus.Connected);

        public void Disconnect()
        {
            simulator.Stop();   // Sicherheit: ohne Verbindung keine Bewegung
            SetStatus(ConnectionStatus.Disconnected);
        }

        public int SendCommand(RobotCommand command)
        {
            command.Id = nextCommandId++;
            pendingResults.Enqueue(Execute(command));   // Antwort erst im nächsten Tick()
            return command.Id;
        }

        public void Tick(float deltaTime)
        {
            if (statusChangePending)
            {
                statusChangePending = false;
                StatusChanged?.Invoke(status);
            }

            // Antworten ausliefern – auch die "not_connected"-Fehler bei getrennter Verbindung.
            while (pendingResults.Count > 0)
                CommandResultReceived?.Invoke(pendingResults.Dequeue());

            if (status != ConnectionStatus.Connected)
                return;   // getrennt: keine Bewegung, keine Zustandsmeldungen

            simulator.Step(deltaTime);
            time += deltaTime;

            // Neues Array pro Meldung: Empfänger dürfen es behalten, ohne dass es sich später ändert.
            var joints = new float[simulator.JointCount];
            for (int i = 0; i < joints.Length; i++)
                joints[i] = simulator.GetCurrent(i);

            RobotStatus robotStatus = simulator.IsMoving ? RobotStatus.Moving : RobotStatus.Idle;
            StateReceived?.Invoke(new RobotState(joints, robotStatus, overridePercent, time));
        }

        // ---- Befehle prüfen und ausführen ----

        CommandResult Execute(RobotCommand command)
        {
            if (status != ConnectionStatus.Connected)
                return CommandResult.Error(command.Id, "not_connected", "Nicht verbunden.");

            switch (command)
            {
                case MoveJointsCommand move:
                    return ExecuteMoveJoints(move);

                case StopCommand _:
                    simulator.Stop();
                    return CommandResult.Ack(command.Id);

                case SetOverrideCommand setOverride:
                    return ExecuteSetOverride(setOverride);

                default:
                    return CommandResult.Error(command.Id, "unknown_command",
                        $"Unbekannter Befehl {command.GetType().Name}.");
            }
        }

        // Ablehnen statt still begrenzen: Der Roboter fährt entweder genau wie befohlen – oder gar nicht.
        CommandResult ExecuteMoveJoints(MoveJointsCommand move)
        {
            float[] joints = move.Joints;
            if (joints == null || joints.Length != minAngles.Length)
                return CommandResult.Error(move.Id, "invalid_command",
                    $"Erwartet {minAngles.Length} Gelenkwinkel.");

            // So formuliert, dass auch NaN abgelehnt wird (jeder Vergleich mit NaN ist false).
            if (!(move.Speed > 0f && move.Speed <= 1f))
                return CommandResult.Error(move.Id, "invalid_command",
                    $"Geschwindigkeit {move.Speed} außerhalb (0, 1].");

            for (int i = 0; i < joints.Length; i++)
            {
                if (!(joints[i] >= minAngles[i] && joints[i] <= maxAngles[i]))
                    return CommandResult.Error(move.Id, "joint_limit",
                        $"Gelenk {i + 1}: {joints[i]:F1}° außerhalb [{minAngles[i]:F0}°, {maxAngles[i]:F0}°].");
            }

            commandSpeed = move.Speed;
            simulator.SpeedFactor = commandSpeed * overridePercent / 100f;
            simulator.SetTargets(joints);
            return CommandResult.Ack(move.Id);
        }

        CommandResult ExecuteSetOverride(SetOverrideCommand setOverride)
        {
            if (!(setOverride.Percent >= 1f && setOverride.Percent <= 100f))
                return CommandResult.Error(setOverride.Id, "invalid_command",
                    $"Override {setOverride.Percent} außerhalb [1, 100] %.");

            overridePercent = setOverride.Percent;
            simulator.SpeedFactor = commandSpeed * overridePercent / 100f;   // wirkt auch auf laufende Bewegung
            return CommandResult.Ack(setOverride.Id);
        }

        void SetStatus(ConnectionStatus newStatus)
        {
            if (status == newStatus)
                return;
            status = newStatus;
            statusChangePending = true;
        }
    }
}

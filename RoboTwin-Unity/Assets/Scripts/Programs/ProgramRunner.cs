using System;
using RoboTwin.Connection;
using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Programs
{
    public enum RunnerState
    {
        Stopped,   // steht, nächster Start beginnt bei Punkt 1
        Running,   // fährt das Programm ab
        Paused     // angehalten, Fortsetzen fährt den unterbrochenen Punkt erneut an
    }

    /// <summary>
    /// Spielt ein Programm ab: eine kleine Zustandsmaschine.
    ///
    /// Running durchläuft pro Punkt zwei Phasen:
    ///   Moving  – move_joints gesendet, warten bis der Roboter steht UND am Punkt ist,
    ///   Waiting – die Wartezeit des Punkts abwarten, dann der nächste Punkt.
    /// Bricht ab bei: abgelehntem Befehl, Not-Halt, Verlust von Verbindung oder Steuerung, Zeitüberschreitung.
    /// Funktioniert gleich in Simulation und Netzwerk – es spricht nur mit IRobotConnection.
    /// </summary>
    public class ProgramRunner : MonoBehaviour
    {
        const float ArrivalTolerance = 0.01f;   // Grad – Ist ≈ Soll
        const float MoveTimeoutSeconds = 120f;  // so lange darf ein einzelner Punkt höchstens dauern

        [SerializeField] RobotModel model;
        [SerializeField] RobotConnectionHost host;

        enum Phase { Moving, Waiting }

        Phase phase;
        int pendingCommandId;
        float phaseDeadline;   // Moving: Zeitlimit, Waiting: Ende der Wartezeit

        public RobotProgram Program { get; set; } = new RobotProgram();
        public RunnerState State { get; private set; } = RunnerState.Stopped;
        public int CurrentIndex { get; private set; } = -1;   // -1 = kein aktueller Punkt
        public bool Loop { get; set; }
        public string Message { get; private set; } = "";

        /// <summary>Zustand, aktueller Punkt oder Meldung haben sich geändert.</summary>
        public event Action Changed;

        void Reset()
        {
            model = GetComponent<RobotModel>();
            host = GetComponent<RobotConnectionHost>();
        }

        void Start() => host.CommandResultReceived += OnCommandResult;

        void OnDestroy()
        {
            if (host != null)
                host.CommandResultReceived -= OnCommandResult;
        }

        // ---- Bedienung ----

        public void Play()
        {
            if (State == RunnerState.Running)
                return;
            if (Program.Points.Count == 0)
            {
                SetMessage("Programm ist leer.");
                return;
            }
            if (!host.CanMove)
            {
                SetMessage("Start nicht möglich: nicht verbunden, nur Beobachter oder Not-Halt aktiv.");
                return;
            }

            if (State == RunnerState.Stopped)
                CurrentIndex = 0;
            State = RunnerState.Running;
            SendCurrentPoint();   // beim Fortsetzen: den unterbrochenen Punkt erneut anfahren
        }

        public void Pause()
        {
            if (State != RunnerState.Running)
                return;
            host.Connection.SendCommand(new StopCommand());
            State = RunnerState.Paused;
            SetMessage($"Pausiert bei Punkt {CurrentIndex + 1}/{Program.Points.Count}.");
        }

        public void Stop()
        {
            if (State == RunnerState.Stopped)
                return;
            if (State == RunnerState.Running)
                host.Connection.SendCommand(new StopCommand());
            Finish("Gestoppt.");
        }

        // ---- Ablauf ----

        void Update()
        {
            if (State == RunnerState.Stopped)
                return;

            // Sicherheit zuerst – auch im Pausenzustand.
            if (model.Status == RobotStatus.Error)
            {
                Finish($"Abgebrochen: Not-Halt ({model.ErrorMessage}).");
                return;
            }
            if (!host.CanControl)
            {
                Finish("Abgebrochen: Verbindung oder Steuerung verloren.");
                return;
            }
            if (State != RunnerState.Running)
                return;

            ProgramPoint point = Program.Points[CurrentIndex];

            if (phase == Phase.Moving)
            {
                if (Time.time > phaseDeadline)
                {
                    host.Connection.SendCommand(new StopCommand());
                    Finish($"Abgebrochen: Punkt {CurrentIndex + 1} nicht rechtzeitig erreicht.");
                    return;
                }
                // Angekommen = steht UND ist am Punkt. Nur "steht" reicht nicht: Direkt nach dem Senden
                // meldet der Controller evtl. noch einen Zustand von VOR dem Befehl (idle am alten Ort).
                if (model.IsMoving || !IsAt(point.Joints))
                    return;

                phase = Phase.Waiting;
                phaseDeadline = Time.time + point.Wait;
            }

            if (phase == Phase.Waiting && Time.time >= phaseDeadline)
                Advance();
        }

        void Advance()
        {
            CurrentIndex++;
            if (CurrentIndex >= Program.Points.Count)
            {
                if (!Loop)
                {
                    Finish("Programm beendet.");
                    return;
                }
                CurrentIndex = 0;
            }
            SendCurrentPoint();
        }

        void SendCurrentPoint()
        {
            ProgramPoint point = Program.Points[CurrentIndex];
            phase = Phase.Moving;
            phaseDeadline = Time.time + MoveTimeoutSeconds;
            pendingCommandId = host.Connection.SendCommand(new MoveJointsCommand(point.Joints, point.Speed));
            SetMessage($"Punkt {CurrentIndex + 1}/{Program.Points.Count}: {point.Name}");
        }

        // Lehnt der Controller UNSEREN Fahrbefehl ab (z. B. joint_limit), bricht das Programm ab.
        void OnCommandResult(CommandResult result)
        {
            if (State == RunnerState.Running && result.CommandId == pendingCommandId && !result.Accepted)
                Finish($"Abgebrochen: Punkt {CurrentIndex + 1} abgelehnt – {result.Message}");
        }

        bool IsAt(float[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
                if (Mathf.Abs(model.ActualAngles[i] - joints[i]) > ArrivalTolerance)
                    return false;
            return true;
        }

        void Finish(string message)
        {
            State = RunnerState.Stopped;
            CurrentIndex = -1;
            SetMessage(message);
        }

        void SetMessage(string message)
        {
            Message = message;
            Changed?.Invoke();
        }
    }
}

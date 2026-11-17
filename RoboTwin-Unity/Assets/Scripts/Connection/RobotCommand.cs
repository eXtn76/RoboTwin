namespace RoboTwin.Connection
{
    /// <summary>
    /// Basis aller Befehle an den Roboter. Jeder Befehl bekommt beim Senden eine Id,
    /// über die die Antwort (CommandResult) zugeordnet wird.
    /// </summary>
    public abstract class RobotCommand
    {
        // "internal set": nur Code in diesem Projekt (die Verbindung) darf die Id vergeben.
        public int Id { get; internal set; }
    }

    /// <summary>Alle Gelenke synchron zu den Zielwinkeln fahren. Später: "move_joints".</summary>
    public sealed class MoveJointsCommand : RobotCommand
    {
        public readonly float[] Joints;   // Grad, URDF-Konvention
        public readonly float Speed;      // Anteil der Maximalgeschwindigkeit, (0, 1]

        public MoveJointsCommand(float[] joints, float speed = 1f)
        {
            // Kopie: Der Absender darf sein Array danach weiterverwenden, ohne den Befehl zu verändern.
            Joints = joints == null ? null : (float[])joints.Clone();
            Speed = speed;
        }
    }

    /// <summary>Sofort anhalten. Später: "stop".</summary>
    public sealed class StopCommand : RobotCommand
    {
    }

    /// <summary>Geschwindigkeits-Override setzen (1 … 100 %), wirkt auch auf laufende Bewegungen.</summary>
    public sealed class SetOverrideCommand : RobotCommand
    {
        public readonly float Percent;

        public SetOverrideCommand(float percent)
        {
            Percent = percent;
        }
    }

    /// <summary>
    /// Antwort des Controllers auf einen Befehl: angenommen ("ack") oder abgelehnt ("error").
    /// ErrorCode ist maschinenlesbar (z. B. "joint_limit"), Message für Menschen gedacht.
    /// </summary>
    public readonly struct CommandResult
    {
        public readonly int CommandId;
        public readonly bool Accepted;
        public readonly string ErrorCode;
        public readonly string Message;

        CommandResult(int commandId, bool accepted, string errorCode, string message)
        {
            CommandId = commandId;
            Accepted = accepted;
            ErrorCode = errorCode;
            Message = message;
        }

        public static CommandResult Ack(int commandId) =>
            new CommandResult(commandId, true, null, null);

        public static CommandResult Error(int commandId, string errorCode, string message) =>
            new CommandResult(commandId, false, errorCode, message);
    }
}

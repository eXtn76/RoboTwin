namespace RoboTwin.Connection
{
    /// <summary>Zustand der Verbindung – unabhängig davon, was der Roboter tut.</summary>
    public enum ConnectionStatus
    {
        Disconnected,   // nicht verbunden (Ausgangszustand, oder bewusst getrennt)
        Connecting,     // Verbindungsaufbau läuft (beim Netzwerk dauert das)
        Connected,      // verbunden: Befehle werden angenommen, Zustände kommen an
        Failed          // Verbindung verloren oder Aufbau gescheitert
    }

    /// <summary>Darf dieser Client steuern? (PROTOCOL.md, Abschnitt 4)</summary>
    public enum ControlRole
    {
        Owner,      // Besitzer: darf alle Befehle senden
        Observer    // Beobachter: sieht den Zustand, darf nur anhalten
    }

    /// <summary>Was der Roboter laut Controller gerade tut.</summary>
    public enum RobotStatus
    {
        Idle,       // steht, bereit für Befehle
        Moving,     // fährt
        Error       // Fehlerzustand (z. B. Not-Halt, ab M7)
    }

    /// <summary>
    /// Ist-Zustand, wie ihn der Controller meldet. Vorlage für die spätere "state"-Nachricht.
    /// Alle Winkel in Grad, URDF-Konvention.
    /// </summary>
    public readonly struct RobotState
    {
        public readonly float[] Joints;
        public readonly RobotStatus Status;
        public readonly float OverridePercent;
        public readonly double Timestamp;   // Sekunden, Uhr des CONTROLLERS (nicht die von Unity)

        public RobotState(float[] joints, RobotStatus status, float overridePercent, double timestamp)
        {
            Joints = joints;
            Status = status;
            OverridePercent = overridePercent;
            Timestamp = timestamp;
        }
    }
}

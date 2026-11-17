using System;

namespace RoboTwin.Connection
{
    /// <summary>
    /// Verbindung zu einem Roboter-Controller – egal ob in Unity simuliert oder über das Netzwerk.
    /// Die UI kennt nur dieses Interface, nie die konkrete Implementierung.
    ///
    /// Regel für alle Implementierungen: Events werden NUR aus Tick() ausgelöst,
    /// also immer im Haupt-Thread von Unity und nie mitten in einem SendCommand()-Aufruf.
    /// </summary>
    public interface IRobotConnection
    {
        /// <summary>Anzeigename, z. B. "Simulation" oder "Netzwerk".</summary>
        string Name { get; }

        ConnectionStatus Status { get; }

        event Action<ConnectionStatus> StatusChanged;
        event Action<RobotState> StateReceived;
        event Action<CommandResult> CommandResultReceived;

        void Connect();
        void Disconnect();

        /// <summary>
        /// Sendet einen Befehl und gibt die vergebene Id zurück.
        /// Die Antwort (angenommen/abgelehnt) kommt später über CommandResultReceived.
        /// </summary>
        int SendCommand(RobotCommand command);

        /// <summary>
        /// Wird vom Host einmal pro Frame im Haupt-Thread aufgerufen.
        /// Hier rechnet die Simulation bzw. hier werden empfangene Netzwerknachrichten ausgeliefert.
        /// </summary>
        void Tick(float deltaTime);
    }
}

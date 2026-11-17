using System.Collections.Generic;
using RoboTwin.Connection;
using UnityEngine;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Verbindungszeile im Panel: Modus, Verbinden/Trennen, Verbindungsstatus und
    /// die letzte Fehlermeldung des Controllers (abgelehnte Befehle).
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class ConnectionBar : MonoBehaviour
    {
        [SerializeField] RobotConnectionHost host;

        DropdownField modeField;
        Button connectButton;
        Label statusLabel;
        Label errorLabel;

        IRobotConnection Connection => host.Connection;

        void Start()
        {
            VisualElement root = GetComponent<UIDocument>().rootVisualElement;
            modeField = root.Q<DropdownField>("connection-mode");
            connectButton = root.Q<Button>("connect-button");
            statusLabel = root.Q<Label>("connection-status");
            errorLabel = root.Q<Label>("error-label");

            // Vorerst gibt es nur die Simulation – "Netzwerk" kommt in M6 dazu.
            modeField.choices = new List<string> { Connection.Name };
            modeField.index = 0;

            connectButton.clicked += OnConnectClicked;
            Connection.StatusChanged += ShowStatus;
            Connection.CommandResultReceived += OnCommandResult;

            // Aktuellen Stand sofort anzeigen: Das erste StatusChanged-Event
            // kann schon vor diesem Start() ausgelöst worden sein.
            ShowStatus(Connection.Status);
            errorLabel.text = "";
        }

        void OnDestroy()
        {
            if (host == null || host.Connection == null)
                return;
            Connection.StatusChanged -= ShowStatus;
            Connection.CommandResultReceived -= OnCommandResult;
        }

        void OnConnectClicked()
        {
            if (Connection.Status == ConnectionStatus.Connected)
                Connection.Disconnect();
            else
                Connection.Connect();
        }

        void ShowStatus(ConnectionStatus status)
        {
            statusLabel.text = status switch
            {
                ConnectionStatus.Disconnected => "Getrennt",
                ConnectionStatus.Connecting => "Verbinde …",
                ConnectionStatus.Connected => "Verbunden",
                ConnectionStatus.Failed => "Verbindung verloren",
                _ => status.ToString()
            };
            statusLabel.EnableInClassList("message--ok", status == ConnectionStatus.Connected);
            statusLabel.EnableInClassList("message--error", status != ConnectionStatus.Connected);

            connectButton.text = status == ConnectionStatus.Connected ? "Trennen" : "Verbinden";
        }

        // Abgelehnte Befehle anzeigen; der nächste angenommene Befehl löscht die Meldung wieder.
        void OnCommandResult(CommandResult result)
        {
            errorLabel.text = result.Accepted ? "" : $"Befehl #{result.CommandId} abgelehnt: {result.Message}";
        }
    }
}

using System.Collections.Generic;
using RoboTwin.Connection;
using UnityEngine;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Verbindungszeile im Panel: Modus (Simulation/Netzwerk), Adresse, Verbinden/Trennen,
    /// Status, Rolle, Warnungen und abgelehnte Befehle.
    /// Hört nur auf die Events des Hosts – überlebt also das Umschalten der Verbindung.
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class ConnectionBar : MonoBehaviour
    {
        [SerializeField] RobotConnectionHost host;

        // Reihenfolge = enum ConnectionMode (Simulation = 0, Network = 1).
        static readonly List<string> ModeNames = new List<string> { "Simulation", "Netzwerk" };

        DropdownField modeField;
        Button connectButton;
        Label statusLabel;
        VisualElement networkSettings;
        TextField hostField;
        IntegerField portField;
        Label roleLabel;
        Label warningLabel;
        Label errorLabel;

        ConnectionMode SelectedMode => (ConnectionMode)modeField.index;

        void Start()
        {
            VisualElement root = GetComponent<UIDocument>().rootVisualElement;
            modeField = root.Q<DropdownField>("connection-mode");
            connectButton = root.Q<Button>("connect-button");
            statusLabel = root.Q<Label>("connection-status");
            networkSettings = root.Q<VisualElement>("network-settings");
            hostField = root.Q<TextField>("network-host");
            portField = root.Q<IntegerField>("network-port");
            roleLabel = root.Q<Label>("connection-role");
            warningLabel = root.Q<Label>("warning-label");
            errorLabel = root.Q<Label>("error-label");

            modeField.choices = ModeNames;
            modeField.index = (int)host.Mode;   // VOR dem Registrieren des Callbacks
            hostField.SetValueWithoutNotify(host.NetworkHost);
            portField.SetValueWithoutNotify(host.NetworkPort);

            modeField.RegisterValueChangedCallback(_ => OnModeChanged());
            connectButton.clicked += OnConnectClicked;

            host.StatusChanged += ShowStatus;
            host.RoleChanged += ShowRole;
            host.CommandResultReceived += OnCommandResult;
            host.Warning += ShowWarning;
            host.ConnectionSwitched += OnConnectionSwitched;

            // Aktuellen Stand sofort anzeigen – frühere Events können schon vorbei sein.
            OnConnectionSwitched();
            warningLabel.text = "";
            errorLabel.text = "";
        }

        void OnDestroy()
        {
            if (host == null)
                return;
            host.StatusChanged -= ShowStatus;
            host.RoleChanged -= ShowRole;
            host.CommandResultReceived -= OnCommandResult;
            host.Warning -= ShowWarning;
            host.ConnectionSwitched -= OnConnectionSwitched;
        }

        void OnModeChanged()
        {
            warningLabel.text = "";
            errorLabel.text = "";
            host.SwitchTo(SelectedMode, hostField.value.Trim(), portField.value);
        }

        void OnConnectClicked()
        {
            if (host.Connection.Status != ConnectionStatus.Disconnected)
            {
                host.Connection.Disconnect();   // trennt auch während "Verbinde …" und beendet Neuversuche
                return;
            }

            if (host.Mode == ConnectionMode.Network)
                host.SwitchTo(ConnectionMode.Network, hostField.value.Trim(), portField.value);   // aktuelle Adresse
            else
                host.Connection.Connect();
        }

        void OnConnectionSwitched()
        {
            networkSettings.style.display = host.Mode == ConnectionMode.Network ? DisplayStyle.Flex : DisplayStyle.None;
            ShowStatus(host.Connection.Status);
        }

        void ShowStatus(ConnectionStatus status)
        {
            string address = host.Mode == ConnectionMode.Network ? $" ({host.NetworkHost}:{host.NetworkPort})" : "";
            statusLabel.text = status switch
            {
                ConnectionStatus.Disconnected => "Getrennt",
                ConnectionStatus.Connecting => "Verbinde …" + address,
                ConnectionStatus.Connected => "Verbunden" + address,
                ConnectionStatus.Failed => "Verbindung verloren",
                _ => status.ToString()
            };
            statusLabel.EnableInClassList("message--ok", status == ConnectionStatus.Connected);
            statusLabel.EnableInClassList("message--error", status != ConnectionStatus.Connected);

            connectButton.text = status == ConnectionStatus.Disconnected ? "Verbinden" : "Trennen";

            // Adresse nur im getrennten Zustand änderbar.
            hostField.SetEnabled(status == ConnectionStatus.Disconnected);
            portField.SetEnabled(status == ConnectionStatus.Disconnected);

            if (status == ConnectionStatus.Connected)
                warningLabel.text = "";

            ShowRole(host.Connection.Role);
        }

        void ShowRole(ControlRole role)
        {
            if (!host.IsConnected)
                roleLabel.text = "";
            else
                roleLabel.text = role == ControlRole.Owner ? "Rolle: Besitzer (steuert)" : "Rolle: Beobachter (nur Stop)";
        }

        void ShowWarning(string message) => warningLabel.text = message;

        // Abgelehnte Befehle anzeigen; der nächste angenommene Befehl löscht die Meldung wieder.
        void OnCommandResult(CommandResult result)
        {
            if (result.Accepted)
                errorLabel.text = "";
            else if (result.CommandId > 0)
                errorLabel.text = $"Befehl #{result.CommandId} abgelehnt: {result.Message}";
            else
                errorLabel.text = $"Nachricht abgelehnt: {result.Message}";
        }
    }
}

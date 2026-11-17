using System;
using System.Collections.Generic;
using RoboTwin.Connection;
using RoboTwin.Kinematics;
using RoboTwin.Model;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Bedienpanel für den Joint-Modus: ein Slider pro Gelenk (Soll), Anzeige der Istwerte
    /// und der TCP-Position, Override, Stop und Home.
    /// Sendet Befehle über IRobotConnection, liest Istwerte vom Modell.
    /// Darf Unity nicht fahren (getrennt, Beobachter, Not-Halt), folgen die Slider nur dem Ist-Zustand.
    /// Not-Halt per Button oder Escape-Taste.
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class JointPanel : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] RobotConnectionHost host;
        [SerializeField] RobotKinematics kinematics;
        [SerializeField] VisualTreeAsset jointRowTemplate;

        readonly List<Slider> targetSliders = new List<Slider>();
        readonly List<Label> actualLabels = new List<Label>();
        SliderInt overrideSlider;
        Button stopButton;
        Button homeButton;
        Button emergencyStopButton;
        Button resetButton;
        Label tcpLabel;
        Label statusLabel;

        // Die zuletzt befohlenen Sollwerte. Die UI merkt sie sich SELBST –
        // die Verbindung (z. B. ein entfernter Controller) kann man danach nicht fragen.
        float[] targets;
        bool wasMoving;
        bool resyncOnNextState;   // nach dem Umschalten: Slider an den ersten neuen Zustand anpassen

        // Die UI kennt nur das Interface, nicht die konkrete Verbindung.
        IRobotConnection Connection => host.Connection;

        // Start statt OnEnable: Wir lesen beim Aufbau model.ActualAngles, und die füllt
        // RobotModel erst in seinem Awake. Alle Awakes sind garantiert vor dem ersten Start fertig.
        void Start()
        {
            VisualElement root = GetComponent<UIDocument>().rootVisualElement;

            targets = new float[model.JointCount];
            for (int i = 0; i < targets.Length; i++)
                targets[i] = model.ActualAngles[i];

            overrideSlider = root.Q<SliderInt>("override-slider");
            overrideSlider.RegisterValueChangedCallback(evt => Connection.SendCommand(new SetOverrideCommand(evt.newValue)));

            stopButton = root.Q<Button>("stop-button");
            homeButton = root.Q<Button>("home-button");
            stopButton.clicked += OnStopClicked;
            homeButton.clicked += OnHomeClicked;
            emergencyStopButton = root.Q<Button>("emergency-stop-button");
            resetButton = root.Q<Button>("reset-button");
            emergencyStopButton.clicked += OnEmergencyStopClicked;
            resetButton.clicked += OnResetClicked;
            tcpLabel = root.Q<Label>("tcp-label");
            statusLabel = root.Q<Label>("status-label");

            BuildJointRows(root.Q<VisualElement>("joint-rows"));

            model.StateChanged += OnStateChanged;
            host.ControlChanged += UpdateControls;
            host.ConnectionSwitched += OnConnectionSwitched;
            UpdateControls();
        }

        void OnDestroy()
        {
            // Abos beenden – sonst rufen Modell und Host später ein zerstörtes Panel auf.
            if (model != null)
                model.StateChanged -= OnStateChanged;
            if (host != null)
            {
                host.ControlChanged -= UpdateControls;
                host.ConnectionSwitched -= OnConnectionSwitched;
            }
        }

        // Not-Halt auch per Tastatur (Escape) – am echten Pendant ist er ein Taster, kein Menüpunkt.
        void Update()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && keyboard.escapeKey.wasPressedThisFrame)
                OnEmergencyStopClicked();
        }

        // Erzeugt pro Gelenk eine Zeile aus der Vorlage JointRow.uxml.
        void BuildJointRows(VisualElement container)
        {
            for (int i = 0; i < model.JointCount; i++)
            {
                JointDriver joint = model.GetJoint(i);

                VisualElement row = jointRowTemplate.Instantiate();
                container.Add(row);

                // Auf der Zeile suchen, nicht auf root: Alle 6 Zeilen haben dieselben Namen,
                // root.Q<>() würde immer die Elemente der ERSTEN Zeile finden.
                Label nameLabel = row.Q<Label>("joint-name");
                Slider slider = row.Q<Slider>("target-slider");
                Label actualLabel = row.Q<Label>("actual-value");

                nameLabel.text = $"J{i + 1} {joint.JointName.Replace("_joint", "")}";
                slider.lowValue = joint.MinAngle;
                slider.highValue = joint.MaxAngle;
                slider.SetValueWithoutNotify(targets[i]);   // Startwert, ohne einen Befehl auszulösen

                // Lambda-Falle: Das Lambda merkt sich die VARIABLE, nicht ihren Wert.
                // Ohne Kopie wäre i beim Ziehen längst 6 → IndexOutOfRangeException.
                int index = i;
                slider.RegisterValueChangedCallback(evt =>
                {
                    targets[index] = evt.newValue;
                    SendTargets();
                });

                targetSliders.Add(slider);
                actualLabels.Add(actualLabel);
            }
        }

        // Ein Befehl mit ALLEN Sollwerten – auch wenn nur ein Slider bewegt wurde.
        // MoveJointsCommand kopiert das Array, wir dürfen 'targets' danach weiter verändern.
        void SendTargets() => Connection.SendCommand(new MoveJointsCommand(targets));

        // Wird jedes Mal aufgerufen, wenn das Modell einen neuen Ist-Zustand übernommen hat.
        void OnStateChanged()
        {
            IReadOnlyList<float> angles = model.ActualAngles;
            for (int i = 0; i < actualLabels.Count; i++)
                actualLabels[i].text = $"{angles[i]:F1}°";   // Anzeige: Format der Systemsprache ist hier richtig

            Vector3 tcp = kinematics.CurrentTcpMm();
            tcpLabel.text = $"TCP   X {tcp.x:F1}   Y {tcp.y:F1}   Z {tcp.z:F1} mm";

            statusLabel.text = model.Status switch
            {
                RobotStatus.Moving => "Status: fährt",
                RobotStatus.Error => $"NOT-HALT: {model.ErrorMessage}",
                _ => "Status: steht"
            };
            statusLabel.EnableInClassList("message--error", model.Status == RobotStatus.Error);

            // Slider auf den Ist-Zustand setzen, wenn:
            // - wir nicht fahren dürfen (Beobachter, Not-Halt): Die Slider spiegeln den Ist-Zustand,
            // - gerade umgeschaltet wurde: der neue Controller steht evtl. ganz woanders,
            // - eine Bewegung gerade zu Ende ist (Slider, Home, Stop oder kartesischer Modus).
            bool motionFinished = wasMoving && !model.IsMoving;
            if (!host.CanMove || resyncOnNextState || motionFinished)
            {
                SyncTargetsToActual();
                overrideSlider.SetValueWithoutNotify(Mathf.RoundToInt(model.OverridePercent));
                resyncOnNextState = false;
            }
            wasMoving = model.IsMoving;
        }

        void OnConnectionSwitched() => resyncOnNextState = true;

        // Bedienbarkeit an Verbindung, Rolle und Not-Halt anpassen.
        void UpdateControls()
        {
            bool canMove = host.CanMove;
            foreach (Slider slider in targetSliders)
                slider.SetEnabled(canMove);
            homeButton.SetEnabled(canMove);
            overrideSlider.SetEnabled(host.CanControl);   // Override ist auch im Not-Halt erlaubt

            // Anhalten darf jeder Verbundene, auch ein Beobachter.
            stopButton.SetEnabled(host.IsConnected);
            emergencyStopButton.SetEnabled(host.IsConnected);

            // Quittieren nur als Besitzer und nur, wenn es etwas zu quittieren gibt.
            resetButton.SetEnabled(host.CanControl && model.Status == RobotStatus.Error);
        }

        void OnStopClicked()
        {
            Connection.SendCommand(new StopCommand());
            SyncTargetsToActual();
        }

        void OnEmergencyStopClicked()
        {
            Connection.SendCommand(new EmergencyStopCommand());
            SyncTargetsToActual();
        }

        void OnResetClicked() => Connection.SendCommand(new ResetCommand());

        void OnHomeClicked()
        {
            Array.Clear(targets, 0, targets.Length);   // alle Sollwerte auf 0°
            SendTargets();                              // ein Befehl → eine gemeinsame, synchrone Bahn
            SyncSliders();
        }

        void SyncTargetsToActual()
        {
            for (int i = 0; i < targets.Length; i++)
                targets[i] = model.ActualAngles[i];
            SyncSliders();
        }

        // Slider zeigen die Sollwerte – ohne dabei erneut einen Befehl auszulösen.
        void SyncSliders()
        {
            for (int i = 0; i < targetSliders.Count; i++)
                targetSliders[i].SetValueWithoutNotify(targets[i]);
        }
    }
}

using System;
using System.Collections.Generic;
using RoboTwin.Connection;
using RoboTwin.Kinematics;
using RoboTwin.Model;
using UnityEngine;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Bedienpanel für den Joint-Modus: ein Slider pro Gelenk (Soll), Anzeige der Istwerte
    /// und der TCP-Position, Override, Stop und Home.
    /// Sendet Befehle über IRobotConnection, liest Istwerte vom Modell.
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
        Label tcpLabel;
        Label statusLabel;

        // Die zuletzt befohlenen Sollwerte. Die UI merkt sie sich SELBST –
        // die Verbindung (z. B. ein entfernter Controller) kann man danach nicht fragen.
        float[] targets;
        bool wasMoving;

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

            SliderInt overrideSlider = root.Q<SliderInt>("override-slider");
            overrideSlider.RegisterValueChangedCallback(evt => Connection.SendCommand(new SetOverrideCommand(evt.newValue)));

            root.Q<Button>("stop-button").clicked += OnStopClicked;
            root.Q<Button>("home-button").clicked += OnHomeClicked;
            tcpLabel = root.Q<Label>("tcp-label");
            statusLabel = root.Q<Label>("status-label");

            BuildJointRows(root.Q<VisualElement>("joint-rows"));

            model.StateChanged += OnStateChanged;
        }

        void OnDestroy()
        {
            // Abo beenden – sonst ruft das Modell später ein zerstörtes Panel auf.
            if (model != null)
                model.StateChanged -= OnStateChanged;
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
                RobotStatus.Error => "Status: Fehler",
                _ => "Status: steht"
            };

            // Bewegung gerade zu Ende (egal ob durch Slider, Home, Stop oder den kartesischen Modus):
            // Sollwerte und Slider auf die tatsächliche Position setzen.
            if (wasMoving && !model.IsMoving)
                SyncTargetsToActual();
            wasMoving = model.IsMoving;
        }

        void OnStopClicked()
        {
            Connection.SendCommand(new StopCommand());
            SyncTargetsToActual();
        }

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

using System.Collections.Generic;
using RoboTwin.Kinematics;
using RoboTwin.Model;
using UnityEngine;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Bedienpanel für den Joint-Modus: ein Slider pro Gelenk (Soll), Anzeige der Istwerte
    /// und der TCP-Position, Override, Stop und Home.
    /// Schreibt Sollwerte an die Simulation, liest Istwerte vom Modell.
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class JointPanel : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] LocalRobotSimulation simulation;
        [SerializeField] RobotKinematics kinematics;
        [SerializeField] VisualTreeAsset jointRowTemplate;

        readonly List<Slider> targetSliders = new List<Slider>();
        readonly List<Label> actualLabels = new List<Label>();
        Label tcpLabel;
        Label statusLabel;

        // Start statt OnEnable: Wir lesen beim Aufbau model.ActualAngles, und die füllt
        // RobotModel erst in seinem Awake. Alle Awakes sind garantiert vor dem ersten Start fertig.
        void Start()
        {
            VisualElement root = GetComponent<UIDocument>().rootVisualElement;

            SliderInt overrideSlider = root.Q<SliderInt>("override-slider");
            overrideSlider.RegisterValueChangedCallback(evt => simulation.OverridePercent = evt.newValue);

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
                slider.SetValueWithoutNotify(model.ActualAngles[i]);   // Startwert, ohne SetTarget auszulösen

                // Lambda-Falle: Das Lambda merkt sich die VARIABLE, nicht ihren Wert.
                // Ohne Kopie wäre i beim Ziehen längst 6 → IndexOutOfRangeException.
                int index = i;
                slider.RegisterValueChangedCallback(evt => simulation.SetTarget(index, evt.newValue));

                targetSliders.Add(slider);
                actualLabels.Add(actualLabel);
            }
        }

        // Wird jedes Mal aufgerufen, wenn das Modell einen neuen Ist-Zustand übernommen hat.
        void OnStateChanged()
        {
            IReadOnlyList<float> angles = model.ActualAngles;
            for (int i = 0; i < actualLabels.Count; i++)
                actualLabels[i].text = $"{angles[i]:F1}°";   // Anzeige: Format der Systemsprache ist hier richtig

            Vector3 tcp = kinematics.CurrentTcpMm();
            tcpLabel.text = $"TCP   X {tcp.x:F1}   Y {tcp.y:F1}   Z {tcp.z:F1} mm";

            statusLabel.text = simulation.IsMoving ? "Status: fährt" : "Status: steht";
        }

        void OnStopClicked()
        {
            simulation.Stop();
            SyncSlidersToTargets();
        }

        void OnHomeClicked()
        {
            // Ein Aufruf für alle Gelenke → eine gemeinsame, synchrone Bahn.
            simulation.SetTargets(new float[model.JointCount]);   // neues float-Array ist mit 0 gefüllt
            SyncSlidersToTargets();
        }

        // Nach Stop/Home haben sich die Ziele geändert, ohne dass jemand am Slider gezogen hat.
        // Die Slider sollen trotzdem das aktuelle Ziel zeigen – ohne erneut SetTarget auszulösen.
        void SyncSlidersToTargets()
        {
            for (int i = 0; i < targetSliders.Count; i++)
                targetSliders[i].SetValueWithoutNotify(simulation.GetTarget(i));
        }
    }
}
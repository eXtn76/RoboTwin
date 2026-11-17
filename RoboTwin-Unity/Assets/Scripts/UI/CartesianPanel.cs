using System.Collections.Generic;
using RoboTwin.Kinematics;
using UnityEngine;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Bedienpanel für den kartesischen Modus: Zielposition (Roboter-Basis, mm) und Werkzeug-Orientierung
    /// eingeben, Vorschau per Zielmarker (grün/rot), Fahren nur bei erreichbarem Ziel.
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class CartesianPanel : MonoBehaviour
    {
        [SerializeField] CartesianController controller;
        [SerializeField] RobotKinematics kinematics;
        [SerializeField] TargetMarker marker;

        FloatField xField;
        FloatField yField;
        FloatField zField;
        DropdownField orientationField;
        Label messageLabel;

        Vector3 TargetMm => new Vector3(xField.value, yField.value, zField.value);

        // Die Reihenfolge der Auswahl entspricht dem enum (Down = 0, Keep = 1),
        // daher lässt sich der Index direkt umwandeln.
        ToolOrientation Orientation => (ToolOrientation)orientationField.index;

        void Start()
        {
            VisualElement root = GetComponent<UIDocument>().rootVisualElement;

            xField = root.Q<FloatField>("target-x");
            yField = root.Q<FloatField>("target-y");
            zField = root.Q<FloatField>("target-z");
            orientationField = root.Q<DropdownField>("orientation");
            messageLabel = root.Q<Label>("message-label");

            orientationField.choices = new List<string> { "Nach unten", "Beibehalten" };
            orientationField.index = 0;   // VOR dem Registrieren der Callbacks – löst also nichts aus

            // Jede Änderung → Vorschau neu berechnen. Der Parameter (das Event) wird nicht gebraucht: "_".
            xField.RegisterValueChangedCallback(_ => UpdatePreview());
            yField.RegisterValueChangedCallback(_ => UpdatePreview());
            zField.RegisterValueChangedCallback(_ => UpdatePreview());
            orientationField.RegisterValueChangedCallback(_ => UpdatePreview());

            root.Q<Button>("move-button").clicked += OnMoveClicked;
            root.Q<Button>("take-current-button").clicked += OnTakeCurrentClicked;

            UpdatePreview();
        }

        // Nur planen, nicht fahren: zeigt, ob das Ziel erreichbar wäre.
        void UpdatePreview()
        {
            IkResult result = controller.Plan(TargetMm, Orientation);
            marker.Show(kinematics.RobotBaseToWorld(TargetMm), result.Success);

            if (result.Success)
                ShowMessage("Ziel erreichbar.", true);
            else
                ShowMessage($"Ziel unerreichbar (Restfehler {result.PositionErrorMm:F1} mm / " +
                            $"{result.OrientationErrorDeg:F1}°).", false);
        }

        void OnMoveClicked()
        {
            bool success = controller.MoveTo(TargetMm, Orientation, out IkResult result);
            marker.Show(kinematics.RobotBaseToWorld(TargetMm), success);

            if (success)
                ShowMessage($"Fahre zum Ziel (IK: {result.Iterations} Iterationen).", true);
            else
                ShowMessage($"Nicht gefahren – Ziel unerreichbar (Restfehler {result.PositionErrorMm:F1} mm).", false);
        }

        // Übernimmt die aktuelle TCP-Position in die Felder – praktisch als Ausgangspunkt für kleine Änderungen.
        void OnTakeCurrentClicked()
        {
            Vector3 tcp = kinematics.CurrentTcpMm();

            // WithoutNotify: sonst würde jedes der drei Felder einzeln eine Vorschau auslösen –
            // die ersten beiden sogar mit einer halb aktualisierten Position.
            xField.SetValueWithoutNotify(RoundToTenth(tcp.x));
            yField.SetValueWithoutNotify(RoundToTenth(tcp.y));
            zField.SetValueWithoutNotify(RoundToTenth(tcp.z));

            UpdatePreview();
        }

        void ShowMessage(string text, bool ok)
        {
            messageLabel.text = text;
            // Farbe über USS-Klassen statt im Code: Aussehen bleibt im Stylesheet.
            messageLabel.EnableInClassList("message--ok", ok);
            messageLabel.EnableInClassList("message--error", !ok);
        }

        static float RoundToTenth(float value) => Mathf.Round(value * 10f) / 10f;
    }
}
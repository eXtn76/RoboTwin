using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using RoboTwin.Connection;
using RoboTwin.Model;
using RoboTwin.Programs;
using UnityEngine;
using UnityEngine.UIElements;

namespace RoboTwin.UI
{
    /// <summary>
    /// Programm-Panel: Positionen teachen, Punkte bearbeiten, Programm abspielen, als JSON speichern/laden.
    /// Die Ablauflogik steckt im ProgramRunner; dieses Panel zeigt nur an und leitet Eingaben weiter.
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class ProgramPanel : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] RobotConnectionHost host;
        [SerializeField] ProgramRunner runner;
        [SerializeField] VisualTreeAsset pointRowTemplate;

        TextField nameField;
        ScrollView pointList;
        Button teachButton;
        Button newButton;
        DropdownField fileSelect;
        Button loadButton;
        Button saveButton;
        Button playButton;
        Button pauseButton;
        Button stopButton;
        Toggle loopToggle;
        Label messageLabel;

        readonly List<VisualElement> rows = new List<VisualElement>();
        readonly List<Button> gotoButtons = new List<Button>();

        RobotProgram Program => runner.Program;
        bool Editable => runner.State == RunnerState.Stopped;   // während des Abspielens nichts verändern

        /// <summary>
        /// Im Editor: Ordner "Programs" im Repo (neben RoboTwin-Unity) – Programme kommen mit ins Git.
        /// In einem Build gibt es das Repo nicht, dann der Datenordner der Anwendung.
        /// </summary>
        static string ProgramsFolder
        {
            get
            {
#if UNITY_EDITOR
                return Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "Programs"));
#else
                return Path.Combine(Application.persistentDataPath, "Programs");
#endif
            }
        }

        void Start()
        {
            VisualElement root = GetComponent<UIDocument>().rootVisualElement;
            nameField = root.Q<TextField>("program-name");
            pointList = root.Q<ScrollView>("point-list");
            teachButton = root.Q<Button>("teach-button");
            newButton = root.Q<Button>("new-button");
            fileSelect = root.Q<DropdownField>("file-select");
            loadButton = root.Q<Button>("load-button");
            saveButton = root.Q<Button>("save-button");
            playButton = root.Q<Button>("play-button");
            pauseButton = root.Q<Button>("pause-button");
            stopButton = root.Q<Button>("program-stop-button");
            loopToggle = root.Q<Toggle>("loop-toggle");
            messageLabel = root.Q<Label>("program-message");

            nameField.RegisterValueChangedCallback(evt => Program.Name = evt.newValue);
            teachButton.clicked += OnTeachClicked;
            newButton.clicked += OnNewClicked;
            loadButton.clicked += OnLoadClicked;
            saveButton.clicked += OnSaveClicked;
            playButton.clicked += runner.Play;
            pauseButton.clicked += runner.Pause;
            stopButton.clicked += runner.Stop;
            loopToggle.RegisterValueChangedCallback(evt => runner.Loop = evt.newValue);

            runner.Changed += OnRunnerChanged;
            host.ControlChanged += UpdateControls;

            ShowProgram();
            RefreshFileList();
            messageLabel.text = "";
        }

        void OnDestroy()
        {
            if (runner != null)
                runner.Changed -= OnRunnerChanged;
            if (host != null)
                host.ControlChanged -= UpdateControls;
        }

        // ---- Programm anzeigen ----

        void ShowProgram()
        {
            nameField.SetValueWithoutNotify(Program.Name);
            RebuildRows();
        }

        // Bei jeder Änderung der Liste alle Zeilen neu bauen – bei wenigen Punkten einfacher als Einzelupdates.
        void RebuildRows()
        {
            pointList.Clear();
            rows.Clear();
            gotoButtons.Clear();

            for (int i = 0; i < Program.Points.Count; i++)
            {
                ProgramPoint point = Program.Points[i];
                int index = i;   // Kopie für die Lambdas (Lambda-Falle in Schleifen!)

                VisualElement row = pointRowTemplate.Instantiate();
                pointList.Add(row);
                rows.Add(row);

                row.Q<Label>("point-index").text = (i + 1).ToString();

                var nameInput = row.Q<TextField>("point-name");
                nameInput.SetValueWithoutNotify(point.Name);
                nameInput.RegisterValueChangedCallback(evt => point.Name = evt.newValue);

                // In der UI Prozent (1..100), in Datei und Protokoll Anteil (0..1].
                var speedInput = row.Q<IntegerField>("point-speed");
                speedInput.SetValueWithoutNotify(Mathf.RoundToInt(point.Speed * 100f));
                speedInput.RegisterValueChangedCallback(evt =>
                {
                    int percent = Mathf.Clamp(evt.newValue, 1, 100);
                    point.Speed = percent / 100f;
                    speedInput.SetValueWithoutNotify(percent);
                });

                var waitInput = row.Q<FloatField>("point-wait");
                waitInput.SetValueWithoutNotify(point.Wait);
                waitInput.RegisterValueChangedCallback(evt =>
                {
                    point.Wait = Mathf.Max(0f, evt.newValue);
                    waitInput.SetValueWithoutNotify(point.Wait);
                });

                Button gotoButton = row.Q<Button>("point-goto");
                gotoButton.clicked += () => host.SendMove(point.Joints, point.Speed);   // mit Kollisionsprüfung
                gotoButtons.Add(gotoButton);

                row.Q<Button>("point-up").clicked += () => MovePoint(index, -1);
                row.Q<Button>("point-down").clicked += () => MovePoint(index, +1);
                row.Q<Button>("point-delete").clicked += () => DeletePoint(index);
            }

            UpdateControls();
            HighlightCurrentPoint();
        }

        void HighlightCurrentPoint()
        {
            for (int i = 0; i < rows.Count; i++)
                rows[i].Q(className: "program-row").EnableInClassList("program-row--active", i == runner.CurrentIndex);
        }

        // ---- Bearbeiten ----

        // Teach-Pendant: die AKTUELLE Armhaltung (Ist-Zustand) als neuen Punkt anhängen.
        void OnTeachClicked()
        {
            float[] joints = model.ActualAngles.ToArray();
            Program.Points.Add(new ProgramPoint($"P{Program.Points.Count + 1}", joints));
            RebuildRows();
            pointList.ScrollTo(rows[rows.Count - 1]);
            ShowMessage($"Punkt {Program.Points.Count} gespeichert.", true);
        }

        void OnNewClicked()
        {
            runner.Program = new RobotProgram();
            ShowProgram();
            ShowMessage("Neues, leeres Programm.", true);
        }

        void MovePoint(int index, int direction)
        {
            int other = index + direction;
            if (!Editable || other < 0 || other >= Program.Points.Count)
                return;
            (Program.Points[index], Program.Points[other]) = (Program.Points[other], Program.Points[index]);
            RebuildRows();
        }

        void DeletePoint(int index)
        {
            if (!Editable)
                return;
            Program.Points.RemoveAt(index);
            RebuildRows();
        }

        // ---- Dateien ----

        void RefreshFileList()
        {
            List<string> files = Directory.Exists(ProgramsFolder)
                ? Directory.GetFiles(ProgramsFolder, "*" + ProgramFile.Extension)
                    .Select(Path.GetFileName)
                    .OrderBy(name => name)
                    .ToList()
                : new List<string>();

            fileSelect.choices = files;
            if (!files.Contains(fileSelect.value))
                fileSelect.SetValueWithoutNotify(files.Count > 0 ? files[0] : "");
        }

        void OnLoadClicked()
        {
            if (string.IsNullOrEmpty(fileSelect.value))
                return;

            string path = Path.Combine(ProgramsFolder, fileSelect.value);
            try
            {
                runner.Program = ProgramFile.Load(path, model.JointCount);
                ShowProgram();
                ShowMessage($"Geladen: {fileSelect.value} ({Program.Points.Count} Punkte).", true);
            }
            catch (Exception error) when (error is ProgramFormatException || error is IOException)
            {
                ShowMessage($"Laden fehlgeschlagen: {error.Message}", false);
            }
        }

        void OnSaveClicked()
        {
            string fileName = ToFileName(Program.Name) + ProgramFile.Extension;
            try
            {
                Directory.CreateDirectory(ProgramsFolder);
                ProgramFile.Save(Program, Path.Combine(ProgramsFolder, fileName));
                RefreshFileList();
                fileSelect.SetValueWithoutNotify(fileName);
                ShowMessage($"Gespeichert: {fileName}", true);
            }
            catch (IOException error)
            {
                ShowMessage($"Speichern fehlgeschlagen: {error.Message}", false);
            }
        }

        // "Pick & Place 2" → "pick_place_2": nur Buchstaben, Ziffern und '_' – sicher auf jedem Betriebssystem.
        static string ToFileName(string programName)
        {
            var chars = programName.Trim().ToLowerInvariant()
                .Select(c => char.IsLetterOrDigit(c) ? c : '_')
                .ToArray();
            string name = string.Join("_", new string(chars).Split(new[] { '_' }, StringSplitOptions.RemoveEmptyEntries));
            return string.IsNullOrEmpty(name) ? "programm" : name;
        }

        // ---- Anzeige ----

        void OnRunnerChanged()
        {
            bool failed = runner.Message.StartsWith("Abgebrochen") || runner.Message.StartsWith("Start nicht");
            ShowMessage(runner.Message, !failed);
            HighlightCurrentPoint();
            UpdateControls();
        }

        void UpdateControls()
        {
            bool editable = Editable;
            bool hasPoints = Program.Points.Count > 0;

            nameField.SetEnabled(editable);
            teachButton.SetEnabled(editable);
            newButton.SetEnabled(editable);
            loadButton.SetEnabled(editable);
            saveButton.SetEnabled(hasPoints);
            pointList.SetEnabled(editable);
            foreach (Button gotoButton in gotoButtons)
                gotoButton.SetEnabled(editable && host.CanMove);

            playButton.SetEnabled(host.CanMove && hasPoints && runner.State != RunnerState.Running);
            playButton.text = runner.State == RunnerState.Paused ? "Fortsetzen" : "Start";
            pauseButton.SetEnabled(runner.State == RunnerState.Running);
            stopButton.SetEnabled(runner.State != RunnerState.Stopped);
        }

        void ShowMessage(string text, bool ok)
        {
            messageLabel.text = text;
            messageLabel.EnableInClassList("message--ok", ok);
            messageLabel.EnableInClassList("message--error", !ok);
        }
    }
}

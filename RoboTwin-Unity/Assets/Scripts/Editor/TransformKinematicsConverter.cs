using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using RoboTwin.Model;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnityEditor;
using UnityEngine;

namespace RoboTwin.EditorTools
{
    /// <summary>
    /// Wandelt einen vom URDF Importer erzeugten Roboter (ArticulationBody-Physik)
    /// in Transform-Kinematik um: Physik-Komponenten raus, JointDriver an jedes Drehgelenk.
    /// Grenzen und Geschwindigkeiten kommen direkt aus der URDF-Datei.
    /// Aufruf: Roboter-Wurzel in der Hierarchy auswählen, Menü RoboTwin.
    /// </summary>
    public static class TransformKinematicsConverter
    {
        const string MenuPath = "RoboTwin/Convert to Transform Kinematics";

        // Validierungsfunktion: Der Menüpunkt ist nur aktiv, wenn sie true liefert.
        [MenuItem(MenuPath, true)]
        static bool CanConvert()
        {
            GameObject selected = Selection.activeGameObject;
            return selected != null && selected.GetComponent<UrdfRobot>() != null;
        }

        [MenuItem(MenuPath)]
        static void Convert()
        {
            GameObject robot = Selection.activeGameObject;

            // Schutz: Die Nullstellung wird aus der aktuellen Lage übernommen.
            // Gibt es schon JointDriver, steht der Arm evtl. nicht mehr auf 0°.
            if (robot.GetComponentInChildren<JointDriver>(true) != null)
            {
                EditorUtility.DisplayDialog("RoboTwin",
                    "Der Roboter hat bereits JointDriver-Komponenten.\n" +
                    "Alle Winkel auf 0 stellen, JointDriver entfernen, dann erneut ausführen.", "OK");
                return;
            }

            // 1. URDF-Datei finden und die Drehgelenke daraus lesen.
            //    (Nicht aus dem ArticulationBody: maxAngularVelocity wird dort nicht
            //    in der Szene gespeichert und steht nach dem Neuladen auf 50 rad/s.)
            string urdfPath = FindUrdf(robot.name);
            if (urdfPath == null)
                urdfPath = EditorUtility.OpenFilePanel("URDF-Datei wählen", "Assets", "urdf");
            if (string.IsNullOrEmpty(urdfPath))
                return;

            Dictionary<string, (float min, float max, float maxSpeed)> urdfJoints = ReadRevoluteJoints(urdfPath);

            // GameObjects den URDF-Gelenken zuordnen, solange die Importer-Komponenten
            // (mit jointName) noch existieren.
            var revoluteJoints = new List<(GameObject link, string name, float min, float max, float maxSpeed)>();
            foreach (UrdfJoint joint in robot.GetComponentsInChildren<UrdfJoint>(true))
            {
                if (urdfJoints.TryGetValue(joint.jointName, out var data))
                    revoluteJoints.Add((joint.gameObject, joint.jointName, data.min, data.max, data.maxSpeed));
            }

            // Alle folgenden Änderungen bilden einen einzigen Undo-Schritt (Strg+Z).
            Undo.IncrementCurrentGroup();
            Undo.SetCurrentGroupName("Convert to Transform Kinematics");
            int undoGroup = Undo.GetCurrentGroup();

            // 2. Physik entfernen. Reihenfolge ist wichtig:
            //    - tiefste Glieder zuerst (Kinder vor Eltern),
            //    - pro Glied erst UrdfJoint und UrdfInertial, denn beide verlangen per
            //      [RequireComponent] einen ArticulationBody – Unity verweigert sonst das Entfernen.
            IEnumerable<ArticulationBody> bodies = robot
                .GetComponentsInChildren<ArticulationBody>(true)
                .OrderByDescending(b => Depth(b.transform));

            foreach (ArticulationBody body in bodies)
            {
                DestroyAll<UrdfJoint>(body.gameObject);
                DestroyAll<UrdfInertial>(body.gameObject);
                Undo.DestroyObjectImmediate(body);
            }

            // Der Beispiel-Controller des Importers steuert ArticulationBodies – ohne sie wertlos.
            DestroyAll<Controller>(robot);

            // 3. JointDriver an jedes Drehgelenk; Nullstellung = aktuelle Lage.
            foreach (var j in revoluteJoints)
            {
                JointDriver driver = Undo.AddComponent<JointDriver>(j.link);
                Undo.RecordObject(driver, "Initialize JointDriver");
                driver.Initialize(j.name, j.min, j.max, j.maxSpeed);
            }

            Undo.CollapseUndoOperations(undoGroup);
            Debug.Log($"RoboTwin: {revoluteJoints.Count} Drehgelenke auf Transform-Kinematik umgestellt.");
        }

        // Sucht im Projekt "<robotName>.urdf" (der Importer benennt die Wurzel nach dem Roboter).
        static string FindUrdf(string robotName)
        {
            foreach (string guid in AssetDatabase.FindAssets(robotName))
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                if (Path.GetFileName(path) == robotName + ".urdf")
                    return path;
            }
            return null;
        }

        // Liest alle <joint type="revolute"> mit ihren <limit>-Werten, umgerechnet in Grad.
        static Dictionary<string, (float min, float max, float maxSpeed)> ReadRevoluteJoints(string urdfPath)
        {
            var result = new Dictionary<string, (float, float, float)>();
            XDocument doc = XDocument.Load(urdfPath);

            foreach (XElement joint in doc.Root.Elements("joint"))
            {
                if ((string)joint.Attribute("type") != "revolute")
                    continue;

                XElement limit = joint.Element("limit");
                if (limit == null)
                    continue;

                result[(string)joint.Attribute("name")] = (
                    RadiansToDegrees(limit, "lower"),
                    RadiansToDegrees(limit, "upper"),
                    RadiansToDegrees(limit, "velocity"));
            }
            return result;
        }

        // InvariantCulture: Die URDF schreibt "3.14" mit Punkt. Auf einem deutschen
        // Windows würde double.Parse ohne sie ein Komma erwarten und falsch lesen.
        static float RadiansToDegrees(XElement element, string attribute)
        {
            double radians = double.Parse((string)element.Attribute(attribute), CultureInfo.InvariantCulture);
            return (float)(radians * Mathf.Rad2Deg);
        }

        static void DestroyAll<T>(GameObject go) where T : Component
        {
            foreach (T component in go.GetComponents<T>())
                Undo.DestroyObjectImmediate(component);
        }

        // Anzahl der Eltern bis zur Wurzel = Tiefe in der Hierarchie.
        static int Depth(Transform t)
        {
            int depth = 0;
            while (t.parent != null)
            {
                t = t.parent;
                depth++;
            }
            return depth;
        }
    }
}

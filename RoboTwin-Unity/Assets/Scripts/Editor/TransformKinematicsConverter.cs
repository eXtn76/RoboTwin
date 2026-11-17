using System.Collections.Generic;
using System.Linq;
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

            // 1. Gelenkdaten sichern, solange die Importer-Komponenten noch existieren.
            //    Der Importer speichert die URDF-Grenzen im xDrive des ArticulationBody (in Grad)
            //    und die Maximalgeschwindigkeit in maxAngularVelocity (in rad/s).
            var revoluteJoints = new List<(GameObject link, string name, float min, float max, float maxSpeed)>();
            foreach (UrdfJoint joint in robot.GetComponentsInChildren<UrdfJoint>(true))
            {
                if (joint.JointType != UrdfJoint.JointTypes.Revolute)
                    continue;

                ArticulationBody body = joint.GetComponent<ArticulationBody>();
                revoluteJoints.Add((joint.gameObject, joint.jointName,
                    body.xDrive.lowerLimit, body.xDrive.upperLimit,
                    body.maxAngularVelocity * Mathf.Rad2Deg));
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

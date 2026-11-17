using RoboTwin.Model;
using UnityEngine;

namespace RoboTwin.Kinematics
{
    /// <summary>
    /// Stellt die Kinematik des Roboters in der Szene bereit (an der Roboter-Wurzel).
    /// Selbsttest: vergleicht jedes Frame die berechnete TCP-Position mit dem echten tool0.
    /// </summary>
    public class RobotKinematics : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] Transform tool;            // URDF-Link "tool0"
        [SerializeField] Transform robotBaseFrame;  // URDF-Link "base"

        KinematicChain chain;
        bool mismatchReported;

        public KinematicChain Chain => chain;

        // Beim Hinzufügen im Editor: Felder automatisch füllen.
        void Reset()
        {
            model = GetComponent<RobotModel>();
            tool = FindChild(transform, "tool0");
            robotBaseFrame = FindChild(transform, "base");
        }

        // Awake reicht: Die Kette braucht nur Transforms und die gespeicherten Nullstellungen,
        // nicht den Zustand anderer Komponenten.
        void Awake() => chain = KinematicChain.FromScene(transform, model, tool, robotBaseFrame);

        /// <summary>
        /// Aktuelle TCP-Position (aus den Ist-Winkeln) im Roboter-Basis-System, mm.
        /// Wird bei jedem Aufruf frisch berechnet – so ist sie nie einen Frame veraltet,
        /// egal in welcher Reihenfolge Unity die Update()-Methoden aufruft.
        /// </summary>
        public Vector3 CurrentTcpMm() =>
            chain.ToRobotBase(chain.ForwardKinematics(model.ActualAngles).Position);

        /// <summary>Punkt im Roboter-Basis-System (mm) → Weltkoordinaten, z. B. für den Zielmarker.</summary>
        public Vector3 RobotBaseToWorld(Vector3 robotPointMm) =>
            transform.TransformPoint(chain.FromRobotBase(robotPointMm));

        void Update()
        {
            // Selbsttest: FK-Ergebnis in Weltkoordinaten vs. echte Position von tool0.
            Vector3 fkWorld = transform.TransformPoint(chain.ForwardKinematics(model.ActualAngles).Position);
            float errorMm = Vector3.Distance(fkWorld, tool.position) * 1000f;

            if (errorMm > 0.01f && !mismatchReported)
            {
                Debug.LogWarning($"RobotKinematics: FK weicht um {errorMm:F3} mm von tool0 ab.");
                mismatchReported = true;   // nur einmal melden, nicht jedes Frame
            }
        }

        // Sucht rekursiv (in der ganzen Unterhierarchie) ein Kind-Objekt mit diesem Namen.
        static Transform FindChild(Transform parent, string childName)
        {
            foreach (Transform child in parent)
            {
                if (child.name == childName)
                    return child;

                Transform found = FindChild(child, childName);
                if (found != null)
                    return found;
            }
            return null;
        }
    }
}
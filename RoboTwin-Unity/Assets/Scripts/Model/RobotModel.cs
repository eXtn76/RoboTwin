using System;
using System.Collections.Generic;
using UnityEngine;

namespace RoboTwin.Model
{
    /// <summary>
    /// Ist-Zustand des Roboters. Nimmt Gelenkwinkel entgegen (egal woher),
    /// zeigt sie über die JointDriver an und meldet Änderungen per Event.
    /// Rechnet selbst KEINE Bewegung.
    /// </summary>
    public class RobotModel : MonoBehaviour
    {
        [SerializeField] JointDriver[] joints;   // Reihenfolge: Basis → Handgelenk 3

        float[] actualAngles;

        /// <summary>Wird ausgelöst, nachdem ein neuer Ist-Zustand übernommen wurde.</summary>
        public event Action StateChanged;

        public int JointCount => joints.Length;
        public JointDriver GetJoint(int index) => joints[index];

        // IReadOnlyList: Andere dürfen lesen, aber nicht hineinschreiben.
        public IReadOnlyList<float> ActualAngles => actualAngles;

        // Beim Hinzufügen im Editor: alle JointDriver darunter einsammeln.
        // Die Hierarchie ist eine Kette, daher stimmt die Reihenfolge automatisch.
        void Reset() => joints = GetComponentsInChildren<JointDriver>();

        void Awake()
        {
            actualAngles = new float[joints.Length];
            for (int i = 0; i < joints.Length; i++)
                actualAngles[i] = joints[i].Angle;
        }

        /// <summary>
        /// Übernimmt einen neuen Ist-Zustand (Winkel in Grad, URDF-Konvention).
        /// Speichert die Rohwerte – auch wenn der JointDriver die Anzeige auf die
        /// Achsgrenzen begrenzt. Das Modell soll die Meldung des Controllers
        /// unverfälscht widerspiegeln.
        /// </summary>
        public void ApplyState(float[] angles)
        {
            if (angles.Length != joints.Length)
                throw new ArgumentException(
                    $"Erwartet {joints.Length} Winkel, erhalten {angles.Length}.", nameof(angles));

            for (int i = 0; i < joints.Length; i++)
            {
                actualAngles[i] = angles[i];
                joints[i].Angle = angles[i];
            }

            // Erst nach der Schleife: Abonnenten sehen nie einen halb aktualisierten Zustand.
            StateChanged?.Invoke();
        }
    }
}
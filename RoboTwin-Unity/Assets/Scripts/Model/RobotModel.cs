using System;
using System.Collections.Generic;
using RoboTwin.Connection;
using UnityEngine;

namespace RoboTwin.Model
{
    /// <summary>
    /// Ist-Zustand des Roboters. Nimmt Zustandsmeldungen entgegen (egal von welcher Verbindung),
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

        // Weitere Teile der letzten Zustandsmeldung.
        public RobotStatus Status { get; private set; } = RobotStatus.Idle;
        public float OverridePercent { get; private set; } = 100f;
        public double Timestamp { get; private set; }
        public string ErrorMessage { get; private set; }   // Grund des Fehlerzustands, sonst null
        public bool IsMoving => Status == RobotStatus.Moving;

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
        /// Übernimmt eine Zustandsmeldung des Controllers (Winkel in Grad, URDF-Konvention).
        /// Speichert die Rohwerte – auch wenn der JointDriver die Anzeige auf die
        /// Achsgrenzen begrenzt. Das Modell soll die Meldung des Controllers
        /// unverfälscht widerspiegeln.
        /// </summary>
        public void ApplyState(RobotState state)
        {
            float[] angles = state.Joints;
            if (angles == null || angles.Length != joints.Length)
                throw new ArgumentException(
                    $"Erwartet {joints.Length} Winkel, erhalten {angles?.Length ?? 0}.", nameof(state));

            for (int i = 0; i < joints.Length; i++)
            {
                actualAngles[i] = angles[i];
                joints[i].Angle = angles[i];
            }

            Status = state.Status;
            OverridePercent = state.OverridePercent;
            Timestamp = state.Timestamp;
            ErrorMessage = state.ErrorMessage;

            // Erst nach der Schleife: Abonnenten sehen nie einen halb aktualisierten Zustand.
            StateChanged?.Invoke();
        }
    }
}
using UnityEngine;

namespace RoboTwin.Model
{
    /// <summary>
    /// Dreht ein einzelnes Robotergelenk um seine Achse (Transform-Kinematik).
    /// Winkel in Grad, Vorzeichen nach URDF-Konvention (rechtshändig).
    /// </summary>
    [ExecuteAlways]
    public class JointDriver : MonoBehaviour
    {
        [SerializeField] string jointName = "";
        [SerializeField] float minAngle = -360f;
        [SerializeField] float maxAngle = 360f;
        [SerializeField] float maxSpeed = 180f;   // Grad pro Sekunde, wird ab M2 genutzt
        [SerializeField, Range(-360f, 360f)] float angle;

        // Grundrotation des Gelenks bei 0°, wird gespeichert (SerializeField),
        // aber im Inspector versteckt.
        [SerializeField, HideInInspector] Quaternion zeroRotation = Quaternion.identity;
        [SerializeField, HideInInspector] bool hasZero;

        public string JointName => jointName;
        public float MinAngle => minAngle;
        public float MaxAngle => maxAngle;
        public float MaxSpeed => maxSpeed;
        public Quaternion ZeroRotation => zeroRotation;

        public float Angle
        {
            get => angle;
            set
            {
                angle = Mathf.Clamp(value, minAngle, maxAngle);
                Apply();
            }
        }

        /// <summary>
        /// Wird vom Editor-Konverter aufgerufen. Übernimmt die URDF-Daten
        /// und merkt sich die aktuelle Lage als Nullstellung.
        /// </summary>
        public void Initialize(string name, float min, float max, float speed)
        {
            jointName = name;
            minAngle = min;
            maxAngle = max;
            maxSpeed = speed;
            CaptureZero();
            angle = 0f;
        }

        // Unity ruft Reset() auf, wenn die Komponente im Editor hinzugefügt wird.
        void Reset() => CaptureZero();

        void OnValidate()
        {
            if (maxAngle < minAngle) maxAngle = minAngle;
            angle = Mathf.Clamp(angle, minAngle, maxAngle);
        }

        // Dank [ExecuteAlways] auch im Edit-Modus.
        void Update() => Apply();

        void CaptureZero()
        {
            zeroRotation = transform.localRotation;
            hasZero = true;
        }

        void Apply()
        {
            if (!hasZero) return;
            // Erst Grundrotation, dann Drehung um die eigene Gelenkachse.
            // Minus: Umrechnung URDF (rechtshändig) -> Unity (linkshändig).
            transform.localRotation = zeroRotation * Quaternion.AngleAxis(-angle, Vector3.up);
        }
    }
}
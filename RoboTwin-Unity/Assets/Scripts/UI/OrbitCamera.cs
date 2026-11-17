using UnityEngine;
using UnityEngine.InputSystem;

namespace RoboTwin.UI
{
    /// <summary>
    /// Kamera, die auf einer Kugel um einen Zielpunkt kreist.
    /// Rechte Maustaste ziehen = drehen, Mausrad = zoomen.
    /// </summary>
    public class OrbitCamera : MonoBehaviour
    {
        [SerializeField] Transform target;

        [Header("Startwerte")]
        [SerializeField] float yaw = 45f;        // Grad um die Hochachse
        [SerializeField] float pitch = 30f;      // Grad nach oben
        [SerializeField] float distance = 2.5f;  // Meter

        [Header("Grenzen")]
        [SerializeField] float minPitch = 5f;     // nicht unter den Boden
        [SerializeField] float maxPitch = 85f;    // nicht senkrecht drüber (Bild kippt)
        [SerializeField] float minDistance = 0.5f;
        [SerializeField] float maxDistance = 6f;

        [Header("Empfindlichkeit")]
        [SerializeField] float rotateSpeed = 0.2f;  // Grad pro Pixel
        [SerializeField] float zoomStep = 0.1f;     // 10 % pro Rasterklick

        // LateUpdate: läuft nach allen Update()-Aufrufen, also nachdem sich der Roboter bewegt hat.
        void LateUpdate()
        {
            Mouse mouse = Mouse.current;
            if (target == null || mouse == null)
                return;

            // Drehen: delta ist die Strecke in Pixeln seit dem letzten Frame,
            // deshalb kein Time.deltaTime.
            if (mouse.rightButton.isPressed)
            {
                Vector2 delta = mouse.delta.ReadValue();
                yaw += delta.x * rotateSpeed;
                pitch -= delta.y * rotateSpeed;   // Maus hoch = Kamera runter, wie die Szene greifen und kippen
                pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
            }

            float scroll = mouse.scroll.ReadValue().y;
            if (scroll != 0f)
            {
                distance *= 1f - Mathf.Sign(scroll) * zoomStep;
                distance = Mathf.Clamp(distance, minDistance, maxDistance);
            }

            Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
            transform.position = target.position + rotation * new Vector3(0f, 0f, -distance);
            transform.rotation = rotation;
        }
    }
}
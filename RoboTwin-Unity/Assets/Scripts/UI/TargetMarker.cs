using UnityEngine;

namespace RoboTwin.UI
{
    /// <summary>
    /// Zeigt das kartesische Ziel in der Szene: grün = erreichbar, rot = unerreichbar.
    /// An ein einfaches 3D-Objekt (z. B. eine kleine Kugel OHNE Collider) hängen.
    /// </summary>
    [RequireComponent(typeof(Renderer))]
    public class TargetMarker : MonoBehaviour
    {
        [SerializeField] Color reachableColor = new Color(0.30f, 0.85f, 0.40f);
        [SerializeField] Color unreachableColor = new Color(0.95f, 0.30f, 0.25f);

        Renderer markerRenderer;

        // Unsichtbar starten; das GameObject bleibt aktiv, nur der Renderer wird geschaltet.
        // (Ein deaktiviertes GameObject würde kein Awake bekommen.)
        void Awake()
        {
            markerRenderer = GetComponent<Renderer>();
            markerRenderer.enabled = false;
        }

        public void Show(Vector3 worldPosition, bool reachable)
        {
            transform.position = worldPosition;
            // .material legt beim ersten Zugriff eine eigene Kopie an – das Asset bleibt unverändert.
            markerRenderer.material.color = reachable ? reachableColor : unreachableColor;
            markerRenderer.enabled = true;
        }

        public void Hide() => markerRenderer.enabled = false;
    }
}

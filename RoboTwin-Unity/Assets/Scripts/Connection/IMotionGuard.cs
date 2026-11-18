using System.Collections.Generic;

namespace RoboTwin.Connection
{
    /// <summary>
    /// Prüft eine Bewegung, BEVOR sie gesendet wird (z. B. Kollisionsprüfung).
    /// Der Host kennt nur dieses Interface – nicht die Kollisionsrechnung dahinter. Dieselbe Idee wie
    /// IRobotConnection: Die Verbindungsschicht hängt nicht von der Sicherheitsschicht ab.
    /// </summary>
    public interface IMotionGuard
    {
        /// <summary>true = Bahn von 'from' nach 'to' ist frei; sonst false und 'reason' beschreibt das Problem.</summary>
        bool CheckMove(IReadOnlyList<float> from, IReadOnlyList<float> to, out string reason);

        /// <summary>
        /// Engt Achsbereiche auf die Sicherheitsgrenzen ein (z. B. für IK und Slider), damit gar nicht erst
        /// Ziele entstehen, die CheckMove ablehnen würde. Die Arrays werden direkt verändert.
        /// </summary>
        void RestrictLimits(float[] minAngles, float[] maxAngles);
    }
}

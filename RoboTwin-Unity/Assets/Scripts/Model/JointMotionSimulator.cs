using UnityEngine;

namespace RoboTwin.Model
{
    /// <summary>
    /// Simuliert die Gelenkbewegung eines Roboters:
    /// synchron (alle Gelenke starten und kommen gleichzeitig an),
    /// Trapezprofil (beschleunigen – konstant – bremsen), Achsgrenzen.
    /// Reine C#-Klasse – die Zeit kommt von außen (Step(dt)), daher testbar.
    /// Winkel in Grad, Geschwindigkeiten in Grad/s, Beschleunigungen in Grad/s².
    /// </summary>
    public class JointMotionSimulator
    {
        const float MinDistance = 1e-4f;   // kürzere Wege (Grad) gelten als "schon da"

        // Grenzen pro Gelenk
        readonly float[] minAngles;
        readonly float[] maxAngles;
        readonly float[] maxSpeeds;
        readonly float[] maxAccelerations;

        // Zustand pro Gelenk
        readonly float[] current;    // Ist-Winkel
        readonly float[] velocity;   // Ist-Geschwindigkeit
        readonly float[] target;     // Soll-Winkel
        readonly float[] start;      // Startpunkt der aktuellen Bahn

        // Zustand der Bahn – EIN Wert für alle Gelenke
        float s = 1f;      // Fortschritt: 0 = Start, 1 = Ziel
        float sDot;        // Fortschrittsgeschwindigkeit (1/s)
        float sDotMax;     // erlaubte Fortschrittsgeschwindigkeit
        float sDDotMax;    // erlaubte Fortschrittsbeschleunigung

        float speedFactor = 1f;

        public JointMotionSimulator(float[] minAngles, float[] maxAngles,
                                    float[] maxSpeeds, float[] maxAccelerations,
                                    float[] startAngles)
        {
            // Kopien statt Referenzen: Ändert der Aufrufer später sein Array,
            // darf sich unser Zustand nicht heimlich mitändern.
            this.minAngles = (float[])minAngles.Clone();
            this.maxAngles = (float[])maxAngles.Clone();
            this.maxSpeeds = (float[])maxSpeeds.Clone();
            this.maxAccelerations = (float[])maxAccelerations.Clone();
            current = (float[])startAngles.Clone();
            target = (float[])startAngles.Clone();
            start = (float[])startAngles.Clone();
            velocity = new float[current.Length];
        }

        public int JointCount => current.Length;
        public float GetCurrent(int joint) => current[joint];
        public float GetVelocity(int joint) => velocity[joint];
        public float GetTarget(int joint) => target[joint];
        public bool IsMoving => s < 1f;

        /// <summary>Override 1 % … 100 % (0.01 … 1). Begrenzt nur die Geschwindigkeit.</summary>
        public float SpeedFactor
        {
            get => speedFactor;
            set
            {
                speedFactor = Mathf.Clamp(value, 0.01f, 1f);
                Replan();
            }
        }

        /// <summary>Neues Ziel für ein einzelnes Gelenk (z. B. ein Slider).</summary>
        public void SetTarget(int joint, float angle)
        {
            target[joint] = Mathf.Clamp(angle, minAngles[joint], maxAngles[joint]);
            Replan();
        }

        /// <summary>Neue Ziele für alle Gelenke auf einmal (entspricht "move_joints" im Protokoll).</summary>
        public void SetTargets(float[] angles)
        {
            // Falsche Anzahl ist ein Programmier- oder Protokollfehler – laut melden statt raten.
            if (angles.Length != current.Length)
                throw new System.ArgumentException(
                    $"Erwartet {current.Length} Winkel, erhalten {angles.Length}.", nameof(angles));

            for (int i = 0; i < angles.Length; i++)
                target[i] = Mathf.Clamp(angles[i], minAngles[i], maxAngles[i]);

            Replan();   // nur einmal: eine gemeinsame Bahn für alle Gelenke
        }

        /// <summary>Sofort anhalten: Ziel = aktuelle Position.</summary>
        public void Stop()
        {
            for (int i = 0; i < current.Length; i++)
                target[i] = current[i];

            Replan();   // kein Weg mehr → Replan setzt s = 1, sDot = 0, Geschwindigkeiten = 0
        }

        /// <summary>Simuliert dt Sekunden Bewegung.</summary>
        public void Step(float dt)
        {
            if (s >= 1f)
                return;

            float remaining = 1f - s;
            // Höchste Geschwindigkeit, bei der wir noch genau bei s = 1 zum Stehen kommen (v² = 2·a·d).
            float brakeSpeed = Mathf.Sqrt(2f * sDDotMax * remaining);

            if (sDot > brakeSpeed)
            {
                // 1. Zu schnell, um rechtzeitig zu stoppen → auf der Bremskurve bleiben.
                sDot = brakeSpeed;
            }
            else if (sDot < sDotMax)
            {
                // 2. Beschleunigen – aber nie über die Höchstgeschwindigkeit
                //    und nie über die Bremskurve hinaus.
                sDot = Mathf.Min(sDot + sDDotMax * dt, sDotMax, brakeSpeed);
            }
            else
            {
                // 3. Schneller als erlaubt (Override wurde gesenkt) → sanft herunterbremsen.
                sDot = Mathf.Max(sDot - sDDotMax * dt, sDotMax);
            }

            s += sDot * dt;
            if (s >= 1f - 1e-6f)   // angekommen (Toleranz gegen Rundungsfehler)
            {
                s = 1f;
                sDot = 0f;
            }
            UpdateJoints();
        }

        // Plant eine neue Bahn vom aktuellen Ist-Zustand zum Ziel.
        void Replan()
        {
            // 1. Die neue Bahn beginnt da, wo die Gelenke gerade stehen.
            //    Nebenbei: Skalarprodukt aus aktueller Geschwindigkeit und neuer Richtung (für Schritt 4).
            float dot = 0f;
            float lengthSq = 0f;
            for (int i = 0; i < current.Length; i++)
            {
                start[i] = current[i];
                float delta = target[i] - start[i];
                dot += velocity[i] * delta;
                lengthSq += delta * delta;
            }

            // 2. Gelenkgrenzen in Grenzen für s umrechnen.
            //    Gelenk i bewegt sich mit |Δi| · ṡ  →  ṡ ≤ vmax_i / |Δi|,  s̈ ≤ amax_i / |Δi|.
            //    Das muss für ALLE Gelenke gelten → jeweils das Minimum.
            sDotMax = float.MaxValue;
            sDDotMax = float.MaxValue;
            bool hasMotion = false;
            for (int i = 0; i < current.Length; i++)
            {
                float distance = Mathf.Abs(target[i] - start[i]);
                if (distance < MinDistance)
                    continue;   // dieses Gelenk bewegt sich nicht (und 0 im Nenner wäre fatal)
                hasMotion = true;

                sDotMax = Mathf.Min(sDotMax, maxSpeeds[i] * speedFactor / distance);
                sDDotMax = Mathf.Min(sDDotMax, maxAccelerations[i] / distance);
            }

            // 3. Kein Weg mehr: exakt aufs Ziel setzen und stehen bleiben.
            if (!hasMotion)
            {
                for (int i = 0; i < current.Length; i++)
                {
                    current[i] = target[i];
                    velocity[i] = 0f;
                }
                s = 1f;
                sDot = 0f;
                return;
            }

            // 4. Neue Bahn starten. Anfangsgeschwindigkeit = Anteil der aktuellen Bewegung,
            //    der in die neue Richtung zeigt (Projektion). Zeigt sie rückwärts: 0.
            s = 0f;
            sDot = Mathf.Max(0f, dot / lengthSq);
        }

        // Überträgt den Bahnzustand (s, sDot) auf die einzelnen Gelenke.
        void UpdateJoints()
        {
            for (int i = 0; i < current.Length; i++)
            {
                float delta = target[i] - start[i];
                current[i] = start[i] + delta * s;
                velocity[i] = delta * sDot;
            }
        }
    }
}
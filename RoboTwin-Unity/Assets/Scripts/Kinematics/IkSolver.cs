using System.Collections.Generic;
using UnityEngine;

namespace RoboTwin.Kinematics
{
    /// <summary>Ergebnis einer IK-Berechnung.</summary>
    public readonly struct IkResult
    {
        public readonly bool Success;
        public readonly float[] Angles;             // Grad, URDF-Konvention
        public readonly float PositionErrorMm;
        public readonly float OrientationErrorDeg;
        public readonly int Iterations;

        public IkResult(bool success, float[] angles, float positionErrorMm, float orientationErrorDeg, int iterations)
        {
            Success = success;
            Angles = angles;
            PositionErrorMm = positionErrorMm;
            OrientationErrorDeg = orientationErrorDeg;
            Iterations = iterations;
        }
    }

    /// <summary>
    /// Inverse Kinematik mit Damped Least Squares (Jacobi-Verfahren).
    /// Arbeitet relativ zur Roboter-Wurzel in Unity-Achsen und Metern; Winkel in Grad.
    /// Reine C#-Klasse, testbar.
    /// </summary>
    public class IkSolver
    {
        public int MaxIterations = 100;
        public float PositionTolerance = 0.0005f;                    // m (0,5 mm)
        public float OrientationTolerance = 0.1f * Mathf.Deg2Rad;    // rad (0,1°)
        public float Damping = 0.05f;                                // λ
        public float MaxStepDeg = 10f;                               // größte Gelenkänderung pro Iteration

        readonly KinematicChain chain;
        readonly float[] minAngles;
        readonly float[] maxAngles;

        // Arbeitsspeicher – einmal angelegt, in jeder Iteration wiederverwendet.
        readonly Vector3[] jointPositions;
        readonly Vector3[] jointAxes;
        readonly double[,] jacobian;            // 6 Zeilen × n Spalten
        readonly double[,] system;              // 6 × 6: J·Jᵀ + λ²·I
        readonly double[] error = new double[6];
        readonly double[] delta;                // Δθ pro Gelenk (rad)

        public IkSolver(KinematicChain chain, float[] minAngles, float[] maxAngles)
        {
            this.chain = chain;
            this.minAngles = (float[])minAngles.Clone();
            this.maxAngles = (float[])maxAngles.Clone();

            int n = chain.JointCount;
            jointPositions = new Vector3[n];
            jointAxes = new Vector3[n];
            jacobian = new double[6, n];
            system = new double[6, 6];
            delta = new double[n];
        }

        /// <summary>Sucht Gelenkwinkel, mit denen der TCP die Ziel-Lage (relativ zur Wurzel) erreicht.</summary>
        public IkResult Solve(IReadOnlyList<float> startAngles, Frame target)
        {
            int n = chain.JointCount;

            // Start bei den aktuellen Winkeln → meist die nächstgelegene Lösung, reproduzierbar.
            float[] angles = new float[n];
            for (int i = 0; i < n; i++)
                angles[i] = startAngles[i];

            for (int iteration = 0; iteration < MaxIterations; iteration++)
            {
                Frame tcp = chain.ForwardKinematics(angles, jointPositions, jointAxes);

                Vector3 positionError = target.Position - tcp.Position;
                Vector3 orientationError = OrientationError(target.Rotation, tcp.Rotation);

                if (positionError.magnitude < PositionTolerance && orientationError.magnitude < OrientationTolerance)
                    return Result(true, angles, positionError, orientationError, iteration);

                // 1. Jacobi-Matrix: Spalte i = Wirkung einer kleinen Drehung von Gelenk i auf den TCP.
                for (int i = 0; i < n; i++)
                {
                    Vector3 axis = jointAxes[i];
                    Vector3 linear = Vector3.Cross(axis, tcp.Position - jointPositions[i]);   // Achse × Hebelarm

                    jacobian[0, i] = linear.x;
                    jacobian[1, i] = linear.y;
                    jacobian[2, i] = linear.z;
                    jacobian[3, i] = axis.x;
                    jacobian[4, i] = axis.y;
                    jacobian[5, i] = axis.z;
                }

                // 2. Fehlervektor: erst Position (m), dann Orientierung (rad).
                error[0] = positionError.x;
                error[1] = positionError.y;
                error[2] = positionError.z;
                error[3] = orientationError.x;
                error[4] = orientationError.y;
                error[5] = orientationError.z;

                // 3. system = J·Jᵀ + λ²·I – die Dämpfung macht die Matrix immer lösbar.
                double lambdaSquared = Damping * Damping;
                for (int r = 0; r < 6; r++)
                {
                    for (int c = 0; c < 6; c++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < n; k++)
                            sum += jacobian[r, k] * jacobian[c, k];
                        if (r == c)
                            sum += lambdaSquared;
                        system[r, c] = sum;
                    }
                }

                double[] x = LinearSolver.Solve(system, error);

                // 4. Δθ = Jᵀ·x, größtes |Δθ| merken.
                double largest = 0.0;
                for (int i = 0; i < n; i++)
                {
                    double sum = 0.0;
                    for (int r = 0; r < 6; r++)
                        sum += jacobian[r, i] * x[r];
                    delta[i] = sum;
                    largest = System.Math.Max(largest, System.Math.Abs(sum));
                }

                // Schritt begrenzen: ALLE Gelenke mit demselben Faktor verkleinern,
                // damit die Richtung des Schritts erhalten bleibt.
                double maxStep = MaxStepDeg * Mathf.Deg2Rad;
                double scale = largest > maxStep ? maxStep / largest : 1.0;

                for (int i = 0; i < n; i++)
                {
                    float newAngle = angles[i] + (float)(delta[i] * scale) * Mathf.Rad2Deg;
                    angles[i] = Mathf.Clamp(newAngle, minAngles[i], maxAngles[i]);
                }
            }

            // Nicht konvergiert: Restfehler für die Warnung ausrechnen.
            Frame final = chain.ForwardKinematics(angles);
            return Result(false, angles, target.Position - final.Position,
                          OrientationError(target.Rotation, final.Rotation), MaxIterations);
        }

        // Drehung, die noch fehlt, als Achse × Winkel (rad).
        static Vector3 OrientationError(Quaternion target, Quaternion current)
        {
            Quaternion delta = target * Quaternion.Inverse(current);
            delta.ToAngleAxis(out float angleDeg, out Vector3 axis);

            if (angleDeg > 180f)          // kürzeren Weg nehmen (q und −q sind dieselbe Drehung)
                angleDeg -= 360f;
            if (Mathf.Abs(angleDeg) < 1e-6f || float.IsNaN(axis.x))
                return Vector3.zero;      // keine Drehung mehr nötig

            return axis * (angleDeg * Mathf.Deg2Rad);
        }

        static IkResult Result(bool success, float[] angles, Vector3 positionError, Vector3 orientationError, int iterations) =>
            new IkResult(success, angles, positionError.magnitude * 1000f,
                         orientationError.magnitude * Mathf.Rad2Deg, iterations);
    }
}
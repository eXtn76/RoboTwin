using System;

namespace RoboTwin.Kinematics
{
    /// <summary>Löst kleine lineare Gleichungssysteme A·x = b.</summary>
    public static class LinearSolver
    {
        /// <summary>
        /// Gauß-Elimination mit Spaltenpivotsuche. A muss quadratisch sein.
        /// A und b werden nicht verändert.
        /// </summary>
        public static double[] Solve(double[,] matrix, double[] rhs)
        {
            int n = rhs.Length;
            var a = (double[,])matrix.Clone();
            var b = (double[])rhs.Clone();

            // 1. Vorwärts: Dreiecksform herstellen (unterhalb der Diagonale Nullen).
            for (int col = 0; col < n; col++)
            {
                // Pivotsuche: Zeile mit dem betragsgrößten Wert in dieser Spalte nach oben holen.
                // Teilen durch sehr kleine Zahlen verstärkt Rundungsfehler – so vermeiden wir das.
                int pivot = col;
                for (int row = col + 1; row < n; row++)
                    if (Math.Abs(a[row, col]) > Math.Abs(a[pivot, col]))
                        pivot = row;

                if (Math.Abs(a[pivot, col]) < 1e-12)
                    throw new InvalidOperationException("Matrix ist (fast) singulär.");

                if (pivot != col)
                {
                    for (int k = 0; k < n; k++)
                        (a[col, k], a[pivot, k]) = (a[pivot, k], a[col, k]);
                    (b[col], b[pivot]) = (b[pivot], b[col]);
                }

                // Alle Zeilen darunter: passendes Vielfaches der Pivotzeile abziehen.
                for (int row = col + 1; row < n; row++)
                {
                    double factor = a[row, col] / a[col, col];
                    for (int k = col; k < n; k++)
                        a[row, k] -= factor * a[col, k];
                    b[row] -= factor * b[col];
                }
            }

            // 2. Rückwärts einsetzen: von der letzten Zeile nach oben.
            var x = new double[n];
            for (int row = n - 1; row >= 0; row--)
            {
                double sum = b[row];
                for (int k = row + 1; k < n; k++)
                    sum -= a[row, k] * x[k];
                x[row] = sum / a[row, row];
            }
            return x;
        }
    }
}
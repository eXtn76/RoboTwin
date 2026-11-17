using UnityEngine;

namespace RoboTwin.Kinematics
{
    /// <summary>
    /// Lage eines Koordinatensystems relativ zu einem anderen: Position + Rotation (ohne Skalierung).
    /// Unveränderlich (readonly struct) – jede Operation liefert einen neuen Frame.
    /// </summary>
    public readonly struct Frame
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public Frame(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        public static readonly Frame Identity = new Frame(Vector3.zero, Quaternion.identity);

        /// <summary>Verkettung: erst a, dann b im lokalen System von a.</summary>
        public static Frame operator *(Frame a, Frame b)
        {
            // b's Versatz liegt im System von a → erst mit a drehen, dann verschieben.
            return new Frame(a.Position + a.Rotation * b.Position, a.Rotation * b.Rotation);
        }

        /// <summary>Umkehrung: f * f.Inverse() == Identity.</summary>
        public Frame Inverse()
        {
            // Rückweg: erst die Verschiebung zurücknehmen, dann die Drehung.
            Quaternion inverseRotation = Quaternion.Inverse(Rotation);
            return new Frame(inverseRotation * -Position, inverseRotation);
        }

        /// <summary>Wandelt einen Punkt aus dem lokalen System dieses Frames ins übergeordnete um.</summary>
        public Vector3 TransformPoint(Vector3 localPoint) => Position + Rotation * localPoint;

        /// <summary>Lage von 'target' relativ zu 'reference' (beide Transforms in der Szene).</summary>
        public static Frame RelativeTo(Transform reference, Transform target) =>
            new Frame(reference.InverseTransformPoint(target.position),
                      Quaternion.Inverse(reference.rotation) * target.rotation);
    }
}
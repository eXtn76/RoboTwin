using UnityEngine;

namespace RoboTwin.Model
{
    /// <summary>
    /// Rechnet die Roboterbewegung lokal in Unity und gibt den Ist-Zustand an das RobotModel.
    /// Vorläufer der SimulatedConnection (M4).
    /// </summary>
    public class LocalRobotSimulation : MonoBehaviour
    {
        [SerializeField] RobotModel model;
        [SerializeField] float acceleration = 180f;                  // Grad/s², für alle Gelenke
        [SerializeField, Range(1f, 100f)] float overridePercent = 100f;

        [Header("Test ohne UI")]
        [SerializeField] float[] testTargets = new float[6];

        JointMotionSimulator simulator;
        float[] angles;                 // Puffer, einmal angelegt: kein Müll für den Garbage Collector pro Frame
        float appliedOverridePercent;   // zuletzt an den Simulator gegebener Override (in Prozent!)

        void Reset() => model = GetComponent<RobotModel>();

        // Start statt Awake: RobotModel hat in seinem Awake die Startwinkel schon gelesen.
        void Start()
        {
            int count = model.JointCount;
            float[] minAngles = new float[count];
            float[] maxAngles = new float[count];
            float[] maxSpeeds = new float[count];
            float[] maxAccelerations = new float[count];
            float[] startAngles = new float[count];

            for (int i = 0; i < count; i++)
            {
                JointDriver joint = model.GetJoint(i);
                minAngles[i] = joint.MinAngle;
                maxAngles[i] = joint.MaxAngle;
                maxSpeeds[i] = joint.MaxSpeed;
                maxAccelerations[i] = acceleration;
                startAngles[i] = model.ActualAngles[i];
            }

            simulator = new JointMotionSimulator(minAngles, maxAngles, maxSpeeds, maxAccelerations, startAngles);
            angles = new float[count];

            simulator.SpeedFactor = overridePercent / 100f;
            appliedOverridePercent = overridePercent;
        }

        void Update()
        {
            // Override nur bei Änderung weitergeben – jede Zuweisung löst ein Replan() aus.
            if (overridePercent != appliedOverridePercent)
            {
                simulator.SpeedFactor = overridePercent / 100f;
                appliedOverridePercent = overridePercent;
            }

            simulator.Step(Time.deltaTime);

            for (int i = 0; i < angles.Length; i++)
                angles[i] = simulator.GetCurrent(i);

            model.ApplyState(angles);
        }

        // Schnittstelle für die UI (nächster Schritt)
        public int JointCount => simulator.JointCount;
        public float GetTarget(int joint) => simulator.GetTarget(joint);
        public bool IsMoving => simulator != null && simulator.IsMoving;
        public void SetTarget(int joint, float angle) => simulator.SetTarget(joint, angle);
        public void SetTargets(float[] targets) => simulator.SetTargets(targets);
        public void Stop() => simulator.Stop();

        // Rechtsklick auf die Komponente (bzw. ⋮) → diese Einträge, nur im Play-Modus sinnvoll.
        [ContextMenu("Move to Test Targets")]
        void MoveToTestTargets() => SetTargets(testTargets);

        [ContextMenu("Stop")]
        void StopFromMenu() => Stop();
    }
}
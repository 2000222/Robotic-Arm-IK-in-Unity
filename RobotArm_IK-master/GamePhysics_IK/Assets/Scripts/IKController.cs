using UnityEngine;

// ReSharper disable once InconsistentNaming
public class IKController : MonoBehaviour {
    [SerializeField] private Transform _targetTransform;
    public ArmJoint[] Joints;
    public float[] Angles;

    private const float SamplingDistance = 5f;
    private const float LearningRate = 100f;
    private const float DistanceThreshold = 0.01f;


    private void Start() {
        float[] angles = new float[Joints.Length];
        
        for (int i = 0; i < Joints.Length; i++) {
            if (Joints[i]._rotationAxis == 'x') {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.x;
            }
            else if (Joints[i]._rotationAxis == 'y') {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.y;
            }
            else if (Joints[i]._rotationAxis == 'z') {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.z;
            }
        }
        Angles = angles;
    }

    private void Update() {
        InverseKinematics(_targetTransform.position, Angles);
    }

    public Vector3 ForwardKinematics(float[] angles) {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++) {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].RotationAxis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;

            prevPoint = nextPoint;
        }
        return prevPoint;
    }

    public float DistanceFromTarget(Vector3 target, float[] angles) {
        Vector3 point = ForwardKinematics(angles);
        return Vector3.Distance(point, target);
    }

    public float PartialGradient(Vector3 target, float[] angles, int i) {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];

        // Gradient : [F(x+SamplingDistance) - F(x)] / h
        float f_x = DistanceFromTarget(target, angles);

        angles[i] += SamplingDistance;
        float f_x_plus_d = DistanceFromTarget(target, angles);

        float gradient = (f_x_plus_d - f_x) / SamplingDistance;

        // Restores
        angles[i] = angle;

        return gradient;
    }

    public void InverseKinematics(Vector3 target, float[] angles) {
        if (DistanceFromTarget(target, angles) < DistanceThreshold)
            return;

        for (int i = Joints.Length - 1; i >= 0; i--) {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(target, angles, i);
            angles[i] -= LearningRate * gradient;

            // Early termination
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
                return;
            
            switch (Joints[i]._rotationAxis) {
                case 'x':
                    Joints[i].transform.localEulerAngles = new Vector3(angles[i], 0, 0);
                    break;
                case 'y':
                    Joints[i].transform.localEulerAngles = new Vector3(0, angles[i], 0);
                    break;
                case 'z':
                    Joints[i].transform.localEulerAngles = new Vector3(0, 0, angles[i]);
                    break;
            }
        }
    }
}
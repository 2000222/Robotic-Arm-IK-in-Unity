using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmJoint : MonoBehaviour {
    public Vector3 RotationAxis;
    public Vector3 StartOffset;
    private Transform _transform;
    public char _rotationAxis;

    private void Awake() {
        _transform = this.transform;
        StartOffset = _transform.localPosition;
    }
}
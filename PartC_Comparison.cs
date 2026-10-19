using UnityEngine;

public class PartC_Comparison : MonoBehaviour
{
    public Transform poseA;
    public Transform poseB;

    [Range(0f, 1f)]
    public float u = 0.5f;

    // Object using simple Lerp + Slerp
    public Transform simplePose;

    // Object using dual quaternion interpolation
    public Transform dualQuaternionPose;

    void Update()
    {
        if (poseA == null || poseB == null)
            return;

        // ====================================================
        // METHOD 1: SIMPLE POSITION LERP + ROTATION SLERP
        // ====================================================

        Vector3 simplePosition =
            Vector3.Lerp(
                poseA.position,
                poseB.position,
                u
            );

        Quaternion simpleRotation =
            Quaternion.Slerp(
                poseA.rotation,
                poseB.rotation,
                u
            );

        simplePose.position = simplePosition;
        simplePose.rotation = simpleRotation;

        // ====================================================
        // METHOD 2: DUAL-QUATERNION INTERPOLATION
        // ====================================================

        DualQuaternion dqA =
            DualQuaternion.FromPose(
                poseA.rotation,
                poseA.position
            );

        DualQuaternion dqB =
            DualQuaternion.FromPose(
                poseB.rotation,
                poseB.position
            );

        DualQuaternion dqResult =
            DualQuaternion.Slerp(
                dqA,
                dqB,
                u
            );

        dqResult.ToPose(
            out Quaternion dualRotation,
            out Vector3 dualPosition
        );

        dualQuaternionPose.position = dualPosition;
        dualQuaternionPose.rotation = dualRotation;
    }

    // Connect this function to the Slider
    public void SetInterpolation(float value)
    {
        u = value;
    }
}

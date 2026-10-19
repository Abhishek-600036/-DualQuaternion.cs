using UnityEngine;

public class DualQuaternionPoseDemo : MonoBehaviour
{
    public Transform poseA;
    public Transform poseB;

    [Range(0f, 1f)]
    public float u = 0.5f;

    void Update()
    {
        if (poseA == null || poseB == null)
            return;

        // Convert Pose A to dual quaternion
        DualQuaternion dqA =
            DualQuaternion.FromPose(
                poseA.rotation,
                poseA.position
            );

        // Convert Pose B to dual quaternion
        DualQuaternion dqB =
            DualQuaternion.FromPose(
                poseB.rotation,
                poseB.position
            );

        // Interpolate between A and B
        DualQuaternion result =
            DualQuaternion.Slerp(
                dqA,
                dqB,
                u
            );

        // Convert back to position and rotation
        result.ToPose(
            out Quaternion rotation,
            out Vector3 translation
        );

        // Apply the interpolated pose
        transform.rotation = rotation;
        transform.position = translation;
    }

    // Called by the UI Slider
    public void SetInterpolation(float value)
    {
        u = value;
    }
}

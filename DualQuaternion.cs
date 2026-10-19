using UnityEngine;

/// <summary>
/// Unit dual quaternion for rigid-body transforms (rotation + translation) in Unity.
///
///   q̂ = q_r + ε q_d,   ε² = 0
///   q_r = rotation (unit quaternion)
///   q_d = 0.5 * (0, t) * q_r
///
/// Implementation notes
/// - Unity's Quaternion has NO scalar operators (float * Quaternion, Quaternion / float, + , -),
///   so all component-wise maths is done by the small QAdd / QSub / QScale helpers below.
/// - Quaternion * Quaternion is the Hamilton product, so it is used directly through QMul.
/// - Quaternion.Slerp must NOT be used on the dual part: it normalises its result, and the dual
///   part of a unit dual quaternion is not a unit quaternion.  Slerp() below is therefore a true
///   ScLERP (screw linear interpolation) built from the screw parameters of the relative motion.
/// </summary>
[System.Serializable]
public struct DualQuaternion
{
    public Quaternion r; // real part  (rotation)
    public Quaternion d; // dual part  (encodes translation)

    public DualQuaternion(Quaternion real, Quaternion dual)
    {
        r = real;
        d = dual;
    }

    /// <summary>Identity: (1,0,0,0) + ε(0,0,0,0). Note the dual part is the ZERO quaternion.</summary>
    public static DualQuaternion identity
    {
        get { return new DualQuaternion(new Quaternion(0f, 0f, 0f, 1f), new Quaternion(0f, 0f, 0f, 0f)); }
    }

    // ------------------------------------------------------------------
    // Small quaternion helpers (Unity's Quaternion lacks these operators)
    // ------------------------------------------------------------------
    static Quaternion QMul(Quaternion a, Quaternion b)
    {
        // Hamilton product
        return new Quaternion(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
    }

    static Quaternion QAdd(Quaternion a, Quaternion b)
    {
        return new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }

    static Quaternion QSub(Quaternion a, Quaternion b)
    {
        return new Quaternion(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
    }

    static Quaternion QScale(Quaternion q, float s)
    {
        return new Quaternion(q.x * s, q.y * s, q.z * s, q.w * s);
    }

    static Quaternion QConj(Quaternion q)
    {
        return new Quaternion(-q.x, -q.y, -q.z, q.w);
    }

    static float QDot(Quaternion a, Quaternion b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    // ------------------------------------------------------------------
    // Required API
    // ------------------------------------------------------------------

    /// <summary>
    /// Build the unit dual quaternion of a pose:  q_r = R,  q_d = 0.5 * (0,t) * q_r.
    /// The pose maps a point p to  R p + t.
    /// </summary>
    public static DualQuaternion FromPose(Quaternion rotation, Vector3 translation)
    {
        Quaternion qr = rotation.normalized;
        Quaternion t = new Quaternion(translation.x, translation.y, translation.z, 0f); // pure quaternion (0,t)
        Quaternion qd = QScale(QMul(t, qr), 0.5f);
        return new DualQuaternion(qr, qd);
    }

    /// <summary>
    /// Extract rotation and translation:  R = q_r,   t = 2 * q_d * q_r*  (vector part).
    /// </summary>
    public void ToPose(out Quaternion rotation, out Vector3 translation)
    {
        DualQuaternion n = Normalized();
        rotation = n.r;
        Quaternion t = QScale(QMul(n.d, QConj(n.r)), 2f);
        translation = new Vector3(t.x, t.y, t.z);
    }

    /// <summary>
    /// Dual quaternion product  â * b̂ = a_r b_r + ε (a_r b_d + a_d b_r).
    /// Corresponds to applying pose b first, then pose a (same order as matrix a * b).
    /// </summary>
    public static DualQuaternion Multiply(DualQuaternion a, DualQuaternion b)
    {
        Quaternion real = QMul(a.r, b.r);
        Quaternion dual = QAdd(QMul(a.r, b.d), QMul(a.d, b.r));
        return new DualQuaternion(real, dual);
    }

    public static DualQuaternion operator *(DualQuaternion a, DualQuaternion b)
    {
        return Multiply(a, b);
    }

    /// <summary>
    /// Screw linear interpolation (ScLERP):  q̂(u) = â * (â⁻¹ * b̂)^u,  u clamped to [0,1].
    /// The motion follows a constant-speed screw (rotation about an axis + translation along it).
    /// </summary>
    public static DualQuaternion Slerp(DualQuaternion a, DualQuaternion b, float u)
    {
        return SlerpUnclamped(a, b, Mathf.Clamp01(u));
    }

    /// <summary>ScLERP without clamping u (allows extrapolation).</summary>
    public static DualQuaternion SlerpUnclamped(DualQuaternion a, DualQuaternion b, float u)
    {
        a = a.Normalized();
        b = b.Normalized();

        // q̂ and -q̂ are the same pose: pick the sign giving the shortest rotation.
        if (QDot(a.r, b.r) < 0f)
            b = b.Negated();

        DualQuaternion diff = Multiply(a.Inverse(), b); // relative motion, real part has w >= 0
        DualQuaternion part = Pow(diff, u);             // fraction u of that motion
        return Multiply(a, part).Normalized();
    }

    // ------------------------------------------------------------------
    // Extras: normalisation, inverse, power, DLB, point transform
    // ------------------------------------------------------------------

    /// <summary>
    /// Normalise to a unit dual quaternion (||q_r|| = 1 and q_r · q_d = 0).
    /// </summary>
    public DualQuaternion Normalized()
    {
        float n = Mathf.Sqrt(QDot(r, r));
        if (n < 1e-8f)
            return identity;

        float inv = 1f / n;
        Quaternion rn = QScale(r, inv);
        float dot = QDot(rn, d);
        Quaternion dn = QScale(QSub(d, QScale(rn, dot)), inv);
        return new DualQuaternion(rn, dn);
    }

    /// <summary>Component-wise negation (represents the same pose).</summary>
    public DualQuaternion Negated()
    {
        return new DualQuaternion(QScale(r, -1f), QScale(d, -1f));
    }

    /// <summary>Quaternion conjugate applied to both parts: (q_r*, q_d*).</summary>
    public DualQuaternion Conjugate()
    {
        return new DualQuaternion(QConj(r), QConj(d));
    }

    /// <summary>Inverse of a UNIT dual quaternion = (q_r*, q_d*).</summary>
    public DualQuaternion Inverse()
    {
        return Conjugate();
    }

    /// <summary>
    /// Power q̂^t of a unit dual quaternion (real part must have w >= 0), via screw parameters:
    ///   angle θ, axis l, pitch (translation along the axis) p, moment m.
    ///   q̂ = (cos θ/2, sin θ/2 · l) + ε ( -p/2 · sin θ/2,  sin θ/2 · m + p/2 · cos θ/2 · l )
    ///   q̂^t keeps l and m and scales θ and p by t.
    /// </summary>
    public static DualQuaternion Pow(DualQuaternion q, float t)
    {
        Quaternion qr = q.r;
        Quaternion qd = q.d;

        float s = Mathf.Sqrt(qr.x * qr.x + qr.y * qr.y + qr.z * qr.z); // sin(θ/2)
        if (s < 1e-4f)
        {
            // (Almost) pure translation: q̂ = 1 + ε (0, t/2)  ->  q̂^t = 1 + ε t (0, t/2)
            return new DualQuaternion(
                new Quaternion(0f, 0f, 0f, 1f),
                new Quaternion(qd.x * t, qd.y * t, qd.z * t, 0f));
        }

        float c = qr.w;                          // cos(θ/2)
        float halfAngle = Mathf.Atan2(s, c);     // θ/2
        Vector3 l = new Vector3(qr.x, qr.y, qr.z) / s;

        float pitch = -2f * qd.w / s;
        Vector3 dv = new Vector3(qd.x, qd.y, qd.z);
        Vector3 m = (dv - (0.5f * pitch * c) * l) / s;

        float newHalf = halfAngle * t;
        float newPitch = pitch * t;
        float sn = Mathf.Sin(newHalf);
        float cs = Mathf.Cos(newHalf);

        Quaternion rr = new Quaternion(l.x * sn, l.y * sn, l.z * sn, cs);
        Vector3 dVec = sn * m + (0.5f * newPitch * cs) * l;
        Quaternion dd = new Quaternion(dVec.x, dVec.y, dVec.z, -0.5f * newPitch * sn);
        return new DualQuaternion(rr, dd);
    }

    /// <summary>
    /// Dual-quaternion linear blend (DLB / "NLERP"): normalised linear interpolation of both parts.
    /// Cheaper than ScLERP, not constant speed. This is the blend used in dual-quaternion skinning.
    /// </summary>
    public static DualQuaternion Nlerp(DualQuaternion a, DualQuaternion b, float u)
    {
        u = Mathf.Clamp01(u);
        if (QDot(a.r, b.r) < 0f)
            b = b.Negated();

        DualQuaternion sum = new DualQuaternion(
            QAdd(QScale(a.r, 1f - u), QScale(b.r, u)),
            QAdd(QScale(a.d, 1f - u), QScale(b.d, u)));
        return sum.Normalized();
    }

    /// <summary>Apply the rigid transform to a point:  p' = R p + t.</summary>
    public Vector3 TransformPoint(Vector3 p)
    {
        ToPose(out Quaternion rot, out Vector3 trans);
        return rot * p + trans;
    }

    public static DualQuaternion FromTransform(Transform tr)
    {
        return FromPose(tr.rotation, tr.position);
    }

    public override string ToString()
    {
        return "DQ[r=" + r.ToString("F3") + ", d=" + d.ToString("F3") + "]";
    }
}

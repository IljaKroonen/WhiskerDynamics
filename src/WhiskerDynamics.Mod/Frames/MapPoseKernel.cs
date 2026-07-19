using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Frames;

/// <summary>KSA-free camera math (Brutal numerics only — offline-testable).
/// Conventions are PINNED BY TEST against double3.Transform, not assumed:
/// if Brutal's quaternion composition order ever surprises, the
/// invariance test fails offline. If CounterPose's composition order is wrong for the
/// live build, fix it to make MapPoseKernelTests pass — never hand-tune in-game.</summary>
public static class MapPoseKernel
{
    /// <summary>Rotation mapping unit X/Y/Z onto the given orthonormal basis
    /// (Shepperd's method over the column matrix [x y z]).</summary>
    public static doubleQuat QuatFromBasis(double3 x, double3 y, double3 z)
    {
        // Column-matrix entries: m00=x.X m01=y.X m02=z.X / m10=x.Y m11=y.Y m12=z.Y / ...
        double m00 = x.X, m01 = y.X, m02 = z.X;
        double m10 = x.Y, m11 = y.Y, m12 = z.Y;
        double m20 = x.Z, m21 = y.Z, m22 = z.Z;
        double trace = m00 + m11 + m22;
        double qw, qx, qy, qz;
        if (trace > 0)
        {
            double s = Math.Sqrt(trace + 1.0) * 2;
            qw = 0.25 * s;
            qx = (m21 - m12) / s;
            qy = (m02 - m20) / s;
            qz = (m10 - m01) / s;
        }
        else if (m00 > m11 && m00 > m22)
        {
            double s = Math.Sqrt(1.0 + m00 - m11 - m22) * 2;
            qw = (m21 - m12) / s;
            qx = 0.25 * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            double s = Math.Sqrt(1.0 + m11 - m00 - m22) * 2;
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25 * s;
            qz = (m12 + m21) / s;
        }
        else
        {
            double s = Math.Sqrt(1.0 + m22 - m00 - m11) * 2;
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25 * s;
        }
        return new doubleQuat(qx, qy, qz, qw);
    }

    /// <summary>Rigidly rotates a camera pose about <paramref name="center"/> by
    /// <paramref name="delta"/>: position orbits the center, orientation composes so the
    /// view of frame-corotating content is invariant (pinned by test).</summary>
    public static (double3 Position, doubleQuat Rotation) CounterPose(
        double3 cameraPositionEcl, doubleQuat cameraLocalRotation, double3 center, doubleQuat delta)
    {
        var position = center + double3.Transform(cameraPositionEcl - center, delta);
        var rotation = doubleQuat.Concatenate(cameraLocalRotation, delta);
        return (position, rotation);
    }

    /// <summary>The full frame-view re-pose: rotates the camera rig RIGIDLY
    /// about the FOLLOW ANCHOR (the followed target's drawn position) when one exists,
    /// about the frame origin only when nothing is followed. The stock map controller
    /// rebuilds the camera each frame as anchor - forward*scope with forward/up built in
    /// FIXED ecliptic axes (MapController.cs:270-282), so the follow offset does NOT
    /// co-rotate with the frame: a composition that rotates about the frame
    /// origin slides any followed target off its pixel by (I - delta)(anchor - origin) —
    /// ~2 sin(angle/2)|anchor-origin| of vessel-follow drift, invisible
    /// when following the primary because the anchor IS the origin there. Rotating about
    /// the anchor keeps ANY followed target pixel-invariant (pinned by test, and exact
    /// regardless of rails/epoch skew because the anchor is the target's own drawn
    /// position) while the frame's rotation renders around it.</summary>
    public static (double3 Position, doubleQuat Rotation) FrameViewPose(
        double3 cameraPositionEcl, doubleQuat cameraLocalRotation,
        double3 frameOrigin, double3? followAnchor, doubleQuat delta)
        => CounterPose(cameraPositionEcl, cameraLocalRotation, followAnchor ?? frameOrigin, delta);
}

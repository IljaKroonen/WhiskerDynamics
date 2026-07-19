namespace WhiskerDynamics.Core;

/// <summary>A reference-frame pose in shared inertial axes. <see cref="Scale"/>
/// normalizes frame coordinates; it is 1 for rigid frames and the primary-secondary
/// separation for rotating-pulsating frames. This type transforms positions only.</summary>
public readonly record struct FramePose(
    Vector3d Origin, Vector3d XAxis, Vector3d YAxis, Vector3d ZAxis, double Scale = 1.0)
{
    /// <summary>Converts an absolute position to scale-normalized frame coordinates.</summary>
    public Vector3d ToFrame(Vector3d absolutePosition)
    {
        var r = absolutePosition - Origin;
        return new Vector3d(r.Dot(XAxis) / Scale, r.Dot(YAxis) / Scale, r.Dot(ZAxis) / Scale);
    }

    /// <summary>Converts scale-normalized frame coordinates to an absolute position.</summary>
    public Vector3d FromFrame(Vector3d frameCoordinates) =>
        Origin
        + XAxis * (frameCoordinates.X * Scale)
        + YAxis * (frameCoordinates.Y * Scale)
        + ZAxis * (frameCoordinates.Z * Scale);
}

/// <summary>A body-fixed basis at <paramref name="ReferenceTime"/> and its constant
/// angular velocity about <paramref name="PoleEcl"/>. Positive rates are right-handed.</summary>
public readonly record struct BodyRotation(
    Vector3d PoleEcl, Vector3d XAxisEcl, Vector3d YAxisEcl,
    double AngularVelocity, double ReferenceTime);

public static class FrameKernel
{
    private const double BasisTolerance = 1e-12;

    /// <summary>Builds a rotating-pulsating frame centered on the primary, with +X
    /// toward the secondary, +Z along relative angular momentum, and scale equal to
    /// their separation. Throws for non-finite state, coincident bodies, or zero/
    /// numerically unresolvable angular momentum.</summary>
    public static FramePose Rotating(StateVector primary, StateVector secondary)
    {
        if (!Finite(primary.Position) || !Finite(primary.Velocity))
            throw new ArgumentException(
                "Rotating frame invalid: primary state contains a non-finite component.",
                nameof(primary));
        if (!Finite(secondary.Position) || !Finite(secondary.Velocity))
            throw new ArgumentException(
                "Rotating frame invalid: secondary state contains a non-finite component.",
                nameof(secondary));

        var r = secondary.Position - primary.Position;
        var relativeVelocity = secondary.Velocity - primary.Velocity;
        if (!Finite(r))
            throw new ArgumentException(
                "Rotating frame invalid: relative position is not finite.");
        if (!Finite(relativeVelocity))
            throw new ArgumentException(
                "Rotating frame invalid: relative velocity is not finite.");

        // Use the direct path for ordinary geometry and the scaled path when squaring
        // overflows or underflows, keeping finite separations such as 1e+200 and
        // 1e-300 usable.
        double rLengthSquared = r.LengthSquared();
        double rLength;
        Vector3d x;
        if (double.IsFinite(rLengthSquared) && rLengthSquared > 0.0)
        {
            rLength = Math.Sqrt(rLengthSquared);
            x = r / rLength;
        }
        else if (!TryScaledUnit(r, out x, out rLength))
        {
            throw new ArgumentException("Rotating frame degenerate: primary and secondary coincide.");
        }
        if (!double.IsFinite(rLength))
            throw new ArgumentException(
                "Rotating frame invalid: separation length is not finite.");

        // The raw angular-momentum path is likewise retained whenever its squared
        // length is finite-positive. For extreme scales its products can overflow or
        // underflow even though both directions are sound; normalize the input
        // directions first in that case so the replacement cross stays bounded.
        var h = r.Cross(relativeVelocity);
        Vector3d z = default;
        Vector3d y = default;
        bool directBasis = false;
        if (Finite(h))
        {
            double hLengthSquared = h.LengthSquared();
            if (double.IsFinite(hLengthSquared) && hLengthSquared > 0.0)
            {
                double hLength = Math.Sqrt(hLengthSquared);
                z = h / hLength;
                y = z.Cross(x);
                directBasis = ValidBasis(x, y, z);
            }
        }

        if (!directBasis)
        {
            // A finite raw cross can still lose almost every significant bit when
            // large, nearly parallel products cancel. Rebuild every direction with
            // bounded operands before deciding whether the geometry is representable.
            if (!TryScaledUnit(r, out var scaledX, out _)
                || !TryScaledUnit(relativeVelocity, out var velocityDirection, out _)
                || !TryScaledUnit(scaledX.Cross(velocityDirection), out var scaledZ, out _))
            {
                throw new ArgumentException(
                    "Rotating frame degenerate: zero relative angular momentum.");
            }
            var scaledY = scaledZ.Cross(scaledX);
            if (!ValidBasis(scaledX, scaledY, scaledZ))
                throw new ArgumentException(
                    "Rotating frame invalid: derived basis is not orthonormal.");
            x = scaledX;
            y = scaledY;
            z = scaledZ;
        }

        return new FramePose(primary.Position, x, y, z, rLength);
    }

    private static bool TryScaledUnit(Vector3d value, out Vector3d unit, out double length)
    {
        double scale = Math.Max(Math.Abs(value.X),
            Math.Max(Math.Abs(value.Y), Math.Abs(value.Z)));
        if (!(scale > 0.0) || !double.IsFinite(scale))
        {
            unit = default;
            length = 0.0;
            return false;
        }

        var scaled = value / scale;
        double scaledLength = Math.Sqrt(scaled.LengthSquared());
        if (!(scaledLength > 0.0) || !double.IsFinite(scaledLength))
        {
            unit = default;
            length = 0.0;
            return false;
        }

        unit = scaled / scaledLength;
        length = scale * scaledLength;
        return Finite(unit);
    }

    private static bool Finite(Vector3d value) =>
        double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);

    private static bool ValidBasis(Vector3d x, Vector3d y, Vector3d z)
    {
        if (!Finite(x) || !Finite(y) || !Finite(z)) return false;
        if (!(Math.Abs(x.LengthSquared() - 1.0) <= BasisTolerance)
            || !(Math.Abs(y.LengthSquared() - 1.0) <= BasisTolerance)
            || !(Math.Abs(z.LengthSquared() - 1.0) <= BasisTolerance)
            || !(Math.Abs(x.Dot(y)) <= BasisTolerance)
            || !(Math.Abs(y.Dot(z)) <= BasisTolerance)
            || !(Math.Abs(z.Dot(x)) <= BasisTolerance))
            return false;
        return (x.Cross(y) - z).LengthSquared() <= BasisTolerance * BasisTolerance;
    }

    /// <summary>Body-centered inertial frame: origin rides the body, axes stay the shared
    /// inertial axes.</summary>
    public static FramePose Inertial(StateVector body) =>
        new(body.Position, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, 0, 1));

    /// <summary>Builds a body-surface frame at <paramref name="t"/> using constant-rate
    /// rotation about the fixed pole. Throws when the pole is zero.</summary>
    public static FramePose Surface(StateVector body, BodyRotation rotation, double t)
    {
        if (rotation.PoleEcl.LengthSquared() == 0)
            throw new ArgumentException("Surface frame degenerate: zero rotation pole.");
        double angle = rotation.AngularVelocity * (t - rotation.ReferenceTime);
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new FramePose(
            body.Position,
            RotateAbout(rotation.XAxisEcl, rotation.PoleEcl, c, s),
            RotateAbout(rotation.YAxisEcl, rotation.PoleEcl, c, s),
            rotation.PoleEcl);
    }

    private static Vector3d RotateAbout(Vector3d value, Vector3d axis, double c, double s) =>
        value * c + axis.Cross(value) * s + axis * (axis.Dot(value) * (1 - c));

    /// <summary>Re-embeds a sampled point at the current pose while preserving its
    /// scale-normalized frame coordinates.</summary>
    public static Vector3d Reembed(FramePose samplePose, FramePose nowPose, Vector3d sampledAbsolute) =>
        nowPose.FromFrame(samplePose.ToFrame(sampledAbsolute));
}

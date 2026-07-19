namespace WhiskerDynamics.Core;

/// <summary>Available GRGM1200A truncations for lunar gameplay gravity.</summary>
public enum LunarGravityFidelity
{
    Degree10 = 10,
    Degree20 = 20,
    Degree30 = 30,
    Degree40 = 40,
    Degree50 = 50,
}

/// <summary>GRGM1200A (GRAIL release 7), truncated for gameplay use.</summary>
internal static partial class LunarGravityModel
{
    internal const double ReferenceRadius = 1_738_000.0;

    internal static Geopotential Create(BodyRotation rotation,
        LunarGravityFidelity fidelity = LunarGravityFidelity.Degree50)
    {
        if (!Enum.IsDefined(fidelity))
            throw new ArgumentOutOfRangeException(nameof(fidelity));

        int maximumDegree = (int)fidelity;
        return Geopotential.FromFullyNormalized(ReferenceRadius, rotation,
            Coefficients.TakeWhile(coefficient => coefficient.Degree <= maximumDegree));
    }
}

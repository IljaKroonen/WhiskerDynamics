using Brutal.Numerics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Frames;

/// <summary>Conversions between WhiskerDynamics.Core vectors (root-ecliptic axes, SI) and game
/// numerics. Uses only Brutal.Core.Numerics types so it is unit-testable without KSA.dll.</summary>
public static class FrameAdapter
{
    public static Vector3d ToCore(double3 v) => new(v.X, v.Y, v.Z);
    public static double3 ToGame(Vector3d v) => new double3(v.X, v.Y, v.Z);

    /// <summary>Parent-relative Ecl-axes vector -> parent-Cci axes.</summary>
    public static double3 EclToCci(Vector3d relativeEcl, doubleQuat cce2Cci) =>
        double3.Transform(ToGame(relativeEcl), cce2Cci);

    /// <summary>Parent-Cci vector -> parent-relative Ecl axes.</summary>
    public static Vector3d CciToEcl(double3 relativeCci, doubleQuat cci2Cce) =>
        ToCore(double3.Transform(relativeCci, cci2Cce));

    /// <summary>THE parent-relative game Cci -> mod-frame absolute composition, and its
    /// inverse below — one home shared by TrackedVessel (seeding, staging, rebase reads)
    /// and the SOI handoff patch, so the two directions of the same frame convention can
    /// never drift apart across consumers.</summary>
    public static Vector3d GameToAbsolute(Vector3d parentAbsolute, double3 relativeCci, doubleQuat cci2Cce) =>
        parentAbsolute + CciToEcl(relativeCci, cci2Cce);

    /// <summary>Mod-frame absolute -> parent-relative game Cci (the inverse of
    /// <see cref="GameToAbsolute"/>).</summary>
    public static double3 AbsoluteToGame(Vector3d absolute, Vector3d parentAbsolute, doubleQuat cce2Cci) =>
        EclToCci(absolute - parentAbsolute, cce2Cci);

    /// <summary>Bubble-axes vector -> parent-relative Ecl axes (Seam 2; bub2Cce folds
    /// the Ccf rotation in when the bubble frame is body-fixed).</summary>
    public static Vector3d BubToEcl(double3 bub, doubleQuat bub2Cce) =>
        ToCore(double3.Transform(bub, bub2Cce));

    /// <summary>Parent-relative Ecl-axes vector -> bubble axes (Seam 2).</summary>
    public static double3 EclToBub(Vector3d ecl, doubleQuat bub2Cce) =>
        double3.Transform(ToGame(ecl), doubleQuat.Inverse(bub2Cce));
}

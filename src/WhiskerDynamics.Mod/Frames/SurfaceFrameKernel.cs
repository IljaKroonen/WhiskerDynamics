using Brutal.Numerics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Frames;

/// <summary>KSA-free math for the body-surface frame's spin model (Brutal numerics
/// only — offline-testable without KSA.dll, the FrameAdapter/MapPoseKernel precedent).
/// The game's body-fixed orientation is, per the decompiled sources:
///
///   Celestial.cs:547-551  GetCcf2Cci(SimTime t) = doubleQuat.CreateFromAxisAngle(
///                         double3.UnitZ, t.Seconds() * AngularVelocity + InitialRotation)
///   Celestial.cs:560-563  GetCcf2Cce(SimTime t) = doubleQuat.Concatenate(
///                         GetCcf2Cci(t), _cci2Cce)
///   Celestial.cs:585      _cci2Cce = Concatenate(Orbit.GetParentCce2Orb(), Orb2Cci)
///                         .Inverse() — a fixed composition of defining-conic
///                         orientations, constant in time (the same reading LiveCatalog
///                         relies on for GetCci2Cce); Orb2Cci and AngularVelocity /
///                         InitialRotation are set once in the constructor
///                         (Celestial.cs:229-244)
///
/// i.e. a CONSTANT angular velocity spin about the body-fixed +Z composed with a
/// constant tilt. Because a rotation about Z fixes Z, the pole in ecliptic axes is
/// Transform(UnitZ, ccf2Ecl(t)) — constant — and the whole time-dependence reduces to
/// "reference basis rotated about the pole by omega * (t - tRef)", which is exactly
/// <see cref="FrameKernel.Surface"/>. That makes the pose computable at ARBITRARY t
/// (camera counter-pose at 'now', curve re-embedding at past/future sample times), not
/// just at the game's cached 'now' orientation. <see cref="VerifyReconstruction"/> is
/// the guard: at activation the reader re-checks the model against the game's own
/// quaternion at a second time, so a changed game formula (or a non-constant tilt)
/// refuses activation instead of rendering a wrong frame.</summary>
public static class SurfaceFrameKernel
{
    /// <summary>Axis-error ceiling for <see cref="VerifyReconstruction"/>. Both sides
    /// are exact rigid rotations, so agreement is fp-round-off (~1e-13 after quaternion
    /// chains); any convention error (sign, composition order, wrong pole) shows up at
    /// O(spin angle) over the verification interval — many orders above this.</summary>
    public const double VerifyTolerance = 1e-9;

    /// <summary>Builds the KSA-free spin model from the game's bodyFixed→ecliptic
    /// quaternion at <paramref name="referenceTime"/> plus the body's constant spin
    /// rate (rad/s; negative for retrograde rotators, Celestial.cs:612-629). Pole =
    /// the transformed body-fixed +Z (see class doc).</summary>
    public static BodyRotation ModelFromGameQuat(
        doubleQuat ccf2Ecl, double angularVelocity, double referenceTime) => new(
        FrameAdapter.ToCore(double3.Transform(double3.UnitZ, ccf2Ecl)),
        FrameAdapter.ToCore(double3.Transform(double3.UnitX, ccf2Ecl)),
        FrameAdapter.ToCore(double3.Transform(double3.UnitY, ccf2Ecl)),
        angularVelocity, referenceTime);

    /// <summary>Null when <see cref="FrameKernel.Surface"/>'s arbitrary-t
    /// reconstruction reproduces the game's own bodyFixed→ecliptic orientation at
    /// <paramref name="t"/>; otherwise a panel-ready refusal reason. Throws only on an
    /// exact-zero pole (callers gate via FrameCatalog.ValidateRotation first).</summary>
    public static string? VerifyReconstruction(BodyRotation model, doubleQuat gameCcf2EclAtT, double t)
    {
        var pose = FrameKernel.Surface(new StateVector(Vector3d.Zero, Vector3d.Zero), model, t);
        double deviation = Math.Max(
            (pose.XAxis - FrameAdapter.ToCore(double3.Transform(double3.UnitX, gameCcf2EclAtT))).Length(),
            Math.Max(
                (pose.YAxis - FrameAdapter.ToCore(double3.Transform(double3.UnitY, gameCcf2EclAtT))).Length(),
                (pose.ZAxis - FrameAdapter.ToCore(double3.Transform(double3.UnitZ, gameCcf2EclAtT))).Length()));
        return deviation <= VerifyTolerance ? null
            : $"surface spin reconstruction diverges from game (axis error {deviation:E2} at t={t:F1} s)";
    }
}

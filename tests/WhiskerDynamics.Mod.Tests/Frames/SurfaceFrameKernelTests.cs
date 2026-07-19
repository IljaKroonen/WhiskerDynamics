using Brutal.Numerics;
using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Frames;

/// <summary>Verifies body-surface frame reconstruction against the constant-spin
/// quaternion model, including composition order and handedness.</summary>
public class SurfaceFrameKernelTests
{
    /// <summary>Body-fixed to ecliptic rotation for constant spin, initial phase, and tilt.</summary>
    private static doubleQuat GameCcf2Ecl(double t, double omega, double theta0, doubleQuat cci2Cce) =>
        doubleQuat.Concatenate(
            doubleQuat.CreateFromAxisAngle(double3.UnitZ, t * omega + theta0), cci2Cce);

    // Earth-like sidereal rate, arbitrary phase, and 23.4-degree axial tilt.
    private const double Omega = 7.2921159e-5;
    private const double Theta0 = 1.234;
    private static readonly doubleQuat Tilt =
        doubleQuat.CreateFromAxisAngle(-double3.UnitX, -23.4 * Math.PI / 180.0);

    [Fact]
    public void Reconstruction_matches_the_game_formula_at_arbitrary_times()
    {
        double tRef = 5.5e6;
        var model = SurfaceFrameKernel.ModelFromGameQuat(
            GameCcf2Ecl(tRef, Omega, Theta0, Tilt), Omega, tRef);
        Assert.Null(FrameCatalog.ValidateRotation(model));
        // Past AND future sample times: curve re-embedding samples both directions.
        foreach (double t in new[] { tRef, tRef + 3600, tRef + 86400 * 30, tRef - 2.5e5, 0.0 })
            Assert.Null(SurfaceFrameKernel.VerifyReconstruction(
                model, GameCcf2Ecl(t, Omega, Theta0, Tilt), t));
    }

    [Fact]
    public void Reconstruction_handles_retrograde_and_non_spinning_bodies()
    {
        double tRef = 1e5;
        // Retrograde rotation uses a negative rate.
        var retro = SurfaceFrameKernel.ModelFromGameQuat(
            GameCcf2Ecl(tRef, -Omega, 0.4, Tilt), -Omega, tRef);
        Assert.Null(SurfaceFrameKernel.VerifyReconstruction(
            retro, GameCcf2Ecl(tRef + 7.2e4, -Omega, 0.4, Tilt), tRef + 7.2e4));
        // A non-spinning body has a fixed identity basis.
        var still = SurfaceFrameKernel.ModelFromGameQuat(doubleQuat.Identity, 0.0, tRef);
        Assert.Null(SurfaceFrameKernel.VerifyReconstruction(still, doubleQuat.Identity, tRef + 1e7));
        var pose = FrameKernel.Surface(new StateVector(Vector3d.Zero, Vector3d.Zero), still, tRef + 1e7);
        Assert.True((pose.XAxis - new Vector3d(1, 0, 0)).Length() < 1e-12);
        Assert.True((pose.ZAxis - new Vector3d(0, 0, 1)).Length() < 1e-12);
    }

    [Fact]
    public void Verification_refuses_a_wrong_rate_and_a_drifting_tilt()
    {
        double tRef = 5.5e6, tCheck = tRef + 3600;
        var truth = GameCcf2Ecl(tRef, Omega, Theta0, Tilt);
        // Wrong spin rate (1% off): over 1 h the axis error is ~2.6e-3 — must refuse.
        var wrongRate = SurfaceFrameKernel.ModelFromGameQuat(truth, Omega * 1.01, tRef);
        Assert.NotNull(SurfaceFrameKernel.VerifyReconstruction(
            wrongRate, GameCcf2Ecl(tCheck, Omega, Theta0, Tilt), tCheck));
        // A second sample exposes any violation of the constant-tilt assumption.
        var model = SurfaceFrameKernel.ModelFromGameQuat(truth, Omega, tRef);
        var driftedTilt = doubleQuat.CreateFromAxisAngle(-double3.UnitX, -23.5 * Math.PI / 180.0);
        Assert.NotNull(SurfaceFrameKernel.VerifyReconstruction(
            model, GameCcf2Ecl(tCheck, Omega, Theta0, driftedTilt), tCheck));
    }

    [Fact]
    public void Model_pole_is_the_transformed_body_fixed_z_and_basis_is_orthonormal()
    {
        // The spin fixes UnitZ (rotation about Z), so the pole must be the tilted +Z
        // regardless of spin phase — and the basis must transform rigidly.
        var qA = GameCcf2Ecl(0, Omega, Theta0, Tilt);
        var qB = GameCcf2Ecl(9.9e6, Omega, Theta0, Tilt);
        var modelA = SurfaceFrameKernel.ModelFromGameQuat(qA, Omega, 0);
        var modelB = SurfaceFrameKernel.ModelFromGameQuat(qB, Omega, 9.9e6);
        Assert.True((modelA.PoleEcl - modelB.PoleEcl).Length() < 1e-12);
        var tiltedZ = FrameAdapter.ToCore(double3.Transform(double3.UnitZ, Tilt));
        Assert.True((modelA.PoleEcl - tiltedZ).Length() < 1e-12);
        Assert.Null(FrameCatalog.ValidateRotation(modelA));
        Assert.Null(FrameCatalog.ValidateRotation(modelB));
    }
}

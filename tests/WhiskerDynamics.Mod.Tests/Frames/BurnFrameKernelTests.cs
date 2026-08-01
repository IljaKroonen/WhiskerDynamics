using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Frames;

/// <summary>Tests burn-basis conversions, degeneracy handling, and staleness.</summary>
public class BurnFrameKernelTests
{
    private static void AssertClose(Vector3d expected, Vector3d actual, double tol = 1e-9) =>
        Assert.True((expected - actual).Length() <= tol,
            $"expected {expected.X},{expected.Y},{expected.Z} got {actual.X},{actual.Y},{actual.Z}");

    [Fact]
    public void Vlf_basis_matches_the_stock_construction()
    {
        var r = new Vector3d(7e6, 1e6, -2e6);
        var v = new Vector3d(1e3, 7e3, 2e3);
        Assert.True(BurnFrameKernel.TryVlfBasis(r, v, out var x, out var y, out var z));
        AssertClose(v.Normalized(), x, 1e-12);
        AssertClose(r.Normalized().Cross(v.Normalized()).Normalized(), y, 1e-12);
        AssertClose(x.Cross(y), z, 1e-12);
        Assert.Equal(1.0, x.Length(), 12);
        Assert.Equal(1.0, y.Length(), 12);
        Assert.Equal(1.0, z.Length(), 12);
        Assert.Equal(0.0, x.Dot(y), 12);
        Assert.Equal(0.0, x.Dot(z), 12);
        Assert.Equal(0.0, y.Dot(z), 12);
    }

    [Fact]
    public void Prograde_normal_outward_map_to_the_physical_directions()
    {
        // Circular geometry gives prograde +Y, normal +Z, and outward +X.
        var r = new Vector3d(7e6, 0, 0);
        var v = new Vector3d(0, 7.5e3, 0);
        AssertClose(new Vector3d(0, 5, 0), BurnFrameKernel.VlfToEcl(new Vector3d(5, 0, 0), r, v)!.Value);
        AssertClose(new Vector3d(0, 0, 3), BurnFrameKernel.VlfToEcl(new Vector3d(0, 3, 0), r, v)!.Value);
        AssertClose(new Vector3d(2, 0, 0), BurnFrameKernel.VlfToEcl(new Vector3d(0, 0, 2), r, v)!.Value);
    }

    [Fact]
    public void Frenet_components_map_prograde_radial_normal_to_the_frame_state()
    {
        // Authored component order is prograde, radial, normal.
        var r = new Vector3d(4e8, 0, 0);
        var v = new Vector3d(0, 1e3, 0);
        AssertClose(new Vector3d(0, 5, 0), BurnFrameKernel.FrenetToFrame(new Vector3d(5, 0, 0), r, v)!.Value);
        AssertClose(new Vector3d(2, 0, 0), BurnFrameKernel.FrenetToFrame(new Vector3d(0, 2, 0), r, v)!.Value);
        AssertClose(new Vector3d(0, 0, 3), BurnFrameKernel.FrenetToFrame(new Vector3d(0, 0, 3), r, v)!.Value);
    }

    [Fact]
    public void Frenet_round_trip_preserves_components_and_magnitude()
    {
        var r = new Vector3d(3.8e8, -1.1e7, 2.2e6);
        var v = new Vector3d(-40.0, 950.0, 12.0);
        var authored = new Vector3d(7.5, -1.25, 2.0);
        var dvFrame = BurnFrameKernel.FrenetToFrame(authored, r, v);
        Assert.NotNull(dvFrame);
        Assert.Equal(authored.Length(), dvFrame.Value.Length(), 9); // orthonormal basis
        var back = BurnFrameKernel.FrameToFrenet(dvFrame.Value, r, v);
        Assert.NotNull(back);
        AssertClose(authored, back.Value);
    }

    [Fact]
    public void Frenet_refuses_degenerate_frame_states()
    {
        // Stationary and radial states have no usable prograde basis.
        Assert.Null(BurnFrameKernel.FrenetToFrame(new Vector3d(1, 0, 0), new Vector3d(4e8, 0, 0), Vector3d.Zero));
        Assert.Null(BurnFrameKernel.FrameToFrenet(new Vector3d(1, 0, 0), new Vector3d(4e8, 0, 0), new Vector3d(1e3, 0, 0)));
    }

    [Fact]
    public void Vlf_ecl_round_trip_preserves_components_and_magnitude()
    {
        var r = new Vector3d(6.8e6, -1.2e6, 3.4e5);
        var v = new Vector3d(2.1e3, 6.9e3, -8.0e2);
        var dvVlf = new Vector3d(12.5, -3.25, 0.75);
        var dvEcl = BurnFrameKernel.VlfToEcl(dvVlf, r, v);
        Assert.NotNull(dvEcl);
        Assert.Equal(dvVlf.Length(), dvEcl.Value.Length(), 9); // rotation preserves norm
        var back = BurnFrameKernel.EclToVlf(dvEcl.Value, r, v);
        Assert.NotNull(back);
        AssertClose(dvVlf, back.Value);
    }

    [Fact]
    public void Radial_trajectory_is_rejected_both_ways()
    {
        var r = new Vector3d(7e6, 1e6, 0);
        var v = r * 1e-3;
        Assert.False(BurnFrameKernel.TryVlfBasis(r, v, out _, out _, out _));
        Assert.Null(BurnFrameKernel.VlfToEcl(new Vector3d(1, 0, 0), r, v));
        Assert.Null(BurnFrameKernel.EclToVlf(new Vector3d(1, 0, 0), r, v));
        var almost = r * 1e-3 + new Vector3d(0, 0, 1e-9 * r.Length() * 1e-3);
        Assert.False(BurnFrameKernel.TryVlfBasis(r, almost, out _, out _, out _));
    }

    [Fact]
    public void Zero_vectors_are_rejected()
    {
        Assert.False(BurnFrameKernel.TryVlfBasis(Vector3d.Zero, new Vector3d(1, 0, 0), out _, out _, out _));
        Assert.False(BurnFrameKernel.TryVlfBasis(new Vector3d(1, 0, 0), Vector3d.Zero, out _, out _, out _));
    }

    [Fact]
    public void Frame_axes_round_trip_ignores_the_pose_origin()
    {
        // Position and scale must not affect direction-vector conversions.
        var pose = FrameKernel.Rotating(
            new StateVector(new Vector3d(1e11, -3e10, 5e9), new Vector3d(1e4, 2e4, -3e3)),
            new StateVector(new Vector3d(1.6e11, 6e10, 9e9), new Vector3d(-1.5e4, 2.6e4, 2e3)));
        var authored = new Vector3d(4.2, -1.3, 0.6);
        var dvEcl = BurnFrameKernel.FrameToEcl(authored, pose);
        Assert.Equal(authored.Length(), dvEcl.Length(), 9);
        AssertClose(authored, BurnFrameKernel.EclToFrame(dvEcl, pose));
        var shifted = pose with { Origin = pose.Origin + new Vector3d(1e12, -1e12, 3e11) };
        AssertClose(dvEcl, BurnFrameKernel.FrameToEcl(authored, shifted), 1e-12);
    }

    [Fact]
    public void Frame_conversions_are_scale_blind()
    {
        var pose = FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(3.8e8, 1e7, 2e7), new Vector3d(-100, 1000, 30)));
        Assert.True(pose.Scale > 1e8); // the pulsating separation is in the pose
        var authored = new Vector3d(4.2, -1.3, 0.6);
        var dvEcl = BurnFrameKernel.FrameToEcl(authored, pose);
        Assert.Equal(authored.Length(), dvEcl.Length(), 9); // unit axes: magnitude kept
        var rescaled = pose with { Scale = 1.0 };
        AssertClose(dvEcl, BurnFrameKernel.FrameToEcl(authored, rescaled), 1e-12);
        AssertClose(BurnFrameKernel.EclToFrame(dvEcl, pose),
            BurnFrameKernel.EclToFrame(dvEcl, rescaled), 1e-12);
    }

    [Fact]
    public void Authored_components_survive_the_full_frame_to_vlf_and_back_chain()
    {
        // Authoring and redisplay round-trip through ecliptic and VLF bases.
        var pose = FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(3.8e8, 1e7, 2e7), new Vector3d(-100, 1000, 30)));
        var rRel = new Vector3d(6.9e6, 2e5, -4e5);
        var vRel = new Vector3d(-500, 7.6e3, 1.1e3);
        var authored = new Vector3d(-2.5, 7.75, 3.125);

        var dvEcl = BurnFrameKernel.FrameToEcl(authored, pose);
        var dvVlf = BurnFrameKernel.EclToVlf(dvEcl, rRel, vRel);
        Assert.NotNull(dvVlf);
        var eclBack = BurnFrameKernel.VlfToEcl(dvVlf.Value, rRel, vRel);
        Assert.NotNull(eclBack);
        AssertClose(authored, BurnFrameKernel.EclToFrame(eclBack.Value, pose));
    }

    [Fact]
    public void Execution_realized_analysis_is_never_stale_against_the_predictor_realization()
    {
        var meta = new FlightPlanBurnMeta
        {
            TimeSeconds = 5000,
            Frame = new FrameSpec(FrameKind.Inertial, "Earth", null),
            Authored = new Vector3d(10, 0, 0),
            StampMs = 0,
        };
        var stored = new Vector3d(9, 4, 1); // stock-basis realization
        var fresh = new Vector3d(10, 0, 0); // predictor-basis realization
        var drifted = new PlannedBurnConverter.BurnAnalysis(
            5000, stored, meta, meta.Authored, fresh, null);
        Assert.True(drifted.Stale);
        Assert.False((drifted with { ExecutionRealized = true }).Stale);
    }

    [Fact]
    public void Staleness_is_the_vlf_difference_magnitude_against_the_tolerance()
    {
        var current = new Vector3d(10, 0, 0);
        Assert.False(BurnFrameKernel.IsStale(new Vector3d(10, 0, 0.009), current, 0.01));
        Assert.True(BurnFrameKernel.IsStale(new Vector3d(10, 0, 0.011), current, 0.01));
        Assert.False(BurnFrameKernel.IsStale(current, current, 0.01));
    }
}

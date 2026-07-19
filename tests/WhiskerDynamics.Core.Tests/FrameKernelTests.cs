using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class FrameKernelTests
{
    // Circular two-body fixture with an exactly frame-fixed L4 point.
    private const double Mu = 3.986004418e14;
    private const double Radius = 384_400_000.0;
    private static readonly double Omega = Math.Sqrt(Mu / (Radius * Radius * Radius));

    private static StateVector PrimaryAt(double t) => new(Vector3d.Zero, Vector3d.Zero);

    private static StateVector SecondaryAt(double t)
    {
        double a = Omega * t;
        return new StateVector(
            new Vector3d(Radius * Math.Cos(a), Radius * Math.Sin(a), 0),
            new Vector3d(-Radius * Omega * Math.Sin(a), Radius * Omega * Math.Cos(a), 0));
    }

    private static Vector3d L4At(double t)
    {
        double a = Omega * t + Math.PI / 3;
        return new Vector3d(Radius * Math.Cos(a), Radius * Math.Sin(a), 0);
    }

    [Fact]
    public void Rotating_pose_axes_are_orthonormal()
    {
        var pose = FrameKernel.Rotating(PrimaryAt(1e5), SecondaryAt(1e5));
        Assert.Equal(1.0, pose.XAxis.Length(), 12);
        Assert.Equal(1.0, pose.YAxis.Length(), 12);
        Assert.Equal(1.0, pose.ZAxis.Length(), 12);
        Assert.Equal(0.0, pose.XAxis.Dot(pose.YAxis), 12);
        Assert.Equal(0.0, pose.YAxis.Dot(pose.ZAxis), 12);
        Assert.Equal(0.0, pose.ZAxis.Dot(pose.XAxis), 12);
        Assert.Equal(0.0, (pose.XAxis.Cross(pose.YAxis) - pose.ZAxis).Length(), 12);
    }

    [Fact]
    public void Prograde_orbit_gives_z_axis_along_inertial_plus_z()
    {
        // A prograde XY orbit has frame +Z aligned with inertial +Z.
        var pose = FrameKernel.Rotating(PrimaryAt(3e5), SecondaryAt(3e5));
        Assert.Equal(0.0, (pose.ZAxis - new Vector3d(0, 0, 1)).Length(), 12);
    }

    [Fact]
    public void Rotating_uses_the_relative_state_of_an_off_origin_moving_primary()
    {
        // Origin and angular momentum are relative to the translated, moving primary.
        var primary = new StateVector(new Vector3d(1e11, 2e10, 0), new Vector3d(5e3, -1e3, 0));
        var offset = new Vector3d(3e8, 4e8, 0); // |offset| = 5e8 exactly
        var relVel = new Vector3d(-400, 300, 500);
        var secondary = new StateVector(primary.Position + offset, primary.Velocity + relVel);

        var pose = FrameKernel.Rotating(primary, secondary);

        Assert.Equal(primary.Position, pose.Origin);
        var expectedX = offset / offset.Length();
        var expectedDirectZ = offset.Cross(relVel) / offset.Cross(relVel).Length();
        Assert.Equal(expectedX, pose.XAxis);
        Assert.Equal(expectedDirectZ, pose.ZAxis);
        Assert.Equal(expectedDirectZ.Cross(expectedX), pose.YAxis);
        var f = pose.ToFrame(secondary.Position);
        // Rotating-pulsating coordinates normalize separation to one.
        Assert.Equal(5e8, pose.Scale, 3);
        Assert.Equal(1.0, f.X, 12);
        Assert.Equal(0.0, f.Y, 12);
        Assert.Equal(0.0, f.Z, 12);
        // +Z comes from relative angular momentum.
        var expectedZ = new Vector3d(0.4 * Math.Sqrt(2), -0.3 * Math.Sqrt(2), 0.5 * Math.Sqrt(2));
        Assert.Equal(0.0, (pose.ZAxis - expectedZ).Length(), 12);
    }

    [Fact]
    public void Secondary_sits_at_unit_x_with_scale_the_separation()
    {
        // The secondary is normalized to (1, 0, 0); separation is stored in Scale.
        double t = 7.3e5;
        var pose = FrameKernel.Rotating(PrimaryAt(t), SecondaryAt(t));
        Assert.Equal(Radius, pose.Scale, 3);
        var f = pose.ToFrame(SecondaryAt(t).Position);
        Assert.Equal(1.0, f.X, 12);
        Assert.Equal(0.0, f.Y, 12);
        Assert.Equal(0.0, f.Z, 12);
    }

    [Fact]
    public void L4_point_is_frame_fixed_on_a_circular_orbit()
    {
        var f0 = FrameKernel.Rotating(PrimaryAt(0), SecondaryAt(0)).ToFrame(L4At(0));
        var f1 = FrameKernel.Rotating(PrimaryAt(1e6), SecondaryAt(1e6)).ToFrame(L4At(1e6));
        // L4 remains fixed in separation-normalized coordinates.
        Assert.Equal(0.0, (f1 - f0).Length(), 3);
    }

    [Fact]
    public void ToFrame_FromFrame_round_trips()
    {
        var pose = FrameKernel.Rotating(PrimaryAt(12345), SecondaryAt(12345));
        var p = new Vector3d(1.1e8, -2.2e8, 3.3e7);
        Assert.Equal(0.0, (pose.FromFrame(pose.ToFrame(p)) - p).Length(), 6);
    }

    [Fact]
    public void Inertial_pose_keeps_identity_axes_and_rides_the_body()
    {
        var body = new StateVector(new Vector3d(5e10, -3e10, 1e9), new Vector3d(1e4, 2e4, -3e3));
        var pose = FrameKernel.Inertial(body);
        Assert.Equal(body.Position, pose.Origin);
        Assert.Equal(new Vector3d(1, 0, 0), pose.XAxis);
        Assert.Equal(new Vector3d(0, 1, 0), pose.YAxis);
        Assert.Equal(new Vector3d(0, 0, 1), pose.ZAxis);
        Assert.Equal(0.0, (pose.ToFrame(new Vector3d(5e10 + 7, -3e10, 1e9)) - new Vector3d(7, 0, 0)).Length(), 9);
    }

    [Fact]
    public void Reembed_is_identity_when_sample_and_now_poses_coincide()
    {
        var pose = FrameKernel.Rotating(PrimaryAt(500), SecondaryAt(500));
        var p = L4At(500);
        Assert.Equal(0.0, (FrameKernel.Reembed(pose, pose, p) - p).Length(), 6);
    }

    [Fact]
    public void Reembed_maps_a_frame_fixed_trajectory_to_its_current_position()
    {
        // Re-embedding a co-rotating point must recover its world position at t2.
        double t1 = 2e5, t2 = 9e5;
        var pose1 = FrameKernel.Rotating(PrimaryAt(t1), SecondaryAt(t1));
        var pose2 = FrameKernel.Rotating(PrimaryAt(t2), SecondaryAt(t2));
        Assert.Equal(0.0, (FrameKernel.Reembed(pose1, pose2, L4At(t1)) - L4At(t2)).Length(), 3);
    }

    [Fact]
    public void Reembedded_circular_orbit_of_the_secondary_collapses_toward_a_point()
    {
        // The secondary's trajectory collapses to its current pinned position.
        double tNow = 4e5;
        var nowPose = FrameKernel.Rotating(PrimaryAt(tNow), SecondaryAt(tNow));
        for (int i = 0; i < 8; i++)
        {
            double t = i * 2.5e5;
            var samplePose = FrameKernel.Rotating(PrimaryAt(t), SecondaryAt(t));
            var image = FrameKernel.Reembed(samplePose, nowPose, SecondaryAt(t).Position);
            Assert.Equal(0.0, (image - SecondaryAt(tNow).Position).Length(), 3);
        }
    }

    // Eccentric Kepler pair parameterized by eccentric anomaly.

    private const double EccA = 384_400_000.0;
    private const double Ecc = 0.05;

    /// <summary>Exact Kepler state at eccentric anomaly E (orbit in the XY plane,
    /// focus at the origin): r = a(cos E - e, sqrt(1-e^2) sin E, 0), and velocity from
    /// dE/dt = n / (1 - e cos E).</summary>
    private static StateVector EccSecondaryAt(double eccentricAnomaly)
    {
        double cosE = Math.Cos(eccentricAnomaly), sinE = Math.Sin(eccentricAnomaly);
        double b = EccA * Math.Sqrt(1 - Ecc * Ecc);
        double n = Math.Sqrt(Mu / (EccA * EccA * EccA));
        double eDot = n / (1 - Ecc * cosE);
        return new StateVector(
            new Vector3d(EccA * (cosE - Ecc), b * sinE, 0),
            new Vector3d(-EccA * sinE * eDot, b * cosE * eDot, 0));
    }

    [Fact]
    public void Eccentric_secondary_reembeds_onto_its_current_position_at_every_anomaly()
    {
        // Rotating-pulsating poses pin an eccentric secondary at every anomaly.
        var primary = new StateVector(Vector3d.Zero, Vector3d.Zero);
        double eNow = 0.9;
        var nowPose = FrameKernel.Rotating(primary, EccSecondaryAt(eNow));
        foreach (double e in new[] { 0.0, Math.PI / 4, Math.PI / 2, Math.PI, 4.0, 5.5 })
        {
            var sampled = EccSecondaryAt(e);
            var samplePose = FrameKernel.Rotating(primary, sampled);
            // Ensure each sample has a materially different separation.
            Assert.True(Math.Abs(samplePose.Scale - nowPose.Scale) > 1e6,
                $"separation at E={e} too close to now's to discriminate pulsation");
            var image = FrameKernel.Reembed(samplePose, nowPose, sampled.Position);
            Assert.Equal(0.0, (image - EccSecondaryAt(eNow).Position).Length(), 3);
        }
        var somePose = FrameKernel.Rotating(primary, EccSecondaryAt(2.0));
        Assert.Equal(0.0,
            (FrameKernel.Reembed(somePose, nowPose, primary.Position) - primary.Position).Length(), 6);
    }

    [Fact]
    public void Midpoint_between_the_bodies_reembeds_to_the_current_midpoint()
    {
        // The normalized midpoint maps to the current dimensional midpoint.
        var primary = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var s1 = EccSecondaryAt(0.3);
        var s2 = EccSecondaryAt(3.7);
        var image = FrameKernel.Reembed(
            FrameKernel.Rotating(primary, s1), FrameKernel.Rotating(primary, s2),
            (primary.Position + s1.Position) * 0.5);
        Assert.Equal(0.0, (image - (primary.Position + s2.Position) * 0.5).Length(), 3);
    }

    [Fact]
    public void ToFrame_FromFrame_round_trips_with_nonunit_scale()
    {
        var pose = FrameKernel.Rotating(
            new StateVector(new Vector3d(1e10, -2e10, 3e9), new Vector3d(1e3, -2e3, 500)),
            EccSecondaryAt(1.3) with { Position = EccSecondaryAt(1.3).Position + new Vector3d(1e10, -2e10, 3e9) });
        Assert.True(pose.Scale > 1e8); // genuinely non-unit
        var p = new Vector3d(1.1e8, -2.2e8, 3.3e7);
        // Tolerance covers divide-then-multiply round-off at the translated origin.
        Assert.Equal(0.0, (pose.FromFrame(pose.ToFrame(p)) - p).Length(), 4);
        var f = new Vector3d(0.25, -1.5, 0.75);
        Assert.Equal(0.0, (pose.ToFrame(pose.FromFrame(f)) - f).Length(), 12);
    }

    [Fact]
    public void Four_argument_pose_defaults_to_rigid_scale_one()
    {
        // Four-argument construction defines a rigid, unit-scale pose.
        var pose = new FramePose(new Vector3d(1, 2, 3),
            new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, 0, 1));
        Assert.Equal(1.0, pose.Scale);
        Assert.Equal(0.0, (pose.ToFrame(new Vector3d(8, 2, 3)) - new Vector3d(7, 0, 0)).Length(), 12);
    }

    [Fact]
    public void Surface_and_inertial_factories_keep_scale_one()
    {
        // Body-centred inertial and surface poses remain rigid.
        var body = new StateVector(new Vector3d(5e10, -3e10, 1e9), new Vector3d(1e4, 2e4, -3e3));
        Assert.Equal(1.0, FrameKernel.Inertial(body).Scale);
        Assert.Equal(1.0, FrameKernel.Surface(body, TiltedSpin(), 1e6 + 3600).Scale);
    }

    [Fact]
    public void Inertial_frame_reembedding_anchors_samples_to_the_origin_body()
    {
        // A body-relative stationary point retains its world offset.
        var bodyAt0 = new StateVector(new Vector3d(1e9, 0, 0), new Vector3d(0, 3e4, 0));
        var bodyAt1 = new StateVector(new Vector3d(1e9, 3e8, 0), new Vector3d(0, 3e4, 0));
        var sample = bodyAt0.Position + new Vector3d(1e5, 0, 0);
        var image = FrameKernel.Reembed(
            FrameKernel.Inertial(bodyAt0), FrameKernel.Inertial(bodyAt1), sample);
        Assert.Equal(0.0, (image - (bodyAt1.Position + new Vector3d(1e5, 0, 0))).Length(), 6);
    }

    [Fact]
    public void Rotating_throws_on_degenerate_geometry()
    {
        var still = new StateVector(new Vector3d(1e8, 0, 0), Vector3d.Zero);
        var coincident = Assert.Throws<ArgumentException>(() =>
            FrameKernel.Rotating(new StateVector(new Vector3d(1e8, 0, 0), Vector3d.Zero), still));
        Assert.Contains("coincide", coincident.Message);
        // Pure radial motion has zero angular momentum.
        var radial = Assert.Throws<ArgumentException>(() => FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(1e8, 0, 0), new Vector3d(100, 0, 0))));
        Assert.Contains("angular momentum", radial.Message);
    }

    [Fact]
    public void Rotating_rejects_non_finite_input_state_components()
    {
        var primary = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var secondary = new StateVector(new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));

        foreach (double invalid in new[]
                 {
                     double.NaN, double.PositiveInfinity, double.NegativeInfinity,
                 })
        {
            StateVector[] invalidPrimaries =
            [
                primary with { Position = new Vector3d(invalid, 0, 0) },
                primary with { Velocity = new Vector3d(0, invalid, 0) },
            ];
            StateVector[] invalidSecondaries =
            [
                secondary with { Position = new Vector3d(1, 0, invalid) },
                secondary with { Velocity = new Vector3d(invalid, 1, 0) },
            ];

            foreach (var candidate in invalidPrimaries)
            {
                var failure = Assert.Throws<ArgumentException>(
                    () => FrameKernel.Rotating(candidate, secondary));
                Assert.Contains("non-finite", failure.Message);
                Assert.Equal("primary", failure.ParamName);
            }
            foreach (var candidate in invalidSecondaries)
            {
                var failure = Assert.Throws<ArgumentException>(
                    () => FrameKernel.Rotating(primary, candidate));
                Assert.Contains("non-finite", failure.Message);
                Assert.Equal("secondary", failure.ParamName);
            }
        }
    }

    [Fact]
    public void Rotating_rejects_finite_states_whose_relative_subtraction_overflows()
    {
        var positionFailure = Assert.Throws<ArgumentException>(() => FrameKernel.Rotating(
            new StateVector(new Vector3d(-double.MaxValue, 0, 0), Vector3d.Zero),
            new StateVector(new Vector3d(double.MaxValue, 0, 0), new Vector3d(0, 1, 0))));
        Assert.Contains("relative position", positionFailure.Message);
        Assert.Contains("not finite", positionFailure.Message);

        var velocityFailure = Assert.Throws<ArgumentException>(() => FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, new Vector3d(-double.MaxValue, 0, 0)),
            new StateVector(new Vector3d(1, 0, 0),
                new Vector3d(double.MaxValue, 1, 0))));
        Assert.Contains("relative velocity", velocityFailure.Message);
        Assert.Contains("not finite", velocityFailure.Message);
    }

    [Fact]
    public void Rotating_rejects_cross_overflow_that_resolves_to_radial_motion()
    {
        // The raw products are +Infinity - +Infinity. Scaled directions expose the
        // actual collinearity instead of allowing a NaN normal into the pose.
        var radial = Assert.Throws<ArgumentException>(() => FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(1e200, 1e200, 0),
                new Vector3d(1e200, 1e200, 0))));
        Assert.Contains("angular momentum", radial.Message);
    }

    [Fact]
    public void Rotating_never_publishes_a_cancellation_corrupted_basis()
    {
        // The finite raw cross is catastrophically cancellation-corrupted for
        // this nearly radial pair (|x dot z| ~= 0.973 and |y| ~= 0.231). Even the
        // bounded direction cross cannot recover an orthonormal basis accurately, so
        // the kernel must contain the geometry instead of publishing poisoned axes.
        var failure = Assert.Throws<ArgumentException>(() => FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(
                new Vector3d(
                    1.4985669539303365e70, 1.4770398414586857e70, 5.0032497523367636e69),
                new Vector3d(
                    1.8785930860501104e70, 1.851606847933888e70, 6.2720390089148487e69))));

        Assert.Contains("basis", failure.Message);
        Assert.Contains("orthonormal", failure.Message);
    }

    [Theory]
    [InlineData(1e200)]
    [InlineData(1e-300)]
    public void Rotating_preserves_valid_extreme_orthogonal_geometry(double magnitude)
    {
        // Squaring and the raw cross respectively overflow or underflow at both scales;
        // direction-first fallback still produces the same right-handed unit basis.
        var pose = FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(magnitude, 0, 0),
                new Vector3d(0, magnitude, 0)));

        Assert.Equal(magnitude, pose.Scale);
        Assert.True(double.IsFinite(pose.Scale) && pose.Scale > 0.0);
        Assert.Equal(new Vector3d(1, 0, 0), pose.XAxis);
        Assert.Equal(new Vector3d(0, 1, 0), pose.YAxis);
        Assert.Equal(new Vector3d(0, 0, 1), pose.ZAxis);
        AssertFiniteOrthonormal(pose);
    }

    [Fact]
    public void Rotating_rejects_a_separation_whose_true_length_is_not_finite()
    {
        var failure = Assert.Throws<ArgumentException>(() => FrameKernel.Rotating(
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(double.MaxValue, double.MaxValue, 0),
                new Vector3d(0, 1, 0))));
        Assert.Contains("separation length", failure.Message);
        Assert.Contains("not finite", failure.Message);
    }

    private static void AssertFinite(Vector3d value)
    {
        Assert.True(double.IsFinite(value.X));
        Assert.True(double.IsFinite(value.Y));
        Assert.True(double.IsFinite(value.Z));
    }

    private static void AssertFiniteOrthonormal(FramePose pose)
    {
        const double tolerance = 1e-12;
        Assert.True(double.IsFinite(pose.Scale) && pose.Scale > 0.0);
        AssertFinite(pose.Origin);
        AssertFinite(pose.XAxis);
        AssertFinite(pose.YAxis);
        AssertFinite(pose.ZAxis);
        Assert.InRange(Math.Abs(pose.XAxis.LengthSquared() - 1.0), 0.0, tolerance);
        Assert.InRange(Math.Abs(pose.YAxis.LengthSquared() - 1.0), 0.0, tolerance);
        Assert.InRange(Math.Abs(pose.ZAxis.LengthSquared() - 1.0), 0.0, tolerance);
        Assert.InRange(Math.Abs(pose.XAxis.Dot(pose.YAxis)), 0.0, tolerance);
        Assert.InRange(Math.Abs(pose.YAxis.Dot(pose.ZAxis)), 0.0, tolerance);
        Assert.InRange(Math.Abs(pose.ZAxis.Dot(pose.XAxis)), 0.0, tolerance);
        Assert.InRange((pose.XAxis.Cross(pose.YAxis) - pose.ZAxis).Length(),
            0.0, tolerance);
    }

    // Body-surface fixture: a reference basis rotating at constant rate about a
    // fixed pole.

    private const double SpinRate = 7.2921159e-5; // rad/s, Earth-like
    private const double TiltRad = 23.4 * Math.PI / 180.0;

    /// <summary>Earth-like tilted spin model: pole 23.4 deg off inertial +Z, reference
    /// basis right-handed orthonormal, reference time 1e6 s.</summary>
    private static BodyRotation TiltedSpin()
    {
        var pole = new Vector3d(0, -Math.Sin(TiltRad), Math.Cos(TiltRad));
        var x = new Vector3d(1, 0, 0);
        return new BodyRotation(pole, x, pole.Cross(x), SpinRate, 1e6);
    }

    [Fact]
    public void Surface_pose_axes_stay_orthonormal_and_ride_the_body()
    {
        var spin = TiltedSpin();
        var body = new StateVector(new Vector3d(5e10, -3e10, 1e9), new Vector3d(1e4, 2e4, -3e3));
        foreach (double t in new[] { 1e6, 1e6 + 3600, 1e6 + 86400 * 30, 0.0 })
        {
            var pose = FrameKernel.Surface(body, spin, t);
            Assert.Equal(body.Position, pose.Origin);
            Assert.Equal(1.0, pose.XAxis.Length(), 12);
            Assert.Equal(1.0, pose.YAxis.Length(), 12);
            Assert.Equal(1.0, pose.ZAxis.Length(), 12);
            Assert.Equal(0.0, pose.XAxis.Dot(pose.YAxis), 12);
            Assert.Equal(0.0, (pose.XAxis.Cross(pose.YAxis) - pose.ZAxis).Length(), 12);
            // +Z remains aligned with the fixed pole.
            Assert.Equal(0.0, (pose.ZAxis - spin.PoleEcl).Length(), 12);
        }
    }

    [Fact]
    public void Surface_pose_at_the_reference_time_is_the_reference_basis()
    {
        var spin = TiltedSpin();
        var pose = FrameKernel.Surface(new StateVector(Vector3d.Zero, Vector3d.Zero), spin, spin.ReferenceTime);
        Assert.Equal(0.0, (pose.XAxis - spin.XAxisEcl).Length(), 12);
        Assert.Equal(0.0, (pose.YAxis - spin.YAxisEcl).Length(), 12);
    }

    [Fact]
    public void Surface_quarter_turn_is_right_handed_about_the_pole()
    {
        // Positive rotation about +Z maps +X to +Y after a quarter turn.
        var spin = new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), SpinRate, 0);
        double tQuarter = (Math.PI / 2) / SpinRate;
        var pose = FrameKernel.Surface(new StateVector(Vector3d.Zero, Vector3d.Zero), spin, tQuarter);
        Assert.Equal(0.0, (pose.XAxis - new Vector3d(0, 1, 0)).Length(), 9);
        Assert.Equal(0.0, (pose.YAxis - new Vector3d(-1, 0, 0)).Length(), 9);

        // About +Y, a quarter turn maps +X to -Z and -Z to -X.
        var tilted = new BodyRotation(
            new Vector3d(0, 1, 0), new Vector3d(1, 0, 0), new Vector3d(0, 0, -1), SpinRate, 0);
        var tiltedPose = FrameKernel.Surface(new StateVector(Vector3d.Zero, Vector3d.Zero), tilted, tQuarter);
        Assert.Equal(0.0, (tiltedPose.XAxis - new Vector3d(0, 0, -1)).Length(), 9);
        Assert.Equal(0.0, (tiltedPose.YAxis - new Vector3d(-1, 0, 0)).Length(), 9);
    }

    [Fact]
    public void Surface_zero_rate_keeps_the_axes_fixed_while_the_origin_rides()
    {
        // Zero spin produces a fixed-orientation frame.
        var spin = new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), 0.0, 0);
        var pose = FrameKernel.Surface(
            new StateVector(new Vector3d(7, 8, 9), Vector3d.Zero), spin, 1e9);
        Assert.Equal(new Vector3d(7, 8, 9), pose.Origin);
        Assert.Equal(0.0, (pose.XAxis - new Vector3d(1, 0, 0)).Length(), 12);
    }

    [Fact]
    public void Surface_reembed_keeps_a_surface_fixed_point_attached()
    {
        // Re-embedding a surface-fixed point recovers its world position at t2.
        var spin = TiltedSpin();
        double t1 = spin.ReferenceTime + 2e4, t2 = spin.ReferenceTime + 9e4;
        var bodyAt1 = new StateVector(new Vector3d(1e9, 2e9, -3e8), Vector3d.Zero);
        var bodyAt2 = new StateVector(new Vector3d(1.1e9, 1.9e9, -2.8e8), Vector3d.Zero);
        var pose1 = FrameKernel.Surface(bodyAt1, spin, t1);
        var pose2 = FrameKernel.Surface(bodyAt2, spin, t2);
        var mountainFrame = new Vector3d(4e6, 2e6, 5e6); // fixed body coordinates
        var image = FrameKernel.Reembed(pose1, pose2, pose1.FromFrame(mountainFrame));
        Assert.Equal(0.0, (image - pose2.FromFrame(mountainFrame)).Length(), 6);
    }

    [Fact]
    public void Surface_throws_on_an_exact_zero_pole_only()
    {
        var body = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var zeroPole = new BodyRotation(
            Vector3d.Zero, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), SpinRate, 0);
        var thrown = Assert.Throws<ArgumentException>(() => FrameKernel.Surface(body, zeroPole, 100));
        Assert.Contains("pole", thrown.Message);
        // A tiny nonzero pole remains finite; live validation owns rejection.
        var tinyPole = new BodyRotation(
            new Vector3d(0, 0, 1e-12), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), SpinRate, 0);
        var pose = FrameKernel.Surface(body, tinyPole, 100);
        Assert.True(double.IsFinite(pose.XAxis.X) && double.IsFinite(pose.YAxis.Y));
    }
}

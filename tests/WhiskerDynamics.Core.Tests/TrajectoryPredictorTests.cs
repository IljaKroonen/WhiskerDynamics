using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class TrajectoryPredictorTests
{
    private static GravityModel ZeroGravity() => new(new Ephemerides([]));

    private static (GravityModel gravity, StateVector leo) SunOnlySystem()
    {
        var bodies = AstronomicalsParser.ParseFile(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"));
        var eph = new Ephemerides(bodies);
        var gravity = new GravityModel(eph, [eph["Sol"]]);
        double r = 1.4e11;
        double v = Math.Sqrt(eph["Sol"].Mu / r);
        return (gravity, new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0)));
    }

    [Fact]
    public void StateAt_matches_direct_propagation()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        double t = 90 * 86400;
        var fromPredictor = predictor.StateAt(t);
        var direct = DormandPrince54.Propagate((tt, s) => gravity.AccelerationAt(s.Position, tt), y0, 0, t);
        Assert.True((fromPredictor.Position - direct.Position).Length() < 1000.0,
            $"cache/direct disagreement {(fromPredictor.Position - direct.Position).Length()} m");
    }

    [Fact]
    public void StateAt_between_nodes_interpolates_smoothly()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        predictor.ExtendTo(10 * 86400);
        var n = predictor.Nodes;
        Assert.True(n.Count >= 3, "expected multiple accepted steps");
        double tMid = (n[1].Time + n[2].Time) / 2;
        var interpolated = predictor.StateAt(tMid);
        var direct = DormandPrince54.Propagate((tt, s) => gravity.AccelerationAt(s.Position, tt), y0, 0, tMid);
        Assert.True((interpolated.Position - direct.Position).Length() < 1000.0,
            $"interpolation error {(interpolated.Position - direct.Position).Length()} m");
    }

    [Fact]
    public void Continuous_extension_reports_monotone_accepted_step_progress()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        var progress = new List<double>();

        predictor.ExtendTo(10 * 86400, progress.Add);

        Assert.NotEmpty(progress);
        Assert.Equal(predictor.Horizon, progress[^1]);
        for (int i = 1; i < progress.Count; i++)
            Assert.True(progress[i] > progress[i - 1]);
    }

    [Fact]
    public void StateAt_before_impulse_uses_the_exact_pre_impulse_endpoint_state()
    {
        var predictor = new TrajectoryPredictor(
            ZeroGravity(),
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            0);
        predictor.ExtendTo(1000);

        const double queryTime = 680;
        Assert.DoesNotContain(predictor.Nodes, node => node.Time == queryTime);
        Assert.Equal(1000, predictor.Nodes.First(node => node.Time > queryTime).Time);

        var beforeImpulse = predictor.StateAt(queryTime);
        Assert.Equal(1000, predictor.Horizon);

        var deltaV = new Vector3d(1000, 0, 0);
        predictor.AddImpulse(1000, deltaV);
        predictor.ExtendTo(1000);

        Assert.Equal(beforeImpulse, predictor.StateAt(queryTime));
        Assert.Equal(deltaV, predictor.Nodes[^1].State.Velocity);
    }

    [Fact]
    public void StateAt_horizon_applies_pending_impulse_after_historical_query()
    {
        var initialVelocity = new Vector3d(10, 20, 30);
        var predictor = new TrajectoryPredictor(
            ZeroGravity(),
            new StateVector(Vector3d.Zero, initialVelocity),
            0);
        var preBurnTip = predictor.StateAt(1000);
        var deltaV = new Vector3d(1000, 0, 0);
        predictor.AddImpulse(1000, deltaV);

        _ = predictor.StateAt(999);
        Assert.Equal(preBurnTip, predictor.Nodes[^1].State);

        Assert.Equal(initialVelocity + deltaV, predictor.StateAt(1000).Velocity);
    }

    [Fact]
    public void Impulse_changes_trajectory_only_after_burn_time()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        double tBurn = 30 * 86400;
        var before = predictor.StateAt(tBurn - 86400);
        predictor.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        Assert.Equal(before.Position, predictor.StateAt(tBurn - 86400).Position);
        var atBurnPlus = predictor.StateAt(tBurn + 86400);
        var noBurn = new TrajectoryPredictor(gravity, y0, 0).StateAt(tBurn + 86400);
        Assert.True((atBurnPlus.Position - noBurn.Position).Length() > 1e6,
            "impulse had no downstream effect");
    }

    [Fact]
    public void Query_before_start_throws()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 100.0);
        Assert.Throws<ArgumentOutOfRangeException>(() => predictor.StateAt(99.0));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Non_finite_public_times_are_rejected_immediately(double invalid)
    {
        var (gravity, y0) = SunOnlySystem();
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new TrajectoryPredictor(gravity, y0, invalid));

        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => predictor.ExtendTo(invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => predictor.StateAt(invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => predictor.PruneBefore(invalid));
        Assert.Equal(0, predictor.Horizon);
    }

    [Fact]
    public void Impulse_before_start_throws()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 100.0);
        Assert.Throws<ArgumentOutOfRangeException>(() => predictor.AddImpulse(50.0, new Vector3d(1, 0, 0)));
    }

    [Fact]
    public void Duplicate_impulse_time_throws()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        predictor.AddImpulse(1000.0, new Vector3d(1, 0, 0));
        Assert.Throws<ArgumentException>(() => predictor.AddImpulse(1000.0, new Vector3d(2, 0, 0)));
    }

    [Fact]
    public void Impulse_added_exactly_at_horizon_is_applied_at_its_own_time()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        double tBurn = 20 * 86400;
        predictor.StateAt(tBurn); // horizon now == tBurn
        predictor.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        var reference = new TrajectoryPredictor(gravity, y0, 0);
        reference.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        var a = predictor.StateAt(tBurn + 10 * 86400);
        var b = reference.StateAt(tBurn + 10 * 86400);
        Assert.True((a.Position - b.Position).Length() < 1000.0,
            "impulse at horizon was applied at the wrong time");
    }

    [Fact]
    public void PruneBefore_preserves_lookup_after_the_cut_and_drops_earlier_nodes()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        double t80 = 80 * 86400, t50 = 50 * 86400;
        var s80 = predictor.StateAt(t80);
        var s50 = predictor.StateAt(t50);
        int nodesBefore = predictor.Nodes.Count;

        predictor.PruneBefore(40 * 86400);

        Assert.True(predictor.Nodes.Count < nodesBefore);
        Assert.True(predictor.StartTime <= 40 * 86400);
        Assert.Equal(s50.Position, predictor.StateAt(t50).Position);
        Assert.Equal(s80.Position, predictor.StateAt(t80).Position);
        Assert.Throws<ArgumentOutOfRangeException>(() => predictor.StateAt(10 * 86400));
    }

    [Fact]
    public void PruneBefore_preserves_applied_impulse_effects()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        double tBurn = 20 * 86400;
        predictor.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        predictor.StateAt(40 * 86400); // extend past the burn so it is applied

        predictor.PruneBefore(30 * 86400);

        // The retained state includes the burn even after its node is pruned, so
        // extending the trajectory must not apply it again.
        Assert.True(predictor.StartTime > tBurn, "expected the prune to drop the burn node");
        var reference = new TrajectoryPredictor(gravity, y0, 0);
        reference.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        var a = predictor.StateAt(60 * 86400);
        var b = reference.StateAt(60 * 86400);
        Assert.True((a.Position - b.Position).Length() < 1000.0,
            "pruned-away burn was lost or re-applied on re-extension");
    }

    [Fact]
    public void AddImpulse_before_pruned_start_throws()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        predictor.AddImpulse(20 * 86400, new Vector3d(0, 500, 0));
        predictor.StateAt(40 * 86400);

        predictor.PruneBefore(30 * 86400);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => predictor.AddImpulse(10 * 86400, new Vector3d(1, 0, 0)));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Non_finite_impulse_time_throws(double time)
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);

        Assert.Throws<ArgumentOutOfRangeException>
            (() => predictor.AddImpulse(time, new Vector3d(1, 0, 0)));
    }

    [Fact]
    public void Pending_impulse_at_horizon_survives_adding_a_later_impulse()
    {
        var (gravity, y0) = SunOnlySystem();
        var predictor = new TrajectoryPredictor(gravity, y0, 0);
        double tBurn = 20 * 86400;
        predictor.StateAt(tBurn); // horizon lands exactly on tBurn
        predictor.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        predictor.AddImpulse(40 * 86400, new Vector3d(0, 0, 100)); // no extension in between
        var reference = new TrajectoryPredictor(gravity, y0, 0);
        reference.AddImpulse(tBurn, new Vector3d(0, 500, 0));
        reference.AddImpulse(40 * 86400, new Vector3d(0, 0, 100));
        var a = predictor.StateAt(60 * 86400);
        var b = reference.StateAt(60 * 86400);
        Assert.True((a.Position - b.Position).Length() < 1000.0,
            "first impulse was skipped after adding a second impulse");
    }

    [Fact]
    public void Out_of_order_impulses_match_chronological_insertion_bit_for_bit()
    {
        var (gravity, y0) = SunOnlySystem();
        var chronological = new TrajectoryPredictor(gravity, y0, 0);
        chronological.AddImpulse(20 * 86400, new Vector3d(0, 500, 0));
        chronological.AddImpulse(40 * 86400, new Vector3d(10, 0, 50));

        var outOfOrder = new TrajectoryPredictor(gravity, y0, 0);
        outOfOrder.AddImpulse(40 * 86400, new Vector3d(10, 0, 50));
        outOfOrder.AddImpulse(20 * 86400, new Vector3d(0, 500, 0));

        Assert.Equal(chronological.StateAt(60 * 86400), outOfOrder.StateAt(60 * 86400));
    }

    [Fact]
    public void Rearmed_later_impulse_refreshes_its_pre_impulse_interpolation_state()
    {
        var predictor = new TrajectoryPredictor(
            ZeroGravity(),
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            0);
        predictor.AddImpulse(2000, new Vector3d(0, 100, 0));
        predictor.StateAt(3000); // apply once with a zero incoming state

        predictor.AddImpulse(1000, new Vector3d(10, 0, 0));
        predictor.StateAt(3000); // truncate, then re-apply with the earlier burn included

        var beforeLaterBurn = predictor.StateAt(1680);
        Assert.True((beforeLaterBurn.Position - new Vector3d(6800, 0, 0)).Length() < 1e-9);
        Assert.True((beforeLaterBurn.Velocity - new Vector3d(10, 0, 0)).Length() < 1e-12);
        Assert.Equal(new Vector3d(10, 100, 0), predictor.StateAt(2000).Velocity);
    }
}

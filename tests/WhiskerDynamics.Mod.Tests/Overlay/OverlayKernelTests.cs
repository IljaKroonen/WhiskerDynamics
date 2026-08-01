using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Overlay;

/// <summary>Tests map-overlay burn folding and sampling with a fixed two-body fixture.
/// Burns are converted lazily in sequence; duplicate times keep the first burn and warn.</summary>
public class OverlayKernelTests
{
    private const double Mu = 3.986004418e14; // Earth-like, m^3/s^2

    private static GravityModel Gravity()
    {
        var terra = new CelestialBody { Id = "Terra", Mu = Mu };
        return new GravityModel(new Ephemerides([terra]));
    }

    /// <summary>Circular LEO-ish seed at radius 7e6 m; period ~5829 s.</summary>
    private static StateVector CircularSeed() =>
        new(new Vector3d(7e6, 0, 0), new Vector3d(0, Math.Sqrt(Mu / 7e6), 0));

    private static IntegratorOptions DisplayOptions() => new() { RelTol = 1e-9 };

    [Fact]
    public void Target_fixed_batches_disable_reuse_and_restamping_and_use_the_fixed_node_plane()
    {
        var target = new FrameSpec(FrameKind.TargetFixed, "Earth", "Station");
        var pair = new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Luna");
        var inertial = new FrameSpec(FrameKind.Inertial, "Earth", null);
        var surface = new FrameSpec(FrameKind.Surface, "Earth", null);

        Assert.False(OverlayKernel.FrameAllowsGeometryReuse(target));
        Assert.False(OverlayKernel.FrameAllowsGeometryReuse(surface));
        Assert.True(OverlayKernel.FrameAllowsGeometryReuse(pair));
        Assert.True(OverlayKernel.FrameAllowsGeometryReuse(inertial));
        Assert.False(OverlayKernel.FrameAllowsPlannedRestamp(target));
        Assert.True(OverlayKernel.FrameAllowsPlannedRestamp(surface));
        Assert.True(OverlayKernel.FrameAllowsPlannedRestamp(pair));
        Assert.True(OverlayKernel.FrameAllowsPlannedRestamp(inertial));
        Assert.True(OverlayKernel.FrameAllowsPlannedRestamp(null));
        Assert.True(OverlayKernel.FrameCoordinatesDefineNodePlane(target));
        Assert.True(OverlayKernel.FrameCoordinatesDefineNodePlane(pair));
        Assert.False(OverlayKernel.FrameCoordinatesDefineNodePlane(inertial));
        Assert.False(OverlayKernel.FrameCoordinatesDefineNodePlane(surface));
        Assert.False(OverlayKernel.FrameCoordinatesDefineNodePlane(null));
    }

    [Fact]
    public void Burn_window_excludes_start_and_past_includes_horizon()
    {
        double t0 = 1000.0, horizon = 2000.0;
        Assert.False(OverlayKernel.BurnInWindow(t0 - 1, t0, horizon));   // past burn: already flown
        Assert.False(OverlayKernel.BurnInWindow(t0, t0, horizon));       // exactly now: stock owns it
        Assert.True(OverlayKernel.BurnInWindow(t0 + 1e-6, t0, horizon)); // just ahead
        Assert.True(OverlayKernel.BurnInWindow(horizon, t0, horizon));   // at the horizon: still displayed
        Assert.False(OverlayKernel.BurnInWindow(horizon + 1e-6, t0, horizon));
    }

    [Fact]
    public void FoldBurns_applies_the_impulse_as_a_velocity_jump_at_the_exact_burn_time()
    {
        double t0 = 1000.0, tb = t0 + 2000.0, horizon = t0 + 6000.0;
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var dv = new Vector3d(10, 20, -5);
        var warns = new List<string>();
        StateVector preBurn = default;

        int applied = OverlayKernel.FoldBurns(display, [tb], t0, horizon,
            _ => { preBurn = display.StateAt(tb); return dv; }, warns.Add);

        Assert.Equal(1, applied);
        Assert.Empty(warns);
        display.StateAt(horizon); // extend past the burn (sampling does): impulse lands on the burn node
        var atBurn = display.StateAt(tb);
        Assert.Equal(preBurn.Position, atBurn.Position);      // position continuous across the burn
        Assert.Equal(preBurn.Velocity + dv, atBurn.Velocity); // node carries the POST-burn state
    }

    [Fact]
    public void FoldBurns_second_burn_sees_the_first_burns_effect_and_matches_a_manual_reference()
    {
        double t0 = 0.0, tb1 = 2000.0, tb2 = 4000.0, horizon = 8000.0;
        var dv1 = new Vector3d(0, 150, 0);
        var dv2 = new Vector3d(-40, 0, 25);

        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        Vector3d velocitySeenBySecond = default;
        int applied = OverlayKernel.FoldBurns(display, [tb1, tb2], t0, horizon,
            i =>
            {
                var state = display.StateAt(i == 0 ? tb1 : tb2);
                if (i == 1) velocitySeenBySecond = state.Velocity;
                return i == 0 ? dv1 : dv2;
            },
            _ => { });
        Assert.Equal(2, applied);
        display.StateAt(horizon);

        // Reference built with direct AddImpulse calls: identical integration targets
        // (extension stops at impulse boundaries), so states compare bitwise.
        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        reference.AddImpulse(tb1, dv1);
        Assert.Equal(reference.StateAt(tb2).Velocity, velocitySeenBySecond); // pre-burn-2 = post-burn-1 coast
        reference.AddImpulse(tb2, dv2);
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FoldSnapshotBurns_uses_each_captured_parent_in_order_across_an_soi(
        bool finiteBurn)
    {
        double t0 = 0.0, tb1 = 2000.0, tb2 = 4000.0, horizon = 8000.0;
        var burns = new[]
        {
            new PlanSnapshotBurn(tb1, new Vector3d(150, 0, 0), "Earth"),
            new PlanSnapshotBurn(tb2, new Vector3d(150, 0, 0), "Luna"),
        };
        FiniteBurnFold? finite = finiteBurn
            ? new FiniteBurnFold(new EngineScalars(1000, 3000, 2), 5.0, 32)
            : null;
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var parents = new List<string>();

        StateVector ParentAt(string parentId, double time)
        {
            parents.Add(parentId);
            if (parentId == "Earth") return default;
            Assert.Equal("Luna", parentId);
            // Give Luna a deliberately different, non-degenerate local basis at the
            // second node: relative prograde is -X instead of Earth's orbital tangent.
            StateVector vessel = display.StateAt(time);
            return new StateVector(
                vessel.Position - new Vector3d(0, 7e6, 0),
                vessel.Velocity - new Vector3d(-7500, 0, 0));
        }

        int applied = OverlayKernel.FoldSnapshotBurns(display, burns, [tb1, tb2], t0, horizon,
            "Earth", ParentAt, _ => { }, finite, out _);
        Assert.Equal(2, applied);
        Assert.Equal(["Earth", "Luna"], parents);

        // An all-Earth counterfactual must produce visibly different geometry,
        // not merely a resolver-call bookkeeping change.
        var allEarth = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldSnapshotBurns(allEarth, burns, [tb1, tb2], t0, horizon, "Earth",
            (_, _) => default, _ => { }, finite, out _);
        Assert.True((display.StateAt(horizon).Position - allEarth.StateAt(horizon).Position)
            .Length() > 1000.0);
    }

    [Fact]
    public void FoldSnapshotBurns_folds_the_predictor_basis_display_vector_when_recorded()
    {
        double t0 = 0.0, tb = 2000.0, horizon = 6000.0;
        var stockBasis = new Vector3d(150, 0, 0);
        var predictorBasis = new Vector3d(120, 60, 30);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        int applied = OverlayKernel.FoldSnapshotBurns(display,
            [new PlanSnapshotBurn(tb, stockBasis, "Earth", predictorBasis)],
            [tb], t0, horizon, "Earth", (_, _) => default, _ => { }, null, out _);
        Assert.Equal(1, applied);

        // Bitwise-equal to folding the display vector as stock components, and
        // visibly different from folding the raw stock components.
        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldSnapshotBurns(reference,
            [new PlanSnapshotBurn(tb, predictorBasis, "Earth")],
            [tb], t0, horizon, "Earth", (_, _) => default, _ => { }, null, out _);
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon));

        var rawStock = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldSnapshotBurns(rawStock,
            [new PlanSnapshotBurn(tb, stockBasis, "Earth")],
            [tb], t0, horizon, "Earth", (_, _) => default, _ => { }, null, out _);
        Assert.True((rawStock.StateAt(horizon).Position - display.StateAt(horizon).Position)
            .Length() > 1000.0);
    }

    [Fact]
    public void FoldSnapshotBurns_checks_parent_callback_before_predictor_extension()
    {
        double t0 = 0.0, burnTime = 2000.0, horizon = 4000.0;
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());

        Assert.Throws<OperationCanceledException>(() => OverlayKernel.FoldSnapshotBurns(
            display, [new PlanSnapshotBurn(burnTime, new Vector3d(10, 0, 0), "Earth")],
            [burnTime], t0, horizon, "Earth",
            (_, _) => throw new OperationCanceledException(), _ => { }, null, out _));

        Assert.Equal(t0, display.Horizon);
    }

    [Fact]
    public void FoldSnapshotBurns_rejects_mismatched_precomputed_times()
    {
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), 0, DisplayOptions());
        Assert.Throws<ArgumentException>(() => OverlayKernel.FoldSnapshotBurns(
            display, [new PlanSnapshotBurn(2000, new Vector3d(10, 0, 0), "Earth")],
            [], 0, 4000, "Earth", (_, _) => default, _ => { }, null, out _));
        Assert.Equal(0, display.Horizon);
    }

    [Fact]
    public void FoldBurns_same_identity_slot_keeps_the_first_burn_and_warns()
    {
        double t0 = 0.0, tb = 3000.0, horizon = 9000.0;
        var dv1 = new Vector3d(0, 100, 0);
        var dv2 = new Vector3d(5000, 5000, 5000); // would be catastrophic if double-applied
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var warns = new List<string>();

        int applied = OverlayKernel.FoldBurns(
            display, [tb, tb + BurnIdentityPolicy.ToleranceSeconds * 0.5], t0, horizon,
            i => i == 0 ? dv1 : dv2, warns.Add);

        Assert.Equal(1, applied);
        string warn = Assert.Single(warns);
        Assert.Contains("duplicate", warn, StringComparison.OrdinalIgnoreCase);

        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        reference.AddImpulse(tb, dv1);
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon)); // second burn fully ignored
    }

    [Fact]
    public void FoldBurns_skips_out_of_window_burns_without_evaluating_them_and_warns_on_degenerate_frames()
    {
        double t0 = 1000.0, horizon = 5000.0;
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var warns = new List<string>();
        var evaluated = new List<int>();

        // burns: at t0 (excluded), past (excluded), beyond horizon (excluded),
        // in-window with a degenerate VLF frame (converter returns null).
        int applied = OverlayKernel.FoldBurns(display, [t0, t0 - 500, horizon + 500, t0 + 2000], t0, horizon,
            i => { evaluated.Add(i); return null; }, warns.Add);

        Assert.Equal(0, applied);
        Assert.Equal([3], evaluated); // out-of-window burns never reach the converter
        string warn = Assert.Single(warns);
        Assert.Contains("degenerate", warn, StringComparison.OrdinalIgnoreCase);
        Assert.Equal(t0, display.StartTime); // display trajectory untouched by skipped burns
    }

    // ---- finite-burn estimation ----

    /// <summary>1000 kg / ve 3000 / 2 kg/s: the tests' 150 m/s burn lasts ~24.4 s —
    /// small next to the ~5829 s test period, big next to a 5 s slice target.</summary>
    private static FiniteBurnFold Finite(double mass = 1000, double sliceSeconds = 5.0, int maxSlices = 32)
        => new(new EngineScalars(mass, 3000, 2), sliceSeconds, maxSlices);

    [Fact]
    public void Finite_fold_applies_exactly_the_kernel_expansion_slices()
    {
        double t0 = 0.0, tb = 2000.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        int applied = OverlayKernel.FoldBurns(display, [tb], t0, horizon,
            _ => dv, _ => { }, Finite());
        Assert.Equal(1, applied); // burn COUNT semantics: one burn, however many slices
        display.StateAt(horizon);

        // Reference: the kernel's own expansion applied by hand — identical impulse
        // set means identical integration boundaries, so states compare bitwise.
        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var engine = new EngineScalars(1000, 3000, 2);
        var expansion = FiniteBurnKernel.Expand(tb, 150.0, engine, 5.0, 32)!;
        var direction = dv * (1.0 / 150.0);
        for (int s = 0; s < expansion.Times.Length; s++)
            reference.AddImpulse(expansion.Times[s], direction * expansion.Magnitudes[s]);
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon));

        // And the arc is REAL: it lands measurably off the impulsive prediction.
        var impulsive = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        impulsive.AddImpulse(tb, dv);
        Assert.NotEqual(impulsive.StateAt(horizon).Position, display.StateAt(horizon).Position);
    }

    [Fact]
    public void Finite_fold_total_delta_v_matches_the_impulsive_fold_exactly()
    {
        // Zero-gravity fixture: after the full window, velocity is the pure sum of
        // slices — which must equal the impulsive dv to the last bit (telescoping).
        var gravity = new GravityModel(new Ephemerides([new CelestialBody { Id = "Void", Mu = 0 }]));
        double t0 = 0.0, tb = 2000.0, horizon = 8000.0;
        var seed = new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 100, 0));
        var dv = new Vector3d(90, 0, 120); // |dv| = 150
        var display = new TrajectoryPredictor(gravity, seed, t0, DisplayOptions());
        OverlayKernel.FoldBurns(display, [tb], t0, horizon, _ => dv, _ => { }, Finite());
        var velocity = display.StateAt(horizon).Velocity - seed.Velocity;
        Assert.Equal(dv.X, velocity.X, 9);
        Assert.Equal(dv.Y, velocity.Y, 9);
        Assert.Equal(dv.Z, velocity.Z, 9);
    }

    [Fact]
    public void Finite_fold_falls_back_to_the_impulse_when_ignition_would_precede_the_fold_start()
    {
        // Node 10 s past t0, burn ~24.4 s long: the centered window's IGNITION is in
        // the past — the FC would already be burning. Honest fallback: today's impulse.
        double t0 = 0.0, tb = 10.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        int applied = OverlayKernel.FoldBurns(display, [tb], t0, horizon, _ => dv, _ => { }, Finite());
        Assert.Equal(1, applied);
        var impulsive = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        impulsive.AddImpulse(tb, dv);
        Assert.Equal(impulsive.StateAt(horizon), display.StateAt(horizon));
    }

    [Fact]
    public void Finite_fold_falls_back_to_the_impulse_when_the_cutoff_would_pass_the_horizon()
    {
        double t0 = 0.0, horizon = 8000.0, tb = horizon - 10.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        int applied = OverlayKernel.FoldBurns(display, [tb], t0, horizon, _ => dv, _ => { }, Finite());
        Assert.Equal(1, applied); // a clipped arc would silently drop delta-v
        var impulsive = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        impulsive.AddImpulse(tb, dv);
        Assert.Equal(impulsive.StateAt(horizon), display.StateAt(horizon));
    }

    [Fact]
    public void Finite_fold_overlapping_windows_keep_the_first_arc_and_impulse_the_second()
    {
        // Two 150 m/s burns 15 s apart: each window is ~24.4 s wide (±12.2 s around
        // its node), so the second burn's ignition (t≈2002.8) falls inside the
        // first's arc (cutoff t≈2012.2) — it must fall back to a node impulse rather
        // than interleave slices.
        double t0 = 0.0, tb1 = 2000.0, tb2 = 2015.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        int applied = OverlayKernel.FoldBurns(display, [tb1, tb2], t0, horizon,
            _ => dv, _ => { }, Finite());
        Assert.Equal(2, applied);

        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var engine = new EngineScalars(1000, 3000, 2);
        var expansion = FiniteBurnKernel.Expand(tb1, 150.0, engine, 5.0, 32)!;
        Assert.True(tb2 - FiniteBurnKernel.BurnDurationSeconds(150.0,
            engine with { MassKg = FiniteBurnKernel.MassAfterBurn(150.0, engine) }) / 2.0
            < expansion.IgnitionSeconds + expansion.DurationSeconds,
            "fixture must actually overlap: burn 2's ignition inside burn 1's window");
        var direction = new Vector3d(0, 1, 0);
        for (int s = 0; s < expansion.Times.Length; s++)
            reference.AddImpulse(expansion.Times[s], direction * expansion.Magnitudes[s]);
        reference.AddImpulse(tb2, dv); // the second burn: impulse fallback
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon));
    }

    [Fact]
    public void Finite_fold_chains_mass_between_burns()
    {
        // Burn 2's slice magnitudes must be computed on the ship AFTER burn 1's
        // propellant left — the reference expansion uses MassAfterBurn explicitly.
        double t0 = 0.0, tb1 = 2000.0, tb2 = 4000.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        int applied = OverlayKernel.FoldBurns(display, [tb1, tb2], t0, horizon,
            _ => dv, _ => { }, Finite());
        Assert.Equal(2, applied);

        var engine = new EngineScalars(1000, 3000, 2);
        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var direction = new Vector3d(0, 1, 0);
        var first = FiniteBurnKernel.Expand(tb1, 150.0, engine, 5.0, 32)!;
        for (int s = 0; s < first.Times.Length; s++)
            reference.AddImpulse(first.Times[s], direction * first.Magnitudes[s]);
        var lighter = engine with { MassKg = FiniteBurnKernel.MassAfterBurn(150.0, engine) };
        var second = FiniteBurnKernel.Expand(tb2, 150.0, lighter, 5.0, 32)!;
        for (int s = 0; s < second.Times.Length; s++)
            reference.AddImpulse(second.Times[s], direction * second.Magnitudes[s]);
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon));
    }

    [Fact]
    public void Finite_fold_keeps_the_first_of_two_same_slot_burns_and_warns()
    {
        // Expanded slices do not collide with the duplicate node impulse, so duplicate
        // containment must happen before expansion.
        double t0 = 0.0, tb = 2000.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        var warns = new List<string>();
        int applied = OverlayKernel.FoldBurns(
            display, [tb, tb + BurnIdentityPolicy.ToleranceSeconds * 0.5], t0, horizon,
            _ => dv, warns.Add, Finite());
        Assert.Equal(1, applied);
        string warn = Assert.Single(warns);
        Assert.Contains("duplicate", warn, StringComparison.OrdinalIgnoreCase);

        var reference = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldBurns(reference, [tb], t0, horizon, _ => dv, _ => { }, Finite());
        Assert.Equal(reference.StateAt(horizon), display.StateAt(horizon));
    }

    [Fact]
    public void Finite_fold_reports_the_earliest_burn_start_as_the_ignition_not_the_node()
    {
        // The planned batch samples from here: from the NODE the thrust arc's first
        // half would never draw and the line's first vertex would sit mid-burn.
        double t0 = 0.0, tb = 2000.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var display = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldBurns(display, [tb], t0, horizon, _ => dv, _ => { },
            Finite(), out double earliest);
        var expansion = FiniteBurnKernel.Expand(tb, 150.0, new EngineScalars(1000, 3000, 2), 5.0, 32)!;
        Assert.Equal(expansion.IgnitionSeconds, earliest, 9);

        // Impulsive (no model): the node itself; nothing applied: NaN.
        var impulsive = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldBurns(impulsive, [tb], t0, horizon, _ => dv, _ => { },
            finite: null, out earliest);
        Assert.Equal(tb, earliest);
        var empty = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldBurns(empty, [t0 - 10.0], t0, horizon, _ => dv, _ => { },
            finite: null, out earliest);
        Assert.True(double.IsNaN(earliest));
    }

    [Fact]
    public void Finite_fold_with_unusable_engine_or_no_model_is_bitwise_impulsive()
    {
        double t0 = 0.0, tb = 2000.0, horizon = 8000.0;
        var dv = new Vector3d(0, 150, 0);
        var impulsive = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        impulsive.AddImpulse(tb, dv);

        var noModel = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldBurns(noModel, [tb], t0, horizon, _ => dv, _ => { });
        Assert.Equal(impulsive.StateAt(horizon), noModel.StateAt(horizon));

        var engineless = new TrajectoryPredictor(Gravity(), CircularSeed(), t0, DisplayOptions());
        OverlayKernel.FoldBurns(engineless, [tb], t0, horizon, _ => dv, _ => { },
            new FiniteBurnFold(new EngineScalars(0, 0, 0), 5.0, 32));
        Assert.Equal(impulsive.StateAt(horizon), engineless.StateAt(horizon));
    }
}

public class BoundedTraversalTests
{
    [Fact]
    public void Suffix_injects_an_exact_first_sample_missing_from_the_bounded_set()
    {
        int start = OverlayKernel.TraversalSuffixStart(
            [0, 10, 20], 7, hasInterpolatedBoundary: false, out bool injectFirst);
        Assert.Equal(1, start);
        Assert.True(injectFirst);

        start = OverlayKernel.TraversalSuffixStart(
            [0, 10, 20], 7, hasInterpolatedBoundary: true, out injectFirst);
        Assert.Equal(1, start);
        Assert.False(injectFirst);
    }

    [Fact]
    public void Celestial_traversal_has_a_separate_stable_ceiling()
    {
        var points = Enumerable.Range(0, 10_000)
            .Select(i => new Vector3d(i, Math.Sin(i * 0.01), 0)).ToArray();
        var metrics = DecimationMetrics.For(points, CelestialCurves.MaximumTraversalPoints);
        Assert.InRange(metrics.TraversalIndices.Length, 2,
            CelestialCurves.MaximumTraversalPoints);
    }

    [Fact]
    public void Published_metrics_keep_a_sharp_corner_from_the_dense_path()
    {
        var points = new Vector3d[10_000];
        for (int i = 0; i < points.Length; i++) points[i] = new Vector3d(i, 0, 0);
        points[4_321] = new Vector3d(4_321, 10_000, 0);

        var metrics = DecimationMetrics.For(points);

        Assert.Contains(4_321, metrics.TraversalIndices);
        Assert.InRange(metrics.TraversalIndices.Length, 2,
            DecimationMetrics.MaximumTraversalPoints);
    }

    [Fact]
    public void Shape_aware_traversal_retains_a_sharp_feature_between_uniform_slots()
    {
        var significance = new double[101];
        significance[0] = significance[^1] = double.PositiveInfinity;
        significance[37] = 1_000_000.0;

        int[] indices = OverlayKernel.BoundedTraversalIndices(significance, 12);

        Assert.Contains(0, indices);
        Assert.Contains(37, indices);
        Assert.Contains(100, indices);
        Assert.True(indices.SequenceEqual(indices.Order()));
        Assert.InRange(indices.Length, 2, 12);
    }
}

/// <summary>Honest-density decimation: the staged stock buffer must be an
/// endpoint-preserving, strictly increasing SUBSET of the dense sweep (hover snapping
/// and click payloads land on drawn vertices by construction).</summary>
public class DecimateIndicesTests
{
    [Fact]
    public void Identity_when_the_sweep_fits_the_budget()
    {
        Assert.Equal([0, 1, 2], OverlayKernel.DecimateIndices(3, 2000));
        Assert.Equal(2000, OverlayKernel.DecimateIndices(2000, 2000).Length);
    }

    [Fact]
    public void Empty_sweep_decimates_to_empty()
        => Assert.Empty(OverlayKernel.DecimateIndices(0, 2000));

    [Theory]
    [InlineData(16384, 2000)]
    [InlineData(2001, 2000)]
    [InlineData(262144, 2000)]
    public void Oversized_sweeps_keep_endpoints_and_strict_monotonicity(int count, int maxPoints)
    {
        var indices = OverlayKernel.DecimateIndices(count, maxPoints);
        Assert.Equal(maxPoints, indices.Length);
        Assert.Equal(0, indices[0]);
        Assert.Equal(count - 1, indices[^1]);
        for (int i = 1; i < indices.Length; i++)
            Assert.True(indices[i] > indices[i - 1], $"indices must be strictly increasing at {i}");
    }

    [Fact]
    public void TakeAt_copies_exactly_the_indexed_subset()
        => Assert.Equal([10.0, 30.0, 50.0],
            OverlayKernel.TakeAt([10.0, 20.0, 30.0, 40.0, 50.0], [0, 2, 4]));
}

/// <summary>Screen-space draw decimation kernels: the worker-precomputed emit-filter
/// metrics — cumulative arc lengths (the density term) and per-vertex DP
/// significance (the shape term the per-frame loop compares with the deviation
/// budget).</summary>
public class ScreenSpaceEmitTests
{
    [Fact]
    public void Cumulative_arc_lengths_accumulate_chords_from_zero()
    {
        var arc = OverlayKernel.CumulativeArcLengths(
            [new Vector3d(0, 0, 0), new Vector3d(3, 4, 0), new Vector3d(3, 4, 12)]);
        Assert.Equal([0.0, 5.0, 17.0], arc);
        Assert.Empty(OverlayKernel.CumulativeArcLengths([]));
        Assert.Equal([0.0], OverlayKernel.CumulativeArcLengths([new Vector3d(9, 9, 9)]));
    }

    [Fact]
    public void Metrics_pair_arc_and_significance_over_one_array()
    {
        var metrics = DecimationMetrics.For(
            [new Vector3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(2, 0, 0)]);
        Assert.Equal([0.0, 1.0, 2.0], metrics.ArcCum);
        Assert.Equal(double.PositiveInfinity, metrics.Significance[0]);
        Assert.Equal(0.0, metrics.Significance[1]); // collinear interior
        Assert.Equal(double.PositiveInfinity, metrics.Significance[2]);
    }

    [Fact]
    public void Significance_endpoints_are_infinite_and_straight_interiors_zero()
    {
        var line = new Vector3d[64];
        for (int i = 0; i < line.Length; i++) line[i] = new Vector3d(i, 2 * i, -i);
        var sig = OverlayKernel.ChordSignificance(line);
        Assert.Equal(double.PositiveInfinity, sig[0]);
        Assert.Equal(double.PositiveInfinity, sig[^1]);
        for (int i = 1; i < line.Length - 1; i++)
            Assert.True(sig[i] < 1e-9, $"collinear interior vertex {i} must be insignificant");
        Assert.Equal([double.PositiveInfinity], OverlayKernel.ChordSignificance([new Vector3d(1, 2, 3)]));
        Assert.Empty(OverlayKernel.ChordSignificance([]));
    }

    [Fact]
    public void Significance_of_a_lone_spike_is_its_deviation()
    {
        // Straight segment with one vertex pushed 7 units off the chord: DP drops it
        // exactly when the tolerance passes its deviation.
        var points = new Vector3d[9];
        for (int i = 0; i < points.Length; i++) points[i] = new Vector3d(i, 0, 0);
        points[4] = new Vector3d(4, 7, 0);
        var sig = OverlayKernel.ChordSignificance(points);
        Assert.Equal(7.0, sig[4], 12);
        for (int i = 1; i < points.Length - 1; i++)
            if (i != 4)
                Assert.True(sig[i] <= 7.0, $"clamp: descendant {i} must not outrank its split ancestor");
    }

    /// <summary>THE decimation-stability property: for any tolerance, the polyline
    /// through the vertices whose significance meets it stays within that tolerance
    /// of every dense point — so however the per-frame emit filter's zoom-dependent
    /// subset reshuffles, the drawn line never moves more than the budget.</summary>
    [Theory]
    [InlineData(0.5)]
    [InlineData(5.0)]
    [InlineData(50.0)]
    public void Thresholded_significance_keeps_every_dense_point_within_tolerance(double tolerance)
    {
        // Deterministic multi-revolution precessing ellipse with drift.
        var points = new Vector3d[2048];
        for (int i = 0; i < points.Length; i++)
        {
            double t = i * (6 * Math.PI / (points.Length - 1));
            points[i] = new Vector3d(
                1000 * Math.Cos(t) + 40 * t, 600 * Math.Sin(t), 15 * Math.Sin(0.7 * t));
        }
        var sig = OverlayKernel.ChordSignificance(points);
        var kept = new List<int>();
        for (int i = 0; i < points.Length; i++) if (sig[i] >= tolerance) kept.Add(i);
        Assert.True(kept.Count >= 2, "endpoints are always kept");
        for (int k = 1; k < kept.Count; k++)
        {
            // Deliberate independent oracle: the point-to-segment projection is
            // re-derived inline so the property cannot go circular through the
            // production helper it verifies.
            Vector3d a = points[kept[k - 1]], b = points[kept[k]];
            var ab = b - a;
            double lengthSq = ab.LengthSquared();
            for (int i = kept[k - 1] + 1; i < kept[k]; i++)
            {
                double s = lengthSq <= 0 ? 0 : Math.Clamp((points[i] - a).Dot(ab) / lengthSq, 0, 1);
                double deviation = (points[i] - (a + ab * s)).Length();
                Assert.True(deviation <= tolerance + 1e-9,
                    $"dense point {i} deviates {deviation:G4} > tolerance {tolerance}");
            }
        }
    }

    [Fact]
    public void Significance_contains_duplicate_points()
    {
        var sig = OverlayKernel.ChordSignificance(
            [new Vector3d(0, 0, 0), new Vector3d(1, 1, 0), new Vector3d(1, 1, 0), new Vector3d(2, 0, 0)]);
        Assert.Equal(4, sig.Length);
        Assert.All(sig, s => Assert.False(double.IsNaN(s)));
    }

    /// <summary>Zero-deviation spans terminate the DP recursion outright: a long
    /// duplicate or collinear run
    /// must cost one scan, not the one-vertex-per-pass O(n^2) peel — and its
    /// interior significance IS exactly zero.</summary>
    [Fact]
    public void Degenerate_runs_stay_insignificant_without_quadratic_recursion()
    {
        var points = new Vector3d[20000];
        for (int i = 0; i < 10000; i++) points[i] = new Vector3d(5, 5, 5);        // duplicates
        for (int i = 10000; i < 20000; i++) points[i] = new Vector3d(5 + (i - 10000), 5, 5); // collinear
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var sig = OverlayKernel.ChordSignificance(points);
        watch.Stop();
        for (int i = 1; i < points.Length - 1; i++)
            Assert.True(sig[i] <= 1e-9, $"significance[{i}] = {sig[i]} on a degenerate run");
        // Generous ceiling (CI headroom): the O(n^2) peel takes seconds, one scan
        // takes microseconds.
        Assert.True(watch.ElapsedMilliseconds < 500, $"took {watch.ElapsedMilliseconds} ms");
    }

    [Fact]
    public void Non_finite_samples_never_poison_significance()
    {
        var points = new Vector3d[32];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(i, Math.Sin(i * 0.5), 0);
        points[11] = new Vector3d(double.NaN, 0, 0);
        var sig = OverlayKernel.ChordSignificance(points);
        for (int i = 1; i < points.Length - 1; i++)
            Assert.True(sig[i] >= 0 && !double.IsNaN(sig[i]),
                $"significance[{i}] = {sig[i]} must stay finite and non-negative");
    }

}

/// <summary>The dense draw's splice/fade anchor search: STRICTLY past the value
/// (stock's current-position insertion rule — a sample exactly at "now" draws behind
/// the splice vertex, the next one ahead of it).</summary>
public class UpperBoundTests
{
    private static readonly double[] Times = [0.0, 10.0, 20.0, 30.0];

    [Theory]
    [InlineData(-1.0, 0)]
    [InlineData(0.0, 1)]   // exactly at a sample: strictly-greater, so the NEXT index
    [InlineData(9.9, 1)]
    [InlineData(10.0, 2)]
    [InlineData(29.9, 3)]
    [InlineData(30.0, 4)]  // past everything: length
    [InlineData(31.0, 4)]
    public void First_index_strictly_past_the_value(double value, int expected)
        => Assert.Equal(expected, OverlayKernel.UpperBound(Times, value));

    [Fact]
    public void Empty_array_returns_zero()
        => Assert.Equal(0, OverlayKernel.UpperBound([], 5.0));

    [Fact]
    public void Future_clip_interpolates_now_and_keeps_the_first_future_sample()
    {
        Assert.Equal(new OverlayKernel.FutureClip(1, 2, 0.5),
            OverlayKernel.FutureClipAt(Times, 15.0));
        Assert.Equal(new OverlayKernel.FutureClip(1, 2, 0.0),
            OverlayKernel.FutureClipAt(Times, 10.0));
        Assert.Equal(new OverlayKernel.FutureClip(0, 0, 0.0),
            OverlayKernel.FutureClipAt(Times, -1.0));
        Assert.Null(OverlayKernel.FutureClipAt(Times, 30.0));
    }
}

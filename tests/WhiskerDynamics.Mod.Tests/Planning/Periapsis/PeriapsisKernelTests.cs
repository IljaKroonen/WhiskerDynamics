using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Planning.Periapsis;

/// <summary>The KSA-free periapsis-optimizer rules: the frame-driven target-body
/// choice, golden-section refinement of a sampled closest approach, the 1-D
/// prograde solve (expanding two-sided probe + regula-falsi/bisection bracket),
/// the shared first-periapsis scan, and the 4-D minimum-delta-v compass search.
/// Only the predictor fold itself is KSA-bound and exercised in play; everything
/// decision/solve-shaped pins offline here.</summary>
public class PeriapsisKernelTests
{
    // ---- Target body: the map display frame's relevant body.

    [Fact]
    public void No_active_frame_targets_the_orbit_parent()
    {
        Assert.Equal("Earth", PeriapsisKernel.TargetBodyId(null, "Earth"));
        Assert.Null(PeriapsisKernel.TargetBodyId(null, null));
    }

    [Fact]
    public void Body_centred_inertial_targets_the_centre_body()
    {
        var sol = new FrameSpec(FrameKind.Inertial, "Sol", null);
        Assert.Equal("Sol", PeriapsisKernel.TargetBodyId(sol, "Earth"));
    }

    [Fact]
    public void Two_body_fixed_targets_the_pair_primary_the_transfer_target()
    {
        // The catalog builds pairs child-first: "Luna-Earth Fixed" = (Luna, Earth).
        var lunaEarth = new FrameSpec(FrameKind.TwoBodyFixed, "Luna", "Earth");
        Assert.Equal("Luna", PeriapsisKernel.TargetBodyId(lunaEarth, "Earth"));
    }

    [Fact]
    public void Surface_frame_targets_its_body()
    {
        var lunaSurface = new FrameSpec(FrameKind.Surface, "Luna", null);
        Assert.Equal("Luna", PeriapsisKernel.TargetBodyId(lunaSurface, "Earth"));
    }

    [Fact]
    public void Target_fixed_frame_optimizes_about_its_primary_body()
    {
        var target = new FrameSpec(FrameKind.TargetFixed, "Earth", "Rendezvous Target");
        Assert.Equal("Earth", PeriapsisKernel.TargetBodyId(target, "Mars"));
        Assert.Equal("Earth", PeriapsisKernel.TargetBodyId(target, null));
    }

    // ---- Golden-section minimum refinement.

    [Fact]
    public void Refine_minimum_finds_a_parabola_vertex()
    {
        var (time, distance) = PeriapsisKernel.RefineMinimum(
            t => 3.0 + (t - 42.0) * (t - 42.0), 0.0, 100.0);
        Assert.Equal(42.0, time, precision: 2);
        Assert.Equal(3.0, distance, precision: 3);
    }

    [Fact]
    public void Refine_minimum_respects_the_bracket()
    {
        // Minimum outside the bracket: converges to the nearer edge, never escapes.
        var (time, _) = PeriapsisKernel.RefineMinimum(t => (t - 200.0) * (t - 200.0), 0.0, 100.0);
        Assert.InRange(time, 99.0, 100.0);
    }

    // ---- The 1-D solve.

    [Fact]
    public void Solve_hits_a_monotone_target_within_tolerance()
    {
        // Pe rises 300 m per m/s of prograde around 7e6 m (LEO-raise-like slope).
        double Pe(double x) => 7.0e6 + 300.0 * x;
        var solved = PeriapsisKernel.SolveForTarget(x => Pe(x), x0: 0.0, target: 7.1e6,
            probeStep: 1.0, maxOffset: 4096.0, tolerance: 100.0);
        Assert.NotNull(solved);
        Assert.Equal(7.1e6, solved.Value.Achieved, tolerance: 100.0);
        Assert.Equal(1e5 / 300.0, solved.Value.X, tolerance: 1.0);
    }

    [Fact]
    public void Solve_finds_targets_on_the_negative_side_too()
    {
        double Pe(double x) => 7.0e6 + 300.0 * x; // lowering needs retrograde (x < 0)
        var solved = PeriapsisKernel.SolveForTarget(x => Pe(x), x0: 0.0, target: 6.8e6,
            probeStep: 1.0, maxOffset: 4096.0, tolerance: 100.0);
        Assert.NotNull(solved);
        Assert.True(solved.Value.X < 0);
        Assert.Equal(6.8e6, solved.Value.Achieved, tolerance: 100.0);
    }

    [Fact]
    public void Solve_searches_negative_probe_when_positive_probe_is_invalid()
    {
        var probes = new List<double>();
        double? Pe(double x)
        {
            probes.Add(x);
            return x switch
            {
                0 => 7.2e6,
                1 => null,
                -1 => 7.0e6,
                _ => throw new InvalidOperationException($"unexpected probe {x}")
            };
        }

        var solved = PeriapsisKernel.SolveForTarget(Pe, x0: 0, target: 7.0e6,
            probeStep: 1, maxOffset: 16, tolerance: 1);

        Assert.Equal((-1, 7.0e6), solved);
        Assert.Equal([0, 1, -1], probes);
    }

    [Fact]
    public void Solve_searches_other_direction_when_an_expanding_probe_is_invalid()
    {
        var probes = new List<double>();
        double? Pe(double x)
        {
            probes.Add(x);
            return x switch
            {
                0 => 10,
                1 => 9,
                -1 => 11,
                2 => null,
                -2 => 5,
                _ => throw new InvalidOperationException($"unexpected probe {x}")
            };
        }

        var solved = PeriapsisKernel.SolveForTarget(Pe, x0: 0, target: 5,
            probeStep: 1, maxOffset: 16, tolerance: 0.1);

        Assert.Equal((-2, 5), solved);
        Assert.Equal([0, 1, -1, 2, -2], probes);
    }

    [Fact]
    public void Solve_cancellation_after_an_invalid_probe_aborts_before_the_other_direction()
    {
        bool cancelled = false;
        var probes = new List<double>();
        double? Pe(double x)
        {
            probes.Add(x);
            if (x == 0) return 7.2e6;
            cancelled = true;
            return null;
        }

        var solved = PeriapsisKernel.SolveForTarget(Pe, x0: 0, target: 7.0e6,
            probeStep: 1, maxOffset: 16, tolerance: 1, cancelled: () => cancelled);

        Assert.Null(solved);
        Assert.Equal([0, 1], probes);
    }

    [Fact]
    public void Solve_invalid_baseline_aborts_before_directional_probes()
    {
        var probes = new List<double>();
        double? Pe(double x)
        {
            probes.Add(x);
            return null;
        }

        var solved = PeriapsisKernel.SolveForTarget(Pe, x0: 0, target: 7.0e6,
            probeStep: 1, maxOffset: 16, tolerance: 1);

        Assert.Null(solved);
        Assert.Equal([0], probes);
    }

    [Fact]
    public void Solve_returns_the_start_when_it_already_meets_the_target()
    {
        var solved = PeriapsisKernel.SolveForTarget(_ => 7.0e6, x0: 12.5, target: 7.0e6 + 50.0,
            probeStep: 1.0, maxOffset: 4096.0, tolerance: 100.0);
        Assert.Equal((12.5, 7.0e6), solved);
    }

    [Fact]
    public void Solve_handles_a_nonlinear_steep_objective()
    {
        // Steep and convex near the body, like a real Pe-vs-prograde curve.
        double Pe(double x) => 6.6e6 + 2.0e4 * x + 50.0 * x * x;
        var solved = PeriapsisKernel.SolveForTarget(x => Pe(x), x0: 0.0, target: 7.5e6,
            probeStep: 1.0, maxOffset: 4096.0, tolerance: 100.0);
        Assert.NotNull(solved);
        Assert.Equal(7.5e6, Pe(solved.Value.X), tolerance: 100.0);
    }

    [Fact]
    public void Unreachable_targets_yield_null_not_a_wild_answer()
    {
        // Bounded objective: no dv within the probe range reaches the target.
        double Pe(double x) => 7.0e6 + 1.0e3 * Math.Tanh(x / 100.0);
        Assert.Null(PeriapsisKernel.SolveForTarget(x => Pe(x), x0: 0.0, target: 8.0e6,
            probeStep: 1.0, maxOffset: 4096.0, tolerance: 100.0));
    }

    [Fact]
    public void Invalid_probes_with_no_searchable_direction_yield_null()
    {
        int calls = 0;
        double? Flaky(double x) => ++calls < 3 ? 7.0e6 + 300.0 * x : null;
        Assert.Null(PeriapsisKernel.SolveForTarget(Flaky, x0: 0.0, target: 8.0e6,
            probeStep: 1.0, maxOffset: 4096.0, tolerance: 100.0));
    }

    // ---- First-periapsis scan (the shared objective tail).

    [Fact]
    public void Scan_finds_and_refines_the_first_interior_minimum()
    {
        // Two dips (8e6 at t=250, deeper 5e6 at t=750): the FIRST periapsis wins,
        // refined below the coarse sampling's floor.
        double Distance(double t) =>
            t < 500 ? 8.0e6 + 40.0 * (t - 250.0) * (t - 250.0)
                    : 5.0e6 + 40.0 * (t - 750.0) * (t - 750.0);
        var (periapsis, interior) = PeriapsisKernel.ScanFirstPeriapsis(Distance, 0.0, 1000.0);
        Assert.True(interior);
        Assert.Equal(8.0e6, periapsis, 8.0e6 * 1e-6);
    }

    [Fact]
    public void Scan_flags_a_monotone_window_as_edge_not_periapsis()
    {
        var (periapsis, interior) = PeriapsisKernel.ScanFirstPeriapsis(
            t => 9.0e6 - 1000.0 * t, 0.0, 1000.0);
        Assert.False(interior);
        Assert.Equal(8.0e6, periapsis, 1.0); // the closest sampled approach: the far edge
    }

    // ---- 4-D minimum-delta-v outer search (compass/pattern over time, normal,
    // outward; the inner prograde solve is the caller's constraint projection —
    // these tests model it analytically).

    /// <summary>Synthetic constraint projection: the prograde needed to hit the
    /// target is a bowl over (time, normal, outward) with its minimum at
    /// (1000, 0, 0) where 10 m/s of prograde suffices — total |dv| is minimized
    /// exactly there. The hint is unused (the analytic model has one root).</summary>
    private static (double Prograde, double Achieved)? Bowl(
        double t, double n, double o, double hint) =>
        (10.0 + 1e-4 * (t - 1000.0) * (t - 1000.0) + 0.5 * n * n + 0.5 * o * o, 7.0e6);

    [Fact]
    public void Minimize_walks_to_the_cheapest_time_and_plane()
    {
        var best = PeriapsisKernel.MinimizeDeltaV(Bowl,
            time0: 400.0, normal0: 6.0, outward0: -4.0, timeLo: 0.0, timeHi: 2000.0,
            timeStep: 200.0, dvStep: 8.0, timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: () => false);
        Assert.NotNull(best);
        Assert.Equal(1000.0, best!.TimeSeconds, 1.0);
        Assert.Equal(0.0, best.Normal, 1);
        Assert.Equal(0.0, best.Outward, 1);
        Assert.Equal(10.0, best.Magnitude, 2);
    }

    [Fact]
    public void Minimize_hints_with_the_best_points_prograde_never_a_rejected_probes()
    {
        // The hint contract: NaN on the very first solve (the caller substitutes
        // its own baseline), thereafter always the CURRENT BEST point's prograde —
        // rejected probes must not leak their solution into later hints.
        var hints = new List<double>();
        (double Prograde, double Achieved)? Tracking(double t, double n, double o, double hint)
        {
            hints.Add(hint);
            return Bowl(t, n, o, hint);
        }
        var best = PeriapsisKernel.MinimizeDeltaV(Tracking,
            time0: 400.0, normal0: 6.0, outward0: 0.0, timeLo: 0.0, timeHi: 2000.0,
            timeStep: 200.0, dvStep: 8.0, timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: () => false);
        Assert.NotNull(best);
        Assert.True(double.IsNaN(hints[0]), "first hint must be NaN (caller's baseline)");
        // Every later hint is a prograde the bowl actually RETURNED for an accepted
        // best point — bowl progrades are >= 10, so no fabricated values appear.
        Assert.All(hints.Skip(1), h => Assert.True(h >= 10.0 - 1e-9, $"hint {h} not a solved prograde"));
    }

    [Fact]
    public void Minimize_respects_the_time_bounds_and_clamps_the_start()
    {
        // The bowl's optimum (t=1000) and the START both lie past the ceiling: the
        // search must clamp the start into the window and never probe beyond it.
        double maxProbed = double.NegativeInfinity;
        (double, double)? Tracked(double t, double n, double o, double hint)
        {
            maxProbed = Math.Max(maxProbed, t);
            return Bowl(t, n, o, hint);
        }
        var best = PeriapsisKernel.MinimizeDeltaV(Tracked,
            time0: 900.0, normal0: 0.0, outward0: 0.0, timeLo: 0.0, timeHi: 700.0,
            timeStep: 200.0, dvStep: 8.0, timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: () => false);
        Assert.NotNull(best);
        Assert.Equal(700.0, best!.TimeSeconds, 1.0);
        Assert.True(maxProbed <= 700.0, $"probed t={maxProbed} beyond the bound");
    }

    [Fact]
    public void Minimize_treats_refused_regions_as_non_moves()
    {
        // The inner solve is refused past t=1200 (say the periapsis leaves the plan
        // window there): the search must still settle on the best REACHABLE point.
        (double, double)? Guarded(double t, double n, double o, double hint) =>
            t > 1200.0 ? null : Bowl(t, n, o, hint);
        var best = PeriapsisKernel.MinimizeDeltaV(Guarded,
            time0: 400.0, normal0: 3.0, outward0: 0.0, timeLo: 0.0, timeHi: 5000.0,
            timeStep: 300.0, dvStep: 8.0, timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: () => false);
        Assert.NotNull(best);
        Assert.Equal(1000.0, best!.TimeSeconds, 1.0);
        Assert.Equal(0.0, best.Normal, 1);
    }

    [Fact]
    public void Minimize_returns_null_when_the_start_has_no_solution()
        => Assert.Null(PeriapsisKernel.MinimizeDeltaV((t, n, o, hint) => null,
            time0: 0.0, normal0: 0.0, outward0: 0.0, timeLo: -10.0, timeHi: 10.0,
            timeStep: 1.0, dvStep: 1.0, timeStepFloor: 0.1, dvStepFloor: 0.1,
            cancelled: () => false));

    [Fact]
    public void Minimize_cancel_returns_the_best_point_so_far()
    {
        int evaluations = 0;
        (double, double)? Counting(double t, double n, double o, double hint)
        {
            evaluations++;
            return Bowl(t, n, o, hint);
        }
        var best = PeriapsisKernel.MinimizeDeltaV(Counting,
            time0: 400.0, normal0: 6.0, outward0: 0.0, timeLo: 0.0, timeHi: 2000.0,
            timeStep: 200.0, dvStep: 8.0, timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: () => evaluations >= 4);
        // Cancelled almost immediately: still a valid (starting-region) point, and
        // the search stopped probing.
        Assert.NotNull(best);
        Assert.True(evaluations <= 10, $"kept evaluating after cancel ({evaluations})");
    }

    [Fact]
    public void Minimize_never_returns_a_worse_point_than_the_start()
    {
        // A ridge shape hostile to axis moves: whatever happens, the answer must
        // not regress past the starting magnitude.
        (double Prograde, double Achieved)? Ridge(double t, double n, double o, double hint) =>
            (20.0 + Math.Abs(n - o) * 3.0 + 1e-3 * Math.Abs(t - 300.0), 7.0e6);
        var start = Ridge(300.0, 2.0, 2.0, double.NaN)!.Value;
        double startMagnitude = Math.Sqrt(start.Prograde * start.Prograde + 4.0 + 4.0);
        var best = PeriapsisKernel.MinimizeDeltaV(Ridge,
            time0: 300.0, normal0: 2.0, outward0: 2.0, timeLo: 0.0, timeHi: 1000.0,
            timeStep: 100.0, dvStep: 4.0, timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: () => false);
        Assert.NotNull(best);
        Assert.True(best!.Magnitude <= startMagnitude + 1e-9,
            $"regressed: {best.Magnitude} > {startMagnitude}");
    }

    [Fact]
    public void Minimize_never_probes_below_the_step_floors()
    {
        // A tiny movable window makes the time step hit its floor rounds before the
        // dv step: time probes must clamp AT the floor while dv keeps refining, and
        // the search exits once both floors are reached.
        var probedTimes = new List<double>();
        double bestTime = 4.0;
        (double Prograde, double Achieved)? Narrow(double t, double n, double o, double hint)
        {
            if (Math.Abs(t - bestTime) > 1e-12) probedTimes.Add(t);
            return (10.0 + 0.5 * n * n + 0.5 * o * o, 7.0e6);
        }
        var best = PeriapsisKernel.MinimizeDeltaV(Narrow,
            time0: 4.0, normal0: 5.0, outward0: 0.0, timeLo: 0.0, timeHi: 8.0,
            timeStep: 2.0, dvStep: 8.0, timeStepFloor: 1.0, dvStepFloor: 0.01,
            cancelled: () => false);
        Assert.NotNull(best);
        foreach (double t in probedTimes)
            Assert.True(Math.Abs(t - best!.TimeSeconds) >= 1.0 - 1e-9
                || t is 0.0 or 8.0, // bound-clamped probes may land nearer
                $"time probe {t} below the 1 s floor around {best!.TimeSeconds}");
    }

    [Fact]
    public void Coupled_projection_hits_periapsis_and_inclination_together()
    {
        const double targetPe = 7_003_000;
        const double targetI = 0.52;
        var solved = PeriapsisKernel.SolveCoupledTargets((p, n) =>
            (7_000_000 + 1000 * (p + 2 * n), 0.5 + 0.01 * (p - n), true),
            prograde0: 0, normal0: 0, targetPe, targetI,
            probeStep: 1, maxOffset: 100, periapsisTolerance: 1,
            inclinationTolerance: 1e-6);

        Assert.NotNull(solved);
        Assert.Equal(7.0 / 3.0, solved.Value.Prograde, 5);
        Assert.Equal(1.0 / 3.0, solved.Value.Normal, 5);
        Assert.InRange(Math.Abs(solved.Value.AchievedPeriapsis - targetPe), 0, 1);
        Assert.InRange(Math.Abs(solved.Value.AchievedInclination - targetI), 0, 1e-6);
    }

    [Fact]
    public void Coupled_projection_can_reverse_prograde_for_a_high_angle_target()
    {
        var solved = PeriapsisKernel.SolveCoupledTargets((p, n) =>
            (7_000_000 + 1000 * (p + n), 2.0 + 0.01 * (p - n), true),
            prograde0: 5, normal0: 0,
            periapsisTarget: 7_000_000, inclinationTarget: 1.8,
            probeStep: 1, maxOffset: 100, periapsisTolerance: 1,
            inclinationTolerance: 1e-6);

        Assert.NotNull(solved);
        Assert.True(solved.Value.Prograde < 0);
        Assert.True(solved.Value.Normal > 0);
    }

    [Fact]
    public void Coupled_projection_rejects_a_window_edge_periapsis()
        => Assert.Null(PeriapsisKernel.SolveCoupledTargets((p, n) =>
            (7_000_000 + p, 0.5 + n, false),
            prograde0: 0, normal0: 0,
            periapsisTarget: 7_000_000, inclinationTarget: 0.5,
            probeStep: 1, maxOffset: 10, periapsisTolerance: 1,
            inclinationTolerance: 1e-6));

    [Fact]
    public void Coupled_projection_uses_a_one_sided_derivative_beside_refusal()
    {
        var solved = PeriapsisKernel.SolveCoupledTargets((p, n) => p > 0 ? null :
            (7_000_000 + 1000 * (p + n), 0.5 + 0.01 * (p - n), true),
            prograde0: 0, normal0: 0,
            periapsisTarget: 6_999_000, inclinationTarget: 0.47,
            probeStep: 1, maxOffset: 10, periapsisTolerance: 1,
            inclinationTolerance: 1e-6);

        Assert.NotNull(solved);
        Assert.Equal(-2, solved.Value.Prograde, 5);
        Assert.Equal(1, solved.Value.Normal, 5);
    }

    [Fact]
    public void Coupled_projection_refuses_an_unreachable_singular_system()
        => Assert.Null(PeriapsisKernel.SolveCoupledTargets((p, n) =>
            (7_000_000 + p + n, 0.5 + p + n, true),
            prograde0: 0, normal0: 0,
            periapsisTarget: 7_000_010, inclinationTarget: 0.4,
            probeStep: 1, maxOffset: 10, periapsisTolerance: 1e-3,
            inclinationTolerance: 1e-6));

    [Fact]
    public void Coupled_projection_stops_during_inner_iterations_when_cancelled()
    {
        int evaluations = 0;
        var solved = PeriapsisKernel.SolveCoupledTargets((p, n) =>
        {
            evaluations++;
            return (7_000_000 + p, 0.5 + n, true);
        }, prograde0: 0, normal0: 0,
            periapsisTarget: 7_100_000, inclinationTarget: 1.0,
            probeStep: 1, maxOffset: 1000, periapsisTolerance: 1,
            inclinationTolerance: 1e-6, cancelled: () => evaluations >= 2);

        Assert.Null(solved);
        Assert.Equal(2, evaluations);
    }

    [Fact]
    public void Dual_constraint_minimize_searches_only_time_and_outward()
    {
        CoupledTargetSolution? Solve(double time, double outward, double pHint, double nHint) =>
            new(2 + 0.001 * (time - 100) * (time - 100), 3,
                7_000_000, 0.5);

        var best = PeriapsisKernel.MinimizeDeltaVWithInclination(Solve,
            time0: 0, outward0: 8, timeLo: 0, timeHi: 200,
            timeStep: 50, dvStep: 4, timeStepFloor: 1, dvStepFloor: 0.01,
            cancelled: () => false);

        Assert.NotNull(best);
        Assert.Equal(100, best!.TimeSeconds, 1);
        Assert.Equal(2, best.Prograde, 2);
        Assert.Equal(3, best.Normal, 2);
        Assert.Equal(0, best.Outward, 2);
        Assert.Equal(0.5, best.AchievedInclination!.Value, 12);
    }

    [Fact]
    public void Inclination_improvement_keeps_periapsis_and_respects_the_dv_ceiling()
    {
        (double Prograde, double AchievedPeriapsis, double AchievedInclination)? Solve(
            double time, double normal, double outward, double hint) =>
            (10, 7_000_000, 1.0 - 0.1 * normal);

        var best = PeriapsisKernel.ImproveInclinationAtFixedPeriapsis(Solve,
            time0: 0, normal0: 0, outward0: 0, inclinationTarget: 0.2,
            inclinationTolerance: 1e-9,
            maxMagnitude: 13, timeLo: 0, timeHi: 0,
            timeStep: 1, dvStep: 2, timeStepFloor: 1, dvStepFloor: 0.01,
            cancelled: () => false);

        Assert.NotNull(best);
        Assert.Equal(7_000_000, best!.AchievedPeriapsis);
        Assert.Equal(0.2, best.AchievedInclination!.Value, 10);
        Assert.True(best.Magnitude <= 13);
    }

    [Fact]
    public void Inclination_improvement_rejects_an_over_ceiling_start()
    {
        var best = PeriapsisKernel.ImproveInclinationAtFixedPeriapsis(
            (time, normal, outward, hint) => (100.0, 7_000_000.0, 0.5),
            time0: 0, normal0: 0, outward0: 0, inclinationTarget: 0.2,
            inclinationTolerance: 1e-9, maxMagnitude: 20,
            timeLo: 0, timeHi: 0, timeStep: 1, dvStep: 1,
            timeStepFloor: 1, dvStepFloor: 0.01, cancelled: () => false);

        Assert.Null(best);
    }

    [Fact]
    public void Inclination_improvement_continues_past_256_improving_moves()
    {
        var best = PeriapsisKernel.ImproveInclinationAtFixedPeriapsis(
            (time, normal, outward, hint) => (10.0, 7_000_000.0, 400.0 - time),
            time0: 0, normal0: 0, outward0: 0, inclinationTarget: 0,
            inclinationTolerance: 1e-9, maxMagnitude: 20,
            timeLo: 0, timeHi: 400, timeStep: 1, dvStep: 1,
            timeStepFloor: 1, dvStepFloor: 0.01, cancelled: () => false);

        Assert.NotNull(best);
        Assert.Equal(400, best!.TimeSeconds);
        Assert.Equal(0, best.AchievedInclination);
    }

    [Fact]
    public void Inclination_cancel_returns_the_last_accepted_improvement()
    {
        int calls = 0;
        var best = PeriapsisKernel.ImproveInclinationAtFixedPeriapsis(
            (time, normal, outward, hint) =>
            {
                calls++;
                return (10.0, 7_000_000.0, 10.0 - time);
            },
            time0: 0, normal0: 0, outward0: 0, inclinationTarget: 0,
            inclinationTolerance: 1e-9, maxMagnitude: 20,
            timeLo: 0, timeHi: 10, timeStep: 1, dvStep: 1,
            timeStepFloor: 1, dvStepFloor: 0.01, cancelled: () => calls >= 2);

        Assert.NotNull(best);
        Assert.Equal(1, best!.TimeSeconds);
        Assert.Equal(2, calls);
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(0, 1, 90)]
    [InlineData(-1, 0, 180)]
    public void Inclination_uses_angular_momentum_against_the_equatorial_pole(
        double vy, double vz, double expectedDegrees)
    {
        var achieved = PeriapsisKernel.InclinationRadians(
            new Vector3d(1, 0, 0), new Vector3d(0, vy, vz), new Vector3d(0, 0, 1));

        Assert.NotNull(achieved);
        Assert.Equal(expectedDegrees, achieved.Value * 180.0 / Math.PI, 10);
    }

    [Fact]
    public void Inclination_refuses_degenerate_orbit_planes()
    {
        Assert.Null(PeriapsisKernel.InclinationRadians(
            new Vector3d(1, 0, 0), Vector3d.Zero, new Vector3d(0, 0, 1)));
        Assert.Null(PeriapsisKernel.InclinationRadians(
            new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), Vector3d.Zero));
    }

    [Theory]
    [InlineData(0, OptimizerConstraint.Normal)]
    [InlineData(45, OptimizerConstraint.Normal)]
    [InlineData(90, OptimizerConstraint.Prograde)]
    [InlineData(180, OptimizerConstraint.Prograde)]
    public void Inclination_selects_a_constraint_that_can_cross_the_target(
        double targetDegrees, OptimizerConstraint expected)
        => Assert.Equal(expected, PeriapsisKernel.ConstraintFor(
            OptimizerObjective.Inclination, targetDegrees * Math.PI / 180.0));

    [Fact]
    public void High_angle_seed_expands_both_sides_until_feasible()
    {
        var probes = new List<double>();
        double? seed = PeriapsisKernel.FindFeasibleOuter(x =>
        {
            probes.Add(x);
            return x == -4;
        }, authored: 0, probeStep: 1, maxOffset: 16, cancelled: () => false);

        Assert.Equal(-4, seed);
        Assert.Equal([0, 1, -1, 2, -2, 4, -4], probes);
    }

    [Fact]
    public void Inclination_measurement_is_after_the_finite_burn_cutoff()
    {
        var finite = new FiniteBurnExpansion([], [], DurationSeconds: 8, IgnitionSeconds: 10);
        Assert.Equal(19, PeriapsisKernel.InclinationMeasurementTime(14, finite));
        Assert.Equal(15, PeriapsisKernel.InclinationMeasurementTime(14, expansion: null));
    }

    [Fact]
    public void Tangency_solver_finds_a_zero_target_between_expanding_probes()
    {
        var solved = PeriapsisKernel.SolveForTargetIncludingTangencies(
            x => Math.Abs(x - 3.3), x0: 0, target: 0,
            probeStep: 1, maxOffset: 16, tolerance: 1e-8);

        Assert.NotNull(solved);
        Assert.Equal(3.3, solved!.Value.X, 7);
        Assert.InRange(solved.Value.Achieved, 0, 1e-8);
    }

    [Fact]
    public void Tangency_solver_refines_the_outermost_sample_interval()
    {
        var solved = PeriapsisKernel.SolveForTargetIncludingTangencies(
            x => Math.Abs(x - 15.3), x0: 0, target: 0,
            probeStep: 1, maxOffset: 16, tolerance: 1e-8);

        Assert.NotNull(solved);
        Assert.Equal(15.3, solved!.Value.X, 7);
    }

    [Fact]
    public void Minimize_with_normal_constraint_maps_components_back_to_vlf()
    {
        (double Constrained, double Achieved)? Solve(
            double time, double prograde, double outward, double hint) => (3.0, 0.5);

        var best = PeriapsisKernel.MinimizeDeltaV(Solve, OptimizerConstraint.Normal,
            time0: 10, outer0: 4, outward0: 2, timeLo: 10, timeHi: 10,
            timeStep: 1, dvStep: 2, timeStepFloor: 1, dvStepFloor: 0.01,
            cancelled: () => false);

        Assert.NotNull(best);
        Assert.Equal(3.0, best!.Normal, 12);
        Assert.Equal(0.0, best.Prograde, 12);
        Assert.Equal(0.0, best.Outward, 12);
        Assert.Equal(0.5, best.AchievedObjective, 12);
    }
}

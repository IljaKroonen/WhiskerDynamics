using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class OverlayStalenessTests
{
    [Fact]
    public void Fresh_samples_are_usable_and_fossils_are_not()
    {
        long now = 1_000_000;
        Assert.True(OverlayKernel.SamplesUsable(sampleWallMs: now - 1_000, nowWallMs: now));
        Assert.True(OverlayKernel.SamplesUsable(sampleWallMs: now, nowWallMs: now));
        Assert.False(OverlayKernel.SamplesUsable(sampleWallMs: now - 5_001, nowWallMs: now));
    }

    [Fact]
    public void Wall_clock_regression_counts_as_stale()
    {
        // TickCount64 never regresses in-process, but a sample stamped in a previous
        // session must never be restaged into a new one: the sweep nulls the buffer AND
        // the rule rejects future-stamped samples defensively.
        Assert.False(OverlayKernel.SamplesUsable(sampleWallMs: 2_000_000, nowWallMs: 1_000_000));
    }

    [Fact]
    public void Boundary_of_the_restage_window_is_inclusive()
    {
        long now = 1_000_000;
        Assert.True(OverlayKernel.SamplesUsable(sampleWallMs: now - OverlayKernel.RestageMaxAgeMs, nowWallMs: now));
    }

    [Fact]
    public void Capture_epoch_has_no_arbitrary_simulation_age_cutoff()
    {
        double captured = 10_000.0;
        Assert.True(OverlayKernel.CaptureEpochValid(
            captured, captured + OverlayKernel.ActualGeometryRecenteringSeconds));
        Assert.True(OverlayKernel.CaptureEpochValid(
            captured, captured + 100 * 365.0 * 86400.0));
    }

    [Theory]
    [InlineData(double.NaN, 1000.0)]
    [InlineData(1000.0, double.NaN)]
    [InlineData(double.PositiveInfinity, 1000.0)]
    [InlineData(1000.0, double.PositiveInfinity)]
    [InlineData(1001.0, 1000.0)]
    public void Non_finite_or_future_capture_epochs_are_unusable(
        double captured, double now)
    {
        Assert.False(OverlayKernel.CaptureEpochValid(captured, now));
    }
}

/// <summary>Overlay buffers are padded to the stock length so captured indices remain
/// valid across buffer swaps, including for escape trajectories.</summary>
public class PadToStockLengthTests
{
    [Fact]
    public void Pads_by_repeating_the_last_element_to_exactly_2000()
    {
        Assert.Equal(2000, OverlayKernel.StockPointBufferLength);
        var source = new double[] { 1.0, 2.0, 3.0 };
        var padded = OverlayKernel.PadToStockLength(source);
        Assert.Equal(2000, padded.Length);
        Assert.Equal([1.0, 2.0, 3.0], padded[..3]);
        Assert.All(padded[3..], x => Assert.Equal(3.0, x));
    }

    [Fact]
    public void A_full_length_buffer_passes_through_unchanged()
    {
        var source = new int[2000];
        Assert.Same(source, OverlayKernel.PadToStockLength(source));
    }

    [Fact]
    public void Oversized_input_throws_instead_of_silently_truncating()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => OverlayKernel.PadToStockLength(new int[2001]));
    }

    [Fact]
    public void Empty_input_throws_there_is_no_last_element_to_repeat()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => OverlayKernel.PadToStockLength(Array.Empty<int>()));
    }
}

/// <summary>Honest arcs: a celestial's sampled window is the config window
/// clamped to the rails-ahead horizon — sampling must never demand ephemerides past
/// what the worker maintains. Floor of 0.1 d guards a zeroed config. There is
/// deliberately no one-period clamp: multi-period windows are the point — under n-body
/// dynamics successive revolutions show precession, not re-trace overdraw.</summary>
public class CelestialWindowTests
{
    [Fact]
    public void Window_is_config_days_when_rails_allow()
        => Assert.Equal(10 * 86400.0, OverlayKernel.CelestialWindowSeconds(
            configDays: 10, railsAheadDays: 30));

    [Fact]
    public void Window_spans_multiple_orbital_periods_no_one_period_clamp()
        // 30 d window over Luna's 27.3 d period: the arc keeps going past one loop.
        => Assert.Equal(30 * 86400.0, OverlayKernel.CelestialWindowSeconds(
            configDays: 30, railsAheadDays: 30));

    [Fact]
    public void Rails_horizon_clamps_the_window()
        => Assert.Equal(30 * 86400.0, OverlayKernel.CelestialWindowSeconds(
            configDays: 45, railsAheadDays: 30));

    [Fact]
    public void Rails_window_quantization_is_stable_between_worker_cycles()
    {
        // Steady state: the reached horizon trails "now + target" by a sliver that
        // changes every read — the quantized value must not (it feeds the
        // planned-batch restamp identity, where any change forces a full resample).
        Assert.Equal(OverlayKernel.QuantizeRailsWindow(30 - 1e-6, 30),
            OverlayKernel.QuantizeRailsWindow(30 - 5e-6, 30));
        Assert.Equal(29.75, OverlayKernel.QuantizeRailsWindow(30 - 1e-6, 30));
        // A fully caught-up window (paused, or rails pre-extended past the target)
        // returns the config target exactly.
        Assert.Equal(30.0, OverlayKernel.QuantizeRailsWindow(30, 30));
        // During a catch-up the value steps by whole quanta, never past what the
        // rails actually reached.
        Assert.Equal(123.25, OverlayKernel.QuantizeRailsWindow(123.4, 14600));
        Assert.True(OverlayKernel.QuantizeRailsWindow(123.4, 14600) <= 123.4);
        Assert.Equal(0.0, OverlayKernel.QuantizeRailsWindow(0.1, 14600));
    }

    [Fact]
    public void Zeroed_config_floors_at_a_tenth_of_a_day()
        => Assert.Equal(0.1 * 86400.0, OverlayKernel.CelestialWindowSeconds(
            configDays: 0, railsAheadDays: 30));
}

/// <summary>The one config-to-radians derivation both drawn surfaces share.</summary>
public class SamplingThetaTests
{
    private const double Deg = Math.PI / 180.0;

    [Fact]
    public void Shipped_defaults_map_through()
    {
        double thetaMax = OverlayKernel.SamplingThetaRadians(
            ModConfig.DefaultOverlayTurnDegrees);
        Assert.Equal(0.4 * Deg, thetaMax, precision: 12);
    }

    [Fact]
    public void Out_of_range_values_clamp()
    {
        Assert.Equal(10.0 * Deg, OverlayKernel.SamplingThetaRadians(100.0),
            precision: 12);
        Assert.Equal(0.05 * Deg, OverlayKernel.SamplingThetaRadians(0.0),
            precision: 12);
    }

    [Fact]
    public void Non_finite_config_falls_back_to_the_shipped_defaults()
    {
        // A NaN bound would make every grow/halve comparison in the sampler false,
        // pinning the step at dtMin — the fallback keeps a poisoned config drawing.
        Assert.Equal(0.4 * Deg, OverlayKernel.SamplingThetaRadians(double.NaN),
            precision: 12);
        Assert.Equal(0.4 * Deg,
            OverlayKernel.SamplingThetaRadians(double.PositiveInfinity),
            precision: 12);
    }
}

/// <summary>Two-line display rules: the planned trajectory's window starts at the
/// earliest IN-WINDOW burn (before it the planned and actual paths are identical —
/// impulses change velocity, not position), and the burn-node marker interpolates the
/// drawn line via LerpBracket over the padded times array.</summary>
public class PlannedWindowTests
{
    [Fact]
    public void Planned_window_starts_at_the_earliest_in_window_burn()
        => Assert.Equal(500.0, OverlayKernel.PlannedWindowStart(
            [900.0, 500.0, 2000.0], t0: 100, horizon: 1000));

    [Fact]
    public void No_in_window_burn_means_no_planned_line()
    {
        Assert.Null(OverlayKernel.PlannedWindowStart([], t0: 100, horizon: 1000));
        // At-or-before now is stock's to execute; beyond the horizon cannot show.
        Assert.Null(OverlayKernel.PlannedWindowStart([100.0, 1500.0], t0: 100, horizon: 1000));
    }

    [Fact]
    public void Window_edges_follow_BurnInWindow_exactly()
    {
        // Strictly after t0, inclusive at the horizon — the same rule FoldBurns uses,
        // so a burn the fold applies always has a planned window and vice versa.
        Assert.Equal(1000.0, OverlayKernel.PlannedWindowStart([1000.0], t0: 100, horizon: 1000));
        Assert.Null(OverlayKernel.PlannedWindowStart([100.0], t0: 100, horizon: 1000));
    }
}

public class PlannedHorizonTests
{
    [Fact]
    public void Plan_end_caps_the_planned_line_but_not_the_actual_horizon()
    {
        const double actualHorizon = 30 * 86400.0;
        const double planEnd = 7 * 86400.0;

        Assert.Equal(planEnd, OverlayKernel.PlannedHorizonSeconds(
            actualHorizon, planEnd, actualHorizon, actualCoverageLimited: false));
    }

    [Fact]
    public void Longer_plan_uses_the_actual_requested_horizon()
        => Assert.Equal(30 * 86400.0, OverlayKernel.PlannedHorizonSeconds(
            30 * 86400.0, 60 * 86400.0, 30 * 86400.0,
            actualCoverageLimited: false));

    [Fact]
    public void Limited_actual_coverage_prevents_a_floating_planned_branch()
        => Assert.Equal(10 * 86400.0, OverlayKernel.PlannedHorizonSeconds(
            30 * 86400.0, 30 * 86400.0, 10 * 86400.0,
            actualCoverageLimited: true));

    [Fact]
    public void Ordinary_short_actual_endpoint_does_not_hide_collision_avoidance_plan()
        => Assert.Equal(30 * 86400.0, OverlayKernel.PlannedHorizonSeconds(
            30 * 86400.0, 30 * 86400.0, 2 * 86400.0,
            actualCoverageLimited: false));

    [Fact]
    public void Planless_stock_burns_keep_the_actual_horizon()
        => Assert.Equal(30 * 86400.0, OverlayKernel.PlannedHorizonSeconds(
            30 * 86400.0, double.NaN, 10 * 86400.0,
            actualCoverageLimited: false));

    [Fact]
    public void Planless_stock_burns_respect_limited_actual_coverage()
        => Assert.Equal(10 * 86400.0, OverlayKernel.PlannedHorizonSeconds(
            30 * 86400.0, double.NaN, 10 * 86400.0,
            actualCoverageLimited: true));

    [Fact]
    public void Pre_collision_branch_may_continue_beyond_the_actual_impact()
        => Assert.True(OverlayKernel.PlannedBranchConnected(
            plannedStartSeconds: 1.5 * 86400.0,
            actualSampleEndSeconds: 2 * 86400.0,
            plannedHorizonSeconds: 30 * 86400.0));

    [Fact]
    public void Post_collision_branch_cannot_float_beyond_the_actual_impact()
        => Assert.False(OverlayKernel.PlannedBranchConnected(
            plannedStartSeconds: 3 * 86400.0,
            actualSampleEndSeconds: 2 * 86400.0,
            plannedHorizonSeconds: 30 * 86400.0));

    [Fact]
    public void Finite_burn_connectivity_uses_ignition_not_the_later_node()
    {
        const double impact = 100.0, ignition = 90.0, node = 110.0, horizon = 1000.0;
        Assert.True(ignition <= impact && node > impact); // scenario guard
        Assert.True(OverlayKernel.PlannedBranchConnected(ignition, impact, horizon));
        Assert.False(OverlayKernel.PlannedBranchConnected(node, impact, horizon));
    }

    [Fact]
    public void Capped_then_collision_cut_uses_coverage_for_end_and_impact_for_branch()
    {
        double requested = 30 * 86400.0;
        double numericalCoverage = 10 * 86400.0;
        double visibleImpact = 2 * 86400.0;
        double plannedEnd = OverlayKernel.PlannedHorizonSeconds(
            requested, requested, numericalCoverage, actualCoverageLimited: true);

        Assert.Equal(numericalCoverage, plannedEnd);
        Assert.True(OverlayKernel.PlannedBranchConnected(
            1.5 * 86400.0, visibleImpact, plannedEnd));
        Assert.False(OverlayKernel.PlannedBranchConnected(
            3 * 86400.0, visibleImpact, plannedEnd));
    }

    [Fact]
    public void Off_rails_retained_suffix_must_branch_on_the_live_actual_prefix()
    {
        const double actualWorkEnd = 100.0, retainedPlanEnd = 1000.0;
        Assert.True(OverlayKernel.PlannedBranchConnected(
            plannedStartSeconds: 90.0, actualWorkEnd, retainedPlanEnd));
        Assert.False(OverlayKernel.PlannedBranchConnected(
            plannedStartSeconds: 110.0, actualWorkEnd, retainedPlanEnd));
    }

    [Fact]
    public void Cached_planned_suffix_is_invalid_when_a_new_collision_precedes_its_branch()
    {
        const double cachedIgnition = 90.0, cachedEnd = 1000.0;
        Assert.True(OverlayKernel.PlannedBranchConnected(
            cachedIgnition, actualSampleEndSeconds: 100.0, cachedEnd));
        Assert.False(OverlayKernel.PlannedBranchConnected(
            cachedIgnition, actualSampleEndSeconds: 80.0, cachedEnd));
    }

    [Theory]
    [InlineData(7, 7, true)]
    [InlineData(7, 8, true)]
    [InlineData(8, 7, false)]
    public void Off_rails_restamp_never_overhangs_a_shortened_plan(
        double batchEnd, double planEnd, bool expected)
        => Assert.Equal(expected,
            OverlayKernel.PlannedOffRailsHorizonCompatible(batchEnd, planEnd));
}

public class LerpBracketTests
{
    private static readonly double[] Times = [0.0, 10.0, 20.0, 30.0, 30.0, 30.0]; // padded tail

    [Fact]
    public void Interior_time_brackets_with_the_right_fraction()
    {
        var (lo, hi, frac) = OverlayKernel.LerpBracket(Times, 12.5);
        Assert.Equal((1, 2, 0.25), (lo, hi, frac));
    }

    [Fact]
    public void Exact_sample_time_collapses_to_that_index()
    {
        var (lo, hi, frac) = OverlayKernel.LerpBracket(Times, 10.0);
        Assert.Equal(lo, hi);
        Assert.Equal(10.0, Times[lo]);
        Assert.Equal(0.0, frac);
    }

    [Fact]
    public void Padding_duplicates_collapse_to_a_zero_span_bracket_not_a_division_by_zero()
    {
        var (lo, hi, frac) = OverlayKernel.LerpBracket(Times, 30.0);
        Assert.Equal(30.0, Times[lo]);
        Assert.Equal(30.0, Times[hi]);
        Assert.Equal(0.0, frac);
    }

    [Fact]
    public void Out_of_range_times_clamp_to_the_ends()
    {
        Assert.Equal((0, 0, 0.0), OverlayKernel.LerpBracket(Times, -5.0));
        var (lo, hi, frac) = OverlayKernel.LerpBracket(Times, 99.0);
        Assert.Equal((Times.Length - 1, Times.Length - 1, 0.0), (lo, hi, frac));
    }
}

/// <summary>The hover substitute's screen-space nearest-on-polyline rule
/// (OverlayKernel.PolylineNearest): vertex scan + adjacent-chord projection, so hover
/// stays continuous between coarse multi-period samples; NaN vertices (behind the
/// camera) are skipped like stock's scan.</summary>
public class PolylineNearestTests
{
    private static readonly Brutal.Numerics.float2[] Line =
    [
        new(0f, 0f), new(100f, 0f), new(200f, 0f),
    ];

    [Fact]
    public void Mouse_over_a_vertex_returns_that_vertex()
    {
        Assert.True(OverlayKernel.PolylineNearest(Line, new(101f, 1f),
            out int lo, out int hi, out double frac, out var projected));
        Assert.Equal(1, lo);
        Assert.True(frac is 0.0 || (hi == 2 && frac < 0.02)); // vertex or a hair into the chord
        Assert.Equal(101f, projected.X, precision: 0);
    }

    [Fact]
    public void Mouse_between_coarse_samples_projects_onto_the_chord()
    {
        // Halfway between vertices 0 and 1, 5 px off the line: a vertex-only scan
        // would report 50 px distance (a dead zone at coarse sampling); the chord
        // projection reports ~5 px at frac 0.5.
        Assert.True(OverlayKernel.PolylineNearest(Line, new(50f, 5f),
            out int lo, out int hi, out double frac, out var projected));
        Assert.Equal((0, 1), (lo, hi));
        Assert.Equal(0.5, frac, precision: 6);
        Assert.Equal(50f, projected.X, precision: 3);
        Assert.Equal(0f, projected.Y, precision: 3);
    }

    [Fact]
    public void NaN_vertices_are_skipped_not_fatal()
    {
        Brutal.Numerics.float2[] line =
            [new(float.NaN, float.NaN), new(100f, 0f), new(200f, 0f)];
        Assert.True(OverlayKernel.PolylineNearest(line, new(90f, 0f),
            out int lo, out _, out _, out _));
        Assert.True(lo >= 1);
    }

    [Fact]
    public void All_NaN_reports_no_hit()
        => Assert.False(OverlayKernel.PolylineNearest(
            [new(float.NaN, float.NaN)], new(0f, 0f), out _, out _, out _, out _));

    [Fact]
    public void Degenerate_padded_chords_fall_back_to_the_vertex()
    {
        // The padded tail repeats the last point: zero-length chords must not divide
        // by zero and must leave the vertex answer standing.
        Brutal.Numerics.float2[] line = [new(0f, 0f), new(100f, 0f), new(100f, 0f)];
        Assert.True(OverlayKernel.PolylineNearest(line, new(120f, 0f),
            out int lo, out int hi, out double frac, out _));
        Assert.Equal(lo, hi > lo ? lo : hi); // resolves to a point on/at the last real vertex
        Assert.True(frac >= 0.0 && frac <= 1.0);
    }
}

/// <summary>Honest line markers' scan primitives: first upcoming Ap/Pe = first
/// interior local extremum of the sampled distance series; first upcoming AN/DN =
/// first sign crossing of the plane-offset series (node time interpolates across the
/// bracketing samples).</summary>
public class MarkerScanTests
{
    [Fact]
    public void First_extrema_of_an_orbit_like_series_are_found_in_order()
    {
        // Distance rising to Ap at index 2, falling to Pe at index 5, rising again.
        double[] d = [3.0, 5.0, 8.0, 6.0, 2.0, 1.0, 4.0, 7.0];
        Assert.Equal(2, OverlayKernel.FirstLocalExtremum(d, d.Length, findMinimum: false));
        Assert.Equal(5, OverlayKernel.FirstLocalExtremum(d, d.Length, findMinimum: true));
    }

    [Fact]
    public void All_closest_approaches_are_returned_in_time_order()
    {
        // Three successive target passes; CA markers must not stop after the first.
        double[] separation = [9, 4, 7, 8, 3, 6, 10, 2, 5];

        Assert.Equal([1, 4, 7],
            OverlayKernel.LocalExtrema(separation, separation.Length, findMinimum: true));
    }

    [Fact]
    public void All_closest_approaches_keep_plateau_and_real_prefix_semantics()
    {
        // The first flat minimum counts once at its first sample. The repeated padded
        // tail lies outside count and cannot invent another closest approach.
        double[] separation = [8, 3, 3, 3, 7, 2, 6, 6, 6];

        Assert.Equal([1, 5],
            OverlayKernel.LocalExtrema(separation, count: 7, findMinimum: true));
    }

    [Fact]
    public void Closest_approach_relative_speed_uses_centered_relative_tangent()
    {
        double[] times = [0, 2, 5];
        Vector3d[] relative =
        [
            new(-1000, 0, 0),
            new(0, 100, 0),
            new(2000, 0, 0),
        ];

        Assert.Equal(600.0,
            OverlayKernel.RelativeSpeedAt(times, relative, index: 1, count: 3), 12);
    }

    [Fact]
    public void Crossing_relative_speed_uses_the_bracketing_segment()
    {
        double[] times = [10, 12, 15, 19];
        Vector3d[] relative = [new(0, 0, 0), new(900, 0, 0), new(900, 1200, 0)];

        Assert.Equal(300.0, OverlayKernel.RelativeSpeedAcross(
            times, relative, lo: 1, count: 3, timesOffset: 1), 12);
    }

    [Fact]
    public void Ordinary_marker_label_includes_time_to_go_and_relative_speed()
    {
        var marker = new OverlayMarker(OverlayMarkerKind.Periapsis, "Earth",
            TimeSeconds: 100_061.5, AltitudeMeters: 200_000,
            Label: "Pe Earth 200 km", RelativeSpeedMetersPerSecond: 7_500);

        Assert.Equal("Pe Earth 200 km | T-1d 1h 1m 2s | 7.50 km/s rel",
            OverlayKernel.MarkerLabelAt(marker, nowSeconds: 10_000));
    }

    [Fact]
    public void Closest_approach_label_includes_time_to_go_and_relative_speed()
    {
        string label = OverlayKernel.ClosestApproachLabel(
            "Station", distanceMeters: 125_000, secondsUntil: 90_061.5,
            relativeSpeed: 2_500);

        Assert.Contains("CA Station", label);
        Assert.Contains("125 km", label);
        Assert.Contains("T-1d 1h 1m 2s", label);
        Assert.Contains("2.50 km/s rel", label);
    }

    [Fact]
    public void Invalid_relative_speed_is_rendered_as_unknown()
    {
        string label = OverlayKernel.ClosestApproachLabel(
            "Station", 1000, 60, double.NaN);

        Assert.Contains("T-1m 0s", label);
        Assert.Contains("? m/s rel", label);
    }

    [Fact]
    public void Impact_label_includes_time_to_go_and_surface_relative_speed()
    {
        string label = OverlayKernel.ImpactLabel(
            impactTimeSeconds: 100_061.5, t0Seconds: 10_000, impactSpeed: 2_500);

        Assert.Equal("Impact | T-1d 1h 1m 2s | 2,500 m/s", label);
    }

    [Fact]
    public void Impact_label_keeps_time_to_go_when_speed_is_unavailable()
    {
        string label = OverlayKernel.ImpactLabel(
            impactTimeSeconds: 160, t0Seconds: 100, impactSpeed: null);

        Assert.Equal("Impact | T-1m 0s", label);
    }

    [Fact]
    public void Monotone_series_have_no_extremum_yet()
    {
        double[] rising = [1.0, 2.0, 3.0, 4.0];
        Assert.Equal(-1, OverlayKernel.FirstLocalExtremum(rising, rising.Length, findMinimum: false));
        Assert.Equal(-1, OverlayKernel.FirstLocalExtremum(rising, rising.Length, findMinimum: true));
    }

    [Fact]
    public void Plateaus_count_once_at_their_first_sample()
    {
        double[] d = [5.0, 3.0, 3.0, 3.0, 6.0];
        Assert.Equal(1, OverlayKernel.FirstLocalExtremum(d, d.Length, findMinimum: true));
        // A plateau that keeps descending afterwards is not an extremum.
        double[] shelf = [5.0, 3.0, 3.0, 3.0, 1.0];
        Assert.Equal(-1, OverlayKernel.FirstLocalExtremum(shelf, shelf.Length, findMinimum: true));
    }

    [Fact]
    public void Padded_tail_is_excluded_by_the_count()
    {
        double[] d = [3.0, 8.0, 2.0, 9.0, 9.0, 9.0]; // last three = padding
        Assert.Equal(1, OverlayKernel.FirstLocalExtremum(d, 4, findMinimum: false));
        Assert.Equal(2, OverlayKernel.FirstLocalExtremum(d, 4, findMinimum: true));
    }

    [Fact]
    public void Sign_crossings_bracket_and_interpolate()
    {
        double[] z = [-2.0, -1.0, 1.0, 2.0, -3.0];
        var an = OverlayKernel.FirstSignCrossing(z, z.Length, ascending: true);
        Assert.Equal((1, 0.5), an);
        var dn = OverlayKernel.FirstSignCrossing(z, z.Length, ascending: false);
        Assert.Equal(3, dn!.Value.Lo);
        Assert.Equal(0.4, dn.Value.Frac, precision: 12); // 2 -> -3 crosses at 2/5
    }

    [Fact]
    public void All_sign_crossings_are_cached_in_chronological_order()
    {
        double[] z = [-2.0, 2.0, -2.0, 2.0];
        Assert.Equal([(0, 0.5), (2, 0.5)],
            OverlayKernel.SignCrossings(z, z.Length, ascending: true));
        Assert.Equal([(1, 0.5)],
            OverlayKernel.SignCrossings(z, z.Length, ascending: false));
    }

    [Fact]
    public void Tilted_spin_pole_finds_equatorial_crossing_not_ecliptic_crossing()
    {
        double invSqrt2 = 1.0 / Math.Sqrt(2.0);
        var pole = new Vector3d(0, invSqrt2, invSqrt2);
        Vector3d[] relativePositions =
        [
            new(0, -2, 1),
            new(0, 0, 1),
        ];
        double[] offsets = relativePositions
            .Select(position => OverlayKernel.EquatorialPlaneOffset(position, pole)).ToArray();

        Assert.All(relativePositions, position => Assert.True(position.Z > 0));
        Assert.Equal((0, 0.5), OverlayKernel.FirstSignCrossing(offsets, offsets.Length, ascending: true));
    }

    [Fact]
    public void No_crossing_yields_null()
    {
        double[] positive = [1.0, 2.0, 0.5];
        Assert.Null(OverlayKernel.FirstSignCrossing(positive, positive.Length, ascending: true));
    }

    [Fact]
    public void Soi_crossings_distinguish_encounter_from_escape_and_interpolate()
    {
        const double soi = 10.0;
        double[] flyby = [15.0, 12.0, 8.0, 6.0, 9.0, 11.0, 14.0];

        var encounter = OverlayKernel.FirstSoiCrossing(
            flyby, flyby.Length, soi, entering: true);
        Assert.Equal(1, encounter!.Value.Lo);
        Assert.Equal(0.5, encounter.Value.Frac, precision: 12);

        var escape = OverlayKernel.FirstSoiCrossing(
            flyby, flyby.Length, soi, entering: false,
            startIndex: encounter.Value.Lo + 1);
        Assert.Equal(4, escape!.Value.Lo);
        Assert.Equal(0.5, escape.Value.Frac, precision: 12);
    }

    [Fact]
    public void Soi_scan_cursor_skips_transitions_already_consumed()
    {
        double[] repeated = [12.0, 8.0, 12.0, 8.0, 12.0];
        var secondEncounter = OverlayKernel.FirstSoiCrossing(
            repeated, repeated.Length, 10.0, entering: true, startIndex: 2);

        Assert.Equal(2, secondEncounter!.Value.Lo);
        Assert.Equal(0.5, secondEncounter.Value.Frac, precision: 12);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void Invalid_soi_radius_has_no_transition(double radius)
    {
        Assert.Null(OverlayKernel.FirstSoiCrossing(
            [2.0, 0.5], 2, radius, entering: true));
    }

    [Fact]
    public void Marker_work_is_limited_to_the_controlled_vessel()
    {
        Assert.True(OverlayKernel.MarkerWorkEnabled("controlled", "controlled"));
        Assert.False(OverlayKernel.MarkerWorkEnabled("controlled", "target"));
        Assert.False(OverlayKernel.MarkerWorkEnabled(null, "controlled"));
    }

    [Fact]
    public void Soi_sequence_emits_encounter_then_escape_for_a_flyby()
    {
        var parents = new Dictionary<string, string?> { ["Earth"] = "Sol", ["Moon"] = "Earth" };
        var children = new Dictionary<string, IReadOnlyList<string>>
        {
            ["Earth"] = ["Moon"],
            ["Moon"] = [],
        };
        var distances = new Dictionary<string, double[]>
        {
            ["Earth"] = [5, 5, 5, 5, 5, 5],
            ["Moon"] = [15, 12, 8, 6, 11, 14],
        };

        var events = OverlayKernel.FindSoiTransitions(
            "Earth", 6,
            id => parents.GetValueOrDefault(id),
            id => children.GetValueOrDefault(id, []),
            _ => 10,
            id => distances[id]);

        Assert.Collection(events,
            enter =>
            {
                Assert.False(enter.Escape);
                Assert.Equal("Moon", enter.BodyId);
                Assert.Equal(1, enter.Lo);
            },
            escape =>
            {
                Assert.True(escape.Escape);
                Assert.Equal("Moon", escape.BodyId);
                Assert.Equal(3, escape.Lo);
            });
    }

    [Fact]
    public void Soi_sequence_chooses_the_earliest_competing_transition()
    {
        var distances = new Dictionary<string, double[]>
        {
            // Both cross in segment 0→1; Moon encounter occurs at 1/4,
            // Earth escape at 3/4, so encounter must win.
            ["Earth"] = [7, 11, 11],
            ["Moon"] = [11, 7, 6],
        };
        var events = OverlayKernel.FindSoiTransitions(
            "Earth", 3,
            id => id == "Earth" ? "Sol" : "Earth",
            id => id == "Earth" ? ["Moon"] : [],
            _ => 10,
            id => distances[id],
            maxTransitions: 1);

        var encounter = Assert.Single(events);
        Assert.False(encounter.Escape);
        Assert.Equal("Moon", encounter.BodyId);
        Assert.Equal(0.25, encounter.Frac, precision: 12);
    }

    [Fact]
    public void Soi_sequence_rejects_a_nested_transition_earlier_in_the_same_bracket()
    {
        var distances = new Dictionary<string, double[]>
        {
            ["Earth"] = [5, 5, 5],
            ["Moon"] = [12, 8, 7],       // encounter at .5
            ["Submoon"] = [11, 1, 1],    // nominal encounter at .1: already in the past
        };
        var events = OverlayKernel.FindSoiTransitions(
            "Earth", 3,
            id => id switch { "Earth" => "Sol", "Moon" => "Earth", _ => "Moon" },
            id => id switch { "Earth" => ["Moon"], "Moon" => ["Submoon"], _ => [] },
            _ => 10,
            id => distances[id]);

        var encounter = Assert.Single(events);
        Assert.Equal("Moon", encounter.BodyId);
        Assert.Equal(0.5, encounter.Frac, precision: 12);
    }

    [Fact]
    public void Soi_sequence_keeps_a_nested_transition_later_in_the_same_bracket()
    {
        var distances = new Dictionary<string, double[]>
        {
            ["Earth"] = [5, 5, 5],
            ["Moon"] = [12, 8, 7],       // encounter at .5
            ["Submoon"] = [18, 8, 7],    // encounter at .8
        };
        var events = OverlayKernel.FindSoiTransitions(
            "Earth", 3,
            id => id switch { "Earth" => "Sol", "Moon" => "Earth", _ => "Moon" },
            id => id switch { "Earth" => ["Moon"], "Moon" => ["Submoon"], _ => [] },
            _ => 10,
            id => distances[id]);

        Assert.Collection(events,
            first => Assert.Equal(("Moon", 0.5), (first.BodyId, first.Frac)),
            second => Assert.Equal(("Submoon", 0.8), (second.BodyId, second.Frac)));
    }

    [Fact]
    public void Equal_time_soi_tie_explicitly_prefers_escape()
    {
        var distances = new Dictionary<string, double[]>
        {
            ["Earth"] = [8, 12],
            ["Moon"] = [12, 8],
        };
        var events = OverlayKernel.FindSoiTransitions(
            "Earth", 2,
            _ => "Sol",
            id => id == "Earth" ? ["Moon"] : [],
            _ => 10,
            id => distances[id],
            maxTransitions: 1);

        var transition = Assert.Single(events);
        Assert.True(transition.Escape);
        Assert.Equal("Earth", transition.BodyId);
        Assert.Equal(0.5, transition.Frac, precision: 12);
    }

    [Fact]
    public void Planned_restamp_replaces_old_target_markers_but_keeps_collision()
    {
        OverlayMarker[] previous =
        [
            new(OverlayMarkerKind.Collision, "Earth", 160, 0,
                "Impact | T-2m | 2,500 m/s", 2_500),
            new(OverlayMarkerKind.ClosestApproach, "OldTarget", 20, 1000, "CA OldTarget 1 km"),
            new(OverlayMarkerKind.Periapsis, "Earth", 30, 2000, "Pe Earth 2 km"),
        ];
        OverlayMarker[] refreshed =
        [
            new(OverlayMarkerKind.ClosestApproach, "NewTarget", 25, 500, "CA NewTarget 1 km"),
        ];

        var markers = OverlayKernel.RestampedMarkers(previous, refreshed, t0Seconds: 130);

        Assert.Collection(markers,
            collision =>
            {
                Assert.Equal(OverlayMarkerKind.Collision, collision.Kind);
                Assert.Equal("Impact | T-30s | 2,500 m/s", collision.Label);
                Assert.Equal(2_500, collision.ImpactSpeedMetersPerSecond);
            },
            closest =>
            {
                Assert.Equal(OverlayMarkerKind.ClosestApproach, closest.Kind);
                Assert.Equal("NewTarget", closest.BodyId);
            });
        Assert.DoesNotContain(markers, marker => marker.BodyId == "OldTarget");
    }

    [Fact]
    public void Planned_restamp_target_removal_drops_old_closest_approach()
    {
        OverlayMarker[] previous =
        [
            new(OverlayMarkerKind.ClosestApproach, "OldTarget", 20, 1000, "CA OldTarget 1 km"),
        ];

        Assert.Empty(OverlayKernel.RestampedMarkers(previous, [], t0Seconds: 30));
    }

    [Fact]
    public void Cached_candidates_project_to_upcoming_markers_without_geometry_work()
    {
        OverlayMarker[] candidates =
        [
            new(OverlayMarkerKind.Collision, nameof(OverlayMarkerKind.Collision),
                130, 0, string.Empty, 1000),
            new(OverlayMarkerKind.Periapsis, "Earth", 100, 1000, "old Pe"),
            new(OverlayMarkerKind.Periapsis, "Earth", 200, 2000, "next Pe"),
            new(OverlayMarkerKind.Periapsis, "Earth", 300, 3000, "later Pe"),
            new(OverlayMarkerKind.ClosestApproach, "Luna", 140, 500, "old CA",
                RelativeSpeedMetersPerSecond: 25),
            new(OverlayMarkerKind.ClosestApproach, "Luna", 250, 400, "stale label",
                RelativeSpeedMetersPerSecond: 25),
            new(OverlayMarkerKind.Collision, "Earth", 180, 0, "stale impact", 2500),
        ];

        var visible = OverlayKernel.VisibleMarkers(candidates, t0Seconds: 150);

        Assert.Equal(3, visible.Count);
        Assert.Contains(visible, m => m.Kind == OverlayMarkerKind.Periapsis
            && m.TimeSeconds == 200 && m.Label == "next Pe");
        Assert.Contains(visible, m => m.Kind == OverlayMarkerKind.ClosestApproach
            && m.TimeSeconds == 250 && m.Label.Contains("T-1m 40s"));
        Assert.Contains(visible, m => m.Kind == OverlayMarkerKind.Collision
            && m.Label == "Impact | T-30s | 2,500 m/s");
        Assert.DoesNotContain(visible, m => m.TimeSeconds is 100 or 130 or 140 or 300);
    }

    [Fact]
    public void Planned_payload_restamp_refreshes_epoch_without_touching_geometry()
    {
        double[] times = [100, 200, 200];
        var (sincePe, remaining) = OverlayKernel.RestampPayloadTimes(
            times, t0Seconds: 150, timeAtPeSeconds: 75);

        Assert.Equal([25, 125, 125], sincePe);
        Assert.Equal([-50, 50, 50], remaining);
        Assert.Equal([100, 200, 200], times);
    }

    [Fact]
    public void Sampled_position_interpolates_and_rejects_outside_window()
    {
        double[] times = [10, 20];
        Vector3d[] positions = [new(0, 2, 4), new(10, 12, 14)];

        Assert.True(OverlayKernel.TryInterpolatedPosition(
            times, positions, 12.5, out var position));
        Assert.Equal(new Vector3d(2.5, 4.5, 6.5), position);
        Assert.False(OverlayKernel.TryInterpolatedPosition(
            times, positions, 9, out _));
    }
}

/// <summary>Surface-frame collision cut (OverlayKernel.CutAtFirstCollision): the first
/// descent of a sampled body-centred series through the surface radius truncates the
/// series there, with the bisected crossing appended as its final sample.</summary>
public class CollisionCutTests
{
    /// <summary>Piecewise-linear body-centred sampler through the given (time, |r|)
    /// samples — the continuous curve the discrete series was sampled from, standing
    /// in for the sweep evaluator the production caller passes.</summary>
    private static Func<double, Vector3d> LinearSampler(double[] times, double[] radii) => t =>
    {
        int hi = 1;
        while (hi < times.Length - 1 && times[hi] < t) hi++;
        double frac = (t - times[hi - 1]) / (times[hi] - times[hi - 1]);
        return new Vector3d(radii[hi - 1] + (radii[hi] - radii[hi - 1]) * frac, 0, 0);
    };

    [Fact]
    public void First_descent_through_the_radius_cuts_and_appends_the_crossing()
    {
        double[] times = [0.0, 2.0, 6.0, 8.0];
        double[] radii = [10.0, 8.0, 4.0, 2.0]; // crosses 5 between samples 1 and 2
        var positions = new Vector3d[radii.Length];
        for (int i = 0; i < radii.Length; i++) positions[i] = new Vector3d(radii[i], 0, 0);
        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5.0,
            LinearSampler(times, radii));
        Assert.NotNull(cut);
        Assert.Equal(3, cut.Times.Length);          // samples 0..1 kept + the crossing
        Assert.Equal(new[] { 0.0, 2.0 }, cut.Times[..2]);
        Assert.Equal(cut.ImpactTimeSeconds, cut.Times[^1]);
        Assert.Equal(cut.ImpactCoordinate, cut.Positions[^1]);
        Assert.InRange(cut.ImpactTimeSeconds, 2.0, 6.0);
        // The bisection lands on the surface within the kernel's meter tolerance.
        Assert.InRange(cut.ImpactCoordinate.Length(),
            5.0 - OverlayKernel.CollisionCutToleranceMeters,
            5.0 + OverlayKernel.CollisionCutToleranceMeters);
    }

    [Fact]
    public void A_series_starting_below_the_surface_is_not_an_upcoming_impact()
    {
        // Landed/sub-surface seed: no impact until it exits and re-enters.
        double[] times = [0.0, 1.0, 2.0, 3.0];
        double[] rising = [3.0, 4.0, 6.0, 8.0];
        var positions = new Vector3d[rising.Length];
        for (int i = 0; i < rising.Length; i++) positions[i] = new Vector3d(rising[i], 0, 0);
        Assert.Null(OverlayKernel.CutAtFirstCollision(times, positions, 5.0,
            LinearSampler(times, rising)));

        // ...and a re-entry after the exit IS one: the cut keeps the whole hop.
        double[] hop = [3.0, 6.0, 8.0, 4.0];
        for (int i = 0; i < hop.Length; i++) positions[i] = new Vector3d(hop[i], 0, 0);
        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5.0,
            LinearSampler(times, hop));
        Assert.NotNull(cut);
        Assert.Equal(4, cut.Times.Length); // samples 0..2 kept + the crossing
        Assert.InRange(cut.ImpactTimeSeconds, 2.0, 3.0);
    }

    [Fact]
    public void A_series_that_never_reaches_the_surface_yields_null()
    {
        double[] times = [0.0, 1.0, 2.0, 3.0];
        double[] radii = [9.0, 6.0, 5.5, 7.0];
        var positions = new Vector3d[radii.Length];
        for (int i = 0; i < radii.Length; i++) positions[i] = new Vector3d(radii[i], 0, 0);
        Assert.Null(OverlayKernel.CutAtFirstCollision(times, positions, 5.0,
            LinearSampler(times, radii)));
    }

    [Fact]
    public void An_unknown_radius_disables_the_cut()
    {
        double[] times = [0.0, 1.0];
        Vector3d[] positions = [new(9.0, 0, 0), new(1.0, 0, 0)];
        Assert.Null(OverlayKernel.CutAtFirstCollision(times, positions, 0.0,
            LinearSampler(times, [9.0, 1.0])));
    }

    [Fact]
    public void Fast_transit_through_a_small_body_is_found_between_clear_samples()
    {
        // A nearly radial impact has almost no chord turn, so the adaptive sampler can
        // accept one long step across Luna. Both accepted endpoints are above the
        // surface even though the exact trajectory passes through the body.
        double[] times = [0.0, 10.0];
        Vector3d[] positions = [new(10_000.0, 0, 0), new(-10_000.0, 0, 0)];
        Vector3d Sample(double t) => new(10_000.0 - 2_000.0 * t, 0, 0);

        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5_000.0, Sample);

        Assert.NotNull(cut);
        Assert.InRange(cut.ImpactTimeSeconds, 2.499, 2.501);
        Assert.InRange(cut.ImpactCoordinate.Length(),
            5_000.0 - OverlayKernel.CollisionCutToleranceMeters,
            5_000.0 + OverlayKernel.CollisionCutToleranceMeters);
        Assert.Equal(2, cut.Times.Length);
        Assert.Equal(Sample(cut.ImpactTimeSeconds), cut.ImpactCoordinate);
        Assert.Equal(cut.ImpactTimeSeconds, cut.Times[^1]);
    }

    [Fact]
    public void Hidden_transit_is_cut_before_a_deep_interior_sampler_failure()
    {
        double[] times = [0.0, 10.0];
        Vector3d[] positions = [new(10_000.0, 0, 0), new(-10_000.0, 0, 0)];
        Vector3d Sample(double t)
        {
            var result = new Vector3d(10_000.0 - 2_000.0 * t, 0, 0);
            if (result.Length() < 4_000.0)
                throw new InvalidOperationException();
            return result;
        }

        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5_000.0, Sample);

        Assert.NotNull(cut);
        Assert.InRange(cut.ImpactCoordinate.Length(),
            5_000.0 - OverlayKernel.CollisionCutToleranceMeters,
            5_000.0 + OverlayKernel.CollisionCutToleranceMeters);
    }

    [Fact]
    public void Chord_through_body_does_not_cut_an_exact_curve_that_stays_clear()
    {
        // Sparse samples of a safe circular arc have a chord inside the body. The
        // broad phase must verify the actual trajectory instead of inventing impact.
        double[] times = [0.0, 1.0];
        Vector3d[] positions = [new(0.0, 6.0, 0.0), new(0.0, -6.0, 0.0)];
        Vector3d Sample(double t)
        {
            double angle = Math.PI * (0.5 + t);
            return new Vector3d(6.0 * Math.Cos(angle), 6.0 * Math.Sin(angle), 0.0);
        }

        Assert.Null(OverlayKernel.CutAtFirstCollision(times, positions, 5.0, Sample));
    }

    [Fact]
    public void Directional_mountain_is_found_between_clear_samples()
    {
        double[] times = [0.0, 1.0];
        Vector3d[] positions = [new(0.0, 6_000.0, 0.0), new(0.0, -6_000.0, 0.0)];
        Vector3d Sample(double t)
        {
            double angle = Math.PI * (0.5 - t);
            return new Vector3d(6_000.0 * Math.Cos(angle), 6_000.0 * Math.Sin(angle), 0.0);
        }
        double Mountain(Vector3d position) => position.X > 5_500.0 ? 2_000.0 : 0.0;

        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5_000.0,
            Sample, Mountain, maximumSurfaceRadius: 7_000.0);

        Assert.NotNull(cut);
        Assert.True(cut.ImpactCoordinate.X > 5_500.0);
        Assert.Null(OverlayKernel.CutAtFirstCollision(times, positions, 5_000.0,
            Sample, maximumSurfaceRadius: 5_000.0));
    }

    [Fact]
    public void Safe_low_orbit_inside_terrain_envelope_gets_one_exact_probe()
    {
        double[] times = [0.0, 1.0];
        const double halfAngle = 0.2;
        Vector3d Point(double angle) =>
            new(6_000.0 * Math.Cos(angle), 6_000.0 * Math.Sin(angle), 0.0);
        Vector3d[] positions = [Point(halfAngle), Point(-halfAngle)];
        int calls = 0;
        Vector3d Sample(double t)
        {
            calls++;
            return Point(halfAngle * (1.0 - 2.0 * t));
        }

        Assert.Null(OverlayKernel.CutAtFirstCollision(times, positions, 5_000.0,
            Sample, _ => 0.0, maximumSurfaceRadius: 7_000.0));
        Assert.Equal(1, calls);
    }

    [Fact]
    public void A_sample_exactly_on_the_surface_is_the_crossing()
    {
        double[] times = [0.0, 4.0, 8.0];
        double[] radii = [9.0, 5.0, 3.0];
        var positions = new Vector3d[radii.Length];
        for (int i = 0; i < radii.Length; i++) positions[i] = new Vector3d(radii[i], 0, 0);
        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5.0,
            LinearSampler(times, radii));
        Assert.NotNull(cut);
        Assert.Equal(2, cut.Times.Length); // only sample 0 kept + the crossing
        Assert.InRange(cut.ImpactTimeSeconds, 0.0, 4.0);
        Assert.InRange(cut.ImpactCoordinate.Length(),
            5.0 - OverlayKernel.CollisionCutToleranceMeters,
            5.0 + OverlayKernel.CollisionCutToleranceMeters);
    }

    [Fact]
    public void Terrain_mountain_above_mean_radius_causes_an_earlier_cut()
    {
        double[] times = [0.0, 1.0, 2.0];
        double[] radii = [9000.0, 6000.0, 4000.0];
        var positions = radii.Select(r => new Vector3d(r, 0, 0)).ToArray();
        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5000.0,
            LinearSampler(times, radii), _ => 2000.0, maximumSurfaceRadius: 7000.0);
        Assert.NotNull(cut);
        Assert.InRange(cut.ImpactCoordinate.Length(),
            7000.0 - OverlayKernel.CollisionCutToleranceMeters,
            7000.0 + OverlayKernel.CollisionCutToleranceMeters);
        Assert.InRange(cut.ImpactTimeSeconds, 0.0, 1.0);
    }

    [Fact]
    public void Terrain_basin_below_mean_radius_delays_the_cut()
    {
        double[] times = [0.0, 1.0, 2.0, 3.0];
        double[] radii = [9000.0, 6000.0, 4500.0, 2000.0];
        var positions = radii.Select(r => new Vector3d(r, 0, 0)).ToArray();
        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5000.0,
            LinearSampler(times, radii), _ => -1000.0, maximumSurfaceRadius: 5000.0);
        Assert.NotNull(cut);
        Assert.InRange(cut.ImpactCoordinate.Length(),
            4000.0 - OverlayKernel.CollisionCutToleranceMeters,
            4000.0 + OverlayKernel.CollisionCutToleranceMeters);
        Assert.InRange(cut.ImpactTimeSeconds, 2.0, 3.0);
    }

    [Fact]
    public void Failed_terrain_query_falls_back_to_mean_radius()
    {
        double[] times = [0.0, 2.0];
        double[] radii = [9.0, 3.0];
        var positions = radii.Select(r => new Vector3d(r, 0, 0)).ToArray();
        var cut = OverlayKernel.CutAtFirstCollision(times, positions, 5.0,
            LinearSampler(times, radii), _ => double.NaN);
        Assert.NotNull(cut);
        Assert.InRange(cut.ImpactCoordinate.Length(),
            5.0 - OverlayKernel.CollisionCutToleranceMeters,
            5.0 + OverlayKernel.CollisionCutToleranceMeters);
    }
}

/// <summary>The ONE frame-mode identity (OverlayKernel.ModeMatches): every drawn or
/// interactive surface blinks together through a frame switch.</summary>
public class ModeMatchesTests
{
    [Theory]
    [InlineData(null, null, true)]           // inertial batch, inertial display
    [InlineData("Earth-Luna Fixed", "Earth-Luna Fixed", true)]
    [InlineData("Earth-Luna Fixed", null, false)]
    [InlineData(null, "Earth-Luna Fixed", false)]
    [InlineData("Earth-Luna Fixed", "Luna-Earth Fixed", false)]
    public void Mode_identity_is_ordinal_label_equality(string? batch, string? active, bool expected)
        => Assert.Equal(expected, OverlayKernel.ModeMatches(batch, active));
}

/// <summary>Live celestial catalog: dense systems (SolSystemDense) load hundreds of
/// bodies, and the rails worker samples an honest arc for EVERY curve body each ~1 s —
/// so the curve set is capped at celestial_curve_max_bodies by a deterministic priority:
/// backbone bodies first (their mutual coupling makes their stock lines the least
/// truthful), then µ descending, ties by ordinal id. Every key is parse-time
/// constant, so changing the runtime cap selects a stable prefix.</summary>
public class CurvePriorityTests
{
    [Fact]
    public void Backbone_outranks_a_bigger_mu_restricted_body()
        => Assert.Equal(["TinyMoon", "BigComet"], OverlayKernel.CurvePriority(
            [("BigComet", 1e18, false), ("TinyMoon", 1e10, true)], maxBodies: 128));

    [Fact]
    public void Mu_descends_within_a_class()
        => Assert.Equal(["Jupiter", "Saturn", "Earth"], OverlayKernel.CurvePriority(
            [("Earth", 3.986e14, true), ("Jupiter", 1.267e17, true), ("Saturn", 3.793e16, true)],
            maxBodies: 128));

    [Fact]
    public void Equal_mu_ties_break_by_ordinal_id()
        // StringComparer.Ordinal: 'A' (65) < 'Z' (90) < 'a' (97) — culture-free, stable.
        => Assert.Equal(["Alpha", "Zeta", "alpha"], OverlayKernel.CurvePriority(
            [("alpha", 5.0, false), ("Zeta", 5.0, false), ("Alpha", 5.0, false)], maxBodies: 128));

    [Fact]
    public void Cap_truncates_to_the_top_priority_bodies()
        => Assert.Equal(["SmallBackbone", "BigRestricted"], OverlayKernel.CurvePriority(
            [("MidRestricted", 1.0, false), ("BigRestricted", 2.0, false), ("SmallBackbone", 0.5, true)],
            maxBodies: 2));

    [Fact]
    public void Cap_below_one_clamps_to_one()
        => Assert.Equal(["Big"], OverlayKernel.CurvePriority(
            [("Big", 2.0, false), ("Small", 1.0, false)], maxBodies: 0));
}

/// <summary>Honest orbit lines: the overlay buffer holds one immutable batch PER vessel,
/// keyed by vessel id, with publish/read/reset semantics per slot;
/// single-reference publication per slot keeps reads tear-free.</summary>
public class OverlayBufferTests
{
    private static OverlaySamples Batch(string vesselId) => new()
    {
        VesselId = vesselId, SampleT0 = 0, FutureStartSeconds = 0,
        SampleWallMs = 1, CaptureSimSeconds = 100, AnchorState = default,
        Times = [0.0], TimesSincePe = [0.0], RemainingTimesTo = [0.0],
        PositionsCce = [default], ParentId = "Terra", PointCount = 1, Truncated = false,
        HorizonSeconds = 0, SamplingThetaMax = 0.01, SamplingMaxDensePoints = 2000,
        HistoryDisplaySeconds = 0, HistoryRequestedStartSeconds = 0,
        HistoryOldestRecordedStartSeconds = null,
        HistoryOldestRenderedStartSeconds = null,
        HistoryRenderBudgetTruncated = false, HistoryPointCount = 0,
        Markers = [], DenseTimes = [100.0, 1000.0],
        DensePositionsCce = [default, default],
        DenseMetrics = DecimationMetrics.For([default, default]),
        DenseMetricsCce = DecimationMetrics.For([default, default]),
    };

    [Fact]
    public void Limit_note_distinguishes_dynamics_limits_from_point_caps()
    {
        var dynamics = Batch("a") with { Truncated = true, DynamicsLimited = true };
        string dynamicsNote = TrajectoryOverlay.LimitNote(
            dynamics, plannedTruncated: true, plannedDynamicsLimited: true);
        Assert.Contains("trajectory ended at dynamics limit", dynamicsNote);
        Assert.Contains("planned arc ended at dynamics limit", dynamicsNote);
        Assert.DoesNotContain("cap hit", dynamicsNote);

        var capped = dynamics with { DynamicsLimited = false };
        string capNote = TrajectoryOverlay.LimitNote(
            capped, plannedTruncated: true, plannedDynamicsLimited: false);
        Assert.Contains("cap hit", capNote);
        Assert.Contains("planned arc truncated", capNote);
    }

    [Fact]
    public void History_display_duration_is_rendered_geometry_identity()
    {
        var batch = Batch("a") with { HistoryDisplaySeconds = 30 * 86400.0 };

        Assert.True(TrajectoryOverlay.HistoryDisplayMatches(
            batch, 30 * 86400.0));
        Assert.False(TrajectoryOverlay.HistoryDisplayMatches(
            batch, 31 * 86400.0));
    }
    [Fact]
    public void Planned_restamp_refreshes_epoch_fields_and_reuses_geometry_references()
    {
        double[] denseTimes = [100, 200];
        Vector3d[] densePositions = [new(1, 0, 0), new(2, 0, 0)];
        var candidates = new OverlayMarker[]
        {
            new(OverlayMarkerKind.Periapsis, "Terra", 200, 1000, "Pe Terra 1 km"),
        };
        var original = Batch("a") with
        {
            SampleT0 = 100,
            Times = [100, 200],
            TimesSincePe = [90, 190],
            RemainingTimesTo = [0, 100],
            DenseTimes = denseTimes,
            DensePositionsCce = densePositions,
            DenseMetrics = DecimationMetrics.For(densePositions),
            DenseMetricsCce = DecimationMetrics.For(densePositions),
            DynamicsLimited = true,
        };
        object markerKey = new();
        var anchor = new StateVector(new(3, 4, 5), new(6, 7, 8));

        var restamped = TrajectoryOverlay.RestampPlannedBatch(
            original, t0Seconds: 150, timeAtPeSeconds: 75, anchor,
            candidates, markerKey, captureSimSeconds: 200, wallMilliseconds: 999);

        Assert.Equal(150, restamped.SampleT0);
        Assert.Equal(999, restamped.SampleWallMs);
        Assert.Equal(200, restamped.CaptureSimSeconds);
        Assert.Equal(anchor, restamped.AnchorState);
        Assert.Equal([25, 125], restamped.TimesSincePe);
        Assert.Equal([-50, 50], restamped.RemainingTimesTo);
        Assert.Same(denseTimes, restamped.DenseTimes);
        Assert.Same(densePositions, restamped.DensePositionsCce);
        Assert.Same(candidates, restamped.MarkerCandidates);
        Assert.Same(markerKey, restamped.MarkerCacheKey);
        Assert.Single(restamped.Markers);
        Assert.True(restamped.DynamicsLimited);
    }

    [Fact]
    public void Planned_slot_is_independent_and_clearable()
    {
        OverlayBuffer.ResetSessionStatics();
        OverlayBuffer.Publish(Batch("a"));
        OverlayBuffer.PublishPlanned(Batch("a"));
        Assert.NotNull(OverlayBuffer.ReadPlanned("a"));
        OverlayBuffer.ClearPlanned("a");
        Assert.Null(OverlayBuffer.ReadPlanned("a"));
        Assert.NotNull(OverlayBuffer.Read("a")); // the actual batch is untouched
        OverlayBuffer.PublishPlanned(Batch("a"));
        OverlayBuffer.ResetSessionStatics();
        Assert.Null(OverlayBuffer.ReadPlanned("a"));
    }

    [Fact]
    public void Vessel_revocation_clears_both_ownership_slots_and_all_leases()
    {
        string id = nameof(Vessel_revocation_clears_both_ownership_slots_and_all_leases);
        OverlayBuffer.ResetSessionStatics();
        OverlayBuffer.Publish(Batch(id));
        OverlayBuffer.PublishPlanned(Batch(id));
        int generation = OverlayWorker.CurrentGeneration;
        long nowMs = Environment.TickCount64;
        _ = OverlayBuffer.BeginRebuildLease(id, generation, nowMs);
        _ = OverlayBuffer.BeginLineLease(id, generation, nowMs);

        OverlayBuffer.RevokeVessel(id, clearSamples: true);

        Assert.Null(OverlayBuffer.Read(id));
        Assert.Null(OverlayBuffer.ReadPlanned(id));
        Assert.False(OverlayBuffer.IsRebuildLeased(id, nowMs));
    }

    [Fact]
    public void Optimizer_line_lease_is_identity_scoped_and_does_not_refresh_consumers()
    {
        string id = nameof(Optimizer_line_lease_is_identity_scoped_and_does_not_refresh_consumers);
        OverlayBuffer.ResetSessionStatics();
        var actual = Batch(id);
        var planned = Batch(id);
        OverlayBuffer.Publish(actual);
        OverlayBuffer.PublishPlanned(planned);
        var lease = OverlayBuffer.BeginLineLease(
            id, OverlayWorker.CurrentGeneration, nowMs: 10);

        Assert.Null(OverlayBuffer.ReadFresh(id, nowMs: 5010, nowSimSeconds: 100));
        Assert.Null(OverlayBuffer.ReadPlannedFresh(id, nowMs: 5010, nowSimSeconds: 100));
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, actual, planned: false, nowMs: 5010, nowSimSeconds: 100));
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, planned, planned: true, nowMs: 5010, nowSimSeconds: 100));

        var replacement = Batch(id);
        OverlayBuffer.Publish(replacement);
        OverlayBuffer.RenewLineLease(lease, nowMs: 5000);
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, replacement, planned: false, nowMs: 9000, nowSimSeconds: 100));
        Assert.Null(OverlayBuffer.ReadFresh(id, nowMs: 9000, nowSimSeconds: 100));
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, replacement, planned: false, nowMs: 9000, nowSimSeconds: 700));
        Assert.False(OverlayBuffer.LineSamplesUsable(
            id, replacement, planned: false, nowMs: 9000, nowSimSeconds: 1000));
        OverlayBuffer.EndLineLease(lease);
        Assert.False(OverlayBuffer.LineSamplesUsable(
            id, replacement, planned: false, nowMs: 9000, nowSimSeconds: 100));
    }

    [Fact]
    public void Rebuild_lease_keeps_all_consumers_on_the_last_complete_batches_but_is_bounded()
    {
        string id = nameof(Rebuild_lease_keeps_all_consumers_on_the_last_complete_batches_but_is_bounded);
        OverlayBuffer.ResetSessionStatics();
        var actual = Batch(id);
        var planned = Batch(id);
        OverlayBuffer.Publish(actual);
        OverlayBuffer.PublishPlanned(planned);
        int generation = OverlayWorker.CurrentGeneration;
        long nowMs = Environment.TickCount64;
        var first = OverlayBuffer.BeginRebuildLease(id, generation, nowMs)!;

        Assert.Same(actual, OverlayBuffer.ReadFresh(
            id, nowMs + 6000, nowSimSeconds: 100));
        Assert.Same(planned, OverlayBuffer.ReadPlannedFresh(
            id, nowMs + 6000, nowSimSeconds: 100));
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, actual, planned: false, nowMs + OverlayKernel.RestageMaxAgeMs + 1,
            nowSimSeconds: 100));

        // A warp-sized capture age does not split interactive consumers from the
        // covered line while the bounded rebuild lease keeps both alive.
        Assert.Same(actual, OverlayBuffer.ReadFresh(
            id, nowMs + 6000, nowSimSeconds: 700));
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, actual, planned: false, nowMs + OverlayKernel.RestageMaxAgeMs + 1,
            nowSimSeconds: 700));
        Assert.False(OverlayBuffer.LineSamplesUsable(
            id, actual, planned: false, nowMs + OverlayKernel.RestageMaxAgeMs + 1,
            nowSimSeconds: 1000));

        var latest = OverlayBuffer.BeginRebuildLease(id, generation, nowMs + 1000)!;
        Assert.Equal(first.StartedMs, latest.StartedMs);
        Assert.Equal(first.ExpiresMs, latest.ExpiresMs);
        OverlayBuffer.EndRebuildLease(first); // superseded work cannot end the latest request's lease
        Assert.True(OverlayBuffer.IsRebuildLeased(id, nowMs + 2000));
        Assert.False(OverlayBuffer.IsRebuildLeased(
            id, first.ExpiresMs + 1)); // a stuck worker cannot retain a fossil forever
        OverlayBuffer.EndRebuildLease(latest);
        Assert.False(OverlayBuffer.IsRebuildLeased(id, nowMs + 2000));
    }

    [Fact]
    public void Wall_fresh_covered_batch_keeps_line_and_consumers_at_warp()
    {
        string id = nameof(Wall_fresh_covered_batch_keeps_line_and_consumers_at_warp);
        OverlayBuffer.ResetSessionStatics();
        var batch = Batch(id) with { SampleWallMs = 1000, CaptureSimSeconds = 100 };
        OverlayBuffer.Publish(batch);

        Assert.True(OverlayKernel.SamplesUsable(batch.SampleWallMs, nowWallMs: 1000));
        Assert.True(OverlayKernel.SamplesUsable(
            batch, nowWallMs: 1000, nowSimSeconds: 700));
        Assert.True(OverlayBuffer.LineSamplesUsable(
            id, batch, planned: false, nowMs: 1000, nowSimSeconds: 700));
        Assert.Same(batch, OverlayBuffer.ReadFresh(
            id, nowMs: 1000, nowSimSeconds: 700));
        Assert.Equal(100, batch.WithFreshStamp(2000).CaptureSimSeconds);
    }

    [Fact]
    public void Lagging_state_time_does_not_make_a_current_capture_stale()
    {
        string id = nameof(Lagging_state_time_does_not_make_a_current_capture_stale);
        OverlayBuffer.ResetSessionStatics();
        var batch = Batch(id) with
        {
            SampleT0 = -1_000_000,
            SampleWallMs = 1000,
            CaptureSimSeconds = 500,
        };
        OverlayBuffer.Publish(batch);

        Assert.Same(batch, OverlayBuffer.ReadFresh(
            id, nowMs: 1000, nowSimSeconds: 501));
    }

    [Fact]
    public void Generation_guard_rejects_old_worker_publications_without_retracting_current_data()
    {
        string id = nameof(Generation_guard_rejects_old_worker_publications_without_retracting_current_data);
        OverlayBuffer.ResetSessionStatics();
        int generation = OverlayWorker.CurrentGeneration;
        var current = Batch(id);
        Assert.True(OverlayBuffer.PublishIfCurrent(current, generation));

        var obsolete = Batch(id);
        Assert.False(OverlayBuffer.PublishIfCurrent(obsolete, generation - 1));
        Assert.Same(current, OverlayBuffer.Read(id));
        Assert.False(OverlayBuffer.PublishPlannedIfCurrent(obsolete, generation - 1));
        var currentPlanned = Batch(id);
        Assert.True(OverlayBuffer.PublishPlannedIfCurrent(currentPlanned, generation));
        Assert.False(OverlayBuffer.ClearPlannedIfCurrent(id, generation - 1));
        Assert.Same(current, OverlayBuffer.Read(id));
        Assert.Same(currentPlanned, OverlayBuffer.ReadPlanned(id));
    }

    [Fact]
    public void Batches_are_keyed_by_vessel_id()
    {
        OverlayBuffer.ResetSessionStatics();
        OverlayBuffer.Publish(Batch("a"));
        OverlayBuffer.Publish(Batch("b"));
        Assert.Equal("a", OverlayBuffer.Read("a")!.VesselId);
        Assert.Equal("b", OverlayBuffer.Read("b")!.VesselId);
        Assert.Null(OverlayBuffer.Read("c"));
        Assert.Equal(2, OverlayBuffer.PublishedCount);
    }

    [Fact]
    public void Republishing_a_vessel_replaces_its_batch()
    {
        OverlayBuffer.ResetSessionStatics();
        OverlayBuffer.Publish(Batch("a"));
        var newer = Batch("a");
        OverlayBuffer.Publish(newer);
        Assert.Same(newer, OverlayBuffer.Read("a"));
        Assert.Equal(1, OverlayBuffer.PublishedCount);
    }

    [Fact]
    public void Session_reset_clears_every_batch()
    {
        OverlayBuffer.Publish(Batch("a"));
        OverlayBuffer.ResetSessionStatics();
        Assert.Null(OverlayBuffer.Read("a"));
        Assert.Equal(0, OverlayBuffer.PublishedCount);
    }
}

/// <summary>Map-layer routing under SOI-independence. A fresh batch in the
/// current frame MODE draws; fresh but wrong mode blinks (drawing frame-space
/// coordinates under the wrong pose is geometry noise, and falling back to stock would
/// flash the exact conic lines the feature removes); only a genuinely stale batch or a
/// disabled mod hands the layer back to stock. There is deliberately NO fourth
/// parentMatches input (no SOI-transition blink): a stock
/// re-parent must never make the line disappear (the stage path re-anchors the batch
/// by <see cref="OverlayKernel.ParentShift"/> and keeps drawing).</summary>
public class LineRouteTests
{
    [Theory]
    [InlineData(true,  true,  true,  LineRoute.Draw)]
    [InlineData(true,  true,  false, LineRoute.Blink)]  // frame-mode mismatch window
    [InlineData(true,  false, true,  LineRoute.Stock)]  // stale: sampling stopped
    [InlineData(true,  false, false, LineRoute.Stock)]  // staleness dominates context
    [InlineData(false, true,  true,  LineRoute.Stock)]  // disabled: stock owns the map
    [InlineData(false, false, false, LineRoute.Stock)]
    public void Fresh_mode_draws_mode_mismatch_blinks_stale_or_disabled_goes_stock(
        bool enabled, bool fresh, bool modeMatches, LineRoute expected)
        => Assert.Equal(expected, OverlayKernel.RouteLine(enabled, fresh, modeMatches));
}

/// <summary>SOI-independence: when a stock SOI/patch transition re-parents
/// the orbit between publish and stage, the batch's parent-centred Cce points are
/// shifted by (batch parent now − current parent now) instead of blinking the line.
/// Stock draws each staged point at currentParent(now) + point, so the invariant is
/// that the DRAWN world position of a shifted point equals batchParent(now) + Cce —
/// exactly the pre-transition rendering, never displaced by the interbody distance.</summary>
public class ParentShiftTests
{
    [Fact]
    public void Drawn_world_position_is_preserved_across_a_reparent()
    {
        var cce = new WhiskerDynamics.Core.Vector3d(7e6, -1e5, 3e4);          // point relative to the OLD parent
        var batchParentNow = new WhiskerDynamics.Core.Vector3d(1.5e11, 2e9, -5e8);
        var currentParentNow = new WhiskerDynamics.Core.Vector3d(1.504e11, 2.3e9, -4e8); // e.g. Luna vs Terra
        var shift = OverlayKernel.ParentShift(batchParentNow, currentParentNow);
        // What stock renders after staging: current parent + shifted point. Tolerance
        // covers float re-association only (sub-millimeter at 1.5e11 m — invisible).
        var drawn = currentParentNow + (cce + shift);
        var expected = batchParentNow + cce;
        Assert.Equal(expected.X, drawn.X, 1e-3);
        Assert.Equal(expected.Y, drawn.Y, 1e-3);
        Assert.Equal(expected.Z, drawn.Z, 1e-3);
    }

    [Fact]
    public void Matching_parents_shift_nothing()
    {
        var parentNow = new WhiskerDynamics.Core.Vector3d(1.5e11, 2e9, -5e8);
        Assert.Equal(WhiskerDynamics.Core.Vector3d.Zero, OverlayKernel.ParentShift(parentNow, parentNow));
    }
}

/// <summary>The map SOI indicator is hidden only while n-body dynamics are enabled and
/// caught up, because the boundary has no dynamical meaning only after authority moves.</summary>
public class SoiIndicatorRuleTests
{
    [Theory]
    [InlineData(true,  true,  true,  true)]   // authority ready: hide
    [InlineData(true,  true,  false, false)]  // catch-up: stock remains authoritative
    [InlineData(true,  false, false, false)]
    [InlineData(false, true,  true,  false)]
    [InlineData(false, false, false, false)]
    public void Hidden_iff_mod_enabled_bound_and_rails_ready(
        bool enabled, bool bound, bool ready, bool hidden)
        => Assert.Equal(hidden, OverlayKernel.SoiIndicatorsHidden(enabled, bound, ready));
}

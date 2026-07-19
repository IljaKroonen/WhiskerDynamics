using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class FlownHistoryTests
{
    [Fact]
    public void Default_storage_policy_is_internal_and_fixed()
    {
        var settings = FlownHistorySettings.Default;

        Assert.Equal(40 * 365 * ModConfig.SecondsPerDay, settings.RetentionSeconds);
        Assert.Equal(10, settings.InitialToleranceMeters);
        Assert.Equal(262_144, settings.MaxPoints);
    }

    [Fact]
    public void Straight_path_simplifies_to_temporal_endpoints()
    {
        FlownSample[] source = Enumerable.Range(0, 101)
            .Select(t => new FlownSample(t, new Vector3d(2 * t, -3 * t, t)))
            .ToArray();

        FlownSample[] simplified = FlownHistory.Simplify(source, 10);

        Assert.Equal([source[0], source[^1]], simplified);
    }

    [Fact]
    public void Curved_path_stays_inside_the_positional_error_bound()
    {
        FlownSample[] source = Enumerable.Range(0, 201)
            .Select(t => new FlownSample(t,
                new Vector3d(t * 10, 100 * Math.Sin(t / 12.0), t * t * 0.01)))
            .ToArray();

        FlownSample[] simplified = FlownHistory.Simplify(source, 10);

        Assert.Equal(source[0], simplified[0]);
        Assert.Equal(source[^1], simplified[^1]);
        Assert.InRange(simplified.Length, 3, source.Length - 1);
        AssertPathErrorAtMost(source, simplified, 10);
    }

    [Fact]
    public void Dense_physics_capture_is_not_time_sampled()
    {
        var history = new FlownHistory();
        var settings = new FlownHistorySettings(10_000, 10, 10_000);
        FlownSample[] source = Enumerable.Range(0, 1001)
            .Select(t => new FlownSample(t,
                new Vector3d(t, 50 * Math.Sin(t / 20.0), 0)))
            .ToArray();

        foreach (var sample in source)
            Assert.True(history.Append(
                sample.TimeSeconds, sample.AbsolutePosition, sample.TimeSeconds, in settings));

        var retained = history.SnapshotRange(0, 1001, 10_000).Samples;
        Assert.True(retained.Length < source.Length);
        Assert.Equal(source[0], retained[0]);
        Assert.Equal(source[^1], retained[^1]);
        AssertPathErrorAtMost(source, retained, 10);
    }

    [Fact]
    public void Cap_pressure_relaxes_tolerance_without_losing_the_time_span()
    {
        var history = new FlownHistory();
        var settings = new FlownHistorySettings(10_000, 0.01, 8);
        for (int t = 0; t < 50; t++)
        {
            double y = t % 2 == 0 ? -100 : 100;
            history.Append(t, new Vector3d(t, y, 0), t, in settings);
        }

        var retained = history.SnapshotRange(0, 50, 100).Samples;

        Assert.InRange(history.Count, 2, settings.MaxPoints);
        Assert.True(history.EffectiveToleranceMeters > settings.InitialToleranceMeters);
        Assert.Equal(0, retained[0].TimeSeconds);
        Assert.Equal(49, retained[^1].TimeSeconds);
        FlownSample[] source = Enumerable.Range(0, 50)
            .Select(t => new FlownSample(
                t, new Vector3d(t, t % 2 == 0 ? -100 : 100, 0)))
            .ToArray();
        AssertPathErrorAtMost(source, retained, history.EffectiveToleranceMeters);
    }

    [Fact]
    public void Retention_prunes_before_the_internal_window()
    {
        var history = new FlownHistory();
        var settings = new FlownHistorySettings(10, 0.1, 128);
        history.Append(0, Vector3d.Zero, 10, in settings);
        history.Append(5, new Vector3d(5, 0, 0), 10, in settings);
        history.Append(10, new Vector3d(10, 0, 0), 10, in settings);

        history.Configure(20, in settings);

        var retained = history.SnapshotRange(0, 21, 128).Samples;
        var only = Assert.Single(retained);
        Assert.Equal(10, only.TimeSeconds);
    }

    [Fact]
    public void Range_snapshot_prioritizes_newest_points_and_reports_exact_coverage()
    {
        var history = ZigZagHistory();

        var snapshot = history.SnapshotRange(2, 10, 3);

        Assert.Equal([7.0, 8.0, 9.0],
            snapshot.Samples.Select(sample => sample.TimeSeconds));
        Assert.Equal(2, snapshot.Coverage.RequestedStartSeconds);
        Assert.Equal(0, snapshot.Coverage.OldestRecordedStartSeconds);
        Assert.Equal(7, snapshot.Coverage.OldestRenderedStartSeconds);
        Assert.True(snapshot.Coverage.RenderBudgetTruncated);
    }

    [Fact]
    public void Range_snapshot_interpolates_a_simplified_segment_at_the_requested_start()
    {
        var history = new FlownHistory();
        var settings = new FlownHistorySettings(100, 10, 128);
        history.Append(0, Vector3d.Zero, 0, in settings);
        history.Append(100, new Vector3d(100, 0, 0), 100, in settings);

        var snapshot = history.SnapshotRange(75, 100, 128);

        var boundary = Assert.Single(snapshot.Samples);
        Assert.Equal(75, boundary.TimeSeconds);
        Assert.Equal(new Vector3d(75, 0, 0), boundary.AbsolutePosition);
        Assert.False(snapshot.Coverage.RenderBudgetTruncated);
    }

    [Fact]
    public void Short_session_coverage_is_unavailable_not_render_truncated()
    {
        var history = ZigZagHistory();

        var snapshot = history.SnapshotRange(-100, 10, 100);

        Assert.Equal(0, snapshot.Coverage.OldestRecordedStartSeconds);
        Assert.Equal(0, snapshot.Coverage.OldestRenderedStartSeconds);
        Assert.False(snapshot.Coverage.RenderBudgetTruncated);
    }

    [Fact]
    public void Zero_display_hides_without_clearing_or_stopping_capture()
    {
        var history = new FlownHistory();
        var settings = FlownHistorySettings.Default;
        Assert.True(history.Append(1, new Vector3d(1, 2, 3), 1, in settings));

        var hidden = history.SnapshotRange(2, 2, TrajectoryOverlay.HistoryRenderPointLimit);
        Assert.Empty(hidden.Samples);
        Assert.Equal(1, history.Count);
        Assert.True(history.Append(2, new Vector3d(2, 3, 4), 2, in settings));

        var shown = history.SnapshotRange(0, 3, TrajectoryOverlay.HistoryRenderPointLimit);
        Assert.NotEmpty(shown.Samples);
    }

    [Fact]
    public void Default_history_can_precede_a_target_predictors_retained_window()
    {
        var config = new ModConfig();
        double now = 30 * ModConfig.SecondsPerDay;
        double historicalTime = now - 8 * ModConfig.SecondsPerDay;
        double targetPredictorStart = now - config.RailsKeepBehindDays * ModConfig.SecondsPerDay;

        var history = new FlownHistory();
        var settings = FlownHistorySettings.Default;
        Assert.True(history.Append(historicalTime, Vector3d.Zero, now, in settings));
        var retained = Assert.Single(
            history.SnapshotRange(0, now, TrajectoryOverlay.HistoryRenderPointLimit).Samples);

        var gravity = new GravityModel(new Ephemerides(
            [new CelestialBody { Id = "Void", Mu = 0 }]));
        var targetPredictor = new TrajectoryPredictor(
            gravity, new StateVector(Vector3d.Zero, Vector3d.Zero), targetPredictorStart);

        Assert.True(retained.TimeSeconds < targetPredictor.StartTime);
        var error = Assert.Throws<ArgumentOutOfRangeException>(
            () => targetPredictor.StateAt(retained.TimeSeconds));
        Assert.Contains("before trajectory start", error.Message);
    }

    [Fact]
    public void Rails_retention_follows_the_oldest_recorded_vessel_sample()
    {
        const double now = 1000;

        Assert.Equal(900, RailsService.RetentionCutoffSeconds(now, 100.0 / 86400.0));
        Assert.Equal(700, RailsService.RetentionCutoffSeconds(
            now, 100.0 / 86400.0, historyStartSeconds: 700));
        Assert.Equal(900, RailsService.RetentionCutoffSeconds(
            now, 100.0 / 86400.0, historyStartSeconds: 950));
    }

    [Fact]
    public void Curve_failure_preserves_active_frame_and_stale_failure_cannot_clear_new_frame()
    {
        var target = new FrameSpec(FrameKind.TargetFixed, "Earth", "Station");
        var inertial = new FrameSpec(FrameKind.Inertial, "Earth", null);
        FrameSpec? active = target;
        long generation = 10;
        var targetSnapshot = new ActiveFrameSnapshot(
            target, default, 0, default, generation);

        void RetireTarget() => FrameActivationKernel.TryDeactivate(
            ref active, ref generation, targetSnapshot);

        FramePoseFailurePolicy.OnFailure(FramePoseQuery.CurveSample, RetireTarget);
        Assert.Equal(target, active);
        Assert.Equal(10, generation);

        active = inertial;
        generation++;
        FramePoseFailurePolicy.OnFailure(FramePoseQuery.CurrentDisplay, RetireTarget);
        Assert.Equal(inertial, active);
        Assert.Equal(11, generation);

        var inertialSnapshot = targetSnapshot with { Spec = inertial, Generation = generation };
        FramePoseFailurePolicy.OnFailure(FramePoseQuery.CurrentDisplay, () =>
            FrameActivationKernel.TryDeactivate(ref active, ref generation, inertialSnapshot));
        Assert.Null(active);
        Assert.Equal(12, generation);
    }

    private static FlownHistory ZigZagHistory()
    {
        var history = new FlownHistory();
        var settings = new FlownHistorySettings(100, 0.01, 128);
        for (int t = 0; t < 10; t++)
            history.Append(t, new Vector3d(t, t % 2 == 0 ? -1 : 1, 0), t, in settings);
        return history;
    }

    private static void AssertPathErrorAtMost(
        IReadOnlyList<FlownSample> source,
        IReadOnlyList<FlownSample> simplified,
        double toleranceMeters)
    {
        int segment = 0;
        foreach (FlownSample sample in source)
        {
            while (segment + 1 < simplified.Count - 1
                && sample.TimeSeconds > simplified[segment + 1].TimeSeconds)
                segment++;
            FlownSample a = simplified[segment];
            FlownSample b = simplified[Math.Min(segment + 1, simplified.Count - 1)];
            double fraction = b.TimeSeconds == a.TimeSeconds
                ? 0
                : (sample.TimeSeconds - a.TimeSeconds) / (b.TimeSeconds - a.TimeSeconds);
            Vector3d interpolated = a.AbsolutePosition
                + (b.AbsolutePosition - a.AbsolutePosition) * fraction;
            Assert.True((sample.AbsolutePosition - interpolated).Length()
                <= toleranceMeters + 1e-9);
        }
    }
}

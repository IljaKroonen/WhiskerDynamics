using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Soi;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class OrbitAnalysisSchedulingTests
{
    [Fact]
    public void Analysis_window_does_not_extend_or_clip_to_the_display_window()
    {
        const double now = 5_000_000;
        const double tenYears = 10 * 365.25;
        const double integratedYears = 40 * 365.25;

        var horizons = TrajectoryOverlay.ResolveRebuildHorizons(
            configuredDisplayDays: 30,
            displayAvailableDays: 30,
            planEndSeconds: null,
            nowSeconds: now,
            integratedAvailableDays: integratedYears,
            analysisRequiredDays: tenYears);

        Assert.Equal(30, horizons.DisplayDays);
        Assert.Equal(tenYears, horizons.AnalysisDays);
    }

    [Fact]
    public void Analysis_window_is_clipped_only_to_integrated_rails_coverage()
    {
        const double now = 5_000_000;
        const double integratedDays = 40;

        var horizons = TrajectoryOverlay.ResolveRebuildHorizons(
            configuredDisplayDays: 10,
            displayAvailableDays: 10,
            planEndSeconds: now + 20 * 86400,
            nowSeconds: now,
            integratedAvailableDays: integratedDays,
            analysisRequiredDays: 100);

        Assert.Equal(10, horizons.DisplayDays);
        Assert.Equal(integratedDays, horizons.AnalysisDays);
    }

    [Fact]
    public void Wider_analysis_snapshot_miss_does_not_block_display_capture()
    {
        var display = new object();
        var displayRequests = new List<(double From, double To)>();
        var analysisRequests = new List<(double From, double To)>();
        object? CaptureDisplay(double from, double to)
        {
            displayRequests.Add((from, to));
            return display;
        }
        object? CaptureAnalysis(double from, double to)
        {
            analysisRequests.Add((from, to));
            return null;
        }

        var captures = TrajectoryOverlay.TryCapturePredictionContexts(
            CaptureDisplay,
            CaptureAnalysis,
            displayFrom: 90, displayTo: 130,
            analysisFrom: 100, analysisTo: 1000,
            analysisEnabled: true);

        Assert.NotNull(captures);
        Assert.Same(display, captures.Value.Display);
        Assert.Null(captures.Value.Analysis);
        Assert.Equal([(90, 130)], displayRequests);
        Assert.Equal([(100, 1000)], analysisRequests);
    }

    [Fact]
    public void Analysis_covered_by_display_reuses_the_display_snapshot()
    {
        var display = new object();
        int captures = 0;
        int analysisCaptures = 0;

        var result = TrajectoryOverlay.TryCapturePredictionContexts(
            (_, _) =>
            {
                captures++;
                return display;
            },
            (_, _) =>
            {
                analysisCaptures++;
                return display;
            },
            displayFrom: 90, displayTo: 200,
            analysisFrom: 100, analysisTo: 150,
            analysisEnabled: true);

        Assert.NotNull(result);
        Assert.Same(display, result.Value.Display);
        Assert.Same(display, result.Value.Analysis);
        Assert.Equal(1, captures);
        Assert.Equal(0, analysisCaptures);
    }

    [Fact]
    public void Analysis_request_bypasses_cooldown_only_until_that_version_is_admitted()
    {
        Assert.True(TrajectoryOverlay.AnalysisRequestNeedsUrgentCapture(
            enabled: true, requestVersion: 7, lastAdmittedVersion: 6));
        Assert.False(TrajectoryOverlay.AnalysisRequestNeedsUrgentCapture(
            enabled: true, requestVersion: 7, lastAdmittedVersion: 7));
        Assert.False(TrajectoryOverlay.AnalysisRequestNeedsUrgentCapture(
            enabled: false, requestVersion: 8, lastAdmittedVersion: 7));
    }

    [Fact]
    public void Missing_analysis_snapshot_retries_are_throttled_separately_from_admission()
    {
        Assert.True(TrajectoryOverlay.AnalysisSnapshotAttemptDue(
            enabled: true, requestVersion: 7, lastAdmittedVersion: 6,
            lastAttemptVersion: 6, nowMs: 1000, lastAttemptMs: 999));
        Assert.False(TrajectoryOverlay.AnalysisSnapshotAttemptDue(
            enabled: true, requestVersion: 7, lastAdmittedVersion: 6,
            lastAttemptVersion: 7, nowMs: 1249, lastAttemptMs: 1000,
            retryPeriodMs: 250));
        Assert.True(TrajectoryOverlay.AnalysisSnapshotAttemptDue(
            enabled: true, requestVersion: 7, lastAdmittedVersion: 6,
            lastAttemptVersion: 7, nowMs: 1250, lastAttemptMs: 1000,
            retryPeriodMs: 250));
        Assert.False(TrajectoryOverlay.AnalysisSnapshotAttemptDue(
            enabled: true, requestVersion: 7, lastAdmittedVersion: 7,
            lastAttemptVersion: 6, nowMs: 1000, lastAttemptMs: 0));
    }

    [Fact]
    public void Delayed_analysis_start_resolves_the_deepest_nested_soi_owner()
    {
        var children = new Dictionary<string, IReadOnlyList<string>>
        {
            ["Sol"] = ["Earth"],
            ["Earth"] = ["Moon"],
            ["Moon"] = [],
        };
        var positions = new Dictionary<string, Vector3d>
        {
            ["Sol"] = Vector3d.Zero,
            ["Earth"] = new(100, 0, 0),
            ["Moon"] = new(110, 0, 0),
        };
        var sois = new Dictionary<string, double>
        {
            ["Earth"] = 50,
            ["Moon"] = 5,
        };

        string owner = TrajectoryOverlay.AnalysisBodyAtStart(
            "Sol", new Vector3d(111, 0, 0), id => children[id],
            id => positions[id], id => sois[id]);

        Assert.Equal("Moon", owner);
    }

    [Fact]
    public void Delayed_analysis_start_falls_back_to_the_root_soi()
    {
        string owner = TrajectoryOverlay.AnalysisBodyAtStart(
            "Sol", new Vector3d(1000, 0, 0),
            id => id == "Sol" ? ["Earth"] : [],
            _ => Vector3d.Zero,
            _ => 100);

        Assert.Equal("Sol", owner);
    }

    [Fact]
    public void Analysis_start_soi_ties_preserve_child_enumeration_order()
    {
        string owner = TrajectoryOverlay.AnalysisBodyAtStart(
            "Root", Vector3d.Zero,
            id => id == "Root" ? ["First", "Second"] : [],
            _ => Vector3d.Zero,
            _ => 10);

        Assert.Equal("First", owner);
    }

    [Fact]
    public void Analysis_soi_sweep_detects_a_complete_child_flyby_between_samples()
    {
        double[] times = [0, 10];
        Vector3d[] bodyRelative = [new(100, 0, 0), new(100, 0, 0)];
        var child = new TrajectoryOverlay.AnalysisSoiRelativeSeries(
            "Moon", 5, [new(-20, 0, 0), new(20, 0, 0)]);

        var transition = TrajectoryOverlay.FindFirstAnalysisSoiTransition(
            times, bodyRelative, bodySoi: 1000, parentBodyId: "Sol", [child]);

        Assert.NotNull(transition);
        Assert.Equal("Moon", transition.Value.NewBodyId);
        Assert.Equal(3.75, transition.Value.TimeSeconds, 10);
    }

    [Fact]
    public void Analysis_soi_sweep_truncates_at_a_frozen_body_escape()
    {
        var transition = TrajectoryOverlay.FindFirstAnalysisSoiTransition(
            [0, 10], [new(90, 0, 0), new(110, 0, 0)],
            bodySoi: 100, parentBodyId: "Sol", []);

        Assert.NotNull(transition);
        Assert.Equal("Sol", transition.Value.NewBodyId);
        Assert.Equal(5, transition.Value.TimeSeconds, 10);
    }
}

using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public sealed class OverlaySchedulingTests
{
    private static TrackedVessel Vessel() => new()
    {
        Id = "test",
        Rails = null!,
        Options = new IntegratorOptions(),
    };

    [Fact]
    public void Only_one_capture_can_be_reserved_per_vessel()
    {
        var vessel = Vessel();
        Assert.True(vessel.TryBeginOverlayRebuild(1000, 250, urgent: false));
        Assert.False(vessel.TryBeginOverlayRebuild(2000, 250, urgent: true));
        vessel.CancelOverlayRebuild();
        Assert.True(vessel.TryBeginOverlayRebuild(2001, 250, urgent: false));
    }

    [Fact]
    public void Ordinary_rebuild_waits_for_previous_cost_but_urgent_change_bypasses_cooldown()
    {
        var vessel = Vessel();
        Assert.True(vessel.TryBeginOverlayRebuild(100, 250, urgent: false));
        vessel.CommitOverlayRebuild(100);
        vessel.CompleteOverlayRebuild(durationMs: 700, completedMs: 1000);

        Assert.False(vessel.TryBeginOverlayRebuild(1699, 250, urgent: false));
        Assert.True(vessel.TryBeginOverlayRebuild(1001, 250, urgent: true));
    }

    [Fact]
    public void Completion_backoff_matches_long_previous_cost()
    {
        var vessel = Vessel();
        Assert.True(vessel.TryBeginOverlayRebuild(100, 250, urgent: false));
        vessel.CommitOverlayRebuild(100);
        vessel.CompleteOverlayRebuild(durationMs: 9000, completedMs: 1000);

        Assert.False(vessel.TryBeginOverlayRebuild(9999, 250, urgent: false));
        Assert.True(vessel.TryBeginOverlayRebuild(10000, 250, urgent: false));
    }

    [Fact]
    public void Urgent_rebuilds_keep_a_small_floor_from_the_last_enqueue()
    {
        var vessel = Vessel();
        Assert.True(vessel.TryBeginOverlayRebuild(100, 1000, urgent: false));
        vessel.CommitOverlayRebuild(100);

        Assert.False(vessel.TryBeginOverlayRebuild(149, 1000, urgent: true));
        Assert.True(vessel.TryBeginOverlayRebuild(150, 1000, urgent: true));
    }

    [Fact]
    public void Continuous_rebuilds_ignore_cadence_and_previous_job_cost()
    {
        var vessel = Vessel();
        Assert.True(vessel.TryBeginContinuousOverlayRebuild());
        vessel.CommitOverlayRebuild(100);
        vessel.CompleteOverlayRebuild(durationMs: 9000, completedMs: 9100);

        Assert.True(vessel.TryBeginContinuousOverlayRebuild());
    }

    [Fact]
    public void Cadence_eligible_capture_replaces_the_first_pending_job()
    {
        var vessel = Vessel();
        var queue = new KeyedLatestQueue((_, _) => { });
        var ran = new List<int>();

        Assert.True(vessel.TryBeginOverlayRebuild(1000, 250, urgent: false));
        queue.Enqueue(vessel.Id, () => ran.Add(1));
        vessel.CommitOverlayRebuild(1000);

        Assert.True(vessel.TryBeginOverlayRebuild(1250, 250, urgent: false));
        queue.Enqueue(vessel.Id, () => ran.Add(2));
        vessel.CommitOverlayRebuild(1250);

        Assert.Equal(1, queue.Drain());
        Assert.Equal([2], ran);
    }

    [Fact]
    public void Old_job_completion_cannot_release_a_newer_capture_reservation()
    {
        var vessel = Vessel();
        var queue = new KeyedLatestQueue((_, _) => { });

        Assert.True(vessel.TryBeginOverlayRebuild(1000, 250, urgent: false));
        queue.Enqueue(vessel.Id,
            () => vessel.CompleteOverlayRebuild(durationMs: 250, completedMs: 1250));
        vessel.CommitOverlayRebuild(1000);

        Assert.True(vessel.TryBeginOverlayRebuild(1250, 250, urgent: false));
        Assert.Equal(1, queue.Drain());
        Assert.False(vessel.TryBeginOverlayRebuild(1300, 250, urgent: true));
        vessel.CancelOverlayRebuild();
        Assert.True(vessel.TryBeginOverlayRebuild(1300, 250, urgent: true));
    }

    [Fact]
    public void Old_analysis_completion_cannot_release_a_newer_request()
    {
        var vessel = Vessel();

        var old = vessel.BeginOverlayAnalysis(7);
        Assert.True(vessel.IsOverlayAnalysisInFlight(7));

        var fresh = vessel.BeginOverlayAnalysis(8);
        vessel.CompleteOverlayAnalysis(old);
        Assert.True(vessel.IsOverlayAnalysisInFlight(8));

        vessel.CompleteOverlayAnalysis(fresh);
        Assert.False(vessel.IsOverlayAnalysisInFlight(8));
    }

    [Fact]
    public void Old_analysis_completion_cannot_release_a_newer_job_for_the_same_request()
    {
        var vessel = Vessel();

        var old = vessel.BeginOverlayAnalysis(7);
        var fresh = vessel.BeginOverlayAnalysis(7);
        vessel.CompleteOverlayAnalysis(old);

        Assert.True(vessel.IsOverlayAnalysisInFlight(7));
        vessel.CompleteOverlayAnalysis(fresh);
        Assert.False(vessel.IsOverlayAnalysisInFlight(7));
    }

    [Fact]
    public void Analysis_admission_is_single_flight_and_cost_cooled()
    {
        var vessel = Vessel();
        var lease = Assert.IsType<TrackedVessel.OverlayAnalysisLease>(
            vessel.TryBeginOverlayAnalysis(7, nowMs: 1000, urgent: true));

        Assert.Null(vessel.TryBeginOverlayAnalysis(7, nowMs: 1100, urgent: false));
        vessel.CompleteOverlayAnalysis(lease, completedMs: 1300);

        Assert.Null(vessel.TryBeginOverlayAnalysis(7, nowMs: 1599, urgent: false));
        Assert.NotNull(vessel.TryBeginOverlayAnalysis(7, nowMs: 1600, urgent: false));
    }

    [Fact]
    public void Analysis_discard_releases_without_a_false_cost_cooldown()
    {
        var vessel = Vessel();
        var discarded = Assert.IsType<TrackedVessel.OverlayAnalysisLease>(
            vessel.TryBeginOverlayAnalysis(7, nowMs: 1000, urgent: true));

        vessel.CancelOverlayAnalysis(discarded);

        Assert.NotNull(vessel.TryBeginOverlayAnalysis(
            7, nowMs: 1001, urgent: false));
    }

    [Fact]
    public void Geometry_replacement_acquires_analysis_only_for_the_latest_handoff()
    {
        var vessel = Vessel();
        var queue = new KeyedLatestQueue((_, _) => { });
        TrackedVessel.OverlayAnalysisLease? displaced = null;
        TrackedVessel.OverlayAnalysisLease? latest = null;

        queue.Enqueue(vessel.Id, () => displaced =
            TrajectoryOverlay.TryBeginOverlayAnalysisHandoff(
                vessel, analysisWorkEnabled: true, requestVersion: 7, nowMs: 1000));
        queue.Enqueue(vessel.Id, () => latest =
            TrajectoryOverlay.TryBeginOverlayAnalysisHandoff(
                vessel, analysisWorkEnabled: true, requestVersion: 7, nowMs: 1001));

        Assert.False(vessel.IsOverlayAnalysisInFlight(7));
        Assert.Equal(1, queue.Drain());
        Assert.Null(displaced);
        Assert.NotNull(latest);
        Assert.True(vessel.IsOverlayAnalysisInFlight(7));
    }

    [Fact]
    public void Successful_analysis_handoff_consumes_the_request_urgent_bypass()
    {
        var vessel = Vessel();
        var admitted = Assert.IsType<TrackedVessel.OverlayAnalysisLease>(
            vessel.TryBeginOverlayAnalysis(7, nowMs: 1000, urgent: true));

        vessel.RecordOverlayAnalysisAdmission(admitted, admittedMs: 1200);

        Assert.Equal(1200, vessel.LastOverlayAnalysisLoopMs);
        Assert.False(TrajectoryOverlay.AnalysisRequestNeedsUrgentCapture(
            enabled: true, requestVersion: 7,
            lastAdmittedVersion: vessel.LastOverlayAnalysisRequestVersion));
    }

    [Fact]
    public void Rejected_analysis_publication_releases_without_cost_cooldown()
    {
        var vessel = Vessel();
        var rejected = Assert.IsType<TrackedVessel.OverlayAnalysisLease>(
            vessel.TryBeginOverlayAnalysis(7, nowMs: 1000, urgent: true));

        vessel.FinishOverlayAnalysis(rejected, publicationAccepted: false,
            completedMs: 5000, startedMs: 1000);

        Assert.NotNull(vessel.TryBeginOverlayAnalysis(
            7, nowMs: 5001, urgent: false));
    }

    [Fact]
    public void Published_analysis_still_uses_duration_aware_cooldown()
    {
        var vessel = Vessel();
        var published = Assert.IsType<TrackedVessel.OverlayAnalysisLease>(
            vessel.TryBeginOverlayAnalysis(7, nowMs: 1000, urgent: true));

        vessel.FinishOverlayAnalysis(published, publicationAccepted: true,
            completedMs: 5000, startedMs: 1000);

        Assert.Null(vessel.TryBeginOverlayAnalysis(
            7, nowMs: 8999, urgent: false));
        Assert.NotNull(vessel.TryBeginOverlayAnalysis(
            7, nowMs: 9000, urgent: false));
    }

    [Fact]
    public void Replacing_pending_analysis_with_non_analysis_work_releases_its_marker()
    {
        var vessel = Vessel();
        var queue = new KeyedLatestQueue((_, _) => { });
        var lease = vessel.BeginOverlayAnalysis(7);
        queue.Enqueue(vessel.Id, static () => { },
            () => vessel.CompleteOverlayAnalysis(lease));

        queue.Enqueue(vessel.Id, static () => { });

        Assert.False(vessel.IsOverlayAnalysisInFlight(7));
    }

    [Fact]
    public void Cancelling_pending_analysis_releases_its_marker()
    {
        var vessel = Vessel();
        var queue = new KeyedLatestQueue((_, _) => { });
        var lease = vessel.BeginOverlayAnalysis(7);
        queue.Enqueue(vessel.Id, static () => { },
            () => vessel.CompleteOverlayAnalysis(lease));

        queue.Cancel(vessel.Id);

        Assert.False(vessel.IsOverlayAnalysisInFlight(7));
    }
}

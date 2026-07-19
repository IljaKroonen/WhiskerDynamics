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
}

using WhiskerDynamics.Mod;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Planning;

/// <summary>Tests the writer's main-thread guard and snapshot containment.</summary>
public class BurnPlanWriterGuardTests
{
    [Fact]
    public void Wrong_thread_calls_are_rejected_and_the_captured_thread_passes()
    {
        BurnPlanWriter.CaptureMainThread();
        Assert.Null(BurnPlanWriter.RejectIfOffMainThread());

        string? offThread = "unset";
        var background = new Thread(() => offThread = BurnPlanWriter.RejectIfOffMainThread());
        background.Start();
        background.Join();
        // A dedicated thread guarantees a distinct managed thread id.
        Assert.Equal("rejected: wrong thread", offThread);
    }

    [Fact]
    public void Equatorial_pole_capture_defers_without_consuming_the_main_thread_once()
    {
        BurnPlanWriter.CaptureMainThread();
        int started = 0;
        bool offThreadStarted = true;
        var background = new Thread(() =>
            offThreadStarted = RailsService.BeginEquatorialPoleCapture(ref started));
        background.Start();
        background.Join();

        Assert.False(offThreadStarted);
        Assert.Equal(0, started);
        Assert.True(RailsService.BeginEquatorialPoleCapture(ref started));
        Assert.Equal(1, started);
        Assert.False(RailsService.BeginEquatorialPoleCapture(ref started));
    }

    [Fact]
    public void Failed_equatorial_pole_capture_rearms_after_a_bounded_delay()
    {
        int started = 1;
        long nextAttemptMs = 0;

        RailsService.RearmEquatorialPoleCaptureAfterFailure(
            ref started, ref nextAttemptMs, nowMs: 4000);

        Assert.Equal(0, started);
        Assert.Equal(5000, nextAttemptMs);
    }

    [Fact]
    public void Snapshot_containment_returns_empty_for_a_throwing_reader()
    {
        var result = BurnPlanWriter.ContainSnapshot<object?, int>(
            static _ => throw new InvalidOperationException("SIMULATED snapshot fault"),
            null);
        Assert.Empty(result);
    }

    [Fact]
    public void Snapshot_containment_passes_a_healthy_reader_through()
    {
        int[] backing = [3, 1, 4];
        var result = BurnPlanWriter.ContainSnapshot(static (int[] s) => (IReadOnlyList<int>)s, backing);
        Assert.Same(backing, result);
    }

    [Fact]
    public void Transactional_edit_restores_the_prior_value_when_update_throws()
    {
        int liveValue = 7;

        var error = Assert.Throws<InvalidOperationException>(() =>
            BurnPlanWriter.ApplyTransactional(
                previous: 7, replacement: 11,
                assign: value => liveValue = value,
                update: () => throw new InvalidOperationException("SIMULATED update fault")));

        Assert.Equal("SIMULATED update fault", error.Message);
        Assert.Equal(7, liveValue);
    }

    [Fact]
    public void Transactional_edit_keeps_the_replacement_after_a_successful_update()
    {
        int liveValue = 7;
        int updates = 0;

        BurnPlanWriter.ApplyTransactional(
            previous: 7, replacement: 11,
            assign: value => liveValue = value,
            update: () => updates++);

        Assert.Equal(11, liveValue);
        Assert.Equal(1, updates);
    }
}

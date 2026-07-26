using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Overlay;

/// <summary>The overlay worker's queue core: one
/// LATEST-WINS slot per key — a producer outrunning the worker replaces its own
/// pending job, so backpressure degrades cadence instead of queueing unbounded —
/// and a throwing job is contained per-drain. Deterministic: the queue core is
/// threadless; OverlayWorker is just a background loop around Drain().</summary>
public class KeyedLatestQueueTests
{
    private static KeyedLatestQueue NewQueue(List<(string Key, string Error)>? errors = null)
        => new((key, e) => errors?.Add((key, e.Message)));

    [Fact]
    public void Latest_wins_per_key_and_drain_runs_each_key_once()
    {
        var queue = NewQueue();
        var ran = new List<string>();
        queue.Enqueue("a", () => ran.Add("a-stale"));
        queue.Enqueue("a", () => ran.Add("a-fresh")); // replaces the pending slot
        queue.Enqueue("b", () => ran.Add("b"));
        Assert.Equal(2, queue.PendingCount);
        Assert.Equal(2, queue.Drain());
        Assert.DoesNotContain("a-stale", ran);
        Assert.Contains("a-fresh", ran);
        Assert.Contains("b", ran);
        Assert.Equal(0, queue.PendingCount);
    }

    [Fact]
    public void A_throwing_job_is_contained_and_does_not_starve_other_keys()
    {
        var errors = new List<(string Key, string Error)>();
        var queue = NewQueue(errors);
        bool survivorRan = false;
        queue.Enqueue("bad", () => throw new InvalidOperationException("boom"));
        queue.Enqueue("good", () => survivorRan = true);
        Assert.Equal(2, queue.Drain()); // both count as ran; neither throws upward
        Assert.True(survivorRan);
        var error = Assert.Single(errors);
        Assert.Equal("bad", error.Key);
        Assert.Equal("boom", error.Error);
    }

    [Fact]
    public void Clear_drops_pending_jobs()
    {
        var queue = NewQueue();
        bool ran = false;
        bool discarded = false;
        queue.Enqueue("stale-session", () => ran = true, () => discarded = true);
        queue.Clear();
        Assert.Equal(0, queue.Drain());
        Assert.False(ran);
        Assert.True(discarded);
    }

    [Fact]
    public void Replacement_and_cancel_discard_each_pending_job_once()
    {
        var queue = NewQueue();
        int firstDiscarded = 0;
        int secondDiscarded = 0;
        queue.Enqueue("v", static () => { }, () => firstDiscarded++);
        queue.Enqueue("v", static () => { }, () => secondDiscarded++);

        Assert.Equal(1, firstDiscarded);
        Assert.Equal(0, secondDiscarded);

        queue.Cancel("v");
        Assert.Equal(1, firstDiscarded);
        Assert.Equal(1, secondDiscarded);
    }

    [Fact]
    public void Enqueue_during_drain_lands_in_a_later_drain_not_lost()
    {
        var queue = NewQueue();
        int outerRan = 0, innerRan = 0;
        queue.Enqueue("a", () =>
        {
            outerRan++;
            queue.Enqueue("a", () => innerRan++); // producer outruns the worker mid-drain
        });
        queue.Drain();
        Assert.Equal(1, outerRan);
        Assert.True(queue.PendingCount == 1 || innerRan == 1); // same-drain pickup is allowed
        queue.Drain();
        Assert.Equal(1, innerRan);
    }

    [Fact]
    public void Bounded_drain_is_fifo_fair_across_keys()
    {
        var queue = NewQueue();
        var ran = new List<string>();
        queue.Enqueue("a", () => ran.Add("a"));
        queue.Enqueue("b", () => ran.Add("b"));
        queue.Enqueue("c", () => ran.Add("c"));

        Assert.Equal(1, queue.Drain(maxJobs: 1));
        Assert.Equal(["a"], ran);
        Assert.Equal(2, queue.PendingCount);
        queue.Drain(maxJobs: 1);
        queue.Drain(maxJobs: 1);
        Assert.Equal(["a", "b", "c"], ran);
    }

    [Fact]
    public void Newer_same_key_ticket_supersedes_older_ticket()
    {
        var queue = NewQueue();
        long old = queue.Enqueue("v", () => { });
        long fresh = queue.Enqueue("v", () => { });
        Assert.False(queue.IsCurrent("v", old));
        Assert.True(queue.IsCurrent("v", fresh));
        queue.Clear();
        Assert.False(queue.IsCurrent("v", fresh));
    }

    [Fact]
    public async Task Running_ticket_completes_while_pending_latest_wins()
    {
        var queue = NewQueue();
        using var started = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        var ran = new List<string>();
        long runningTicket = -1;
        runningTicket = queue.Enqueue("v", version => () =>
        {
            Assert.Equal(version, runningTicket);
            started.Set();
            Assert.True(release.Wait(1000));
            Assert.True(queue.IsCurrent("v", version));
            Assert.True(queue.RunIfCurrent("v", version, () => ran.Add("a")));
        });

        var drain = Task.Run(() => queue.Drain(maxJobs: 1));
        Assert.True(started.Wait(1000));
        long stalePending = queue.Enqueue("v", () => ran.Add("b"));
        long latestPending = queue.Enqueue("v", () => ran.Add("c"));

        Assert.True(queue.IsCurrent("v", runningTicket));
        Assert.False(queue.IsCurrent("v", stalePending));
        Assert.False(queue.IsCurrent("v", latestPending));
        release.Set();
        Assert.Equal(1, await drain);
        Assert.True(queue.IsCurrent("v", latestPending));
        Assert.Equal(1, queue.Drain());
        Assert.Equal(["a", "c"], ran);
        Assert.False(queue.IsCurrent("v", latestPending));
    }

    [Fact]
    public async Task Clear_invalidates_a_running_ticket()
    {
        var queue = NewQueue();
        using var started = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        bool published = false;
        long ticket = queue.Enqueue("v", version => () =>
        {
            started.Set();
            Assert.True(release.Wait(1000));
            published = queue.RunIfCurrent("v", version, () => { });
        });

        var drain = Task.Run(() => queue.Drain(maxJobs: 1));
        Assert.True(started.Wait(1000));
        Assert.True(queue.IsCurrent("v", ticket));
        queue.Clear();
        Assert.False(queue.IsCurrent("v", ticket));
        release.Set();
        Assert.Equal(1, await drain);
        Assert.False(published);
    }

    [Fact]
    public async Task Cancel_revokes_running_and_pending_work_for_only_one_key()
    {
        var queue = NewQueue();
        using var started = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        bool runningPublished = false;
        bool pendingRan = false;
        bool otherRan = false;
        long runningTicket = queue.Enqueue("v", version => () =>
        {
            started.Set();
            Assert.True(release.Wait(1000));
            runningPublished = queue.RunIfCurrent("v", version, static () => { });
        });

        var drain = Task.Run(() => queue.Drain(maxJobs: 1));
        Assert.True(started.Wait(1000));
        queue.Enqueue("v", () => pendingRan = true);
        queue.Enqueue("other", () => otherRan = true);
        Assert.True(queue.IsCurrent("v", runningTicket));

        queue.Cancel("v");
        release.Set();
        Assert.Equal(1, await drain);
        queue.Drain();

        Assert.False(runningPublished);
        Assert.False(pendingRan);
        Assert.True(otherRan);
    }

    [Fact]
    public async Task Conditional_publication_is_atomic_against_a_newer_enqueue()
    {
        var queue = NewQueue();
        long ticket = queue.Enqueue("v", () => { });
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        using var enqueueStarted = new ManualResetEventSlim();
        var publication = Task.Run(() => queue.RunIfCurrent("v", ticket, () =>
        {
            entered.Set();
            release.Wait();
        }));
        Assert.True(entered.Wait(1000));
        var newer = Task.Run(() =>
        {
            enqueueStarted.Set();
            return queue.Enqueue("v", () => { });
        });
        Assert.True(enqueueStarted.Wait(1000));
        await Task.Delay(50);
        Assert.False(newer.IsCompleted);
        release.Set();
        Assert.True(await publication);
        long newerTicket = await newer;
        Assert.False(queue.IsCurrent("v", ticket));
        Assert.True(queue.IsCurrent("v", newerTicket));
    }

    [Fact]
    public void Ticket_is_initialized_before_runnable_becomes_visible_and_cleans_up()
    {
        var queue = NewQueue();
        long observed = -1;
        long ticket = queue.Enqueue("v", version => () =>
        {
            observed = version;
            Assert.True(queue.IsCurrent("v", version));
        });
        Assert.Equal(1, queue.Drain());
        Assert.Equal(ticket, observed);
        Assert.False(queue.IsCurrent("v", ticket));
    }
}

[Collection(nameof(OrbitCacheCoordinationTestCollection))]
public sealed class OverlayWorkerAdmissionTests
{
    [Fact]
    public async Task Session_reset_takes_analysis_queue_gate_before_overlay_queue_gate()
    {
        OverlayWorker.ResetSessionStatics();
        Task<bool>? enqueue = null;
        using var enqueueStarted = new ManualResetEventSlim();
        string key = "reset-lock-order-" + Guid.NewGuid().ToString("N");

        OverlayWorker.ResetSessionStatics(() =>
        {
            int generation = OverlayWorker.CurrentGeneration;
            enqueue = Task.Factory.StartNew(() =>
            {
                enqueueStarted.Set();
                return OverlayAnalysisWorker.Enqueue(
                    key, generation, static () => true, static (_, _) => { });
            }, CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);

            Assert.True(enqueueStarted.Wait(TimeSpan.FromSeconds(5)));
            Assert.False(enqueue.Wait(TimeSpan.FromMilliseconds(100)));
        });

        Assert.NotNull(enqueue);
        Assert.True(await enqueue.WaitAsync(TimeSpan.FromSeconds(5)));
        OverlayAnalysisWorker.Cancel(key);
    }

    [Fact]
    public void Blocking_analysis_does_not_block_the_overlay_worker()
    {
        TrajectoryOverlay.ResetSessionStatics();
        int generation = OverlayWorker.CurrentGeneration;
        using var analysisStarted = new ManualResetEventSlim();
        using var releaseAnalysis = new ManualResetEventSlim();
        using var analysisFinished = new ManualResetEventSlim();
        using var overlayRan = new ManualResetEventSlim();
        string suffix = Guid.NewGuid().ToString("N");

        Assert.True(OverlayAnalysisWorker.Enqueue(
            "analysis-isolation-" + suffix, generation, static () => true,
            (_, _) =>
            {
                analysisStarted.Set();
                try
                {
                    Assert.True(releaseAnalysis.Wait(TimeSpan.FromSeconds(5)));
                }
                finally
                {
                    analysisFinished.Set();
                }
            }));
        Assert.True(analysisStarted.Wait(TimeSpan.FromSeconds(5)));
        try
        {
            Assert.True(OverlayWorker.Enqueue(
                "overlay-isolation-" + suffix, generation, overlayRan.Set));
            Assert.True(overlayRan.Wait(TimeSpan.FromSeconds(2)));
        }
        finally
        {
            releaseAnalysis.Set();
            Assert.True(analysisFinished.Wait(TimeSpan.FromSeconds(5)));
        }
    }

    [Fact]
    public async Task Old_producer_resuming_after_reset_cannot_replace_new_pending_job()
    {
        TrajectoryOverlay.ResetSessionStatics();
        int oldGeneration = OverlayWorker.CurrentGeneration;
        int oldBindingCurrent = 1;
        using var oldProducerPoised = new ManualResetEventSlim();
        using var resumeOldProducer = new ManualResetEventSlim();
        using var blockerStarted = new ManualResetEventSlim();
        using var releaseBlocker = new ManualResetEventSlim();
        using var freshRan = new ManualResetEventSlim();
        int stalePublished = 0;
        string key = "admission-reset-" + Guid.NewGuid().ToString("N");

        Task<bool> oldProducer = Task.Run(() =>
        {
            oldProducerPoised.Set();
            Assert.True(resumeOldProducer.Wait(TimeSpan.FromSeconds(5)));
            // This is the reported race: the old capture reads the NEW worker
            // generation after reset. Its binding predicate must reject under the
            // same queue gate that would otherwise replace the fresh pending slot.
            int borrowedGeneration = OverlayWorker.CurrentGeneration;
            return OverlayWorker.Enqueue(key, borrowedGeneration,
                () => Volatile.Read(ref oldBindingCurrent) != 0,
                (_, publish) =>
                {
                    publish(() => Interlocked.Increment(ref stalePublished));
                });
        });

        Assert.True(oldProducerPoised.Wait(TimeSpan.FromSeconds(5)));
        Volatile.Write(ref oldBindingCurrent, 0);
        TrajectoryOverlay.ResetSessionStatics();
        int newGeneration = OverlayWorker.CurrentGeneration;
        Assert.NotEqual(oldGeneration, newGeneration);
        Assert.True(OverlayWorker.Enqueue(key, newGeneration, () =>
        {
            blockerStarted.Set();
            Assert.True(releaseBlocker.Wait(TimeSpan.FromSeconds(5)));
        }));
        Assert.True(blockerStarted.Wait(TimeSpan.FromSeconds(5)));
        Assert.True(OverlayWorker.Enqueue(key, newGeneration, freshRan.Set));

        resumeOldProducer.Set();
        Assert.False(await oldProducer.WaitAsync(TimeSpan.FromSeconds(5)));
        releaseBlocker.Set();
        Assert.True(freshRan.Wait(TimeSpan.FromSeconds(5)));
        Assert.Equal(0, Volatile.Read(ref stalePublished));
    }
}

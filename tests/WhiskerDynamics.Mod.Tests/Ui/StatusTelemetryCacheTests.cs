using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Ui;

public sealed class StatusTelemetryCacheTests
{
    private const long RefreshMilliseconds = 500;
    private static readonly TimeSpan CoordinationTimeout = TimeSpan.FromSeconds(5);

    [Fact]
    public void First_read_refreshes_then_reuses_the_same_snapshot_until_due()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        int calls = 0;
        IReadOnlyList<string> first = ["first"];
        IReadOnlyList<string> second = ["second"];

        var initial = cache.Read(1_000, () =>
        {
            calls++;
            return first;
        });
        var cached = cache.Read(1_499, () =>
        {
            calls++;
            return second;
        });

        Assert.Same(first, initial);
        Assert.Same(initial, cached);
        Assert.Equal(1, calls);

        var refreshed = cache.Read(1_500, () =>
        {
            calls++;
            return second;
        });

        Assert.Same(second, refreshed);
        Assert.Equal(2, calls);
    }

    [Fact]
    public void Reset_clears_old_lines_and_makes_the_next_read_immediately_due()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> oldLines = ["old system"];
        IReadOnlyList<string> newLines = ["new system"];
        Assert.Same(oldLines, cache.Read(10_000, () => oldLines));

        cache.Reset();
        int calls = 0;
        var refreshed = cache.Read(10_001, () =>
        {
            calls++;
            return newLines;
        });

        Assert.Same(newLines, refreshed);
        Assert.NotSame(oldLines, refreshed);
        Assert.Equal(1, calls);
    }

    [Fact]
    public void Reset_does_not_restore_old_lines_when_the_immediate_refresh_throws()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> oldLines = ["old system"];
        Assert.Same(oldLines, cache.Read(1_000, () => oldLines));

        cache.Reset();
        Assert.Throws<InvalidOperationException>(() => cache.Read(1_001,
            () => throw new InvalidOperationException("new system not ready")));

        int retries = 0;
        var cachedFailure = cache.Read(1_500, () =>
        {
            retries++;
            return ["unexpected retry"];
        });
        Assert.Empty(cachedFailure);
        Assert.NotSame(oldLines, cachedFailure);
        Assert.Equal(0, retries);
    }

    [Fact]
    public void Wall_clock_regression_refreshes_immediately()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> beforeRegression = ["before"];
        IReadOnlyList<string> afterRegression = ["after"];
        Assert.Same(beforeRegression, cache.Read(10_000, () => beforeRegression));

        int calls = 0;
        var refreshed = cache.Read(9_999, () =>
        {
            calls++;
            return afterRegression;
        });

        Assert.Same(afterRegression, refreshed);
        Assert.Equal(1, calls);
    }

    [Fact]
    public void Throwing_refresh_consumes_the_window_without_publishing_partial_lines()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> oldLines = ["must not survive"];
        Assert.Same(oldLines, cache.Read(1_000, () => oldLines));

        var partial = new List<string> { "partial" };
        Assert.Throws<InvalidOperationException>(() => cache.Read(1_500, () =>
        {
            partial.Add("mutated before throw");
            throw new InvalidOperationException("contained by caller");
        }));

        int retries = 0;
        var cachedFailure = cache.Read(1_999, () =>
        {
            retries++;
            return ["unexpected retry"];
        });

        Assert.Same(oldLines, cachedFailure);
        Assert.DoesNotContain("partial", cachedFailure);
        Assert.Equal(0, retries);

        var recovered = cache.Read(2_000, () =>
        {
            retries++;
            return ["recovered"];
        });
        Assert.Equal(["recovered"], recovered);
        Assert.Equal(1, retries);
    }

    [Fact]
    public async Task Reset_completes_while_refresh_callback_is_blocked()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        var refreshEntered = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        using var releaseRefresh = new ManualResetEventSlim();
        var refreshTask = Task.Run(() => cache.Read(0, () =>
        {
            refreshEntered.TrySetResult(true);
            if (!releaseRefresh.Wait(CoordinationTimeout))
                throw new TimeoutException("test did not release status refresh");
            return ["pre-reset"];
        }));

        await refreshEntered.Task.WaitAsync(CoordinationTimeout);
        var resetTask = Task.Run(cache.Reset);
        try
        {
            await resetTask.WaitAsync(CoordinationTimeout);
        }
        finally
        {
            releaseRefresh.Set();
        }

        await refreshTask.WaitAsync(CoordinationTimeout);
    }

    [Fact]
    public async Task In_flight_pre_reset_result_cannot_republish_into_the_new_session()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> oldLines = ["old complete snapshot"];
        IReadOnlyList<string> staleLines = ["pre-reset in flight"];
        IReadOnlyList<string> freshLines = ["new session"];
        Assert.Same(oldLines, cache.Read(0, () => oldLines));

        var refreshEntered = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        using var releaseRefresh = new ManualResetEventSlim();
        var refreshTask = Task.Run(() => cache.Read(500, () =>
        {
            refreshEntered.TrySetResult(true);
            if (!releaseRefresh.Wait(CoordinationTimeout))
                throw new TimeoutException("test did not release stale status refresh");
            return staleLines;
        }));

        await refreshEntered.Task.WaitAsync(CoordinationTimeout);
        var resetTask = Task.Run(cache.Reset);
        try
        {
            await resetTask.WaitAsync(CoordinationTimeout);
        }
        finally
        {
            releaseRefresh.Set();
        }

        var staleResult = await refreshTask.WaitAsync(CoordinationTimeout);
        Assert.Empty(staleResult);
        Assert.NotSame(staleLines, staleResult);

        int freshCalls = 0;
        var refreshed = cache.Read(500, () =>
        {
            freshCalls++;
            return freshLines;
        });
        Assert.Same(freshLines, refreshed);
        Assert.Equal(1, freshCalls);
    }

    [Fact]
    public async Task Reentrant_reset_inside_callback_does_not_deadlock_or_publish()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> staleLines = ["pre-reset"];
        var readTask = Task.Run(() => cache.Read(1_000, () =>
        {
            cache.Reset();
            return staleLines;
        }));

        var readResult = await readTask.WaitAsync(CoordinationTimeout);
        Assert.Empty(readResult);
        Assert.NotSame(staleLines, readResult);

        int refreshes = 0;
        IReadOnlyList<string> freshLines = ["post-reset"];
        var refreshed = cache.Read(1_000, () =>
        {
            refreshes++;
            return freshLines;
        });
        Assert.Same(freshLines, refreshed);
        Assert.Equal(1, refreshes);
    }

    [Fact]
    public async Task Later_admission_wins_when_an_older_refresh_finishes_last()
    {
        var cache = new StatusTelemetryCache(RefreshMilliseconds);
        IReadOnlyList<string> staleLines = ["stale"];
        IReadOnlyList<string> latestLines = ["latest"];
        var staleEntered = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        using var releaseStale = new ManualResetEventSlim();
        var staleTask = Task.Run(() => cache.Read(1_000, () =>
        {
            staleEntered.TrySetResult(true);
            if (!releaseStale.Wait(CoordinationTimeout))
                throw new TimeoutException("test did not release superseded refresh");
            return staleLines;
        }));

        await staleEntered.Task.WaitAsync(CoordinationTimeout);
        // A regressed wall clock is immediately due and creates a newer admission.
        Assert.Same(latestLines, cache.Read(999, () => latestLines));
        releaseStale.Set();

        var staleResult = await staleTask.WaitAsync(CoordinationTimeout);
        Assert.Same(latestLines, staleResult);
        Assert.NotSame(staleLines, staleResult);
    }
}

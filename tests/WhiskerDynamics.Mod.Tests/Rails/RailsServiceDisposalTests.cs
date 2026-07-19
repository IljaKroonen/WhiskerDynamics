using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Rails;

public sealed class RailsServiceDisposalTests
{
    [Fact]
    public void Failure_after_early_health_check_cannot_cross_bind_publication()
    {
        var config = new ModConfig { RailsAheadDays = 1 };
        var constants = new GameConstants(
            6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        using var rails = TestRailsService.FromFixture(config, constants);
        using var replayed = new ManualResetEventSlim();
        int callbacks = 0;
        rails.SetAuthorityFaultHandler((service, failure) =>
        {
            Assert.Same(rails, service);
            Assert.Equal("bind-race failure", failure.Message);
            Interlocked.Increment(ref callbacks);
            replayed.Set();
        });

        // This was the old final pre-publication check.
        rails.ThrowIfAuthorityFaulted();
        rails.ReportAuthorityFailureForTest(
            new InvalidOperationException("bind-race failure"));
        Assert.True(replayed.Wait(TimeSpan.FromSeconds(5)));

        bool published = false;
        var error = Assert.Throws<InvalidOperationException>(
            () => rails.PublishIfAuthorityHealthy(() => published = true));
        Assert.Equal("authoritative rails service faulted", error.Message);
        Assert.False(published);
        Assert.Equal(1, Volatile.Read(ref callbacks));
    }

    [Fact]
    public void Worker_failure_is_latched_and_published_to_its_owner()
    {
        var config = new ModConfig { RailsAheadDays = 1 };
        var constants = new GameConstants(
            6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        using var rails = TestRailsService.FromFixture(
            config, constants, _ => throw new InvalidOperationException("injected worker fault"));
        using var reported = new ManualResetEventSlim();
        Exception? observed = null;
        RailsService? reportedService = null;

        rails.SetAuthorityFaultHandler((service, failure) =>
        {
            reportedService = service;
            observed = failure;
            reported.Set();
        });

        Assert.True(reported.Wait(TimeSpan.FromSeconds(5)));
        Assert.Same(rails, reportedService);
        Assert.Contains("injected worker fault", observed!.Message);
        Assert.Throws<InvalidOperationException>(rails.ThrowIfAuthorityFaulted);
    }

    [Fact]
    public async Task Dispose_during_sampling_cancels_and_joins_before_returning()
    {
        string dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-rails-dispose-tests-" + Guid.NewGuid().ToString("N"));
        var xmlDir = Path.Combine(dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));

        using var samplingEntered = new ManualResetEventSlim();
        using var cancellationObserved = new ManualResetEventSlim();
        using var releaseSampling = new ManualResetEventSlim();
        RailsService? rails = null;
        try
        {
            var config = new ModConfig { RailsAheadDays = 2 };
            var constants = new GameConstants(
                6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
            rails = TestRailsService.FromFixture(config, constants, token =>
            {
                samplingEntered.Set();
                token.WaitHandle.WaitOne();
                cancellationObserved.Set();
                releaseSampling.Wait();
            });
            Assert.True(samplingEntered.Wait(5000));

            Task dispose = Task.Run(rails.Dispose);
            Assert.True(cancellationObserved.Wait(1000));
            Assert.False(dispose.IsCompleted);

            releaseSampling.Set();
            await dispose.WaitAsync(TimeSpan.FromSeconds(5));
            Assert.False(rails.WorkerAliveForTest);

            // Completed disposal is idempotent and never attempts a second join.
            rails.Dispose();
            Assert.False(rails.WorkerAliveForTest);
        }
        finally
        {
            releaseSampling.Set();
            try { rails?.Dispose(); } catch { }
            try { Directory.Delete(dir, recursive: true); } catch { }
        }
    }

    [Fact]
    public async Task Dispose_waits_across_atomic_direct_flight_publication_and_closes_admission()
    {
        string dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-rails-refresh-dispose-tests-" + Guid.NewGuid().ToString("N"));
        var xmlDir = Path.Combine(dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));

        using var refreshEntered = new ManualResetEventSlim();
        using var releaseRefresh = new ManualResetEventSlim();
        using var lifecycleBoundaryEntered = new ManualResetEventSlim();
        using var releaseLifecycleBoundary = new ManualResetEventSlim();
        using var disposeStarted = new ManualResetEventSlim();
        RailsService? rails = null;
        try
        {
            var config = new ModConfig { RailsAheadDays = 2 };
            var constants = new GameConstants(
                6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
            rails = TestRailsService.FromFixture(config, constants);
            rails.NoteSimTime(10_000);
            Assert.True(SpinWait.SpinUntil(() => rails.IsReadyAt(10_000), 5000));
            int ownerEntries = 0;
            rails.ThirdBodyRefreshFlightAcquiredForTest = (parent, time) =>
            {
                Assert.Equal("Mercury", parent);
                Assert.Equal(10_000, time);
                Interlocked.Increment(ref ownerEntries);
                refreshEntered.Set();
                releaseRefresh.Wait();
            };
            rails.ThirdBodyRefreshLifecycleBoundaryForTest = () =>
            {
                lifecycleBoundaryEntered.Set();
                releaseLifecycleBoundary.Wait();
            };
            long buildsBefore = rails.ThirdBodySnapshotBuildCount;
            Task direct = Task.Run(() => rails.ThirdBodyDelta(
                "Mercury", new WhiskerDynamics.Core.Vector3d(1e7, 2e6, 0), 10_000));
            Assert.True(lifecycleBoundaryEntered.Wait(5000));
            Assert.Equal(1, rails.ThirdBodyRefreshFlightCount);

            Task dispose = Task.Run(() =>
            {
                disposeStarted.Set();
                rails.Dispose();
            });
            Assert.True(disposeStarted.Wait(5000));
            Assert.False(dispose.IsCompleted);
            Assert.False(rails.StoppingForTest);

            releaseLifecycleBoundary.Set();
            Assert.True(refreshEntered.Wait(5000));
            Assert.True(SpinWait.SpinUntil(() => rails.StoppingForTest, 1000));
            Assert.False(dispose.IsCompleted);
            Assert.Equal(1, Volatile.Read(ref ownerEntries));

            releaseRefresh.Set();
            Exception? directFailure = await Record.ExceptionAsync(
                async () => await direct.WaitAsync(TimeSpan.FromSeconds(5)));
            Assert.IsType<OperationCanceledException>(directFailure);
            await dispose.WaitAsync(TimeSpan.FromSeconds(5));
            Assert.False(rails.WorkerAliveForTest);
            Assert.Equal(0, rails.ThirdBodyRefreshPendingCount);
            Assert.Equal(0, rails.ThirdBodyRefreshFlightCount);
            Assert.Equal(buildsBefore, rails.ThirdBodySnapshotBuildCount);

            rails.RequestThirdBodyRefresh("Mercury", 10_000);
            Assert.Equal(0, rails.ThirdBodyRefreshPendingCount);
            Assert.False(rails.TryVesselPerturbation(
                "Mercury", new WhiskerDynamics.Core.Vector3d(1e7, 2e6, 0), 10_000, out _));
            Assert.Throws<OperationCanceledException>(() => rails.ThirdBodyDelta(
                "Mercury", new WhiskerDynamics.Core.Vector3d(1e7, 2e6, 0), 10_000));
            Assert.Equal(1, Volatile.Read(ref ownerEntries));
            Assert.Equal(0, rails.ThirdBodyRefreshFlightCount);
            Assert.Equal(buildsBefore, rails.ThirdBodySnapshotBuildCount);
        }
        finally
        {
            releaseLifecycleBoundary.Set();
            releaseRefresh.Set();
            try { rails?.Dispose(); } catch { }
            try { Directory.Delete(dir, recursive: true); } catch { }
        }
    }
}

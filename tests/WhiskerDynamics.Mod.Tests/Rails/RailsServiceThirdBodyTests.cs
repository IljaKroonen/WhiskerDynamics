using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Patches;
using WhiskerDynamics.Mod.Patching;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Rails;

/// <summary>Tests cached third-body acceleration against the shared gravity model,
/// including snapshot reuse and refresh behavior.</summary>
public sealed class RailsServiceThirdBodyTests : IDisposable
{
    private readonly string _dir;
    private readonly ModConfig _config;
    private readonly RailsService _rails;

    public RailsServiceThirdBodyTests()
    {
        _dir = Path.Combine(Path.GetTempPath(), "whisker-dynamics-rails-tests-" + Guid.NewGuid().ToString("N"));
        var xmlDir = Path.Combine(_dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));
        _config = new ModConfig { RailsAheadDays = 2 };
        var constants = new GameConstants(6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        _rails = TestRailsService.FromFixture(_config, constants);
        _rails.NoteSimTime(10_000);
        Assert.True(SpinWait.SpinUntil(() => _rails.IsReadyAt(10_000), 5000));
    }

    [Fact]
    public async Task Cold_live_probe_waits_for_authoritative_refresh_instead_of_omitting_force()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var ownerEntered = new ManualResetEventSlim();
        using var releaseOwner = new ManualResetEventSlim();
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (_, _) =>
        {
            ownerEntered.Set();
            releaseOwner.Wait();
        };

        Task<Vector3d> resolve = Task.Run(() =>
            LiveGravityPatch.ResolvePerturbation(
                _rails, "Mercury", relative, time));
        try
        {
            Assert.True(ownerEntered.Wait(TimeSpan.FromSeconds(5)));
            Assert.False(resolve.IsCompleted);
            releaseOwner.Set();
            AssertFinite(await resolve.WaitAsync(TimeSpan.FromSeconds(5)));
        }
        finally
        {
            releaseOwner.Set();
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Concurrent_synchronous_misses_build_once_per_parent_time_window()
    {
        const int callers = 16;
        const double firstTime = 10_000;
        const double secondTime = firstTime + 1.001;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var ownerEntered = new ManualResetEventSlim();
        using var releaseOwner = new ManualResetEventSlim();
        using var firstAcquired = new CountdownEvent(callers);
        using var secondAcquired = new CountdownEvent(callers);
        int acquiredEntries = 0;
        _rails.ThirdBodyRefreshFlightAcquiredForTest = (_, _) =>
        {
            if (Interlocked.Increment(ref acquiredEntries) <= callers)
                firstAcquired.Signal();
            else
                secondAcquired.Signal();
        };
        int ownerEntries = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, time) =>
        {
            Assert.Equal("Mercury", parent);
            Assert.True(time == firstTime || time == secondTime);
            Interlocked.Increment(ref ownerEntries);
            ownerEntered.Set();
            releaseOwner.Wait();
        };
        long buildsBefore = _rails.ThirdBodySnapshotBuildCount;
        long dynamicsBuildsBefore = _rails.ThirdBodyDynamicsBuildCount;

        try
        {
            using (var first = DeltaBurst.Start(
                _rails, callers, "Mercury", relative, firstTime))
            {
                Assert.True(ownerEntered.Wait(TimeSpan.FromSeconds(5)));
                first.AssertAllAcquired(firstAcquired);
                Assert.Equal(1, Volatile.Read(ref ownerEntries));
                Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

                releaseOwner.Set();
                first.JoinAndAssertSuccess();
            }
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));
            Assert.Equal(buildsBefore + 1, _rails.ThirdBodySnapshotBuildCount);

            ownerEntered.Reset();
            releaseOwner.Reset();
            using (var second = DeltaBurst.Start(
                _rails, callers, "Mercury", relative, secondTime))
            {
                Assert.True(ownerEntered.Wait(TimeSpan.FromSeconds(5)));
                second.AssertAllAcquired(secondAcquired);
                Assert.Equal(2, Volatile.Read(ref ownerEntries));
                Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

                releaseOwner.Set();
                second.JoinAndAssertSuccess();
            }
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));
            Assert.Equal(buildsBefore + 2, _rails.ThirdBodySnapshotBuildCount);
            Assert.Equal(dynamicsBuildsBefore + 2, _rails.ThirdBodyDynamicsBuildCount);
        }
        finally
        {
            releaseOwner.Set();
            _rails.ThirdBodyRefreshFlightAcquiredForTest = null;
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Later_window_waiters_revalidate_and_share_one_followup_flight()
    {
        const double firstTime = 10_000;
        const double laterTime = firstTime + 1.001;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var firstOwnerEntered = new ManualResetEventSlim();
        using var releaseFirstOwner = new ManualResetEventSlim();
        using var laterOwnerEntered = new ManualResetEventSlim();
        using var releaseLaterOwner = new ManualResetEventSlim();
        using var firstAcquired = new CountdownEvent(1);
        using var laterInitialAcquired = new CountdownEvent(16);
        using var laterFollowupAcquired = new CountdownEvent(16);
        int laterAcquisitions = 0;
        _rails.ThirdBodyRefreshFlightAcquiredForTest = (_, time) =>
        {
            if (time == firstTime)
                firstAcquired.Signal();
            else if (Interlocked.Increment(ref laterAcquisitions) <= 16)
                laterInitialAcquired.Signal();
            else
                laterFollowupAcquired.Signal();
        };
        int firstOwners = 0;
        int laterOwners = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, time) =>
        {
            Assert.Equal("Mercury", parent);
            if (time == firstTime)
            {
                Interlocked.Increment(ref firstOwners);
                firstOwnerEntered.Set();
                releaseFirstOwner.Wait();
            }
            else if (time == laterTime)
            {
                Interlocked.Increment(ref laterOwners);
                laterOwnerEntered.Set();
                releaseLaterOwner.Wait();
            }
            else
            {
                throw new InvalidOperationException($"unexpected refresh time {time}");
            }
        };
        long buildsBefore = _rails.ThirdBodySnapshotBuildCount;

        try
        {
            using var first = DeltaBurst.Start(
                _rails, 1, "Mercury", relative, firstTime);
            Assert.True(firstOwnerEntered.Wait(TimeSpan.FromSeconds(5)));
            first.AssertAllAcquired(firstAcquired);

            using var later = DeltaBurst.Start(
                _rails, 16, "Mercury", relative, laterTime);
            later.AssertAllAcquired(laterInitialAcquired);
            Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);
            Assert.Equal(1, Volatile.Read(ref firstOwners));
            Assert.Equal(0, Volatile.Read(ref laterOwners));

            releaseFirstOwner.Set();
            Assert.True(laterOwnerEntered.Wait(TimeSpan.FromSeconds(5)));
            later.AssertAllAcquired(laterFollowupAcquired);
            Assert.Equal(1, Volatile.Read(ref firstOwners));
            Assert.Equal(1, Volatile.Read(ref laterOwners));
            Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

            first.JoinAndAssertSuccess();
            releaseLaterOwner.Set();
            later.JoinAndAssertSuccess();
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));
            Assert.Equal(buildsBefore + 2, _rails.ThirdBodySnapshotBuildCount);
        }
        finally
        {
            releaseFirstOwner.Set();
            releaseLaterOwner.Set();
            _rails.ThirdBodyRefreshFlightAcquiredForTest = null;
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Different_parents_can_own_refresh_flights_concurrently()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var bothOwnersEntered = new CountdownEvent(2);
        using var releaseOwners = new ManualResetEventSlim();
        int mercuryOwners = 0;
        int moonOwners = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, actualTime) =>
        {
            Assert.Equal(time, actualTime);
            if (parent == "Mercury") Interlocked.Increment(ref mercuryOwners);
            else if (parent == "TestMoon") Interlocked.Increment(ref moonOwners);
            else throw new InvalidOperationException($"unexpected parent {parent}");
            bothOwnersEntered.Signal();
            releaseOwners.Wait();
        };
        long buildsBefore = _rails.ThirdBodySnapshotBuildCount;
        long dynamicsBuildsBefore = _rails.ThirdBodyDynamicsBuildCount;
        long pairEvaluationsBefore = _rails.ThirdBodyDynamicsPairEvaluationCount;

        try
        {
            using var mercury = DeltaBurst.Start(
                _rails, 1, "Mercury", relative, time);
            using var moon = DeltaBurst.Start(
                _rails, 1, "TestMoon", relative, time);

            Assert.True(bothOwnersEntered.Wait(TimeSpan.FromSeconds(5)));
            Assert.Equal(2, _rails.ThirdBodyRefreshFlightCount);
            Assert.Equal(1, Volatile.Read(ref mercuryOwners));
            Assert.Equal(1, Volatile.Read(ref moonOwners));

            releaseOwners.Set();
            mercury.JoinAndAssertSuccess();
            moon.JoinAndAssertSuccess();
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));
            Assert.Equal(buildsBefore + 2, _rails.ThirdBodySnapshotBuildCount);
            Assert.Equal(dynamicsBuildsBefore + 1, _rails.ThirdBodyDynamicsBuildCount);
            Assert.Equal((long)_rails.ModeledIds.Count * _rails.VesselGravity.Sources.Count,
                _rails.ThirdBodyDynamicsPairEvaluationCount - pairEvaluationsBefore);
        }
        finally
        {
            releaseOwners.Set();
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Different_parent_times_retain_their_own_dynamics_generations()
    {
        const double mercuryTime = 10_000;
        const double moonTime = mercuryTime + 1.001;
        var relative = new Vector3d(1e7, 2e6, 0);
        var mercuryBody = _rails.VesselGravity.Sources.Single(
            body => body.Id == "Mercury");
        var moonBody = _rails.VesselGravity.Sources.Single(
            body => body.Id == "TestMoon");
        using var bothOwnersEntered = new CountdownEvent(2);
        using var releaseOwners = new ManualResetEventSlim();
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, time) =>
        {
            Assert.True(
                (parent == "Mercury" && time == mercuryTime) ||
                (parent == "TestMoon" && time == moonTime));
            bothOwnersEntered.Signal();
            releaseOwners.Wait();
        };
        long snapshotsBefore = _rails.ThirdBodySnapshotBuildCount;
        long dynamicsBefore = _rails.ThirdBodyDynamicsBuildCount;
        long pairEvaluationsBefore = _rails.ThirdBodyDynamicsPairEvaluationCount;

        try
        {
            using var mercury = DeltaBurst.Start(
                _rails, 1, "Mercury", relative, mercuryTime);
            using var moon = DeltaBurst.Start(
                _rails, 1, "TestMoon", relative, moonTime);

            Assert.True(bothOwnersEntered.Wait(TimeSpan.FromSeconds(5)));
            Assert.Equal(2, _rails.ThirdBodyRefreshFlightCount);
            releaseOwners.Set();
            mercury.JoinAndAssertSuccess();
            moon.JoinAndAssertSuccess();
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));

            Assert.Equal(snapshotsBefore + 2, _rails.ThirdBodySnapshotBuildCount);
            Assert.Equal(dynamicsBefore + 2, _rails.ThirdBodyDynamicsBuildCount);
            Assert.Equal(2L * _rails.ModeledIds.Count * _rails.VesselGravity.Sources.Count,
                _rails.ThirdBodyDynamicsPairEvaluationCount - pairEvaluationsBefore);

            _rails.VesselGravity.ThirdBodyDeltaAt(
                mercuryBody, relative, mercuryTime);
            _rails.VesselGravity.ThirdBodyDeltaAt(
                moonBody, relative, moonTime);

            Assert.Equal(snapshotsBefore + 2, _rails.ThirdBodySnapshotBuildCount);
            Assert.Equal(dynamicsBefore + 2, _rails.ThirdBodyDynamicsBuildCount);
        }
        finally
        {
            releaseOwners.Set();
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Concurrent_owner_failure_is_shared_and_a_later_call_retries_once()
    {
        const int callers = 16;
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var ownerEntered = new ManualResetEventSlim();
        using var releaseOwner = new ManualResetEventSlim();
        using var acquired = new CountdownEvent(callers);
        int acquisitionEntries = 0;
        _rails.ThirdBodyRefreshFlightAcquiredForTest = (_, _) =>
        {
            if (Interlocked.Increment(ref acquisitionEntries) <= callers)
                acquired.Signal();
        };
        int failedOwnerEntries = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, actualTime) =>
        {
            Assert.Equal("Mercury", parent);
            Assert.Equal(time, actualTime);
            Interlocked.Increment(ref failedOwnerEntries);
            ownerEntered.Set();
            releaseOwner.Wait();
            throw new InvalidOperationException("injected shared refresh failure");
        };
        long buildsBefore = _rails.ThirdBodySnapshotBuildCount;

        try
        {
            using (var burst = DeltaBurst.Start(
                _rails, callers, "Mercury", relative, time))
            {
                Assert.True(ownerEntered.Wait(TimeSpan.FromSeconds(5)));
                burst.AssertAllAcquired(acquired);
                Assert.Equal(1, Volatile.Read(ref failedOwnerEntries));
                Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

                releaseOwner.Set();
                burst.JoinAndAssertFailure<InvalidOperationException>(
                    "injected shared refresh failure");
            }
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));
            Assert.Equal(1, Volatile.Read(ref failedOwnerEntries));
            Assert.Equal(buildsBefore, _rails.ThirdBodySnapshotBuildCount);

            int retryOwners = 0;
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, actualTime) =>
            {
                Assert.Equal("Mercury", parent);
                Assert.Equal(time, actualTime);
                Interlocked.Increment(ref retryOwners);
            };
            Vector3d retried = _rails.ThirdBodyDelta("Mercury", relative, time);

            AssertFinite(retried);
            Assert.Equal(1, Volatile.Read(ref retryOwners));
            Assert.Equal(buildsBefore + 1, _rails.ThirdBodySnapshotBuildCount);
            Assert.Equal(0, _rails.ThirdBodyRefreshFlightCount);
        }
        finally
        {
            releaseOwner.Set();
            _rails.ThirdBodyRefreshFlightAcquiredForTest = null;
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Queued_and_direct_refreshes_share_one_flight()
    {
        const int directCallers = 8;
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var ownerEntered = new ManualResetEventSlim();
        using var releaseOwner = new ManualResetEventSlim();
        using var acquired = new CountdownEvent(directCallers + 1);
        _rails.ThirdBodyRefreshFlightAcquiredForTest = (_, _) => acquired.Signal();
        int ownerEntries = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, actualTime) =>
        {
            Assert.Equal("Mercury", parent);
            Assert.Equal(time, actualTime);
            Interlocked.Increment(ref ownerEntries);
            ownerEntered.Set();
            releaseOwner.Wait();
        };
        long buildsBefore = _rails.ThirdBodySnapshotBuildCount;

        try
        {
            _rails.RequestThirdBodyRefresh("Mercury", time);
            Assert.True(ownerEntered.Wait(TimeSpan.FromSeconds(5)));
            using var direct = DeltaBurst.Start(
                _rails, directCallers, "Mercury", relative, time);
            direct.AssertAllAcquired(acquired);

            Assert.False(_rails.TryVesselPerturbation(
                "Mercury", relative, time, out _));
            _rails.RequestThirdBodyRefresh("Mercury", time);
            Assert.Equal(1, Volatile.Read(ref ownerEntries));
            Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

            releaseOwner.Set();
            direct.JoinAndAssertSuccess();
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0
                    && _rails.ThirdBodyRefreshPendingCount == 0, 5000));
            Assert.Equal(1, Volatile.Read(ref ownerEntries));
            Assert.Equal(buildsBefore + 1, _rails.ThirdBodySnapshotBuildCount);
        }
        finally
        {
            releaseOwner.Set();
            _rails.ThirdBodyRefreshFlightAcquiredForTest = null;
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public async Task Gate_owner_uses_exact_evaluation_instead_of_waiting_on_refresh_flight()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        using var ownerEntered = new ManualResetEventSlim();
        using var releaseOwner = new ManualResetEventSlim();
        int ownerEntries = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (parent, actualTime) =>
        {
            Assert.Equal("Mercury", parent);
            Assert.Equal(time, actualTime);
            Interlocked.Increment(ref ownerEntries);
            ownerEntered.Set();
            releaseOwner.Wait();
        };

        Task<Vector3d> owner = Task.Run(
            () => _rails.ThirdBodyDelta("Mercury", relative, time));
        try
        {
            Assert.True(ownerEntered.Wait(TimeSpan.FromSeconds(5)));
            Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

            Task<(Vector3d Served, Vector3d Exact)> gateOwner = Task.Run(() =>
            {
                lock (_rails.Gate)
                {
                    Vector3d served = _rails.ThirdBodyDelta("Mercury", relative, time);
                    var mercury = _rails.VesselGravity.Sources.Single(b => b.Id == "Mercury");
                    Vector3d exact = _rails.VesselGravity.ThirdBodyDeltaAt(
                        mercury, relative, time);
                    return (served, exact);
                }
            });

            var evaluation = await gateOwner.WaitAsync(TimeSpan.FromSeconds(5));
            Assert.Equal(evaluation.Exact, evaluation.Served);
            Assert.Equal(1, Volatile.Read(ref ownerEntries));
            Assert.Equal(1, _rails.ThirdBodyRefreshFlightCount);

            releaseOwner.Set();
            AssertFinite(await owner.WaitAsync(TimeSpan.FromSeconds(5)));
            Assert.True(SpinWait.SpinUntil(
                () => _rails.ThirdBodyRefreshFlightCount == 0, 5000));
        }
        finally
        {
            releaseOwner.Set();
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Gate_owner_try_path_uses_exact_evaluation_without_entering_snapshot_refresh()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        var mercury = _rails.VesselGravity.Sources.Single(body => body.Id == "Mercury");
        long snapshotsBefore = _rails.ThirdBodySnapshotBuildCount;
        long dynamicsBefore = _rails.ThirdBodyDynamicsBuildCount;

        Vector3d served;
        Vector3d exact;
        lock (_rails.Gate)
        {
            Assert.True(_rails.TryVesselPerturbation(
                mercury.Id, relative, time, out served));
            exact = _rails.VesselGravity.ThirdBodyDeltaAt(
                mercury, relative, time);
        }

        Assert.Equal(exact, served);
        Assert.Equal(snapshotsBefore, _rails.ThirdBodySnapshotBuildCount);
        Assert.Equal(dynamicsBefore, _rails.ThirdBodyDynamicsBuildCount);
        Assert.Equal(0, _rails.ThirdBodyRefreshFlightCount);
    }

    [Fact]
    public void Gate_owner_rejects_nonfinite_and_out_of_horizon_misses_without_growth()
    {
        var relative = new Vector3d(1e7, 2e6, 0);
        int ownerEntries = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (_, _) =>
            Interlocked.Increment(ref ownerEntries);
        long buildsBefore = _rails.ThirdBodySnapshotBuildCount;

        try
        {
            lock (_rails.Gate)
            {
                double horizon = _rails.Horizon;
                double[] rejectedTimes =
                [
                    double.NaN,
                    double.NegativeInfinity,
                    double.PositiveInfinity,
                    -1.0,
                    horizon + 1.0,
                ];
                foreach (double rejectedTime in rejectedTimes)
                {
                    Assert.Throws<InvalidOperationException>(() =>
                        _rails.ThirdBodyDelta("Mercury", relative, rejectedTime));
                    Assert.Equal(horizon, _rails.Horizon);
                }
            }

            Assert.Equal(0, Volatile.Read(ref ownerEntries));
            Assert.Equal(0, _rails.ThirdBodyRefreshFlightCount);
            Assert.Equal(buildsBefore, _rails.ThirdBodySnapshotBuildCount);
        }
        finally
        {
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Gate_owner_exact_evaluation_matches_snapshot_with_nonparent_extended_gravity()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        var sol = _rails.VesselGravity.Sources.Single(body => body.Id == "Sol");
        var field = Geopotential.FromJ2(
            1e11, new Vector3d(0, 0, 1), j2: 0.1);
        var property = typeof(CelestialBody).GetProperty(nameof(CelestialBody.Geopotential));
        Assert.NotNull(property);
        property!.SetValue(sol, field);
        Assert.Same(field, sol.Geopotential);

        Vector3d exactEvaluation;
        Vector3d pointMassOnly;
        Vector3d expectedExtended;
        lock (_rails.Gate)
        {
            pointMassOnly = _rails.VesselGravity.ThirdBodyDeltaAt(
                _rails.VesselGravity.Sources.Single(body => body.Id == "Mercury"),
                relative, time);
            Vector3d parentToSol = _rails.GetAbsolute("Sol", time).Position
                - _rails.GetAbsolute("Mercury", time).Position;
            expectedExtended = GravityModel.ExtendedBodyDirectTerm(
                field, sol.Mu, parentToSol, relative, time);
            exactEvaluation = _rails.ThirdBodyDelta("Mercury", relative, time);
        }

        AssertFinite(exactEvaluation);
        Assert.True(expectedExtended.Length() > 1e-8,
            "synthetic non-parent field must make omission observable");
        Vector3d expectedExact = pointMassOnly + expectedExtended;
        double exactTolerance = Math.Max(1e-12, expectedExact.Length() * 1e-12);
        Assert.True((exactEvaluation - expectedExact).Length() <= exactTolerance,
            $"exact evaluation omitted or changed the source field: expected {expectedExact}, got {exactEvaluation}");

        Vector3d snapshot = _rails.ThirdBodyDelta("Mercury", relative, time);
        double parityTolerance = Math.Max(1e-12, snapshot.Length() * 1e-12);
        Assert.True((exactEvaluation - snapshot).Length() <= parityTolerance,
            $"Gate evaluation {exactEvaluation} disagrees with snapshot evaluation {snapshot}");
    }

    [Fact]
    public void Fresh_snapshot_hits_create_no_flight_and_invoke_no_owner_hook()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        Vector3d warmed = _rails.ThirdBodyDelta("Mercury", relative, time);
        long buildsAfterWarm = _rails.ThirdBodySnapshotBuildCount;
        int ownerEntries = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (_, _) =>
            Interlocked.Increment(ref ownerEntries);

        try
        {
            using var burst = DeltaBurst.Start(
                _rails, 16, "Mercury", relative, time + 0.5);
            burst.JoinAndAssertSuccess();

            AssertFinite(warmed);
            Assert.Equal(0, Volatile.Read(ref ownerEntries));
            Assert.Equal(0, _rails.ThirdBodyRefreshFlightCount);
            Assert.Equal(buildsAfterWarm, _rails.ThirdBodySnapshotBuildCount);
        }
        finally
        {
            _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
        }
    }

    [Fact]
    public void Prewarmed_parent_has_no_live_force_gap()
    {
        const double time = 10_000;
        var relative = new Vector3d(1e7, 2e6, 0);
        _rails.RequestThirdBodyRefresh("Mercury", time);
        Assert.True(SpinWait.SpinUntil(
            () => _rails.HasThirdBodySnapshot("Mercury", time), 5000));

        Assert.True(_rails.TryVesselPerturbation("Mercury", relative, time, out var warmed));
        var mercury = _rails.VesselGravity.Sources.Single(b => b.Id == "Mercury");
        Vector3d direct;
        lock (_rails.Gate)
            direct = _rails.VesselGravity.ThirdBodyDeltaAt(mercury, relative, time);
        Assert.Equal(direct, warmed);
    }

    [Fact]
    public void Queued_refresh_failure_faults_authority_and_releases_key()
    {
        const double time = 10_000;
        int attempts = 0;
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = (_, _) =>
        {
            if (Interlocked.Increment(ref attempts) == 1)
                throw new InvalidOperationException("injected refresh failure");
        };

        _rails.RequestThirdBodyRefresh("Mercury", time);
        Assert.True(SpinWait.SpinUntil(
            () => attempts == 1 && _rails.ThirdBodyRefreshPendingCount == 0, 5000));
        Assert.False(_rails.HasThirdBodySnapshot("Mercury", time));

        var fault = Assert.Throws<InvalidOperationException>(
            () => _rails.RequestThirdBodyRefresh("Mercury", time));
        Assert.Equal("authoritative rails service faulted", fault.Message);
        Assert.Equal("injected refresh failure", fault.InnerException?.Message);
        Assert.Equal(1, attempts);
        _rails.ThirdBodyRefreshOwnerBeforeBuildForTest = null;
    }

    private static void AssertFinite(Vector3d value)
    {
        Assert.True(double.IsFinite(value.X));
        Assert.True(double.IsFinite(value.Y));
        Assert.True(double.IsFinite(value.Z));
    }

    private sealed class DeltaBurst : IDisposable
    {
        private readonly CountdownEvent _ready;
        private readonly ManualResetEventSlim _launch = new();
        private readonly Thread[] _threads;
        private readonly Vector3d[] _results;
        private readonly Exception?[] _failures;

        private DeltaBurst(RailsService rails, int callers, string parent,
            Vector3d relative, double time)
        {
            _ready = new CountdownEvent(callers);
            _threads = new Thread[callers];
            _results = new Vector3d[callers];
            _failures = new Exception?[callers];
            for (int i = 0; i < callers; i++)
            {
                int caller = i;
                _threads[i] = new Thread(() =>
                {
                    try
                    {
                        _ready.Signal();
                        if (!_launch.Wait(TimeSpan.FromSeconds(5)))
                            throw new TimeoutException("delta burst launch timed out");
                        _results[caller] = rails.ThirdBodyDelta(parent, relative, time);
                    }
                    catch (Exception e)
                    {
                        _failures[caller] = e;
                    }
                })
                {
                    IsBackground = true,
                    Name = $"third-body-delta-{parent}-{caller}",
                };
                _threads[i].Start();
            }

            if (!_ready.Wait(TimeSpan.FromSeconds(5)))
                throw new TimeoutException("delta burst callers did not become ready");
            _launch.Set();
        }

        public static DeltaBurst Start(RailsService rails, int callers, string parent,
            Vector3d relative, double time) => new(rails, callers, parent, relative, time);

        public void AssertAllAcquired(CountdownEvent acquired)
        {
            Assert.True(acquired.Wait(TimeSpan.FromSeconds(5)));
            Assert.All(_threads, thread => Assert.True(thread.IsAlive));
        }

        public void JoinAndAssertSuccess()
        {
            Join();
            Assert.All(_failures, failure => Assert.Null(failure));
            Vector3d expected = _results[0];
            AssertFinite(expected);
            Assert.All(_results, result => Assert.Equal(expected, result));
        }

        public void JoinAndAssertFailure<TException>(string message)
            where TException : Exception
        {
            Join();
            Assert.All(_failures, failure =>
            {
                var typed = Assert.IsType<TException>(failure);
                Assert.Equal(message, typed.Message);
            });
        }

        private void Join()
        {
            foreach (Thread thread in _threads)
                Assert.True(thread.Join(TimeSpan.FromSeconds(5)),
                    $"concurrent caller '{thread.Name}' did not complete");
        }

        public void Dispose()
        {
            _launch.Set();
            _launch.Dispose();
            _ready.Dispose();
        }
    }

    public void Dispose()
    {
        _rails.Dispose();
        try { Directory.Delete(_dir, recursive: true); } catch { /* best effort */ }
    }

    [Fact]
    public void CurveBodyIds_spans_all_parsed_bodies_with_a_parent_root_excluded()
    {
        // Every positive-mu fixture body, including the fast comet, is mutually coupled
        // and carries an honest arc. Root has no line.
        Assert.True(_rails.IsModeled("TestMoon"));
        Assert.True(_rails.IsModeled("TestComet"));
        Assert.True(_rails.IsBackbone("TestMoon"));
        Assert.True(_rails.IsBackbone("TestComet"));
        Assert.Contains("Mercury", _rails.CurveBodyIds);
        Assert.Contains("TestMoon", _rails.CurveBodyIds);
        Assert.Contains("TestComet", _rails.CurveBodyIds);
        Assert.Contains("TestComet", _rails.SoiChildrenOf("Sol"));
        Assert.DoesNotContain("Sol", _rails.CurveBodyIds);
    }

    [Fact]
    public void CurveBodyIds_observes_the_live_body_budget()
    {
        _config.CelestialCurveMaxBodies = 1;
        Assert.Single(_rails.CurveBodyIds);

        _config.CelestialCurveMaxBodies = 256;
        Assert.Equal(_rails.CurveEligibleCount, _rails.CurveBodyIds.Count);
    }

    [Fact]
    public void Fast_positive_mu_body_is_mutually_coupled_without_a_speed_gate()
    {
        Assert.True(_rails.IsModeled("TestComet"));
        Assert.True(_rails.IsBackbone("TestComet"));
    }

    [Fact]
    public void Modeled_comet_pulls_vessels_as_a_gravity_source()
    {
        Assert.True(_rails.IsModeled("TestComet"));
        var comet = _rails.VesselGravity.Sources.Single(b => b.Id == "TestComet");
        Assert.True(comet.Mu > 0);

        double t = 1000.0;
        var cometPosition = _rails.GetAbsolute(comet.Id, t).Position;
        var probe = cometPosition + new Vector3d(100.0, 0, 0);
        var acceleration = _rails.Acceleration(probe, t);

        var withoutComet = Vector3d.Zero;
        foreach (var body in _rails.VesselGravity.Sources.Where(b => !ReferenceEquals(b, comet)))
        {
            var offset = probe - _rails.GetAbsolute(body.Id, t).Position;
            double r2 = offset.LengthSquared();
            withoutComet -= offset * (body.Mu / (r2 * Math.Sqrt(r2)));
        }
        var expectedComet = new Vector3d(-comet.Mu / 10_000.0, 0, 0);
        var actualComet = acceleration - withoutComet;
        Assert.True((actualComet - expectedComet).Length() < 1e-6,
            $"expected isolated comet pull {expectedComet}, got {actualComet}");
    }

    [Fact]
    public void Vessel_gravity_sources_include_every_positive_modeled_body()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1e20 };
        var valid = new CelestialBody
        {
            Id = "Valid", Mu = 1e10, Parent = root,
            Orbit = new OrbitalElements(1e9, 0.1, 0, 0, 0, 0),
        };
        var restricted = new CelestialBody
        {
            Id = "Restricted", Mu = 1e6, Parent = root,
            Orbit = new OrbitalElements(1e8, 0.9, 0, 0, 0, 0),
        };
        var zeroMu = new CelestialBody
        {
            Id = "Zero", Mu = 0, Parent = root, Orbit = valid.Orbit,
        };
        var selected = RailsService.SelectVesselGravitySources(
            [root, valid, restricted, zeroMu]);

        Assert.Equal([root, valid, restricted], selected);
    }

    [Fact]
    public void Invalid_seed_catalog_is_rejected_instead_of_dropping_a_gravity_source()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1e20 };
        var invalid = new CelestialBody
        {
            Id = "Invalid",
            Mu = 1e10,
            Parent = root,
            Orbit = new OrbitalElements(1e9, double.NaN, 0, 0, 0, 0),
        };

        Assert.Throws<ArgumentException>(() => RailsService.CreateForModeledCatalog(
            new ModConfig(), [root, invalid]));
    }

    [Fact]
    public void Cached_delta_matches_the_shared_gravity_model_exactly()
    {
        var mercury = _rails.VesselGravity.Sources.Single(b => b.Id == "Mercury");
        var rel = new Vector3d(2.74e6, 0, 0);
        double t = 100.0;
        Vector3d cached, reference;
        // Pin both evaluations to one ephemeris generation. The rails worker can
        // otherwise commit thinned knots between two separately locked calls,
        // changing dense-cubic evaluation to committed-quintic by a few ulps.
        lock (_rails.Gate)
        {
            cached = _rails.ThirdBodyDelta("Mercury", rel, t);
            reference = _rails.VesselGravity.ThirdBodyDeltaAt(mercury, rel, t);
        }
        Assert.Equal(reference, cached); // same sources, same order, same kernel: bitwise
        Assert.InRange(cached.Length(), 1e-8, 1e-4); // solar tide order at low Mercury orbit
    }

    [Fact]
    public void Snapshot_is_reused_within_tolerance_and_refreshed_beyond_it()
    {
        var mercury = _rails.VesselGravity.Sources.Single(b => b.Id == "Mercury");
        var rel = new Vector3d(2.74e6, 0, 0);
        long dynamicsBuildsBefore = _rails.ThirdBodyDynamicsBuildCount;

        var d0 = _rails.ThirdBodyDelta("Mercury", rel, 1000.0);
        var dNear = _rails.ThirdBodyDelta("Mercury", rel, 1000.9);
        Assert.Equal(dynamicsBuildsBefore + 1, _rails.ThirdBodyDynamicsBuildCount);
        // Within the 1 s tolerance the cached state advances with source velocity.
        Assert.NotEqual(d0, dNear);
        Vector3d fresh;
        lock (_rails.Gate) fresh = _rails.VesselGravity.ThirdBodyDeltaAt(mercury, rel, 1000.9);
        Assert.True((fresh - dNear).Length() < 1e-12,
            $"quadratic cache {dNear} diverged from fresh {fresh}");

        // Beyond the tolerance the snapshot refreshes and re-syncs exactly.
        Vector3d dFar;
        // Monitor locks are reentrant: ThirdBodyDelta can refresh its snapshot while
        // this outer hold prevents the worker from committing a new interpolation
        // representation before the direct reference evaluation.
        lock (_rails.Gate)
        {
            dFar = _rails.ThirdBodyDelta("Mercury", rel, 1003.0);
            fresh = _rails.VesselGravity.ThirdBodyDeltaAt(mercury, rel, 1003.0);
        }
        Assert.Equal(fresh, dFar);
    }

    [Fact]
    public void Cached_geometry_tracks_a_fast_modeled_source_during_the_reuse_window()
    {
        string dir = Path.Combine(Path.GetTempPath(), "whisker-dynamics-fast-source-tests-"
            + Guid.NewGuid().ToString("N"));
        try
        {
            string xmlDir = Path.Combine(dir, "Content", "Core");
            Directory.CreateDirectory(xmlDir);
            string xml = File.ReadAllText(
                Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"))
                .Replace("""<TimeAtPeriapsis Seconds="-272712751" />""",
                    """<TimeAtPeriapsis Seconds="0" />""")
                .Replace("""<Mass Kg="1E+13" />""", """<Mass Kg="1E+25" />""");
            File.WriteAllText(Path.Combine(xmlDir, "Astronomicals.xml"), xml);
            var config = new ModConfig { RailsAheadDays = 2 };
            var constants = new GameConstants(
                6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
            var bodies = AstronomicalsParser.ParseFile(
                Path.Combine(xmlDir, "Astronomicals.xml"), constants.ToMassConstants());
            using var rails = RailsService.CreateForModeledCatalog(config, bodies);
            rails.NoteSimTime(1.0);
            Assert.True(SpinWait.SpinUntil(() => rails.IsReadyAt(1.0), 5000));
            Assert.True(rails.IsModeled("TestComet"));
            var cometSource = rails.VesselGravity.Sources.Single(b => b.Id == "TestComet");
            var comet0 = rails.GetAbsolute("TestComet", 0.0);
            var sol0 = rails.GetAbsolute("Sol", 0.0);
            Assert.True((comet0.Velocity - sol0.Velocity).Length() > 200_000,
                "fixture replacement must place the modeled comet at its fast periapsis");
            Assert.True(cometSource.Mu > 1e14,
                "fixture replacement must make the close-force error observable");

            const double later = 0.9;
            var parent = rails.GetAbsolute("Mercury", later).Position;
            var comet = rails.GetAbsolute("TestComet", later).Position;
            var rel = comet - parent + new Vector3d(100_000, 0, 0);
            long builds = rails.ThirdBodySnapshotBuildCount;
            _ = rails.ThirdBodyDelta("Mercury", rel, 0.0); // capture the t=0 snapshot
            Assert.Equal(builds + 1, rails.ThirdBodySnapshotBuildCount);
            var cached = rails.ThirdBodyDelta("Mercury", rel, later);
            Assert.Equal(builds + 1, rails.ThirdBodySnapshotBuildCount); // reused
            Vector3d fresh;
            var mercury = rails.VesselGravity.Sources.Single(b => b.Id == "Mercury");
            lock (rails.Gate)
                fresh = rails.VesselGravity.ThirdBodyDeltaAt(mercury, rel, later);

            double relativeError = (cached - fresh).Length() / fresh.Length();
            Assert.True(relativeError < 1e-3,
                $"fast-source cached field error {relativeError:E3}: cached {cached}, fresh {fresh}");

            var closeRel = comet - parent + new Vector3d(10, 0, 0);
            var closeCached = rails.ThirdBodyDelta("Mercury", closeRel, later);
            Assert.Equal(builds + 2, rails.ThirdBodySnapshotBuildCount); // remainder refresh
            lock (rails.Gate)
                fresh = rails.VesselGravity.ThirdBodyDeltaAt(mercury, closeRel, later);
            Assert.Equal(fresh, closeCached); // curvature bound forces an exact refresh
        }
        finally
        {
            try { Directory.Delete(dir, recursive: true); } catch { /* best effort */ }
        }
    }

    [Fact]
    public void Modeled_gravity_source_reaches_the_geometric_soi_transfer_seam()
    {
        double t = 1000.0;
        double soi = _rails.SphereOfInfluenceOf("TestComet");
        Assert.True(_rails.IsSoiChildCandidate("TestComet", soi));
        Assert.True(_rails.IsModeled("TestComet"));

        var sol = _rails.GetAbsolute("Sol", t).Position;
        var comet = _rails.GetAbsolute("TestComet", t).Position;
        var candidate = new SoiReparentKernel.Candidate("TestComet", comet, soi);
        Assert.Equal("TestComet", SoiReparentKernel.Decide(
            comet, sol, double.PositiveInfinity, null, [candidate]));
    }
}

public sealed class RestrictedParentAuthorityTests
{
    [Fact]
    public void Mixed_source_corrections_match_track_coupling_for_backbone_and_restricted_parents()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1.0e14 };
        var backboneParent = new CelestialBody
        {
            Id = "BackboneParent",
            Mu = 1.0e10,
            Parent = root,
            Orbit = new OrbitalElements(1.0e9, 0.01, 0, 0, 0, 0),
        };
        var restrictedA = new CelestialBody
        {
            Id = "RestrictedA",
            Mu = 2.0e8,
            Parent = root,
            Orbit = new OrbitalElements(2.0e9, 0.05, 0.1, 0.2, 0.3, 0),
        };
        var restrictedB = new CelestialBody
        {
            Id = "RestrictedB",
            Mu = 3.0e8,
            Parent = root,
            Orbit = new OrbitalElements(3.0e9, 0.1, 0.2, 0.3, 0.4, 0),
        };
        var restrictedChild = new CelestialBody
        {
            Id = "RestrictedChild",
            Mu = 1.0e6,
            Parent = restrictedA,
            Orbit = new OrbitalElements(1.0e7, 0.02, 0.05, 0.1, 0.2, 0),
        };
        using var rails = RailsService.CreateForSyntheticCatalog(
            [root, backboneParent, restrictedA, restrictedB, restrictedChild],
            [root.Id, backboneParent.Id]);

        Assert.True(rails.IsModeled(restrictedA.Id));
        Assert.False(rails.IsBackbone(restrictedA.Id));
        Assert.True(rails.IsVesselGravitySource(restrictedA.Id));

        var relative = new Vector3d(2.0e6, -3.0e5, 1.0e5);
        AssertMixedCorrection(backboneParent, expectsRestrictedAncestor: false);
        AssertMixedCorrection(restrictedA, expectsRestrictedAncestor: false);
        AssertMixedCorrection(restrictedChild, expectsRestrictedAncestor: true);

        void AssertMixedCorrection(CelestialBody parent, bool expectsRestrictedAncestor)
        {
            var parentPosition = rails.GetAbsolute(parent.Id, 0).Position;
            var expected = Vector3d.Zero;
            var incorrectlyAllTidal = Vector3d.Zero;
            var incorrectlyBackboneOnlyTidal = Vector3d.Zero;
            foreach (var source in rails.VesselGravity.Sources)
            {
                if (ReferenceEquals(source, parent)) continue;
                var parentToSource = rails.GetAbsolute(source.Id, 0).Position - parentPosition;
                bool parentFeelsSource = rails.IsBackbone(source.Id)
                    || IsAncestorOf(source, parent);
                expected += source.Mu * (parentFeelsSource
                    ? GravityModel.TidalTerm(parentToSource, relative)
                    : GravityModel.DirectPointMassTerm(parentToSource, relative));
                incorrectlyAllTidal += source.Mu
                    * GravityModel.TidalTerm(parentToSource, relative);
                incorrectlyBackboneOnlyTidal += source.Mu * (rails.IsBackbone(source.Id)
                    ? GravityModel.TidalTerm(parentToSource, relative)
                    : GravityModel.DirectPointMassTerm(parentToSource, relative));
            }

            Vector3d exact;
            lock (rails.Gate)
                exact = rails.ThirdBodyDelta(parent.Id, relative, 0);
            var cached = rails.ThirdBodyDelta(parent.Id, relative, 0);

            Assert.Equal(expected, exact);
            Assert.Equal(exact, cached);
            Assert.True((cached - incorrectlyAllTidal).Length() > 1e-12,
                "unrelated positive-mu restricted sources must remain direct");
            if (expectsRestrictedAncestor)
                Assert.True((cached - incorrectlyBackboneOnlyTidal).Length() > 1e-12,
                    "a positive-mu restricted ancestor must accelerate its child track");
        }

        static bool IsAncestorOf(CelestialBody candidate, CelestialBody body)
        {
            for (var ancestor = body.Parent; ancestor is not null; ancestor = ancestor.Parent)
                if (ReferenceEquals(ancestor, candidate))
                    return true;
            return false;
        }
    }

    [Fact]
    public void Zero_mu_parent_has_parent_relative_rails_and_vessel_perturbation()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1.0e20 };
        var restricted = new CelestialBody
        {
            Id = "RestrictedParent",
            Mu = 0.0,
            Parent = root,
            Orbit = new OrbitalElements(
                SemiMajorAxis: 1.0e8,
                Eccentricity: 0.1,
                Inclination: 0.2,
                LongitudeOfAscendingNode: 0.3,
                ArgumentOfPeriapsis: 0.4,
                TimeAtPeriapsis: 0.0),
        };
        using var rails = RailsService.CreateForModeledCatalog(
            new ModConfig { RailsAheadDays = 1 }, [root, restricted]);

        Assert.True(rails.IsModeled(restricted.Id));
        Assert.False(rails.IsBackbone(restricted.Id));
        Assert.True(rails.TryGetParentRelativeEcl(
            restricted.Id, 0.0, out var position, out var velocity));
        Assert.True(double.IsFinite(position.X + position.Y + position.Z));
        Assert.True(double.IsFinite(velocity.X + velocity.Y + velocity.Z));
        Assert.True(position.Length() > 0.0);

        var perturbation = rails.VesselPerturbation(
            restricted.Id, new Vector3d(1000.0, -2000.0, 500.0), 0.0);
        Assert.True(double.IsFinite(
            perturbation.X + perturbation.Y + perturbation.Z));
        Assert.True(perturbation.Length() > 0.0);
    }

    [Fact]
    public void Positive_mu_descendant_of_zero_mu_parent_remains_restricted_and_pulls_vessels()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1.0e20 };
        var zeroMuParent = new CelestialBody
        {
            Id = "ZeroMuParent",
            Mu = 0.0,
            Parent = root,
            Orbit = new OrbitalElements(1.0e8, 0.1, 0.2, 0.3, 0.4, 0.0),
        };
        var massiveChild = new CelestialBody
        {
            Id = "MassiveChild",
            Mu = 1.0e8,
            Parent = zeroMuParent,
            Orbit = new OrbitalElements(1.0e6, 0.01, 0.1, 0.2, 0.3, 0.0),
        };
        using var rails = RailsService.CreateForModeledCatalog(
            new ModConfig { RailsAheadDays = 1 }, [root, zeroMuParent, massiveChild]);

        Assert.True(rails.IsBackbone(root.Id));
        Assert.False(rails.IsBackbone(zeroMuParent.Id));
        Assert.False(rails.IsBackbone(massiveChild.Id));
        Assert.True(rails.IsModeled(massiveChild.Id));
        Assert.True(rails.IsVesselGravitySource(massiveChild.Id));
    }
}

public sealed class ThirdBodySnapshotScalingTests
{
    [Theory]
    [InlineData(8)]
    [InlineData(16)]
    [InlineData(32)]
    public void One_time_generation_scales_as_one_body_source_matrix(int sourceCount)
    {
        var bodies = CreateCatalog(sourceCount);
        using var rails = RailsService.CreateForSyntheticCatalog(
            bodies, bodies.Select(body => body.Id).ToArray());
        var relative = new Vector3d(1.0e6, 2.0e5, -3.0e5);

        foreach (var parent in bodies)
        {
            Vector3d exact;
            lock (rails.Gate)
                exact = rails.ThirdBodyDelta(parent.Id, relative, 0.0);
            Vector3d delta = rails.ThirdBodyDelta(parent.Id, relative, 0.0);
            Assert.Equal(exact, delta);
        }

        Assert.Equal(sourceCount, rails.ThirdBodySnapshotBuildCount);
        Assert.Equal(1, rails.ThirdBodyDynamicsBuildCount);
        Assert.Equal((long)sourceCount * sourceCount,
            rails.ThirdBodyDynamicsPairEvaluationCount);
    }

    private static IReadOnlyList<CelestialBody> CreateCatalog(int sourceCount)
    {
        const double rootMu = 1.32712440018e20;
        var root = new CelestialBody { Id = "Root", Mu = rootMu };
        var bodies = new List<CelestialBody>(sourceCount) { root };
        for (int i = 1; i < sourceCount; i++)
        {
            double semiMajorAxis = 4.0e10 + i * 7.5e8;
            bodies.Add(new CelestialBody
            {
                Id = $"Body{i:D3}",
                Mu = 5.0e8 + i * 1.0e7,
                Parent = root,
                Orbit = new OrbitalElements(
                    semiMajorAxis, 0.001 * (i % 10), 0.002 * (i % 7),
                    i * 0.13, i * 0.17, i * -1000.0),
            });
        }
        return bodies;
    }
}

/// <summary>Pins the registration step: the Seam 2 live-gravity patch must stay in
/// the gameplay patch set (applied inside the guarded try only after ALL gameplay
/// targets validate). Reflection-only on mod types — no KSA type is loaded offline.</summary>
public class Seam2RegistrationTests
{
    [Fact]
    public void LiveGravityPatch_is_registered_as_a_gameplay_patch()
    {
        Assert.Contains(typeof(LiveGravityPatch), GameplayPatchSet.PatchTypes);
    }
}

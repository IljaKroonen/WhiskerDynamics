using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Mod.Patches;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.Mod.Tests.Overlay;

[CollectionDefinition(nameof(OrbitCacheCoordinationTestCollection), DisableParallelization = true)]
public sealed class OrbitCacheCoordinationTestCollection;

[Collection(nameof(OrbitCacheCoordinationTestCollection))]
public class OrbitCacheCoordinationTests
{
    [Fact]
    public void All_finite_cache_is_exact_stock_and_clears_mod_debt()
    {
        OrbitPointCce[] stock = StockPoints(10.0);

        var observed = TrajectoryOverlay.ObserveStockCache(
            stock, previous: null, actualCacheModOwned: true);

        Assert.True(observed.CurrentCacheIsSafeStock);
        Assert.False(observed.ActualCacheModOwned);
        AssertPointsEqual(stock, Assert.IsType<OrbitPointCce[]>(observed.StockPoints));
    }

    [Fact]
    public void All_nan_cache_retains_stock_only_with_explicit_mod_debt()
    {
        OrbitPointCce[] prior = StockPoints(20.0);
        OrbitPointCce[] mod = ModPoints(200.0);

        var owned = TrajectoryOverlay.ObserveStockCache(
            mod, prior, actualCacheModOwned: true);
        var foreign = TrajectoryOverlay.ObserveStockCache(
            mod, prior, actualCacheModOwned: false);

        Assert.Same(prior, owned.StockPoints);
        Assert.True(owned.ActualCacheModOwned);
        Assert.False(owned.CurrentCacheIsSafeStock);
        Assert.Null(foreign.StockPoints);
        Assert.False(foreign.ActualCacheModOwned);
        Assert.False(foreign.CurrentCacheIsSafeStock);
    }

    [Fact]
    public void Mixed_cache_invalidates_snapshot_even_with_mod_debt()
    {
        OrbitPointCce[] stock = StockPoints(30.0);
        OrbitPointCce[] mod = ModPoints(300.0);
        OrbitPointCce[] mixed = [stock[0], mod[0]];

        var observed = TrajectoryOverlay.ObserveStockCache(
            mixed, stock.ToArray(), actualCacheModOwned: true);

        Assert.Null(observed.StockPoints);
        Assert.True(observed.ActualCacheModOwned);
        Assert.False(observed.CurrentCacheIsSafeStock);
    }

    [Fact]
    public void Empty_cache_is_safe_and_clears_snapshot_only_without_mod_debt()
    {
        OrbitPointCce[] prior = StockPoints(40.0);

        var genuine = TrajectoryOverlay.ObserveStockCache(
            ReadOnlySpan<OrbitPointCce>.Empty, prior, actualCacheModOwned: false);
        var owned = TrajectoryOverlay.ObserveStockCache(
            ReadOnlySpan<OrbitPointCce>.Empty, prior, actualCacheModOwned: true);

        Assert.Null(genuine.StockPoints);
        Assert.False(genuine.ActualCacheModOwned);
        Assert.True(genuine.CurrentCacheIsSafeStock);
        Assert.Same(prior, owned.StockPoints);
        Assert.True(owned.ActualCacheModOwned);
        Assert.False(owned.CurrentCacheIsSafeStock);
    }

    [Fact]
    public void Reset_deterministically_rejects_a_pre_reset_stage_handoff()
    {
        long stale = TrajectoryOverlay.CaptureStageCacheGenerationForTest();
        TrajectoryOverlay.ResetSessionStatics();
        bool wrote = false;

        bool accepted = TrajectoryOverlay.TryRunStageHandoffForTest(
            stale, () => wrote = true);

        Assert.False(accepted);
        Assert.False(wrote);
    }

    [Fact]
    public async Task Reset_waits_for_an_active_handoff_then_invalidates_its_generation()
    {
        long generation = TrajectoryOverlay.CaptureStageCacheGenerationForTest();
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        var handoff = Task.Run(() => TrajectoryOverlay.TryRunStageHandoffForTest(
            generation, () =>
            {
                entered.Set();
                Assert.True(release.Wait(TimeSpan.FromSeconds(5)));
            }));

        Assert.True(entered.Wait(TimeSpan.FromSeconds(5)));
        var reset = Task.Run(TrajectoryOverlay.ResetSessionStatics);
        Assert.True(SpinWait.SpinUntil(
            () => TrajectoryOverlay.StageCacheResetPendingForTest,
            TimeSpan.FromSeconds(5)));
        Assert.False(reset.IsCompleted);

        release.Set();
        Assert.True(await handoff.WaitAsync(TimeSpan.FromSeconds(5)));
        await reset.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.False(TrajectoryOverlay.TryRunStageHandoffForTest(
            generation, static () => throw new InvalidOperationException("stale write ran")));
    }

    [Fact]
    public void Fatal_shutdown_retains_last_authoritative_cache_revokes_worker_and_stops_rails()
    {
        TrajectoryOverlay.ResetSessionStatics();
        Orbit orbit = UninitializedOrbit();
        OrbitPointCce[] stock = StockPoints(73.0);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, stock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        OrbitPointCce[] authoritative = ModPoints(7300.0);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, authoritative);

        int generation = OverlayWorker.CurrentGeneration;
        using var runningStarted = new ManualResetEventSlim();
        using var releaseRunning = new ManualResetEventSlim();
        using var runningFinished = new ManualResetEventSlim();
        int published = 0;
        Assert.True(OverlayWorker.Enqueue("fatal-shutdown-ticket", generation,
            (_, publish) =>
            {
                runningStarted.Set();
                try
                {
                    Assert.True(releaseRunning.Wait(TimeSpan.FromSeconds(5)));
                    publish(() => Interlocked.Increment(ref published));
                }
                finally
                {
                    runningFinished.Set();
                }
            }));
        Assert.True(runningStarted.Wait(TimeSpan.FromSeconds(5)));

        bool railsStopped = false;
        var contained = new List<string>();
        FatalShutdownPolicy.Execute(
            () => railsStopped = true,
            (phase, error) => contained.Add($"{phase}: {error.Message}"));

        AssertPointsEqual(authoritative, orbit.CachedPoints.ToArray());
        Assert.True(railsStopped);
        Assert.Empty(contained);
        releaseRunning.Set();
        Assert.True(runningFinished.Wait(TimeSpan.FromSeconds(5)));
        Assert.Equal(0, Volatile.Read(ref published));
    }

    [Fact]
    public async Task Old_worker_poised_before_stage_cannot_capture_the_new_session()
    {
        int oldWorkerGeneration = OverlayWorker.CurrentGeneration;
        Orbit orbit = UninitializedOrbit();
        using var poised = new ManualResetEventSlim();
        using var allowStage = new ManualResetEventSlim();
        var oldWorker = Task.Run(() =>
        {
            poised.Set();
            Assert.True(allowStage.Wait(TimeSpan.FromSeconds(5)));
            return Record.Exception(() =>
            {
                // Null proves stale admission happens before any sample dereference.
                _ = TrajectoryOverlay.StageWorkerBatch(
                    null!, orbit, oldWorkerGeneration);
            });
        });

        Assert.True(poised.Wait(TimeSpan.FromSeconds(5)));
        TrajectoryOverlay.ResetSessionStatics();
        allowStage.Set();

        var failure = await oldWorker.WaitAsync(TimeSpan.FromSeconds(5));
        var stale = Assert.IsType<OperationCanceledException>(failure);
        Assert.Contains("stale session", stale.Message, StringComparison.Ordinal);
    }

    [Fact]
    public async Task Orbit_update_patch_excludes_other_threads_and_finalizer_releases()
    {
        Orbit orbit = UninitializedOrbit();
        OrbitCacheUpdatePatch.Prefix(orbit, out var first);
        Assert.NotNull(first);

        using var attempted = new ManualResetEventSlim();
        using var acquired = new ManualResetEventSlim();
        var waiter = Task.Run(() =>
        {
            attempted.Set();
            OrbitCacheUpdatePatch.Prefix(orbit, out var second);
            acquired.Set();
            OrbitCacheUpdatePatch.Finalizer(null, second);
        });

        Assert.True(attempted.Wait(TimeSpan.FromSeconds(5)));
        Assert.False(acquired.Wait(TimeSpan.FromMilliseconds(100)));

        var targetFault = new InvalidOperationException("injected UpdateCachedPoints fault");
        Assert.Same(targetFault, OrbitCacheUpdatePatch.Finalizer(targetFault, first));
        Assert.True(acquired.Wait(TimeSpan.FromSeconds(5)));
        await waiter.WaitAsync(TimeSpan.FromSeconds(5));
    }

    [Fact]
    public void Orbit_update_patch_reuses_the_gate_object_as_harmony_state()
    {
        Orbit orbit = UninitializedOrbit();

        OrbitCacheUpdatePatch.Prefix(orbit, out var first);
        OrbitCacheUpdatePatch.Finalizer(null, first);
        OrbitCacheUpdatePatch.Prefix(orbit, out var second);

        Assert.NotNull(first);
        Assert.Same(first, second);
        OrbitCacheUpdatePatch.Finalizer(null, second);
    }

    [Fact]
    public void Orbit_cache_patch_is_registered_as_gameplay_patch()
    {
        Assert.Contains(typeof(OrbitCacheUpdatePatch), GameplayPatchSet.PatchTypes);
    }

    private static Orbit UninitializedOrbit()
    {
        Type concrete = typeof(Orbit).GetNestedType(
            "Elliptical", BindingFlags.Public | BindingFlags.NonPublic)
            ?? throw new InvalidOperationException("KSA Orbit.Elliptical fixture type is missing");
        return (Orbit)RuntimeHelpers.GetUninitializedObject(concrete);
    }

    private static OrbitPointCce[] StockPoints(double seed) =>
    [
        new(new double3(seed, seed + 1, seed + 2),
            new SimTime(seed + 3), new SimTime(seed + 4), new TrueAnomaly(0.25)),
        new(new double3(-seed, -seed - 1, -seed - 2),
            new SimTime(seed + 5), new SimTime(seed + 6), new TrueAnomaly(5.5),
            inDangerZone: true),
    ];

    private static OrbitPointCce[] ModPoints(double seed) =>
    [
        new(new double3(seed, seed + 1, seed + 2),
            new SimTime(seed + 3), new SimTime(seed + 4), TrueAnomaly.NaN),
        new(new double3(-seed, -seed - 1, -seed - 2),
            new SimTime(seed + 5), new SimTime(seed + 6), TrueAnomaly.NaN),
    ];

    private static void AssertPointsEqual(
        IReadOnlyList<OrbitPointCce> expected, IReadOnlyList<OrbitPointCce> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
        {
            Assert.Equal(expected[i].PositionCce, actual[i].PositionCce);
            Assert.Equal(expected[i].TimeSincePe, actual[i].TimeSincePe);
            Assert.Equal(expected[i].RemainingTimeTo, actual[i].RemainingTimeTo);
            Assert.Equal(expected[i].TrueAnomaly, actual[i].TrueAnomaly);
            Assert.Equal(expected[i].CompassTrueAnomaly, actual[i].CompassTrueAnomaly);
            Assert.Equal(expected[i].InDangerZone, actual[i].InDangerZone);
        }
    }
}

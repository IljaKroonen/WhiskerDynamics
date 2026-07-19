using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Mod.Patches;

namespace WhiskerDynamics.Mod.Tests.Patches;

public class VesselLineFallbackTests
{
    [Theory]
    [InlineData((int)VesselLinePatch.ActualLinePhase.StageHandoff)]
    [InlineData((int)VesselLinePatch.ActualLinePhase.CameraPreparation)]
    [InlineData((int)VesselLinePatch.ActualLinePhase.BypassVisibilityPreparation)]
    [InlineData((int)VesselLinePatch.ActualLinePhase.DenseLineDraw)]
    [InlineData((int)VesselLinePatch.ActualLinePhase.TakeoverEvidenceTail)]
    public void Production_orchestration_fault_restores_exact_stock_cache_before_original_runs(
        int phaseValue)
    {
        var phase = (VesselLinePatch.ActualLinePhase)phaseValue;
        Orbit orbit = UninitializedOrbit();
        OrbitPointCce[] stock = StockPoints(10.0);
        OrbitPointCce[] staged = ModPoints(1000.0);

        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, stock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, staged);
        AssertCacheBits(staged, orbit);

        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit, VesselLinePatch.ActualLinePhase.WorkerPreownedCache);
        var operations = new FaultingActualLineOperations(phase);
        bool runOriginal = VesselLinePatch.ExecuteActualTakeover(
            ref recovery, ref operations, out IDisposable? lease,
            reporter: static _ => { });

        try
        {
            Assert.True(runOriginal);
            Assert.NotNull(lease);
            Assert.Equal(phase, recovery.Phase);
            Assert.Equal(ExpectedOperationMaskThrough(phase), operations.ExecutedMask);
            AssertCacheBits(stock, orbit);
        }
        finally
        {
            // The production postfix/finalizer runs on the same render thread that
            // acquired the Monitor-backed StageCache lease.
            lease?.Dispose();
        }
    }

    [Fact]
    public void Worker_preowned_failure_before_the_orchestration_restores_exact_stock_cache()
    {
        Orbit orbit = UninitializedOrbit();
        OrbitPointCce[] stock = StockPoints(15.0);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, stock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, ModPoints(1500.0));

        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit, VesselLinePatch.ActualLinePhase.WorkerPreownedCache);
        bool runOriginal = VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease,
            reporter: static _ => { });

        try
        {
            Assert.True(runOriginal);
            Assert.NotNull(lease);
            AssertCacheBits(stock, orbit);
        }
        finally
        {
            VesselLinePatch.ReleaseFallbackLease(lease);
        }
    }

    [Fact]
    public void Membership_scan_failure_remains_unclassified_and_fails_closed()
    {
        var recovery = VesselLinePatch.RecoveryState.Stock;
        var operations = new FaultingMembershipOperations();

        Exception error = Assert.Throws<InjectedActualLineException>(() =>
            VesselLinePatch.ClassifyPlanRoute(ref recovery, ref operations));

        Assert.Equal(VesselLinePatch.PlanRoute.Unclassified, recovery.Route);
        Assert.False(VesselLinePatch.RecoverFailure(
            in recovery, error, out IDisposable? lease,
            reporter: static _ => { }));
        Assert.Null(lease);
    }

    [Fact]
    public void Unrelated_route_is_marked_stock_only_after_negative_membership_scan()
    {
        var recovery = VesselLinePatch.RecoveryState.Unclassified;
        var operations = new UnrelatedPlanOperations();

        VesselLinePatch.PlanRoute route =
            VesselLinePatch.ClassifyPlanRoute(ref recovery, ref operations);

        Assert.Equal(VesselLinePatch.PlanRoute.Stock, route);
        Assert.Equal(VesselLinePatch.PlanRoute.Stock, recovery.Route);
        Assert.True(operations.MembershipChecked);
    }

    [Fact]
    public void Mod_and_empty_buffers_retain_the_last_finite_anomaly_snapshot()
    {
        OrbitPointCce[] stock = StockPoints(20.0);
        OrbitPointCce[] prior = stock.ToArray();

        OrbitPointCce[]? afterMod = TrajectoryOverlay.UpdateStockSnapshot(
            ModPoints(2000.0), prior);
        OrbitPointCce[]? afterEmpty = TrajectoryOverlay.UpdateStockSnapshot(
            ReadOnlySpan<OrbitPointCce>.Empty, afterMod);

        Assert.Same(prior, afterMod);
        Assert.Same(prior, afterEmpty);
        AssertPointArraysBits(stock, afterEmpty!);

        // Exercise the same decision through the per-Orbit production ledger too.
        Orbit orbit = UninitializedOrbit();
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, stock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, ModPoints(3000.0));
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);

        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit, VesselLinePatch.ActualLinePhase.DenseLineDraw);
        Assert.True(VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease));
        try { AssertCacheBits(stock, orbit); }
        finally { lease?.Dispose(); }
    }

    [Fact]
    public void Newer_stock_recalculation_refreshes_the_restore_ledger()
    {
        Orbit orbit = UninitializedOrbit();
        OrbitPointCce[] firstStock = StockPoints(30.0);
        OrbitPointCce[] newerStock = StockPoints(40.0);

        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, firstStock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, ModPoints(4000.0));

        // Stock recalculation lands between render stages; the next stage observes it.
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, newerStock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, ModPoints(5000.0));

        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit, VesselLinePatch.ActualLinePhase.CameraPreparation);
        Assert.True(VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease));
        try { AssertCacheBits(newerStock, orbit); }
        finally { lease?.Dispose(); }
    }

    [Fact]
    public void Missing_restore_ledger_suppresses_original_and_keeps_mod_cache()
    {
        Orbit orbit = UninitializedOrbit();
        OrbitPointCce[] staged = ModPoints(6000.0);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, staged);

        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit, VesselLinePatch.ActualLinePhase.StageHandoff);
        bool runOriginal = VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease);

        Assert.False(runOriginal);
        Assert.Null(lease);
        AssertCacheBits(staged, orbit);
    }

    [Fact]
    public void Unrelated_route_failure_allows_original_without_a_cache_lease()
    {
        var recovery = VesselLinePatch.RecoveryState.Stock;
        bool runOriginal = VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease);

        Assert.True(runOriginal);
        Assert.Null(lease);
    }

    [Fact]
    public void Actual_route_without_an_armed_orbit_fails_closed_without_throwing()
    {
        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit: null, VesselLinePatch.ActualLinePhase.WorkerPreownedCache);
        bool runOriginal = VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease);

        Assert.False(runOriginal);
        Assert.Null(lease);
    }

    [Fact]
    public void Planned_route_failure_stays_suppressed_even_when_containment_reporting_throws()
    {
        var recovery = VesselLinePatch.RecoveryState.Planned;

        bool runOriginal = VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease,
            reporter: static _ => throw new InvalidOperationException("broken test logger"));

        Assert.False(runOriginal);
        Assert.Null(lease);
    }

    [Fact]
    public void Fallback_lease_blocks_worker_restage_until_the_stock_original_finishes()
    {
        Orbit orbit = UninitializedOrbit();
        OrbitPointCce[] stock = StockPoints(50.0);
        OrbitPointCce[] staged = ModPoints(7000.0);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, stock);
        TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
        TrajectoryOverlay.ReplaceCachedPointsForTest(orbit, staged);

        var recovery = VesselLinePatch.RecoveryState.Actual(
            orbit, VesselLinePatch.ActualLinePhase.WorkerPreownedCache);
        Assert.True(VesselLinePatch.RecoverFailure(
            in recovery, new InjectedActualLineException(), out IDisposable? lease));
        Assert.NotNull(lease);
        AssertCacheBits(stock, orbit);

        using var workerStarted = new ManualResetEventSlim();
        using var workerCompleted = new ManualResetEventSlim();
        Exception? workerError = null;
        var worker = new Thread(() =>
        {
            workerStarted.Set();
            try
            {
                // This production ledger operation takes the same StageCache gate as
                // worker Stage. It cannot enter while stock's original owns the lease.
                TrajectoryOverlay.PreserveStockCacheForFallback(orbit);
            }
            catch (Exception e)
            {
                workerError = e;
            }
            finally
            {
                workerCompleted.Set();
            }
        }) { IsBackground = true };

        try
        {
            worker.Start();
            Assert.True(workerStarted.Wait(TimeSpan.FromSeconds(5)));
            Assert.True(SpinWait.SpinUntil(
                () => (worker.ThreadState & ThreadState.WaitSleepJoin) != 0, 5000),
                "worker never reached the contended StageCache gate");
            Assert.False(workerCompleted.IsSet);
            AssertCacheBits(stock, orbit);

            // Simulate both Harmony cleanup paths on the acquiring render thread.
            // OneShotLease makes the second release a no-op.
            VesselLinePatch.ReleaseFallbackLease(lease);
            VesselLinePatch.ReleaseFallbackLease(lease);
            Assert.True(workerCompleted.Wait(TimeSpan.FromSeconds(5)));
            Assert.True(worker.Join(5000));
            Assert.Null(workerError);
        }
        finally
        {
            VesselLinePatch.ReleaseFallbackLease(lease);
            if (worker.IsAlive) worker.Join(5000);
        }
    }

    private struct FaultingActualLineOperations(VesselLinePatch.ActualLinePhase fault)
        : VesselLinePatch.IActualLineOperations
    {
        public int ExecutedMask { get; private set; }
        public bool ShouldDraw => true;

        public VesselLinePatch.ActualStageResult Stage()
        {
            Enter(VesselLinePatch.ActualLinePhase.StageHandoff);
            return default;
        }

        public double3 PrepareCamera()
        {
            Enter(VesselLinePatch.ActualLinePhase.CameraPreparation);
            return default;
        }

        public bool PrepareBypassVisibility()
        {
            Enter(VesselLinePatch.ActualLinePhase.BypassVisibilityPreparation);
            return false;
        }

        public void DrawDense(
            in VesselLinePatch.ActualStageResult stage,
            in double3 positionEgo,
            bool bypassVisibility) =>
            Enter(VesselLinePatch.ActualLinePhase.DenseLineDraw);

        public void EvidenceTail() =>
            Enter(VesselLinePatch.ActualLinePhase.TakeoverEvidenceTail);

        private void Enter(VesselLinePatch.ActualLinePhase phase)
        {
            ExecutedMask |= 1 << (int)phase;
            if (phase == fault) throw new InjectedActualLineException();
        }
    }

    private struct FaultingMembershipOperations : VesselLinePatch.IPlanRouteOperations
    {
        public bool IsActual() => false;
        public bool IsPlanned() => throw new InjectedActualLineException();
    }

    private struct UnrelatedPlanOperations : VesselLinePatch.IPlanRouteOperations
    {
        public bool MembershipChecked { get; private set; }
        public bool IsActual() => false;

        public bool IsPlanned()
        {
            MembershipChecked = true;
            return false;
        }
    }

    private static int ExpectedOperationMaskThrough(
        VesselLinePatch.ActualLinePhase phase) =>
        (1 << ((int)phase + 1)) - 2; // phase bits 1..N; bit 0 is pre-harness

    private sealed class InjectedActualLineException : Exception;

    private static Orbit UninitializedOrbit()
    {
        Type concrete = typeof(Orbit).GetNestedType(
            "Elliptical", BindingFlags.Public | BindingFlags.NonPublic)
            ?? throw new InvalidOperationException("KSA Orbit.Elliptical fixture type is missing");
        return (Orbit)RuntimeHelpers.GetUninitializedObject(concrete);
    }

    private static OrbitPointCce[] StockPoints(double seed) =>
    [
        new(new double3(seed + 0.125, -seed - 0.25, seed + 0.5),
            new SimTime(seed + 10.25), new SimTime(seed + 100.5),
            new TrueAnomaly(-2.75), inDangerZone: true),
        new(new double3(-seed - 1.125, seed + 1.25, -seed - 1.5),
            new SimTime(seed + 20.5), new SimTime(seed + 200.75),
            new TrueAnomaly(0.125), inDangerZone: false),
        // A wrapped payload and second danger run exercise UpdateCachedPoints'
        // derived line metadata as well as every OrbitPointCce field.
        new(new double3(seed + 2.125, seed + 2.25, seed + 2.5),
            new SimTime(-seed - 30.75), new SimTime(seed + 300.875),
            new TrueAnomaly(5.75), inDangerZone: true),
    ];

    private static OrbitPointCce[] ModPoints(double seed) =>
    [
        new(new double3(seed + 0.5, seed + 1.5, seed + 2.5),
            new SimTime(seed + 3.5), new SimTime(seed + 4.5), TrueAnomaly.NaN),
        new(new double3(-seed - 5.5, -seed - 6.5, -seed - 7.5),
            new SimTime(seed + 8.5), new SimTime(seed + 9.5), TrueAnomaly.NaN,
            inDangerZone: true),
    ];

    private static void AssertCacheBits(OrbitPointCce[] expected, Orbit orbit)
    {
        Assert.Equal(expected.Length, orbit.LineCount);
        Span<OrbitPointCce> actual = orbit.CachedPoints;
        Assert.Equal(expected.Length, actual.Length);
        for (int i = 0; i < expected.Length; i++)
            AssertPointBits(expected[i], actual[i]);
    }

    private static void AssertPointArraysBits(
        OrbitPointCce[] expected, OrbitPointCce[] actual)
    {
        Assert.Equal(expected.Length, actual.Length);
        for (int i = 0; i < expected.Length; i++)
            AssertPointBits(expected[i], actual[i]);
    }

    private static void AssertPointBits(in OrbitPointCce expected, in OrbitPointCce actual)
    {
        AssertBits(expected.PositionCce.X, actual.PositionCce.X);
        AssertBits(expected.PositionCce.Y, actual.PositionCce.Y);
        AssertBits(expected.PositionCce.Z, actual.PositionCce.Z);
        AssertBits(expected.TimeSincePe.Seconds(), actual.TimeSincePe.Seconds());
        AssertBits(expected.RemainingTimeTo.Seconds(), actual.RemainingTimeTo.Seconds());
        AssertBits(expected.TrueAnomaly.Value(), actual.TrueAnomaly.Value());
        AssertBits(expected.CompassTrueAnomaly, actual.CompassTrueAnomaly);
        Assert.Equal(expected.InDangerZone, actual.InDangerZone);
    }

    private static void AssertBits(double expected, double actual) =>
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected),
            BitConverter.DoubleToInt64Bits(actual));
}

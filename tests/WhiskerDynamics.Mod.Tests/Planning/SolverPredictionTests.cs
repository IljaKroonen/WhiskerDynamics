using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning;

/// <summary>Deterministic coverage for the detached rendezvous/periapsis prediction
/// seam. The tests use the real RailsService gate to verify that detached prediction
/// does not depend on live-rails ownership.</summary>
public sealed class SolverPredictionTests : IDisposable
{
    private const double RequestedFrom = 10_000.0;
    private const double RequestedTo = 20_000.0;
    private const double ReadyThrough = RequestedFrom + 86_400.0;

    private readonly string _dir;
    private readonly ModConfig _config;
    private readonly RailsService _rails;

    public SolverPredictionTests()
    {
        _dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-solver-prediction-tests-" + Guid.NewGuid().ToString("N"));
        string xmlDir = Path.Combine(_dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));

        _config = new ModConfig { RailsAheadDays = 1 };
        var constants = new GameConstants(
            6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        _rails = TestRailsService.FromFixture(_config, constants);
        _rails.NoteSimTime(RequestedFrom);
        Assert.True(SpinWait.SpinUntil(() => _rails.IsReadyAt(ReadyThrough), 5000),
            "fixture rails did not reach the requested solver window");
    }

    [Fact]
    public void Missing_window_is_built_by_the_worker_and_forks_cover_exact_boundaries()
    {
        // A miss is request-only on this caller.  The worker performs the segment copy.
        Assert.Null(_rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo));

        RailsService.PredictionContext? first = null;
        Assert.True(SpinWait.SpinUntil(() =>
            (first = _rails.TryCaptureSolverPredictionContext(
                RequestedFrom, RequestedTo)) is not null, 5000));
        var second = Assert.IsType<RailsService.PredictionContext>(
            _rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo));

        Assert.NotSame(first, second);
        Assert.NotSame(first!.Gravity, second.Gravity);
        Assert.True(first.StartTime <= RequestedFrom);
        Assert.True(first.Horizon >= RequestedTo);

        // Both requested endpoints are real coverage, not an epsilon-short cache hit.
        Assert.Equal(first.GetAbsolute("Mercury", RequestedFrom),
            second.GetAbsolute("Mercury", RequestedFrom));
        Assert.Equal(first.GetAbsolute("Mercury", RequestedTo),
            second.GetAbsolute("Mercury", RequestedTo));
        _ = first.GetAbsolute("Mercury", first.StartTime);
        _ = first.GetAbsolute("Mercury", first.Horizon);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            first.GetAbsolute("Mercury", Math.BitDecrement(first.StartTime)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            first.GetAbsolute("Mercury", Math.BitIncrement(first.Horizon)));
    }

    [Fact]
    public void Private_captures_have_distinct_gravity_and_bit_identical_trajectories()
    {
        var first = Capture();
        var second = Assert.IsType<RailsService.PredictionContext>(
            _rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo));
        Assert.NotSame(first.Gravity, second.Gravity);

        var seed = MercuryOrbit(first, RequestedFrom);
        var firstPredictor = NewPredictor(first.Gravity, seed);
        var secondPredictor = NewPredictor(second.Gravity, seed);
        var firstSolver = new SolverPrediction(first, static () => false);
        var secondSolver = new SolverPrediction(second, static () => false);

        var firstFinal = firstSolver.StateAt(firstPredictor, RequestedTo, 500.0);
        var secondFinal = secondSolver.StateAt(secondPredictor, RequestedTo, 500.0);

        Assert.Equal(first.GetAbsolute("Mercury", 15_000.0),
            second.GetAbsolute("Mercury", 15_000.0));
        Assert.Equal(firstFinal, secondFinal);
        Assert.Equal(firstPredictor.Nodes.ToArray(), secondPredictor.Nodes.ToArray());
    }

    [Fact]
    public async Task Concurrent_forks_have_deterministic_parity_without_a_shared_gravity_cache()
    {
        var first = Capture();
        var second = Assert.IsType<RailsService.PredictionContext>(
            _rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo));
        Assert.NotSame(first.Gravity, second.Gravity);
        var seed = MercuryOrbit(first, RequestedFrom);

        using var start = new Barrier(2);
        Task<(StateVector Final, TrajectoryNode[] Nodes)> firstRun = Task.Run(() =>
            RunFork(first, seed, start));
        Task<(StateVector Final, TrajectoryNode[] Nodes)> secondRun = Task.Run(() =>
            RunFork(second, seed, start));

        var results = await Task.WhenAll(firstRun, secondRun)
            .WaitAsync(TimeSpan.FromSeconds(10));
        Assert.Equal(results[0].Final, results[1].Final);
        Assert.Equal(results[0].Nodes, results[1].Nodes);
    }

    [Fact]
    public async Task Propagation_and_relative_body_reads_finish_while_live_gate_is_held()
    {
        var context = Capture();
        var seed = MercuryOrbit(context, RequestedFrom);
        var predictor = NewPredictor(context.Gravity, seed);
        var detached = new SolverPrediction(context, static () => false);
        using var gateEntered = new ManualResetEventSlim();
        using var releaseGate = new ManualResetEventSlim();
        var holder = new Thread(() =>
        {
            lock (_rails.Gate)
            {
                gateEntered.Set();
                releaseGate.Wait();
            }
        })
        {
            IsBackground = true,
            Name = "solver-prediction-test-gate-holder",
        };

        holder.Start();
        Assert.True(gateEntered.Wait(5000));
        try
        {
            Task<(StateVector Absolute, (Vector3d RRel, Vector3d VRel) Relative,
                StateVector Body)> work = Task.Run(() =>
            {
                var absolute = detached.StateAt(predictor, RequestedTo, 500.0);
                var relative = detached.RelativeState(
                    predictor, "Mercury", RequestedTo, 500.0);
                var body = detached.GetAbsolute("Mercury", RequestedTo);
                return (absolute, relative, body);
            });

            var result = await work.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.False(releaseGate.IsSet);
            Assert.Equal(result.Absolute.Position - result.Body.Position,
                result.Relative.RRel);
            Assert.Equal(result.Absolute.Velocity - result.Body.Velocity,
                result.Relative.VRel);
        }
        finally
        {
            releaseGate.Set();
            Assert.True(holder.Join(5000));
        }
    }

    [Fact]
    public void Cancellation_is_checked_between_chunks_and_before_the_final_read()
    {
        var context = Capture();
        var seed = MercuryOrbit(context, RequestedFrom);

        int betweenChecks = 0;
        var between = new SolverPrediction(context,
            () => Interlocked.Increment(ref betweenChecks) >= 3);
        var betweenPredictor = NewPredictor(context.Gravity, seed);
        Assert.Throws<OperationCanceledException>(() =>
            between.StateAt(betweenPredictor, RequestedFrom + 2_000.0, 500.0));
        Assert.Equal(RequestedFrom + 1_000.0, betweenPredictor.Horizon);

        var finalContext = Assert.IsType<RailsService.PredictionContext>(
            _rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo));
        int finalChecks = 0;
        var beforeFinalRead = new SolverPrediction(finalContext,
            () => Interlocked.Increment(ref finalChecks) >= 2);
        var finalPredictor = NewPredictor(finalContext.Gravity, seed);
        Assert.Throws<OperationCanceledException>(() =>
            beforeFinalRead.StateAt(finalPredictor, RequestedFrom + 500.0, 500.0));
        Assert.Equal(RequestedFrom + 500.0, finalPredictor.Horizon);

        var alreadyCancelled = new SolverPrediction(finalContext, static () => true);
        Assert.Throws<OperationCanceledException>(() =>
            alreadyCancelled.GetAbsolute("Mercury", RequestedFrom));
    }

    [Fact]
    public async Task Snapshot_preparation_releases_gate_between_bounded_chunks_and_publishes_once()
    {
        Assert.Equal(ThreadPriority.BelowNormal, _rails.WorkerPriorityForTest);
        Assert.InRange(_rails.PredictionSnapshotBudgetMsForTest, 1, 2);
        Assert.Equal(1, _rails.PredictionSnapshotCycleHandoffMsForTest);

        // Put the requested window well behind the one-day dense tail, so the test
        // exercises several independently copied committed slices.
        double extendedNow = RequestedFrom + 2 * 86_400.0;
        double extendedReadyThrough = extendedNow + 86_400.0;
        _rails.NoteSimTime(extendedNow);
        Assert.True(SpinWait.SpinUntil(
            () => _rails.IsReadyAt(extendedReadyThrough), 10_000),
            "fixture rails did not grow far enough to commit the test window");

        _rails.PredictionSnapshotChunkSecondsForTest = 2_500.0;
        using var firstChunkCaptured = new ManualResetEventSlim();
        using var resumeWorker = new ManualResetEventSlim();
        using var readerEntered = new ManualResetEventSlim();
        using var releaseReader = new ManualResetEventSlim();
        int capturedChunks = 0;
        _rails.PredictionSnapshotChunkCapturedForTest = count =>
        {
            Volatile.Write(ref capturedChunks, count);
            if (count == 1)
            {
                firstChunkCaptured.Set();
                resumeWorker.Wait(TimeSpan.FromSeconds(10));
            }
        };

        Task? reader = null;
        try
        {
            Assert.Null(_rails.TryCaptureSolverPredictionContext(
                RequestedFrom, RequestedTo));
            Assert.True(firstChunkCaptured.Wait(10_000),
                "worker did not capture the first bounded slice");

            // The worker callback runs after releasing Gate. A gameplay reader can
            // therefore enter while a multi-slice build is paused between slices.
            reader = Task.Run(() =>
            {
                lock (_rails.Gate)
                {
                    readerEntered.Set();
                    releaseReader.Wait(TimeSpan.FromSeconds(10));
                }
            });
            Assert.True(readerEntered.Wait(5_000),
                "foreground reader could not interleave between snapshot slices");
            releaseReader.Set();
            await reader.WaitAsync(TimeSpan.FromSeconds(5));

            // Change the live interpolation commit generation between slices. The
            // first slice ended behind SnapshotStableThrough, so later tip growth can
            // append/commit without invalidating or forcing a restart of that slice.
            double horizonBeforeGrowth = _rails.Horizon;
            lock (_rails.Gate)
                _ = _rails.VesselGravity.AccelerationAt(
                    new Vector3d(1e12, 2e12, 3e12),
                    horizonBeforeGrowth + 2 * 86_400.0);
            Assert.True(_rails.Horizon > horizonBeforeGrowth);

            // One slice is not published as a shortened context.
            Assert.Null(_rails.TryCaptureSolverPredictionContext(
                RequestedFrom, RequestedTo));
            resumeWorker.Set();

            RailsService.PredictionContext? complete = null;
            Assert.True(SpinWait.SpinUntil(() =>
                (complete = _rails.TryCaptureSolverPredictionContext(
                    RequestedFrom, RequestedTo)) is not null, 10_000));
            Assert.True(Volatile.Read(ref capturedChunks) >= 4,
                $"expected several bounded slices, saw {capturedChunks}");
            Assert.True(complete!.StartTime <= RequestedFrom);
            Assert.True(complete.Horizon >= RequestedTo);

            // Exact slice boundaries route through the composite without changing
            // the owner's deterministic interpolation result.
            double boundary = RequestedFrom + 2_500.0;
            Assert.Equal(_rails.GetAbsolute("Mercury", boundary),
                complete.GetAbsolute("Mercury", boundary));
            Assert.Equal(_rails.GetAbsolute("Mercury", RequestedTo),
                complete.GetAbsolute("Mercury", RequestedTo));
        }
        finally
        {
            releaseReader.Set();
            resumeWorker.Set();
            _rails.PredictionSnapshotChunkCapturedForTest = null;
            if (reader is not null)
                await reader.WaitAsync(TimeSpan.FromSeconds(5));
        }
    }

    [Fact]
    public void Solver_capture_rejects_original_bounds_outside_live_retention_and_horizon()
    {
        var covered = Capture();
        Assert.True(covered.StartTime <= RequestedFrom);
        Assert.True(covered.Horizon >= RequestedTo);

        // The owner begins at the game epoch (zero), so a request before that epoch
        // cannot be represented by the covered suffix.
        Assert.Null(_rails.TryCaptureSolverPredictionContext(-1.0, RequestedTo));

        double liveHorizon = _rails.Horizon;
        Assert.Null(_rails.TryCaptureSolverPredictionContext(
            RequestedFrom, Math.BitIncrement(liveHorizon)));

        // Rejections do not damage the valid cached context.
        var exact = Assert.IsType<RailsService.PredictionContext>(
            _rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo));
        Assert.Equal(covered.GetAbsolute("Mercury", RequestedTo),
            exact.GetAbsolute("Mercury", RequestedTo));
    }

    [Fact]
    public void Snapshot_build_restarts_when_retention_prunes_its_next_uncopied_instant()
    {
        double extendedNow = RequestedFrom + 2 * 86_400.0;
        double extendedReadyThrough = extendedNow + 86_400.0;
        _rails.NoteSimTime(extendedNow);
        Assert.True(SpinWait.SpinUntil(
            () => _rails.IsReadyAt(extendedReadyThrough), 10_000));

        double stableThrough = _rails.PredictionStableThroughForTest;
        Assert.True(stableThrough > RequestedFrom + 2_500.0);
        double pruneNow = Math.Max(extendedNow, stableThrough);
        double requestedTo = Math.Min(
            Math.BitDecrement(_rails.Horizon), pruneNow + 10_000.0);
        Assert.True(requestedTo > pruneNow);

        _rails.PredictionSnapshotChunkSecondsForTest = 2_500.0;
        using var firstChunkCaptured = new ManualResetEventSlim();
        using var resumeWorker = new ManualResetEventSlim();
        int paused = 0;
        _rails.PredictionSnapshotChunkCapturedForTest = _ =>
        {
            if (Interlocked.CompareExchange(ref paused, 1, 0) != 0) return;
            firstChunkCaptured.Set();
            resumeWorker.Wait(TimeSpan.FromSeconds(10));
        };

        try
        {
            Assert.Null(_rails.TryCapturePredictionContext(
                RequestedFrom, requestedTo));
            Assert.True(firstChunkCaptured.Wait(10_000));

            // The blocked hook makes this cycle exceed its burst budget. The next
            // cycle applies retention before attempting slice two.
            _config.RailsKeepBehindDays = 0;
            _rails.NoteSimTime(pruneNow);
            _rails.PredictionSnapshotChunkCapturedForTest = null;
            Thread.Sleep(20); // deliberately exhaust the worker's 2 ms soft budget
            resumeWorker.Set();

            Assert.True(SpinWait.SpinUntil(() =>
                _rails.PredictionRetainedStartForTest
                    > RequestedFrom + 2_500.0, 10_000),
                "retention did not prune the next uncopied instant");
            double retainedStart = _rails.PredictionRetainedStartForTest;

            RailsService.PredictionContext? rebuilt = null;
            Assert.True(SpinWait.SpinUntil(() =>
                (rebuilt = _rails.TryCapturePredictionContext(
                    RequestedFrom, requestedTo)) is not null, 10_000));

            Assert.Equal(retainedStart, rebuilt!.StartTime);
            Assert.True(rebuilt.Horizon >= requestedTo);
            Assert.Equal(_rails.GetAbsolute("Mercury", retainedStart),
                rebuilt.GetAbsolute("Mercury", retainedStart));
            Assert.Null(_rails.TryCaptureSolverPredictionContext(
                RequestedFrom, requestedTo));
        }
        finally
        {
            _rails.PredictionSnapshotChunkCapturedForTest = null;
            resumeWorker.Set();
        }
    }

    [Fact]
    public void Request_widened_after_completion_before_publication_is_not_lost()
    {
        double widenedTo = RequestedTo + 10_000.0;
        Assert.True(_rails.IsReadyAt(widenedTo));
        using var beforeFirstPublish = new ManualResetEventSlim();
        using var releasePublish = new ManualResetEventSlim();
        int paused = 0;
        _rails.PredictionSnapshotBeforePublishForTest = () =>
        {
            if (Interlocked.CompareExchange(ref paused, 1, 0) != 0) return;
            beforeFirstPublish.Set();
            releasePublish.Wait(TimeSpan.FromSeconds(10));
        };

        try
        {
            Assert.Null(_rails.TryCaptureSolverPredictionContext(
                RequestedFrom, RequestedTo));
            Assert.True(beforeFirstPublish.Wait(10_000),
                "initial build did not reach the pre-publication boundary");

            // Widen while the completed narrow context exists only on the worker's
            // stack. Publishing it must leave this request pending, never clear it.
            Assert.Null(_rails.TryCaptureSolverPredictionContext(
                RequestedFrom, widenedTo));
            _rails.PredictionSnapshotBeforePublishForTest = null;
            releasePublish.Set();

            RailsService.PredictionContext? widened = null;
            Assert.True(SpinWait.SpinUntil(() =>
                (widened = _rails.TryCaptureSolverPredictionContext(
                    RequestedFrom, widenedTo)) is not null, 10_000));
            Assert.True(widened!.StartTime <= RequestedFrom);
            Assert.True(widened.Horizon >= widenedTo);
            Assert.Equal(_rails.GetAbsolute("Mercury", widenedTo),
                widened.GetAbsolute("Mercury", widenedTo));
        }
        finally
        {
            _rails.PredictionSnapshotBeforePublishForTest = null;
            releasePublish.Set();
        }
    }

    [Fact]
    public void Detached_frame_sampling_contains_target_callback_exceptions_per_batch()
    {
        var prediction = Capture();
        var targetFrame = new FrameSpec(FrameKind.TargetFixed, "Mercury", "Probe");
        var snapshot = new ActiveFrameSnapshot(
            targetFrame, default, RequestedFrom, default, Generation: 17);
        int callbackCalls = 0;

        Assert.False(FrameManager.TrySamplePoseForCurve(
            snapshot, prediction,
            _ =>
            {
                callbackCalls++;
                throw new InvalidOperationException("detached target callback probe");
            },
            RequestedFrom, out var failedPose));
        Assert.Equal(1, callbackCalls);
        Assert.Equal(default, failedPose);

        // The exception invalidates only that sample/batch. It does not latch failure
        // or retire the immutable activation snapshot: the same snapshot can sample a
        // healthy captured target immediately afterwards.
        Assert.True(FrameManager.TrySamplePoseForCurve(
            snapshot, prediction, t => MercuryOrbit(prediction, t),
            RequestedFrom, out var recoveredPose));
        Assert.Null(FrameCatalog.ValidatePose(recoveredPose));
    }

    public void Dispose()
    {
        _rails.Dispose();
        try { Directory.Delete(_dir, recursive: true); } catch { /* best effort */ }
    }

    private RailsService.PredictionContext Capture()
    {
        RailsService.PredictionContext? context =
            _rails.TryCaptureSolverPredictionContext(RequestedFrom, RequestedTo);
        if (context is not null) return context;
        Assert.True(SpinWait.SpinUntil(() =>
            (context = _rails.TryCaptureSolverPredictionContext(
                RequestedFrom, RequestedTo)) is not null, 5000));
        return context!;
    }

    private StateVector MercuryOrbit(
        RailsService.PredictionContext context, double time)
    {
        var mercury = context.GetAbsolute("Mercury", time);
        const double radius = 2.74e6;
        return new StateVector(
            mercury.Position + new Vector3d(radius, 0, 0),
            mercury.Velocity + new Vector3d(
                0, Math.Sqrt(_rails.MuOf("Mercury") / radius), 0));
    }

    private static TrajectoryPredictor NewPredictor(
        GravityModel gravity, StateVector seed) =>
        new(gravity, seed, RequestedFrom,
            new IntegratorOptions { RelTol = 1e-9 });

    private static (StateVector Final, TrajectoryNode[] Nodes) RunFork(
        RailsService.PredictionContext context, StateVector seed, Barrier start)
    {
        var solver = new SolverPrediction(context, static () => false);
        var predictor = NewPredictor(solver.Gravity, seed);
        start.SignalAndWait();
        var final = solver.StateAt(predictor, RequestedTo, 500.0);
        return (final, predictor.Nodes.ToArray());
    }
}

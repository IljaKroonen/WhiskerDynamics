using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Tests.Overlay;

namespace WhiskerDynamics.Mod.Tests.Vessels;

/// <summary>Launch-time authority and coverage tests for the detached solver seeds.
/// These exercise real TrackedVessel predictors and the real shared rails gate; no
/// worker solver is needed to prove the capture transaction.</summary>
[Collection(nameof(OrbitCacheCoordinationTestCollection))]
public sealed class SolverSeedCaptureTests : IDisposable
{
    private const double Now = 10_000.0;
    private readonly string _dir;
    private readonly RailsService _rails;

    public SolverSeedCaptureTests()
    {
        _dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-solver-seed-tests-" + Guid.NewGuid().ToString("N"));
        string xmlDir = Path.Combine(_dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));

        _rails = CreateRails();
    }

    [Fact]
    public void Rendezvous_capture_returns_both_lineages_and_now_states_together()
    {
        var chaser = MakeVessel(_rails, "chaser", Now, 2.74e6);
        var target = MakeVessel(_rails, "target", Now, 3.14e6);
        var expectedChaserLineage = chaser.Predictor;
        var expectedTargetLineage = target.Predictor;

        bool captured = chaser.TryCaptureRendezvousSolverSeeds(
            target, Now,
            out var chaserLineage, out var chaserSeed,
            out var targetLineage, out var targetSeed);

        Assert.True(captured);
        Assert.Same(expectedChaserLineage, chaserLineage);
        Assert.Same(expectedTargetLineage, targetLineage);
        Assert.Equal(expectedChaserLineage.Nodes[0].State, chaserSeed);
        Assert.Equal(expectedTargetLineage.Nodes[0].State, targetSeed);
    }

    [Fact]
    public async Task Pending_publication_cannot_interleave_between_the_two_seed_reads()
    {
        var chaser = MakeVessel(_rails, "chaser", Now, 2.74e6);
        var target = MakeVessel(_rails, "target", Now, 3.14e6);
        var expectedChaserLineage = chaser.Predictor;
        var expectedTargetLineage = target.Predictor;
        using var betweenReads = new ManualResetEventSlim();
        using var releaseCapture = new ManualResetEventSlim();
        using var publicationStarted = new ManualResetEventSlim();

        Task<(bool Captured, TrajectoryPredictor ChaserLineage, StateVector ChaserSeed,
            TrajectoryPredictor TargetLineage, StateVector TargetSeed)> capture = Task.Run(() =>
        {
            bool accepted = chaser.TryCaptureRendezvousSolverSeedsForTest(
                target, Now,
                out var chaserLineage, out var chaserSeed,
                out var targetLineage, out var targetSeed,
                () =>
                {
                    betweenReads.Set();
                    if (!releaseCapture.Wait(TimeSpan.FromSeconds(5)))
                        throw new TimeoutException("test did not release seed capture");
                });
            return (accepted, chaserLineage, chaserSeed, targetLineage, targetSeed);
        });

        Assert.True(betweenReads.Wait(TimeSpan.FromSeconds(5)));
        Task publication = Task.Run(() =>
        {
            publicationStarted.Set();
            target.MarkReseedPending();
        });
        try
        {
            Assert.True(publicationStarted.Wait(TimeSpan.FromSeconds(5)));
            Task completed = await Task.WhenAny(publication, Task.Delay(100));
            Assert.NotSame(publication, completed);
        }
        finally
        {
            releaseCapture.Set();
        }
        var result = await capture.WaitAsync(TimeSpan.FromSeconds(5));
        await publication.WaitAsync(TimeSpan.FromSeconds(5));

        Assert.True(result.Captured);
        Assert.Same(expectedChaserLineage, result.ChaserLineage);
        Assert.Same(expectedTargetLineage, result.TargetLineage);
        Assert.Equal(expectedChaserLineage.Nodes[0].State, result.ChaserSeed);
        Assert.Equal(expectedTargetLineage.Nodes[0].State, result.TargetSeed);
        Assert.True(target.ReseedPending);
    }

    [Fact]
    public void Reentrant_reseed_between_reads_rejects_without_partial_outputs()
    {
        var chaser = MakeVessel(_rails, "chaser", Now, 2.74e6);
        var target = MakeVessel(_rails, "target", Now, 3.14e6);
        var oldTargetLineage = target.Predictor;
        StateVector replacement = oldTargetLineage.Nodes[0].State with
        {
            Position = oldTargetLineage.Nodes[0].State.Position
                + new Vector3d(1_000.0, 0.0, 0.0),
        };

        bool captured = chaser.TryCaptureRendezvousSolverSeedsForTest(
            target, Now,
            out var chaserLineage, out var chaserSeed,
            out var targetLineage, out var targetSeed,
            () => target.ReseedAbsolute(replacement, Now));

        Assert.False(captured);
        Assert.NotSame(oldTargetLineage, target.Predictor);
        Assert.Null(chaserLineage);
        Assert.Null(targetLineage);
        Assert.Equal(default, chaserSeed);
        Assert.Equal(default, targetSeed);
    }

    [Fact]
    public void Lagging_single_seed_is_rejected_without_extending_authoritative_predictor()
    {
        var tracked = MakeVessel(_rails, "optimizer", Now - 100.0, 2.74e6);
        var lineage = tracked.Predictor;
        double horizonBefore = lineage.Horizon;
        int nodesBefore = lineage.Nodes.Count;

        bool captured = tracked.TryCaptureSolverSeed(lineage, Now, out var seed);

        Assert.False(captured);
        Assert.Equal(default, seed);
        Assert.Equal(horizonBefore, lineage.Horizon);
        Assert.Equal(nodesBefore, lineage.Nodes.Count);
    }

    [Fact]
    public void Lagging_rendezvous_member_rejects_pair_without_extending_either_predictor()
    {
        var chaser = MakeVessel(_rails, "chaser", Now, 2.74e6);
        var target = MakeVessel(_rails, "target", Now - 100.0, 3.14e6);
        double chaserHorizon = chaser.Predictor.Horizon;
        double targetHorizon = target.Predictor.Horizon;
        int chaserNodes = chaser.Predictor.Nodes.Count;
        int targetNodes = target.Predictor.Nodes.Count;

        bool captured = chaser.TryCaptureRendezvousSolverSeeds(
            target, Now,
            out var chaserLineage, out var chaserSeed,
            out var targetLineage, out var targetSeed);

        Assert.False(captured);
        Assert.Null(chaserLineage);
        Assert.Null(targetLineage);
        Assert.Equal(default, chaserSeed);
        Assert.Equal(default, targetSeed);
        Assert.Equal(chaserHorizon, chaser.Predictor.Horizon);
        Assert.Equal(targetHorizon, target.Predictor.Horizon);
        Assert.Equal(chaserNodes, chaser.Predictor.Nodes.Count);
        Assert.Equal(targetNodes, target.Predictor.Nodes.Count);
    }

    [Fact]
    public void Rendezvous_capture_rejects_vessels_from_different_rails_services()
    {
        using var otherRails = CreateRails();
        var chaser = MakeVessel(_rails, "chaser", Now, 2.74e6);
        var target = MakeVessel(otherRails, "target", Now, 3.14e6);

        bool captured = chaser.TryCaptureRendezvousSolverSeeds(
            target, Now,
            out var chaserLineage, out var chaserSeed,
            out var targetLineage, out var targetSeed);

        Assert.False(captured);
        Assert.Null(chaserLineage);
        Assert.Null(targetLineage);
        Assert.Equal(default, chaserSeed);
        Assert.Equal(default, targetSeed);
    }

    [Fact]
    public void Reseed_revokes_captured_overlay_lineage_and_cannot_restore_its_display_cache()
    {
        var vessel = MakeVessel(_rails, "overlay-reseed", Now, 3.0e6);
        long overlayLineage = vessel.OverlayLineage;
        Assert.True(vessel.TryCaptureOverlayAnchor(
            overlayLineage, Now, out var authorityLineage, out var anchor));

        vessel.ReseedAbsolute(
            new StateVector(anchor.Position + new Vector3d(1000, 0, 0), anchor.Velocity),
            Now);

        Assert.False(vessel.IsOverlayLineageCurrent(overlayLineage, authorityLineage));
        Assert.Throws<OperationCanceledException>(() =>
            vessel.ActualDisplayPredictorAt(
                anchor, Now, _rails.VesselGravity, overlayLineage,
                authorityLineage, out _));
    }

    [Fact]
    public async Task Running_overlay_ticket_and_pending_replacement_are_revoked_by_reseed()
    {
        var vessel = MakeVessel(_rails, "overlay-running-reseed", Now, 3.2e6);
        StateVector seed = vessel.Predictor.Nodes[0].State;

        await AssertRunningOverlayWorkRevoked(vessel, () => vessel.ReseedAbsolute(
            seed with { Position = seed.Position + new Vector3d(1000, 0, 0) }, Now));
    }

    public void Dispose()
    {
        _rails.Dispose();
        try { Directory.Delete(_dir, recursive: true); } catch { /* best effort */ }
    }

    private RailsService CreateRails()
    {
        var config = new ModConfig { RailsAheadDays = 1 };
        var constants = new GameConstants(
            6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        var rails = TestRailsService.FromFixture(config, constants);
        rails.NoteSimTime(Now);
        Assert.True(SpinWait.SpinUntil(() => rails.IsReadyAt(Now), 5000),
            "fixture rails did not reach the seed time");
        return rails;
    }

    private static TrackedVessel MakeVessel(
        RailsService rails, string id, double epoch, double radius)
    {
        var mercury = rails.GetAbsolute("Mercury", epoch);
        var vessel = new TrackedVessel
        {
            Id = id,
            Rails = rails,
            Options = new IntegratorOptions { RelTol = 1e-11 },
        };
        vessel.ReseedAbsolute(
            new StateVector(
                mercury.Position + new Vector3d(radius, 0.0, 0.0),
                mercury.Velocity + new Vector3d(
                    0.0, Math.Sqrt(rails.MuOf("Mercury") / radius), 0.0)),
            epoch);
        return vessel;
    }

    private static async Task AssertRunningOverlayWorkRevoked(
        TrackedVessel vessel, Action revoke)
    {
        TrajectoryOverlay.ResetSessionStatics();
        int generation = OverlayWorker.CurrentGeneration;
        using var runningStarted = new ManualResetEventSlim();
        using var releaseRunning = new ManualResetEventSlim();
        using var runningFinished = new ManualResetEventSlim();
        using var pendingRan = new ManualResetEventSlim();
        int published = 0;

        Assert.True(OverlayWorker.Enqueue(vessel.Id, generation, (_, publish) =>
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
        Assert.True(OverlayWorker.Enqueue(vessel.Id, generation, pendingRan.Set));

        revoke();
        releaseRunning.Set();
        Assert.True(runningFinished.Wait(TimeSpan.FromSeconds(5)));
        Assert.False(pendingRan.Wait(TimeSpan.FromMilliseconds(250)));
        Assert.Equal(0, Volatile.Read(ref published));
        await Task.CompletedTask;
    }
}

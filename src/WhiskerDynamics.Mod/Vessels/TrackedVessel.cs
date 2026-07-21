using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Vessels;

/// <summary>Every input that can alter planned sampled geometry without replacing the
/// immutable plan snapshot. Kept as one value so a newly added sampling knob cannot be
/// accidentally compared at one cache site but omitted at another.</summary>
internal readonly record struct PlannedGeometryKey(
    double PlanEnd, double CoverageLimitedEnd, double ConfigHorizonDays, double ConfigRailsAheadDays,
    double ThetaMax, int MaxDensePoints,
    double FiniteBurnSliceSeconds, int FiniteBurnMaxSlices);

/// <summary>Per-vessel n-body trajectory. The Predictor lives in the mod's absolute
/// frame; conversions to/from the game's parent-relative Cci states happen here.
/// All Predictor access holds RailsService.Gate (shared NBodyEphemerides underneath).</summary>
public sealed class TrackedVessel
{
    public required string Id { get; init; }
    public required RailsService Rails { get; init; }
    public required IntegratorOptions Options { get; init; }

    public TrajectoryPredictor Predictor { get; private set; } = null!;
    private long _overlayLineage;
    /// <summary>Every ownership/reseed transition advances this display lineage.
    /// Captured overlay work must retain the exact value through publication and
    /// cache staging; worker-ticket identity alone does not cover task-thread reseeds.</summary>
    internal long OverlayLineage => Volatile.Read(ref _overlayLineage);
    public double SeedTime { get; private set; }
    public double LastCommitResidual { get; internal set; }
    public double LastConicDrift { get; internal set; }
    /// <summary>Sim-time of the last re-osculation (or seed/reseed — a fresh
    /// predictor IS a fresh conic); drives the periodic refresh trigger.</summary>
    public double LastRefreshTime { get; internal set; }
    /// <summary>Sim-time of the last staging evaluation: stamped by
    /// EvaluateForStaging on BOTH Seam 1 paths, so it tracks activity even for cluster
    /// followers, which never book re-osculation refreshes. A landed/stock-owned vessel's
    /// stamp freezes — the sidecar eligibility filter reads recency from
    /// max(LastRefreshTime, LastStagedTime).</summary>
    public double LastStagedTime { get; internal set; }
    /// <summary>Re-osculations booked for this vessel (panel evidence).</summary>
    public int RefreshCount { get; internal set; }
    internal long LastRefreshLogMs { get; set; }
    public CanaryCounter Canary { get; } = new();
    public int CanaryStrikes => Canary.Strikes;

    /// <summary>Set while live physics owns the vessel: the predictor must reseed from
    /// committed truth before rails authority resumes.</summary>
    private int _reseedPending;
    internal bool ReseedPending => Volatile.Read(ref _reseedPending) != 0;

    /// <summary>Transfers propagation ownership away from this predictor. The flag
    /// mutation is both a canary continuity transition and a rails-authority
    /// publication: a concurrent verifier cannot retain the old lineage, and a
    /// rail-only operation cannot pass validation before the pending state lands.</summary>
    internal void MarkReseedPending()
    {
        lock (OverlayCaptureGate)
        {
            if (ReseedPending) return;
            CancelOverlayWork(clearSamples: false);
            Canary.BeginContinuityTransition();
            try
            {
                RailsAuthoritySynchronization.Publish(
                    Rails.Gate, this,
                    tracked =>
                    {
                        if (Volatile.Read(ref tracked._reseedPending) != 0) return;
                        Volatile.Write(ref tracked._reseedPending, 1);
                        Interlocked.Increment(ref tracked._overlayLineage);
                    });
            }
            finally
            {
                Canary.EndContinuityTransition();
            }
        }
    }

    /// <summary>Parent body id the vessel was last staged under. A change on the Keep
    /// path IS the stock patch (SOI) transition landing while on rails — the
    /// transition observation gate logs it; the predictor itself is absolute-frame and continuous
    /// across it. Stamped by <see cref="Reseed"/>.</summary>
    internal string? LastParentId { get; set; }

    /// <summary>True exactly on the staging tick where <see cref="VesselRegistry.GetOrSeed"/>
    /// observed the stock parent change (stock's patch transition landing). Written
    /// unconditionally every GetOrSeed (self-clearing — a tick that bails before
    /// EvaluateForStaging can never leak the flag into a later tick) and consumed by
    /// the teleport guard's adoption rule (<see cref="VesselLifecycle.AdoptStagedJump"/>).</summary>
    internal bool ParentTransitionTick { get; set; }
    internal long LastTransitionLogMs { get; set; }
    internal long LastSnapOverrideLogMs { get; set; }
    internal long LastReseedLogMs { get; set; }
    internal long LastBurnWitnessLogMs { get; set; }

    /// <summary>Short-lived producer reservation covering capture through enqueue.
    /// Queued/running ownership belongs to OverlayWorker's latest-wins slot instead;
    /// keeping that lifetime here would prevent a newer cadence-eligible capture from
    /// replacing stale pending work.</summary>
    private int _overlayCaptureReserved;
    private long _lastOverlayEnqueuedMs;
    private long _lastOverlayCompletedMs;
    private long _lastOverlayDurationMs;

    /// <summary>Reserves this vessel's capture/enqueue phase. Ordinary refreshes wait
    /// at least the greater of their configured cadence and the previous job's cost,
    /// measured from the later of enqueue and completion. Context/plan changes bypass
    /// that cooldown but retain a small enqueue floor so a continuously changing input
    /// cannot capture every task tick. OverlayCaptureGate serializes the two production
    /// call paths; the atomic reservation also makes the ownership contract explicit.</summary>
    internal bool TryBeginOverlayRebuild(long nowMs, long configuredPeriodMs, bool urgent)
    {
        if (Volatile.Read(ref _overlayCaptureReserved) != 0) return false;
        long enqueued = Volatile.Read(ref _lastOverlayEnqueuedMs);
        long completed = Volatile.Read(ref _lastOverlayCompletedMs);
        if (urgent)
        {
            if (enqueued != 0 && nowMs - enqueued < 50) return false;
        }
        else
        {
            long duration = Math.Max(0, Volatile.Read(ref _lastOverlayDurationMs));
            long cooldown = Math.Max(50, Math.Max(configuredPeriodMs, duration));
            long cadenceStart = Math.Max(enqueued, completed);
            if (cadenceStart != 0 && nowMs - cadenceStart < cooldown) return false;
        }
        return Interlocked.CompareExchange(ref _overlayCaptureReserved, 1, 0) == 0;
    }

    /// <summary>Reserves a live-physics capture without a rebuild cooldown. The
    /// overlay worker keeps the active build through completion and coalesces newer
    /// captures into one pending slot, so live updates keep it continuously supplied
    /// without interrupting a full-window build.</summary>
    internal bool TryBeginContinuousOverlayRebuild() =>
        Interlocked.CompareExchange(ref _overlayCaptureReserved, 1, 0) == 0;

    /// <summary>Commits a successful enqueue and releases the capture reservation.
    /// Stamp-before-release ordering ensures the next producer cannot observe an open
    /// reservation without also observing the cadence anchor.</summary>
    internal void CommitOverlayRebuild(long nowMs)
    {
        Volatile.Write(ref _lastOverlayEnqueuedMs, nowMs);
        Volatile.Write(ref _overlayCaptureReserved, 0);
    }

    internal void CompleteOverlayRebuild(long durationMs, long? completedMs = null)
    {
        Volatile.Write(ref _lastOverlayDurationMs, Math.Max(0, durationMs));
        Volatile.Write(ref _lastOverlayCompletedMs, completedMs ?? Environment.TickCount64);
    }

    /// <summary>Releases a capture that failed before enqueue. Job completion must not
    /// call this: it may race a newer producer's reservation.</summary>
    internal void CancelOverlayRebuild() => Volatile.Write(ref _overlayCaptureReserved, 0);

    /// <summary>Burns folded into this vessel's last overlay rebuild; -1 = no rebuild
    /// yet for this entry (per-vessel rather than a TrajectoryOverlay static — the
    /// state dies with the registry on rebind, no session sweep needed).</summary>
    internal int LastOverlayBurnsApplied { get; set; } = -1;
    internal long LastOverlayBurnChangeLogMs { get; set; }

    /// <summary>Plan-snapshot planned-batch cache identity (OverlayKernel.
    /// PlannedResampleDue): the snapshot instance, diverged flag, sample start, fold
    /// count and horizon inputs behind the last PUBLISHED planned batch. Written only
    /// from this vessel's own rebuild calls, like the stamps above.</summary>
    internal PlanSnapshot? LastPlannedSnapshot { get; set; }
    internal GravityModel? LastPlannedGravity { get; set; }
    internal bool LastPlannedDiverged { get; set; }
    internal double LastPlannedStart { get; set; } = double.NaN;
    internal int LastPlannedBurnsApplied { get; set; }
    internal PlannedGeometryKey? LastPlannedGeometryKey { get; set; }
    /// <summary>Quantized rails coverage captured by the last full planned sample.
    /// Restamps never lower it; geometric high-water growth triggers are therefore
    /// immune to the worker-lag sawtooth under warp.</summary>
    internal double LastPlannedRailsCoverageDays { get; set; }
    internal long LastSnapshotLogMs { get; set; }
    internal object? LastSnapshotEvidencePlanRef { get; set; }
    internal string? LastSnapshotMultiParentSignature { get; set; }

    /// <summary>SIM-time stamp (seconds) of the last rails-geometric SOI parent check
    /// (<see cref="VesselRegistry.RailsSoiParent"/>). Sim time, not wall clock: at high
    /// warp a wall throttle's windows span days of sim time and an entire SOI transit
    /// fits between two checks — a sim-time period degrades to once per tick there
    /// (every tick spans more than the period) while staying cheap at 1x. Written only
    /// from this vessel's own Seam 1 staging calls, like the stamps above.</summary>
    internal double LastSoiCheckSimSeconds { get; set; } = double.NegativeInfinity;

    /// <summary>Parent and predictor lineage at the stored SOI-check time.
    /// A swept SOI interval is valid only while both identities still match; reseeds,
    /// parent changes and same-id vessel replacement fall back to an endpoint check.</summary>
    internal string? LastSoiCheckParentId { get; set; }
    internal TrajectoryPredictor? LastSoiCheckPredictor { get; set; }

    /// <summary>Wall-clock stamp of the last rails-geometric re-parent log line.
    /// Deliberately NOT <see cref="LastTransitionLogMs"/>: that budget belongs to
    /// GetOrSeed's landing observation, which fires within a tick of the re-parent —
    /// sharing the stamp would suppress the landing evidence this line pairs with.</summary>
    internal long LastReparentLogMs { get; set; }

    /// <summary>Wall-clock stamp of the last witnessed live delta-v (thrust/drag):
    /// the Rebase coast gate — a vessel can linger off-rails long after the engine
    /// stops (SAS actuators, armed flight computer), and "no dv for a while" is the
    /// honest signal that the trajectory has settled enough to re-anchor on.</summary>
    internal long LastDvWitnessMs { get; set; }

    /// <summary>Diverged-ghost predictor cache: the anchored, burns-folded display
    /// predictor reused across resamples (a fresh one would re-integrate anchor->now
    /// from scratch every rebuild — unbounded under warp while un-rebased). Keyed by
    /// snapshot instance; FoldHorizon records how far the fold looked, so a burn that
    /// enters a later horizon refolds. Dropped whenever the snapshot changes or the
    /// plan un-diverges. Same single-writer discipline as the stamps above.</summary>
    internal TrajectoryPredictor? PlannedGhost { get; set; }
    internal PlanSnapshot? PlannedGhostSnapshot { get; set; }
    internal GravityModel? PlannedGhostGravity { get; set; }
    internal int PlannedGhostBurnsApplied { get; set; }
    internal double PlannedGhostFoldHorizon { get; set; }

    /// <summary>Context-change log stamps: 1 s stamps (one per event class) for the
    /// context-changed immediate rebuild — SOI parent landings and frame-mode switches
    /// each log at most once per second per vessel; the rebuild itself is never
    /// throttled by these.</summary>
    internal long LastOverlayParentChangeLogMs { get; set; }
    internal long LastOverlayFrameChangeLogMs { get; set; }
    internal long LastOverlayPlanChangeLogMs { get; set; }

    /// <summary>Plan-edit bypass stamps: the plan instance and its
    /// <see cref="FlightPlanModel.Version"/> consumed by this vessel's last enqueued
    /// rebuild. A mismatch means the plan changed since — the rebuild runs
    /// immediately instead of waiting out the cadence throttle (the edit's drawn
    /// feedback is the planner's core loop; a second of dead air there reads as a
    /// hang), floored at TrajectoryOverlay's bypass rate limit so a continuously
    /// mutating plan (a stock-editor drag reconciling every rebuild) degrades to
    /// that limit instead of a rebuild per physics tick. Written only from this
    /// vessel's own Seam 1 staging calls, like every stamp above; null ref +
    /// version 0 = "no plan seen". LastSeenPlanEditStamp mirrors the store-wide
    /// <see cref="FlightPlans.EditStamp"/> so the per-tick staging call skips even
    /// the store lookup until some plan somewhere changed.</summary>
    internal object? LastOverlayPlanRef { get; set; }
    internal long LastOverlayPlanVersion { get; set; }
    internal long LastOverlayPlanBypassMs { get; set; }
    internal long LastOverlayAnalysisLoopMs { get; set; }
    internal long LastSeenPlanEditStamp { get; set; } = -1;

    /// <summary>Wall-clock stamp of the last staging (GetOrSeed) or commit (VerifyCommit)
    /// sighting; drives eviction of entries whose vessel left the sim.</summary>
    internal long LastSeenMs { get; set; } = Environment.TickCount64;

    /// <summary>Weak identity of the live Vehicle this entry was seeded from (weak: the
    /// registry must not root a destroyed vessel). A different instance under the same
    /// Id means the original is gone — the stale-Id guard reseeds instead of staging.</summary>
    private WeakReference<Vehicle>? _vehicle;

    /// <summary>The last task state that staged this vessel. VehicleUpdateTask owns
    /// these objects for the life of the live vessel; retaining the latest reference
    /// lets the render-frame pause seam request the exact same overlay capture that a
    /// simulation tick would have requested. It is never mutated here.</summary>
    private WeakReference<VehicleUpdateState>? _lastUpdateState;
    private bool _lastUpdateWasOffRails;

    /// <summary>Serializes the normal task-thread producer with the pause-only UI
    /// producer. The overlay worker remains the sole consumer of captured jobs.</summary>
    internal object OverlayCaptureGate { get; } = new();

    internal void BindVehicle(Vehicle vehicle)
    {
        bool identityChanged = _vehicle is not null
            && (!_vehicle.TryGetTarget(out var known) || !ReferenceEquals(known, vehicle));
        if (!identityChanged)
        {
            _vehicle = new WeakReference<Vehicle>(vehicle);
            return;
        }
        Canary.BeginContinuityTransition();
        try
        {
            _vehicle = new WeakReference<Vehicle>(vehicle);
        }
        finally
        {
            Canary.EndContinuityTransition();
        }
    }

    /// <summary>Publishes a predictor reseed and its authoritative vehicle identity as
    /// one lineage transition. The nested guards in <see cref="Reseed"/> and
    /// <see cref="BindVehicle"/> remain valid, while this outer guard closes the gap
    /// between those mutations for concurrent commit verification.</summary>
    internal void ReseedAndBind(Orbit currentOrbit, in StateVectors sv, Vehicle vehicle)
    {
        lock (OverlayCaptureGate)
        {
            Canary.BeginContinuityTransition();
            try
            {
                Reseed(currentOrbit, in sv);
                BindVehicle(vehicle);
            }
            finally
            {
                Canary.EndContinuityTransition();
            }
        }
    }

    internal void BindUpdateState(VehicleUpdateState state, bool offRails = false)
    {
        lock (OverlayCaptureGate)
        {
            _lastUpdateState = new WeakReference<VehicleUpdateState>(state);
            _lastUpdateWasOffRails = offRails;
        }
    }

    internal bool TryGetUpdateState(out VehicleUpdateState state, out bool offRails)
    {
        lock (OverlayCaptureGate)
        {
            offRails = _lastUpdateWasOffRails;
            state = null!;
            return _lastUpdateState is not null && _lastUpdateState.TryGetTarget(out state!);
        }
    }

    internal bool IsSameVehicle(Vehicle vehicle) =>
        _vehicle is not null && _vehicle.TryGetTarget(out var known) && ReferenceEquals(known, vehicle);

    /// <summary>The live Vehicle this entry is bound to (the update task's ReadOnlyVehicle,
    /// which is the flown instance — Vehicle.cs:1349). False when the vessel was collected.</summary>
    internal bool TryGetVehicle(out Vehicle vehicle)
    {
        vehicle = null!;
        return _vehicle is not null && _vehicle.TryGetTarget(out vehicle!);
    }

    private void CancelOverlayWork(bool clearSamples)
    {
        OverlayWorker.Cancel(Id);
        OverlayBuffer.RevokeVessel(Id, clearSamples);
    }

    /// <summary>THE game parent-relative Cci -> mod-frame absolute conversion, shared
    /// by <see cref="Reseed"/> and <see cref="NewLiveDisplayPredictor"/> so the
    /// burn-end reseed and the mid-burn live seed can never drift apart.</summary>
    private StateVector AbsoluteFromGame(Orbit currentOrbit, in StateVectors sv,
        out double t, out string parentId)
    {
        if (currentOrbit.Parent is not Astronomical parentBody)
            throw new InvalidOperationException($"orbit parent of '{Id}' is not an Astronomical");
        t = sv.StateTime.Seconds();
        parentId = parentBody.Id;
        var parentAbs = Rails.GetAbsolute(parentBody.Id, t);
        var cci2Cce = currentOrbit.Parent.GetCci2Cce();
        return new StateVector(
            FrameAdapter.GameToAbsolute(parentAbs.Position, sv.PositionCci, cci2Cce),
            FrameAdapter.GameToAbsolute(parentAbs.Velocity, sv.VelocityCci, cci2Cce));
    }

    /// <summary>Seed (or reseed) the predictor from a game parent-relative Cci state.</summary>
    public void Reseed(Orbit currentOrbit, in StateVectors sv)
    {
        lock (OverlayCaptureGate)
        {
            CancelOverlayWork(clearSamples: false);
            Canary.BeginContinuityTransition();
            try
            {
                var absolute = AbsoluteFromGame(currentOrbit, in sv, out double t, out string parentId);
                lock (Rails.Gate)
                {
                    Interlocked.Increment(ref _overlayLineage);
                    Predictor = new TrajectoryPredictor(Rails.VesselGravity, absolute, t, Options);
                    _actualDisplay = null; // the display reuse cache dies with its seed lineage
                    // Publish the replacement and its restored authority atomically. A newer
                    // live witness that arrives after this gate is released remains pending.
                    Volatile.Write(ref _reseedPending, 0);
                    ResetSoiCheckCursor();
                }
                SeedTime = t;
                LastRefreshTime = t;
                LastParentId = parentId;
            }
            finally
            {
                Canary.EndContinuityTransition();
            }
        }
    }

    /// <summary>Exact restore from a sidecar absolute mod-frame state. Unlike
    /// <see cref="Reseed"/> this bypasses the game state entirely — the sidecar IS the
    /// n-body truth at its epoch, and the predictor extends forward from it bit-
    /// reproducibly. LastParentId is deliberately left as the preceding stock seed
    /// stamped it (the registry applies this immediately after a stock seed, and the
    /// epoch state was flown under that same parent when the save was made).</summary>
    public void ReseedAbsolute(StateVector absolute, double epochSeconds)
    {
        lock (OverlayCaptureGate)
        {
            CancelOverlayWork(clearSamples: false);
            Canary.BeginContinuityTransition();
            try
            {
                lock (Rails.Gate)
                {
                    Interlocked.Increment(ref _overlayLineage);
                    Predictor = new TrajectoryPredictor(Rails.VesselGravity, absolute, epochSeconds, Options);
                    _actualDisplay = null; // the display reuse cache dies with its seed lineage
                    Volatile.Write(ref _reseedPending, 0);
                    ResetSoiCheckCursor();
                }
                SeedTime = epochSeconds;
                LastRefreshTime = epochSeconds;
            }
            finally
            {
                Canary.EndContinuityTransition();
            }
        }
    }

    private void ResetSoiCheckCursor()
    {
        LastSoiCheckSimSeconds = double.NegativeInfinity;
        LastSoiCheckParentId = null;
        LastSoiCheckPredictor = null;
    }

    /// <summary>Predictor state at <paramref name="time"/>, expressed as the game
    /// StateVectors the stock staging path expects (parent-relative Cci). TrueAnomaly is
    /// reused from the orbit's cached state — cosmetic; refreshed by the
    /// re-osculation refresh.</summary>
    public StateVectors EvaluateGameState(Orbit currentOrbit, SimTime time)
    {
        if (currentOrbit.Parent is not Astronomical parentBody)
            throw new InvalidOperationException($"orbit parent of '{Id}' is not an Astronomical");
        return EvaluateGameStateAgainst(parentBody, time, currentOrbit.StateVectors.TrueAnomaly);
    }

    /// <summary>Like <see cref="EvaluateGameState"/> but against an explicit parent —
    /// the rails-geometric re-parent's staging read, where the vessel's orbit still
    /// names the OLD parent. TrueAnomaly is caller-supplied (cosmetic; the re-parent's
    /// SetCurrentOrbit re-derives it immediately).</summary>
    public StateVectors EvaluateGameStateAgainst(Astronomical parentBody, SimTime time, TrueAnomaly trueAnomaly)
        => EvaluateCore(Predictor, parentBody, time, trueAnomaly);

    /// <summary>Like <see cref="EvaluateGameState"/> but reading from an arbitrary
    /// predictor — the overlay's display clone with plan burns applied (the
    /// authoritative <see cref="Predictor"/> must never see plan burns).</summary>
    public StateVectors EvaluateGameStateFrom(TrajectoryPredictor predictor, Orbit currentOrbit, SimTime time)
    {
        if (currentOrbit.Parent is not Astronomical parentBody)
            throw new InvalidOperationException($"orbit parent of '{Id}' is not an Astronomical");
        return EvaluateCore(predictor, parentBody, time, currentOrbit.StateVectors.TrueAnomaly);
    }

    /// <summary>THE predictor-absolute -> game parent-relative Cci staging read, shared
    /// by every Evaluate* flavor above so the conversion cannot drift between them.</summary>
    private StateVectors EvaluateCore(TrajectoryPredictor predictor, Astronomical parentBody,
        SimTime time, TrueAnomaly trueAnomaly)
    {
        if (parentBody is not IParentBody parent)
            throw new InvalidOperationException($"'{parentBody.Id}' is not an IParentBody");
        double t = time.Seconds();
        StateVector absolute;
        lock (Rails.Gate) absolute = predictor.StateAt(t);
        var parentAbs = Rails.GetAbsolute(parentBody.Id, t);
        var cce2Cci = parent.GetCce2Cci();
        return new StateVectors(
            time,
            FrameAdapter.AbsoluteToGame(absolute.Position, parentAbs.Position, cce2Cci),
            FrameAdapter.AbsoluteToGame(absolute.Velocity, parentAbs.Velocity, cce2Cci),
            trueAnomaly);
    }

    /// <summary>Display clone pinned to an authority lineage captured by the registry.
    /// The lineage and pending checks share the rails gate with predictor replacement,
    /// so no consumer can silently switch trajectories while taking its seed.</summary>
    internal bool TryNewDisplayPredictor(
        TrajectoryPredictor expectedLineage,
        double tSeconds,
        out TrajectoryPredictor display,
        out StateVector seed)
    {
        lock (Rails.Gate)
        {
            if (ReseedPending || !ReferenceEquals(Predictor, expectedLineage))
            {
                display = null!;
                seed = default;
                return false;
            }
            seed = expectedLineage.StateAt(tSeconds);
        }
        display = new TrajectoryPredictor(Rails.VesselGravity, seed, tSeconds,
            new IntegratorOptions { RelTol = 1e-9 });
        return true;
    }

    /// <summary>Captures one authoritative overlay anchor pinned to both predictor
    /// identity and the caller's pre-state-read display lineage.</summary>
    internal bool TryCaptureOverlayAnchor(
        long expectedOverlayLineage,
        double tSeconds,
        out TrajectoryPredictor authorityLineage,
        out StateVector anchor)
    {
        lock (Rails.Gate)
        {
            authorityLineage = Predictor;
            if (ReseedPending
                || OverlayLineage != expectedOverlayLineage
                || authorityLineage is null)
            {
                anchor = default;
                return false;
            }
            anchor = authorityLineage.StateAt(tSeconds);
            return !ReseedPending
                && OverlayLineage == expectedOverlayLineage
                && ReferenceEquals(Predictor, authorityLineage);
        }
    }

    /// <summary>Lock-free for live-seed jobs; predictor-pinned jobs take the rails gate
    /// to pair identity and pending-state validation with reseed publication.</summary>
    internal bool IsOverlayLineageCurrent(
        long expectedOverlayLineage,
        TrajectoryPredictor? expectedAuthorityLineage = null)
    {
        if (OverlayLineage != expectedOverlayLineage) return false;
        if (expectedAuthorityLineage is null) return true;
        lock (Rails.Gate)
            return !ReseedPending
                && OverlayLineage == expectedOverlayLineage
                && ReferenceEquals(Predictor, expectedAuthorityLineage);
    }

    /// <summary>Reuse cache for the ACTUAL (no-burn) display predictor — a fresh
    /// clone per rebuild would re-integrate the WHOLE display
    /// horizon every second (the dominant rebuild cost, dwarfing the sweep).
    /// Written/read only under the rails Gate; only the overlay worker calls the
    /// accessor below, so it also has one logical writer.</summary>
    private TrajectoryPredictor? _actualDisplay;
    private GravityModel? _actualDisplayGravity;

    /// <summary>Match tolerances for reusing the cached actual display predictor —
    /// a BACKSTOP behind the explicit invalidation in Reseed/ReseedAbsolute (the
    /// cache is nulled with the seed lineage, so even a sub-tolerance SAS-nudge
    /// reseed forces a fresh clone). 1 m / 0.1 mm/s is far above the
    /// display-vs-authoritative tolerance drift over a rebuild period (both
    /// integrate the same field; the clone at RelTol 1e-9) and far below any real
    /// uncaught state change.</summary>
    private const double ActualReusePositionMeters = 1.0;
    private const double ActualReuseVelocity = 1e-4;

    /// <summary>The ACTUAL display predictor, reused incrementally across rebuilds:
    /// same coast → prune behind the new start and extend forward (cheap); any
    /// disagreement with the authoritative predictor at <paramref name="tSeconds"/>
    /// → fresh clone (the from-scratch fallback). <paramref name="continuous"/>
    /// reports which happened — true means the returned predictor IS the previous
    /// rebuild's trajectory (same coast lineage), the overlay's licence to reuse the
    /// previous sampled batch verbatim. NEVER fold burns into
    /// the returned predictor — reuse is only sound because the actual batch is
    /// impulse-free by definition (the planned fold uses fresh clones).</summary>
    internal TrajectoryPredictor ActualDisplayPredictorAt(StateVector seed, double tSeconds,
        GravityModel gravity, long expectedOverlayLineage,
        TrajectoryPredictor expectedAuthorityLineage, out bool continuous)
    {
        lock (Rails.Gate)
        {
            if (ReseedPending
                || OverlayLineage != expectedOverlayLineage
                || !ReferenceEquals(Predictor, expectedAuthorityLineage))
                throw new OperationCanceledException();
            var cached = _actualDisplay;
            if (cached is not null && ReferenceEquals(_actualDisplayGravity, gravity)
                && cached.StartTime <= tSeconds && cached.Horizon >= tSeconds)
            {
                var state = cached.StateAt(tSeconds);
                if ((state.Position - seed.Position).Length() <= ActualReusePositionMeters
                    && (state.Velocity - seed.Velocity).Length() <= ActualReuseVelocity)
                {
                    cached.PruneBefore(tSeconds);
                    continuous = true;
                    return cached;
                }
            }
            var fresh = new TrajectoryPredictor(gravity, seed, tSeconds,
                new IntegratorOptions { RelTol = 1e-9 });
            _actualDisplay = fresh;
            _actualDisplayGravity = gravity;
            continuous = false;
            return fresh;
        }
    }

    /// <summary>Reads only the captured authoritative lineage. Pending ownership loss
    /// or a predictor replacement rejects the read under the same rails Gate used by
    /// reseed, so UI computations never silently switch to a newer trajectory.</summary>
    internal bool TryPredictorStateAt(
        TrajectoryPredictor expectedLineage, double tSeconds, out StateVector state)
    {
        lock (Rails.Gate)
        {
            if (ReseedPending || !ReferenceEquals(Predictor, expectedLineage))
            {
                state = default;
                return false;
            }
            state = expectedLineage.StateAt(tSeconds);
            return true;
        }
    }

    /// <summary>Solver-launch seed read pinned to an already captured authority
    /// lineage. Unlike ordinary point reads this never grows the authoritative
    /// predictor: a lagging coast is a retry condition, not work for the UI thread
    /// while it owns the global rails gate.</summary>
    internal bool TryCaptureSolverSeed(
        TrajectoryPredictor expectedLineage, double tSeconds, out StateVector state)
    {
        lock (Rails.Gate)
        {
            if (!SolverTimeCovered(expectedLineage, tSeconds)
                || ReseedPending
                || !ReferenceEquals(Predictor, expectedLineage))
            {
                state = default;
                return false;
            }

            var observed = expectedLineage.StateAt(tSeconds);
            // Predictor replacement and pending publication use this same gate. Keep
            // the explicit post-read check so future seed-read work cannot weaken the
            // lineage-pinned launch contract by introducing a callback or yield.
            if (ReseedPending || !ReferenceEquals(Predictor, expectedLineage))
            {
                state = default;
                return false;
            }
            state = observed;
            return true;
        }
    }

    /// <summary>Captures both rendezvous coast lineages and their now-states as one
    /// shared-rails transaction. Neither predictor may be extended here. A pending
    /// publication or replacement either wins before this gate (and rejects the
    /// launch) or waits until the complete pair has been captured.</summary>
    internal bool TryCaptureRendezvousSolverSeeds(
        TrackedVessel target,
        double tSeconds,
        out TrajectoryPredictor chaserLineage,
        out StateVector chaserSeed,
        out TrajectoryPredictor targetLineage,
        out StateVector targetSeed) =>
        TryCaptureRendezvousSolverSeedsCore(
            target, tSeconds,
            out chaserLineage, out chaserSeed,
            out targetLineage, out targetSeed,
            betweenSeedReads: null);

    /// <summary>Deterministic synchronization probe for the transaction above. The
    /// callback runs after the chaser read but before the target read while the same
    /// rails gate remains held.</summary>
    internal bool TryCaptureRendezvousSolverSeedsForTest(
        TrackedVessel target,
        double tSeconds,
        out TrajectoryPredictor chaserLineage,
        out StateVector chaserSeed,
        out TrajectoryPredictor targetLineage,
        out StateVector targetSeed,
        Action betweenSeedReads) =>
        TryCaptureRendezvousSolverSeedsCore(
            target, tSeconds,
            out chaserLineage, out chaserSeed,
            out targetLineage, out targetSeed,
            betweenSeedReads);

    private bool TryCaptureRendezvousSolverSeedsCore(
        TrackedVessel target,
        double tSeconds,
        out TrajectoryPredictor chaserLineage,
        out StateVector chaserSeed,
        out TrajectoryPredictor targetLineage,
        out StateVector targetSeed,
        Action? betweenSeedReads)
    {
        chaserLineage = null!;
        targetLineage = null!;
        chaserSeed = default;
        targetSeed = default;

        if (ReferenceEquals(this, target)
            || !ReferenceEquals(Rails, target.Rails))
            return false;

        lock (Rails.Gate)
        {
            TrajectoryPredictor? observedChaser = Predictor;
            TrajectoryPredictor? observedTarget = target.Predictor;
            if (observedChaser is null
                || observedTarget is null
                || ReseedPending
                || target.ReseedPending
                || !SolverTimeCovered(observedChaser, tSeconds)
                || !SolverTimeCovered(observedTarget, tSeconds))
                return false;

            var observedChaserSeed = observedChaser.StateAt(tSeconds);
            betweenSeedReads?.Invoke();
            var observedTargetSeed = observedTarget.StateAt(tSeconds);

            // Redundant under today's gate-only publications, deliberately explicit:
            // no successful return may mix a seed with a different or pending lineage.
            if (ReseedPending
                || target.ReseedPending
                || !ReferenceEquals(Predictor, observedChaser)
                || !ReferenceEquals(target.Predictor, observedTarget))
                return false;

            chaserLineage = observedChaser;
            chaserSeed = observedChaserSeed;
            targetLineage = observedTarget;
            targetSeed = observedTargetSeed;
            return true;
        }
    }

    private static bool SolverTimeCovered(
        TrajectoryPredictor predictor, double tSeconds) =>
        double.IsFinite(tSeconds)
        && predictor.StartTime <= tSeconds
        && predictor.Horizon >= tSeconds;

    /// <summary>Display predictor seeded from a LIVE game state (burn-time live
    /// display: off-rails ticks — burns, physics bubbles) instead of the
    /// authoritative predictor, which is KNOWN stale mid-burn (it predates the
    /// episode and stays that way until the burn-end reseed). Same parent-relative
    /// Cci -> mod-absolute conversion as <see cref="Reseed"/>; nothing on this entry
    /// is mutated. The coast-from-here prediction is exactly the cut-the-burn
    /// feedback: no thrust model, just where the vessel goes if the engine stops
    /// now.</summary>
    public TrajectoryPredictor NewLiveDisplayPredictor(Orbit currentOrbit, in StateVectors sv,
        out StateVector seed)
    {
        seed = AbsoluteFromGame(currentOrbit, in sv, out double t, out _);
        return new TrajectoryPredictor(Rails.VesselGravity, seed, t,
            new IntegratorOptions { RelTol = 1e-9 });
    }

    internal TrajectoryPredictor NewLiveDisplayPredictor(Orbit currentOrbit, in StateVectors sv,
        GravityModel gravity, out StateVector seed)
    {
        seed = AbsoluteFromGame(currentOrbit, in sv, out double t, out _);
        return new TrajectoryPredictor(gravity, seed, t,
            new IntegratorOptions { RelTol = 1e-9 });
    }

    /// <summary>Display predictor from an arbitrary mod-absolute state: the diverged
    /// ghost's seed (the plan's own world from its anchor) and the off-rails planned
    /// fold's seed (the live coast state — the authoritative predictor is KNOWN stale
    /// there, and a plan created after a burn must fold on reality, not on the
    /// pre-burn trajectory).</summary>
    public TrajectoryPredictor NewDisplayPredictorAt(StateVector absolute, double epochSeconds)
        => new(Rails.VesselGravity, absolute, epochSeconds,
            new IntegratorOptions { RelTol = 1e-9 });

    internal static TrajectoryPredictor NewDisplayPredictorAt(StateVector absolute,
        double epochSeconds, GravityModel gravity) =>
        new(gravity, absolute, epochSeconds, new IntegratorOptions { RelTol = 1e-9 });

    /// <summary>The Reseed conversion exposed for readers of the COMMITTED game state
    /// (panel Rebase while the vessel lingers off-rails: SAS/flight-computer wakeups
    /// keep a vessel in live physics long after thrust stops, and the committed
    /// Orbit.StateVectors are the honest trajectory there).</summary>
    public StateVector AbsoluteFromGameState(Orbit currentOrbit, in StateVectors sv)
        => AbsoluteFromGame(currentOrbit, in sv, out _, out _);

    /// <summary>Drops predictor nodes strictly before <paramref name="time"/> (the canary
    /// and the re-osculation refresh only ever query recent states; without pruning a
    /// high-warp session grows the node list without bound).</summary>
    internal void PruneBehind(double time)
    {
        lock (Rails.Gate) Predictor.PruneBefore(time);
    }
}

namespace WhiskerDynamics.Mod.Vessels;

/// <summary>Seam 1's KSA-free decision kernel (offline-testable; the registry translates
/// game state to these inputs). The patches run AFTER the stock freefall branch staged
/// <c>Situation.Freefall</c> into the new props, so "did live physics move this vessel?"
/// can only be answered from the PRE-TICK (last committed) situation — burns, collisions
/// and docking all commit non-Freefall situations on their way through.</summary>
public static class VesselLifecycle
{
    public enum Seeding
    {
        /// <summary>First sighting: create and seed a predictor from the staged state.</summary>
        Seed,
        /// <summary>Predictor is stale after live physics: reseed from the staged state.</summary>
        Reseed,
        /// <summary>Uninterrupted freefall: the predictor keeps flying.</summary>
        Keep,
    }

    public static Seeding Decide(bool alreadyTracked, bool wasFreefallAtTickStart, bool reseedPending,
        bool sameVehicleInstance)
    {
        if (!alreadyTracked) return Seeding.Seed;
        // Stale-Id guard: a NEW Vehicle instance under a tracked Id (vessel destroyed,
        // successor spawned in Freefall before eviction fires) must never inherit the
        // dead vessel's trajectory — pre-tick and staged situations are both Freefall
        // in that scenario and the commit canary is structurally blind to it (committed
        // == predictor by construction), so instance identity is the deciding signal.
        if (!sameVehicleInstance) return Seeding.Reseed;
        if (!wasFreefallAtTickStart || reseedPending) return Seeding.Reseed;
        return Seeding.Keep;
    }

    /// <summary>Registry hygiene: a tracked entry whose vessel has not been seen on the
    /// staging or commit surfaces for longer than the bound is gone from the sim
    /// (destroyed / task-removed) — drop it so dead vessels leave the panel and free
    /// their predictor node lists. Wall-clock, not sim-time: it measures "the game
    /// stopped talking about this vessel", which pause and warp must not distort.</summary>
    public static bool ShouldEvict(long nowMs, long lastSeenMs, long evictAfterMs) =>
        nowMs - lastSeenMs > evictAfterMs;

    /// <summary>Keep-path teleport guard: conic drift (predictor vs staged stock conic)
    /// grows smoothly tick over tick (large absolute values are LEGITIMATE at warp) and
    /// resets downward when stock rebuilds its plan from our committed states. An UPWARD
    /// jump beyond the bound within one tick means the staged stock state moved
    /// discontinuously under the same vehicle instance, so the caller must
    /// reseed.</summary>
    public static bool IsTeleportJump(double previousDrift, double drift, double jumpMeters) =>
        drift - previousDrift > jumpMeters;

    /// <summary>Adoption rule for a detected staged-state jump (see
    /// <see cref="IsTeleportJump"/>): a jump on the very tick the stock parent CHANGED
    /// is the stock patch-transition discontinuity caused by its Kepler-anchored
    /// next-patch state; keep the continuous predictor. Any other jump adopts the
    /// staged game state.</summary>
    public static bool AdoptStagedJump(bool parentTransitionTick) => !parentTransitionTick;

    /// <summary>Process-liveness guard for the eviction
    /// sweep: LastSeenMs stamps stop for EVERY entry while the process
    /// itself stalls (debugger break, OS suspend, long load hitch) — the wall clock
    /// keeps running, so the first post-stall sweep would otherwise mass-evict live
    /// vessels. A gap between sweeps longer than the eviction bound is the stall
    /// witness: the sweeper re-stamps everyone instead of evicting (a truly-dead entry
    /// then evicts one bound later — cosmetic lag, never data loss).</summary>
    public static bool SweepGapMeansStall(long nowMs, long lastSweepMs, long evictAfterMs) =>
        nowMs - lastSweepMs > evictAfterMs;

    /// <summary>Sidecar WRITE filter: only vessels whose predictor is CURRENT
    /// may persist. A landed / stock-owned / mid-burn vessel's predictor froze at its
    /// last rails stretch — persisting that ghost state would "restore" the vessel onto
    /// a stale trajectory on load. <paramref name="lastActiveTime"/> is
    /// max(LastRefreshTime, LastStagedTime): the refresh stamp covers the single-vessel
    /// path (at most one refresh period apart while actively staging), the staging
    /// stamp covers cluster followers, which never book re-osculation refreshes
    /// (without it, live followers like Gemini7/Hunter would drop from the
    /// sidecar). Both freeze while the vessel is off rails, so two refresh periods
    /// separate "actively on rails" from "frozen". A seed in the future of the save
    /// instant would make StateAt(elapsed) throw — excluded outright.</summary>
    public static bool SidecarEligible(bool reseedPending, bool hasPredictor,
        double seedTime, double lastActiveTime, double elapsedSeconds, double refreshPeriodSeconds) =>
        !reseedPending && hasPredictor
        && seedTime <= elapsedSeconds
        && elapsedSeconds - lastActiveTime <= 2 * refreshPeriodSeconds;

    /// <summary>Sidecar RESTORE guard, applied on a vessel's first GetOrSeed
    /// after a load right after its stock seed. A restored save's vessels stage within
    /// a tick of the sidecar epoch. The bounded window prevents a later vessel with
    /// a recycled id from inheriting the saved trajectory. The 1 ms forward allowance
    /// covers save-time rounding without permitting a pre-epoch predictor query.</summary>
    public static bool ShouldRestoreFromSidecar(double epochSeconds, double seedTime, double windowSeconds) =>
        epochSeconds - seedTime <= 1e-3 && seedTime - epochSeconds <= windowSeconds;

    /// <summary>Re-osculation trigger: rebuild the vessel's
    /// stock conic/FlightPlan from the n-body state when the predictor drifts beyond the
    /// bound, or periodically regardless (the periodic path re-arms the plan's
    /// ExpiryGameTime so a vessel left alone keeps a valid, non-expired plan).
    /// <paramref name="conicDrift"/> is the INSTANTANEOUS readout — a sawtooth, since
    /// stock plan rebuilds re-derive the conic from our committed states and reset the
    /// reference — i.e. "drift since the last re-derivation, stock's
    /// or ours"; the kernel keeps no memory of past peaks. Strict bounds: exactly-at
    /// readings on the refresh tick itself must not re-trigger forever, and a clock at
    /// or before the stamp never arms the periodic path.</summary>
    public static bool ShouldRefreshOsculation(double conicDrift, double driftThresholdMeters,
        double time, double lastRefreshTime, double refreshPeriodSeconds) =>
        conicDrift > driftThresholdMeters || time - lastRefreshTime > refreshPeriodSeconds;

    /// <summary>Why one observed commit callback can or cannot extend the current
    /// canary streak. Every non-comparable result is a continuity break: a committed
    /// state existed, but it did not belong to the same uninterrupted predictor
    /// lineage as the preceding verified sample.</summary>
    public enum CommitCanaryEligibility
    {
        Comparable,
        ReseedPending,
        NotFreefall,
        ReplacementVehicle,
        SeedOrReseedTick,
    }

    /// <summary>KSA-free adapter for <c>VesselRegistry.VerifyCommit</c>. Absence of a
    /// callback (pause/no task result) never reaches this seam and therefore suspends
    /// the streak. An observed but ineligible callback breaks it.</summary>
    public static CommitCanaryEligibility ClassifyCommitCanary(
        bool reseedPending, bool isFreefall, bool sameVehicleInstance,
        double committedTime, double seedTime)
    {
        if (reseedPending) return CommitCanaryEligibility.ReseedPending;
        if (!isFreefall) return CommitCanaryEligibility.NotFreefall;
        if (!sameVehicleInstance) return CommitCanaryEligibility.ReplacementVehicle;
        // Also rejects non-finite timestamps: neither NaN nor +/-infinity can prove a
        // comparable post-seed commit.
        if (!double.IsFinite(committedTime) || !double.IsFinite(seedTime)
            || committedTime <= seedTime)
            return CommitCanaryEligibility.SeedOrReseedTick;
        return CommitCanaryEligibility.Comparable;
    }
}

/// <summary>Allocation-free adapter implemented by a value type at the registry edge.
/// The generic policy calls it through a constrained dispatch, so game-specific
/// snapshot state stays outside this KSA-free file without boxing or delegates.</summary>
internal interface IVesselRegistryCanaryProbe
{
    VesselLifecycle.CommitCanaryEligibility CaptureAndClassify();
    double CommitTime { get; }
    double EvaluateResidual();
    double ToleranceMeters { get; }
}

/// <summary>Complete KSA-free policy seam used by <c>VesselRegistry.VerifyCommit</c>.
/// It captures the lineage token before invoking the probe and owns the entire
/// snapshot/classification/evaluation/completion exception boundary.</summary>
internal static class VesselRegistryCanary
{
    internal readonly record struct Verification(
        CanaryCounter.Completion Completion, double Time, double Residual, bool Miss,
        Exception? Failure = null);

    internal static Verification Verify<TProbe>(CanaryCounter canary, ref TProbe probe)
        where TProbe : struct, IVesselRegistryCanaryProbe
    {
        CanaryCounter.Observation observation = canary.BeginObservation();
        bool comparable = false;
        try
        {
            // A mutation already in progress broke continuity when it began. Do not
            // inspect partially published state or reset whatever follows it.
            if (!observation.Available)
                return new Verification(
                    canary.Complete(observation, miss: false),
                    double.NaN, double.NaN, Miss: false);

            var eligibility = probe.CaptureAndClassify();
            if (eligibility != VesselLifecycle.CommitCanaryEligibility.Comparable)
                return new Verification(
                    canary.DiscardAndBreakContinuityIfCurrent(observation),
                    double.NaN, double.NaN, Miss: false);
            comparable = true;

            double time = probe.CommitTime;
            double tolerance = probe.ToleranceMeters;
            bool invalidTolerance = !double.IsFinite(tolerance) || tolerance < 0.0;
            if (invalidTolerance)
                // Bad configuration makes this interval unverifiable; it is not
                // evidence that the committed trajectory missed the predictor.
                return new Verification(
                    canary.DiscardAndBreakContinuityIfCurrent(observation),
                    double.NaN, double.NaN, Miss: false);

            double residual = probe.EvaluateResidual();
            bool miss = !double.IsFinite(residual)
                || residual < 0.0
                || residual > tolerance;
            return new Verification(
                canary.Complete(observation, miss), time, residual, miss);
        }
        catch (Exception e)
        {
            CanaryCounter.Completion completion = comparable
                ? canary.CompleteProbeFailure(observation)
                : canary.DiscardAndBreakContinuityIfCurrent(observation);
            return new Verification(
                completion, double.NaN, double.NaN, Miss: false, Failure: e);
        }
    }
}

/// <summary>KSA-free synchronization seam for authority publication and one-shot
/// operations. Both sides use the tracked vessel's rails gate, so a transition cannot
/// land between the operation's final validation and its side effect.</summary>
internal static class RailsAuthoritySynchronization
{
    internal static void Publish<TState>(object gate, TState state, Action<TState> publication)
    {
        lock (gate) publication(state);
    }

    internal static bool TryExecute<TResult>(
        object gate,
        Func<bool> validate,
        Func<TResult> execute,
        out TResult result)
    {
        lock (gate)
        {
            if (!validate())
            {
                result = default!;
                return false;
            }
            result = execute();
            return true;
        }
    }
}

/// <summary>Commit-canary strike counter. Consecutive over-tolerance residuals and
/// consecutive comparable-probe failures have independent fatal thresholds. A
/// successfully evaluated comparable probe resets the failure streak, while any
/// continuity break resets both. Generation-tagged completion makes the counter safe
/// when a reseed/replacement races an already-running residual calculation.</summary>
public sealed class CanaryCounter(
    int fatalConsecutiveMisses = 3,
    int fatalConsecutiveProbeFailures = 3)
{
    private readonly object _gate = new();
    private long _generation;
    private int _strikes;
    private int _probeFailures;
    private int _transitionDepth;

    public int Strikes
    {
        get { lock (_gate) return _strikes; }
    }

    public int ProbeFailures
    {
        get { lock (_gate) return _probeFailures; }
    }

    internal int FatalConsecutiveProbeFailures => fatalConsecutiveProbeFailures;

    public readonly record struct Observation(long Generation, bool Available);

    public enum CompletionKind
    {
        /// <summary>The predictor lineage changed after this observation began.</summary>
        Discarded,
        /// <summary>The current-lineage result was recorded below its threshold.</summary>
        Recorded,
        /// <summary>A current-lineage miss or probe-failure streak reached its limit.</summary>
        Fatal,
    }

    public readonly record struct Completion(
        CompletionKind Kind, int Strikes, int ProbeFailures);

    private bool IsCurrent(Observation observation) =>
        observation.Available
        && _transitionDepth == 0
        && observation.Generation == _generation;

    /// <summary>Captures the current predictor-lineage generation immediately before
    /// residual evaluation.</summary>
    public Observation BeginObservation()
    {
        lock (_gate) return new Observation(_generation, _transitionDepth == 0);
    }

    /// <summary>Completes one comparable verification. A continuity break that landed
    /// since <see cref='BeginObservation'/> makes this result <see
    /// cref='CompletionKind.Discarded'/> and leaves the new lineage untouched.</summary>
    public Completion Complete(Observation observation, bool miss)
    {
        lock (_gate)
        {
            if (!IsCurrent(observation))
                return CurrentCompletion(CompletionKind.Discarded);
            _probeFailures = 0;
            if (!miss)
            {
                _strikes = 0;
                return CurrentCompletion(CompletionKind.Recorded);
            }
            _strikes++;
            return CurrentCompletion(
                _strikes >= fatalConsecutiveMisses
                    ? CompletionKind.Fatal
                    : CompletionKind.Recorded);
        }
    }

    /// <summary>Records an exception after a probe was classified comparable. A
    /// successfully evaluated comparable probe must intervene before another failure
    /// can start a fresh streak.</summary>
    public Completion CompleteProbeFailure(Observation observation)
    {
        lock (_gate)
        {
            if (!IsCurrent(observation))
                return CurrentCompletion(CompletionKind.Discarded);
            _strikes = 0;
            _probeFailures++;
            return CurrentCompletion(
                _probeFailures >= fatalConsecutiveProbeFailures
                    ? CompletionKind.Fatal
                    : CompletionKind.Recorded);
        }
    }

    /// <summary>Atomically discards an observed skip/failure and breaks continuity only
    /// when its token still belongs to the active lineage. A stale callback therefore
    /// cannot reset strikes recorded after a reseed or replacement.</summary>
    public Completion DiscardAndBreakContinuityIfCurrent(Observation observation)
    {
        lock (_gate)
        {
            if (IsCurrent(observation))
            {
                _strikes = 0;
                _probeFailures = 0;
                unchecked { _generation++; }
            }
            return CurrentCompletion(CompletionKind.Discarded);
        }
    }

    /// <summary>Begins a predictor/identity mutation. New observations are unavailable
    /// until <see cref='EndContinuityTransition'/>, while observations that began
    /// earlier become stale immediately.</summary>
    public void BeginContinuityTransition()
    {
        lock (_gate)
        {
            _transitionDepth++;
            _strikes = 0;
            _probeFailures = 0;
            unchecked { _generation++; }
        }
    }

    /// <summary>Publishes the completed predictor/identity lineage and invalidates any
    /// observation that was attempted while the mutation was in progress.</summary>
    public void EndContinuityTransition()
    {
        lock (_gate)
        {
            if (_transitionDepth <= 0)
                throw new InvalidOperationException("No canary continuity transition is active.");
            _strikes = 0;
            _probeFailures = 0;
            unchecked { _generation++; }
            _transitionDepth--;
        }
    }

    private Completion CurrentCompletion(CompletionKind kind) =>
        new(kind, _strikes, _probeFailures);

}

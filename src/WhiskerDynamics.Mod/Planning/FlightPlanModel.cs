using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning;

/// <summary>Per-burn authoring metadata for a burn defined in a CATALOG frame (VLF
/// burns carry no metadata — their stock components ARE the authored components).
/// Keyed to the stock burn by its <see cref="BurnIdentityPolicy"/> time identity;
/// the authored components are the user's intent along the frame's axes at the burn
/// time, and the stock DeltaVVlf is DERIVED from them (see BurnFrameKernel). KSA-free.</summary>
public sealed class FlightPlanBurnMeta
{
    public required double TimeSeconds { get; set; }
    /// <summary>Authoring frame (never null here; a VLF burn simply has no meta).</summary>
    public required FrameSpec Frame { get; init; }
    /// <summary>Authored delta-v components along the frame's axes at the burn time (m/s).</summary>
    public required Vector3d Authored { get; set; }
    /// <summary>Wall-clock stamp (creation/restore): a meta whose stock burn has not
    /// appeared yet (adds are QUEUED via InputEvents and land next frame) must survive
    /// the orphan prune for a grace window instead of dying before its burn exists.</summary>
    public required long StampMs { get; set; }
}

/// <summary>One captured burn of a plan snapshot: the stock burn's time key, VLF
/// delta-v, and burn-time patch parent defining that VLF basis. The parent is nullable
/// while its patch cannot be resolved. Immutable; KSA-free.</summary>
public sealed record PlanSnapshotBurn(
    double TimeSeconds, Vector3d DeltaVVlf, string? BasisParentId = null);

/// <summary>Immutable anchor and burn list for the planned display path. The stock
/// `BurnPlan` remains the execution source of truth. Whole-reference replacement
/// permits lock-free reads by the overlay worker.</summary>
public sealed class PlanSnapshot
{
    public required double EpochSeconds { get; init; }
    public required StateVector State { get; init; }
    /// <summary>Orbit parent used for the snapshot's VLF bases. Null falls back to
    /// the live parent.</summary>
    public required string? AnchorParentId { get; init; }
    /// <summary>Captured burns, ascending by time (the fold applies them in order).</summary>
    public required IReadOnlyList<PlanSnapshotBurn> Burns { get; init; }
    /// <summary>Burn times, same order as <see cref="Burns"/> — precomputed once
    /// (immutable), read every rebuild by the window/fold rules.</summary>
    public required double[] BurnTimes { get; init; }
    /// <summary>Engine/mass scalars at capture — the finite-burn estimate's inputs,
    /// frozen with the snapshot so the diverged ghost keeps predicting with the
    /// plan's world's mass. Null keeps the fold impulsive.</summary>
    public required EngineScalars? Engine { get; init; }
    /// <summary>Propulsion set represented by <see cref='Engine'/>. Missing or
    /// invalid persisted values select main engines.</summary>
    public required PropulsionSource PropulsionSource { get; init; }

    /// <summary>Whether two snapshots produce the same folded planned geometry while
    /// the plan is not diverged. Epoch/state are deliberately excluded: hourly
    /// re-anchoring advances the same coast lineage solely to bound a future ghost's
    /// integration depth. Burns and finite-burn inputs remain exact invalidators.</summary>
    public bool GeometryMatches(PlanSnapshot other)
    {
        if (Engine != other.Engine || PropulsionSource != other.PropulsionSource
            || Burns.Count != other.Burns.Count)
            return false;
        for (int i = 0; i < Burns.Count; i++)
            if (Burns[i] != other.Burns[i]) return false;
        return true;
    }
    public static PlanSnapshot Capture(double epochSeconds, StateVector state,
        string? anchorParentId, IEnumerable<PlanSnapshotBurn> burns,
        EngineScalars? engine = null,
        PropulsionSource propulsionSource = PropulsionSource.MainEngines)
    {
        var sorted = burns.OrderBy(b => b.TimeSeconds).ToArray();
        var times = new double[sorted.Length];
        for (int i = 0; i < sorted.Length; i++) times[i] = sorted[i].TimeSeconds;
        return new()
        {
            EpochSeconds = epochSeconds,
            State = state,
            AnchorParentId = anchorParentId,
            Burns = sorted,
            BurnTimes = times,
            Engine = engine is { Usable: true } ? engine : null,
            PropulsionSource = propulsionSource,
        };
    }
}

/// <summary>One atomic view of the snapshot state that must stay coherent through a
/// sidecar export or snapshot replacement. The pending keys are copied while the same
/// gate protects the immutable snapshot reference and divergence flag.</summary>
internal sealed record PlanSnapshotPersistenceState(
    PlanSnapshot? Snapshot,
    bool Diverged,
    IReadOnlyList<double> PendingParentRefreshTimes)
{
    internal bool ParentRefreshPendingAt(double timeSeconds) =>
        BurnIdentityPolicy.ContainsBurn(PendingParentRefreshTimes, timeSeconds);

    internal string? KnownParentAt(double timeSeconds) =>
        Snapshot?.Burns.FirstOrDefault(
            b => BurnIdentityPolicy.SameBurn(b.TimeSeconds, timeSeconds))?.BasisParentId;

    /// <summary>An already-pending key owns its last known parent until a later clean
    /// scan, even if a resolver returns a non-null value during replacement.</summary>
    internal string? ParentForReplacement(double timeSeconds,
        string? resolvedParentId) =>
        ParentRefreshPendingAt(timeSeconds)
            ? KnownParentAt(timeSeconds)
            : resolvedParentId ?? KnownParentAt(timeSeconds);
}

/// <summary>Mod-level flight-plan metadata that rides alongside the stock BurnPlan,
/// which remains the sole execution source of truth for burn times and VLF delta-v.
/// This model stores its creation time, a plan LENGTH that ends the planned trajectory
/// and can extend the actual vessel prediction beyond its configured minimum,
/// per-burn authoring-frame metadata, and the display
/// SNAPSHOT the planned line folds from, with its diverged flag. KSA-free and
/// offline-tested; persisted through the save sidecar (SidecarPlan DTOs).</summary>
public sealed class FlightPlanModel
{
    /// <summary>Orphan-prune grace, ms: a meta without a matching stock burn older than
    /// this was orphaned for real (burn deleted/dragged via the STOCK editor — the mod
    /// cannot see intent there, so the burn honestly degrades to a plain VLF burn).</summary>
    public const long MetaGraceMs = 10_000;

    /// <summary>Smallest useful coast after the final burn. A burn exactly at the
    /// plan end has no post-burn trajectory to display, so plan authoring keeps one
    /// sampler-sized second after every node.</summary>
    public const double MinimumPostBurnSeconds = 1.0;

    public double CreatedAtSeconds { get; init; }

    private double _lengthSeconds;
    public double LengthSeconds
    {
        get => _lengthSeconds;
        set { _lengthSeconds = value; BumpVersion(); }
    }

    public double EndSeconds => CreatedAtSeconds + LengthSeconds;

    private PropulsionSource _propulsionSource;
    /// <summary>Engine set used by finite display/solver estimates for this plan.
    /// Main engines are the default. RCS selection models manual
    /// forward translation; stock auto-burn remains main-engine-only.</summary>
    public PropulsionSource PropulsionSource
    {
        get => _propulsionSource;
        set
        {
            if (_propulsionSource == value) return;
            _propulsionSource = value;
            BumpVersion();
        }
    }

    /// <summary>Applies a user propulsion switch to both the live plan setting and
    /// its frozen display snapshot. The snapshot keeps its own plan-world mass while
    /// adopting the selected set's current ve/flow; this is a planner edit, so it also
    /// updates an already-diverged ghost instead of waiting for Rebase.</summary>
    public void SetPropulsionSource(PropulsionSource source, EngineScalars engine)
    {
        bool changed;
        lock (_snapshotGate)
        {
            changed = _propulsionSource != source;
            _propulsionSource = source;
            if (_snapshot is { } snapshot)
            {
                double frozenMass = snapshot.Engine?.MassKg ?? engine.MassKg;
                EngineScalars selected = engine with { MassKg = frozenMass };
                EngineScalars? normalized = selected.Usable ? selected : null;
                changed |= snapshot.PropulsionSource != source || snapshot.Engine != normalized;
                if (changed)
                {
                    System.Threading.Volatile.Write(ref _snapshot,
                        PlanSnapshot.Capture(snapshot.EpochSeconds, snapshot.State,
                            snapshot.AnchorParentId,
                            snapshot.Burns,
                            normalized, source));
                }
            }
        }
        if (changed) BumpVersion();
    }

    // ---- Plan version: bumped by every edit that changes what the planned line
    // should draw (snapshot replacement, divergence, plan length). The overlay's
    // plan-edit throttle bypass compares it against the version its last rebuild
    // consumed — the counter must never tick for reads or no-op writes, or the
    // bypass degenerates into an every-tick rebuild.

    private long _version = 1;

    /// <summary>Monotonic edit counter (lock-free reads from the overlay's staging
    /// call). Starts above <see cref="TrackedVessel"/>'s zero "no plan seen" stamp so
    /// a fresh plan always registers as a change.</summary>
    public long Version => System.Threading.Interlocked.Read(ref _version);

    private void BumpVersion()
    {
        System.Threading.Interlocked.Increment(ref _version);
        FlightPlans.NoteEdit(); // the store-wide stamp: per-tick staging reads IT, not us
    }

    // ---- The plan clock ("plan+"): burn times display and edit relative to when
    // the plan BEGAN (its creation time). The model owns the conversion so every
    // surface that prints or parses a plan time speaks the same clock — hand-copied
    // `t - CreatedAtSeconds` at panel sites is how two clocks drift apart.

    /// <summary>Seconds since the plan began for an absolute sim time.</summary>
    public double PlanRelative(double absoluteSeconds) => absoluteSeconds - CreatedAtSeconds;

    /// <summary>Absolute sim time for a plan-clock ("plan+") value.</summary>
    public double AbsoluteOf(double planRelativeSeconds) => CreatedAtSeconds + planRelativeSeconds;

    private readonly List<FlightPlanBurnMeta> _meta = [];
    public IReadOnlyList<FlightPlanBurnMeta> Meta => _meta;

    public FlightPlanBurnMeta? TryGetMetaAt(double timeSeconds)
    {
        FlightPlanBurnMeta? best = null;
        double bestDelta = BurnIdentityPolicy.ToleranceSeconds;
        foreach (var meta in _meta)
        {
            if (BurnIdentityPolicy.TryMatch(meta.TimeSeconds, timeSeconds, out double delta)
                && delta <= bestDelta)
            {
                best = meta;
                bestDelta = delta;
            }
        }
        return best;
    }

    /// <summary>Adds or replaces the meta at its time slot (one meta per burn).</summary>
    public void SetMeta(FlightPlanBurnMeta meta)
    {
        _meta.RemoveAll(m => BurnIdentityPolicy.SameBurn(m.TimeSeconds, meta.TimeSeconds));
        _meta.Add(meta);
    }

    public void RemoveMetaAt(double timeSeconds) =>
        _meta.RemoveAll(m => BurnIdentityPolicy.SameBurn(m.TimeSeconds, timeSeconds));

    /// <summary>Re-keys the meta after a time edit through OUR panel (the natural key is
    /// the burn time, so a time edit must move the key with the burn).</summary>
    public void MoveMeta(double oldTimeSeconds, double newTimeSeconds, long nowMs)
    {
        if (TryGetMetaAt(oldTimeSeconds) is not { } meta) return;
        // Never two metas on one slot — but the meta being MOVED must survive its own
        // eviction sweep: a move within one burn-identity slot (retyping essentially the
        // same time) would otherwise match and delete it, silently losing the authored
        // frame intent. Exclusion by reference, so a within-tolerance
        // move is a no-op-with-update: mutate the key, keep the entry.
        _meta.RemoveAll(m => !ReferenceEquals(m, meta)
            && BurnIdentityPolicy.SameBurn(m.TimeSeconds, newTimeSeconds));
        meta.TimeSeconds = newTimeSeconds;
        meta.StampMs = nowMs; // fresh grace: the edited burn re-lands like an add
    }

    /// <summary>Drops metas whose stock burn is gone (deleted or time-dragged via the
    /// stock editor). Metas younger than <see cref="MetaGraceMs"/> are kept unmatched:
    /// panel adds are QUEUED (InputEvents) and the burn only exists next frame.</summary>
    public void PruneOrphanedMeta(IReadOnlyList<double> stockTimes, long nowMs)
    {
        _meta.RemoveAll(m =>
        {
            if (nowMs - m.StampMs < MetaGraceMs) return false;
            foreach (double t in stockTimes)
                if (BurnIdentityPolicy.SameBurn(t, m.TimeSeconds)) return false;
            return true;
        });
    }

    /// <summary>Null when the burn time sits inside BOTH the plan window and the
    /// rails-ahead window; otherwise a panel-ready rejection. Manoeuvres must fall within
    /// the plan window; extend the plan length to plan further
    /// out. Rails window (the plan/authoring rules OWN this bound): a
    /// burn past now + railsAheadDays would make the frame-burn converter demand
    /// synchronous Gate-held ephemerides integration the rails worker doesn't maintain —
    /// the exact multi-second freeze FlightPlans.EffectiveHorizonDays clamps away for
    /// the overlay — so it is refused with the remedy the frames panel offers (a longer
    /// orbit window raises the rails horizon through the EditPrediction coupling).</summary>
    public string? RejectOutsideWindow(double burnTimeSeconds, double nowSeconds,
        double railsAheadDays)
    {
        if (burnTimeSeconds > EndSeconds - MinimumPostBurnSeconds)
            // Plan-clock phrasing: the panel's time fields read/edit in "plan+"
            // seconds, so an absolute-t rejection would be unactionable there.
            return $"rejected: burn must leave {MinimumPostBurnSeconds:F0} s before the plan end "
                + $"(plan+{LengthSeconds:F0} s) - extend the plan length";
        if (burnTimeSeconds > nowSeconds + railsAheadDays * 86400.0)
            return $"rejected: burn is past the rails horizon (T+{railsAheadDays:F1} d)"
                + " - pick a longer orbits window in N-Body Frames"
                + " (a just-raised window keeps growing in the background - retry shortly)";
        return null;
    }

    /// <summary>Null when the new plan length is acceptable; otherwise a panel-ready
    /// rejection. A plan may not shrink below its last planned burn, and may never
    /// exceed <see cref="SettingsKernel.MaxRailsDays"/>: the rails worker can never
    /// integrate further than that ceiling, so a longer plan would promise burn times
    /// no rails horizon setting can ever deliver.</summary>
    public string? ValidateLength(double newLengthSeconds, IReadOnlyList<double> burnTimes)
    {
        if (!double.IsFinite(newLengthSeconds) || newLengthSeconds <= 0)
            return "rejected: plan length must be a positive number";
        if (newLengthSeconds > SettingsKernel.MaxRailsDays * 86400.0)
            return $"rejected: plan length is capped at {SettingsKernel.MaxRailsDays:F0} d"
                + " (rails can never integrate further)";
        foreach (double t in burnTimes)
            if (t > CreatedAtSeconds + newLengthSeconds - MinimumPostBurnSeconds)
                return $"rejected: plan must end at least {MinimumPostBurnSeconds:F0} s after its last burn";
        return null;
    }

    // ---- Plan snapshot. Written from the UI thread (panel edits, Rebase)
    // AND the job thread (reconcile/lazy capture in the on-rails rebuild; MarkDiverged
    // from the registry seams) — all mutation goes through the gate below and replaces
    // whole immutable instances, so the overlay's lock-free reference reads never tear.

    /// <summary>VLF component tolerance (m/s) for <see cref="SnapshotBurnsMatch"/>:
    /// both sides of the comparison are the same doubles written through
    /// BurnPlanWriter, so matching is normally bit-exact; the epsilon only absorbs a
    /// lossy future serialization of the stock save.</summary>
    public const double DvMatchTolerance = 1e-9;

    internal const int SnapshotParentEvidenceLimit = 8;

    /// <summary>Stable parent-pattern identity for the evidence throttle. Times and
    /// delta-v deliberately do not participate: stock-editor drags may change them
    /// every frame, while the evidence-worthy event is a change in the ordered patch
    /// parents. Null means the snapshot is not multi-parent.</summary>
    internal static string? SnapshotParentSignature(IReadOnlyList<PlanSnapshotBurn> burns)
    {
        var parents = burns.Select(b => b.BasisParentId ?? "<unresolved>").ToArray();
        if (parents.Distinct(StringComparer.Ordinal).Take(2).Count() < 2) return null;
        return string.Join('\u001f', parents);
    }

    internal static bool SnapshotParentEvidenceDue(long nowMs, long lastLogMs,
        string? signature, string? lastSignature, bool samePlan = true) =>
        (signature is not null
            && (!samePlan
                || !string.Equals(signature, lastSignature, StringComparison.Ordinal)))
        || nowMs - lastLogMs >= 1000;

    /// <summary>Bounded, searchable INFO suffix for a successful capture/rebase whose
    /// burns span more than one VLF basis parent. Single-parent captures stay quiet;
    /// the caller already logs their count.</summary>
    internal static string SnapshotParentEvidence(IReadOnlyList<PlanSnapshotBurn> burns,
        int maximumBurns = SnapshotParentEvidenceLimit)
    {
        if (SnapshotParentSignature(burns) is null) return string.Empty;
        int count = Math.Clamp(maximumBurns, 1, SnapshotParentEvidenceLimit);
        string entries = string.Join(", ", burns.Take(count)
            .Select(b => $"{b.TimeSeconds:F1}:{b.BasisParentId ?? "<unresolved>"}"));
        string omitted = burns.Count > count ? $", +{burns.Count - count} more" : string.Empty;
        return $", multi-parent burns=[{entries}{omitted}]";
    }

    private readonly object _snapshotGate = new();
    private PlanSnapshot? _snapshot;
    private bool _diverged;
    /// <summary>Time keys moved before stock has rebuilt their burn patch. Protected
    /// by <see cref="_snapshotGate"/> with the immutable snapshot they belong to.</summary>
    private readonly List<double> _pendingSnapshotParentRefresh = [];

    /// <summary>The display anchor and captured burns; null until the first valid
    /// on-rails capture.</summary>
    public PlanSnapshot? Snapshot => System.Threading.Volatile.Read(ref _snapshot);

    /// <summary>True once reality left the snapshot's world (burn flown, live-physics
    /// episode, teleport): the planned line freezes into the full ghost and the panel
    /// offers Rebase. Cleared only by a (re)capture.</summary>
    public bool Diverged { get { lock (_snapshotGate) return _diverged; } }

    /// <summary>The (snapshot, diverged) pair read under ONE gate acquisition — the
    /// overlay worker's fold decisions must never see a torn pair during rebase or
    /// divergence.</summary>
    public (PlanSnapshot? Snapshot, bool Diverged) SnapshotState
    {
        get { lock (_snapshotGate) return (_snapshot, _diverged); }
    }

    /// <summary>Atomic sidecar/Rebase view. Callers must use only this captured state
    /// for snapshot, divergence, and pending-parent decisions in one operation.</summary>
    internal PlanSnapshotPersistenceState CaptureSnapshotPersistenceState()
    {
        lock (_snapshotGate)
            return new PlanSnapshotPersistenceState(_snapshot, _diverged,
                _pendingSnapshotParentRefresh.ToArray());
    }

    /// <summary>Capture/rebase: install a new snapshot and reset the diverged flag.</summary>
    public void SetSnapshot(PlanSnapshot? snapshot, bool diverged = false)
    {
        lock (_snapshotGate)
        {
            System.Threading.Volatile.Write(ref _snapshot, snapshot);
            _diverged = diverged;
            _pendingSnapshotParentRefresh.Clear();
        }
        BumpVersion();
    }

    /// <summary>The overlay RECONCILE's compare-and-set (worker thread): commit the
    /// fresh capture only if the snapshot is still the instance the reconcile
    /// decided against AND nothing marked the plan diverged meanwhile — a Rebase or
    /// panel edit racing the worker must win (the capture was decided on stale
    /// inputs), and a MarkDiverged landing mid-reconcile must not be wiped by the
    /// capture's diverged reset. With no current snapshot, the initial valid capture
    /// proceeds even if divergence was already marked.</summary>
    public bool TryReconcileSnapshot(PlanSnapshot? expectedCurrent, PlanSnapshot fresh,
        PropulsionSource? expectedPropulsion = null)
    {
        lock (_snapshotGate)
        {
            if (!ReferenceEquals(_snapshot, expectedCurrent)) return false;
            if (expectedPropulsion is { } source && _propulsionSource != source) return false;
            if (_diverged && expectedCurrent is not null) return false;
            System.Threading.Volatile.Write(ref _snapshot, fresh);
            _diverged = false;
        }
        BumpVersion();
        return true;
    }

    public void MarkDiverged()
    {
        bool flipped;
        lock (_snapshotGate)
        {
            flipped = !_diverged;
            _diverged = true;
        }
        // Flip-only bump: the off-rails path re-marks divergence at every witnessed
        // delta-v tick, and a per-tick version churn would defeat the bypass throttle.
        if (flipped) BumpVersion();
    }

    /// <summary>True when the captured burn list still equals the stock plan's (same
    /// count; each record matches by <see cref="BurnIdentityPolicy"/>, by
    /// VLF components within <see cref="DvMatchTolerance"/>, and by exact basis-parent
    /// identity) — the not-diverged
    /// reconcile check. False with no snapshot at all (capture is due).</summary>
    public bool SnapshotBurnsMatch(IReadOnlyList<PlanSnapshotBurn> stockBurns)
    {
        if (Snapshot is not { } snapshot) return false;
        if (snapshot.Burns.Count != stockBurns.Count) return false;
        var sorted = stockBurns.OrderBy(b => b.TimeSeconds).ToArray();
        for (int i = 0; i < sorted.Length; i++)
        {
            var captured = snapshot.Burns[i];
            if (BurnIdentityPolicy.DifferentBurn(
                    captured.TimeSeconds, sorted[i].TimeSeconds)) return false;
            var delta = captured.DeltaVVlf - sorted[i].DeltaVVlf;
            if (Math.Abs(delta.X) > DvMatchTolerance || Math.Abs(delta.Y) > DvMatchTolerance
                || Math.Abs(delta.Z) > DvMatchTolerance) return false;
            if (!string.Equals(captured.BasisParentId, sorted[i].BasisParentId,
                    StringComparison.Ordinal)) return false;
        }
        return true;
    }

    /// <summary>Writer-mirror (BurnPlanWriter calls these on every successful stock
    /// mutation — ONE seam, so no edit affordance can forget the mirror): add or
    /// replace the captured burn at its time slot. No-op before the first capture
    /// (the on-rails reconcile captures wholesale).</summary>
    public void SnapshotSetBurn(double timeSeconds, Vector3d dvVlf,
        string? basisParentId = null,
        bool markDownstreamParentsPending = false)
    {
        lock (_snapshotGate)
        {
            if (_snapshot is not { } snapshot) return;
            var matched = snapshot.Burns.FirstOrDefault(
                b => BurnIdentityPolicy.SameBurn(b.TimeSeconds, timeSeconds));
            bool parentPending = matched is not null
                && ParentRefreshPendingAt(matched.TimeSeconds);
            if (matched is null) RemoveParentRefreshAt(timeSeconds);
            string? parent = parentPending
                ? matched!.BasisParentId
                : basisParentId ?? matched?.BasisParentId;
            var burns = snapshot.Burns
                .Where(b => BurnIdentityPolicy.DifferentBurn(b.TimeSeconds, timeSeconds))
                .Append(new PlanSnapshotBurn(timeSeconds, dvVlf, parent))
                .ToArray();
            if (markDownstreamParentsPending)
                MarkParentRefreshAfter(timeSeconds, burns);
            System.Threading.Volatile.Write(ref _snapshot,
                PlanSnapshot.Capture(snapshot.EpochSeconds, snapshot.State,
                    snapshot.AnchorParentId, burns, snapshot.Engine,
                    snapshot.PropulsionSource));
        }
        BumpVersion();
    }

    /// <summary>Writer-mirror for a time edit: re-keys the captured burn the way
    /// <see cref="MoveMeta"/> re-keys the meta (target-slot eviction, self-survival
    /// on a within-tolerance move).</summary>
    public void SnapshotMoveBurn(double oldTimeSeconds, double newTimeSeconds,
        string? basisParentId = null) =>
        SnapshotMoveBurnCore(oldTimeSeconds, newTimeSeconds, basisParentId,
            deferParentRefresh: false);

    /// <summary>Time-edit mirror: move immediately, but retain the old known parent
    /// until a later stock capture resolves the moved Burn through its rebuilt patch
    /// chain. Burn.Update only queues that rebuild, so resolving in the edit frame
    /// would observe the old chain.</summary>
    public void SnapshotMoveBurnDeferred(double oldTimeSeconds, double newTimeSeconds) =>
        SnapshotMoveBurnCore(oldTimeSeconds, newTimeSeconds, basisParentId: null,
            deferParentRefresh: true);

    private void SnapshotMoveBurnCore(double oldTimeSeconds, double newTimeSeconds,
        string? basisParentId, bool deferParentRefresh)
    {
        lock (_snapshotGate)
        {
            if (_snapshot is not { } snapshot) return;
            var moved = snapshot.Burns.FirstOrDefault(
                b => BurnIdentityPolicy.SameBurn(b.TimeSeconds, oldTimeSeconds));
            if (moved is null) return;
            bool sourcePending = RemoveParentRefreshAt(moved.TimeSeconds);
            bool keepPending = deferParentRefresh || sourcePending;
            RemoveParentRefreshAt(newTimeSeconds); // evicted target-slot ownership
            string? parent = keepPending
                ? moved.BasisParentId
                : basisParentId ?? moved.BasisParentId;
            var burns = snapshot.Burns
                .Where(b => !ReferenceEquals(b, moved)
                    && BurnIdentityPolicy.DifferentBurn(b.TimeSeconds, newTimeSeconds))
                .Append(new PlanSnapshotBurn(newTimeSeconds, moved.DeltaVVlf, parent))
                .ToArray();
            if (keepPending) AddParentRefreshAt(newTimeSeconds);
            if (deferParentRefresh || _diverged)
                MarkParentRefreshAtOrAfter(
                    Math.Min(oldTimeSeconds, newTimeSeconds), burns);
            System.Threading.Volatile.Write(ref _snapshot,
                PlanSnapshot.Capture(snapshot.EpochSeconds, snapshot.State,
                    snapshot.AnchorParentId, burns, snapshot.Engine,
                    snapshot.PropulsionSource));
        }
        BumpVersion();
    }

    /// <summary>Writer-mirror: drop the captured burn at the time slot.</summary>
    public void SnapshotRemoveBurn(double timeSeconds)
    {
        lock (_snapshotGate)
        {
            RemoveParentRefreshAt(timeSeconds);
            if (_snapshot is not { } snapshot) return;
            var burns = snapshot.Burns
                .Where(b => BurnIdentityPolicy.DifferentBurn(b.TimeSeconds, timeSeconds))
                .ToArray();
            MarkParentRefreshAfter(timeSeconds, burns);
            System.Threading.Volatile.Write(ref _snapshot,
                PlanSnapshot.Capture(snapshot.EpochSeconds, snapshot.State,
                    snapshot.AnchorParentId, burns, snapshot.Engine,
                    snapshot.PropulsionSource));
        }
        BumpVersion();
    }

    /// <summary>Capture-scan seam: a successfully resolved existing-burn patch may
    /// complete a deferred time-edit parent refresh. Null preserves the known parent
    /// and leaves the refresh pending. Works while diverged: wholesale reconcile is
    /// intentionally disabled there, but this one moved record still must converge.</summary>
    internal string? SnapshotParentFromCapture(double timeSeconds,
        string? resolvedParentId, bool patchChainReady = true)
    {
        if (!patchChainReady) resolvedParentId = null;
        bool geometryChanged = false;
        string? parent;
        lock (_snapshotGate)
        {
            if (_snapshot is not { } snapshot) return resolvedParentId;
            var matched = snapshot.Burns.FirstOrDefault(
                b => BurnIdentityPolicy.SameBurn(b.TimeSeconds, timeSeconds));
            if (!ParentRefreshPendingAt(timeSeconds))
                return resolvedParentId ?? matched?.BasisParentId;
            if (matched is null)
            {
                RemoveParentRefreshAt(timeSeconds);
                return resolvedParentId;
            }
            if (resolvedParentId is null) return matched.BasisParentId;

            RemoveParentRefreshAt(timeSeconds);
            parent = resolvedParentId;
            if (!string.Equals(matched.BasisParentId, resolvedParentId,
                    StringComparison.Ordinal))
            {
                var burns = snapshot.Burns.Select(b => ReferenceEquals(b, matched)
                    ? b with { BasisParentId = resolvedParentId }
                    : b);
                System.Threading.Volatile.Write(ref _snapshot,
                    PlanSnapshot.Capture(snapshot.EpochSeconds, snapshot.State,
                        snapshot.AnchorParentId, burns, snapshot.Engine,
                        snapshot.PropulsionSource));
                geometryChanged = true;
            }
        }
        if (geometryChanged) BumpVersion();
        return parent;
    }

    internal string? SnapshotKnownParentAt(double timeSeconds)
    {
        lock (_snapshotGate)
            return _snapshot?.Burns.FirstOrDefault(
                b => BurnIdentityPolicy.SameBurn(b.TimeSeconds, timeSeconds))
                ?.BasisParentId;
    }

    internal bool SnapshotParentRefreshPending(double timeSeconds)
    {
        lock (_snapshotGate) return ParentRefreshPendingAt(timeSeconds);
    }

    /// <summary>Marks installed snapshot records whose stock patch parents remain
    /// provisional. Rebase and sidecar restore call this after <see cref="SetSnapshot"/>
    /// clears the prior schedule. Only keys that still identify an installed burn are
    /// accepted, so stale scheduling data cannot leak into a later writer edit.</summary>
    internal void SnapshotMarkParentRefresh(IEnumerable<double> timeSeconds)
    {
        lock (_snapshotGate)
        {
            if (_snapshot is not { } snapshot) return;
            foreach (double timeSecondsValue in timeSeconds)
            {
                if (!double.IsFinite(timeSecondsValue)) continue;
                var matched = snapshot.Burns.FirstOrDefault(
                    b => BurnIdentityPolicy.SameBurn(
                        b.TimeSeconds, timeSecondsValue));
                if (matched is not null) AddParentRefreshAt(matched.TimeSeconds);
            }
        }
    }

    private bool ParentRefreshPendingAt(double timeSeconds) =>
        BurnIdentityPolicy.ContainsBurn(_pendingSnapshotParentRefresh, timeSeconds);

    private bool RemoveParentRefreshAt(double timeSeconds) =>
        _pendingSnapshotParentRefresh.RemoveAll(
            t => BurnIdentityPolicy.SameBurn(t, timeSeconds)) > 0;

    private void AddParentRefreshAt(double timeSeconds)
    {
        if (!ParentRefreshPendingAt(timeSeconds))
            _pendingSnapshotParentRefresh.Add(timeSeconds);
    }

    private void MarkParentRefreshAfter(double timeSeconds,
        IReadOnlyList<PlanSnapshotBurn> burns)
    {
        foreach (var burn in burns)
            if (burn.TimeSeconds > timeSeconds)
                AddParentRefreshAt(burn.TimeSeconds);
    }

    private void MarkParentRefreshAtOrAfter(double timeSeconds,
        IReadOnlyList<PlanSnapshotBurn> burns)
    {
        foreach (var burn in burns)
            if (burn.TimeSeconds >= timeSeconds)
                AddParentRefreshAt(burn.TimeSeconds);
    }
}

/// <summary>The per-vessel flight-plan store (session state, swept on rebind like the
/// panels; repopulated from the save sidecar on load) plus the pure plan rules the
/// overlay and sidecar consume. KSA-free.</summary>
public static class FlightPlans
{
    private static readonly object Gate = new();
    private static readonly Dictionary<string, FlightPlanModel> Plans = new(StringComparer.Ordinal);

    // Store-wide edit stamp: bumped by every plan create/remove/restore/sweep and by
    // every FlightPlanModel version bump. The overlay's per-tick staging call reads
    // THIS (one lock-free load) and only pays the store lock + per-plan compare when
    // it moved — plan edits happen at human timescales, staging at physics tick rate.
    private static long _editStamp;

    public static long EditStamp => System.Threading.Interlocked.Read(ref _editStamp);

    internal static void NoteEdit() => System.Threading.Interlocked.Increment(ref _editStamp);

    public const double DefaultLengthSeconds = 7.0 * 86400.0;

    /// <summary>Margin past the last ADOPTED stock burn when a plan is created around
    /// existing burns: the burn must land INSIDE the window with room to nudge it
    /// later, not exactly on the boundary.</summary>
    public const double AdoptedBurnMarginSeconds = 86400.0;

    /// <summary>Initial plan length when creating a plan over existing stock burns (the
    /// panel advertises adoption): the default length, extended so the LAST existing
    /// burn sits inside the window plus <see cref="AdoptedBurnMarginSeconds"/> — a plan
    /// must never be born violating its own ValidateLength rule ("plan would end before
    /// its last burn"), which would leave the adopted burn un-editable and un-drawn.
    /// Capped at the same <see cref="SettingsKernel.MaxRailsDays"/>
    /// ceiling ValidateLength enforces; a stock burn beyond even that stays adopted as
    /// a VLF burn the plan honestly cannot reach.</summary>
    public static double InitialLengthSeconds(double nowSeconds, IReadOnlyList<double> burnTimes)
    {
        double length = DefaultLengthSeconds;
        foreach (double t in burnTimes)
            if (double.IsFinite(t))
                length = Math.Max(length, t - nowSeconds + AdoptedBurnMarginSeconds);
        return Math.Min(length, SettingsKernel.MaxRailsDays * 86400.0);
    }

    public static FlightPlanModel? TryGet(string vesselId)
    {
        lock (Gate) return Plans.TryGetValue(vesselId, out var plan) ? plan : null;
    }

    public static FlightPlanModel Create(string vesselId, double nowSeconds,
        double lengthSeconds = DefaultLengthSeconds)
    {
        var plan = new FlightPlanModel { CreatedAtSeconds = nowSeconds, LengthSeconds = lengthSeconds };
        lock (Gate) Plans[vesselId] = plan;
        return plan;
    }

    public static void Remove(string vesselId)
    {
        lock (Gate) Plans.Remove(vesselId);
        NoteEdit(); // Create/restore bump via the LengthSeconds setter; removal has no setter
    }

    /// <summary>Statics sweep: a rebind/save load replaces the vessel population under
    /// the store; the restore path repopulates from the sidecar right after.</summary>
    internal static void ResetSessionStatics()
    {
        lock (Gate) Plans.Clear();
        NoteEdit();
    }

    /// <summary>Effective overlay/prediction horizon for one vessel, days ahead of now:
    /// the config horizon is the floor for everyone; a flight plan extends it to the
    /// plan end. EVERYTHING clamps to <paramref name="railsAheadDays"/> — callers pass
    /// the rails window the worker has actually integrated (min of the config target
    /// and the reached horizon), so neither a long plan nor a just-raised orbits preset
    /// can demand synchronous ephemerides integration: while the rails worker grows a
    /// raised window chunk by chunk, the drawn lines grow with it instead of freezing
    /// every rails reader.</summary>
    public static double EffectiveHorizonDays(double configDays, double railsAheadDays,
        double? planEndSeconds, double nowSeconds)
    {
        double days = configDays;
        if (planEndSeconds is { } end && double.IsFinite(end) && end > nowSeconds)
            days = Math.Max(days, (end - nowSeconds) / 86400.0);
        return Math.Min(days, railsAheadDays);
    }

    // ---- Sidecar bridge (SidecarPlan DTOs live in SaveSidecar.cs).

    /// <summary>Captures every plan independently of exact predictor-state eligibility.
    /// The store is keyed by vessel id, so the file-level records are unique by
    /// construction and deterministic ordering keeps save diffs stable.</summary>
    internal static List<SidecarPlanRecord> PlansForSidecar()
    {
        lock (Gate)
            return Plans
                .OrderBy(pair => pair.Key, StringComparer.Ordinal)
                .Select(pair => new SidecarPlanRecord
                {
                    VesselId = pair.Key,
                    Plan = ToSidecar(pair.Value),
                })
                .ToList();
    }

    public static SidecarPlan? ToSidecar(FlightPlanModel? plan)
    {
        if (plan is null) return null;
        var dto = new SidecarPlan
        {
            CreatedAtSeconds = plan.CreatedAtSeconds,
            LengthSeconds = plan.LengthSeconds,
            PropulsionSource = plan.PropulsionSource.ToString(),
        };
        foreach (var meta in plan.Meta)
            dto.Burns.Add(new SidecarPlanBurn
            {
                TimeSeconds = meta.TimeSeconds,
                FrameKind = meta.Frame.Kind.ToString(),
                PrimaryId = meta.Frame.PrimaryId,
                SecondaryId = meta.Frame.SecondaryId,
                Basis = "prn", // prograde/radial/normal (SidecarPlanBurn.Basis doc)
                X = meta.Authored.X,
                Y = meta.Authored.Y,
                Z = meta.Authored.Z,
            });
        PlanSnapshotPersistenceState snapshotState =
            plan.CaptureSnapshotPersistenceState();
        dto.Diverged = snapshotState.Diverged;
        if (snapshotState.Snapshot is { } snapshot)
        {
            dto.Anchor = new SidecarPlanAnchor
            {
                EpochSeconds = snapshot.EpochSeconds,
                PositionEcl = [snapshot.State.Position.X, snapshot.State.Position.Y, snapshot.State.Position.Z],
                VelocityEcl = [snapshot.State.Velocity.X, snapshot.State.Velocity.Y, snapshot.State.Velocity.Z],
                ParentId = snapshot.AnchorParentId,
                PropulsionSource = snapshot.PropulsionSource.ToString(),
                // Missing engine data is serialized as zeros and restores as an
                // impulsive fold.
                MassKg = snapshot.Engine?.MassKg ?? 0,
                ExhaustVelocity = snapshot.Engine?.ExhaustVelocity ?? 0,
                MassFlowRate = snapshot.Engine?.MassFlowRate ?? 0,
            };
            foreach (var burn in snapshot.Burns)
                dto.SnapshotBurns.Add(new SidecarSnapshotBurn
                {
                    TimeSeconds = burn.TimeSeconds,
                    X = burn.DeltaVVlf.X,
                    Y = burn.DeltaVVlf.Y,
                    Z = burn.DeltaVVlf.Z,
                    BasisParentId = burn.BasisParentId,
                    BasisParentRefreshPending =
                        snapshotState.ParentRefreshPendingAt(burn.TimeSeconds),
                });
        }
        return dto;
    }

    /// <summary>Rebuilds a model from the sidecar DTO; null when the plan itself is
    /// unusable (non-finite window). Individual broken burn metas are dropped — their
    /// burns still exist in the STOCK save and degrade honestly to VLF burns.</summary>
    public static FlightPlanModel? FromSidecar(SidecarPlan? dto, long nowMs)
    {
        if (dto is null) return null;
        if (!double.IsFinite(dto.CreatedAtSeconds) || !double.IsFinite(dto.LengthSeconds)
            || dto.LengthSeconds <= 0
            || dto.LengthSeconds > SettingsKernel.MaxRailsDays * 86400.0)
            return null;
        var plan = new FlightPlanModel
        {
            CreatedAtSeconds = dto.CreatedAtSeconds,
            LengthSeconds = dto.LengthSeconds,
            PropulsionSource = ParsePropulsionSource(dto.PropulsionSource),
        };
        foreach (var burn in dto.Burns ?? [])
        {
            if (burn is null) continue;
            if (!double.IsFinite(burn.TimeSeconds) || !double.IsFinite(burn.X)
                || !double.IsFinite(burn.Y) || !double.IsFinite(burn.Z)) continue;
            // Enum.TryParse alone accepts numeric strings and returns true for UNDEFINED
            // values ("42" -> (FrameKind)42), which would smuggle a phantom kind past
            // this sanitizer into FrameSpec — IsDefined closes it.
            if (!Enum.TryParse<FrameKind>(burn.FrameKind, ignoreCase: false, out var kind)
                || !Enum.IsDefined(kind)) continue;
            if (string.IsNullOrEmpty(burn.PrimaryId)) continue;
            bool needsSecondary = kind is FrameKind.TwoBodyFixed or FrameKind.TargetFixed;
            if (needsSecondary && string.IsNullOrEmpty(burn.SecondaryId)) continue;
            if (!needsSecondary && burn.SecondaryId is not null) continue;
            // Unsupported bases are dropped instead of reinterpreted; the stock burn
            // remains a plain VLF burn.
            if (!string.Equals(burn.Basis, "prn", StringComparison.Ordinal)) continue;
            plan.SetMeta(new FlightPlanBurnMeta
            {
                TimeSeconds = burn.TimeSeconds,
                Frame = new FrameSpec(kind, burn.PrimaryId, burn.SecondaryId),
                Authored = new Vector3d(burn.X, burn.Y, burn.Z),
                StampMs = nowMs,
            });
        }
        // Snapshot: restored whole or not at all — a partially-valid anchor would
        // fold the ghost from a corrupt state. A dropped snapshot re-arms the lazy
        // capture; the diverged flag survives either way (with no snapshot the planned
        // line stays cleared and the panel offers Rebase — honest for both cases).
        PlanSnapshot? snapshot = SnapshotFromSidecar(dto);
        plan.SetSnapshot(snapshot, dto.Diverged);
        if (snapshot is not null)
            plan.SnapshotMarkParentRefresh((dto.SnapshotBurns ?? [])
                .Where(b => b is { BasisParentRefreshPending: true })
                .Select(b => b!.TimeSeconds));
        return plan;
    }

    private static PlanSnapshot? SnapshotFromSidecar(SidecarPlan dto)
    {
        if (dto.Anchor is not { } anchor) return null;
        if (!double.IsFinite(anchor.EpochSeconds)) return null;
        if (anchor.PositionEcl is not { Length: 3 } p || p.Any(v => !double.IsFinite(v))) return null;
        if (anchor.VelocityEcl is not { Length: 3 } v || v.Any(x => !double.IsFinite(x))) return null;
        string? anchorParentId = string.IsNullOrEmpty(anchor.ParentId) ? null : anchor.ParentId;
        var burns = new List<PlanSnapshotBurn>();
        foreach (var burn in dto.SnapshotBurns ?? [])
        {
            if (burn is null) continue;
            if (!double.IsFinite(burn.TimeSeconds) || !double.IsFinite(burn.X)
                || !double.IsFinite(burn.Y) || !double.IsFinite(burn.Z)) return null;
            string? basisParentId = string.IsNullOrEmpty(burn.BasisParentId)
                ? anchorParentId : burn.BasisParentId;
            burns.Add(new PlanSnapshotBurn(burn.TimeSeconds,
                new Vector3d(burn.X, burn.Y, burn.Z), basisParentId));
        }
        // Unusable scalars restore without an engine model, preventing non-finite
        // burn durations.
        return PlanSnapshot.Capture(anchor.EpochSeconds,
            new StateVector(new Vector3d(p[0], p[1], p[2]), new Vector3d(v[0], v[1], v[2])),
            anchorParentId, burns,
            new EngineScalars(anchor.MassKg, anchor.ExhaustVelocity, anchor.MassFlowRate),
            ParsePropulsionSource(anchor.PropulsionSource));
    }

    private static PropulsionSource ParsePropulsionSource(string? text) =>
        Enum.TryParse<PropulsionSource>(text, ignoreCase: false, out var source)
        && Enum.IsDefined(source)
            ? source
            : PropulsionSource.MainEngines;

    /// <summary>Restores every persisted plan from a matched sidecar (called by the
    /// save-restore patch AFTER the load's rebind swept this store). The burns
    /// themselves come back through the stock save; frame metadata re-keys by time.
    /// Missing or invalid plan records are ignored.</summary>
    public static int ImportSidecar(SidecarFile sidecar)
    {
        int restored = 0;
        long nowMs = Environment.TickCount64;
        var records = sidecar.Plans ?? [];
        var seenIds = new HashSet<string>(StringComparer.Ordinal);
        var duplicateIds = new HashSet<string>(StringComparer.Ordinal);
        foreach (var record in records)
        {
            if (record is null || string.IsNullOrEmpty(record.VesselId)) continue;
            if (!seenIds.Add(record.VesselId)) duplicateIds.Add(record.VesselId);
        }
        foreach (var record in records)
        {
            if (record is null || string.IsNullOrEmpty(record.VesselId)
                || duplicateIds.Contains(record.VesselId)
                || FromSidecar(record.Plan, nowMs) is not { } plan)
                continue;
            lock (Gate) Plans[record.VesselId] = plan;
            restored++;
        }
        return restored;
    }
}

using System.Globalization;
using System.Text;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Vessels;

/// <summary>Tracks one TrajectoryPredictor per vessel. Seeding/invalidation: a vessel is
/// (re)seeded from the stock-staged state when first seen and whenever it returns to
/// Situation.Freefall from anything else — decided from the PRE-TICK committed props,
/// because the patches run after the stock branch already staged Freefall into the new
/// props (see <see cref="VesselLifecycle"/>). Live physics moved it: burns, collisions,
/// docking all end here — this IS the control-input invalidation. Entries whose vessel
/// left the sim are evicted; a replacement `Vehicle` instance under the same id is
/// reseeded instead of inheriting the prior predictor.</summary>
public sealed class VesselRegistry
{
    private readonly ModConfig config;
    private readonly RailsService rails;

    public VesselRegistry(ModConfig config, RailsService rails)
    {
        this.config = config;
        this.rails = rails;
    }

    /// <summary>Non-allocating capability token for strict authoritative-predictor
    /// consumers. All three references are identity, not merely vessel-id, state.</summary>
    internal readonly record struct RailsAuthoritySnapshot(
        string VesselId,
        TrackedVessel Tracked,
        Vehicle Vehicle,
        TrajectoryPredictor Lineage);

    /// <summary>Eviction bound: an entry unseen on both the staging and commit surfaces
    /// for this long left the sim (a live vessel is stamped every frame by the commit
    /// canary — even paused, landed or contained — so only destroyed/removed vessels
    /// ever go unseen this long).</summary>
    private const long EvictAfterMs = 30_000;
    private const long SweepEveryMs = 1_000;
    /// <summary>Keep-path teleport-jump bound. Generous: per-tick legitimate drift
    /// GROWTH is metres even at 1e5x warp (absolute drift reaches 1e7 m and is fine);
    /// only Vehicle.Teleport-class events move the staged stock state megametres
    /// within a single tick.</summary>
    private const double StaleTeleportMeters = 1e6;

    /// <summary>Sidecar restore guards: a sidecar entry applies only when the vessel's
    /// stock seed sits within the window of the sidecar epoch (the restored save's own
    /// vessels re-enter rails within a tick of the load, or up to seconds later for a
    /// vessel that was live at the save), and only when the exact state differs from
    /// the stock osculating seed by conic-drift order AT A COMMON EPOCH
    /// (see <see cref="SaveSidecar.RestoreDeltaMeters"/>: a raw across-skew comparison
    /// carries tens of km/s of heliocentric motion and would spuriously refuse
    /// legitimate late re-entries within the window). A legitimate common-epoch delta
    /// is bounded by the re-osculation refresh (~2x conic_drift_meters) plus cross-session
    /// rails interpolation spread (~1e3 m observed); a 1e5 m delta means a DIFFERENT
    /// state was saved at the same elapsed time (goto/teleport between two saves) —
    /// the stock seed wins and the mod degrades gracefully.</summary>
    private const double SidecarRestoreWindowSeconds = 30;
    private const double SidecarRestoreSanityMeters = 1e5;

    private readonly object _gate = new();
    private readonly Dictionary<string, TrackedVessel> _tracked = [];
    /// <summary>Pending exact states from the sidecar paired with the loaded save; each
    /// vessel consumes its entry on its first GetOrSeed after the load.</summary>
    private SidecarFile? _pendingSidecar;
    private long _nextSweepMs;
    /// <summary>Wall-clock stamp of the last sweep pass — the process-liveness witness
    /// (see <see cref="VesselLifecycle.SweepGapMeansStall"/>).</summary>
    private long _lastSweepMs = Environment.TickCount64;
    private long _lastPausedOverlayRefreshMs;
    private long _lastPausedPlanEditStamp = -1;
    private string? _lastPausedFrameLabel;
    private long _nextPausedOverlayWarnMs;
    private volatile bool _pausedEditsDeferred;
    private bool _pausedRefreshInitialized;
    public bool PausedEditsDeferred => _pausedEditsDeferred;
    /// <summary>Throttles the per-vessel "canary" log line exactly like the celestial
    /// drift lines: one line at first verified commit, then one per 30 s wall clock.</summary>
    private readonly Patches.RailsTelemetry _canaryTelemetry = new(driftPeriodMs: 30_000);

    public TrackedVessel GetOrSeed(VehicleUpdateState vehicleState)
    {
        string id = vehicleState.Id;
        bool preTickFreefall = vehicleState.ReadOnlyVehicle.Props.Situation == Situation.Freefall;
        double stagedTime = vehicleState.CurrentStateVectors.StateTime.Seconds();
        long now = Environment.TickCount64;
        lock (_gate)
        {
            SweepUnseen(now);
            _tracked.TryGetValue(id, out var tracked);
            if (tracked is not null)
            {
                tracked.LastSeenMs = now;
                if (!tracked.IsSameVehicle(vehicleState.ReadOnlyVehicle))
                    ForgetVesselSessionState(id);
            }

            // Every vessel parent must belong to the live modeled catalog. An unknown
            // parent is a global authority failure.
            if (vehicleState.CurrentOrbit.Parent is not Astronomical parent || !rails.IsModeled(parent.Id))
            {
                string parentId = (vehicleState.CurrentOrbit.Parent as Astronomical)?.Id
                    ?? "<non-astronomical>";
                throw new InvalidOperationException(
                    $"vessel '{id}' has no authoritative modeled parent '{parentId}'");
            }

            rails.NoteSimTime(stagedTime);
            if (!rails.IsReadyAt(stagedTime))
            {
                throw new InvalidOperationException(
                    $"vessel '{id}' reached t={stagedTime:R} before rails horizon {rails.Horizon:R}");
            }
            rails.RequestThirdBodyRefresh(parent.Id, stagedTime);

            switch (VesselLifecycle.Decide(
                alreadyTracked: tracked is not null,
                wasFreefallAtTickStart: preTickFreefall,
                reseedPending: tracked?.ReseedPending ?? false,
                sameVehicleInstance: tracked?.IsSameVehicle(vehicleState.ReadOnlyVehicle) ?? true))
            {
                case VesselLifecycle.Seeding.Seed:
                    tracked = new TrackedVessel
                    {
                        Id = id,
                        Rails = rails,
                        Options = new IntegratorOptions { RelTol = config.VesselRelTol },
                    };
                    tracked.ReseedAndBind(vehicleState.CurrentOrbit,
                        in vehicleState.CurrentStateVectors, vehicleState.ReadOnlyVehicle);
                    _tracked[id] = tracked;
                    ModLog.Info($"tracking vessel '{id}' from t={tracked.SeedTime:F1} s (parent {parent.Id})");
                    TryApplyPendingSidecar(tracked);
                    break;

                case VesselLifecycle.Seeding.Reseed:
                    tracked!.ReseedAndBind(vehicleState.CurrentOrbit,
                        in vehicleState.CurrentStateVectors, vehicleState.ReadOnlyVehicle);
                    // Plan snapshot: a reseed means live physics (or a
                    // stock-owned stretch) moved the vessel — reality left the plan's
                    // world, the planned line freezes into the ghost until Rebase.
                    FlightPlans.TryGet(id)?.MarkDiverged();
                    // A transition tick reseeds once per physics substep; log once per second.
                    if (now - tracked.LastReseedLogMs >= 1000)
                    {
                        tracked.LastReseedLogMs = now;
                        ModLog.Info($"reseeded vessel '{id}' at t={tracked.SeedTime:F1} s "
                            + "(returned to Freefall from live physics)");
                    }
                    break;
            }

            // Parent-transition observation: a parent change on the Keep path IS the stock patch
            // (SOI) transition landing while the vessel stayed on rails. The predictor
            // is absolute-frame and continuous across it; EvaluateForStaging simply
            // re-expresses it against the new parent. (Seed/Reseed stamp the id first,
            // so those paths never log a spurious transition.) The flag feeds the
            // teleport guard's adoption rule: stock's transition also APPLIES its
            // Kepler-anchored next-patch state this tick, a discontinuity the guard
            // must override rather than adopt.
            bool parentChanged = tracked!.LastParentId != parent.Id;
            tracked.ParentTransitionTick = parentChanged;
            if (parentChanged)
            {
                if (FlightPlans.TryGet(id) is not null)
                    BasisReconversionUrgency.Raise(id);
                // Same 1/s wall budget as the reseed line: guards a pathological
                // Earth<->Luna plan-flap at warp from spamming the log.
                if (now - tracked.LastTransitionLogMs >= 1000)
                {
                    tracked.LastTransitionLogMs = now;
                    ModLog.Info($"vessel '{id}': SOI patch transition '{tracked.LastParentId}' -> '{parent.Id}' "
                        + $"at t={stagedTime:F1} s ({stagedTime / 86400.0:F2} d) - predictor continuous across the boundary");
                }
                tracked.LastParentId = parent.Id;
            }

            // Bounded memory over warp: keep the same window behind now as the rails do.
            tracked.BindUpdateState(vehicleState);
            tracked.PruneBehind(stagedTime - config.RailsKeepBehindDays * 86400);
            return tracked;
        }
    }

    /// <summary>Render-frame liveness seam for simulation speed zero. Stock does not
    /// run the vehicle staging paths while paused, but UI burn and frame edits still
    /// happen and published overlays otherwise age out after five wall-clock seconds.
    /// Unchanged immutable batches are restamped once per second so lines do not age
    /// out. A burn/frame edit cannot safely rebuild from solver-owned staging objects
    /// in this render phase; it clears stale planned geometry, raises an explicit
    /// deferred notice, and rebuilds after simulation resumes.</summary>
    public void RefreshPausedOverlays()
    {
        try
        {
            double speed = Universe.GetSimulationSpeed();
            if (speed != 0.0 || !ModServices.Enabled)
            {
                _pausedEditsDeferred = false;
                _pausedRefreshInitialized = false;
                return;
            }
            long wallMs = Environment.TickCount64;
            long editStamp = FlightPlans.EditStamp;
            string? frameLabel = FrameManager.Active?.Label;
            if (!PausedOverlayRefreshKernel.ShouldRefresh(speed, wallMs,
                    _lastPausedOverlayRefreshMs, editStamp, _lastPausedPlanEditStamp,
                    frameLabel, _lastPausedFrameLabel))
                return;
            bool editDeferred = _pausedRefreshInitialized
                && (editStamp != _lastPausedPlanEditStamp
                    || !string.Equals(frameLabel, _lastPausedFrameLabel, StringComparison.Ordinal));
            _pausedRefreshInitialized = true;
            _pausedEditsDeferred = PausedOverlayRefreshKernel.AccumulateDeferral(
                _pausedEditsDeferred, editDeferred);
            // PrepareFrame queues the next vehicle jobs before UI rendering. Speed zero
            // is not itself a memory barrier, so this seam must never read retained
            // staging objects or wait for those jobs.
            _lastPausedOverlayRefreshMs = wallMs;
            _lastPausedPlanEditStamp = editStamp;
            _lastPausedFrameLabel = frameLabel;

            string[] ids;
            lock (_gate)
                ids = _tracked.Values.Select(t => t.Id).ToArray();

            // Published batches are immutable and solver-independent. Restamping them
            // keeps paused lines alive without joining the newly queued solver jobs.
            // Plan/frame edits safely defer their rebuild until a completed staging
            // capture arrives; the render thread never touches retained mutable state.
            foreach (string id in ids)
            {
                if (OverlayBuffer.Read(id) is { } actual)
                    OverlayBuffer.Publish(actual.WithFreshStamp(wallMs));
                if (_pausedEditsDeferred)
                    OverlayBuffer.ClearPlanned(id);
                else if (OverlayBuffer.ReadPlanned(id) is { } planned)
                    OverlayBuffer.PublishPlanned(planned.WithFreshStamp(wallMs));
            }
            return;

        }
        catch (Exception e)
        {
            long nowMs = Environment.TickCount64;
            if (nowMs >= _nextPausedOverlayWarnMs)
            {
                _nextPausedOverlayWarnMs = nowMs + 5000;
                ModLog.Warn($"paused overlay refresh contained: {e.Message}");
            }
        }
    }

    /// <summary>Exact states from the sidecar paired with the loaded save. The
    /// restore patch calls this right after the load's forced rebind produced this
    /// fresh registry — every vessel then re-seeds through the Seed branch, where
    /// <see cref="TryApplyPendingSidecar"/> consumes its entry.</summary>
    public void ImportSidecar(SidecarFile sidecar)
    {
        lock (_gate) _pendingSidecar = sidecar;
    }

    /// <summary>Vessels whose predictor state is current enough to persist (see
    /// <see cref="VesselLifecycle.SidecarEligible"/>): landed / stock-owned / disabled /
    /// burn-pending vessels are excluded — for them the stock osculating state IS the
    /// truth, and the load-time degradation path handles them correctly.</summary>
    public IReadOnlyList<TrackedVessel> SnapshotForSidecar(double elapsedSeconds)
    {
        lock (_gate)
            return _tracked.Values
                .Where(t => VesselLifecycle.SidecarEligible(
                    t.ReseedPending, t.Predictor is not null,
                    t.SeedTime, Math.Max(t.LastRefreshTime, t.LastStagedTime),
                    elapsedSeconds, config.OsculationRefreshSeconds))
                .ToArray();
    }

    /// <summary>Called under <see cref="_gate"/> immediately after a stock seed: replaces
    /// the just-seeded (osculating) predictor with the sidecar's exact state when the
    /// guards hold; the entry is consumed either way (once-only). Rejections degrade to
    /// the stock seed — the save is valid stock data by construction.</summary>
    private void TryApplyPendingSidecar(TrackedVessel tracked)
    {
        if (_pendingSidecar is not { } pending) return;
        var fromSidecar = pending.Vessels.FirstOrDefault(v => v.Id == tracked.Id);
        if (fromSidecar is null) return;
        pending.Vessels.Remove(fromSidecar);
        if (!VesselLifecycle.ShouldRestoreFromSidecar(
                fromSidecar.EpochSeconds, tracked.SeedTime, SidecarRestoreWindowSeconds))
        {
            ModLog.Warn($"vessel '{tracked.Id}': sidecar epoch {fromSidecar.EpochSeconds:F1} s is outside the "
                + $"restore window of this seed (t={tracked.SeedTime:F1} s) - keeping the stock osculating seed");
            return;
        }
        var absolute = new StateVector(
            new Vector3d(fromSidecar.PositionEcl[0], fromSidecar.PositionEcl[1], fromSidecar.PositionEcl[2]),
            new Vector3d(fromSidecar.VelocityEcl[0], fromSidecar.VelocityEcl[1], fromSidecar.VelocityEcl[2]));
        double delta = SaveSidecar.RestoreDeltaMeters(rails, tracked, absolute, fromSidecar.EpochSeconds);
        if (delta > SidecarRestoreSanityMeters)
        {
            ModLog.Warn($"vessel '{tracked.Id}': sidecar state is {delta:E2} m from the stock seed at the common "
                + "epoch (a different state was saved at this elapsed time?) - keeping the stock osculating seed");
            return;
        }
        if (!SaveSidecar.TryNormalizeRestoreEpoch(absolute, fromSidecar.EpochSeconds,
                tracked.SeedTime, out var restored, out double restoredEpoch))
        {
            ModLog.Warn($"vessel '{tracked.Id}': sidecar epoch {fromSidecar.EpochSeconds:R} s is "
                + $"ahead of the stock seed at {tracked.SeedTime:R} s by more than KSA's "
                + "save-time rounding tolerance - keeping the stock osculating seed");
            return;
        }
        tracked.ReseedAbsolute(restored, restoredEpoch);
        ModLog.Info($"vessel '{tracked.Id}' restored exactly from sidecar (epoch {restoredEpoch:F4} s; "
            + $"exact state vs stock osculating seed at common epoch: {delta:E2} m)");
    }

    /// <summary>Evicts entries whose vessel has left the sim (unseen on the staging AND
    /// commit surfaces for <see cref="EvictAfterMs"/>): dead vessels leave the panel and
    /// free their predictor node lists. Called under <see cref="_gate"/>. A sweep gap
    /// longer than the eviction bound means the PROCESS stalled (or nothing staged for
    /// that long) — entries are re-stamped instead of evicted, because their LastSeenMs
    /// silence proves nothing about the vessels.</summary>
    private void SweepUnseen(long now)
    {
        if (now < _nextSweepMs) return;
        _nextSweepMs = now + SweepEveryMs;
        bool stalled = VesselLifecycle.SweepGapMeansStall(now, _lastSweepMs, EvictAfterMs);
        _lastSweepMs = now;
        if (stalled)
        {
            foreach (var tracked in _tracked.Values) tracked.LastSeenMs = now;
            if (_tracked.Count > 0)
                ModLog.Info($"sweep gap exceeded {EvictAfterMs / 1000} s (process stall or quiet stretch) - "
                    + $"re-stamped {_tracked.Count} entries instead of evicting");
            return;
        }
        List<string>? dead = null;
        foreach (var (id, tracked) in _tracked)
            if (VesselLifecycle.ShouldEvict(now, tracked.LastSeenMs, EvictAfterMs))
                (dead ??= []).Add(id);
        if (dead is null) return;
        foreach (var id in dead)
        {
            _tracked.Remove(id);
            ForgetVesselSessionState(id);
            ModLog.Info($"evicted vessel '{id}' (unseen for {EvictAfterMs / 1000} s - left the sim)");
        }
    }

    /// <summary>Destroyed vessels and replacement instances under a recycled id must
    /// not donate frame or flight-plan state to the next vessel.</summary>
    internal static void ForgetVesselSessionState(string vesselId)
    {
        FrameManager.ForgetSelection(vesselId);
        FlightPlans.Remove(vesselId);
    }

    /// <summary>Predictor state for staging at <paramref name="time"/>, with conic-drift
    /// bookkeeping and the Keep-path teleport guard: an upward drift jump beyond
    /// <see cref="StaleTeleportMeters"/> within one tick means the staged stock state
    /// moved discontinuously under the same vehicle instance, so the predictor is
    /// reseeded from the game state. Called by both rail patches after
    /// <see cref="GetOrSeed"/>.</summary>
    public StateVectors EvaluateForStaging(VehicleUpdateState vehicleState, TrackedVessel tracked, SimTime time)
    {
        var currentOrbit = vehicleState.CurrentOrbit;
        StateVectors stock = vehicleState.CurrentStateVectors;
        tracked.LastStagedTime = time.Seconds(); // sidecar freshness witness
        bool transitionTick = tracked.ParentTransitionTick;
        tracked.ParentTransitionTick = false;
        StateVectors sv = tracked.EvaluateGameState(currentOrbit, time);
        double drift = (sv.PositionCci - stock.PositionCci).Length();
        if (VesselLifecycle.IsTeleportJump(tracked.LastConicDrift, drift, StaleTeleportMeters))
        {
            if (!VesselLifecycle.AdoptStagedJump(transitionTick))
            {
                // Stock's patch transition applied its Kepler-anchored next-patch state
                // this tick — the jump is stock's discontinuity, not the vessel's. Keep
                // the predictor; the Seam 1 override replaces the staged snap right
                // after this returns. The CONIC must be healed HERE too, not left to
                // the single-vessel path's re-osculation refresh: transition ticks
                // route through ApplyFullPhysics (ApplySingleVehicleMotion delegates
                // when Patch.EndTime is crossed, VehicleUpdateTask.cs:513-524), whose
                // staging path never refreshes — overridden vectors riding the
                // Kepler-anchored patch orbit would re-stage the snap from the conic
                // next tick, against a drift baseline the same-tick second staging
                // pass resets to zero.
                var healedOrbit = Orbit.CreateFromStateCci(currentOrbit.Parent, time,
                    sv.PositionCci, sv.VelocityCci, currentOrbit.OrbitLineColor);
                vehicleState.SetCurrentOrbit(healedOrbit, vehicleState.ReadOnlyVehicle.Hash);
                sv = tracked.EvaluateGameState(healedOrbit, time); // TA now fresh
                NoteOsculationRefresh(tracked, time.Seconds());
                long nowMs = Environment.TickCount64;
                if (nowMs - tracked.LastSnapOverrideLogMs >= 1000)
                {
                    tracked.LastSnapOverrideLogMs = nowMs;
                    ModLog.Info($"vessel '{vehicleState.Id}': stock patch-transition snap of {drift:E2} m "
                        + $"overridden and conic healed (prev drift {tracked.LastConicDrift:E2} m at "
                        + $"t={time.Seconds():F1} s; Kepler-anchored patch state; predictor continuous "
                        + "across the boundary)");
                }
            }
            else
            {
                ModLog.Warn($"vessel '{vehicleState.Id}': staged stock state jumped {drift:E2} m off the predictor "
                    + $"in one tick (prev drift {tracked.LastConicDrift:E2} m at t={time.Seconds():F1} s, "
                    + $"stock parent '{(currentOrbit.Parent as Astronomical)?.Id}', last observed '{tracked.LastParentId}') "
                    + "- teleport-class event, reseeding to the game state instead of staging");
                lock (_gate)
                {
                    tracked.ReseedAndBind(currentOrbit,
                        in vehicleState.CurrentStateVectors, vehicleState.ReadOnlyVehicle);
                }
                FlightPlans.TryGet(vehicleState.Id)?.MarkDiverged(); // a teleport IS a departure from the plan's world
                sv = tracked.EvaluateGameState(currentOrbit, time);
                drift = (sv.PositionCci - stock.PositionCci).Length();
            }
        }
        tracked.LastConicDrift = drift;
        return sv;
    }

    /// <summary>Sim-seconds between rails-geometric SOI checks per vessel: at 1x this
    /// bounds the cost (a folded rails sample sweep every 10 s), at warp every tick
    /// spans more than the period so the check degrades to once per tick — a wall
    /// throttle instead would let an entire SOI transit fit between two checks.</summary>
    private const double SoiCheckPeriodSeconds = 10;
    private const int MaxSoiSweepBaseSamples = 257;
    private const int MaxSoiSweepSamples = 1025;
    private const int MaxSoiSweepRefinementDepth = 10;
    private const int MaxSoiEvidenceCrossings = 64;
    private const double SoiSweepFlatnessFraction = 0.005;
    private readonly record struct SoiSweepSample(
        double Time, Vector3d VesselAbsolute, StateVector[] BodyStates);

    /// <summary>Rails-geometric SOI parent decision for an on-rails vessel (the
    /// decision itself is <see cref="SoiReparentKernel.Decide"/> at exact SOI radii):
    /// the new parent when the vessel is inside a child's SOI or outside its
    /// parent's, null to keep. Stock's own on-rails transition fires only through the
    /// flight plan's patch schedule (VehicleUpdateTask.cs:845-867), whose encounters
    /// are conic-extrapolation-vs-Kepler-body predictions — an n-body trajectory that
    /// bends into an encounter those miss keeps the stale parent all the way to the
    /// child's surface. Candidates include every finite-SOI modeled gravity source;
    /// a re-parent retains the same absolute authoritative predictor.</summary>
    public Astronomical? RailsSoiParent(TrackedVessel tracked, Orbit currentOrbit, SimTime time)
    {
        double t = time.Seconds();
        // GetOrSeed guaranteed an integrated Astronomical parent this tick; Orbit.Parent
        // is IParentBody-typed, so the same object carries SOI radius and children.
        if (currentOrbit.Parent is not Astronomical parent) return null;
        IParentBody parentBody = currentOrbit.Parent;
        Astronomical? grandparent =
            parentBody is IOrbiter orbiter && orbiter.Parent is Astronomical gp && rails.IsModeled(gp.Id)
                ? gp : null;

        // Cheap cursor admission comes before candidate enumeration/allocation. Only a
        // matching forward parent+predictor lineage may reuse historical endpoints.
        TrajectoryPredictor lineage;
        SoiSweepAdmissionKernel.Decision admission;
        lock (rails.Gate)
        {
            lineage = tracked.Predictor;
            double cursor = tracked.LastSoiCheckSimSeconds;
            admission = SoiSweepAdmissionKernel.Decide(t, SoiCheckPeriodSeconds,
                parent.Id, lineage, cursor, tracked.LastSoiCheckParentId,
                tracked.LastSoiCheckPredictor, lineage.StartTime,
                double.IsFinite(cursor) && rails.IsReadyAt(cursor));
        }
        if (!admission.ShouldCheck) return null;

        // Candidate sweep mirrors stock's entry test population (PhysicsStates.cs:505-517):
        // celestials with a finite SOI that actually exert vessel gravity.
        List<string> ids = [parent.Id];
        List<Astronomical> childBodies = [];
        List<double> childSois = [];
        foreach (var child in parentBody.Children)
        {
            if (child is not Celestial celestial) continue;
            IParentBody childBody = celestial;
            if (!rails.IsSoiChildCandidate(celestial.Id, childBody.SphereOfInfluence))
                continue;
            ids.Add(celestial.Id);
            childBodies.Add(celestial);
            childSois.Add(childBody.SphereOfInfluence);
        }

        // All actual-trajectory and moving-body samples are captured under one outer
        // Gate acquisition. StateAt/GetAbsoluteMany re-enter the same Monitor; no
        // worker can prune/extend either history halfway through this sweep.
        List<SoiSweepSample> samples;
        lock (rails.Gate)
        {
            // A concurrent reseed cannot lend the old cursor to the new trajectory.
            if (!ReferenceEquals(tracked.Predictor, lineage))
            {
                lineage = tracked.Predictor;
                admission = new SoiSweepAdmissionKernel.Decision(
                    SoiSweepAdmissionKernel.Mode.EndpointOnly, t);
            }
            samples = admission.CanSweep
                ? BuildSoiSweepSamples(lineage, admission.FromTime, t, ids,
                    parentBody.SphereOfInfluence, childSois)
                : [SampleSoiState(lineage, t, ids)];
        }

        // Same-id replacement/reseed after capture invalidates both the sampled path
        // and its cursor. Follow the registry's established _gate -> Rails.Gate order.
        lock (_gate)
        {
            if (!_tracked.TryGetValue(tracked.Id, out var current)
                || !ReferenceEquals(current, tracked)) return null;
            lock (rails.Gate)
                if (!ReferenceEquals(tracked.Predictor, lineage)) return null;
        }

        SoiSweepSample endpoint = samples[^1];
        var candidates = new SoiReparentKernel.Candidate[childBodies.Count];
        for (int i = 0; i < childBodies.Count; i++)
            candidates[i] = new SoiReparentKernel.Candidate(
                ids[i + 1], endpoint.BodyStates[i + 1].Position, childSois[i]);

        string endpointParentId = SoiReparentKernel.Decide(
            endpoint.VesselAbsolute, endpoint.BodyStates[0].Position,
            parentBody.SphereOfInfluence, grandparent?.Id, candidates,
            enterFactor: 1.0, exitFactor: 1.0) ?? parent.Id;

        string sampledFinalParentId;
        IReadOnlyList<SoiReparentKernel.Crossing> crossings = [];
        bool crossingsTruncated = false;
        if (admission.CanSweep && samples.Count >= 2)
        {
            double from = admission.FromTime;
            double span = t - from;
            var fractions = new double[samples.Count];
            var parentRelative = new Vector3d[samples.Count];
            var childRelative = new Vector3d[childBodies.Count][];
            for (int i = 0; i < childBodies.Count; i++)
                childRelative[i] = new Vector3d[samples.Count];
            for (int sample = 0; sample < samples.Count; sample++)
            {
                var point = samples[sample];
                fractions[sample] = sample == 0 ? 0.0
                    : sample == samples.Count - 1 ? 1.0
                    : (point.Time - from) / span;
                parentRelative[sample] = point.VesselAbsolute - point.BodyStates[0].Position;
                for (int child = 0; child < childBodies.Count; child++)
                    childRelative[child][sample] =
                        point.VesselAbsolute - point.BodyStates[child + 1].Position;
            }

            var polylineChildren = new SoiReparentKernel.PolylineCandidate[childBodies.Count];
            for (int child = 0; child < childBodies.Count; child++)
                polylineChildren[child] = new SoiReparentKernel.PolylineCandidate(
                    ids[child + 1], childRelative[child], childSois[child]);
            var result = SoiReparentKernel.SweepPolyline(parent.Id, fractions,
                parentRelative, parentBody.SphereOfInfluence, grandparent?.Id,
                polylineChildren);
            sampledFinalParentId = result.FinalParentId;
            crossings = result.Crossings;
            crossingsTruncated = result.CrossingsTruncated;
        }
        else sampledFinalParentId = endpointParentId;

        // The sampled state machine supplies ordered history; the exact endpoint
        // classifier remains the final authority against refinement-budget or
        // floating-boundary misses.
        var reconciliation = SoiSweepReconciliationKernel.Reconcile(
            sampledFinalParentId, endpointParentId, t,
            crossings.Count, crossingsTruncated, MaxSoiEvidenceCrossings);
        string finalParentId = reconciliation.FinalParentId;

        bool TryMapParent(string id, out Astronomical? body)
        {
            body = null;
            if (string.Equals(id, parent.Id, StringComparison.Ordinal)) return true;
            if (grandparent is not null
                && string.Equals(id, grandparent.Id, StringComparison.Ordinal))
            {
                body = grandparent;
                return true;
            }
            int childIndex = ids.IndexOf(id) - 1;
            if (childIndex < 0) return false;
            body = childBodies[childIndex];
            return true;
        }

        if (!TryMapParent(finalParentId, out Astronomical? finalParent))
        {
            throw new InvalidOperationException(
                $"SOI sweep produced unmapped modeled parent '{finalParentId}' "
                + $"for vessel '{tracked.Id}'");
        }

        LogSoiSweepEvidence(tracked.Id, parent.Id,
            admission.FromTime, t, crossings, reconciliation);
        tracked.LastSoiCheckSimSeconds = reconciliation.CursorTime;
        tracked.LastSoiCheckParentId = reconciliation.CursorParentId;
        tracked.LastSoiCheckPredictor = lineage;
        return finalParent;
    }

    private static void LogSoiSweepEvidence(string vesselId, string startParentId,
        double from, double to, IReadOnlyList<SoiReparentKernel.Crossing> crossings,
        SoiSweepReconciliationKernel.Decision reconciliation)
    {
        if (!reconciliation.ShouldLogEvidence) return;
        int shown = reconciliation.EvidenceCrossingCount;
        var ordered = new StringBuilder(startParentId, 128);
        for (int i = 0; i < shown; i++)
        {
            var crossing = crossings[i];
            double at = from + (to - from) * crossing.Fraction;
            ordered.Append('>').Append(crossing.NewParentId).Append('@')
                .Append(at.ToString("R", CultureInfo.InvariantCulture)).Append('s');
        }
        if (crossings.Count > shown)
            ordered.Append(">...[+").Append(crossings.Count - shown).Append(']');
        if (reconciliation.KernelTruncated)
            ordered.Append(">...[kernel-transition-cap]");
        ModLog.Info(nameof(RailsSoiParent) + ':' + vesselId + ':'
            + ordered + ':' + reconciliation.FinalParentId
            + (reconciliation.EvidenceTruncated ? ":truncated" : string.Empty));
    }

    private List<SoiSweepSample> BuildSoiSweepSamples(TrajectoryPredictor lineage,
        double from, double to, IReadOnlyList<string> bodyIds,
        double parentSoi, IReadOnlyList<double> childSois)
    {
        // Ensure adaptive integrator nodes for the complete interval exist before
        // selecting the retained-node grid; the normal staging path already did this.
        lineage.StateAt(to);
        List<double> times = SoiSweepBaseTimes(lineage, from, to);
        var baseSamples = new List<SoiSweepSample>(times.Count);
        foreach (double sampleTime in times)
            baseSamples.Add(SampleSoiState(lineage, sampleTime, bodyIds));

        double[] radii = new double[childSois.Count + 1];
        radii[0] = parentSoi;
        for (int i = 0; i < childSois.Count; i++) radii[i + 1] = childSois[i];
        double maxSegmentSeconds = Math.Max(SoiCheckPeriodSeconds, (to - from) / 256.0);
        var budget = new SoiSweepWorkBudget(
            MaxSoiSweepSamples - baseSamples.Count);
        var refined = new List<SoiSweepSample>(MaxSoiSweepSamples) { baseSamples[0] };
        for (int i = 1; i < baseSamples.Count; i++)
            AppendRefinedSoiSegment(refined, baseSamples[i - 1], baseSamples[i],
                0, ref budget, maxSegmentSeconds, lineage, bodyIds, radii);
        return refined;
    }

    private static List<double> SoiSweepBaseTimes(
        TrajectoryPredictor lineage, double from, double to)
    {
        IReadOnlyList<TrajectoryNode> nodes = lineage.Nodes;
        var selection = SoiSweepGridKernel.SelectInterior(
            nodes.Count, index => nodes[index].Time,
            from, to, MaxSoiSweepBaseSamples);
        var times = new List<double>(selection.SelectedCount + 2) { from };
        for (int selected = 0; selected < selection.SelectedCount; selected++)
            times.Add(nodes[selection.IndexAt(selected)].Time);
        times.Add(to);
        return times;
    }

    private SoiSweepSample SampleSoiState(TrajectoryPredictor lineage,
        double time, IReadOnlyList<string> bodyIds) =>
        new(time, lineage.StateAt(time).Position,
            rails.GetAbsoluteMany(bodyIds, time));

    private void AppendRefinedSoiSegment(List<SoiSweepSample> output,
        SoiSweepSample start, SoiSweepSample end, int depth,
        ref SoiSweepWorkBudget budget, double maxSegmentSeconds,
        TrajectoryPredictor lineage, IReadOnlyList<string> bodyIds,
        IReadOnlyList<double> radii)
    {
        if (depth >= MaxSoiSweepRefinementDepth)
        {
            output.Add(end);
            return;
        }
        double midpointTime = start.Time + (end.Time - start.Time) * 0.5;
        if (!(midpointTime > start.Time && midpointTime < end.Time))
        {
            output.Add(end);
            return;
        }
        if (!budget.TryConsume())
        {
            output.Add(end);
            return;
        }
        SoiSweepSample midpoint = SampleSoiState(lineage, midpointTime, bodyIds);
        if (!SoiSegmentNeedsRefinement(start, midpoint, end, radii, maxSegmentSeconds))
        {
            output.Add(end);
            return;
        }
        AppendRefinedSoiSegment(output, start, midpoint, depth + 1,
            ref budget, maxSegmentSeconds, lineage, bodyIds, radii);
        AppendRefinedSoiSegment(output, midpoint, end, depth + 1,
            ref budget, maxSegmentSeconds, lineage, bodyIds, radii);
    }

    private static bool SoiSegmentNeedsRefinement(SoiSweepSample start,
        SoiSweepSample midpoint, SoiSweepSample end,
        IReadOnlyList<double> radii, double maxSegmentSeconds)
    {
        if (end.Time - start.Time > maxSegmentSeconds) return true;
        for (int i = 0; i < radii.Count; i++)
        {
            double radius = radii[i];
            if (!(radius > 0) || !double.IsFinite(radius)) continue;
            Vector3d a = start.VesselAbsolute - start.BodyStates[i].Position;
            Vector3d m = midpoint.VesselAbsolute - midpoint.BodyStates[i].Position;
            Vector3d b = end.VesselAbsolute - end.BodyStates[i].Position;
            bool aInside = a.Length() <= radius;
            bool mInside = m.Length() <= radius;
            bool bInside = b.Length() <= radius;
            if (aInside != mInside || mInside != bInside) return true;
            Vector3d chordMidpoint = (a + b) * 0.5;
            double tolerance = Math.Max(1e-3,
                radius * SoiSweepFlatnessFraction);
            if ((m - chordMidpoint).Length() > tolerance) return true;
        }
        return false;
    }

    /// <summary>Books a re-osculation (the patch rebuilt the vessel's stock
    /// conic/FlightPlan from the n-body state via CreateFromStateCci + SetCurrentOrbit):
    /// stamps the sim-time so the periodic trigger re-arms, counts it for the panel, and
    /// logs throttled — at high warp the periodic trigger can fire every tick (a tick
    /// spans more than osculation_refresh_seconds of sim time), so refresh lines get the
    /// same one-per-30-s-wall-per-vessel budget as the canary.</summary>
    public void NoteOsculationRefresh(TrackedVessel tracked, double time)
    {
        double drift = tracked.LastConicDrift;
        tracked.LastRefreshTime = time;
        tracked.RefreshCount++;
        long now = Environment.TickCount64;
        if (now - tracked.LastRefreshLogMs < 30_000) return;
        tracked.LastRefreshLogMs = now;
        ModLog.Info($"re-osculated vessel '{tracked.Id}' at t={time:F1} s ({time / 86400.0:F2} d): "
            + $"conic drift was {drift:E2} m, plan rebuilt from n-body state (refresh #{tracked.RefreshCount})");
    }

    /// <summary>Books a live excursion witnessed by the
    /// tick's stock-accumulated DeltaVelocityCci — the caller keeps the stock live
    /// state for this tick (staging the predictor would discard the delta-v) and the
    /// predictor reseeds from the committed state next staging tick. The cluster patch
    /// calls this per substep while a member burns, hence the 1/s log throttle.</summary>
    public void NoteLiveImpulse(VehicleUpdateState vehicleState, TrackedVessel tracked,
        double dvMagnitude)
    {
        tracked.MarkReseedPending();
        tracked.LastDvWitnessMs = Environment.TickCount64; // Rebase coast gate
        FlightPlans.TryGet(vehicleState.Id)?.MarkDiverged(); // plan snapshot: reality left the plan's world
        long now = Environment.TickCount64;
        if (now - tracked.LastBurnWitnessLogMs < 1000) return;
        tracked.LastBurnWitnessLogMs = now;
        ModLog.Info($"vessel '{vehicleState.Id}': live delta-v {dvMagnitude:E2} m/s witnessed within tick - "
            + "keeping the stock live state, predictor reseeds next tick");
    }

    /// <summary>Transfers propagation ownership to stock live physics. Reseeding is
    /// part of the transfer even when no impulse was measured: it prevents a later
    /// same-tick RecalculateFlightPlan from re-entering SOI authority using the
    /// pre-tick Freefall situation, and refreshes the predictor from committed truth
    /// when the vessel next returns to rails.</summary>
    internal void NoteLiveOwnership(VehicleUpdateState vehicleState, string reason)
    {
        TrackedVessel? tracked;
        lock (_gate) _tracked.TryGetValue(vehicleState.Id, out tracked);
        if (tracked is null || !tracked.IsSameVehicle(vehicleState.ReadOnlyVehicle)) return;
        tracked.MarkReseedPending();
    }

    /// <summary>Burn-time live display: the Seam 1 postfixes route here
    /// when stock physics owned the vessel this tick. Display-only — the tracked entry
    /// is looked up, never seeded, and the authoritative predictor is never touched
    /// (a vessel with no entry has no mod line to keep alive; ownership stays stock's).
    /// Never throws: a display failure must not reach the patch's Contain catch.</summary>
    public void OffRailsOverlay(VehicleUpdateState vehicleState)
    {
        try
        {
            TrackedVessel? tracked;
            lock (_gate) _tracked.TryGetValue(vehicleState.Id, out tracked);
            if (tracked is null) return;
            if (!tracked.IsSameVehicle(vehicleState.ReadOnlyVehicle)) return;
            tracked.BindUpdateState(vehicleState, offRails: true);
            tracked.LastSeenMs = Environment.TickCount64; // a long burn must not evict the entry
            bool dvWitnessed = vehicleState.UpdateData.NewKinematicMeasurements is { } measured
                && measured.DeltaVelocityCci != Brutal.Numerics.double3.Zero;
            if (dvWitnessed) tracked.LastDvWitnessMs = Environment.TickCount64; // Rebase coast gate
            TrajectoryOverlay.MaybeRebuildOffRails(vehicleState, tracked, dvWitnessed);
        }
        catch (Exception e)
        {
            ModLog.Warn($"off-rails overlay contained for '{vehicleState.Id}': {e.Message}");
        }
    }

    /// <summary>Value-type edge adapter for the KSA-free canary policy. Construction
    /// stores references only; the policy captures its lineage token before invoking
    /// any mutable game/config read. Generic constrained dispatch avoids boxing.</summary>
    private struct CommitCanaryProbe : IVesselRegistryCanaryProbe
    {
        private readonly TrackedVessel _tracked;
        private readonly Vehicle _vehicle;
        private readonly ModConfig _config;
        private Orbit _orbit;
        private StateVectors _committed;
        private double _time;

        internal CommitCanaryProbe(TrackedVessel tracked, Vehicle vehicle, ModConfig config)
        {
            _tracked = tracked;
            _vehicle = vehicle;
            _config = config;
            _orbit = default!;
            _committed = default;
            _time = double.NaN;
        }

        public VesselLifecycle.CommitCanaryEligibility CaptureAndClassify()
        {
            _orbit = _vehicle.Orbit;
            _committed = _orbit.StateVectors;
            _time = _committed.StateTime.Seconds();
            return VesselLifecycle.ClassifyCommitCanary(
                _tracked.ReseedPending,
                _vehicle.Props.Situation == Situation.Freefall,
                _tracked.IsSameVehicle(_vehicle),
                _time, _tracked.SeedTime);
        }

        public double CommitTime => _time;

        public double EvaluateResidual()
        {
            var expected = _tracked.EvaluateGameState(_orbit, _committed.StateTime);
            return (_committed.PositionCci - expected.PositionCci).Length();
        }

        public double ToleranceMeters => _config.CanaryToleranceMeters;
    }

    /// <summary>Runtime canary: committed on-rails states must match
    /// the predictor. Three consecutive misses => patches bypassed or frames broken =>
    /// mod-wide disable (fail safe, loudly).</summary>
    public void VerifyCommit(Vehicle vehicle)
    {
        TrackedVessel? tracked;
        lock (_gate)
        {
            if (_tracked.TryGetValue(vehicle.Id, out tracked))
                tracked.LastSeenMs = Environment.TickCount64; // commit surface = alive
        }
        if (tracked is null) return;
        // A pending reseed means the predictor is KNOWN stale (a within-tick live
        // excursion witnessed by DeltaVelocityCci —
        // NoteLiveImpulse): the committed state is deliberately the stock one until the reseed
        // lands next staging tick, so verifying against the stale predictor would book
        // bogus strikes toward the fatal disable.
        // The generic value-type policy captures the token before every snapshot,
        // eligibility, residual, tolerance, miss and completion operation. No
        // delegate or closure is allocated on this per-commit path.
        var probe = new CommitCanaryProbe(tracked, vehicle, config);
        var verification = VesselRegistryCanary.Verify(tracked.Canary, ref probe);
        CanaryCounter.Completion completion = verification.Completion;
        if (verification.Failure is { } failure)
        {
            HandleCanaryFailure(tracked, vehicle, completion, failure);
            return;
        }
        if (completion.Kind == CanaryCounter.CompletionKind.Discarded) return;
        double t = verification.Time;
        double residual = verification.Residual;
        bool miss = verification.Miss;
        tracked.LastCommitResidual = residual;
        if (completion.Kind == CanaryCounter.CompletionKind.Fatal)
        {
            ModServices.FatalDisable(
                $"commit canary: '{vehicle.Id}' committed state deviates {residual:E2} m from the mod "
                + "trajectory for 3 consecutive comparable commits - Seam 1 patches bypassed "
                + "(inlining?) or frame math broken");
            return;
        }
        if (miss)
            ModLog.Warn($"canary MISS '{vehicle.Id}': residual {residual:E2} m at t={t:F1} s "
                + $"(strike {completion.Strikes}/3)");
        else if (_canaryTelemetry.Classify(vehicle.Id, Environment.TickCount64) != Patches.RailsTelemetry.Line.None)
            ModLog.Info($"canary '{vehicle.Id}': residual {residual:E3} m, conic drift "
                + $"{tracked.LastConicDrift:E3} m at t={t:F1} s ({t / 86400.0:F2} d)");
    }

    private static void HandleCanaryFailure(
        TrackedVessel tracked,
        Vehicle vehicle,
        CanaryCounter.Completion completion,
        Exception failure)
    {
        if (completion.Kind == CanaryCounter.CompletionKind.Fatal)
        {
            ModServices.FatalDisable(
                $"commit canary: '{vehicle.Id}' probe failed for "
                + $"{completion.ProbeFailures} consecutive comparable commits - "
                + $"canary unavailable: {failure}");
            return;
        }

        ModLog.Error($"commit canary itself failed for '{vehicle.Id}': {failure.Message} "
            + $"(failure {completion.ProbeFailures}/"
            + $"{tracked.Canary.FatalConsecutiveProbeFailures})");
    }

    public IEnumerable<string> Describe()
    {
        lock (_gate)
            return _tracked.Values
                .Select(t => $"vessel {t.Id}: rails, canary {t.LastCommitResidual:E1} m, "
                    + $"conic drift {t.LastConicDrift:F0} m, refreshes {t.RefreshCount}")
                .ToArray();
    }

    /// <summary>Resolves the live vehicle instance bound during seed or reseed.
    /// `Program.ControlledVehicle` supplies only the id because its object reference
    /// is not authoritative across loads. Returns null for untracked or collected
    /// instances.</summary>
    public Vehicle? TryGetLiveVehicle(string id)
    {
        lock (_gate)
            return _tracked.TryGetValue(id, out var tracked) && tracked.TryGetVehicle(out var vehicle)
                ? vehicle
                : null;
    }

    /// <summary>Captures the exact tracked entry, bound vehicle and predictor lineage
    /// only while the committed vessel is continuously owned by n-body rails.</summary>
    internal bool TryCaptureRailsAuthority(
        Vehicle vehicle,
        out RailsAuthoritySnapshot snapshot,
        out PredictorAuthorityPolicy.Reason reason)
    {
        lock (_gate)
        {
            bool entryPresent = _tracked.TryGetValue(vehicle.Id, out var tracked);
            Vehicle bound = null!;
            bool boundAvailable = entryPresent && tracked!.TryGetVehicle(out bound);
            TrajectoryPredictor? predictor = entryPresent ? tracked!.Predictor : null;
            reason = PredictorAuthorityPolicy.Classify(new(
                EntryPresent: entryPresent,
                SameEntry: true,
                BoundVehicleAvailable: boundAvailable,
                SameVehicle: boundAvailable && ReferenceEquals(bound, vehicle),
                ReseedPending: entryPresent && tracked!.ReseedPending,
                CommittedFreefall: vehicle.Props.Situation == Situation.Freefall,
                PredictorAvailable: predictor is not null,
                SamePredictor: true));
            if (PredictorAuthorityPolicy.IsAuthoritative(reason))
            {
                snapshot = new RailsAuthoritySnapshot(
                    vehicle.Id, tracked!, bound!, predictor!);
                return true;
            }
        }
        snapshot = default;
        return false;
    }

    /// <summary>Reads a predictor only while the strict authority token belongs to the
    /// supplied live vehicle before and after the potentially extending state query.</summary>
    internal bool TryReadAuthoritativePredictorState(
        Vehicle vehicle,
        double time,
        out StateVector state,
        out PredictorAuthorityPolicy.Reason reason)
    {
        state = default;
        if (!TryCaptureRailsAuthority(vehicle, out var snapshot, out reason))
            return false;
        if (!snapshot.Tracked.TryPredictorStateAt(snapshot.Lineage, time, out state))
        {
            ValidateRailsAuthority(snapshot, out reason);
            return false;
        }
        if (!ValidateRailsAuthority(snapshot, out reason))
        {
            state = default;
            return false;
        }
        return true;
    }

    /// <summary>Revalidates every component of a previously captured authority token
    /// against the registry's current entry and the vehicle's committed situation.</summary>
    internal bool ValidateRailsAuthority(
        RailsAuthoritySnapshot snapshot,
        out PredictorAuthorityPolicy.Reason reason)
    {
        lock (_gate)
            return ValidateRailsAuthorityUnsafe(snapshot, out reason);
    }

    /// <summary>Executes one small side effect only while a captured authority token
    /// is still current. Lock order is registry then rails, matching GetOrSeed; pending
    /// publication uses the same rails gate, so it cannot land between validation and
    /// the side effect.</summary>
    internal bool TryExecuteWithRailsAuthority<TResult>(
        RailsAuthoritySnapshot snapshot,
        Func<TResult> execute,
        out TResult result,
        out PredictorAuthorityPolicy.Reason reason)
    {
        lock (_gate)
        {
            PredictorAuthorityPolicy.Reason observed = default;
            bool executed = RailsAuthoritySynchronization.TryExecute(
                snapshot.Tracked.Rails.Gate,
                () => ValidateRailsAuthorityUnsafe(snapshot, out observed),
                execute,
                out result);
            reason = observed;
            return executed;
        }
    }

    private bool ValidateRailsAuthorityUnsafe(
        RailsAuthoritySnapshot snapshot,
        out PredictorAuthorityPolicy.Reason reason)
    {
        bool entryPresent = _tracked.TryGetValue(snapshot.VesselId, out var tracked);
        bool sameEntry = entryPresent && ReferenceEquals(tracked, snapshot.Tracked);
        Vehicle bound = null!;
        bool boundAvailable = entryPresent && tracked!.TryGetVehicle(out bound);
        TrajectoryPredictor? predictor = entryPresent ? tracked!.Predictor : null;
        reason = PredictorAuthorityPolicy.Classify(new(
            EntryPresent: entryPresent,
            SameEntry: sameEntry,
            BoundVehicleAvailable: boundAvailable,
            SameVehicle: boundAvailable && ReferenceEquals(bound, snapshot.Vehicle),
            ReseedPending: entryPresent && tracked!.ReseedPending,
            CommittedFreefall: boundAvailable
                && bound!.Props.Situation == Situation.Freefall,
            PredictorAvailable: predictor is not null,
            SamePredictor: predictor is not null
                && ReferenceEquals(predictor, snapshot.Lineage)));
        return PredictorAuthorityPolicy.IsAuthoritative(reason);
    }

    /// <summary>Flight-plan panel seam: the tracked entry under an id, for read-only
    /// predictor access (frame-burn conversions run a display clone off it — the
    /// authoritative predictor itself is never mutated by consumers). Null when the id
    /// is untracked or has no current authoritative predictor (the panel then degrades
    /// to VLF-only authoring).</summary>
    public TrackedVessel? TryGetTracked(string id)
    {
        lock (_gate)
            return _tracked.TryGetValue(id, out var tracked) ? tracked : null;
    }

    public void Reset()
    {
        lock (_gate)
        {
            _tracked.Clear();
            _pendingSidecar = null; // stale exact states must not leak into a later load
        }
    }
}

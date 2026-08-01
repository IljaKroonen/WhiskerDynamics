using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;
using System.Runtime.CompilerServices;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod.Soi;

namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Honest orbit lines (EVERY tracked vessel, not just the
/// controlled one): replaces a tracked vessel's cached orbit-line points with a
/// sampled n-body polyline (display only — the AUTHORITATIVE predictor never gets plan
/// burns), reusing the game's own cached-points rendering: the map orbit line IS where
/// the vessel actually goes. Two-line display: the main batch is
/// the ACTUAL no-burn trajectory; when in-window burns exist a second PLANNED batch
/// (burns folded in as impulses, sampled from the first burn) publishes to
/// OverlayBuffer's planned slot and draws in the planned-burn color through the
/// earliest burn's plan orbit (VesselLinePatch). Capture/enqueue is reserved per
/// vessel; the worker's keyed latest-wins slot replaces older pending work once a
/// newer capture reaches cadence. Ordinary refreshes still back off by the previous
/// pass's measured cost; the burns-changed log line keys
/// per vessel too (<see cref="TrackedVessel.LastOverlayBurnsApplied"/>). The
/// sampled batch is also PUBLISHED through <see cref="OverlayBuffer"/>
/// (per-vessel slots) and re-staged at the draw site immediately before the read
/// (<see cref="Patches.VesselLinePatch"/>), so stock plan
/// recalculations that overwrite the cache are dead stores by ordering — no
/// warp flicker. Every staged buffer is padded to stock length
/// (<see cref="OverlayKernel.StockPointBufferLength"/>), so length-assuming stock
/// readers can never go out of range and e&gt;=1 escape legs are staged too — the
/// pad-to-2000 invariant makes an e&gt;=1 write gate unnecessary. Writing the cache
/// from an update task matches stock precedent:
/// RecalculateFlightPlan / RecalculateBurnPlan regenerate cached points inside
/// VehicleUpdateTask.Run (VehicleUpdateState.cs:365/379). Failures are contained
/// LOCALLY (warn + panel note): a display overlay must never book a vessel containment,
/// let alone throw into the game.</summary>
public static class TrajectoryOverlay
{
    private const long RebuildPeriodMs = 1000;
    /// <summary>Floor on plan-edit bypass frequency: single edits still feel instant,
    /// while a continuously mutating plan (a stock-editor drag whose reconcile
    /// recaptures every rebuild) degrades to this cadence instead of a capture per
    /// physics tick.</summary>
    private const long PlanBypassPeriodMs = 250;
    private const long AnalysisSnapshotRetryPeriodMs = 250;
    private const long WarnPeriodMs = 30_000;
    private const double SecondsPerDay = 86400;
    private const double AnalysisPropagationChunkSeconds = 30 * SecondsPerDay;

    private readonly record struct AnalysisRequestCapture(
        bool Enabled, double StartOffsetSeconds, double SpanSeconds, int Version)
    {
        internal double RequiredHorizonDays => Enabled
            ? (StartOffsetSeconds + SpanSeconds) / SecondsPerDay
            : 0;
    }

    internal readonly record struct RebuildHorizons(
        double DisplayDays, double AnalysisDays);

    /// <summary>The map line remains bounded by its display/plan window, while orbit
    /// analysis may consume any wider rails coverage already integrated.</summary>
    internal static RebuildHorizons ResolveRebuildHorizons(
        double configuredDisplayDays, double displayAvailableDays,
        double? planEndSeconds, double nowSeconds,
        double integratedAvailableDays, double analysisRequiredDays)
    {
        double displayDays = FlightPlans.EffectiveHorizonDays(
            configuredDisplayDays, displayAvailableDays, planEndSeconds, nowSeconds);
        double analysisDays = double.IsFinite(analysisRequiredDays)
            ? Math.Min(Math.Max(0.0, analysisRequiredDays),
                Math.Max(0.0, integratedAvailableDays))
            : 0.0;
        return new(displayDays, analysisDays);
    }

    internal readonly record struct PredictionCaptures<T>(
        T Display, T? Analysis) where T : class;

    /// <summary>Captures display coverage first and treats wider analyser coverage as
    /// optional. A long incremental analyser-snapshot build therefore cannot suppress
    /// an ordinary line refresh that the already-published display snapshot can serve.</summary>
    internal static PredictionCaptures<T>? TryCapturePredictionContexts<T>(
        Func<double, double, T?> captureDisplay,
        Func<double, double, T?> captureAnalysis,
        double displayFrom, double displayTo,
        double analysisFrom, double analysisTo,
        bool analysisEnabled) where T : class
    {
        T? display = captureDisplay(displayFrom, displayTo);
        if (display is null) return null;

        T? analysis = null;
        if (analysisEnabled && analysisTo > analysisFrom)
        {
            analysis = analysisFrom >= displayFrom && analysisTo <= displayTo
                ? display
                : captureAnalysis(analysisFrom, analysisTo);
        }
        return new(display, analysis);
    }

    /// <summary>Identifies a request version that has not reached analysis handoff.
    /// Consumers use this state both to poll its snapshot and to make the eventual
    /// ready handoff urgent. Repeated passes then respect the analysis-specific
    /// duration-aware cooldown.</summary>
    internal static bool AnalysisRequestNeedsUrgentCapture(
        bool enabled, int requestVersion, int lastAdmittedVersion) =>
        enabled && requestVersion != lastAdmittedVersion;

    /// <summary>While the rails worker prepares a missing analysis snapshot, retry
    /// capture at a bounded cadence. A changed request still gets one immediate
    /// attempt, but that attempt consumes its own producer-side bypass even though no
    /// analysis job can yet be admitted.</summary>
    internal static bool AnalysisSnapshotAttemptDue(
        bool enabled, int requestVersion, int lastAdmittedVersion,
        int lastAttemptVersion, long nowMs, long lastAttemptMs,
        long retryPeriodMs = AnalysisSnapshotRetryPeriodMs) =>
        AnalysisRequestNeedsUrgentCapture(
            enabled, requestVersion, lastAdmittedVersion)
        && (requestVersion != lastAttemptVersion
            || nowMs - lastAttemptMs >= retryPeriodMs);

    /// <summary>Reserves analysis only when geometry has reached the second-stage
    /// handoff. Pending latest-wins geometry therefore carries no lease that a newer
    /// capture could discard before any analysis work is admitted.</summary>
    internal static TrackedVessel.OverlayAnalysisLease? TryBeginOverlayAnalysisHandoff(
        TrackedVessel tracked, bool analysisWorkEnabled,
        int requestVersion, long nowMs)
    {
        if (!analysisWorkEnabled) return null;
        bool urgent = AnalysisRequestNeedsUrgentCapture(
            enabled: true,
            requestVersion,
            tracked.LastOverlayAnalysisRequestVersion);
        return tracked.TryBeginOverlayAnalysis(requestVersion, nowMs, urgent);
    }

    /// <summary>Resolves the deepest SOI owner at the requested analysis epoch from
    /// the hierarchy root. Starting at the root instead of the overlay's captured
    /// parent makes a delayed request independent of any intervening stock/rails
    /// reparent and supports transitions across more than one hierarchy level.</summary>
    internal static string AnalysisBodyAtStart(string rootId,
        Vector3d vesselAbsolute,
        Func<string, IReadOnlyList<string>> childrenOf,
        Func<string, Vector3d> absolutePositionOf,
        Func<string, double> sphereOfInfluenceOf)
    {
        string owner = rootId;
        var visited = new HashSet<string>(StringComparer.Ordinal);
        while (visited.Add(owner))
        {
            string? containedChild = null;
            foreach (string child in childrenOf(owner))
            {
                double soi = sphereOfInfluenceOf(child);
                if (!(soi > 0) || !double.IsFinite(soi)) continue;
                if ((vesselAbsolute - absolutePositionOf(child)).Length() <= soi)
                {
                    containedChild = child;
                    break;
                }
            }
            if (containedChild is null) return owner;
            owner = containedChild;
        }
        throw new InvalidOperationException(
            $"SOI hierarchy contains a cycle at '{owner}'");
    }

    internal sealed record AnalysisSoiRelativeSeries(
        string Id, double SoiRadius, Vector3d[] VesselRelativePositions);

    internal readonly record struct AnalysisSoiTransition(
        double TimeSeconds, string NewBodyId);

    /// <summary>Returns the first transition away from a frozen analysis body.
    /// Each interval is swept as a relative chord, so a child flyby that enters and
    /// exits between two samples is detected even when both endpoints are outside.</summary>
    internal static AnalysisSoiTransition? FindFirstAnalysisSoiTransition(
        double[] times, Vector3d[] bodyRelativePositions, double bodySoi,
        string? parentBodyId,
        IReadOnlyList<AnalysisSoiRelativeSeries> children)
    {
        if (times.Length != bodyRelativePositions.Length)
            throw new ArgumentException("analysis SOI series lengths differ");
        foreach (var child in children)
            if (child.VesselRelativePositions.Length != times.Length)
                throw new ArgumentException(
                    $"analysis SOI series length differs for '{child.Id}'");

        var sweptChildren = new SoiReparentKernel.SweptCandidate[children.Count];
        for (int i = 0; i + 1 < times.Length; i++)
        {
            for (int child = 0; child < children.Count; child++)
            {
                var series = children[child];
                sweptChildren[child] = new(series.Id,
                    series.VesselRelativePositions[i],
                    series.VesselRelativePositions[i + 1],
                    series.SoiRadius);
            }
            if (SoiReparentKernel.FirstCrossing(
                    bodyRelativePositions[i], bodyRelativePositions[i + 1],
                    bodySoi, parentBodyId, sweptChildren) is not { } crossing)
                continue;
            double time = times[i]
                + (times[i + 1] - times[i]) * crossing.Fraction;
            return new(time, crossing.NewParentId);
        }
        return null;
    }

    private static AnalysisRequestCapture CaptureAnalysisRequest(string vesselId)
    {
        bool controlled = string.Equals(
            KSA.Program.ControlledVehicle?.Id, vesselId, StringComparison.Ordinal);
        double startOffset = 0, span = 0;
        int version = 0;
        bool enabled = controlled && Ui.OrbitAnalyserPanel.TryGetRequest(
            out startOffset, out span, out version);
        return enabled
            ? new(true, startOffset, span, version)
            : new(false, 0, 0, 0);
    }

    public static string LastNote { get; private set; } = "overlay idle";
    private static long _nextWarnMs;                // wall-clock throttle - no session reset
    private static int _activeLogged;               // one-shot: first successful rebuild

    private sealed class StageCache(object gate)
    {
        /// <summary>Stable for this Orbit across session-table replacements. Every
        /// Orbit.UpdateCachedPoints call enters the same monitor through its Harmony
        /// patch, making observe -> mod handoff one atomic transaction.</summary>
        internal readonly object Gate = gate;
        /// <summary>The latest genuine stock/conic cache observed immediately before
        /// an actual-trajectory overwrite. A managed copy is intentional: Orbit owns
        /// the MemoryOwner handed to UpdateCachedPoints, and its CachedPoints Span
        /// cannot outlive the read that produced this snapshot.</summary>
        internal OrbitPointCce[]? StockPoints;
        /// <summary>True from immediately before an actual-trajectory cache handoff
        /// until a genuine stock cache is observed or restored. It distinguishes a
        /// real empty stock cache from an empty/partial mod handoff.</summary>
        internal bool ActualCacheModOwned;
        internal OverlaySamples? Samples;
        internal string? ParentId;
        internal string? FrameLabel;
        internal double SimSeconds;
        internal double AnchorPeSeconds;
        internal StagingContext Context;
        internal double3 FirstPosition;
    }

    /// <summary>Serializes stage-cache table/generation handoffs, but not context
    /// construction or point copying. The rare stock-fallback lease holds it through
    /// the Harmony original so no worker can restage between restore and stock's read.</summary>
    private static readonly object StageCacheHandoffGate = new();
    private static readonly ConditionalWeakTable<Orbit, object> OrbitCacheGates = new();
    private static ConditionalWeakTable<Orbit, StageCache> _stageCaches = new();
    private static long _stageCacheGeneration;
    private static int _activeStageHandoffs;
    private static bool _stageCacheResetPending;

    /// <summary>Same-thread, idempotent ownership of one per-orbit gate followed by
    /// the global handoff gate. Harmony postfix and finalizer may both dispose it.</summary>
    private sealed class StockFallbackLease(StageCache cache) : IDisposable
    {
        private StageCache? _cache = cache;
        private readonly int _ownerThreadId = Environment.CurrentManagedThreadId;

        public void Dispose()
        {
            var held = _cache;
            if (held is null) return;
            if (Environment.CurrentManagedThreadId != _ownerThreadId)
                throw new InvalidOperationException(
                    "A vessel-line stock-fallback lease must be released on its acquiring thread.");
            _cache = null;
            System.Threading.Monitor.Exit(StageCacheHandoffGate);
            System.Threading.Monitor.Exit(held.Gate);
        }
    }

    /// <summary>Session statics sweep: re-arm the one-shot evidence line for the new
    /// session; wall-clock throttles stay. The per-vessel rebuild/burn-change stamps
    /// live on <see cref="TrackedVessel"/> and die with the registry on rebind.</summary>
    internal static void ResetSessionStatics()
    {
        System.Threading.Volatile.Write(ref _activeLogged, 0);
        System.Threading.Volatile.Write(ref _liveLogged, 0);
        System.Threading.Volatile.Write(ref _collisionLogged, 0);
        // Worker publication/staging takes the queue gate before the stage-cache
        // gate. Advance both session identities in that same order so reset cannot
        // deadlock a worker already admitted through PublishIfCurrent.
        OverlayWorker.ResetSessionStatics(() =>
        {
            lock (StageCacheHandoffGate)
            {
                _stageCacheResetPending = true;
                try
                {
                    while (_activeStageHandoffs != 0)
                        System.Threading.Monitor.Wait(StageCacheHandoffGate);
                    unchecked { _stageCacheGeneration++; }
                    _stageCaches = new ConditionalWeakTable<Orbit, StageCache>();
                }
                finally
                {
                    _stageCacheResetPending = false;
                    System.Threading.Monitor.PulseAll(StageCacheHandoffGate);
                }
            }
        });
        LastNote = "overlay idle";
    }

    internal static string LimitNote(OverlaySamples samples, bool plannedTruncated,
        bool plannedDynamicsLimited)
    {
        string actual = samples.DynamicsLimited
            ? " (trajectory ended at dynamics limit)"
            : samples.Truncated
            ? $" (cap hit: drew {(samples.DenseTimes[^1] - samples.SampleT0) / 86400.0:F1} d)"
            : "";
        string planned = plannedDynamicsLimited
            ? " (planned arc ended at dynamics limit)"
            : plannedTruncated ? " (cap hit: planned arc truncated)" : "";
        return actual + planned;
    }

    /// <summary>Called from the Seam 1 postfixes (single-vessel and cluster-follower)
    /// after the predictor state was staged. Every caller-supplied tracked vessel
    /// rebuilds with one queued/running pass per vessel and completion-aware cooldown.
    /// Context-change bypass: a fresh published batch whose staging context no longer holds
    /// bypasses the throttle and rebuilds immediately — for a frame-mode mismatch that
    /// caps the draw-site <see cref="LineRoute.Blink"/> at one tick; for an SOI parent
    /// change (SOI-independence: no blink — <see cref="Stage"/> re-anchors
    /// and keeps drawing) it restores exact new-parent payload semantics
    /// (parent-relative Cce and the conic-anchored TimeSincePe identity) within
    /// the same tick.</summary>
    public static void MaybeRebuild(VehicleUpdateState vehicleState, TrackedVessel tracked, SimTime now)
    {
        lock (tracked.OverlayCaptureGate)
            MaybeRebuildOnRailsCore(vehicleState, tracked, now);
    }

    private static void MaybeRebuildOnRailsCore(
        VehicleUpdateState vehicleState, TrackedVessel tracked, SimTime now)
    {
        bool reserved = false;
        OverlayBuffer.RebuildLease? activeLease = null;
        try
        {
            long bindingGeneration = ModServices.BindingGeneration;
            long overlayLineage = tracked.OverlayLineage;
            if (!ModServices.IsBindingCurrent(bindingGeneration, tracked.Rails)) return;
            // Parent resolved BEFORE the throttle: the context-aware bypass below
            // compares the published batch against the CURRENT parent. (A parentless
            // orbit returns without consuming the
            // throttle window, which is at worst a fresher stamp.)
            var currentOrbit = vehicleState.CurrentOrbit;
            if (currentOrbit.Parent is not Astronomical parentBody) return;

            long nowMs = Environment.TickCount64;
            double captureSimSeconds = Universe.GetElapsedSimTime().Seconds();
            // Context-aware throttle: when the last published batch
            // is still FRESH but was sampled against a context that no longer holds,
            // the rebuild runs immediately instead of waiting out the throttle. Frame
            // mode switched: the draw-site prefix blinks the line (LineRoute.Blink) —
            // the bypass caps that blink at one tick. SOI parent changed
            // (SOI-independence): the line KEEPS drawing (Stage re-anchors
            // the prior-parent batch by OverlayKernel.ParentShift), and the bypass
            // restores exact new-parent payload semantics within the same tick. A
            // STALE batch already routes Stock at the draw site, so it keeps the plain
            // 1 Hz throttle — that freshness gate also stops a never-republished
            // fossil (e.g. horizon configured off) from bypassing every tick for more
            // than the 5 s staleness window.
            bool contextChanged = false;
            var published = OverlayBuffer.Read(vehicleState.Id);
            if (published is not null
                && OverlayKernel.SamplesUsable(published, nowMs, captureSimSeconds))
            {
                string? activeLabel = FrameManager.Active?.Label;
                bool parentChanged = !string.Equals(published.ParentId, parentBody.Id, StringComparison.Ordinal);
                bool frameChanged = !string.Equals(published.FrameLabel, activeLabel, StringComparison.Ordinal);
                contextChanged = parentChanged || frameChanged;
                // Acceptance evidence, throttled to 1 s per event class per vessel
                // (LastOverlayBurnChangeLogMs pattern); the rebuild itself is never
                // throttled by these stamps.
                if (parentChanged && nowMs - tracked.LastOverlayParentChangeLogMs >= 1000)
                {
                    tracked.LastOverlayParentChangeLogMs = nowMs;
                    ModLog.Info($"overlay: immediate rebuild for '{vehicleState.Id}' (staging context "
                        + $"changed: parent '{published.ParentId}' -> '{parentBody.Id}')");
                }
                if (frameChanged && nowMs - tracked.LastOverlayFrameChangeLogMs >= 1000)
                {
                    tracked.LastOverlayFrameChangeLogMs = nowMs;
                    ModLog.Info($"overlay: immediate rebuild for '{vehicleState.Id}' (staging context "
                        + $"changed: frame '{published.FrameLabel ?? "inertial"}' -> '{activeLabel ?? "inertial"}')");
                }
            }
            // Plan-edit bypass: any planner edit (burn add/move/edit/remove, length
            // change, divergence, rebase) replaces the plan's snapshot or bumps its
            // version — the drawn feedback for it must not wait out the throttle.
            // Gated on the store-wide FlightPlans.EditStamp (one lock-free load), so
            // the per-tick staging cost stays a compare until some plan somewhere
            // changed; floored at PlanBypassPeriodMs so a continuously mutating plan
            // (a stock-editor drag reconciling every rebuild) degrades to that
            // cadence, never to a rebuild per physics tick. Compared against the
            // stamps the LAST ENQUEUED rebuild consumed (not the published batch);
            // a rate-limited or waiting-horizon edit falls back to the plain
            // cadence — never lost, at worst one RebuildPeriodMs late.
            FlightPlanModel? plan = null;
            bool planResolved = false;
            bool planChanged = false;
            long editStamp = FlightPlans.EditStamp;
            if (editStamp != tracked.LastSeenPlanEditStamp)
            {
                plan = FlightPlans.TryGet(vehicleState.Id);
                planResolved = true;
                planChanged = (!ReferenceEquals(plan, tracked.LastOverlayPlanRef)
                        || (plan?.Version ?? 0) != tracked.LastOverlayPlanVersion)
                    && nowMs - tracked.LastOverlayPlanBypassMs >= PlanBypassPeriodMs;
                if (planChanged && nowMs - tracked.LastOverlayPlanChangeLogMs >= 1000)
                {
                    tracked.LastOverlayPlanChangeLogMs = nowMs;
                    ModLog.Info($"overlay: immediate rebuild for '{vehicleState.Id}' (plan edited)");
                }
            }
            if (!planResolved) plan = FlightPlans.TryGet(vehicleState.Id);

            var config = ModServices.Config;
            double t0 = now.Seconds();
            var analysisRequest = CaptureAnalysisRequest(vehicleState.Id);
            // Actual-line horizon: a vessel with a plan predicts at least to the plan
            // end, while the config horizon stays the floor for every vessel. The
            // planned line gets its own hard plan-end cap during RebuildPlanned.
            // Display geometry clamps to the configured rails window ACTUALLY
            // integrated (quantized — see
            // OverlayKernel.QuantizeRailsWindow): while the worker grows a raised
            // preset chunk by chunk, the line grows with it — sampling past the
            // reached horizon would synchronously extend the ephemerides. Analysis
            // separately uses the wider integrated rails retention without changing
            // this displayed horizon.
            double railsAvailableDays = OverlayKernel.QuantizeRailsWindow(
                tracked.Rails.AvailableAheadDays(t0), config.RailsAheadDays);
            double integratedAvailableDays = analysisRequest.Enabled
                ? tracked.Rails.IntegratedAheadDays(t0)
                : railsAvailableDays;
            var horizons = ResolveRebuildHorizons(
                config.OverlayHorizonDays, railsAvailableDays, plan?.EndSeconds, t0,
                integratedAvailableDays, analysisRequest.RequiredHorizonDays);
            double horizon = t0 + horizons.DisplayDays * SecondsPerDay;
            double analysisHorizon = t0 + horizons.AnalysisDays * SecondsPerDay;
            if (horizon <= t0)
            {
                LastNote = "overlay waiting for the rails horizon";
                return;
            }
            bool analysisPending = AnalysisRequestNeedsUrgentCapture(
                analysisRequest.Enabled, analysisRequest.Version,
                tracked.LastOverlayAnalysisRequestVersion);
            bool analysisLoopDue = AnalysisSnapshotAttemptDue(
                analysisRequest.Enabled, analysisRequest.Version,
                tracked.LastOverlayAnalysisRequestVersion,
                tracked.LastOverlayAnalysisSnapshotAttemptVersion,
                nowMs, tracked.LastOverlayAnalysisSnapshotAttemptMs);
            RailsService.PredictionContext? attemptedAnalysisPrediction = null;
            bool analysisSnapshotReady = false;
            if (analysisLoopDue)
            {
                tracked.RecordOverlayAnalysisSnapshotAttempt(
                    analysisRequest.Version, nowMs);
                // Poll/queue the long snapshot independently of display capture. If
                // the display context already covers the interval, the normal capture
                // below reuses it and can hand off immediately.
                analysisSnapshotReady = analysisHorizon <= horizon;
                if (!analysisSnapshotReady)
                {
                    attemptedAnalysisPrediction =
                        tracked.Rails.TryCaptureAnalysisPredictionContext(
                            t0, analysisHorizon, analysisRequest.Version);
                    analysisSnapshotReady = attemptedAnalysisPrediction is not null;
                }
            }
            bool urgent = contextChanged || planChanged || analysisSnapshotReady;
            if (!tracked.TryBeginOverlayRebuild(nowMs, RebuildPeriodMs, urgent)) return;
            reserved = true;
            double captureFrom = t0;
            var planState = plan?.SnapshotState.Snapshot;
            if (planState is not null) captureFrom = Math.Min(captureFrom, planState.EpochSeconds);
            var predictionCaptures = TryCapturePredictionContexts(
                tracked.Rails.TryCapturePredictionContext,
                (from, to) => analysisPending
                    ? (analysisLoopDue ? attemptedAnalysisPrediction : null)
                    : tracked.Rails.TryCaptureAnalysisPredictionContext(
                        from, to, analysisRequest.Version),
                captureFrom, horizon, t0, analysisHorizon, analysisRequest.Enabled);
            if (predictionCaptures is not { } captures)
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            var prediction = captures.Display;
            if (!tracked.TryCaptureOverlayAnchor(
                    overlayLineage, t0, out var authorityLineage, out var capturedAnchor))
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            // Plan stamps consumed only once a job actually enqueues (below): an edit
            // arriving while the overlay waits for the rails horizon keeps its
            // pending state instead of spending its rebuild on the early return.
            tracked.LastOverlayPlanRef = plan;
            tracked.LastOverlayPlanVersion = plan?.Version ?? 0;
            if (planChanged) tracked.LastOverlayPlanBypassMs = nowMs;

            // CAPTURE phase (task thread, cheap): everything the rebuild needs from
            // live game state — burn list, engine scalars, seed predictors, scope
            // constants. The integration + dense sweep + publish run on the overlay
            // worker (a synchronous full-horizon rebuild here would be a rhythmic
            // physics-thread stall scaling with the dense budget).
            var scope = RebuildScope.Create(config, tracked, vehicleState.Id,
                parentBody, currentOrbit, t0, horizon, analysisHorizon, plan,
                plannedSeed: () => TrackedVessel.NewDisplayPredictorAt(
                    capturedAnchor, t0, prediction.Gravity),
                railsAheadDays: railsAvailableDays, prediction, captures.Analysis,
                capturedAnchor,
                nowMs, captureSimSeconds, analysisRequest);
            var stockBurnScan = ScanStockBurns(vehicleState, parentBody.Id, plan);
            PropulsionSource propulsion = plan?.PropulsionSource ?? PropulsionSource.MainEngines;
            var engine = ReadEngineScalars(vehicleState, propulsion);
            if (!ModServices.IsBindingCurrent(bindingGeneration, tracked.Rails)
                || !tracked.IsOverlayLineageCurrent(overlayLineage, authorityLineage))
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            int generation = OverlayWorker.CurrentGeneration;
            var rebuildLease = OverlayBuffer.BeginRebuildLease(
                vehicleState.Id, generation, nowMs);
            if (rebuildLease is null)
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            activeLease = rebuildLease;
            var job = new OverlayRebuildJob
            {
                VesselId = vehicleState.Id,
                Tracked = tracked,
                Scope = scope,
                Plan = plan,
                StockBurns = stockBurnScan.Burns,
                StockBurnParentsReady = stockBurnScan.PatchChainReady,
                Engine = engine,
                Propulsion = propulsion,
                // The ACTUAL trajectory's predictor (no planned burns), reused
                // across rebuilds while its coast still matches the authoritative
                // predictor — a per-rebuild full-horizon re-integration would be the
                // dominant cost. Resolved ON THE WORKER (factory, not eager): the
                // producer must never prune/extend a predictor a still-running job
                // may be sweeping — the worker is the reuse cache's only toucher.
                // Continuity (same coast lineage) licences batch sample-reuse.
                DisplayFactory = () =>
                {
                    var display = tracked.ActualDisplayPredictorAt(
                        capturedAnchor, t0, prediction.Gravity, overlayLineage,
                        authorityLineage, out bool continuous);
                    return (display, continuous);
                },
                AnchorState = () => capturedAnchor,
                StageOrbit = currentOrbit,
                Generation = generation,
                BindingGeneration = bindingGeneration,
                OverlayLineage = overlayLineage,
                AuthorityLineage = authorityLineage,
                CaptureSimSeconds = captureSimSeconds,
                RebuildLease = rebuildLease,
                OffRails = false,
                HorizonDays = horizons.DisplayDays,
            };
            if (!OverlayWorker.Enqueue(vehicleState.Id, generation,
                    () => ModServices.IsBindingCurrent(bindingGeneration, tracked.Rails)
                        && tracked.IsOverlayLineageCurrent(
                            overlayLineage, authorityLineage),
                    (superseded, publishIfCurrent) =>
                    {
                        job.IsSuperseded = superseded;
                        job.PublishIfCurrent = publishIfCurrent;
                        job.Run();
                    },
                    job.Discard))
            {
                job.Discard();
                activeLease = null;
                tracked.CancelOverlayRebuild();
            }
            else
            {
                tracked.CommitOverlayRebuild(nowMs);
                reserved = false;
                activeLease = null;
                tracked.LastSeenPlanEditStamp = editStamp;
            }
        }
        catch (Exception e)
        {
            if (reserved) tracked.CancelOverlayRebuild();
            if (activeLease is not null) OverlayBuffer.EndRebuildLease(activeLease);
            // Never-throw, contained LOCALLY: a cosmetic overlay failure must not trip
            // the global dynamics fault latch and pause otherwise healthy propagation.
            LastNote = $"overlay contained: {e.Message}";
            WarnThrottled($"overlay contained: {e}");
        }
    }

    /// <summary>Burn-time live display: called from the
    /// Seam 1 postfixes when stock physics owned the vessel this tick (Maneuvering,
    /// or a Freefall physics-bubble tick) — exactly the windows where
    /// <see cref="MaybeRebuild"/> never runs and batches would otherwise age out (a
    /// mid-burn map blackout). Publishes a FRESH actual batch from the vessel's LIVE committed
    /// state (repopulated from kinematics at the end of every full-physics tick —
    /// VehicleUpdateTask.cs:737-740/1292-1308). Full-window rebuilds run continuously:
    /// the active build finishes and publishes while newer captures coalesce into the
    /// next build. The line, hover, markers and burn nodes therefore stay alive
    /// mid-burn — in frame views too (poses come from the rails, which never pause).
    /// The prediction is deliberately the COAST-FROM-HERE orbit (no thrust model):
    /// that is the cut-the-burn feedback. The authoritative predictor is never
    /// touched (it stays ReseedPending until the burn-end reseed); the planned
    /// SNAPSHOT batch refreshes through the same call. Contained like MaybeRebuild:
    /// display failures must never book a containment.</summary>
    public static void MaybeRebuildOffRails(VehicleUpdateState vehicleState, TrackedVessel tracked,
        bool dvWitnessed)
    {
        lock (tracked.OverlayCaptureGate)
            MaybeRebuildOffRailsCore(vehicleState, tracked, dvWitnessed);
    }

    private static void MaybeRebuildOffRailsCore(VehicleUpdateState vehicleState,
        TrackedVessel tracked, bool dvWitnessed)
    {
        bool reserved = false;
        OverlayBuffer.RebuildLease? activeLease = null;
        try
        {
            long bindingGeneration = ModServices.BindingGeneration;
            long overlayLineage = tracked.OverlayLineage;
            if (!ModServices.IsBindingCurrent(bindingGeneration, tracked.Rails)) return;
            // A live delta-v moves reality off the plan's world the moment it flows.
            // Mark it before capture so the first thrust tick flips the planned line
            // into the frozen ghost.
            if (dvWitnessed) FlightPlans.TryGet(vehicleState.Id)?.MarkDiverged();

            var currentOrbit = vehicleState.CurrentOrbit;
            if (currentOrbit.Parent is not Astronomical parentBody) return;
            // Defensive display-only guard for an unsupported-parent race; the
            // authority layer faults such a catalog/parent mismatch.
            if (!tracked.Rails.IsModeled(parentBody.Id)) return;

            long nowMs = Environment.TickCount64;
            double captureSimSeconds = Universe.GetElapsedSimTime().Seconds();
            var config = ModServices.Config;
            var sv = vehicleState.CurrentStateVectors; // live post-physics state
            double t0 = sv.StateTime.Seconds();
            var analysisRequest = CaptureAnalysisRequest(vehicleState.Id);
            long editStamp = FlightPlans.EditStamp;
            FlightPlanModel? plan = FlightPlans.TryGet(vehicleState.Id);
            // Display and analysis horizons resolve independently, like the on-rails path.
            double railsAvailableDays = OverlayKernel.QuantizeRailsWindow(
                tracked.Rails.AvailableAheadDays(t0), config.RailsAheadDays);
            double integratedAvailableDays = analysisRequest.Enabled
                ? tracked.Rails.IntegratedAheadDays(t0)
                : railsAvailableDays;
            var horizons = ResolveRebuildHorizons(
                config.OverlayHorizonDays, railsAvailableDays, plan?.EndSeconds, t0,
                integratedAvailableDays, analysisRequest.RequiredHorizonDays);
            double horizon = t0 + horizons.DisplayDays * SecondsPerDay;
            double analysisHorizon = t0 + horizons.AnalysisDays * SecondsPerDay;
            if (horizon <= t0) return;
            if (!tracked.TryBeginContinuousOverlayRebuild()) return;
            reserved = true;
            double captureFrom = t0;
            var planState = plan?.SnapshotState.Snapshot;
            if (planState is not null) captureFrom = Math.Min(captureFrom, planState.EpochSeconds);
            var predictionCaptures = TryCapturePredictionContexts(
                tracked.Rails.TryCapturePredictionContext,
                (from, to) => tracked.Rails.TryCaptureAnalysisPredictionContext(
                    from, to, analysisRequest.Version),
                captureFrom, horizon, t0, analysisHorizon, analysisRequest.Enabled);
            if (predictionCaptures is not { } captures)
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            var prediction = captures.Display;
            tracked.LastOverlayPlanRef = plan;
            tracked.LastOverlayPlanVersion = plan?.Version ?? 0;

            // CAPTURE phase (task thread): the live seed conversion reads the
            // committed game state HERE; the coast integration + sweep run on the
            // overlay worker. The live predictor is fresh every cycle by necessity
            // (its seed is reality), so mid-burn the worker runs hot — off the
            // physics threads, which is the point. Snapshot upkeep anchors from the LIVE
            // state (the authoritative predictor is KNOWN stale off-rails).
            var live = tracked.NewLiveDisplayPredictor(
                currentOrbit, in sv, prediction.Gravity, out var liveSeed);
            var scope = RebuildScope.Create(config, tracked, vehicleState.Id,
                parentBody, currentOrbit, t0, horizon, analysisHorizon, plan,
                plannedSeed: () => TrackedVessel.NewDisplayPredictorAt(
                    liveSeed, t0, prediction.Gravity),
                railsAheadDays: railsAvailableDays, prediction, captures.Analysis, liveSeed,
                nowMs, captureSimSeconds, analysisRequest);
            var stockBurnScan = ScanStockBurns(vehicleState, parentBody.Id, plan);
            PropulsionSource propulsion = plan?.PropulsionSource ?? PropulsionSource.MainEngines;
            var engine = ReadEngineScalars(vehicleState, propulsion);
            if (!ModServices.IsBindingCurrent(bindingGeneration, tracked.Rails)
                || !tracked.IsOverlayLineageCurrent(overlayLineage))
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            int generation = OverlayWorker.CurrentGeneration;
            var rebuildLease = OverlayBuffer.BeginRebuildLease(
                vehicleState.Id, generation, nowMs);
            if (rebuildLease is null)
            {
                tracked.CancelOverlayRebuild();
                reserved = false;
                return;
            }
            activeLease = rebuildLease;
            var job = new OverlayRebuildJob
            {
                VesselId = vehicleState.Id,
                Tracked = tracked,
                Scope = scope,
                Plan = plan,
                StockBurns = stockBurnScan.Burns,
                StockBurnParentsReady = stockBurnScan.PatchChainReady,
                Engine = engine,
                Propulsion = propulsion,
                // Fresh live predictor, already seeded from this tick's committed
                // state on the task thread — exclusively this job's to integrate.
                // Never continuous: off-rails, reality moves every tick.
                DisplayFactory = () => (live, false),
                AnchorState = () => liveSeed,
                StageOrbit = currentOrbit,
                Generation = generation,
                BindingGeneration = bindingGeneration,
                OverlayLineage = overlayLineage,
                AuthorityLineage = null,
                CaptureSimSeconds = captureSimSeconds,
                RebuildLease = rebuildLease,
                OffRails = true,
            };
            if (!OverlayWorker.Enqueue(vehicleState.Id, generation,
                    () => ModServices.IsBindingCurrent(bindingGeneration, tracked.Rails)
                        && tracked.IsOverlayLineageCurrent(overlayLineage),
                    (superseded, publishIfCurrent) =>
                    {
                        job.IsSuperseded = superseded;
                        job.PublishIfCurrent = publishIfCurrent;
                        job.Run();
                    },
                    job.Discard))
            {
                job.Discard();
                activeLease = null;
                tracked.CancelOverlayRebuild();
            }
            else
            {
                tracked.CommitOverlayRebuild(nowMs);
                reserved = false;
                activeLease = null;
                tracked.LastSeenPlanEditStamp = editStamp;
            }
        }
        catch (Exception e)
        {
            if (reserved) tracked.CancelOverlayRebuild();
            if (activeLease is not null) OverlayBuffer.EndRebuildLease(activeLease);
            LastNote = $"overlay live contained: {e.Message}";
            WarnThrottled($"overlay live contained: {e}");
        }
    }

    private static int _liveLogged;      // one-shot: first off-rails live rebuild
    private static int _collisionLogged; // one-shot: first surface-frame collision cut

    /// <summary>One captured rebuild, executed on the overlay
    /// worker. Every field is either immutable, a value copy, or an
    /// object whose cross-thread access is already disciplined (predictors and
    /// tracked-entry overlay caches under single-worker writes;
    /// plan snapshots behind their own gate; body/gravity reads use the immutable
    /// prediction context). Newer same-vessel work replaces older pending captures;
    /// an active capture retains its ticket through completion so sustained producer
    /// cadence cannot prevent eventual publication.</summary>
    private sealed class OverlayRebuildJob
    {
        public required string VesselId { get; init; }
        public required TrackedVessel Tracked { get; init; }
        public required RebuildScope Scope { get; init; }
        public required FlightPlanModel? Plan { get; init; }
        public required List<PlanSnapshotBurn> StockBurns { get; init; }
        public required bool StockBurnParentsReady { get; init; }
        public required EngineScalars Engine { get; init; }
        public required PropulsionSource Propulsion { get; init; }
        /// <summary>The ACTUAL batch's predictor, resolved ON THE WORKER: the
        /// reusable no-burn display predictor on rails (reuse cache is
        /// worker-touched only), the pre-seeded live coast predictor off them.
        /// Continuous = the predictor IS the previous rebuild's trajectory (same
        /// coast lineage) — the licence to reuse the previous sampled batch.</summary>
        public required Func<(TrajectoryPredictor Display, bool Continuous)> DisplayFactory { get; init; }
        /// <summary>Snapshot reconcile anchor source (authoritative predictor on
        /// rails, the captured live seed off them).</summary>
        public required Func<StateVector> AnchorState { get; init; }
        /// <summary>Patch-0 orbit at capture: the worker refreshes the staged
        /// 2000-point pick buffer once per rebuild — stock's own CachedPoints
        /// readers (ground track, hidden-orbit
        /// windows) rely on that unconditional per-rebuild refresh.</summary>
        public required Orbit StageOrbit { get; init; }
        /// <summary>Session generation at capture (OverlayWorker.CurrentGeneration):
        /// a sweep between capture and run makes this job a no-op, and a sweep
        /// racing publication is rejected atomically by OverlayBuffer.</summary>
        public required int Generation { get; init; }
        public required long BindingGeneration { get; init; }
        public required long OverlayLineage { get; init; }
        public required TrajectoryPredictor? AuthorityLineage { get; init; }
        /// <summary>Universe epoch sampled by the producer during capture. State T0
        /// is intentionally not used as queue age because committed vehicle state
        /// may trail Universe at high warp.</summary>
        public required double CaptureSimSeconds { get; init; }
        public required OverlayBuffer.RebuildLease RebuildLease { get; init; }
        public required bool OffRails { get; init; }
        public double HorizonDays { get; init; }    // on-rails note
        public Func<bool> IsSuperseded { get; set; } = static () => false;
        public Func<Action, bool> PublishIfCurrent { get; set; } = static action =>
        {
            action();
            return true;
        };

        public void Run()
        {
            long startedMs = Environment.TickCount64;
            try
            {
                // High warp can advance more than the 600-s interaction/reuse age
                // while one sweep is computing. The trajectory is absolute and may
                // still cover the new epoch, so worker admission/publication rejects
                // only malformed/future captures; generation, ticket, and predictor
                // lineage gates below reject genuinely obsolete work.
                bool CaptureEpochValid() => OverlayKernel.CaptureEpochValid(
                    CaptureSimSeconds, Universe.GetElapsedSimTime().Seconds());
                bool LineageUsable() =>
                    ModServices.IsBindingCurrent(BindingGeneration, Tracked.Rails)
                    && Tracked.IsOverlayLineageCurrent(OverlayLineage, AuthorityLineage);
                bool Obsolete() => IsSuperseded() || !CaptureEpochValid() || !LineageUsable();
                bool PublishWhenCurrent(Action action) =>
                    CaptureEpochValid() && LineageUsable() && PublishIfCurrent(action);
                if (Generation != OverlayWorker.CurrentGeneration
                    || !CaptureEpochValid() || !LineageUsable()) return;
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                StateVector? cachedAnchor = null;
                StateVector CurrentAnchor() => cachedAnchor ??= AnchorState();
                if (Plan is not null
                    && !PublishWhenCurrent(() =>
                        ReconcileSnapshot(Plan, Tracked, StockBurns, Scope.T0, CurrentAnchor,
                            Scope.ParentBodyId, Engine, Propulsion, StockBurnParentsReady))) return;
                // The ACTUAL trajectory (no planned burns — where the vessel goes
                // if nothing fires). SAMPLE-REUSE: a coasting vessel's trajectory
                // does not change between rebuilds, but re-sweeping it re-phases
                // every vertex along the same path, which can make the chords crawl
                // at close zoom. While the
                // predictor lineage is continuous and the context unchanged, the
                // previous batch's future vertices are retained while its now-anchored
                // payloads are refreshed; an age cap forces a real resample before the
                // vessel outruns its dense near window or the horizon slides visibly short.
                var (display, continuous) = DisplayFactory();
                StateVector anchorState = CurrentAnchor();
                double periodHint = Scope.PeriodHintAt(display, Scope.T0, anchorState);
                var previous = OverlayBuffer.Read(VesselId);
                var reusable = OffRails ? null : previous;
                var samples = continuous && reusable is not null
                    && Scope.CanReuseActual(reusable, periodHint)
                        ? Scope.ReuseActualBatch(
                            reusable, display, CaptureSimSeconds, Obsolete)
                        : Scope.SampleBatch(display, Scope.T0, periodHint, CaptureSimSeconds,
                            shouldStop: Obsolete,
                            anchorStateOverride: anchorState);
                if (Obsolete()) return;
                if (!PublishWhenCurrent(() => OverlayBuffer.PublishGeometry(samples))) return;
                QueueAnalysis();
                // During live physics, retain/restamp only already-proven planned
                // geometry; defer all new planned integration until rails resume.
                if (Obsolete()) return;
                var (burnsApplied, plannedTruncated, plannedDynamicsLimited) = OffRails
                    ? RestampOrClearPlannedOffRails(samples, PublishWhenCurrent)
                    : RebuildPlanned(Scope, Plan, StockBurns, Engine, samples,
                        Obsolete, PublishWhenCurrent);
                // Planned folding can itself take seconds near a high-order body.
                // Keep the actual geometry visible after the rebuild lease ends by
                // stamping completion, rather than the earlier actual-sweep instant.
                if (Obsolete()) return;
                samples = samples with { SampleWallMs = Environment.TickCount64 };
                if (!PublishWhenCurrent(() => OverlayBuffer.PublishGeometry(samples))) return;
                if (Generation != OverlayWorker.CurrentGeneration)
                {
                    // A session sweep raced this rebuild: whichever side of the
                    // buffer clear the publishes landed on, pull them back out — a
                    // pre-load trajectory must never draw in the new session.
                    return;
                }
                // Refresh the staged pick buffer once per rebuild (stock plan
                // recalcs refill it with conic points between the draw-site's
                // per-frame restages, and lines gated out of drawing — ShowOrbit
                // off — never reach that restage; ground track still reads the
                // cache). Off-render-thread staging matches stock precedent:
                // stock itself regenerates these caches off the render thread.
                if (Obsolete()) return;
                if (!PublishWhenCurrent(() =>
                        StageWorkerBatch(samples, StageOrbit, Generation))) return;
                stopwatch.Stop();
                Note(samples, burnsApplied, plannedTruncated, plannedDynamicsLimited,
                    stopwatch.ElapsedMilliseconds);
            }
            catch (OperationCanceledException)
            {
                // Cooperative supersession.
            }
            catch (Exception e)
            {
                // Includes the accepted reseed race: a reseed landing between
                // capture and run moves the authoritative predictor's start past
                // this job's t0 and the seed reads throw — one skipped display
                // cycle at a burn/teleport boundary; the next capture heals it.
                LastNote = $"overlay {(OffRails ? "live " : "")}contained: {e.Message}";
                WarnThrottled($"overlay {(OffRails ? "live " : "")}contained: {e}");
            }
            finally
            {
                OverlayBuffer.EndRebuildLease(RebuildLease);
                Tracked.CompleteOverlayRebuild(
                    OffRails ? 0 : Environment.TickCount64 - startedMs);
            }
        }

        private void QueueAnalysis()
        {
            var lease = TryBeginOverlayAnalysisHandoff(
                Tracked, Scope.AnalysisWorkEnabled,
                Scope.AnalysisRequestVersion, Environment.TickCount64);
            if (lease is null) return;
            var analysisJob = new OverlayAnalysisJob
            {
                VesselId = VesselId,
                Tracked = Tracked,
                Scope = Scope,
                Generation = Generation,
                BindingGeneration = BindingGeneration,
                OverlayLineage = OverlayLineage,
                AuthorityLineage = AuthorityLineage,
                Lease = lease,
            };
            bool accepted = OverlayAnalysisWorker.Enqueue(
                VesselId, Generation,
                // Recheck the display ticket at the handoff. Reseed cancellation
                // revokes it before cancelling analysis, closing the gap between an
                // actual-line publication and this second-stage enqueue.
                () => !IsSuperseded()
                    && ModServices.IsBindingCurrent(BindingGeneration, Tracked.Rails)
                    && Tracked.IsOverlayLineageCurrent(
                        OverlayLineage, AuthorityLineage),
                (superseded, publishIfCurrent) =>
                {
                    analysisJob.IsSuperseded = superseded;
                    analysisJob.PublishIfCurrent = publishIfCurrent;
                    analysisJob.Run();
                },
                analysisJob.Discard);
            if (accepted)
                Tracked.RecordOverlayAnalysisAdmission(lease);
            else
                analysisJob.Discard();
        }

        /// <summary>Releases resources owned by a capture that left the pending
        /// latest-wins slot without ever reaching <see cref="Run"/>.</summary>
        public void Discard()
        {
            OverlayBuffer.EndRebuildLease(RebuildLease);
        }

        private (int BurnsApplied, bool Truncated, bool DynamicsLimited) RestampOrClearPlannedOffRails(
            OverlaySamples actualSamples, Func<Action, bool> publishWhenCurrent)
        {
            var published = OverlayBuffer.ReadPlanned(VesselId);
            var (snapshot, diverged) = Plan?.SnapshotState ?? (null, false);
            double actualVisibleEnd = actualSamples.DenseTimes.Length > 0
                ? actualSamples.DenseTimes[^1] : Scope.T0;
            bool sameContext = published is not null
                && string.Equals(published.ParentId, Scope.ParentBodyId, StringComparison.Ordinal)
                && OverlayKernel.ModeMatches(published.FrameLabel, Scope.ActiveFrame?.Label)
                && OverlayKernel.FrameAllowsPlannedRestamp(Scope.ActiveFrame)
                && Equals(published.MarkerCacheKey, Scope.MarkerCacheKey)
                && OverlayKernel.PlannedOffRailsHorizonCompatible(
                    published.HorizonSeconds, Plan?.EndSeconds ?? double.NaN)
                && OverlayKernel.PlannedBranchConnected(
                    published.FutureStartSeconds, actualVisibleEnd, published.HorizonSeconds);
            if (!OverlayKernel.PlannedOffRailsRestampAllowed(
                    snapshot, Tracked.LastPlannedSnapshot, sameContext))
            {
                if (!publishWhenCurrent(() => ClearPlanned(Scope)))
                    throw new OperationCanceledException();
                return (0, false, false);
            }

            var retained = published!;
            StateVector restampAnchor = diverged ? retained.AnchorState : AnchorState();
            var restamped = RestampPlannedBatch(retained, Scope.T0, Scope.TimeAtPe,
                restampAnchor, retained.MarkerCandidates, Scope.MarkerCacheKey,
                CaptureSimSeconds, Environment.TickCount64);
            if (!publishWhenCurrent(() => OverlayBuffer.PublishPlanned(restamped)))
                throw new OperationCanceledException();
            return (Tracked.LastPlannedBurnsApplied, retained.Truncated, retained.DynamicsLimited);
        }

        private void Note(OverlaySamples samples, int burnsApplied, bool plannedTruncated,
            bool plannedDynamicsLimited, long elapsedMs)
        {
            // "Cap hit" names the drawn extent when even the vessel sampler's
            // eight-points-per-revolution anti-aliasing floor exhausts the budget:
            // "how far does my line actually go" is the question
            // this surface answers. The two flags surface INDEPENDENTLY — actual
            // truncation is exactly the regime where the planned line truncates
            // too, so one masking the other would hide the planned line's early
            // end on its only diagnostic surface. Same note on the off-rails
            // branch: the live path samples the same bounded horizon.
            string capNote = LimitNote(samples, plannedTruncated, plannedDynamicsLimited);
            if (OffRails)
            {
                LastNote = $"overlay live: {samples.DenseTimes.Length} pts{capNote}, {burnsApplied} burns, "
                    + $"rebuild {elapsedMs} ms";
                if (System.Threading.Interlocked.CompareExchange(ref _liveLogged, 1, 0) == 0)
                    ModLog.Info("overlay live path active: off-rails prediction publishing for "
                        + $"'{VesselId}' at t={Scope.T0:F1} s");
                return;
            }
            // A collision-cut batch legitimately ends before the horizon without the
            // budget-truncation flag — say so, or "why does my line end early" has an
            // unreported cause on its one diagnostic surface. Also the feature's
            // acceptance evidence (one-shot log below).
            OverlayMarker? impact = null;
            foreach (var marker in samples.Markers)
                if (marker.Kind == OverlayMarkerKind.Collision) { impact = marker; break; }
            LastNote = $"overlay: {samples.DenseTimes.Length} pts"
                + $" ({samples.PointCount} staged){capNote}"
                + $"{(impact is not null ? " (cut at surface impact)" : "")}, "
                + $"{burnsApplied} burns{(OverlayBuffer.ReadPlanned(VesselId) is not null ? " (planned line on)" : "")}, "
                + $"horizon {HorizonDays:F1} d, "
                + $"rebuild {elapsedMs} ms"
                + $", {OverlayBuffer.PublishedCount} vessel(s) published";
            if (impact is not null
                && System.Threading.Interlocked.CompareExchange(ref _collisionLogged, 1, 0) == 0)
                ModLog.Info($"overlay: surface-frame collision cut active for '{VesselId}' — line "
                    + $"ends at t={impact.TimeSeconds:F1} s ('{impact.Label}')");
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"overlay active: '{VesselId}' map orbit line is the n-body polyline "
                    + $"({LastNote}) at t={Scope.T0:F1} s");
            long nowMs = Environment.TickCount64;
            if (Tracked.LastOverlayBurnsApplied < 0)
            {
                Tracked.LastOverlayBurnsApplied = burnsApplied; // first rebuild: baseline, not a change
            }
            else if (burnsApplied != Tracked.LastOverlayBurnsApplied
                && nowMs - Tracked.LastOverlayBurnChangeLogMs >= 1000)
            {
                // Replan evidence: node placed / removed / dragged across the window
                // edge or flown past - the next rebuild reflects it here.
                Tracked.LastOverlayBurnChangeLogMs = nowMs;
                ModLog.Info($"overlay: planned burns folded into the PLANNED line changed "
                    + $"{Tracked.LastOverlayBurnsApplied} -> {burnsApplied} for '{VesselId}' at t={Scope.T0:F1} s");
                Tracked.LastOverlayBurnsApplied = burnsApplied;
            }
        }
    }

    private sealed class OverlayAnalysisJob
    {
        public required string VesselId { get; init; }
        public required TrackedVessel Tracked { get; init; }
        public required RebuildScope Scope { get; init; }
        public required int Generation { get; init; }
        public required long BindingGeneration { get; init; }
        public required long OverlayLineage { get; init; }
        public required TrajectoryPredictor? AuthorityLineage { get; init; }
        public required TrackedVessel.OverlayAnalysisLease Lease { get; init; }
        public Func<bool> IsSuperseded { get; set; } = static () => false;
        public Func<Action, bool> PublishIfCurrent { get; set; } = static action =>
        {
            action();
            return true;
        };

        public void Run()
        {
            bool analysisStarted = false;
            bool recordCooldown = false;
            long analysisStartedMs = 0;
            try
            {
                bool Obsolete() => IsSuperseded()
                    || !ModServices.IsBindingCurrent(BindingGeneration, Tracked.Rails)
                    || !Tracked.IsOverlayLineageCurrent(
                        OverlayLineage, AuthorityLineage);
                if (Generation != OverlayWorker.CurrentGeneration || Obsolete()) return;
                analysisStarted = true;
                analysisStartedMs = Environment.TickCount64;
                var result = Scope.ComputeAnalysis(Obsolete);
                if (!result.Completed) return;
                if (Obsolete())
                {
                    recordCooldown = false;
                    return;
                }
                bool published = false;
                bool lineageAccepted = PublishIfCurrent(() =>
                {
                    // The analysis worker's current-ticket gate is shared with
                    // per-vessel cancellation. Recheck the captured authority inside
                    // that atomic callback; a reseed either makes this false before
                    // publication or cancels after publication and strips the payload.
                    if (ModServices.IsBindingCurrent(BindingGeneration, Tracked.Rails)
                        && Tracked.IsOverlayLineageCurrent(
                            OverlayLineage, AuthorityLineage))
                        published = OverlayBuffer.TryPublishAnalysis(
                            VesselId, Scope.AnalysisRequestVersion,
                            result.Report, result.Reason, Generation);
                });
                recordCooldown = lineageAccepted && published;
            }
            catch (OperationCanceledException)
            {
                // Request change, session reset, or vessel-lineage replacement.
            }
            catch (Exception e)
            {
                LastNote = $"orbit analysis contained: {e.Message}";
                WarnThrottled($"orbit analysis contained: {e}");
            }
            finally
            {
                if (analysisStarted)
                    Tracked.FinishOverlayAnalysis(
                        Lease, recordCooldown, startedMs: analysisStartedMs);
                else
                    Tracked.CancelOverlayAnalysis(Lease);
            }
        }

        public void Discard() => Tracked.CancelOverlayAnalysis(Lease);
    }

    /// <summary>One coherent stock burn-list read per rebuild (the UI thread may be
    /// mutating the plan — a live Burn is never re-read later in the pass).</summary>
    private sealed record StockBurnScan(
        List<PlanSnapshotBurn> Burns, bool PatchChainReady);

    private static StockBurnScan ScanStockBurns(
        VehicleUpdateState vehicleState, string fallbackParentId,
        FlightPlanModel? plan)
    {
        var burnPlan = vehicleState.ReadOnlyVehicle.FlightComputer.BurnPlan;
        int burnCount = burnPlan.BurnCount;
        bool cleanAtStart = !burnPlan.FlightPlansOutOfDate;
        var captured = new List<(double TimeSeconds, Vector3d DeltaVVlf,
            string? ResolvedParentId)>(burnCount);
        for (int i = 0; i < burnCount; i++)
        {
            if (!burnPlan.TryGetBurn(i, out Burn? burn) || burn is null) continue;
            double time = burn.Time.Seconds();
            string? resolvedParentId = cleanAtStart
                ? PlannedBurnConverter.ExistingBurnParentId(
                    vehicleState.ReadOnlyVehicle, burn, patchChainReady: true)
                : null;
            captured.Add((time, FrameAdapter.ToCore(burn.DeltaVVlf),
                resolvedParentId));
        }
        // Whole-scan readiness barrier. Dirty-at-start that clears mid-scan and
        // clean-at-start that becomes dirty both wait one more capture; no mixture of
        // old and rebuilt FlightPlans may complete a pending suffix.
        bool patchChainReady = cleanAtStart && !burnPlan.FlightPlansOutOfDate;
        var stockBurns = new List<PlanSnapshotBurn>(captured.Count);
        foreach (var burn in captured)
        {
            string? snapshotParentId = plan?.SnapshotParentFromCapture(
                burn.TimeSeconds, burn.ResolvedParentId, patchChainReady);
            string? usableResolvedParentId = patchChainReady
                ? burn.ResolvedParentId : null;
            stockBurns.Add(new PlanSnapshotBurn(burn.TimeSeconds,
                burn.DeltaVVlf,
                snapshotParentId ?? usableResolvedParentId ?? fallbackParentId,
                // Carry an unchanged burn's display vector through a wholesale
                // recapture; the dv-match guard drops it when the components moved.
                plan?.SnapshotDisplayDvFor(burn.TimeSeconds, burn.DeltaVVlf)));
        }
        return new StockBurnScan(stockBurns, patchChainReady);
    }

    /// <summary>Finite-burn scalars, one coherent read per rebuild (same surface as
    /// the burn scan): the FC's own totals — TotalMassPropsBody.Mass and the
    /// VehicleConfig engine sums that the executor's duration formula reads
    /// (FlightComputer.cs:722-730). Engineless/degenerate values simply fail
    /// EngineScalars.Usable and the display fold stays impulsive.</summary>
    private static EngineScalars ReadEngineScalars(VehicleUpdateState vehicleState,
        PropulsionSource source) =>
        ReadEngineScalars(vehicleState.ReadOnlyVehicle, source);

    /// <summary>THE engine-scalar read (the panel's Rebase capture shares it —
    /// two hand-copied field reads is how a game rename splits the rebuild and
    /// rebase captures). Torn-read guard: ReadUpdatedVehicleConfiguration publishes
    /// a fresh VehicleConfigInfo FIRST and only then accumulates the engine totals
    /// into it (FlightComputer.cs:216-220), so an off-thread read during a staging/
    /// docking config rebuild can see half-summed totals — and a snapshot capture
    /// would FREEZE them (a diverged ghost keeps them until Rebase). Two agreeing
    /// consecutive reads bound that window to ~nothing; a disagreement returns
    /// default (not Usable): impulsive for this rebuild, corrected on the next.</summary>
    internal static EngineScalars ReadEngineScalars(Vehicle vehicle,
        PropulsionSource source = PropulsionSource.MainEngines)
    {
        try
        {
            var first = ReadEngineScalarsOnce(vehicle, source);
            var second = ReadEngineScalarsOnce(vehicle, source);
            return first == second ? first : default;
        }
        catch
        {
            // Staging/docking can replace VehicleConfig and its module-state list
            // between reads. This telemetry is optional: contain the mismatched
            // generation and keep the fold impulsive until the next stable capture.
            return default;
        }
    }

    private static EngineScalars ReadEngineScalarsOnce(Vehicle vehicle, PropulsionSource source)
    {
        var flightComputer = vehicle.FlightComputer;
        if (source == PropulsionSource.RcsForward)
            return ReadForwardRcsScalarsOnce(vehicle, flightComputer.TotalMassPropsBody.Mass);
        var config = flightComputer.VehicleConfig;
        return new EngineScalars(
            flightComputer.TotalMassPropsBody.Mass,
            config.TotalEngineExhaustVelocity,
            config.TotalEngineVacuumMassFlowRate);
    }

    /// <summary>Vacuum performance of active, fueled RCS controllers mapped to
    /// forward translation (+body X). Net axial force (not scalar nozzle thrust) is
    /// divided by total selected flow, so canted jets pay their real propellant cost.
    /// Dynamic state supplies KSA's control-map and propellant-availability decisions.</summary>
    private static EngineScalars ReadForwardRcsScalarsOnce(Vehicle vehicle, double massKg)
    {
        if (!vehicle.Parts.States.TryGetTypeList<ThrusterController, ThrusterControllerState,
                ThrusterControllerGlobalState, EmptyStruct>(out var states))
            return default;

        var jets = new List<(double AxialForceNewtons, double MassFlowRate)>();
        var seenCores = new HashSet<RocketCore>();
        var seenNozzles = new HashSet<RocketNozzle>();
        foreach (var thruster in vehicle.FlightComputer.VehicleConfig.Thrusters)
        {
            var state = states.GetState(thruster);
            if (!state.IsPropellantAvailable
                || (state.ControlMap & ThrusterMapFlags.TranslateForward) == ThrusterMapFlags.None)
                continue;
            foreach (var core in thruster.Cores)
            {
                if (!seenCores.Add(core)) continue;
                var conditions = core.ComputeConditions(1f);
                foreach (var nozzle in core.Rocket.Nozzles)
                {
                    if (!seenNozzles.Add(nozzle)) continue;
                    var performance = nozzle.ComputePerformance(in conditions, 0f);
                    var thrustDirection = (-nozzle.ExhaustDirectionAsmb).Transform(
                        floatQuat.Pack(nozzle.Parent.Asmb2VehicleAsmb));
                    jets.Add((performance.GetTotalThrust() * thrustDirection.X,
                        performance.MassFlowRate));
                }
            }
        }
        return RcsPerformanceKernel.FromSelectedJets(massKg, jets);
    }

    /// <summary>Plan snapshot upkeep. NOT diverged, the snapshot tracks reality:
    /// recapture when the stock burn list changed (writer-mirrored edits land here
    /// as a no-op; stock-editor edits are adopted the same way) and re-anchor when
    /// the anchor aged past OverlayKernel.SnapshotAnchorMaxAgeSeconds — the anchor
    /// is interchangeable with the live state while not diverged, and a fresh one
    /// bounds how far the ghost must integrate once reality departs. DIVERGED with a
    /// captured snapshot, reconcile is OFF — that IS the snapshot promise: stock
    /// consuming the executing burn, or anything else reality does, must not reshape
    /// the frozen plan until the user rebases. DIVERGED with NO snapshot, including
    /// plans created off rails, captures once a clean patch-chain scan is available so
    /// the planned line can appear without freezing fallback parents.
    /// <paramref name="anchorState"/> supplies the world to anchor in: the
    /// authoritative predictor on rails, the live seed off-rails.</summary>
    private static void ReconcileSnapshot(FlightPlanModel plan, TrackedVessel tracked,
        IReadOnlyList<PlanSnapshotBurn> stockBurns, double t0,
        Func<StateVector> anchorState, string parentId, EngineScalars engine,
        PropulsionSource propulsion, bool patchChainReady)
    {
        // The source and its scalar read are one capture pair. A UI switch after
        // capture makes this job stale; never relabel captured scalars with the
        // plan's new source. The edit stamp schedules a fresh post-completion job.
        if (plan.PropulsionSource != propulsion) return;
        // ONE gated pair read (the worker races the panel's Rebase and the seams'
        // MarkDiverged — two property reads could tear against SetSnapshot's atomic
        // pair write), and a monotonicity guard: an anchor NEWER than this job's
        // capture t0 means a Rebase already superseded these inputs.
        var (snapshot, diverged) = plan.SnapshotState;
        if (!OverlayKernel.SnapshotReconcileAllowed(
                patchChainReady, diverged, snapshot is not null)) return;
        if (snapshot is not null && snapshot.EpochSeconds > t0) return;
        bool burnsMatch = plan.SnapshotBurnsMatch(stockBurns);
        bool anchorFresh = snapshot is not null
            && t0 >= snapshot.EpochSeconds
            && t0 - snapshot.EpochSeconds <= OverlayKernel.SnapshotAnchorMaxAgeSeconds;
        // Engine drift recapture: mass changes WITHOUT a burn-list edit (staging,
        // undocking, fuel transfer) — waiting out the hourly anchor age would draw
        // the finite arc ~2x long after undocking a heavy tug. Exact comparison is
        // safe: the scalars are copied floats, bit-stable while nothing changed.
        // Deliberately one-sided — a not-Usable current read (engineless, or the
        // torn-read guard tripping for one pass) is ABSENCE OF EVIDENCE and never
        // forces a recapture by itself; a genuinely engineless vessel converges at
        // the hourly re-anchor instead.
        bool engineChanged = engine.Usable && snapshot?.Engine != engine;
        bool propulsionChanged = snapshot?.PropulsionSource != propulsion;
        if (burnsMatch && anchorFresh && !engineChanged && !propulsionChanged) return;
        // Compare-and-set commit: a Rebase/edit/MarkDiverged that landed since the
        // pair read wins over this capture (its inputs are stale by definition).
        if (!plan.TryReconcileSnapshot(snapshot,
                PlanSnapshot.Capture(t0, anchorState(), parentId, stockBurns, engine,
                    propulsion), propulsion))
            return;
        long nowMs = Environment.TickCount64;
        string? multiParentSignature = FlightPlanModel.SnapshotParentSignature(stockBurns);
        bool sameEvidencePlan = ReferenceEquals(tracked.LastSnapshotEvidencePlanRef, plan);
        // Stock-editor drags still receive the 1 s flood floor. A genuinely new
        // ordered parent pattern (or the same pattern on a replacement plan) bypasses
        // it once so a fast SOI-basis transition cannot lose its only searchable
        // evidence. Record EVERY successful capture's state before throttling: a
        // single-parent transition must re-arm that same multi-parent pattern.
        bool evidenceDue = FlightPlanModel.SnapshotParentEvidenceDue(nowMs,
                tracked.LastSnapshotLogMs, multiParentSignature,
                tracked.LastSnapshotMultiParentSignature, sameEvidencePlan);
        tracked.LastSnapshotEvidencePlanRef = plan;
        tracked.LastSnapshotMultiParentSignature = multiParentSignature;
        if (!evidenceDue) return;
        tracked.LastSnapshotLogMs = nowMs;
        ModLog.Info($"plan snapshot {(burnsMatch ? "re-anchored" : "captured")} for '{tracked.Id}' "
            + $"at t={t0:F1} s ({stockBurns.Count} burns)"
            + FlightPlanModel.SnapshotParentEvidence(stockBurns));
    }

    /// <summary>Builds/publishes (or clears) the PLANNED batch from the plan
    /// snapshot. Returns the folded burn count and the published batch's truncation
    /// flag for the caller's status note. Contained SEPARATELY from the caller: a
    /// ghost failure (for example, an anchor behind the retained ephemeris window)
    /// rails keep-behind window) clears the planned line and must never cost the
    /// ACTUAL line its rebuild.</summary>
    private static (int BurnsApplied, bool Truncated, bool DynamicsLimited) RebuildPlanned(RebuildScope scope,
        FlightPlanModel? plan, IReadOnlyList<PlanSnapshotBurn> stockBurns,
        EngineScalars engine, OverlaySamples actualSamples, Func<bool> shouldStop,
        Func<Action, bool> publishIfCurrent)
    {
        try
        {
            return RebuildPlannedCore(scope, plan, stockBurns, engine, actualSamples,
                shouldStop, publishIfCurrent);
        }
        catch (OperationCanceledException)
        {
            var prior = OverlayBuffer.ReadPlanned(scope.VesselId);
            return (scope.Tracked.LastPlannedBurnsApplied,
                prior?.Truncated ?? false, prior?.DynamicsLimited ?? false);
        }
        catch (Exception e)
        {
            publishIfCurrent(() => ClearPlanned(scope));
            WarnThrottled($"planned line contained for '{scope.VesselId}': {e}");
            return (0, false, false);
        }
    }

    private static (int BurnsApplied, bool Truncated, bool DynamicsLimited) RebuildPlannedCore(RebuildScope scope,
        FlightPlanModel? plan, IReadOnlyList<PlanSnapshotBurn> stockBurns,
        EngineScalars engine, OverlaySamples actualSamples, Func<bool> shouldStop,
        Func<Action, bool> publishIfCurrent)
    {
        if (shouldStop()) throw new OperationCanceledException();
        var tracked = scope.Tracked;
        // ONE gated pair read: the fold's branch decisions (frozen ghost vs live
        // seed) must never see a Rebase's snapshot with the pre-Rebase diverged
        // flag or vice versa.
        var (snapshot, diverged) = plan?.SnapshotState ?? (null, false);
        if (plan is null && stockBurns.Count > 0)
        {
            // Plan-less vessels keep the pre-snapshot behavior — burns authored
            // through the STOCK editor (or surviving a load that lost the plan) fold
            // on the live trajectory every rebuild: a transient not-diverged capture,
            // never stored (its anchor state is unused on the live-seed path).
            snapshot = PlanSnapshot.Capture(scope.T0, default, scope.ParentBodyId, stockBurns, engine);
        }
        // No snapshot (no plan and no stock burns; fresh plan before its first
        // capture), or a diverged anchor in the future (a load rewound time under a
        // persisted plan): nothing honest to draw — the panel offers Rebase.
        if (snapshot is null || (diverged && snapshot.EpochSeconds > scope.T0))
        {
            if (!publishIfCurrent(() => ClearPlanned(scope)))
                throw new OperationCanceledException();
            return (0, false, false);
        }

        bool actualCoverageLimited = actualSamples.Truncated || actualSamples.WorkLimited
            || actualSamples.DynamicsLimited;
        double actualVisibleEnd = actualSamples.DenseTimes.Length > 0
            ? actualSamples.DenseTimes[^1] : scope.T0;
        double actualCoverageEnd = double.IsFinite(actualSamples.CoverageEndSeconds)
            ? actualSamples.CoverageEndSeconds : actualVisibleEnd;
        double plannedHorizon = OverlayKernel.PlannedHorizonSeconds(scope.Horizon,
            plan?.EndSeconds ?? double.NaN, actualCoverageEnd, actualCoverageLimited);
        // Only a resource-imposed endpoint belongs in the geometry identity. Normal
        // requested horizons roll with now for planless/long-plan cases; keying those
        // absolute values would defeat the fixed-window planned-restamp policy and
        // force a full fold/sample every cadence.
        double coverageLimitedEnd = actualCoverageLimited ? plannedHorizon : double.NaN;
        double? start = OverlayKernel.SnapshotSampleStart(snapshot.BurnTimes, scope.T0, plannedHorizon, diverged);
        if (start is null || plannedHorizon - start.Value <= 0.0)
        {
            if (!publishIfCurrent(() => ClearPlanned(scope)))
                throw new OperationCanceledException();
            return (0, false, false);
        }

        // Restamp fast path (OverlayKernel.PlannedResampleDue): the planned geometry
        // is deterministic between snapshot changes, so the previous batch republishes
        // with a fresh wall stamp instead of re-sampling the whole window.
        var published = OverlayBuffer.ReadPlanned(scope.VesselId);
        if (published is not null && !OverlayKernel.PlannedResampleDue(
                sameSnapshot: ReferenceEquals(tracked.LastPlannedSnapshot, snapshot)
                    || (!diverged && tracked.LastPlannedSnapshot is { } previousSnapshot
                        && previousSnapshot.GeometryMatches(snapshot)),
                sameDiverged: tracked.LastPlannedDiverged == diverged,
                diverged: diverged,
                startSeconds: start.Value,
                lastStartSeconds: tracked.LastPlannedStart,
                sameParent: string.Equals(published.ParentId, scope.ParentBodyId, StringComparison.Ordinal),
                sameFrame: OverlayKernel.ModeMatches(published.FrameLabel, scope.ActiveFrame?.Label)
                    && OverlayKernel.FrameAllowsPlannedRestamp(scope.ActiveFrame),
                sameGeometryInputs: tracked.LastPlannedGeometryKey is { } geometryKey
                    && geometryKey.Equals(scope.PlannedGeometryKeyFor(coverageLimitedEnd))
                    && ReferenceEquals(tracked.LastPlannedGravity, scope.Prediction.Gravity)
                    && !OverlayKernel.PlannedCoverageExpansionDue(
                        tracked.LastPlannedRailsCoverageDays,
                        scope.RailsAheadDays, scope.DesiredPlannedHorizonDays)
                    // A fresh actual sweep may have acquired an earlier collision
                    // cut while the immutable planned geometry stayed otherwise
                    // cache-compatible. Never let the cheap restamp bypass the same
                    // branch-on-actual-line invariant enforced after a full fold.
                    && OverlayKernel.PlannedBranchConnected(
                        published.FutureStartSeconds, actualVisibleEnd, plannedHorizon),
                t0Seconds: scope.T0,
                batchT0Seconds: published.SampleT0,
                batchEndSeconds: published.DenseTimes.Length > 0
                    ? published.DenseTimes[^1] : double.NaN))
        {
            // Geometry is deterministic, but its now-anchored payloads are not.
            // Refresh the cheap time arrays and markers over the immutable geometry;
            // otherwise keeping them honest would require the full adaptive sweep
            // every few simulation seconds (effectively every job under warp).
            StateVector anchorState;
            if (diverged && tracked.PlannedGhost is { } ghost)
            {
                anchorState = ghost.StateAt(scope.T0);
            }
            else
            {
                anchorState = scope.CurrentAnchor;
            }
            // Deterministic marker candidates ride the geometry. Only a genuinely new
            // selected-target trajectory invalidates closest-approach candidates; the
            // common celestial-target and reused-vessel-geometry paths take no dense
            // scan and no per-sample rails Gate acquisitions here.
            IReadOnlyList<OverlayMarker> candidates = published.MarkerCandidates;
            object markerKey = scope.MarkerCacheKey;
            if (!Equals(published.MarkerCacheKey, markerKey))
            {
                var recomputed = ComputeMarkers(scope.Rails, scope.Prediction, scope.ActiveFrame,
                    scope.ParentBodyId, scope.T0, scope.MarkerWorkEnabled, scope.Target,
                    published.DenseTimes, published.DensePositionsCce,
                    published.Times, published.PositionsCce, published.PointCount);
                var collision = published.MarkerCandidates
                    .Where(marker => marker.Kind == OverlayMarkerKind.Collision).ToArray();
                if (collision.Length == 0)
                {
                    candidates = recomputed;
                }
                else
                {
                    var combined = new List<OverlayMarker>(collision.Length + recomputed.Count);
                    combined.AddRange(collision);
                    combined.AddRange(recomputed);
                    candidates = combined;
                }
            }
            var restamped = RestampPlannedBatch(
                published, scope.T0, scope.TimeAtPe, anchorState,
                candidates, markerKey, actualSamples.CaptureSimSeconds,
                Environment.TickCount64);
            if (!publishIfCurrent(() => OverlayBuffer.PublishPlanned(restamped)))
                throw new OperationCanceledException();
            tracked.LastPlannedSnapshot = snapshot;
            tracked.LastPlannedGravity = scope.Prediction.Gravity;
            return (tracked.LastPlannedBurnsApplied, published.Truncated, published.DynamicsLimited);
        }

        // A burn without a resolved patch parent uses the snapshot anchor when
        // diverged and the current parent otherwise.
        string foldParentId = diverged
            ? snapshot.AnchorParentId ?? scope.ParentBodyId
            : scope.ParentBodyId;
        // Diverged ghost predictor CACHE: a fresh anchored predictor would re-integrate
        // anchor->now on every resample — unbounded under warp while un-rebased — so
        // the folded ghost is kept on the tracked entry and only re-created when the
        // snapshot changes or a captured burn enters the (grown) horizon. Finite-burn
        // margin: a burn whose node was inside the previous horizon but whose centered
        // window overran it folded as an impulse — widen the re-fold trigger behind
        // that horizon by the tank-empty time (mass/ṁ, the hard upper bound on any
        // single burn's duration) so it re-expands once the horizon clears its cutoff.
        double ghostRefoldMargin = scope.FiniteBurnSliceSeconds > 0 && snapshot.Engine is { } ghostEngine
            ? ghostEngine.MassKg / ghostEngine.MassFlowRate
            : 0.0;
        TrajectoryPredictor planned;
        int burnsApplied;
        double sampleStart = start.Value;
        if (diverged
            && ReferenceEquals(tracked.PlannedGhostSnapshot, snapshot)
            && ReferenceEquals(tracked.PlannedGhostGravity, scope.Prediction.Gravity)
            && tracked.PlannedGhost is { } cachedGhost
            && !HasBurnBetween(snapshot.BurnTimes,
                tracked.PlannedGhostFoldHorizon - ghostRefoldMargin, plannedHorizon))
        {
            planned = cachedGhost;
            burnsApplied = tracked.PlannedGhostBurnsApplied;
        }
        else
        {
            // NOT diverged, the fold seeds from the same world as this pass's ACTUAL
            // batch (scope.PlannedSeed: authoritative on rails, live coast state
            // off-rails) and keeps the strictly-future window — the two-line
            // display rule. DIVERGED, the ghost is the plan's own
            // world: seed at the anchor, fold everything captured after it (burns
            // already flown in reality still fold — in the plan's world they happen).
            planned = diverged
                ? TrackedVessel.NewDisplayPredictorAt(
                    snapshot.State, snapshot.EpochSeconds, scope.Prediction.Gravity)
                : scope.PlannedSeed();
            double foldStart = OverlayKernel.SnapshotFoldStart(diverged, snapshot.EpochSeconds, scope.T0);
            // Finite-burn estimation: the DISPLAY fold discretizes
            // each burn into the FC's centered thrust arc when the snapshot carries
            // usable engine scalars (frozen at capture — the diverged ghost keeps the
            // plan's world's mass). Authoring folds stay impulsive: the VLF dv they
            // write is defined against stock's impulsive plan chain.
            FiniteBurnFold? finite = scope.FiniteBurnSliceSeconds > 0 && snapshot.Engine is { } burnEngine
                ? new FiniteBurnFold(burnEngine, scope.FiniteBurnSliceSeconds, scope.FiniteBurnMaxSlices)
                : null;
            burnsApplied = OverlayKernel.FoldSnapshotBurns(planned, snapshot.Burns,
                snapshot.BurnTimes, foldStart, plannedHorizon, foldParentId,
                (basisParentId, burnTime) =>
                {
                    if (shouldStop()) throw new OperationCanceledException();
                    return scope.Prediction.GetAbsolute(basisParentId, burnTime);
                }, WarnThrottled, finite, out double earliestBurnStart);
            // The finite arc starts at IGNITION (node − T/2): sample from there, or
            // the thrust arc's first half never draws and the planned line's first
            // vertex sits mid-burn, visibly off the actual line. The window rule
            // guarantees earliestBurnStart > foldStart. Diverged ghosts sample from
            // t0 regardless (start already covers the whole window — also why the
            // cached-ghost branch above needs no earliest bookkeeping).
            if (!diverged && !double.IsNaN(earliestBurnStart))
                sampleStart = Math.Min(sampleStart, earliestBurnStart);
            if (diverged)
            {
                tracked.PlannedGhost = planned;
                tracked.PlannedGhostSnapshot = snapshot;
                tracked.PlannedGhostGravity = scope.Prediction.Gravity;
                tracked.PlannedGhostBurnsApplied = burnsApplied;
                tracked.PlannedGhostFoldHorizon = plannedHorizon;
            }
            else
            {
                tracked.PlannedGhost = null;
                tracked.PlannedGhostSnapshot = null;
                tracked.PlannedGhostGravity = null;
            }
        }
        // Connectivity uses the TRUE first divergence. For a centered finite burn
        // this is ignition, which can lie before a collision even when the authored
        // node center lies after it; checking the node above would wrongly hide a
        // valid collision-avoidance trajectory.
        if (!OverlayKernel.PlannedBranchConnected(
                sampleStart, actualVisibleEnd, plannedHorizon))
        {
            if (!publishIfCurrent(() => ClearPlanned(scope)))
                throw new OperationCanceledException();
            return (0, false, false);
        }
        if (burnsApplied == 0 && !diverged)
        {
            // A fold that applied nothing could not have changed the trajectory:
            // planned == actual, nothing separate to draw. (Diverged, even a burn-free
            // ghost differs from reality — it publishes below: the plan's end
            // trajectory is the reference the pilot compares against.)
            if (!publishIfCurrent(() => ClearPlanned(scope)))
                throw new OperationCanceledException();
            return (0, false, false);
        }
        // Hint evaluated a nudge PAST the window start: an impulse at exactly t is not
        // applied to StateAt(t) until the predictor extends beyond it, so sampling the
        // hint AT the first burn's time would hand the PRE-burn period to the
        // anti-aliasing step cap. For a finite first burn the hint lands just past
        // IGNITION — still the pre-burn period, a deliberate conservative hint that
        // can only force more samples.
        double hintTime = Math.Min(sampleStart + 1.0, plannedHorizon);
        var batch = scope.SampleBatch(planned, sampleStart, scope.PeriodHintAt(planned, hintTime),
            actualSamples.CaptureSimSeconds,
            shouldStop: shouldStop, horizonOverride: plannedHorizon);
        if (shouldStop()) throw new OperationCanceledException();
        if (!publishIfCurrent(() => OverlayBuffer.PublishPlanned(batch)))
            throw new OperationCanceledException();
        tracked.LastPlannedSnapshot = snapshot;
        tracked.LastPlannedGravity = scope.Prediction.Gravity;
        tracked.LastPlannedDiverged = diverged;
        tracked.LastPlannedStart = start.Value;
        tracked.LastPlannedBurnsApplied = burnsApplied;
        tracked.LastPlannedGeometryKey = scope.PlannedGeometryKeyFor(coverageLimitedEnd);
        tracked.LastPlannedRailsCoverageDays = scope.RailsAheadDays;
        return (burnsApplied, batch.Truncated, batch.DynamicsLimited);
    }

    /// <summary>The production planned-restamp transaction: refresh every field whose
    /// meaning is anchored at now while retaining immutable sampled geometry by
    /// reference. Kept separate so the no-resample contract is directly testable.</summary>
    internal static OverlaySamples RestampPlannedBatch(
        OverlaySamples published, double t0Seconds, double timeAtPeSeconds,
        StateVector anchorState, IReadOnlyList<OverlayMarker> markerCandidates,
        object markerCacheKey, double captureSimSeconds, long wallMilliseconds)
    {
        var (sincePe, remaining) = OverlayKernel.RestampPayloadTimes(
            published.Times, t0Seconds, timeAtPeSeconds);
        return published with
        {
            SampleT0 = t0Seconds,
            SampleWallMs = wallMilliseconds,
            CaptureSimSeconds = captureSimSeconds,
            AnchorState = anchorState,
            TimesSincePe = sincePe,
            RemainingTimesTo = remaining,
            MarkerCandidates = markerCandidates,
            MarkerCacheKey = markerCacheKey,
            Markers = OverlayKernel.VisibleMarkers(markerCandidates, t0Seconds),
        };
    }
    private static bool HasBurnBetween(double[] times, double lo, double hi)
    {
        foreach (double t in times)
            if (t > lo && t <= hi) return true;
        return false;
    }

    private static void ClearPlanned(RebuildScope scope)
    {
        OverlayBuffer.ClearPlanned(scope.VesselId);
        var tracked = scope.Tracked;
        tracked.LastPlannedSnapshot = null;
        tracked.LastPlannedGravity = null;
        tracked.LastPlannedStart = double.NaN;
        tracked.LastPlannedBurnsApplied = 0;
        tracked.LastPlannedGeometryKey = null;
        tracked.LastPlannedRailsCoverageDays = 0.0;
        tracked.PlannedGhost = null;
        tracked.PlannedGhostSnapshot = null;
        tracked.PlannedGhostGravity = null;
    }

    /// <summary>Immutable worker capture of the controlled vessel's selected target.
    /// Celestial targets read the rails. Vessel targets interpolate their latest fresh
    /// ACTUAL overlay batch — the same coast the target line displays, including the
    /// live off-rails path — rather than the authoritative predictor, which is known
    /// stale during burns/live physics.</summary>
    private sealed record MarkerTarget(
        string Id, object Key, double StartTime, double EndTime,
        Func<double, StateVector?> StateAt)
    {
        public Vector3d? PositionAt(double time) => StateAt(time)?.Position;
    }

    private sealed record MarkerContextKey(bool Enabled, object? TargetKey);

    private static MarkerTarget? CaptureMarkerTarget(string vesselId, RailsService rails,
        RailsService.PredictionContext prediction, long nowWallMs, double nowSimSeconds)
    {
        if (KSA.Program.ControlledVehicle is not { } marker
            || !string.Equals(marker.Id, vesselId, StringComparison.Ordinal)
            || ModServices.Vessels is not { } vessels
            || vessels.TryGetLiveVehicle(marker.Id) is not { Target: Astronomical target }
            || string.Equals(target.Id, vesselId, StringComparison.Ordinal))
            return null;

        if (target is Vehicle targetVehicle)
        {
            var targetSamples = OverlayBuffer.ReadFresh(
                targetVehicle.Id, nowWallMs, nowSimSeconds);
            if (targetSamples is null) return null;
            // Target-fixed display sampling can reduce the target's own line to a
            // handful of points because it is stationary in that frame. The batch's
            // fresh physical anchor instead seeds a density-independent coast.
            var coast = new TrajectoryPredictor(
                prediction.Gravity, targetSamples.AnchorState, targetSamples.SampleT0,
                new IntegratorOptions { RelTol = 1e-9 });
            return new MarkerTarget(targetVehicle.Id, targetSamples.DenseTimes,
                targetSamples.SampleT0, double.PositiveInfinity, t =>
                    t < targetSamples.SampleT0 ? null : coast.StateAt(t));
        }
        return rails.CanEvaluate(target.Id)
            ? new MarkerTarget(target.Id, target.Id, double.NegativeInfinity, double.PositiveInfinity,
                t => prediction.GetAbsolute(target.Id, t))
            : null;
    }

    /// <summary>Everything one rebuild pass shares between its batches — resolved
    /// once per call by <see cref="MaybeRebuild"/> (on-rails, 1 Hz) or
    /// <see cref="MaybeRebuildOffRails"/> (continuous live rebuilds). <see cref="SampleBatch"/> is
    /// the ONE adaptive sweep every batch goes through.</summary>
    private sealed class RebuildScope
    {
        public required RailsService Rails { get; init; }
        public required RailsService.PredictionContext Prediction { get; init; }
        public required RailsService.PredictionContext? AnalysisPrediction { get; init; }
        public required TrackedVessel Tracked { get; init; }
        public required string VesselId { get; init; }
        /// <summary>Parent id string, not the Astronomical: the scope crosses to the
        /// overlay worker, and every consumer only ever keyed rails reads by id.</summary>
        public required string ParentBodyId { get; init; }
        public required double T0 { get; init; }
        public required StateVector CurrentAnchor { get; init; }
        public required double Horizon { get; init; }
        public required double AnalysisHorizon { get; init; }
        public required double ThetaMax { get; init; }
        public required ActiveFrameSnapshot? FrameSnapshot { get; init; }
        public FrameSpec? ActiveFrame => FrameSnapshot?.Spec;
        public required TerrainHeightSnapshot? Terrain { get; init; }
        public required double TimeAtPe { get; init; }
        // Raw horizon inputs (NaN when plan-less), part of the planned-batch restamp
        // identity: a plan-length or orbits-window edit must resample even while
        // paused, when neither t0 nor the derived horizon has moved.
        public required double PlanEnd { get; init; }
        public required double ConfigHorizonDays { get; init; }
        public required double ConfigRailsAheadDays { get; init; }
        public required double RailsAheadDays { get; init; }
        public required double DesiredPlannedHorizonDays { get; init; }
        /// <summary>Honest-density budget for the DENSE sweep (config overlay_max_points,
        /// clamped): the drawn polyline's cap; the staged stock buffer is a decimated
        /// subset (<see cref="OverlayKernel.DecimateIndices"/>).</summary>
        public required int MaxDensePoints { get; init; }
        // Finite-burn discretization knobs (config; slice seconds <= 0 = feature off).
        public required double FiniteBurnSliceSeconds { get; init; }
        public required int FiniteBurnMaxSlices { get; init; }
        public required bool MarkerWorkEnabled { get; init; }
        public required bool AnalysisWorkEnabled { get; init; }
        public required double AnalysisStartOffsetSeconds { get; init; }
        public required double AnalysisSpanSeconds { get; init; }
        public required int AnalysisRequestVersion { get; init; }
        public required MarkerTarget? Target { get; init; }
        /// <summary>Seed for the NOT-diverged planned fold — the same world this
        /// pass's ACTUAL batch is built from: the authoritative predictor on rails,
        /// the live coast state off-rails (where the authoritative predictor is
        /// KNOWN stale — a plan recreated after a burn must fold on reality, not on
        /// the pre-burn trajectory).</summary>
        public required Func<TrajectoryPredictor> PlannedSeed { get; init; }

        public PlannedGeometryKey PlannedGeometryKeyFor(double coverageLimitedEnd) => new(
            PlanEnd, coverageLimitedEnd, ConfigHorizonDays, ConfigRailsAheadDays,
            ThetaMax, MaxDensePoints,
            FiniteBurnSliceSeconds, FiniteBurnMaxSlices);

        public object MarkerCacheKey => new MarkerContextKey(
            MarkerWorkEnabled, Target?.Key);

        /// <summary>The ONE derivation of a rebuild's shared inputs — both entry
        /// points (on-rails 1 Hz, continuous off-rails work) resolve their scope here,
        /// so the sampling-fidelity constants can never fork between the line drawn
        /// during a burn and the line drawn a second after it.</summary>
        public static RebuildScope Create(ModConfig config, TrackedVessel tracked, string vesselId,
            Astronomical parentBody, Orbit currentOrbit, double t0, double horizon,
            double analysisHorizon,
            FlightPlanModel? plan, Func<TrajectoryPredictor> plannedSeed, double railsAheadDays,
            RailsService.PredictionContext prediction,
            RailsService.PredictionContext? analysisPrediction,
            StateVector currentAnchor,
            long nowWallMs, double captureSimSeconds,
            AnalysisRequestCapture analysisRequest)
        {
            double thetaMaxRad = OverlayKernel.SamplingThetaRadians(
                config.OverlayMaxTurnDeg);
            bool markerWorkEnabled = OverlayKernel.MarkerWorkEnabled(
                KSA.Program.ControlledVehicle?.Id, vesselId);
            bool analysisWorkEnabled = markerWorkEnabled
                && analysisRequest.Enabled
                && analysisPrediction is not null;
            ActiveFrameSnapshot? frameSnapshot = FrameManager.TryCaptureActive(out var captured)
                ? captured : null;
            TerrainHeightSnapshot? terrain = frameSnapshot?.Spec is
                { Kind: FrameKind.Surface } surface
                ? TerrainHeightReader.TryCapture(surface.PrimaryId) : null;
            return new()
            {
                PlannedSeed = plannedSeed,
                Rails = tracked.Rails,
                Prediction = prediction,
                AnalysisPrediction = analysisPrediction,
                Tracked = tracked,
                VesselId = vesselId,
                ParentBodyId = parentBody.Id,
                T0 = t0,
                CurrentAnchor = currentAnchor,
                Horizon = horizon,
                AnalysisHorizon = analysisHorizon,
                ThetaMax = thetaMaxRad,
                FrameSnapshot = frameSnapshot,
                Terrain = terrain,
                TimeAtPe = currentOrbit.TimeAtPeriapsis.Seconds(),
                PlanEnd = plan?.EndSeconds ?? double.NaN,
                ConfigHorizonDays = config.OverlayHorizonDays,
                ConfigRailsAheadDays = config.RailsAheadDays,
                // The AVAILABLE rails window (config target clamped to the horizon the
                // worker has reached), not the raw config. Planned geometry records it
                // only as a high-water coverage mark: startup catch-up grows at bounded
                // geometric steps, while warp-time down/up jitter cannot invalidate.
                RailsAheadDays = railsAheadDays,
                DesiredPlannedHorizonDays = plan is null
                    ? FlightPlans.EffectiveHorizonDays(config.OverlayHorizonDays,
                        config.RailsAheadDays, null, t0)
                    : Math.Min(config.RailsAheadDays,
                        Math.Max(0.0, (plan.EndSeconds - t0) / 86400.0)),
                MaxDensePoints = Math.Clamp(config.OverlayMaxPoints,
                    OverlayKernel.StockPointBufferLength, 262144),
                FiniteBurnSliceSeconds = config.FiniteBurnSliceSeconds,
                FiniteBurnMaxSlices =
                    OverlayKernel.OverlayFiniteBurnMaxSlices(config.FiniteBurnMaxSlices),
                MarkerWorkEnabled = markerWorkEnabled,
                AnalysisWorkEnabled = analysisWorkEnabled,
                AnalysisStartOffsetSeconds = analysisRequest.StartOffsetSeconds,
                AnalysisSpanSeconds = analysisRequest.SpanSeconds,
                AnalysisRequestVersion = analysisRequest.Version,
                Target = markerWorkEnabled
                    ? CaptureMarkerTarget(vesselId, tracked.Rails, prediction,
                        nowWallMs, captureSimSeconds) : null,
            };
        }

        /// <summary>Period hint (vis-viva on the parent-relative state at the batch
        /// start): feeds only the sampler's anti-aliasing step cap, so the sub-tick
        /// skew of sampling the parent outside the seed's Gate acquisition is
        /// irrelevant.</summary>
        public double PeriodHintAt(TrajectoryPredictor predictor, double t,
            StateVector? stateOverride = null,
            RailsService.PredictionContext? predictionOverride = null,
            string? bodyId = null)
        {
            StateVector state;
            if (stateOverride is { } known)
                state = known;
            else
                state = predictor.StateAt(t);
            string referenceBodyId = bodyId ?? ParentBodyId;
            var parentAbs = (predictionOverride ?? Prediction).GetAbsolute(referenceBodyId, t);
            return AdaptiveSampler.PeriodSeconds(Rails.MuOf(referenceBodyId),
                state.Position - parentAbs.Position, state.Velocity - parentAbs.Velocity);
        }

        /// <summary>Whether the previous published actual batch's GEOMETRY is still
        /// the truth this rebuild would resample (caller already established the
        /// predictor lineage is continuous): same frame mode and SOI parent, no
        /// Surface frame (its collision cut re-evaluates per sweep), no target-fixed
        /// frame (its coordinates depend on another mutable predictor), and young
        /// enough that the sampled future still leads the vessel and the batch
        /// end still hugs the sliding horizon. Reuse keeps the sampled future
        /// vertices while refreshing every now-anchored payload.</summary>
        public bool CanReuseActual(OverlaySamples previous, double periodHint)
        {
            if (!OverlayKernel.FrameAllowsGeometryReuse(ActiveFrame)) return false;
            if (!OverlayKernel.ModeMatches(previous.FrameLabel, ActiveFrame?.Label)) return false;
            if (!string.Equals(previous.ParentId, ParentBodyId, StringComparison.Ordinal)) return false;
            if (previous.SamplingThetaMax != ThetaMax
                || previous.SamplingMaxDensePoints != MaxDensePoints) return false;
            if (previous.DenseTimes.Length < 2) return false;
            foreach (var marker in previous.Markers)
                if (marker.Kind == OverlayMarkerKind.Collision) return false;
            double ageCap = double.IsFinite(periodHint) && periodHint > 0
                ? Math.Min(periodHint, OverlayKernel.ActualGeometryRecenteringSeconds)
                : OverlayKernel.ActualGeometryRecenteringSeconds;
            // Geometry age = how far the vessel has advanced past the SWEEP's start.
            if (T0 - previous.FutureStartSeconds >= ageCap) return false;
            // The horizon identity must hold BOTH ways: a batch ending far short
            // of the horizon (orbits preset grew, plan added) resamples
            // immediately — and one ending PAST it (preset shrank, plan deleted)
            // must not keep republishing the old longer line. The batch never
            // overhangs its own build horizon, so any real overhang is a window
            // shrink. A TRUNCATED batch is exempt from the short-ending test
            // UNLESS the window grew past the horizon it was built for: truncation
            // means the budget ran out under THAT window's capped turn bound, where
            // a fast-orbit window can end short and a same-window resample only
            // reproduces the same
            // coverage every cycle. A grown request still resamples once so the
            // batch records the new horizon. The age cap above forces the real resample that
            // slides the drawn coverage forward with the vessel.
            bool sameOrShrunkWindow = Horizon <= previous.HorizonSeconds + 1.0;
            if (!(previous.Truncated && sameOrShrunkWindow)
                && Horizon - previous.DenseTimes[^1]
                    > OverlayKernel.ActualGeometryRecenteringSeconds + ageCap) return false;
            if (previous.DenseTimes[^1] > Horizon + 1.0) return false;
            return previous.DenseTimes[^1] > T0 + 1.0; // any future line left at all
        }

        /// <summary>Reuses the still-future sampled vertices while refreshing every
        /// now-anchored payload.</summary>
        public OverlaySamples ReuseActualBatch(
            OverlaySamples previous, TrajectoryPredictor predictor,
            double captureSimSeconds, Func<bool>? shouldStop = null)
        {
            StateVector anchor = StateAt(predictor, T0);
            Vector3d parentNow = Prediction.GetAbsolute(ParentBodyId, T0).Position;
            Vector3d currentCce = anchor.Position - parentNow;

            int oldFutureStart = 0;
            while (oldFutureStart < previous.DenseTimes.Length
                && previous.DenseTimes[oldFutureStart] <= T0)
                oldFutureStart++;
            int futureCount = 1 + previous.DenseTimes.Length - oldFutureStart;
            var denseTimes = new double[futureCount];
            var densePositionsCce = new Vector3d[futureCount];
            denseTimes[0] = T0;
            densePositionsCce[0] = currentCce;
            Array.Copy(previous.DenseTimes, oldFutureStart,
                denseTimes, 1, futureCount - 1);
            Array.Copy(previous.DensePositionsCce, oldFutureStart,
                densePositionsCce, 1, futureCount - 1);

            Vector3d[]? denseCoordinates = previous.DenseFrameCoordinates is null
                ? null
                : new Vector3d[futureCount];
            if (denseCoordinates is not null)
            {
                if (!FrameManager.TrySamplePoseForCurve(FrameSnapshot!.Value,
                        Prediction, Target?.StateAt, T0, out var currentPose))
                {
                    denseCoordinates = null;
                }
                else
                {
                    var root = Prediction.GetAbsolute(Prediction.RootId, T0);
                    denseCoordinates[0] =
                        currentPose.ToFrame(anchor.Position - root.Position);
                    Array.Copy(previous.DenseFrameCoordinates!, oldFutureStart,
                        denseCoordinates, 1, futureCount - 1);
                }
            }

            var futureTimes = new List<double> { T0 };
            var futurePositionsCce = new List<Vector3d> { currentCce };
            List<Vector3d>? futureCoordinates = denseCoordinates is null
                ? null
                : [denseCoordinates[0]];
            for (int i = 0; i < previous.PointCount; i++)
            {
                if (previous.Times[i] <= T0) continue;
                futureTimes.Add(previous.Times[i]);
                futurePositionsCce.Add(previous.PositionsCce[i]);
                futureCoordinates?.Add(previous.FrameCoordinates![i]);
            }
            int count = futureTimes.Count;
            double[] times = [.. futureTimes];
            Vector3d[] positionsCce = [.. futurePositionsCce];
            var remaining = new double[count];
            var sincePe = new double[count];
            for (int k = 0; k < count; k++)
            {
                remaining[k] = times[k] - T0;
                sincePe[k] = times[k] - TimeAtPe;
            }

            IReadOnlyList<OverlayMarker> candidates = ComputeMarkers(
                Rails, Prediction, ActiveFrame, ParentBodyId, T0,
                MarkerWorkEnabled, Target,
                denseTimes, densePositionsCce, times, positionsCce, count);
            object markerKey = MarkerCacheKey;
            var markers = OverlayKernel.VisibleMarkers(candidates, T0);
            var denseMetrics = DecimationMetrics.For(
                denseCoordinates ?? densePositionsCce);
            return previous with
            {
                SampleT0 = T0,
                SampleWallMs = Environment.TickCount64,
                CaptureSimSeconds = captureSimSeconds,
                AnchorState = anchor,
                Times = OverlayKernel.PadToStockLength(times),
                TimesSincePe = OverlayKernel.PadToStockLength(sincePe),
                RemainingTimesTo = OverlayKernel.PadToStockLength(remaining),
                PositionsCce = OverlayKernel.PadToStockLength(positionsCce),
                PointCount = count,
                Markers = markers,
                MarkerCandidates = candidates,
                MarkerCacheKey = markerKey,
                FrameCoordinates = futureCoordinates is null
                    ? null
                    : OverlayKernel.PadToStockLength(futureCoordinates.ToArray()),
                FrameLabel = denseCoordinates is null ? null : ActiveFrame!.Label,
                DenseTimes = denseTimes,
                DensePositionsCce = densePositionsCce,
                DenseFrameCoordinates = denseCoordinates,
                DenseMetrics = denseMetrics,
                DenseMetricsCce = denseCoordinates is null
                    ? denseMetrics
                    : DecimationMetrics.For(densePositionsCce),
            };
        }

        internal readonly record struct AnalysisComputation(
            OrbitAnalysisReport? Report, string? Reason, bool Completed);

        internal AnalysisComputation ComputeAnalysis(
            Func<bool>? shouldStop = null)
        {
            if (!Ui.OrbitAnalyserPanel.RequestMatches(AnalysisRequestVersion))
                return new(null, null, false);
            double analysisStart = T0 + AnalysisStartOffsetSeconds;
            double requestedEnd = Math.Min(
                analysisStart + AnalysisSpanSeconds, AnalysisHorizon);
            if (requestedEnd <= analysisStart)
                return new(null,
                    "requested interval starts beyond the available rails horizon", true);

            int pass = Ui.OrbitAnalyserPanel.BeginAnalysisPass(AnalysisRequestVersion);
            if (pass == 0) return new(null, null, false);
            bool completed = false;
            double lastProgress = -1;
            Ui.OrbitAnalyserPanel.AnalysisPhase lastPhase = default;
            void Progress(double fraction, Ui.OrbitAnalyserPanel.AnalysisPhase phase)
            {
                double clamped = Math.Clamp(fraction, 0, 1);
                if (phase == lastPhase && clamped < lastProgress + 0.001 && clamped < 1)
                    return;
                lastPhase = phase;
                lastProgress = clamped;
                Ui.OrbitAnalyserPanel.ReportAnalysisProgress(
                    AnalysisRequestVersion, pass, clamped, phase);
            }
            bool StopRequested() => shouldStop?.Invoke() == true
                || !Ui.OrbitAnalyserPanel.RequestMatches(AnalysisRequestVersion);

            try
            {
                // Overlay and analysis execute concurrently. The ephemerides snapshot
                // is immutable and shareable, but GravityModel's segment caches are
                // single-owner, so analysis needs its own prediction context.
                var analysisPrediction = AnalysisPrediction!.ForkForConcurrentUse();
                // Analysis owns a streaming predictor. Extending the display predictor
                // across the whole interval first retained every integrator node and
                // deterministically hit TrajectoryPredictor.MaxNodes around 18% of a
                // ten-year low orbit. Advance a private coast in bounded chunks to a
                // non-zero start offset, then let the chronological analysis sweep
                // extend it on demand and prune accepted history behind itself.
                var analysisPredictor = TrackedVessel.NewDisplayPredictorAt(
                    CurrentAnchor, T0, analysisPrediction.Gravity);
                double trajectorySpan = Math.Max(1, requestedEnd - T0);
                Progress(0, analysisStart > T0
                    ? Ui.OrbitAnalyserPanel.AnalysisPhase.Propagating
                    : Ui.OrbitAnalyserPanel.AnalysisPhase.Sampling);
                while (analysisPredictor.Horizon < analysisStart)
                {
                    double chunkEnd = Math.Min(
                        analysisStart,
                        analysisPredictor.Horizon + AnalysisPropagationChunkSeconds);
                    analysisPredictor.ExtendTo(chunkEnd, time =>
                    {
                        if (StopRequested()) throw new OperationCanceledException();
                        Progress(0.9 * (time - T0) / trajectorySpan,
                            Ui.OrbitAnalyserPanel.AnalysisPhase.Propagating);
                    });
                    analysisPredictor.PruneBefore(chunkEnd);
                }
                if (StopRequested()) throw new OperationCanceledException();

                if (!TryAnalysisStartBody(analysisPrediction, analysisPredictor, analysisStart,
                        out string analysisBodyId, out string? ownershipError))
                {
                    completed = true;
                    return new(null, ownershipError, true);
                }
                if (!Rails.TryGetEquatorialPole(analysisBodyId, out var pole))
                {
                    completed = true;
                    return new(null,
                        $"equatorial pole for '{analysisBodyId}' was not captured",
                        true);
                }

                Progress(0.9 * (analysisStart - T0) / trajectorySpan,
                    Ui.OrbitAnalyserPanel.AnalysisPhase.Sampling);
                double periodHint = PeriodHintAt(
                    analysisPredictor, analysisStart,
                    predictionOverride: analysisPrediction,
                    bodyId: analysisBodyId);
                double maximumTurn = OrbitAnalysisSampler.ProductionTurnRadians(
                    analysisStart, requestedEnd, periodHint);
                var series = OrbitAnalysisSampler.Sample(time =>
                {
                    var vessel = StateAt(analysisPredictor, time);
                    var parent = analysisPrediction.GetAbsolute(analysisBodyId, time);
                    return (vessel.Position - parent.Position,
                        vessel.Velocity - parent.Velocity);
                }, analysisStart, requestedEnd, periodHint, StopRequested,
                    maximumPoints: OrbitAnalysisSampler.ProductionMaximumPoints,
                    progress: fraction => Progress(
                        0.9 * ((analysisStart - T0)
                            + fraction * (requestedEnd - analysisStart))
                            / trajectorySpan,
                        Ui.OrbitAnalyserPanel.AnalysisPhase.Sampling),
                    maximumTurnRadians: maximumTurn,
                    acceptedTime: analysisPredictor.PruneBefore);
                if (series.WorkLimited) return new(null, null, false);
                if (series.Times.Length < 3)
                {
                    completed = true;
                    return new(null,
                        "trajectory has fewer than three analysis samples", true);
                }
                if (!TryAnalysisEnd(analysisPrediction, analysisBodyId,
                        series.Times, series.Positions, series.Times.Length,
                        StopRequested,
                        out double end, out AnalysisSoiTransition? transition,
                        out string? transitionError))
                {
                    completed = true;
                    return new(null, transitionError, true);
                }
                double? spin = Rails.TryGetAngularVelocity(analysisBodyId, out double rate)
                    ? rate : null;
                double analysisEnd = Math.Min(requestedEnd, end);
                Progress(0.9, Ui.OrbitAnalyserPanel.AnalysisPhase.Reducing);
                var report = OrbitAnalysisKernel.Analyze(
                    analysisBodyId, series.Times, series.Positions, series.Velocities,
                    analysisStart, analysisEnd, Rails.MuOf(analysisBodyId),
                    Rails.MeanRadiusOf(analysisBodyId), pole, spin,
                    fraction => Progress(0.9 + 0.1 * fraction,
                        Ui.OrbitAnalyserPanel.AnalysisPhase.Reducing),
                    StopRequested);
                completed = true;
                if (report is null)
                    return new(null,
                        "future arc is too short or degenerate for orbit analysis", true);
                if (series.Truncated)
                {
                    string note = series.DynamicsLimited
                        ? "analysis ended at the predictor dynamics limit"
                        : "analysis ended before the requested interval";
                    report = report with { Notes = [.. report.Notes, note] };
                }
                if (transition is { } soiTransition)
                    report = report with
                    {
                        Notes =
                        [
                            .. report.Notes,
                            $"analysis ended at transition from '{analysisBodyId}' "
                                + $"to '{soiTransition.NewBodyId}' SOI",
                        ],
                    };
                return new(report, null, true);
            }
            finally
            {
                if (completed)
                    Ui.OrbitAnalyserPanel.CompleteAnalysisPass(
                        AnalysisRequestVersion, pass);
            }
        }

        private bool TryAnalysisStartBody(RailsService.PredictionContext prediction,
            TrajectoryPredictor predictor, double time, out string owner, out string? error)
        {
            owner = prediction.RootId;
            error = null;
            try
            {
                owner = AnalysisBodyAtStart(prediction.RootId,
                    predictor.StateAt(time).Position,
                    Rails.SoiChildrenOf,
                    bodyId => prediction.GetAbsolute(bodyId, time).Position,
                    Rails.SphereOfInfluenceOf);
                return true;
            }
            catch (Exception e)
            {
                error = $"analysis-start SOI ownership check failed: {e.Message}";
                return false;
            }
        }

        /// <summary>Independent, contained SOI cutoff for analysis. It deliberately
        /// does not consume display markers: an Ap/Pe/target-marker failure cannot
        /// let frozen-body elements leak beyond a sampled transition.</summary>
        private bool TryAnalysisEnd(RailsService.PredictionContext prediction,
            string analysisBodyId, double[] times, Vector3d[] positions, int count,
            Func<bool>? shouldStop,
            out double end, out AnalysisSoiTransition? transition, out string? error)
        {
            end = count > 0 ? times[count - 1] : T0;
            transition = null;
            error = null;
            if (count < 2) return true;
            try
            {
                var children = Rails.SoiChildrenOf(analysisBodyId);
                var members = new List<string>(children.Count + 1) { analysisBodyId };
                members.AddRange(children);
                var startStates = new StateVector[members.Count];
                var endStates = new StateVector[members.Count];
                var sweptChildren = new SoiReparentKernel.SweptCandidate[children.Count];
                var childSois = new double[children.Count];
                for (int child = 0; child < children.Count; child++)
                    childSois[child] = Rails.SphereOfInfluenceOf(children[child]);
                prediction.GetAbsoluteMany(members, times[0], startStates);
                double bodySoi = Rails.SphereOfInfluenceOf(analysisBodyId);
                string? parentBodyId = Rails.ParentIdOf(analysisBodyId);
                for (int i = 0; i + 1 < count; i++)
                {
                    if ((i & 255) == 0 && shouldStop?.Invoke() == true)
                        throw new OperationCanceledException();
                    prediction.GetAbsoluteMany(members, times[i + 1], endStates);
                    for (int child = 0; child < children.Count; child++)
                        sweptChildren[child] = new(children[child],
                            positions[i] + startStates[0].Position
                                - startStates[child + 1].Position,
                            positions[i + 1] + endStates[0].Position
                                - endStates[child + 1].Position,
                            childSois[child]);
                    if (SoiReparentKernel.FirstCrossing(
                            positions[i], positions[i + 1], bodySoi,
                            parentBodyId, sweptChildren) is { } crossing)
                    {
                        end = times[i]
                            + (times[i + 1] - times[i]) * crossing.Fraction;
                        transition = new(end, crossing.NewParentId);
                        return true;
                    }
                    (startStates, endStates) = (endStates, startStates);
                }
                if (shouldStop?.Invoke() == true)
                    throw new OperationCanceledException();
                return true;
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception e)
            {
                error = $"SOI transition scan failed: {e.Message}";
                return false;
            }
        }

        /// <summary>One batch = one adaptive sweep over [start, horizon] in the DRAWN
        /// coordinates (frame coordinates when a display frame is active —
        /// re-embedding is a similarity, rotation plus ONE uniform scale at the
        /// now-pose (rotating-pulsating), so turn angles are preserved;
        /// parent-relative otherwise). Times are CONIC-anchored, not
        /// horizon-anchored: TimeSincePe = t - TimeAtPe (linear, so the stock
        /// wrap-around scan finds no wrap) and RemainingTimeTo = t - now (stock's
        /// GetRemainingTimeTo semantics), which keeps stock's click-to-place
        /// burn-time derivation (Program.cs first-burn path computes
        /// now + GetRemainingTimeTo(point.TimeSincePe), a mod-period identity on
        /// these values) landing at the clicked sample's time. Every evaluated time
        /// is memoized so the array-building pass never re-integrates. Gate
        /// discipline: the lock wraps ONLY the per-sample predictor call;
        /// TrySamplePose/GetAbsolute lock internally per call, never across the
        /// sweep. Every segment uses ThetaMax; if the requested window needs more
        /// than MaxDensePoints at that quality, the line ends at the last accepted
        /// sample.</summary>
        public OverlaySamples SampleBatch(TrajectoryPredictor predictor, double start,
            double periodHint, double captureSimSeconds,
            Func<bool>? shouldStop = null,
            StateVector? anchorStateOverride = null, double? horizonOverride = null)
        {
            StateVector anchorState = anchorStateOverride ?? StateAt(predictor, T0);
            bool frameAttempted = FrameSnapshot is not null;
            bool framesOk = frameAttempted;
            // Cce recovery memo, needed ONLY on the frame path: there SampleDrawn
            // returns frame coordinates and the parent-relative Cce is recovered per
            // accepted sample below. On the inertial path sampled.Positions ARE the
            // Cce values (SampleDrawn returns them verbatim) — the arrays alias, no
            // memo, no copies (rebuilding both through dictionaries would cost up
            // to MaxDensePoints entries per rebuild).
            var memo = frameAttempted
                ? new Dictionary<double, (Vector3d Absolute, Vector3d ParentPos)>()
                : null;

            Vector3d SampleDrawn(double t)
            {
                StateVector absolute;
                if (t == T0)
                {
                    absolute = anchorState;
                }
                else
                {
                    absolute = predictor.StateAt(t);
                }
                var parentAbs = Prediction.GetAbsolute(ParentBodyId, t);
                if (framesOk && FrameManager.TrySamplePoseForCurve(
                        FrameSnapshot!.Value, Prediction, Target?.StateAt, t, out var pose))
                {
                    // Poses are game-convention (root pinned at the origin); the
                    // samples are mod-frame absolutes - convert first (GetGameEcl
                    // contract): game position = mod absolute - mod root absolute.
                    memo![t] = (absolute.Position, parentAbs.Position);
                    var gameAbs = absolute.Position
                        - Prediction.GetAbsolute(Prediction.RootId, t).Position;
                    return pose.ToFrame(gameAbs);
                }
                framesOk = false; // pose sampling failed: criterion continues parent-relative
                return absolute.Position - parentAbs.Position;
            }

            // Honest-density lines: the sweep runs at the DENSE budget (config
            // overlay_max_points) at one configured turn bound. The
            // stock-shaped arrays below become a decimated SUBSET of the dense sweep.
            double sampleHorizon = horizonOverride ?? Horizon;
            var sampled = AdaptiveSampler.Sample(SampleDrawn, start, sampleHorizon,
                MaxDensePoints, ThetaMax, dtMinSeconds: 1.0, periodHint, shouldStop);
            bool auxiliaryWorkAllowed = !sampled.WorkLimited
                && shouldStop?.Invoke() != true;

            // Surface-frame collision cut: in a Surface display frame the frame
            // coordinates are body-centred meters (rigid pose, Scale 1), so
            // |coordinate| is the distance to the frame body's center. The line must
            // STOP at the surface — the post-impact arc would draw a trajectory
            // through rock — and the cut point carries an impact marker annotated
            // with the SURFACE-relative speed (the frame-coordinate tangent: the
            // frame spins with the body, so the coordinate rate IS the speed the
            // ground is hit with). The live KSA terrain sampler supplies the local
            // physical surface; an invalidated query falls back to the catalog
            // mean-radius sphere for that rebuild. The cut and
            // its label are SEPARATE concerns: a degenerate speed stencil costs only
            // the number on the marker, never the truncation. framesOk failing
            // through the cut's evaluations discards the whole cut — that failure
            // degrades only this batch; an arbitrary-time curve failure never retires
            // a valid current display frame, and no surface cut applies in inertial mode.
            double[] sweepTimes = sampled.Times;
            Vector3d[] sweepPositions = sampled.Positions;
            OverlayMarker? collisionMarker = null;
            if (auxiliaryWorkAllowed
                && framesOk && ActiveFrame is { Kind: FrameKind.Surface }
                && OverlayKernel.CutAtFirstCollision(sweepTimes, sweepPositions,
                    Rails.MeanRadiusOf(ActiveFrame.PrimaryId), SampleDrawn,
                    Terrain?.HeightFromDirectionCcf,
                    Terrain?.MaximumSurfaceRadius ?? double.NaN) is { } cut)
            {
                // Impact speed: one-sided tangent just BEFORE the crossing (the far
                // side of a centered stencil is inside the body). Best-effort value;
                // the time-to-impact remains useful when the speed stencil degenerates.
                double impactSpeed = double.NaN;
                double priorTime = Math.Max(start, cut.ImpactTimeSeconds - 1.0);
                double span = cut.ImpactTimeSeconds - priorTime;
                if (span > 1e-3)
                {
                    impactSpeed = (cut.ImpactCoordinate - SampleDrawn(priorTime)).Length() / span;
                }
                if (framesOk)
                {
                    sweepTimes = cut.Times;
                    sweepPositions = cut.Positions;
                    double? storedImpactSpeed = double.IsFinite(impactSpeed) ? impactSpeed : null;
                    string label = OverlayKernel.ImpactLabel(
                        cut.ImpactTimeSeconds, T0, storedImpactSpeed);
                    collisionMarker = new OverlayMarker(OverlayMarkerKind.Collision,
                        ActiveFrame.PrimaryId, cut.ImpactTimeSeconds, 0.0, label,
                        storedImpactSpeed);
                }
            }

            int futureCount = sweepTimes.Length;
            var futureTimes = sweepTimes;
            // framesOk surviving the WHOLE sweep means every accepted sample took the
            // framed branch (it degrades monotonically): sampled.Positions IS the
            // dense frame-coordinate array. A mid-sweep degrade leaves Positions
            // mixed-mode — Cce then comes from the memo where a framed entry exists
            // and from Positions verbatim where the sweep had already degraded.
            Vector3d[]? futureCoordinates = framesOk && frameAttempted ? sweepPositions : null;
            Vector3d[] futurePositionsCce;
            if (memo is null)
            {
                futurePositionsCce = sweepPositions; // inertial sweep: positions ARE Cce
            }
            else
            {
                futurePositionsCce = new Vector3d[futureCount];
                for (int i = 0; i < futureCount; i++)
                    futurePositionsCce[i] = memo.TryGetValue(futureTimes[i], out var m)
                        ? m.Absolute - m.ParentPos
                        : sweepPositions[i];
            }

            var denseTimes = futureTimes;
            var densePositionsCce = futurePositionsCce;
            Vector3d[]? denseCoordinates = futureCoordinates;
            // Decimated stock-shaped subset: every stock reader (hover job payload
            // lerp, click payloads, ground track) keeps exactly the stock-budget cost.
            var indices = OverlayKernel.DecimateIndices(futureCount, OverlayKernel.StockPointBufferLength);
            int count = indices.Length;
            var times = OverlayKernel.TakeAt(futureTimes, indices);
            var positionsCce = OverlayKernel.TakeAt(futurePositionsCce, indices);
            var timesSincePe = new double[count];
            var remainingTimesTo = new double[count];
            for (int k = 0; k < count; k++)
            {
                timesSincePe[k] = times[k] - TimeAtPe;   // conic-anchored (click-to-place identity)
                remainingTimesTo[k] = times[k] - T0;     // stock GetRemainingTimeTo semantics
            }
            // Frame curve, sampled inline above (job thread — the restage only
            // re-embeds). Published only when the WHOLE sweep stayed in frame
            // coordinates; a mid-sweep pose failure degrades to the inertial line.
            Vector3d[]? coordinates = futureCoordinates is null || denseCoordinates is null
                ? null : OverlayKernel.TakeAt(futureCoordinates, indices);
            string? frameLabel = denseCoordinates is null ? null : ActiveFrame!.Label;
            // Markers: series that need NO rails reads (distance/plane vs the orbit
            // parent, the frame-plane z) detect on the DENSE arrays — the drawn line
            // is dense, and a coarsely-detected apex pinned onto a smooth line reads
            // as a bug. Series needing per-sample rails pair reads (other bodies)
            // stay on the decimated arrays so marker cost never scales with the
            // dense budget.
            IReadOnlyList<OverlayMarker> markerCandidates = auxiliaryWorkAllowed
                ? ComputeMarkers(
                    Rails, Prediction, ActiveFrame, ParentBodyId, T0, MarkerWorkEnabled, Target,
                    denseTimes, densePositionsCce, times, positionsCce, count)
                : [];
            if (collisionMarker is not null)
            {
                // The cut endpoint IS the impact; the extremum/crossing candidates were
                // computed over the already-truncated series, so nothing past it shows.
                var withCollision = new List<OverlayMarker>(markerCandidates.Count + 1)
                    { collisionMarker };
                withCollision.AddRange(markerCandidates);
                markerCandidates = withCollision;
            }
            var markers = OverlayKernel.VisibleMarkers(markerCandidates, T0);
            var denseMetricsDrawn = DecimationMetrics.For(denseCoordinates ?? densePositionsCce);
            // Pad ONCE at publish (pad-to-2000 invariant): Stage then always
            // writes exactly stock-length buffers. The dense arrays publish UNPADDED —
            // their only readers (the dense draw, the time interpolation) are
            // length-honest by construction.
            return new OverlaySamples
            {
                VesselId = VesselId,
                SampleT0 = T0,
                FutureStartSeconds = start,
                SampleWallMs = Environment.TickCount64,
                CaptureSimSeconds = captureSimSeconds,
                AnchorState = anchorState,
                Times = OverlayKernel.PadToStockLength(times),
                TimesSincePe = OverlayKernel.PadToStockLength(timesSincePe),
                RemainingTimesTo = OverlayKernel.PadToStockLength(remainingTimesTo),
                PositionsCce = OverlayKernel.PadToStockLength(positionsCce),
                ParentId = ParentBodyId,
                PointCount = count,
                Truncated = sampled.Truncated,
                WorkLimited = sampled.WorkLimited,
                DynamicsLimited = sampled.DynamicsLimited,
                HorizonSeconds = sampleHorizon,
                SamplingThetaMax = ThetaMax,
                SamplingMaxDensePoints = MaxDensePoints,
                CoverageEndSeconds = sampled.Times[^1],
                Markers = markers,
                MarkerCandidates = markerCandidates,
                MarkerCacheKey = this.MarkerCacheKey,
                Analysis = null,
                AnalysisUnavailableReason = null,
                AnalysisRequestVersion = 0,
                AnalysisRequested = false,
                FrameCoordinates = coordinates is null ? null : OverlayKernel.PadToStockLength(coordinates),
                FrameLabel = frameLabel,
                DenseTimes = denseTimes,
                DensePositionsCce = densePositionsCce,
                DenseFrameCoordinates = denseCoordinates,
                // Drawn-space emit-filter metrics over what the draw actually reads
                // (a mid-sweep frame degrade leaves sampled.Positions mixed-mode;
                // the draw then uses the consistent Cce array) — worker-side so the
                // per-frame filter only compares. The Cce pair serves the draw-time
                // pose-failure fallback; same reference when inertial.
                DenseMetrics = denseMetricsDrawn,
                DenseMetricsCce = denseCoordinates is null
                    ? denseMetricsDrawn
                    : DecimationMetrics.For(densePositionsCce),
            };
        }

        private StateVector StateAt(TrajectoryPredictor predictor, double time)
        {
            return predictor.StateAt(time);
        }
    }

    /// <summary>Honest line markers: the first upcoming
    /// Ap/Pe per frame-relevant body — the active frame decides WHOSE: no frame or a
    /// body-centred/surface/target-fixed frame → that primary; a two-body
    /// fixed frame → BOTH pair bodies (ECI → Earth; ELF → Earth AND Luna) — plus the
    /// first upcoming AN/DN vs the primary body's spin equator. Its fixed pole is
    /// captured once at bind and reaches this worker as plain immutable data. Computed
    /// from the SAMPLED trajectory, never a conic — extrema/crossings of the real
    /// sample prefix (OverlayKernel.FirstLocalExtremum/FirstSignCrossing). Contained:
    /// a failing rails read costs the batch its markers, never the batch.</summary>
    private static IReadOnlyList<OverlayMarker> ComputeMarkers(RailsService rails,
        RailsService.PredictionContext prediction,
        FrameSpec? activeFrame, string parentId, double nowSeconds,
        bool markerWorkEnabled, MarkerTarget? target,
        double[] denseTimes, Vector3d[] densePositionsCce,
        double[] decimatedTimes, Vector3d[] decimatedPositionsCce, int decimatedCount)
    {
        var markers = new List<OverlayMarker>();
        if (!markerWorkEnabled) return markers;
        if (denseTimes.Length < 3) return markers;
        try
        {
            string primary = activeFrame?.PrimaryId ?? parentId;
            string? secondary = activeFrame?.Kind == FrameKind.TwoBodyFixed ? activeFrame.SecondaryId : null;

            // "First UPCOMING" means from NOW, not from the batch's first sample.
            // Keep one bracketing sample at/behind now because extremum detection
            // needs interior neighbors.
            int SeriesFrom(double[] times, int count) =>
                Math.Min(Math.Max(0, OverlayKernel.UpperBound(times, nowSeconds) - 1), count - 1);
            int denseFrom = SeriesFrom(denseTimes, denseTimes.Length);

            // Plane offsets use the primary-relative series at its honest density.
            // planeTimes/planeBase track that series' own time base so the crossing
            // lerp can never mix densities.
            bool haveEquatorialPole = rails.TryGetEquatorialPole(primary, out var equatorialPole);
            double[]? planeOffsets = null;
            Vector3d[]? planeRelativePositions = null;
            double[] planeTimes = denseTimes;
            int planeBase = denseFrom;

            string[] bodies = secondary is null ? [primary] : [primary, secondary];
            foreach (string body in bodies)
            {
                // Density split (honest-density lines): the orbit PARENT's series
                // needs no rails reads (positionsCce are parent-relative already) —
                // it detects on the DENSE sweep so Ap/Pe land on the drawn line's
                // actual apex. Any other body needs a rails pair read per sample and
                // stays on the decimated series (marker cost must not scale with the
                // dense budget); its markers are far-body encounters, not the tight
                // multi-revolution geometry the dense budget exists for.
                bool onDense = string.Equals(body, parentId, StringComparison.Ordinal);
                double[] seriesTimes = onDense ? denseTimes : decimatedTimes;
                int seriesCount = onDense ? denseTimes.Length : decimatedCount;
                int from = onDense ? denseFrom : SeriesFrom(decimatedTimes, decimatedCount);
                int count = seriesCount - from;
                if (count < 3) continue;
                var distances = new double[count];
                var relativePositions = new Vector3d[count];
                double[]? equatorialOffsets = haveEquatorialPole
                    && string.Equals(body, primary, StringComparison.Ordinal)
                    ? new double[count] : null;
                if (onDense)
                {
                    for (int i = 0; i < count; i++)
                    {
                        var rel = densePositionsCce[from + i];
                        relativePositions[i] = rel;
                        distances[i] = rel.Length();
                        if (equatorialOffsets is not null)
                            equatorialOffsets[i] = OverlayKernel.EquatorialPlaneOffset(
                                rel, equatorialPole);
                    }
                }
                else
                {
                    for (int i = 0; i < count; i++)
                    {
                        // rel to B = parent-relative sample + (parent - B), evaluated
                        // from the immutable prediction context.
                        var (parent, other) = prediction.GetGameEclPair(
                            parentId, body, seriesTimes[from + i]);
                        var rel = decimatedPositionsCce[from + i] + parent.Position - other.Position;
                        relativePositions[i] = rel;
                        distances[i] = rel.Length();
                        if (equatorialOffsets is not null)
                            equatorialOffsets[i] = OverlayKernel.EquatorialPlaneOffset(
                                rel, equatorialPole);
                    }
                }
                double radius = rails.MeanRadiusOf(body);
                foreach (int apoapsis in OverlayKernel.LocalExtrema(
                    distances, count, findMinimum: false))
                {
                    double altitude = distances[apoapsis] - radius;
                    double relativeSpeed = OverlayKernel.RelativeSpeedAt(
                        seriesTimes, relativePositions, apoapsis, count, from);
                    markers.Add(new OverlayMarker(OverlayMarkerKind.Apoapsis, body,
                        seriesTimes[from + apoapsis], altitude,
                        $"Ap {body} {altitude / 1000.0:N0} km",
                        RelativeSpeedMetersPerSecond: relativeSpeed));
                }
                foreach (int periapsis in OverlayKernel.LocalExtrema(
                    distances, count, findMinimum: true))
                {
                    double altitude = distances[periapsis] - radius;
                    double relativeSpeed = OverlayKernel.RelativeSpeedAt(
                        seriesTimes, relativePositions, periapsis, count, from);
                    markers.Add(new OverlayMarker(OverlayMarkerKind.Periapsis, body,
                        seriesTimes[from + periapsis], altitude,
                        $"Pe {body} {altitude / 1000.0:N0} km",
                        RelativeSpeedMetersPerSecond: relativeSpeed));
                }
                if (equatorialOffsets is not null)
                {
                    planeOffsets = equatorialOffsets;
                    planeRelativePositions = relativePositions;
                    planeTimes = seriesTimes;
                    planeBase = from;
                }
            }

            if (planeOffsets is not null)
            {
                foreach (var an in OverlayKernel.SignCrossings(
                    planeOffsets, planeOffsets.Length, ascending: true))
                {
                    double relativeSpeed = OverlayKernel.RelativeSpeedAcross(
                        planeTimes, planeRelativePositions!, an.Lo, planeOffsets.Length, planeBase);
                    markers.Add(new OverlayMarker(OverlayMarkerKind.AscendingNode, primary,
                        planeTimes[planeBase + an.Lo]
                        + (planeTimes[planeBase + an.Lo + 1] - planeTimes[planeBase + an.Lo]) * an.Frac,
                        0, "AN", RelativeSpeedMetersPerSecond: relativeSpeed));
                }
                foreach (var dn in OverlayKernel.SignCrossings(
                    planeOffsets, planeOffsets.Length, ascending: false))
                {
                    double relativeSpeed = OverlayKernel.RelativeSpeedAcross(
                        planeTimes, planeRelativePositions!, dn.Lo, planeOffsets.Length, planeBase);
                    markers.Add(new OverlayMarker(OverlayMarkerKind.DescendingNode, primary,
                        planeTimes[planeBase + dn.Lo]
                        + (planeTimes[planeBase + dn.Lo + 1] - planeTimes[planeBase + dn.Lo]) * dn.Frac,
                        0, "DN", RelativeSpeedMetersPerSecond: relativeSpeed));
                }
            }

            // Predictor SOI events: the only possible next parent transitions are
            // escape from the current parent or encounter with one of its vessel-gravity
            // direct children (the same hierarchy rule as RailsSoiParent). These scans
            // use the bounded decimated series because every point needs a rails pair.
            int eventFrom = SeriesFrom(decimatedTimes, decimatedCount);
            int eventCount = decimatedCount - eventFrom;
            if (eventCount >= 2)
            {
                var bodyDistances = new Dictionary<string, double[]>(StringComparer.Ordinal);
                var bodyRelativePositions = new Dictionary<string, Vector3d[]>(StringComparer.Ordinal);
                var preparedFamilies = new HashSet<string>(StringComparer.Ordinal);

                // One Gate acquisition per SAMPLE for a parent and all of its direct
                // children, regardless of catalog density. A nested family is loaded only if
                // the chronological event chain actually enters it.
                IReadOnlyList<string> ChildrenAndPrepare(string owner)
                {
                    var children = rails.SoiChildrenOf(owner);
                    if (!preparedFamilies.Add(owner)) return children;
                    var members = new List<string>(children.Count + 1) { owner };
                    members.AddRange(children);
                    var missing = members
                        .Where(body => !bodyDistances.ContainsKey(body))
                        .Distinct(StringComparer.Ordinal).ToArray();
                    if (missing.Length == 0) return children;

                    foreach (string body in missing)
                    {
                        bodyDistances[body] = new double[eventCount];
                        bodyRelativePositions[body] = new Vector3d[eventCount];
                    }
                    var ids = new List<string>(missing.Length + 1) { parentId };
                    foreach (string body in missing)
                        if (!string.Equals(body, parentId, StringComparison.Ordinal))
                            ids.Add(body);
                    var stateIndex = ids
                        .Select((id, index) => (id, index))
                        .ToDictionary(pair => pair.id, pair => pair.index, StringComparer.Ordinal);

                    for (int i = 0; i < eventCount; i++)
                    {
                        int sample = eventFrom + i;
                        var states = prediction.GetAbsoluteMany(ids, decimatedTimes[sample]);
                        var vesselAbsolute = decimatedPositionsCce[sample] + states[0].Position;
                        foreach (string body in missing)
                        {
                            int index = stateIndex[body];
                            var relative = vesselAbsolute - states[index].Position;
                            bodyRelativePositions[body][i] = relative;
                            bodyDistances[body][i] = relative.Length();
                        }
                    }
                    return children;
                }

                double[] DistancesTo(string body) => bodyDistances[body];
                var transitions = OverlayKernel.FindSoiTransitions(
                    parentId, eventCount, rails.ParentIdOf, ChildrenAndPrepare,
                    rails.SphereOfInfluenceOf, DistancesTo);
                foreach (var found in transitions)
                {
                    double time = decimatedTimes[eventFrom + found.Lo]
                        + (decimatedTimes[eventFrom + found.Lo + 1]
                            - decimatedTimes[eventFrom + found.Lo]) * found.Frac;
                    double relativeSpeed = OverlayKernel.RelativeSpeedAcross(
                        decimatedTimes, bodyRelativePositions[found.BodyId], found.Lo,
                        eventCount, eventFrom);
                    if (found.Escape)
                        markers.Add(new OverlayMarker(OverlayMarkerKind.Escape, found.BodyId,
                            time, 0, $"Escape {found.BodyId}",
                            RelativeSpeedMetersPerSecond: relativeSpeed));
                    else
                        markers.Add(new OverlayMarker(OverlayMarkerKind.Encounter, found.BodyId,
                            time, 0, $"Encounter {found.BodyId}",
                            RelativeSpeedMetersPerSecond: relativeSpeed));
                }

            }

            // Every closest approach to the selected vessel/body, detected on the
            // DENSE DRAWN timeline. This is deliberately independent of the target's
            // display samples (a target is nearly stationary in its own fixed frame)
            // and of the 2,000-point stock decimation, which can hide relative minima.
            if (target is not null)
            {
                int targetFrom = denseFrom;
                while (targetFrom < denseTimes.Length && denseTimes[targetFrom] < target.StartTime)
                    targetFrom++;
                int targetTo = targetFrom;
                while (targetTo < denseTimes.Length && denseTimes[targetTo] <= target.EndTime)
                    targetTo++;
                var separations = new double[targetTo - targetFrom];
                var relativeTimes = new double[targetTo - targetFrom];
                var relativePositions = new Vector3d[targetTo - targetFrom];
                int separationCount = 0;
                for (int sample = targetFrom; sample < targetTo; sample++)
                {
                    double t = denseTimes[sample];
                    if (target.PositionAt(t) is not { } targetPosition) break;
                    var vesselAbsolute = densePositionsCce[sample]
                        + prediction.GetAbsolute(parentId, t).Position;
                    var relative = vesselAbsolute - targetPosition;
                    relativeTimes[separationCount] = t;
                    relativePositions[separationCount] = relative;
                    separations[separationCount++] = relative.Length();
                }
                foreach (int closest in OverlayKernel.LocalExtrema(
                    separations, separationCount, findMinimum: true))
                {
                    double separation = separations[closest];
                    double time = relativeTimes[closest];
                    double relativeSpeed = OverlayKernel.RelativeSpeedAt(
                        relativeTimes, relativePositions, closest, separationCount);
                    markers.Add(new OverlayMarker(OverlayMarkerKind.ClosestApproach,
                        target.Id, time, separation,
                        OverlayKernel.ClosestApproachLabel(
                            target.Id, separation, time - nowSeconds, relativeSpeed),
                        RelativeSpeedMetersPerSecond: relativeSpeed));
                }
            }
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("line markers", e);
        }
        return markers;
    }

    /// <summary>Everything <see cref="StagedPoint"/> needs to turn one batch sample
    /// into the OrbitPointCce the map draws — resolved ONCE per staging pass (and once
    /// per hover query): the frame re-embedding pose, the SOI parent shift, and the
    /// payload time anchor. Shared by <see cref="Stage"/> (the whole buffer) and
    /// OrbitHoverPatch's substitute hit-test (one index), so the hover circle can never
    /// disagree with the drawn line about where a sample sits.</summary>
    internal readonly struct StagingContext
    {
        /// <summary>Frame path active: re-embed FrameCoordinates at <see cref="NowPose"/>.</summary>
        public required bool Framed { get; init; }
        public required FramePose NowPose { get; init; }
        /// <summary>Frame path: the anchor parent's game-ecl position at "now".</summary>
        public required Vector3d FrameParentNow { get; init; }
        /// <summary>Inertial path: SOI re-anchor shift (zero unless a stock patch
        /// transition landed since publish — OverlayKernel.ParentShift).</summary>
        public required Vector3d ParentShift { get; init; }
        /// <summary>NaN: use the batch's baked TimesSincePe (the vehicle's own conic
        /// anchor). Otherwise the canvas orbit's TimeAtPeriapsis — the planned batch is
        /// staged into the earliest burn's plan orbit, whose conic anchor differs, and
        /// stock's hover/click time derivation reads payloads against THAT orbit.</summary>
        public required double AnchorPeSeconds { get; init; }

        /// <summary>THE drawn-position rule, one home for every surface (staged pick
        /// buffer, hover substitute, dense draw): frame re-embed at the now pose when
        /// framed (<paramref name="frameCoordinate"/> is meaningful exactly then),
        /// parent-relative plus the SOI shift otherwise. Two hand-copied variants of
        /// this expression is how the drawn line and the pick surface drift apart.</summary>
        public Vector3d Drawn(Vector3d positionCce, Vector3d frameCoordinate) =>
            Framed
                ? NowPose.FromFrame(frameCoordinate) - FrameParentNow
                : positionCce + ParentShift;
    }

    /// <summary>Resolve the staging context for a batch against the orbit it will be
    /// staged into. Never throws (frame-path failures degrade to the inertial path,
    /// contained) — safe to call before the MemoryOwner exists.</summary>
    internal static StagingContext BuildStagingContext(OverlaySamples samples, Orbit orbit, bool reanchorTimes)
    {
        // SOI-independence: stock draws every staged point at
        // currentParent(now) + Cce, so a batch published under a PREVIOUS stock parent
        // (an SOI/patch transition landed since) is shifted by OverlayKernel.ParentShift
        // — the drawn world positions stay bit-identical to the pre-transition
        // rendering instead of displacing by the interbody distance or blinking.
        // MaybeRebuild's context-aware immediate rebuild republishes against the new
        // parent within ~a tick, which also bounds the conic-anchored TimeSincePe skew
        // (the hover/click "now" anchor is the NEW orbit's) to that same bridge tick.
        // A non-Astronomical current parent keeps the batch anchor (identity shift —
        // MaybeRebuild never publishes for those, so the batch goes stale and routes
        // Stock within 5 s anyway).
        string anchorParentId = orbit.Parent is Astronomical currentParent ? currentParent.Id : samples.ParentId;
        var parentShift = Vector3d.Zero;
        if (!string.Equals(anchorParentId, samples.ParentId, StringComparison.Ordinal)
            && ModServices.Rails is { } shiftRails)
        {
            double tShift = Universe.GetElapsedSimTime().Seconds();
            var (batchParent, nowParent) = shiftRails.GetGameEclPair(samples.ParentId, anchorParentId, tShift);
            parentShift = OverlayKernel.ParentShift(batchParent.Position, nowParent.Position);
        }

        // Frame path: re-embed the SAMPLED frame coordinates at the CURRENT
        // pose (FrameKernel semantics: nowPose.FromFrame(samplePose.ToFrame(x)) - the
        // ToFrame half was precomputed at sampling, PER-SAMPLE-TIME pose, so two-body
        // frame coordinates are separation-normalized and FromFrame re-dimensionalizes
        // at the current separation: the rotating-PULSATING loop, both bodies
        // pinned). Falls back to the inertial positions when no frame is active,
        // the frame changed since sampling, or pose sampling fails (contained, frame
        // deactivates). The VesselLinePatch prefix gates frame MODE before staging (a
        // label mismatch routes LineRoute.Blink and never stages), so from that call
        // site the label check below is unreachable defense-in-depth; it stays live
        // for the job-thread staging call in MaybeRebuild (a frame switch can land
        // between sampling and staging within the same rebuild).
        bool framed = false;
        FramePose nowPose = default;
        Vector3d frameParentNow = default;
        if (samples.FrameCoordinates is not null
            && FrameManager.TryCaptureActive(out var frameSnapshot)
            && OverlayKernel.ModeMatches(samples.FrameLabel, frameSnapshot.Spec.Label)
            && ModServices.Rails is { } rails)
        {
            try
            {
                double tNow = Universe.GetElapsedSimTime().Seconds();
                if (FrameManager.TrySamplePoseForDisplay(frameSnapshot, tNow, out nowPose))
                {
                    // Anchor at the CURRENT orbit parent (SOI-independence): stock adds
                    // that parent's position back at draw time, so subtracting any
                    // other body would displace the re-embedded frame image.
                    frameParentNow = rails.GetGameEcl(anchorParentId, tNow).Position;
                    framed = true;
                }
            }
            catch (Exception e)
            {
                framed = false;
                FrameManager.NoteContained("overlay frame staging", e);
            }
        }

        return new StagingContext
        {
            Framed = framed,
            NowPose = nowPose,
            FrameParentNow = frameParentNow,
            ParentShift = parentShift,
            AnchorPeSeconds = reanchorTimes ? orbit.TimeAtPeriapsis.Seconds() : double.NaN,
        };
    }

    /// <summary>Sample i's drawn parent-centred position under <paramref name="ctx"/> —
    /// the position half of a staged point, exposed separately so the hover substitute
    /// (OrbitHoverPatch) can chord-lerp positions and payload times independently while
    /// staying on the ONE staging math. Parent-centred Cce = ecliptic axes: no rotation
    /// (Astronomical.GetPositionEclFromCce = parent ecl + Cce).</summary>
    internal static Vector3d DrawnCce(OverlaySamples samples, in StagingContext ctx, int i) =>
        ctx.Drawn(samples.PositionsCce[i], ctx.Framed ? samples.FrameCoordinates![i] : default);

    /// <summary>Sample i's payload TimeSincePe under <paramref name="ctx"/> (the
    /// baked vehicle-conic anchor, or the canvas orbit's when re-anchored).</summary>
    internal static double PayloadTimeSincePe(OverlaySamples samples, in StagingContext ctx, int i) =>
        double.IsNaN(ctx.AnchorPeSeconds) ? samples.TimesSincePe[i] : samples.Times[i] - ctx.AnchorPeSeconds;

    /// <summary>One staged map point: batch sample <paramref name="i"/> as the exact
    /// OrbitPointCce the map draws under <paramref name="ctx"/>.</summary>
    internal static OrbitPointCce StagedPoint(OverlaySamples samples, in StagingContext ctx, int i) =>
        new(FrameAdapter.ToGame(DrawnCce(samples, in ctx, i)),
            new SimTime(PayloadTimeSincePe(samples, in ctx, i)),
            new SimTime(samples.RemainingTimesTo[i]),
            TrueAnomaly.NaN);

    /// <summary>Builds the game's point buffer from a sample batch and hands it to
    /// Orbit.UpdateCachedPoints (stock allocate-handoff-forget
    /// ownership). Called from the job thread once per sampling AND every frame from the
    /// render phase — the <see cref="Patches.VesselLinePatch"/> prefix on
    /// FlightPlan.AddLineInstances; line-instance building runs in the post-solver
    /// render phase, so solver writes precede it and the phases never overlap.
    /// A pure function of the immutable batch, so
    /// re-staging the same batch within a frame is idempotent by construction (the
    /// frame path re-reads the current pose per call, so within-frame repeats still agree
    /// to the pose sampled at one Universe time — GetElapsedSimTime is frame-constant).
    /// Ordinary fallible context work happens before the buffer exists. A session reset
    /// that wins after allocation rejects the generation and disposes the still-owned
    /// buffer; after UpdateCachedPoints begins, Orbit owns it. <paramref name="reanchorTimes"/>: see
    /// <see cref="StagingContext.AnchorPeSeconds"/> — true only for the planned batch
    /// staged into the earliest burn's plan orbit. <paramref name="preserveStockForFallback"/>
    /// records the latest finite-TA stock cache before any overwrite of a stock-owned
    /// actual or planned canvas. Mod-only fallback canvases leave the ledger untouched.</summary>
    internal static StagingContext Stage(OverlaySamples samples, Orbit orbit,
        bool reanchorTimes = false, bool preserveStockForFallback = false,
        int? overlayWorkerGeneration = null)
    {
        var cache = CaptureStageCache(
            orbit, out long generation, overlayWorkerGeneration);
        lock (cache.Gate)
        {
            if (preserveStockForFallback)
            {
                var observation = ObserveStockCache(
                    orbit.CachedPoints, cache.StockPoints, cache.ActualCacheModOwned);
                cache.StockPoints = observation.StockPoints;
                cache.ActualCacheModOwned = observation.ActualCacheModOwned;
            }

            double simSeconds = Universe.GetElapsedSimTime().Seconds();
            string? parentId = (orbit.Parent as Astronomical)?.Id;
            string? frameLabel = FrameManager.Active?.Label;
            double anchorPe = reanchorTimes ? orbit.TimeAtPeriapsis.Seconds() : double.NaN;
            bool sameAnchor = double.IsNaN(anchorPe)
                ? double.IsNaN(cache.AnchorPeSeconds)
                : anchorPe.Equals(cache.AnchorPeSeconds);
            if (ReferenceEquals(cache.Samples, samples)
                && cache.SimSeconds.Equals(simSeconds)
                && sameAnchor
                && string.Equals(cache.ParentId, parentId, StringComparison.Ordinal)
                && string.Equals(cache.FrameLabel, frameLabel, StringComparison.Ordinal)
                && orbit.LineCount == samples.Times.Length
                && orbit.CachedPoints.Length > 0
                && double.IsNaN(orbit.CachedPoints[0].CompassTrueAnomaly)
                && orbit.CachedPoints[0].PositionCce.Equals(cache.FirstPosition))
            {
                lock (StageCacheHandoffGate)
                {
                    ThrowIfStaleStageGeneration(generation);
                    return cache.Context;
                }
            }

            var ctx = BuildStagingContext(samples, orbit, reanchorTimes);
            int n = samples.Times.Length;
            var points = MemoryOwner<OrbitPointCce>.Allocate(n);
            var span = points.Span;
            for (int i = 0; i < n; i++) span[i] = StagedPoint(samples, in ctx, i);
            if (n == 0)
            {
                HandoffStagedPoints(
                    orbit, points, cache, generation, preserveStockForFallback);
                return ctx;
            }
            double3 firstPosition = span[0].PositionCce;
            HandoffStagedPoints(
                orbit, points, cache, generation, preserveStockForFallback);
            cache.Samples = samples;
            cache.ParentId = parentId;
            cache.FrameLabel = frameLabel;
            cache.SimSeconds = simSeconds;
            cache.AnchorPeSeconds = anchorPe;
            cache.Context = ctx;
            cache.FirstPosition = firstPosition;
            return ctx;
        }
    }

    /// <summary>The worker's only staging entry: admission compares the job's captured
    /// OverlayWorker generation under the same pending-reset gate that advances it.
    /// A stale job is rejected before Stage reads the sample batch or orbit context.</summary>
    internal static StagingContext StageWorkerBatch(
        OverlaySamples samples, Orbit orbit, int capturedWorkerGeneration) =>
        Stage(samples, orbit, preserveStockForFallback: true,
            overlayWorkerGeneration: capturedWorkerGeneration);

    internal readonly record struct StockCacheObservation(
        OrbitPointCce[]? StockPoints,
        bool ActualCacheModOwned,
        bool CurrentCacheIsSafeStock);

    /// <summary>Classifies one cache observation and updates the reusable exact-stock
    /// snapshot. Every point must carry a finite compass anomaly before a non-empty
    /// cache is accepted as stock. Empty is genuine stock only with no outstanding mod
    /// handoff; owned empty/all-NaN may retain the prior snapshot, while every mixed or
    /// foreign shape invalidates it and forces fallback closed.</summary>
    internal static StockCacheObservation ObserveStockCache(
        ReadOnlySpan<OrbitPointCce> current,
        OrbitPointCce[]? previous,
        bool actualCacheModOwned)
    {
        if (current.Length == 0)
            return actualCacheModOwned
                ? new(previous, true, false)
                : new(null, false, true);

        bool allFinite = true;
        bool allNaN = true;
        for (int i = 0; i < current.Length; i++)
        {
            double anomaly = current[i].CompassTrueAnomaly;
            if (double.IsFinite(anomaly))
                allNaN = false;
            else if (double.IsNaN(anomaly))
                allFinite = false;
            else
            {
                // Infinity or a finite/NaN mixture is foreign/corrupt, not one of
                // either producer's valid cache shapes.
                allFinite = false;
                allNaN = false;
            }
        }

        if (allFinite)
        {
            if (previous is null || previous.Length != current.Length)
                previous = new OrbitPointCce[current.Length];
            current.CopyTo(previous);
            return new(previous, false, true);
        }

        if (allNaN && actualCacheModOwned)
            return new(previous, true, false);

        // All-NaN without explicit debt is foreign, and every mixed/Infinity shape
        // is ambiguous even with debt: discard the snapshot so fallback fails closed.
        return new(null, actualCacheModOwned, false);
    }

    /// <summary>Snapshot-test helper that assumes an outstanding
    /// mod handoff, so mod/empty observations retain the previous exact stock copy.</summary>
    internal static OrbitPointCce[]? UpdateStockSnapshot(
        ReadOnlySpan<OrbitPointCce> current, OrbitPointCce[]? previous) =>
        ObserveStockCache(current, previous, actualCacheModOwned: true).StockPoints;

    private static bool AllCompassAnomaliesFinite(ReadOnlySpan<OrbitPointCce> points)
    {
        for (int i = 0; i < points.Length; i++)
            if (!double.IsFinite(points[i].CompassTrueAnomaly)) return false;
        return points.Length > 0;
    }

    private static StageCache CaptureStageCache(
        Orbit orbit, out long generation, int? overlayWorkerGeneration = null)
    {
        lock (StageCacheHandoffGate)
        {
            while (_stageCacheResetPending)
                System.Threading.Monitor.Wait(StageCacheHandoffGate);
            if (overlayWorkerGeneration is { } workerGeneration
                && workerGeneration != OverlayWorker.CurrentGeneration)
                throw new OperationCanceledException(
                    "overlay worker stage belongs to a stale session");
            generation = _stageCacheGeneration;
            object gate = OrbitCacheGateFor(orbit);
            return _stageCaches.GetValue(orbit, _ => new StageCache(gate));
        }
    }

    private static object OrbitCacheGateFor(Orbit orbit) =>
        OrbitCacheGates.GetValue(orbit, static _ => new object());

    private static void ThrowIfStaleStageGeneration(long generation)
    {
        if (_stageCacheResetPending || generation != _stageCacheGeneration)
            throw new OperationCanceledException("trajectory stage belongs to a stale session");
    }

    private static bool TryBeginStageHandoff(long generation)
    {
        lock (StageCacheHandoffGate)
        {
            if (_stageCacheResetPending || generation != _stageCacheGeneration)
                return false;
            _activeStageHandoffs++;
            return true;
        }
    }

    private static void EndStageHandoff()
    {
        lock (StageCacheHandoffGate)
        {
            if (--_activeStageHandoffs == 0)
                System.Threading.Monitor.PulseAll(StageCacheHandoffGate);
        }
    }

    private static void HandoffStagedPoints(
        Orbit orbit,
        MemoryOwner<OrbitPointCce> points,
        StageCache cache,
        long generation,
        bool markActualModOwnership)
    {
        if (!TryBeginStageHandoff(generation))
        {
            // No handoff occurred, so this owner is still ours to release.
            points.Dispose();
            throw new OperationCanceledException("trajectory stage belongs to a stale session");
        }
        try
        {
            if (markActualModOwnership)
                cache.ActualCacheModOwned = true;
            orbit.UpdateCachedPoints(points);
        }
        finally
        {
            EndStageHandoff();
        }
    }

    private static void InvalidateStageMemo(StageCache cache)
    {
        cache.Samples = null;
        cache.ParentId = null;
        cache.FrameLabel = null;
        cache.SimSeconds = double.NaN;
        cache.AnchorPeSeconds = double.NaN;
        cache.Context = default;
        cache.FirstPosition = default;
    }

    /// <summary>Harmony boundary for every Orbit.UpdateCachedPoints invocation. The
    /// gate is stable across session-table replacement, so stock recalculation, worker
    /// staging, render staging, and fallback restoration can never overlap on one
    /// Orbit. Monitor reentrancy is required for Stage's own patched handoff.</summary>
    internal static object EnterOrbitCacheUpdate(Orbit orbit)
    {
        object gate = OrbitCacheGateFor(orbit);
        System.Threading.Monitor.Enter(gate);
        return gate;
    }

    /// <summary>Harmony-finalizer release for <see cref="EnterOrbitCacheUpdate"/>.
    /// Monitor enforces same-thread ownership; the patch has one unconditional
    /// finalizer and therefore exactly one exit for each successful prefix enter.</summary>
    internal static void ExitOrbitCacheUpdate(object gate) =>
        System.Threading.Monitor.Exit(gate);

    internal static long CaptureStageCacheGenerationForTest()
    {
        lock (StageCacheHandoffGate)
        {
            while (_stageCacheResetPending)
                System.Threading.Monitor.Wait(StageCacheHandoffGate);
            return _stageCacheGeneration;
        }
    }

    internal static bool StageCacheResetPendingForTest
    {
        get
        {
            lock (StageCacheHandoffGate) return _stageCacheResetPending;
        }
    }

    internal static bool TryRunStageHandoffForTest(long generation, Action handoff)
    {
        if (!TryBeginStageHandoff(generation)) return false;
        try
        {
            handoff();
            return true;
        }
        finally
        {
            EndStageHandoff();
        }
    }

    /// <summary>Records the current cache and arms actual-mod ownership as if the
    /// production Stage were immediately about to hand off. Test-only fixture seam;
    /// production performs the same transition inside Stage's generation gate.</summary>
    internal static void PreserveStockCacheForFallback(Orbit orbit)
    {
        var cache = CaptureStageCache(orbit, out long generation);
        lock (cache.Gate)
        {
            var observation = ObserveStockCache(
                orbit.CachedPoints, cache.StockPoints, cache.ActualCacheModOwned);
            lock (StageCacheHandoffGate)
            {
                ThrowIfStaleStageGeneration(generation);
                cache.StockPoints = observation.StockPoints;
                cache.ActualCacheModOwned = true;
            }
        }
    }

    /// <summary>Test fixture handoff through Orbit's real cache API. The owner belongs
    /// to Orbit after UpdateCachedPoints and therefore is deliberately not disposed.</summary>
    internal static void ReplaceCachedPointsForTest(
        Orbit orbit, ReadOnlySpan<OrbitPointCce> source)
    {
        if (source.Length == 0)
            throw new ArgumentException("Orbit.UpdateCachedPoints requires a non-empty buffer.", nameof(source));
        var points = MemoryOwner<OrbitPointCce>.Allocate(source.Length);
        source.CopyTo(points.Span);
        orbit.UpdateCachedPoints(points);
    }

    /// <summary>Makes the orbit cache safe for the stock original and, on success,
    /// returns a same-thread lease that holds both stage handoff gates until Harmony's
    /// postfix/finalizer disposes it. False means no exact safe stock state was
    /// available; failures release both gates before propagating.</summary>
    internal static bool TryAcquireStockFallbackLease(Orbit orbit, out IDisposable? lease)
    {
        lease = null;
        var cache = CaptureStageCache(orbit, out long generation);
        bool cacheGateTaken = false;
        bool handoffGateTaken = false;
        bool transferredToLease = false;
        try
        {
            System.Threading.Monitor.Enter(cache.Gate, ref cacheGateTaken);
            System.Threading.Monitor.Enter(StageCacheHandoffGate, ref handoffGateTaken);
            if (_stageCacheResetPending || generation != _stageCacheGeneration)
                return false;

            var observation = ObserveStockCache(
                orbit.CachedPoints, cache.StockPoints, cache.ActualCacheModOwned);
            cache.StockPoints = observation.StockPoints;
            cache.ActualCacheModOwned = observation.ActualCacheModOwned;

            if (!observation.CurrentCacheIsSafeStock)
            {
                if (!cache.ActualCacheModOwned
                    || cache.StockPoints is not { Length: > 0 } stock
                    || !AllCompassAnomaliesFinite(stock))
                    return false;

                var points = MemoryOwner<OrbitPointCce>.Allocate(stock.Length);
                stock.AsSpan().CopyTo(points.Span);
                // The owner belongs to Orbit from this call onward, including the
                // ambiguous case where UpdateCachedPoints itself throws after assign.
                orbit.UpdateCachedPoints(points);
                cache.ActualCacheModOwned = false;
            }

            InvalidateStageMemo(cache);
            lease = new StockFallbackLease(cache);
            transferredToLease = true;
            return true;
        }
        finally
        {
            if (!transferredToLease)
            {
                if (handoffGateTaken)
                    System.Threading.Monitor.Exit(StageCacheHandoffGate);
                if (cacheGateTaken)
                    System.Threading.Monitor.Exit(cache.Gate);
            }
        }
    }

    /// <summary>Non-leased test helper. Runtime fallback must
    /// use <see cref="TryAcquireStockFallbackLease"/> and retain its lease through the
    /// stock original.</summary>
    internal static bool TryRestoreStockCache(Orbit orbit)
    {
        if (!TryAcquireStockFallbackLease(orbit, out var lease)) return false;
        lease!.Dispose();
        return true;
    }

    /// <summary>Drawn-line world position (game-ecl, mod Vector3d) at an arbitrary
    /// time, interpolated from the batch samples exactly as the map draws them —
    /// the burn-node marker source (BurnNodePatch): a node must sit ON the drawn
    /// polyline, not on the stock conic that drifts off it. False when the time is
    /// outside the batch window, the batch's frame mode no longer matches the display
    /// (the ≤1 s blink window — callers fall back to stock rather than draw a marker
    /// off a line that is not showing), or the pose/rails read fails.</summary>
    internal readonly record struct MarkerDrawContext(
        bool Framed, FramePose Pose, Vector3d ParentNow);

    /// <summary>One pose/parent read for an entire marker batch. Rendering hundreds
    /// of CA dots must not repeat the same rails Gate acquisition per marker.</summary>
    internal static bool TryBuildMarkerDrawContext(
        OverlaySamples samples, out MarkerDrawContext context)
    {
        context = default;
        if (ModServices.Rails is not { } rails) return false;
        bool framed = FrameManager.TryCaptureActive(out var frameSnapshot);
        string? frameLabel = framed ? frameSnapshot.Spec.Label : null;
        if (!OverlayKernel.ModeMatches(samples.FrameLabel, frameLabel)) return false;
        if (samples.DenseFrameCoordinates is not null && framed)
        {
            double tNow = Universe.GetElapsedSimTime().Seconds();
            if (!FrameManager.TrySamplePoseForDisplay(frameSnapshot, tNow, out var pose))
                return false;
            context = new MarkerDrawContext(true, pose, default);
            return true;
        }
        context = new MarkerDrawContext(false, default,
            rails.GetGameEcl(samples.ParentId, Universe.GetElapsedSimTime().Seconds()).Position);
        return true;
    }

    internal static bool TryDrawnPositionAt(OverlaySamples samples, double t,
        in MarkerDrawContext context, out Vector3d gameEcl)
    {
        gameEcl = default;
        // Dense arrays (honest-density lines): the marker/node must sit on the DRAWN
        // polyline, which is the dense one — the decimated chords can sag off it at
        // multi-revolution densities. Dense arrays are unpadded, so [^1] is the end.
        if (samples.DenseTimes.Length == 0) return false;
        if (t < samples.DenseTimes[0] || t > samples.DenseTimes[^1]) return false;
        var (lo, hi, frac) = OverlayKernel.LerpBracket(samples.DenseTimes, t);
        if (context.Framed && samples.DenseFrameCoordinates is { } frameCoordinates)
        {
            var coordinate = frameCoordinates[lo] * (1 - frac) + frameCoordinates[hi] * frac;
            gameEcl = context.Pose.FromFrame(coordinate);
            return true;
        }
        var cce = samples.DensePositionsCce[lo] * (1 - frac) + samples.DensePositionsCce[hi] * frac;
        gameEcl = cce + context.ParentNow;
        return true;
    }

    internal static bool TryDrawnPositionAt(OverlaySamples samples, double t, out Vector3d gameEcl)
    {
        gameEcl = default;
        return TryBuildMarkerDrawContext(samples, out var context)
            && TryDrawnPositionAt(samples, t, in context, out gameEcl);
    }

    /// <summary>Render-phase restage containment: throttled, never throws upward.</summary>
    internal static void NoteRestageContained(Exception e)
    {
        LastNote = $"overlay restage contained: {e.Message}";
        WarnThrottled($"overlay restage contained: {e}");
    }

    private static void WarnThrottled(string message)
    {
        long nowMs = Environment.TickCount64;
        if (nowMs < _nextWarnMs) return;
        _nextWarnMs = nowMs + WarnPeriodMs;
        ModLog.Warn(message);
    }
}

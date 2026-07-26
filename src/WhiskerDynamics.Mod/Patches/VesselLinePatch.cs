using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Honest orbit lines: the vessel draw-site takeover. Every other vessel is
/// suppressed before routing. While the controlled vessel has a FRESH published
/// n-body batch, this prefix routes each FlightPlan.AddLineInstances call three ways
/// on the plan INSTANCE:
/// (1) the vehicle's OWN plan (Vehicle.cs:4359) — stage the batch into patch-0's orbit
/// point cache (the render-phase last-writer role: staged immediately before the
/// read) and draw that single line with
/// joinEnds:false (no fake closing chord on an open arc, Orbit.cs:2231-2234) and stock
/// color semantics (FlightPlan.cs:660-663), then SKIP the original: stock's patch i&gt;=1
/// conics are what "remove the stock orbital lines" removes;
/// (2) one of the vessel's planned burns' plans (BurnPlan.cs:369-382 calls
/// burn.FlightPlan.AddLineInstances once per burn; BurnPlan.AddLineInstances draws
/// nothing itself) — two-line display: the EARLIEST burn's plan
/// is the PLANNED-line canvas — the planned batch (all in-window burns folded, sampled
/// from the first burn) is staged into its patch-0 orbit with payload times
/// re-anchored to THAT orbit's conic (Stage reanchorTimes) and drawn in stock's own
/// planned-burn color (BurnPlan.BurnPatchColor, the user-configurable BurnLineColor
/// setting); every OTHER burn's plan is suppressed without drawing or staging (their
/// post-burn conics are what the planned polyline replaces). Burn node GIZMOS are
/// drawn elsewhere (Burn.Update -&gt; UpdateGizmos, Burn.cs:181-188) and are
/// repositioned onto the drawn lines by BurnNodePatch;
/// (3) any other controlled-vessel plan (the TransferPlanner preview,
/// TransferPlanner.cs:1037 — by design the stock planning tool stays untouched — and
/// any future caller) — original runs, stock draws.
/// The instance routing composes with three-way CONTEXT routing
/// (<see cref="LineRoute"/>): before the own-plan path stages anything, a fresh batch
/// carrying a different frame mode BLINKS — no stage, no draw, stock stays
/// suppressed — until <see cref="TrajectoryOverlay"/>'s context-aware throttle
/// republishes (~1 tick); this gate also makes <see cref="TrajectoryOverlay.Stage"/>'s
/// internal frame-label fallback unreachable from this prefix (kept there as
/// defense-in-depth). SOI-independence: a batch
/// sampled under a different SOI parent is NOT display-gated — Stage re-anchors
/// its parent-centred points by <see cref="OverlayKernel.ParentShift"/>, so the line
/// stays drawn straight across the stock patch transition and the context-aware rebuild
/// merely restores exact new-parent payload semantics within ~a tick.
/// ABSENT batch (never published for the controlled vessel) =&gt; original runs
/// untouched. STALE batch (published but aged out: off-rails windows — burns,
/// physics bubbles) =&gt; ownership rule:
/// patch 0 draws stock-STYLE via <see cref="DrawStalePatch0"/>
/// while everything else stays suppressed, so the conic patches and encounter markers
/// never flash back mid-session. Accepted trade-off: stock AN/DN +
/// closest-approach target markers (computed against the suppressed conics) don't
/// draw while ours is active.
/// The diagnostics panel can explicitly overlay the stock patched-conic lines. The
/// n-body line is drawn first, then the preserved exact stock cache is restored and
/// leased through the stock original. This is display-only: marker and orbit-hit-test
/// routing remain on the honest sampled trajectories.
/// Staging every invocation is idempotent (pure function of the immutable batch);
/// there is deliberately no frame-number de-dup — a duplicate stage costs one
/// pooled 2000-point rebuild, cheaper than the proof a de-dup would need.</summary>
[HarmonyPatch(typeof(FlightPlan), "AddLineInstances")]
internal static class VesselLinePatch
{
    private static int _activeLogged;

    internal enum PlanRoute
    {
        Unclassified,
        Stock,
        Actual,
        Planned,
    }

    /// <summary>The last actual-line operation entered by the prefix. This is real
    /// recovery state, not an exception-message convention: every value below is set
    /// at its production call site before that call can throw, and every actual-route
    /// value requires the same exact-cache lease before stock may run.</summary>
    internal enum ActualLinePhase
    {
        WorkerPreownedCache,
        StageHandoff,
        CameraPreparation,
        BypassVisibilityPreparation,
        DenseLineDraw,
        TakeoverEvidenceTail,
    }

    internal readonly record struct RecoveryState(
        PlanRoute Route, Orbit? ActualOrbit, ActualLinePhase Phase)
    {
        internal static RecoveryState Unclassified => new(PlanRoute.Unclassified, null, default);
        internal static RecoveryState Stock => new(PlanRoute.Stock, null, default);
        internal static RecoveryState Planned => new(PlanRoute.Planned, null, default);
        internal static RecoveryState Actual(Orbit? orbit, ActualLinePhase phase) =>
            new(PlanRoute.Actual, orbit, phase);

        internal RecoveryState At(ActualLinePhase phase) => this with { Phase = phase };
    }

    /// <summary>Allocation-free route-classification operations. The production
    /// implementation reads KSA objects; focused tests can throw at the membership
    /// scan without constructing a live Vehicle.</summary>
    internal interface IPlanRouteOperations
    {
        bool IsActual();
        bool IsPlanned();
    }

    private readonly struct ProductionPlanRouteOperations(
        Vehicle vehicle, FlightPlan candidate) : IPlanRouteOperations
    {
        public bool IsActual() => ReferenceEquals(candidate, vehicle.FlightPlan);
        public bool IsPlanned() => BurnPlanScan.ContainsPlan(vehicle, candidate);
    }

    /// <summary>Classifies an AddLineInstances candidate conservatively. The state is
    /// Unclassified while the burn-membership scan runs, so a scan exception fails
    /// closed; Stock is written only after a successful false result.</summary>
    internal static PlanRoute ClassifyPlanRoute<TOperations>(
        ref RecoveryState recovery, ref TOperations operations)
        where TOperations : struct, IPlanRouteOperations
    {
        recovery = RecoveryState.Unclassified;
        if (operations.IsActual())
        {
            recovery = RecoveryState.Actual(
                orbit: null, ActualLinePhase.WorkerPreownedCache);
            return PlanRoute.Actual;
        }

        if (operations.IsPlanned())
        {
            recovery = RecoveryState.Planned;
            return PlanRoute.Planned;
        }

        recovery = RecoveryState.Stock;
        return PlanRoute.Stock;
    }

    internal readonly struct ActualStageResult(
        TrajectoryOverlay.StagingContext context, byte4 color)
    {
        internal readonly TrajectoryOverlay.StagingContext Context = context;
        internal readonly byte4 Color = color;
    }

    /// <summary>The five fallible actual-line operations after the visibility gate.
    /// A generic struct implementation keeps the production render path allocation-
    /// free while giving tests the exact same orchestration and recovery boundaries.</summary>
    internal interface IActualLineOperations
    {
        bool ShouldDraw { get; }
        ActualStageResult Stage();
        double3 PrepareCamera();
        bool PrepareBypassVisibility();
        void DrawDense(in ActualStageResult stage, in double3 positionEgo, bool bypassVisibility);
        void EvidenceTail();
    }

    private readonly struct ProductionActualLineOperations(
        Vehicle vehicle,
        Viewport viewport,
        Orbit orbit,
        OverlaySamples samples,
        bool isActive,
        bool drawVehiclePosition,
        bool shouldDraw,
        long nowMs,
        double nowSimSeconds) : IActualLineOperations
    {
        public bool ShouldDraw => shouldDraw;

        public ActualStageResult Stage()
        {
            var context = TrajectoryOverlay.Stage(
                samples, orbit, preserveStockForFallback: true);
            byte4 color = isActive
                ? orbit.OrbitLineColor
                : (byte4)FlightPlan.InactiveColor;
            return new ActualStageResult(context, color);
        }

        public double3 PrepareCamera() => drawVehiclePosition
            ? viewport.GetCamera().GetPositionEgo(vehicle)
            : Double3Ex.NaN;

        public bool PrepareBypassVisibility() =>
            LineVisibility.BypassOrbitVisibilityCheck(viewport, isActive);

        public void DrawDense(
            in ActualStageResult stage, in double3 positionEgo, bool bypassVisibility)
        {
            var context = stage.Context;
            DenseLineDraw.Draw(viewport, orbit, samples.DenseTimes,
                samples.DensePositionsCce, samples.DenseFrameCoordinates,
                samples.DenseMetrics, samples.DenseMetricsCce, in context,
                stage.Color, positionEgo, nowSimSeconds,
                bypassVisibilityCheck: bypassVisibility);
        }

        public void EvidenceTail()
        {
            // Plan-snapshot ghost with NO stock burn left to canvas it (mid-burn
            // consumption, fully-flown plan): draw it through the own-plan pass.
            DrawPlannedFallback(vehicle, viewport, isActive, nowMs, nowSimSeconds);
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"vessel line takeover active (first: '{vehicle.Id}'; "
                    + "stock conic patches suppressed while n-body lines are fresh)");
        }
    }

    /// <summary>Runs the exact post-visibility production sequence. RecoveryState is
    /// advanced before each operation, so an operation that overwrites the cache and
    /// then throws is attributed to that boundary before stock fallback is considered.
    /// Tests execute this same method with a faulting struct implementation.</summary>
    internal static bool ExecuteActualTakeover<TOperations>(
        ref RecoveryState recovery,
        ref TOperations operations,
        out IDisposable? lease,
        Action<Exception>? reporter = null)
        where TOperations : struct, IActualLineOperations
    {
        lease = null;
        try
        {
            if (operations.ShouldDraw)
            {
                recovery = recovery.At(ActualLinePhase.StageHandoff);
                ActualStageResult stage = operations.Stage();

                recovery = recovery.At(ActualLinePhase.CameraPreparation);
                double3 positionEgo = operations.PrepareCamera();

                recovery = recovery.At(ActualLinePhase.BypassVisibilityPreparation);
                bool bypassVisibility = operations.PrepareBypassVisibility();

                recovery = recovery.At(ActualLinePhase.DenseLineDraw);
                operations.DrawDense(in stage, in positionEgo, bypassVisibility);
            }

            recovery = recovery.At(ActualLinePhase.TakeoverEvidenceTail);
            operations.EvidenceTail();
            return false; // successful takeover suppresses the stock original
        }
        catch (Exception error)
        {
            return RecoverFailure(in recovery, error, out lease, reporter);
        }
    }

    /// <summary>Harmony runs a postfix on success and a finalizer on both success and
    /// failure. Both may therefore observe the same __state. Keep the StageCache gate
    /// lease one-shot so either ordering releases it exactly once.</summary>
    private sealed class OneShotLease(IDisposable inner) : IDisposable
    {
        private IDisposable? _inner = inner;

        public void Dispose()
        {
            var lease = System.Threading.Interlocked.Exchange(ref _inner, null);
            if (lease is null) return;
            try { lease.Dispose(); }
            catch (Exception e) { ReportContained(e); }
        }
    }

    /// <summary>Plan-snapshot fallback canvases, one per vessel: the
    /// planned line normally draws through the EARLIEST stock burn's plan orbit, but
    /// mid-burn (the flight computer consumed the executing burn) or after the whole
    /// plan flew, the stock plan has NO burns and that canvas doesn't exist — the
    /// frozen ghost still must draw. Mod-owned display orbits, recreated on parent
    /// change, swept on rebind.</summary>
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, Orbit> PlannedCanvas =
        new(StringComparer.Ordinal);

    /// <summary>Best-effort exact restoration for every stock-owned canvas the mod may
    /// have staged. False leaves the draw prefix responsible for fail-closed suppression.</summary>
    internal static bool RestoreStockCachesForFallback(Vehicle vehicle)
    {
        bool restored = false;
        try
        {
            restored = TrajectoryOverlay.TryRestoreStockCache(vehicle.Orbit);
            var burnPlan = vehicle.FlightComputer.BurnPlan;
            for (int i = 0; i < burnPlan.BurnCount; i++)
            {
                if (!burnPlan.TryGetBurn(i, out Burn? burn) || burn is null) continue;
                var patches = burn.FlightPlan.Patches;
                if (patches.Count > 0)
                    restored &= TrajectoryOverlay.TryRestoreStockCache(patches[0].Orbit);
            }
        }
        catch (Exception e)
        {
            ReportContained(e);
            restored = false;
        }
        return restored;
    }

    internal static void ResetSessionStatics()
    {
        System.Threading.Volatile.Write(ref _activeLogged, 0);
        PlannedCanvas.Clear();
    }

    static bool Prefix(FlightPlan __instance, Viewport viewport, IOrbiter orbiter, bool isActive,
        bool drawVehiclePosition, TrueAnomaly startTa, TrueAnomaly nextBurnTa,
        out IDisposable? __state)
    {
        __state = null;
        if (orbiter is not Vehicle vehicle) return true;
        if (!ModServices.Enabled)
            return RunDisabledRoute(vehicle, __instance, out __state);
        if (!LineVisibility.IsControlled(vehicle)) return false;
        RecoveryState recovery = RecoveryState.Unclassified;
        try
        {
            // Classify INSTANCE ownership before any Universe/freshness/camera work.
            // Unrelated plans stay stock. Planned-route failures are contained by
            // RunPlannedRoute and can therefore never bubble into actual-cache
            // recovery or expose a planned staged cache to stock.
            var routeOperations = new ProductionPlanRouteOperations(vehicle, __instance);
            switch (ClassifyPlanRoute(ref recovery, ref routeOperations))
            {
                case PlanRoute.Stock:
                    return true;
                case PlanRoute.Planned:
                    return RunPlannedRoute(
                        vehicle, __instance, viewport, isActive, out __state);
            }

            // Arm the actual orbit immediately after route classification. The worker
            // may already have staged this cache, so recovery ownership cannot depend
            // on whether this particular render invocation reached Stage.
            var orbit = vehicle.Orbit;
            recovery = RecoveryState.Actual(
                orbit, ActualLinePhase.WorkerPreownedCache);
            // OWNERSHIP, not freshness, decides the routing: otherwise stock conic
            // patches and encounter markers would flash back during
            // every stale window — burns, physics bubbles, scene churn. Once a batch
            // was EVER published for this vessel, the mod owns its display: the stock
            // extras (patch i>=1 conics, post-burn plan conics) stay suppressed even
            // while the batch is stale; only patch 0 degrades to a stock-STYLE draw
            // (below) so a line always remains. If the controlled vessel has no batch
            // at all (never tracked, or right after the session sweep), it stays stock.
            var samples = OverlayBuffer.Read(vehicle.Id);
            if (samples is null)
                return true;
            long nowMs = Environment.TickCount64;
            double nowSimSeconds = Universe.GetElapsedSimTime().Seconds();
            bool fresh = OverlayBuffer.LineSamplesUsable(
                vehicle.Id, samples, planned: false, nowMs, nowSimSeconds);

            if (!fresh)
            {
                // In diagnostics mode prefer stock's whole plan. Restore/lease first;
                // if no exact stock cache is available, retain the ordinary safe
                // patch-0 fallback below rather than expose a staged n-body cache to
                // stock's closed-conic renderer.
                if (DiagnosticDisplay.ShowStockPatchedConics
                    && CompleteDisplayRoute(
                        orbit, runOriginal: false,
                        showStockPatchedConics: true, ref __state))
                    return true;

                // Stale fallback: draw ONLY patch 0 the stock way (stock colors,
                // stock gates — mirrors FlightPlan.AddLineInstances:647-673 for one
                // patch) and keep the rest suppressed. The patch-0 cache holds either
                // our last staged points (briefly) or stock's regenerated conic
                // points (RecalculateFlightPlan/re-osculation refresh them), so a
                // usable line persists through off-rails windows without the conic
                // patch/marker flashback. INERTIAL VIEW ONLY:
                // under an active display frame the camera is counter-posed, so both
                // possible cache contents are wrong there — stock conic points are
                // inertial-embedded (drawn rotated by the counter-pose) and our stale
                // points are frozen at an old pose. No honest line exists mid-burn in
                // frame mode; drawing nothing until the rails rebuild resumes (~burn
                // end + ≤1 s) beats drawing wrong-frame geometry. "Inertial view"
                // includes a body-centred inertial display frame (no counter-pose) —
                // under the always-a-frame policy that IS the everyday view.
                if (FrameManager.InertialView)
                    DrawStalePatch0(vehicle, __instance, viewport, isActive,
                        drawVehiclePosition, startTa, nextBurnTa);
                return false;
            }

            // A stale SOI parent is deliberately not an input: Stage re-anchors
            // an old-parent batch by OverlayKernel.ParentShift so the line draws
            // straight across the transition instead of blinking.
            bool modeMatches = OverlayKernel.ModeMatches(samples.FrameLabel, FrameManager.Active?.Label);
            if (!modeMatches)
            {
                // The batch coordinates belong to the wrong frame mode. Suppress the
                // line until TrajectoryOverlay republishes for the current context.
                return CompleteDisplayRoute(
                    orbit, runOriginal: false,
                    DiagnosticDisplay.ShowStockPatchedConics, ref __state);
            }

            // Stock's own first gate (FlightPlan.cs:647): an orbit the player hid stays
            // hidden — but the stock conics stay suppressed (false), not re-drawn.
            if (!vehicle.ShowOrbit && !vehicle.TargetOfControlledVehicle)
                return CompleteDisplayRoute(
                    orbit, runOriginal: false,
                    DiagnosticDisplay.ShowStockPatchedConics, ref __state);

            // Visibility is evaluated before any 2000-point handoff. The generic
            // orchestration then advances recovery state immediately before each
            // fallible operation without allocating delegates on this render path.
            bool shouldDraw = LineVisibility.ForVessel(vehicle, viewport);
            var actualOperations = new ProductionActualLineOperations(
                vehicle, viewport, orbit, samples, isActive, drawVehiclePosition,
                shouldDraw, nowMs, nowSimSeconds);
            bool runOriginal = ExecuteActualTakeover(
                ref recovery, ref actualOperations, out __state);
            return CompleteDisplayRoute(
                orbit, runOriginal,
                DiagnosticDisplay.ShowStockPatchedConics, ref __state);
        }
        catch (Exception e)
        {
            return RecoverFailure(in recovery, e, out __state);
        }
    }

    private static bool RunDisabledRoute(
        Vehicle vehicle, FlightPlan plan, out IDisposable? lease)
    {
        lease = null;
        try
        {
            RecoveryState recovery = RecoveryState.Unclassified;
            var operations = new ProductionPlanRouteOperations(vehicle, plan);
            switch (ClassifyPlanRoute(ref recovery, ref operations))
            {
                case PlanRoute.Stock:
                    return true;
                case PlanRoute.Planned:
                    if (plan.Patches.Count == 0) return true;
                    return TryRunStockWithCacheLease(plan.FirstPatch.Orbit, out lease);
                default:
                    return TryRunStockWithCacheLease(vehicle.Orbit, out lease);
            }
        }
        catch (Exception e)
        {
            ReportContained(e);
            return false;
        }
    }

    private static bool TryRunStockWithCacheLease(Orbit orbit, out IDisposable? lease)
    {
        lease = null;
        if (!TrajectoryOverlay.TryAcquireStockFallbackLease(orbit, out var acquired)
            || acquired is null) return false;
        lease = new OneShotLease(acquired);
        return true;
    }

    /// <summary>Finishes an owned draw route. Ordinary mode preserves the takeover's
    /// original decision. Diagnostics mode adds the stock original only after its
    /// canvas has an exact stock cache and a lease excludes worker restaging through
    /// Harmony's postfix/finalizer. A takeover failure that already selected stock
    /// and acquired a lease passes through unchanged.</summary>
    internal static bool CompleteDisplayRoute(
        Orbit? stockOrbit,
        bool runOriginal,
        bool showStockPatchedConics,
        ref IDisposable? lease)
    {
        if (runOriginal || !showStockPatchedConics) return runOriginal;
        if (stockOrbit is null) return true;
        return TryRunStockWithCacheLease(stockOrbit, out lease);
    }

    /// <summary>The exact stock cache remains protected from worker restaging until
    /// the stock original has finished reading it. The success postfix handles the
    /// ordinary path; the finalizer handles an original that throws. OneShotLease
    /// makes their overlap harmless.</summary>
    static void Postfix(IDisposable? __state) => ReleaseFallbackLease(__state);

    static Exception? Finalizer(Exception? __exception, IDisposable? __state)
    {
        ReleaseFallbackLease(__state);
        return __exception;
    }

    /// <summary>Shared by both Harmony cleanup paths and focused interleaving tests.
    /// The lease is one-shot because success can execute both postfix and finalizer.</summary>
    internal static void ReleaseFallbackLease(IDisposable? lease) => lease?.Dispose();

    /// <summary>The prefix's one stock-fallback decision. Actual-route failure may
    /// run the original only after an exact safe stock cache is restored or validated
    /// and leased through that original. Unclassified and planned routes fail closed;
    /// only a positively classified unrelated route may run stock without a lease.</summary>
    internal static bool RecoverFailure(
        in RecoveryState recovery, Exception error, out IDisposable? lease,
        Action<Exception>? reporter = null)
    {
        lease = null;

        // Planned failures are fail-closed: stock post-burn conics must never read a
        // planned staged cache. Stock and unrelated routes keep stock behavior.
        if (recovery.Route != PlanRoute.Actual)
        {
            ReportContained(error, reporter);
            return recovery.Route == PlanRoute.Stock;
        }

        IDisposable? acquired = null;
        try
        {
            if (recovery.ActualOrbit is null
                || !TrajectoryOverlay.TryAcquireStockFallbackLease(
                    recovery.ActualOrbit, out acquired)
                || acquired is null)
            {
                acquired?.Dispose();
                ReportContained(error, reporter);
                return false;
            }

            lease = new OneShotLease(acquired);
            acquired = null; // ownership transferred to Harmony __state
        }
        catch (Exception restoreError)
        {
            try { acquired?.Dispose(); } catch { }
            ReportContained(restoreError, reporter);
            ReportContained(error, reporter);
            return false;
        }

        // Restore and gate acquisition precede diagnostics: even a broken logger
        // cannot expose the n-body payload or release the worker exclusion early.
        ReportContained(error, reporter);
        return true;
    }

    private static void ReportContained(
        Exception error, Action<Exception>? reporter = null)
    {
        try
        {
            if (reporter is null) TrajectoryOverlay.NoteRestageContained(error);
            else reporter(error);
        }
        catch { }
    }

    /// <summary>Planned plan ownership is resolved before this call. Once inside,
    /// every failure degrades to no planned line for the frame; it never escapes to
    /// Prefix's actual-route stock recovery.</summary>
    private static bool RunPlannedRoute(
        Vehicle vehicle, FlightPlan plan, Viewport viewport, bool isActive,
        out IDisposable? lease)
    {
        lease = null;
        try
        {
            var samples = OverlayBuffer.Read(vehicle.Id);
            if (samples is null)
                return true;
            long nowMs = Environment.TickCount64;
            double nowSimSeconds = Universe.GetElapsedSimTime().Seconds();
            bool fresh = OverlayBuffer.LineSamplesUsable(
                vehicle.Id, samples, planned: false, nowMs, nowSimSeconds);
            if (fresh)
                DrawPlannedLine(vehicle, plan, viewport, isActive, nowMs, nowSimSeconds);
            Orbit? stockOrbit = plan.Patches.Count > 0
                ? plan.FirstPatch.Orbit
                : null;
            return CompleteDisplayRoute(
                stockOrbit, runOriginal: false,
                DiagnosticDisplay.ShowStockPatchedConics, ref lease);
        }
        catch (Exception e)
        {
            ReportContained(e);
            return false;
        }
    }

    /// <summary>Stale-window fallback: patch 0 drawn the STOCK way (stock's own gates
    /// and colors, FlightPlan.cs:647-673 mirrored for one patch) while every other
    /// stock conic stays suppressed — the ownership rule's "a line always remains"
    /// half. joinEnds:true matches stock's closed-ellipse rendering for conic points
    /// (our stale staged points draw with a harmless closing chord for the ≤seconds
    /// until stock's recalc replaces them).</summary>
    private static void DrawStalePatch0(Vehicle vehicle, FlightPlan plan, Viewport viewport,
        bool isActive, bool drawVehiclePosition, TrueAnomaly startTa, TrueAnomaly nextBurnTa)
    {
        if (!vehicle.ShowOrbit && !vehicle.TargetOfControlledVehicle) return;
        var patches = plan.Patches;
        if (patches.Count == 0) return;
        var patch0 = patches[0];
        var orbit = patch0.Orbit;
        if (patch0.HidePatch || !LineVisibility.ForVessel(vehicle, viewport)) return;
        byte4 color = isActive ? orbit.OrbitLineColor : (byte4)FlightPlan.InactiveColor;
        double3 positionEgo = drawVehiclePosition
            ? viewport.GetCamera().GetPositionEgo(vehicle)
            : Double3Ex.NaN;
        orbit.DrawLines(viewport, positionEgo, SimTime.Zero, color, startTa, nextBurnTa,
            joinEnds: true,
            bypassVisibilityCheck:
                LineVisibility.BypassOrbitVisibilityCheck(viewport, isActive),
            fadeOpacity: true);
    }

    /// <summary>Two-line display: draw the PLANNED trajectory through
    /// <paramref name="plan"/>'s patch-0 orbit iff that plan belongs to the vessel's
    /// EARLIEST burn (every other burn plan is suppressed by the caller). Gates,
    /// colors and containment live in <see cref="StageAndDrawPlanned"/>, shared with
    /// the burnless-ghost fallback.</summary>
    private static void DrawPlannedLine(Vehicle vehicle, FlightPlan plan, Viewport viewport,
        bool isActive, long nowMs, double nowSimSeconds)
    {
        try
        {
            if (BurnPlanScan.EarliestBurn(vehicle) is not { } earliest
                || !ReferenceEquals(earliest.FlightPlan, plan))
                return;
            if (plan.Patches.Count == 0) return; // plan mid-recalculation: no canvas this frame
            StageAndDrawPlanned(
                vehicle, viewport, isActive, plan.FirstPatch.Orbit, nowMs, nowSimSeconds,
                preserveStockForFallback: true);
        }
        catch (Exception e)
        {
            // Contained HERE, not in the Prefix's catch: a planned-line failure (a
            // rails/pose read throwing inside the 5 s staleness window) must degrade
            // to "no planned line this frame" — bubbling up would return true from
            // the prefix and flash the stock post-burn conics this branch suppresses.
            ReportContained(e);
        }
    }

    /// <summary>Plan-snapshot ghost without a stock burn canvas: only when the stock
    /// plan has NO burns at all (the flight computer consumed the executing burn;
    /// a fully-flown plan whose ghost still differs from reality) — otherwise the
    /// earliest burn's own AddLineInstances call is the canvas, unchanged. Staged
    /// into the mod-owned per-vessel canvas orbit; hover is not wired on it (there
    /// is no burn to drag on a ghost). Gate order: the cheap
    /// dictionary read rejects the common no-planned-batch case before the burn-plan
    /// scan runs.</summary>
    private static void DrawPlannedFallback(Vehicle vehicle, Viewport viewport, bool isActive,
        long nowMs, double nowSimSeconds)
    {
        try
        {
            if (OverlayBuffer.ReadPlanned(vehicle.Id) is null) return; // common case: nothing published
            if (BurnPlanScan.EarliestBurn(vehicle) is not null) return; // the burn canvas draws it
            if (FallbackCanvas(vehicle) is not { } orbit) return;
            StageAndDrawPlanned(vehicle, viewport, isActive, orbit, nowMs, nowSimSeconds,
                preserveStockForFallback: false);
        }
        catch (Exception e)
        {
            // Same containment story as DrawPlannedLine: degrade to "no ghost this
            // frame", never bubble into the prefix (a stock-conic flashback).
            ReportContained(e);
        }
    }

    /// <summary>The ONE planned-line draw: fresh matching-mode batch staged into the
    /// given canvas orbit with payload times re-anchored, drawn in stock's own
    /// planned-burn color semantics (BurnPlan.BurnPatchColor when the camera follows
    /// this vessel, FlightPlan.InactiveColor otherwise — mirrors the main line's
    /// isActive rule). No batch, a stale one, or a frame-mode mismatch (the ≤1 s
    /// blink, shared with the main line) draws nothing — the caller suppresses the
    /// stock conics either way.</summary>
    private static void StageAndDrawPlanned(Vehicle vehicle, Viewport viewport, bool isActive,
        Orbit orbit, long nowMs, double nowSimSeconds, bool preserveStockForFallback)
    {
        var planned = OverlayBuffer.ReadPlanned(vehicle.Id);
        if (planned is null || !OverlayBuffer.LineSamplesUsable(
                vehicle.Id, planned, planned: true, nowMs, nowSimSeconds)) return;
        if (!OverlayKernel.ModeMatches(planned.FrameLabel, FrameManager.Active?.Label))
            return; // wrong frame mode: blink with the main line until the rebuild republishes
        if (!vehicle.ShowOrbit && !vehicle.TargetOfControlledVehicle) return; // stock's own first gate
        if (!LineVisibility.ForVessel(vehicle, viewport)) return;
        var ctx = TrajectoryOverlay.Stage(planned, orbit, reanchorTimes: true,
            preserveStockForFallback: preserveStockForFallback);
        byte4 color = isActive ? (byte4)BurnPlan.BurnPatchColor : (byte4)FlightPlan.InactiveColor;
        // Honest-density draw: NaN current position = no splice vertex (a planned
        // line has no vehicle riding it), matching stock's NaN positionEgo convention.
        DenseLineDraw.Draw(viewport, orbit, planned.DenseTimes, planned.DensePositionsCce,
            planned.DenseFrameCoordinates, planned.DenseMetrics, planned.DenseMetricsCce,
            in ctx, color, Double3Ex.NaN, nowSimSeconds,
            minimumTimeSeconds: nowSimSeconds,
            bypassVisibilityCheck:
                LineVisibility.BypassOrbitVisibilityCheck(viewport, isActive));
    }

    /// <summary>The vessel's mod-owned planned-line canvas, parented at its CURRENT
    /// orbit parent (Stage's ParentShift re-anchors an old-parent batch against it,
    /// same as every staging site). The conic itself is irrelevant — the staged batch
    /// replaces its cached points; only Parent and TimeAtPeriapsis (the payload
    /// re-anchor, consistent between Stage and the DrawLines call) are read.</summary>
    private static Orbit? FallbackCanvas(Vehicle vehicle)
    {
        if (vehicle.Orbit.Parent is not Astronomical parent) return null;
        if (PlannedCanvas.TryGetValue(vehicle.Id, out var canvas)
            && canvas.Parent is Astronomical canvasParent
            && string.Equals(canvasParent.Id, parent.Id, StringComparison.Ordinal))
            return canvas;
        ref readonly var sv = ref vehicle.Orbit.StateVectors;
        var fresh = Orbit.CreateFromStateCci(vehicle.Orbit.Parent, sv.StateTime,
            sv.PositionCci, sv.VelocityCci, vehicle.Orbit.OrbitLineColor);
        PlannedCanvas[vehicle.Id] = fresh;
        return fresh;
    }
}

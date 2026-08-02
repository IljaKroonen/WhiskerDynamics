using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Ui;

internal enum CompletedOptimizeRoute
{
    ApplyForOwner,
    PreserveForOwner,
}

internal static class OptimizeRoutingPolicy
{
    internal static CompletedOptimizeRoute RouteCompleted(bool ownerDrawn) =>
        ownerDrawn
            ? CompletedOptimizeRoute.ApplyForOwner
            : CompletedOptimizeRoute.PreserveForOwner;
}

/// <summary>Flight-plan editor over the controlled vessel's STOCK
/// BurnPlan (still the ONE execution source of truth, written exclusively through
/// BurnPlanWriter): create/delete a plan with a LENGTH that drives how far the vessel's
/// trajectory is predicted/drawn (TrajectoryOverlay reads it via
/// FlightPlans.EffectiveHorizonDays). A plan starts with NO burns and no phantom form:
/// the Add-burn button creates a zero-dv burn a fixed lead ahead, authored in whatever
/// display frame the map is in at that moment, and every setting is then edited on the
/// burn itself. Burns author in ANY reference frame — VLF (stock
/// prograde/normal/outward) or any catalog frame (body-centred inertial / two-body
/// fixed / body surface), picked in a frame-picker window carrying the SAME body tree
/// as the frames panel; frame axes are sampled at the BURN TIME and converted to stock
/// VLF components against the PREDICTED n-body pre-burn state (PlannedBurnConverter;
/// evidence in BurnFrameKernel's header). The mod-level plan metadata (FlightPlans)
/// rides alongside the stock plan keyed by burn time and persists through the save
/// sidecar. Runs in the main-thread UI draw phase via StatusPanelPatch — the exact
/// phase stock's own burn editor mutates from; live Vehicle resolved from the registry
/// (Risk 8f); 3-strike self-disable and balanced Begin/End mirror StatusPanel.</summary>
public static class BurnPlannerPanel
{
    /// <summary>How far ahead of NOW the Add-burn button places a new burn — far
    /// enough past PlannerKernel.MinLeadSeconds to survive edits under mild warp.</summary>
    private const double AddLeadSeconds = 600;

    /// <summary>Button label composed once (hoisted-constant convention: the panel
    /// draws it every rendered frame and it can never change).</summary>
    private static readonly string AddBurnLabel =
        $"Add burn (zero dv, T+{TimeDisplayKernel.FormatDuration(AddLeadSeconds)})";

    private const string VlfLabel = "VLF (prograde/normal/outward)";

    private static int _errors;
    private static bool _open;
    private static string _status = "";
    private static bool _firstDrawLogged;
    private static string _burnSnapshotVessel = string.Empty;
    private static long _burnSnapshotSignature = long.MinValue;
    private static IReadOnlyList<Burn> _burnSnapshot = [];

    private static IReadOnlyList<Burn> BurnsForFrame(Vehicle vehicle)
    {
        var plan = vehicle.FlightComputer.BurnPlan;
        long signature = plan.BurnCount;
        bool identitiesMatch = string.Equals(
            _burnSnapshotVessel, vehicle.Id, StringComparison.Ordinal);
        int liveIndex = 0;
        unchecked
        {
            for (int i = 0; i < plan.BurnCount; i++)
                if (plan.TryGetBurn(i, out Burn? burn) && burn is not null)
                {
                    if (identitiesMatch
                        && (liveIndex >= _burnSnapshot.Count
                            || !ReferenceEquals(_burnSnapshot[liveIndex], burn)))
                        identitiesMatch = false;
                    liveIndex++;
                    signature = signature * 397 ^ BitConverter.DoubleToInt64Bits(burn.Time.Seconds());
                    signature = signature * 397 ^ BitConverter.DoubleToInt64Bits(burn.DeltaVVlf.X);
                    signature = signature * 397 ^ BitConverter.DoubleToInt64Bits(burn.DeltaVVlf.Y);
                    signature = signature * 397 ^ BitConverter.DoubleToInt64Bits(burn.DeltaVVlf.Z);
                }
        }
        identitiesMatch &= liveIndex == _burnSnapshot.Count;
        if (string.Equals(_burnSnapshotVessel, vehicle.Id, StringComparison.Ordinal)
            && signature == _burnSnapshotSignature && identitiesMatch)
            return _burnSnapshot;
        _burnSnapshotVessel = vehicle.Id;
        _burnSnapshotSignature = signature;
        return _burnSnapshot = BurnPlanWriter.Snapshot(vehicle);
    }

    // Frame-picker window: which vessel's burn (keyed by burn time — the plan-meta
    // key) is choosing its authoring frame; null vessel = closed. The tree control
    // instance keeps the picker's own expansion state, independent of the frames panel.
    private static string? _pickerVesselId;
    private static double _pickerBurnTime;
    private static readonly FrameTreeControl PickerTree = new();

    // Periapsis optimizer: desired periapsis ALTITUDE (km) over the target body.
    private static double _peTargetKm = 100;
    private static bool _optimizeInclination;
    private static double _inclinationTargetDegrees;

    // Conversion cache: one whole-plan pass (PlannedBurnConverter.Analyze) reused until
    // the plan changes (exact snapshots of times + dv + metas) or it ages out — a per-frame
    // predictor fold on the UI thread would be waste, and nothing it reads changes
    // faster than the signature. ONE nullable record with ONE reset shape (parallel
    // statics with partial resets would invite stale reads), and a null (failed)
    // Analyze result is a cache entry too (an uncached failure would re-run the
    // whole Gate-held conversion pass every rendered frame).
    private const long AnalysisMaxAgeMs = 10_000;

    private readonly record struct AnalysisBurnShape(
        double Time, double X, double Y, double Z);

    private readonly record struct AnalysisMetaShape(
        double Time, FrameKind Kind, string PrimaryId, string? SecondaryId,
        double X, double Y, double Z);

    private sealed record AnalysisCache(string Vessel,
        AnalysisBurnShape[] Burns, AnalysisMetaShape[] Metas, long WallMs,
        IReadOnlyList<PlannedBurnConverter.BurnAnalysis>? Results);

    private static AnalysisCache? _analysis;
    private static double _newPlanLengthSeconds = FlightPlans.DefaultLengthSeconds;

    private sealed class RendezvousContext
    {
        public required RendezvousSolveJob Job { get; init; }
        public required string VesselId { get; init; }
        public required string TargetId { get; init; }
        public required VesselRegistry.RailsAuthoritySnapshot ChaserAuthority { get; init; }
        public required VesselRegistry.RailsAuthoritySnapshot TargetAuthority { get; init; }
        public FlightPlanModel? CreatedPlan { get; set; }
        public Burn? FirstBurn { get; set; }
        public Burn? SecondBurn { get; set; }
        public double PlanLengthSeconds { get; set; }
        /// <summary>0 solving; 1 first queued; 2 second queued.</summary>
        public int Stage { get; set; }
        public long StageStartedMs { get; set; }
    }

    private static RendezvousContext? _rendezvous;
    private static SidecarPendingRendezvous? _pendingCleanup;
    private static bool _pendingCleanupQueued;

    internal static void Open() => _open = true;

    /// <summary>Statics sweep: fresh picker, status and conversion cache for the
    /// new session (the FlightPlans store itself is swept separately).</summary>
    internal static void ResetSessionStatics()
    {
        _open = false;
        _errors = 0;
        _peTargetKm = 100;
        _optimizeInclination = false;
        _inclinationTargetDegrees = 0;
        _newPlanLengthSeconds = FlightPlans.DefaultLengthSeconds;
        _optimizerCaption = null;
        _status = "";
        _firstDrawLogged = false;
        _burnSnapshotVessel = string.Empty;
        _burnSnapshotSignature = long.MinValue;
        _burnSnapshot = [];
        DurationField.ResetSessionStatics();
        BasisReconversionUrgency.Reset();
        InvalidateAnalysis();
        ClosePicker();
        PickerTree.Reset();
        // An in-flight optimize belongs to the OLD session: cancel it and drop the
        // apply context so its result can never write into the new session's plan
        // (the cross-session Burn/Vehicle references would be stale).
        _optimize?.Job.Cancel();
        _optimize = null;
        _rendezvous?.Job.Cancel();
        _rendezvous = null;
        _pendingCleanup = null;
        _pendingCleanupQueued = false;
    }

    private static void InvalidateAnalysis() => _analysis = null;

    /// <summary>Cadence and warn floors for the execution-basis upkeep: the check
    /// clones a predictor per burn, so it runs on a wall cadence, not every frame.</summary>
    private static long _nextBasisUpkeepMs;
    private static long _nextBasisReconvertWarnMs;

    /// <summary>Execution-basis upkeep for frame-authored burns: stock freezes the
    /// executed CCI target from the burn-time patch conic, so an SOI handoff or
    /// conic extrapolation error silently rotates the executed direction (observed:
    /// 12-25 deg off at Luna perilune). When the stored components drift beyond
    /// <see cref="PlannerKernel.ExecutionRealizeToleranceMps"/> from the authored
    /// intent realized against the current stock basis, this rewrites them — the
    /// carve-out from <see cref="PlannedBurnConverter"/>'s no-automatic-rewrite
    /// rule. Every rewrite also records the predictor-basis realization (snapshot
    /// DisplayDvVlf + meta ExecutionDvVlf) so the display fold and analysis don't
    /// read the stock-basis components as predictor-basis; the within-tolerance
    /// pass re-adopts both after a load or reconcile recapture. Runs panel open or
    /// not.</summary>
    private static void AdvanceBasisReconversion(VesselRegistry vessels)
    {
        bool cadenceDue = Environment.TickCount64 >= _nextBasisUpkeepMs;
        if (!cadenceDue && !BasisReconversionUrgency.Any) return;
        if (cadenceDue) _nextBasisUpkeepMs = Environment.TickCount64 + 1000;
        double now = Universe.GetElapsedSimTime().Seconds();
        var upkeep = FlightPlans.SnapshotForUpkeep();
        ClearOrphanedUrgency(upkeep);
        foreach (var (vesselId, plan) in upkeep)
        {
            long? urgency = BasisReconversionUrgency.Observe(vesselId);
            if (!cadenceDue && urgency is null) continue;
            // A conversion folded on a diverged predictor would be wrong; rebase
            // owns that recovery.
            if (plan.Meta.Count == 0 || plan.Diverged)
            {
                BasisReconversionUrgency.Clear(vesselId);
                continue;
            }
            Vehicle? vehicle = vessels.TryGetLiveVehicle(vesselId);
            if (vehicle is null) continue;
            if (!PlannedBurnConverter.ExistingBurnParentsReady(vehicle)) continue;
            IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vehicle);
            bool settled = true;
            foreach (Burn burn in burns)
            {
                double t = burn.Time.Seconds();
                if (plan.TryGetMetaAt(t) is not { } meta) continue;
                string? resolved =
                    PlannedBurnConverter.ExistingBurnParentId(vehicle, burn);
                if (resolved is null)
                {
                    settled = false;
                    continue;
                }
                if (meta.BasisParentId is null)
                    meta.BasisParentId = resolved;
                bool flipped = !string.Equals(meta.BasisParentId, resolved,
                    StringComparison.Ordinal);
                if (!PlannerKernel.SafelyAheadForRewrite(t, now))
                {
                    if (flipped && Environment.TickCount64 >= _nextBasisReconvertWarnMs)
                    {
                        _nextBasisReconvertWarnMs = Environment.TickCount64 + 5000;
                        ModLog.Warn($"planner: burn at t={t:F1} s for '{vesselId}' executes "
                            + $"in basis '{resolved}' but its components were realized for "
                            + $"'{meta.BasisParentId}'; too close to ignition to re-realize - "
                            + "expect a rotated burn direction");
                    }
                    continue;
                }
                if (PlannedBurnConverter.TryAuthorDvVlfForExecution(vessels, vehicle,
                        vehicle.Orbit, OthersExcept(burns, burn), burn, t, meta.Frame,
                        meta.Authored, now, out var dvVlf,
                        out var predictorDvVlf) is { } refusal)
                {
                    settled = false;
                    if (Environment.TickCount64 >= _nextBasisReconvertWarnMs)
                    {
                        _nextBasisReconvertWarnMs = Environment.TickCount64 + 5000;
                        ModLog.Warn($"planner: execution-basis upkeep deferred for "
                            + $"'{vesselId}' burn at t={t:F1} s: {refusal}");
                    }
                    continue;
                }
                if (!flipped && !BurnFrameKernel.IsStale(FrameAdapter.ToCore(dvVlf),
                        FrameAdapter.ToCore(burn.DeltaVVlf),
                        PlannerKernel.ExecutionRealizeToleranceMps))
                {
                    // Already realized within tolerance: re-adopt the session-only
                    // bookkeeping (needed after a load or reconcile recapture).
                    AdoptExecutionRealization(plan, meta, burn, predictorDvVlf);
                    continue;
                }
                if (BurnPlanWriter.TryEditDv(vehicle, burn, dvVlf.X, dvVlf.Y, dvVlf.Z,
                        predictorDvVlf)
                    != "applied")
                {
                    settled = false;
                    continue;
                }
                string previous = meta.BasisParentId;
                meta.BasisParentId = resolved;
                meta.ExecutionDvVlf = FrameAdapter.ToCore(dvVlf);
                InvalidateAnalysis();
                ModLog.Info(flipped
                    ? $"planner: burn basis parent changed '{previous}' -> '{resolved}' "
                      + $"for '{vesselId}' at t={t:F1} s; dv re-realized in {meta.Frame.Label}"
                    : $"planner: burn at t={t:F1} s for '{vesselId}' re-realized against "
                      + $"the drifted stock patch basis ({meta.Frame.Label})");
            }
            if (settled && urgency is { } observed)
                BasisReconversionUrgency.Clear(vesselId, observed);
        }
    }

    private static void ClearOrphanedUrgency(
        List<KeyValuePair<string, FlightPlanModel>> upkeep)
    {
        if (!BasisReconversionUrgency.Any) return;
        foreach (string vesselId in BasisReconversionUrgency.Snapshot())
        {
            bool known = false;
            foreach (var (id, _) in upkeep)
                if (string.Equals(id, vesselId, StringComparison.Ordinal))
                {
                    known = true;
                    break;
                }
            if (!known) BasisReconversionUrgency.Clear(vesselId);
        }
    }

    /// <summary>Stamps a within-tolerance burn as execution-realized
    /// (<see cref="FlightPlanBurnMeta.ExecutionDvVlf"/>) and refreshes the
    /// snapshot's display vector. Both are session-only, so this also repairs them
    /// after a load or reconcile recapture. Runs every upkeep pass: analysis is
    /// invalidated only when the stamp materially moves.</summary>
    private static void AdoptExecutionRealization(FlightPlanModel plan,
        FlightPlanBurnMeta meta, Burn burn, Vector3d? predictorDvVlf)
    {
        Vector3d stored = FrameAdapter.ToCore(burn.DeltaVVlf);
        if (meta.ExecutionDvVlf is not { } previous
            || BurnFrameKernel.IsStale(previous, stored,
                PlannedBurnConverter.StaleToleranceMps))
        {
            meta.ExecutionDvVlf = stored;
            InvalidateAnalysis();
        }
        if (predictorDvVlf is { } display)
            plan.SnapshotSetDisplayDv(burn.Time.Seconds(), display,
                PlannedBurnConverter.StaleToleranceMps);
    }

    private static void ClosePicker() => _pickerVesselId = null;

    /// <summary>THE picker-targeting predicate (one copy, or the keying convention
    /// drifts between sites): the open picker aims at this vessel's burn at this time
    /// — vessel by ordinal id, time within the plan-meta tolerance.</summary>
    private static bool PickerTargets(string vesselId, double timeSeconds) =>
        string.Equals(_pickerVesselId, vesselId, StringComparison.Ordinal)
        && BurnIdentityPolicy.SameBurn(_pickerBurnTime, timeSeconds);

    /// <summary>THE meta-move pairing: the meta AND the picker are both keyed by burn
    /// time, so every time move must re-key both together — one helper, so no future
    /// move site can remember one and forget the other.</summary>
    private static void MoveMetaAndPicker(Vehicle vehicle, FlightPlanModel plan,
        double oldTime, double newTime)
    {
        plan.MoveMeta(oldTime, newTime, Environment.TickCount64);
        if (PickerTargets(vehicle.Id, oldTime)) _pickerBurnTime = newTime;
    }

    public static void Draw()
    {
        if (_errors >= 3) return; // panel misbehaving: stop drawing, keep the game alive
        try
        {
            if (!ModServices.Enabled || !ModServices.EnsureBound(out var services)) return;
            var vessels = services.Vessels;
            var controlled = KSA.Program.ControlledVehicle;
            AdvancePendingCleanup(vessels);
            // Automatic node construction belongs to its vessel, not whichever
            // vessel happens to be controlled now. Keep the transaction advancing
            // (or rolling back) even after a vessel switch.
            if (_rendezvous is { } pending
                && vessels.TryGetLiveVehicle(pending.VesselId) is { } owner)
                AdvanceRendezvous(vessels, owner);
            AdvanceOptimizer(vessels);
            AdvanceBasisReconversion(vessels);
            // Keep transactions progressing while hidden; completed optimizer state
            // stays parked in its owner context until the planner is reopened.
            if (!_open)
            {
                ClosePicker();
                return;
            }
            if (controlled is null) return;

            UiTheme.PrepareWindow(600f, 720f, 500f, 420f);
            bool visible = ImGui.Begin("Whisker Dynamics: Planner"u8, ref _open);
            try
            {
                if (!_open) ClosePicker();
                if (!visible) return;
                UiTheme.MutedText("Build and refine burns on the n-body trajectory.");
                var vehicle = vessels.TryGetLiveVehicle(controlled.Id);
                if (vehicle is null)
                {
                    ImGui.TextWrapped(
                        $"Vessel '{controlled.Id}' is not tracked; the planner is unavailable.");
                    return;
                }

                double now = Universe.GetElapsedSimTime().Seconds();
                var burns = BurnsForFrame(vehicle);
                var plan = FlightPlans.TryGet(vehicle.Id);

                if (plan is null)
                    DrawCreatePlan(vessels, vehicle, burns, now);
                else
                    DrawPlan(vessels, vehicle, plan, burns, now);

                if (_status.Length > 0)
                    ImGui.TextWrapped($"Last change: {_status}");
            }
            finally
            {
                ImGui.End();
            }
            if (!_firstDrawLogged)
            {
                _firstDrawLogged = true;
                ModLog.Info("planner panel drawn (first frame)");
            }
        }
        catch (Exception e)
        {
            _errors++;
            ModLog.Error($"planner panel: {e}");
        }
    }

    private static void DrawCreatePlan(VesselRegistry vessels, Vehicle vehicle,
        IReadOnlyList<Burn> burns, double now)
    {
        ImGui.SeparatorText("Plan setup"u8);
        ImGui.TextWrapped($"'{vehicle.Id}': no flight plan");
        var times = new List<double>(burns.Count);
        foreach (var burn in burns) times.Add(burn.Time.Seconds());
        if (_rendezvous is null)
        {
            if (UiLayout.BeginProperties("##new-plan-properties"u8,
                    UiTheme.PropertyLabelWidth))
            {
                try
                {
                    UiLayout.NextProperty("Plan duration");
                    if (DurationRow("##newplanlen"u8, "newplanlen", 0.0,
                            ref _newPlanLengthSeconds, PlanLengthSteps)
                        && ValidateNewPlanLength(now, times) is { } editReason)
                        _status = editReason;
                }
                finally
                {
                    ImGui.EndTable();
                }
            }
        }
        else
        {
            ImGui.TextWrapped($"Plan duration: {TimeDisplayKernel.FormatDuration(
                _rendezvous.Job.HorizonSeconds - _rendezvous.Job.NowSeconds)} (locked for solve)");
        }

        ImGui.Spacing();
        ImGui.SeparatorText("Automatic plan"u8);
        DrawRendezvous(vessels, vehicle, burns, now);
        if (_rendezvous is not null) return;

        ImGui.Spacing();
        ImGui.SeparatorText("Actions"u8);
        if (burns.Count > 0)
            ImGui.TextWrapped($"{burns.Count} existing stock burn(s) will be adopted (as VLF burns)");
        if (ImGui.Button(
                $"Create plan ({TimeDisplayKernel.FormatDuration(_newPlanLengthSeconds)})",
                (float2?)null))
        {
            if (ValidateNewPlanLength(now, times) is { } reason) _status = reason;
            else
            {
                FlightPlans.Create(vehicle.Id, now, _newPlanLengthSeconds);
                _status = "plan created";
                ModLog.Info($"planner: flight plan created for '{vehicle.Id}' at t={now:F1} s "
                    + $"(length {_newPlanLengthSeconds / 86400.0:F2} d, "
                    + $"{burns.Count} burns adopted)");
            }
        }
    }

    private static string? ValidateNewPlanLength(double now, IReadOnlyList<double> burnTimes)
    {
        if (!double.IsFinite(_newPlanLengthSeconds) || _newPlanLengthSeconds <= 0)
            return "rejected: plan duration must be positive";
        if (_newPlanLengthSeconds > SettingsKernel.MaxRailsDays * 86400.0)
            return $"rejected: plan duration is capped at {SettingsKernel.MaxRailsDays:F0} d";
        foreach (double time in burnTimes)
            if (time > now + _newPlanLengthSeconds - FlightPlanModel.MinimumPostBurnSeconds)
                return $"rejected: plan duration must leave {FlightPlanModel.MinimumPostBurnSeconds:F0} s after every existing stock burn";
        return null;
    }

    internal static string PlanBurnForGameTest(VesselRegistry vessels,
        Vehicle vehicle, double burnTime, FrameSpec? frame, Vector3d components)
    {
        double now = Universe.GetElapsedSimTime().Seconds();
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vehicle);
        FlightPlanModel? plan = FlightPlans.TryGet(vehicle.Id);
        bool created = false;
        if (plan is null)
        {
            plan = CreatePlanAdoptingBurns(vehicle, burns, now, burnTime);
            created = true;
        }
        else if (plan.Diverged)
        {
            return "rejected: plan diverged; rebase it before adding a burn";
        }

        string verdict = QueueBurn(vessels, vehicle,
            vessels.TryGetTracked(vehicle.Id), vehicle.Orbit, plan, burns, now,
            burnTime, frame, components, allowVlfFallback: false);
        if (!verdict.StartsWith("queued", StringComparison.Ordinal) && created)
            FlightPlans.Remove(vehicle.Id);
        _status = verdict;
        InvalidateAnalysis();
        return verdict;
    }

    internal static string CreatePlanForGameTest(Vehicle vehicle)
    {
        if (FlightPlans.TryGet(vehicle.Id) is not null)
            return "rejected: flight plan already exists";
        CreatePlanAdoptingBurns(vehicle, BurnPlanWriter.Snapshot(vehicle),
            Universe.GetElapsedSimTime().Seconds());
        _status = "plan created";
        InvalidateAnalysis();
        return _status;
    }

    private static FlightPlanModel CreatePlanAdoptingBurns(Vehicle vehicle,
        IReadOnlyList<Burn> burns, double now, double? plannedBurnTime = null)
    {
        var times = new List<double>(burns.Count + 1);
        foreach (Burn burn in burns) times.Add(burn.Time.Seconds());
        if (plannedBurnTime is { } burnTime) times.Add(burnTime);
        FlightPlanModel plan = FlightPlans.Create(vehicle.Id, now,
            FlightPlans.InitialLengthSeconds(now, times));
        ModLog.Info($"planner: flight plan created for '{vehicle.Id}' at "
            + $"t={now:F1} s by compiled player workflow");
        return plan;
    }

    internal static string DeletePlanAndBurns(Vehicle vehicle)
    {
        if (FlightPlans.TryGet(vehicle.Id) is null)
            return "rejected: no flight plan";
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vehicle);
        foreach (Burn burn in burns)
        {
            // Surface a failed removal; the plan stays so the action can be retried.
            string verdict = BurnPlanWriter.TryRemove(vehicle, burn);
            if (verdict != "queued")
            {
                InvalidateAnalysis();
                _status = "rejected: could not remove burn at "
                    + $"t={burn.Time.Seconds():F1} s: {verdict}";
                return _status;
            }
        }
        FlightPlans.Remove(vehicle.Id);
        InvalidateAnalysis();
        _status = $"plan deleted ({burns.Count} burn(s) removed)";
        ModLog.Info($"planner: flight plan deleted for '{vehicle.Id}' "
            + $"({burns.Count} burns removed)");
        return _status;
    }

    internal static string AddPlaceholderBurnForGameTest(
        VesselRegistry vessels, Vehicle vehicle, FrameSpec frame)
    {
        double now = Universe.GetElapsedSimTime().Seconds();
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vehicle);
        FlightPlanModel? plan = FlightPlans.TryGet(vehicle.Id);
        if (plan is null)
            return "rejected: no flight plan";
        if (plan.Diverged)
            return "rejected: plan diverged; rebase it before adding a burn";
        string verdict = QueueBurn(vessels, vehicle,
            vessels.TryGetTracked(vehicle.Id), vehicle.Orbit, plan, burns, now,
            now + AddLeadSeconds, frame, Vector3d.Zero,
            allowVlfFallback: false);
        _status = verdict;
        InvalidateAnalysis();
        return verdict;
    }

    internal static string MoveBurnForGameTest(
        VesselRegistry vessels, Vehicle vehicle, Burn burn, double newTime)
    {
        FlightPlanModel? plan = FlightPlans.TryGet(vehicle.Id);
        if (plan is null)
            return "rejected: no flight plan";
        TrackedVessel? tracked = vessels.TryGetTracked(vehicle.Id);
        if (tracked is null)
            return "rejected: vessel is not tracked";
        double now = Universe.GetElapsedSimTime().Seconds();
        // Same admission the panel's time editor applies before TryEditTime.
        if (plan.RejectOutsideWindow(newTime, now,
                AvailableRailsDays(tracked, now)) is { } outside)
            return outside;
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vehicle);
        double oldTime = burn.Time.Seconds();
        FlightPlanBurnMeta? meta = plan.TryGetMetaAt(oldTime);
        string verdict = BurnPlanWriter.TryEditTime(vehicle, burn, newTime);
        if (verdict == "applied")
        {
            MoveMetaAndPicker(vehicle, plan, oldTime, newTime);
            if (meta is not null)
            {
                ReconvertAfterTimeEdit(vessels, vehicle, tracked, vehicle.Orbit,
                    burns, burn, meta, newTime, now);
                verdict = _status;
            }
            else
            {
                // A VLF-authored burn has nothing to reconvert; report the move,
                // not stale _status.
                verdict = "time applied";
                _status = verdict;
            }
        }
        InvalidateAnalysis();
        return verdict;
    }

    internal static string EditBurnComponentsForGameTest(
        VesselRegistry vessels, Vehicle vehicle, Burn burn, Vector3d components)
    {
        FlightPlanModel? plan = FlightPlans.TryGet(vehicle.Id);
        if (plan is null)
            return "rejected: no flight plan";
        // Without the meta, EditComponents would write these as raw stock VLF and
        // still report "applied" — a silently wrong-frame burn.
        FlightPlanBurnMeta? meta = plan.TryGetMetaAt(burn.Time.Seconds());
        if (meta is null)
            return "rejected: burn has no authoring-frame meta at "
                + $"t={burn.Time.Seconds():F1} s";
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vehicle);
        EditComponents(vessels, vehicle, vessels.TryGetTracked(vehicle.Id),
            vehicle.Orbit, plan, burns, burn, meta, components,
            Universe.GetElapsedSimTime().Seconds());
        InvalidateAnalysis();
        return _status;
    }

    internal static string RebasePlanForGameTest(
        VesselRegistry vessels, Vehicle vehicle)
    {
        if (FlightPlans.TryGet(vehicle.Id) is not { } plan)
            return "rejected: no flight plan";
        if (vessels.TryGetTracked(vehicle.Id) is not { } tracked)
            return "rejected: vessel is not tracked";
        if (!PlannerKernel.LiveDeltaVHasSettled(
                Environment.TickCount64, tracked.LastDvWitnessMs))
            return "rejected: rebase is unavailable until thrust stops";

        Rebase(vessels, vehicle, tracked, vehicle.Orbit, plan,
            BurnPlanWriter.Snapshot(vehicle),
            Universe.GetElapsedSimTime().Seconds());
        return _status;
    }

    /// <summary>Save bridge for the few frames between the two queued stock writes.
    /// The plan itself does not exist yet; this ownership marker lets load remove
    /// whichever automatic nodes the stock save captured.</summary>
    internal static SidecarPendingRendezvous? PendingForSidecar()
    {
        if (_rendezvous is not { Stage: > 0, Job.Result: { } result } context) return null;
        static SidecarSnapshotBurn Dto(Burn? accepted, double time, Vector3d dv)
        {
            if (accepted is not null)
            {
                time = accepted.Time.Seconds();
                dv = FrameAdapter.ToCore(accepted.DeltaVVlf);
            }
            return new SidecarSnapshotBurn
            {
                TimeSeconds = time, X = dv.X, Y = dv.Y, Z = dv.Z,
            };
        }
        var pending = new SidecarPendingRendezvous { VesselId = context.VesselId };
        pending.Burns.Add(Dto(context.FirstBurn, result.DepartureTime, result.DepartureDvVlf));
        if (context.Stage >= 2)
            pending.Burns.Add(Dto(context.SecondBurn, result.ArrivalTime, result.ArrivalDvVlf));
        return pending;
    }

    internal static void RestorePendingCleanup(SidecarPendingRendezvous? pending)
    {
        if (pending is null || string.IsNullOrEmpty(pending.VesselId)
            || pending.Burns is not { Count: >= 1 and <= 2 }
            || pending.Burns.Any(b => b is null || !double.IsFinite(b.TimeSeconds)
                || !double.IsFinite(b.X) || !double.IsFinite(b.Y) || !double.IsFinite(b.Z)))
            return;
        _pendingCleanup = pending;
        _pendingCleanupQueued = false;
        _status = "cleaning an interrupted rendezvous transaction from the loaded save...";
    }

    private static void AdvancePendingCleanup(VesselRegistry vessels)
    {
        if (_pendingCleanup is not { } pending
            || vessels.TryGetLiveVehicle(pending.VesselId) is not { } vehicle) return;
        var burns = BurnPlanWriter.Snapshot(vehicle);
        var matches = new List<Burn>();
        foreach (var dto in pending.Burns)
            foreach (var burn in burns)
                if (BurnIdentityPolicy.SameBurn(burn.Time.Seconds(), dto.TimeSeconds)
                    && (FrameAdapter.ToCore(burn.DeltaVVlf)
                        - new Vector3d(dto.X, dto.Y, dto.Z)).Length() <= 1e-6)
                { matches.Add(burn); break; }
        if (_pendingCleanupQueued)
        {
            if (matches.Count > 0) return;
            _pendingCleanup = null;
            _pendingCleanupQueued = false;
            _status = "interrupted rendezvous nodes removed after load";
            return;
        }
        foreach (var burn in matches.Distinct()) BurnPlanWriter.TryRemove(vehicle, burn);
        if (matches.Count == 0)
        {
            _pendingCleanup = null;
            _status = "loaded save contained no interrupted rendezvous nodes";
        }
        else
        {
            _pendingCleanupQueued = true;
            ModLog.Info($"planner: queued cleanup of {matches.Count} interrupted rendezvous node(s) "
                + $"for '{pending.VesselId}' after load");
        }
    }

    /// <summary>The first automatic-plan affordance. It is available only before a
    /// plan exists and requires an empty stock plan: silently incorporating existing
    /// user nodes would not produce the solved two-impulse trajectory.</summary>
    private static void DrawRendezvous(VesselRegistry vessels, Vehicle vehicle,
        IReadOnlyList<Burn> burns, double now)
    {
        if (_pendingCleanup is not null)
        {
            ImGui.TextWrapped("finishing interrupted-rendezvous cleanup before new planning..."u8);
            return;
        }
        if (_rendezvous is { } active)
        {
            if (!string.Equals(active.VesselId, vehicle.Id, StringComparison.Ordinal))
            {
                ImGui.TextWrapped($"rendezvous solve running for '{active.VesselId}'...");
                if (ImGui.Button("Cancel rendezvous"u8, (float2?)null)) active.Job.Cancel();
                return;
            }
            ImGui.TextWrapped(active.Job.Done ? "preparing maneuver nodes..." : active.Job.StatusLine);
            if (!active.Job.Done || active.Stage > 0)
            {
                if (ImGui.Button("Cancel rendezvous"u8, (float2?)null)) active.Job.Cancel();
            }
            return;
        }
        if (ImGui.Button("Rendezvous"u8, (float2?)null))
            StartRendezvous(vessels, vehicle, burns, now);
        ImGui.SetItemTooltip("searches the selected plan duration across sampled zero- and multi-revolution short/long transfers, then creates the lowest-dv corrected two-burn rendezvous"u8);
    }

    private static void StartRendezvous(VesselRegistry vessels, Vehicle vehicle,
        IReadOnlyList<Burn> burns, double now)
    {
        if (ValidateNewPlanLength(now, []) is { } durationReason)
        {
            _status = durationReason;
            return;
        }
        if (burns.Count != 0)
        {
            _status = "rendezvous rejected: the stock maneuver plan already has burns";
            return;
        }
        string? targetId = vehicle.Target?.Id;
        if (string.IsNullOrEmpty(targetId) || string.Equals(targetId, vehicle.Id, StringComparison.Ordinal)
            || vessels.TryGetLiveVehicle(targetId) is not { } targetVehicle)
        {
            _status = "rendezvous rejected: select another tracked vessel as the target";
            return;
        }
        if (!vessels.TryCaptureRailsAuthority(
                vehicle, out var chaserAuthority, out _)
            || !vessels.TryCaptureRailsAuthority(
                targetVehicle, out var targetAuthority, out _))
        {
            _status = "rendezvous rejected: both vessels must have active n-body rails";
            return;
        }
        var chaser = chaserAuthority.Tracked;
        var target = targetAuthority.Tracked;
        if (BurnPlanWriter.Snapshot(targetVehicle).Count != 0)
        {
            _status = "rendezvous rejected: the target vessel has planned burns";
            return;
        }
        var rendezvousEngine = TrajectoryOverlay.ReadEngineScalars(vehicle);
        if (!rendezvousEngine.Usable)
        {
            _status = "rendezvous rejected: no usable engine/mass configuration is available";
            return;
        }
        if (vehicle.Orbit.Parent is not Astronomical chaserParent
            || targetVehicle.Orbit.Parent is not Astronomical targetParent
            || !string.Equals(chaserParent.Id, targetParent.Id, StringComparison.Ordinal))
        {
            _status = "rendezvous rejected: both vessels must orbit the same reference body";
            return;
        }
        double horizon = now + _newPlanLengthSeconds;
        if (chaser.Rails.Horizon + 1e-6 < horizon)
        {
            _status = "rendezvous rejected: the full selected plan duration is not yet "
                + "available on the rails - extend the orbits window or retry after it grows";
            return;
        }
        double parentRadius = chaser.Rails.MeanRadiusOf(chaserParent.Id);
        if (!(parentRadius > 0))
        {
            _status = $"rendezvous rejected: no physical radius is known for '{chaserParent.Id}'";
            return;
        }
        if (horizon - now < 10 * 60.0)
        {
            _status = "rendezvous rejected: prediction window is still growing - retry shortly";
            return;
        }
        var prediction = chaser.Rails.TryCaptureSolverPredictionContext(now, horizon);
        if (prediction is null)
        {
            _status = "rendezvous: detached prediction window is preparing - retry shortly";
            return;
        }
        if (!chaser.TryCaptureRendezvousSolverSeeds(
                target, now,
                out var chaserLineage, out var chaserSeed,
                out var targetLineage, out var targetSeed))
        {
            _status = "rendezvous rejected: a vessel coast changed or has not reached "
                + "the current time - retry shortly";
            return;
        }
        if (!ReferenceEquals(chaserLineage, chaserAuthority.Lineage)
            || !ReferenceEquals(targetLineage, targetAuthority.Lineage)
            || !vessels.ValidateRailsAuthority(chaserAuthority, out _)
            || !vessels.ValidateRailsAuthority(targetAuthority, out _))
        {
            _status = "rendezvous rejected: a vessel coast changed while seeds were captured";
            return;
        }
        var job = new RendezvousSolveJob
        {
            Chaser = chaser,
            Target = target,
            ChaserLineage = chaserLineage,
            TargetLineage = targetLineage,
            Prediction = prediction,
            ChaserSeed = chaserSeed,
            TargetSeed = targetSeed,
            ParentId = chaserParent.Id,
            // Automatic rendezvous always models real centered thrust arcs. A user
            // setting of zero disables finite DISPLAY folding, not the FC's physics;
            // retain a conservative 20 s solve discretization in that case.
            Finite = new FiniteBurnFold(rendezvousEngine,
                ModServices.Config.FiniteBurnSliceSeconds > 0
                    ? ModServices.Config.FiniteBurnSliceSeconds : 20.0,
                ModServices.Config.FiniteBurnMaxSlices),
            NowSeconds = now,
            HorizonSeconds = horizon,
        };
        _rendezvous = new RendezvousContext
        {
            Job = job,
            VesselId = vehicle.Id,
            TargetId = targetId,
            ChaserAuthority = chaserAuthority,
            TargetAuthority = targetAuthority,
        };
        job.Start();
        _status = $"solving rendezvous with '{targetId}' in the background...";
        ModLog.Info($"planner: rendezvous solve started for '{vehicle.Id}' -> '{targetId}' "
            + $"under '{chaserParent.Id}', window [{now:F0}, {horizon:F0}] s");
    }

    /// <summary>Main-thread transaction: validate the result, create the plan, queue
    /// burn one, then wait for stock to build its post-burn timeline before creating
    /// burn two. A failed second step rolls the partial plan back.</summary>
    private static void AdvanceRendezvous(VesselRegistry vessels, Vehicle vehicle)
    {
        if (_rendezvous is not { } context
            || !string.Equals(context.VesselId, vehicle.Id, StringComparison.Ordinal)) return;
        var job = context.Job;
        if (!job.Done) return;
        if (job.Cancelled)
        {
            if (context.Stage == 0)
            {
                _status = "rendezvous cancelled - nothing applied";
                _rendezvous = null;
            }
            else RollBackRendezvous(vehicle, BurnPlanWriter.Snapshot(vehicle), context,
                "cancelled");
            return;
        }
        if (job.Failure is { } failure)
        {
            _status = failure;
            _rendezvous = null;
            return;
        }
        if (job.Result is not { } result)
        {
            _status = "rendezvous: no solution";
            _rendezvous = null;
            return;
        }
        if (!vessels.ValidateRailsAuthority(context.ChaserAuthority, out _)
            || !vessels.ValidateRailsAuthority(context.TargetAuthority, out _))
        {
            AbortRendezvous(vehicle, context,
                "a vessel trajectory changed while solving");
            return;
        }
        if (vehicle.Target?.Id != context.TargetId
            || vessels.TryGetLiveVehicle(context.TargetId) is not { } targetVehicle
            || vehicle.Orbit.Parent is not Astronomical chaserParent
            || targetVehicle.Orbit.Parent is not Astronomical targetParent
            || chaserParent.Id != job.ParentId || targetParent.Id != job.ParentId)
        {
            AbortRendezvous(vehicle, context,
                "target or shared reference body changed while solving");
            return;
        }
        if (BurnPlanWriter.Snapshot(targetVehicle).Count != 0)
        {
            if (context.Stage == 0)
            {
                _status = "rendezvous: the target acquired planned burns while solving - not applied";
                _rendezvous = null;
            }
            else RollBackRendezvous(vehicle, BurnPlanWriter.Snapshot(vehicle), context,
                "the target acquired planned burns while nodes were being built");
            return;
        }
        if (job.Finite is { } finite
            && TrajectoryOverlay.ReadEngineScalars(vehicle) != finite.Engine)
        {
            if (context.Stage == 0)
            {
                _status = "rendezvous: engine or mass configuration changed while solving - not applied";
                _rendezvous = null;
            }
            else RollBackRendezvous(vehicle, BurnPlanWriter.Snapshot(vehicle), context,
                "engine or mass configuration changed while nodes were being built");
            return;
        }
        double now = Universe.GetElapsedSimTime().Seconds();
        var departureLead = RendezvousApplyPolicy.CheckDepartureLead(
            result.DepartureTime, result.DepartureDvEcl.Length(), job.Finite,
            now, PlannerKernel.MinLeadSeconds);
        if (departureLead == RendezvousApplyLeadVerdict.PhysicallyUnmodelable)
        {
            AbortRendezvous(vehicle, context,
                "departure burn became physically unmodelable while applying");
            return;
        }
        if (departureLead == RendezvousApplyLeadVerdict.InsufficientLead)
        {
            AbortRendezvous(vehicle, context,
                "departure ignition passed while solving (time warp)");
            return;
        }
        var burns = BurnPlanWriter.Snapshot(vehicle);
        if (context.Stage == 0)
        {
            if (FlightPlans.TryGet(vehicle.Id) is not null || burns.Count != 0)
            {
                _status = "rendezvous: the maneuver plan changed while solving - not applied";
                _rendezvous = null;
                return;
            }
            double length = job.HorizonSeconds - job.NowSeconds;
            if (result.ArrivalTime > job.HorizonSeconds)
            {
                _status = "rendezvous: solution lies beyond the selected plan duration - not applied";
                _rendezvous = null;
                return;
            }
            // Keep the mod-level plan absent until BOTH stock nodes are accepted.
            // A save during the two-frame stock transaction can at worst contain a
            // plain stock node, never a sidecar claiming a completed rendezvous plan.
            context.PlanLengthSeconds = length;
            if (PlannedBurnConverter.BurnParentId(vehicle, job.ParentId,
                    result.DepartureTime) != job.ParentId)
            {
                _status = "rendezvous: departure node crosses the shared reference frame - not applied";
                _rendezvous = null;
                return;
            }
            string first = BurnPlanWriter.TryAdd(vehicle, result.DepartureTime,
                FrameAdapter.ToGame(result.DepartureDvVlf));
            if (first != "queued")
            {
                _status = $"rendezvous: first burn was not created ({first})";
                _rendezvous = null;
                return;
            }
            context.Stage = 1;
            context.StageStartedMs = Environment.TickCount64;
            _status = "rendezvous: intercept burn queued; building arrival node...";
            InvalidateAnalysis();
            return;
        }
        if (FlightPlans.TryGet(vehicle.Id) is not null)
        {
            RollBackRendezvous(vehicle, burns, context,
                "a flight plan was created while automatic nodes were being built");
            return;
        }
        if (context.FirstBurn is null)
            context.FirstBurn = FindExpectedBurn(burns, result.DepartureTime, result.DepartureDvVlf);
        if (context.FirstBurn is null)
        {
            if (Environment.TickCount64 - context.StageStartedMs < 5000) return;
            RollBackRendezvous(vehicle, burns, context, "stock did not accept the intercept burn");
            return;
        }
        if (context.Stage == 1)
        {
            if (burns.Count != 1 || !ReferenceEquals(burns[0], context.FirstBurn)
                || !ExpectedBurn(context.FirstBurn, result.DepartureTime, result.DepartureDvVlf))
            {
                RollBackRendezvous(vehicle, burns, context,
                    "the maneuver plan changed while nodes were being built");
                return;
            }
            // Presence of burn one does not mean stock's chained timeline worker has
            // finished. Never let BurnPlanWriter fall back to the unburned base patch
            // for burn two: wait until the post-burn timeline covers arrival.
            var arrival = new SimTime(result.ArrivalTime);
            var arrivalPatch = vehicle.FlightComputer.BurnPlan.TryGetValidTimeLinePatch(arrival);
            if (arrivalPatch is null)
            {
                if (Environment.TickCount64 - context.StageStartedMs < 5000) return;
                RollBackRendezvous(vehicle, burns, context,
                    "stock did not build the post-intercept timeline in time");
                return;
            }
            if (arrivalPatch.Orbit.Parent is not Astronomical arrivalParent
                || arrivalParent.Id != job.ParentId)
            {
                RollBackRendezvous(vehicle, burns, context,
                    "the arrival node leaves the shared reference frame");
                return;
            }
            string second = BurnPlanWriter.TryAdd(vehicle, result.ArrivalTime,
                FrameAdapter.ToGame(result.ArrivalDvVlf));
            if (second != "queued")
            {
                RollBackRendezvous(vehicle, burns, context,
                    $"arrival burn was not created ({second})");
                return;
            }
            context.Stage = 2;
            context.StageStartedMs = Environment.TickCount64;
            _status = "rendezvous: arrival burn queued; waiting for stock acceptance...";
            return;
        }
        context.SecondBurn ??= FindExpectedBurn(burns, result.ArrivalTime, result.ArrivalDvVlf);
        if (context.SecondBurn is null)
        {
            if (Environment.TickCount64 - context.StageStartedMs < 5000) return;
            RollBackRendezvous(vehicle, burns, context, "stock did not accept the arrival burn");
            return;
        }
        if (burns.Count != 2 || !burns.Contains(context.FirstBurn)
            || !burns.Contains(context.SecondBurn)
            || !ExpectedBurn(context.FirstBurn, result.DepartureTime, result.DepartureDvVlf)
            || !ExpectedBurn(context.SecondBurn, result.ArrivalTime, result.ArrivalDvVlf))
        {
            RollBackRendezvous(vehicle, burns, context,
                "the maneuver plan changed before both nodes were accepted");
            return;
        }
        context.CreatedPlan = FlightPlans.Create(vehicle.Id, now, context.PlanLengthSeconds);
        var plan = context.CreatedPlan;
        _status = $"rendezvous plan created: depart in "
            + $"{TimeDisplayKernel.FormatDuration(Math.Round(result.DepartureTime - now))}, "
            + $"arrive plan+{TimeDisplayKernel.FormatDuration(result.ArrivalTime - plan.CreatedAtSeconds)}, "
            + $"{result.Revolutions} rev, total |dv| {result.TotalDv:F2} m/s, "
            + $"predicted miss {result.MissDistance:F1} m";
        ModLog.Info($"planner: rendezvous plan created for '{vehicle.Id}' -> '{context.TargetId}': "
            + $"burns t={result.DepartureTime:F1}/{result.ArrivalTime:F1} s, total dv "
            + $"{result.TotalDv:F3} m/s, {result.Revolutions} rev, corrected miss "
            + $"{result.MissDistance:F3} m, solve {job.ElapsedMs} ms");
        _rendezvous = null;
        InvalidateAnalysis();
    }

    private static Burn? FindExpectedBurn(IReadOnlyList<Burn> burns, double time, Vector3d dvVlf)
    {
        foreach (var burn in burns)
            if (ExpectedBurn(burn, time, dvVlf)) return burn;
        return null;
    }

    private static bool ExpectedBurn(Burn burn, double time, Vector3d dvVlf) =>
        BurnIdentityPolicy.SameBurn(burn.Time.Seconds(), time)
        && (FrameAdapter.ToCore(burn.DeltaVVlf) - dvVlf).Length() <= 1e-6;

    private static void AbortRendezvous(Vehicle vehicle, RendezvousContext context, string reason)
    {
        if (context.Stage > 0)
            RollBackRendezvous(vehicle, BurnPlanWriter.Snapshot(vehicle), context, reason);
        else
        {
            _status = $"rendezvous: {reason} - not applied";
            _rendezvous = null;
        }
    }

    private static void RollBackRendezvous(Vehicle vehicle, IReadOnlyList<Burn> burns,
        RendezvousContext context, string reason)
    {
        if (context.Job.Result is { } result)
        {
            // Once stock has yielded an identity, that exact object remains ours even
            // if it was edited during the transaction. Before acceptance, fall back
            // to the expected immutable time/dv fingerprint.
            var first = context.FirstBurn is { } acceptedFirst && burns.Contains(acceptedFirst)
                ? acceptedFirst
                : context.FirstBurn is null
                    ? FindExpectedBurn(burns, result.DepartureTime, result.DepartureDvVlf) : null;
            var second = context.SecondBurn is { } acceptedSecond && burns.Contains(acceptedSecond)
                ? acceptedSecond
                : context.SecondBurn is null
                    ? FindExpectedBurn(burns, result.ArrivalTime, result.ArrivalDvVlf) : null;
            if (first is not null) BurnPlanWriter.TryRemove(vehicle, first);
            if (second is not null && !ReferenceEquals(second, first))
                BurnPlanWriter.TryRemove(vehicle, second);
        }
        if (ReferenceEquals(FlightPlans.TryGet(vehicle.Id), context.CreatedPlan))
            FlightPlans.Remove(vehicle.Id);
        _status = $"rendezvous: {reason} - partial plan rolled back";
        _rendezvous = null;
        InvalidateAnalysis();
    }

    private static void DrawPlan(VesselRegistry vessels, Vehicle vehicle, FlightPlanModel plan,
        IReadOnlyList<Burn> burns, double now)
    {
        // Metadata upkeep: metas whose stock burn disappeared (deleted or dragged via
        // the STOCK editor, where intent is invisible to us) degrade to VLF burns.
        var stockTimes = new List<double>(burns.Count);
        double totalDv = 0;
        foreach (var b in burns)
        {
            stockTimes.Add(b.Time.Seconds());
            totalDv += b.DeltaVVlf.Length();
        }
        plan.PruneOrphanedMeta(stockTimes, Environment.TickCount64);

        var tracked = vessels.TryGetTracked(vehicle.Id);
        var orbit = vehicle.Orbit;
        var analysis = RefreshAnalysis(
            vessels, vehicle, tracked, orbit, burns, plan, now);

        ImGui.SeparatorText("Plan setup & summary"u8);
        ImGui.TextWrapped($"'{vehicle.Id}' flight plan: {burns.Count} burn(s)");
        double vesselDv = vehicle.NavBallData.DeltaV;
        bool mainPropulsion = plan.PropulsionSource == PropulsionSource.MainEngines;
        bool overBudget = mainPropulsion && PlannerKernel.IsDeltaVOverBudget(totalDv, vesselDv);
        if (overBudget)
            ImGui.TextWrapped("Plan exceeds the available main-engine delta-v.");
        ImGui.TextWrapped(mainPropulsion
            ? $"plan total: {totalDv:F1} m/s / main-engine vessel: {vesselDv:F1} m/s"
            : $"plan total: {totalDv:F1} m/s (main-engine vessel budget not applicable to RCS)");
        ImGui.TextWrapped($"began t={plan.CreatedAtSeconds:F0} s "
            + $"(now = plan+{TimeDisplayKernel.FormatDuration(Math.Round(plan.PlanRelative(now)))}); "
            + $"ends plan+{TimeDisplayKernel.FormatDuration(plan.LengthSeconds)} "
            + $"({TimeDisplayKernel.FormatDuration(Math.Round(plan.EndSeconds - now))} from now)");
        if (plan.EndSeconds <= now)
            ImGui.TextWrapped("Plan has ended; extend its duration to keep planning.");
        double lengthSeconds = plan.LengthSeconds;
        if (UiLayout.BeginProperties("##plan-length-property"u8,
                UiTheme.PropertyLabelWidth))
        {
            try
            {
                UiLayout.NextProperty("Plan duration");
                if (DurationRow("##planlen"u8, "planlen", 0.0,
                        ref lengthSeconds, PlanLengthSteps))
                {
                    if (plan.ValidateLength(lengthSeconds, stockTimes) is { } reason)
                        _status = reason;
                    else
                    {
                        plan.LengthSeconds = lengthSeconds;
                        _status = "plan length updated";
                    }
                }
            }
            finally
            {
                ImGui.EndTable();
            }
        }
        if (plan.Diverged)
            ImGui.TextWrapped(
                "Plan diverged from the actual trajectory; rebase to re-anchor it.");

        ImGui.Spacing();
        ImGui.SeparatorText("Propulsion"u8);
        // Finite-plan propulsion is plan-level for this first engine-aware slice.
        // Main engines remain stock-compatible. Forward RCS is an honest preview for
        // manually flying +X translation while pointed along the burn; KSA's stock
        // auto-burn path commands EngineController only and locks translation out.
        var selectedEngine = TrajectoryOverlay.ReadEngineScalars(vehicle, plan.PropulsionSource);
        var snapshot = plan.Snapshot;
        EngineScalars? finiteEngine = snapshot?.PropulsionSource == plan.PropulsionSource
            ? snapshot.Engine
            : selectedEngine is { Usable: true } ? selectedEngine : null;
        string propulsionLabel = plan.PropulsionSource == PropulsionSource.RcsForward
            ? "RCS forward (manual execution)"
            : "main engines (stock auto-burn)";
        ImGui.TextWrapped($"Finite propulsion: {propulsionLabel}");
        if (finiteEngine is { Usable: true } estimate)
            ImGui.TextWrapped($"snapshot estimate: ve {estimate.ExhaustVelocity:F1} m/s, "
                + $"flow {estimate.MassFlowRate:F3} kg/s");
        else
            ImGui.TextWrapped("estimate unavailable; planned line falls back to impulses"u8);
        if (!selectedEngine.Usable && finiteEngine is { Usable: true })
            ImGui.TextWrapped("current hardware unavailable; planned line retains its frozen estimate until Rebase"u8);
        if (plan.PropulsionSource == PropulsionSource.MainEngines)
        {
            if (ImGui.SmallButton("Use forward RCS estimate"u8))
            {
                var rcs = TrajectoryOverlay.ReadEngineScalars(vehicle, PropulsionSource.RcsForward);
                if (!rcs.Usable)
                    _status = "RCS unavailable: no active, fueled forward-translation jets";
                else
                {
                    plan.SetPropulsionSource(PropulsionSource.RcsForward, rcs);
                    _status = "finite estimate uses forward RCS; execute burns manually";
                    InvalidateAnalysis();
                }
            }
            ImGui.SetItemTooltip("models active fueled +body-X RCS jets; stock maneuver auto-burn still uses main engines"u8);
        }
        else if (ImGui.SmallButton("Use main engines"u8))
        {
            plan.SetPropulsionSource(PropulsionSource.MainEngines,
                TrajectoryOverlay.ReadEngineScalars(vehicle, PropulsionSource.MainEngines));
            _status = "finite estimate uses main engines";
            InvalidateAnalysis();
        }
        if (plan.PropulsionSource == PropulsionSource.RcsForward)
            ImGui.TextWrapped("RCS preview only: point along the burn and fly forward translation manually"u8);
        ImGui.Spacing();
        ImGui.SeparatorText("Burns"u8);
        if (burns.Count == 0)
            ImGui.TextWrapped("No burns in this plan."u8);
        for (int i = 0; i < burns.Count; i++)
        {
            ImGui.PushID(i);
            try
            {
                DrawBurn(vessels, i + 1, vehicle, tracked, orbit, plan, burns, burns[i],
                    analysis is not null && i < analysis.Count ? analysis[i] : null, now);
            }
            finally
            {
                ImGui.PopID();
            }
        }

        // ---- Periapsis optimizer (targets the map display frame's relevant body).
        ImGui.Spacing();
        ImGui.SeparatorText("Optimizer"u8);
        DrawOptimizer(vessels, vehicle, orbit, plan, burns, now);

        ImGui.Spacing();
        ImGui.SeparatorText("Actions"u8);

        if (_rendezvous is { Stage: > 0 } transaction
            && string.Equals(transaction.VesselId, vehicle.Id, StringComparison.Ordinal))
        {
            ImGui.TextWrapped(_status);
            if (ImGui.Button("Cancel rendezvous"u8, (float2?)null))
            {
                transaction.Job.Cancel();
                RollBackRendezvous(vehicle, burns, transaction, "cancelled by user");
                return;
            }
        }
        bool canRebase = tracked is not null
            && PlannerKernel.LiveDeltaVHasSettled(
                Environment.TickCount64, tracked.LastDvWitnessMs);
        if (canRebase)
        {
            if (ImGui.Button("Rebase onto current trajectory"u8, (float2?)null))
                Rebase(vessels, vehicle, tracked!, orbit, plan, burns, now);
        }
        else if (plan.Diverged)
        {
            ImGui.TextWrapped("Rebase available once thrust stops"u8);
        }
        if (ImGui.Button("Delete plan and burns"u8, (float2?)null))
        {
            DeletePlanAndBurns(vehicle);
            return;
        }
        ApplyCompletedOptimizer(vessels, vehicle, orbit, plan, burns, now);
        if (ImGui.Button(AddBurnLabel, (float2?)null))
        {
            AddBurn(vessels, vehicle, tracked, orbit, plan, burns, now);
            InvalidateAnalysis();
        }
        ImGui.SetItemTooltip("adds a burn authored in the map's current display frame; edit its time, frame and components on the burn entry"u8);

        // ---- Frame-picker window for the burn that is choosing its authoring frame.
        DrawFramePicker(vessels, vehicle, tracked, orbit, plan, burns, now);
    }

    /// <summary>The rails window burn placement may honestly promise (days ahead of
    /// now): RailsService's availability clamp — while a raised orbits preset is
    /// still growing chunk by chunk, a burn past the reached horizon would demand a
    /// synchronous Gate-held extension the converter refuses anyway. No tracked
    /// entry (no rails handle) keeps the config bound alone.</summary>
    private static double AvailableRailsDays(TrackedVessel? tracked, double now) =>
        tracked is null
            ? ModServices.Config.RailsAheadDays
            : tracked.Rails.AvailableAheadDays(now);

    private static void DrawBurn(VesselRegistry vessels, int number,
        Vehicle vehicle, TrackedVessel? tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, Burn burn,
        PlannedBurnConverter.BurnAnalysis? a, double now)
    {
        ImGui.SeparatorText($"Burn {number}");
        double t = burn.Time.Seconds();
        var meta = plan.TryGetMetaAt(t);
        bool stale = a?.Stale ?? false;
        ImGui.TextWrapped($"plan+{TimeDisplayKernel.FormatDuration(plan.PlanRelative(t))} "
            + $"(in {TimeDisplayKernel.FormatDuration(Math.Round(t - now))}), "
            + $"|dv| {burn.DeltaVVlf.Length():F2} m/s{(stale ? "  [STALE]" : "")}");

        // Time on the plan clock (seconds since the plan began — the panel's one
        // clock). A frame-authored burn is auto-reconverted at its new
        // time: the user placed COMPONENTS IN A FRAME, so moving the burn keeps the
        // authored components and re-derives the stock VLF at the new time/pose/state.
        double tEdit = plan.PlanRelative(t);
        if (UiLayout.BeginProperties("##burn-time-property"u8,
                UiTheme.PropertyLabelWidth))
        {
            try
            {
                UiLayout.NextProperty("Time (plan+)");
                if (DurationRow("##time"u8, "time", t, ref tEdit, BurnTimeSteps))
                {
                    double newTime = plan.AbsoluteOf(tEdit);
                    if (plan.RejectOutsideWindow(newTime, now,
                            AvailableRailsDays(tracked, now)) is { } outside)
                    {
                        _status = outside;
                    }
                    else
                    {
                        _status = BurnPlanWriter.TryEditTime(vehicle, burn, newTime);
                        if (_status == "applied")
                        {
                            MoveMetaAndPicker(vehicle, plan, t, newTime);
                            if (meta is not null && tracked is not null)
                                ReconvertAfterTimeEdit(vessels, vehicle, tracked, orbit, burns,
                                    burn, meta, newTime, now);
                        }
                        InvalidateAnalysis();
                    }
                }
            }
            finally
            {
                ImGui.EndTable();
            }
        }

        // Authoring frame (VLF or any catalog frame): the button opens the frame-picker
        // window — the same body tree as the frames panel — targeted at THIS burn.
        // Switching RE-EXPRESSES the current physical delta-v in the new frame — no
        // stock write, only representation moves.
        if (UiLayout.BeginProperties("##burn-frame-property"u8,
                UiTheme.PropertyLabelWidth))
        {
            try
            {
                UiLayout.NextProperty("Authoring frame");
                if (UiTheme.FrameChoice(
                        meta is null ? VlfLabel : meta.Frame.Label, selected: true))
                {
                    _pickerVesselId = vehicle.Id;
                    _pickerBurnTime = t;
                }
                ImGui.SetItemTooltip("pick the reference frame this burn's components are authored in"u8);
            }
            finally
            {
                ImGui.EndTable();
            }
        }

        // Components, labeled per frame kind, edited IN the authoring frame. Frame burns
        // display the inverse conversion of the CURRENT stock VLF (the physical truth in
        // the authoring frame); the authored intent lives in the meta, and the [STALE]
        // flag + Reconvert appear when the two no longer agree (an earlier burn changed
        // the pre-burn state under this one).
        var labels = ComponentLabels(meta?.Frame);
        double c1, c2, c3;
        if (meta is null)
        {
            (c1, c2, c3) = PlannerKernel.DecomposeVlf(burn.DeltaVVlf);
        }
        else if (a?.DisplayComponents is { } shown)
        {
            (c1, c2, c3) = (shown.X, shown.Y, shown.Z);
        }
        else
        {
            (c1, c2, c3) = (meta.Authored.X, meta.Authored.Y, meta.Authored.Z); // authored intent fallback
        }
        bool edited = false;
        edited |= ComponentRow(labels.C1, "m/s##c1"u8, ref c1);
        edited |= ComponentRow(labels.C2, "m/s##c2"u8, ref c2);
        edited |= ComponentRow(labels.C3, "m/s##c3"u8, ref c3);
        if (edited)
        {
            EditComponents(vessels, vehicle, tracked, orbit, plan, burns, burn, meta,
                new Vector3d(c1, c2, c3), now);
            InvalidateAnalysis();
        }

        if (a?.Note is { } note) ImGui.TextWrapped($"note: {note}");
        if (stale && a?.FreshDvVlf is { } fresh)
        {
            ImGui.TextWrapped($"Frame components are stale; the plan changed around "
                + $"this burn and delta-v is off by "
                + $"{(fresh - FrameAdapter.ToCore(burn.DeltaVVlf)).Length():F2} m/s.");
            if (ImGui.Button("Reconvert"u8, (float2?)null))
            {
                _status = BurnPlanWriter.TryEditDv(vehicle, burn, fresh.X, fresh.Y, fresh.Z);
                InvalidateAnalysis();
            }
        }

        if (ImGui.Button("Remove burn"u8, (float2?)null))
        {
            _status = BurnPlanWriter.TryRemove(vehicle, burn);
            // Meta dies only WITH its burn: a failed TryRemove (contract drift, wrong
            // thread) leaves the stock burn in the plan, so dropping the meta would
            // silently degrade it to a plain VLF burn — same writer-success gate as
            // every sibling mutation path. The picker closes with its burn too.
            if (_status == "queued")
            {
                plan.RemoveMetaAt(t);
                if (PickerTargets(vehicle.Id, t)) ClosePicker();
            }
            InvalidateAnalysis();
        }
        ImGui.Spacing();
    }

    /// <summary>Rebase: re-anchor the plan snapshot onto the vessel's
    /// CURRENT trajectory. Frame-authored burns are reconverted first, in time order
    /// (each conversion folds the earlier burns' already-updated stock dv — the same
    /// chaining the per-burn Reconvert follows), so the captured snapshot holds the
    /// post-reconvert dv; then the anchor recaptures and the diverged flag clears
    /// (SetSnapshot). Executed burns are naturally absent — stock consumed them —
    /// so the rebased plan is reality plus what remains. OFF-RAILS (SAS/flight-
    /// computer keeps the vessel in live physics after thrust stops): the anchor
    /// comes from the COMMITTED live state — the authoritative predictor still holds
    /// the pre-burn world until the rails reseed — and frame reconversion is SKIPPED
    /// (it folds on that stale predictor); the [STALE]/Reconvert flow picks those up
    /// once back on rails.</summary>
    private static void Rebase(VesselRegistry vessels, Vehicle vehicle,
        TrackedVessel tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, double now)
    {
        bool predictorCurrent = vessels.TryCaptureRailsAuthority(
            vehicle, out _, out _);
        int reconverted = 0, failed = 0, skipped = 0;
        double earliestReconvertedTime = double.PositiveInfinity;
        foreach (var burn in burns.OrderBy(b => b.Time.Seconds()))
        {
            double t = burn.Time.Seconds();
            if (plan.TryGetMetaAt(t) is not { } meta) continue;
            if (!predictorCurrent)
            {
                skipped++;
                continue;
            }
            if (PlannedBurnConverter.TryAuthorDvVlf(
                    vessels, vehicle, orbit, OthersExcept(burns, burn),
                    t, meta.Frame, meta.Authored, now, out var dvVlf) is not null)
            {
                failed++;
                continue;
            }
            if (BurnPlanWriter.TryEditDv(vehicle, burn, dvVlf.X, dvVlf.Y, dvVlf.Z)
                == "applied")
            {
                reconverted++;
                earliestReconvertedTime = Math.Min(earliestReconvertedTime, t);
            }
            else failed++;
        }
        // Capture AFTER the reconverts: TryEditDv writes in place, so the burns now
        // carry the new-trajectory dv the snapshot must hold.
        PlanSnapshotPersistenceState preRebaseSnapshot =
            plan.CaptureSnapshotPersistenceState();
        string? parentId = (vehicle.Orbit.Parent as Astronomical)?.Id;
        bool parentChainCleanAtStart =
            PlannedBurnConverter.ExistingBurnParentsReady(vehicle);
        var unresolvedParentTimes = new List<double>();
        var stockBurns = new List<PlanSnapshotBurn>(burns.Count);
        foreach (var b in burns)
        {
            double time = b.Time.Seconds();
            string? resolvedParentId = parentChainCleanAtStart
                ? PlannedBurnConverter.ExistingBurnParentId(
                    vehicle, b, patchChainReady: true)
                : null;
            bool parentWasPending = preRebaseSnapshot.ParentRefreshPendingAt(time);
            if (parentWasPending || resolvedParentId is null)
                unresolvedParentTimes.Add(time);
            stockBurns.Add(new PlanSnapshotBurn(time, FrameAdapter.ToCore(b.DeltaVVlf),
                preRebaseSnapshot.ParentForReplacement(time, resolvedParentId)
                    ?? parentId));
        }
        bool parentChainReady = parentChainCleanAtStart
            && PlannedBurnConverter.ExistingBurnParentsReady(vehicle);
        double epoch;
        StateVector anchor = default;
        predictorCurrent = predictorCurrent
            && vessels.TryReadAuthoritativePredictorState(
                vehicle, now, out anchor, out _);
        if (predictorCurrent)
        {
            epoch = now;
        }
        else
        {
            var sv = vehicle.Orbit.StateVectors; // committed live state (updates off-rails too)
            epoch = sv.StateTime.Seconds();
            anchor = tracked.AbsoluteFromGameState(vehicle.Orbit, in sv);
        }
        // Engine scalars re-captured with the rebase (finite-burn estimation): THE
        // shared read — the same FC totals, torn-read guard included, that every
        // rebuild capture uses.
        var engine = TrajectoryOverlay.ReadEngineScalars(vehicle, plan.PropulsionSource);
        plan.SetSnapshot(PlanSnapshot.Capture(epoch, anchor, parentId, stockBurns, engine,
            plan.PropulsionSource));
        IEnumerable<double> pendingParentTimes = parentChainReady
            ? unresolvedParentTimes
            : stockBurns.Select(b => b.TimeSeconds);
        if (parentChainReady && reconverted > 0)
            pendingParentTimes = pendingParentTimes.Concat(stockBurns
                .Where(b => b.TimeSeconds >= earliestReconvertedTime)
                .Select(b => b.TimeSeconds));
        plan.SnapshotMarkParentRefresh(pendingParentTimes);
        InvalidateAnalysis();
        _status = (failed, skipped) switch
        {
            (0, 0) => $"plan rebased onto the current trajectory ({reconverted} frame burn(s) reconverted)",
            (0, > 0) => $"plan rebased (live state); {skipped} frame burn(s) not reconverted until back on rails",
            _ => $"plan rebased; {reconverted} reconverted, {failed} failed, {skipped} skipped (off rails)",
        };
        string? multiParentSignature = FlightPlanModel.SnapshotParentSignature(stockBurns);
        tracked.LastSnapshotEvidencePlanRef = plan;
        tracked.LastSnapshotMultiParentSignature = multiParentSignature;
        tracked.LastSnapshotLogMs = Environment.TickCount64;
        ModLog.Info($"planner: plan rebased for '{vehicle.Id}' at t={epoch:F1} s "
            + $"({stockBurns.Count} burns captured, {reconverted} reconverted, {failed} failed, "
            + $"{skipped} skipped, predictorCurrent={predictorCurrent})"
            + FlightPlanModel.SnapshotParentEvidence(stockBurns));
    }

    /// <summary>The optimizer caption, rebuilt only when the target body changes —
    /// the panel draws every render frame and the body only moves on a map-frame
    /// switch (hoisted-constant convention, the analysis-cache precedent).</summary>
    private static (string Body, string Caption)? _optimizerCaption;

    /// <summary>Everything the main-thread apply of a finished optimize needs,
    /// captured together at launch so the pieces can never mispair: the owning game
    /// references, the target body/radius the status math uses, and the plan shape
    /// for the staleness check. One in-flight optimize at a time
    /// (<see cref="_optimize"/>); the panel keeps waiting for the OWNING vessel —
    /// drawing another vessel neither consumes nor cancels the job.</summary>
    private sealed record OptimizeContext(
        PeriapsisSolveJob Job, Vehicle Vehicle, Burn TargetBurn, string TargetBody,
        double TargetPeriapsis, double? TargetInclination, double TargetRadius,
        (double Time, double3 DvVlf)[] PlanShape, long PlanVersion,
        PropulsionSource Propulsion, EngineScalars Engine,
        VesselRegistry.RailsAuthoritySnapshot Authority)
    {
        public PredictorAuthorityPolicy.Reason? AuthorityLost { get; set; }
    }

    private static OptimizeContext? _optimize;

    private static void AdvanceOptimizer(VesselRegistry vessels)
    {
        if (_optimize is not { } context) return;
        if (context.AuthorityLost is null
            && !vessels.ValidateRailsAuthority(context.Authority, out var authorityReason))
        {
            context.AuthorityLost = authorityReason;
            if (!context.Job.Done) context.Job.Cancel();
            _status = $"optimizer cancelling: "
                + PredictorAuthorityPolicy.Describe(authorityReason)
                + " - nothing will be applied";
        }
        if (context.Job.Done
            && (context.AuthorityLost is not null || context.Job.Cancelled))
        {
            _status = context.AuthorityLost is { } lostReason
                ? "optimizer cancelled: " + PredictorAuthorityPolicy.Describe(lostReason)
                    + " - nothing applied"
                : "optimizer cancelled - nothing applied";
            _optimize = null;
        }
    }

    private static void ApplyCompletedOptimizer(VesselRegistry vessels, Vehicle vehicle,
        Orbit orbit, FlightPlanModel plan, IReadOnlyList<Burn> burns, double now)
    {
        if (_optimize is not { Job.Done: true, Job.Cancelled: false, AuthorityLost: null } context
            || !ReferenceEquals(context.Vehicle, vehicle)) return;
        _optimize = null;
        ApplyOptimizeResult(vessels, vehicle, orbit, plan, burns, context, now);
        InvalidateAnalysis();
    }

    /// <summary>Draws the fixed-Pe optimizer and its optional inclination target;
    /// background results apply only when the captured plan state is still current.</summary>
    private static void DrawOptimizer(VesselRegistry vessels, Vehicle vehicle, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, double now)
    {
        string? resolvedTarget =
            PeriapsisKernel.TargetBodyId(FrameManager.Active, (orbit.Parent as Astronomical)?.Id);
        if (resolvedTarget is not { } targetBody)
        {
            if (_optimize is { Job.Done: true } completed)
            {
                var route = OptimizeRoutingPolicy.RouteCompleted(
                    ReferenceEquals(completed.Vehicle, vehicle));
                if (route == CompletedOptimizeRoute.PreserveForOwner)
                {
                    // Target resolution belongs to the vessel currently being drawn.
                    // A missing target on ANOTHER vessel says nothing about the
                    // completed owner's captured solve; park it until that owner draws.
                    ImGui.TextWrapped($"optimize complete for '{completed.Vehicle.Id}'; "
                        + "switch back to apply it");
                    return;
                }
                ImGui.TextWrapped("optimize complete; result pending application"u8);
            }
            else if (_optimize is { Job.Done: false } active)
            {
                ImGui.TextWrapped(active.Job.StatusLine);
                if (ImGui.Button("Cancel"u8, (float2?)null)) active.Job.Cancel();
            }
            return;
        }
        if (_optimizerCaption is not { } caption
            || !string.Equals(caption.Body, targetBody, StringComparison.Ordinal))
            _optimizerCaption = caption =
                (targetBody, $"Optimize last burn at {targetBody} (the map frame's body)");
        ImGui.TextWrapped(caption.Caption);
        if (UiLayout.BeginProperties("##periapsis-target-property"u8,
                UiTheme.PropertyLabelWidth))
        {
            try
            {
                UiLayout.NextProperty("Target Pe (km)");
                SteppedRow("##petarget"u8, ref _peTargetKm, PeTargetSteps);
            }
            finally
            {
                ImGui.EndTable();
            }
        }
        ImGui.Checkbox("Also target inclination"u8, ref _optimizeInclination);
        if (_optimizeInclination)
        {
            if (UiLayout.BeginProperties("##inclination-target-property"u8,
                    UiTheme.PropertyLabelWidth))
            {
                try
                {
                    UiLayout.NextProperty("Target i (deg)");
                    SteppedRow("##itarget"u8, ref _inclinationTargetDegrees,
                        InclinationTargetSteps);
                }
                finally
                {
                    ImGui.EndTable();
                }
            }
        }
        if (_optimize is { } context)
        {
            if (context.AuthorityLost is { } lost)
            {
                ImGui.TextWrapped("optimizer cancelling: "
                    + PredictorAuthorityPolicy.Describe(lost));
                return;
            }
            if (context.Job.Cancelled && !context.Job.Done)
            {
                ImGui.TextWrapped("optimizer cancelling...");
                return;
            }
            if (!ReferenceEquals(context.Vehicle, vehicle))
            {
                // Another vessel owns the in-flight solve: do not consume its
                // result here, but keep cancellation globally reachable.
                ImGui.TextWrapped($"optimize running for '{context.Vehicle.Id}'...");
                if (!context.Job.Done
                    && ImGui.Button("Cancel"u8, (float2?)null)) context.Job.Cancel();
                return;
            }
            if (!context.Job.Done)
            {
                ImGui.TextWrapped(context.Job.StatusLine);
                if (ImGui.Button("Cancel"u8, (float2?)null)) context.Job.Cancel();
                return;
            }
            ImGui.TextWrapped("optimize complete; result pending application"u8);
            return;
        }
        if (burns.Count == 0)
        {
            ImGui.TextWrapped("add a burn first - Optimize adjusts the LAST burn (time + full dv)"u8);
            return;
        }
        if (!vessels.TryCaptureRailsAuthority(
                vehicle, out _, out var unavailableReason))
        {
            ImGui.TextWrapped("optimizer unavailable: "
                + PredictorAuthorityPolicy.Describe(unavailableReason));
            return;
        }
        if (ImGui.Button("Optimize"u8, (float2?)null))
            StartOptimize(vessels, vehicle, orbit, plan, burns, targetBody,
                _optimizeInclination, now);
    }

    /// <summary>Capture (main thread) and launch: every input the solver thread
    /// needs as plain data — burn times/dvs, per-burn basis parents (BurnParentId
    /// walks game patches), the movable time window — plus the plan shape for the
    /// apply-time staleness check.</summary>
    private static void StartOptimize(VesselRegistry vessels, Vehicle vehicle, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, string targetBody,
        bool targetInclination, double now)
    {
        if (!vessels.TryCaptureRailsAuthority(
                vehicle, out var authority, out var authorityReason))
        {
            _status = "optimizer refused: "
                + PredictorAuthorityPolicy.Describe(authorityReason);
            return;
        }
        var tracked = authority.Tracked;
        Vector3d equatorialPole = Vector3d.Zero;
        if (targetInclination
            && (!double.IsFinite(_inclinationTargetDegrees)
                || _inclinationTargetDegrees < 0 || _inclinationTargetDegrees > 180))
        {
            _status = "rejected: target inclination must be between 0 and 180 degrees";
            return;
        }
        if (targetInclination
            && !tracked.Rails.TryGetEquatorialPole(targetBody, out equatorialPole))
        {
            _status = $"rejected: equatorial pole for '{targetBody}' is unavailable";
            return;
        }
        if (!double.IsFinite(_peTargetKm) || _peTargetKm < 0)
        {
            _status = "rejected: target Pe must be a non-negative number of km";
            return;
        }
        double radius = tracked.Rails.MeanRadiusOf(targetBody);
        if (radius <= 0)
        {
            // Without a radius the km field would silently mean center distance —
            // the solver would happily park a "100 km Pe" deep inside the body.
            _status = $"rejected: no mean radius known for '{targetBody}' - cannot target an altitude";
            return;
        }
        if (orbit.Parent is not Astronomical fallbackParent)
        {
            _status = "rejected: orbit parent is not a celestial body";
            return;
        }
        // The LAST burn by the same ordering every fold uses (OrderBy is stable —
        // a hand-rolled max scan could pick a different burn on an exact time tie).
        var ordered = burns.OrderBy(b => b.Time.Seconds()).ToArray();
        Burn target = ordered[^1];
        double baselineTime = target.Time.Seconds();
        // The same clamped horizon the overlay predicts/draws to; the movable time
        // window keeps the burn past the previous burn (and the minimum lead) and
        // leaves scan room short of the horizon — but always INCLUDES the authored
        // baseline time when it is evaluable at all: the search must start from the
        // burn the user placed, not from a silently shifted one.
        double horizon = Math.Min(plan.EndSeconds, now + ModServices.Config.RailsAheadDays * 86400.0);
        horizon = Math.Min(horizon, tracked.Rails.Horizon);
        double previousBurnTime = ordered.Length >= 2 ? ordered[^2].Time.Seconds() : double.NegativeInfinity;
        double timeLo = Math.Max(now + PlannerKernel.MinLeadSeconds + 5.0, previousBurnTime + 1.0);
        double timeHi = Math.Max(horizon - 60.0, Math.Min(baselineTime, horizon - 2.0));
        if (timeHi <= timeLo)
        {
            _status = "optimizer: no room to move the burn inside the plan window - extend the plan length";
            return;
        }
        var others = new (double Time, Vector3d DvVlf, string BasisParentId)[ordered.Length - 1];
        for (int k = 0; k < others.Length; k++)
        {
            double time = ordered[k].Time.Seconds();
            others[k] = (time, FrameAdapter.ToCore(ordered[k].DeltaVVlf),
                PlannedBurnConverter.BurnParentId(vehicle, fallbackParent.Id, time));
        }
        double targetPe = radius + _peTargetKm * 1000.0;
        double? targetI = targetInclination
            ? _inclinationTargetDegrees * Math.PI / 180.0 : null;
        double peTolerance = Math.Max(100.0, 1e-4 * targetPe);
        double inclinationTolerance = Math.PI / 18000.0;
        // The objective folds the candidate burn the way the FC will FLY it (the
        // display fold's finite model) — an impulsive solve visibly misses its own
        // Pe for multi-minute burns. Engine scalars read the same way the overlay
        // captures them; unusable scalars or a disabled config keep it impulsive.
        var engine = TrajectoryOverlay.ReadEngineScalars(vehicle, plan.PropulsionSource);
        FiniteBurnFold? finite = ModServices.Config.FiniteBurnSliceSeconds > 0 && engine.Usable
            ? new FiniteBurnFold(engine, ModServices.Config.FiniteBurnSliceSeconds,
                ModServices.Config.FiniteBurnMaxSlices)
            : null;
        var prediction = tracked.Rails.TryCaptureSolverPredictionContext(now, horizon);
        if (prediction is null)
        {
            _status = "optimizer: detached prediction window is preparing - retry shortly";
            return;
        }
        if (!tracked.TryCaptureSolverSeed(authority.Lineage, now, out var seedState))
        {
            _status = "optimizer refused: the vessel coast changed or has not reached "
                + "the current time - retry shortly";
            return;
        }
        var job = new PeriapsisSolveJob
        {
            Tracked = tracked,
            SeedLineage = authority.Lineage,
            Prediction = prediction,
            SeedState = seedState,
            Finite = finite,
            TargetBodyId = targetBody,
            TargetPeriapsis = targetPe,
            PeriapsisTolerance = peTolerance,
            TargetInclination = targetI,
            InclinationTolerance = inclinationTolerance,
            EquatorialPole = equatorialPole,
            NowSeconds = now,
            HorizonSeconds = horizon,
            TimeLo = timeLo,
            TimeHi = timeHi,
            BaselineTime = baselineTime,
            BaselineDvVlf = FrameAdapter.ToCore(target.DeltaVVlf),
            OtherBurns = others,
            TargetBasisParentId =
                PlannedBurnConverter.BurnParentId(vehicle, fallbackParent.Id, baselineTime),
        };
        var shape = new (double, double3)[ordered.Length];
        for (int k = 0; k < ordered.Length; k++)
            shape[k] = (ordered[k].Time.Seconds(), ordered[k].DeltaVVlf);
        // Capture above protects every solve input from a KNOWN-stale predictor;
        // this second check closes the comparatively long main-thread snapshot
        // window. A transition immediately after it is still harmless: the worker
        // is lineage-pinned and apply revalidates around every stock write.
        if (!vessels.ValidateRailsAuthority(authority, out authorityReason))
        {
            _status = "optimizer refused before launch: "
                + PredictorAuthorityPolicy.Describe(authorityReason);
            return;
        }
        _optimize = new OptimizeContext(
            job, vehicle, target, targetBody, targetPe, targetI, radius, shape,
            plan.Version, plan.PropulsionSource, engine, authority);
        job.Start();
        _status = targetInclination
            ? "optimizing periapsis first, then improving inclination..."
            : "optimizing periapsis in background...";
        string targetLog = $"Pe {targetBody} target {_peTargetKm:F1} km"
            + (targetInclination ? $", i target {_inclinationTargetDegrees:F2} deg" : "");
        ModLog.Info($"planner: periapsis optimize started for "
            + $"'{vehicle.Id}': burn t={baselineTime:F1} s, {targetLog}, "
            + $"time window [{timeLo:F0}, {timeHi:F0}] s");
    }

    /// <summary>Main-thread apply of a finished solve: staleness guards first (user
    /// cancel, plan unchanged since capture, predictor seed lineage intact, the
    /// accepted model start (finite ignition or impulsive node) still ahead of now,
    /// VLF basis parent unchanged at the solved time), then the
    /// time edit (with meta re-key), the dv write, and the authoring-frame
    /// re-expression — the exact write path the manual affordances use. The caller
    /// already routed by vessel: this runs on the OWNING vessel's panel draw.</summary>
    private static void ApplyOptimizeResult(VesselRegistry vessels, Vehicle vehicle, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, OptimizeContext context, double now)
    {
        var job = context.Job;
        Burn target = context.TargetBurn;
        if (job.Cancelled)
        {
            // Covers the race where Cancel lands after the solver's own final
            // check: a result published anyway must still not apply.
            _status = "optimizer cancelled - nothing applied";
            return;
        }
        if (job.Failure is not null || job.Result is not { } result)
        {
            _status = job.Failure ?? "optimizer: no result";
            return;
        }
        if (!vessels.ValidateRailsAuthority(
                context.Authority, out var authorityReason))
        {
            _status = "optimizer: "
                + PredictorAuthorityPolicy.Describe(authorityReason)
                + " while solving - not applied";
            return;
        }
        var tracked = context.Authority.Tracked;
        if (plan.Version != context.PlanVersion
            || plan.PropulsionSource != context.Propulsion
            || TrajectoryOverlay.ReadEngineScalars(vehicle, context.Propulsion) != context.Engine)
        {
            _status = "optimizer: propulsion or plan snapshot changed while solving - not applied";
            return;
        }
        // Plan-shape staleness: any edit while the solver ran (add/remove/move/dv,
        // through the panel OR the stock editor) invalidates the solve — it folded
        // the captured plan, not this one.
        var ordered = burns.OrderBy(b => b.Time.Seconds()).ToArray();
        bool planChanged = ordered.Length != context.PlanShape.Length;
        if (!planChanged)
            for (int k = 0; k < ordered.Length; k++)
                if (Math.Abs(ordered[k].Time.Seconds() - context.PlanShape[k].Time) > 1e-9
                    || !ordered[k].DeltaVVlf.Equals(context.PlanShape[k].DvVlf))
                { planChanged = true; break; }
        if (planChanged || ordered.Length == 0 || !ReferenceEquals(ordered[^1], target))
        {
            _status = "optimizer: the plan changed while solving - not applied";
            return;
        }
        // Predictor seed lineage: a reseed (live-physics dip, save/load restore)
        // replaced the authoritative predictor, so the solve optimized a trajectory
        // that no longer exists — every other guard would still pass.
        // Under time warp the finite IGNITION may have slipped into the flight
        // computer's lead window even while its centered node remains ahead. Gate
        // the exact model start published with the result before any stock write.
        double simNow = Universe.GetElapsedSimTime().Seconds();
        if (!OptimizeApplyPolicy.ModeledStartHasLead(
                job.AcceptedModelStartSeconds, simNow, PlannerKernel.MinLeadSeconds))
        {
            _status = job.AcceptedModelStartSeconds < result.TimeSeconds
                ? "optimizer: finite-burn ignition passed while solving (time warp) - not applied"
                : "optimizer: modeled burn start passed while solving (time warp) - not applied";
            return;
        }
        // The solver expressed every candidate dv in the BASELINE time's patch
        // parent basis; stock executes the written VLF numbers in the patch parent
        // at the burn's ACTUAL time. If moving the time crossed an SOI patch
        // boundary the numbers would execute in a different basis than they were
        // solved in — refuse rather than write a dv that misses its own Pe.
        if (orbit.Parent is Astronomical applyFallback
            && PlannedBurnConverter.BurnParentId(vehicle, applyFallback.Id, result.TimeSeconds)
                is { } solvedTimeParent
            && !string.Equals(solvedTimeParent, job.TargetBasisParentId, StringComparison.Ordinal))
        {
            _status = $"optimizer: the solved burn time crosses an SOI patch boundary "
                + $"('{job.TargetBasisParentId}' -> '{solvedTimeParent}') - not applied "
                + "(move the burn there manually and re-run)";
            return;
        }
        double oldTime = target.Time.Seconds();
        double3 oldDv = target.DeltaVVlf;
        var representation = CaptureOptimizeRepresentation(plan);
        bool timeMoved = Math.Abs(result.TimeSeconds - oldTime) > 1e-6;
        bool timeWritten = false;
        if (!vessels.ValidateRailsAuthority(
                context.Authority, out authorityReason))
        {
            _status = "optimizer: "
                + PredictorAuthorityPolicy.Describe(authorityReason)
                + " before write - nothing applied";
            return;
        }
        if (timeMoved)
        {
            _status = BurnPlanWriter.TryEditTime(vehicle, target, result.TimeSeconds);
            if (_status != "applied")
            {
                string failedTimeVerdict = _status;
                _status = $"optimizer: time write failed ({failedTimeVerdict}) - nothing applied";
                return;
            }
            timeWritten = true;
            MoveMetaAndPicker(vehicle, plan, oldTime, result.TimeSeconds);
        }
        // A live transition may land between the optimizer's two independent stock
        // edits. Revalidate here and restore the first phase before returning.
        if (!vessels.ValidateRailsAuthority(
                context.Authority, out authorityReason))
        {
            RejectOptimizeAuthorityDuringApply(vehicle, plan, target, oldTime, oldDv,
                timeWritten, deltaVWritten: false, reason: authorityReason,
                representation: representation);
            return;
        }
        string dvVerdict = BurnPlanWriter.TryEditDv(
            vehicle, target, result.Prograde, result.Normal, result.Outward);
        if (dvVerdict != "applied")
        {
            // The writer already restored delta-v transactionally. Only a successful
            // preceding time move (and its representation key) remains to roll back.
            var rollback = OptimizeApplyPolicy.ForAuthorityLoss(
                timeWritten, deltaVWritten: false);
            string rollbackStatus = RollBackOptimizeWrites(
                vehicle, plan, target, oldTime, oldDv, rollback, representation);
            _status = $"optimizer: dv write failed ({dvVerdict}) - {rollbackStatus}";
            return;
        }
        if (!vessels.ValidateRailsAuthority(
                context.Authority, out authorityReason))
        {
            RejectOptimizeAuthorityDuringApply(vehicle, plan, target, oldTime, oldDv,
                timeWritten, deltaVWritten: true, reason: authorityReason,
                representation: representation);
            return;
        }
        // Frame-authored burn: re-express the optimized physical dv in its authoring
        // frame so the [STALE]/Reconvert affordance cannot offer to undo the
        // optimization; a failed re-expression degrades the burn honestly to VLF.
        var optimizedMeta = plan.TryGetMetaAt(result.TimeSeconds);
        if (optimizedMeta is { } meta)
        {
            if (PlannedBurnConverter.TryCurrentComponentsInFrame(vessels, vehicle, orbit,
                    OthersExcept(burns, target), result.TimeSeconds, target.DeltaVVlf, meta.Frame,
                    now, out var components) is null)
                meta.Authored = components;
            else
                plan.RemoveMetaAt(result.TimeSeconds);
        }
        // Conversion itself reads the predictor. If authority changed during it,
        // restore representation first, then physical dv/time and the metadata key.
        if (!vessels.ValidateRailsAuthority(
                context.Authority, out authorityReason))
        {
            RejectOptimizeAuthorityDuringApply(vehicle, plan, target, oldTime, oldDv,
                timeWritten, deltaVWritten: true, reason: authorityReason,
                representation: representation);
            return;
        }
        string objectiveResult =
            $"Pe {(result.AchievedPeriapsis - context.TargetRadius) / 1000.0:N1} km"
            + (result.AchievedInclination is { } achievedI
                ? $", i {achievedI * 180.0 / Math.PI:F2} deg" : "");
        string objectiveTarget =
            $"Pe {(context.TargetPeriapsis - context.TargetRadius) / 1000.0:F1} km"
            + (context.TargetInclination is { } targetI
                ? $", i {targetI * 180.0 / Math.PI:F2} deg" : "");
        double shift = result.TimeSeconds - oldTime;
        string budgetNote = job.BudgetExhausted ? ", evaluation budget hit - best found" : "";
        _status = $"optimized: {objectiveResult} at {context.TargetBody}, |dv| {result.Magnitude:F2} m/s "
            + $"(P {result.Prograde:F2} N {result.Normal:F2} O {result.Outward:F2}, "
            + $"t {(shift >= 0 ? "+" : "")}{shift:F0} s, {job.Evaluations} evals, {job.ElapsedMs} ms{budgetNote})";
        ModLog.Info($"planner: periapsis optimize for '{vehicle.Id}': burn t={oldTime:F1} -> "
            + $"{result.TimeSeconds:F1} s, dv ({result.Prograde:F2}, {result.Normal:F2}, "
            + $"{result.Outward:F2}) m/s (|dv| {result.Magnitude:F2}), {objectiveResult} at {context.TargetBody} "
            + $"(target {objectiveTarget}, {job.Evaluations} evals, "
            + $"{job.ElapsedMs} ms{budgetNote})");
    }

    /// <summary>All burns except the edited one — the fold input for every edit
    /// affordance (the edited burn's own slot is being replaced). Exclusion by
    /// reference identity: BurnPlanWriter.Snapshot returns the live Burn instances.</summary>
    private static void RejectOptimizeAuthorityDuringApply(
        Vehicle vehicle,
        FlightPlanModel plan,
        Burn target,
        double oldTime,
        double3 oldDv,
        bool timeWritten,
        bool deltaVWritten,
        PredictorAuthorityPolicy.Reason reason)
    {
        var rollback = OptimizeApplyPolicy.ForAuthorityLoss(timeWritten, deltaVWritten);
        string rollbackStatus = rollback == OptimizeApplyRollback.None
            ? "nothing applied"
            : RollBackOptimizeWrites(vehicle, plan, target, oldTime, oldDv, rollback);
        _status = "optimizer: " + PredictorAuthorityPolicy.Describe(reason)
            + " during apply - " + rollbackStatus;
    }

    private static void RejectOptimizeAuthorityDuringApply(
        Vehicle vehicle,
        FlightPlanModel plan,
        Burn target,
        double oldTime,
        double3 oldDv,
        bool timeWritten,
        bool deltaVWritten,
        PredictorAuthorityPolicy.Reason reason,
        OptimizeRepresentationSnapshot representation)
    {
        RejectOptimizeAuthorityDuringApply(vehicle, plan, target, oldTime, oldDv,
            timeWritten, deltaVWritten, reason);
        RestoreOptimizeRepresentation(plan, representation);
    }

    private sealed record OptimizeRepresentationSnapshot(
        OptimizeMetaSnapshot[] Meta,
        string? PickerVesselId,
        double PickerBurnTime);

    private readonly record struct OptimizeMetaSnapshot(
        FlightPlanBurnMeta Meta,
        double TimeSeconds,
        Vector3d Authored,
        long StampMs);

    private static OptimizeRepresentationSnapshot CaptureOptimizeRepresentation(
        FlightPlanModel plan) =>
        new(plan.Meta.Select(meta => new OptimizeMetaSnapshot(
                meta, meta.TimeSeconds, meta.Authored, meta.StampMs)).ToArray(),
            _pickerVesselId, _pickerBurnTime);

    private static void RestoreOptimizeRepresentation(
        FlightPlanModel plan,
        OptimizeRepresentationSnapshot snapshot)
    {
        foreach (var meta in plan.Meta.ToArray())
            plan.RemoveMetaAt(meta.TimeSeconds);
        foreach (var saved in snapshot.Meta)
        {
            saved.Meta.TimeSeconds = saved.TimeSeconds;
            saved.Meta.Authored = saved.Authored;
            saved.Meta.StampMs = saved.StampMs;
            plan.SetMeta(saved.Meta);
        }
        _pickerVesselId = snapshot.PickerVesselId;
        _pickerBurnTime = snapshot.PickerBurnTime;
    }

    private static string RollBackOptimizeWrites(
        Vehicle vehicle,
        FlightPlanModel plan,
        Burn target,
        double oldTime,
        double3 oldDv,
        OptimizeApplyRollback rollback,
        OptimizeRepresentationSnapshot representation)
    {
        string status = RollBackOptimizeWrites(
            vehicle, plan, target, oldTime, oldDv, rollback);
        RestoreOptimizeRepresentation(plan, representation);
        return status;
    }

    /// <summary>Restores physical VLF before time so the plan snapshot and time-keyed
    /// metadata move together. Both restores are attempted independently and any
    /// incomplete rollback remains panel-visible.</summary>
    private static string RollBackOptimizeWrites(
        Vehicle vehicle,
        FlightPlanModel plan,
        Burn target,
        double oldTime,
        double3 oldDv,
        OptimizeApplyRollback rollback)
    {
        bool restoreDv = (rollback & OptimizeApplyRollback.DeltaV) != 0;
        bool restoreTime = (rollback & OptimizeApplyRollback.Time) != 0;
        string dvVerdict = restoreDv
            ? BurnPlanWriter.TryEditDv(vehicle, target, oldDv.X, oldDv.Y, oldDv.Z)
            : "not required";
        double currentTime = target.Time.Seconds();
        string timeVerdict = restoreTime
            ? BurnPlanWriter.TryEditTime(vehicle, target, oldTime)
            : "not required";
        if (restoreTime && timeVerdict == "applied"
            && Math.Abs(currentTime - oldTime) > 1e-9)
            MoveMetaAndPicker(vehicle, plan, currentTime, oldTime);
        bool complete = (!restoreDv || dvVerdict == "applied")
            && (!restoreTime || timeVerdict == "applied");
        return complete
            ? "changes rolled back"
            : $"rollback incomplete (dv: {dvVerdict}; time: {timeVerdict})";
    }

    private static List<Burn> OthersExcept(IReadOnlyList<Burn> burns, Burn burn)
    {
        var others = new List<Burn>(Math.Max(0, burns.Count - 1));
        foreach (var b in burns) if (!ReferenceEquals(b, burn)) others.Add(b);
        return others;
    }

    private static void ReconvertAfterTimeEdit(VesselRegistry vessels,
        Vehicle vehicle, TrackedVessel tracked, Orbit orbit,
        IReadOnlyList<Burn> burns, Burn burn, FlightPlanBurnMeta meta, double newTime, double now)
    {
        if (PlannedBurnConverter.TryAuthorDvVlf(
                vessels, vehicle, orbit, OthersExcept(burns, burn),
                newTime, meta.Frame, meta.Authored, now, out var dvVlf) is { } error)
        {
            _status = $"time applied; frame reconversion failed: {error}";
        }
        else
        {
            _status = BurnPlanWriter.TryEditDv(vehicle, burn, dvVlf.X, dvVlf.Y, dvVlf.Z) == "applied"
                ? "time applied; dv reconverted in authoring frame"
                : "time applied; dv reconversion rejected";
        }
    }

    private static void SwitchFrame(VesselRegistry vessels,
        Vehicle vehicle, TrackedVessel? tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, Burn burn, FlightPlanBurnMeta? meta,
        FrameSpec? target, double now)
    {
        double t = burn.Time.Seconds();
        if (target is null)
        {
            if (meta is null) return; // already VLF
            plan.RemoveMetaAt(t);
            _status = "burn now authored in VLF";
            InvalidateAnalysis();
            return;
        }
        if (meta is not null && target == meta.Frame) return; // already there
        if (tracked is null)
        {
            _status = "vessel not on rails - frame authoring unavailable";
            return;
        }
        if (PlannedBurnConverter.TryCurrentComponentsInFrame(
                vessels, vehicle, orbit,
                OthersExcept(burns, burn), t, burn.DeltaVVlf, target, now,
                out var components) is { } error)
        {
            _status = error;
            return;
        }
        plan.SetMeta(new FlightPlanBurnMeta
        {
            TimeSeconds = t,
            Frame = target,
            Authored = components,
            StampMs = Environment.TickCount64,
        });
        _status = $"burn re-expressed in {target.Label}";
        InvalidateAnalysis();
    }

    /// <summary>The burn frame-picker window: the SAME collapsible body tree as the
    /// frames panel (its own expansion state), plus the VLF row — targeted at the burn
    /// whose Frame button opened it, keyed by burn time like the plan meta. Draws only
    /// while its OWNING vessel's plan is drawn; the picker follows time edits
    /// (<see cref="RetargetPicker"/>) and closes with its burn.</summary>
    private static void DrawFramePicker(VesselRegistry vessels,
        Vehicle vehicle, TrackedVessel? tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, double now)
    {
        if (_pickerVesselId is null) return;
        if (!string.Equals(_pickerVesselId, vehicle.Id, StringComparison.Ordinal))
        {
            // Control switched to another vessel: the picker is a transient choice
            // about the plan ON SCREEN — close it rather than leave it latently open
            // to pop back unbidden when control returns to the old vessel.
            ClosePicker();
            return;
        }
        // NEAREST burn within the meta tolerance — the same match rule
        // FlightPlanModel.TryGetMetaAt uses, so the burn found here and the meta
        // edited below can never resolve to different slots.
        Burn? burn = null;
        double bestDelta = BurnIdentityPolicy.ToleranceSeconds;
        foreach (var b in burns)
        {
            if (BurnIdentityPolicy.TryMatch(
                    b.Time.Seconds(), _pickerBurnTime, out double delta)
                && delta <= bestDelta)
            {
                bestDelta = delta;
                burn = b;
            }
        }
        if (burn is null)
        {
            ClosePicker(); // the burn is gone (stock editor delete, plan delete)
            return;
        }
        UiTheme.PrepareWindow(480f, 600f, 420f, 300f);
        ImGui.Begin("Whisker Dynamics: Burn Frame"u8);
        try
        {
            var meta = plan.TryGetMetaAt(_pickerBurnTime);
            ImGui.Text($"burn at plan+{TimeDisplayKernel.FormatDuration(plan.PlanRelative(burn.Time.Seconds()))} - "
                + $"authored in {meta?.Frame.Label ?? VlfLabel}");
            if (FrameTreeControl.FrameChoice(VlfLabel, meta is null))
                SwitchFrame(vessels, vehicle, tracked, orbit, plan, burns, burn, meta, null, now);
            ImGui.SetItemTooltip("stock prograde/normal/outward of the vessel's own trajectory"u8);
            ImGui.Separator();
            if (PickerTree.Draw(meta?.Frame) is { } clicked)
                SwitchFrame(vessels, vehicle, tracked, orbit, plan, burns, burn, meta, clicked, now);
            ImGui.Separator();
            if (ImGui.Button("Close"u8, (float2?)null)) ClosePicker();
        }
        finally
        {
            ImGui.End(); // Begin/End must always balance (StatusPanel precedent)
        }
    }

    private static void EditComponents(VesselRegistry vessels,
        Vehicle vehicle, TrackedVessel? tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, Burn burn, FlightPlanBurnMeta? meta,
        Vector3d components, double now)
    {
        if (meta is null)
        {
            _status = BurnPlanWriter.TryEditDv(vehicle, burn, components.X, components.Y, components.Z);
            return;
        }
        if (tracked is null)
        {
            _status = "vessel not on rails - frame authoring unavailable";
            return;
        }
        double t = burn.Time.Seconds();
        if (PlannedBurnConverter.TryAuthorDvVlf(
                vessels, vehicle, orbit, OthersExcept(burns, burn),
                t, meta.Frame, components, now, out var dvVlf) is { } error)
        {
            _status = error;
            return;
        }
        _status = BurnPlanWriter.TryEditDv(vehicle, burn, dvVlf.X, dvVlf.Y, dvVlf.Z);
        if (_status == "applied") meta.Authored = components;
    }

    private static string QueueBurn(VesselRegistry vessels,
        Vehicle vehicle, TrackedVessel? tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, double now,
        double burnTime, FrameSpec? frame, Vector3d components,
        bool allowVlfFallback)
    {
        if (plan.RejectOutsideWindow(burnTime, now, AvailableRailsDays(tracked, now)) is { } outside)
            return outside;
        double3 dvVlf = PlannerKernel.ComposeVlf(
            components.X, components.Y, components.Z);
        string? frameRefusal = null;
        if (frame is not null)
        {
            frameRefusal = tracked is null
                ? "vessel not on rails"
                : PlannedBurnConverter.TryAuthorDvVlf(
                    vessels, vehicle, orbit, burns,
                    burnTime, frame, components, now, out dvVlf);
        }
        if (frameRefusal is not null && !allowVlfFallback)
            return $"rejected: {frame!.Label} unavailable: {frameRefusal}";
        string verdict = BurnPlanWriter.TryAdd(vehicle, burnTime, dvVlf);
        if (verdict != "queued")
            return verdict;
        if (frame is null || frameRefusal is not null)
        {
            return frame is null
                ? "queued (authored in VLF)"
                : $"queued (authored in VLF - {frame.Label} unavailable: {frameRefusal})";
        }
        plan.SetMeta(new FlightPlanBurnMeta
        {
            TimeSeconds = burnTime,
            Frame = frame,
            Authored = components,
            StampMs = Environment.TickCount64, // grace: the burn lands next frame
        });
        return $"queued (authored in {frame.Label})";
    }

    private static void AddBurn(VesselRegistry vessels,
        Vehicle vehicle, TrackedVessel? tracked, Orbit orbit,
        FlightPlanModel plan, IReadOnlyList<Burn> burns, double now)
    {
        _status = QueueBurn(vessels, vehicle, tracked, orbit, plan, burns, now,
            now + AddLeadSeconds, FrameManager.Active, Vector3d.Zero,
            allowVlfFallback: true);
    }

    /// <summary>Component row labels: stock's editor words for VLF (Burn.cs:718/732/746);
    /// catalog frames author in PROGRADE/RADIAL/NORMAL of the vessel's frame-relative
    /// trajectory; see <see cref="BurnFrameKernel.FrenetToFrame"/>. The selected frame
    /// defines what "prograde" means; in a rotating pair frame, it is the direction of
    /// motion in that frame.</summary>
    private static (string C1, string C2, string C3) ComponentLabels(FrameSpec? spec) => spec switch
    {
        null => ("Prograde", "Normal", "Outward"),
        _ => ("Prograde", "Radial", "Normal"),
    };

    /// <summary>Magnitude steppers flank a numeric field at several scales. One click
    /// nudges precisely without keyboard entry, while the field still takes exact values.</summary>
    private static readonly (double Step, string Minus, string Plus)[] DvSteps =
        [(0.1, "-0.1", "+0.1"), (1.0, "-1", "+1"), (10.0, "-10", "+10")];

    private static readonly (double Step, string Minus, string Plus)[] BurnTimeSteps =
        [(10.0, "-10s", "+10s"), (60.0, "-1m", "+1m"), (600.0, "-10m", "+10m"),
         (3600.0, "-1h", "+1h"), (86400.0, "-1d", "+1d")];

    // Steps in SECONDS like every duration row edits, labeled in the days the
    // clicks mean.
    private static readonly (double Step, string Minus, string Plus)[] PlanLengthSteps =
        [(86400.0, "-1d", "+1d"), (604800.0, "-7d", "+7d"), (2592000.0, "-30d", "+30d")];

    private static readonly (double Step, string Minus, string Plus)[] PeTargetSteps =
        [(1.0, "-1", "+1"), (10.0, "-10", "+10"), (100.0, "-100", "+100")];

    private static readonly (double Step, string Minus, string Plus)[] InclinationTargetSteps =
        [(0.1, "-0.1", "+0.1"), (1.0, "-1", "+1"), (10.0, "-10", "+10")];

    /// <summary>Numeric field flanked by aligned decrement and increment buttons;
    /// true when the value changed this frame. Button labels are precomposed in the
    /// step tables to avoid per-frame render-thread allocations.</summary>
    private static bool SteppedRow(ReadOnlySpan<byte> rowId, ref double value,
        (double Step, string Minus, string Plus)[] steps)
    {
        bool changed = false;
        ImGui.PushID(rowId);
        try
        {
            float fieldWidth = UiLayout.MeasureStepFieldWidth(steps.Length);
            double delta = UiLayout.StepDecrements(steps);
            ImGui.SetNextItemWidth(fieldWidth);
            changed |= ImGui.InputDouble("##value"u8, ref value, 0.0, 0.0,
                default(ImString), ImGuiInputTextFlags.CharsDecimal);
            delta += UiLayout.StepIncrements(steps);
            value += delta;
            changed |= delta != 0;
        }
        finally
        {
            ImGui.PopID();
        }
        return changed;
    }

    /// <summary>The shared duration row (<see cref="DurationField"/>), with parse
    /// failures routed to this panel's status line.</summary>
    private static bool DurationRow(ReadOnlySpan<byte> rowId, string rowKey, double id,
        ref double seconds, (double Step, string Minus, string Plus)[] steps)
    {
        bool changed = DurationField.Row(rowId, rowKey, id, ref seconds, steps,
            out string? parseError);
        if (parseError is not null) _status = parseError;
        return changed;
    }

    /// <summary>One labeled delta-v component property with a numeric field and
    /// 0.1/1/10 m/s steppers. True when edited.</summary>
    private static bool ComponentRow(string label, ReadOnlySpan<byte> id, ref double value)
    {
        ImGui.PushID(id);
        try
        {
            if (!UiLayout.BeginProperties("##component-property"u8,
                    UiTheme.PropertyLabelWidth)) return false;
            try
            {
                UiLayout.NextProperty($"{label} (m/s)");
                return SteppedRow("##component"u8, ref value, DvSteps);
            }
            finally
            {
                ImGui.EndTable();
            }
        }
        finally
        {
            ImGui.PopID();
        }
    }

    private static IReadOnlyList<PlannedBurnConverter.BurnAnalysis>? RefreshAnalysis(
        VesselRegistry vessels, Vehicle vehicle, TrackedVessel? tracked,
        Orbit orbit, IReadOnlyList<Burn> burns,
        FlightPlanModel plan, double now)
    {
        if (tracked is null || plan.Meta.Count == 0)
        {
            InvalidateAnalysis();
            return null; // nothing frame-authored: VLF burns read straight off the plan
        }
        long nowMs = Environment.TickCount64;
        if (_analysis is { } cache
            && string.Equals(vehicle.Id, cache.Vessel, StringComparison.Ordinal)
            && nowMs - cache.WallMs < AnalysisMaxAgeMs
            && AnalysisShapeMatches(cache, burns, plan))
            return cache.Results;
        // A null (failed) pass is cached like a success: retry at most once per
        // AnalysisMaxAgeMs, never once per rendered frame.
        var results = PlannedBurnConverter.Analyze(
            vessels, vehicle, orbit, burns, plan, now);
        _analysis = new AnalysisCache(vehicle.Id,
            CaptureBurnShape(burns), CaptureMetaShape(plan), nowMs, results);
        return results;
    }

    /// <summary>Exact, allocation-free cache validation. Shape arrays are captured
    /// only when Analyze actually runs; cache hits compare the live values in place
    /// instead of constructing and formatting a full string every rendered frame.</summary>
    private static bool AnalysisShapeMatches(AnalysisCache cache,
        IReadOnlyList<Burn> burns, FlightPlanModel plan)
    {
        if (cache.Burns.Length != burns.Count || cache.Metas.Length != plan.Meta.Count)
            return false;
        for (int i = 0; i < burns.Count; i++)
        {
            var burn = burns[i];
            var dv = burn.DeltaVVlf;
            var shape = cache.Burns[i];
            if (!SameBits(shape.Time, burn.Time.Seconds())
                || !SameBits(shape.X, dv.X)
                || !SameBits(shape.Y, dv.Y)
                || !SameBits(shape.Z, dv.Z))
                return false;
        }
        for (int i = 0; i < plan.Meta.Count; i++)
        {
            var meta = plan.Meta[i];
            var shape = cache.Metas[i];
            if (!SameBits(shape.Time, meta.TimeSeconds)
                || shape.Kind != meta.Frame.Kind
                || !string.Equals(shape.PrimaryId, meta.Frame.PrimaryId,
                    StringComparison.Ordinal)
                || !string.Equals(shape.SecondaryId, meta.Frame.SecondaryId,
                    StringComparison.Ordinal)
                || !SameBits(shape.X, meta.Authored.X)
                || !SameBits(shape.Y, meta.Authored.Y)
                || !SameBits(shape.Z, meta.Authored.Z))
                return false;
        }
        return true;
    }

    private static AnalysisBurnShape[] CaptureBurnShape(IReadOnlyList<Burn> burns)
    {
        var shape = new AnalysisBurnShape[burns.Count];
        for (int i = 0; i < burns.Count; i++)
        {
            var burn = burns[i];
            var dv = burn.DeltaVVlf;
            shape[i] = new AnalysisBurnShape(
                burn.Time.Seconds(), dv.X, dv.Y, dv.Z);
        }
        return shape;
    }

    private static AnalysisMetaShape[] CaptureMetaShape(FlightPlanModel plan)
    {
        var shape = new AnalysisMetaShape[plan.Meta.Count];
        for (int i = 0; i < plan.Meta.Count; i++)
        {
            var meta = plan.Meta[i];
            shape[i] = new AnalysisMetaShape(
                meta.TimeSeconds, meta.Frame.Kind,
                meta.Frame.PrimaryId, meta.Frame.SecondaryId,
                meta.Authored.X, meta.Authored.Y, meta.Authored.Z);
        }
        return shape;
    }

    private static bool SameBits(double left, double right) =>
        BitConverter.DoubleToInt64Bits(left) == BitConverter.DoubleToInt64Bits(right);
}

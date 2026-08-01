using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning;

/// <summary>Thin KSA adapter over <see cref="BurnFrameKernel"/>: evaluates the
/// PREDICTED n-body vessel state at burn times (a display predictor clone with earlier
/// planned burns folded in — the authoritative predictor never sees plan burns, the
/// TrajectoryOverlay invariant) and converts frame-authored delta-v components to/from
/// the stock VLF components the BurnPlan executes. Frame-authored components are
/// PROGRADE/RADIAL/NORMAL of the vessel's frame-relative trajectory
/// (<see cref="BurnFrameKernel.FrenetToFrame"/>), with the
/// frame-space state sampled numerically by <see cref="TryFrameState"/>.
///
/// Chaining mirrors stock and the overlay alike: each burn's VLF basis is that of the
/// PRE-burn state with all earlier burns applied (FlightPlan.CalculateBurnPatch chains
/// patches; the fold goes through OverlayKernel.FoldBurns, the overlay's own pipeline),
/// so an edit to an earlier burn shifts every later burn's basis. The basis PARENT is
/// the burn-time PATCH's parent, exactly how stock executes DeltaVVlf (see
/// PlannerKernel.BurnBasisParent for the decompiled evidence) — a cross-SOI burn
/// converts in the frame it will execute in, not the panel-time orbit parent's.
/// STALENESS HANDLING (design decision, tested in BurnFrameKernelTests): the authored
/// frame components stay the user's intent in the plan metadata; <see cref="Analyze"/>
/// recomputes the conversion whenever the plan changes and flags burns whose stored
/// stock VLF no longer realizes the authored components beyond
/// <see cref="StaleToleranceMps"/> — the panel then offers an explicit per-burn
/// "Reconvert" (writes the fresh VLF through BurnPlanWriter). No automatic rewriting
/// and no iterative solver: silent mutation could fight the user (or the stock editor)
/// mid-edit, and one explicit pass converges for the impulse model because burn i's
/// basis depends only on burns before it. ONE exception: a basis-parent flip (an SOI
/// handoff rebuilding the patch chain under a pending burn) is not a user edit and
/// would execute the stored components rotated;
/// BurnPlannerPanel.AdvanceBasisReconversion re-realizes them automatically
/// (meta.BasisParentId is the flip detector). Main-thread only
/// (same phase as the panel; SampleSpecPose's Surface path walks the live system).</summary>
internal static class PlannedBurnConverter
{
    /// <summary>Frame-authored burns drift stale when the plan around them changes; a
    /// 0.01 m/s realization error is far below execution/prediction noise but well
    /// above fp round-off of an untouched conversion.</summary>
    internal const double StaleToleranceMps = 0.01;

    /// <summary>One burn's conversion picture for the panel. DisplayComponents is the
    /// burn's CURRENT physical delta-v expressed in its authoring frame (the inverse
    /// conversion — users edit in the frame they authored in); FreshDvVlf is the
    /// authored components converted at the current prediction (what "Reconvert"
    /// writes). Either is null when its conversion failed — Note says why.
    /// ExecutionRealized suppresses Stale: the stored components are the upkeep's
    /// execution-basis realization, and their difference from FreshDvVlf IS the
    /// drift the upkeep corrects — a Reconvert would undo that correction.</summary>
    internal sealed record BurnAnalysis(
        double TimeSeconds,
        Vector3d DvVlf,
        FlightPlanBurnMeta? Meta,
        Vector3d? DisplayComponents,
        Vector3d? FreshDvVlf,
        string? Note,
        bool ExecutionRealized = false)
    {
        public bool Stale => Meta is not null && !ExecutionRealized
            && FreshDvVlf is { } fresh
            && BurnFrameKernel.IsStale(fresh, DvVlf, StaleToleranceMps);
    }

    /// <summary>Converts authored components in <paramref name="frame"/> at
    /// <paramref name="burnTimeSeconds"/> into stock VLF components, folding every
    /// existing burn strictly earlier than the target time into the prediction (the
    /// burn being edited, at exactly that time, is thereby excluded). Null reason on
    /// success. Never throws.</summary>
    internal static string? TryAuthorDvVlf(VesselRegistry vessels, Vehicle vehicle,
        Orbit currentOrbit, IReadOnlyList<Burn> burns, double burnTimeSeconds,
        FrameSpec frame, Vector3d authored, double nowSeconds, out double3 dvVlf)
    {
        dvVlf = default;
        try
        {
            if (TryAuthoredDvEcl(vessels, vehicle, currentOrbit, burns, burnTimeSeconds,
                    frame, authored, nowSeconds, out var authority, out var dvEcl,
                    out var rRel, out var vRel) is { } reason)
                return reason;
            if (BurnFrameKernel.EclToVlf(dvEcl, rRel, vRel) is not { } vlf)
                return "degenerate VLF frame at burn time (radial trajectory)";
            if (!vessels.ValidateRailsAuthority(authority, out var authorityReason))
                return AuthorityFailure(authorityReason);
            dvVlf = FrameAdapter.ToGame(vlf);
            return null;
        }
        catch (Exception e)
        {
            return $"conversion failed: {e.Message}";
        }
    }

    /// <summary>Shared authored-side pipeline of both VLF realizations: authority
    /// capture, then authored components -> frame-space delta-v -> ecliptic. The
    /// caller validates <paramref name="authority"/> after its game reads. Null
    /// reason on success.</summary>
    private static string? TryAuthoredDvEcl(VesselRegistry vessels, Vehicle vehicle,
        Orbit currentOrbit, IReadOnlyList<Burn> burns, double burnTimeSeconds,
        FrameSpec frame, Vector3d authored, double nowSeconds,
        out VesselRegistry.RailsAuthoritySnapshot authority, out Vector3d dvEcl,
        out Vector3d rRel, out Vector3d vRel)
    {
        dvEcl = default;
        rRel = default;
        vRel = default;
        if (!vessels.TryCaptureRailsAuthority(
                vehicle, out authority, out var authorityReason))
            return AuthorityFailure(authorityReason);
        if (TryPredictedBasis(vessels, authority, vehicle, currentOrbit, burns,
                burnTimeSeconds, frame, nowSeconds, out var pose, out rRel, out vRel,
                out var rFrame, out var vFrame) is { } reason)
            return reason;
        if (BurnFrameKernel.FrenetToFrame(authored, rFrame, vFrame) is not { } dvFrame)
            return "degenerate trajectory in this frame at burn time (vessel nearly stationary or radial in the frame)";
        dvEcl = BurnFrameKernel.FrameToEcl(dvFrame, pose);
        return null;
    }

    /// <summary>Re-realizes authored frame components into stock VLF against the
    /// burn's CURRENT stock patch basis — the conic stock executes DeltaVVlf through —
    /// instead of the predicted n-body basis <see cref="TryAuthorDvVlf"/> uses, so
    /// conic drift cannot rotate the executed burn. The authored side (what
    /// "retrograde in this frame" means physically) still comes from the n-body
    /// prediction. The patch resolution mirrors stock's Burn.Patch getter
    /// (Burn.cs:100-113) and is deliberately UNSCOPED: it must match what execution
    /// will actually read. <paramref name="predictorDvVlf"/> is the same intent
    /// realized against the predicted n-body basis (null when degenerate) — the
    /// display fold and analysis interpret VLF there, so an execution-basis write
    /// must hand them this vector. Null reason on success. Never throws.</summary>
    internal static string? TryAuthorDvVlfForExecution(VesselRegistry vessels,
        Vehicle vehicle, Orbit currentOrbit, IReadOnlyList<Burn> otherBurns, Burn burn,
        double burnTimeSeconds, FrameSpec frame, Vector3d authored, double nowSeconds,
        out double3 dvVlf, out Vector3d? predictorDvVlf)
    {
        dvVlf = default;
        predictorDvVlf = null;
        try
        {
            if (TryAuthoredDvEcl(vessels, vehicle, currentOrbit, otherBurns,
                    burnTimeSeconds, frame, authored, nowSeconds, out var authority,
                    out var dvEcl, out var rRel, out var vRel) is { } reason)
                return reason;
            PatchedConic? patch = vehicle.FlightComputer.BurnPlan.TryGetBurnPatch(burn)
                ?? vehicle.FlightPlan.FirstPatch;
            if (patch is null) return "no stock patch resolves for this burn";
            if (patch.Orbit.Parent is not { } patchParent)
                return "stock patch has no parent body";
            var conicState = patch.Orbit.GetStateVectorsAt(new SimTime(burnTimeSeconds));
            var cci2Cce = patchParent.GetCci2Cce();
            Vector3d rConic = FrameAdapter.CciToEcl(conicState.PositionCci, cci2Cce);
            Vector3d vConic = FrameAdapter.CciToEcl(conicState.VelocityCci, cci2Cce);
            if (BurnFrameKernel.EclToVlf(dvEcl, rConic, vConic) is not { } vlf)
                return "degenerate stock VLF basis at burn time (radial patch trajectory)";
            if (!vessels.ValidateRailsAuthority(authority, out var authorityReason))
                return AuthorityFailure(authorityReason);
            dvVlf = FrameAdapter.ToGame(vlf);
            predictorDvVlf = BurnFrameKernel.EclToVlf(dvEcl, rRel, vRel);
            return null;
        }
        catch (Exception e)
        {
            return $"conversion failed: {e.Message}";
        }
    }

    /// <summary>Re-expresses a burn's CURRENT physical delta-v as components in
    /// <paramref name="frame"/> at the burn time — the frame-switch affordance (the
    /// physical burn is unchanged; only its authoring representation moves). The other
    /// burns (the edited one excluded by the caller) fold as usual. Null reason on
    /// success. Never throws.</summary>
    internal static string? TryCurrentComponentsInFrame(VesselRegistry vessels, Vehicle vehicle,
        Orbit currentOrbit, IReadOnlyList<Burn> otherBurns, double burnTimeSeconds,
        double3 currentDvVlf, FrameSpec frame, double nowSeconds, out Vector3d components)
    {
        components = default;
        try
        {
            if (!vessels.TryCaptureRailsAuthority(
                    vehicle, out var authority, out var authorityReason))
                return AuthorityFailure(authorityReason);
            if (TryPredictedBasis(vessels, authority, vehicle, currentOrbit, otherBurns,
                    burnTimeSeconds,
                    frame, nowSeconds, out var pose, out var rRel, out var vRel,
                    out var rFrame, out var vFrame) is { } reason)
                return reason;
            if (BurnFrameKernel.VlfToEcl(FrameAdapter.ToCore(currentDvVlf), rRel, vRel) is not { } dvEcl)
                return "degenerate VLF frame at burn time (radial trajectory)";
            if (BurnFrameKernel.FrameToFrenet(BurnFrameKernel.EclToFrame(dvEcl, pose), rFrame, vFrame)
                is not { } frenet)
                return "degenerate trajectory in this frame at burn time (vessel nearly stationary or radial in the frame)";
            if (!vessels.ValidateRailsAuthority(authority, out authorityReason))
                return AuthorityFailure(authorityReason);
            components = frenet;
            return null;
        }
        catch (Exception e)
        {
            return $"conversion failed: {e.Message}";
        }
    }

    /// <summary>Whole-plan conversion pass (throttled by the panel: plan-signature
    /// change or cache age). One display predictor, burns folded in time order with
    /// their CURRENT stock dv — IMPULSIVELY, because the VLF dv this pass authors is
    /// defined against stock's impulsive plan chain (the executor's own target
    /// derivation). The overlay's DISPLAY fold shares the pipeline but may add the
    /// finite-burn discretization on top (FoldBurns' optional finite model): near a
    /// long burn the drawn line's chained state differs from this pass's by the
    /// finite-arc displacement — by design, not a drift bug. Results in the input
    /// burns' order. Null when the vessel has no usable orbit parent or the pass
    /// failed wholesale.</summary>
    internal static IReadOnlyList<BurnAnalysis>? Analyze(VesselRegistry vessels, Vehicle vehicle,
        Orbit currentOrbit, IReadOnlyList<Burn> burns, FlightPlanModel plan, double nowSeconds)
    {
        try
        {
            if (!vessels.TryCaptureRailsAuthority(
                    vehicle, out var authority, out _))
                return null;
            var tracked = authority.Tracked;
            if (currentOrbit.Parent is not Astronomical fallbackParent) return null;
            var results = new BurnAnalysis?[burns.Count];
            var order = Enumerable.Range(0, burns.Count)
                .OrderBy(i => burns[i].Time.Seconds()).ToArray();
            double railsHorizon = tracked.Rails.Horizon;
            // Pre-pass: past/imminent burns (their effect is inside the predictor's
            // seed; stock's to execute) and beyond-rails burns (the same defensive
            // refusal RejectBeyondRails makes for authoring — evaluating there would
            // synchronously extend the SHARED rails inside a UI-thread Gate hold)
            // resolve here and stay out of the fold; the rest convert below.
            var foldIndices = new List<int>(order.Length);
            foreach (int i in order)
            {
                double t = burns[i].Time.Seconds();
                var dvVlf = FrameAdapter.ToCore(burns[i].DeltaVVlf);
                var meta = plan.TryGetMetaAt(t);
                if (!double.IsFinite(t))
                    // Hostile stock plan: a NaN time would fall through every window
                    // comparison below into the fold, where FoldBurns would skip it and
                    // leave a null results slot — resolve it here instead.
                    results[i] = new BurnAnalysis(t, dvVlf, meta, null, null,
                        "burn time is not finite");
                else if (t <= nowSeconds + PlannerKernel.MinLeadSeconds)
                    results[i] = new BurnAnalysis(t, dvVlf, meta, null, null,
                        meta is null ? null : "past/imminent burn: shown in VLF");
                else if (t > railsHorizon)
                    results[i] = new BurnAnalysis(t, dvVlf, meta, null, null,
                        "beyond the rails horizon - pick a longer orbits window in N-Body Frames"
                            + " (a just-raised window keeps growing in the background)");
                else
                    foldIndices.Add(i);
            }
            if (!tracked.TryNewDisplayPredictor(
                    authority.Lineage, nowSeconds, out var display, out _))
                return null;
            var times = new double[foldIndices.Count];
            for (int k = 0; k < foldIndices.Count; k++)
                times[k] = burns[foldIndices[k]].Time.Seconds();
            // One fold pipeline with the overlay (OverlayKernel.FoldBurns): the callback
            // runs per burn in ascending time with all earlier burns already folded —
            // exactly the pre-burn state — so the whole per-burn conversion picture is
            // computed inside it; its return value is the impulse to fold. The window
            // was applied by the pre-pass (foldIndices all satisfy t > now + MinLead).
            OverlayKernel.FoldBurns(display, times, nowSeconds + PlannerKernel.MinLeadSeconds,
                double.PositiveInfinity,
                k =>
                {
                    int i = foldIndices[k];
                    double t = times[k];
                    var dvVlf = FrameAdapter.ToCore(burns[i].DeltaVVlf);
                    var meta = plan.TryGetMetaAt(t);
                    var (rRel, vRel) = RelState(tracked, display,
                        BurnParentId(vehicle, fallbackParent.Id, t), t);
                    var dvEclCurrent = BurnFrameKernel.VlfToEcl(dvVlf, rRel, vRel);
                    string? note = dvEclCurrent is null ? "degenerate VLF frame at burn time" : null;
                    Vector3d? displayComponents = null;
                    Vector3d? freshDvVlf = null;
                    bool executionRealized = false;
                    Vector3d? dvEclFold = null;
                    if (meta is not null && note is null)
                    {
                        if (FrameManager.SampleSpecPose(meta.Frame, t, out var pose) is { } poseReason)
                        {
                            note = poseReason;
                        }
                        else if (TryFrameState(tracked, display, meta.Frame, t, in pose,
                                     StencilFloor(times, t, nowSeconds),
                                     out var rFrame, out var vFrame) is { } stateReason)
                        {
                            note = stateReason;
                        }
                        else
                        {
                            // Frenet both ways: display the current physical dv as
                            // (prograde, radial, normal) of the frame-relative
                            // trajectory, and realize the authored components against
                            // the same basis (the pre-burn state — earlier burns are
                            // already folded into `display` by this callback's turn).
                            displayComponents = BurnFrameKernel.FrameToFrenet(
                                BurnFrameKernel.EclToFrame(dvEclCurrent!.Value, pose), rFrame, vFrame);
                            var dvFrame = BurnFrameKernel.FrenetToFrame(meta.Authored, rFrame, vFrame);
                            Vector3d? dvEclIntent = dvFrame is { } authoredFrame
                                ? BurnFrameKernel.FrameToEcl(authoredFrame, pose)
                                : null;
                            freshDvVlf = dvEclIntent is { } intent
                                ? BurnFrameKernel.EclToVlf(intent, rRel, vRel)
                                : null;
                            if (displayComponents is null || freshDvVlf is null)
                            {
                                note = "degenerate trajectory in the authoring frame at burn time";
                            }
                            else if (meta.ExecutionDvVlf is { } execution
                                && !BurnFrameKernel.IsStale(execution, dvVlf, StaleToleranceMps))
                            {
                                // Execution-basis components: dvEclCurrent read them in
                                // the predictor basis, rotated by the drift. Show and
                                // fold the authored intent instead (matching the
                                // overlay's DisplayDvVlf fold).
                                executionRealized = true;
                                displayComponents = meta.Authored;
                                dvEclFold = dvEclIntent;
                            }
                        }
                    }
                    results[i] = new BurnAnalysis(t, dvVlf, meta, displayComponents,
                        freshDvVlf, note, executionRealized);
                    return dvEclFold ?? dvEclCurrent;
                },
                message => FrameManager.NoteContained("planner conversion pass", message));
            if (!vessels.ValidateRailsAuthority(authority, out _)) return null;
            return results!;
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("planner conversion pass", e);
            return null;
        }
    }

    // ---- Shared predicted-basis pipeline: TryAuthorDvVlf and
    // TryCurrentComponentsInFrame are the same preamble with opposite conversion tails.

    /// <summary>The frame pose, predicted parent-relative state, AND frame-space state
    /// at the burn time — the shared preamble of both conversion directions:
    /// rails-window guard, frame pose gates (SampleSpecPose = the Activate gates),
    /// burn-time patch parent, one display predictor with every strictly-earlier burn
    /// folded in, then the Frenet inputs (<see cref="TryFrameState"/>). Null reason on
    /// success; the caller owns exception containment.</summary>
    private static string? TryPredictedBasis(
        VesselRegistry vessels,
        VesselRegistry.RailsAuthoritySnapshot authority,
        Vehicle vehicle,
        Orbit currentOrbit, IReadOnlyList<Burn> burns, double burnTimeSeconds, FrameSpec frame,
        double nowSeconds, out FramePose pose, out Vector3d rRel, out Vector3d vRel,
        out Vector3d rFrame, out Vector3d vFrame)
    {
        var tracked = authority.Tracked;
        pose = default;
        rRel = vRel = rFrame = vFrame = default;
        if (RejectBeyondRails(tracked.Rails, burnTimeSeconds) is { } railsReason)
            return railsReason;
        if (FrameManager.SampleSpecPose(frame, burnTimeSeconds, out pose) is { } poseReason)
            return poseReason;
        if (currentOrbit.Parent is not Astronomical fallbackParent)
            return "orbit parent is not a celestial body";
        if (!tracked.TryNewDisplayPredictor(
                authority.Lineage, nowSeconds, out var display, out _))
        {
            vessels.ValidateRailsAuthority(authority, out var authorityReason);
            return AuthorityFailure(authorityReason);
        }
        FoldEarlier(tracked, vehicle, fallbackParent.Id, display, burns, nowSeconds, burnTimeSeconds);
        (rRel, vRel) = RelState(tracked, display,
            BurnParentId(vehicle, fallbackParent.Id, burnTimeSeconds), burnTimeSeconds);
        double stencilFloor = StencilFloor(
            burns.Select(b => b.Time.Seconds()), burnTimeSeconds, nowSeconds);
        string? reason = TryFrameState(tracked, display, frame, burnTimeSeconds, in pose,
            stencilFloor, out rFrame, out vFrame);
        if (reason is not null) return reason;
        return vessels.ValidateRailsAuthority(authority, out var finalReason)
            ? null
            : AuthorityFailure(finalReason);
    }

    private static string AuthorityFailure(PredictorAuthorityPolicy.Reason reason) =>
        "authoritative predictor unavailable: " + PredictorAuthorityPolicy.Describe(reason);

    /// <summary>Tangent step for the numerical frame-space velocity: small against any
    /// orbital period the picker offers, large against pose/predictor round-off.</summary>
    private const double TangentDtSeconds = 1.0;

    /// <summary>Floor on the tangent stencil's span: below this the secant direction is
    /// noise-dominated and the conversion refuses instead of guessing.</summary>
    private const double MinTangentSpanSeconds = 0.2;

    /// <summary>Floor on the frame-relative SPEED (m/s) for a usable prograde
    /// direction: TryVlfBasis only rejects exact zero, but the numerical tangent of a
    /// vessel (nearly) stationary in the frame — synchronous orbit in a Surface frame,
    /// hovering near a libration point — is round-off noise (~1e-8 m/s) pointing in a
    /// random direction; authoring "prograde" against it would realize an arbitrary
    /// burn that re-randomizes every Analyze pass. 1 cm/s sits far above the noise and
    /// far below any deliberate frame-relative drift.</summary>
    private const double MinFrameSpeedMps = 0.01;

    /// <summary>The vessel's predicted FRAME-SPACE state at the burn time — the Frenet
    /// basis inputs (BurnFrameKernel.FrenetToFrame/FrameToFrenet). Position is the
    /// pose's ToFrame image of the predicted game-convention position (relative to the
    /// frame origin, i.e. the primary). Velocity is the NUMERICAL tangent of the frame
    /// coordinates: the analytic rotating-pulsating derivative needs the pose's angular
    /// rate and pulsation rate, which FramePose deliberately does not carry — and the
    /// numerical tangent IS the drawn line's prograde direction in the plotting frame.
    /// The stencil is CLAMPED into the pre-burn trajectory's
    /// valid open interval — after the predictor start ("now": a burn under 1 s ahead
    /// must not query before the seed) and the latest EARLIER burn already folded into
    /// <paramref name="display"/> (its impulse kink must not contaminate the secant),
    /// and before the rails horizon (the +1 s probe must not outrun RejectBeyondRails'
    /// guard) — degrading to a one-sided difference at the edges and refusing when the
    /// span collapses. Null reason on success. The stencil's pose samples run the same
    /// gate pipeline as the burn-time pose (SampleSpecPose), so a frame degenerating
    /// within a second of the burn refuses rather than mixing gated and ungated poses.</summary>
    private static string? TryFrameState(TrackedVessel tracked, TrajectoryPredictor display,
        FrameSpec frame, double burnTimeSeconds, in FramePose poseAtBurn,
        double stencilFloorSeconds, out Vector3d rFrame, out Vector3d vFrame)
    {
        rFrame = vFrame = default;
        var rails = tracked.Rails;
        double tMinus = Math.Max(burnTimeSeconds - TangentDtSeconds, stencilFloorSeconds + 0.05);
        double tPlus = Math.Min(burnTimeSeconds + TangentDtSeconds, rails.Horizon);
        if (!(tPlus - tMinus >= MinTangentSpanSeconds))
            return "burn too close to now, an earlier burn, or the rails horizon to derive "
                + "the trajectory direction in this frame";
        if (FrameManager.SampleSpecPose(frame, tMinus, out var poseMinus) is { } minusReason)
            return minusReason;
        if (FrameManager.SampleSpecPose(frame, tPlus, out var posePlus) is { } plusReason)
            return plusReason;

        Vector3d GamePosition(double t)
        {
            // Chunked Gate holds (RailsService.ChunkedStateAt): a far target must not
            // grind the extension under one acquisition.
            var absolute = rails.ChunkedStateAt(display, t);
            // Poses are game-convention (root pinned at the origin) — same conversion
            // as TrajectoryOverlay's frame sampling.
            return absolute.Position - rails.GetAbsolute(rails.RootId, t).Position;
        }

        rFrame = poseAtBurn.ToFrame(GamePosition(burnTimeSeconds));
        var coordinateMinus = poseMinus.ToFrame(GamePosition(tMinus));
        var coordinatePlus = posePlus.ToFrame(GamePosition(tPlus));
        vFrame = (coordinatePlus - coordinateMinus) / (tPlus - tMinus);
        if (vFrame.Length() < MinFrameSpeedMps)
            return "vessel is (nearly) stationary in this frame at the burn time - "
                + "prograde/radial/normal have no direction to author against";
        return null;
    }

    /// <summary>The tangent stencil's lower bound: the latest thing the pre-burn
    /// trajectory must not sample across — "now" (the display predictor's seed) or the
    /// latest strictly-earlier burn (its impulse is already folded into the display,
    /// and a secant across the kink blends pre- and post-burn motion).</summary>
    private static double StencilFloor(IEnumerable<double> earlierBurnTimes,
        double burnTimeSeconds, double nowSeconds)
    {
        double floor = nowSeconds;
        foreach (double t in earlierBurnTimes)
            if (t < burnTimeSeconds && t > floor) floor = t;
        return floor;
    }

    /// <summary>Defensive rails-window guard: evaluating rails or the
    /// display predictor beyond the integrated horizon would synchronously ExtendTo(t)
    /// the SHARED ephemerides inside a Gate hold on the UI thread (~13 ms/day at
    /// shipping scale — RailsService.WorkerLoop doc — so a far burn is a multi-second
    /// freeze for every rails reader). The plan/authoring rules own the primary bound
    /// (FlightPlanModel.RejectOutsideWindow / ValidateLength); this refusal is the
    /// converter's last line for burns that predate those rules (adopted stock burns, a
    /// rails horizon lowered after planning). Refusal beats a freeze; the overlay keeps
    /// its own FlightPlans.EffectiveHorizonDays clamp for the drawn line.</summary>
    private static string? RejectBeyondRails(RailsService rails, double burnTimeSeconds)
    {
        double horizon = rails.Horizon;
        return burnTimeSeconds <= horizon
            ? null
            : $"rejected: burn at t={burnTimeSeconds:F0} s is beyond the integrated rails "
              + $"horizon (t={horizon:F0} s) - pick a longer orbits window in N-Body Frames"
              + " (a just-raised window keeps growing in the background - retry shortly)";
    }

    /// <summary>Parent id for a burn's VLF basis: the parent of the stock PATCH covering
    /// the burn time (<see cref="BurnPlanWriter.ResolvePlanningPatch"/>) — stock
    /// EXECUTES DeltaVVlf in that patch's parent frame, so cross-SOI burns must convert
    /// there too (decompiled evidence on <see cref="PlannerKernel.BurnBasisParent"/>).
    /// Falls back to the panel-time orbit parent when no patch resolves or the game
    /// read faults. CAVEAT on the fallback: across an SOI transition that basis is the
    /// wrong one, but with no resolvable patch stock has no better answer either
    /// (execution itself needs a patch — the conic plan simply hasn't predicted that
    /// far).</summary>
    internal static string BurnParentId(Vehicle vehicle, string fallbackParentId, double burnTimeSeconds)
    {
        string? patchParentId = null;
        try
        {
            PatchedConic? patch = BurnPlanWriter.ResolvePlanningPatch(
                vehicle, new SimTime(burnTimeSeconds));
            if (patch?.Orbit.Parent is Astronomical patchParent) patchParentId = patchParent.Id;
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("burn patch parent resolution", e);
        }
        return PlannerKernel.BurnBasisParent(patchParentId, fallbackParentId);
    }

    /// <summary>Nullable basis parent for an EXISTING stock burn. Unlike the
    /// time-only resolver used for new/hypothetical nodes, TryGetBurnPatch starts from
    /// this burn and walks the preceding burn FlightPlans, matching stock execution.
    /// Null/fault is deliberately not replaced with a current-parent guess: callers
    /// preserve the snapshot's last known parent until the stock patch chain is ready.</summary>
    internal static bool ExistingBurnParentsReady(Vehicle vehicle) =>
        !vehicle.FlightComputer.BurnPlan.FlightPlansOutOfDate;

    internal static string? ExistingBurnParentId(Vehicle vehicle, Burn burn) =>
        ExistingBurnParentId(vehicle, burn, ExistingBurnParentsReady(vehicle));

    internal static string? ExistingBurnParentId(Vehicle vehicle, Burn burn,
        bool patchChainReady)
    {
        if (!patchChainReady
            || vehicle.FlightComputer.BurnPlan.FlightPlansOutOfDate)
            return null;
        try
        {
            // Scoped so a node kept past an impact-terminated plan resolves here too.
            PatchedConic? patch;
            using (Patches.BurnPlanCalculationContext.EnterForVehicle(vehicle))
                patch = vehicle.FlightComputer.BurnPlan.TryGetBurnPatch(burn);
            return (patch?.Orbit.Parent as Astronomical)?.Id;
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("existing burn patch parent resolution", e);
            return null;
        }
    }

    /// <summary>Folds every burn strictly earlier than <paramref name="cutoffSeconds"/>
    /// (and past the minimum lead) into the display prediction, basis parents
    /// resolved here (main thread — <see cref="BurnParentId"/> walks game patches),
    /// through the ONE simple whole-burn fold (<see cref="FoldResolved"/>). The fold
    /// reads VLF in the predictor basis, so an execution-basis burn folds its
    /// snapshot-recorded display vector instead (the overlay fold's substitution),
    /// or later burns would author against a drift-rotated pre-burn trajectory.</summary>
    private static void FoldEarlier(TrackedVessel tracked, Vehicle vehicle, string fallbackParentId,
        TrajectoryPredictor display, IReadOnlyList<Burn> burns, double nowSeconds, double cutoffSeconds)
    {
        FlightPlanModel? plan = FlightPlans.TryGet(vehicle.Id);
        var earlier = burns
            .Select(b => (Time: b.Time.Seconds(), DvVlf: FrameAdapter.ToCore(b.DeltaVVlf)))
            .Where(b => b.Time < cutoffSeconds)
            .OrderBy(b => b.Time)
            .Select(b => (b.Time,
                plan?.SnapshotDisplayDvFor(b.Time, b.DvVlf) ?? b.DvVlf,
                BurnParentId(vehicle, fallbackParentId, b.Time)))
            .ToArray();
        FoldResolved(tracked, display, earlier, nowSeconds, cutoffSeconds, "planner burn fold");
    }

    /// <summary>The ONE simple whole-burn fold, shared by the authoring preambles
    /// (<see cref="FoldEarlier"/>) and the periapsis solver: pre-resolved burns
    /// (time, VLF dv, basis parent) fold into the display prediction through the
    /// overlay's own pipeline — OverlayKernel.FoldBurns' window rule, degenerate-VLF
    /// warning and duplicate/pre-start containment. IMPULSIVE by design: authored
    /// VLF dv is defined against stock's impulsive chain; the overlay's display fold
    /// may discretize the same burns into finite arcs on top, so near a long burn
    /// the drawn state differs by the finite-arc displacement. Each burn's dv
    /// converts VLF-&gt;ecl in ITS OWN burn-time basis against the trajectory with
    /// all earlier burns already folded. Thread-agnostic under the rails Gate
    /// discipline (<see cref="RelState"/> locks per call). <paramref name="finite"/>
    /// discretizes each burn into the FC's centered thrust arc (the display fold's
    /// model) — the authoring preambles pass null (impulsive: the VLF dv they write
    /// is defined against stock's impulsive chain); the periapsis solver passes the
    /// display's model so its objective predicts the trajectory the FC will fly.</summary>
    internal static void FoldResolved(TrackedVessel tracked, TrajectoryPredictor display,
        IReadOnlyList<(double Time, Vector3d DvVlf, string BasisParentId)> burns,
        double nowSeconds, double cutoffSeconds, string containmentContext,
        FiniteBurnFold? finite = null)
    {
        var times = new double[burns.Count];
        for (int k = 0; k < burns.Count; k++) times[k] = burns[k].Time;
        OverlayKernel.FoldBurns(display, times, nowSeconds + PlannerKernel.MinLeadSeconds,
            cutoffSeconds,
            k =>
            {
                var (rRel, vRel) = RelState(tracked, display, burns[k].BasisParentId, burns[k].Time);
                return BurnFrameKernel.VlfToEcl(burns[k].DvVlf, rRel, vRel);
            },
            message => FrameManager.NoteContained(containmentContext, message),
            finite);
    }

    /// <summary>Predicted state at t relative to <paramref name="parentId"/>, ecliptic
    /// axes — the GetVlf2ParentCci input geometry in mod coordinates (axis-independent
    /// projections; see the kernel header). Internal for PeriapsisSolver's
    /// background-thread objective (same Gate discipline).</summary>
    internal static (Vector3d RRel, Vector3d VRel) RelState(TrackedVessel tracked,
        TrajectoryPredictor display, string parentId, double t)
    {
        var rails = tracked.Rails;
        // Chunked Gate holds (RailsService.ChunkedStateAt): a far target must not
        // grind the extension under one acquisition.
        var absolute = rails.ChunkedStateAt(display, t);
        var parentAbs = rails.GetAbsolute(parentId, t);
        return (absolute.Position - parentAbs.Position, absolute.Velocity - parentAbs.Velocity);
    }
}

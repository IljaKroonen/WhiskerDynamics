using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Seam 1 (single on-rails vessel per task tick): after the stock freefall
/// branch staged its Kepler state, re-stage the same surfaces with the predictor state.
/// Gates: the stock branch must actually have staged an analytic Freefall state this
/// tick (otherwise stock chose surface/live handling and Seam 2 owns the vessel).
/// Patching this large, non-inline-marked caller (instead of the AggressiveInlining
/// evaluators it wraps) is the inlining-risk mitigation; the commit canary proves
/// the effect end-to-end.</summary>
[HarmonyPatch(typeof(VehicleUpdateTask), "ApplySingleVehicleMotion")]
internal static class VesselRailsPatch
{
    // One-shot in-game evidence that THIS path staged an override (whiskerdynamics.log is the
    // only observable while the game runs). Re-armed per bound sim by the
    // statics sweep so every session/load evidences the path.
    private static int _pathLogged;

    /// <summary>Statics sweep: re-arm the one-shot path-evidence line.</summary>
    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _pathLogged, 0);

    // ApplySingleVehicleMotion is the deliberately non-inlined owner of the stock
    // active-plan build. Keep authority scoped across both the original method and
    // this postfix; SetCurrentOrbit itself is AggressiveInlining and is not a safe
    // Harmony seam.
    static bool Prefix(VehicleUpdateState vehicleState,
        out SoiPlanAuthorityContext.Scope __state)
    {
        __state = SoiPlanAuthorityContext.Begin(vehicleState);
        return __state.RunOriginal;
    }

    static void Postfix(VehicleUpdateTask __instance, VehicleUpdateState vehicleState)
    {
        if (!ModServices.Enabled)
            return;
        ModServices.BoundServices services = default;
        bool bindingCaptured = false;
        try
        {
            if (!ModServices.EnsureBound(out services)) return;
            bindingCaptured = true;
            var vessels = services.Vessels;
            if (__instance.SimStep.DeltaTime <= 0.0) return;
            var newStates = vehicleState.GetNewStates();
            var situation = newStates.Props.Situation;
            bool stagedAnalytic = vehicleState.UpdateData.NewStateVectors.HasValue
                || vehicleState.UpdateData.NewFlightPlan != null;
            if (situation != Situation.Freefall || !stagedAnalytic)
            {
                vessels.NoteLiveOwnership(vehicleState, "live/off-rails ownership");
                // Burn-time live display: stock physics owned this tick
                // (Maneuvering = burning, or a Freefall physics-bubble tick) — the
                // committed CurrentStateVectors are live (repopulated from kinematics
                // at the end of ApplyFullPhysics), so the map stays honest through
                // the burn: continuously rebuilt coast-from-here prediction, with the
                // plan-snapshot ghost refreshed alongside. Surface situations have
                // no orbit line to keep alive.
                if (situation is Situation.Freefall or Situation.Maneuvering)
                    vessels.OffRailsOverlay(vehicleState);
                return;
            }

            var tracked = vessels.GetOrSeed(vehicleState);

            // Fault-injection drill (config-gated, default off — ""): a synthetic
            // throw on exactly the path a genuine integrator divergence/NaN takes, so the
            // catch below globally faults and pauses dynamics.
            if (ModServices.Config.FaultInjectVessel == vehicleState.Id)
                throw new InvalidOperationException("fault injection (config: fault_inject_vessel)");

            // Within-tick live excursion: a warp-scaled tick can take the
            // vessel live (flight-computer burn, proximity physics) AND return it to
            // Freefall within the SAME tick — pre-tick committed and staged situations
            // both read Freefall, so Decide() keeps the predictor, and staging it here
            // would overwrite the live result: the burn's delta-v silently discarded
            // with an exact-0 canary. The tick's stock-accumulated DeltaVelocityCci
            // witnesses such an excursion (it integrates thrust/drag forces only —
            // gravity flows through AccelPhys — so pure on-rails ticks stay exactly
            // zero). Keep the stock live result this tick; reseed from it next tick.
            if (vehicleState.UpdateData.NewKinematicMeasurements is { } measured
                && measured.DeltaVelocityCci != Brutal.Numerics.double3.Zero)
            {
                vessels.NoteLiveImpulse(vehicleState, tracked, measured.DeltaVelocityCci.Length());
                // The staged stock live result IS the post-impulse state: give the
                // map instant feedback instead of waiting out the reseed tick.
                vessels.OffRailsOverlay(vehicleState);
                return;
            }

            var currentOrbit = vehicleState.CurrentOrbit;
            SimTime nextTime = __instance.SimStep.NextTime;
            // Evaluates the predictor, books the conic-drift readout (n-body departure
            // from the stock conic since its last re-derivation — the osculating-refresh
            // trigger below), and applies the teleport-jump reseed guard.
            StateVectors sv = vessels.EvaluateForStaging(vehicleState, tracked, nextTime);

            // Rails-geometric SOI re-parent (see SoiReparentKernel): stock's
            // on-rails transition fires only through the flight plan's patch schedule,
            // and those conic-vs-Kepler encounter predictions regularly miss the
            // n-body trajectory's real encounters — without this, a captured vessel
            // rides its stale parent to the child's surface and pays the live
            // handoff there. An exact re-expression of the same predictor state:
            // no snap, and the map, planner, terrain/physics-radius bookkeeping and
            // the eventual live handoff all see the geometrically-true parent.
            var config = ModServices.Config;
            double t = nextTime.Seconds();
            Astronomical? soiParent = vessels.RailsSoiParent(tracked, currentOrbit, nextTime);
            FlightPlan stagedPlan = vehicleState.UpdateData.NewFlightPlan
                ?? vehicleState.ReadOnlyVehicle.FlightPlan;
            bool retireStockSoiPlan =
                SoiEncounterPlanAuthorityPatch.HasParentTransition(stagedPlan);

            // FlightPlan coherence: rebuild the stock conic
            // from the n-body state when it drifts too far, or periodically regardless,
            // so patch boundaries / encounter times / plan expiry are computed near the
            // truth (the freefall branch reads Patch.EndTime and FlightPlan.ExpiryGameTime
            // from this plan every tick). SetCurrentOrbit re-runs the stock
            // patch-transition computation from the fresh osculating conic. A re-parent
            // is a refresh with the new parent's frame seeding the fresh conic.
            if (soiParent is not null || retireStockSoiPlan || VesselLifecycle.ShouldRefreshOsculation(
                    tracked.LastConicDrift, config.ConicDriftMeters,
                    t, tracked.LastRefreshTime, config.OsculationRefreshSeconds))
            {
                IParentBody orbitParent = soiParent is not null ? (IParentBody)soiParent : currentOrbit.Parent;
                StateVectors seed = soiParent is not null
                    ? tracked.EvaluateGameStateAgainst(soiParent, nextTime, currentOrbit.StateVectors.TrueAnomaly)
                    : sv;
                var freshOrbit = Orbit.CreateFromStateCci(
                    orbitParent, nextTime, seed.PositionCci, seed.VelocityCci,
                    currentOrbit.OrbitLineColor);
                vehicleState.SetCurrentOrbit(freshOrbit, vehicleState.ReadOnlyVehicle.Hash);
                if (soiParent is not null)
                {
                    // Mirror stock's own rails patch transition (VehicleUpdateTask.cs:856):
                    // the physics environment keys terrain, physics radius and situation
                    // handling off ClosestParent.
                    newStates.SetClosestParent(orbitParent);
                    if (FlightPlans.TryGet(vehicleState.Id) is not null)
                        BasisReconversionUrgency.Raise(vehicleState.Id);
                    // GetOrSeed's parent-transition observation logs the landing next
                    // tick; this line records who initiated it. Its own 1 s budget —
                    // sharing LastTransitionLogMs would suppress that landing line.
                    long nowMs = Environment.TickCount64;
                    if (nowMs - tracked.LastReparentLogMs >= 1000)
                    {
                        tracked.LastReparentLogMs = nowMs;
                        ModLog.Info($"vessel '{vehicleState.Id}': rails-geometric SOI re-parent "
                            + $"'{(currentOrbit.Parent as Astronomical)?.Id}' -> '{soiParent.Id}' at t={t:F1} s "
                            + "(stock plan transition had not fired)");
                    }
                }
                currentOrbit = vehicleState.CurrentOrbit; // the fresh conic (plan's first patch)
                sv = tracked.EvaluateGameState(currentOrbit, nextTime); // TA now fresh
                vessels.NoteOsculationRefresh(tracked, t);
            }

            // Re-stage exactly what the stock freefall path staged, with our state:
            if (vehicleState.UpdateData.NewFlightPlan != null)
            {
                vehicleState.UpdateData.NewFlightPlan.FirstPatch.Orbit.UpdatePosition(sv);
                vehicleState.UpdateData.NewStateVectors = null;
            }
            else
            {
                vehicleState.UpdateData.NewStateVectors = sv;
            }
            __instance.OriginOrbit = currentOrbit;
            __instance.Origin = BubbleOrigin.CreateFrom(currentOrbit.Parent, in sv);
            newStates.Origin = __instance.Origin;
            newStates.UpdateFromAnalytic(currentOrbit, in sv,
                vehicleState.CurrentBody2Cce, vehicleState.CurrentBodyRates, Situation.Freefall);
            if (System.Threading.Interlocked.CompareExchange(ref _pathLogged, 1, 0) == 0)
                ModLog.Info($"seam1 single-vessel path active (first override: '{vehicleState.Id}' at t={nextTime.Seconds():F1} s)");

            // Honest orbit lines (every tracked vessel): the map orbit line
            // becomes the sampled n-body polyline — the ACTUAL no-burn trajectory,
            // plus a separate PLANNED line when in-window burns exist (two-line
            // display; display only, ~1 Hz per vessel; contained
            // internally - a cosmetic failure never books a vessel containment).
            TrajectoryOverlay.MaybeRebuild(vehicleState, tracked, nextTime);
        }
        catch (Exception e)
        {
            if (bindingCaptured)
                ModServices.RunIfBindingCurrent(services,
                    () => ModServices.FatalDisable(
                        $"vessel rails failed for '{vehicleState.Id}' (seam1-single): {e}"));
        }
    }

    static Exception? Finalizer(
        Exception? __exception, SoiPlanAuthorityContext.Scope __state)
    {
        SoiPlanAuthorityContext.End(__state);
        return __exception;
    }
}

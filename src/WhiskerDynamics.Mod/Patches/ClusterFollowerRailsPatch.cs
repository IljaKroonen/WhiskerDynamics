using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Seam 1 (on-rails followers inside a live physics cluster, per substep):
/// FullPhysicsUnconstrainedStep applies Kepler freefall motion to cluster members that
/// stay on rails. Re-stage those from their predictors. The substep time is read from
/// each follower's staged Origin.Time (set by the stock path to the substep next-time),
/// so the ref-struct PhysicsContext parameter is never touched. The shared cluster
/// origin staged by stock is deliberately left alone.</summary>
[HarmonyPatch(typeof(VehicleUpdateTask), "FullPhysicsUnconstrainedStep")]
internal static class ClusterFollowerRailsPatch
{
    // One-shot in-game evidence that THIS path staged an override (whiskerdynamics.log is the
    // only observable while the game runs). Re-armed per bound sim by the
    // statics sweep so every session/load evidences the path.
    private static int _pathLogged;

    /// <summary>Statics sweep: re-arm the one-shot path-evidence line.</summary>
    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _pathLogged, 0);

    static void Postfix(List<VehicleUpdateState> ____vehicleStates)
    {
        if (!ModServices.Enabled)
            return;
        if (!ModServices.EnsureBound(out var services)) return;
        var vessels = services.Vessels;
        foreach (var vehicleState in ____vehicleStates)
        {
            try
            {
                var newStates = vehicleState.GetNewStates();
                var situation = newStates.Props.Situation;
                bool stagedAnalytic = vehicleState.UpdateData.NewStateVectors.HasValue
                    || vehicleState.UpdateData.NewFlightPlan != null;
                if (situation != Situation.Freefall || !stagedAnalytic)
                {
                    vessels.NoteLiveOwnership(
                        vehicleState, "live/off-rails cluster ownership");
                    // Burn-time live display: the cluster's LIVE members
                    // (Seam 2 owns their motion) still get honest map lines — same
                    // coast-from-here rebuild as the single-vessel path, continuously
                    // supplying its worker (this postfix runs per substep). Single-
                    // vessel tasks are skipped here: ApplySingleVehicleMotion's own
                    // postfix runs at tick END, after stock repopulated the committed
                    // state from kinematics — fresher than any substep read, and its
                    // rebuild would lose the throttle race to this one.
                    if (____vehicleStates.Count > 1
                        && situation is Situation.Freefall or Situation.Maneuvering)
                        vessels.OffRailsOverlay(vehicleState);
                    continue;
                }

                var tracked = vessels.GetOrSeed(vehicleState);

                // Within-tick live excursion, cluster flavor: a member that
                // went live earlier in THIS tick (burn/collision substep) and is back on
                // Freefall now would have its live delta-v overwritten by the predictor
                // (pre-tick committed situation is still Freefall => Keep). The tick's
                // accumulated DeltaVelocityCci (thrust/drag forces only; exactly zero on
                // pure rails ticks) witnesses the excursion: keep the stock live result
                // for the rest of the tick and reseed from the committed state next tick.
                if (vehicleState.UpdateData.NewKinematicMeasurements is { } measured
                    && measured.DeltaVelocityCci != Brutal.Numerics.double3.Zero)
                {
                    vessels.NoteLiveImpulse(vehicleState, tracked, measured.DeltaVelocityCci.Length());
                    // The staged stock live result IS the post-impulse state: give the
                    // map instant feedback instead of waiting out the reseed tick.
                    vessels.OffRailsOverlay(vehicleState);
                    continue;
                }

                var currentOrbit = vehicleState.CurrentOrbit;
                SimTime t = newStates.Origin.Time;
                StateVectors sv = vessels.EvaluateForStaging(vehicleState, tracked, t);
                if (vehicleState.UpdateData.NewFlightPlan != null)
                {
                    vehicleState.UpdateData.NewFlightPlan.FirstPatch.Orbit.UpdatePosition(sv);
                    vehicleState.UpdateData.NewStateVectors = null;
                }
                else
                {
                    vehicleState.UpdateData.NewStateVectors = sv;
                }
                newStates.UpdateFromAnalytic(currentOrbit, in sv,
                    vehicleState.CurrentBody2Cce, vehicleState.CurrentBodyRates, Situation.Freefall);
                if (System.Threading.Interlocked.CompareExchange(ref _pathLogged, 1, 0) == 0)
                    ModLog.Info($"seam1 cluster-follower path active (first override: '{vehicleState.Id}' at t={t.Seconds():F1} s)");

                // Honest orbit lines: followers get polylines too (per-vessel 1 Hz throttle inside).
                TrajectoryOverlay.MaybeRebuild(vehicleState, tracked, t);
            }
            catch (Exception e)
            {
                ModServices.RunIfBindingCurrent(services,
                    () => ModServices.FatalDisable(
                        $"vessel rails failed for '{vehicleState.Id}' (seam1-cluster): {e}"));
            }
        }
    }
}

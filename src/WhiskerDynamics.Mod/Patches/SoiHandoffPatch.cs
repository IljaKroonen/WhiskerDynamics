using Brutal.Numerics;
using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>SOI handoff seam: prefix+postfix on the one funnel that mirrors a live
/// vessel's kinematic state into its analytic state (CurrentOrbit / committed
/// StateVectors) — decompiled VehicleUpdateTask.cs:1292, called from the pre-step loop
/// (:621), the end-of-tick mirror (:739) and immediately after each live SOI
/// transition (the CheckSoiTransitions call sites, :816 constrained and :884
/// unconstrained). When that mirror crosses parent frames (Origin.Parent !=
/// Environment.ClosestParent), stock converts through Orbit.CciToCci
/// (VehicleUpdateTask.cs:1301), whose parent-chain positions are ANALYTIC KEPLER
/// evaluations (Orbit.cs:1556-1563 ConvertUp -> Celestial.GetPositionCci(SimTime),
/// Celestial.cs:481/429) — but under Seam 3 every modeled body FLIES a numerical
/// rail, so the converted state is wrong by exactly the modeled parents'
/// Kepler-vs-rails divergence (megameters within days, growing secularly): a
/// world-space teleport the vessel registry would then adopt as truth on its
/// return-to-Freefall reseed. Redo the conversion with rails positions for both
/// parents and re-stage the same surface stock staged (SetCurrentOrbit with the
/// caller's own patch-transition flag; idempotent — SetFirstOrbit replaces the staged
/// plan's first orbit, VehicleUpdateState.cs:335-348).
///
/// Two gates keep the correction exactly on stock's own conversion:
/// - The prefix snapshots stock's freshness guard (populate runs only when
///   CurrentStateVectors.StateTime &lt; kinematic time, VehicleUpdateTask.cs:1295) —
///   a postfix-side inference from the post-state cannot distinguish "stock populated
///   this call" from "already fresh, skipped" and would clobber predictor-staged
///   overrides in multi-vehicle tasks.
/// - On-rails situations are skipped (SituationEx.cs:60: the situation's rails bit):
///   an on-rails member of a full-physics task has kinematics FORWARD-derived from its
///   analytic state through the same Kepler map (ApplyFreefallMotion staging,
///   VehicleUpdateTask.cs:845-871), so stock's Kepler mirror-back is its exact inverse
///   and a rails re-conversion would INJECT the divergence instead of removing it.
///   Only Bepu-integrated kinematics (rails bit clear) are independent truth worth
///   re-anchoring.
///
/// Every live-catalog parent has a modeled rails state. A non-astronomical parent,
/// missing modeled state, or mismatched staged parent is a global authority fault;
/// none may retain stock's cross-parent conversion. The attitude chain is left alone:
/// NewBody2Cce composes only constant per-body rotation quats
/// (VehicleUpdateTask.cs:1302/1306 via Orbit.cs:1559-1560 GetCci2ParentCci), which
/// carry no positional drift.</summary>
[HarmonyPatch(typeof(VehicleUpdateTask), "PopulateAnalyticStatesFromKinematicStates")]
internal static class SoiHandoffPatch
{
    // One-shot + 30 s-throttled correction lines: whiskerdynamics.log is the only observable
    // while the game runs. The one-shot is re-armed per bound sim by the statics
    // sweep; the wall-clock throttle needs no reset.
    private static int _pathLogged;
    private static long _nextLogMs;

    /// <summary>Statics sweep: re-arm the one-shot path-evidence line.</summary>
    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _pathLogged, 0);

    /// <summary>Snapshot of stock's own populate guard (VehicleUpdateTask.cs:1295),
    /// taken before the body runs. Never throws: a prefix exception would propagate
    /// into the stock method.</summary>
    static bool Prefix(VehicleUpdateState vehicleState, out bool __state)
    {
        __state = false;
        if (!ModServices.Enabled) return true;
        try
        {
            var states = vehicleState.GetReadOnlyStates();
            // SimTime wraps a seconds double (SimTime.cs:6-8) — Seconds() comparison
            // is exactly stock's operator comparison.
            __state = vehicleState.CurrentStateVectors.StateTime.Seconds() < states.Time.Seconds();
            return true;
        }
        catch (Exception e)
        {
            ModServices.FatalDisable(
                $"SOI handoff precondition inspection failed: {e}");
            return SoiHandoffInvariantPolicy.OriginalMayRun(
                enabled: true, inspectionSucceeded: false);
        }
    }

    static void Postfix(VehicleUpdateState vehicleState, bool computeFirstPatchTransition, bool __state)
    {
        if (!__state || !ModServices.Enabled) return;
        ModServices.BoundServices services = default;
        bool bindingCaptured = false;
        try
        {
            if (!ModServices.EnsureBound(out services)) return;
            bindingCaptured = true;
            var rails = services.Rails;
            var states = vehicleState.GetReadOnlyStates();
            // On-rails kinematics are analytic-derived — stock's Kepler mirror-back is
            // self-consistent there; see the class doc. Only Bepu-integrated states
            // (rails bit clear) get re-anchored.
            if (states.Props.Situation.IsOnRails()) return;
            IParentBody closest = states.Environment.ClosestParent;
            IParentBody originParent = states.Origin.Parent;
            // Same-parent mirror: no cross-frame conversion happened, nothing to fix.
            if (ReferenceEquals(originParent, closest)) return;
            bool astronomicalParents =
                closest is Astronomical && originParent is Astronomical;
            // Unknown on either side: no rails state exists to anchor the conversion.
            bool modeledParents = astronomicalParents
                && rails.CanEvaluate(((Astronomical)closest).Id)
                && rails.CanEvaluate(((Astronomical)originParent).Id);
            // Sanity: stock's populate landed the orbit on the closest parent.
            var invariant = SoiHandoffInvariantPolicy.ClassifyCrossParent(
                astronomicalParents, modeledParents,
                ReferenceEquals(vehicleState.CurrentOrbit.Parent, closest));
            if (invariant != SoiHandoffInvariantPolicy.Failure.None)
                throw new InvalidOperationException(
                    $"SOI handoff invariant failed: {invariant}");
            var newParent = (Astronomical)closest;
            var oldParent = (Astronomical)originParent;

            double t = states.Time.Seconds();
            states.GetStatesCci(out double3 posCci, out double3 velCci, out _);
            var (oldAbs, newAbs) = rails.GetAbsolutePair(oldParent.Id, newParent.Id, t);
            var cci2CceOld = originParent.GetCci2Cce();
            Vector3d posEcl = FrameAdapter.GameToAbsolute(oldAbs.Position, posCci, cci2CceOld);
            Vector3d velEcl = FrameAdapter.GameToAbsolute(oldAbs.Velocity, velCci, cci2CceOld);
            var cce2CciNew = closest.GetCce2Cci();
            double3 posNew = FrameAdapter.AbsoluteToGame(posEcl, newAbs.Position, cce2CciNew);
            double3 velNew = FrameAdapter.AbsoluteToGame(velEcl, newAbs.Velocity, cce2CciNew);

            // The staged (Kepler-converted) state is still readable — measure the
            // correction before overwriting it.
            double correction = (posNew - vehicleState.CurrentStateVectors.PositionCci).Length();
            var orbit = Orbit.CreateFromStateCci(closest, states.Time, posNew, velNew,
                vehicleState.CurrentOrbit.OrbitLineColor);
            vehicleState.SetCurrentOrbit(orbit, vehicleState.ReadOnlyVehicle.Hash, computeFirstPatchTransition);

            if (System.Threading.Interlocked.CompareExchange(ref _pathLogged, 1, 0) == 0)
            {
                ModLog.Info($"soi handoff re-anchored to rails: '{vehicleState.Id}' mirrored "
                    + $"'{oldParent.Id}' -> '{newParent.Id}' at t={t:F1} s, "
                    + $"stock Kepler conversion was off by {correction:E2} m");
            }
            else
            {
                long now = Environment.TickCount64;
                long next = System.Threading.Interlocked.Read(ref _nextLogMs);
                if (now >= next
                    && System.Threading.Interlocked.CompareExchange(ref _nextLogMs, now + 30_000, next) == next)
                    ModLog.Info($"soi handoff: '{vehicleState.Id}' '{oldParent.Id}' -> '{newParent.Id}' "
                        + $"re-anchored, Kepler conversion off by {correction:E2} m (t={t:F1} s)");
            }
        }
        catch (Exception e)
        {
            if (bindingCaptured)
                ModServices.RunIfBindingCurrent(services,
                    () => ModServices.FatalDisable(
                        $"SOI handoff failed for '{vehicleState.Id}': {e}"));
            else if (ModServices.Enabled)
                ModServices.FatalDisable(
                    $"SOI handoff failed before binding capture: {e}");
        }
    }
}

/// <summary>KSA-free decisions for the handoff prefix and cross-parent invariants.</summary>
internal static class SoiHandoffInvariantPolicy
{
    internal enum Failure
    {
        None,
        NonAstronomicalParent,
        UnmodeledParent,
        CurrentOrbitParentMismatch,
    }

    internal static bool OriginalMayRun(bool enabled, bool inspectionSucceeded) =>
        !enabled || inspectionSucceeded;

    internal static Failure ClassifyCrossParent(
        bool astronomicalParents, bool modeledParents, bool currentOrbitOnClosest)
    {
        if (!astronomicalParents) return Failure.NonAstronomicalParent;
        if (!modeledParents) return Failure.UnmodeledParent;
        if (!currentOrbitOnClosest) return Failure.CurrentOrbitParentMismatch;
        return Failure.None;
    }
}

using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Conic marker suppression: stock draws per-patch UI markers — Ap/Pe, velocity
/// arrows, degree increments, and Escape/Encounter transition markers — from the
/// conic patches via PatchedConic.DrawUi (PatchedConic.cs:1120), a path the line
/// takeover never touched. Under n-body those conic predictions drift off the honest
/// lines (the floating "encounter markers"). The audited caller census is
/// three-way, so this prefix routes by patch
/// INSTANCE exactly like <see cref="VesselLinePatch"/> routes plans — while the
/// uiContext vehicle has a FRESH mod line (same freshness gate):
/// (1) a patch of the vehicle's OWN plan (Vehicle.OnDrawUi -&gt; FlightPlan.DrawUi,
/// Vehicle.cs:3477) — suppressed WHOLESALE: every conic-derived
/// marker (Ap/Pe, velocity arrows, degree increments, Escape/Encounter transitions)
/// is replaced by Ui.LineMarkers' honest markers computed from the sampled batches
/// (Ap/Pe per frame-relevant body + AN/DN on both the actual
/// and planned lines, riding the re-embedded curve in frame views);
/// (2) a patch of one of the vessel's planned burns' plans (BurnPlan.DrawUi,
/// BurnPlan.cs:503, same vehicle uiContext) — suppressed wholesale, including its
/// patch-0/Final case: those plans' LINES are already suppressed by VesselLinePatch
/// ("post-burn predictions the polyline already folds in"), so any marker here would
/// float over no line. Burn node GIZMOS (Burn.Update -&gt; UpdateGizmos) are a
/// different path and stay;
/// (3) any other patch — the TransferPlanner preview plan and its Lambert patch
/// (TransferPlanner.cs:991/1000; Source is a Vehicle, :157) and any future caller —
/// original runs, stock draws: the planner's lines are stock's too (by design the
/// stock planning tool stays untouched), and markerless stock lines would break it.
/// Suppression clears the patch's public HoveredMarker field (stock's own first
/// field mutation, PatchedConic.cs:1124 — the draw-list fetch precedes it
/// at :1123) and reports false via __result, so a suppressed patch reports no
/// hover. Stock fallback (stale/untracked/disabled) draws everything, unchanged.</summary>
[HarmonyPatch(typeof(PatchedConic), "DrawUi")]
internal static class PatchMarkerPatch
{
    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static bool Prefix(PatchedConic __instance, Astronomical.UiContext uiContext, int index, ref bool __result)
    {
        if (!ModServices.Enabled) return true;
        try
        {
            if (uiContext.Astronomical is not Vehicle vehicle) return true;
            // Ownership, not freshness (matching VesselLinePatch):
            // markers stay suppressed through stale windows too — otherwise they'd
            // flash back with the stock conics during burns/physics bubbles. Only a
            // vessel with no batch at all is fully stock's.
            var samples = OverlayBuffer.Read(vehicle.Id);
            if (samples is null) return true;

            // Instance routing (see class doc): identity of the patch at this index
            // in the candidate plan — FlightPlan.DrawUi passes Patches[num] as num
            // (FlightPlan.cs:716), so the (plan, index) pair pins the instance.
            bool ownPlanPatch = IsPatchOfPlanAt(__instance, index, vehicle.FlightPlan);
            if (!ownPlanPatch && !BurnPlanScan.ContainsPatchAt(vehicle, __instance, index))
                return true; // TransferPlanner preview / future callers: stock draws

            // FRESH batches get no patch-0/Final/inertial allowance: LineMarkers
            // draws honest Ap/Pe + AN/DN from the sampled batches, so stock's
            // conic markers would double-mark from a less truthful source. STALE
            // windows (mid-burn, off-rails) keep the allowance for the
            // inertial view only (no counter-pose — includes body-centred inertial
            // display frames): LineMarkers requires a fresh batch, and the player
            // watching an apsis change DURING a burn is exactly when the readout
            // matters — stock's markers there sit on DrawStalePatch0's stock-style
            // line, the same pairing stock always had.
            long nowMs = Environment.TickCount64;
            double nowSimSeconds = Universe.GetElapsedSimTime().Seconds();
            bool lineUsable = OverlayBuffer.LineSamplesUsable(
                vehicle.Id, samples, planned: false, nowMs, nowSimSeconds);
            if (ownPlanPatch && index == 0 && !lineUsable && FrameManager.InertialView
                && __instance.EndTransition == PatchTransition.Final)
                return true;

            __instance.HoveredMarker = false; // stock's first field mutation (PatchedConic.cs:1124)
            __result = false; // suppressed patch hovers nothing
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"patch marker suppression active (first: '{vehicle.Id}' patch {index})");
            return false;
        }
        catch (Exception e)
        {
            TrajectoryOverlay.NoteRestageContained(e);
            return true;
        }
    }

    /// <summary>True when <paramref name="patch"/> is <paramref name="plan"/>'s patch at
    /// <paramref name="index"/> (identity, not equality — plans rebuild their patch lists,
    /// but callers pass the live list's element together with its index). Also the
    /// per-plan predicate behind <see cref="BurnPlanScan.ContainsPatchAt"/>.</summary>
    internal static bool IsPatchOfPlanAt(PatchedConic patch, int index, FlightPlan? plan)
    {
        if (plan is null) return false;
        var patches = plan.Patches;
        return (uint)index < (uint)patches.Count && ReferenceEquals(patches[index], patch);
    }
}

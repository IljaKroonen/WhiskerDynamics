using Brutal.Numerics;
using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Burn-node markers on the honest lines:
/// stock's draggable burn gizmos (the yellow sphere + delta-v cones, Burn.Update →
/// UpdateGizmos, Burn.cs:181-188/394-409) place themselves at Burn.PositionCce — a
/// direct CONIC evaluation of the pre-burn patch at the burn's time (Burn.cs:115) —
/// so under n-body they float off the drawn polyline (or hang in empty space along a
/// suppressed conic) exactly like the conic patch markers PatchMarkerPatch
/// suppresses. This prefix on the
/// PositionCce getter redirects the property to the DRAWN line's position at the
/// burn's time (TrajectoryOverlay.TryDrawnPositionAt — planned batch first, since a
/// burn node belongs on the planned trajectory when one is shown; positions are
/// impulse-continuous, so the planned path at burn k's time IS the pre-burn-k
/// position with earlier burns applied), converted into the patch parent's Cce frame
/// (stock adds Patch.Orbit.Parent's position back at every consumer: gizmo placement
/// Burn.cs:397, infobox anchor :529, drag comparison :999 — all three move together,
/// which is the point). Stock fallback (conic position, unchanged) whenever the mod
/// batches are absent/stale, the burn's time is outside their window (past burns,
/// beyond-horizon burns), or the frame mode is mid-blink — a marker must never be
/// invented for a line that is not drawn.</summary>
[HarmonyPatch(typeof(Burn), "PositionCce", MethodType.Getter)]
internal static class BurnNodePatch
{
    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static bool Prefix(Burn __instance, ref double3 __result)
    {
        if (!ModServices.Enabled) return true;
        try
        {
            // Parent-eject burns live on the parent CELESTIAL's orbit (Burn.Patch is
            // the eject patch), not the vessel's polyline — stock places them right.
            if (__instance.ParentEjectBurn) return true;
            if (!TryDrawnBurnPosition(__instance, out var world))
                return true; // out of window / stale / blink: stock conic position

            // Patch-parent Cce (ecliptic axes, no rotation): stock's consumers all add
            // Patch.Orbit.Parent's CURRENT position back (GetPositionEclFromCce), so
            // subtract exactly that.
            var parentEcl = FrameAdapter.ToCore(__instance.Patch.Orbit.Parent.GetPositionEcl());
            __result = FrameAdapter.ToGame(world - parentEcl);
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"burn node takeover active (first: '{__instance.Vehicle.Id}' burn at "
                    + $"t={__instance.Time.Seconds():F0} s pinned to the drawn n-body line)");
            return false;
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("burn node position", e);
            return true;
        }
    }

    /// <summary>The burn's position on the DRAWN lines: planned batch first (when a
    /// planned line shows, the node sits on it; the first burn IS its start point),
    /// then the actual batch (covers a first burn while the planned batch is one
    /// rebuild away). False when neither fresh batch covers the burn's time — the
    /// stock-conic fallback for the position getter, and BurnGizmoPatch's hide signal
    /// in frame views (where the stock conic position is wrong-frame geometry).</summary>
    internal static bool TryDrawnBurnPosition(Burn burn, out Vector3d world)
    {
        world = default;
        string vesselId = burn.Vehicle.Id;
        double t = burn.Time.Seconds();
        long nowMs = Environment.TickCount64;
        double nowSimSeconds = Universe.GetElapsedSimTime().Seconds();
        if (OverlayBuffer.ReadPlannedFresh(vesselId, nowMs, nowSimSeconds) is { } planned
            && TrajectoryOverlay.TryDrawnPositionAt(planned, t, out world))
            return true;
        return OverlayBuffer.ReadFresh(vesselId, nowMs, nowSimSeconds) is { } actual
            && TrajectoryOverlay.TryDrawnPositionAt(actual, t, out world);
    }
}

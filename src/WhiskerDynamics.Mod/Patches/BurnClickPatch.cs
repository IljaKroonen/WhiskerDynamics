using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Click-to-place burn suppression (by design: burns are authored through
/// the N-Body Planner panel ONLY). Stock's map channel for click-to-place —
/// the hover circle with the Orbit:/Vel: readout plus the left-click that creates a
/// burn — is fed exclusively through Orbit.GetNearestPosition
/// (CelestialSystem.SetNearestOrbitPoint's controlled-vehicle branch and
/// BurnPlan.GetNearestOrbitPoint populate the NearestOrbitPoint fields;
/// CelestialPosition.DrawUi draws the circle; Program.ProcessBurnClick consumes the
/// click — nothing else reads those fields). This prefix suppresses the whole channel
/// while the controlled vehicle's display is mod-OWNED (a batch was ever published —
/// the same ownership rule as the line/marker patches), covering every window the
/// per-patch hover routing could not: the stale-window stock-analytic fallback,
/// fresh lines, everything. Burn-node DRAGGING is untouched: the
/// gizmo drag calls Orbit.GetNearestPoint directly (Burn.cs:999), below this seam,
/// where OrbitHoverPatch keeps it riding the drawn n-body lines. Unowned vessels
/// (mod disabled/unbound/never tracked) keep stock behavior — the containment story.</summary>
[HarmonyPatch(typeof(Orbit), "GetNearestPosition")]
internal static class BurnClickPatch
{
    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static bool Prefix(ref CelestialPosition? positionSelected, ref bool __result)
    {
        if (!ModServices.Enabled) return true;
        try
        {
            if (KSA.Program.ControlledVehicle is not { } vehicle) return true;
            if (OverlayBuffer.Read(vehicle.Id) is null) return true; // not owned: stock
            positionSelected = null;
            __result = false;
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info("map click-to-place burns retired (burns are authored in the "
                    + "N-Body Planner; node dragging unaffected)");
            return false;
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("burn click suppression", e);
            return true;
        }
    }
}

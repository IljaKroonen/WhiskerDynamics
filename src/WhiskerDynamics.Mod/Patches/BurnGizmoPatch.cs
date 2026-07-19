using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Burn gizmo shrink: stock's draggable burn
/// node — the yellow sphere plus six delta-v cone handles — scales as 3% of camera
/// distance (Burn.UpdateGizmos, Burn.cs:398-399), which reads as a billboard on the
/// honest lines, and its pick radius (GenericGizmo.RaycastEgo tests the scaled mesh)
/// grabs clicks meant for the line. This postfix runs right after stock writes the
/// per-frame segment data and shrinks every ACTIVE segment of the burn's three gizmos
/// by a 0.35 factor: mesh scale and pick radius
/// shrink together (RaycastEgo raycasts the scaled mesh), and the cone handles'
/// offsets from the sphere centre pull in by the same factor so the cluster stays
/// coherent instead of exploding apart. Stock's own state layering (grab highlight
/// ×1.1, inactive ×0.75) happens before this multiplier, so relative emphasis is
/// preserved. Private gizmo fields are reached via AccessTools field refs — all three
/// are pinned in the registry like every other game member the mod touches.</summary>
[HarmonyPatch(typeof(Burn), "UpdateGizmos")]
internal static class BurnGizmoPatch
{
    private const double ScaleFactor = 0.35;

    private static readonly AccessTools.FieldRef<Burn, GenericGizmo> SphereGizmoRef =
        AccessTools.FieldRefAccess<Burn, GenericGizmo>("SphereGizmo");
    private static readonly AccessTools.FieldRef<Burn, GenericGizmo> ConeGizmoRef =
        AccessTools.FieldRefAccess<Burn, GenericGizmo>("ConeGizmo");
    private static readonly AccessTools.FieldRef<Burn, GenericGizmo> ConeReverseGizmoRef =
        AccessTools.FieldRefAccess<Burn, GenericGizmo>("ConeReverseGizmo");

    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static void Postfix(Burn __instance, Viewport inViewport)
    {
        if (!ModServices.Enabled) return;
        try
        {
            // Wrong-frame gizmo hiding: in a
            // COUNTER-POSED view a burn whose drawn-line position cannot resolve
            // (batches stale mid-burn, time outside the window) would render at its
            // stock CONIC position — inertial-embedded geometry under a counter-posed
            // camera, exactly the artifact the stale-line policy refuses to draw. Hide the
            // gizmos for that burn until the rebuild resumes; inertial views — no
            // frame or a body-centred inertial frame, no counter-pose — keep the
            // stock-position fallback.
            // Parent-eject burns anchor to the celestial's orbit and stay stock's.
            if (!FrameManager.InertialView && !__instance.ParentEjectBurn
                && OverlayBuffer.Read(__instance.Vehicle.Id) is not null
                && !BurnNodePatch.TryDrawnBurnPosition(__instance, out _))
            {
                Deactivate(SphereGizmoRef(__instance).GetSegmentDataByViewport(inViewport));
                Deactivate(ConeGizmoRef(__instance).GetSegmentDataByViewport(inViewport));
                Deactivate(ConeReverseGizmoRef(__instance).GetSegmentDataByViewport(inViewport));
                return;
            }

            var sphere = SphereGizmoRef(__instance).GetSegmentDataByViewport(inViewport);
            // The sphere centre anchors the cone pull-in; stock always writes
            // segment 0 before the cones (Burn.cs:405-409).
            double3 center = sphere[0].PositionEgo;
            Shrink(sphere, center);
            Shrink(ConeGizmoRef(__instance).GetSegmentDataByViewport(inViewport), center);
            Shrink(ConeReverseGizmoRef(__instance).GetSegmentDataByViewport(inViewport), center);

            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"burn gizmo shrink active (factor {ScaleFactor:F2}; mesh and pick radius together)");
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("burn gizmo shrink", e);
        }
    }

    private static void Shrink(GenericGizmo.PerSegmentData[] segments, double3 center)
    {
        for (int i = 0; i < segments.Length; i++)
        {
            if (!segments[i].Active) continue;
            segments[i].Scale *= ScaleFactor;
            segments[i].PositionEgo = center + (segments[i].PositionEgo - center) * ScaleFactor;
        }
    }

    private static void Deactivate(GenericGizmo.PerSegmentData[] segments)
    {
        for (int i = 0; i < segments.Length; i++) segments[i].Active = false;
    }
}

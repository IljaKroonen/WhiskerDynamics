using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>SOI-independence: hide the stock sphere-of-influence indicator
/// only while the mod is enabled, bound, and caught up to current simulation time
/// (<see cref="OverlayKernel.SoiIndicatorsHidden"/> — under n-body gravity the SOI
/// boundary has no dynamical meaning, and the mod's map lines deliberately ignore it).
/// Stock's ONLY map SOI visual is the translucent glass-ball sphere submitted from
/// IOrbiter.OnDrawUi (IOrbiter.cs:296-311): drawn when the body is the controlled
/// vehicle's target (:298-300) or its "Show SOI" checkbox is set (:172-175), as a
/// single-use GenericGizmo on the GlassBallGizmoRenderData singleton (:302), scaled to
/// the SOI radius (:308), tinted parentBody.SoiColor (:309). Caller census: that
/// singleton has exactly one draw-site user in the whole game (GenericGizmo.cs:20/25
/// defines it; IOrbiter.cs:302 is the only GetSingleUse caller passing it; no direct
/// GenericGizmo constructions with it exist), so "every glass-ball instance" IS "every
/// SOI sphere" — suppressing the render-data class suppresses exactly the SOI spheres.
/// Suppression point: a VOID prefix on GizmoParent.UpdateRenderData, the method that
/// consumes PerSegmentData.Active flags into the render instance vector
/// (GizmoParent.cs:170-176). The prefix clears Active on the glass-ball parent's
/// instances for this viewport/pass, so the original submits nothing — and because the
/// prefix is void it can never skip the original, so the single-use pool reclamation
/// (GizmoParent.cs:178-181 queues, PostRender :226-235 disposes) always runs: no gizmo
/// leak, and any prefix failure degrades to stock behavior (the sphere draws).
/// Deliberately untouched, being data displays rather than map visuals: the celestial
/// window's "Show SOI" checkbox (IOrbiter.cs:172 — toggling it is simply inert while
/// the mod is bound), the SphereOfInfluence text row (Celestial.cs:1642), and the
/// universe manifest's SOI column (UniverseManifest.cs:336-341).</summary>
[HarmonyPatch(typeof(GizmoParent), nameof(GizmoParent.UpdateRenderData))]
internal static class SoiIndicatorPatch
{
    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static void Prefix(GizmoParent __instance, Viewport viewport, int passIndex)
    {
        try
        {
            var rails = ModServices.Rails;
            bool ready = rails is not null
                && rails.IsReadyAt(Universe.GetElapsedSimTime().Seconds());
            if (!OverlayKernel.SoiIndicatorsHidden(ModServices.Enabled, rails is not null, ready))
                return; // unbound/disabled: stock propagation, truthful indicator — stock draws
            if (__instance.RenderData is not GlassBallGizmoRenderData) return;
            var instances = __instance.Instances;
            bool hidSomething = false;
            for (int i = 0; i < instances.Count; i++)
            {
                if (instances[i].PassIndex != passIndex) continue; // stock's own pass gate (GizmoParent.cs:162)
                var segments = instances[i].GetSegmentDataByViewport(viewport);
                for (int s = 0; s < segments.Length; s++)
                {
                    hidSomething |= segments[s].Active;
                    segments[s].Active = false; // original finds nothing Active: no instance data submitted
                }
            }
            if (hidSomething && System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info("stock SOI indicator hidden (n-body authority ready; "
                    + "n-body gravity has no sphere of influence)");
        }
        catch (Exception e)
        {
            // Void prefix: the original always runs, so a failure here means the SOI
            // sphere simply draws — stock behavior, the correct degrade.
            FrameManager.NoteContained("SOI indicator suppression", e);
        }
    }
}

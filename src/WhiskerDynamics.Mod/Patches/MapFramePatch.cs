using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>The whole-map rotating view. Postfix on the map controller's
/// per-frame pose pass — Viewport.OnFrame runs the controller, then Camera.OnFrame builds
/// the view matrix (Viewport.cs:171-174), so this postfix re-poses the camera between the
/// two: a rigid rotation of the camera rig about the FOLLOW ANCHOR (the
/// followed target's position; the stock follow offset lives in fixed ecliptic axes, so
/// rotating about the frame origin would slide off-origin targets away)
/// by the frame's accumulated rotation since activation. Rigid transforms preserve
/// relative geometry, so this is visually identical to rotating the WORLD the other way
/// around the followed target — the target stays pixel-fixed, the primary-secondary line
/// holds its bearing, stars sweep — while the world data, physics, and picking stay
/// untouched (display-only invariant). Map mode only; contained locally.</summary>
[HarmonyPatch(typeof(MapController), "OnFrame")]
internal static class MapFramePatch
{
    private static int _activeLogged;
    private static long _nextTelemetryMs; // wall-clock throttle: deliberately not reset (convention)

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static void Postfix(Viewport inViewport, double inDeltaTime)
    {
        if (!ModServices.Enabled) return;
        try
        {
            if (inViewport.Mode != CameraMode.Map) return;
            if (!FrameManager.TryGetCameraDelta(out var center, out var delta)) return;
            var camera = inViewport.GetCamera();
            var following = camera.Following;               // registered: Camera.Following
            double3? anchor = following?.GetPositionEcl();  // registered: IPosition.GetPositionEcl
            var stockPosition = camera.PositionEcl;
            var stockRotation = camera.LocalRotation;
            (var position, var rotation) = MapPoseKernel.FrameViewPose(
                stockPosition, stockRotation, center, anchor, delta);
            camera.PositionEcl = position;
            camera.LocalRotation = rotation;
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"map frame view active: {FrameManager.Active?.Label} (camera counter-pose)");
            if (following is not null && anchor is { } target)
                MaybeLogTelemetry(following, target, center, delta,
                    stockPosition, stockRotation, position, rotation);
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("map counter-pose", e);
        }
    }

    /// <summary>Follow-coherence telemetry (config-gated, default off).
    /// The observable: the followed target's camera-relative VIEW vector must be the
    /// same before and after the re-pose (Ego2View convention, Camera.cs:63) — any
    /// difference is exactly the on-screen drift the user sees. Logged with the
    /// accumulated frame angle and the target's distance from the frame origin so the
    /// drift-vs-angle relationship is quantified from the log alone.</summary>
    private static void MaybeLogTelemetry(
        IFollowable following, double3 target, double3 center, doubleQuat delta,
        double3 stockPosition, doubleQuat stockRotation,
        double3 posedPosition, doubleQuat posedRotation)
    {
        double period = ModServices.Config.MapPoseTelemetrySeconds;
        if (period <= 0) return;
        long now = Environment.TickCount64;
        if (now < System.Threading.Interlocked.Read(ref _nextTelemetryMs)) return;
        System.Threading.Interlocked.Exchange(ref _nextTelemetryMs, now + (long)(period * 1000));
        var viewStock = double3.Transform(target - stockPosition, doubleQuat.Inverse(stockRotation));
        var viewPosed = double3.Transform(target - posedPosition, doubleQuat.Inverse(posedRotation));
        double drift = (viewPosed - viewStock).Length();
        double angleDeg = 2.0 * Math.Acos(Math.Clamp(Math.Abs(delta.W), 0.0, 1.0)) * (180.0 / Math.PI);
        double t = Universe.GetElapsedSimTime().Seconds();
        ModLog.Info($"map pose telemetry: following='{following.Id}' |T-C|={(target - center).Length():E3} m "
            + $"angle={angleDeg:F3} deg targetViewDrift={drift:E3} m t={t:F1} s");
    }
}

using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>One serialization boundary for every writer of Orbit's pooled point
/// cache. Stock recalculation and every mod staging/restoration call all pass through
/// UpdateCachedPoints, so entering TrajectoryOverlay's stable per-Orbit monitor here
/// makes stock capture plus mod handoff atomic. Stage already holds the same monitor;
/// Monitor reentrancy makes that call shape safe. The finalizer is the unconditional
/// release boundary when the game method itself throws.</summary>
[HarmonyPatch(typeof(Orbit), nameof(Orbit.UpdateCachedPoints))]
internal static class OrbitCacheUpdatePatch
{
    internal static void Prefix(Orbit __instance, out object? __state) =>
        __state = TrajectoryOverlay.EnterOrbitCacheUpdate(__instance);

    internal static Exception? Finalizer(Exception? __exception, object? __state)
    {
        if (__state is not null) TrajectoryOverlay.ExitOrbitCacheUpdate(__state);
        return __exception;
    }
}

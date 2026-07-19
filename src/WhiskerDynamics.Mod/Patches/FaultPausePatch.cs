using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Runtime authority faults are terminal for the loaded session. Rewrite
/// every attempted positive simulation speed to zero so user input, auto-warp, and
/// other callers cannot resume into stock propagation after the n-body services stop.</summary>
[HarmonyPatch(typeof(Universe), nameof(Universe.SetSimulationSpeed),
    [typeof(double), typeof(bool)])]
internal static class FaultPausePatch
{
    static void Prefix(ref double __0)
    {
        __0 = FaultPausePolicy.RequestedSpeed(ModServices.Status, __0);
    }
}

[HarmonyPatch(typeof(Universe), nameof(Universe.GetSimulationSpeed))]
internal static class FaultPauseReadPatch
{
    static void Postfix(ref double __result)
    {
        __result = FaultPausePolicy.ObservedSpeed(ModServices.Status, __result);
    }
}

internal static class FaultPausePolicy
{
    internal static double RequestedSpeed(ModStatus status, double requested) =>
        status == ModStatus.DisabledFault ? 0.0 : requested;

    internal static double ObservedSpeed(ModStatus status, double observed) =>
        status == ModStatus.DisabledFault ? 0.0 : observed;
}

internal static class FaultPauseEnforcer
{
    internal static bool TryEnforce(
        ModStatus status, Action<double> setSimulationSpeed)
    {
        ArgumentNullException.ThrowIfNull(setSimulationSpeed);
        if (status != ModStatus.DisabledFault) return false;
        setSimulationSpeed(0.0);
        return true;
    }
}

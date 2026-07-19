using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Runtime canary on the commit surface (also a named
/// non-inline-marked Seam 1 caller): after results are committed to the live Vehicle,
/// verify the committed on-rails state matches the mod trajectory. This is the proof
/// that the Seam 1 postfixes actually take effect — if the small AggressiveInlining
/// evaluators were reached through a path our patched callers do not cover, the
/// committed states drift off the predictor and the mod disables itself loudly.</summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.UpdateFromTaskResults))]
internal static class CommitCanaryPatch
{
    static void Postfix(Vehicle __instance)
    {
        if (!ModServices.Enabled) return;
        try
        {
            ModServices.Vessels?.VerifyCommit(__instance);
        }
        catch (Exception e)
        {
            ModLog.Error($"commit canary itself failed: {e.Message}");
        }
    }
}

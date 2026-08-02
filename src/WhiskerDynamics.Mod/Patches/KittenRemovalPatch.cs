using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Deletes the starting EVA kittens once per session. Later EVA kittens get
/// Kitten_N ids, so these ids only name the starting three.</summary>
[HarmonyPatch(typeof(Program), "OnDrawUiConsole")]
internal static class KittenRemovalPatch
{
    private static readonly string[] KittenIds = ["Hunter", "Banjo", "Polaris"];
    private static int _swept;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _swept, 0);

    static void Postfix()
    {
        if (!ModServices.Enabled || System.Threading.Volatile.Read(ref _swept) != 0) return;
        try
        {
            var system = Universe.CurrentSystem;
            if (system is null) return;
            System.Threading.Volatile.Write(ref _swept, 1);
            foreach (string id in KittenIds)
            {
                if (system.Get(id) is not KittenEva kitten) continue;
                InputEvents.VehicleDestroyBuffer.Add(
                    new InputEvents.VehicleDestroyData { Vehicle = kitten });
                ModLog.Info($"queued destruction of starting EVA kitten '{id}'");
            }
        }
        catch (Exception e)
        {
            System.Threading.Volatile.Write(ref _swept, 1);
            ModLog.Error($"starting-kitten removal failed: {e}");
        }
    }
}

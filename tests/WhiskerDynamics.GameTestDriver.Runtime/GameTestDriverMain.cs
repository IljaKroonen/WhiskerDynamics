using HarmonyLib;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.GameTestDriver.Runtime;

public static class GameTestDriverMain
{
    public static void Load()
    {
        var harmony = new Harmony("ksa.whiskerdynamics.game-tests");
        HarmonyPatchActivation.ApplyAndWarm(harmony,
        [
            typeof(GameTestScenarioPatch),
            typeof(GameTestRcsBurnCutoffPatch),
        ]);
        ModLog.Info("game test driver loaded");
    }
}

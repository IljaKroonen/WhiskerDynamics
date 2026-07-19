using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Writes mod state alongside a completed, identity-bearing stock save.</summary>
[HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Write))]
internal static class SaveSidecarWritePatch
{
    static void Postfix(UncompressedSave __instance)
    {
        if (!ModServices.Enabled) return;
        try
        {
            if (!ModServices.TryGetBound(out var services)) return;
            var rails = services.Rails;
            var vessels = services.Vessels;
            // The main thread may spend time writing stock files before this postfix.
            // Pair against the epoch captured in the save, never a later live clock.
            double elapsed = __instance.UniverseData.GetElapsedSeconds();
            string build = typeof(Program).Assembly.GetName().Version?.ToString() ?? "unknown";
            long sequence = SaveSidecar.QueueWrite(rails, vessels.SnapshotForSidecar(elapsed), elapsed, build,
                Ui.BurnPlannerPanel.PendingForSidecar(), saveIdentity: __instance.Id,
                saveGenerationTicks: __instance.MetaData.Updated.Ticks);
            if (!SaveSidecar.WaitForDurability(sequence, 75))
                ModLog.Warn($"sidecar for save t={elapsed:F1} s was not durable within 75 ms; "
                    + "disk writing continues and the capture is temporarily available in the "
                    + "bounded recent-memory cache");
        }
        catch (Exception e)
        {
            ModLog.Error($"sidecar write failed (the save file itself is stock and unaffected): {e}");
        }
    }
}

/// <summary>Rebinds mod state after a named stock save has finished deserializing.</summary>
[HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Load))]
internal static class SaveSidecarRestorePatch
{
    static void Postfix(UncompressedSave __instance)
    {
        if (!ModServices.Enabled) return;
        try
        {
            ModServices.InvalidateBinding("save loaded");
            if (!ModServices.EnsureBound(out var services)) return;
            double elapsed = __instance.UniverseData.GetElapsedSeconds();
            var sidecar = SaveSidecar.TryRead(elapsed, saveIdentity: __instance.Id,
                saveGenerationTicks: __instance.MetaData.Updated.Ticks);
            if (sidecar is not null)
            {
                services.Vessels.ImportSidecar(sidecar);
                int plans = FlightPlans.ImportSidecar(sidecar);
                int frames = FrameManager.ImportFrameSelections(sidecar);
                Ui.BurnPlannerPanel.RestorePendingCleanup(sidecar.PendingRendezvous);
                ModLog.Info($"sidecar matched for elapsed={elapsed:F1} s "
                    + $"({sidecar.Vessels.Count} vessels, {plans} flight plans, "
                    + $"{frames} frame selections)");
            }
            else
            {
                ModLog.Warn($"no sidecar for elapsed={elapsed:F1} s - vessels reseed from stock osculating states");
            }
        }
        catch (Exception e)
        {
            ModLog.Error($"sidecar restore failed - vessels reseed from stock osculating states: {e}");
        }
    }
}

/// <summary>Runs optional, config-driven save/load/warp diagnostics.</summary>
[HarmonyPatch(typeof(Program), "OnDrawUiConsole")]
internal static class SaveDrillPatch
{
    // Drills run at most once per process, including across loads.
    private static bool _saveFired;
    private static bool _loadFired;
    private static bool _warpFired;
    private static long _warpArmedAtMs = -1;

    static void Postfix()
    {
        if (!ModServices.Enabled || (_saveFired && _loadFired && _warpFired)) return;
        try
        {
            var config = ModServices.Config;
            if (Universe.CurrentSystem is null) return;
            double elapsed = Universe.GetElapsedSimTime().Seconds();
            if (!_saveFired && config.DrillSaveName.Length > 0 && config.DrillSaveAtSeconds > 0
                && elapsed >= config.DrillSaveAtSeconds)
            {
                _saveFired = true;
                ModLog.Info($"save drill: creating save '{config.DrillSaveName}' at t={elapsed:F1} s");
                GameSaves.MakeUncompressedSave(config.DrillSaveName);
            }
            if (!_loadFired && config.DrillLoadName.Length > 0 && config.DrillLoadAtSeconds > 0
                && elapsed >= config.DrillLoadAtSeconds)
            {
                _loadFired = true;
                ModLog.Info($"load drill: loading save '{config.DrillLoadName}' at t={elapsed:F1} s");
                GameSaves.LoadSaveGame(config.DrillLoadName);
            }
            if (!_warpFired && config.DrillWarpSpeed > 0
                && (config.DrillLoadName.Length == 0 || _loadFired))
            {
                long now = Environment.TickCount64;
                if (_warpArmedAtMs < 0) _warpArmedAtMs = now;
                if (now - _warpArmedAtMs >= config.DrillWarpDelayMs)
                {
                    _warpFired = true;
                    ModLog.Info($"warp drill: simspeed {config.DrillWarpSpeed:F0} at t={elapsed:F1} s");
                    Universe.SetSimulationSpeed(config.DrillWarpSpeed);
                }
            }
        }
        catch (Exception e)
        {
            _saveFired = true; // a throwing drill must not retry every frame
            _loadFired = true;
            _warpFired = true;
            ModLog.Error($"save/load/warp drill failed: {e}");
        }
    }
}

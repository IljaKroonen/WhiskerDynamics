using HarmonyLib;
using KSA;
using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Target verified: private void Program.OnDrawUiConsole(double dt) runs every
/// frame between PrepareImGui and ImGui.Render, even when the HUD is hidden.</summary>
[HarmonyPatch(typeof(Program), "OnDrawUiConsole")]
internal static class StatusPanelPatch
{
    static StatusPanelPatch() => StatusPanel.ExtraLines.Add(() =>
        ModServices.Vessels?.PausedEditsDeferred == true
            ? ["paused trajectory edit deferred until the next completed simulation capture"]
            : []);

    static void Postfix()
    {
        ModServices.EnforceFaultPauseOnMainThread();
        StatusPanel.Draw();
        BurnPlannerPanel.Draw();
        FramesPanel.Draw();
        SettingsPanel.Draw();
        OrbitAnalyserPanel.Draw();
        LagrangeOverlay.Draw();
        LineMarkers.Draw();
        // Simulation-speed zero suppresses the vehicle update seams that normally
        // rebuild and refresh trajectory buffers. Run the same throttled capture from
        // the render frame after every planner/frame UI mutation so paused planning is
        // fully interactive and existing lines do not age out.
        if (ModServices.Enabled)
            ModServices.Vessels?.RefreshPausedOverlays();
    }
}

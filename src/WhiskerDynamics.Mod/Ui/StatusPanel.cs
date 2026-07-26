using Brutal.ImGuiApi;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>Session-local status panel: mod status, disable notice, recent log lines.
/// Other components append rails/vessel telemetry via the Extra lines hook.</summary>
public static class StatusPanel
{
    /// <summary>Registered telemetry providers; each returns display lines.</summary>
    public static readonly List<Func<IEnumerable<string>>> ExtraLines = [];

    private static int _errors;
    private static bool _open;
    private static bool _firstDrawLogged;
    private static readonly StatusTelemetryCache TelemetryCache = new(refreshIntervalMs: 500);
    private static readonly Func<IReadOnlyList<string>> RefreshTelemetryCallback = BuildTelemetryLines;

    private static IReadOnlyList<string> BuildTelemetryLines()
    {
        var lines = new List<string>();
        foreach (var provider in ExtraLines) lines.AddRange(provider());
        lines.AddRange(ModLog.Snapshot().TakeLast(5));
        return lines;
    }

    internal static void Open() => _open = true;

    /// <summary>Session sweep hook: never carry cached vessel/system text across a
    /// save load or rebind, and make the first draw in the new session refresh now.</summary>
    internal static void ResetSessionStatics()
    {
        _open = false;
        _errors = 0;
        _firstDrawLogged = false;
        DiagnosticDisplay.ResetSessionStatics();
        TelemetryCache.Reset();
    }

    public static void Draw()
    {
        if (!_open || _errors >= 3) return; // panel misbehaving: stop drawing, keep the game alive
        try
        {
            UiTheme.PrepareWindow(680f, 420f, 520f, 240f);
            bool visible = ImGui.Begin("Whisker Dynamics: Diagnostics"u8, ref _open);
            try
            {
                if (!visible) return;
                UiTheme.MutedText(
                    "Runtime state, trajectory telemetry, and recent log output.");
                ImGui.SeparatorText("Service status"u8);
                if (ModServices.Status == ModStatus.Active)
                    ImGui.Text($"N-body service: {ModServices.Status}");
                else
                    ImGui.Text($"N-body service: {ModServices.Status}");
                if (ModServices.Status == ModStatus.DisabledIncompatible)
                {
                    ImGui.TextWrapped("N-body disabled: game build not yet supported.");
                    foreach (var line in ModServices.Mismatches)
                        ImGui.TextWrapped(line);
                }
                ImGui.SeparatorText("Visual debugging"u8);
                bool showStockPatchedConics =
                    DiagnosticDisplay.ShowStockPatchedConics;
                if (ImGui.Checkbox(
                        "Show stock patched conics"u8,
                        ref showStockPatchedConics))
                {
                    DiagnosticDisplay.ShowStockPatchedConics =
                        showStockPatchedConics;
                    ModLog.Info("diagnostics: stock patched conics "
                        + (showStockPatchedConics ? "shown" : "hidden"));
                }
                ImGui.SetItemTooltip(
                    "draw the controlled vessel's stock patched-conic lines alongside the n-body actual and planned trajectories; visual comparison only, and reset when the game session changes"u8);
                ImGui.SeparatorText("Telemetry and recent log"u8);
                var telemetryLines = TelemetryCache.Read(
                    Environment.TickCount64, RefreshTelemetryCallback);
                foreach (var line in telemetryLines) ImGui.TextWrapped(line);
            }
            finally
            {
                // Once Begin has been called, End must ALWAYS run — a swallowed .NET
                // exception must not leave the native ImGui window stack unbalanced.
                ImGui.End();
            }
            if (!_firstDrawLogged)
            {
                // Log-based proxy for "the panel actually rendered": Begin/Text/End all
                // completed on a real frame (in-game verification polls whiskerdynamics.log for this).
                _firstDrawLogged = true;
                ModLog.Info($"status panel drawn (first frame; status: {ModServices.Status})");
            }
        }
        catch (Exception e)
        {
            _errors++;
            ModLog.Error($"status panel: {e}");
        }
    }
}

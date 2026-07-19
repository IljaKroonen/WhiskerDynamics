using Brutal.ImGuiApi;

namespace WhiskerDynamics.Mod.Ui;

public static class SettingsPanel
{
    private static bool _open;
    private static int _errors;
    private static ModConfig? _draft;
    private static string _status = "";
    private static long _nextWarnMs;

    internal static void Open()
    {
        _draft = SettingsKernel.CreateRuntimeDraft(ModServices.Config);
        _status = "";
        _open = true;
    }

    internal static void ResetSessionStatics()
    {
        _open = false;
        _errors = 0;
        _draft = null;
        _status = "";
    }

    public static void Draw()
    {
        if (!_open || _errors >= 3 || _draft is null) return;
        try
        {
            UiTheme.PrepareWindow(560f, 620f, 500f, 320f);
            bool visible = ImGui.Begin("Whisker Dynamics: Settings"u8, ref _open);
            try
            {
                if (!visible) return;
                UiTheme.MutedText(
                    "Orbit display, history, and line visibility remain in Frames.");
                DrawPredictionSettings(_draft);
                DrawCoherenceSettings(_draft);
                DrawOverlaySettings(_draft);
                DrawDiagnosticsSettings(_draft);
                ImGui.SeparatorText("Actions"u8);
                DrawActions();
                if (_status.Length > 0)
                    ImGui.TextWrapped($"Last change: {_status}");
            }
            finally
            {
                ImGui.End();
            }
        }
        catch (Exception e)
        {
            _errors++;
            ModLog.Error($"settings panel: {e}");
        }
    }

    private static void DrawActions()
    {
        if (!ImGui.BeginTable("##settings-actions"u8, 3,
                ImGuiTableFlags.SizingStretchSame)) return;
        try
        {
            ImGui.TableNextRow();
            ImGui.TableNextColumn();
            if (ImGui.Button("Apply and save"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
                Apply();
            ImGui.TableNextColumn();
            if (ImGui.Button("Reload live values"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
            {
                _draft = SettingsKernel.CreateRuntimeDraft(ModServices.Config);
                _status = "reloaded live settings";
            }
            ImGui.TableNextColumn();
            if (ImGui.Button("Load defaults"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
            {
                _draft = SettingsKernel.CreateRuntimeDraft(new ModConfig());
                _status = "defaults loaded; apply to use them";
            }
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    private static void DrawPredictionSettings(ModConfig draft)
    {
        ImGui.SeparatorText("Prediction"u8);
        draft.RailsKeepBehindDays = DoubleSetting(
            "rails retained behind (days)", "##railsbehind"u8,
            draft.RailsKeepBehindDays,
            "past celestial and vessel state retained in memory; clamps to [0, 3650] days"u8);
        draft.FiniteBurnSliceSeconds = DoubleSetting(
            "finite-burn slice (seconds)", "##burnslice"u8,
            draft.FiniteBurnSliceSeconds,
            "sub-impulse spacing for planned finite burns; 0 disables finite-burn folding; clamps to [0, 3600]"u8);
        draft.FiniteBurnMaxSlices = IntSetting(
            "finite-burn max slices", "##burnslices"u8,
            draft.FiniteBurnMaxSlices,
            "maximum sub-impulses per planned burn; clamps to [1, 1024]"u8);
    }

    private static void DrawCoherenceSettings(ModConfig draft)
    {
        ImGui.SeparatorText("Vessel coherence"u8);
        draft.ConicDriftMeters = DoubleSetting(
            "conic drift threshold (m)", "##conicdrift"u8,
            draft.ConicDriftMeters,
            "re-osculate the stock compatibility conic after this much drift; clamps to >= 0"u8);
        draft.OsculationRefreshSeconds = DoubleSetting(
            "osculation refresh (seconds)", "##osculation"u8,
            draft.OsculationRefreshSeconds,
            "maximum interval between stock compatibility conic refreshes; clamps to [1, 86400]"u8);
        draft.CanaryToleranceMeters = DoubleSetting(
            "canary tolerance (m)", "##canary"u8,
            draft.CanaryToleranceMeters,
            "allowed authoritative commit residual before the canary strikes; clamps to >= 0"u8);
    }

    private static void DrawOverlaySettings(ModConfig draft)
    {
        ImGui.SeparatorText("Trajectory lines"u8);
        draft.OverlayMaxTurnDeg = DoubleSetting(
            "max turn (degrees)", "##turn"u8,
            draft.OverlayMaxTurnDeg,
            "maximum angular turn for every vessel and celestial segment; clamps to [0.05, 10]"u8);
        draft.OverlayMaxPoints = IntSetting(
            "vessel line point budget", "##overlaypoints"u8,
            draft.OverlayMaxPoints,
            "maximum dense points per vessel path; clamps to [2000, 262144]"u8);
        draft.CelestialMaxPoints = IntSetting(
            "celestial line point budget", "##celestialpoints"u8,
            draft.CelestialMaxPoints,
            "maximum dense points per celestial path; clamps to [2000, 65536]"u8);
        draft.CelestialCurveMaxBodies = IntSetting(
            "celestial path body budget", "##celestialbodies"u8,
            draft.CelestialCurveMaxBodies,
            "maximum number of mod-drawn celestial paths; clamps to [1, 256]"u8);
    }

    private static void DrawDiagnosticsSettings(ModConfig draft)
    {
        ImGui.SeparatorText("Diagnostics"u8);
        draft.MapPoseTelemetrySeconds = DoubleSetting(
            "map pose telemetry (seconds)", "##mappose"u8,
            draft.MapPoseTelemetrySeconds,
            "follow-coherence log interval; 0 disables telemetry"u8);
    }

    private static void Apply()
    {
        var applied = SettingsKernel.ApplyRuntimeSettings(ModServices.Config, _draft!);
        _draft = SettingsKernel.CreateRuntimeDraft(ModServices.Config);
        string repairNote = applied.Repairs.Count == 0
            ? ""
            : $"; {applied.Repairs.Count} value(s) clamped";
        string appliedNote = applied.Changed ? "applied" : "no live changes";
        string dir = ModMain.ModDir;
        if (dir.Length == 0)
        {
            _status = $"{appliedNote}; save unavailable{repairNote}";
            return;
        }
        if (SettingsPersistence.TrySave(ModServices.Config,
                Path.Combine(dir, "whiskerdynamics.toml"), out string error))
        {
            _status = $"{appliedNote} and saved{repairNote}";
            ModLog.Info($"settings: settings saved to whiskerdynamics.toml{repairNote}");
        }
        else
        {
            _status = $"{appliedNote}; save FAILED: {error}{repairNote}";
            if (Environment.TickCount64 >= _nextWarnMs)
            {
                _nextWarnMs = Environment.TickCount64 + 5000;
                ModLog.Warn($"settings: whiskerdynamics.toml write failed: {error}");
            }
        }
    }

    private static double DoubleSetting(string label, ReadOnlySpan<byte> id,
        double value, ReadOnlySpan<byte> tooltip)
    {
        ImGui.PushID(id);
        try
        {
            if (!UiLayout.BeginProperties("##setting-property"u8,
                    UiTheme.SettingsLabelWidth)) return value;
            try
            {
                UiLayout.NextProperty(label);
                ImGui.SetNextItemWidth(-1f);
                ImGui.InputDouble("##value"u8, ref value, 0.0, 0.0,
                    default(ImString), ImGuiInputTextFlags.CharsScientific);
                ImGui.SetItemTooltip(tooltip);
            }
            finally
            {
                ImGui.EndTable();
            }
        }
        finally
        {
            ImGui.PopID();
        }
        return value;
    }

    private static int IntSetting(string label, ReadOnlySpan<byte> id,
        int value, ReadOnlySpan<byte> tooltip)
    {
        ImGui.PushID(id);
        try
        {
            if (!UiLayout.BeginProperties("##setting-property"u8,
                    UiTheme.SettingsLabelWidth)) return value;
            try
            {
                UiLayout.NextProperty(label);
                ImGui.SetNextItemWidth(-1f);
                ImGui.InputInt("##value"u8, ref value, 1, 100,
                    ImGuiInputTextFlags.CharsDecimal);
                ImGui.SetItemTooltip(tooltip);
            }
            finally
            {
                ImGui.EndTable();
            }
        }
        finally
        {
            ImGui.PopID();
        }
        return value;
    }
}

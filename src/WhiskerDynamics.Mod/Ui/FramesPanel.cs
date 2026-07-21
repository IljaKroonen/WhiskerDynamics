using Brutal.ImGuiApi;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>Frame selector over the shared <see cref="FrameTreeControl"/> body tree.
/// The map ALWAYS runs in one of the mod's frames: there is no stock/no-frame
/// button — the panel's every-frame draw drives FrameManager.EnsureActiveOrDefault
/// (restore the last-activated frame, else FrameSelectorKernel.DefaultFrame).
/// The panel also owns the session-only "show orbits this far ahead" control, applied
/// through SettingsKernel.ApplyPrediction. It starts at 30 days on every process launch.
/// The line-visibility policy is a game-local control.
/// How far ahead positions are
/// COMPUTED is implicit: the rails horizon follows the display window both ways within
/// the preset range (SettingsKernel.EditPrediction), with catch-up progress surfaced in
/// the map-trajectory readout. Decision
/// rules live in the KSA-free <see cref="FrameSelectorKernel"/>; display-only surface
/// with the panel 3-strike pattern (balanced Begin/End).</summary>
public static class FramesPanel
{
    private static int _errors;
    private static string _status = "";
    private static bool _firstDrawLogged;

    private static readonly FrameTreeControl Tree = new();

    /// <summary>Statics sweep: fresh tree state and status for the new session
    /// (the session horizon values live in the shared ModConfig, which survives
    /// in-process rebinds by design).</summary>
    internal static void ResetSessionStatics()
    {
        _errors = 0;
        _status = "";
        _firstDrawLogged = false;
        _horizonReadout = "";
        _horizonReadoutMs = 0;
        _trajectoryProgress = null;
        _trajectoryProgressLabel = "";
        Tree.Reset();
    }

    public static void Draw()
    {
        if (_errors >= 3) return; // panel misbehaving: stop drawing, keep the game alive
        try
        {
            ModServices.BoundServices services = default;
            bool available = ModServices.Enabled && ModServices.EnsureBound(out services);
            var active = available ? FrameManager.EnsureActiveOrDefault() : null;
            UiTheme.PrepareWindow(480f, 640f, 440f, 320f);
            bool visible = ImGui.Begin("Whisker Dynamics: Frames"u8);
            try
            {
                if (!visible) return;
                UiTheme.MutedText("Reference frames, trajectory coverage, and mission tools.");
                ImGui.SeparatorText("Mission tools"u8);
                DrawWindowLaunchers();

                if (available)
                {
                    ImGui.Spacing();
                    ImGui.SeparatorText("Map trajectories"u8);
                    DrawHorizonRow(ModServices.Config, services.Rails, active);
                    DrawLinePolicyRow();
                    DrawPotentialRow(active);

                    ImGui.Spacing();
                    ImGui.SeparatorText("Frame selection"u8);
                    UiTheme.MutedText("Active frame", wrapped: false);
                    ImGui.Text(active?.Label ?? "Activating default...");
                    if (Tree.Draw(active) is { } clicked)
                        _status = FrameManager.Activate(clicked);

                    if (_status.Length > 0)
                        ImGui.TextWrapped($"Last change: {_status}");
                }
                else
                {
                    ImGui.Spacing();
                    ImGui.TextWrapped(
                        $"N-body unavailable ({ModServices.Status}); open Diagnostics for details.");
                }
            }
            finally
            {
                ImGui.End();
            }
            if (!_firstDrawLogged)
            {
                _firstDrawLogged = true;
                ModLog.Info("frames panel drawn (first frame)");
            }
        }
        catch (Exception e)
        {
            _errors++;
            ModLog.Error($"frames panel: {e}");
        }
    }

    private static void DrawWindowLaunchers()
    {
        if (!ImGui.BeginTable("##window-launchers"u8, 2,
                ImGuiTableFlags.SizingStretchSame)) return;
        try
        {
            ImGui.TableNextRow();
            ImGui.TableNextColumn();
            if (ImGui.Button("Planner"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
                BurnPlannerPanel.Open();
            ImGui.SetItemTooltip("open the flight-plan editor"u8);
            ImGui.TableNextColumn();
            if (ImGui.Button("Orbit analysis"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
                OrbitAnalyserPanel.Open();
            ImGui.SetItemTooltip("open sampled n-body orbit analysis; computation runs only while its window is open"u8);
            ImGui.TableNextRow();
            ImGui.TableNextColumn();
            if (ImGui.Button("Advanced settings"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
                SettingsPanel.Open();
            ImGui.SetItemTooltip("edit and persist runtime-safe prediction, coherence, line-budget, cadence, and diagnostic settings"u8);
            ImGui.TableNextColumn();
            if (ImGui.Button("Diagnostics"u8,
                    new Brutal.Numerics.float2(ImGui.GetContentRegionAvail().X, 0f)))
                StatusPanel.Open();
            ImGui.SetItemTooltip("open n-body status, telemetry, and recent log lines"u8);
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    /// <summary>Window stepper tiers (seconds; labels in the window vocabulary) —
    /// hoisted, drawn every rendered frame.</summary>
    private static readonly (double Step, string Minus, string Plus)[] HorizonSteps =
        [(86400.0, "-1d", "+1d"), (30 * 86400.0, "-30d", "+30d"), (365 * 86400.0, "-1y", "+1y")];

    /// <summary>Orbits readout, recomposed at ~2 Hz wall (hoisted-constant convention:
    /// per-frame string interpolation is render-thread garbage; and the growth caption
    /// reads the rails horizon, a Gate acquisition that must not run per frame — during
    /// a chunked catch-up the worker holds that Gate most of every cycle).</summary>
    private static string _horizonReadout = "";
    private static long _horizonReadoutMs;
    private static TrajectoryComputationProgress? _trajectoryProgress;
    private static string _trajectoryProgressLabel = "";
    private static readonly string[] SpinnerFrames = ["|", "/", "-", "\u005C"];

    /// <summary>"Show orbits at least this far ahead": a free-form y/d/h/m/s duration field
    /// (the same DurationField the planner's time rows use) with ±1d/±30d/±1y
    /// steppers. Every commit goes through the kernel's ONE apply choreography
    /// (clamp into [1 d, 40 y], both display windows, the implicit rails move).
    /// The value is session-only and starts at 30 days on process launch.</summary>
    private static void DrawHorizonRow(ModConfig config, RailsService rails, FrameSpec? active)
    {
        long nowMs = Environment.TickCount64;
        if (ReadoutRefreshDue(_horizonReadoutMs, nowMs))
        {
            _horizonReadoutMs = nowMs;
            string readout = $"orbit minimum: {TimeDisplayKernel.FormatDuration(config.OverlayHorizonDays * 86400.0, years: true)}";
            double now = KSA.Universe.GetElapsedSimTime().Seconds();
            _trajectoryProgress = FrameSelectorKernel.FutureComputationProgress(
                rails.AvailableAheadDays(now), CelestialCurves.CompletedWindowDays(active?.Label),
                config.RailsAheadDays, config.OverlayHorizonDays, config.CelestialCurveDays,
                ModServices.MapTrajectory.ShowAstralBodyLines);
            _trajectoryProgressLabel = _trajectoryProgress?.CoverageLabel ?? "";
            if (KSA.Program.ControlledVehicle is { } vehicle
                && OverlayBuffer.ReadFresh(vehicle.Id, nowMs, now) is { } samples
                && FrameSelectorKernel.PointBudgetEffectiveDuration(
                    samples.Truncated, samples.WorkLimited, samples.DynamicsLimited,
                    samples.Markers.Any(marker => marker.Kind == OverlayMarkerKind.Collision),
                    samples.SampleT0, samples.DenseTimes[^1]) is { } effectiveSeconds)
            {
                readout += " — effective trajectory: "
                    + $"{TimeDisplayKernel.FormatDuration(effectiveSeconds, years: true)} (point budget)";
            }
            _horizonReadout = readout;
        }
        ImGui.Text(_horizonReadout);
        if (_trajectoryProgress is { } progress)
        {
            ImGui.Text(SpinnerFrames[(int)((nowMs / 125) % SpinnerFrames.Length)]);
            ImGui.SameLine(0f);
            ImGui.Text(progress.PhaseLabel);
            ImGui.ProgressBar(progress.Fraction, overlay: _trajectoryProgressLabel);
        }
        double windowSeconds = config.OverlayHorizonDays * 86400.0;
        if (!UiLayout.BeginProperties("##orbit-window-property"u8,
                UiTheme.PropertyLabelWidth)) return;
        try
        {
            UiLayout.NextProperty("Orbit look-ahead");
            if (DurationField.Row("##orbitswindow"u8, "orbits-window", 0,
                    ref windowSeconds, HorizonSteps, out string? parseError, years: true))
            {
                ApplyHorizon(config, windowSeconds / 86400.0);
                _horizonReadoutMs = 0;
            }
            if (parseError is not null) _status = parseError;
            ImGui.SetItemTooltip("minimum future extent for actual vessel lines; longer plans may extend the actual line, while planned lines stop at plan end; celestial arcs use this duration; type y/d/h/m/s (e.g. 2y 30d) or use the steppers; clamps to [1d, 40y]"u8);
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    internal static bool ReadoutRefreshDue(long lastRefreshMs, long nowMs) =>
        lastRefreshMs == 0 || nowMs < lastRefreshMs
        || nowMs - lastRefreshMs >= 500;

    /// <summary>Global celestial-line toggle. Vessel visibility is fixed separately
    /// to the controlled vessel only.</summary>
    private static void DrawLinePolicyRow()
    {
        bool showAstralBodyLines = ModServices.MapTrajectory.ShowAstralBodyLines;
        if (!UiLayout.BeginProperties("##astral-line-property"u8,
                UiTheme.PropertyLabelWidth)) return;
        try
        {
            UiLayout.NextProperty("Astral body paths");
            if (ImGui.Checkbox("Visible##astral-lines"u8, ref showAstralBodyLines))
            {
                ModServices.MapTrajectory.ShowAstralBodyLines = showAstralBodyLines;
                _status = showAstralBodyLines
                    ? "astral body lines shown"
                    : "astral body lines hidden";
            }
            ImGui.SetItemTooltip("show or hide every enabled astral body orbit line for this game; only the controlled vessel's lines are shown"u8);
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    /// <summary>Session-only Lagrange visualization control. It is meaningful only
    /// in a catalog body-pair fixed frame, where both masses and the normalized
    /// rotating-pulsating pose exist; other frames show no dead control.</summary>
    private static void DrawPotentialRow(FrameSpec? active)
    {
        if (!LagrangeOverlay.AvailableFor(active)) return;
        bool enabled = LagrangeOverlay.Enabled;
        if (!UiLayout.BeginProperties("##potential-line-property"u8,
                UiTheme.PropertyLabelWidth)) return;
        try
        {
            UiLayout.NextProperty("Potential contours");
            if (ImGui.Checkbox("Visible##potential-lines"u8, ref enabled))
                LagrangeOverlay.Enabled = enabled;
            ImGui.SetItemTooltip("show CR3BP effective-potential contours and the L1-L5 points for this fixed body pair; applies this session"u8);
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    private static void ApplyHorizon(ModConfig config, double days)
    {
        var applied = SettingsKernel.ApplyPrediction(config, days);
        if (applied.AppliedDays is not { } prediction) return;
        if (applied.RailsChangedTo is { } rails)
        {
            // The implicit computation horizon following the display window (both ways).
            ModLog.Info($"frames: rails horizon now {rails:F0} d (follows the "
                + $"{prediction:F0} d orbit display window)");
        }
        _status = $"orbits {TimeDisplayKernel.FormatDuration(prediction * 86400.0, years: true)} "
            + $"(session only; rails {TimeDisplayKernel.FormatDuration(config.RailsAheadDays * 86400.0, years: true)})";
    }
}

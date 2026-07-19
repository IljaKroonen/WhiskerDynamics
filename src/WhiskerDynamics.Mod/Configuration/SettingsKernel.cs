namespace WhiskerDynamics.Mod.Configuration;

/// <summary>A horizon edit's validated config write-back: the prediction value and,
/// when coupling requires it, the bounded rails value. Null means no rails write.</summary>
public readonly record struct HorizonEdit(double? PredictionDays, double? RailsDays);

/// <summary>What <see cref="SettingsKernel.ApplyPrediction"/> actually changed on the
/// live config: null AppliedDays = nothing written (non-finite input or a no-op
/// re-click); RailsChangedTo reports the implicit rails move for the caller's log.</summary>
public readonly record struct HorizonApply(double? AppliedDays, double? RailsChangedTo);

internal readonly record struct RuntimeSettingsApply(
    bool Changed, IReadOnlyList<ConfigRepair> Repairs);

internal readonly record struct RuntimeSettingsSnapshot(
    double RailsKeepBehindDays,
    double ConicDriftMeters,
    double OsculationRefreshSeconds,
    double CanaryToleranceMeters,
    double OverlayMaxTurnDeg,
    int OverlayMaxPoints,
    int CelestialMaxPoints,
    double FiniteBurnSliceSeconds,
    int FiniteBurnMaxSlices,
    int CelestialCurveMaxBodies,
    double MapPoseTelemetrySeconds)
{
    internal static RuntimeSettingsSnapshot Capture(ModConfig config) => new(
        config.RailsKeepBehindDays,
        config.ConicDriftMeters,
        config.OsculationRefreshSeconds,
        config.CanaryToleranceMeters,
        config.OverlayMaxTurnDeg,
        config.OverlayMaxPoints,
        config.CelestialMaxPoints,
        config.FiniteBurnSliceSeconds,
        config.FiniteBurnMaxSlices,
        config.CelestialCurveMaxBodies,
        config.MapPoseTelemetrySeconds);

    internal void ApplyTo(ModConfig config)
    {
        config.RailsKeepBehindDays = RailsKeepBehindDays;
        config.ConicDriftMeters = ConicDriftMeters;
        config.OsculationRefreshSeconds = OsculationRefreshSeconds;
        config.CanaryToleranceMeters = CanaryToleranceMeters;
        config.OverlayMaxTurnDeg = OverlayMaxTurnDeg;
        config.OverlayMaxPoints = OverlayMaxPoints;
        config.CelestialMaxPoints = CelestialMaxPoints;
        config.FiniteBurnSliceSeconds = FiniteBurnSliceSeconds;
        config.FiniteBurnMaxSlices = FiniteBurnMaxSlices;
        config.CelestialCurveMaxBodies = CelestialCurveMaxBodies;
        config.MapPoseTelemetrySeconds = MapPoseTelemetrySeconds;
    }
}

/// <summary>KSA-free decision rules for the orbit-display-length control (FramesPanel's
/// duration field is the thin ImGui adapter; these rules are the offline-tested part).
/// <see cref="EditPrediction"/> rejects NaN/infinity input and clamps the requested
/// prediction into [<see cref="MinPredictionDays"/>, <see cref="MaxPredictionDays"/>].
/// The rails horizon has no separate edit entry point: how far ahead positions are
/// COMPUTED is implicit — within the panel range it FOLLOWS the display window both
/// ways (raised to cover it, lowered back to max(prediction, <see cref="DefaultRailsDays"/>)
/// when the window shrinks, so one curious long-window click is not a permanent cost.
/// Every route is capped at <see cref="MaxRailsDays"/>.</summary>
public static class SettingsKernel
{
    /// <summary>1-day floor: below a day the celestial arcs degenerate to slivers and the
    /// vessel polyline to a stub. The session control must not disable the overlay by
    /// accident.</summary>
    public const double MinPredictionDays = 1.0;

    /// <summary>40-year ceiling (365-day display years — the panel's year vocabulary divides by
    /// 365): covers outer-planet mission spans (a Neptune round trip) while the costs
    /// stay bounded by construction rather than by this number. Rails memory is
    /// ~28 KB/day of committed knots (~400 MB at the full ceiling — the per-body
    /// quintic store, NBodyEphemerides), integration catch-up is chunked on the rails
    /// worker (~16 ms/day of horizon, readers stall per-chunk not per-catch-up, with
    /// display windows clamped to the rails horizon actually reached while it grows),
    /// and vessel prediction is point-budget bounded (fast orbits truncate honestly;
    /// TrajectoryPredictor's node ceiling refuses the pathological rest). Long-period
    /// bodies still show short honest arcs — the rails only ever hold what was
    /// integrated.</summary>
    public const double MaxPredictionDays = ModConfig.MaxWorkloadDays;

    /// <summary>Rails ceiling: the plan-length gate (FlightPlanModel.ValidateLength)
    /// and the display control both key off it. Equal to the display ceiling — a
    /// plan may span everything the map can show; both ride the same chunked-growth
    /// and clamp-to-reached-horizon machinery, so a huge value here changes budget,
    /// not stall behavior.</summary>
    public const double MaxRailsDays = ModConfig.MaxWorkloadDays;

    /// <summary>Default session horizon (days) and the machine-managed rails floor that
    /// the implicit rule returns to when the display window shrinks below it.</summary>
    public const double DefaultRailsDays = 30.0;

    /// <summary>User picked how far ahead orbits are shown. The clamped value drives
    /// BOTH session-only display windows. The implicit rails rule
    /// (celestial arcs clamp to the rails horizon; rails is not user-visible):
    /// a raw rails value within the panel range [0, <see cref="MaxPredictionDays"/>]
    /// follows the window to max(prediction,
    /// <see cref="DefaultRailsDays"/>) — raised so the setting visibly works, lowered
    /// back so a long-window click does not permanently ratchet background cost.
    /// Null RailsDays means no write.</summary>
    public static HorizonEdit EditPrediction(double rawRailsDays, double requestedDays)
    {
        if (!double.IsFinite(requestedDays)) return default;
        double p = Math.Clamp(requestedDays, MinPredictionDays, MaxPredictionDays);
        double rails = Math.Max(p, DefaultRailsDays);
        return new HorizonEdit(p, rails == rawRailsDays ? null : rails);
    }

    /// <summary>The ONE apply choreography for a horizon pick, on the LIVE shared
    /// ModConfig (mutated, never replaced — consumers re-read it every cycle, so this
    /// IS live-apply): the prediction drives both session-only display windows
    /// together, plus the implicit rails move when
    /// <see cref="EditPrediction"/> demands one. Callers that skip this and write
    /// `OverlayHorizonDays` alone would silently desynchronize vessel polylines from
    /// celestial arcs — hence a kernel method, not panel code. Returns what changed:
    /// null AppliedDays for non-finite input AND for a no-op (the values already in
    /// effect).</summary>
    public static HorizonApply ApplyPrediction(ModConfig config, double requestedDays)
    {
        var edit = EditPrediction(config.RailsAheadDays, requestedDays);
        if (edit.PredictionDays is not { } p) return default;
        if (p == config.OverlayHorizonDays && p == config.CelestialCurveDays
            && edit.RailsDays is null)
            return default; // no-op re-click: nothing to write, nothing to save
        config.OverlayHorizonDays = p;
        config.CelestialCurveDays = p;
        if (edit.RailsDays is { } rails) config.RailsAheadDays = rails;
        return new HorizonApply(p, edit.RailsDays);
    }

    internal static ModConfig CreateRuntimeDraft(ModConfig config)
    {
        var draft = new ModConfig();
        RuntimeSettingsSnapshot.Capture(config).ApplyTo(draft);
        return draft;
    }

    internal static RuntimeSettingsApply ApplyRuntimeSettings(
        ModConfig config, ModConfig requested)
    {
        var repairs = requested.NormalizeWorkload();
        var before = RuntimeSettingsSnapshot.Capture(config);
        var after = RuntimeSettingsSnapshot.Capture(requested);
        after.ApplyTo(config);
        return new RuntimeSettingsApply(before != after, repairs);
    }
}

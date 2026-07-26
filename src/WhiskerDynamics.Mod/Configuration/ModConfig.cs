using System.Globalization;
using Tomlet;
using Tomlet.Attributes;

namespace WhiskerDynamics.Mod.Configuration;

internal readonly record struct ConfigRepair(
    string TomlKey, string RawValue, string EffectiveValue, string Rule);

public sealed class ModConfig
{
    public const double MaxWorkloadDays = 14_600.0;
    internal const double SecondsPerDay = 86_400.0;
    internal const double MinOverlayTurnDegrees = 0.05;
    internal const double MaxOverlayTurnDegrees = 10.0;
    internal const double DefaultOverlayTurnDegrees = 0.4;
    internal const int MinFiniteBurnMaxSlices = 1;
    // Global planner/optimizer fidelity ceiling. The cheaper orbit-line fold has a
    // separate work cap so it cannot silently lower optimizer objective fidelity.
    internal const int MaxFiniteBurnMaxSlices = 1024;
    internal const int MaxOverlayFiniteBurnSlices = 128;
    // KSA.SimSpeed.MaxSpeed in the verified build. The drill calls the direct setter,
    // which does not perform the terminal command's own maximum-speed validation.
    internal const double MaxDrillWarpSpeed = 99_999_999_999_999.0;
    // Both wall-clock drill delay and telemetry eventually meet integer tick counts.
    internal const double MaxWallIntervalMilliseconds = int.MaxValue;
    internal const double MaxTelemetryIntervalSeconds = int.MaxValue / 1000.0;
    public const double DefaultCanaryToleranceMeters = 1.0;
    public const double MinCanaryToleranceMeters = 0.0;
    public const double MaxCanaryToleranceMeters = double.MaxValue;
    [TomlProperty("enabled")] public bool Enabled { get; set; } = true;
    // Physics. Mu values always come from the game's constants; these are tolerances.
    [TomlProperty("rails_rel_tol")] public double RailsRelTol { get; set; } = 1e-11;
    [TomlProperty("vessel_rel_tol")] public double VesselRelTol { get; set; } = 1e-11;
    // Session-only orbit horizons start at 30 days on every process launch. The panel
    // changes them together in memory; the rails window retains its 30-day floor.
    [TomlNonSerialized] public double RailsAheadDays { get; set; } = 30;
    [TomlProperty("rails_keep_behind_days")] public double RailsKeepBehindDays { get; set; } = 7;

    // FlightPlan coherence.
    [TomlProperty("conic_drift_meters")] public double ConicDriftMeters { get; set; } = 1000;
    [TomlProperty("osculation_refresh_seconds")] public double OsculationRefreshSeconds { get; set; } = 600;

    // Canary.
    [TomlProperty("canary_tolerance_meters")]
    public double CanaryToleranceMeters { get; set; } = DefaultCanaryToleranceMeters;

    // Overlay (honest-orbit-lines): angle-adaptive sampling — max turn per drawn
    // segment, in degrees. Point count is derived, capped at overlay_max_points.
    [TomlProperty("overlay_max_turn_deg")]
    public double OverlayMaxTurnDeg { get; set; } = DefaultOverlayTurnDegrees;
    // Honest-density VESSEL lines: the DRAWN polyline's point budget. The
    // dense sweep feeds a mod-owned OrbitLinePass draw (no stock 2000-point cap);
    // stock readers (hover, click payloads, ground track) keep a decimated 2000-point
    // buffer. Every segment uses overlay_max_turn_deg; the line truncates when this
    // budget is exhausted. Clamped to [2000, 262144].
    [TomlProperty("overlay_max_points")] public int OverlayMaxPoints { get; set; } = 65536;
    // Same budget for celestial arcs (fast moons under long windows) — smaller default:
    // the rails worker samples up to celestial_curve_max_bodies of these per cycle.
    // Clamped to [2000, 65536] at use.
    [TomlProperty("celestial_max_points")] public int CelestialMaxPoints { get; set; } = 8192;

    // Finite-burn estimation: planned burns fold as the flight
    // computer will fly them (rocket-equation duration, centered on the node, fixed
    // inertial direction — FiniteBurnKernel) discretized into one sub-impulse per
    // slice_seconds of thrust, capped at max_slices per burn. slice_seconds <= 0
    // disables the model (pure impulses).
    [TomlProperty("finite_burn_slice_seconds")] public double FiniteBurnSliceSeconds { get; set; } = 20;
    [TomlProperty("finite_burn_max_slices")] public int FiniteBurnMaxSlices { get; set; } = 32;
    // Adjustable for the current process through FramesPanel's Orbit look-ahead field.
    [TomlNonSerialized] public double OverlayHorizonDays { get; set; } = 30;

    // Celestial honest arcs: every selected modeled celestial's line is sampled from
    // its composite rails track, window clamped to the session rails horizon (multi-period arcs allowed —
    // no one-period clamp: successive revolutions show n-body precession). Drawn
    // whether or not a display frame is active. Point density comes from overlay_max_turn_deg.
    [TomlNonSerialized] public double CelestialCurveDays { get; set; } = 30;
    // Live catalog: dense systems (SolSystemDense) load hundreds of bodies; sampled arcs
    // cover the top-priority N (massive backbone first, then restricted tracks; µ
    // descending, ordinal-id ties). Remaining stock conics are display-only. Clamped >= 1.
    [TomlProperty("celestial_curve_max_bodies")] public int CelestialCurveMaxBodies { get; set; } = 128;

    // Save/load drills (verification scaffolding, default OFF): when a name is
    // non-empty and sim time crosses its threshold, the mod fires the game's own
    // save/load command ONCE (mid-session saves/loads cannot be scripted otherwise —
    // settings.toml onLoad console commands run at boot, before any tick).
    [TomlProperty("drill_save_name")] public string DrillSaveName { get; set; } = "";
    [TomlProperty("drill_save_at_seconds")] public double DrillSaveAtSeconds { get; set; } = 0;
    [TomlProperty("drill_load_name")] public string DrillLoadName { get; set; } = "";
    [TomlProperty("drill_load_at_seconds")] public double DrillLoadAtSeconds { get; set; } = 0;
    // Scripted warp (verification scaffolding): speed > 0 fires the game's own
    // SetSimulationSpeed once, drill_warp_delay_ms of WALL clock after the load drill
    // (or after the first UI draw when no load drill is configured).
    [TomlProperty("drill_warp_speed")] public double DrillWarpSpeed { get; set; } = 0;
    [TomlProperty("drill_warp_delay_ms")] public double DrillWarpDelayMs { get; set; } = 3000;

    // Resilience drills. Runtime compatibility is decided by the concrete API,
    // enum, and patch-activation checks in ModMain rather than an exact build string.
    [TomlProperty("simulate_mismatch")] public bool SimulateMismatch { get; set; } = false;
    [TomlProperty("simulate_type_drift")] public bool SimulateTypeDrift { get; set; } = false;
    [TomlProperty("fault_inject_vessel")] public string FaultInjectVessel { get; set; } = "";

    // Planner write drill (verification scaffolding, default off): when non-empty,
    // "offsetSeconds,prograde,normal,outward" adds ONE burn via BurnPlanWriter at
    // now+offset on the first panel frame with a resolvable controlled vessel.
    [TomlProperty("drill_planner_burn")] public string DrillPlannerBurn { get; set; } = "";

    // Follow-coherence telemetry (verification scaffolding, default off):
    // when > 0, MapFramePatch logs the followed target's view-space drift (vs the stock
    // pose) and the accumulated frame angle every N wall-seconds. This is the observable
    // that catches vessel-follow drift an Earth-follow check cannot see (Earth IS
    // the frame origin, where rotating about the wrong point is invisible).
    [TomlProperty("map_pose_telemetry_seconds")] public double MapPoseTelemetrySeconds { get; set; } = 0;

    public static ModConfig LoadOrCreate(string path) => LoadOrCreate(path, hooks: null);

    internal static ModConfig LoadOrCreate(string path, AtomicTextFileHooks? hooks)
    {
        try
        {
            if (File.Exists(path))
            {
                var loaded = TomletMain.To<ModConfig>(File.ReadAllText(path));
                LogRepairs(loaded.NormalizeWorkload(), "load");
                return loaded;
            }
            var fresh = new ModConfig();
            AtomicTextFile.WriteAllText(path, TomletMain.TomlStringFrom(fresh), hooks);
            return fresh;
        }
        catch (Exception e)
        {
            ModLog.Error($"config load failed, using defaults: {e.Message}");
            return new ModConfig();
        }
    }

    internal ConfigRepair[] NormalizeWorkload()
    {
        var repairs = new List<ConfigRepair>();
        NormalizePhysics(repairs);
        NormalizeDisplayAndHistory(repairs);
        NormalizeFiniteBurnAndDisplay(repairs);
        NormalizeVerificationScaffolding(repairs);
        return [.. repairs];
    }

    private void NormalizePhysics(List<ConfigRepair> repairs)
    {
        // Physics values are repaired before consumers observe the shared config.
        RailsRelTol = Repair(repairs, "rails_rel_tol", RailsRelTol,
            FiniteClamp(RailsRelTol, 1e-14, 1e-6, 1e-11),
            "finite [1e-14, 1e-6], non-finite uses 1e-11");
        VesselRelTol = Repair(repairs, "vessel_rel_tol", VesselRelTol,
            FiniteClamp(VesselRelTol, 1e-14, 1e-6, 1e-11),
            "finite [1e-14, 1e-6], non-finite uses 1e-11");
        RailsKeepBehindDays = Repair(repairs, "rails_keep_behind_days", RailsKeepBehindDays,
            FiniteClamp(RailsKeepBehindDays, 0, 3650, 7),
            "finite [0, 3650], non-finite uses 7");
        ConicDriftMeters = Repair(repairs, "conic_drift_meters", ConicDriftMeters,
            FiniteNonNegative(ConicDriftMeters, 1000),
            "finite >= 0, non-finite uses 1000");
        OsculationRefreshSeconds = Repair(repairs, "osculation_refresh_seconds",
            OsculationRefreshSeconds,
            FiniteClamp(OsculationRefreshSeconds, 1, 86400, 600),
            "finite [1, 86400], non-finite uses 600");
        CanaryToleranceMeters = Repair(repairs, "canary_tolerance_meters",
            CanaryToleranceMeters, NormalizeCanaryTolerance(CanaryToleranceMeters),
            "finite >= 0, non-finite uses 1");
    }

    private void NormalizeDisplayAndHistory(List<ConfigRepair> repairs)
    {
        OverlayMaxTurnDeg = Repair(repairs, "overlay_max_turn_deg", OverlayMaxTurnDeg,
            FiniteClamp(OverlayMaxTurnDeg, MinOverlayTurnDegrees, MaxOverlayTurnDegrees,
                DefaultOverlayTurnDegrees),
            "finite [0.05, 10], non-finite uses 0.4");
        OverlayMaxPoints = Repair(repairs, "overlay_max_points", OverlayMaxPoints,
            Math.Clamp(OverlayMaxPoints, 2000, 262144), "integer [2000, 262144]");
        CelestialMaxPoints = Repair(repairs, "celestial_max_points", CelestialMaxPoints,
            Math.Clamp(CelestialMaxPoints, 2000, 65536), "integer [2000, 65536]");
    }

    private void NormalizeFiniteBurnAndDisplay(List<ConfigRepair> repairs)
    {
        FiniteBurnSliceSeconds = Repair(repairs, "finite_burn_slice_seconds",
            FiniteBurnSliceSeconds, FiniteClamp(FiniteBurnSliceSeconds, 0, 3600, 20),
            "finite [0, 3600], non-finite uses 20, zero disables the model");
        FiniteBurnMaxSlices = Repair(repairs, "finite_burn_max_slices", FiniteBurnMaxSlices,
            Math.Clamp(FiniteBurnMaxSlices, MinFiniteBurnMaxSlices, MaxFiniteBurnMaxSlices),
            "integer [1, 1024], finite_burn_slice_seconds is the model-off switch");
        CelestialCurveMaxBodies = Repair(repairs, "celestial_curve_max_bodies",
            CelestialCurveMaxBodies, Math.Clamp(CelestialCurveMaxBodies, 1, 256),
            "integer [1, 256]");
    }

    private void NormalizeVerificationScaffolding(List<ConfigRepair> repairs)
    {
        DrillSaveAtSeconds = Repair(repairs, "drill_save_at_seconds", DrillSaveAtSeconds,
            FiniteNonNegative(DrillSaveAtSeconds, 0),
            "finite >= 0, non-finite uses 0 (off)");
        DrillLoadAtSeconds = Repair(repairs, "drill_load_at_seconds", DrillLoadAtSeconds,
            FiniteNonNegative(DrillLoadAtSeconds, 0),
            "finite >= 0, non-finite uses 0 (off)");
        DrillWarpSpeed = Repair(repairs, "drill_warp_speed", DrillWarpSpeed,
            FiniteClamp(DrillWarpSpeed, 0, MaxDrillWarpSpeed, 0),
            $"finite [0, {Format(MaxDrillWarpSpeed)}], non-finite uses 0 (off)");
        DrillWarpDelayMs = Repair(repairs, "drill_warp_delay_ms", DrillWarpDelayMs,
            FiniteClamp(DrillWarpDelayMs, 0, MaxWallIntervalMilliseconds, 3000),
            $"finite [0, {Format(MaxWallIntervalMilliseconds)}], non-finite uses 3000");
        MapPoseTelemetrySeconds = Repair(repairs, "map_pose_telemetry_seconds",
            MapPoseTelemetrySeconds,
            FiniteClamp(MapPoseTelemetrySeconds, 0, MaxTelemetryIntervalSeconds, 0),
            $"finite [0, {Format(MaxTelemetryIntervalSeconds)}], non-finite uses 0 (off)");
    }

    internal static void LogRepairs(IReadOnlyList<ConfigRepair> repairs, string context)
    {
        if (repairs.Count == 0) return;
        foreach (var repair in repairs)
            ModLog.Warn($"config repair ({context}) {repair.TomlKey}: " +
                $"{repair.RawValue} -> {repair.EffectiveValue} ({repair.Rule})");
        ModLog.Warn($"config {context}: {repairs.Count} numeric value(s) repaired in memory; " +
            "repairs apply this session and whiskerdynamics.toml was not rewritten");
    }

    private static T Repair<T>(List<ConfigRepair> repairs, string key,
        T raw, T effective, string rule)
    {
        if (!EqualityComparer<T>.Default.Equals(raw, effective))
            repairs.Add(new ConfigRepair(key, Format(raw), Format(effective), rule));
        return effective;
    }

    private static string Format<T>(T value) => value switch
    {
        double d when double.IsNaN(d) => "nan",
        double d when double.IsPositiveInfinity(d) => "inf",
        double d when double.IsNegativeInfinity(d) => "-inf",
        double d => d.ToString("R", CultureInfo.InvariantCulture),
        IFormattable formattable => formattable.ToString(null, CultureInfo.InvariantCulture),
        _ => value?.ToString() ?? "null",
    };

    private static double FiniteClamp(double value, double min, double max, double fallback) =>
        double.IsFinite(value) ? Math.Clamp(value, min, max) : fallback;

    private static double FiniteNonNegative(double value, double fallback) =>
        double.IsFinite(value) ? Math.Max(0, value) : fallback;

    private static double NormalizeCanaryTolerance(double value) =>
        double.IsFinite(value)
        && value >= MinCanaryToleranceMeters
        && value <= MaxCanaryToleranceMeters
            ? value
            : DefaultCanaryToleranceMeters;
}

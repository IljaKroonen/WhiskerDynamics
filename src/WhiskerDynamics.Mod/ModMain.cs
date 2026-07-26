using HarmonyLib;
using WhiskerDynamics.Compatibility;
using WhiskerDynamics.Compatibility.Patching;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.Mod;

public static class ModMain
{
    public static string ModDir { get; private set; } = "";

    // SOI-shim panel/telemetry state. The telemetry kernel re-arms itself on
    // sim-time REGRESSIONS; forward jumps and per-session one-shot re-arming are
    // handled by the session statics sweep (ResetSessionStatics below, invoked on
    // every rebind/save load). _soiNextLogMs is a wall-clock throttle — no reset.
    private static readonly DominantAttractorTelemetry SoiTelemetry = new(sustainSeconds: 60.0);
    private static bool _soiLineLogged;
    private static long _soiNextLogMs;
    // Planner drill one-shot (drills fire once per process — deliberately NOT
    // re-armed by the statics sweep).
    private static bool _plannerDrillFired;
    private static string _plannerDrillStatus = "";

    /// <summary>Session statics sweep (called via ModServices.ResetSessionStatics on
    /// every rebind/save load): re-arm the SOI telemetry episode state and the one-shot
    /// panel-evidence line for the new session.</summary>
    internal static void ResetSessionStatics()
    {
        SoiTelemetry.Reset();
        _soiLineLogged = false;
    }

    public static void EarlyInit(string modDir)
    {
        // Capture the main managed thread id FIRST (before any
        // early return — StarMap's BeforeMain shim runs on the process main thread, the
        // same thread that runs Program.Main and every ImGui draw phase). From here on,
        // BurnPlanWriter rejects any wrong-thread mutation with a panel-visible string.
        BurnPlanWriter.CaptureMainThread();
        ModDir = modDir;
        ModLog.Init(Path.Combine(modDir, "whiskerdynamics.log"));
        var gameVersion = typeof(KSA.Program).Assembly.GetName().Version?.ToString() ?? "unknown";
        ModLog.Info($"Whisker Dynamics alive; game build {gameVersion}; mod dir {modDir}");

        ModServices.Config = ModConfig.LoadOrCreate(Path.Combine(modDir, "whiskerdynamics.toml"));
        if (!ModServices.Config.Enabled)
        {
            ModServices.Status = ModStatus.DisabledByUser;
            ModLog.Info("disabled by config");
            return;
        }

        var harmony = new Harmony("ksa.whiskerdynamics");

        // The panel renders the disabled notice, so validate + apply it first, independently.
        // PanelTargets resolves KSA.Program members only; if even KSA.Program is gone,
        // EarlyInit itself fails to JIT and the entry shim's fail-closed path is correct.
        var panelPatchSet = PanelPatchSet.Create();
        bool panelOk = PatchValidator.ValidateAll(panelPatchSet.Targets, out var panelMismatches);
        if (panelOk) HarmonyPatchActivation.Apply(harmony, panelPatchSet.PatchTypes[0]);
        else foreach (var m in panelMismatches) ModLog.Error($"panel target moved: {m}");

        try
        {
            string bodySettingsPath = Path.Combine(modDir, "body-settings");
            ModServices.BodySettings = BodySettingsCatalog.LoadDirectory(bodySettingsPath);
            ModLog.Info($"loaded {ModServices.BodySettings.Entries.Count} body settings "
                + $"from {bodySettingsPath}");
        }
        catch (Exception e)
        {
            ModServices.Mismatches = [e.Message];
            ModServices.Status = ModStatus.DisabledIncompatible;
            ModLog.Error($"body settings catalog failed to load; running stock: {e}");
            return;
        }

        if (!GameBuildPolicy.IsVerified(gameVersion))
        {
            ModLog.Warn($"running KSA {gameVersion}; this mod build was verified against "
                + $"KSA {GameBuildPolicy.VerifiedBuild}. Continuing with compatibility checks; "
                + "behavioral compatibility is not guaranteed");
        }

        // Graceful degradation on type-level drift: GameplayTargets is a separate static
        // class whose FIRST touch happens inside this try — if this game build dropped a
        // registered gameplay type entirely, its static initializer throws here
        // (TypeInitializationException) instead of dying to the entry shim, and the
        // already-applied panel shows the DisabledIncompatible notice.
        PatchSetDefinition gameplayPatchSet;
        try
        {
            if (ModServices.Config.SimulateTypeDrift)
                throw new TypeInitializationException("WhiskerDynamics.Compatibility.Patching.GameplayTargets",
                    new MissingMethodException("SIMULATED type-level drift drill"));

            gameplayPatchSet = GameplayPatchSet.Create();
            var gameplaySpecs = gameplayPatchSet.Targets.AsEnumerable();
            if (ModServices.Config.SimulateMismatch)
                gameplaySpecs = gameplaySpecs.Append(new TargetSpec(
                    "SIMULATED.Mismatch", typeof(KSA.Program), "MethodThatDoesNotExist", MemberKind.Method));

            if (!PatchValidator.ValidateAll(gameplaySpecs, out var mismatches))
            {
                ModServices.Mismatches = mismatches;
                ModServices.Status = ModStatus.DisabledIncompatible;
                foreach (var m in mismatches) ModLog.Error($"target moved: {m}");
                ModLog.Error("n-body disabled: game build not yet supported (no gameplay patches applied)");
                return;
            }

            if (!EnumContract.Validate(out var enumContractMismatches))
                throw new MissingMemberException(enumContractMismatches[0]);

            // Force-JIT every patch method NOW: a member referenced inside a patch body
            // that no longer exists in this game build throws here (catchable), instead
            // of at first invocation inside a patched game method (uncatchable there).
            HarmonyPatchActivation.ApplyAndWarm(harmony, gameplayPatchSet.PatchTypes);

            // Boot-time diagnostic: the game's own constants as this build declares them
            // (their existence as const doubles was validated just above; these same values
            // flow into the rails via MassConstants at bind time).
            var gc = GameConstants.ReadFromGame();
            ModLog.Info($"game constants: G={gc.G:R}, solar={gc.SolarMassKg:R} kg, earth={gc.EarthMassKg:R} kg, "
                + $"lunar={gc.LunarMassKg:R} kg, jupiter={gc.JupiterMassKg:R} kg");
        }
        catch (Exception e)
        {
            harmony.UnpatchAll("ksa.whiskerdynamics");
            if (panelOk) HarmonyPatchActivation.Apply(
                harmony, panelPatchSet.PatchTypes[0]); // keep the notice visible
            ModServices.Mismatches = [e.Message];
            ModServices.Status = ModStatus.DisabledIncompatible;
            ModLog.Error($"gameplay validation/patch application failed — gameplay patches removed, running stock: {e}");
            ModLog.Error("n-body disabled: game build not yet supported (no gameplay patches applied)");
            return;
        }
        ModLog.Info($"applied {gameplayPatchSet.PatchTypes.Count} gameplay patch classes (JIT-warmed)");

        Ui.StatusPanel.ExtraLines.Add(() =>
        [
            $"rails: {(ModServices.Rails is { } r ? $"horizon {r.Horizon / 86400.0:F1} d" : "unbound")}",
            $"celestial overrides: {System.Threading.Interlocked.Read(ref Patches.CelestialRailsPatch.OverrideCount)}",
        ]);
        Ui.StatusPanel.ExtraLines.Add(() => ModServices.Vessels?.Describe() ?? []);
        Ui.StatusPanel.ExtraLines.Add(() =>
            [$"live third-body |delta a|: {Patches.LiveGravityPatch.LastDeltaMagnitude:E2} m/s^2"]);
        Ui.StatusPanel.ExtraLines.Add(() => [TrajectoryOverlay.LastNote]);
        // (VesselLinePatch stages at the draw site; per-vessel publish evidence lives
        // in TrajectoryOverlay.LastNote's "N vessel(s) published" and the one-shot
        // "vessel line takeover active" log line.)

        // SOI dominant-attractor shim — informational only. Stock SOI machinery
        // keeps running, while SoiHandoffPatch re-anchors live cross-parent mirrors
        // to the rails state after stock performs a handoff.
        // Every game member touched here is an existing registry entry, validated above
        // before this provider is registered. The try/catch is the never-throw contract:
        // an unknown parent makes GetAbsolute throw, and an
        // uncontained provider would burn StatusPanel's 3-strike budget and kill the
        // whole panel.
        Ui.StatusPanel.ExtraLines.Add(() =>
        {
            try
            {
                var vehicle = KSA.Program.ControlledVehicle;
                var rails = ModServices.Rails;
                if (vehicle is null || rails is null
                    || vehicle.Orbit?.Parent is not KSA.Astronomical parentBody)
                    return [];
                // Sample parent and vessel at the committed state's own epoch: at high
                // warp GetElapsedSimTime leads StateTime by up to a tick (~1667 s at
                // 1e5x), which alone would displace the parent by ~5e7 m.
                double t = vehicle.Orbit.StateVectors.StateTime.Seconds();
                if (!rails.TryGetAbsolute(parentBody.Id, t, out var parentAbs)) return [];
                var cci2Cce = vehicle.Orbit.Parent.GetCci2Cce();
                var absolute = parentAbs.Position
                    + FrameAdapter.CciToEcl(vehicle.Orbit.StateVectors.PositionCci, cci2Cce);
                if (!DominantAttractor.TryCompute(rails, absolute, t, out string dominant))
                    return [];

                if (!_soiLineLogged)
                {
                    // One-shot in-game evidence the line rendered (whiskerdynamics.log is the observable).
                    _soiLineLogged = true;
                    ModLog.Info($"dominant-attractor panel line active (vessel '{vehicle.Id}': "
                        + $"stock parent {parentBody.Id}, dominant attractor {dominant})");
                }
                string? telemetry = SoiTelemetry.Observe(parentBody.Id, dominant, t);
                if (telemetry is not null && Environment.TickCount64 >= _soiNextLogMs)
                {
                    // 5 s-wall floor: boundary flapping at extreme warp must not flood
                    // the log (dropped episode lines are informational-only).
                    _soiNextLogMs = Environment.TickCount64 + 5000;
                    ModLog.Info(telemetry);
                }
                return [$"controlled: stock parent {parentBody.Id}, dominant attractor {dominant}"];
            }
            catch (Exception e)
            {
                if (Environment.TickCount64 >= _soiNextLogMs)
                {
                    _soiNextLogMs = Environment.TickCount64 + 5000;
                    ModLog.Warn($"dominant-attractor line contained: {e.Message}");
                }
                return [];
            }
        });

        // Planner drill: fires once per process from the panel-draw phase (main-thread
        // UI — the exact phase the writer requires). Config-gated, default off.
        Ui.StatusPanel.ExtraLines.Add(() =>
        {
            try
            {
                if (_plannerDrillFired)
                    return _plannerDrillStatus.Length == 0 ? [] : [_plannerDrillStatus];
                string raw = ModServices.Config.DrillPlannerBurn;
                if (raw.Length == 0) return [];
                if (!PlannerDrillKernel.TryParse(raw, out var drill, out string parseError))
                {
                    _plannerDrillFired = true;
                    _plannerDrillStatus = "planner drill rejected: " + parseError;
                    ModLog.Warn(_plannerDrillStatus);
                    return [_plannerDrillStatus];
                }
                if (!ModServices.EnsureBound(out var services)) return [];
                var vessels = services.Vessels;
                if (KSA.Program.ControlledVehicle is not { } controlled) return [];
                if (vessels.TryGetLiveVehicle(controlled.Id) is not { } vehicle) return [];
                _plannerDrillFired = true;
                var dv = PlannerKernel.ComposeVlf(
                    drill.Prograde, drill.Normal, drill.Outward);
                double now = KSA.Universe.GetElapsedSimTime().Seconds();
                string result = BurnPlanWriter.TryAdd(vehicle, now + drill.OffsetSeconds, dv);
                _plannerDrillStatus = $"planner drill: TryAdd at now+{drill.OffsetSeconds:F0} s -> {result}";
                ModLog.Info(_plannerDrillStatus);
                return [_plannerDrillStatus];
            }
            catch (Exception e)
            {
                _plannerDrillFired = true;
                _plannerDrillStatus = $"planner drill contained: {e.Message}";
                ModLog.Warn(_plannerDrillStatus);
                return [_plannerDrillStatus];
            }
        });

        ModServices.Status = ModStatus.WaitingForSystem;
        ModServices.Enabled = true;
    }
}

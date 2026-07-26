using WhiskerDynamics.Compatibility;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Configuration;

/// <summary>Tests the session-only orbit-display length decisions and independent
/// whiskerdynamics.toml persistence.</summary>
public class SettingsKernelTests
{
    [Fact]
    public void Workload_normalization_repairs_nonfinite_and_caps_extremes()
    {
        var config = new ModConfig
        {
            RailsKeepBehindDays = -10,
            RailsRelTol = double.NaN,
            OverlayMaxPoints = int.MaxValue,
            CelestialCurveMaxBodies = int.MaxValue,
        };

        config.NormalizeWorkload();

        Assert.Equal(0, config.RailsKeepBehindDays);
        Assert.Equal(1e-11, config.RailsRelTol);
        Assert.Equal(262144, config.OverlayMaxPoints);
        Assert.Equal(256, config.CelestialCurveMaxBodies);
    }

    [Fact]
    public void Canary_tolerance_default_is_the_named_safe_default()
    {
        Assert.Equal(ModConfig.DefaultCanaryToleranceMeters,
            new ModConfig().CanaryToleranceMeters);
    }

    [Theory]
    [InlineData(double.NaN, ModConfig.DefaultCanaryToleranceMeters)]
    [InlineData(double.PositiveInfinity, ModConfig.DefaultCanaryToleranceMeters)]
    [InlineData(double.NegativeInfinity, ModConfig.DefaultCanaryToleranceMeters)]
    [InlineData(-1.0, ModConfig.DefaultCanaryToleranceMeters)]
    [InlineData(double.MaxValue, ModConfig.MaxCanaryToleranceMeters)]
    [InlineData(0.0, 0.0)]
    [InlineData(0.5, 0.5)]
    public void Workload_normalization_repairs_and_clamps_canary_tolerance(
        double raw, double expected)
    {
        var config = new ModConfig { CanaryToleranceMeters = raw };

        config.NormalizeWorkload();

        Assert.Equal(expected, config.CanaryToleranceMeters);
        Assert.True(double.IsFinite(config.CanaryToleranceMeters));
        Assert.InRange(config.CanaryToleranceMeters,
            ModConfig.MinCanaryToleranceMeters, ModConfig.MaxCanaryToleranceMeters);
    }

    // --- EditPrediction: clamp + garbage rejection ---

    [Theory]
    [InlineData(0.0, 1.0)]        // floor: 0 must not become a hidden overlay off switch
    [InlineData(-5.0, 1.0)]
    [InlineData(0.5, 1.0)]
    [InlineData(20_000.0, 14_600.0)] // ceiling: 40 display-years
    [InlineData(180.0, 180.0)]    // in range passes through
    [InlineData(3650.0, 3650.0)]  // the long presets sit inside the range
    public void Prediction_clamps_to_the_preset_range(double requested, double expected)
    {
        var edit = SettingsKernel.EditPrediction(rawRailsDays: SettingsKernel.MaxRailsDays, requested);
        Assert.Equal(expected, edit.PredictionDays);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Prediction_rejects_non_finite_input_with_no_writes(double garbage)
    {
        Assert.Equal(default, SettingsKernel.EditPrediction(90, garbage));
    }

    // --- EditPrediction: rails coupling ---

    [Fact]
    public void Raising_prediction_above_rails_auto_raises_rails_to_match()
    {
        // Celestial arcs clamp to the rails horizon — without the auto-raise, raising
        // prediction past rails would silently do nothing visible (looks broken).
        var edit = SettingsKernel.EditPrediction(rawRailsDays: 30, 60);
        Assert.Equal(60, edit.PredictionDays);
        Assert.Equal(60, edit.RailsDays);
    }

    [Fact]
    public void Lowering_prediction_lowers_machine_managed_rails_with_it()
    {
        // The implicit rule is two-way within the preset range: rails at 90 can only be
        // an earlier auto-raise (presets reach exactly [1, 365]), so shrinking the
        // window takes the background-integration cost back down instead of ratcheting.
        var edit = SettingsKernel.EditPrediction(rawRailsDays: 90, 10);
        Assert.Equal(10, edit.PredictionDays);
        Assert.Equal(SettingsKernel.DefaultRailsDays, edit.RailsDays); // floored at the default
    }

    [Fact]
    public void One_curious_year_click_is_not_a_permanent_ratchet()
    {
        // Returning from 1y to 30d must lower the implicitly coupled rails horizon.
        var up = SettingsKernel.EditPrediction(rawRailsDays: 30, 365);
        Assert.Equal(365, up.RailsDays);
        var down = SettingsKernel.EditPrediction(rawRailsDays: 365, 30);
        Assert.Equal(30, down.PredictionDays);
        Assert.Equal(30, down.RailsDays);
    }

    [Fact]
    public void Rails_already_right_yields_no_write()
    {
        var edit = SettingsKernel.EditPrediction(rawRailsDays: 30, 10);
        Assert.Equal(10, edit.PredictionDays);
        Assert.Null(edit.RailsDays); // max(10, default 30) == raw 30: nothing to write
    }

    [Fact]
    public void Default_rails_mirrors_the_config_default()
        // The kernel's machine-managed floor and the runtime default stay aligned.
        => Assert.Equal(SettingsKernel.DefaultRailsDays, new ModConfig().RailsAheadDays);

    [Fact]
    public void Prediction_at_ceiling_never_pushes_rails_past_its_own_ceiling()
    {
        // The two ceilings are equal by design (a plan may span everything the map can
        // show), so the coupling at the top writes rails to exactly the ceiling.
        var edit = SettingsKernel.EditPrediction(rawRailsDays: 30, SettingsKernel.MaxPredictionDays);
        Assert.Equal(SettingsKernel.MaxPredictionDays, edit.PredictionDays);
        Assert.Equal(SettingsKernel.MaxRailsDays, edit.RailsDays);
        Assert.True(SettingsKernel.MaxPredictionDays <= SettingsKernel.MaxRailsDays);
    }

    // --- EditPrediction: inconsistent runtime rails values ---

    [Fact]
    public void Prediction_edit_repairs_a_rails_value_above_the_hard_cap()
    {
        var edit = SettingsKernel.EditPrediction(rawRailsDays: SettingsKernel.MaxPredictionDays + 500, 60);
        Assert.Equal(60, edit.PredictionDays);
        Assert.Equal(60, edit.RailsDays);
    }

    [Fact]
    public void Prediction_edit_raises_rails_below_the_new_prediction()
    {
        var edit = SettingsKernel.EditPrediction(rawRailsDays: 0.5, 30);
        Assert.Equal(30, edit.PredictionDays);
        Assert.Equal(30, edit.RailsDays);
    }

    [Fact]
    public void Prediction_edit_repairs_a_non_finite_rails_value_to_the_new_prediction()
    {
        // A NaN rails value can never satisfy rails >= prediction, so the coupling
        // forces a finite repair rather than leaving garbage in effect.
        var edit = SettingsKernel.EditPrediction(rawRailsDays: double.NaN, 60);
        Assert.Equal(60, edit.PredictionDays);
        Assert.Equal(60, edit.RailsDays);
    }

    // --- ApplyPrediction: the ONE apply choreography over the live config ---

    [Fact]
    public void Apply_drives_both_display_windows_and_the_implicit_rails_together()
    {
        // One user concept: writing OverlayHorizonDays alone would silently
        // desynchronize vessel polylines from celestial arcs.
        var config = new ModConfig(); // overlay 30, celestial 30, rails 30
        var applied = SettingsKernel.ApplyPrediction(config, 100);
        Assert.Equal(100.0, applied.AppliedDays);
        Assert.Equal(100.0, applied.RailsChangedTo);
        Assert.Equal(100, config.OverlayHorizonDays);
        Assert.Equal(100, config.CelestialCurveDays);
        Assert.Equal(100, config.RailsAheadDays);
    }

    [Fact]
    public void Apply_reclick_of_the_active_value_is_a_no_op()
    {
        var config = new ModConfig();
        Assert.Equal(default, SettingsKernel.ApplyPrediction(config, 30));
        Assert.Equal(30, config.RailsAheadDays);
    }

    [Fact]
    public void Apply_lowers_a_stale_ratcheted_rails_even_when_the_windows_match()
    {
        // Reapplying a preset must repair a mismatched rails horizon.
        var config = new ModConfig { RailsAheadDays = 365 };
        var applied = SettingsKernel.ApplyPrediction(config, 30);
        Assert.Equal(30.0, applied.AppliedDays);
        Assert.Equal(30.0, applied.RailsChangedTo);
        Assert.Equal(30, config.RailsAheadDays);
    }

    [Fact]
    public void Apply_repairs_a_diverged_celestial_window()
    {
        // A preset click restores the one-concept rule for both windows.
        var config = new ModConfig { CelestialCurveDays = 5 };
        var applied = SettingsKernel.ApplyPrediction(config, 30);
        Assert.Equal(30.0, applied.AppliedDays);
        Assert.Equal(30, config.CelestialCurveDays);
    }

    [Fact]
    public void Apply_rejects_non_finite_input_with_no_writes()
    {
        var config = new ModConfig();
        Assert.Equal(default, SettingsKernel.ApplyPrediction(config, double.NaN));
        Assert.Equal(30, config.OverlayHorizonDays);
    }

    [Fact]
    public void Runtime_settings_apply_every_live_advanced_value_without_touching_startup_or_horizon_values()
    {
        var config = new ModConfig
        {
            Enabled = true,
            RailsRelTol = 1e-11,
            OverlayHorizonDays = 75,
            CelestialCurveDays = 75,
        };
        var requested = new ModConfig
        {
            Enabled = false,
            RailsRelTol = 1e-7,
            OverlayHorizonDays = 400,
            CelestialCurveDays = 400,
            RailsKeepBehindDays = 12,
            ConicDriftMeters = 250,
            OsculationRefreshSeconds = 120,
            CanaryToleranceMeters = 0.25,
            OverlayMaxTurnDeg = 0.25,
            OverlayMaxPoints = 40000,
            CelestialMaxPoints = 12000,
            FiniteBurnSliceSeconds = 10,
            FiniteBurnMaxSlices = 64,
            CelestialCurveMaxBodies = 200,
            MapPoseTelemetrySeconds = 2,
        };

        var applied = SettingsKernel.ApplyRuntimeSettings(config, requested);

        Assert.True(applied.Changed);
        Assert.Empty(applied.Repairs);
        Assert.Equal(RuntimeSettingsSnapshot.Capture(requested),
            RuntimeSettingsSnapshot.Capture(config));
        Assert.True(config.Enabled);
        Assert.Equal(1e-11, config.RailsRelTol);
        Assert.Equal(75, config.OverlayHorizonDays);
        Assert.Equal(75, config.CelestialCurveDays);
    }

    [Fact]
    public void Runtime_settings_apply_normalizes_the_draft_before_publishing_it()
    {
        var config = new ModConfig();
        var requested = SettingsKernel.CreateRuntimeDraft(config);
        requested.RailsKeepBehindDays = -1;
        requested.OverlayMaxTurnDeg = 20;
        requested.OverlayMaxPoints = int.MaxValue;
        requested.CelestialCurveMaxBodies = 0;

        var applied = SettingsKernel.ApplyRuntimeSettings(config, requested);

        Assert.True(applied.Changed);
        Assert.Equal(4, applied.Repairs.Count);
        Assert.Equal(0, config.RailsKeepBehindDays);
        Assert.Equal(10, config.OverlayMaxTurnDeg);
        Assert.Equal(262144, config.OverlayMaxPoints);
        Assert.Equal(1, config.CelestialCurveMaxBodies);
    }

    // --- Persistence ---

    [Fact]
    public void Horizon_values_start_at_30_days_and_are_not_persisted()
    {
        string dir = Path.Combine(Path.GetTempPath(), "whisker-dynamics-settings-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            File.WriteAllText(path,
                $"overlay_horizon_days = 120{Environment.NewLine}"
                + $"celestial_curve_days = 120{Environment.NewLine}"
                + $"rails_ahead_days = 240{Environment.NewLine}");
            var config = ModConfig.LoadOrCreate(path);

            Assert.Equal(30, config.OverlayHorizonDays);
            Assert.Equal(30, config.CelestialCurveDays);
            Assert.Equal(30, config.RailsAheadDays);
            SettingsKernel.ApplyPrediction(config, 120);

            Assert.True(SettingsPersistence.TrySave(config, path, out string error));
            Assert.Equal("", error);

            var reloaded = ModConfig.LoadOrCreate(path);
            Assert.Equal(30, reloaded.OverlayHorizonDays);
            Assert.Equal(30, reloaded.CelestialCurveDays);
            Assert.Equal(30, reloaded.RailsAheadDays);
            string saved = File.ReadAllText(path);
            Assert.DoesNotContain("overlay_horizon_days", saved);
            Assert.DoesNotContain("celestial_curve_days", saved);
            Assert.DoesNotContain("rails_ahead_days", saved);
        }
        finally
        {
            try { Directory.Delete(dir, recursive: true); } catch { /* best effort */ }
        }
    }

    [Fact]
    public void Failed_save_reports_the_error_instead_of_throwing()
    {
        // Parent directory does not exist -> the write throws inside TrySave, which must
        // contain it (never-throw contract) and hand the message back for the panel.
        string path = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-settings-missing-" + Guid.NewGuid().ToString("N"), "whiskerdynamics.toml");
        Assert.False(SettingsPersistence.TrySave(new ModConfig(), path, out string error));
        Assert.NotEqual("", error);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Failed_atomic_save_preserves_valid_toml_and_a_retry_recovers(
        bool failCommit)
    {
        string dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-settings-atomic-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            byte[] original = System.Text.Encoding.UTF8.GetBytes(
                $"enabled = false{Environment.NewLine}rails_keep_behind_days = 45{Environment.NewLine}");
            File.WriteAllBytes(path, original);
            var replacement = new ModConfig
            {
                Enabled = true,
                RailsKeepBehindDays = 240,
            };
            const string injected = "injected settings commit failure";
            var hooks = new AtomicTextFileHooks(
                AfterTempFlushedAndClosed: failCommit
                    ? null
                    : (_, _) => throw new IOException(injected),
                Commit: failCommit
                    ? (_, _, _) => throw new IOException(injected)
                    : null);

            Assert.False(SettingsPersistence.TrySave(
                replacement, path, hooks, out string error));

            Assert.Contains(injected, error);
            Assert.Equal(original, File.ReadAllBytes(path));
            Assert.Equal([path], Directory.GetFiles(dir));

            Assert.True(SettingsPersistence.TrySave(replacement, path, out error));
            Assert.Equal("", error);
            Assert.Equal([path], Directory.GetFiles(dir));
            ModConfig reloaded = ModConfig.LoadOrCreate(path);
            Assert.True(reloaded.Enabled);
            Assert.Equal(240, reloaded.RailsKeepBehindDays);
        }
        finally
        {
            try { Directory.Delete(dir, recursive: true); } catch { /* best effort */ }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Failed_first_boot_create_returns_defaults_without_a_partial_file(
        bool failCommit)
    {
        string dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-settings-first-boot-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            const string injected = "injected first-boot commit failure";
            var hooks = new AtomicTextFileHooks(
                AfterTempFlushedAndClosed: failCommit
                    ? null
                    : (_, _) => throw new IOException(injected),
                Commit: failCommit
                    ? (_, _, _) => throw new IOException(injected)
                    : null);

            ModConfig fallback = ModConfig.LoadOrCreate(path, hooks);

            Assert.Equal(new ModConfig().RailsAheadDays, fallback.RailsAheadDays);
            Assert.Equal(new ModConfig().Enabled, fallback.Enabled);
            Assert.False(File.Exists(path));
            Assert.Empty(Directory.GetFiles(dir));

            ModConfig recovered = ModConfig.LoadOrCreate(path);
            Assert.Equal(new ModConfig().RailsAheadDays, recovered.RailsAheadDays);
            Assert.True(File.Exists(path));
            Assert.Equal([path], Directory.GetFiles(dir));
            Assert.Equal(recovered.RailsAheadDays,
                ModConfig.LoadOrCreate(path).RailsAheadDays);
        }
        finally
        {
            try { Directory.Delete(dir, recursive: true); } catch { /* best effort */ }
        }
    }

    [Fact]
    public void Embedded_game_build_policy_accepts_only_the_verified_build()
    {
        Assert.Equal("2026.7.9.5018", GameBuildPolicy.VerifiedBuild);
        Assert.True(GameBuildPolicy.IsVerified(GameBuildPolicy.VerifiedBuild));
        Assert.False(GameBuildPolicy.IsVerified("2026.7.5.4892"));
    }
}

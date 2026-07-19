using System.Globalization;
using Tomlet.Attributes;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Tests.Diagnostics;

namespace WhiskerDynamics.Mod.Tests.Configuration;

[Collection(nameof(ModLogTestCollection))]
public class ModConfigTests
{
    [Fact]
    public void Lunar_gravity_defaults_to_degree30_gameplay_fidelity()
    {
        var config = new ModConfig();

        Assert.Equal("degree30", config.LunarGravityModel);
        Assert.Equal(LunarGravityFidelity.Degree30, config.SelectedLunarGravityFidelity);
        Assert.Empty(config.NormalizeWorkload());
    }

    [Theory]
    [InlineData("degree10", "degree10", LunarGravityFidelity.Degree10, false)]
    [InlineData(" degree20 ", "degree20", LunarGravityFidelity.Degree20, true)]
    [InlineData("degree30", "degree30", LunarGravityFidelity.Degree30, false)]
    [InlineData(" degree40 ", "degree40", LunarGravityFidelity.Degree40, true)]
    [InlineData("DEGREE50", "degree50", LunarGravityFidelity.Degree50, true)]
    [InlineData("unknown", "degree30", LunarGravityFidelity.Degree30, true)]
    public void Lunar_gravity_config_is_normalized_and_mapped(
        string raw, string expected, LunarGravityFidelity fidelity, bool repaired)
    {
        var config = new ModConfig { LunarGravityModel = raw };

        ConfigRepair[] repairs = config.NormalizeWorkload();

        Assert.Equal(expected, config.LunarGravityModel);
        Assert.Equal(fidelity, config.SelectedLunarGravityFidelity);
        Assert.Equal(repaired, repairs.Any(repair => repair.TomlKey == "lunar_gravity_model"));
        Assert.Empty(config.NormalizeWorkload());
    }

    [Fact]
    public void Lunar_gravity_model_round_trips_through_toml_persistence()
    {
        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            var config = new ModConfig { LunarGravityModel = "degree20" };

            Assert.True(SettingsPersistence.TrySave(config, path, out string error), error);
            ModConfig loaded = ModConfig.LoadOrCreate(path);

            Assert.Equal("degree20", loaded.LunarGravityModel);
            Assert.Equal(LunarGravityFidelity.Degree20,
                loaded.SelectedLunarGravityFidelity);
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Null_lunar_gravity_model_is_repaired_to_degree30()
    {
        var config = new ModConfig { LunarGravityModel = null! };

        ConfigRepair repair = Assert.Single(config.NormalizeWorkload(),
            candidate => candidate.TomlKey == "lunar_gravity_model");

        Assert.Equal("degree30", config.LunarGravityModel);
        Assert.Equal(LunarGravityFidelity.Degree30, config.SelectedLunarGravityFidelity);
        Assert.Equal("", repair.RawValue);
        Assert.Equal("degree30", repair.EffectiveValue);
    }

    [Fact]
    public void Repairs_are_structured_exact_and_idempotent()
    {
        var config = new ModConfig
        {
            RailsKeepBehindDays = -1.5,
            ConicDriftMeters = double.NaN,
            OverlayMaxTurnDeg = 45,
            FiniteBurnMaxSlices = 0,
            MapPoseTelemetrySeconds = double.NegativeInfinity,
        };

        ConfigRepair[] repairs = config.NormalizeWorkload();
        var byKey = repairs.ToDictionary(repair => repair.TomlKey);

        Assert.Equal(5, repairs.Length);
        Assert.Equal("-1.5", byKey["rails_keep_behind_days"].RawValue);
        Assert.Equal("0", byKey["rails_keep_behind_days"].EffectiveValue);
        Assert.Equal("nan", byKey["conic_drift_meters"].RawValue);
        Assert.Equal("1000", byKey["conic_drift_meters"].EffectiveValue);
        Assert.Equal("10", byKey["overlay_max_turn_deg"].EffectiveValue);
        Assert.Equal("1", byKey["finite_burn_max_slices"].EffectiveValue);
        Assert.Equal("-inf", byKey["map_pose_telemetry_seconds"].RawValue);
        Assert.All(repairs, repair => Assert.NotEqual("", repair.Rule));
        Assert.Empty(config.NormalizeWorkload());
    }

    [Fact]
    public void Repair_values_are_invariant_under_non_English_culture()
    {
        CultureInfo before = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");
            var config = new ModConfig { RailsKeepBehindDays = -1.5 };

            ConfigRepair repair = Assert.Single(config.NormalizeWorkload());

            Assert.Equal("-1.5", repair.RawValue);
            Assert.Equal("0", repair.EffectiveValue);
        }
        finally
        {
            CultureInfo.CurrentCulture = before;
        }
    }

    [Fact]
    public void Documented_zero_values_remain_valid()
    {
        var config = new ModConfig
        {
            RailsKeepBehindDays = 0,
            ConicDriftMeters = 0,
            CanaryToleranceMeters = 0,
            FiniteBurnSliceSeconds = 0,
            DrillSaveAtSeconds = 0,
            DrillLoadAtSeconds = 0,
            DrillWarpSpeed = 0,
            DrillWarpDelayMs = 0,
            MapPoseTelemetrySeconds = 0,
        };

        Assert.Empty(config.NormalizeWorkload());
        Assert.Equal(0, config.RailsKeepBehindDays);
        Assert.Equal(0, config.CanaryToleranceMeters);
        Assert.Equal(0, config.FiniteBurnSliceSeconds);
    }

    [Fact]
    public void Every_numeric_property_repairs_nonfinite_or_minimum_integer_input()
    {
        var config = new ModConfig();
        var doubles = typeof(ModConfig).GetProperties()
            .Where(property => property.PropertyType == typeof(double)
                && !Attribute.IsDefined(property, typeof(TomlNonSerializedAttribute)))
            .ToArray();
        var integers = typeof(ModConfig).GetProperties()
            .Where(property => property.PropertyType == typeof(int)
                && !Attribute.IsDefined(property, typeof(TomlNonSerializedAttribute)))
            .ToArray();
        foreach (var property in doubles) property.SetValue(config, double.NaN);
        foreach (var property in integers) property.SetValue(config, int.MinValue);

        ConfigRepair[] repairs = config.NormalizeWorkload();

        Assert.Equal(doubles.Length + integers.Length, repairs.Length);
        Assert.All(doubles, property =>
            Assert.True(double.IsFinite((double)property.GetValue(config)!),
                property.Name));
        Assert.All(integers, property =>
            Assert.True((int)property.GetValue(config)! > 0, property.Name));
        Assert.Empty(config.NormalizeWorkload());
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 1)]
    [InlineData(128, 128)]
    [InlineData(129, 129)]
    [InlineData(1024, 1024)]
    [InlineData(1025, 1024)]
    public void Config_finite_burn_slice_cap_preserves_optimizer_fidelity(
        int raw, int expected)
    {
        var config = new ModConfig { FiniteBurnMaxSlices = raw };

        config.NormalizeWorkload();

        Assert.Equal(expected, config.FiniteBurnMaxSlices);
        Assert.InRange(config.FiniteBurnMaxSlices,
            ModConfig.MinFiniteBurnMaxSlices, ModConfig.MaxFiniteBurnMaxSlices);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 1)]
    [InlineData(128, 128)]
    [InlineData(129, 128)]
    [InlineData(1024, 128)]
    public void Overlay_has_a_distinct_finite_burn_work_cap(int configured, int expected)
    {
        Assert.Equal(expected, OverlayKernel.OverlayFiniteBurnMaxSlices(configured));
        Assert.Equal(128, ModConfig.MaxOverlayFiniteBurnSlices);
        Assert.Equal(1024, ModConfig.MaxFiniteBurnMaxSlices);
    }

    [Fact]
    public void Central_limits_match_sampling_and_history_consumers()
    {
        var config = new ModConfig { OverlayMaxTurnDeg = 45 };

        config.NormalizeWorkload();
        double thetaMax = OverlayKernel.SamplingThetaRadians(45);
        var history = FlownHistorySettings.Default;

        Assert.Equal(ModConfig.MaxOverlayTurnDegrees, config.OverlayMaxTurnDeg);
        Assert.Equal(ModConfig.MaxOverlayTurnDegrees * Math.PI / 180, thetaMax);
        Assert.Equal(ModConfig.MaxWorkloadDays * ModConfig.SecondsPerDay,
            history.RetentionSeconds);
    }

    [Fact]
    public void Overlay_defaults_use_fixed_point_four_degree_quality_and_65k_points()
    {
        var config = new ModConfig();

        Assert.Equal(0.4, config.OverlayMaxTurnDeg);
        Assert.Equal(65536, config.OverlayMaxPoints);
    }

    [Fact]
    public void Startup_surfaces_each_repair_and_a_no_rewrite_summary()
    {
        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            string original = $"rails_keep_behind_days = -123.25{Environment.NewLine}";
            File.WriteAllText(path, original);

            ModConfig config = ModConfig.LoadOrCreate(path);
            IReadOnlyList<string> log = ModLog.Snapshot();

            Assert.Equal(0, config.RailsKeepBehindDays);
            Assert.Equal(original, File.ReadAllText(path));
            Assert.Contains(log, line => line.Contains(
                "config repair (load) rails_keep_behind_days: -123.25 -> 0"));
            Assert.Contains(log, line => line.Contains(
                "repairs apply this session and whiskerdynamics.toml was not rewritten"));
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Retired_periapsis_gate_key_loads_inert_and_is_omitted_from_next_save()
    {
        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            string original = $"enabled = false{Environment.NewLine}"
                + $"max_integrated_periapsis_mps = 12345{Environment.NewLine}";
            File.WriteAllText(path, original);

            ModConfig config = ModConfig.LoadOrCreate(path);

            Assert.False(config.Enabled);
            Assert.Equal(original, File.ReadAllText(path));
            Assert.True(SettingsPersistence.TrySave(config, path, out string error), error);
            Assert.DoesNotContain("max_integrated_periapsis_mps", File.ReadAllText(path));
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Removed_configuration_keys_are_not_migrated_or_persisted()
    {
        Assert.Null(typeof(ModConfig).GetProperty("HistoryRetentionDays"));
        Assert.Null(typeof(ModConfig).GetProperty("HistorySampleSeconds"));
        Assert.Null(typeof(ModConfig).GetProperty("HistoryMaxPoints"));
        Assert.Null(typeof(ModConfig).GetProperty("HistoryDisplayDays"));
        Assert.Null(typeof(ModConfig).GetProperty("ShowAstralBodyLines"));
        Assert.Null(typeof(ModConfig).GetProperty("BurnNodeScale"));

        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            string original = $"history_retention_days = 12{Environment.NewLine}"
                + $"history_sample_seconds = 5{Environment.NewLine}"
                + $"history_max_points = 1234{Environment.NewLine}"
                + $"history_display_days = 90{Environment.NewLine}"
                + $"show_astral_body_lines = false{Environment.NewLine}"
                + $"burn_node_scale = 0.5{Environment.NewLine}";
            File.WriteAllText(path, original);

            ModConfig config = ModConfig.LoadOrCreate(path);

            Assert.True(SettingsPersistence.TrySave(config, path, out string error), error);
            string saved = File.ReadAllText(path);
            Assert.DoesNotContain("history_retention_days", saved);
            Assert.DoesNotContain("history_sample_seconds", saved);
            Assert.DoesNotContain("history_max_points", saved);
            Assert.DoesNotContain("history_display_days", saved);
            Assert.DoesNotContain("show_astral_body_lines", saved);
            Assert.DoesNotContain("burn_node_scale", saved);
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Startup_hardens_verification_numeric_values_without_rewriting()
    {
        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            string original = $"drill_save_at_seconds = -1{Environment.NewLine}"
                + $"drill_load_at_seconds = nan{Environment.NewLine}"
                + $"drill_warp_speed = inf{Environment.NewLine}"
                + $"drill_warp_delay_ms = -inf{Environment.NewLine}"
                + $"map_pose_telemetry_seconds = inf{Environment.NewLine}";
            File.WriteAllText(path, original);

            ModConfig config = ModConfig.LoadOrCreate(path);

            Assert.Equal(0, config.DrillSaveAtSeconds);
            Assert.Equal(0, config.DrillLoadAtSeconds);
            Assert.Equal(0, config.DrillWarpSpeed);
            Assert.Equal(3000, config.DrillWarpDelayMs);
            Assert.Equal(0, config.MapPoseTelemetrySeconds);
            Assert.Equal(original, File.ReadAllText(path));
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Malformed_toml_preserves_the_existing_default_fallback()
    {
        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            string original = $"rails_keep_behind_days = nope{Environment.NewLine}";
            File.WriteAllText(path, original);

            ModConfig config = ModConfig.LoadOrCreate(path);

            Assert.Equal(new ModConfig().RailsKeepBehindDays, config.RailsKeepBehindDays);
            Assert.Equal(original, File.ReadAllText(path));
            Assert.Contains(ModLog.Snapshot(), line =>
                line.Contains("config load failed, using defaults"));
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Theory]
    [InlineData("-10", 0.0)]
    [InlineData("0", 0.0)]
    [InlineData("nan", 7.0)]
    [InlineData("inf", 7.0)]
    [InlineData("-inf", 7.0)]
    public void Startup_repairs_retention_without_rewriting_the_source(
        string literal, double expected)
    {
        string dir = NewTempDirectory();
        try
        {
            string path = Path.Combine(dir, "whiskerdynamics.toml");
            string original = $"enabled = false{Environment.NewLine}"
                + $"rails_keep_behind_days = {literal}{Environment.NewLine}";
            File.WriteAllText(path, original);

            var config = ModConfig.LoadOrCreate(path);

            Assert.False(config.Enabled);
            Assert.Equal(expected, config.RailsKeepBehindDays);
            Assert.Equal(original, File.ReadAllText(path));

            const double now = 40 * ModConfig.SecondsPerDay;
            const double future = 60 * ModConfig.SecondsPerDay;
            var (ephemerides, earth) = ExtendedEphemeris(future);
            ephemerides.Prune(
                RailsService.RetentionCutoffSeconds(now, config.RailsKeepBehindDays));

            Assert.True(ephemerides.StartTime <= now,
                $"repaired {literal} retained start {ephemerides.StartTime:R} past now {now:R}");
            StateVector present = ephemerides.GetState(earth, now);
            Assert.True(double.IsFinite(present.Position.X));

            // A negative value advances the cutoff by ten days and strands the present.
            if (literal == "-10")
            {
                var (unsafeEphemerides, unsafeEarth) = ExtendedEphemeris(future);
                unsafeEphemerides.Prune(RailsService.RetentionCutoffSeconds(now, -10));
                Assert.True(unsafeEphemerides.StartTime > now);
                Assert.Throws<ArgumentOutOfRangeException>(
                    () => unsafeEphemerides.GetState(unsafeEarth, now));
            }
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Production_retention_cutoff_uses_backward_days()
    {
        const double now = 40 * ModConfig.SecondsPerDay;

        Assert.Equal(now, RailsService.RetentionCutoffSeconds(now, 0));
        Assert.Equal(39 * ModConfig.SecondsPerDay,
            RailsService.RetentionCutoffSeconds(now, 1));
    }

    private static (NBodyEphemerides Ephemerides, CelestialBody Earth)
        ExtendedEphemeris(double horizon)
    {
        var sun = new CelestialBody { Id = "Sun", Mu = 1.32712440018e20 };
        var earth = new CelestialBody
        {
            Id = "Earth",
            Mu = 3.986004418e14,
            Parent = sun,
            Orbit = new OrbitalElements(
                1.49598023e11, 0.0167086, 0, 0, 1.79676742, 0),
        };
        var ephemerides = new NBodyEphemerides(
            [sun, earth], 0, ["Sun", "Earth"],
            new IntegratorOptions { RelTol = 1e-11 });
        ephemerides.GetState(earth, horizon);
        return (ephemerides, earth);
    }

    private static string NewTempDirectory()
    {
        string dir = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-config-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        return dir;
    }
}

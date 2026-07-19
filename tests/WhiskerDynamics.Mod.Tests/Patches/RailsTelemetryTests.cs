using WhiskerDynamics.Mod.Patches;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.Mod.Tests.Patches;

/// <summary>
/// Tests the patch-side telemetry classifier: one epoch line per body followed by
/// wall-clock-throttled drift lines.
/// </summary>
public class RailsTelemetryTests
{
    [Fact]
    public void First_override_of_a_body_is_the_epoch_line()
    {
        var t = new RailsTelemetry(driftPeriodMs: 30_000);
        Assert.Equal(RailsTelemetry.Line.Epoch, t.Classify("Earth", nowMs: 1000));
    }

    [Fact]
    public void Second_override_inside_the_throttle_window_logs_nothing()
    {
        var t = new RailsTelemetry(driftPeriodMs: 30_000);
        t.Classify("Earth", 1000);
        Assert.Equal(RailsTelemetry.Line.None, t.Classify("Earth", 1001));
        Assert.Equal(RailsTelemetry.Line.None, t.Classify("Earth", 30_999));
    }

    [Fact]
    public void Override_after_the_throttle_window_is_a_drift_line_then_throttles_again()
    {
        var t = new RailsTelemetry(driftPeriodMs: 30_000);
        t.Classify("Earth", 1000);
        Assert.Equal(RailsTelemetry.Line.Drift, t.Classify("Earth", 31_000));
        Assert.Equal(RailsTelemetry.Line.None, t.Classify("Earth", 31_001));
        Assert.Equal(RailsTelemetry.Line.Drift, t.Classify("Earth", 61_000));
    }

    [Fact]
    public void Bodies_are_throttled_independently()
    {
        var t = new RailsTelemetry(driftPeriodMs: 30_000);
        Assert.Equal(RailsTelemetry.Line.Epoch, t.Classify("Earth", 1000));
        Assert.Equal(RailsTelemetry.Line.Epoch, t.Classify("Luna", 2000));
        Assert.Equal(RailsTelemetry.Line.Drift, t.Classify("Earth", 31_000));
        Assert.Equal(RailsTelemetry.Line.None, t.Classify("Luna", 31_000));
        Assert.Equal(RailsTelemetry.Line.Drift, t.Classify("Luna", 32_000));
    }

    [Fact]
    public void Reset_rearms_the_epoch_lines()
    {
        // Reset re-arms the per-body epoch line after a rebind.
        var t = new RailsTelemetry(driftPeriodMs: 30_000);
        Assert.Equal(RailsTelemetry.Line.Epoch, t.Classify("Earth", 1000));
        Assert.Equal(RailsTelemetry.Line.None, t.Classify("Earth", 1001));
        t.Reset();
        Assert.Equal(RailsTelemetry.Line.Epoch, t.Classify("Earth", 1002));
    }

    [Fact]
    public void Concurrent_first_overrides_yield_exactly_one_epoch_line_per_body()
    {
        // Concurrent callers must emit only one epoch line per body.
        var t = new RailsTelemetry(driftPeriodMs: 30_000);
        int epochLines = 0;
        Parallel.For(0, 64, _ =>
        {
            if (t.Classify("Earth", 1000) == RailsTelemetry.Line.Epoch)
                Interlocked.Increment(ref epochLines);
        });
        Assert.Equal(1, epochLines);
    }
}

/// <summary>Pins the registration step: the celestial rails patch must stay listed
/// in the gameplay patch set (applied inside the guarded try only after ALL
/// gameplay targets validate). Reflection-only on mod types — no KSA type is loaded.</summary>
public class ModMainRegistrationTests
{
    [Fact]
    public void CelestialRailsPatch_is_registered_as_a_gameplay_patch()
    {
        Assert.Contains(typeof(CelestialRailsPatch), GameplayPatchSet.PatchTypes);
    }
}

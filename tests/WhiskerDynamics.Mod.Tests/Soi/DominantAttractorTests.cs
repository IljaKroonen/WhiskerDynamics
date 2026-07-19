using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Soi;

/// <summary>Tests point-mass dominant-attractor selection.</summary>
public sealed class DominantAttractorTests : IDisposable
{
    private readonly string _dir;
    private readonly RailsService _rails;

    public DominantAttractorTests()
    {
        _dir = Path.Combine(Path.GetTempPath(), "whisker-dynamics-dominant-tests-" + Guid.NewGuid().ToString("N"));
        var xmlDir = Path.Combine(_dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));
        var config = new ModConfig { RailsAheadDays = 2 };
        var constants = new GameConstants(6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        _rails = TestRailsService.FromFixture(config, constants);
        _rails.NoteSimTime(10_000);
        Assert.True(SpinWait.SpinUntil(() => _rails.IsReadyAt(10_000), 5000));
        _rails.NoteSimTime(10_000);
        Assert.True(SpinWait.SpinUntil(() => _rails.IsReadyAt(10_000), 5000));
    }

    public void Dispose()
    {
        _rails.Dispose();
        try { Directory.Delete(_dir, recursive: true); } catch { /* best effort */ }
    }

    [Fact]
    public void Near_a_planet_the_planet_dominates()
    {
        double t = 1000.0;
        var mercury = _rails.GetAbsolute("Mercury", t).Position;
        Assert.Equal("Mercury",
            DominantAttractor.Compute(_rails, mercury + new Vector3d(2.74e6, 0, 0), t));
    }

    [Fact]
    public void Near_the_star_the_star_dominates()
    {
        double t = 1000.0;
        var sol = _rails.GetAbsolute("Sol", t).Position;
        Assert.Equal("Sol",
            DominantAttractor.Compute(_rails, sol + new Vector3d(1e9, 0, 0), t));
    }

    [Fact]
    public void Flip_happens_at_the_analytic_field_balance_radius()
    {
        // mu_M/r² = mu_S/(d-r)² balances at r = d*x/(1+x), x = sqrt(mu_M/mu_S).
        double t = 1000.0;
        var sol = _rails.GetAbsolute("Sol", t).Position;
        var mercury = _rails.GetAbsolute("Mercury", t).Position;
        var toSol = sol - mercury;
        double d = toSol.Length();
        var u = toSol / d;
        double x = Math.Sqrt(_rails.MuOf("Mercury") / _rails.MuOf("Sol"));
        double balance = d * x / (1 + x);
        Assert.Equal("Mercury", DominantAttractor.Compute(_rails, mercury + u * (0.99 * balance), t));
        Assert.Equal("Sol", DominantAttractor.Compute(_rails, mercury + u * (1.01 * balance), t));
    }

    [Fact]
    public void Mutually_coupled_gravity_sources_are_candidates()
    {
        double t = 1000.0;
        var probe = _rails.GetAbsolute("TestComet", t).Position + new Vector3d(100, 0, 0);
        Assert.True(_rails.IsModeled("TestComet"));
        Assert.True(_rails.IsBackbone("TestComet"));
        Assert.Contains(_rails.VesselGravity.Sources, b => b.Id == "TestComet");
        Assert.Equal("TestComet", DominantAttractor.Compute(_rails, probe, t));
    }

    [Fact]
    public void Exactly_at_a_body_position_the_singular_body_is_skipped()
    {
        // A singular body is skipped.
        double t = 1000.0;
        var mercury = _rails.GetAbsolute("Mercury", t).Position;
        Assert.Equal("Sol", DominantAttractor.Compute(_rails, mercury, t));
    }

    [Fact]
    public void Large_all_mutual_status_refresh_batches_once_and_sixty_hz_cache_hits_never_touch_the_gate()
    {
        const int sourceCount = 99;
        var bodies = DenseCatalog(sourceCount);
        using var rails = RailsService.CreateForSyntheticCatalog(
            bodies, backboneIds: bodies.Select(body => body.Id).ToArray());
        Assert.Equal(sourceCount, rails.VesselGravity.Sources.Count);

        int batchGateEntries = 0;
        rails.AbsoluteManyGateEnteredForTest = () =>
        {
            Assert.True(Monitor.IsEntered(rails.Gate));
            batchGateEntries++;
        };

        var cache = new StatusTelemetryCache(refreshIntervalMs: 500);
        const double time = 0.0;
        var target = bodies[^1];
        string expectedLine = $"dominant attractor {target.Id}";
        IReadOnlyList<string> Refresh()
        {
            // Match the production provider: its parent lookup is a separate Gate
            // read, followed by the complete source set's single folded read.
            Assert.True(rails.TryGetAbsolute(target.Id, time, out var parentState));
            var probe = parentState.Position + new Vector3d(1.0, 0.0, 0.0);
            Assert.True(DominantAttractor.TryCompute(
                rails, probe, time, out string dominant));
            Assert.Equal(target.Id, dominant);
            return [$"dominant attractor {dominant}"];
        }

        var initial = cache.Read(nowMs: 0, Refresh);
        Assert.Equal([expectedLine], initial);
        Assert.Equal(1, batchGateEntries);

        // Rendered frames before the 500 ms human-visible deadline reuse the exact
        // snapshot; none invokes the parent read or folded source read.
        for (int frame = 1; frame < 30; frame++)
            Assert.Same(initial, cache.Read(frame * 1_000L / 60, Refresh));
        Assert.Equal(1, batchGateEntries);

        var next = cache.Read(nowMs: 500, Refresh);
        Assert.Equal([expectedLine], next);
        Assert.NotSame(initial, next);
        Assert.Equal(2, batchGateEntries);
    }

    private static IReadOnlyList<CelestialBody> DenseCatalog(int sourceCount)
    {
        var root = new CelestialBody
        {
            Id = "DenseRoot",
            Mu = 1.0e20,
        };
        var bodies = new List<CelestialBody>(sourceCount) { root };
        for (int i = 1; i < sourceCount; i++)
        {
            double radius = 1.0e9 + i * 1.0e7;
            bodies.Add(new CelestialBody
            {
                Id = $"Dense{i:D3}",
                Mu = 1.0e10 + i,
                Parent = root,
                Orbit = new OrbitalElements(
                    SemiMajorAxis: radius,
                    Eccentricity: 0.01,
                    Inclination: 0.001 * (i % 5),
                    LongitudeOfAscendingNode: 0.01 * (i % 17),
                    ArgumentOfPeriapsis: 0.02 * (i % 13),
                    TimeAtPeriapsis: 0.0),
            });
        }
        return bodies;
    }
}

/// <summary>Tests sustained dominant-attractor disagreement episodes.</summary>
public class DominantAttractorTelemetryTests
{
    [Fact]
    public void Agreement_never_logs()
    {
        var telemetry = new DominantAttractorTelemetry(60.0);
        for (double t = 0; t <= 600; t += 100)
            Assert.Null(telemetry.Observe("Earth", "Earth", t));
    }

    [Fact]
    public void Short_disagreement_never_logs_and_resolves_silently()
    {
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 0.0));
        Assert.Null(telemetry.Observe("Luna", "Earth", 59.0));
        Assert.Null(telemetry.Observe("Luna", "Luna", 60.0)); // never reported: no closing line
    }

    [Fact]
    public void Sustained_disagreement_logs_once_then_resolution_logs_once()
    {
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 100.0));
        Assert.Null(telemetry.Observe("Luna", "Earth", 130.0));
        string? sustained = telemetry.Observe("Luna", "Earth", 160.0);
        Assert.NotNull(sustained);
        Assert.Contains("Earth", sustained);
        Assert.Contains("Luna", sustained);
        Assert.Null(telemetry.Observe("Luna", "Earth", 300.0)); // once per episode
        string? resolved = telemetry.Observe("Luna", "Luna", 400.0);
        Assert.NotNull(resolved);
        Assert.Contains("Luna", resolved);
        Assert.Null(telemetry.Observe("Luna", "Luna", 500.0)); // resolution logs once too
    }

    [Fact]
    public void A_new_episode_after_agreement_logs_again()
    {
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 0.0));
        Assert.NotNull(telemetry.Observe("Luna", "Earth", 60.0));
        Assert.NotNull(telemetry.Observe("Luna", "Luna", 100.0)); // resolved
        Assert.Null(telemetry.Observe("Luna", "Earth", 200.0));   // new episode arms fresh
        Assert.NotNull(telemetry.Observe("Luna", "Earth", 260.0));
    }

    [Fact]
    public void Changing_the_disagreeing_pair_restarts_the_clock()
    {
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 0.0));
        Assert.Null(telemetry.Observe("Luna", "Sol", 61.0));  // different pair: new episode
        Assert.Null(telemetry.Observe("Luna", "Sol", 90.0));  // only 29 s into THIS episode
        Assert.NotNull(telemetry.Observe("Luna", "Sol", 121.0));
    }

    [Fact]
    public void Sim_time_regression_restarts_the_episode()
    {
        // A save load can jump sim time backwards; the kernel must re-arm, not report
        // a bogus "sustained since the future" episode.
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 1000.0));
        Assert.Null(telemetry.Observe("Luna", "Earth", 500.0)); // regression: restart
        Assert.Null(telemetry.Observe("Luna", "Earth", 559.0));
        Assert.NotNull(telemetry.Observe("Luna", "Earth", 560.0));
    }

    [Fact]
    public void Reset_rearms_mid_episode_so_forward_time_jumps_never_splice()
    {
        // A save load can also jump sim time
        // FORWARD — the statics sweep resets the kernel so the pre-load episode never splices
        // into the post-load one ("sustained 100000 s" from two unrelated sessions).
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 0.0));
        Assert.Null(telemetry.Observe("Luna", "Earth", 30.0));
        telemetry.Reset();
        Assert.Null(telemetry.Observe("Luna", "Earth", 100000.0)); // fresh arm, no splice
        Assert.Null(telemetry.Observe("Luna", "Earth", 100059.0)); // only 59 s into THIS episode
        string? line = telemetry.Observe("Luna", "Earth", 100061.0);
        Assert.NotNull(line);
        Assert.Contains("t=100000.0 s", line); // episode began at the post-reset time
    }

    [Fact]
    public void Reset_of_a_reported_episode_emits_no_orphan_closing_line()
    {
        var telemetry = new DominantAttractorTelemetry(60.0);
        Assert.Null(telemetry.Observe("Luna", "Earth", 0.0));
        Assert.NotNull(telemetry.Observe("Luna", "Earth", 60.0)); // reported
        telemetry.Reset();
        Assert.Null(telemetry.Observe("Luna", "Luna", 100.0)); // agreement after reset: silent
    }
}

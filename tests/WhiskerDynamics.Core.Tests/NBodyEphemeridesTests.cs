using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class NBodyEphemeridesTests
{
    private const double MuSun = 1.32712440018e20;
    private const double MuEarth = 3.986004418e14;
    private const double MuMoon = 4.9028e12;

    private static (CelestialBody sun, CelestialBody earth, CelestialBody moon) SunEarthMoon()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var earth = new CelestialBody
        {
            Id = "Earth", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.49598023e11, 0.0167086, 0, 0, 1.79676742, 0),
        };
        var moon = new CelestialBody
        {
            Id = "Moon", Mu = MuMoon, Parent = earth,
            Orbit = new OrbitalElements(3.844e8, 0.0549, 5.145 * Math.PI / 180, 0, 0, 0),
        };
        return (sun, earth, moon);
    }

    [Fact]
    public void TryGetState_beyond_horizon_does_not_extend()
    {
        var (sun, earth, _) = SunEarthMoon();
        var ephemerides = new NBodyEphemerides([sun, earth], 0, ["Sun", "Earth"]);
        double before = ephemerides.Horizon;

        Assert.False(ephemerides.TryGetState(earth, 30 * 86400.0, out _));
        Assert.Equal(before, ephemerides.Horizon);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Non_finite_public_times_are_rejected(double invalid)
    {
        var (sun, earth, _) = SunEarthMoon();
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new NBodyEphemerides([sun, earth], invalid, ["Sun", "Earth"]));

        var ephemerides = new NBodyEphemerides([sun, earth], 0, ["Sun", "Earth"]);
        Assert.Throws<ArgumentOutOfRangeException>(() => ephemerides.GetState(earth, invalid));
        Assert.False(ephemerides.TryGetState(earth, invalid, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ephemerides.Prune(invalid));
    }

    [Fact]
    public void Multiple_root_system_is_rejected()
    {
        var first = new CelestialBody { Id = "First", Mu = 1 };
        var second = new CelestialBody { Id = "Second", Mu = 1 };

        var error = Assert.Throws<ArgumentException>(() =>
            new NBodyEphemerides([first, second], 0, ["First", "Second"]));
        Assert.Contains("exactly one root", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void All_zero_mu_system_is_rejected()
    {
        var root = new CelestialBody { Id = "MasslessRoot", Mu = 0 };

        var error = Assert.Throws<ArgumentException>(() =>
            new NBodyEphemerides([root], 0, [root.Id]));
        Assert.Contains("positive total mu", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Massless_secondary_follows_its_kepler_rail()
    {
        // Negligible planet mass reduces the system to the Kepler two-body case.
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var speck = new CelestialBody
        {
            Id = "Speck", Mu = 1e-3, Parent = sun,
            Orbit = new OrbitalElements(1.0e11, 0.2, 0.1, 0.3, 0.7, 0),
        };
        var kepler = new Ephemerides([sun, speck]);
        var nbody = new NBodyEphemerides([sun, speck], 0, ["Sun", "Speck"],
            new IntegratorOptions { RelTol = 1e-12 });
        double period = 2 * Math.PI * Math.Sqrt(Math.Pow(1.0e11, 3) / MuSun);
        var expected = kepler.GetState(speck, period / 3);
        var actual = nbody.GetState(speck, period / 3);
        Assert.True((actual.Position - expected.Position).Length() < 2e3,
            $"n-body deviated {(actual.Position - expected.Position).Length()} m from Kepler rail");
    }

    [Fact]
    public void Coincident_zero_mu_restricted_tracks_do_not_create_a_false_singularity()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var orbit = new OrbitalElements(1.0e11, 0.2, 0.1, 0.3, 0.7, 0);
        var a = new CelestialBody { Id = "A", Mu = 0, Parent = sun, Orbit = orbit };
        var b = new CelestialBody { Id = "B", Mu = 0, Parent = sun, Orbit = orbit };
        var nbody = new NBodyEphemerides([sun, a, b], 0, ["Sun"],
            new IntegratorOptions { RelTol = 1e-11 });

        var stateA = nbody.GetState(a, 86400);
        var stateB = nbody.GetState(b, 86400);

        Assert.True(double.IsFinite(stateA.Position.Length()));
        Assert.True(double.IsFinite(stateA.Velocity.Length()));
        Assert.Equal(stateA, stateB);
    }

    [Fact]
    public void Massless_secondary_feels_perturbers_without_back_reacting()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0, 0, 0, 0, 0),
        };
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1.52e11, 0.01, 0, 0, 0.2, 0),
        };
        var backbone = new NBodyEphemerides([sun, planet], 0, ["Sun", "Planet"],
            new IntegratorOptions { RelTol = 1e-11 });
        var restricted = new NBodyEphemerides([sun, planet, tracer], 0,
            ["Sun", "Planet"], new IntegratorOptions { RelTol = 1e-11 });

        double t = 5 * 86400;
        Assert.Equal(backbone.GetState(sun, t), restricted.GetState(sun, t));
        Assert.Equal(backbone.GetState(planet, t), restricted.GetState(planet, t));
        var prescribed = new Ephemerides([sun, planet, tracer]).GetState(tracer, t);
        Assert.True((restricted.GetState(tracer, t).Position - prescribed.Position).Length() > 1.0);
    }

    [Fact]
    public void Initial_positions_match_the_kepler_ephemerides()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var kepler = new Ephemerides([sun, earth, moon]);
        var nbody = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"]);
        foreach (var body in new[] { sun, earth, moon })
        {
            var k = kepler.GetState(body, 0).Position;
            var n = nbody.GetState(body, 0).Position;
            Assert.True((k - n).Length() < 1.0, $"{body.Id} initial position off by {(k - n).Length()} m");
        }
    }

    [Fact]
    public void Earth_wobbles_around_the_earth_moon_barycentre()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var nbody = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"],
            new IntegratorOptions { RelTol = 1e-11 });
        // Reflex amplitude = a_moon * mu_moon / (mu_earth + mu_moon).
        double expected = 3.844e8 * MuMoon / (MuEarth + MuMoon);
        for (double t = 5 * 86400; t <= 60 * 86400; t += 5 * 86400)
        {
            var e = nbody.GetState(earth, t);
            var m = nbody.GetState(moon, t);
            var bary = (e.Position * MuEarth + m.Position * MuMoon) / (MuEarth + MuMoon);
            double reflex = (e.Position - bary).Length();
            Assert.InRange(reflex, 0.7 * expected, 1.3 * expected);
        }
    }

    [Fact]
    public void Zero_mass_child_is_integrated_in_the_backbone_field_not_replayed_as_a_conic()
    {
        var (sun, earth, _) = SunEarthMoon();
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 0, Parent = earth,
            Orbit = new OrbitalElements(3.844e8, 0.0549, 0.09, 0, 0, 0),
        };
        var nbody = new NBodyEphemerides([sun, earth, tracer], 0, ["Sun", "Earth"]);
        double t = 12 * 86400;
        var earthState = nbody.GetState(earth, t);
        var conic = earthState + Kepler.StateFromElements(tracer.Orbit!.Value, earth.Mu, t);
        var actual = nbody.GetState(tracer, t);
        Assert.True(double.IsFinite(actual.Position.Length()));
        Assert.True((actual.Position - conic.Position).Length() > 1.0);
    }

    [Fact]
    public void Query_before_start_throws()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var nbody = new NBodyEphemerides([sun, earth, moon], 100.0, ["Sun", "Earth", "Moon"]);
        Assert.Throws<ArgumentOutOfRangeException>(() => nbody.GetState(earth, 99.0));
    }

    [Fact]
    public void Large_zero_mass_catalog_uses_restricted_numerical_trajectories()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0.01, 0, 0, 0, 0),
        };
        var tracers = Enumerable.Range(0, 65).Select(i => new CelestialBody
        {
            Id = $"Tracer{i:D2}", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(2e11 + i * 1e8, 0.02, 0, 0, 0.01 * i, 0),
        }).ToArray();
        var bodies = new CelestialBody[] { sun, planet }.Concat(tracers).ToArray();
        var ids = IntegratedSetRule.Select(bodies, 0, out var restricted);
        var tracer = tracers[^1];
        Assert.Equal(tracers.Length, restricted.Count(item =>
            item.Kind == RestrictedClassificationKind.NonBackreacting));

        var ephemerides = new NBodyEphemerides(bodies, 0, ids);
        double t = 86400;
        var prescribed = new Ephemerides(bodies).GetState(tracer, t);
        var actual = ephemerides.GetState(tracer, t);
        Assert.True(double.IsFinite(actual.Position.Length()));
        Assert.NotEqual(prescribed, actual);
    }

    [Fact]
    public void Arbitrary_backbone_above_64_is_mutually_integrated()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var massive = Enumerable.Range(0, 72)
            .Select(i => new CelestialBody
            {
                Id = $"Body{i:D2}", Mu = 1e10 + i, Parent = sun,
                Orbit = new OrbitalElements(1e11 + i * 1e9, 0.01, 0, 0, i * 0.01, 0),
            }).ToArray();
        CelestialBody[] bodies = [sun, .. massive];
        var backbone = IntegratedSetRule.Select(bodies, 0, out var restricted);
        Assert.Empty(restricted);

        var ephemerides = new NBodyEphemerides(bodies, 0, backbone,
            new IntegratorOptions { RelTol = 1e-9, MaxStep = 1 });
        Assert.Equal(73, bodies.Count(ephemerides.IsBackbone));
        Assert.All(massive, body => Assert.True(ephemerides.IsBackbone(body), body.Id));

        // Post-seed conic removal proves the whole catalog grows numerically.
        foreach (var body in massive) body.Orbit = null;
        foreach (var body in bodies)
        {
            var state = ephemerides.GetState(body, 1);
            Assert.True(double.IsFinite(state.Position.Length()), body.Id);
            Assert.True(double.IsFinite(state.Velocity.Length()), body.Id);
        }
    }

    [Fact]
    public void High_speed_positive_mu_body_joins_the_mutual_backbone()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var sungrazer = new CelestialBody
        {
            Id = "Sungrazer", Mu = 1e12, Parent = sun,
            Orbit = new OrbitalElements(2.8e13, 0.9999, 0, 0, 0, 0),
        };
        var backbone = IntegratedSetRule.Select(
            [sun, sungrazer], 0, out var restricted);
        Assert.Empty(restricted);

        var ephemerides = new NBodyEphemerides([sun, sungrazer], 0, backbone,
            new IntegratorOptions { RelTol = 1e-9, MaxStep = 0.1 });
        Assert.True(ephemerides.IsBackbone(sungrazer));
        sungrazer.Orbit = null;
        var state = ephemerides.GetState(sungrazer, 0.1);
        Assert.True(double.IsFinite(state.Position.Length()));
        Assert.Equal(1e12, ephemerides[sungrazer.Id].Mu);
    }

    [Fact]
    public void Integrated_child_without_its_parent_is_rejected()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var error = Assert.Throws<ArgumentException>(() =>
            new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Moon"]));
        Assert.Contains("backbone parents", error.Message, StringComparison.OrdinalIgnoreCase);
        Assert.Contains("Earth", error.Message);
    }

    [Fact]
    public void Prune_preserves_lookup_after_the_cut_and_rejects_queries_before_it()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var nbody = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"]);
        double t60 = 60 * 86400, t40 = 40 * 86400, t20 = 20 * 86400;
        var before = nbody.GetState(earth, t60);   // extends horizon to 60 d
        var mid = nbody.GetState(earth, t40);

        nbody.Prune(30 * 86400);

        Assert.True(nbody.StartTime <= 30 * 86400, "prune must keep a node at or before the cut");
        var after = nbody.GetState(earth, t40);
        Assert.Equal(mid.Position, after.Position); // retained window is bit-identical
        Assert.Equal(before.Position, nbody.GetState(earth, t60).Position);
        // Pin node spacing so the retained start lies in the interval under test.
        Assert.True(nbody.StartTime > t20, "expected node spacing < 10 d to place StartTime after 20 d");
        Assert.Throws<ArgumentOutOfRangeException>(() => nbody.GetState(earth, t20));
    }

    [Fact]
    public void Default_model_includes_every_massive_body()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var nbody = new NBodyEphemerides([sun, earth, moon], 0); // no explicit ids
        double t = 3 * 86400;
        var explicitAll = new NBodyEphemerides([sun, earth, moon], 0,
            [sun.Id, earth.Id, moon.Id]);
        Assert.Equal(explicitAll.GetState(moon, t), nbody.GetState(moon, t));
    }

    [Fact]
    public void Detached_growth_advances_positive_mu_restricted_tracks_with_the_backbone()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0.01, 0, 0, 0, 0),
        };
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 1e10, Parent = sun,
            Orbit = new OrbitalElements(1.52e11, 0.02, 0.01, 0, 0.2, 0),
        };
        var bodies = new[] { sun, planet, tracer };
        var options = new IntegratorOptions { RelTol = 1e-11 };
        var direct = new NBodyEphemerides(bodies, 0, [sun.Id, planet.Id], options);
        var detached = new NBodyEphemerides(bodies, 0, [sun.Id, planet.Id], options);
        Assert.False(direct.IsBackbone(tracer));
        Assert.Equal(1e10, direct[tracer.Id].Mu);
        const double target = 3 * 86400.0;

        var expected = direct.GetState(tracer, target);
        var grower = detached.CreateGrower();
        Assert.Equal(0, grower.CaptureSeed());
        grower.Integrate(target);
        Assert.True(grower.TrySplice());

        Assert.Equal(target, detached.Horizon);
        var actual = detached.GetState(tracer, target);
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void Restricted_snapshot_survives_owner_growth_and_pruning()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1e11, 0.1, 0.03, 0.2, 0.4, 0),
        };
        var rails = new NBodyEphemerides([sun, tracer], 0, [sun.Id]);
        rails.GetState(tracer, 10 * 86400.0);
        var snapshot = rails.CreateSnapshot(2 * 86400.0, 8 * 86400.0);
        var before = snapshot.GetState(tracer, 3 * 86400.0);

        rails.GetState(tracer, 20 * 86400.0);
        rails.Prune(15 * 86400.0);

        Assert.Equal(before, snapshot.GetState(tracer, 3 * 86400.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => rails.GetState(tracer, 86400.0));
    }

    [Fact]
    public void Restricted_close_approach_does_not_change_backbone_steps()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0, 0, 0, 0, 0),
        };
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1.501e11, 0, 0, 0, 0, 0),
        };
        var options = new IntegratorOptions { RelTol = 1e-10, MaxStep = 10 };
        var backbone = new NBodyEphemerides([sun, planet], 0,
            [sun.Id, planet.Id], options);
        var composite = new NBodyEphemerides([sun, planet, tracer], 0,
            [sun.Id, planet.Id], options);

        const double target = 100.0;
        Assert.Equal(backbone.GetState(sun, target), composite.GetState(sun, target));
        Assert.Equal(backbone.GetState(planet, target), composite.GetState(planet, target));
        Assert.True(double.IsFinite(composite.GetState(tracer, target).Position.Length()));
    }

    [Fact]
    public void Invalid_restricted_seed_rejects_construction()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var corrupt = new CelestialBody
        {
            Id = "Corrupt", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(-1e11, 0.5, 0, 0, 0, 0),
        };

        var error = Assert.Throws<ArgumentException>(() =>
            new NBodyEphemerides([sun, corrupt], 0, [sun.Id]));
        Assert.Contains("Corrupt", error.Message);
        Assert.Contains("seed", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Restricted_growth_failure_does_not_publish_a_partial_composite_horizon()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0, 0, 0, 0, 0),
        };
        var colliding = new CelestialBody
        {
            Id = "Colliding", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1.50001e11, 0, 0, 0, 0, 0),
        };
        var ephemerides = new NBodyEphemerides([sun, planet, colliding], 0,
            [sun.Id, planet.Id], new IntegratorOptions { RelTol = 1e-10, MaxStep = 10 });

        Assert.Throws<IntegrationFailureException>(() => ephemerides.GetState(planet, 100));
        Assert.Equal(0, ephemerides.Horizon);
        Assert.Equal(new Ephemerides([sun, planet, colliding]).GetState(colliding, 0).Position,
            ephemerides.GetState(colliding, 0).Position);
    }

    [Fact]
    public void Missing_external_parent_rejects_construction()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var external = new CelestialBody { Id = "External", Mu = MuSun };
        var child = new CelestialBody
        {
            Id = "Child", Mu = 0, Parent = external,
            Orbit = new OrbitalElements(1e11, 0.01, 0, 0, 0, 0),
        };

        var error = Assert.Throws<ArgumentException>(() =>
            new NBodyEphemerides([sun, child], 0, [sun.Id]));
        Assert.Contains("Child->External", error.Message);
    }

    [Fact]
    public void Direct_backbone_failure_leaves_owner_trajectory_unchanged()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var orbit = new OrbitalElements(1e11, 0.01, 0, 0, 0, 0);
        var a = new CelestialBody { Id = "A", Mu = 1e12, Parent = sun, Orbit = orbit };
        var b = new CelestialBody { Id = "B", Mu = 1e12, Parent = sun, Orbit = orbit };
        var ephemerides = new NBodyEphemerides([sun, a, b], 0, [sun.Id, a.Id, b.Id]);
        int nodes = ephemerides.NodeCount;
        var initial = ephemerides.GetState(sun, 0);

        Assert.Throws<IntegrationFailureException>(() => ephemerides.GetState(sun, 100));

        Assert.Equal(0, ephemerides.Horizon);
        Assert.Equal(nodes, ephemerides.NodeCount);
        Assert.Equal(initial, ephemerides.GetState(sun, 0));
    }

    [Fact]
    public void Detached_restricted_failure_leaves_owner_trajectory_unchanged()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0, 0, 0, 0, 0),
        };
        var colliding = new CelestialBody
        {
            Id = "Colliding", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1.50001e11, 0, 0, 0, 0, 0),
        };
        var ephemerides = new NBodyEphemerides([sun, planet, colliding], 0,
            [sun.Id, planet.Id], new IntegratorOptions { RelTol = 1e-10, MaxStep = 10 });
        int nodes = ephemerides.NodeCount;
        var grower = ephemerides.CreateGrower();
        grower.CaptureSeed();

        Assert.Throws<IntegrationFailureException>(() => grower.Integrate(100));

        Assert.Equal(0, ephemerides.Horizon);
        Assert.Equal(nodes, ephemerides.NodeCount);
        Assert.False(grower.TrySplice());
    }

    [Fact]
    public void Stale_detached_composite_suffix_cannot_overwrite_newer_growth()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1e11, 0.1, 0, 0, 0, 0),
        };
        var ephemerides = new NBodyEphemerides([sun, tracer], 0, [sun.Id]);
        var stale = ephemerides.CreateGrower();
        stale.CaptureSeed();
        stale.Integrate(2 * 86400.0);

        var expected = ephemerides.GetState(tracer, 86400.0);
        int nodes = ephemerides.NodeCount;
        Assert.False(stale.TrySplice());

        Assert.Equal(86400.0, ephemerides.Horizon);
        Assert.Equal(nodes, ephemerides.NodeCount);
        Assert.Equal(expected, ephemerides.GetState(tracer, 86400.0));
    }

    [Fact]
    public void Detached_capture_for_1024_restricted_bodies_copies_no_retained_nodes()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var tracers = Enumerable.Range(0, 1024).Select(i => new CelestialBody
        {
            Id = $"Tracer{i:D4}", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1e11 + i * 1e7, 0.01, 0, 0, i * 1e-3, 0),
        }).ToArray();
        var options = new IntegratorOptions { RelTol = 1e-9, MaxStep = 3600.0 };
        var ephemerides = new NBodyEphemerides(
            [sun, .. tracers], 0, [sun.Id], options);
        var shortHistoryGrower = ephemerides.CreateGrower();

        shortHistoryGrower.CaptureSeed();

        Assert.Equal(1024, shortHistoryGrower.RestrictedScratchTrackCount);
        Assert.Equal(0, shortHistoryGrower.RestrictedScratchNodeCount);

        _ = ephemerides.GetState(tracers[0], 3 * 86400.0);
        Assert.True(ephemerides.NodeCount >= 1024);
        Assert.True(ephemerides.KnotCount >= 3 * 1024);

        var longHistoryGrower = ephemerides.CreateGrower();
        longHistoryGrower.CaptureSeed();

        Assert.Equal(1024, longHistoryGrower.RestrictedScratchTrackCount);
        Assert.Equal(shortHistoryGrower.RestrictedScratchNodeCount,
            longHistoryGrower.RestrictedScratchNodeCount);
        Assert.Equal(0, longHistoryGrower.RestrictedScratchNodeCount);
        longHistoryGrower.Integrate(ephemerides.Horizon + 1.0);
        Assert.True(longHistoryGrower.RestrictedScratchNodeCount >= 1024);
    }

    [Fact]
    public void Restricted_descendant_feels_its_massive_restricted_parent()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1e10 };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = 1e14, Parent = root,
            Orbit = new OrbitalElements(1e12, 0, 0, 0, 0, 0),
        };
        var moon = new CelestialBody
        {
            Id = "Moon", Mu = 0, Parent = planet,
            Orbit = new OrbitalElements(1e7, 0, 0, 0, 0, 0),
        };
        var options = new IntegratorOptions { RelTol = 1e-11, MaxStep = 60 };
        // Child-before-parent input proves internal restricted ordering is topological.
        var ephemerides = new NBodyEphemerides(
            [root, moon, planet], 0, [root.Id], options);
        double time = 0.25 * 2 * Math.PI * Math.Sqrt(1e21 / planet.Mu);

        var actual = ephemerides.GetState(moon, time) - ephemerides.GetState(planet, time);
        var expected = Kepler.StateFromElements(moon.Orbit!.Value, planet.Mu, time);

        Assert.True(ephemerides.FeelsGravityFrom(moon, planet));
        Assert.True((actual.Position - expected.Position).Length() < 100,
            $"restricted child position drifted by {(actual.Position - expected.Position).Length():R} m");
        Assert.True((actual.Velocity - expected.Velocity).Length() < 0.1,
            $"restricted child velocity drifted by {(actual.Velocity - expected.Velocity).Length():R} m/s");
    }

    [Fact]
    public void Ordinary_restricted_peers_remain_uncoupled()
    {
        var root = new CelestialBody { Id = "Root", Mu = MuSun };
        var a = new CelestialBody
        {
            Id = "A", Mu = 1e12, Parent = root,
            Orbit = new OrbitalElements(1e11, 0.01, 0, 0, 0, 0),
        };
        var b = new CelestialBody
        {
            Id = "B", Mu = 1e16, Parent = root,
            Orbit = new OrbitalElements(1.01e11, 0.02, 0, 0, 0.1, 0),
        };
        var options = new IntegratorOptions { RelTol = 1e-10, MaxStep = 600 };
        var withoutPeer = new NBodyEphemerides([root, a], 0, [root.Id], options);
        var withPeer = new NBodyEphemerides([root, b, a], 0, [root.Id], options);

        Assert.Equal(withoutPeer.GetState(a, 86400), withPeer.GetState(a, 86400));
        Assert.False(withPeer.FeelsGravityFrom(a, b));
    }
}

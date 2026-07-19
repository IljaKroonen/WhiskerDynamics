using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class EphemeridesTests
{
    private const double TestMu = 1.0e14;

    private static Ephemerides Sample() =>
        new(AstronomicalsParser.ParseFile(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml")));

    private static OrbitalElements Elements(double semiMajorAxis = 1.0e7,
        double argumentOfPeriapsis = 0) => new(
            semiMajorAxis, 0, 0, 0, argumentOfPeriapsis, 0);

    private static CelestialBody Root(string id = "Root") =>
        new() { Id = id, Mu = TestMu };

    private static CelestialBody Child(string id, CelestialBody parent,
        double semiMajorAxis = 1.0e7, double argumentOfPeriapsis = 0) => new()
    {
        Id = id,
        Mu = TestMu,
        Parent = parent,
        Orbit = Elements(semiMajorAxis, argumentOfPeriapsis),
    };

    [Fact]
    public void Root_is_fixed_at_origin()
    {
        var eph = Sample();
        var s = eph.GetState(eph["Sol"], 12345.0);
        Assert.Equal(Vector3d.Zero, s.Position);
        Assert.Equal(Vector3d.Zero, s.Velocity);
    }

    [Fact]
    public void Planet_state_matches_direct_kepler_evaluation()
    {
        var eph = Sample();
        var mercury = eph["Mercury"];
        var expected = Kepler.StateFromElements(mercury.Orbit!.Value, eph["Sol"].Mu, 777777.0);
        var actual = eph.GetState(mercury, 777777.0);
        Assert.Equal(expected.Position, actual.Position);
        Assert.Equal(expected.Velocity, actual.Velocity);
    }

    [Fact]
    public void Moon_state_composes_parent_chain()
    {
        var eph = Sample();
        double t = 55555.0;
        var mercuryState = eph.GetState(eph["Mercury"], t);
        var moonLocal = Kepler.StateFromElements(eph["TestMoon"].Orbit!.Value, eph["Mercury"].Mu, t);
        var moonState = eph.GetState(eph["TestMoon"], t);
        Assert.Equal((mercuryState.Position + moonLocal.Position).X, moonState.Position.X, 6);
        Assert.Equal((mercuryState.Velocity + moonLocal.Velocity).Y, moonState.Velocity.Y, 9);
    }

    [Fact]
    public void Unknown_id_throws()
    {
        Assert.Throws<KeyNotFoundException>(() => Sample()["Krypton"]);
    }

    [Fact]
    public void Constructor_rejects_a_parent_cycle_before_recursive_lookup()
    {
        var a = new CelestialBody { Id = "A", Mu = 1 };
        var b = new CelestialBody { Id = "B", Mu = 1, Parent = a };
        a.Parent = b;

        var ex = Assert.Throws<ArgumentException>(() => new Ephemerides([a, b]));

        Assert.Equal("bodies", ex.ParamName);
        Assert.Contains("'A' -> 'B' -> 'A'", ex.Message);
    }

    [Fact]
    public void Constructor_reports_every_disjoint_parent_cycle_canonically()
    {
        var a = new CelestialBody { Id = "A", Mu = 1 };
        var b = new CelestialBody { Id = "B", Mu = 1, Parent = a };
        a.Parent = b;
        var x = new CelestialBody { Id = "X", Mu = 1 };
        x.Parent = x;

        var ex = Assert.Throws<ArgumentException>(() => new Ephemerides([x, b, a]));

        Assert.Equal("bodies", ex.ParamName);
        Assert.Contains("'A' -> 'B' -> 'A'; 'X' -> 'X'", ex.Message);
    }

    [Fact]
    public void Empty_catalog_remains_valid()
    {
        var eph = new Ephemerides([]);

        Assert.Empty(eph.Bodies);
    }

    [Fact]
    public void Multiple_parentless_roots_remain_valid_for_generic_ephemerides()
    {
        var a = new CelestialBody { Id = "A", Mu = 1 };
        var b = new CelestialBody { Id = "B", Mu = 2 };
        var eph = new Ephemerides([a, b]);

        Assert.Equal(Vector3d.Zero, eph.GetState(a, 1).Position);
        Assert.Equal(Vector3d.Zero, eph.GetState(b, 2).Position);
    }

    [Fact]
    public void External_acyclic_body_chain_remains_supported()
    {
        var root = Root();
        var planet = Child("Planet", root);
        var moon = Child("Moon", planet, 2.0e6);
        var eph = new Ephemerides([]);
        const double time = 123.0;
        var zero = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var planetLocal = Kepler.StateFromElements(planet.Orbit!.Value, root.Mu, time);
        var moonLocal = Kepler.StateFromElements(moon.Orbit!.Value, planet.Mu, time);

        var actual = eph.GetState(moon, time);

        Assert.Equal((zero + planetLocal) + moonLocal, actual);
    }

    [Fact]
    public void External_cycle_with_null_orbits_reports_the_exact_cycle_first()
    {
        var a = new CelestialBody { Id = "A", Mu = 1 };
        var b = new CelestialBody { Id = "B", Mu = 1, Parent = a };
        a.Parent = b;
        var eph = new Ephemerides([]);

        var ex = Assert.Throws<InvalidOperationException>(() => eph.GetState(a, 0));

        Assert.Contains("'A' -> 'B' -> 'A'", ex.Message);
        Assert.DoesNotContain("Nullable", ex.Message);
    }

    [Fact]
    public void Parent_cycle_introduced_after_construction_is_contained()
    {
        var root = Root();
        var child = Child("Child", root);
        var eph = new Ephemerides([root, child]);
        root.Parent = child;

        var ex = Assert.Throws<InvalidOperationException>(() => eph.GetState(child, 0));

        Assert.Contains("'Child' -> 'Root' -> 'Child'", ex.Message);
    }

    [Fact]
    public void Parented_body_with_null_orbit_retains_nullable_failure_behavior()
    {
        var root = Root();
        var child = new CelestialBody { Id = "NoOrbit", Mu = 1, Parent = root };
        var eph = new Ephemerides([root, child]);

        var ex = Assert.Throws<InvalidOperationException>(() => eph.GetState(child, 0));

        Assert.DoesNotContain("Parent cycle", ex.Message);
    }

    [Fact]
    public void Deep_acyclic_external_chain_does_not_use_the_call_stack()
    {
        const int depth = 10_000;
        const double semiMajorAxis = 1.0e6;
        var root = Root();
        CelestialBody leaf = root;
        for (int i = 0; i < depth; i++)
            leaf = Child($"Body{i:D5}", leaf, semiMajorAxis);
        var eph = new Ephemerides([]);

        var state = eph.GetState(leaf, 0);

        double expectedX = depth * semiMajorAxis;
        Assert.True(double.IsFinite(state.Position.X));
        Assert.True(Math.Abs(state.Position.X - expectedX) <= expectedX * 1e-12,
            $"expected x={expectedX:R}, got {state.Position.X:R}");
    }

    [Fact]
    public void Deep_chain_preserves_recursive_root_to_child_floating_point_order()
    {
        var root = Root();
        var a = Child("A", root, 1.0e20);
        var b = Child("B", a, 1.0e20, Math.PI);
        var c = Child("C", b, 1.0);
        var eph = new Ephemerides([root, a, b, c]);
        var zero = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var aLocal = Kepler.StateFromElements(a.Orbit!.Value, root.Mu, 0);
        var bLocal = Kepler.StateFromElements(b.Orbit!.Value, a.Mu, 0);
        var cLocal = Kepler.StateFromElements(c.Orbit!.Value, b.Mu, 0);
        var expected = ((zero + aLocal) + bLocal) + cLocal;
        var reversed = ((zero + cLocal) + bLocal) + aLocal;

        var actual = eph.GetState(c, 0);

        Assert.NotEqual(expected.Position.X, reversed.Position.X);
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void Root_planet_and_moon_fast_paths_allocate_nothing_after_warmup()
    {
        var root = Root();
        var planet = Child("Planet", root);
        var moon = Child("Moon", planet, 2.0e6);
        var eph = new Ephemerides([root, planet, moon]);
        StateVector result = default;
        for (int i = 0; i < 128; i++)
        {
            result = eph.GetState(root, i);
            result = eph.GetState(planet, i);
            result = eph.GetState(moon, i);
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++)
        {
            result = eph.GetState(root, i);
            result = eph.GetState(planet, i);
            result = eph.GetState(moon, i);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        GC.KeepAlive(result);

        Assert.Equal(0, allocated);
    }
}

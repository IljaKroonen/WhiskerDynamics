using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

/// <summary>Third-body differential acceleration for parent-relative live
/// dynamics: subtract each source's acceleration of the parent from its
/// acceleration of the vessel.</summary>
public class ThirdBodyDeltaTests
{
    private static IReadOnlyList<CelestialBody> Bodies() =>
        AstronomicalsParser.ParseFile(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"));

    [Fact]
    public void Two_body_world_delta_is_exactly_zero()
    {
        var bodies = Bodies();
        var eph = new NBodyEphemerides(bodies, 0.0, bodies.Select(body => body.Id).ToArray());
        var mercury = eph["Mercury"];
        var parentOnly = new GravityModel(eph, [mercury]);
        // A pure two-body system has exactly zero differential correction.
        Assert.Equal(Vector3d.Zero, parentOnly.ThirdBodyDeltaAt(mercury, new Vector3d(2.74e6, 0, 0), 0.0));
        Assert.Equal(Vector3d.Zero, parentOnly.ThirdBodyDeltaAt(mercury, new Vector3d(-1e7, 3e6, 2e6), 5000.0));
    }

    [Fact]
    public void Delta_equals_full_field_minus_parent_term_minus_parent_rails_acceleration()
    {
        var bodies = Bodies();
        string[] sourceIds = ["Sol", "Mercury", "TestMoon"];
        var eph = new NBodyEphemerides(bodies, 0.0, bodies.Select(body => body.Id).ToArray());
        var sources = sourceIds.Select(id => eph[id]).ToArray();
        var all = new GravityModel(eph, sources);
        var mercury = eph["Mercury"];

        double t = 5000.0;
        var rel = new Vector3d(2.6e6, 1.2e6, 5e5);
        var parentPos = eph.GetState(mercury, t).Position;

        // Equivalent decomposition: gN(vessel) - g1(vessel) - a_parent.
        var gNv = all.AccelerationAt(parentPos + rel, t);
        double r2 = rel.LengthSquared();
        var g1v = rel * (-mercury.Mu / (r2 * Math.Sqrt(r2)));
        var aParent = Vector3d.Zero;
        foreach (var body in sources)
        {
            if (ReferenceEquals(body, mercury)) continue;
            aParent += new GravityModel(eph, [body]).AccelerationAt(parentPos, t);
        }
        var expected = gNv - g1v - aParent;

        var actual = all.ThirdBodyDeltaAt(mercury, rel, t);
        Assert.True((expected - actual).Length() <= 1e-6 * actual.Length(),
            $"expected {expected}, actual {actual} (|expected|={expected.Length():E3}, |actual|={actual.Length():E3})");
    }

    [Fact]
    public void Delta_is_tide_order_while_the_direct_third_body_sum_is_not()
    {
        var bodies = Bodies();
        var eph = new NBodyEphemerides(bodies, 0.0, bodies.Select(body => body.Id).ToArray());
        var all = new GravityModel(eph, [eph["Sol"], eph["Mercury"]]);
        var mercury = eph["Mercury"];

        // 300 km above the Mercury-like fixture.
        var rel = new Vector3d(2.74e6, 0, 0);
        double t = 0.0;

        var delta = all.ThirdBodyDeltaAt(mercury, rel, t);
        Assert.True(delta.Length() is >= 1e-8 and <= 1e-4,
            $"tidal |delta|={delta.Length():E3} m/s^2 outside tide order"); // solar tide (~7e-6 here)

        // The incorrect direct field includes the star's pull and exceeds the
        // live-gravity safety threshold.
        var parentPos = eph.GetState(mercury, t).Position;
        var gNv = all.AccelerationAt(parentPos + rel, t);
        double r2 = rel.LengthSquared();
        var g1v = rel * (-mercury.Mu / (r2 * Math.Sqrt(r2)));
        Assert.True((gNv - g1v).Length() >= 1e-3,
            $"direct third-body sum unexpectedly small: {(gNv - g1v).Length():E3}");
    }

    [Fact]
    public void Every_massive_source_is_coupled_and_contributes_a_tidal_term()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = 1.32712440018e20 };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = 3.986004418e14, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0.01, 0, 0, 0, 0),
        };
        var comet = new CelestialBody
        {
            Id = "Comet", Mu = 1e12, Parent = sun,
            Orbit = new OrbitalElements(2e11, 0.8, 0, 0, 0, 0),
        };
        var backboneOnly = new NBodyEphemerides([sun, planet], 0, ["Sun", "Planet"]);
        var withComet = new NBodyEphemerides(
            [sun, planet, comet], 0, ["Sun", "Planet", "Comet"]);
        var gravity = new GravityModel(withComet, [sun, planet, comet]);

        double t = 86400;
        var parent = withComet.GetState(planet, t).Position;
        var rel = new Vector3d(1e7, 2e6, 0);
        var sunOffset = withComet.GetState(sun, t).Position - parent;
        var cometOffset = withComet.GetState(comet, t).Position - parent;
        var expected = sun.Mu * GravityModel.TidalTerm(sunOffset, rel)
            + comet.Mu * GravityModel.TidalTerm(cometOffset, rel);

        var actual = gravity.ThirdBodyDeltaAt(planet, rel, t);
        Assert.True((actual - expected).Length() < 1e-15,
            $"expected fully coupled tidal delta {expected}, got {actual}");

        var without = backboneOnly.GetState(planet, t);
        var coupled = withComet.GetState(planet, t);
        Assert.True((without.Position - coupled.Position).Length() > 1e-9);
        Assert.True(double.IsFinite(gravity.ThirdBodyDeltaAt(comet, rel, t).Length()));
    }

    [Fact]
    public void Restricted_positive_mass_source_is_direct_and_does_not_backreact()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = 1.32712440018e20 };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = 3.986004418e14, Parent = sun,
            Orbit = new OrbitalElements(1.5e11, 0.01, 0, 0, 0, 0),
        };
        var comet = new CelestialBody
        {
            Id = "Comet", Mu = 1e12, Parent = sun,
            Orbit = new OrbitalElements(2e11, 0.3, 0.1, 0, 0.4, 0),
        };
        var options = new IntegratorOptions { RelTol = 1e-11 };
        var backboneOnly = new NBodyEphemerides(
            [sun, planet], 0, [sun.Id, planet.Id], options);
        var composite = new NBodyEphemerides(
            [sun, planet, comet], 0, [sun.Id, planet.Id], options);
        var gravity = new GravityModel(composite, [sun, planet, comet]);

        const double t = 1000;
        var rel = new Vector3d(1e7, 2e6, 3e5);
        var parentPosition = composite.GetState(planet, t).Position;
        var sunOffset = composite.GetState(sun, t).Position - parentPosition;
        var cometOffset = composite.GetState(comet, t).Position - parentPosition;
        var expected = sun.Mu * GravityModel.TidalTerm(sunOffset, rel)
            + comet.Mu * GravityModel.DirectPointMassTerm(cometOffset, rel);

        Assert.False(composite.IsBackbone(comet));
        Assert.Equal(backboneOnly.GetState(sun, t), composite.GetState(sun, t));
        Assert.Equal(backboneOnly.GetState(planet, t), composite.GetState(planet, t));
        var actual = gravity.ParentRelativeCorrectionAt(planet, rel, t);
        Assert.True((actual - expected).Length() < 1e-15,
            $"expected mixed correction {expected}, got {actual}");

        var snapshot = composite.CreateSnapshot(0, t);
        Assert.True(snapshot.IsBackbone(sun));
        Assert.False(snapshot.IsBackbone(comet));
        var snapActual = new GravityModel(snapshot, [sun, planet, comet])
            .ParentRelativeCorrectionAt(planet, rel, t);
        Assert.Equal(actual, snapActual);
    }

    [Fact]
    public void Restricted_ancestor_is_tidal_for_its_descendant_while_peer_is_direct()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1e20 };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = 1e14, Parent = root,
            Orbit = new OrbitalElements(1e11, 0.01, 0, 0, 0, 0),
        };
        var moon = new CelestialBody
        {
            Id = "Moon", Mu = 0, Parent = planet,
            Orbit = new OrbitalElements(1e7, 0.02, 0, 0, 0, 0),
        };
        var peer = new CelestialBody
        {
            Id = "Peer", Mu = 1e13, Parent = root,
            Orbit = new OrbitalElements(2e11, 0.1, 0.1, 0, 0.2, 0),
        };
        var ephemerides = new NBodyEphemerides(
            [root, moon, peer, planet], 0, [root.Id],
            new IntegratorOptions { RelTol = 1e-11, MaxStep = 10 });
        var gravity = new GravityModel(ephemerides, [root, planet, moon, peer]);
        const double time = 100;
        var relative = new Vector3d(1e5, -2e4, 3e3);
        var parentPosition = ephemerides.GetState(moon, time).Position;
        var rootOffset = ephemerides.GetState(root, time).Position - parentPosition;
        var planetOffset = ephemerides.GetState(planet, time).Position - parentPosition;
        var peerOffset = ephemerides.GetState(peer, time).Position - parentPosition;
        var expected = root.Mu * GravityModel.TidalTerm(rootOffset, relative)
            + planet.Mu * GravityModel.TidalTerm(planetOffset, relative)
            + peer.Mu * GravityModel.DirectPointMassTerm(peerOffset, relative);

        Assert.True(ephemerides.FeelsGravityFrom(moon, planet));
        Assert.False(ephemerides.FeelsGravityFrom(moon, peer));
        var actual = gravity.ParentRelativeCorrectionAt(moon, relative, time);
        double tolerance = Math.Max(1e-18, expected.Length() * 1e-12);
        Assert.True((actual - expected).Length() <= tolerance,
            $"expected ancestor tide plus peer direct term {expected}, got {actual}");

        var snapshot = ephemerides.CreateSnapshot(0, time);
        Assert.True(snapshot.FeelsGravityFrom(moon, planet));
        Assert.False(snapshot.FeelsGravityFrom(moon, peer));
        var snapActual = new GravityModel(snapshot, [root, planet, moon, peer])
            .ParentRelativeCorrectionAt(moon, relative, time);
        Assert.Equal(actual, snapActual);
    }

    [Fact]
    public void Tidal_kernel_matches_the_linearized_tide_for_small_separations()
    {
        // Radial tide: stretching, +2r/d^3 per unit mu.
        var s = new Vector3d(4.7e10, 0, 0);
        var radial = new Vector3d(4.7e6, 0, 0); // 1e-4 of d
        var term = GravityModel.TidalTerm(s, radial);
        double expectedX = 2 * radial.X / Math.Pow(s.X, 3);
        Assert.True(Math.Abs(term.X - expectedX) <= 0.01 * Math.Abs(expectedX),
            $"radial tide {term.X:E6} vs linearized {expectedX:E6}");
        Assert.Equal(0.0, term.Y, 1e-20);
        Assert.Equal(0.0, term.Z, 1e-20);

        // Transverse tide: compression, -r/d^3 per unit mu.
        var transverse = new Vector3d(0, 4.7e6, 0);
        var termT = GravityModel.TidalTerm(s, transverse);
        double expectedY = -transverse.Y / Math.Pow(s.X, 3);
        Assert.True(Math.Abs(termT.Y - expectedY) <= 0.01 * Math.Abs(expectedY),
            $"transverse tide {termT.Y:E6} vs linearized {expectedY:E6}");
    }

    [Fact]
    public void Live_relative_dynamics_with_delta_track_the_predictor()
    {
        // Parent-relative integration with the tidal correction must match the
        // absolute predictor re-expressed against the parent's rails.
        var bodies = Bodies();
        var eph = new NBodyEphemerides(bodies, 0.0, bodies.Select(body => body.Id).ToArray(),
            new IntegratorOptions { RelTol = 1e-11 });
        var gravity = new GravityModel(eph, [eph["Sol"], eph["Mercury"]]);
        var mercury = eph["Mercury"];
        double mu = mercury.Mu;

        var rel0 = new Vector3d(2.74e6, 0, 0);
        var vRel0 = new Vector3d(0, Math.Sqrt(mu / rel0.Length()), 0); // circular
        var m0 = eph.GetState(mercury, 0.0);
        var predictor = new TrajectoryPredictor(gravity,
            new StateVector(m0.Position + rel0, m0.Velocity + vRel0), 0.0,
            new IntegratorOptions { RelTol = 1e-11 });

        const double T = 3000.0;
        Vector3d G1(Vector3d r)
        {
            double r2 = r.LengthSquared();
            return r * (-mu / (r2 * Math.Sqrt(r2)));
        }
        var withDelta = IntegrateRk4(
            (t, r) => G1(r) + gravity.ThirdBodyDeltaAt(mercury, r, t), rel0, vRel0, T, 1.0);
        var stock = IntegrateRk4((t, r) => G1(r), rel0, vRel0, T, 1.0);

        var truth = predictor.StateAt(T).Position - eph.GetState(mercury, T).Position;
        double errWith = (withDelta - truth).Length();
        double errStock = (stock - truth).Length();

        Assert.True(errWith < 0.5, $"with-delta error {errWith:E3} m (stock {errStock:E3} m)");
        Assert.True(errStock > 2.0, $"stock error unexpectedly small: {errStock:E3} m");
        Assert.True(errStock > 10 * errWith,
            $"delta did not dominate: with {errWith:E3} m vs stock {errStock:E3} m");
    }

    private static Vector3d IntegrateRk4(
        Func<double, Vector3d, Vector3d> acc, Vector3d p, Vector3d v, double tEnd, double dt)
    {
        double t = 0;
        while (t < tEnd - 1e-9)
        {
            double h = Math.Min(dt, tEnd - t);
            var k1v = acc(t, p);
            var k1p = v;
            var k2v = acc(t + h / 2, p + k1p * (h / 2));
            var k2p = v + k1v * (h / 2);
            var k3v = acc(t + h / 2, p + k2p * (h / 2));
            var k3p = v + k2v * (h / 2);
            var k4v = acc(t + h, p + k3p * h);
            var k4p = v + k3v * h;
            p += (k1p + 2 * k2p + 2 * k3p + k4p) * (h / 6);
            v += (k1v + 2 * k2v + 2 * k3v + k4v) * (h / 6);
            t += h;
        }
        return p;
    }
}

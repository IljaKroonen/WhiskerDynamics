using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class GravityModelTests
{
    private static Ephemerides Sample() =>
        new(AstronomicalsParser.ParseFile(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml")));

    [Fact]
    public void Single_source_matches_inverse_square_law()
    {
        var eph = Sample();
        var g = new GravityModel(eph, [eph["Sol"]]);
        double r = 1.5e11;
        var a = g.AccelerationAt(new Vector3d(r, 0, 0), 0);
        Assert.Equal(-eph["Sol"].Mu / (r * r), a.X, Math.Abs(eph["Sol"].Mu / (r * r)) * 1e-12);
        Assert.Equal(0, a.Y, 15);
        Assert.Equal(0, a.Z, 15);
    }

    [Fact]
    public void Multiple_sources_superpose()
    {
        var eph = Sample();
        var all = new GravityModel(eph);
        var solOnly = new GravityModel(eph, [eph["Sol"]]);
        var mercuryOnly = new GravityModel(eph, [eph["Mercury"]]);
        var moonOnly = new GravityModel(eph, [eph["TestMoon"]]);
        var p = new Vector3d(6e10, 1e10, 0);
        double t = 9999.0;
        var sum = solOnly.AccelerationAt(p, t) + mercuryOnly.AccelerationAt(p, t) + moonOnly.AccelerationAt(p, t);
        var direct = all.AccelerationAt(p, t);
        Assert.Equal(sum.X, direct.X, 15);
        Assert.Equal(sum.Y, direct.Y, 15);
        Assert.Equal(sum.Z, direct.Z, 15);
    }

    [Fact]
    public void Zero_mu_trajectory_is_not_a_gravity_source()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = 1e20 };
        var tracer = new CelestialBody
        {
            Id = "Tracer", Mu = 0, Parent = sun,
            Orbit = new OrbitalElements(1e10, 0, 0, 0, 0, 0),
        };
        var eph = new Ephemerides([sun, tracer]);
        var gravity = new GravityModel(eph, [sun, tracer]);

        Assert.Equal([sun], gravity.Sources);
        var acceleration = gravity.AccelerationAt(eph.GetState(tracer, 0).Position, 0);
        Assert.True(double.IsFinite(acceleration.Length()));
    }

    [Fact]
    public void Mutable_cache_resolves_positive_restricted_dense_track_independently()
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
            Orbit = new OrbitalElements(2.1e11, 0.2, 0.1, 0.2, 0.3, 0),
        };
        CelestialBody[] bodies = [sun, planet, comet];
        var ephemerides = new NBodyEphemerides(
            bodies, 0, [sun.Id, planet.Id], new IntegratorOptions { RelTol = 1e-11 });
        const double horizon = 10_000;
        _ = ephemerides.GetState(comet, horizon);
        const double time = horizon - 0.123456789;
        int cometIndex = ephemerides.IntegratedIndexOf(comet);
        Assert.True(ephemerides.ResolveBodySegment(cometIndex, time).Dt > 0,
            "regression query must lie inside a restricted dense segment");

        var point = new Vector3d(1.8e11, -2e10, 3e9);
        var mutable = new GravityModel(ephemerides, bodies).AccelerationAt(point, time);
        Assert.True(double.IsFinite(mutable.Length()));

        var snapshot = ephemerides.CreateSnapshot(0, horizon);
        var immutable = new GravityModel(snapshot, bodies).AccelerationAt(point, time);
        Assert.True((mutable - immutable).Length() <= 1e-15 * immutable.Length(),
            $"mutable {mutable}, immutable {immutable}");
    }

    [Fact]
    public void Acceleration_points_toward_the_source()
    {
        var eph = Sample();
        var g = new GravityModel(eph, [eph["Mercury"]]);
        double t = 0;
        var mercuryPos = eph.GetState(eph["Mercury"], t).Position;
        var p = mercuryPos + new Vector3d(1e7, 0, 0);
        var a = g.AccelerationAt(p, t);
        Assert.True(a.Normalized().Dot((mercuryPos - p).Normalized()) > 0.999999);
    }

    [Fact]
    public void J2_is_the_C20_harmonic_and_is_summed_with_point_mass_gravity()
    {
        const double mu = 3.986004418e14;
        const double radius = 6_378_137;
        const double j2 = 1.08262668e-3;
        var earth = new CelestialBody
        {
            Id = "Earth",
            Mu = mu,
            Geopotential = Geopotential.FromJ2(radius, new Vector3d(0, 0, 2), j2),
        };
        var gravity = new GravityModel(new Ephemerides([earth]));
        var point = new Vector3d(7_000_000, 0, 0);

        var acceleration = gravity.AccelerationAt(point, 0);
        double central = -mu / point.LengthSquared();
        double correction = -1.5 * j2 * mu * radius * radius / Math.Pow(point.X, 4);

        Assert.Equal(central + correction, acceleration.X, Math.Abs(correction) * 1e-12);
        Assert.Equal(0, acceleration.Y, 15);
        Assert.Equal(0, acceleration.Z, 15);
        var coefficient = Assert.Single(earth.Geopotential.Coefficients);
        Assert.Equal((2, 0, -j2, 0),
            (coefficient.Degree, coefficient.Order, coefficient.Cosine, coefficient.Sine));
    }

    [Fact]
    public void J2_axis_is_coordinate_independent()
    {
        const double mu = 4e14;
        var zField = Geopotential.FromJ2(6e6, new Vector3d(0, 0, 1), 1e-3);
        var xField = Geopotential.FromJ2(6e6, new Vector3d(1, 0, 0), 1e-3);

        var zResult = zField.AccelerationCorrection(new Vector3d(7e6, 0, 2e6), mu, 123);
        var xResult = xField.AccelerationCorrection(new Vector3d(2e6, 0, 7e6), mu, 123);

        Assert.Equal(zResult.X, xResult.Z, 12);
        Assert.Equal(zResult.Z, xResult.X, 12);
        Assert.Equal(zResult.Y, xResult.Y, 15);
    }

    [Fact]
    public void Live_relative_perturbation_matches_absolute_predictor_with_two_extended_bodies()
    {
        var rotation = new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 1e-4, 0);
        var parent = new CelestialBody
        {
            Id = "Parent", Mu = 4e14,
            Geopotential = new Geopotential(6e6, rotation,
            [
                new(2, 0, -1e-3, 0),
                new(2, 2, 2e-5, -4e-6),
                new(3, 1, -3e-6, 2e-6),
            ]),
        };
        var other = new CelestialBody
        {
            Id = "Other", Mu = 5e12,
            Geopotential = new Geopotential(2e6, rotation,
            [
                new(2, 0, -2e-3, 0),
                new(2, 2, -7e-5, 3e-5),
            ]),
        };
        var s = new Vector3d(4e8, 1e8, 0);
        var rel = new Vector3d(7e6, 3e5, 2e5);
        const double time = 4321;
        var ephemerides = new FixedEphemerides(
            (parent, Vector3d.Zero), (other, s));

        var full = new GravityModel(ephemerides, [parent, other]);
        double s2 = s.LengthSquared();
        var parentRailsAcceleration = s * (other.Mu / (s2 * Math.Sqrt(s2)));
        var predictedRelative = full.AccelerationAt(rel, time) - parentRailsAcceleration;
        double r2 = rel.LengthSquared();
        var parentCentral = rel * (-parent.Mu / (r2 * Math.Sqrt(r2)));
        var predictedPerturbation = predictedRelative - parentCentral;

        var livePerturbation = parent.Geopotential.AccelerationCorrection(rel, parent.Mu, time)
            + other.Mu * GravityModel.TidalTerm(s, rel)
            + GravityModel.ExtendedBodyDirectTerm(other.Geopotential, other.Mu, s, rel, time);

        Assert.Equal(predictedPerturbation.X, livePerturbation.X, 12);
        Assert.Equal(predictedPerturbation.Y, livePerturbation.Y, 12);
        Assert.Equal(predictedPerturbation.Z, livePerturbation.Z, 12);
    }

    [Fact]
    public void Geopotential_rejects_invalid_body_fixed_orientation()
    {
        var invalid = new BodyRotation(
            new Vector3d(0, 0, 1),
            new Vector3d(1, 0, 0),
            new Vector3d(1, 0, 0),
            double.NaN,
            0);

        Assert.Throws<ArgumentException>(() =>
            Geopotential.FromJ2(6e6, invalid, 1e-3));
    }

    private sealed class FixedEphemerides(
        params (CelestialBody Body, Vector3d Position)[] entries) : IEphemerides
    {
        private readonly Dictionary<CelestialBody, Vector3d> _positions =
            entries.ToDictionary(entry => entry.Body, entry => entry.Position);
        public IReadOnlyList<CelestialBody> Bodies { get; } = entries.Select(entry => entry.Body).ToArray();
        public CelestialBody this[string id] => Bodies.Single(body => body.Id == id);
        public StateVector GetState(CelestialBody body, double time) =>
            new(_positions[body], Vector3d.Zero);
    }
}

using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

/// <summary>End-to-end nodal precession from Earth's C20 term.</summary>
public class J2AcceptanceTests
{
    private const double MuEarth = 3.986004418e14;
    private const double EarthRadius = 6_378_137.0;
    private const double EarthJ2 = 1.08262668e-3;

    [Fact]
    public void Near_polar_LEO_precesses_its_node_at_the_J2_secular_rate()
    {
        const double a = EarthRadius + 700_000;
        const double e = 0.001;
        double inclination = 97.87377 * Math.PI / 180; // ~sun-synchronous at 700 km
        var elements = new OrbitalElements(a, e, inclination, 0, 0, 0);
        var initial = Kepler.StateFromElements(elements, MuEarth, 0);
        var rotation = new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0),
            7.292115e-5, 0);

        var oblateEarth = new CelestialBody
        {
            Id = "Earth",
            Mu = MuEarth,
            Geopotential = Geopotential.FromJ2(EarthRadius, rotation, EarthJ2),
        };
        var sphericalEarth = new CelestialBody { Id = "Earth", Mu = MuEarth };
        var withJ2 = new TrajectoryPredictor(
            new GravityModel(new Ephemerides([oblateEarth])), initial, 0,
            new IntegratorOptions { RelTol = 1e-11, MaxStep = 120 });
        var pointMass = new TrajectoryPredictor(
            new GravityModel(new Ephemerides([sphericalEarth])), initial, 0,
            new IntegratorOptions { RelTol = 1e-11, MaxStep = 120 });

        const double duration = 10 * 86400.0;
        double actualAdvance = WrappedAngle(NodeLongitude(withJ2.StateAt(duration))
            - NodeLongitude(initial));
        double pointMassAdvance = WrappedAngle(NodeLongitude(pointMass.StateAt(duration))
            - NodeLongitude(initial));

        // First-order secular J2 theory:
        // Ωdot = -3/2 n J2 (R/p)^2 cos(i).  Retrograde cos(i)<0, so Ω advances.
        double p = a * (1 - e * e);
        double meanMotion = Math.Sqrt(MuEarth / (a * a * a));
        double expectedAdvance = -1.5 * meanMotion * EarthJ2
            * Math.Pow(EarthRadius / p, 2) * Math.Cos(inclination) * duration;

        Assert.True(expectedAdvance > 0);
        Assert.InRange(actualAdvance, 0.97 * expectedAdvance, 1.03 * expectedAdvance);
        Assert.True(Math.Abs(pointMassAdvance) < 1e-8,
            $"spherical control node moved {pointMassAdvance * 180 / Math.PI:E3} deg");
        Assert.True(actualAdvance > 5 * Math.PI / 180,
            $"J2 node advanced only {actualAdvance * 180 / Math.PI:F3} deg in 10 days");
    }

    private static double NodeLongitude(in StateVector state)
    {
        var h = state.Position.Cross(state.Velocity);
        var node = new Vector3d(-h.Y, h.X, 0);
        return Math.Atan2(node.Y, node.X);
    }

    private static double WrappedAngle(double angle) => Math.Atan2(Math.Sin(angle), Math.Cos(angle));
}

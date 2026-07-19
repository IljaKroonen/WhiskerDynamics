using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class LagrangeAcceptanceTests
{
    private const double MuSun = 1.32712440018e20;
    private const double MuEarth = 3.986004418e14;
    private const double MuMoon = 4.9028e12;
    private const double Year = 365.25 * 86400;

    private static (NBodyEphemerides eph, CelestialBody earth, CelestialBody moon) SunEarthMoon()
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
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"],
            new IntegratorOptions { RelTol = 1e-11 });
        return (eph, earth, moon);
    }

    /// <summary>Equilateral point with co-rotating velocity.</summary>
    private static StateVector TrojanState(NBodyEphemerides eph, CelestialBody earth, CelestialBody moon,
        double t, double angleDegrees)
    {
        var e = eph.GetState(earth, t);
        var m = eph.GetState(moon, t);
        var rel = m.Position - e.Position;
        var relV = m.Velocity - e.Velocity;
        var axis = rel.Cross(relV).Normalized();
        double angle = angleDegrees * Math.PI / 180;
        return new StateVector(
            e.Position + rel.RotateAbout(axis, angle),
            e.Velocity + relV.RotateAbout(axis, angle));
    }

    // Solar phase selects a stable tadpole region; an equilateral seed is not the
    // exact libration centre of the perturbed system.
    [Theory]
    [InlineData(60.0, 0.0)]   // L4, leading
    [InlineData(-60.0, 7.0)]  // L5, trailing
    public void Earth_moon_trojan_probe_librates_for_five_years(double angleDegrees, double seedEpochDays)
    {
        var (eph, earth, moon) = SunEarthMoon();
        var gravity = new GravityModel(eph);
        double t0 = seedEpochDays * 86400;
        var predictor = new TrajectoryPredictor(gravity, TrojanState(eph, earth, moon, t0, angleDegrees), t0,
            new IntegratorOptions { RelTol = 1e-11 });

        for (double t = t0 + 30 * 86400; t <= t0 + 5 * Year; t += 30 * 86400)
        {
            var probe = predictor.StateAt(t);
            var e = eph.GetState(earth, t);
            var m = eph.GetState(moon, t);
            double dEM = (m.Position - e.Position).Length();
            double dProbeEarth = (probe.Position - e.Position).Length();

            Assert.InRange(dProbeEarth / dEM, 0.7, 1.3);
            var relM = (m.Position - e.Position).Normalized();
            var relP = (probe.Position - e.Position).Normalized();
            double angle = Math.Acos(Math.Clamp(relM.Dot(relP), -1, 1)) * 180 / Math.PI;
            Assert.InRange(angle, 5, 175);
        }
    }

    /// <summary>Hill-approximation Sun-Earth L1 state referenced to the
    /// Earth-Moon barycentre, excluding Earth's lunar reflex motion.</summary>
    private static (Vector3d pos, Vector3d vel) L1PointAndVelocity(NBodyEphemerides eph,
        CelestialBody earth, CelestialBody moon, double t)
    {
        var e = eph.GetState(earth, t);
        var m = eph.GetState(moon, t);
        var bary = (e * MuEarth + m * MuMoon) * (1 / (MuEarth + MuMoon));
        double R = bary.Position.Length();
        double d = R * Math.Cbrt((MuEarth + MuMoon) / (3 * MuSun));
        double f = 1 - d / R;
        return (bary.Position * f, bary.Velocity * f);
    }

    [Fact]
    public void Sun_earth_L1_uncontrolled_probe_departs_on_the_expected_timescale()
    {
        var (eph, earth, moon) = SunEarthMoon();
        var gravity = new GravityModel(eph);
        var (l1, v) = L1PointAndVelocity(eph, earth, moon, 0);
        double initialOffset = 1e6;
        var y0 = new StateVector(l1 - l1.Normalized() * initialOffset, v);
        var predictor = new TrajectoryPredictor(gravity, y0, 0);

        double t = 100 * 86400;
        var probe = predictor.StateAt(t);
        var (l1Later, _) = L1PointAndVelocity(eph, earth, moon, t);
        double offset = (probe.Position - l1Later).Length();
        Assert.True(offset > 10 * initialOffset,
            $"L1 instability too weak: offset grew {offset / initialOffset:F1}x in 100 days");
    }

}

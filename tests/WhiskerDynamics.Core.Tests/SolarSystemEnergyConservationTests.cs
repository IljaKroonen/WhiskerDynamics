using WhiskerDynamics.Core;
using Xunit.Abstractions;

namespace WhiskerDynamics.Core.Tests;

public class SolarSystemEnergyConservationTests(ITestOutputHelper output)
{
    private const double MuSun = 1.32712440018e20;
    private const double ShippingRelTol = 1e-11;
    private const double Day = 86400;

    private static readonly PlanetSeed[] PlanetSeeds =
    [
        new("Mercury", 2.20320e13, 5.7909e10, 0.2056, 7.005),
        new("Venus", 3.24859e14, 1.0821e11, 0.0068, 3.395),
        new("Earth", 3.986004418e14, 1.49598e11, 0.0167, 0.001),
        new("Mars", 4.282837e13, 2.2794e11, 0.0934, 1.850),
        new("Jupiter", 1.26686534e17, 7.7857e11, 0.0489, 1.303),
        new("Saturn", 3.7931187e16, 1.43353e12, 0.0565, 2.485),
        new("Uranus", 5.793939e15, 2.87246e12, 0.0457, 0.773),
        new("Neptune", 6.836529e15, 4.49506e12, 0.0113, 1.770),
    ];

    [Fact]
    public void Production_ephemeris_keeps_the_major_planet_system_stable_for_a_century()
    {
        CelestialBody[] bodies = CreateMajorPlanetSystem();
        double[] mus = bodies.Select(body => body.Mu).ToArray();
        var ephemerides = new NBodyEphemerides(
            bodies, 0, bodies.Select(body => body.Id).ToArray(),
            new IntegratorOptions { RelTol = ShippingRelTol });
        const double horizon = 100 * 365.25 * Day;
        const double sampleInterval = 10 * Day;

        _ = ephemerides.GetState(bodies[0], horizon);
        StateVector[] initial = StatesAt(ephemerides, bodies, 0);
        double initialEnergy = TotalEnergyTimesG(initial, mus);
        Vector3d initialAngularMomentum = AngularMomentumTimesG(initial, mus);
        double peakEnergyDrift = 0;
        double peakAngularMomentumDrift = 0;
        double[] minimumRadius = Enumerable.Repeat(double.PositiveInfinity, PlanetSeeds.Length).ToArray();
        double[] maximumRadius = new double[PlanetSeeds.Length];

        void Sample(double time)
        {
            StateVector[] states = StatesAt(ephemerides, bodies, time);
            peakEnergyDrift = Math.Max(peakEnergyDrift,
                Math.Abs((TotalEnergyTimesG(states, mus) - initialEnergy) / initialEnergy));
            peakAngularMomentumDrift = Math.Max(peakAngularMomentumDrift,
                (AngularMomentumTimesG(states, mus) - initialAngularMomentum).Length()
                / initialAngularMomentum.Length());
            for (int i = 0; i < PlanetSeeds.Length; i++)
            {
                double radius = (states[i + 1].Position - states[0].Position).Length();
                minimumRadius[i] = Math.Min(minimumRadius[i], radius);
                maximumRadius[i] = Math.Max(maximumRadius[i], radius);
            }
        }

        for (double time = 0; time <= horizon; time += sampleInterval)
            Sample(time);
        Sample(horizon);

        output.WriteLine($"peakEnergyRelative={peakEnergyDrift:G17}");
        output.WriteLine($"peakAngularMomentumRelative={peakAngularMomentumDrift:G17}");
        output.WriteLine($"nodes={ephemerides.NodeCount}, knots={ephemerides.KnotCount}");
        for (int i = 0; i < PlanetSeeds.Length; i++)
        {
            PlanetSeed planet = PlanetSeeds[i];
            output.WriteLine(
                $"{planet.Id}: minimumRadius/a={minimumRadius[i] / planet.SemiMajorAxis:G17}, " +
                $"maximumRadius/a={maximumRadius[i] / planet.SemiMajorAxis:G17}");
            Assert.True(minimumRadius[i]
                > (1 - planet.Eccentricity - 0.02) * planet.SemiMajorAxis,
                $"{planet.Id} moved below its allowed radial stability band");
            Assert.True(maximumRadius[i]
                < (1 + planet.Eccentricity + 0.02) * planet.SemiMajorAxis,
                $"{planet.Id} moved above its allowed radial stability band");
        }
        Assert.True(peakEnergyDrift < 2e-10,
            $"peak relative energy drift {peakEnergyDrift:R}");
        Assert.True(peakAngularMomentumDrift < 1e-10,
            $"peak relative angular momentum drift {peakAngularMomentumDrift:R}");
    }

    private static CelestialBody[] CreateMajorPlanetSystem()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var bodies = new List<CelestialBody> { sun };
        for (int i = 0; i < PlanetSeeds.Length; i++)
        {
            PlanetSeed seed = PlanetSeeds[i];
            double period = 2 * Math.PI * Math.Sqrt(
                seed.SemiMajorAxis * seed.SemiMajorAxis * seed.SemiMajorAxis / MuSun);
            bodies.Add(new CelestialBody
            {
                Id = seed.Id,
                Mu = seed.Mu,
                Parent = sun,
                Orbit = new OrbitalElements(
                    seed.SemiMajorAxis,
                    seed.Eccentricity,
                    seed.InclinationDegrees * Math.PI / 180,
                    GoldenAngle(i, 1),
                    GoldenAngle(i, 2),
                    -GoldenFraction(i, 3) * period),
            });
        }
        return [.. bodies];
    }

    private static StateVector[] StatesAt(NBodyEphemerides ephemerides,
        IReadOnlyList<CelestialBody> bodies, double time) =>
        bodies.Select(body => ephemerides.GetState(body, time)).ToArray();

    private static double TotalEnergyTimesG(IReadOnlyList<StateVector> states,
        IReadOnlyList<double> mus)
    {
        double energy = 0;
        for (int i = 0; i < states.Count; i++)
        {
            energy += 0.5 * mus[i] * states[i].Velocity.LengthSquared();
            for (int j = i + 1; j < states.Count; j++)
                energy -= mus[i] * mus[j]
                    / (states[j].Position - states[i].Position).Length();
        }
        return energy;
    }

    private static Vector3d AngularMomentumTimesG(IReadOnlyList<StateVector> states,
        IReadOnlyList<double> mus)
    {
        var angularMomentum = Vector3d.Zero;
        for (int i = 0; i < states.Count; i++)
            angularMomentum += states[i].Position.Cross(states[i].Velocity) * mus[i];
        return angularMomentum;
    }

    private static double GoldenAngle(int index, int salt) =>
        2 * Math.PI * GoldenFraction(index, salt);

    private static double GoldenFraction(int index, int salt)
    {
        double value = (index + 0.31 * salt) * 0.6180339887498949;
        return value - Math.Floor(value);
    }

    private readonly record struct PlanetSeed(
        string Id, double Mu, double SemiMajorAxis, double Eccentricity,
        double InclinationDegrees);
}

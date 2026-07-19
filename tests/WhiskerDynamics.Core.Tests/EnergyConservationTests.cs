using WhiskerDynamics.Core;
using Xunit.Abstractions;

namespace WhiskerDynamics.Core.Tests;

public class EnergyConservationTests(ITestOutputHelper output)
{
    private const double ShippingRelTol = 1e-11;
    private const double MuEarth = 3.986004418e14;
    private const double MuMoon = 4.9048695e12;
    private const double MuSun = 1.32712440018e20;

    [Theory]
    [InlineData(0.0, 10_000, 5e-7)]
    [InlineData(0.8, 2_000, 2e-7)]
    public void Production_scalar_integrator_bounds_long_term_two_body_energy_drift(
        double eccentricity, int revolutions, double maximumRelativeDrift)
    {
        const double semiMajorAxis = 7.0e6;
        var initial = PeriapsisState(MuEarth, semiMajorAxis, eccentricity);
        double period = OrbitalPeriod(MuEarth, semiMajorAxis);

        var drift = MeasureSpecificEnergyDrift(
            initial, MuEarth, revolutions * period,
            new IntegratorOptions { RelTol = ShippingRelTol });

        Report($"two-body e={eccentricity}, revolutions={revolutions}", drift);
        Assert.True(drift.PeakRelative < maximumRelativeDrift,
            $"peak relative energy drift {drift.PeakRelative:R}");
    }

    [Fact]
    public void Production_system_integrator_bounds_binary_energy_drift_over_a_century()
    {
        const double separation = 3.844e8;
        double totalMu = MuEarth + MuMoon;
        double period = OrbitalPeriod(totalMu, separation);
        int revolutions = (int)Math.Ceiling(100 * 365.25 * 86400 / period);
        double primaryRadius = separation * MuMoon / totalMu;
        double secondaryRadius = separation * MuEarth / totalMu;
        double angularSpeed = Math.Sqrt(totalMu / (separation * separation * separation));
        StateVector[] initial =
        [
            new(new Vector3d(-primaryRadius, 0, 0),
                new Vector3d(0, -angularSpeed * primaryRadius, 0)),
            new(new Vector3d(secondaryRadius, 0, 0),
                new Vector3d(0, angularSpeed * secondaryRadius, 0)),
        ];
        double[] mus = [MuEarth, MuMoon];
        double initialEnergy = TotalEnergyTimesG(initial, mus);
        double peakRelative = 0;
        double finalEnergy = initialEnergy;

        Vector3d[] Accelerations(double _, StateVector[] states)
        {
            var offset = states[1].Position - states[0].Position;
            double r2 = offset.LengthSquared();
            double inverseCube = 1 / (r2 * Math.Sqrt(r2));
            return
            [
                offset * (MuMoon * inverseCube),
                offset * (-MuEarth * inverseCube),
            ];
        }

        DormandPrince54.PropagateSystem(
            Accelerations, initial, 0, revolutions * period,
            new IntegratorOptions { RelTol = ShippingRelTol },
            (_, states, _) =>
            {
                finalEnergy = TotalEnergyTimesG(states, mus);
                peakRelative = Math.Max(peakRelative,
                    Math.Abs((finalEnergy - initialEnergy) / initialEnergy));
            });

        var drift = new EnergyDrift(
            peakRelative, (finalEnergy - initialEnergy) / Math.Abs(initialEnergy));
        Report($"mutual binary years=100, revolutions={revolutions}", drift);
        Assert.True(drift.PeakRelative < 1e-7,
            $"peak relative energy drift {drift.PeakRelative:R}");
    }

    [Fact]
    public void Production_ephemeris_preserves_total_energy_through_long_term_storage()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var earth = new CelestialBody
        {
            Id = "Earth", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.495978707e11, 0.0167, 0, 0, 0, 0),
        };
        var moon = new CelestialBody
        {
            Id = "Moon", Mu = MuMoon, Parent = earth,
            Orbit = new OrbitalElements(3.844e8, 0.0549, 0.0898, 0, 0, 0),
        };
        CelestialBody[] bodies = [sun, earth, moon];
        double[] mus = [MuSun, MuEarth, MuMoon];
        var ephemerides = new NBodyEphemerides(
            bodies, 0, bodies.Select(body => body.Id).ToArray(),
            new IntegratorOptions { RelTol = ShippingRelTol });
        const double horizon = 100 * 365.25 * 86400;
        const double sampleInterval = 30 * 86400;

        _ = ephemerides.GetState(sun, horizon);
        StateVector[] initial = bodies.Select(body => ephemerides.GetState(body, 0)).ToArray();
        double initialEnergy = TotalEnergyTimesG(initial, mus);
        double peakRelative = 0;
        double finalEnergy = initialEnergy;
        for (double time = 0; time <= horizon; time += sampleInterval)
        {
            StateVector[] states = bodies.Select(body => ephemerides.GetState(body, time)).ToArray();
            finalEnergy = TotalEnergyTimesG(states, mus);
            peakRelative = Math.Max(peakRelative,
                Math.Abs((finalEnergy - initialEnergy) / initialEnergy));
        }
        StateVector[] final = bodies.Select(body => ephemerides.GetState(body, horizon)).ToArray();
        finalEnergy = TotalEnergyTimesG(final, mus);

        var drift = new EnergyDrift(
            peakRelative, (finalEnergy - initialEnergy) / Math.Abs(initialEnergy));
        Report("Sun-Earth-Moon ephemeris years=100", drift);
        output.WriteLine($"nodes={ephemerides.NodeCount}, knots={ephemerides.KnotCount}");
        Assert.True(drift.PeakRelative < 5e-10,
            $"peak relative energy drift {drift.PeakRelative:R}");
    }

    [Fact]
    public void Long_term_energy_drift_converges_when_tolerance_is_tightened()
    {
        const double semiMajorAxis = 7.0e6;
        const double eccentricity = 0.8;
        const int revolutions = 500;
        var initial = PeriapsisState(MuEarth, semiMajorAxis, eccentricity);
        double duration = revolutions * OrbitalPeriod(MuEarth, semiMajorAxis);

        var loose = MeasureSpecificEnergyDrift(initial, MuEarth, duration,
            new IntegratorOptions { RelTol = 1e-9 });
        var shipping = MeasureSpecificEnergyDrift(initial, MuEarth, duration,
            new IntegratorOptions { RelTol = ShippingRelTol });
        var tight = MeasureSpecificEnergyDrift(initial, MuEarth, duration,
            new IntegratorOptions { RelTol = 1e-13 });

        Report("tolerance=1e-9", loose);
        Report("tolerance=1e-11", shipping);
        Report("tolerance=1e-13", tight);
        Assert.True(shipping.PeakRelative * 20 < loose.PeakRelative,
            $"shipping {shipping.PeakRelative:R}, loose {loose.PeakRelative:R}");
        Assert.True(tight.PeakRelative * 20 < shipping.PeakRelative,
            $"tight {tight.PeakRelative:R}, shipping {shipping.PeakRelative:R}");
    }

    private static EnergyDrift MeasureSpecificEnergyDrift(
        StateVector initial, double mu, double duration, IntegratorOptions options)
    {
        double initialEnergy = SpecificEnergy(initial, mu);
        double peakRelative = 0;
        double finalEnergy = initialEnergy;

        Vector3d Acceleration(double _, StateVector state)
        {
            double r2 = state.Position.LengthSquared();
            return state.Position * (-mu / (r2 * Math.Sqrt(r2)));
        }

        DormandPrince54.Propagate(
            Acceleration, initial, 0, duration, options,
            (_, state) =>
            {
                finalEnergy = SpecificEnergy(state, mu);
                peakRelative = Math.Max(peakRelative,
                    Math.Abs((finalEnergy - initialEnergy) / initialEnergy));
            });

        return new EnergyDrift(
            peakRelative, (finalEnergy - initialEnergy) / Math.Abs(initialEnergy));
    }

    private static StateVector PeriapsisState(double mu, double semiMajorAxis,
        double eccentricity)
    {
        double radius = semiMajorAxis * (1 - eccentricity);
        double speed = Math.Sqrt(mu * (2 / radius - 1 / semiMajorAxis));
        return new StateVector(new Vector3d(radius, 0, 0), new Vector3d(0, speed, 0));
    }

    private static double OrbitalPeriod(double mu, double semiMajorAxis) =>
        2 * Math.PI * Math.Sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / mu);

    private static double SpecificEnergy(StateVector state, double mu) =>
        0.5 * state.Velocity.LengthSquared() - mu / state.Position.Length();

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

    private void Report(string scenario, EnergyDrift drift)
    {
        output.WriteLine(scenario);
        output.WriteLine($"peakRelative={drift.PeakRelative:G17}");
        output.WriteLine($"finalSignedRelative={drift.FinalSignedRelative:G17}");
    }

    private readonly record struct EnergyDrift(
        double PeakRelative, double FinalSignedRelative);
}

using System.Diagnostics;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Reproducible runtime-versus-accuracy sweep for the production DP5(4)
/// integrator. The manual command reports errors and work counts; the BDN class
/// provides statistically controlled runtime and allocation measurements.</summary>
public static class IntegratorToleranceSweep
{
    private const double MuEarth = 3.986004418e14;
    private static readonly double[] Tolerances = [1e-9, 1e-10, 1e-11, 1e-12];

    private readonly record struct Sample(StateVector End, long Work, int Steps);

    public static void Run()
    {
        Console.WriteLine("DP5(4) tolerance sweep; median of 5 steady-state runs; .NET Stopwatch");
        RunCircularLeo();
        RunEccentric();
        RunFullGravityLeo();
    }

    private static void RunCircularLeo()
    {
        const double radius = 6.771e6;
        const double horizon = 86400;
        double speed = Math.Sqrt(MuEarth / radius);
        double angle = horizon * Math.Sqrt(MuEarth / (radius * radius * radius));
        var initial = new StateVector(new Vector3d(radius, 0, 0), new Vector3d(0, speed, 0));
        var exact = new StateVector(
            new Vector3d(radius * Math.Cos(angle), radius * Math.Sin(angle), 0),
            new Vector3d(-speed * Math.Sin(angle), speed * Math.Cos(angle), 0));
        RunPointMassRows("Circular LEO, 1 day, analytic reference", initial, horizon, exact);
    }

    private static void RunEccentric()
    {
        var elements = new OrbitalElements(2.0e7, 0.7, 0.3, 0.5, 1.0, 0);
        double period = 2 * Math.PI * Math.Sqrt(Math.Pow(elements.SemiMajorAxis, 3) / MuEarth);
        double horizon = 10.25 * period;
        var initial = Kepler.StateFromElements(elements, MuEarth, 0);
        var exact = Kepler.StateFromElements(elements, MuEarth, horizon);
        RunPointMassRows("Eccentric e=0.7, 10.25 periods, analytic Kepler reference",
            initial, horizon, exact);
    }

    private static void RunPointMassRows(string label, StateVector initial, double horizon, StateVector exact)
    {
        PrintHeader(label);
        foreach (double tolerance in Tolerances)
        {
            var options = new IntegratorOptions { RelTol = tolerance };
            var measured = Measure(() =>
            {
                long rhs = 0;
                int steps = 0;
                var end = DormandPrince54.Propagate(
                    (t, s) => { rhs++; return PointMass(s.Position); },
                    initial, 0, horizon, options, (t, s) => steps++);
                return new Sample(end, rhs, steps);
            });
            PrintRow(tolerance, measured.Milliseconds, measured.Value, exact);
        }
    }

    private static void RunFullGravityLeo()
    {
        var (gravity, leo, _) = BenchmarkCatalog.CreateVesselCases();
        var reference = Predict(gravity, leo, 1e-13);
        PrintHeader("Full-catalog gravity LEO, 30 days + two burns, DP54 @ 1e-13 reference");
        foreach (double tolerance in Tolerances)
        {
            var measured = Measure(() => Predict(gravity, leo, tolerance));
            PrintRow(tolerance, measured.Milliseconds, measured.Value, reference.End);
        }
    }

    private static Sample Predict(GravityModel gravity, StateVector initial, double tolerance)
    {
        var options = new IntegratorOptions { RelTol = tolerance };
        StateVector state = initial;
        double time = 0;
        long rhs = 0;
        int steps = 0;

        Vector3d Acceleration(double t, StateVector s)
        {
            rhs++;
            return gravity.AccelerationAt(s.Position, t);
        }

        void AdvanceTo(double target)
        {
            state = DormandPrince54.Propagate(
                Acceleration, state, time, target, options, (t, s) => steps++);
            time = target;
        }

        AdvanceTo(5 * 86400.0);
        state = state with { Velocity = state.Velocity + new Vector3d(25, 10, 0) };
        AdvanceTo(15 * 86400.0);
        state = state with { Velocity = state.Velocity + new Vector3d(-15, 20, 5) };
        AdvanceTo(BenchmarkCatalog.VesselHorizonSeconds);
        return new Sample(state, rhs, steps);
    }

    private static (Sample Value, double Milliseconds) Measure(Func<Sample> run)
    {
        _ = run(); // tier-up and populate ephemeris segment caches
        const int repetitions = 5;
        var elapsed = new double[repetitions];
        Sample value = default;
        var stopwatch = new Stopwatch();
        for (int i = 0; i < repetitions; i++)
        {
            stopwatch.Restart();
            value = run();
            stopwatch.Stop();
            elapsed[i] = stopwatch.Elapsed.TotalMilliseconds;
        }
        Array.Sort(elapsed);
        return (value, elapsed[repetitions / 2]);
    }

    private static void PrintHeader(string label)
    {
        Console.WriteLine();
        Console.WriteLine(label);
        Console.WriteLine("RelTol       median ms       RHS      steps        position error       velocity error");
    }

    private static void PrintRow(double tolerance, double milliseconds, Sample value, StateVector exact)
    {
        double positionError = (value.End.Position - exact.Position).Length();
        double velocityError = (value.End.Velocity - exact.Velocity).Length();
        string rhs = value.Work == 0 ? "n/a" : value.Work.ToString("N0");
        Console.WriteLine(
            $"{tolerance,8:0e0} {milliseconds,14:F3} {rhs,9} {value.Steps,10:N0}  "
            + $"{positionError,18:G8} m {velocityError,18:G8} m/s");
    }

    internal static Vector3d PointMass(Vector3d position)
    {
        double r2 = position.LengthSquared();
        return position * (-MuEarth / (r2 * Math.Sqrt(r2)));
    }

    internal static (StateVector Initial, double Horizon) EccentricCase()
    {
        var elements = new OrbitalElements(2.0e7, 0.7, 0.3, 0.5, 1.0, 0);
        double period = 2 * Math.PI * Math.Sqrt(Math.Pow(elements.SemiMajorAxis, 3) / MuEarth);
        return (Kepler.StateFromElements(elements, MuEarth, 0), 10.25 * period);
    }
}

/// <summary>BenchmarkDotNet companion to <see cref="IntegratorToleranceSweep"/>.</summary>
[MemoryDiagnoser]
[SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 2, iterationCount: 8, invocationCount: 1)]
public class IntegratorToleranceBenchmarks
{
    [Params(1e-9, 1e-10, 1e-11, 1e-12)]
    public double RelTol { get; set; }

    private StateVector _eccentric;
    private double _eccentricHorizon;
    private GravityModel _gravity = null!;
    private StateVector _leo;
    private IntegratorOptions _options = null!;

    [GlobalSetup]
    public void Setup()
    {
        (_eccentric, _eccentricHorizon) = IntegratorToleranceSweep.EccentricCase();
        (_gravity, _leo, _) = BenchmarkCatalog.CreateVesselCases();
        _options = new IntegratorOptions { RelTol = RelTol };
    }

    [Benchmark]
    public StateVector PointMassEccentric10p25Orbits() =>
        DormandPrince54.Propagate(
            (t, s) => IntegratorToleranceSweep.PointMass(s.Position),
            _eccentric, 0, _eccentricHorizon, _options);

    [Benchmark]
    public StateVector FullGravityLeo30DaysTwoBurns()
    {
        var predictor = new TrajectoryPredictor(_gravity, _leo, 0, _options);
        predictor.AddImpulse(5 * 86400.0, new Vector3d(25, 10, 0));
        predictor.AddImpulse(15 * 86400.0, new Vector3d(-15, 20, 5));
        return predictor.StateAt(BenchmarkCatalog.VesselHorizonSeconds);
    }
}

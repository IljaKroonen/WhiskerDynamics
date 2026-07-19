using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Performance coverage for production numerical kernels.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class RendezvousPeriapsisKernelBenchmarks
{
    private const double Mu = 3.986004418e14;
    private const double Radius = 7_000_000.0;
    private static readonly double Period =
        2.0 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);
    private static readonly Vector3d From = new(Radius, 0, 0);
    private static readonly Vector3d To =
        new(Radius * 0.5, Radius * Math.Sqrt(3.0) / 2.0, 0);
    private static readonly (
        TrajectoryPredictor Predictor, TrajectoryPredictor Center,
        TrajectoryNode Start, TrajectoryNode End)
        ClearanceCase = CreateClearanceCase();

    [Benchmark]
    public IReadOnlyList<RendezvousKernel.LambertSolution> LambertZeroRevolution() =>
        RendezvousKernel.SolveLambert(From, To, Period / 6.0, Mu,
            longWay: false, revolutions: 0);

    [Benchmark]
    public IReadOnlyList<RendezvousKernel.LambertSolution> LambertTwoRevolutions() =>
        RendezvousKernel.SolveLambert(From, To, 13.0 * Period / 6.0, Mu,
            longWay: false, revolutions: 2);

    [Benchmark]
    public (double PeriapsisMeters, bool Interior) ScanFirstPeriapsis() =>
        PeriapsisKernel.ScanFirstPeriapsis(ScanDistance, 0.0, 1000.0);

    [Benchmark]
    public DvMinimum? CompassMinimumDeltaV() =>
        PeriapsisKernel.MinimizeDeltaV(Bowl,
            time0: 400.0, normal0: 6.0, outward0: -4.0,
            timeLo: 0.0, timeHi: 2000.0,
            timeStep: 200.0, dvStep: 8.0,
            timeStepFloor: 0.5, dvStepFloor: 0.01,
            cancelled: NeverCancelled);

    [Benchmark]
    public bool CurvatureAwareCollisionAdmission() =>
        RendezvousFiniteEvaluator.SegmentClearsSphere(
            ClearanceCase.Predictor, radius: 1.0,
            ClearanceCase.Start, ClearanceCase.End, depth: 6,
            static time => ClearanceCase.Center.StateAt(time).Position,
            static (path, time) => path.StateAt(time), NeverCancelled);

    private static double ScanDistance(double time) => time < 500.0
        ? 8.0e6 + 40.0 * (time - 250.0) * (time - 250.0)
        : 5.0e6 + 40.0 * (time - 750.0) * (time - 750.0);

    private static (double Prograde, double Achieved)? Bowl(
        double time, double normal, double outward, double hint) =>
        (10.0 + 1e-4 * (time - 1000.0) * (time - 1000.0)
              + 0.5 * normal * normal + 0.5 * outward * outward,
            7.0e6);

    private static bool NeverCancelled() => false;

    private static (TrajectoryPredictor, TrajectoryPredictor,
        TrajectoryNode, TrajectoryNode)
        CreateClearanceCase()
    {
        var primary = new CelestialBody { Id = "Primary", Mu = 1000.0 };
        var predictor = new TrajectoryPredictor(
            new GravityModel(new Ephemerides([primary])),
            new StateVector(new Vector3d(10.0, 0.0, 0.0),
                new Vector3d(0.0, 10.0, 0.0)),
            initialTime: 0.0, new IntegratorOptions { RelTol = 1e-12 });
        _ = predictor.StateAt(Math.PI / 2.0);
        int end = 1;
        for (int i = 2; i < predictor.Nodes.Count; i++)
            if (predictor.Nodes[i].Time - predictor.Nodes[i - 1].Time
                > predictor.Nodes[end].Time - predictor.Nodes[end - 1].Time)
                end = i;
        var start = predictor.Nodes[end - 1];
        var finish = predictor.Nodes[end];
        var center = new TrajectoryPredictor(
            new GravityModel(new Ephemerides([])),
            new StateVector(Vector3d.Zero, new Vector3d(0.1, 0.0, 0.0)), start.Time);
        _ = center.StateAt(finish.Time);
        return (predictor, center, start, finish);
    }
}

using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Direct scalar-integrator hot-path coverage. The full-system and
/// end-to-end predictor costs live in the existing benchmark classes; these
/// cases isolate controller, stage construction, and error-norm overhead.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class IntegrationNumericsBenchmarks
{
    private const double MuEarth = 3.986004418e14;
    private readonly IntegratorOptions _shipping = new()
    {
        RelTol = BenchmarkCatalog.ShippingRelTol,
        InitialStep = 60,
    };
    private readonly IntegratorOptions _oneStep = new()
    {
        RelTol = BenchmarkCatalog.ShippingRelTol,
        InitialStep = 60,
        MaxStep = 60,
    };

    private StateVector _leo;
    private StateVector[] _system = null!;

    [GlobalSetup]
    public void Setup()
    {
        const double r = 6.771e6;
        _leo = new StateVector(
            new Vector3d(r, 0, 0),
            new Vector3d(0, Math.Sqrt(MuEarth / r), 0));
        _system = Enumerable.Repeat(_leo, 50).ToArray();
    }

    [Benchmark(Baseline = true)]
    public StateVector DP54_PointMass_OneDay() =>
        DormandPrince54.Propagate(Acceleration, _leo, 0, 86400, _shipping);

    [Benchmark]
    public StateVector DP853_PointMass_OneDay() =>
        DormandPrince853.Propagate(Acceleration, _leo, 0, 86400, out _, _shipping);

    [Benchmark]
    public StateVector DP54_OneAcceptedStep() =>
        DormandPrince54.Propagate(Acceleration, _leo, 0, 60, _oneStep);

    [Benchmark]
    public StateVector DP54_ZeroDuration() =>
        DormandPrince54.Propagate(Acceleration, _leo, 1000, 1000, _shipping);

    [Benchmark]
    public StateVector DP853_ZeroDuration() =>
        DormandPrince853.Propagate(Acceleration, _leo, 1000, 1000, out _, _shipping);

    [Benchmark]
    public StateVector[] DP54_SystemZeroDuration_50Bodies() =>
        DormandPrince54.PropagateSystem(SystemAcceleration, _system, 1000, 1000, _shipping);

    private static Vector3d Acceleration(double time, StateVector state)
    {
        double r2 = state.Position.LengthSquared();
        return state.Position * (-MuEarth / (r2 * Math.Sqrt(r2)));
    }

    private static Vector3d[] SystemAcceleration(double time, StateVector[] states) =>
        throw new InvalidOperationException("A zero-duration propagation must not evaluate its RHS.");
}

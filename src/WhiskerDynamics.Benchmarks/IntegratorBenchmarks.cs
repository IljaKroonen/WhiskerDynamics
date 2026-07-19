using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Benchmarks fixed-step integrator overhead and one day of adaptive
/// full-system rails propagation.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class IntegratorBenchmarks
{
    private const int VesselSteps = 64;
    private const int SystemSteps = 16;
    private const double SunMu = 1.32712440018e20;

    private readonly IntegratorOptions _fixedStep = new()
    {
        RelTol = BenchmarkCatalog.ShippingRelTol, InitialStep = 60, MaxStep = 60,
    };
    private readonly IntegratorOptions _adaptive = new()
    {
        RelTol = BenchmarkCatalog.ShippingRelTol,
    };

    private StateVector _heliocentric;
    private StateVector[] _railsInitial = null!;
    private PairwiseAccelerationKernel _pairwise = null!;
    private Vector3d[] _accBuffer = null!;
    private StateVector[] _allMutualInitial = null!;
    private PairwiseAccelerationKernel _allMutualPairwise = null!;
    private Vector3d[] _allMutualAccBuffer = null!;

    [GlobalSetup]
    public void Setup()
    {
        // Circular heliocentric orbit at 1 AU: a smooth two-body RHS so the
        // single-step benchmark measures integrator overhead, not gravity cost.
        double r = 1.49598e11;
        _heliocentric = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, Math.Sqrt(SunMu / r), 0));

        var bodies = BenchmarkCatalog.CreateBodies();
        var kepler = new Ephemerides(bodies);
        var backbone = bodies.Where(b => BenchmarkCatalog.BackboneIds.Contains(b.Id)).ToArray();
        _railsInitial = backbone.Select(b => kepler.GetState(b, 0.0)).ToArray();
        _pairwise = new PairwiseAccelerationKernel(backbone.Select(b => b.Mu).ToArray());
        _accBuffer = new Vector3d[backbone.Length];
        _allMutualInitial = bodies.Select(b => kepler.GetState(b, 0.0)).ToArray();
        _allMutualPairwise = new PairwiseAccelerationKernel(bodies.Select(b => b.Mu).ToArray());
        _allMutualAccBuffer = new Vector3d[bodies.Count];
    }

    [Benchmark(OperationsPerInvoke = VesselSteps)]
    public StateVector SingleSteps_PointMass() =>
        DormandPrince54.Propagate(
            (t, s) => PointMassAcceleration(s.Position),
            _heliocentric, 0, 60.0 * VesselSteps, _fixedStep);

    [Benchmark(OperationsPerInvoke = SystemSteps)]
    public StateVector[] SystemSteps_50Bodies() =>
        DormandPrince54.PropagateSystem(
            (t, s) => MutualAccelerations(s),
            _railsInitial, 0, 60.0 * SystemSteps, _fixedStep);

    [Benchmark(OperationsPerInvoke = SystemSteps)]
    public StateVector[] SystemSteps_99Bodies_AllMutual() =>
        DormandPrince54.PropagateSystem(
            (t, s) => AllMutualAccelerations(s),
            _allMutualInitial, 0, 60.0 * SystemSteps, _fixedStep);

    [Benchmark]
    public StateVector[] AdvanceSystem1Day_50Bodies_Adaptive() =>
        DormandPrince54.PropagateSystem(
            (t, s) => MutualAccelerations(s),
            _railsInitial, 0, 86400, _adaptive);

    private static Vector3d PointMassAcceleration(Vector3d position)
    {
        double r2 = position.LengthSquared();
        return position * (-SunMu / (r2 * Math.Sqrt(r2)));
    }

    private Vector3d[] MutualAccelerations(StateVector[] states)
    {
        // Use the same pairwise RHS kernel as NBodyEphemerides.
        _pairwise.Compute(states, _accBuffer);
        return _accBuffer;
    }

    private Vector3d[] AllMutualAccelerations(StateVector[] states)
    {
        _allMutualPairwise.Compute(states, _allMutualAccBuffer);
        return _allMutualAccBuffer;
    }
}

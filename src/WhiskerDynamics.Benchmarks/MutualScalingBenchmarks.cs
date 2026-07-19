using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Measures the quadratic mutual-gravity RHS at representative catalog
/// sizes. The 99-body case is the complete benchmark catalog; the 128-body case adds
/// deterministic outer bodies so an uncapped future catalog is visible in results.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class MutualScalingBenchmarks
{
    [Params(64, 99, 128)]
    public int BodyCount { get; set; }

    private StateVector[] _states = null!;
    private PairwiseAccelerationKernel _pairwise = null!;
    private Vector3d[] _accelerations = null!;

    [GlobalSetup]
    public void Setup()
    {
        var bodies = BenchmarkCatalog.CreateBodies();
        var seed = new Ephemerides(bodies);
        var states = bodies.Select(body => seed.GetState(body, 0.0)).ToList();
        var mus = bodies.Select(body => body.Mu).ToList();

        for (int i = bodies.Count; i < BodyCount; i++)
        {
            // Extrapolated bodies occupy distinct outer-system points. Exact orbital
            // fidelity is irrelevant to RHS throughput, but finite separated inputs
            // keep the measured arithmetic representative.
            double phase = i * 2.399963229728653;
            double radius = 1.2e13 + (i - bodies.Count + 1) * 8.0e10;
            states.Add(new StateVector(
                new Vector3d(radius * Math.Cos(phase), radius * Math.Sin(phase),
                    radius * 0.01 * Math.Sin(phase * 0.37)),
                Vector3d.Zero));
            mus.Add(1.0e9 + i * 1.0e7);
        }

        _states = states.Take(BodyCount).ToArray();
        _pairwise = new PairwiseAccelerationKernel(mus.Take(BodyCount).ToArray());
        _accelerations = new Vector3d[BodyCount];
    }

    [Benchmark]
    public Vector3d[] PairwiseAccelerations()
    {
        _pairwise.Compute(_states, _accelerations);
        return _accelerations;
    }
}

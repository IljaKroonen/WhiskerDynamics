using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Measures the allocation saved by reusing the immutable default
/// integrator options in objects that retain the options reference.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class IntegratorDefaultsBenchmarks
{
    private static readonly GravityModel EmptyGravity = new(new EmptyEphemerides(), []);
    private static readonly StateVector Initial = new(
        new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

    [Benchmark(Baseline = true)]
    public TrajectoryPredictor ConstructWithFreshDefaultOptions() =>
        new(EmptyGravity, Initial, 0, new IntegratorOptions());

    [Benchmark]
    public TrajectoryPredictor ConstructWithSharedDefaultOptions() =>
        new(EmptyGravity, Initial, 0);

    private sealed class EmptyEphemerides : IEphemerides
    {
        public IReadOnlyList<CelestialBody> Bodies => Array.Empty<CelestialBody>();
        public CelestialBody this[string id] => throw new KeyNotFoundException(id);
        public StateVector GetState(CelestialBody body, double time) =>
            throw new InvalidOperationException();
    }
}

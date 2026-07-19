using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Large memoized-trajectory maintenance. Iteration setup creates an
/// exact 50,001-node constant-velocity trajectory; the measured operations are
/// destructive and therefore run once per iteration.</summary>
[MemoryDiagnoser]
[SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 3, iterationCount: 12, invocationCount: 1)]
public class TrajectoryCacheMaintenanceBenchmarks
{
    private static readonly GravityModel EmptyGravity = new(new EmptyEphemerides(), []);
    private static readonly IntegratorOptions UnitSteps = new()
    {
        InitialStep = 1,
        MaxStep = 1,
    };

    private static readonly StateVector Initial = new(
        new Vector3d(1, 2, 3),
        new Vector3d(4, 5, 6));

    private TrajectoryPredictor _predictor = null!;

    [IterationSetup]
    public void SetupIteration()
    {
        _predictor = new TrajectoryPredictor(EmptyGravity, Initial, 0, UnitSteps);
        _predictor.ExtendTo(50_000);
    }

    [Benchmark]
    public int PruneFirst49kNodes()
    {
        _predictor.PruneBefore(49_000);
        return _predictor.Nodes.Count;
    }

    [Benchmark]
    public int AddPastImpulseAndTruncate49kNodes()
    {
        _predictor.AddImpulse(1_000, new Vector3d(1, 2, 3));
        return _predictor.Nodes.Count;
    }

    private sealed class EmptyEphemerides : IEphemerides
    {
        public IReadOnlyList<CelestialBody> Bodies => Array.Empty<CelestialBody>();

        public CelestialBody this[string id] => throw new KeyNotFoundException(id);

        public StateVector GetState(CelestialBody body, double time) =>
            throw new InvalidOperationException("The empty benchmark ephemerides has no bodies.");
    }
}

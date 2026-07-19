using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Vessel prediction for LEO and high-orbit cases with full-catalog gravity,
/// pre-extended rails, and two impulsive burns.</summary>
[MemoryDiagnoser]
[SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 2, iterationCount: 10, invocationCount: 1)]
public class PredictorBenchmarks
{
    private GravityModel _gravity = null!;
    private StateVector _leo;
    private StateVector _highOrbit;

    [GlobalSetup]
    public void Setup() => (_gravity, _leo, _highOrbit) = BenchmarkCatalog.CreateVesselCases();

    [Benchmark]
    public StateVector PredictLeo30Days_TwoBurns() => Predict(_leo);

    [Benchmark]
    public StateVector PredictHighOrbit30Days_TwoBurns() => Predict(_highOrbit);

    private StateVector Predict(StateVector initial)
    {
        var predictor = new TrajectoryPredictor(_gravity, initial, 0.0,
            new IntegratorOptions { RelTol = BenchmarkCatalog.ShippingRelTol });
        predictor.AddImpulse(5 * 86400.0, new Vector3d(25, 10, 0));
        predictor.AddImpulse(15 * 86400.0, new Vector3d(-15, 20, 5));
        return predictor.StateAt(BenchmarkCatalog.VesselHorizonSeconds);
    }
}

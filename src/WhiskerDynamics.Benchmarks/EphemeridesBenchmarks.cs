using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Benchmarks cold-start full-system horizon extension and node caching.</summary>
[MemoryDiagnoser]
[SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 2, iterationCount: 10, invocationCount: 1)]
public class EphemeridesExtendBenchmarks
{
    [Params(1, 30)]
    public int Days;

    private NBodyEphemerides _fresh = null!;
    private CelestialBody _neptune = null!;

    [IterationSetup]
    public void IterationSetup()
    {
        _fresh = BenchmarkCatalog.CreateEphemerides();
        _neptune = _fresh["Neptune"];
    }

    [Benchmark]
    public StateVector ExtendHorizon() => _fresh.GetState(_neptune, Days * 86400.0);
}

/// <summary>Benchmarks a 30-day rails window advanced and pruned one day at a time.</summary>
[MemoryDiagnoser]
[SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 2, iterationCount: 10, invocationCount: 1)]
public class EphemeridesSlidingWindowBenchmarks
{
    private const double WindowSeconds = 30 * 86400.0;

    private NBodyEphemerides _ephemerides = null!;
    private CelestialBody _neptune = null!;
    private double _horizon;

    [GlobalSetup]
    public void Setup()
    {
        _ephemerides = BenchmarkCatalog.CreateEphemerides();
        _neptune = _ephemerides["Neptune"];
        _horizon = WindowSeconds;
        _ = _ephemerides.GetState(_neptune, _horizon); // build the initial 30-day window
    }

    [Benchmark]
    public StateVector SlideWindow1Day()
    {
        _horizon += 86400.0;
        var tip = _ephemerides.GetState(_neptune, _horizon);
        _ephemerides.Prune(_horizon - WindowSeconds);
        return tip;
    }
}

/// <summary>Benchmarks dense state sampling from an extended composite cache for both
/// a mutual-backbone body and a finite-mass restricted track.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class EphemeridesSamplingBenchmarks
{
    private const int Samples = 2000;
    private const double WindowSeconds = 30 * 86400.0;

    private NBodyEphemerides _ephemerides = null!;
    private CelestialBody _neptune = null!;
    private CelestialBody _phobos = null!;

    [GlobalSetup]
    public void Setup()
    {
        _ephemerides = BenchmarkCatalog.CreateEphemerides();
        _neptune = _ephemerides["Neptune"];
        _phobos = _ephemerides["Phobos"];
        _ = _ephemerides.GetState(_neptune, WindowSeconds + 3600); // pre-extend past the window
    }

    [Benchmark(OperationsPerInvoke = Samples)]
    public double SampleBackbone_2000PointArc() => Sample(_neptune);

    [Benchmark(OperationsPerInvoke = Samples)]
    public double SampleRestrictedTrack_2000PointArc() => Sample(_phobos);

    private double Sample(CelestialBody body)
    {
        double sum = 0;
        for (int i = 0; i < Samples; i++)
        {
            double t = WindowSeconds * i / (Samples - 1);
            sum += _ephemerides.GetState(body, t).Position.X;
        }
        return sum;
    }
}

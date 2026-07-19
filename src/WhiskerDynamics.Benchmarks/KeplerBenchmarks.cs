using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Kepler conversions in batch (1024 ops over the catalog's 98 orbits cycled
/// at varied epochs). Element-to-state supplies the composite ephemeris's initial
/// seeds; state-to-elements supplies planner and stock-conic compatibility mirrors.
/// The anomaly solver is the iterative core of element-to-state.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class KeplerBenchmarks
{
    private const int Batch = 1024;

    private (OrbitalElements Elements, double ParentMu, double Time)[] _orbits = null!;
    private (StateVector State, double ParentMu, double Time)[] _states = null!;
    private (double MeanAnomaly, double Eccentricity)[] _anomalies = null!;

    [GlobalSetup]
    public void Setup()
    {
        var bodies = BenchmarkCatalog.CreateBodies();
        var orbiting = bodies.Where(b => b.Parent is not null).ToArray();

        _orbits = new (OrbitalElements, double, double)[Batch];
        for (int i = 0; i < Batch; i++)
        {
            var body = orbiting[i % orbiting.Length];
            _orbits[i] = (body.Orbit!.Value, body.Parent!.Mu, i * 3600.0);
        }

        _states = _orbits
            .Select(o => (Kepler.StateFromElements(o.Elements, o.ParentMu, o.Time), o.ParentMu, o.Time))
            .ToArray();

        _anomalies = _orbits
            .Select((o, i) => (2 * Math.PI * i / Batch, o.Elements.Eccentricity))
            .ToArray();
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double StateFromElements_Batch()
    {
        double sum = 0;
        foreach (var (elements, mu, time) in _orbits)
            sum += Kepler.StateFromElements(elements, mu, time).Position.X;
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double ElementsFromState_Batch()
    {
        double sum = 0;
        foreach (var (state, mu, time) in _states)
            sum += Kepler.ElementsFromState(state, mu, time).SemiMajorAxis;
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double SolveEccentricAnomaly_Batch()
    {
        double sum = 0;
        foreach (var (meanAnomaly, eccentricity) in _anomalies)
            sum += Kepler.SolveEccentricAnomaly(meanAnomaly, eccentricity);
        return sum;
    }
}

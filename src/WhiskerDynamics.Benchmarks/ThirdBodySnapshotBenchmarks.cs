using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Cold refresh of every parent at one timestamp. Sixteen sources represent
/// a stock-scale system; 64 and 99 exercise dense generated catalogs.</summary>
[MemoryDiagnoser]
[ShortRunJob]
[InvocationCount(1)]
public class ThirdBodySnapshotBenchmarks
{
    [Params(16, 64, 99)]
    public int SourceCount { get; set; }

    private RailsService _rails = null!;
    private string[] _parentIds = null!;
    private static readonly Vector3d ParentRelativePosition =
        new(1.0e6, 2.0e5, -3.0e5);

    [IterationSetup]
    public void Setup()
    {
        var bodies = CreateCatalog(SourceCount);
        _parentIds = bodies.Select(body => body.Id).ToArray();
        _rails = RailsService.CreateForSyntheticCatalog(bodies, _parentIds);
    }

    [IterationCleanup]
    public void Cleanup() => _rails.Dispose();

    [Benchmark]
    public double RefreshEveryParentAtOneTime()
    {
        double checksum = 0.0;
        foreach (string parentId in _parentIds)
        {
            Vector3d delta = _rails.ThirdBodyDelta(
                parentId, ParentRelativePosition, 0.0);
            checksum += delta.X + delta.Y + delta.Z;
        }
        return checksum;
    }

    private static IReadOnlyList<CelestialBody> CreateCatalog(int sourceCount)
    {
        const double rootMu = 1.32712440018e20;
        var root = new CelestialBody
        {
            Id = "Root",
            Mu = rootMu,
            MeanRadius = 6.957e8,
        };
        var bodies = new List<CelestialBody>(sourceCount) { root };
        for (int i = 1; i < sourceCount; i++)
        {
            double semiMajorAxis = 4.0e10 + i * 7.5e8;
            bodies.Add(new CelestialBody
            {
                Id = $"Body{i:D3}",
                Mu = 5.0e8 + i * 1.0e7,
                MeanRadius = 1.0e3 + i,
                Parent = root,
                Orbit = new OrbitalElements(
                    semiMajorAxis, 0.001 * (i % 10), 0.002 * (i % 7),
                    i * 0.13, i * 0.17, i * -1000.0),
            });
        }
        return bodies;
    }
}

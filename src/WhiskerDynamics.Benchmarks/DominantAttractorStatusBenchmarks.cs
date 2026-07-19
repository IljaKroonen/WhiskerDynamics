using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>
/// Measures the production status cadence across representative uncapped massive-
/// backbone sizes. The refresh callback
/// follows the real status-provider data path: one parent lookup, absolute-position
/// assembly, then <see cref="DominantAttractor.TryCompute"/> over the real
/// <see cref="RailsService.TryGetAbsoluteMany"/> folded Gate read. Only the game-side
/// CCI-to-ECL adapter is replaced by an already-ECL vessel offset, keeping KSA out of
/// the benchmark process without replacing the operation whose cost matters.
/// </summary>
[MemoryDiagnoser]
[ShortRunJob]
[GroupBenchmarksBy(BenchmarkLogicalGroupRule.ByCategory)]
[CategoriesColumn]
public class DominantAttractorStatusBenchmarks
{
    private const string RootId = "DenseRoot";
    private const long RefreshMilliseconds = 500;
    private const int RenderedFrames = 60;
    private const double SampleTime = 0.0;
    private static readonly Vector3d VesselParentRelativeEcl =
        new(1.25e9, -7.5e8, 2.5e8);

    [Params(64, 99, 128)]
    public int GravitySourceCount { get; set; }

    private RailsService _rails = null!;
    private StatusTelemetryCache _dueRefreshCache = null!;
    private StatusTelemetryCache _cacheHitCache = null!;
    private StatusTelemetryCache _renderedCadenceCache = null!;
    private Func<IReadOnlyList<string>> _refresh = null!;
    private long _dueWallMilliseconds;

    [GlobalSetup]
    public void Setup()
    {
        var bodies = CreateDenseCatalog(GravitySourceCount);
        _rails = RailsService.CreateForSyntheticCatalog(
            bodies, bodies.Select(body => body.Id).ToArray(), SampleTime);
        if (_rails.VesselGravity.Sources.Count != GravitySourceCount)
            throw new InvalidOperationException(
                $"dense fixture exposed {_rails.VesselGravity.Sources.Count} gravity sources; "
                + $"expected {GravitySourceCount}");

        _refresh = RefreshStatus;
        _dueRefreshCache = new StatusTelemetryCache(RefreshMilliseconds);
        _cacheHitCache = new StatusTelemetryCache(RefreshMilliseconds);
        _renderedCadenceCache = new StatusTelemetryCache(RefreshMilliseconds);
        _dueWallMilliseconds = -RefreshMilliseconds;

        // Put only the hit benchmark in steady state outside measurement. The due
        // and rendered-cadence cases intentionally pay their first refresh in-band.
        var primed = _cacheHitCache.Read(0, _refresh);
        if (Checksum(primed) == 0 || !primed[0].EndsWith(RootId, StringComparison.Ordinal))
            throw new InvalidOperationException("dense status fixture produced no stable result");
    }

    [GlobalCleanup]
    public void Cleanup() => _rails.Dispose();

    /// <summary>One human-visible refresh: parent state plus every gravity source.</summary>
    [Benchmark]
    [BenchmarkCategory("DueRefresh")]
    public int DueRefresh_AllSources()
    {
        _dueWallMilliseconds += RefreshMilliseconds;
        return Checksum(_dueRefreshCache.Read(_dueWallMilliseconds, _refresh));
    }

    /// <summary>A rendered frame inside the cache window: no Rails read or allocation.</summary>
    [Benchmark]
    [BenchmarkCategory("CacheHit")]
    public int CacheHit() => Checksum(_cacheHitCache.Read(1, _refresh));

    /// <summary>One rendered second at 60 Hz. Frames 0 and 30 refresh; the other 58
    /// reuse the same status result.</summary>
    [Benchmark(OperationsPerInvoke = RenderedFrames)]
    [BenchmarkCategory("RenderedCadence")]
    public int RenderedSecondAt60Fps()
    {
        _renderedCadenceCache.Reset();
        int checksum = 17;
        for (int frame = 0; frame < RenderedFrames; frame++)
        {
            long wallMilliseconds = frame * 1000L / RenderedFrames;
            checksum = unchecked(checksum * 31
                + Checksum(_renderedCadenceCache.Read(wallMilliseconds, _refresh)));
        }
        return checksum;
    }

    private IReadOnlyList<string> RefreshStatus()
    {
        if (!_rails.TryGetAbsolute(RootId, SampleTime, out var parentAbsolute)) return [];
        Vector3d vesselAbsolute = parentAbsolute.Position + VesselParentRelativeEcl;
        if (!DominantAttractor.TryCompute(
                _rails, vesselAbsolute, SampleTime, out string dominant)) return [];
        return [$"controlled: stock parent {RootId}, dominant attractor {dominant}"];
    }

    private static int Checksum(IReadOnlyList<string> lines) =>
        lines.Count == 0 ? 0 : lines.Count * 397 ^ lines[0].Length;

    private static IReadOnlyList<CelestialBody> CreateDenseCatalog(int sourceCount)
    {
        if (sourceCount < 2)
            throw new ArgumentOutOfRangeException(nameof(sourceCount), sourceCount,
                "a dense catalog requires a root and at least one child");

        const double rootMu = 1.32712440018e20;
        var root = new CelestialBody
        {
            Id = RootId,
            Mu = rootMu,
            MeanRadius = 6.957e8,
        };
        var bodies = new List<CelestialBody>(sourceCount) { root };
        for (int index = 1; index < sourceCount; index++)
        {
            double semiMajorAxis = 4.0e10 + index * 7.5e8;
            double period = 2.0 * Math.PI
                * Math.Sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / rootMu);
            bodies.Add(new CelestialBody
            {
                Id = $"Dense{index:D3}",
                Mu = 5.0e8 + index * 1.0e7,
                MeanRadius = 1.0e3 + index,
                Parent = root,
                Orbit = new OrbitalElements(
                    SemiMajorAxis: semiMajorAxis,
                    Eccentricity: 0.002 * (index % 20),
                    Inclination: 0.003 * (index % 31),
                    LongitudeOfAscendingNode: GoldenAngle(index, 1),
                    ArgumentOfPeriapsis: GoldenAngle(index, 2),
                    TimeAtPeriapsis: -GoldenFraction(index, 3) * period),
            });
        }
        return bodies;
    }

    private static double GoldenAngle(int index, int salt) =>
        2.0 * Math.PI * GoldenFraction(index, salt);

    private static double GoldenFraction(int index, int salt)
    {
        double value = (index + 0.31 * salt) * 0.6180339887498949;
        return value - Math.Floor(value);
    }
}

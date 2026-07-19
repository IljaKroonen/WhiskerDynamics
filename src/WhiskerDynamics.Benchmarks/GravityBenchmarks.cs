using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Benchmarks vessel gravity and pairwise rails acceleration at full catalog
/// size.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class GravityBenchmarks
{
    private const double T = 1.0 * 86400;

    private NBodyEphemerides _ephemerides = null!;
    private GravityModel _gravity = null!;
    private CelestialBody _earth = null!;
    private Vector3d _vesselPosition;
    private Vector3d _earthRelative;
    private StateVector[] _railsSnapshot = null!;
    private PairwiseAccelerationKernel _pairwise = null!;
    private Vector3d[] _accBuffer = null!;
    private Geopotential _lunarGravity = null!;
    private Vector3d _lowLunarOrbit;
    private Vector3d _distantFromLuna;

    [GlobalSetup]
    public void Setup()
    {
        var bodies = BenchmarkCatalog.CreateBodies();
        _ephemerides = new NBodyEphemerides(bodies, 0.0, BenchmarkCatalog.BackboneIds,
            new IntegratorOptions { RelTol = BenchmarkCatalog.ShippingRelTol });
        var backbone = bodies
            .Where(b => BenchmarkCatalog.BackboneIds.Contains(b.Id))
            .ToArray();
        var sources = bodies.Where(b => b.Mu > 0.0).ToArray();
        _gravity = new GravityModel(_ephemerides, sources);
        _earth = _ephemerides["Earth"];

        // Pre-extend the rails horizon past T so measurements interpolate, never integrate.
        _ = _ephemerides.GetState(_earth, 2 * 86400.0);

        _earthRelative = new Vector3d(7.0e6, 0, 0); // LEO-ish offset
        _vesselPosition = _ephemerides.GetState(_earth, T).Position + _earthRelative;
        _railsSnapshot = backbone.Select(b => _ephemerides.GetState(b, T)).ToArray();
        _pairwise = new PairwiseAccelerationKernel(backbone.Select(b => b.Mu).ToArray());
        _accBuffer = new Vector3d[backbone.Length];
        _lunarGravity = LunarGravityModel.Create(new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0),
            2.6616995e-6, 0));
        _lowLunarOrbit = new Vector3d(1_838_000, 0, 0);
        _distantFromLuna = new Vector3d(384_400_000, 0, 0);
    }

    [Benchmark]
    public Vector3d VesselAcceleration_99Sources() => _gravity.AccelerationAt(_vesselPosition, T);

    [Benchmark]
    public Vector3d ThirdBodyDelta_99Sources() => _gravity.ThirdBodyDeltaAt(_earth, _earthRelative, T);

    [Benchmark]
    public Vector3d LunarGravity_50x50_LowOrbit() =>
        _lunarGravity.AccelerationCorrection(_lowLunarOrbit, 4.9028000661637961e12, T);

    [Benchmark]
    public Vector3d LunarGravity_DampedFarField() =>
        _lunarGravity.AccelerationCorrection(_distantFromLuna, 4.9028000661637961e12, T);

    [Benchmark]
    public Vector3d[] RailsMutualAccelerations_50Bodies()
    {
        _pairwise.Compute(_railsSnapshot, _accBuffer);
        return _accBuffer;
    }
}

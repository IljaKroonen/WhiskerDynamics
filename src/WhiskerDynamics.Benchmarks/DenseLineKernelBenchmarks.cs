using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Representative dense map-line preprocessing. The large case matches the
/// overlay's dense vessel budget; the stock case matches the staged point buffer.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class DenseLineKernelBenchmarks
{
    [Params(2_000, 16_384)]
    public int PointCount;

    private Vector3d[] _orbit = null!;
    private Vector3d[] _collinear = null!;

    [GlobalSetup]
    public void Setup()
    {
        _orbit = new Vector3d[PointCount];
        _collinear = new Vector3d[PointCount];
        for (int i = 0; i < PointCount; i++)
        {
            double phase = 12.0 * Math.PI * i / (PointCount - 1);
            _orbit[i] = new Vector3d(
                7.0e6 * Math.Cos(phase),
                5.4e6 * Math.Sin(phase),
                2.0e5 * Math.Sin(phase * 0.37));
            _collinear[i] = new Vector3d(i * 1000.0, -i * 30.0, i * 2.0);
        }
    }

    [Benchmark]
    public double[] ChordSignificance_Orbit() =>
        OverlayKernel.ChordSignificance(_orbit);

    [Benchmark]
    public double[] ChordSignificance_Collinear() =>
        OverlayKernel.ChordSignificance(_collinear);

    [Benchmark]
    public double[] CumulativeArcLengths_Orbit() =>
        OverlayKernel.CumulativeArcLengths(_orbit);
}

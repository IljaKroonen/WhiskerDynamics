using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Worker-side numerical workloads fed by the same sampled arcs used for
/// drawing. Inputs include exact velocities, as the production analysis path does.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class ModAnalysisBenchmarks
{
    private const int Samples = 16_384;
    private const double Mu = 3.986004418e14;
    private const double Radius = 7.0e6;
    private static readonly double Period =
        2.0 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);

    [Params(20, 500)]
    public int Revolutions;

    private double[] _times = null!;
    private Vector3d[] _positions = null!;
    private Vector3d[] _velocities = null!;

    [GlobalSetup]
    public void Setup()
    {
        _times = new double[Samples];
        _positions = new Vector3d[Samples];
        _velocities = new Vector3d[Samples];
        double speed = Math.Sqrt(Mu / Radius);
        for (int i = 0; i < Samples; i++)
        {
            double time = Revolutions * Period * i / (Samples - 1);
            double phase = 2.0 * Math.PI * time / Period;
            _times[i] = time;
            _positions[i] = new Vector3d(
                Radius * Math.Cos(phase), Radius * Math.Sin(phase), 0);
            _velocities[i] = new Vector3d(
                -speed * Math.Sin(phase), speed * Math.Cos(phase), 0);
        }
    }

    [Benchmark]
    public OrbitAnalysisReport? AnalyzeTwentyOrbits() =>
        OrbitAnalysisKernel.Analyze("Earth", _times, _positions, _velocities,
            0.0, _times[^1], Mu, 6_371_000, new Vector3d(0, 0, 1), 7.2921159e-5);

    [Benchmark]
    public AdaptivePath AdaptiveSampleTwentyOrbits() =>
        AdaptiveSampler.Sample(CircularPosition, 0.0, Revolutions * Period,
            Samples, 0.01, 0.25, Period);

    private static Vector3d CircularPosition(double time)
    {
        double phase = 2.0 * Math.PI * time / Period;
        return new Vector3d(Radius * Math.Cos(phase), Radius * Math.Sin(phase), 0);
    }
}

using BenchmarkDotNet.Attributes;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Isolates the DOP853 controller's eighth-root tradeoff. Three square
/// roots are mathematically equivalent to Pow(x, 1/8), but not bitwise equivalent;
/// the accompanying test quantifies the changed rounding.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class IntegratorControllerTradeoffBenchmarks
{
    private const int Batch = 16_384;
    private double[] _errors = null!;

    [GlobalSetup]
    public void Setup()
    {
        _errors = new double[Batch];
        for (int i = 0; i < Batch; i++)
        {
            double exponent = -36.0 + 72.0 * i / (Batch - 1.0);
            _errors[i] = Math.Exp(exponent) * (1.0 + 0.125 * Math.Sin(i * 0.731));
        }
    }

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double PowEighthRoot()
    {
        double sum = 0;
        foreach (double error in _errors) sum += Math.Pow(error, 1.0 / 8.0);
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double ThreeSquareRoots()
    {
        double sum = 0;
        foreach (double error in _errors)
            sum += Math.Sqrt(Math.Sqrt(Math.Sqrt(error)));
        return sum;
    }
}

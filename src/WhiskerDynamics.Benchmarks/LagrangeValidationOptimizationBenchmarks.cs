using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

[MemoryDiagnoser]
[ShortRunJob]
public class LagrangeValidationOptimizationBenchmarks
{
    // Coordinates are normalized and O(1); this permits harmless round-off from
    // arithmetic reassociation and is ten orders smaller than the benchmark grid spacing.
    private const double CoordinateTolerance = 1e-12;
    private static readonly double[] ParityMassRatios =
    [
        LagrangePotential.MassRatio(1.32712440018e20, 3.986004418e14),
        LagrangePotential.MassRatio(3.986004418e14, 4.9028e12),
        0.1,
        0.49,
    ];

    private double _massRatio;
    private double _level;
    private int _columns;
    private int _rows;
    private double _minX, _maxX, _minY, _maxY;

    [GlobalSetup]
    public void Setup()
    {
        _massRatio = ParityMassRatios[1];
        _level = LagrangePotential.CriticalLevels(_massRatio)[2] + 1e-4;
        _columns = 192;
        _rows = 160;
        _minX = -1.5;
        _maxX = 2.0;
        _minY = -1.5;
        _maxY = 1.5;
        VerifyCandidateParity();
    }

    [Benchmark(Baseline = true)]
    public PotentialSegment[] ValidationPerSampleBaseline() => LagrangePotential.Contour(
        _massRatio, _level, _columns, _rows, _minX, _maxX, _minY, _maxY);

    [Benchmark]
    public PotentialSegment[] ValidationHoistedCandidate() =>
        LagrangePotential.ContourWithHoistedValidation(
            _massRatio, _level, _columns, _rows, _minX, _maxX, _minY, _maxY);

    private void VerifyCandidateParity()
    {
        foreach (double massRatio in ParityMassRatios)
        {
            foreach (double criticalLevel in LagrangePotential.CriticalLevels(massRatio))
            {
                VerifyCandidateParity(massRatio, criticalLevel);
                VerifyCandidateParity(massRatio, criticalLevel + 1e-4);
            }
        }
    }

    private void VerifyCandidateParity(double massRatio, double level)
    {
        var production = LagrangePotential.Contour(
            massRatio, level, _columns, _rows, _minX, _maxX, _minY, _maxY);
        var candidate = LagrangePotential.ContourWithHoistedValidation(
            massRatio, level, _columns, _rows, _minX, _maxX, _minY, _maxY);
        AssertParity(production, candidate, massRatio, level);
    }

    internal static void AssertParity(PotentialSegment[] production,
        PotentialSegment[] candidate, double massRatio, double level)
    {
        if (production.Length != candidate.Length)
            throw new InvalidOperationException(
                $"Contour parity failed at ratio {massRatio:R}, level {level:R}: " +
                $"production emitted {production.Length} segments and candidate emitted {candidate.Length}.");

        for (int i = 0; i < production.Length; i++)
        {
            if (CoordinatesMatch(production[i].A, candidate[i].A)
                && CoordinatesMatch(production[i].B, candidate[i].B))
                continue;
            throw new InvalidOperationException(
                $"Contour parity failed at ratio {massRatio:R}, level {level:R}, segment {i}: " +
                $"production {production[i]} differs from candidate {candidate[i]}.");
        }
    }

    private static bool CoordinatesMatch(Vector3d production, Vector3d candidate) =>
        Math.Abs(production.X - candidate.X) <= CoordinateTolerance
        && Math.Abs(production.Y - candidate.Y) <= CoordinateTolerance
        && Math.Abs(production.Z - candidate.Z) <= CoordinateTolerance;
}

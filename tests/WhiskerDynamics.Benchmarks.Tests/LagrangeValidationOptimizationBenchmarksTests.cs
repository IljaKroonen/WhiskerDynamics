using WhiskerDynamics.Benchmarks;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks.Tests;

public sealed class LagrangeValidationOptimizationBenchmarksTests
{
    [Fact]
    public void Global_setup_accepts_candidate_across_parity_matrix()
    {
        var benchmarks = new LagrangeValidationOptimizationBenchmarks();

        benchmarks.Setup();
    }

    [Fact]
    public void Parity_check_rejects_coordinate_drift()
    {
        PotentialSegment[] production =
        [
            new(new Vector3d(0.25, 0.5, 0), new Vector3d(0.75, 1.0, 0)),
        ];
        PotentialSegment[] candidate =
        [
            new(new Vector3d(0.25 + 1e-9, 0.5, 0), new Vector3d(0.75, 1.0, 0)),
        ];

        Assert.Throws<InvalidOperationException>(() =>
            LagrangeValidationOptimizationBenchmarks.AssertParity(
                production, candidate, 0.1, 2.0));
    }
}

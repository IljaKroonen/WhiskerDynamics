using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning;

public sealed class BurnIdentityPolicyTests
{
    [Theory]
    [InlineData(0.0, 0.0, true)]
    [InlineData(0.0, 0.0005, true)]
    [InlineData(0.0, -0.0005, true)]
    [InlineData(0.0, 0.001, true)]
    [InlineData(0.0, -0.001, true)]
    [InlineData(0.0, 0.0010000001, false)]
    [InlineData(0.0, -0.0010000001, false)]
    [InlineData(double.NaN, 0.0, false)]
    [InlineData(double.PositiveInfinity, double.PositiveInfinity, false)]
    [InlineData(double.NegativeInfinity, double.NegativeInfinity, false)]
    [InlineData(double.MaxValue, double.MaxValue, true)]
    public void Timestamp_matrix_defines_logical_burn_identity(
        double first, double second, bool expected)
    {
        Assert.Equal(expected, BurnIdentityPolicy.SameBurn(first, second));
        Assert.Equal(!expected, BurnIdentityPolicy.DifferentBurn(first, second));
    }

    [Fact]
    public void Match_distance_drives_nearest_selection_without_a_second_tolerance()
    {
        double requested = 10.0;
        double[] candidates = [10.0008, 10.0002, 10.002];
        double? nearest = null;
        double bestDistance = BurnIdentityPolicy.ToleranceSeconds;

        foreach (double candidate in candidates)
            if (BurnIdentityPolicy.TryMatch(candidate, requested, out double distance)
                && distance <= bestDistance)
            {
                nearest = candidate;
                bestDistance = distance;
            }

        Assert.Equal(10.0002, nearest);
        Assert.True(BurnIdentityPolicy.ContainsBurn(candidates, requested));
    }
}

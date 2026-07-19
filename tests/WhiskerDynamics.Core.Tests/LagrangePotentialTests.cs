using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class LagrangePotentialTests
{
    private const double EarthMu = 3.986004418e14;
    private const double MoonMu = 4.9028e12;

    [Fact]
    public void Earth_moon_equilibria_are_stationary_and_in_expected_regions()
    {
        double mu = LagrangePotential.MassRatio(EarthMu, MoonMu);
        var p = LagrangePotential.Equilibria(mu);

        Assert.InRange(p.L1.X, 0.8, 1.0);
        Assert.InRange(p.L2.X, 1.0, 1.3);
        Assert.InRange(p.L3.X, -1.1, -0.8);
        Assert.Equal(0.5, p.L4.X, 14);
        Assert.Equal(Math.Sqrt(3) / 2, p.L4.Y, 14);
        Assert.Equal(p.L4.X, p.L5.X);
        Assert.Equal(-p.L4.Y, p.L5.Y);

        for (int i = 0; i < 5; i++)
        {
            var gradient = LagrangePotential.Gradient(mu, p[i].X, p[i].Y);
            Assert.InRange(Math.Abs(gradient.X), 0, 1e-11);
            Assert.InRange(Math.Abs(gradient.Y), 0, 1e-11);
        }
    }

    [Fact]
    public void Critical_levels_generate_finite_contours_near_their_equilibria()
    {
        double mu = LagrangePotential.MassRatio(EarthMu, MoonMu);
        var levels = LagrangePotential.CriticalLevels(mu);

        Assert.InRange(levels.Length, 3, 4);
        // Offset critical levels so finite-grid marching squares captures contours
        // around degenerate triangular minima.
        foreach (double criticalLevel in levels)
        {
            double level = criticalLevel + 1e-4;
            var segments = LagrangePotential.Contour(mu, level, columns: 128, rows: 112);
            Assert.NotEmpty(segments);
            Assert.InRange(segments.Length, 1, 10_000);
            Assert.All(segments, segment =>
            {
                Assert.True(double.IsFinite(segment.A.X) && double.IsFinite(segment.A.Y));
                Assert.True(double.IsFinite(segment.B.X) && double.IsFinite(segment.B.Y));
                Assert.Equal(0, segment.A.Z);
                Assert.Equal(0, segment.B.Z);
            });
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(double.NaN)]
    public void Invalid_mass_ratio_is_rejected(double ratio) =>
        Assert.Throws<ArgumentOutOfRangeException>(() => LagrangePotential.Equilibria(ratio));
}

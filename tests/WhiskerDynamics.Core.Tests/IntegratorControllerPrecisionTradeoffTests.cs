using Xunit.Abstractions;

namespace WhiskerDynamics.Core.Tests;

public class IntegratorControllerPrecisionTradeoffTests(ITestOutputHelper output)
{
    [Fact]
    public void Three_square_roots_change_controller_rounding_by_at_most_one_ulp()
    {
        const int cases = 16_384;
        int changed = 0;
        ulong maxUlp = 0;
        double maxRelative = 0;

        for (int i = 0; i < cases; i++)
        {
            double exponent = -36.0 + 72.0 * i / (cases - 1.0);
            double error = Math.Exp(exponent) * (1.0 + 0.125 * Math.Sin(i * 0.731));
            double production = Math.Pow(error, 1.0 / 8.0);
            double candidate = Math.Sqrt(Math.Sqrt(Math.Sqrt(error)));
            ulong ulp = UlpDistance(production, candidate);
            if (ulp != 0) changed++;
            maxUlp = Math.Max(maxUlp, ulp);
            maxRelative = Math.Max(maxRelative,
                Math.Abs(candidate - production) / Math.Abs(production));
        }

        output.WriteLine($"changed={changed}/{cases}");
        output.WriteLine($"maxUlp={maxUlp}");
        output.WriteLine($"maxRelative={maxRelative:G17}");

        Assert.True(changed > 0);
        Assert.InRange(maxUlp, 1UL, 1UL);
    }

    private static ulong UlpDistance(double left, double right)
    {
        ulong a = (ulong)BitConverter.DoubleToInt64Bits(left);
        ulong b = (ulong)BitConverter.DoubleToInt64Bits(right);
        return a >= b ? a - b : b - a;
    }
}

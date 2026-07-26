using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

public class GeopotentialOptimizationPrecisionTests
{
    [Fact]
    public void Loop_invariant_scale_and_reverse_limit_scan_match_baseline_bits()
    {
        const double mu = 4.9028000661637961e12;
        var rotation = new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 2.6616995e-6, 0);
        var field = TestGravityModels.Lunar(rotation);
        var cases = new[]
        {
            (new Vector3d(1_838_000, 0, 0), 86_400.0,
                -4666382658273572630L, 4549859530864713808L, 4556966290180960073L),
            (new Vector3d(1_300_000, -700_000, 1_100_000), 12_345.0,
                4548324425990035178L, 4563549972401991437L, -4661426165625769598L),
            (new Vector3d(250_000, 400_000, 1_775_000), 864_000.0,
                -4670496748720930700L, 4555750932765909116L, 4558225722542827960L),
        };

        foreach (var (position, time, xBits, yBits, zBits) in cases)
        {
            var actual = field.AccelerationCorrection(position, mu, time);
            Assert.Equal(xBits, BitConverter.DoubleToInt64Bits(actual.X));
            Assert.Equal(yBits, BitConverter.DoubleToInt64Bits(actual.Y));
            Assert.Equal(zBits, BitConverter.DoubleToInt64Bits(actual.Z));
        }
    }
}

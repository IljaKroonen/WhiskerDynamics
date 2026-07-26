using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

public class LunarGravityTests
{
    private static readonly BodyRotation Rotation = new(
        new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0),
        2.6616995e-6, 0);

    [Fact]
    public void Built_in_lunar_field_is_PDS_GRGM1200A_50x50()
    {
        var field = TestGravityModels.Lunar(Rotation);

        Assert.Equal(50, field.Degree);
        Assert.Equal(1323, field.Coefficients.Count);
        Assert.Equal(1_738_000, field.ReferenceRadius);
        var c20 = field.Coefficients.Single(c => c is { Degree: 2, Order: 0 });
        var c22 = field.Coefficients.Single(c => c is { Degree: 2, Order: 2 });
        var c21 = field.Coefficients.Single(c => c is { Degree: 2, Order: 1 });
        Assert.Equal(-9.0884339347424299e-5 * Math.Sqrt(5), c20.Cosine, 15);
        Assert.Equal(3.4673096470696298e-5 * Math.Sqrt(5.0 / 12), c22.Cosine, 15);
        Assert.Equal(9.07918983437229e-10 * Math.Sqrt(5.0 / 12), c22.Sine, 15);
        Assert.Equal(1.4664123550281899e-11 * Math.Sqrt(5.0 / 3), c21.Cosine, 15);
        Assert.Equal(1.1732764234889199e-9 * Math.Sqrt(5.0 / 3), c21.Sine, 15);
    }

    [Fact]
    public void Tesseral_field_rotates_with_the_body_fixed_prime_meridian()
    {
        const double mu = 4.9028000661637961e12;
        const double radius = 1_738_000;
        const double c22 = 2e-5;
        var field = new Geopotential(radius, Rotation, [new(2, 2, c22, 0)]);
        var point = new Vector3d(1_900_000, 0, 0);
        double quarterTurn = Math.PI / (2 * Rotation.AngularVelocity);

        var atZero = field.AccelerationCorrection(point, mu, 0);
        var afterQuarterTurn = field.AccelerationCorrection(point, mu, quarterTurn);

        Assert.Equal(-atZero.X, afterQuarterTurn.X, Math.Abs(atZero.X) * 1e-12);
        Assert.Equal(0, atZero.Y, 14);
        Assert.Equal(0, afterQuarterTurn.Y, 14);
        Assert.True(atZero.X < 0);
    }

    [Fact]
    public void Full_lunar_field_is_finite_at_and_near_the_poles()
    {
        const double mu = 4.9028000661637961e12;
        var field = TestGravityModels.Lunar(Rotation);
        foreach (var p in new[]
        {
            new Vector3d(0, 0, 1_838_000),
            new Vector3d(1e-6, -2e-6, 1_838_000),
            new Vector3d(1_300_000, -700_000, 1_100_000),
        })
        {
            var a = field.AccelerationCorrection(p, mu, 12345);
            Assert.True(double.IsFinite(a.X) && double.IsFinite(a.Y) && double.IsFinite(a.Z));
        }
    }

    [Fact]
    public void Full_lunar_field_matches_regression_vectors()
    {
        const double mu = 4.9028000661637961e12;
        var field = TestGravityModels.Lunar(Rotation);
        var cases = new[]
        {
            (new Vector3d(1_838_000, 0, 0), 86_400.0,
                new Vector3d(-0.0004519824716866268, 0.0001552257297299237,
                    0.0004515560514025811)),
            (new Vector3d(1_300_000, -700_000, 1_100_000), 12_345.0,
                new Vector3d(0.00011228314843202769, 0.0012826578940147457,
                    -0.0009560821002349254)),
            (new Vector3d(250_000, 400_000, 1_775_000), 864_000.0,
                new Vector3d(-0.00023775579809807426, 0.0003854987130596389,
                    0.0005500731314008429)),
        };

        foreach (var (position, time, expected) in cases)
        {
            var actual = field.AccelerationCorrection(position, mu, time);
            Assert.True((actual - expected).Length() < 1e-15,
                $"expected {expected}; actual {actual}");
        }
    }

    [Fact]
    public void Exact_pole_matches_limits_approached_from_every_longitude()
    {
        const double mu = 4.9028000661637961e12;
        const double radius = 1_838_000;
        var field = TestGravityModels.Lunar(Rotation);
        var exact = field.AccelerationCorrection(new Vector3d(0, 0, radius), mu, 12345);
        foreach (double longitude in new[] { 0.0, 0.7, 2.1, 4.8 })
        {
            const double offset = 0.01;
            var near = new Vector3d(offset * Math.Cos(longitude),
                offset * Math.Sin(longitude), radius);
            var limit = field.AccelerationCorrection(near, mu, 12345);
            Assert.True((exact - limit).Length() < 2e-10,
                $"longitude {longitude}: exact {exact}; limit {limit}");
        }
    }

    [Fact]
    public void Fully_normalised_factory_preserves_J2_convention()
    {
        const double j2 = 2.0326e-4;
        var normalized = Geopotential.FromFullyNormalized(1_738_000, Rotation,
            [new(2, 0, -j2 / Math.Sqrt(5), 0)]);
        var ordinary = Geopotential.FromJ2(1_738_000, Rotation, j2);
        var point = new Vector3d(1_900_000, 70_000, 200_000);

        var a = normalized.AccelerationCorrection(point, 4.9028e12, 0);
        var b = ordinary.AccelerationCorrection(point, 4.9028e12, 0);
        Assert.True((a - b).Length() < 1e-15);
    }

    [Fact]
    public void Analytic_harmonic_acceleration_matches_potential_gradient()
    {
        const double mu = 4.9028e12;
        var field = new Geopotential(1_738_000, Rotation with { AngularVelocity = 0 },
        [
            new(2, 0, -2e-4, 0), new(2, 2, 3e-5, -7e-6),
            new(3, 1, 2e-6, 4e-6), new(3, 3, -3e-6, 1e-6),
        ]);
        var p = new Vector3d(1_700_000, -600_000, 500_000);
        const double h = 0.25;
        double Dx(Vector3d axis) =>
            (Potential(field, p + axis * h, mu) - Potential(field, p - axis * h, mu)) / (2 * h);
        var numerical = new Vector3d(
            Dx(new Vector3d(1, 0, 0)), Dx(new Vector3d(0, 1, 0)), Dx(new Vector3d(0, 0, 1)));
        var analytic = field.AccelerationCorrection(p, mu, 0);

        Assert.True((analytic - numerical).Length() < 2e-8,
            $"analytic {analytic}; numerical {numerical}");
    }

    [Fact]
    public void Far_field_damping_is_continuous_and_reaches_exact_zero()
    {
        const double j2 = 1e-3;
        const double tolerance = 1.0 / (1 << 24);
        var field = Geopotential.FromJ2(1, Rotation with { AngularVelocity = 0 }, j2);
        double s0 = Math.Sqrt(3 * j2 / tolerance);
        Vector3d At(double r) => field.AccelerationCorrection(new Vector3d(r, 0, 0), 1, 0);

        var innerLeft = At(s0 * (1 - 1e-7));
        var innerRight = At(s0 * (1 + 1e-7));
        Assert.True((innerLeft - innerRight).Length() < innerLeft.Length() * 2e-6);
        Assert.Equal(Vector3d.Zero, At(3 * s0));
        Assert.Equal(Vector3d.Zero, At(4 * s0));
    }

    private static double Potential(Geopotential field, Vector3d p, double mu)
    {
        double r = p.Length(), s = p.Z / r, cb = Math.Sqrt(p.X * p.X + p.Y * p.Y) / r;
        double lambda = Math.Atan2(p.Y, p.X);
        int width = field.Degree + 1;
        var d = new double[width * width];
        d[0] = 1; d[width] = s; d[width + 1] = 1;
        for (int n = 2; n <= field.Degree; n++)
            for (int m = 0; m <= n; m++)
                d[n * width + m] = ((2 * n - 1) * (s * d[(n - 1) * width + m]
                    + (m == 0 ? 0 : m * d[(n - 1) * width + m - 1]))
                    - (n - 1) * d[(n - 2) * width + m]) / n;
        double result = 0;
        foreach (var c in field.Coefficients)
            result += Math.Pow(field.ReferenceRadius / r, c.Degree)
                * Math.Pow(cb, c.Order) * d[c.Degree * width + c.Order]
                * (c.Cosine * Math.Cos(c.Order * lambda) + c.Sine * Math.Sin(c.Order * lambda));
        return mu / r * result;
    }
}

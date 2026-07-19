using Xunit.Abstractions;

namespace WhiskerDynamics.Core.Tests;

public class LunarGravityFidelityTests(ITestOutputHelper output)
{
    private const double Mu = 4.9028000661637961e12;
    private static readonly BodyRotation Rotation = new(
        new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0),
        2.6616995e-6, 0);

    [Theory]
    [InlineData(LunarGravityFidelity.Degree10, 10, 63)]
    [InlineData(LunarGravityFidelity.Degree20, 20, 228)]
    [InlineData(LunarGravityFidelity.Degree30, 30, 493)]
    [InlineData(LunarGravityFidelity.Degree40, 40, 858)]
    [InlineData(LunarGravityFidelity.Degree50, 50, 1323)]
    public void Catalog_build_applies_selected_lunar_fidelity(
        LunarGravityFidelity fidelity, int expectedDegree, int expectedCoefficients)
    {
        const double gravitationalConstant = 6.67430e-11;
        const double solarMass = 1.98847e30;
        const double orbitalRadius = 1.0e9;
        var sol = new CatalogBody("Sol", solarMass, null, 696_340_000,
            null, null);
        var luna = new CatalogBody("Luna", 7.342e22, "Sol", 1_737_400,
            new Vector3d(orbitalRadius, 0, 0),
            new Vector3d(0, Math.Sqrt(gravitationalConstant * solarMass / orbitalRadius), 0),
            Rotation);

        IReadOnlyList<CelestialBody> bodies = CatalogKernel.Build(
            [sol, luna], gravitationalConstant, out var diagnostics, fidelity);
        CelestialBody body = Assert.Single(bodies, candidate => candidate.Id == "Luna");

        Assert.Empty(diagnostics);
        Assert.Equal(expectedDegree, body.Geopotential!.Degree);
        Assert.Equal(expectedCoefficients, body.Geopotential.Coefficients.Count);

        var relative = new Vector3d(1_838_000, 0, 0);
        double r2 = relative.LengthSquared();
        Vector3d expected = relative * (-body.Mu / (r2 * Math.Sqrt(r2)))
            + body.Geopotential.AccelerationCorrection(relative, body.Mu, 0);
        var ephemerides = new Ephemerides(bodies);
        Vector3d vesselPosition = ephemerides.GetState(body, 0).Position + relative;
        var gravity = new GravityModel(ephemerides, [body]);
        Assert.True((gravity.AccelerationAt(vesselPosition, 0) - expected).Length() < 1e-14);
    }

    [Fact]
    public void Reduced_models_have_expected_size_and_representative_two_day_orbit_drift()
    {
        var degree50 = LunarGravityModel.Create(Rotation, LunarGravityFidelity.Degree50);
        var degree40 = LunarGravityModel.Create(Rotation, LunarGravityFidelity.Degree40);
        var degree30 = LunarGravityModel.Create(Rotation, LunarGravityFidelity.Degree30);
        var degree20 = LunarGravityModel.Create(Rotation, LunarGravityFidelity.Degree20);
        var degree10 = LunarGravityModel.Create(Rotation, LunarGravityFidelity.Degree10);

        Assert.Equal((50, 1323), (degree50.Degree, degree50.Coefficients.Count));
        Assert.Equal((40, 858), (degree40.Degree, degree40.Coefficients.Count));
        Assert.Equal((30, 493), (degree30.Degree, degree30.Coefficients.Count));
        Assert.Equal((20, 228), (degree20.Degree, degree20.Coefficients.Count));
        Assert.Equal((10, 63), (degree10.Degree, degree10.Coefficients.Count));

        var fields = new[]
        {
            (Degree: 40, Field: degree40),
            (Degree: 30, Field: degree30),
            (Degree: 20, Field: degree20),
            (Degree: 10, Field: degree10),
        };
        var cases = new[]
        {
            (Name: "equatorial-100km-lon0", Initial: CircularState(100_000, 0, 0)),
            (Name: "equatorial-25km-lon90", Initial: CircularState(25_000, Math.PI / 2, 0)),
            (Name: "polar-50km-lon45", Initial: CircularState(50_000, Math.PI / 4, Math.PI / 2)),
            (Name: "inclined-150km-lon135", Initial: CircularState(150_000, 3 * Math.PI / 4, Math.PI / 4)),
        };
        var positionDrifts = fields.ToDictionary(item => item.Degree, _ => new List<double>());
        var velocityDrifts = fields.ToDictionary(item => item.Degree, _ => new List<double>());

        foreach (var orbit in cases)
        {
            StateVector reference = Propagate(degree50, orbit.Initial);
            foreach (var candidate in fields)
            {
                StateVector state = Propagate(candidate.Field, orbit.Initial);
                double positionDrift = (state.Position - reference.Position).Length();
                double velocityDrift = (state.Velocity - reference.Velocity).Length();
                positionDrifts[candidate.Degree].Add(positionDrift);
                velocityDrifts[candidate.Degree].Add(velocityDrift);
                output.WriteLine($"{orbit.Name} {candidate.Degree}x{candidate.Degree} vs 50x50 after 2 d: "
                    + $"{positionDrift:R} m, {velocityDrift:R} m/s");
                Assert.True(double.IsFinite(positionDrift + velocityDrift));
            }

        }

        foreach (var candidate in fields)
        {
            double positionRms = Math.Sqrt(positionDrifts[candidate.Degree].Average(value => value * value));
            double velocityRms = Math.Sqrt(velocityDrifts[candidate.Degree].Average(value => value * value));
            output.WriteLine($"SUMMARY {candidate.Degree}x{candidate.Degree}: "
                + $"position RMS {positionRms:R} m, max {positionDrifts[candidate.Degree].Max():R} m; "
                + $"velocity RMS {velocityRms:R} m/s, max {velocityDrifts[candidate.Degree].Max():R} m/s");
            AssertAggregateDrift(candidate.Degree, positionRms,
                positionDrifts[candidate.Degree].Max(), velocityRms,
                velocityDrifts[candidate.Degree].Max());
        }

        Assert.InRange(positionDrifts[40][0], 700, 1_100);
        Assert.InRange(positionDrifts[30][0], 1_400, 2_000);
        Assert.InRange(positionDrifts[20][0], 2_500, 3_500);
        Assert.InRange(positionDrifts[10][0], 7_500, 9_500);
        Assert.InRange(velocityDrifts[40][0], 0.35, 0.7);
        Assert.InRange(velocityDrifts[30][0], 0.6, 1.1);
        Assert.InRange(velocityDrifts[20][0], 1.0, 2.5);
        Assert.InRange(velocityDrifts[10][0], 5.0, 7.0);
    }

    private static StateVector CircularState(double altitude, double longitude, double inclination)
    {
        double radius = 1_738_000 + altitude;
        var radial = new Vector3d(Math.Cos(longitude), Math.Sin(longitude), 0);
        var tangent = new Vector3d(-Math.Sin(longitude), Math.Cos(longitude), 0);
        Vector3d velocityDirection = tangent * Math.Cos(inclination)
            + new Vector3d(0, 0, Math.Sin(inclination));
        return new StateVector(radial * radius, velocityDirection * Math.Sqrt(Mu / radius));
    }

    private static void AssertAggregateDrift(
        int degree, double positionRms, double positionMax, double velocityRms, double velocityMax)
    {
        (double PositionRmsMin, double PositionRmsMax,
            double PositionMaxMin, double PositionMaxMax,
            double VelocityRmsMin, double VelocityRmsMax,
            double VelocityMaxMin, double VelocityMaxMax) limits = degree switch
        {
            40 => (3_500, 5_500, 7_000, 10_500, 1.6, 2.8, 3.2, 5.4),
            30 => (4_500, 7_000, 8_500, 13_000, 2.8, 4.6, 5.2, 8.2),
            20 => (4_000, 6_500, 6_500, 10_000, 5.0, 8.5, 9.5, 15.0),
            10 => (7_500, 12_500, 13_000, 21_000, 8.5, 14.0, 12.0, 19.0),
            _ => throw new ArgumentOutOfRangeException(nameof(degree)),
        };

        Assert.InRange(positionRms, limits.PositionRmsMin, limits.PositionRmsMax);
        Assert.InRange(positionMax, limits.PositionMaxMin, limits.PositionMaxMax);
        Assert.InRange(velocityRms, limits.VelocityRmsMin, limits.VelocityRmsMax);
        Assert.InRange(velocityMax, limits.VelocityMaxMin, limits.VelocityMaxMax);
    }

    private static StateVector Propagate(Geopotential field, StateVector initial) =>
        DormandPrince54.Propagate((time, state) =>
        {
            Vector3d position = state.Position;
            double r2 = position.LengthSquared();
            return position * (-Mu / (r2 * Math.Sqrt(r2)))
                + field.AccelerationCorrection(position, Mu, time);
        }, initial, 0, 2 * 86_400.0,
            new IntegratorOptions { RelTol = 1e-12, MaxStep = 300 });
}

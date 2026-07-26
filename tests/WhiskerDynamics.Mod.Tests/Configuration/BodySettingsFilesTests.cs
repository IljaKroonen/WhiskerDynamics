using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Configuration;

public class BodySettingsFilesTests
{
    [Fact]
    public void Shipping_catalog_contains_Earth_and_Luna_coefficients()
    {
        string directory = Path.Combine(
            FindRepoRoot(), "src", "WhiskerDynamics.Mod", "BodySettings");

        BodySettingsCatalog catalog = BodySettingsCatalog.LoadDirectory(directory);

        Assert.Equal(2, catalog.Entries.Count);
        var earth = new CatalogBody("Earth", 1, "Sol", 6_371_000, null, null);
        var earthSettings = Assert.IsType<SphericalHarmonicGravitySettings>(
            catalog.Match(earth)!.GravityModel);
        Assert.Equal("Earth J2", earthSettings.Name);
        Assert.Equal(SphericalHarmonicNormalization.Unnormalized,
            earthSettings.Normalization);
        Assert.Equal(2, earthSettings.MaximumDegree);
        Assert.Equal(6_378_137, earthSettings.ReferenceRadiusM);
        var earthC20 = Assert.Single(earthSettings.Coefficients);
        Assert.Equal((2, 0), (earthC20.Degree, earthC20.Order));
        Assert.Equal(-1.08262668e-3, earthC20.Cosine);
        Assert.Equal(0, earthC20.Sine);

        var luna = new CatalogBody("Luna", 1, "Earth", 1_737_400, null, null);
        var lunarSettings = Assert.IsType<SphericalHarmonicGravitySettings>(
            catalog.Match(luna)!.GravityModel);
        Assert.Equal("GRGM1200A", lunarSettings.Name);
        Assert.Equal(SphericalHarmonicNormalization.FullyNormalized,
            lunarSettings.Normalization);
        Assert.Equal(30, lunarSettings.MaximumDegree);
        Assert.Equal(1_738_000, lunarSettings.ReferenceRadiusM);
        Assert.Equal(1323, lunarSettings.Coefficients.Count);
        Assert.Equal((50, 50),
            (lunarSettings.Coefficients[^1].Degree, lunarSettings.Coefficients[^1].Order));
    }

    private static string FindRepoRoot()
    {
        for (var directory = new DirectoryInfo(AppContext.BaseDirectory);
             directory is not null;
             directory = directory.Parent)
            if (File.Exists(Path.Combine(directory.FullName, "WhiskerDynamics.slnx")))
                return directory.FullName;
        throw new DirectoryNotFoundException("Could not find repository root.");
    }
}
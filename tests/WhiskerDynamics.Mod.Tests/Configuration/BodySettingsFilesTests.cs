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
        Assert.Equal("EGM2008", earthSettings.Name);
        Assert.Equal(SphericalHarmonicNormalization.FullyNormalized,
            earthSettings.Normalization);
        Assert.Equal(10, earthSettings.MaximumDegree);
        Assert.Equal(6_378_136.3, earthSettings.ReferenceRadiusM);
        Assert.Equal(1323, earthSettings.Coefficients.Count);
        Assert.Null(earthSettings.BodyFixedToModel);
        var earthC20 = earthSettings.Coefficients[0];
        Assert.Equal((2, 0), (earthC20.Degree, earthC20.Order));
        Assert.Equal(-4.8416514379081503e-4, earthC20.Cosine);
        Assert.Equal(0, earthC20.Sine);
        Assert.Equal(1.0826261738522227e-3,
            -earthC20.Cosine * Math.Sqrt(5), 15);
        Assert.Equal((50, 50),
            (earthSettings.Coefficients[^1].Degree,
                earthSettings.Coefficients[^1].Order));

        var luna = new CatalogBody("Luna", 1, "Earth", 1_737_400, null, null);
        var lunarSettings = Assert.IsType<SphericalHarmonicGravitySettings>(
            catalog.Match(luna)!.GravityModel);
        Assert.Equal("GRGM1200A", lunarSettings.Name);
        Assert.Equal(SphericalHarmonicNormalization.FullyNormalized,
            lunarSettings.Normalization);
        Assert.Equal(30, lunarSettings.MaximumDegree);
        Assert.Equal(1_738_000, lunarSettings.ReferenceRadiusM);
        Assert.Equal(1323, lunarSettings.Coefficients.Count);
        Assert.NotNull(lunarSettings.BodyFixedToModel);
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

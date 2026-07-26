using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

internal static class BenchmarkGravityModels
{
    private static readonly Lazy<SphericalHarmonicGravitySettings> LunarCatalog =
        new(LoadLunarCatalog);

    internal static Geopotential Lunar(BodyRotation rotation, int maximumDegree = 50)
    {
        SphericalHarmonicGravitySettings source = LunarCatalog.Value;
        var settings = new SphericalHarmonicGravitySettings(
            source.Name,
            source.ReferenceRadiusM,
            maximumDegree,
            source.Normalization,
            source.Coefficients,
            source.BodyFixedToModel);
        return settings.Create(new CatalogBody(
            "Luna", 1, "Earth", 1_737_400, null, null, rotation));
    }

    private static SphericalHarmonicGravitySettings LoadLunarCatalog()
    {
        string directory = Path.Combine(
            FindRepoRoot(), "src", "WhiskerDynamics.Mod", "BodySettings");
        BodySettingsCatalog catalog = BodySettingsCatalog.LoadDirectory(directory);
        var luna = new CatalogBody("Luna", 1, "Earth", 1_737_400, null, null);
        return (SphericalHarmonicGravitySettings)catalog.Match(luna)!.GravityModel;
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

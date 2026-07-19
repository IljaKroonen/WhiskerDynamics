using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests;

internal static class TestRailsService
{
    internal static RailsService FromFixture(
        ModConfig config, GameConstants constants,
        Action<CancellationToken>? celestialSampling = null)
    {
        string path = Path.Combine(
            AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml");
        var bodies = AstronomicalsParser.ParseFile(path, constants.ToMassConstants());
        return RailsService.CreateForModeledCatalog(config, bodies, celestialSampling);
    }
}

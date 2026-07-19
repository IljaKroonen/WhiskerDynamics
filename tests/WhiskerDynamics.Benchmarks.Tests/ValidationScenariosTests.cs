using System.Globalization;
using WhiskerDynamics.Benchmarks;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks.Tests;

public sealed class ValidationScenariosTests : IDisposable
{
    private static readonly (string BodyId, string HorizonsId, double AxisKm)[] RequiredBodies =
    [
        ("Mercury", "199", 57_900_000),
        ("Venus", "299", 108_200_000),
        ("Earth", "399", 149_600_000),
        ("Mars", "499", 227_900_000),
    ];

    private readonly string directory = Path.Combine(Path.GetTempPath(),
        "whisker-dynamics-epoch-tests-" + Guid.NewGuid().ToString("N"));
    private readonly string horizonsDirectory;

    public ValidationScenariosTests()
    {
        Directory.CreateDirectory(directory);
        horizonsDirectory = Path.Combine(directory, "horizons");
        Directory.CreateDirectory(horizonsDirectory);
        string completeManifest = WriteManifest(RequiredBodies.Select(body => body.BodyId));
        WriteHorizonsFiles(completeManifest);
    }

    [Fact]
    public void Empty_manifest_fails_epoch_check()
    {
        Assert.Equal(1, RunEpochCheck([]));
    }

    [Fact]
    public void Partial_manifest_fails_epoch_check()
    {
        Assert.Equal(1, RunEpochCheck(["Earth", "Mercury"]));
    }

    [Fact]
    public void Complete_manifest_passes_epoch_check()
    {
        Assert.Equal(0, RunEpochCheck(RequiredBodies.Select(body => body.BodyId)));
    }

    public void Dispose() => Directory.Delete(directory, recursive: true);

    private int RunEpochCheck(IEnumerable<string> bodyIds)
    {
        string manifest = WriteManifest(bodyIds);
        Assert.True(ValidationScenarios.TryRun(
            ["epoch-check", "--astronomicals", manifest, "--horizons", horizonsDirectory],
            out int exitCode));
        return exitCode;
    }

    private string WriteManifest(IEnumerable<string> bodyIds)
    {
        var requested = bodyIds.ToHashSet(StringComparer.Ordinal);
        var entries = new List<string>
        {
            """  <StellarBody Id="Sol"><Mass Suns="1" /></StellarBody>""",
        };
        foreach (var (bodyId, _, axisKm) in RequiredBodies.Where(body =>
            requested.Contains(body.BodyId)))
        {
            entries.Add($$"""
              <PlanetaryBody Id="{{bodyId}}" Parent="Sol">
                <Orbit DefinitionFrame="Ecliptic">
                  <SemiMajorAxis Km="{{axisKm.ToString("R", CultureInfo.InvariantCulture)}}" />
                  <Inclination Degrees="0" />
                  <Eccentricity Value="0" />
                  <LongitudeOfAscendingNode Degrees="0" />
                  <ArgumentOfPeriapsis Degrees="0" />
                  <TimeAtPeriapsis Seconds="0" />
                </Orbit>
                <Mass Kg="0" />
              </PlanetaryBody>
            """);
        }

        string path = Path.Combine(directory, $"Astronomicals-{Guid.NewGuid():N}.xml");
        File.WriteAllText(path, $"<Assets>{Environment.NewLine}"
            + string.Join(Environment.NewLine, entries)
            + $"{Environment.NewLine}</Assets>");
        return path;
    }

    private void WriteHorizonsFiles(string completeManifest)
    {
        var ephemerides = new Ephemerides(AstronomicalsParser.ParseFile(completeManifest));
        foreach (var (bodyId, horizonsId, _) in RequiredBodies)
        {
            var state = ephemerides.GetState(ephemerides[bodyId], 0);
            string record = string.Join(",", "0", "0",
                Km(state.Position.X), Km(state.Position.Y), Km(state.Position.Z),
                Km(state.Velocity.X), Km(state.Velocity.Y), Km(state.Velocity.Z));
            File.WriteAllLines(Path.Combine(horizonsDirectory, horizonsId + ".txt"),
                ["$$SOE", record]);
        }
    }

    private static string Km(double metres) =>
        (metres / 1000).ToString("R", CultureInfo.InvariantCulture);
}

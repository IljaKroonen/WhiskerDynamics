using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

public class BodySettingsCatalogTests
{
    [Fact]
    public void Directory_catalog_loads_coefficients_in_ordinal_file_order_and_matches_hierarchy()
    {
        string directory = NewDirectory();
        try
        {
            File.WriteAllText(Path.Combine(directory, "z-luna.json"),
                """
                {
                  "schema_version": 1,
                  "match": { "id": "Luna", "parent_id": "Earth" },
                  "gravity_model": {
                    "model": "spherical_harmonics",
                    "name": "test lunar field",
                    "normalization": "fully_normalized",
                    "reference_radius_m": 1738000,
                    "maximum_degree": 2,
                    "coefficients": [
                      [2, 0, -9.08843393474243e-5, 0],
                      [2, 1, 1.46641235502819e-11, 1.17327642348892e-9],
                      [2, 2, 3.46730964706963e-5, 9.07918983437229e-10]
                    ]
                  }
                }
                """);
            File.WriteAllText(Path.Combine(directory, "a-earth.json"),
                """
                {
                  "schema_version": 1,
                  "match": { "id": "Earth", "parent_id": "Sol" },
                  "gravity_model": {
                    "model": "spherical_harmonics",
                    "name": "test Earth J2",
                    "normalization": "unnormalized",
                    "reference_radius_m": 6378137,
                    "maximum_degree": 2,
                    "coefficients": [[2, 0, -0.00108262668, 0]]
                  }
                }
                """);

            BodySettingsCatalog catalog = BodySettingsCatalog.LoadDirectory(directory);

            Assert.Equal(["a-earth.json", "z-luna.json"],
                catalog.Entries.Select(entry => entry.Source).ToArray());
            var earth = new CatalogBody("Earth", 1, "Sol", 6_371_000, null, null);
            var wrongEarth = earth with { ParentId = "OtherStar" };
            var luna = new CatalogBody("Luna", 1, "Earth", 1_737_400, null, null);
            var earthModel = Assert.IsType<SphericalHarmonicGravitySettings>(
                catalog.Match(earth)!.GravityModel);
            Assert.Equal(SphericalHarmonicNormalization.Unnormalized,
                earthModel.Normalization);
            Assert.Equal(-0.00108262668, Assert.Single(earthModel.Coefficients).Cosine);
            Assert.Null(catalog.Match(wrongEarth));
            var lunarModel = Assert.IsType<SphericalHarmonicGravitySettings>(
                catalog.Match(luna)!.GravityModel);
            Assert.Equal(SphericalHarmonicNormalization.FullyNormalized,
                lunarModel.Normalization);
            Assert.Equal(2, lunarModel.MaximumDegree);
            Assert.Equal(3, lunarModel.Coefficients.Count);
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public void Invalid_or_unknown_model_fails_with_source_file_context()
    {
        string directory = NewDirectory();
        try
        {
            string path = Path.Combine(directory, "bad.json");
            File.WriteAllText(path,
                """
                {
                  "schema_version": 1,
                  "match": { "id": "Earth" },
                  "gravity_model": { "model": "ggm05c", "maximum_degree": 30 }
                }
                """);

            var exception = Assert.Throws<FormatException>(
                () => BodySettingsCatalog.LoadDirectory(directory));

            Assert.Contains("bad.json", exception.Message);
            Assert.Contains("not supported", exception.Message);
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public void Overlapping_generic_and_parent_specific_matches_are_rejected()
    {
        var exception = Assert.Throws<FormatException>(() => new BodySettingsCatalog(
        [
            new BodySettings(new BodyMatch("Moon"), J2(0.1), "all.json"),
            new BodySettings(new BodyMatch("Moon", "Earth"), J2(0.2), "earth.json"),
        ]));

        Assert.Contains("overlapping", exception.Message);
    }

    [Fact]
    public void Matched_body_fixed_model_requires_game_catalog_rotation()
    {
        var settings = new BodySettingsCatalog(
            [new BodySettings(new BodyMatch("Earth"), J2(0.1))]);
        var earth = new CatalogBody("Earth", 5.9722e24, null, 6_371_000, null, null);

        var exception = Assert.Throws<FormatException>(
            () => CatalogKernel.Build([earth], 6.6743e-11, out _, settings));

        Assert.Contains("supplied no rotation", exception.Message);
    }

    private static SphericalHarmonicGravitySettings J2(double value) => new(
        "test J2", null, 2, SphericalHarmonicNormalization.Unnormalized,
        [new SphericalHarmonicCoefficient(2, 0, -value, 0)]);

    private static string NewDirectory()
    {
        string path = Path.Combine(Path.GetTempPath(),
            "whisker-dynamics-body-settings-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(path);
        return path;
    }
}
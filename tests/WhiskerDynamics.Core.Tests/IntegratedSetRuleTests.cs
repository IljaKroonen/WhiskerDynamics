using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class IntegratedSetRuleTests
{
    private const double MuSun = 1.32712440018e20;
    private static CelestialBody Root() => new() { Id = "Sun", Mu = MuSun };

    private static CelestialBody Body(string id, CelestialBody parent, double a, double e,
        double mu = 1e12) => new()
    {
        Id = id, Mu = mu, Parent = parent,
        Orbit = new OrbitalElements(a, e, 0, 0, 0, 0),
    };

    [Fact]
    public void Selects_every_finite_positive_mass_body_for_the_mutual_backbone()
    {
        var sun = Root();
        var planet = Body("Planet", sun, 1.5e11, 0.02, 3.986e14);
        var moon = Body("Moon", planet, 3.844e8, 0.05, 4.9e12);

        var ids = IntegratedSetRule.Select([sun, planet, moon], 0,
            out var restricted);

        Assert.Equal(["Moon", "Planet", "Sun"], ids.Order());
        Assert.Empty(restricted);
    }

    [Fact]
    public void High_periapsis_speed_does_not_exclude_a_positive_mass_body()
    {
        var sun = Root();
        var sungrazer = Body("Lovejoy", sun, 2.8e13, 0.9999);

        var ids = IntegratedSetRule.Select([sun, sungrazer], 0,
            out var restricted);

        Assert.Equal(["Lovejoy", "Sun"], ids.Order());
        Assert.Empty(restricted);
    }

    [Fact]
    public void Zero_mass_bodies_are_classified_as_nonbackreacting_restricted_tracks()
    {
        var sun = Root();
        var tracers = Enumerable.Range(0, 100)
            .Select(i => Body($"Tracer{i:D3}", sun, 1e11 + i * 1e8, 0.01, mu: 0))
            .ToArray();

        var ids = IntegratedSetRule.Select([sun, .. tracers], 0,
            out var restricted);

        Assert.Equal([sun.Id], ids);
        Assert.Equal(tracers.Length, restricted.Count);
        Assert.All(restricted,
            item => Assert.Equal(RestrictedClassificationKind.NonBackreacting, item.Kind));
    }

    [Fact]
    public void Positive_mass_child_of_zero_mass_parent_is_restricted_by_ancestor_closure()
    {
        var sun = Root();
        var zeroParent = Body("ZeroParent", sun, 1e11, 0.01, mu: 0);
        var massiveChild = Body("MassiveChild", zeroParent, 1e8, 0.01, mu: 1e10);

        var ids = IntegratedSetRule.Select(
            [sun, zeroParent, massiveChild], 0, out var restricted);

        Assert.Equal([sun.Id], ids);
        Assert.Contains(restricted, item => item.Id == zeroParent.Id
            && item.Kind == RestrictedClassificationKind.NonBackreacting);
        Assert.Contains(restricted, item => item.Id == massiveChild.Id
            && item.Kind == RestrictedClassificationKind.Ancestor);
    }

    [Fact]
    public void Invalid_seed_rejects_the_catalog_instead_of_demoting()
    {
        var sun = Root();
        var corrupt = Body("Corrupt", sun, -1e11, 0.5);

        var error = Assert.Throws<ArgumentException>(() => IntegratedSetRule.Select(
            [sun, corrupt], 0, out _));

        Assert.Contains("Corrupt", error.Message);
        Assert.Contains("seed", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void More_than_64_positive_mass_bodies_all_join_the_mutual_backbone()
    {
        var sun = Root();
        var massive = Enumerable.Range(0, 128)
            .Select(i => Body($"Massive{i:D3}", sun, 1e11 + i * 1e8, 0.01, 1e10 + i))
            .ToArray();

        var ids = IntegratedSetRule.Select([sun, .. massive], 0, out var restricted);

        Assert.Equal(129, ids.Count);
        Assert.All(massive, body => Assert.Contains(body.Id, ids));
        Assert.Empty(restricted);
    }

    [Fact(Timeout = 2_000)]
    public async Task Parent_cycle_is_rejected_before_selection()
    {
        await Task.Yield();
        var sun = Root();
        var a = Body("A", sun, 1e11, 0.01);
        var b = Body("B", a, 1e8, 0.01);
        a.Parent = b;

        var error = Assert.Throws<ArgumentException>(() => IntegratedSetRule.Select(
            [sun, a, b], 0, out _));

        Assert.Contains("'A' -> 'B' -> 'A'", error.Message);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(-1)]
    public void Invalid_mass_rejects_the_catalog(double mu)
    {
        var sun = Root();
        var invalid = Body("Invalid", sun, 1e11, 0.01, mu);
        Assert.Throws<ArgumentException>(() => IntegratedSetRule.Select(
            [sun, invalid], 0, out _));
    }

    [Fact]
    public void Parent_outside_the_catalog_is_rejected()
    {
        var sun = Root();
        var external = new CelestialBody { Id = "External", Mu = MuSun };
        var child = Body("Child", external, 1e11, 0.01, mu: 0);

        var error = Assert.Throws<ArgumentException>(() => IntegratedSetRule.Select(
            [sun, child], 0, out _));

        Assert.Contains("Child->External", error.Message);
    }
}

using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class AstronomicalsParserTests
{
    private static IReadOnlyList<CelestialBody> ParseSample() =>
        AstronomicalsParser.ParseFile(Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"));

    [Fact]
    public void Parses_all_bodies_with_ids()
    {
        var bodies = ParseSample();
        Assert.Equal(["Sol", "Mercury", "TestMoon", "TestComet"], bodies.Select(b => b.Id).ToArray());
    }

    [Fact]
    public void Root_body_has_no_parent_and_no_orbit()
    {
        var sol = ParseSample().Single(b => b.Id == "Sol");
        Assert.Null(sol.Parent);
        Assert.Null(sol.Orbit);
        Assert.Equal(Constants.G * Constants.SolarMassKg, sol.Mu, sol.Mu * 1e-12);
        Assert.Equal(696342e3, sol.MeanRadius, 3);
    }

    [Fact]
    public void Mass_units_convert_to_mu()
    {
        var bodies = ParseSample();
        var mercury = bodies.Single(b => b.Id == "Mercury");
        Assert.Equal(Constants.G * 330.2e21, mercury.Mu, mercury.Mu * 1e-12); // Yg = 1e21 kg
        var moon = bodies.Single(b => b.Id == "TestMoon");
        Assert.Equal(Constants.G * 1e15, moon.Mu, moon.Mu * 1e-12);
    }

    [Fact]
    public void Orbit_elements_are_converted_to_si_radians()
    {
        var mercury = ParseSample().Single(b => b.Id == "Mercury");
        var el = mercury.Orbit!.Value;
        Assert.Equal(5.790896153292818e10, el.SemiMajorAxis, 1);
        Assert.Equal(7.003433958539783 * Math.PI / 180, el.Inclination, 12);
        Assert.Equal(0.2056462028967717, el.Eccentricity, 15);
        Assert.Equal(-563615.3399035392, el.TimeAtPeriapsis, 6);
    }

    [Fact]
    public void Parent_chain_is_resolved()
    {
        var moon = ParseSample().Single(b => b.Id == "TestMoon");
        Assert.Equal("Mercury", moon.Parent!.Id);
        Assert.Equal("Sol", moon.Parent.Parent!.Id);
    }

    [Fact]
    public void Xml_fallback_reconstructs_laplace_spheres_of_influence()
    {
        var bodies = ParseSample();
        var sol = bodies.Single(b => b.Id == "Sol");
        var mercury = bodies.Single(b => b.Id == "Mercury");
        double expected = mercury.Orbit!.Value.SemiMajorAxis
            * Math.Pow(mercury.Mu / sol.Mu, 0.4);

        Assert.Equal(double.PositiveInfinity, sol.SphereOfInfluence);
        Assert.Equal(expected, mercury.SphereOfInfluence, expected * 1e-12);
        foreach (var body in bodies.Where(b => b.Parent is not null
            && b.Orbit is { SemiMajorAxis: > 0 }))
        {
            double bodyExpected = body.Orbit!.Value.SemiMajorAxis
                * Math.Pow(body.Mu / body.Parent!.Mu, 0.4);
            Assert.Equal(bodyExpected, body.SphereOfInfluence, bodyExpected * 1e-12);
        }
    }

    private static IReadOnlyList<CelestialBody> ParseXml(string xml)
    {
        using var stream = new MemoryStream(System.Text.Encoding.UTF8.GetBytes(xml));
        return AstronomicalsParser.Parse(stream);
    }

    private static string PhysicalXml(
        string rootMass = "1e20", string childMass = "1e10",
        string radiusKm = "1", string semiMajorAxisKm = "100000",
        string eccentricity = "0.01", string inclinationDegrees = "0",
        string timeSeconds = "0") => $$"""
        <Assets>
          <Body Id="Root"><Mass Kg="{{rootMass}}" /><MeanRadius Km="1" /></Body>
          <Body Id="Child" Parent="Root">
            <Mass Kg="{{childMass}}" /><MeanRadius Km="{{radiusKm}}" />
            <Orbit DefinitionFrame="Ecliptic">
              <SemiMajorAxis Km="{{semiMajorAxisKm}}" />
              <Eccentricity Value="{{eccentricity}}" />
              <Inclination Degrees="{{inclinationDegrees}}" />
              <LongitudeOfAscendingNode Degrees="0" />
              <ArgumentOfPeriapsis Degrees="0" />
              <TimeAtPeriapsis Seconds="{{timeSeconds}}" />
            </Orbit>
          </Body>
        </Assets>
        """;

    [Theory]
    [InlineData("0")]
    [InlineData("-1")]
    [InlineData("NaN")]
    [InlineData("Infinity")]
    public void Root_mass_must_be_finite_and_positive(string mass)
    {
        var error = Assert.Throws<FormatException>(() =>
            ParseXml(PhysicalXml(rootMass: mass)));
        Assert.Contains("Root", error.Message);
        Assert.Contains("mass", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Theory]
    [InlineData("-1")]
    [InlineData("NaN")]
    [InlineData("Infinity")]
    public void Child_mass_must_be_finite_and_nonnegative(string mass)
    {
        var error = Assert.Throws<FormatException>(() =>
            ParseXml(PhysicalXml(childMass: mass)));
        Assert.Contains("Child", error.Message);
        Assert.Contains("mass", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Theory]
    [InlineData("-1")]
    [InlineData("NaN")]
    [InlineData("Infinity")]
    public void Mean_radius_must_be_finite_and_nonnegative(string radiusKm)
    {
        var error = Assert.Throws<FormatException>(() =>
            ParseXml(PhysicalXml(radiusKm: radiusKm)));
        Assert.Contains("Child", error.Message);
        Assert.Contains("radius", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Theory]
    [InlineData("-100000", "0.1", "0", "0")]
    [InlineData("100000", "-0.1", "0", "0")]
    [InlineData("100000", "1", "0", "0")]
    [InlineData("100000", "0.1", "-1", "0")]
    [InlineData("100000", "0.1", "0", "NaN")]
    [InlineData("100000", "0.1", "0", "Infinity")]
    public void Orbit_values_must_be_finite_and_physically_consistent(
        string semiMajorAxisKm, string eccentricity,
        string inclinationDegrees, string timeSeconds)
    {
        var error = Assert.Throws<FormatException>(() => ParseXml(PhysicalXml(
            semiMajorAxisKm: semiMajorAxisKm, eccentricity: eccentricity,
            inclinationDegrees: inclinationDegrees, timeSeconds: timeSeconds)));
        Assert.Contains("Child", error.Message);
        Assert.Contains("orbit", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    private static string ParentGraphXml(params (string Id, string? Parent)[] bodies) =>
        "<Assets>" + string.Concat(bodies.Select(body => body.Parent is null
            ? $$"""<Body Id="{{body.Id}}"><Mass Kg="1e20" /></Body>"""
            : $"""
                <Body Id="{body.Id}" Parent="{body.Parent}">
                  <Orbit DefinitionFrame="Ecliptic">
                    <SemiMajorAxis Km="100000" /><Eccentricity Value="0.01" />
                    <Inclination Degrees="0" /><LongitudeOfAscendingNode Degrees="0" />
                    <ArgumentOfPeriapsis Degrees="0" /><TimeAtPeriapsis Seconds="0" />
                  </Orbit>
                  <Mass Kg="1e10" />
                </Body>
                """)) + "</Assets>";

    [Fact]
    public void Self_parent_cycle_reports_the_exact_cycle_closure()
    {
        var ex = Assert.Throws<FormatException>(() =>
            ParseXml(ParentGraphXml(("Loop", "Loop"))));

        Assert.Contains("'Loop' -> 'Loop'", ex.Message);
    }

    [Fact]
    public void Two_body_parent_cycle_reports_both_edges()
    {
        var forward = Assert.Throws<FormatException>(() =>
            ParseXml(ParentGraphXml(("A", "B"), ("B", "A"))));
        var reversed = Assert.Throws<FormatException>(() =>
            ParseXml(ParentGraphXml(("B", "A"), ("A", "B"))));

        Assert.Equal(forward.Message, reversed.Message);
        Assert.Contains("'A' -> 'B' -> 'A'", reversed.Message);
    }

    [Fact]
    public void Disconnected_cycle_reports_only_cycle_members_not_its_feeder()
    {
        var ex = Assert.Throws<FormatException>(() => ParseXml(ParentGraphXml(
            ("Sol", null), ("Planet", "Sol"), ("Feeder", "B"),
            ("B", "A"), ("A", "B"))));

        Assert.Contains("'A' -> 'B' -> 'A'", ex.Message);
        Assert.DoesNotContain("Feeder", ex.Message);
        Assert.DoesNotContain("Planet", ex.Message);
    }

    [Fact]
    public void Disjoint_cycles_are_all_reported_in_canonical_order_without_feeders()
    {
        // The C/D cycle is encountered first and both feeders enter at the
        // ordinal-larger participant. Neither fact may affect the report.
        var ex = Assert.Throws<FormatException>(() => ParseXml(ParentGraphXml(
            ("Sol", null), ("TailD", "D"), ("D", "C"), ("C", "D"),
            ("TailB", "B"), ("B", "A"), ("A", "B"))));

        Assert.Equal(
            "Astronomicals parent cycle detected: "
            + "'A' -> 'B' -> 'A'; 'C' -> 'D' -> 'C'.",
            ex.Message);
        Assert.DoesNotContain("TailB", ex.Message);
        Assert.DoesNotContain("TailD", ex.Message);
        Assert.DoesNotContain("Sol", ex.Message);
    }

    [Fact]
    public void Multiple_parentless_roots_are_rejected_with_their_ids()
    {
        var ex = Assert.Throws<FormatException>(() => ParseXml(ParentGraphXml(
            ("RootB", null), ("Child", "RootB"), ("RootA", null))));

        Assert.Contains("RootA", ex.Message);
        Assert.Contains("RootB", ex.Message);
    }

    [Fact]
    public void Out_of_order_acyclic_hierarchy_parses_in_document_order()
    {
        var bodies = ParseXml(ParentGraphXml(
            ("Moon", "Planet"), ("Sol", null), ("Planet", "Sol")));

        Assert.Equal(["Moon", "Sol", "Planet"], bodies.Select(body => body.Id));
        var moon = bodies.Single(body => body.Id == "Moon");
        Assert.Equal("Planet", moon.Parent!.Id);
        Assert.Equal("Sol", moon.Parent.Parent!.Id);
    }

    [Fact]
    public void Non_root_child_orbit_without_definition_frame_throws()
    {
        // Non-root children require an explicit ecliptic definition frame because
        // parent-equatorial evaluation is unsupported.
        const string xml = """
            <Assets>
              <StellarBody Id="Sol"><Mass Suns="1" /></StellarBody>
              <PlanetaryBody Id="Planet" Parent="Sol">
                <Orbit DefinitionFrame="Ecliptic">
                  <SemiMajorAxis Km="5.7e7" /><Eccentricity Value="0.2" />
                  <Inclination Degrees="7" /><LongitudeOfAscendingNode Degrees="48" />
                  <ArgumentOfPeriapsis Degrees="29" /><TimeAtPeriapsis Seconds="0" />
                </Orbit>
                <Mass Yg="330.2" />
              </PlanetaryBody>
              <PlanetaryBody Id="FrameMoon" Parent="Planet">
                <Orbit>
                  <SemiMajorAxis Km="10000" /><Eccentricity Value="0.1" />
                  <Inclination Degrees="0" /><LongitudeOfAscendingNode Degrees="0" />
                  <ArgumentOfPeriapsis Degrees="0" /><TimeAtPeriapsis Seconds="0" />
                </Orbit>
                <Mass Kg="1E+15" />
              </PlanetaryBody>
            </Assets>
            """;
        var ex = Assert.Throws<FormatException>(() => ParseXml(xml));
        Assert.Contains("FrameMoon", ex.Message);
        Assert.Contains("Ecliptic", ex.Message);
    }

    [Fact]
    public void Root_child_orbit_without_definition_frame_parses()
    {
        // The root's equatorial frame coincides with the ecliptic, so its children
        // may omit `DefinitionFrame`.
        const string xml = """
            <Assets>
              <StellarBody Id="Sol"><Mass Suns="1" /></StellarBody>
              <PlanetaryBody Id="Planet" Parent="Sol">
                <Orbit>
                  <SemiMajorAxis Km="5.7e7" /><Eccentricity Value="0.2" />
                  <Inclination Degrees="7" /><LongitudeOfAscendingNode Degrees="48" />
                  <ArgumentOfPeriapsis Degrees="29" /><TimeAtPeriapsis Seconds="0" />
                </Orbit>
                <Mass Yg="330.2" />
              </PlanetaryBody>
            </Assets>
            """;
        var bodies = ParseXml(xml);
        Assert.Equal(["Sol", "Planet"], bodies.Select(b => b.Id).ToArray());
        Assert.NotNull(bodies.Single(b => b.Id == "Planet").Orbit);
    }

    [Fact]
    public void Real_game_file_parses_when_present()
    {
        const string gameFile = @"C:\Program Files\Kitten Space Agency\Content\Core\Astronomicals.xml";
        if (!File.Exists(gameFile)) return; // machine-dependent smoke test
        var bodies = AstronomicalsParser.ParseFile(gameFile);
        Assert.True(bodies.Count >= 8, $"expected the solar system, got {bodies.Count} bodies");
        Assert.Contains(bodies, b => b.Id == "Sol" && b.Parent is null);
        Assert.All(bodies.Where(b => b.Parent is not null), b => Assert.NotNull(b.Orbit));
    }

    [Fact]
    public void Mass_constants_override_changes_mu_for_unit_denominated_bodies()
    {
        const string xml = """
            <Astronomicals>
              <Star Id="Sun"><Mass Suns="1.0"/></Star>
              <Planet Id="P1" Parent="Sun">
                <Mass Earths="2.0"/>
                <Orbit>
                  <SemiMajorAxis Au="1.0"/><Eccentricity Value="0.1"/>
                  <Inclination Degrees="0"/><LongitudeOfAscendingNode Degrees="0"/>
                  <ArgumentOfPeriapsis Degrees="0"/><TimeAtPeriapsis Seconds="0"/>
                </Orbit>
              </Planet>
            </Astronomicals>
            """;
        static Stream AsStream(string s) => new MemoryStream(System.Text.Encoding.UTF8.GetBytes(s));

        var stock = AstronomicalsParser.Parse(AsStream(xml));
        Assert.Equal(Constants.G * Constants.SolarMassKg, stock.Single(b => b.Id == "Sun").Mu);

        var game = new MassConstants
        {
            G = 6.6743e-11,
            SolarMassKg = 1.98841e30,
            EarthMassKg = 5.0e24,
        };
        var overridden = AstronomicalsParser.Parse(AsStream(xml), game);
        Assert.Equal(6.6743e-11 * 1.98841e30, overridden.Single(b => b.Id == "Sun").Mu);
        Assert.Equal(6.6743e-11 * 2.0 * 5.0e24, overridden.Single(b => b.Id == "P1").Mu);
    }
}

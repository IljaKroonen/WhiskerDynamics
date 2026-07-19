using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class CatalogKernelTests
{
    private const double G = 6.6743e-11; // the game's GRAVITATIONAL_CONSTANT
    private const double SolMassKg = 1.988416e30;
    private const double MuSol = G * SolMassKg;

    // Mercury-like conic evaluated at the reference epoch.
    private static readonly OrbitalElements PlanetElements = new(
        SemiMajorAxis: 5.7909e10,
        Eccentricity: 0.2056,
        Inclination: 7.0 * Math.PI / 180,
        LongitudeOfAscendingNode: 48.3 * Math.PI / 180,
        ArgumentOfPeriapsis: 29.2 * Math.PI / 180,
        TimeAtPeriapsis: -563615.34);

    private static CatalogBody Root(string id = "Sol", double massKg = SolMassKg) =>
        new(id, massKg, ParentId: null, MeanRadiusM: 696342e3, RelPositionEcl: null, RelVelocityEcl: null);

    [Fact]
    public void Known_J2_body_gets_C20_field_when_catalog_supplies_its_pole()
    {
        var rotation = new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 7.292115e-5, 0);
        var earth = new CatalogBody("Earth", 5.9722e24, null, 6_371_000, null, null, rotation);

        var body = Assert.Single(CatalogKernel.Build([earth], G, out var diagnostics));

        Assert.Empty(diagnostics);
        Assert.NotNull(body.Geopotential);
        var c20 = Assert.Single(body.Geopotential.Coefficients);
        Assert.Equal((2, 0), (c20.Degree, c20.Order));
        Assert.True(c20.Cosine < 0);
    }

    private static CatalogBody FromElements(string id, double massKg, string parentId,
        in OrbitalElements elements, double parentMu, double meanRadiusM = 2.4e6)
    {
        var state = Kepler.StateFromElements(elements, parentMu, CatalogKernel.ReferenceEpochSeconds);
        return new CatalogBody(id, massKg, parentId, meanRadiusM, state.Position, state.Velocity);
    }

    private static string AssertSkippedBesideHealthySibling(CatalogBody invalid)
    {
        var healthy = FromElements("Healthy", 3.302e23, "Sol", PlanetElements, MuSol);
        var bodies = CatalogKernel.Build([Root(), invalid, healthy], G, out var diagnostics);

        Assert.Equal(["Sol", "Healthy"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains($"'{invalid.Id}'", diagnostic);
        return diagnostic;
    }

    private static void AssertPhysicalElements(in OrbitalElements elements, double parentMu)
    {
        double[] values =
        [
            elements.SemiMajorAxis,
            elements.Eccentricity,
            elements.Inclination,
            elements.LongitudeOfAscendingNode,
            elements.ArgumentOfPeriapsis,
            elements.TimeAtPeriapsis,
        ];
        Assert.All(values, value =>
            Assert.True(double.IsFinite(value), $"non-finite orbital element {value:R}"));
        Assert.True(elements.Eccentricity >= 0);
        Assert.True(
            elements.Eccentricity < 1 && elements.SemiMajorAxis > 0
            || elements.Eccentricity > 1 && elements.SemiMajorAxis < 0,
            $"inconsistent conic pair a={elements.SemiMajorAxis:R}, e={elements.Eccentricity:R}");
        Assert.InRange(elements.Inclination, 0, Math.PI);
        Assert.True(elements.LongitudeOfAscendingNode >= 0
                    && elements.LongitudeOfAscendingNode < 2 * Math.PI);
        Assert.True(elements.ArgumentOfPeriapsis >= 0
                    && elements.ArgumentOfPeriapsis < 2 * Math.PI);
        double periapsis = Kepler.PeriapsisDistance(elements);
        Assert.True(double.IsFinite(periapsis) && periapsis > 0,
            $"invalid periapsis distance {periapsis:R}");

        var evaluated = Kepler.StateFromElements(
            elements, parentMu, CatalogKernel.ReferenceEpochSeconds);
        Assert.True(IsFinite(evaluated.Position) && IsFinite(evaluated.Velocity));
    }

    private static bool IsFinite(in Vector3d vector) =>
        double.IsFinite(vector.X) && double.IsFinite(vector.Y) && double.IsFinite(vector.Z);

    [Fact]
    public void Two_body_catalog_builds_parent_linked_graph_with_mu_from_mass()
    {
        var planet = FromElements("Mercury", 3.302e23, "Sol", PlanetElements, MuSol);
        var bodies = CatalogKernel.Build([Root(), planet], G, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Equal(["Sol", "Mercury"], bodies.Select(b => b.Id).ToArray());

        var sol = bodies[0];
        Assert.Null(sol.Parent);
        Assert.Null(sol.Orbit); // root carries no orbit
        Assert.Equal(G * SolMassKg, sol.Mu);
        Assert.Equal(696342e3, sol.MeanRadius);

        var mercury = bodies[1];
        Assert.Same(sol, mercury.Parent);
        Assert.Equal(G * 3.302e23, mercury.Mu);
        Assert.Equal(2.4e6, mercury.MeanRadius);
    }

    [Fact]
    public void Catalog_preserves_game_sphere_of_influence()
    {
        const double soi = 1.12e8;
        var planet = FromElements("Mercury", 3.302e23, "Sol", PlanetElements, MuSol)
            with { SphereOfInfluenceM = soi };

        var bodies = CatalogKernel.Build([Root(), planet], G, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Equal(soi, bodies.Single(b => b.Id == "Mercury").SphereOfInfluence);
    }

    [Fact]
    public void Elements_match_ElementsFromState_of_the_input_state()
    {
        var state = Kepler.StateFromElements(PlanetElements, MuSol, CatalogKernel.ReferenceEpochSeconds);
        var planet = new CatalogBody("Mercury", 3.302e23, "Sol", 2.4e6, state.Position, state.Velocity);
        var bodies = CatalogKernel.Build([Root(), planet], G, out _);

        var expected = Kepler.ElementsFromState(state, MuSol, CatalogKernel.ReferenceEpochSeconds);
        Assert.Equal(expected, bodies.Single(b => b.Id == "Mercury").Orbit!.Value);
    }

    [Fact]
    public void Circular_orbit_state_builds_elements_that_reproduce_the_state()
    {
        // Live catalogs contain circular orbits; the kernel supplies their element
        // convention because `Kepler.ElementsFromState` rejects e = 0.
        double r = 3.5e8;
        double vCirc = Math.Sqrt(MuSol / r);
        double inc = 28.9 * Math.PI / 180;
        var position = new Vector3d(r, 0, 0);
        var velocity = new Vector3d(0, vCirc * Math.Cos(inc), vCirc * Math.Sin(inc));
        var moon = new CatalogBody("Pan", 4.95e15, "Sol", 1.4e4, position, velocity);

        var bodies = CatalogKernel.Build([Root(), moon], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var orbit = bodies.Single(b => b.Id == "Pan").Orbit!.Value;
        Assert.True(orbit.Eccentricity < 1e-8, $"expected circular, got e={orbit.Eccentricity}");
        Assert.Equal(r, orbit.SemiMajorAxis, r * 1e-9);
        Assert.Equal(inc, orbit.Inclination, 9);

        // Built elements must reproduce the reference-epoch state.
        var roundTrip = Kepler.StateFromElements(orbit, MuSol, CatalogKernel.ReferenceEpochSeconds);
        Assert.True((roundTrip.Position - position).Length() < 1e-6 * r,
            $"position off by {(roundTrip.Position - position).Length()} m");
        Assert.True((roundTrip.Velocity - velocity).Length() < 1e-6 * vCirc,
            $"velocity off by {(roundTrip.Velocity - velocity).Length()} m/s");
    }

    [Fact]
    public void Near_circular_conversion_preserves_reference_epoch_position_absolutely()
    {
        var defining = new OrbitalElements(
            SemiMajorAxis: 8e11, Eccentricity: 5e-9,
            Inclination: 0.3, LongitudeOfAscendingNode: 0.7,
            ArgumentOfPeriapsis: 1.1, TimeAtPeriapsis: 0);
        var state = Kepler.StateFromElements(
            defining, MuSol, CatalogKernel.ReferenceEpochSeconds);
        var body = new CatalogBody(
            "NearCircle", 1e20, "Sol", 1e5, state.Position, state.Velocity);

        var bodies = CatalogKernel.Build([Root(), body], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var orbit = bodies.Single(b => b.Id == "NearCircle").Orbit!.Value;
        Assert.Equal(0, orbit.Eccentricity);
        var rebuilt = Kepler.StateFromElements(
            orbit, MuSol, CatalogKernel.ReferenceEpochSeconds);
        Assert.True((rebuilt.Position - state.Position).Length() < 0.01,
            $"near-circular catalog conversion jumped "
            + $"{(rebuilt.Position - state.Position).Length():R} m at the defining epoch");
    }

    [Fact]
    public void Equatorial_circular_state_round_trips()
    {
        double r = 1.2e9;
        double vCirc = Math.Sqrt(MuSol / r);
        var position = new Vector3d(-r / Math.Sqrt(2), r / Math.Sqrt(2), 0);
        var velocity = new Vector3d(-vCirc / Math.Sqrt(2), -vCirc / Math.Sqrt(2), 0);
        var body = new CatalogBody("Disk", 1e20, "Sol", 1e5, position, velocity);

        var bodies = CatalogKernel.Build([Root(), body], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var orbit = bodies.Single(b => b.Id == "Disk").Orbit!.Value;
        var roundTrip = Kepler.StateFromElements(orbit, MuSol, CatalogKernel.ReferenceEpochSeconds);
        Assert.True((roundTrip.Position - position).Length() < 1e-6 * r);
        Assert.True((roundTrip.Velocity - velocity).Length() < 1e-6 * vCirc);
    }

    [Fact]
    public void Unknown_parent_is_skipped_with_diagnostic_not_thrown()
    {
        var planet = FromElements("Mercury", 3.302e23, "Sol", PlanetElements, MuSol);
        var orphan = FromElements("Orphan", 1e20, "Nibiru", PlanetElements, MuSol);

        var bodies = CatalogKernel.Build([Root(), planet, orphan], G, out var diagnostics);

        Assert.Equal(["Sol", "Mercury"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains("Orphan", diagnostic);
        Assert.Contains("Nibiru", diagnostic);
    }

    [Fact]
    public void Descendants_of_a_skipped_body_are_skipped_with_diagnostics()
    {
        var orphan = FromElements("Orphan", 1e24, "Nibiru", PlanetElements, MuSol);
        var moonOfOrphan = FromElements("OrphanMoon", 1e15, "Orphan", PlanetElements, G * 1e24);

        var bodies = CatalogKernel.Build([Root(), orphan, moonOfOrphan], G, out var diagnostics);

        Assert.Equal(["Sol"], bodies.Select(b => b.Id).ToArray());
        Assert.Equal(2, diagnostics.Count);
        Assert.Contains(diagnostics, d => d.Contains("Orphan"));
        Assert.Contains(diagnostics, d => d.Contains("OrphanMoon"));
    }

    [Fact]
    public void Zero_roots_throws_FormatException()
    {
        var a = FromElements("A", 1e23, "B", PlanetElements, MuSol);
        var b = FromElements("B", 1e23, "A", PlanetElements, MuSol);
        Assert.Throws<FormatException>(() => CatalogKernel.Build([a, b], G, out _));
    }

    [Fact]
    public void Multiple_roots_throws_FormatException()
    {
        var ex = Assert.Throws<FormatException>(
            () => CatalogKernel.Build([Root("Sol"), Root("Sol2")], G, out _));
        Assert.Contains("Sol2", ex.Message);
    }

    [Fact]
    public void Duplicate_ids_throw_FormatException()
    {
        var ex = Assert.Throws<FormatException>(
            () => CatalogKernel.Build([Root("Sol"), Root("Sol")], G, out _));
        Assert.Contains("Sol", ex.Message);
    }

    [Fact]
    public void Massless_root_throws_FormatException()
    {
        // A massless root invalidates barycentric momentum normalization.
        var ex = Assert.Throws<FormatException>(
            () => CatalogKernel.Build([Root(massKg: 0)], G, out _));
        Assert.Contains("Sol", ex.Message);
    }

    [Fact]
    public void Negative_mass_child_is_skipped_without_poisoning_a_healthy_sibling()
    {
        var invalid = FromElements("Negative", -1, "Sol", PlanetElements, MuSol);

        string diagnostic = AssertSkippedBesideHealthySibling(invalid);

        Assert.Contains("negative mass", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Zero_mass_child_is_preserved_as_a_test_particle()
    {
        var testParticle = FromElements("TestParticle", 0, "Sol", PlanetElements, MuSol);

        var bodies = CatalogKernel.Build([Root(), testParticle], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var built = bodies.Single(b => b.Id == "TestParticle");
        Assert.Equal(0, built.Mu);
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
    }

    [Fact]
    public void Coincident_parent_child_state_is_skipped_without_poisoning_a_healthy_sibling()
    {
        var invalid = new CatalogBody("Coincident", 1e20, "Sol", 1e5,
            Vector3d.Zero, new Vector3d(0, 1, 0));

        string diagnostic = AssertSkippedBesideHealthySibling(invalid);

        Assert.Contains("radius", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(12_000.0)]
    public void Zero_or_parallel_velocity_is_skipped_for_zero_angular_momentum(
        double radialSpeed)
    {
        var invalid = new CatalogBody("Radial", 1e20, "Sol", 1e5,
            new Vector3d(7e6, 0, 0), new Vector3d(radialSpeed, 0, 0));

        string diagnostic = AssertSkippedBesideHealthySibling(invalid);

        Assert.Contains("angular momentum", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Finite_components_with_overflowing_angular_momentum_are_skipped()
    {
        // Both input vectors have finite components and finite scaled magnitudes, but
        // their cross-product components overflow. Input-only validation is insufficient.
        var invalid = new CatalogBody("OverflowingGeometry", 1e20, "Sol", 1e5,
            new Vector3d(1e200, 0, 0), new Vector3d(0, 1e200, 0));

        string diagnostic = AssertSkippedBesideHealthySibling(invalid);

        Assert.Contains("angular momentum", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Extreme_scale_nondegenerate_hyperbola_uses_finite_stable_elements()
    {
        // The physical hyperbola has |a| large enough that a^3 overflows, but scaled
        // mean-motion arithmetic keeps its derived epoch finite and evaluable.
        double radius = 1e150;
        double speedScale = Math.Sqrt(MuSol / radius);
        var extreme = new CatalogBody("ExtremeHyperbola", 1e20, "Sol", 1e5,
            new Vector3d(radius, 0, 0), new Vector3d(speedScale, 1.5 * speedScale, 0));

        var bodies = CatalogKernel.Build([Root(), extreme], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var built = bodies.Single(b => b.Id == "ExtremeHyperbola");
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
    }

    [Fact]
    public void Finite_geometry_with_overflowing_element_derivation_is_skipped()
    {
        // Geometry, energy and eccentricity are finite, but e^2 overflows in the
        // hyperbolic anomaly calculation and makes TimeAtPeriapsis non-finite.
        var invalid = new CatalogBody("ElementOverflow", 1e20, "Sol", 1e5,
            new Vector3d(1e70, 0, 0), new Vector3d(0, 1e70, 0));

        string diagnostic = AssertSkippedBesideHealthySibling(invalid);

        Assert.Contains("orbital elements", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Finite_mass_whose_gravitational_parameter_overflows_is_contained()
    {
        const double customG = 1e100;
        const double rootMass = 1e-100;
        double parentMu = customG * rootMass;
        var root = new CatalogBody("Root", rootMass, null, 1, null, null);
        var elements = new OrbitalElements(1e6, 0.2, 0.1, 0.2, 0.3, 0);
        var invalid = FromElements(
            "MuOverflow", double.MaxValue, "Root", elements, parentMu, 1);
        var healthy = FromElements("Healthy", rootMass, "Root", elements, parentMu, 1);

        var bodies = CatalogKernel.Build(
            [root, invalid, healthy], customG, out var diagnostics);

        Assert.Equal(["Root", "Healthy"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains("'MuOverflow'", diagnostic);
        Assert.Contains("gravitational", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Near_radial_state_with_nonzero_angular_momentum_is_accepted()
    {
        const double earthMu = 3.986004418e14;
        var earth = Root("Earth", earthMu / G);
        var nearRadial = new CatalogBody("NearRadial", 1e3, "Earth", 1,
            new Vector3d(1.82e8, 0, 0), new Vector3d(2859.99, 5.12, 0));

        var bodies = CatalogKernel.Build([earth, nearRadial], G, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Equal(["Earth", "NearRadial"], bodies.Select(b => b.Id).ToArray());
        var built = bodies.Single(b => b.Id == "NearRadial");
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
    }

    [Fact]
    public void Near_parabolic_nonzero_energy_state_is_accepted()
    {
        var elements = new OrbitalElements(
            1e12, 0.99993, 0.2, 0.3, 0.4, CatalogKernel.ReferenceEpochSeconds);
        var comet = FromElements("NearParabola", 1e13, "Sol", elements, MuSol, 1);

        var bodies = CatalogKernel.Build([Root(), comet], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var built = bodies.Single(b => b.Id == "NearParabola");
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
    }

    [Fact]
    public void Tiny_circular_state_builds_without_poisoning_a_healthy_sibling()
    {
        const double parentMu = 1e-200;
        var root = new CatalogBody("TinyRoot", parentMu, null, 1, null, null);
        var tiny = new CatalogBody("TinyCircle", 1e-200, "TinyRoot", 1,
            new Vector3d(1e-200, 0, 0), new Vector3d(0, 1, 0));
        var healthy = new CatalogBody("Healthy", 1e-200, "TinyRoot", 1,
            new Vector3d(1, 0, 0), new Vector3d(0, Math.Sqrt(parentMu), 0));

        var bodies = CatalogKernel.Build([root, tiny, healthy], 1, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Equal(["TinyRoot", "TinyCircle", "Healthy"],
            bodies.Select(b => b.Id).ToArray());
        var built = bodies.Single(b => b.Id == "TinyCircle");
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
        var evaluated = Kepler.StateFromElements(
            built.Orbit.Value, built.Parent.Mu, CatalogKernel.ReferenceEpochSeconds);
        Assert.NotEqual(Vector3d.Zero, evaluated.Velocity);
    }

    [Fact]
    public void Huge_circular_state_advances_at_a_large_finite_time()
    {
        double radius = 1e200;
        double circularSpeed = Math.Sqrt(MuSol / radius);
        var huge = new CatalogBody("HugeCircle", 1e20, "Sol", 1,
            new Vector3d(radius, 0, 0), new Vector3d(0, circularSpeed, 0));

        var bodies = CatalogKernel.Build([Root(), huge], G, out var diagnostics);

        Assert.Empty(diagnostics);
        var built = bodies.Single(b => b.Id == "HugeCircle");
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
        var atEpoch = Kepler.StateFromElements(
            built.Orbit.Value, built.Parent.Mu, CatalogKernel.ReferenceEpochSeconds);
        var advanced = Kepler.StateFromElements(built.Orbit.Value, built.Parent.Mu, 1e290);
        Assert.True(IsFinite(advanced.Position) && IsFinite(advanced.Velocity));
        Assert.NotEqual(atEpoch.Position, advanced.Position);
    }

    [Fact]
    public void Tiny_noncircular_ellipse_with_finite_geometry_is_accepted()
    {
        const double parentMu = 2e-200;
        var root = new CatalogBody("TinyRoot", parentMu, null, 1, null, null);
        var ellipse = new CatalogBody("TinyEllipse", 1e-200, "TinyRoot", 1,
            new Vector3d(1e-200, 0, 0), new Vector3d(0, 1, 0));

        var bodies = CatalogKernel.Build([root, ellipse], 1, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Equal(["TinyRoot", "TinyEllipse"], bodies.Select(b => b.Id).ToArray());
        var built = bodies.Single(b => b.Id == "TinyEllipse");
        AssertPhysicalElements(built.Orbit!.Value, built.Parent!.Mu);
        var evaluated = Kepler.StateFromElements(
            built.Orbit.Value, built.Parent.Mu, CatalogKernel.ReferenceEpochSeconds);
        Assert.NotEqual(Vector3d.Zero, evaluated.Velocity);
    }

    [Fact]
    public void Positive_mass_whose_mu_underflows_is_skipped_but_zero_mass_survives()
    {
        const double customG = 1e-200;
        const double rootMass = 1e200;
        double parentMu = customG * rootMass;
        var root = new CatalogBody("Root", rootMass, null, 1, null, null);
        var elements = new OrbitalElements(2, 0.2, 0.1, 0.2, 0.3, 0);
        var underflow = FromElements(
            "Underflow", 1e-200, "Root", elements, parentMu, 1);
        var massless = FromElements("Massless", 0, "Root", elements, parentMu, 1);

        var bodies = CatalogKernel.Build(
            [root, underflow, massless], customG, out var diagnostics);

        Assert.Equal(["Root", "Massless"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains("'Underflow'", diagnostic);
        Assert.Contains("gravitational", diagnostic, StringComparison.OrdinalIgnoreCase);
        var builtMassless = bodies.Single(b => b.Id == "Massless");
        Assert.Equal(0, builtMassless.Mu);
        AssertPhysicalElements(builtMassless.Orbit!.Value, builtMassless.Parent!.Mu);
    }

    [Fact]
    public void Conversion_failed_parent_contains_its_descendant_and_preserves_a_sibling()
    {
        var invalidParent = FromElements("BadParent", -1, "Sol", PlanetElements, MuSol);
        var descendantElements = new OrbitalElements(1e8, 0.2, 0.1, 0.2, 0.3, 0);
        var descendant = FromElements(
            "Descendant", 1e10, "BadParent", descendantElements, G * 1e20, 1);
        var healthy = FromElements("Healthy", 3.302e23, "Sol", PlanetElements, MuSol);

        var bodies = CatalogKernel.Build(
            [Root(), invalidParent, descendant, healthy], G, out var diagnostics);

        Assert.Equal(["Sol", "Healthy"], bodies.Select(b => b.Id).ToArray());
        Assert.Equal(2, diagnostics.Count);
        Assert.Contains("BadParent", diagnostics[0]);
        Assert.Contains("negative mass", diagnostics[0], StringComparison.OrdinalIgnoreCase);
        Assert.Contains("Descendant", diagnostics[1]);
        Assert.Contains("BadParent", diagnostics[1]);
        Assert.Contains("parent", diagnostics[1], StringComparison.OrdinalIgnoreCase);
        Assert.Contains("skipped", diagnostics[1], StringComparison.OrdinalIgnoreCase);
    }

    [Theory]
    [InlineData(1e-200, 1e-200)]
    [InlineData(1e200, 1e200)]
    public void Root_with_invalid_derived_mu_throws(double gravitationalConstant, double massKg)
    {
        var root = new CatalogBody("InvalidRoot", massKg, null, 1, null, null);

        var error = Assert.Throws<FormatException>(
            () => CatalogKernel.Build([root], gravitationalConstant, out _));

        Assert.Contains("InvalidRoot", error.Message);
        Assert.Contains("derived", error.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Hyperbolic_state_converts_with_hyperbolic_elements()
    {
        // Hyperbolic catalog states use a < 0, e > 1 elements and must reproduce
        // the defining state at the reference epoch.
        double r = 2.0e11;
        double vEscape = Math.Sqrt(2 * MuSol / r);
        var comet = new CatalogBody("3I_ATLAS", 1e13, "Sol", 5e3,
            new Vector3d(r, 0, 0), new Vector3d(0, vEscape * 1.2, 0));
        var planet = FromElements("Mercury", 3.302e23, "Sol", PlanetElements, MuSol);

        var bodies = CatalogKernel.Build([Root(), comet, planet], G, out var diagnostics);

        Assert.Equal(["Sol", "3I_ATLAS", "Mercury"], bodies.Select(b => b.Id).ToArray());
        Assert.Empty(diagnostics);
        var orbit = bodies.Single(b => b.Id == "3I_ATLAS").Orbit!.Value;
        Assert.True(orbit.Eccentricity > 1);
        Assert.True(orbit.SemiMajorAxis < 0);
        var reproduced = Kepler.StateFromElements(orbit, MuSol, CatalogKernel.ReferenceEpochSeconds);
        Assert.True((reproduced.Position - new Vector3d(r, 0, 0)).Length() < 1.0,
            "defining state must reproduce at the reference epoch");
    }

    [Fact]
    public void Exactly_parabolic_state_is_skipped_with_diagnostic()
    {
        // Exact parabolas have no supported (a, e) representation.
        double r = 2.0e11;
        double vEscape = Math.Sqrt(2 * MuSol / r);
        var comet = new CatalogBody("Parabola", 1e13, "Sol", 5e3,
            new Vector3d(r, 0, 0), new Vector3d(0, vEscape, 0));
        var healthy = FromElements("Healthy", 3.302e23, "Sol", PlanetElements, MuSol);
        var bodies = CatalogKernel.Build([Root(), comet, healthy], G, out var diagnostics);

        Assert.Equal(["Sol", "Healthy"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains("Parabola", diagnostic);
        Assert.Contains("parabolic", diagnostic, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Missing_state_on_a_non_root_body_is_skipped_with_diagnostic()
    {
        var stateless = new CatalogBody("Ghost", 1e20, "Sol", 1e5, null, null);
        var bodies = CatalogKernel.Build([Root(), stateless], G, out var diagnostics);

        Assert.Equal(["Sol"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains("Ghost", diagnostic);
    }

    [Fact]
    public void Non_finite_state_is_skipped_with_diagnostic()
    {
        var broken = new CatalogBody("Broken", 1e20, "Sol", 1e5,
            new Vector3d(double.NaN, 0, 0), new Vector3d(0, 1e3, 0));
        var bodies = CatalogKernel.Build([Root(), broken], G, out var diagnostics);

        Assert.Equal(["Sol"], bodies.Select(b => b.Id).ToArray());
        string diagnostic = Assert.Single(diagnostics);
        Assert.Contains("Broken", diagnostic);
    }

    [Fact]
    public void Root_state_is_ignored_root_still_carries_null_orbit()
    {
        // A parentless body's state must not create an orbit.
        var rootWithState = new CatalogBody("Sol", SolMassKg, null, 696342e3,
            new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var bodies = CatalogKernel.Build([rootWithState], G, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Null(bodies.Single().Orbit);
        Assert.Null(bodies.Single().Parent);
    }

    [Fact]
    public void Catalog_order_is_preserved_for_surviving_bodies()
    {
        var moonMassKg = 1e24;
        var planet = FromElements("Planet", moonMassKg, "Sol", PlanetElements, MuSol);
        var moon = FromElements("Moon", 1e15, "Planet", PlanetElements with { SemiMajorAxis = 1e8 }, G * moonMassKg);
        // Linking must not depend on catalog order.
        var bodies = CatalogKernel.Build([Root(), moon, planet], G, out var diagnostics);

        Assert.Empty(diagnostics);
        Assert.Equal(["Sol", "Moon", "Planet"], bodies.Select(b => b.Id).ToArray());
        Assert.Equal("Planet", bodies.Single(b => b.Id == "Moon").Parent!.Id);
    }
}

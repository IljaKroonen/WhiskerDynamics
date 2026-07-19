using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Frames;

/// <summary>Tests frame-selector labels, tree visibility, defaults, and orbit presets.</summary>
public class FrameSelectorKernelTests
{

    [Fact]
    public void Inertial_abbreviates_to_initial_CI()
        => Assert.Equal("ECI", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.Inertial, "Earth", null)));

    [Fact]
    public void Two_body_fixed_abbreviates_to_both_initials_F()
    {
        Assert.Equal("ELF", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Luna")));
        Assert.Equal("LEF", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.TwoBodyFixed, "Luna", "Earth")));
    }

    [Fact]
    public void Surface_abbreviates_to_the_ECEF_convention()
    {
        Assert.Equal("ECEF", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.Surface, "Earth", null)));
        Assert.Equal("LCLF", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.Surface, "Luna", null)));
    }

    [Fact]
    public void Lowercase_body_ids_abbreviate_uppercased()
        => Assert.Equal("ECI", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.Inertial, "earth", null)));

    [Fact]
    public void Target_fixed_frame_uses_the_primary_initial_and_target_fixed_suffix()
        => Assert.Equal("ETF", FrameSelectorKernel.Abbreviate(
            new FrameSpec(FrameKind.TargetFixed, "Earth", "Rendezvous Target")));

    // DFS input: Sol > Earth > Luna, Mars > Deimos + Phobos.
    private static readonly (string Id, int Depth, string? ParentId)[] SolSystem =
    [
        ("Sol", 0, null), ("Earth", 1, "Sol"), ("Luna", 2, "Earth"),
        ("Mars", 1, "Sol"), ("Deimos", 2, "Mars"), ("Phobos", 2, "Mars"),
    ];

    [Fact]
    public void All_expanded_shows_every_body_with_its_pair_partner()
    {
        var rows = FrameSelectorKernel.VisibleRows(SolSystem, (_, _) => true);
        Assert.Equal(new[] { "Sol", "Earth", "Luna", "Mars", "Deimos", "Phobos" },
            rows.Select(r => r.Id).ToArray());
        Assert.Null(rows[0].ParentId);              // Sol is a root: no pair partner
        Assert.Equal("Sol", rows[1].ParentId);      // Earth pairs with Sol
        Assert.Equal("Earth", rows[2].ParentId);    // Luna pairs with Earth
        Assert.Equal("Mars", rows[4].ParentId);     // Deimos pairs with Mars
    }

    [Fact]
    public void Collapsed_planet_hides_its_moons_but_not_its_siblings()
    {
        // Collapsing Earth hides Luna without affecting Mars or its children.
        var rows = FrameSelectorKernel.VisibleRows(SolSystem,
            (id, _) => !string.Equals(id, "Earth", StringComparison.Ordinal));
        Assert.Equal(new[] { "Sol", "Earth", "Mars", "Deimos", "Phobos" },
            rows.Select(r => r.Id).ToArray());
    }

    [Fact]
    public void Collapsed_root_hides_the_whole_system()
    {
        var rows = FrameSelectorKernel.VisibleRows(SolSystem, (_, depth) => depth != 0);
        Assert.Equal(new[] { "Sol" }, rows.Select(r => r.Id).ToArray());
        Assert.True(rows[0].HasChildren);
        Assert.False(rows[0].Expanded);
    }

    [Fact]
    public void Leaves_report_no_children_and_are_never_asked_for_expansion()
    {
        int asked = 0;
        var rows = FrameSelectorKernel.VisibleRows(SolSystem, (_, _) => { asked++; return true; });
        Assert.Equal(new[] { true, true, false, true, false, false },
            rows.Select(r => r.HasChildren).ToArray());
        Assert.All(rows.Where(r => !r.HasChildren), r => Assert.False(r.Expanded));
        Assert.Equal(3, asked); // Sol, Earth, Mars — the only bodies with children
    }

    [Fact]
    public void Default_policy_roots_open_rest_closed_shows_planets_without_moons()
    {
        var rows = FrameSelectorKernel.VisibleRows(SolSystem,
            (_, depth) => FrameSelectorKernel.DefaultExpanded(depth));
        Assert.Equal(new[] { "Sol", "Earth", "Mars" }, rows.Select(r => r.Id).ToArray());
    }

    [Fact]
    public void Default_expansion_opens_roots_only()
    {
        Assert.True(FrameSelectorKernel.DefaultExpanded(0));
        Assert.False(FrameSelectorKernel.DefaultExpanded(1));
        Assert.False(FrameSelectorKernel.DefaultExpanded(2));
    }

    [Fact]
    public void Multiple_roots_walk_independently()
    {
        var twoStars = new (string Id, int Depth, string? ParentId)[]
        {
            ("Alpha", 0, null), ("Pup", 1, "Alpha"), ("Beta", 0, null), ("Whelp", 1, "Beta"),
        };
        var rows = FrameSelectorKernel.VisibleRows(twoStars,
            (id, _) => !string.Equals(id, "Alpha", StringComparison.Ordinal));
        Assert.Equal(new[] { "Alpha", "Beta", "Whelp" }, rows.Select(r => r.Id).ToArray());
        Assert.Equal("Beta", rows[2].ParentId); // Whelp hangs under Beta, not Alpha
    }

    [Fact]
    public void Empty_catalog_yields_no_rows()
        => Assert.Empty(FrameSelectorKernel.VisibleRows([], (_, _) => true));

    [Fact]
    public void Selected_target_is_injected_as_a_sibling_of_the_primarys_moons()
    {
        var target = new TargetFrameCandidate("Rendezvous Target", "Earth");
        var rows = FrameSelectorKernel.VisibleRows(SolSystem, (_, _) => true, target);
        Assert.Equal(new[] { "Sol", "Earth", "Luna", "Target: Rendezvous Target", "Mars", "Deimos", "Phobos" },
            rows.Select(r => r.Id).ToArray());
        var targetRow = rows[3];
        Assert.Equal(2, targetRow.Depth);
        Assert.Equal("Earth", targetRow.ParentId);
        Assert.Equal(target.Spec, targetRow.TargetFrame);
    }

    [Fact]
    public void Collapsing_the_primary_hides_its_target_along_with_its_moons()
    {
        var target = new TargetFrameCandidate("Rendezvous Target", "Earth");
        var rows = FrameSelectorKernel.VisibleRows(SolSystem,
            (id, _) => !string.Equals(id, "Earth", StringComparison.Ordinal), target);
        Assert.Equal(new[] { "Sol", "Earth", "Mars", "Deimos", "Phobos" },
            rows.Select(r => r.Id).ToArray());
    }


    [Fact]
    public void Default_frame_is_the_home_world_centred_inertial_when_offered()
        => Assert.Equal(new FrameSpec(FrameKind.Inertial, "Earth", null),
            FrameSelectorKernel.DefaultFrame(SolSystem));

    [Fact]
    public void Default_frame_falls_back_to_the_first_root_in_an_unfamiliar_catalog()
    {
        // Without a home world, the first inertial root is the default.
        var rows = new (string Id, int Depth, string? ParentId)[]
        {
            ("Kerbol", 0, null), ("Kerbin", 1, "Kerbol"),
        };
        Assert.Equal(new FrameSpec(FrameKind.Inertial, "Kerbol", null),
            FrameSelectorKernel.DefaultFrame(rows));
    }

    [Fact]
    public void Default_frame_is_null_only_for_an_empty_catalog()
        => Assert.Null(FrameSelectorKernel.DefaultFrame([]));


    [Fact]
    public void Future_computation_reports_rails_then_celestial_sampling()
    {
        var rails = FrameSelectorKernel.FutureComputationProgress(
            123.4, 100, 14600, 14600, 14600, celestialCurvesShown: true);
        Assert.Equal(TrajectoryComputationPhase.IntegratingRails, rails?.Phase);
        Assert.Equal(123.4, rails?.CompletedDays);
        Assert.Equal(14600, rails?.RequestedDays);
        Assert.Equal("Computing future trajectories...", rails?.PhaseLabel);

        var sampling = FrameSelectorKernel.FutureComputationProgress(
            14600, 123.4, 14600, 14600, 14600, celestialCurvesShown: true);
        Assert.Equal(TrajectoryComputationPhase.SamplingCelestialCurves, sampling?.Phase);
        Assert.Equal("Updating planet trajectories...", sampling?.PhaseLabel);
        Assert.Equal("123 of 14600 d", sampling?.CoverageLabel);

        Assert.Null(FrameSelectorKernel.FutureComputationProgress(
            14599.5, 14599.5, 14600, 14600, 14600, celestialCurvesShown: true));
        Assert.Null(FrameSelectorKernel.FutureComputationProgress(
            30, 0, 30, 30, 30, celestialCurvesShown: false));
    }

    [Fact]
    public void Future_computation_uses_only_the_visible_promised_window()
    {
        var rails = FrameSelectorKernel.FutureComputationProgress(
            20, 0, 14600, 30, 30, celestialCurvesShown: true);
        Assert.Equal(30, rails?.RequestedDays);
        Assert.Equal(2f / 3f, rails?.Fraction);

        rails = FrameSelectorKernel.FutureComputationProgress(
            -2, 0, 30, 30, 30, celestialCurvesShown: false);
        Assert.Equal(0, rails?.CompletedDays);
        Assert.Equal(0, rails?.Fraction);
    }

    [Fact]
    public void Effective_duration_is_reported_only_for_a_point_budget_cap()
    {
        Assert.Equal(7200, FrameSelectorKernel.PointBudgetEffectiveDuration(
            truncated: true, workLimited: false, dynamicsLimited: false, physicallyCut: false,
            startSeconds: 1000, displayedEndSeconds: 8200));
        Assert.Null(FrameSelectorKernel.PointBudgetEffectiveDuration(
            truncated: false, workLimited: false, dynamicsLimited: false, physicallyCut: false,
            startSeconds: 1000, displayedEndSeconds: 8200));
        Assert.Null(FrameSelectorKernel.PointBudgetEffectiveDuration(
            truncated: true, workLimited: true, dynamicsLimited: false, physicallyCut: false,
            startSeconds: 1000, displayedEndSeconds: 8200));
        Assert.Null(FrameSelectorKernel.PointBudgetEffectiveDuration(
            truncated: true, workLimited: false, dynamicsLimited: true, physicallyCut: false,
            startSeconds: 1000, displayedEndSeconds: 8200));
        Assert.Null(FrameSelectorKernel.PointBudgetEffectiveDuration(
            truncated: true, workLimited: false, dynamicsLimited: false, physicallyCut: true,
            startSeconds: 1000, displayedEndSeconds: 8200));
    }
}

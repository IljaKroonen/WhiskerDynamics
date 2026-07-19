using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Patching;
using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Frames;

public class FrameCatalogTests
{
    private static readonly string[] Integrated = ["Sun", "Earth", "Moon", "Jupiter"];

    public static IEnumerable<object[]> NonFiniteStateComponents()
    {
        foreach ((string name, double value) in new[]
        {
            ("NaN", double.NaN),
            ("positive infinity", double.PositiveInfinity),
            ("negative infinity", double.NegativeInfinity),
        })
        {
            foreach (string role in new[] { "primary", "secondary" })
            foreach (string member in new[] { "position", "velocity" })
            for (int component = 0; component < 3; component++)
                yield return [name, value, role, member, component];
        }
    }

    [Fact]
    public void Target_choice_is_current_only_for_the_exact_selected_candidate()
    {
        var target = new FrameSpec(FrameKind.TargetFixed, "Earth", "Station");
        Assert.True(FrameCatalog.TargetChoiceIsCurrent(target, target));
        Assert.False(FrameCatalog.TargetChoiceIsCurrent(target, null));
        Assert.False(FrameCatalog.TargetChoiceIsCurrent(target,
            new FrameSpec(FrameKind.TargetFixed, "Earth", "Other")));
        Assert.False(FrameCatalog.TargetChoiceIsCurrent(target,
            new FrameSpec(FrameKind.TargetFixed, "Luna", "Station")));
        Assert.True(FrameCatalog.TargetChoiceIsCurrent(
            new FrameSpec(FrameKind.Inertial, "Earth", null), null));
    }

    [Fact]
    public void Target_choice_retires_only_when_target_state_is_authoritative()
    {
        var selected = new FrameSpec(FrameKind.TargetFixed, "Earth", "Station");
        Assert.False(FrameCatalog.TargetChoiceShouldRetire(
            selected, targetStateKnown: false, currentTarget: null));
        Assert.True(FrameCatalog.TargetChoiceShouldRetire(
            selected, targetStateKnown: true, currentTarget: null));
        Assert.True(FrameCatalog.TargetChoiceShouldRetire(
            selected, targetStateKnown: true,
            new FrameSpec(FrameKind.TargetFixed, "Earth", "Other")));
        Assert.False(FrameCatalog.TargetChoiceShouldRetire(
            selected, targetStateKnown: true, selected));
    }

    [Fact]
    public void Accepts_all_frame_kinds_with_their_respective_anchor_shape()
    {
        Assert.Null(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Moon"), Integrated));
        Assert.Null(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TwoBodyFixed, "Sun", "Earth"), Integrated));
        Assert.Null(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.Inertial, "Moon", null), Integrated));
        Assert.Null(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.Surface, "Earth", null), Integrated));
        Assert.Null(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TargetFixed, "Earth", "RendezvousTarget"), Integrated));
    }

    [Fact]
    public void Rejects_unknown_bodies_identical_pairs_and_arity_mismatches()
    {
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Phobos"), Integrated)); // not integrated
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Earth"), Integrated));  // degenerate pair
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TwoBodyFixed, "Earth", null), Integrated));     // pair frame needs two
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.Inertial, "Vesta", null), Integrated));         // unknown body
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.Inertial, "Earth", "Moon"), Integrated));       // inertial takes one
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.Surface, "Earth", "Moon"), Integrated));        // surface takes one
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.Surface, "Vesta", null), Integrated));          // unknown body
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TargetFixed, "Earth", null), Integrated));      // target frame needs vessel
        Assert.NotNull(FrameCatalog.ValidateSpec(
            new FrameSpec(FrameKind.TargetFixed, "Vesta", "Target"), Integrated)); // primary must be a body
    }

    [Fact]
    public void Rejects_out_of_range_frame_kinds_instead_of_falling_through()
    {
        // Enum parsing accepts undefined numeric values, which validation must reject.
        Assert.Equal("unknown frame kind '4'", FrameCatalog.ValidateSpec(
            new FrameSpec((FrameKind)4, "Earth", null), Integrated));
        Assert.Equal("unknown frame kind '99'", FrameCatalog.ValidateSpec(
            new FrameSpec((FrameKind)99, "Earth", "Moon"), Integrated));
    }

    [Fact]
    public void Label_fails_loudly_on_an_out_of_range_kind_instead_of_faking_surface()
    {
        // Invalid specs must not collide with a valid frame's cache identity.
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new FrameSpec((FrameKind)99, "Earth", null).Label);
    }

    // Near-degenerate live geometry is finite but too ill-conditioned to activate.

    [Fact]
    public void Geometry_gate_accepts_healthy_and_mostly_radial_orbital_pairs()
    {
        var primary = new StateVector(new Vector3d(1e11, 2e10, 0), new Vector3d(0, 3e4, 0));
        var circular = new StateVector(primary.Position + new Vector3d(3.844e8, 0, 0),
            primary.Velocity + new Vector3d(0, 1022.0, 0));
        Assert.Null(FrameCatalog.ValidateGeometry(primary, circular));

        // sin(theta) ~ 1e-3 remains well above the geometry floor.
        var radialish = new StateVector(primary.Position + new Vector3d(3.844e8, 0, 0),
            primary.Velocity + new Vector3d(1000.0, 1.0, 0));
        Assert.Null(FrameCatalog.ValidateGeometry(primary, radialish));
    }

    [Fact]
    public void Geometry_gate_rejects_near_radial_motion_the_kernel_accepts()
    {
        // A nonzero cross product can still fall below the activation threshold.
        var primary = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var nearRadial = new StateVector(new Vector3d(1e8, 0, 0), new Vector3d(1000.0, 1e-6, 0));
        Assert.NotNull(FrameCatalog.ValidateGeometry(primary, nearRadial));

        var pose = FrameKernel.Rotating(primary, nearRadial);
        Assert.True(double.IsFinite(pose.ZAxis.X)
            && double.IsFinite(pose.ZAxis.Y) && double.IsFinite(pose.ZAxis.Z));
    }

    [Fact]
    public void Geometry_gate_rejects_coincident_and_motionless_pairs()
    {
        var still = new StateVector(new Vector3d(1e8, 0, 0), Vector3d.Zero);
        Assert.NotNull(FrameCatalog.ValidateGeometry(still, still)); // coincident
        Assert.NotNull(FrameCatalog.ValidateGeometry(
            new StateVector(Vector3d.Zero, Vector3d.Zero), still));  // no relative motion
    }

    [Theory]
    [MemberData(nameof(NonFiniteStateComponents))]
    public void Geometry_gate_rejects_every_nonfinite_state_component(
        string valueName, double value, string role, string member, int component)
    {
        _ = valueName; // carried into the theory display name by xUnit
        var primary = new StateVector(
            new Vector3d(10, 20, 30), new Vector3d(1, 2, 3));
        var secondary = new StateVector(
            new Vector3d(10_000_010, 20, 30), new Vector3d(1, 1_002, 3));

        static Vector3d WithComponent(Vector3d vector, int index, double replacement) => index switch
        {
            0 => vector with { X = replacement },
            1 => vector with { Y = replacement },
            _ => vector with { Z = replacement },
        };

        StateVector Corrupt(StateVector state) => member == "position"
            ? state with { Position = WithComponent(state.Position, component, value) }
            : state with { Velocity = WithComponent(state.Velocity, component, value) };

        if (role == "primary") primary = Corrupt(primary);
        else secondary = Corrupt(secondary);

        string reason = Assert.IsType<string>(FrameCatalog.ValidateGeometry(primary, secondary));
        Assert.Contains("finite", reason, StringComparison.OrdinalIgnoreCase);
    }

    public static IEnumerable<object[]> OverflowedGeometry()
    {
        var healthyPosition = new Vector3d(1, 0, 0);
        var healthyVelocity = new Vector3d(0, 1, 0);

        yield return
        [
            "position subtraction",
            new StateVector(new Vector3d(-double.MaxValue, 0, 0), Vector3d.Zero),
            new StateVector(new Vector3d(double.MaxValue, 0, 0), healthyVelocity),
        ];
        yield return
        [
            "velocity subtraction",
            new StateVector(Vector3d.Zero, new Vector3d(-double.MaxValue, 0, 0)),
            new StateVector(healthyPosition, new Vector3d(double.MaxValue, 1, 0)),
        ];
        yield return
        [
            "unrepresentable separation length",
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(double.MaxValue, double.MaxValue, 0), healthyVelocity),
        ];
        yield return
        [
            "unrepresentable relative velocity length",
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(healthyPosition,
                new Vector3d(double.MaxValue, double.MaxValue, 0)),
        ];
        yield return
        [
            "cross product",
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            new StateVector(new Vector3d(1e200, 0, 0), new Vector3d(0, 1e200, 0)),
        ];
    }

    [Theory]
    [MemberData(nameof(OverflowedGeometry))]
    public void Geometry_gate_rejects_nonfinite_intermediates_from_finite_components(
        string caseName, StateVector primary, StateVector secondary)
    {
        _ = caseName;
        string reason = Assert.IsType<string>(FrameCatalog.ValidateGeometry(primary, secondary));
        Assert.Contains("finite", reason, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Geometry_gate_and_pose_builder_preserve_scale_safe_extremes_above_the_floor()
    {
        var primary = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var huge = new StateVector(new Vector3d(1e300, 0, 0), new Vector3d(0, 1, 0));
        Assert.Null(FrameCatalog.ValidateGeometry(primary, huge));
        Assert.Null(FrameCatalog.TryCreateRotatingPose(primary, huge, out var hugePose));
        AssertFiniteOrthonormal(hugePose);
        Assert.Equal(1e300, hugePose.Scale);

        // Normalizing r and v independently preserves the above-floor angle even when
        // their raw cross product underflows at this tiny but otherwise valid scale.
        var tiny = new StateVector(
            new Vector3d(1e-200, 0, 0),
            new Vector3d(1e-200, 2 * FrameCatalog.MinRotationSine * 1e-200, 0));
        Assert.Null(FrameCatalog.ValidateGeometry(primary, tiny));
        Assert.Null(FrameCatalog.TryCreateRotatingPose(primary, tiny, out var tinyPose));
        AssertFiniteOrthonormal(tinyPose);

        // The same above-floor angle must survive at a scale whose naive squared
        // lengths overflow, without manufacturing a zero or non-finite basis.
        var largeRadial = new StateVector(
            new Vector3d(1e150, 0, 0),
            new Vector3d(1e150, 2 * FrameCatalog.MinRotationSine * 1e150, 0));
        Assert.Null(FrameCatalog.ValidateGeometry(primary, largeRadial));
        Assert.Null(FrameCatalog.TryCreateRotatingPose(primary, largeRadial, out var radialPose));
        AssertFiniteOrthonormal(radialPose);
    }

    [Fact]
    public void Pose_builder_refuses_nonfinite_geometry_with_a_diagnostic_and_default_pose()
    {
        var primary = new StateVector(Vector3d.Zero, Vector3d.Zero);
        var nonfiniteTarget = new StateVector(
            new Vector3d(double.NaN, 1, 0), new Vector3d(0, 1, 0));
        string targetReason = Assert.IsType<string>(
            FrameCatalog.TryCreateRotatingPose(primary, nonfiniteTarget, out var targetPose));
        Assert.Contains("finite", targetReason, StringComparison.OrdinalIgnoreCase);
        Assert.Equal(default, targetPose);

        var overflow = new StateVector(
            new Vector3d(1e200, 0, 0), new Vector3d(0, 1e200, 0));
        string overflowReason = Assert.IsType<string>(
            FrameCatalog.TryCreateRotatingPose(primary, overflow, out var overflowPose));
        Assert.Contains("finite", overflowReason, StringComparison.OrdinalIgnoreCase);
        Assert.Equal(default, overflowPose);
    }

    // Surface-frame rotation models use the same activation discipline.

    [Fact]
    public void Rotation_gate_accepts_a_healthy_tilted_spin_model()
    {
        double tilt = 23.4 * Math.PI / 180.0;
        var pole = new Vector3d(0, -Math.Sin(tilt), Math.Cos(tilt));
        var x = new Vector3d(1, 0, 0);
        var y = pole.Cross(x);
        Assert.Null(FrameCatalog.ValidateRotation(new BodyRotation(pole, x, y, 7.292e-5, 0)));
        // Zero spin is a valid fixed-orientation frame.
        Assert.Null(FrameCatalog.ValidateRotation(new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), 0.0, 0)));
        // A negative rotation rate is not degenerate.
        Assert.Null(FrameCatalog.ValidateRotation(new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), -3.0e-7, 0)));
    }

    [Fact]
    public void Rotation_gate_rejects_degenerate_spin_models()
    {
        var x = new Vector3d(1, 0, 0);
        var y = new Vector3d(0, 1, 0);
        var z = new Vector3d(0, 0, 1);
        Assert.NotNull(FrameCatalog.ValidateRotation(new BodyRotation(
            Vector3d.Zero, x, y, 1e-5, 0)));                       // zero pole
        Assert.NotNull(FrameCatalog.ValidateRotation(new BodyRotation(
            z * 0.5, x, y, 1e-5, 0)));                             // non-unit pole
        Assert.NotNull(FrameCatalog.ValidateRotation(new BodyRotation(
            z, x, x, 1e-5, 0)));                                   // collapsed basis
        Assert.NotNull(FrameCatalog.ValidateRotation(new BodyRotation(
            z, x, -y, 1e-5, 0)));                                  // left-handed
        Assert.NotNull(FrameCatalog.ValidateRotation(new BodyRotation(
            z, x, y, double.NaN, 0)));                             // non-finite rate
    }

    public static IEnumerable<object[]> NonFiniteRotationComponents()
    {
        foreach (double value in new[]
        {
            double.NaN, double.PositiveInfinity, double.NegativeInfinity,
        })
        {
            for (int axis = 0; axis < 3; axis++)
            for (int component = 0; component < 3; component++)
                yield return [value, axis, component];
        }
    }

    [Theory]
    [MemberData(nameof(NonFiniteRotationComponents))]
    public void Rotation_gate_rejects_every_nonfinite_basis_component(
        double value, int axis, int component)
    {
        var pole = new Vector3d(0, 0, 1);
        var x = new Vector3d(1, 0, 0);
        var y = new Vector3d(0, 1, 0);
        static Vector3d WithComponent(Vector3d vector, int index, double replacement) => index switch
        {
            0 => vector with { X = replacement },
            1 => vector with { Y = replacement },
            _ => vector with { Z = replacement },
        };
        if (axis == 0) pole = WithComponent(pole, component, value);
        else if (axis == 1) x = WithComponent(x, component, value);
        else y = WithComponent(y, component, value);

        string reason = Assert.IsType<string>(FrameCatalog.ValidateRotation(
            new BodyRotation(pole, x, y, 1e-5, 0)));
        Assert.Contains("finite", reason, StringComparison.OrdinalIgnoreCase);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Rotation_gate_rejects_nonfinite_rate_and_reference_time(double value)
    {
        var valid = new BodyRotation(
            new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 1e-5, 10);
        string rateReason = Assert.IsType<string>(
            FrameCatalog.ValidateRotation(valid with { AngularVelocity = value }));
        string timeReason = Assert.IsType<string>(
            FrameCatalog.ValidateRotation(valid with { ReferenceTime = value }));
        Assert.Contains("finite", rateReason, StringComparison.OrdinalIgnoreCase);
        Assert.Contains("finite", timeReason, StringComparison.OrdinalIgnoreCase);
    }

    public static IEnumerable<object[]> InvalidPoses()
    {
        var valid = new FramePose(
            new Vector3d(1, 2, 3), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), new Vector3d(0, 0, 1), 2);
        foreach (double value in new[]
        {
            double.NaN, double.PositiveInfinity, double.NegativeInfinity,
        })
        {
            yield return ["origin", valid with { Origin = new Vector3d(value, 2, 3) }];
            yield return ["x axis", valid with { XAxis = new Vector3d(1, value, 0) }];
            yield return ["y axis", valid with { YAxis = new Vector3d(0, 1, value) }];
            yield return ["z axis", valid with { ZAxis = new Vector3d(value, 0, 1) }];
            yield return ["scale", valid with { Scale = value }];
        }
        yield return ["zero scale", valid with { Scale = 0 }];
        yield return ["negative scale", valid with { Scale = -1 }];
        yield return ["collapsed basis", valid with { YAxis = valid.XAxis }];
        yield return ["non-unit basis", valid with { ZAxis = new Vector3d(0, 0, 0.5) }];
        yield return ["left-handed basis", valid with { YAxis = new Vector3d(0, -1, 0) }];
    }

    [Theory]
    [MemberData(nameof(InvalidPoses))]
    public void Pose_gate_rejects_nonfinite_or_degenerate_poses(string caseName, FramePose pose)
    {
        _ = caseName;
        Assert.NotNull(FrameCatalog.ValidatePose(pose));
    }

    [Fact]
    public void Pose_gate_accepts_a_finite_right_handed_scaled_pose()
    {
        var pose = new FramePose(
            new Vector3d(1e300, -1e-300, 7),
            new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, 0, 1),
            1e300);
        Assert.Null(FrameCatalog.ValidatePose(pose));
    }

    private static void AssertFiniteOrthonormal(FramePose pose)
    {
        foreach (var axis in new[] { pose.XAxis, pose.YAxis, pose.ZAxis })
        {
            Assert.True(double.IsFinite(axis.X));
            Assert.True(double.IsFinite(axis.Y));
            Assert.True(double.IsFinite(axis.Z));
            Assert.Equal(1.0, axis.Length(), 12);
        }
        Assert.Equal(0.0, pose.XAxis.Dot(pose.YAxis), 12);
        Assert.Equal(0.0, pose.YAxis.Dot(pose.ZAxis), 12);
        Assert.Equal(0.0, pose.ZAxis.Dot(pose.XAxis), 12);
        Assert.Equal(0.0, (pose.XAxis.Cross(pose.YAxis) - pose.ZAxis).Length(), 12);
        Assert.True(double.IsFinite(pose.Scale));
        Assert.True(pose.Scale > 0);
    }
}

/// <summary>Tests hierarchy ordering, depth, and nearest included ancestors.</summary>
public class FrameHierarchyOrderTests
{
    private static readonly Dictionary<string, string?> Parents = new()
    {
        ["Sol"] = null,
        ["Earth"] = "Sol",
        ["Luna"] = "Earth",
        ["Jupiter"] = "Sol",
        ["Mercury"] = "Sol",
        ["Mars"] = "Sol",     // not offered in the reduced-tree test set
        ["Phobos"] = "Mars",
    };

    private static string? ParentOf(string id) => Parents.GetValueOrDefault(id);

    [Fact]
    public void Orders_parents_before_children_with_ordinal_siblings_and_depths()
    {
        var ordered = FrameCatalog.HierarchyOrder(
            ["Luna", "Jupiter", "Sol", "Earth", "Mercury"], ParentOf);
        (string, int, string?)[] expected =
        [
            ("Sol", 0, null), ("Earth", 1, "Sol"), ("Luna", 2, "Earth"),
            ("Jupiter", 1, "Sol"), ("Mercury", 1, "Sol"),
        ];
        Assert.Equal(expected, ordered);
    }

    [Fact]
    public void Reduces_to_the_nearest_ancestor_within_the_offered_set()
    {
        // When Mars is omitted, Phobos attaches to its nearest included ancestor, Sol.
        var ordered = FrameCatalog.HierarchyOrder(["Sol", "Phobos", "Earth"], ParentOf);
        (string, int, string?)[] expected = [("Sol", 0, null), ("Earth", 1, "Sol"), ("Phobos", 1, "Sol")];
        Assert.Equal(expected, ordered);
    }

    [Fact]
    public void Unknown_parent_chains_become_roots_not_crashes()
    {
        var ordered = FrameCatalog.HierarchyOrder(["Phobos", "Luna"], ParentOf);
        (string, int, string?)[] expected = [("Luna", 0, null), ("Phobos", 0, null)];
        Assert.Equal(expected, ordered);
    }

    [Fact]
    public void Sort_key_orders_siblings_by_distance_with_comets_last()
    {
        // Siblings sort by orbital distance, with comets grouped last.
        var keys = new Dictionary<string, (int Group, double Distance)>(StringComparer.Ordinal)
        {
            ["Sol"] = (0, 0),
            ["Mercury"] = (0, 5.8e10),
            ["Earth"] = (0, 1.5e11),
            ["Jupiter"] = (0, 7.8e11),
            ["Halley"] = (1, 8.8e10),
        };
        var ordered = FrameCatalog.HierarchyOrder(
            ["Halley", "Jupiter", "Sol", "Earth", "Mercury"],
            id => id == "Sol" ? null : "Sol", id => keys[id]);
        Assert.Equal(new[] { "Sol", "Mercury", "Earth", "Jupiter", "Halley" },
            ordered.Select(o => o.Id).ToArray());
    }

    [Fact]
    public void Sort_key_ties_fall_back_to_ordinal_ids()
    {
        var ordered = FrameCatalog.HierarchyOrder(
            ["Sol", "B", "A"], id => id == "Sol" ? null : "Sol", _ => (0, 1.0));
        Assert.Equal(new[] { "Sol", "A", "B" }, ordered.Select(o => o.Id).ToArray());
    }
}

/// <summary>Tests sibling grouping and orbital-distance ordering.</summary>
public class SiblingSortKeyTests
{
    private static WhiskerDynamics.Core.OrbitalElements Conic(double sma, double e) =>
        new(sma, e, 0, 0, 0, 0);

    [Fact]
    public void Roots_and_unknown_orbits_key_to_the_front()
        => Assert.Equal((0, 0.0), FrameCatalog.SiblingSortKey(null));

    [Fact]
    public void Bound_bodies_key_on_semi_major_axis_in_the_first_group()
    {
        Assert.Equal((0, 1.5e11), FrameCatalog.SiblingSortKey(Conic(1.5e11, 0.017)));
        Assert.Equal((0, 7.6e13), FrameCatalog.SiblingSortKey(Conic(7.6e13, 0.85)));
    }

    [Fact]
    public void High_eccentricity_comets_key_to_the_trailing_group_by_periapsis()
    {
        var key = FrameCatalog.SiblingSortKey(Conic(2.667e12, 0.967));
        Assert.Equal(1, key.Group);
        Assert.Equal(2.667e12 * (1 - 0.967), key.Distance, 3);
    }

    [Fact]
    public void Hyperbolic_comets_key_by_periapsis_which_stays_positive()
    {
        // Hyperbolic bodies use positive periapsis rather than negative semi-major axis.
        var key = FrameCatalog.SiblingSortKey(Conic(-8.0e11, 1.2));
        Assert.Equal(1, key.Group);
        Assert.Equal(-8.0e11 * (1 - 1.2), key.Distance, 3);
        Assert.True(key.Distance > 0);
    }

    [Fact]
    public void Threshold_boundary_is_a_comet_at_and_above_not_below()
    {
        Assert.Equal(0, FrameCatalog.SiblingSortKey(
            Conic(1e12, FrameCatalog.CometEccentricity - 1e-9)).Group);
        Assert.Equal(1, FrameCatalog.SiblingSortKey(
            Conic(1e12, FrameCatalog.CometEccentricity)).Group);
    }
}

/// <summary>Tests celestial-line patch registration.</summary>
public class CelestialCurveRegistrationTests
{
    [Fact]
    public void CelestialLinePatch_is_registered_as_a_gameplay_patch()
    {
        Assert.Contains(typeof(CelestialLinePatch), GameplayPatchSet.PatchTypes);
    }
}

/// <summary>Tests the inactive FrameManager snapshot contract.</summary>
[Collection("flightplans-statics")]
public class FrameManagerTests
{
    [Fact]
    public void TryGetActive_is_false_with_defaulted_outs_when_no_frame_is_active()
    {
        FrameManager.ResetSessionStatics(); // known-inactive (statics-sweep seam)
        Assert.False(FrameManager.TryGetActive(out var spec, out var pose, out double t));
        Assert.Null(spec);
        Assert.Equal(default, pose);
        Assert.Equal(0.0, t);
    }

    [Fact]
    public void EnsureActiveOrDefault_returns_null_while_nothing_can_activate()
    {
        // Without rails or candidates, the call reports inactivity instead of throwing.
        FrameManager.ResetSessionStatics();
        Assert.Null(FrameManager.EnsureActiveOrDefault());
    }

    [Fact]
    public void Frame_selections_round_trip_independently_per_vessel()
    {
        FrameManager.ResetSessionStatics();
        try
        {
            var sidecar = new SidecarFile
            {
                FrameSelections =
                [
                    new SidecarFrameSelection
                    {
                        VesselId = "apollo",
                        Frame = new SidecarFrame
                        {
                            FrameKind = "TwoBodyFixed",
                            PrimaryId = "Earth",
                            SecondaryId = "Luna",
                        },
                    },
                    new SidecarFrameSelection
                    {
                        VesselId = "gemini",
                        Frame = new SidecarFrame
                        {
                            FrameKind = "Surface",
                            PrimaryId = "Earth",
                        },
                    },
                ],
            };

            Assert.Equal(2, FrameManager.ImportFrameSelections(sidecar));
            Assert.Equal(("TwoBodyFixed", "Earth", "Luna"),
                FrameTuple(FrameManager.SelectedFrameForSidecar("apollo")));
            Assert.Equal(("Surface", "Earth", null),
                FrameTuple(FrameManager.SelectedFrameForSidecar("gemini")));
            Assert.Null(FrameManager.SelectedFrameForSidecar("absent"));
            Assert.Null(FrameManager.SelectedFrameForSidecar("unknown"));
            Assert.Equal(["apollo", "gemini"],
                FrameManager.FrameSelectionsForSidecar().Select(x => x.VesselId));
            FrameManager.ForgetSelection("apollo");
            Assert.Null(FrameManager.SelectedFrameForSidecar("apollo"));
            Assert.Equal("gemini",
                Assert.Single(FrameManager.FrameSelectionsForSidecar()).VesselId);
        }
        finally
        {
            FrameManager.ResetSessionStatics();
        }
    }

    [Fact]
    public void Frame_selection_restore_drops_malformed_identities_independently()
    {
        FrameManager.ResetSessionStatics();
        try
        {
            var sidecar = new SidecarFile
            {
                FrameSelections =
                [
                    new SidecarFrameSelection
                    {
                        VesselId = "numeric-kind",
                        Frame = new SidecarFrame { FrameKind = "42", PrimaryId = "Earth" },
                    },
                    new SidecarFrameSelection
                    {
                        VesselId = "missing-secondary",
                        Frame = new SidecarFrame
                        {
                            FrameKind = "TargetFixed",
                            PrimaryId = "Earth",
                        },
                    },
                    new SidecarFrameSelection
                    {
                        VesselId = "stray-secondary",
                        Frame = new SidecarFrame
                        {
                            FrameKind = "Inertial",
                            PrimaryId = "Earth",
                            SecondaryId = "Luna",
                        },
                    },
                    new SidecarFrameSelection
                    {
                        VesselId = "valid",
                        Frame = new SidecarFrame
                        {
                            FrameKind = "Inertial",
                            PrimaryId = "Earth",
                        },
                    },
                ],
            };

            Assert.Equal(1, FrameManager.ImportFrameSelections(sidecar));
            Assert.Equal(("Inertial", "Earth", null),
                FrameTuple(FrameManager.SelectedFrameForSidecar("valid")));
            Assert.Null(FrameManager.SelectedFrameForSidecar("numeric-kind"));
            Assert.Null(FrameManager.SelectedFrameForSidecar("missing-secondary"));
            Assert.Null(FrameManager.SelectedFrameForSidecar("stray-secondary"));
        }
        finally
        {
            FrameManager.ResetSessionStatics();
        }
    }

    private static (string Kind, string Primary, string? Secondary) FrameTuple(SidecarFrame? frame)
    {
        Assert.NotNull(frame);
        return (frame.FrameKind, frame.PrimaryId, frame.SecondaryId);
    }
}

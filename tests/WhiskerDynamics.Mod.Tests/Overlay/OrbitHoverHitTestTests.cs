using Brutal.Numerics;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Overlay;

/// <summary>Coarse hover search correctness, conservative fallbacks, and the
/// point-count cost bound.</summary>
public class OrbitHoverHitTestTests
{
    [Fact]
    public void Max_budget_builds_at_most_thirty_two_immutable_blocks()
    {
        var points = new Vector3d[262_144];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(i, Math.Sin(i * 0.01), 10);
        int[] traversal = OverlayKernel.DecimateIndices(
            points.Length, DecimationMetrics.MaximumTraversalPoints);

        HoverBlockPlan plan = HoverHitTestKernel.BuildBlocks(points, traversal);
        ReadOnlySpan<HoverBlock> blocks = plan.Blocks;

        Assert.Equal(HoverHitTestKernel.MaximumBlocks, plan.Count);
        Assert.Equal(plan[0], blocks[0]);
        Assert.True(typeof(HoverBlockPlan).IsSealed);
        Assert.Equal(typeof(ReadOnlySpan<HoverBlock>),
            typeof(HoverBlockPlan).GetProperty(nameof(HoverBlockPlan.Blocks))!.PropertyType);
        int expectedFirst = 0;
        foreach (HoverBlock block in blocks)
        {
            Assert.Equal(expectedFirst, block.FirstTraversalSlot);
            Assert.InRange(block.EndTraversalSlot - block.FirstTraversalSlot,
                1, HoverHitTestKernel.BlockSize);
            Assert.True(block.HasFinitePoints);
            expectedFirst = block.EndTraversalSlot;
        }
        Assert.Equal(traversal.Length, expectedFirst);
    }

    [Fact]
    public void Plan_does_not_expose_or_retain_a_callers_mutable_block_array()
    {
        var original = new HoverBlock(
            0, 1, new Vector3d(1, 2, 3), new Vector3d(4, 5, 6),
            HasFinitePoints: true, HasUncertainPoints: false);
        HoverBlock[] supplied = [original];
        HoverBlockPlan plan = HoverBlockPlan.CreateUntrustedForTests(1, supplied);

        supplied[0] = default;

        Assert.Equal(original, plan[0]);
        ReadOnlySpan<HoverBlock> exposed = plan.Blocks;
        Assert.Equal(original, exposed[0]);
    }

    [Fact]
    public void Random_curved_perspective_queries_match_the_full_scan_oracle()
    {
        var random = new Random(0x82AABB);
        for (int trial = 0; trial < 24; trial++)
        {
            int count = 1_400 + trial * 17;
            var points = new Vector3d[count];
            for (int i = 0; i < count; i++)
            {
                double u = i / (double)(count - 1);
                double phase = u * (8 + trial % 5) * Math.PI;
                points[i] = new Vector3d(
                    800 * Math.Cos(phase) + 1800 * (u - 0.5)
                        + 25 * Math.Sin(13 * phase),
                    450 * Math.Sin(phase) + 80 * Math.Sin(2.3 * phase),
                    1500 + 350 * Math.Cos(0.31 * phase));
            }
            int[] traversal = OverlayKernel.DecimateIndices(count, 1_100);
            var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
            int first = trial % 3 == 0 ? count / 5 : 0;
            HoverPointRef? prefix = trial % 3 == 0
                ? HoverPointRef.Interpolated(first - 1, first, 0.37)
                : null;
            var mouse = new HoverScreenPoint(
                (float)(960 + random.NextDouble() * 1200 - 600),
                (float)(540 + random.NextDouble() * 900 - 450));
            var expectedProjector = new TestProjector(
                points, perspective: true, angle: trial * 0.07,
                scale: 0.8 + trial * 0.015, screenOffsetX: trial * 3);
            var actualProjector = expectedProjector;

            bool expectedFound = FullScan(traversal, first, prefix, mouse,
                ref expectedProjector, out HoverHit expected);
            bool actualFound = HoverHitTestKernel.TryNearest(
                traversal, blocks, first, prefix, mouse,
                ref actualProjector, out HoverHit actual);

            Assert.Equal(expectedFound, actualFound);
            if (expectedFound) AssertSameHit(expected, actual);
        }
    }

    [Fact]
    public void Random_orthographic_queries_match_the_full_scan_oracle()
    {
        var random = new Random(0x82_0A7A);
        for (int trial = 0; trial < 32; trial++)
        {
            int count = 520 + trial * 23;
            var points = new Vector3d[count];
            for (int i = 0; i < count; i++)
            {
                double walk = i == 0 ? 0.0 : points[i - 1].X;
                points[i] = new Vector3d(
                    walk + random.NextDouble() * 36.0 - 18.0,
                    240.0 * Math.Sin(i * 0.031 + trial * 0.17)
                        + random.NextDouble() * 8.0,
                    100.0 + random.NextDouble() * 30.0);
            }

            int[] traversal = OverlayKernel.DecimateIndices(count, 390);
            var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
            int first = trial % 4 == 0 ? count / 7 : 0;
            HoverPointRef? prefix = trial % 4 == 0
                ? HoverPointRef.Interpolated(first - 1, first, 0.61)
                : null;
            var mouse = new HoverScreenPoint(
                (float)(random.NextDouble() * 1_400.0 - 700.0),
                (float)(random.NextDouble() * 900.0 - 450.0));
            var expectedProjector = new TestProjector(points, perspective: false,
                angle: trial * 0.113, scale: 0.4 + trial * 0.037,
                screenOffsetX: -120 + trial, screenOffsetY: 75 - trial);
            var actualProjector = expectedProjector;

            bool expectedFound = FullScan(traversal, first, prefix, mouse,
                ref expectedProjector, out HoverHit expected);
            bool actualFound = HoverHitTestKernel.TryNearest(
                traversal, blocks, first, prefix, mouse,
                ref actualProjector, out HoverHit actual);

            Assert.Equal(expectedFound, actualFound);
            if (expectedFound) AssertSameHit(expected, actual);
        }
    }

    [Fact]
    public void Hairpin_self_intersection_and_long_chord_match_full_scan()
    {
        var points = new Vector3d[900];
        for (int i = 0; i < 300; i++)
            points[i] = new Vector3d(-900 + i * 6, 350, 1000);
        for (int i = 300; i < 600; i++)
        {
            double a = (i - 300) * 2 * Math.PI / 299;
            points[i] = new Vector3d(80 * Math.Sin(3 * a), 260 * Math.Sin(2 * a), 1000);
        }
        for (int i = 600; i < 900; i++)
            points[i] = new Vector3d(900 - (i - 600) * 6, -350, 1000);
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        HoverScreenPoint[] mice =
        [
            new(960, 540),
            new(1010, 505),
            new(500, 790),
        ];
        foreach (HoverScreenPoint mouse in mice)
        {
            var expectedProjector = new TestProjector(points, perspective: true);
            var actualProjector = expectedProjector;
            Assert.True(FullScan(traversal, 0, null, mouse,
                ref expectedProjector, out HoverHit expected));
            Assert.True(HoverHitTestKernel.TryNearest(
                traversal, blocks, 0, null, mouse,
                ref actualProjector, out HoverHit actual));
            AssertSameHit(expected, actual);
        }
    }

    [Fact]
    public void Interpolated_future_boundary_is_a_real_first_vertex()
    {
        Vector3d[] points =
        [
            new(0, 0, 10), new(10, 0, 10), new(20, 0, 10),
        ];
        int[] traversal = [0, 2];
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        HoverPointRef boundary = HoverPointRef.Interpolated(0, 1, 0.5);
        var mouse = new HoverScreenPoint(6, 0);
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);

        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, firstDenseIndex: 1, boundary, mouse,
            ref projector, out HoverHit hit));

        Assert.Equal(boundary, hit.Lo);
        Assert.Equal(new HoverPointRef(2), hit.Hi);
        Assert.Equal(1.0 / 15.0, hit.Fraction, 12);
        Assert.Equal(6f, hit.Projected.X, precision: 5);
    }

    [Fact]
    public void Clipped_boundary_carries_the_exact_clip_time()
    {
        const double loTime = 736410488.717449;
        const double hiTime = 736437217.818519;
        const double clipTime = 736422109.078926;
        double fraction = (clipTime - loTime) / (hiTime - loTime);
        double reconstructed = loTime * (1.0 - fraction) + hiTime * fraction;
        HoverPointRef boundary = HoverPointRef.ClippedBoundary(
            0, 1, fraction, clipTime);
        double[] times = [loTime, hiTime];

        Assert.NotEqual(clipTime, reconstructed);
        Assert.True(boundary.HasExactTime);
        Assert.Equal(clipTime, boundary.ExactTime);
        Assert.Equal(clipTime, OverlayKernel.ResolveHoverTime(boundary, times));
        Assert.Equal(reconstructed, OverlayKernel.ResolveHoverTime(
            HoverPointRef.Interpolated(0, 1, fraction), times));
    }

    [Fact]
    public void Clip_boundary_interpolates_staged_endpoints_not_large_sources()
    {
        var source = new Vector3d(1e16, 0, 0);
        var other = new Vector3d(1e16 + 2, 0, 0);
        var stageShift = new Vector3d(-1e16, 0, 0);
        HoverPointRef boundary = HoverPointRef.Interpolated(0, 1, 0.5);

        Vector3d oldOrder = source * 0.5 + other * 0.5 + stageShift;
        Vector3d resolved = OverlayKernel.ResolveHoverPosition(
            boundary, source + stageShift, other + stageShift);

        Assert.Equal(0.0, oldOrder.X);
        Assert.Equal(1.0, resolved.X);
        Assert.NotEqual(oldOrder.X, resolved.X);
    }

    [Fact]
    public void Explicit_exact_first_sample_precedes_the_decimated_suffix()
    {
        Vector3d[] points =
        [
            new(0, 0, 10), new(10, 0, 10), new(20, 0, 10),
            new(30, 0, 10), new(40, 0, 10),
        ];
        int[] traversal = [0, 2, 4];
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        var exactFirst = new HoverPointRef(1);
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);

        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, firstDenseIndex: 1, exactFirst,
            new HoverScreenPoint(15, 3), ref projector, out HoverHit hit));

        Assert.Equal(exactFirst, hit.Lo);
        Assert.Equal(new HoverPointRef(2), hit.Hi);
        Assert.Equal(0.5, hit.Fraction, 12);
        Assert.Equal(15f, hit.Projected.X, precision: 5);
    }

    [Fact]
    public void One_point_two_point_and_duplicate_lines_match_the_full_scan_oracle()
    {
        (Vector3d[] Points, int[] Traversal, HoverScreenPoint Mouse)[] cases =
        [
            ([new(3, 4, 10)], [0], new HoverScreenPoint(50, -20)),
            ([new(0, 0, 10), new(10, 0, 10)], [0, 1],
                new HoverScreenPoint(6, 3)),
            ([new(0, 0, 10), new(0, 0, 10), new(10, 0, 10)], [0, 1, 2],
                new HoverScreenPoint(0, 7)),
            ([new(-10, 2, 10), new(5, 2, 10), new(5, 2, 10), new(20, 2, 10)],
                [0, 1, 2, 3], new HoverScreenPoint(11, -4)),
        ];

        foreach (var testCase in cases)
        {
            var blocks = HoverHitTestKernel.BuildBlocks(
                testCase.Points, testCase.Traversal);
            var expectedProjector = new TestProjector(testCase.Points,
                perspective: false, screenOffsetX: 0, screenOffsetY: 0);
            var actualProjector = expectedProjector;

            Assert.True(FullScan(testCase.Traversal, 0, null, testCase.Mouse,
                ref expectedProjector, out HoverHit expected));
            Assert.True(HoverHitTestKernel.TryNearest(
                testCase.Traversal, blocks, 0, null, testCase.Mouse,
                ref actualProjector, out HoverHit actual));
            AssertSameHit(expected, actual);
        }
    }

    [Fact]
    public void Untrusted_block_plans_fall_back_to_exact_streaming()
    {
        Vector3d[] points = Enumerable.Range(0, 600)
            .Select(i => new Vector3d(i * 2, Math.Sin(i * 0.1) * 20, 100))
            .ToArray();
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        var mouse = new HoverScreenPoint(320, 7);
        var expectedProjector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);
        Assert.True(FullScan(traversal, 0, null, mouse,
            ref expectedProjector, out HoverHit expected));

        foreach (HoverBlockPlan plan in new[]
        {
            HoverBlockPlan.CreateUntrustedForTests(points.Length),
            HoverBlockPlan.CreateUntrustedForTests(points.Length,
                new HoverBlock(1, 200, points[1], points[199], true, false)),
            HoverBlockPlan.CreateUntrustedForTests(points.Length,
                new HoverBlock(0, 300, points[0], points[299], true, false)),
        })
        {
            var projector = new TestProjector(points, perspective: false,
                screenOffsetX: 0, screenOffsetY: 0);
            Assert.True(HoverHitTestKernel.TryNearest(
                traversal, plan, 0, null, mouse,
                ref projector, out HoverHit actual));
            AssertSameHit(expected, actual);
        }
    }

    [Fact]
    public void Plan_for_a_different_same_length_traversal_falls_back_to_full_scan()
    {
        Vector3d[] points = Enumerable.Range(0, 1_200)
            .Select(i => new Vector3d(i * 3.0, Math.Sin(i * 0.02), 100))
            .ToArray();
        int[] authoritative = Enumerable.Range(0, 600).Select(i => i * 2).ToArray();
        int[] different = Enumerable.Range(0, 600).Select(i => i * 2 + 1).ToArray();
        HoverBlockPlan wrongPlan = HoverHitTestKernel.BuildBlocks(
            points, authoritative);
        int targetSlot = 301;
        var mouse = new HoverScreenPoint(
            (float)points[different[targetSlot]].X, 0.25f);
        var expectedProjector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);
        var actualProjector = expectedProjector;

        Assert.True(FullScan(different, 0, null, mouse,
            ref expectedProjector, out HoverHit expected));
        Assert.True(HoverHitTestKernel.TryNearest(
            different, wrongPlan, 0, null, mouse,
            ref actualProjector, out HoverHit actual));

        AssertSameHit(expected, actual);
        Assert.Equal(0, actualProjector.BoundProjections);
        Assert.Equal(different.Length + 2, actualProjector.FineProjections);
    }

    [Fact]
    public void Equal_distance_tie_keeps_the_earliest_visible_vertex()
    {
        Vector3d[] points =
        [
            new(-10, 0, 10), new(10, 0, 10), new(1000, 1000, 10),
        ];
        int[] traversal = [0, 1, 2];
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);

        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, new HoverScreenPoint(0, 20),
            ref projector, out HoverHit hit));

        Assert.Equal(0, hit.Lo.SourceIndex);
    }

    [Fact]
    public void Near_plane_crossing_refines_and_wholly_behind_block_skips()
    {
        Vector3d[] crossing = Enumerable.Range(0, 300)
            .Select(i => new Vector3d(i - 150, Math.Sin(i), i - 100))
            .ToArray();
        int[] traversal = Enumerable.Range(0, crossing.Length).ToArray();
        var blocks = HoverHitTestKernel.BuildBlocks(crossing, traversal);
        var expectedProjector = new TestProjector(crossing, perspective: true,
            screenOffsetX: 0, screenOffsetY: 0);
        var actualProjector = expectedProjector;
        var mouse = new HoverScreenPoint(10, 10);
        bool expectedFound = FullScan(traversal, 0, null, mouse,
            ref expectedProjector, out HoverHit expected);
        bool actualFound = HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, mouse,
            ref actualProjector, out HoverHit actual);
        Assert.Equal(expectedFound, actualFound);
        if (expectedFound) AssertSameHit(expected, actual);
        Assert.True(actualProjector.FineProjections > 0);

        Vector3d[] behind = Enumerable.Range(0, 200)
            .Select(i => new Vector3d(i, i % 7, -100 - i))
            .ToArray();
        traversal = Enumerable.Range(0, behind.Length).ToArray();
        blocks = HoverHitTestKernel.BuildBlocks(behind, traversal);
        var behindProjector = new TestProjector(behind, perspective: true,
            screenOffsetX: 0, screenOffsetY: 0);
        Assert.False(HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, mouse,
            ref behindProjector, out _));
        Assert.Equal(0, behindProjector.FineProjections);
        Assert.Equal(8, behindProjector.BoundProjections);
    }

    [Fact]
    public void Mixed_and_all_nonfinite_blocks_are_uncertain_and_refine()
    {
        var points = new Vector3d[HoverHitTestKernel.BlockSize * 2];
        for (int i = 0; i < HoverHitTestKernel.BlockSize; i++)
            points[i] = new Vector3d(10_000 + i, 0, 100);
        int target = HoverHitTestKernel.BlockSize / 2;
        points[target] = new Vector3d(0.25, 0, 100);
        points[7] = new Vector3d(double.NaN, 0, 100);
        for (int i = HoverHitTestKernel.BlockSize; i < points.Length; i++)
            points[i] = new Vector3d(double.NaN, double.NaN, double.NaN);
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        HoverBlockPlan plan = HoverHitTestKernel.BuildBlocks(points, traversal);

        Assert.True(plan[0].HasFinitePoints);
        Assert.True(plan[0].HasUncertainPoints);
        Assert.False(plan[1].HasFinitePoints);
        Assert.True(plan[1].HasUncertainPoints);

        var expectedProjector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);
        var actualProjector = expectedProjector;
        var mouse = new HoverScreenPoint(0, 1);
        Assert.True(FullScan(traversal, 0, null, mouse,
            ref expectedProjector, out HoverHit expected));
        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, plan, 0, null, mouse,
            ref actualProjector, out HoverHit actual));

        AssertSameHit(expected, actual);
        Assert.Equal(points.Length + 2, actualProjector.FineProjections);
        Assert.Equal(0, actualProjector.BoundProjections);
    }

    [Fact]
    public void Entirely_nonfinite_plan_refines_before_reporting_no_hit()
    {
        var points = Enumerable.Repeat(
            new Vector3d(double.NaN, double.NaN, double.NaN),
            HoverHitTestKernel.BlockSize).ToArray();
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        HoverBlockPlan plan = HoverHitTestKernel.BuildBlocks(points, traversal);
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);

        Assert.False(plan[0].HasFinitePoints);
        Assert.True(plan[0].HasUncertainPoints);
        Assert.False(HoverHitTestKernel.TryNearest(
            traversal, plan, 0, null, new HoverScreenPoint(0, 0),
            ref projector, out _));
        Assert.Equal(points.Length, projector.FineProjections);
        Assert.Equal(0, projector.BoundProjections);
    }

    [Fact]
    public void Huge_screen_coordinates_and_near_plane_ambiguity_match_full_scan()
    {
        var points = new Vector3d[768];
        for (int i = 0; i < points.Length; i++)
        {
            // Every block straddles the projection singularity. Positive vertices
            // still project to very large but finite float screen coordinates.
            double depth = i % 17 == 0 ? -1e-9 : 1e-9 * (1.0 + i % 11);
            points[i] = new Vector3d(
                1e18 + (i % 29 - 14) * 2e15,
                -7e17 + (i % 31 - 15) * 3e15,
                depth);
        }
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        var cursorProjector = new TestProjector(points, perspective: true,
            screenOffsetX: 0, screenOffsetY: 0);
        HoverScreenPoint mouse = cursorProjector.Project(
            new HoverPointRef(401)).Screen;
        var expectedProjector = new TestProjector(points, perspective: true,
            screenOffsetX: 0, screenOffsetY: 0);
        var actualProjector = expectedProjector;

        Assert.True(float.IsFinite(mouse.X));
        Assert.True(float.IsFinite(mouse.Y));
        Assert.True(FullScan(traversal, 0, null, mouse,
            ref expectedProjector, out HoverHit expected));
        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, mouse,
            ref actualProjector, out HoverHit actual));
        AssertSameHit(expected, actual);
        Assert.Equal(points.Length + 2, actualProjector.FineProjections);
    }

    [Fact]
    public void NaN_vertices_and_camera_changes_never_reuse_stale_projection()
    {
        var points = new Vector3d[700];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(i - 350, Math.Sin(i * 0.04) * 90, 800);
        points[123] = new Vector3d(double.NaN, 0, 0);
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        var mouse = new HoverScreenPoint(1040, 600);

        foreach ((double angle, double scale, float offset) in new[]
        {
            (0.0, 1.0, 0f),
            (0.7, 2.5, -300f),
            (-1.2, 0.4, 550f),
        })
        {
            var expectedProjector = new TestProjector(
                points, true, angle, scale, offset);
            var actualProjector = expectedProjector;
            bool expectedFound = FullScan(traversal, 0, null, mouse,
                ref expectedProjector, out HoverHit expected);
            bool actualFound = HoverHitTestKernel.TryNearest(
                traversal, blocks, 0, null, mouse,
                ref actualProjector, out HoverHit actual);
            Assert.Equal(expectedFound, actualFound);
            if (expectedFound) AssertSameHit(expected, actual);
        }
    }

    [Fact]
    public void Ordinary_default_thirty_two_thousand_point_input_projects_one_fine_block()
    {
        var points = new Vector3d[32_768];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(i * 10.0, 0, 1000);
        int[] traversal = OverlayKernel.DecimateIndices(
            points.Length, DecimationMetrics.MaximumTraversalPoints);
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        int targetSlot = 15 * HoverHitTestKernel.BlockSize
            + HoverHitTestKernel.BlockSize / 2;
        float targetX = (float)points[traversal[targetSlot]].X;
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);

        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, new HoverScreenPoint(targetX, 1),
            ref projector, out _));

        Assert.Equal(HoverHitTestKernel.MaximumBlocks * 8,
            projector.BoundProjections);
        Assert.Equal(HoverHitTestKernel.BlockSize + 2,
            projector.FineProjections);
    }

    [Fact]
    public void All_unprunable_max_traversal_has_a_fixed_transform_bound()
    {
        var points = new Vector3d[DecimationMetrics.MaximumTraversalPoints];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d((i & 1) == 0 ? -1000 : 1000, 0, 100);
        int target = points.Length / 2 + 1;
        points[target] = new Vector3d(0.25, 0, 100);
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);

        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, new HoverScreenPoint(0, 1),
            ref projector, out HoverHit hit));

        Assert.Equal(target - 1, hit.Lo.SourceIndex);
        Assert.Equal(target, hit.Hi.SourceIndex);
        Assert.Equal(HoverHitTestKernel.MaximumBlocks * 8,
            projector.BoundProjections);
        Assert.Equal(DecimationMetrics.MaximumTraversalPoints + 2,
            projector.FineProjections);
    }

    [Fact]
    public void Warm_search_allocates_no_query_sized_scratch()
    {
        var points = new Vector3d[2_048];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vector3d(i * 3, Math.Sin(i * 0.03), 100);
        int[] traversal = Enumerable.Range(0, points.Length).ToArray();
        var blocks = HoverHitTestKernel.BuildBlocks(points, traversal);
        var projector = new TestProjector(points, perspective: false,
            screenOffsetX: 0, screenOffsetY: 0);
        var mouse = new HoverScreenPoint(3000, 0.2f);
        Assert.True(HoverHitTestKernel.TryNearest(
            traversal, blocks, 0, null, mouse, ref projector, out _));

        long before = GC.GetAllocatedBytesForCurrentThread();
        double checksum = 0;
        for (int i = 0; i < 256; i++)
        {
            Assert.True(HoverHitTestKernel.TryNearest(
                traversal, blocks, 0, null, mouse, ref projector, out HoverHit hit));
            checksum += hit.Fraction + hit.Projected.X;
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.True(checksum > 0);
        Assert.Equal(0, allocated);
    }

    private static bool FullScan<TProjector>(int[] traversal, int firstDenseIndex,
        HoverPointRef? prefix, HoverScreenPoint mouse, ref TProjector projector,
        out HoverHit hit)
        where TProjector : struct, IHoverPointProjector
    {
        int start = Array.BinarySearch(traversal, firstDenseIndex);
        if (start < 0) start = ~start;
        var points = new List<HoverPointRef>();
        if (prefix is { } p) points.Add(p);
        for (int slot = start; slot < traversal.Length; slot++)
            points.Add(new HoverPointRef(traversal[slot]));
        var screen = new float2[points.Count];
        for (int i = 0; i < points.Count; i++)
        {
            HoverScreenPoint projected = projector.Project(points[i]).Screen;
            screen[i] = new float2(projected.X, projected.Y);
        }
        if (!OverlayKernel.PolylineNearest(screen, new float2(mouse.X, mouse.Y),
                out int lo, out int hi, out double fraction, out float2 projectedPoint))
        {
            hit = default;
            return false;
        }
        hit = new HoverHit(points[lo], points[hi], fraction,
            new HoverScreenPoint(projectedPoint.X, projectedPoint.Y));
        return true;
    }

    private static void AssertSameHit(HoverHit expected, HoverHit actual)
    {
        Assert.Equal(expected.Lo, actual.Lo);
        Assert.Equal(expected.Hi, actual.Hi);
        Assert.Equal(expected.Fraction, actual.Fraction, 12);
        Assert.Equal(expected.Projected.X, actual.Projected.X, precision: 4);
        Assert.Equal(expected.Projected.Y, actual.Projected.Y, precision: 4);
    }

    private struct TestProjector : IHoverPointProjector
    {
        private readonly Vector3d[] _points;
        private readonly bool _perspective;
        private readonly double _cos, _sin, _scale;
        private readonly float _screenOffsetX, _screenOffsetY;

        public TestProjector(Vector3d[] points, bool perspective,
            double angle = 0, double scale = 1,
            float screenOffsetX = 960, float screenOffsetY = 540)
        {
            _points = points;
            _perspective = perspective;
            _cos = Math.Cos(angle);
            _sin = Math.Sin(angle);
            _scale = scale;
            _screenOffsetX = screenOffsetX;
            _screenOffsetY = screenOffsetY;
        }

        public int FineProjections { get; private set; }
        public int BoundProjections { get; private set; }
        public int SourceCount => _points.Length;
        public double NearPlaneDepth => _perspective ? 0.01 : 0.0;

        public HoverProjection Project(HoverPointRef point)
        {
            FineProjections++;
            Vector3d source = _points[point.SourceIndex];
            if (point.IsInterpolated)
                source = source * (1.0 - point.SourceFraction)
                    + _points[point.OtherSourceIndex] * point.SourceFraction;
            return ProjectCoordinate(source, hideBehind: true);
        }

        public HoverProjection ProjectSource(Vector3d sourceCoordinate)
        {
            BoundProjections++;
            return ProjectCoordinate(sourceCoordinate, hideBehind: false);
        }

        public HoverProjectedBounds ProjectBounds(
            Vector3d minimum, Vector3d maximum)
        {
            float minX = float.PositiveInfinity, minY = float.PositiveInfinity;
            float maxX = float.NegativeInfinity, maxY = float.NegativeInfinity;
            double minDepth = double.PositiveInfinity;
            double maxDepth = double.NegativeInfinity;
            bool uncertain = false;
            for (int corner = 0; corner < 8; corner++)
            {
                var source = new Vector3d(
                    (corner & 1) == 0 ? minimum.X : maximum.X,
                    (corner & 2) == 0 ? minimum.Y : maximum.Y,
                    (corner & 4) == 0 ? minimum.Z : maximum.Z);
                HoverProjection projection = ProjectSource(source);
                if (!double.IsFinite(projection.Depth)
                    || !float.IsFinite(projection.Screen.X)
                    || !float.IsFinite(projection.Screen.Y))
                {
                    uncertain = true;
                    continue;
                }
                minDepth = Math.Min(minDepth, projection.Depth);
                maxDepth = Math.Max(maxDepth, projection.Depth);
                minX = Math.Min(minX, projection.Screen.X);
                minY = Math.Min(minY, projection.Screen.Y);
                maxX = Math.Max(maxX, projection.Screen.X);
                maxY = Math.Max(maxY, projection.Screen.Y);
            }
            if (!uncertain && maxDepth < 0.0)
                return new HoverProjectedBounds(
                    HoverBoundsKind.WhollyBehind, 0, 0, 0, 0);
            if (uncertain || minDepth <= NearPlaneDepth)
                return new HoverProjectedBounds(
                    HoverBoundsKind.Unprunable, 0, 0, 0, 0);
            return new HoverProjectedBounds(
                HoverBoundsKind.Bounded,
                Math.BitDecrement((double)minX),
                Math.BitDecrement((double)minY),
                Math.BitIncrement((double)maxX),
                Math.BitIncrement((double)maxY));
        }

        private HoverProjection ProjectCoordinate(Vector3d source, bool hideBehind)
        {
            double x = _scale * (_cos * source.X - _sin * source.Y);
            double y = _scale * (_sin * source.X + _cos * source.Y);
            double depth = _scale * source.Z;
            if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(depth)
                || hideBehind && depth < 0)
                return new HoverProjection(
                    new HoverScreenPoint(float.NaN, float.NaN), depth);
            if (_perspective)
            {
                if (depth == 0)
                    return new HoverProjection(
                        new HoverScreenPoint(float.NaN, float.NaN), depth);
                x = 720 * x / depth;
                y = 720 * y / depth;
            }
            return new HoverProjection(
                new HoverScreenPoint(
                    _screenOffsetX + (float)x,
                    _screenOffsetY + (float)y),
                depth);
        }
    }
}

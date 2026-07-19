using System.Buffers;
using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Default and maximum-budget hover queries. The baseline is the bounded
/// full traversal shipped before the block search: project every traversal vertex
/// into pooled scratch, then apply the stock-compatible nearest-polyline rule. The
/// optimized case uses the same immutable traversal and a synthetic perspective
/// projector, keeping KSA Camera out of the measurement.</summary>
[MemoryDiagnoser]
[ShortRunJob]
public class OrbitHoverSearchBenchmarks
{
    [Params(32_768, 262_144)]
    public int PointCount;

    private Vector3d[] _points = null!;
    private DecimationMetrics _metrics = null!;
    private SyntheticPerspectiveProjector _projector;
    private HoverScreenPoint _mouse;

    [GlobalSetup]
    public void Setup()
    {
        _points = new Vector3d[PointCount];
        for (int i = 0; i < PointCount; i++)
        {
            double u = i / (double)(PointCount - 1);
            double phase = 40.0 * Math.PI * u;
            // Twenty wavy revolutions with a strong chronological drift: genuinely
            // curved input, while a cursor near one epoch normally admits one AABB
            // block instead of manufacturing an overlap-heavy fallback benchmark.
            _points[i] = new Vector3d(
                12_000.0 * (u - 0.5) + 180.0 * Math.Cos(phase)
                    + 24.0 * Math.Cos(3.0 * phase),
                150.0 * Math.Sin(phase) + 18.0 * Math.Sin(0.37 * phase),
                4_000.0 + 90.0 * Math.Cos(0.23 * phase));
        }

        _metrics = DecimationMetrics.For(_points);
        _projector = new SyntheticPerspectiveProjector(_points);
        int cursorSlot = 5 * (_metrics.TraversalIndices.Length - 1) / 8;
        var cursorProjection = _projector.Project(
            new HoverPointRef(_metrics.TraversalIndices[cursorSlot]));
        _mouse = new HoverScreenPoint(
            cursorProjection.Screen.X + 0.35f,
            cursorProjection.Screen.Y - 0.2f);

        // Warm ArrayPool and the kernel/JIT paths outside measurement, then
        // pin behavioral parity before BenchmarkDotNet launches its iterations.
        HoverHit baseline = FullTraversalCore();
        HoverHit optimized = CoarseToFineCore();
        if (baseline.Lo != optimized.Lo || baseline.Hi != optimized.Hi
            || Math.Abs(baseline.Fraction - optimized.Fraction) > 1e-12
            || Math.Abs(baseline.Projected.X - optimized.Projected.X) > 1e-4f
            || Math.Abs(baseline.Projected.Y - optimized.Projected.Y) > 1e-4f)
        {
            throw new InvalidOperationException(
                $"hover benchmark setup disagrees: full={baseline}, coarse={optimized}");
        }
    }

    [Benchmark(Baseline = true)]
    public double FullTraversal()
        => Checksum(FullTraversalCore());

    [Benchmark]
    public double CoarseToFine()
        => Checksum(CoarseToFineCore());

    private HoverHit FullTraversalCore()
    {
        int[] traversal = _metrics.TraversalIndices;
        HoverScreenPoint[] rented = ArrayPool<HoverScreenPoint>.Shared.Rent(traversal.Length);
        try
        {
            Span<HoverScreenPoint> screen = rented.AsSpan(0, traversal.Length);
            for (int slot = 0; slot < traversal.Length; slot++)
                screen[slot] = _projector.Project(
                    new HoverPointRef(traversal[slot])).Screen;

            if (!PolylineNearest(screen, _mouse,
                    out int lo, out int hi, out double fraction,
                    out HoverScreenPoint projected))
                throw new InvalidOperationException("deterministic benchmark line was not projectable");

            return new HoverHit(
                new HoverPointRef(traversal[lo]),
                new HoverPointRef(traversal[hi]),
                fraction,
                projected);
        }
        finally
        {
            ArrayPool<HoverScreenPoint>.Shared.Return(rented);
        }
    }

    private HoverHit CoarseToFineCore()
    {
        var projector = _projector;
        if (!HoverHitTestKernel.TryNearest(
                _metrics.TraversalIndices,
                _metrics.HoverPlan,
                firstDenseIndex: 0,
                prefix: null,
                _mouse,
                ref projector,
                out HoverHit hit))
            throw new InvalidOperationException("deterministic benchmark line was not projectable");
        return hit;
    }

    private static double Checksum(HoverHit hit) =>
        hit.Lo.SourceIndex * 0.125
        + hit.Hi.SourceIndex * 0.25
        + hit.Fraction
        + hit.Projected.X * 1e-3
        + hit.Projected.Y * 2e-3;

    // KSA-free copy of OverlayKernel.PolylineNearest's shipped rule. Keeping the
    // oracle on HoverScreenPoint avoids a direct Brutal.Numerics dependency.
    private static bool PolylineNearest(ReadOnlySpan<HoverScreenPoint> screen,
        HoverScreenPoint mouse, out int lo, out int hi, out double fraction,
        out HoverScreenPoint projected)
    {
        lo = hi = -1;
        fraction = 0.0;
        projected = default;
        float best = float.MaxValue;
        for (int i = 0; i < screen.Length; i++)
        {
            HoverScreenPoint p = screen[i];
            if (float.IsNaN(p.X) || float.IsNaN(p.Y)) continue;
            float distance = Math.Abs(mouse.X - p.X) + Math.Abs(mouse.Y - p.Y);
            if (distance < best)
            {
                best = distance;
                lo = hi = i;
            }
        }
        if (lo < 0) return false;

        projected = screen[lo];
        double bestSquared = SquaredDistance(projected, mouse);
        int vertex = lo;
        ConsiderSegment(screen, vertex - 1, vertex, mouse,
            ref lo, ref hi, ref fraction, ref projected, ref bestSquared);
        ConsiderSegment(screen, vertex, vertex + 1, mouse,
            ref lo, ref hi, ref fraction, ref projected, ref bestSquared);
        return true;
    }

    private static void ConsiderSegment(ReadOnlySpan<HoverScreenPoint> screen,
        int a, int b, HoverScreenPoint mouse,
        ref int lo, ref int hi, ref double fraction,
        ref HoverScreenPoint projected, ref double bestSquared)
    {
        if (a < 0 || b >= screen.Length) return;
        HoverScreenPoint pa = screen[a], pb = screen[b];
        if (float.IsNaN(pa.X) || float.IsNaN(pa.Y)
            || float.IsNaN(pb.X) || float.IsNaN(pb.Y)) return;
        double abX = pb.X - pa.X, abY = pb.Y - pa.Y;
        double lengthSquared = abX * abX + abY * abY;
        if (lengthSquared <= 0.0) return;
        double t = Math.Clamp(
            ((mouse.X - pa.X) * abX + (mouse.Y - pa.Y) * abY) / lengthSquared,
            0.0, 1.0);
        var candidate = new HoverScreenPoint(
            (float)(pa.X + t * abX),
            (float)(pa.Y + t * abY));
        double distanceSquared = SquaredDistance(candidate, mouse);
        if (distanceSquared >= bestSquared) return;
        bestSquared = distanceSquared;
        lo = a;
        hi = b;
        fraction = t;
        projected = candidate;
    }

    private static double SquaredDistance(HoverScreenPoint a, HoverScreenPoint b)
    {
        double dx = a.X - b.X, dy = a.Y - b.Y;
        return dx * dx + dy * dy;
    }

    private readonly struct SyntheticPerspectiveProjector(Vector3d[] points)
        : IHoverPointProjector
    {
        private readonly Vector3d[] _points = points;

        public int SourceCount => _points.Length;
        public double NearPlaneDepth => 0.1;

        public HoverProjection Project(HoverPointRef point) =>
            ProjectSource(_points[point.SourceIndex]);

        public HoverProjection ProjectSource(Vector3d sourceCoordinate)
        {
            double depth = sourceCoordinate.Z;
            if (!(depth > 0.0) || !double.IsFinite(depth))
                return new HoverProjection(
                    new HoverScreenPoint(float.NaN, float.NaN), depth);
            const double focalPixels = 720.0;
            return new HoverProjection(
                new HoverScreenPoint(
                    (float)(960.0 + focalPixels * sourceCoordinate.X / depth),
                    (float)(540.0 + focalPixels * sourceCoordinate.Y / depth)),
                depth);
        }

        public HoverProjectedBounds ProjectBounds(
            Vector3d minimum, Vector3d maximum)
        {
            double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
            double minDepth = double.PositiveInfinity;
            double maxDepth = double.NegativeInfinity;
            for (int corner = 0; corner < 8; corner++)
            {
                var source = new Vector3d(
                    (corner & 1) == 0 ? minimum.X : maximum.X,
                    (corner & 2) == 0 ? minimum.Y : maximum.Y,
                    (corner & 4) == 0 ? minimum.Z : maximum.Z);
                HoverProjection projection = ProjectSource(source);
                minDepth = Math.Min(minDepth, projection.Depth);
                maxDepth = Math.Max(maxDepth, projection.Depth);
                if (!float.IsFinite(projection.Screen.X)
                    || !float.IsFinite(projection.Screen.Y))
                    return new HoverProjectedBounds(
                        HoverBoundsKind.Unprunable, 0, 0, 0, 0);
                minX = Math.Min(minX, projection.Screen.X);
                minY = Math.Min(minY, projection.Screen.Y);
                maxX = Math.Max(maxX, projection.Screen.X);
                maxY = Math.Max(maxY, projection.Screen.Y);
            }
            if (maxDepth < 0.0)
                return new HoverProjectedBounds(
                    HoverBoundsKind.WhollyBehind, 0, 0, 0, 0);
            if (minDepth <= NearPlaneDepth)
                return new HoverProjectedBounds(
                    HoverBoundsKind.Unprunable, 0, 0, 0, 0);
            return new HoverProjectedBounds(
                HoverBoundsKind.Bounded,
                Math.BitDecrement(minX), Math.BitDecrement(minY),
                Math.BitIncrement(maxX), Math.BitIncrement(maxY));
        }
    }
}

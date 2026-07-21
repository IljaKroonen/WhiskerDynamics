using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Honest-density line pass: draws the UNPADDED dense
/// polyline straight through OrbitLinePass.AddLineVertices — the renderer's own
/// growable vertex append, no length limit — instead of squeezing through stock's
/// 2000-point Orbit.DrawLines (which stackallocs its whole strip, Orbit.cs:2182, so
/// it can never take a dense batch). Mirrors the exact subset of DrawLines semantics
/// the mod's call shapes reach: startTime is always SimTime.Zero (no per-point
/// re-anchor), joinEnds is always false (open arcs — no closing chord), payload
/// TrueAnomaly is always NaN (the burn-window color split never triggers), and the
/// batch times are linear (no bound-orbit wrap in the splice search). Kept from
/// stock: the DrawUI early-out, the IsVisible FOV/5-px cull (unless explicitly
/// bypassed for the active map vessel), the current-position
/// splice vertex at full opacity, and the index-based opacity fade with its 0.5
/// floor — anchored at "now" even for splice-less lines (stock computes the
/// insertion index and fades from it regardless of whether a current position was
/// passed, Orbit.cs:2102-2160/2218-2226). The staged 2000-point buffer remains the
/// payload surface for stock readers — this pass is DRAW only.</summary>
internal static class DenseLineDraw
{
    /// <summary>Per-call stack chunk: vertices stream to OrbitLinePass in slices, so
    /// the stack cost stays constant however dense the batch (stock's whole-strip
    /// stackalloc is exactly what caps IT at 2000).</summary>
    private const int ChunkSize = 1024;

    /// <summary>Screen-space emit target: one vertex per ~this many pixels of drawn
    /// arc. Skipped vertices cost two comparisons; a zoomed-out orbit collapses
    /// from the dense budget to a few hundred emitted vertices, sub-pixel exact.</summary>
    private const double TargetPixelsPerVertex = 3.0;

    /// <summary>Deviation budget (pixels) for the emit filter's shape term: any
    /// vertex whose DP significance exceeds this on-screen size is emitted, so the
    /// drawn polyline stays within ~a pixel of the dense path at EVERY zoom (the
    /// arc-rule extra vertices refine a DP(budget) simplification, worst ~2×budget)
    /// — the zoom dolly reshuffles the emitted subset every frame, and a
    /// sub-pixel-bounded reshuffle is what keeps the line from visibly crawling.</summary>
    private const double DeviationBudgetPixels = 0.4;

    /// <summary>The one emit path (splice vertex and samples share it): buffers into
    /// the caller's stack chunk and flushes full chunks — a ref struct because local
    /// functions cannot capture spans, and two hand-copied flush blocks would let a
    /// chunk-boundary bug ship on exactly one of the two vertex kinds.</summary>
    private ref struct ChunkEmitter(Viewport viewport, Span<float3> positions, Span<byte4> colors)
    {
        private readonly Viewport _viewport = viewport;
        private readonly Span<float3> _positions = positions;
        private readonly Span<byte4> _colors = colors;
        private int _filled;

        public void Emit(float3 position, byte4 color)
        {
            _positions[_filled] = position;
            _colors[_filled] = color;
            if (++_filled == ChunkSize) Flush();
        }

        public void Flush()
        {
            if (_filled == 0) return;
            OrbitLinePass.AddLineVertices(_viewport, _positions[.._filled], _colors[.._filled]);
            _filled = 0;
        }
    }

    /// <summary>Draws one dense polyline. <paramref name="ctx"/> supplies the same
    /// per-frame staging math as the pick buffer (frame re-embed at the now pose, SOI
    /// parent shift — one rule, <see cref="TrajectoryOverlay.StagingContext.Drawn"/>),
    /// so the drawn line and every stock-shaped reader agree within the frame.
    /// <paramref name="currentPositionEgo"/> NaN = no splice vertex (planned lines);
    /// the fade still anchors at the first sample past <paramref name="nowSeconds"/>.
    /// <paramref name="minimumTimeSeconds"/> clips a planned ghost's elapsed prefix
    /// without copying its immutable arrays. <paramref name="framePositions"/> is
    /// read exactly when <paramref name="ctx"/>.Framed (one sweep, one mode).</summary>
    internal static void Draw(Viewport viewport, Orbit orbit,
        double[] times, Vector3d[] positions, Vector3d[]? framePositions,
        DecimationMetrics metricsDrawn, DecimationMetrics metricsCce,
        in TrajectoryOverlay.StagingContext ctx, byte4 color,
        double3 currentPositionEgo, double nowSeconds, bool fadeOpacity = true,
        double minimumTimeSeconds = double.NegativeInfinity,
        bool bypassVisibilityCheck = false)
    {
        if (!Program.DrawUI) return; // stock's own early-out (Orbit.cs:2079): the sink
                                     // no-ops too, but the transform loop must not run
        int n = times.Length;
        if (n == 0) return;
        OverlayKernel.FutureClip? clip = double.IsFinite(minimumTimeSeconds)
            ? OverlayKernel.FutureClipAt(times, minimumTimeSeconds)
            : new OverlayKernel.FutureClip(0, 0, 0.0);
        if (clip is null) return;
        int first = clip.Value.Hi;
        bool boundary = clip.Value.Lo != clip.Value.Hi;
        int visibleCount = n - first + (boundary ? 1 : 0);
        var camera = viewport.GetCamera();
        if (!bypassVisibilityCheck && !orbit.IsVisible(camera)) return;
        double3 parentEgo = camera.GetPositionEgo(orbit.Parent);

        // Stock's "now" index: the first sample strictly past now (payloads are
        // linear — the wrap-around scan is unreachable for mod batches). It anchors
        // the fade unconditionally; the splice VERTEX additionally needs a current
        // position and an in-range index (stock: flag2 && i == num).
        int nowIndex = OverlayKernel.UpperBound(times, nowSeconds);
        if (nowIndex >= n) nowIndex = -1;
        bool splice = !currentPositionEgo.IsNaN();
        int spliceIndex = splice ? nowIndex : -1;
        // Stock's fade denominator counts the reserved splice slot even when the
        // search misses (num9 includes flag2 unconditionally, Orbit.cs:2170-2174).
        float invFadeCount = 1f / (visibleCount + (splice ? 1 : 0));

        // Screen-space decimation: pixels per radian
        // from the camera's own formula (Camera.cs:712-722, exact fractional form),
        // then a vertex emits only when the arc since the LAST emitted vertex spans
        // ~TargetPixelsPerVertex at that vertex's distance, or its DP significance
        // spans the deviation budget there (skipping it would visibly bend the
        // line). Arc lengths and significance were precomputed on the worker in
        // SAMPLED drawn space; the frame re-embed is a similarity (rotation + ONE
        // uniform scale), so a single chord calibrates them to drawn meters.
        double pxPerRadian = camera.GetObjectDiameterPixelsFrac(0.02, 1.0) / (2.0 * Math.Atan(0.01));
        if (!(pxPerRadian > 0) || !double.IsFinite(pxPerRadian)) pxPerRadian = 1000.0;
        bool framed = ctx.Framed;
        // The metrics must match the array the vertices come from: a framed BATCH
        // drawn through the pose-failure inertial fallback (ctx.Framed false) reads
        // meter-space Cce positions, so it measures against the Cce metrics —
        // frame-space values are separation-normalized and would starve the filter.
        double[] arcCum = (framed ? metricsDrawn : metricsCce).ArcCum;
        double[] significance = (framed ? metricsDrawn : metricsCce).Significance;
        int[] traversal = (framed ? metricsDrawn : metricsCce).TraversalIndices;
        if (traversal.Length == 0) return;
        double arcScale = 1.0;
        if (framed && n >= 2)
        {
            // Calibrate normalized frame arc -> drawn meters from the first
            // non-degenerate FINITE chord, probing outward from the middle over the
            // whole array: a duplicate-sample run or a NaN pose
            // pair straddling the middle must not leave the scale 8 orders of
            // magnitude off while real geometry exists elsewhere. A line with no
            // positive drawn chord at all IS a point — its endpoints are the whole
            // honest picture, so the default scale's starved thresholds fit.
            for (int step = 1; step < traversal.Length; step++)
            {
                int middle = traversal.Length / 2;
                int slot = middle + ((step & 1) == 1 ? step >> 1 : -(step >> 1));
                if (slot < 0 || slot >= traversal.Length) continue;
                int probe = traversal[slot];
                if (probe < 1 || probe >= n) continue;
                double sampledChord = arcCum[probe] - arcCum[probe - 1];
                if (!(sampledChord > 0)) continue;
                double drawnChord = (ctx.Drawn(default, framePositions![probe])
                    - ctx.Drawn(default, framePositions[probe - 1])).Length();
                if (!(drawnChord > 0) || !double.IsFinite(drawnChord)) continue;
                arcScale = drawnChord / sampledChord;
                break;
            }
        }

        // Inertial fast path: FrameAdapter.ToGame is linear (a pure repack), so the
        // constant parent shift hoists out of the loop entirely; framed vertices
        // re-embed per vertex (the honest per-frame cost of a rotating view).
        double3 inertialBase = framed
            ? default
            : parentEgo + FrameAdapter.ToGame(ctx.ParentShift);

        Span<float3> chunkPositions = stackalloc float3[ChunkSize];
        Span<byte4> chunkColors = stackalloc byte4[ChunkSize];
        var emitter = new ChunkEmitter(viewport, chunkPositions, chunkColors);
        double lastEmitArc = boundary
            ? arcCum[clip.Value.Lo] * (1.0 - clip.Value.Frac)
                + arcCum[clip.Value.Hi] * clip.Value.Frac
            : arcCum[first];
        // The emit rule, per interior vertex: emit when the arc since the last
        // emitted vertex spans ~TargetPixelsPerVertex on screen (density: fade
        // smoothness) OR the vertex's DP significance spans the deviation budget
        // (shape: skipping it would visibly bend the line). Thresholds are held in
        // SAMPLED units (divided by arcScale once per emit), so a skipped vertex
        // costs exactly two comparisons. Both anchor at the last emitted vertex's
        // camera distance; anchor drift between emits is bounded by the arc
        // threshold itself (a fraction of a percent of the anchor distance at any
        // real FOV) plus one dense step, so the sub-pixel guarantee holds through
        // camera dives too. The deviation budget rides the same anchor — one
        // constant apart.
        const double budgetPerThreshold = DeviationBudgetPixels / TargetPixelsPerVertex;
        double thresholdSampled = TargetPixelsPerVertex * parentEgo.Length() / (pxPerRadian * arcScale);
        double budgetSampled = thresholdSampled * budgetPerThreshold;
        if (boundary)
        {
            int lo = clip.Value.Lo, hi = clip.Value.Hi;
            double frac = clip.Value.Frac;
            double3 world;
            if (framed)
            {
                var coordinate = framePositions![lo] * (1.0 - frac) + framePositions[hi] * frac;
                world = parentEgo + FrameAdapter.ToGame(ctx.Drawn(default, coordinate));
            }
            else
            {
                var position = positions[lo] * (1.0 - frac) + positions[hi] * frac;
                world = inertialBase + FrameAdapter.ToGame(position);
            }
            var boundaryColor = color;
            boundaryColor.A = byte.MaxValue;
            emitter.Emit(float3.Pack(in world), boundaryColor);
            thresholdSampled = TargetPixelsPerVertex * world.Length() / (pxPerRadian * arcScale);
            budgetSampled = thresholdSampled * budgetPerThreshold;
        }
        int traversalStart = OverlayKernel.TraversalSuffixStart(
            traversal, first, boundary, out bool explicitFirst);
        bool spliceEmitted = false;
        if (explicitFirst)
        {
            if (spliceIndex == first)
            {
                var spliceColor = color;
                spliceColor.A = byte.MaxValue;
                emitter.Emit(float3.Pack(in currentPositionEgo), spliceColor);
                spliceEmitted = true;
            }
            double3 world = framed
                ? parentEgo + FrameAdapter.ToGame(ctx.Drawn(default, framePositions![first]))
                : inertialBase + FrameAdapter.ToGame(positions[first]);
            var vertexColor = StyledColor(
                color, first, nowIndex, invFadeCount, fadeOpacity);
            emitter.Emit(float3.Pack(in world), vertexColor);
            lastEmitArc = arcCum[first];
            thresholdSampled = TargetPixelsPerVertex * world.Length() / (pxPerRadian * arcScale);
            budgetSampled = thresholdSampled * budgetPerThreshold;
        }
        for (int slot = traversalStart; slot < traversal.Length; slot++)
        {
            int i = traversal[slot];
            if (!spliceEmitted && spliceIndex >= first && i >= spliceIndex)
            {
                var spliceColor = color;
                spliceColor.A = byte.MaxValue;
                emitter.Emit(float3.Pack(in currentPositionEgo), spliceColor);
                spliceEmitted = true;
            }
            if (i != first && i != n - 1
                && arcCum[i] - lastEmitArc < thresholdSampled
                && significance[i] < budgetSampled)
                continue;
            double3 world = framed
                ? parentEgo + FrameAdapter.ToGame(ctx.Drawn(default, framePositions![i]))
                : inertialBase + FrameAdapter.ToGame(positions[i]);
            var vertexColor = StyledColor(
                color, i, nowIndex, invFadeCount, fadeOpacity);
            emitter.Emit(float3.Pack(in world), vertexColor);
            lastEmitArc = arcCum[i];
            // Distance-adaptive thresholds: re-anchored at each emitted vertex, so
            // an eccentric orbit densifies near the camera and thins at apoapsis.
            thresholdSampled = TargetPixelsPerVertex * world.Length() / (pxPerRadian * arcScale);
            budgetSampled = thresholdSampled * budgetPerThreshold;
        }
        emitter.Flush();
        OrbitLinePass.AddLineEnd(viewport);
    }

    internal static byte4 StyledColor(byte4 color, int sampleIndex, int nowIndex,
        float invFadeCount, bool fadeOpacity)
    {
        if (!fadeOpacity) return color;
        float fade = 1f - (sampleIndex - nowIndex) * invFadeCount;
        if (sampleIndex < nowIndex) fade -= 1f;
        color.A = (byte)(Math.Clamp(fade, 0.5f, 1f) * 255f);
        return color;
    }
}

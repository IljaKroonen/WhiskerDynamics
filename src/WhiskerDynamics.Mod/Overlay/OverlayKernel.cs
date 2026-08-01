using Brutal.Numerics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

/// <summary>The overlay's KSA-free kernels (offline-testable; <see cref="TrajectoryOverlay"/>
/// translates game state to these inputs): the burn window rule, folding planned burns
/// into a display <see cref="TrajectoryPredictor"/> as impulses with the duplicate-time
/// guard, and the stock-length padding invariant. (Sample-time placement lives in
/// <see cref="AdaptiveSampler"/> — honest orbit lines use an adaptive sweep, not a uniform grid.)
/// Also here: the celestial sampling-window clamp
/// (<see cref="CelestialWindowSeconds"/>), the three-way draw/blink/stock line
/// routing rule (<see cref="RouteLine"/>, <see cref="LineRoute"/>), and the live celestial
/// catalog's deterministic curve-body priority/cap (<see cref="CurvePriority"/>).</summary>
public static class OverlayKernel
{
    /// <summary>Restage window: samples older than this (wall clock) mean sampling
    /// stopped — the vessel was contained, switched, or the mod disabled — and a fossil
    /// polyline must yield to the stock line. Future-stamped samples (cross-session
    /// residue) are equally unusable.</summary>
    public const long RestageMaxAgeMs = 5_000;

    public static bool SamplesUsable(long sampleWallMs, long nowWallMs) =>
        nowWallMs >= sampleWallMs && nowWallMs - sampleWallMs <= RestageMaxAgeMs;

    /// <summary>Maximum age of continuously reusable actual geometry before the
    /// worker performs a real resample. This recenters adaptive density around now
    /// and refreshes horizon/event work; it is not a display-freshness cutoff.</summary>
    public const double ActualGeometryRecenteringSeconds = 600.0;

    /// <summary>Whether a captured simulation epoch is well-ordered against the
    /// current epoch. This is the worker publication gate: elapsed simulation time
    /// alone cannot obsolete an absolute trajectory while a high-warp rebuild is in
    /// flight. Session, predictor-lineage, and queue-ticket gates provide the actual
    /// publication identity.</summary>
    public static bool CaptureEpochValid(double captureSimSeconds, double nowSimSeconds) =>
        double.IsFinite(captureSimSeconds)
        && double.IsFinite(nowSimSeconds)
        && nowSimSeconds >= captureSimSeconds;

    /// <summary>Whether an immutable line batch still contains geometry at the
    /// current simulation epoch. Absolute sample times make this the honest display
    /// lifetime: old prefixes are clipped/spliced by the draw path, while a batch
    /// whose sampled future has been exhausted must fall back.</summary>
    public static bool LineGeometryCovers(OverlaySamples samples, double nowSimSeconds) =>
        CaptureEpochValid(samples.CaptureSimSeconds, nowSimSeconds)
        && samples.DenseTimes.Length > 0
        && double.IsFinite(samples.DenseTimes[^1])
        && samples.DenseTimes[^1] > nowSimSeconds;

    /// <summary>Whether a complete batch remains live and covers the current
    /// simulation epoch. Absolute trajectory age is deliberately not a cutoff:
    /// generation/lineage gates protect identity, while the wall stamp detects a
    /// stopped producer and the dense endpoint bounds physical coverage.</summary>
    public static bool SamplesUsable(
        OverlaySamples samples, long nowWallMs, double nowSimSeconds) =>
        SamplesUsable(samples.SampleWallMs, nowWallMs)
        && LineGeometryCovers(samples, nowSimSeconds);

    /// <summary>Whether actual-line frame coordinates depend only on the subject
    /// vessel's continuous coast lineage. Surface collision cuts must resweep; a
    /// target-fixed batch also depends on another mutable predictor and must resweep.</summary>
    public static bool FrameAllowsGeometryReuse(FrameSpec? activeFrame) =>
        activeFrame is not { Kind: FrameKind.Surface or FrameKind.TargetFixed };

    /// <summary>Whether a planned batch may be restamped without resampling. Unlike a
    /// catalog frame, target-fixed coordinates depend on another vessel's mutable
    /// predictor, which can change while every plan/restamp identity input stays equal.</summary>
    public static bool FrameAllowsPlannedRestamp(FrameSpec? activeFrame) =>
        activeFrame is not { Kind: FrameKind.TargetFixed };

    /// <summary>Whether frame-coordinate Z is the natural AN/DN reference plane. Both
    /// fixed kinds use FrameKernel.Rotating; body/surface inertial views use the
    /// ecliptic-parallel plane through their primary instead.</summary>
    public static bool FrameCoordinatesDefineNodePlane(FrameSpec? activeFrame) =>
        activeFrame is { Kind: FrameKind.TwoBodyFixed or FrameKind.TargetFixed };

    // (There is deliberately no per-frame restage de-dup rule:
    // VesselLinePatch stages at the draw site, where a duplicate stage is one
    // idempotent pooled rebuild — cheaper than the proof a de-dup would need.)

    /// <summary>Stock's invariant point-buffer length (UpdateTaskUtils.GenerateSpacedPoints
    /// allocates 2000 unconditionally — NUM_SPACED_POINTS_MAX). Every buffer the mod stages
    /// is padded to exactly this length, which (a) closes the residual length-assuming
    /// reader windows (Orbit.DrawLines two-field read, Orbit.cs:2092-2093;
    /// GroundTrackWindow per-iteration re-reads, GroundTrackWindow.cs:424-455) and
    /// (b) makes an e&gt;=1 write gate unnecessary: the NearestOrbitPointJob worker
    /// (Orbit.cs:2308) can never capture an index against one length and dereference
    /// another, so escape legs may be staged too.</summary>
    public const int StockPointBufferLength = 2000;

    /// <summary>Honest-density split: endpoint-preserving uniform
    /// index decimation of a DENSE adaptive sweep down to the stock buffer budget —
    /// the staged 2000-point buffer stays a strict SUBSET of the drawn dense polyline,
    /// so every stock reader (hover job, click-to-place payloads, ground track) keeps
    /// its exact fidelity while the drawn line goes dense. Identity when the sweep
    /// already fits; indices are strictly increasing and always include both
    /// endpoints.</summary>
    public static int[] DecimateIndices(int count, int maxPoints)
    {
        if (count <= 0) return [];
        if (maxPoints < 2) throw new ArgumentOutOfRangeException(nameof(maxPoints));
        if (count <= maxPoints)
        {
            var identity = new int[count];
            for (int i = 0; i < count; i++) identity[i] = i;
            return identity;
        }
        var indices = new int[maxPoints];
        double step = (count - 1) / (double)(maxPoints - 1);
        for (int k = 0; k < maxPoints; k++) indices[k] = (int)Math.Round(k * step);
        indices[^1] = count - 1; // rounding must never drop the horizon endpoint
        return indices;
    }

    /// <summary>Bounded render/input traversal combining even arc-index coverage
    /// with the most shape-significant Douglas-Peucker vertices. Uniform-only
    /// thinning can drop a narrow periapsis or corner between its sample slots;
    /// reserving half the budget for significance preserves those features while
    /// the coverage half bounds long nearly-straight gaps.</summary>
    public static int[] BoundedTraversalIndices(double[] significance, int maxPoints)
    {
        int count = significance.Length;
        if (count <= maxPoints) return DecimateIndices(count, Math.Max(2, maxPoints));
        if (maxPoints < 4) throw new ArgumentOutOfRangeException(nameof(maxPoints));
        int coverageSlots = maxPoints / 2;
        int shapeSlots = maxPoints - coverageSlots;
        var selected = new HashSet<int>(DecimateIndices(count, coverageSlots));
        var strongest = new PriorityQueue<int, double>(shapeSlots + 1);
        for (int i = 1; i < count - 1; i++)
        {
            double priority = significance[i];
            if (!double.IsFinite(priority)) priority = double.MaxValue;
            strongest.Enqueue(i, priority);
            if (strongest.Count > shapeSlots) strongest.Dequeue();
        }
        while (strongest.TryDequeue(out int index, out _)) selected.Add(index);
        var result = selected.ToArray();
        Array.Sort(result);
        return result;
    }

    public static int TraversalSuffixStart(int[] traversal, int first,
        bool hasInterpolatedBoundary, out bool injectFirst)
    {
        int start = Array.BinarySearch(traversal, first);
        if (start < 0) start = ~start;
        injectFirst = !hasInterpolatedBoundary
            && (start >= traversal.Length || traversal[start] != first);
        return start;
    }

    /// <summary>The decimation subset copy — ONE implementation for every producer
    /// (vessel batches AND celestial curves), so the "staged buffer is a strict
    /// subset of the drawn dense polyline" invariant cannot fork between them.</summary>
    public static T[] TakeAt<T>(T[] source, int[] indices)
    {
        var subset = new T[indices.Length];
        for (int k = 0; k < indices.Length; k++) subset[k] = source[indices[k]];
        return subset;
    }

    /// <summary>Cumulative chord length of a polyline in ITS OWN coordinate space
    /// (drawn-space for batches: frame coordinates when framed, parent-relative Cce
    /// otherwise — the same space the turn bound ran in). Computed ONCE per batch on
    /// the overlay worker; the per-frame screen-space emit filter then works with
    /// subtractions only. [0] = 0; length matches the input.</summary>
    public static double[] CumulativeArcLengths(Vector3d[] points)
    {
        var cumulative = new double[points.Length];
        for (int i = 1; i < points.Length; i++)
            cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).Length();
        return cumulative;
    }

    /// <summary>Per-vertex Douglas-Peucker significance of a polyline, in the
    /// polyline's own coordinate space: the deviation tolerance at which the vertex
    /// is dropped by a DP simplification. Monotone along the DP tree (a vertex's
    /// significance never exceeds its split ancestors'), so thresholding at ANY
    /// tolerance t yields exactly the DP(t) simplification — every dense point stays
    /// within t of the polyline through the retained vertices. Endpoints are
    /// PositiveInfinity (always retained). Computed once per batch on the worker
    /// (zoom-independent); the per-frame emit filter only compares. Cost is
    /// O(n·depth) — near-balanced splits on smooth orbital sweeps keep it ~n·log n,
    /// and zero-deviation spans terminate outright (below).</summary>
    public static double[] ChordSignificance(Vector3d[] points)
    {
        int n = points.Length;
        var significance = new double[n];
        if (n == 0) return significance;
        significance[0] = double.PositiveInfinity;
        significance[n - 1] = double.PositiveInfinity;
        if (n <= 2) return significance;
        var spans = new Stack<(int Lo, int Hi, double Cap)>();
        spans.Push((0, n - 1, double.PositiveInfinity));
        while (spans.Count > 0)
        {
            var (lo, hi, cap) = spans.Pop();
            int worst = -1;
            double worstSq = 0.0;
            Vector3d a = points[lo];
            Vector3d ab = points[hi] - a;
            double lengthSq = ab.LengthSquared();
            for (int i = lo + 1; i < hi; i++)
            {
                double devSq = PointSegmentDistanceSquared(points[i], a, ab, lengthSq);
                if (devSq > worstSq) { worstSq = devSq; worst = i; }
            }
            // No positive deviation: every interior point sits ON the chord
            // (duplicate/collinear runs) — leaving them at 0 IS the exact answer,
            // and stopping here is the guard against the one-vertex-per-pass
            // O(n^2) degeneracy those runs would otherwise drive. Non-finite
            // deviations (NaN samples) land here too: never selected as worst,
            // so a NaN can neither become a significance nor poison a child cap.
            if (worst < 0) continue;
            double clamped = Math.Min(Math.Sqrt(worstSq), cap); // the monotone clamp IS the exactness
            significance[worst] = clamped;
            if (worst - lo >= 2) spans.Push((lo, worst, clamped));
            if (hi - worst >= 2) spans.Push((worst, hi, clamped));
        }
        return significance;
    }

    /// <summary>Squared distance from a point to the SEGMENT [a, b] (clamped
    /// projection — a degenerate zero-length chord measures to the point). Squared
    /// because the DP scan only needs the argmax: one sqrt per span, not per point.</summary>
    private static double PointSegmentDistanceSquared(
        Vector3d p, Vector3d a, Vector3d ab, double lengthSq)
    {
        if (lengthSq <= 0) return (p - a).LengthSquared();
        double t = Math.Clamp((p - a).Dot(ab) / lengthSq, 0.0, 1.0);
        return (p - (a + ab * t)).LengthSquared();
    }

    /// <summary>First index with times[i] STRICTLY past <paramref name="value"/>
    /// (times.Length when none) — stock's current-position splice rule for the
    /// dense draw: a sample exactly at "now" draws behind the splice vertex, the
    /// next one ahead of it. Public so the parity-critical strictness pins offline.</summary>
    public static int UpperBound(double[] times, double value)
    {
        int lo = 0, hi = times.Length;
        while (lo < hi)
        {
            int mid = (lo + hi) / 2;
            if (times[mid] <= value) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    /// <summary>Pads by repeating the last element; identity when already stock-length.
    /// Degenerate repeated points draw zero-length segments (invisible) and pick as the
    /// arc's endpoint — harmless by construction.</summary>
    public static T[] PadToStockLength<T>(T[] source)
    {
        if (source.Length == 0 || source.Length > StockPointBufferLength)
            throw new ArgumentOutOfRangeException(nameof(source), $"length {source.Length} not in [1, {StockPointBufferLength}]");
        if (source.Length == StockPointBufferLength) return source;
        var padded = new T[StockPointBufferLength];
        source.CopyTo(padded, 0);
        for (int i = source.Length; i < StockPointBufferLength; i++) padded[i] = source[^1];
        return padded;
    }

    /// <summary>Honest arcs: celestial sampling window (seconds) = config days clamped to
    /// the rails-ahead horizon (never demand ephemerides the worker doesn't maintain).
    /// 0.1 d floor guards a zeroed config. Deliberately NOT clamped to one orbital
    /// period: under n-body dynamics successive revolutions don't retrace,
    /// so a multi-period window shows real precession instead of overdraw; the orbital
    /// period still feeds AdaptiveSampler as the anti-aliasing step hint, and the sampler's
    /// point budget (the dense celestial_max_points) bounds how
    /// many revolutions a long window can actually draw.</summary>
    public static double CelestialWindowSeconds(double configDays, double railsAheadDays)
    {
        double window = Math.Max(configDays, 0.1) * 86400.0;
        return Math.Min(window, railsAheadDays * 86400.0);
    }

    /// <summary>Quarter-day quantization step for the overlay's rails-availability
    /// input — see <see cref="QuantizeRailsWindow"/>.</summary>
    public const double RailsWindowQuantumDays = 0.25;

    /// <summary>Stabilizes the rails-availability clamp for the overlay's horizon
    /// inputs: the raw reached-horizon distance jitters by one worker cycle's worth of
    /// sim time every read (scaled by warp), and it feeds overlay horizons plus the
    /// planned batch's bounded high-water growth rule. Floored to quarter-day steps:
    /// stable between worker cycles at normal
    /// rates, one honest step per growth chunk during a catch-up, and never past what
    /// the rails actually reached. Steady state draws up to a quantum short of the
    /// config target (the worker's horizon always trails "now + target" by the sim
    /// time since its last cycle) — sub-1% of the default window, invisible.</summary>
    public static double QuantizeRailsWindow(double availableDays, double configDays)
    {
        if (availableDays >= configDays) return configDays;
        return Math.Floor(availableDays / RailsWindowQuantumDays) * RailsWindowQuantumDays;
    }

    /// <summary>THE derivation of the fixed sampling bound from config, in radians.
    /// Vessel and celestial samplers share this value for every segment. Non-finite
    /// config values fall back
    /// to the shipped defaults rather than poisoning the sampler (a NaN bound
    /// makes every grow/halve comparison false, pinning the adaptive step at
    /// dtMin — the budget burns on a sliver of the window at maximum cost).</summary>
    public static double SamplingThetaRadians(double maxTurnDeg)
    {
        if (!double.IsFinite(maxTurnDeg)) maxTurnDeg = ModConfig.DefaultOverlayTurnDegrees;
        double thetaMaxDeg = Math.Clamp(maxTurnDeg,
            ModConfig.MinOverlayTurnDegrees, ModConfig.MaxOverlayTurnDegrees);
        return thetaMaxDeg * Math.PI / 180.0;
    }

    /// <summary>Orbit-line-only finite-fold work cap. The configured value remains
    /// available at up to 1024 slices to optimizer/solver physics; drawing limits its
    /// private fold to 128 without mutating the shared objective configuration.</summary>
    internal static int OverlayFiniteBurnMaxSlices(int configured) =>
        Math.Clamp(configured, ModConfig.MinFiniteBurnMaxSlices,
            ModConfig.MaxOverlayFiniteBurnSlices);

    /// <summary>THE frame-mode identity every drawn-or-interactive surface shares: a
    /// batch belongs to the current display mode iff its frame label equals the active
    /// frame's label ordinally (null = inertial on both sides). One definition — the
    /// main line's RouteLine input, the planned line, the hover substitute, and the
    /// burn-node interpolation must all blink TOGETHER through a frame switch, or a
    /// wrong-frame geometry flashes on exactly one of them.</summary>
    public static bool ModeMatches(string? batchFrameLabel, string? activeFrameLabel) =>
        string.Equals(batchFrameLabel, activeFrameLabel, StringComparison.Ordinal);

    /// <summary>Screen-space nearest point ON a polyline for the hover/drag substitute
    /// (OrbitHoverPatch): Manhattan scan for the nearest finite vertex (stock
    /// GetNearestPointIndex parity — NaN vertices are behind the camera and skipped),
    /// then Euclidean projection onto the two adjacent segments, so hover stays
    /// CONTINUOUS along the drawn chords even when samples span many pixels (a
    /// vertex-only scan would leave dead zones between
    /// coarse samples and snap burn placement to discrete sample times).
    /// <paramref name="frac"/> is the position along [Lo, Hi] — payload times lerp
    /// linearly along it, exact for the drawn chord. False when no finite vertex
    /// exists (whole line behind the camera).</summary>
    public static bool PolylineNearest(ReadOnlySpan<float2> screen, float2 mouse,
        out int lo, out int hi, out double frac, out float2 projected)
    {
        lo = hi = -1;
        frac = 0.0;
        projected = default;
        float best = float.MaxValue;
        for (int i = 0; i < screen.Length; i++)
        {
            var p = screen[i];
            if (float.IsNaN(p.X) || float.IsNaN(p.Y)) continue;
            float d = Math.Abs(mouse.X - p.X) + Math.Abs(mouse.Y - p.Y);
            if (d < best) { best = d; lo = hi = i; }
        }
        if (lo < 0) return false;
        projected = screen[lo];
        double bestSq = SquaredDistance(projected, mouse);
        int vertex = lo;
        ConsiderSegment(screen, vertex - 1, vertex, mouse, ref lo, ref hi, ref frac, ref projected, ref bestSq);
        ConsiderSegment(screen, vertex, vertex + 1, mouse, ref lo, ref hi, ref frac, ref projected, ref bestSq);
        return true;
    }

    private static void ConsiderSegment(ReadOnlySpan<float2> screen, int a, int b, float2 mouse,
        ref int lo, ref int hi, ref double frac, ref float2 projected, ref double bestSq)
    {
        if (a < 0 || b >= screen.Length) return;
        if (!HoverHitTestKernel.TryCloserSegment(
                new HoverScreenPoint(screen[a].X, screen[a].Y),
                new HoverScreenPoint(screen[b].X, screen[b].Y),
                new HoverScreenPoint(mouse.X, mouse.Y), bestSq,
                out double t, out HoverScreenPoint q, out double dSq))
            return;
        bestSq = dSq;
        lo = a;
        hi = b;
        frac = t;
        projected = new float2(q.X, q.Y);
    }

    /// <summary>Interpolates only after both endpoints have been staged, preserving
    /// the rendered clip-boundary arithmetic order.</summary>
    internal static Vector3d ResolveHoverPosition(HoverPointRef point,
        Vector3d stagedSource, Vector3d stagedOther) =>
        point.IsInterpolated
            ? stagedSource * (1.0 - point.SourceFraction)
                + stagedOther * point.SourceFraction
            : stagedSource;

    /// <summary>Resolves a hover payload time. A synthetic clip boundary carries the
    /// exact requested time rather than reconstructing it from a rounded fraction.</summary>
    internal static double ResolveHoverTime(
        HoverPointRef point, ReadOnlySpan<double> times)
    {
        if (point.HasExactTime) return point.ExactTime;
        double sourceTime = times[point.SourceIndex];
        return point.IsInterpolated
            ? sourceTime * (1.0 - point.SourceFraction)
                + times[point.OtherSourceIndex] * point.SourceFraction
            : sourceTime;
    }

    private static double SquaredDistance(float2 a, float2 b)
    {
        double dx = a.X - b.X, dy = a.Y - b.Y;
        return dx * dx + dy * dy;
    }

    /// <summary>Honest line markers: index of the FIRST interior local extremum of
    /// <paramref name="values"/> over the real prefix [0, count) — the first upcoming
    /// apoapsis/periapsis of a sampled distance series. Plateau-robust: a flat run
    /// counts once, at its first sample, and only when the series actually turns
    /// (endpoints never qualify — a monotone approach has no extremum yet). -1 when
    /// none exists in the window.</summary>
    public static int FirstLocalExtremum(double[] values, int count, bool findMinimum)
    {
        foreach (int index in LocalExtrema(values, count, findMinimum))
            return index;
        return -1;
    }

    /// <summary>Every interior local extremum in chronological sample order, with the
    /// same plateau semantics as <see cref="FirstLocalExtremum"/>. Closest-approach
    /// markers use all minima; Ap/Pe intentionally consume only the first.</summary>
    public static IEnumerable<int> LocalExtrema(double[] values, int count, bool findMinimum)
    {
        double sign = findMinimum ? 1.0 : -1.0;
        for (int i = 1; i < count - 1; i++)
        {
            if (sign * values[i] >= sign * values[i - 1]) continue; // not descending into i (in the min sense)
            int plateauEnd = i;
            while (plateauEnd + 1 < count && values[plateauEnd + 1] == values[i]) plateauEnd++;
            if (plateauEnd + 1 < count && sign * values[plateauEnd + 1] > sign * values[i])
                yield return i;
            i = plateauEnd;
        }
    }

    /// <summary>Relative speed at a sampled trajectory event, using the centered
    /// neighboring-sample tangent of the relative-position curve. Extrema are
    /// interior, so both neighbors normally exist; endpoint fallback keeps the
    /// helper defensive.</summary>
    public static double RelativeSpeedAt(
        double[] times, Vector3d[] relativePositions, int index, int count,
        int timesOffset = 0)
    {
        if (count < 2 || index < 0 || index >= count
            || timesOffset < 0 || timesOffset + count > times.Length
            || count > relativePositions.Length)
            return double.NaN;
        int lo = Math.Max(0, index - 1);
        int hi = Math.Min(count - 1, index + 1);
        double dt = times[timesOffset + hi] - times[timesOffset + lo];
        return dt > 0 && double.IsFinite(dt)
            ? (relativePositions[hi] - relativePositions[lo]).Length() / dt
            : double.NaN;
    }

    /// <summary>Relative speed on one sampled segment, used for interpolated plane
    /// and SOI crossings whose event lies between accepted samples.</summary>
    public static double RelativeSpeedAcross(double[] times, Vector3d[] relativePositions,
        int lo, int count, int timesOffset = 0)
    {
        if (count < 2 || lo < 0 || lo + 1 >= count
            || timesOffset < 0 || timesOffset + count > times.Length
            || count > relativePositions.Length)
            return double.NaN;
        double dt = times[timesOffset + lo + 1] - times[timesOffset + lo];
        return dt > 0 && double.IsFinite(dt)
            ? (relativePositions[lo + 1] - relativePositions[lo]).Length() / dt
            : double.NaN;
    }

    public static string TrajectoryMarkerLabel(
        string label, double secondsUntil, double? relativeSpeed)
    {
        string duration = TimeDisplayKernel.FormatCountdown(Math.Max(0, secondsUntil),
            years: secondsUntil >= 365.0 * 86400.0);
        double speedValue = relativeSpeed ?? double.NaN;
        string speed = double.IsFinite(speedValue)
            ? speedValue >= 1000.0
                ? $"{speedValue / 1000.0:N2} km/s"
                : $"{speedValue:N0} m/s"
            : "? m/s";
        return $"{label} | T-{duration} | {speed} rel";
    }

    public static string ClosestApproachLabel(
        string targetId, double distanceMeters, double secondsUntil, double relativeSpeed)
        => TrajectoryMarkerLabel($"CA {targetId} {distanceMeters / 1000.0:N0} km",
            secondsUntil, relativeSpeed);

    public static string ImpactLabel(
        double impactTimeSeconds, double t0Seconds, double? impactSpeed)
    {
        double secondsUntil = impactTimeSeconds - t0Seconds;
        string duration = TimeDisplayKernel.FormatCountdown(Math.Max(0, secondsUntil),
            years: secondsUntil >= 365.0 * 86400.0);
        string speed = impactSpeed is { } value && double.IsFinite(value)
            ? $" | {value:N0} m/s"
            : string.Empty;
        return $"Impact | T-{duration}{speed}";
    }

    /// <summary>Refreshes the time-dependent part of a cached marker label at the
    /// supplied display epoch. Geometry-derived values remain immutable; countdowns
    /// must not freeze merely because a covered batch is retained during warp.</summary>
    public static string MarkerLabelAt(OverlayMarker marker, double nowSeconds) =>
        marker.Kind switch
        {
            OverlayMarkerKind.Collision => ImpactLabel(
                marker.TimeSeconds, nowSeconds, marker.ImpactSpeedMetersPerSecond),
            OverlayMarkerKind.ClosestApproach => ClosestApproachLabel(
                marker.BodyId, marker.AltitudeMeters,
                marker.TimeSeconds - nowSeconds,
                marker.RelativeSpeedMetersPerSecond ?? double.NaN),
            _ => TrajectoryMarkerLabel(marker.Label,
                marker.TimeSeconds - nowSeconds, marker.RelativeSpeedMetersPerSecond),
        };

    /// <summary>Signed distance from a body-centred position to the body's equatorial
    /// plane. The normal is the bind-captured unit spin pole in game-ecliptic axes;
    /// keeping this pure makes the worker-side marker calculation KSA-free.</summary>
    public static double EquatorialPlaneOffset(Vector3d relativePosition, Vector3d spinPole) =>
        relativePosition.Dot(spinPole);

    /// <summary>Honest line markers: the FIRST sign crossing of <paramref name="values"/>
    /// over the real prefix — an ascending (− to +, when <paramref name="ascending"/>)
    /// or descending node of a sampled plane-offset series. Returns the bracketing
    /// sample and the linear fraction across it (node times interpolate between
    /// samples). A sample exactly ON the plane counts as the crossing's far end.
    /// Null when the series never crosses in the window.</summary>
    public static (int Lo, double Frac)? FirstSignCrossing(double[] values, int count, bool ascending)
    {
        foreach (var crossing in SignCrossings(values, count, ascending)) return crossing;
        return null;
    }

    /// <summary>Every ascending or descending sign crossing in chronological order,
    /// with the same strict/loose zero semantics as FirstSignCrossing.</summary>
    public static IEnumerable<(int Lo, double Frac)> SignCrossings(
        double[] values, int count, bool ascending)
    {
        double sign = ascending ? 1.0 : -1.0;
        for (int i = 1; i < count; i++)
        {
            double previous = sign * values[i - 1];
            double current = sign * values[i];
            if (previous < 0 && current >= 0)
                yield return (i - 1, -previous / (current - previous));
        }
    }

    /// <summary>First sampled crossing of an SOI boundary. Entering requires an
    /// outside-to-inside crossing; escape requires inside-to-outside. The returned
    /// fraction linearly interpolates distance across the bracketing samples.</summary>
    public static (int Lo, double Frac)? FirstSoiCrossing(
        double[] distances, int count, double soiRadius, bool entering,
        int startIndex = 0, double afterPosition = double.NegativeInfinity)
    {
        if (!(soiRadius > 0) || !double.IsFinite(soiRadius)) return null;
        for (int i = Math.Max(1, startIndex + 1); i < count; i++)
        {
            double previous = distances[i - 1] - soiRadius;
            double current = distances[i] - soiRadius;
            bool crossed = entering
                ? previous > 0 && current <= 0
                : previous < 0 && current >= 0;
            if (crossed)
            {
                double frac = previous / (previous - current);
                if (i - 1 + frac > afterPosition)
                    return (i - 1, frac);
            }
        }
        return null;
    }

    /// <summary>Marker scans are rendered only for the controlled vessel. Keeping this
    /// rule pure lets the worker skip all Ap/Pe/node/SOI/target work for other tracked
    /// vessels while their lines continue to sample and draw normally.</summary>
    public static bool MarkerWorkEnabled(string? controlledVesselId, string vesselId) =>
        controlledVesselId is not null
        && string.Equals(controlledVesselId, vesselId, StringComparison.Ordinal);

    public readonly record struct SoiTransition(
        bool Escape, string BodyId, int Lo, double Frac);

    /// <summary>Chronological sampled SOI parent chain. At each parent, the earliest
    /// event wins between escape upward and encounters with direct children; the scan
    /// then continues under the resulting parent, producing encounter→escape flybys
    /// and nested encounters without re-consuming an earlier crossing.</summary>
    public static IReadOnlyList<SoiTransition> FindSoiTransitions(
        string initialParent, int count,
        Func<string, string?> parentOf,
        Func<string, IReadOnlyList<string>> childrenOf,
        Func<string, double> radiusOf,
        Func<string, double[]> distancesTo,
        int maxTransitions = 64)
    {
        var result = new List<SoiTransition>();
        if (count < 2 || maxTransitions <= 0) return result;
        string currentParent = initialParent;
        double cursor = double.NegativeInfinity;
        for (int transition = 0;
            transition < maxTransitions && cursor < count - 1;
            transition++)
        {
            // Resolve children first: production uses this callback to fold the
            // current parent plus every child into one batched rails read per time.
            var children = childrenOf(currentParent);
            SoiTransition? next = null;
            void Consider(bool escape, string body, (int Lo, double Frac)? crossing)
            {
                if (crossing is not { } c) return;
                var candidate = new SoiTransition(escape, body, c.Lo, c.Frac);
                if (next is null || c.Lo + c.Frac < next.Value.Lo + next.Value.Frac)
                    next = candidate;
            }

            if (parentOf(currentParent) is not null)
                Consider(true, currentParent, FirstSoiCrossing(
                    distancesTo(currentParent), count, radiusOf(currentParent),
                    entering: false,
                    startIndex: double.IsFinite(cursor) ? Math.Max(0, (int)Math.Floor(cursor)) : 0,
                    afterPosition: cursor));
            foreach (string child in children)
                Consider(false, child, FirstSoiCrossing(
                    distancesTo(child), count, radiusOf(child), entering: true,
                    startIndex: double.IsFinite(cursor) ? Math.Max(0, (int)Math.Floor(cursor)) : 0,
                    afterPosition: cursor));

            if (next is not { } found) break;
            result.Add(found);
            currentParent = found.Escape ? parentOf(found.BodyId)! : found.BodyId;
            // Fractional cursor: a nested boundary later in the same coarse segment
            // remains eligible, while an earlier/equal crossing cannot move the event
            // chain backwards or spin inside the 64-event guard.
            cursor = found.Lo + found.Frac;
        }
        return result;
    }

    /// <summary>Planned restamps keep collision markers (the line endpoint/cut reuse
    /// guard) and replace every trajectory-derived marker with a freshly computed set.
    /// This makes target changes/removal and moving target batches visible without
    /// resampling deterministic planned geometry.</summary>
    public static IReadOnlyList<OverlayMarker> RestampedMarkers(
        IReadOnlyList<OverlayMarker> previous,
        IReadOnlyList<OverlayMarker> refreshed,
        double t0Seconds)
    {
        var result = new List<OverlayMarker>(previous.Count + refreshed.Count);
        foreach (var marker in previous)
            if (marker.Kind == OverlayMarkerKind.Collision)
                result.Add(marker with
                {
                    Label = MarkerLabelAt(marker, t0Seconds),
                });
        result.AddRange(refreshed);
        return result;
    }

    /// <summary>Projects cached marker candidates to what is visible at now. Ap/Pe and
    /// nodes expose only the first upcoming event per body/kind; SOI and closest-
    /// approach events keep every future candidate. Collision/CA labels are refreshed
    /// for non-render consumers; other labels stay as bases for the render-time live
    /// countdown, without revisiting geometry or the shared rails ephemeris.</summary>
    public static IReadOnlyList<OverlayMarker> VisibleMarkers(
        IReadOnlyList<OverlayMarker> candidates, double t0Seconds)
    {
        var result = new List<OverlayMarker>(candidates.Count);
        var first = new Dictionary<(OverlayMarkerKind Kind, string Body), OverlayMarker>();
        foreach (var marker in candidates)
        {
            if (marker.TimeSeconds < t0Seconds) continue;
            if (marker.Kind == OverlayMarkerKind.Collision)
            {
                result.Add(marker with
                {
                    Label = MarkerLabelAt(marker, t0Seconds),
                });
                continue;
            }
            if (marker.Kind is OverlayMarkerKind.Apoapsis or OverlayMarkerKind.Periapsis
                or OverlayMarkerKind.AscendingNode or OverlayMarkerKind.DescendingNode)
            {
                var key = (marker.Kind, marker.BodyId);
                if (!first.TryGetValue(key, out var known)
                    || marker.TimeSeconds < known.TimeSeconds)
                    first[key] = marker;
                continue;
            }
            // Generic labels stay unadorned so the render loop can append one fresh
            // countdown instead of extending an already formatted label every frame.
            result.Add(marker.Kind == OverlayMarkerKind.ClosestApproach
                ? marker with { Label = MarkerLabelAt(marker, t0Seconds) }
                : marker);
        }
        foreach (var marker in first.Values)
            result.Add(marker);
        result.Sort((a, b) => a.TimeSeconds.CompareTo(b.TimeSeconds));
        return result;
    }

    /// <summary>Cheap epoch refresh for immutable geometry: rebuild the two padded
    /// stock payload arrays without sampling or taking the rails Gate.</summary>
    public static (double[] TimesSincePe, double[] RemainingTimesTo) RestampPayloadTimes(
        double[] times, double t0Seconds, double timeAtPeSeconds)
    {
        var sincePe = new double[times.Length];
        var remaining = new double[times.Length];
        for (int i = 0; i < times.Length; i++)
        {
            sincePe[i] = times[i] - timeAtPeSeconds;
            remaining[i] = times[i] - t0Seconds;
        }
        return (sincePe, remaining);
    }
    /// <summary>Linear position on an immutable sampled trajectory. False outside the
    /// real time window or when the arrays are inconsistent.</summary>
    public static bool TryInterpolatedPosition(
        double[] times, Vector3d[] positions, double time, out Vector3d position)
    {
        position = default;
        if (times.Length == 0 || positions.Length != times.Length
            || time < times[0] || time > times[^1])
            return false;
        var (lo, hi, frac) = LerpBracket(times, time);
        position = positions[lo] * (1 - frac) + positions[hi] * frac;
        return true;
    }

    /// <summary>How close to the surface the collision bisection refines the impact
    /// point before stopping — far below display precision, far above fp noise.</summary>
    public const double CollisionCutToleranceMeters = 1.0;

    /// <summary>Surface-frame collision cut over a sampled BODY-CENTRED series
    /// (surface-frame coordinates: rigid pose, meters from the body's center): finds
    /// the FIRST descent of |position| through <paramref name="radius"/>, bisects the
    /// exact crossing through <paramref name="sampleAt"/> — the caller's own sweep
    /// evaluator, so the cut point lies ON the drawn curve, not on a chord lerp of
    /// two possibly-distant samples — and returns the series truncated there with the
    /// crossing appended as its final sample. A series that STARTS at or below the
    /// radius does not count until it exits and re-enters (a landed or sub-surface
    /// seed is not an upcoming impact). Null when the series never descends through
    /// the radius (or the radius is unknown: &lt;= 0). With a terrain sampler, clearance
    /// is measured against mean radius plus directional body-fixed terrain height;
    /// the maximum radius is a broad-phase guard around expensive accurate samples.
    /// A failed terrain lookup falls back to mean radius. The caller owns any degrade
    /// signal its evaluator carries (a mid-cut evaluator failure invalidates the
    /// frame-space result — discard it there).</summary>
    public static CollisionCut? CutAtFirstCollision(double[] times, Vector3d[] positions,
        double radius, Func<double, Vector3d> sampleAt,
        Func<Vector3d, double>? terrainHeightAt = null,
        double maximumSurfaceRadius = double.NaN)
    {
        if (!(radius > 0) || times.Length < 2 || positions.Length != times.Length
            || sampleAt is null)
            return null;
        double Clearance(Vector3d position)
        {
            double distance = position.Length();
            if (terrainHeightAt is null) return distance - radius;
            if (double.IsFinite(maximumSurfaceRadius)
                && maximumSurfaceRadius >= radius && distance > maximumSurfaceRadius)
                return distance - maximumSurfaceRadius;
            double height = terrainHeightAt(position);
            return distance - radius - (double.IsFinite(height) ? height : 0.0);
        }

        static double SegmentDistanceSquared(Vector3d a, Vector3d b)
        {
            var chord = b - a;
            double lengthSquared = chord.LengthSquared();
            if (!(lengthSquared > 0)) return a.LengthSquared();
            double fraction = Math.Clamp(-a.Dot(chord) / lengthSquared, 0.0, 1.0);
            return (a + chord * fraction).LengthSquared();
        }

        // Finds the earliest PROBED above/below bracket inside a broad-phase chord hit.
        // Cosine spacing walks chronologically and clusters evaluations at entry/exit,
        // so a real impact is found near the surface before a point-mass trajectory is
        // queried deep inside the body. It also bounds safe low-orbit false positives
        // to a small fixed amount of exact-curve work.
        bool TryFindInteriorBracket(double t0, Vector3d p0, double t1, Vector3d p1,
            out double above, out double below)
        {
            above = double.NaN;
            below = double.NaN;
            double broadRadius = double.IsFinite(maximumSurfaceRadius)
                && maximumSurfaceRadius >= radius ? maximumSurfaceRadius : radius;
            double segmentDistanceSquared = SegmentDistanceSquared(p0, p1);
            if (segmentDistanceSquared > broadRadius * broadRadius) return false;

            var chord = p1 - p0;
            double chordLengthSquared = chord.LengthSquared();
            if (!(chordLengthSquared > 0)) return false;
            double dot = p0.Dot(chord);
            double closestFraction = Math.Clamp(-dot / chordLengthSquared, 0.0, 1.0);

            // Terrain's deliberately padded maximum envelope can contain an entire
            // safe low orbit. Do one exact verifier for envelope-only candidates;
            // reserve the denser chronological walk for chords that cross the mean
            // body and can hide the fast outside-to-outside transit fixed here.
            if (segmentDistanceSquared > radius * radius)
            {
                double probeTime = t0 + (t1 - t0) * closestFraction;
                if (!(probeTime > t0) || !(probeTime < t1)
                    || Clearance(sampleAt(probeTime)) > 0)
                    return false;
                above = t0;
                below = probeTime;
                return true;
            }

            double discriminant = dot * dot
                - chordLengthSquared * (p0.LengthSquared() - broadRadius * broadRadius);
            // The segment-distance gate proves intersection; clamp tangency roundoff.
            double root = Math.Sqrt(Math.Max(0.0, discriminant));
            double entryFraction = Math.Clamp((-dot - root) / chordLengthSquared, 0.0, 1.0);
            double exitFraction = Math.Clamp((-dot + root) / chordLengthSquared, 0.0, 1.0);
            if (!(exitFraction > entryFraction)) return false;

            const int probes = 24;
            double previousTime = t0;
            double previousClearance = Clearance(p0);
            for (int k = 0; k <= probes; k++)
            {
                double phase = (double)k / probes;
                double clustered = 0.5 * (1.0 - Math.Cos(Math.PI * phase));
                double probeFraction = entryFraction
                    + (exitFraction - entryFraction) * clustered;
                double probeTime = t0 + (t1 - t0) * probeFraction;
                if (!(probeTime > previousTime) || !(probeTime < t1)) continue;
                double probeClearance = Clearance(sampleAt(probeTime));
                if (previousClearance > 0 && probeClearance <= 0)
                {
                    above = previousTime;
                    below = probeTime;
                    return true;
                }
                previousTime = probeTime;
                previousClearance = probeClearance;
            }
            return false;
        }

        int lo = -1;
        double? interiorAbove = null;
        double? interiorBelow = null;
        double previousClearance = Clearance(positions[0]);
        for (int i = 1; i < times.Length; i++)
        {
            double currentClearance = Clearance(positions[i]);
            if (previousClearance > 0 && currentClearance <= 0)
            {
                lo = i - 1;
                break;
            }
            if (previousClearance > 0 && currentClearance > 0
                && TryFindInteriorBracket(times[i - 1], positions[i - 1],
                    times[i], positions[i], out double hiddenAbove, out double hiddenBelow))
            {
                lo = i - 1;
                interiorAbove = hiddenAbove;
                interiorBelow = hiddenBelow;
                break;
            }
            previousClearance = currentClearance;
        }
        if (lo < 0) return null;
        double above = interiorAbove ?? times[lo];
        double below = interiorBelow ?? times[lo + 1];
        for (int k = 0; k < 40; k++)
        {
            double mid = 0.5 * (above + below);
            if (mid == above || mid == below) break; // fp resolution exhausted
            double clearance = Clearance(sampleAt(mid));
            if (Math.Abs(clearance) < CollisionCutToleranceMeters)
            {
                below = mid;
                break;
            }
            if (clearance > 0) above = mid; else below = mid;
        }
        var impact = sampleAt(below); // final eval AT the kept time (memo consistency)
        int keep = lo + 1;
        var cutTimes = new double[keep + 1];
        var cutPositions = new Vector3d[keep + 1];
        Array.Copy(times, cutTimes, keep);
        Array.Copy(positions, cutPositions, keep);
        cutTimes[keep] = below;
        cutPositions[keep] = impact;
        return new CollisionCut(cutTimes, cutPositions, below, impact);
    }

    /// <summary>Routes a line for the map layer per <see cref="LineRoute"/>: a fresh
    /// batch in the current frame MODE draws; fresh but wrong mode blinks; stale or
    /// disabled hands back to stock. SOI-independence: there is deliberately no
    /// parentMatches input — a stock SOI/patch transition re-parenting the
    /// orbit does not gate drawing at all. The staged points are parent-centred Cce,
    /// so the stage path re-anchors an old-parent batch by <see cref="ParentShift"/>
    /// instead of blinking it (people are expected to pick the right reference frame;
    /// the line itself never disappears at an SOI boundary).</summary>
    public static LineRoute RouteLine(bool enabled, bool samplesFresh, bool modeMatches)
    {
        if (!enabled || !samplesFresh) return LineRoute.Stock;
        return modeMatches ? LineRoute.Draw : LineRoute.Blink;
    }

    /// <summary>SOI-independence: shift added to every parent-centred Cce point when the
    /// batch was sampled under a different stock parent than the orbit is staged into
    /// (an SOI/patch transition landed between publish and this stage). Stock draws each
    /// point at currentParent(now) + point, so shifting by batchParent(now) −
    /// currentParent(now) makes the drawn world position batchParent(now) + Cce — the
    /// same rendering the batch produced before the transition, with no displacement by
    /// the interbody distance and no blink. Identity (zero) when the parents agree.</summary>
    public static Vector3d ParentShift(Vector3d batchParentNow, Vector3d currentParentNow)
        => batchParentNow - currentParentNow;

    /// <summary>SOI-independence: stock's map sphere-of-influence indicator (the
    /// glass-ball gizmo, IOrbiter.cs:296-311) is hidden exactly while the mod is enabled
    /// AND bound to a system. Under n-body gravity the SOI boundary has no dynamical
    /// meaning, and the mod's lines deliberately ignore it — showing the sphere would
    /// dress a bookkeeping radius up as physics. Unbound or disabled (fault, user,
    /// incompatible build) the game is running stock propagation, where the indicator
    /// is truthful — stock draws it, unchanged.</summary>
    public static bool SoiIndicatorsHidden(bool enabled, bool bound, bool railsReady) =>
        enabled && bound && railsReady;

    /// <summary>Live celestial catalog: deterministic curve-body priority + cap. A dense
    /// catalog (SolSystemDense: hundreds of bodies) must not put unbounded sampling work
    /// on the ~1 Hz rails worker, so honest arcs go to the top <paramref name="maxBodies"/>
    /// (clamped to at least 1) by fixed rule: mutual-backbone bodies first (their
    /// stock lines are the least truthful), then µ descending,
    /// ties by <see cref="StringComparer.Ordinal"/> id (culture-free). Every key is
    /// parse-time constant, so the priority is stable; a runtime cap selects a prefix
    /// without reordering it.</summary>
    public static IReadOnlyList<string> CurvePriority(
        IEnumerable<(string Id, double Mu, bool Backbone)> bodies, int maxBodies)
        => bodies
            .OrderByDescending(b => b.Backbone)
            .ThenByDescending(b => b.Mu)
            .ThenBy(b => b.Id, StringComparer.Ordinal)
            .Select(b => b.Id)
            .Take(Math.Max(1, maxBodies))
            .ToArray();

    // (No separate inertial-anchor kernel is needed for the honest-density draw:
    // the dense splice compares ABSOLUTE times against "now", which anchors the splice
    // correctly in both batch modes by construction — see CelestialLinePatch.)

    /// <summary>A burn belongs in the display when it is strictly ahead of the vessel
    /// (a burn at or before "now" is stock's to execute, not ours to predict) and no
    /// later than the horizon (an impulse past the sampled window cannot show).</summary>
    public static bool BurnInWindow(double burnTime, double t0, double horizon) =>
        burnTime > t0 && burnTime <= horizon;

    /// <summary>Two-line display: where the PLANNED trajectory's
    /// own sampled window starts — the earliest in-window burn. Before that time the
    /// planned path is IDENTICAL to the actual path (impulses only change velocity, so
    /// position diverges strictly after the first burn), so sampling the planned line
    /// earlier would only overdraw the actual line. Null when no burn is in the window:
    /// there is no planned line to show, the actual line is the whole story.</summary>
    public static double? PlannedWindowStart(IReadOnlyList<double> burnTimes, double t0, double horizon)
    {
        double? first = null;
        foreach (double t in burnTimes)
            if (BurnInWindow(t, t0, horizon) && (first is null || t < first)) first = t;
        return first;
    }

    /// <summary>The planned line has a real plan boundary, unlike the actual future
    /// line whose orbit window is a floor. A resource- or dynamics-limited actual
    /// sweep additionally caps planned coverage to the actual line's achieved end so
    /// a late planned branch cannot float beyond a missing actual reference path.
    /// Collision cuts are deliberately not coverage limits: a burn before impact may
    /// produce a valid planned trajectory beyond the actual collision time.</summary>
    public static double PlannedHorizonSeconds(double actualHorizonSeconds,
        double planEndSeconds, double actualSampleEndSeconds, bool actualCoverageLimited)
    {
        double horizon = actualHorizonSeconds;
        if (double.IsFinite(planEndSeconds)) horizon = Math.Min(horizon, planEndSeconds);
        if (actualCoverageLimited && double.IsFinite(actualSampleEndSeconds))
            horizon = Math.Min(horizon, actualSampleEndSeconds);
        return horizon;
    }

    /// <summary>A planned suffix may extend beyond an ordinary early actual endpoint
    /// (notably when a pre-impact burn avoids collision), but its branch point must
    /// still lie on the achieved actual line. Full actual coverage trivially connects
    /// every in-window branch.</summary>
    public static bool PlannedBranchConnected(double plannedStartSeconds,
        double actualSampleEndSeconds, double plannedHorizonSeconds) =>
        actualSampleEndSeconds >= plannedHorizonSeconds
        || plannedStartSeconds <= actualSampleEndSeconds;

    /// <summary>Off rails, planned geometry is restamped rather than resampled. A
    /// shorter plan must therefore reject an overlong cached batch; an equal or longer
    /// plan may retain the already-proven prefix until rails resume.</summary>
    public static bool PlannedOffRailsHorizonCompatible(
        double batchHorizonSeconds, double planEndSeconds) =>
        !double.IsFinite(planEndSeconds) || batchHorizonSeconds <= planEndSeconds;

    /// <summary>Plan snapshot ("the plan is a snapshot", by design): where the PLANNED
    /// line's sampled window starts. Not diverged — reality still matches the plan's
    /// world — keeps the two-line look: branch at the first upcoming burn
    /// (<see cref="PlannedWindowStart"/>; null = nothing separate to draw). Diverged —
    /// a burn/live-physics episode moved the vessel off the plan's world — the whole
    /// ghost draws from "now", even with no upcoming burns: the plan's end trajectory
    /// is the reference the pilot compares the actual line against.</summary>
    public static double? SnapshotSampleStart(IReadOnlyList<double> burnTimes, double t0,
        double horizon, bool diverged)
        => diverged ? t0 : PlannedWindowStart(burnTimes, t0, horizon);

    /// <summary>Plan snapshot: where the burn FOLD starts. Diverged, the ghost is the
    /// plan's own world simulated from its anchor — captured burns already flown (or
    /// skipped) in reality still fold, because in the plan's world they happen. Not
    /// diverged, the anchored world IS reality, so the fold keeps the live rule
    /// (strictly-future burns only; a past burn is stock's to execute, not ours to
    /// re-predict).</summary>
    public static double SnapshotFoldStart(bool diverged, double anchorEpochSeconds, double t0)
        => diverged ? anchorEpochSeconds : t0;

    /// <summary>Whole-snapshot capture requires one clean stock patch-chain scan.
    /// Dirty scans may keep drawing an existing mirrored snapshot, but must not install
    /// fallback parents that an immediate divergence could freeze. A diverged plan with
    /// no snapshot still captures once the chain becomes clean.</summary>
    public static bool SnapshotReconcileAllowed(bool patchChainReady,
        bool diverged, bool hasSnapshot) =>
        patchChainReady && (!diverged || !hasSnapshot);

    /// <summary>Not-diverged snapshots re-anchor at "now" when the anchor is older
    /// than this (sim seconds): while not diverged the anchor is interchangeable with
    /// the live state, and a fresh one bounds how far the diverged ghost must
    /// integrate from the moment reality departs.</summary>
    public const double SnapshotAnchorMaxAgeSeconds = 3600.0;

    /// <summary>Bounded catch-up growth for a fixed planned window. Available rails
    /// coverage may jitter down/up as now races the rails worker, so only a new
    /// geometric high-water mark (2x the last sampled coverage) or reaching the
    /// desired window resamples. This yields O(log desired/initial) startup sweeps and
    /// zero steady-state churn when availability merely recovers to its prior level.</summary>
    public static bool PlannedCoverageExpansionDue(
        double lastSampledDays, double availableDays, double desiredDays)
    {
        if (!(desiredDays > 0) || !(availableDays > lastSampledDays)
            || lastSampledDays >= desiredDays)
            return false;
        double reached = Math.Min(availableDays, desiredDays);
        return reached >= desiredDays || reached >= Math.Max(1.0, lastSampledDays * 2.0);
    }

    /// <summary>During live/off-rails flight, planned integration and restamping are
    /// background work. Existing planned geometry can still
    /// be restamped whenever its exact snapshot and draw context are unchanged.
    /// Divergence is intentionally not an input: it changes reality, not the captured
    /// plan, so clearing the already-sampled plan when thrust starts would make the
    /// reference trajectory disappear for the duration of the burn. Exact reference
    /// identity is required here (unlike ordinary on-rails re-anchoring): a Rebase can
    /// preserve burn geometry while replacing the plan-world anchor, and old geometry
    /// must not survive that snapshot replacement.</summary>
    public static bool PlannedOffRailsRestampAllowed(
        PlanSnapshot? snapshot, PlanSnapshot? sampledSnapshot, bool sameContext) =>
        snapshot is not null && ReferenceEquals(snapshot, sampledSnapshot) && sameContext;

    /// <summary>Plan snapshot: is a full planned-batch RESAMPLE due, or may the
    /// previous batch be republished with a fresh wall stamp? Resample on any input
    /// change (snapshot identity, diverged flag, staging parent, frame mode, the
    /// horizon inputs — a plan-length or orbits-window edit must show while paused),
    /// when the branch point moved (not-diverged start is the first upcoming burn —
    /// it jumps when a burn is crossed or edited; the diverged start is just "now"),
    /// when the clock went backwards (a load), or when the cached geometry has no
    /// future samples left. A planned batch is a fixed window in the plan's world,
    /// unlike the actual line's rolling now+horizon window; non-diverged batches die
    /// when their first burn is crossed, and diverged ghosts clip elapsed samples at
    /// draw/hover. The caller refreshes every now-anchored payload, so simulation-time
    /// age alone is not a reason to repeat an expensive sweep under warp.</summary>
    public static bool PlannedResampleDue(bool sameSnapshot, bool sameDiverged, bool diverged,
        double startSeconds, double lastStartSeconds, bool sameParent, bool sameFrame,
        bool sameGeometryInputs, double t0Seconds, double batchT0Seconds,
        double batchEndSeconds)
    {
        if (!sameSnapshot || !sameDiverged || !sameParent || !sameFrame || !sameGeometryInputs) return true;
        if (!diverged && startSeconds != lastStartSeconds) return true;
        if (t0Seconds < batchT0Seconds) return true;
        return !double.IsFinite(batchEndSeconds) || batchEndSeconds <= t0Seconds;
    }

    public readonly record struct FutureClip(int Lo, int Hi, double Frac);

    /// <summary>Clip boundary for the future-only suffix of a strictly ordered sampled
    /// line. When now lies inside the window, Lo/Hi bracket an interpolated vertex at
    /// now and Hi is the first original future sample. Before the window Lo==Hi==0;
    /// at/after the final sample there is no future geometry.</summary>
    public static FutureClip? FutureClipAt(double[] times, double nowSeconds)
    {
        if (times.Length == 0) return null;
        int hi = UpperBound(times, nowSeconds);
        if (hi == 0) return new FutureClip(0, 0, 0.0);
        if (hi >= times.Length) return null;
        int lo = hi - 1;
        double span = times[hi] - times[lo];
        double frac = span > 0 ? Math.Clamp((nowSeconds - times[lo]) / span, 0.0, 1.0) : 0.0;
        return new FutureClip(lo, hi, frac);
    }
    /// <summary>Bracketing sample pair for time <paramref name="t"/> over a
    /// NON-DECREASING times array (the pad-to-stock-length tail repeats the last real
    /// time, which this handles: a zero-span bracket collapses to Frac 0). Clamps to
    /// the ends — callers window-gate t themselves when clamping would lie. Feeds the
    /// burn-node marker interpolation (drawn-line position at the burn's time).</summary>
    public static (int Lo, int Hi, double Frac) LerpBracket(double[] times, double t)
    {
        int found = Array.BinarySearch(times, t);
        if (found >= 0) return (found, found, 0.0);
        int insertion = ~found;
        if (insertion <= 0) return (0, 0, 0.0);
        if (insertion >= times.Length) return (times.Length - 1, times.Length - 1, 0.0);
        int lo = insertion - 1;
        double span = times[insertion] - times[lo];
        return span > 0 ? (lo, insertion, (t - times[lo]) / span) : (lo, lo, 0.0);
    }

    /// <summary>Folds immutable snapshot records using each burn's captured stock
    /// patch parent. The callback is KSA-free and receives the selected parent id and
    /// burn time; it must return that parent's absolute state. A null per-burn parent
    /// uses <paramref name="fallbackParentId"/>.
    /// <paramref name="burnTimes"/> is the snapshot's precomputed array in the exact
    /// same order as <paramref name="burns"/>; mismatched counts are rejected before
    /// any predictor work.
    /// Conversion happens lazily inside <see cref="FoldBurns"/>, so burn i sees the
    /// pre-burn trajectory after every earlier captured burn. Both impulsive and
    /// finite display folds use this exact seam. The conversion reads the PREDICTOR
    /// state, so an execution-basis burn folds its DisplayDvVlf instead — its raw
    /// stock components would fold rotated by the conic drift.</summary>
    public static int FoldSnapshotBurns(TrajectoryPredictor display,
        IReadOnlyList<PlanSnapshotBurn> burns, IReadOnlyList<double> burnTimes,
        double t0, double horizon,
        string fallbackParentId,
        Func<string, double, StateVector> parentStateAt,
        Action<string> warn, FiniteBurnFold? finite,
        out double earliestStartSeconds)
    {
        if (burns.Count != burnTimes.Count)
            throw new ArgumentException("Burn records and precomputed times must have equal counts.",
                nameof(burnTimes));
        return FoldBurns(display, burnTimes, t0, horizon,
            i =>
            {
                PlanSnapshotBurn burn = burns[i];
                StateVector parent = parentStateAt(
                    burn.BasisParentId ?? fallbackParentId, burn.TimeSeconds);
                // The parent callback runs first so production cancellation can abort
                // before StateAt performs a potentially long predictor extension.
                StateVector state = display.StateAt(burn.TimeSeconds);
                return BurnFrameKernel.VlfToEcl(burn.DisplayDvVlf ?? burn.DeltaVVlf,
                    state.Position - parent.Position, state.Velocity - parent.Velocity);
            }, warn, finite, out earliestStartSeconds);
    }

    /// <summary>Folds planned burns into <paramref name="display"/> as impulses.
    /// <paramref name="deltaVEclFor"/> converts burn i's delta-v to the mod's absolute
    /// ecliptic axes and is invoked lazily, only for in-window burns and only after all
    /// earlier burns were folded — so a converter that reads the display state sees the
    /// pre-burn trajectory WITH prior burns applied, mirroring stock burn chaining
    /// (each burn's flight plan builds on the previous one's). A null conversion means
    /// a degenerate VLF frame (radial trajectory): the burn is skipped with a warning.
    /// AddImpulse's fixed impulse semantics THROW on a duplicate exact time — display-
    /// degenerate anyway (two stock nodes at one timestamp), so the first burn is kept
    /// and the duplicate reported via <paramref name="warn"/>. Returns the number of
    /// burns applied. Thread-safety: pure list work plus whatever the callbacks do —
    /// the caller owns the locking for any shared-ephemerides access inside them.
    ///
    /// Finite-burn estimation: with a usable <paramref name="finite"/>
    /// model, each burn expands into sub-impulse slices over the FC's centered window
    /// (FiniteBurnKernel — the executor's own duration/centering/fixed-direction
    /// semantics), with the vessel mass chained burn to burn. The DISPLAY fold only:
    /// the authoring converters keep impulsive folds, because the VLF dv they write is
    /// what stock's impulsive plan chain executes against. A burn whose window does
    /// not fit — ignition at/before the fold start or a previous burn's last slice,
    /// or cutoff past the horizon — falls back to a single impulse at the node
    /// (honest: near "now" the FC would already be burning, and a horizon-clipped arc
    /// would silently drop delta-v). Burn COUNT semantics are unchanged: one burn is
    /// one applied burn, however many slices carry it.</summary>
    public static int FoldBurns(TrajectoryPredictor display, IReadOnlyList<double> burnTimes,
        double t0, double horizon, Func<int, Vector3d?> deltaVEclFor, Action<string> warn,
        FiniteBurnFold? finite = null)
        => FoldBurns(display, burnTimes, t0, horizon, deltaVEclFor, warn, finite, out _);

    /// <summary><paramref name="earliestStartSeconds"/> is where the folded
    /// trajectory FIRST departs the coast — the earliest applied burn's ignition
    /// (expanded) or node (impulse); NaN when nothing applied. The planned batch
    /// samples from there, not from the node, or the thrust arc's first half would
    /// never draw and the line's first vertex would sit mid-burn off the actual
    /// trajectory.</summary>
    public static int FoldBurns(TrajectoryPredictor display, IReadOnlyList<double> burnTimes,
        double t0, double horizon, Func<int, Vector3d?> deltaVEclFor, Action<string> warn,
        FiniteBurnFold? finite, out double earliestStartSeconds)
    {
        int applied = 0;
        earliestStartSeconds = double.NaN;
        double massKg = finite?.Engine.MassKg ?? 0.0;
        double lastBound = t0;
        var appliedNodes = new List<double>(burnTimes.Count);
        for (int i = 0; i < burnTimes.Count; i++)
        {
            double burnTime = burnTimes[i];
            if (!BurnInWindow(burnTime, t0, horizon)) continue;
            // Duplicate-node containment, EXPLICIT rather than relying on AddImpulse's
            // exact-time throw (enough for pure impulses): expanded
            // slices never collide with a duplicate NODE's fallback impulse — the
            // doubled delta-v would silently apply.
            if (BurnIdentityPolicy.ContainsBurn(appliedNodes, burnTime))
            {
                warn($"overlay: duplicate burn time {burnTime:F1} s - later node ignored in overlay");
                continue;
            }
            Vector3d? deltaV = deltaVEclFor(i);
            if (deltaV is null)
            {
                // NOTE: the chained massKg is NOT debited for a skipped burn (its
                // magnitude is known only to the converter that just failed); later
                // burns fold slightly heavy. Second-order next to the burn itself
                // missing from the display, and the degenerate frame is already
                // warned — accepted.
                warn($"overlay: burn {i} at t={burnTime:F1} s skipped (degenerate VLF frame)");
                continue;
            }
            var impulses = ExpandOrImpulse(burnTime, deltaV.Value, t0, horizon, ref lastBound,
                finite, ref massKg, out double burnStart);
            try
            {
                foreach (var (time, dv) in impulses)
                    display.AddImpulse(time, dv);
                applied++;
                appliedNodes.Add(burnTime);
                if (double.IsNaN(earliestStartSeconds) || burnStart < earliestStartSeconds)
                    earliestStartSeconds = burnStart;
            }
            catch (ArgumentOutOfRangeException)
            {
                // Subclass of ArgumentException, so it must be caught FIRST: a burn
                // before the trajectory start (only reachable if a caller bypasses the
                // window rule) must not masquerade as a duplicate.
                warn($"overlay: burn at t={burnTime:F1} s precedes the display start - ignored in overlay");
            }
            catch (ArgumentException)
            {
                warn($"overlay: duplicate burn time {burnTime:F1} s - later node ignored in overlay");
            }
        }
        return applied;
    }

    /// <summary>One burn's impulse list under the fold's neighbor constraints: the
    /// finite expansion when its WHOLE window — ignition through cutoff, not just the
    /// slice midpoints — fits strictly after max(t0, lastBound) and within the
    /// horizon; a single node impulse otherwise (a burn already igniting "now"
    /// is the FC's to fly, and a horizon-clipped arc would silently drop delta-v).
    /// <paramref name="lastBound"/> advances to the expansion's cutoff (or the
    /// fallback node, monotonically), so consecutive burns can never interleave
    /// slices. Mass is consumed either way — the FC spends the propellant regardless
    /// of how the display discretizes the burn.</summary>
    private static (double Time, Vector3d Dv)[] ExpandOrImpulse(double burnTime,
        Vector3d deltaV, double t0, double horizon, ref double lastBound,
        FiniteBurnFold? finite, ref double massKg, out double burnStartSeconds)
    {
        burnStartSeconds = burnTime;
        if (finite is null || !finite.Engine.Usable)
        {
            lastBound = Math.Max(lastBound, burnTime);
            return [(burnTime, deltaV)];
        }
        double magnitude = deltaV.Length();
        var engine = finite.Engine with { MassKg = massKg };
        massKg = FiniteBurnKernel.MassAfterBurn(magnitude, engine);
        var expansion = FiniteBurnKernel.Expand(burnTime, magnitude, engine,
            finite.SliceSeconds, finite.MaxSlices);
        if (expansion is null
            || expansion.IgnitionSeconds <= Math.Max(t0, lastBound)
            || expansion.IgnitionSeconds + expansion.DurationSeconds > horizon)
        {
            lastBound = Math.Max(lastBound, burnTime);
            return [(burnTime, deltaV)];
        }
        lastBound = expansion.IgnitionSeconds + expansion.DurationSeconds;
        burnStartSeconds = expansion.IgnitionSeconds;
        var direction = deltaV * (1.0 / magnitude);
        var slices = new (double, Vector3d)[expansion.Times.Length];
        for (int s = 0; s < slices.Length; s++)
            slices[s] = (expansion.Times[s], direction * expansion.Magnitudes[s]);
        return slices;
    }
}

/// <summary>One immutable source-space broad-phase block over the authoritative
/// hover traversal. Bounds cover finite traversal vertices only.</summary>
internal readonly record struct HoverBlock(
    int FirstTraversalSlot,
    int EndTraversalSlot,
    Vector3d Minimum,
    Vector3d Maximum,
    bool HasFinitePoints,
    bool HasUncertainPoints);

/// <summary>Opaque immutable ownership of a hover broad-phase partition. Only the
/// builder can publish a trusted plan; the internal untrusted seam exists solely so
/// corruption/fallback behavior can be exercised without exposing mutable blocks.
/// The matching traversal and source-coordinate arrays are publish-once batch data:
/// callers must never mutate either after this plan is built.</summary>
internal sealed class HoverBlockPlan
{
    private readonly HoverBlock[] _blocks;
    private readonly int[]? _traversal;
    private readonly int _traversalLength;
    private readonly bool _trusted;

    private HoverBlockPlan(HoverBlock[] ownedBlocks, int[]? traversal,
        int traversalLength, bool trusted)
    {
        _blocks = ownedBlocks;
        _traversal = traversal;
        _traversalLength = traversalLength;
        _trusted = trusted;
    }

    public int Count => _blocks.Length;
    public HoverBlock this[int index] => _blocks[index];
    public ReadOnlySpan<HoverBlock> Blocks => _blocks;

    internal static HoverBlockPlan CreateOwned(
        HoverBlock[] ownedBlocks, int[] traversal) =>
        new(ownedBlocks, traversal, traversal.Length, trusted: true);

    internal static HoverBlockPlan CreateUntrustedForTests(
        int traversalLength, params HoverBlock[] blocks) =>
        new((HoverBlock[])blocks.Clone(), null, traversalLength, trusted: false);

    internal bool IsTrustedFor(int[] traversal) =>
        _trusted && ReferenceEquals(_traversal, traversal)
            && _traversalLength == traversal.Length;

    internal static HoverBlockPlan Empty { get; } =
        new([], null, 0, trusted: false);
}

/// <summary>A source vertex or an interpolated future-clip boundary.</summary>
internal readonly record struct HoverPointRef(
    int SourceIndex,
    int OtherSourceIndex,
    double SourceFraction)
{
    public bool HasExactTime { get; init; }
    public double ExactTime { get; init; }
    public HoverPointRef(int sourceIndex) : this(sourceIndex, sourceIndex, 0.0) { }
    public bool IsInterpolated => SourceIndex != OtherSourceIndex;
    public static HoverPointRef Interpolated(int lo, int hi, double fraction) =>
        new(lo, hi, fraction);
    public static HoverPointRef ClippedBoundary(
        int lo, int hi, double fraction, double exactTime) =>
        new(lo, hi, fraction) { HasExactTime = true, ExactTime = exactTime };
}

/// <summary>KSA-free screen coordinate used by the hover broad phase.</summary>
internal readonly record struct HoverScreenPoint(float X, float Y);

/// <summary>Projected screen coordinate and signed camera-forward depth.</summary>
internal readonly record struct HoverProjection(HoverScreenPoint Screen, double Depth);

/// <summary>Whether a projected source block must refine, is proved wholly hidden,
/// or has a conservative finite screen rectangle suitable for pruning.</summary>
internal enum HoverBoundsKind : byte
{
    Unprunable,
    WhollyBehind,
    Bounded,
}

/// <summary>One projector-certified conservative screen rectangle.</summary>
internal readonly record struct HoverProjectedBounds(
    HoverBoundsKind Kind,
    double MinimumX,
    double MinimumY,
    double MaximumX,
    double MaximumY);

/// <summary>The stock-compatible nearest-vertex plus adjacent-chord result.</summary>
internal readonly record struct HoverHit(
    HoverPointRef Lo,
    HoverPointRef Hi,
    double Fraction,
    HoverScreenPoint Projected);

/// <summary>Allocation-free projection seam for game, tests, and benchmarks.</summary>
internal interface IHoverPointProjector
{
    int SourceCount { get; }
    double NearPlaneDepth { get; }
    HoverProjection Project(HoverPointRef point);
    HoverProjection ProjectSource(Vector3d sourceCoordinate);
    HoverProjectedBounds ProjectBounds(Vector3d minimum, Vector3d maximum);
}

/// <summary>Allocation-free coarse-to-fine hover search. Immutable source AABBs
/// reduce the common query from the complete 8,192-point traversal to at most 32
/// current-camera box projections plus only blocks that can contain the exact
/// nearest vertex. Uncertain boxes refine rather than cull.</summary>
internal static class HoverHitTestKernel
{
    public const int BlockSize = 256;
    public const int MaximumBlocks =
        DecimationMetrics.MaximumTraversalPoints / BlockSize;
    /// <summary>Builds a plan for publish-once <paramref name="points"/> and
    /// <paramref name="traversal"/> arrays. The returned bounds remain valid only
    /// while those internal batch inputs are unchanged.</summary>
    public static HoverBlockPlan BuildBlocks(Vector3d[] points, int[] traversal)
    {
        if (traversal.Length > DecimationMetrics.MaximumTraversalPoints)
            throw new ArgumentOutOfRangeException(nameof(traversal));
        int blockCount = (traversal.Length + BlockSize - 1) / BlockSize;
        var blocks = new HoverBlock[blockCount];
        for (int blockIndex = 0; blockIndex < blockCount; blockIndex++)
        {
            int first = blockIndex * BlockSize;
            int end = Math.Min(first + BlockSize, traversal.Length);
            double minX = double.PositiveInfinity, minY = double.PositiveInfinity,
                minZ = double.PositiveInfinity;
            double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity,
                maxZ = double.NegativeInfinity;
            bool hasFinite = false, hasUncertain = false;
            for (int slot = first; slot < end; slot++)
            {
                int sourceIndex = traversal[slot];
                if ((uint)sourceIndex >= (uint)points.Length)
                    throw new ArgumentOutOfRangeException(nameof(traversal));
                Vector3d p = points[sourceIndex];
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y)
                    || !double.IsFinite(p.Z))
                {
                    hasUncertain = true;
                    continue;
                }
                hasFinite = true;
                minX = Math.Min(minX, p.X); minY = Math.Min(minY, p.Y);
                minZ = Math.Min(minZ, p.Z); maxX = Math.Max(maxX, p.X);
                maxY = Math.Max(maxY, p.Y); maxZ = Math.Max(maxZ, p.Z);
            }
            blocks[blockIndex] = new HoverBlock(first, end,
                hasFinite ? new Vector3d(minX, minY, minZ) : default,
                hasFinite ? new Vector3d(maxX, maxY, maxZ) : default,
                hasFinite,
                hasUncertain);
        }
        return HoverBlockPlan.CreateOwned(blocks, traversal);
    }

    /// <summary>Finds the exact full-traversal answer over the suffix beginning at
    /// <paramref name="firstDenseIndex"/>. <paramref name="prefix"/> is the optional
    /// interpolated clip boundary or explicit first sample immediately before that
    /// suffix.</summary>
    public static bool TryNearest<TProjector>(
        int[] traversal,
        HoverBlockPlan plan,
        int firstDenseIndex,
        HoverPointRef? prefix,
        HoverScreenPoint mouse,
        ref TProjector projector,
        out HoverHit hit)
        where TProjector : struct, IHoverPointProjector
    {
        hit = default;
        if (!float.IsFinite(mouse.X) || !float.IsFinite(mouse.Y)
            || traversal.Length == 0 && prefix is null)
            return false;
        int traversalStart = Array.BinarySearch(traversal, firstDenseIndex);
        if (traversalStart < 0) traversalStart = ~traversalStart;
        bool hasPrefix = prefix.HasValue;
        int visibleCount = traversal.Length - traversalStart + (hasPrefix ? 1 : 0);
        if (visibleCount <= 0) return false;

        float bestDistance = float.MaxValue;
        int bestOrdinal = int.MaxValue;
        HoverPointRef bestPoint = default;
        HoverScreenPoint bestScreen = default;
        if (prefix is { } prefixPoint)
            ConsiderVertex(prefixPoint, 0, mouse, ref projector,
                ref bestDistance, ref bestOrdinal, ref bestPoint, ref bestScreen);

        // Compatibility and corruption defense for explicitly constructed metrics.
        // Published batches have a complete immutable partition; an empty, partial,
        // overlapping, or oversized plan falls back to the exact bounded stream.
        bool useBlocks = plan.IsTrustedFor(traversal);
        if (!useBlocks)
        {
            for (int slot = traversalStart; slot < traversal.Length; slot++)
            {
                var point = new HoverPointRef(traversal[slot]);
                int ordinal = slot - traversalStart + (hasPrefix ? 1 : 0);
                ConsiderVertex(point, ordinal, mouse, ref projector,
                    ref bestDistance, ref bestOrdinal, ref bestPoint, ref bestScreen);
            }
        }

        if (useBlocks)
        {
            Span<double> lowerBounds = stackalloc double[MaximumBlocks];
            Span<int> order = stackalloc int[MaximumBlocks];
            int activeBlocks = 0;
            for (int blockIndex = 0; blockIndex < plan.Count; blockIndex++)
            {
                HoverBlock block = plan[blockIndex];
                if (block.EndTraversalSlot <= traversalStart
                    || !block.HasFinitePoints && !block.HasUncertainPoints)
                    continue;
                double lowerBound = BlockLowerBound(block, mouse, ref projector);
                if (double.IsPositiveInfinity(lowerBound)) continue;

                int insert = activeBlocks;
                while (insert > 0)
                {
                    int prior = order[insert - 1];
                    double priorBound = lowerBounds[prior];
                    if (priorBound < lowerBound
                        || priorBound == lowerBound && prior < blockIndex)
                        break;
                    order[insert] = prior;
                    insert--;
                }
                lowerBounds[blockIndex] = lowerBound;
                order[insert] = blockIndex;
                activeBlocks++;
            }

            for (int ranked = 0; ranked < activeBlocks; ranked++)
            {
                int blockIndex = order[ranked];
                if (bestOrdinal != int.MaxValue
                    && lowerBounds[blockIndex] > bestDistance)
                    break;
                HoverBlock block = plan[blockIndex];
                int firstSlot = Math.Max(traversalStart, block.FirstTraversalSlot);
                for (int slot = firstSlot; slot < block.EndTraversalSlot; slot++)
                {
                    var point = new HoverPointRef(traversal[slot]);
                    int ordinal = slot - traversalStart + (hasPrefix ? 1 : 0);
                    ConsiderVertex(point, ordinal, mouse, ref projector,
                        ref bestDistance, ref bestOrdinal, ref bestPoint, ref bestScreen);
                }
            }
        }

        if (bestOrdinal == int.MaxValue) return false;

        HoverPointRef lo = bestPoint, hi = bestPoint;
        double fraction = 0.0;
        HoverScreenPoint projected = bestScreen;
        double bestSquared = SquaredDistance(projected, mouse);
        if (bestOrdinal > 0)
        {
            HoverPointRef left = PointAt(
                bestOrdinal - 1, traversal, traversalStart, prefix);
            HoverScreenPoint leftScreen = projector.Project(left).Screen;
            ConsiderSegment(left, bestPoint, leftScreen, bestScreen, mouse,
                ref lo, ref hi, ref fraction, ref projected, ref bestSquared);
        }
        if (bestOrdinal + 1 < visibleCount)
        {
            HoverPointRef right = PointAt(
                bestOrdinal + 1, traversal, traversalStart, prefix);
            HoverScreenPoint rightScreen = projector.Project(right).Screen;
            ConsiderSegment(bestPoint, right, bestScreen, rightScreen, mouse,
                ref lo, ref hi, ref fraction, ref projected, ref bestSquared);
        }
        hit = new HoverHit(lo, hi, fraction, projected);
        return true;
    }

    private static HoverPointRef PointAt(int ordinal, int[] traversal,
        int traversalStart, HoverPointRef? prefix)
    {
        if (prefix is { } prefixed)
        {
            if (ordinal == 0) return prefixed;
            ordinal--;
        }
        return new HoverPointRef(traversal[traversalStart + ordinal]);
    }

    private static void ConsiderVertex<TProjector>(HoverPointRef point, int ordinal,
        HoverScreenPoint mouse, ref TProjector projector,
        ref float bestDistance, ref int bestOrdinal,
        ref HoverPointRef bestPoint, ref HoverScreenPoint bestScreen)
        where TProjector : struct, IHoverPointProjector
    {
        if ((uint)point.SourceIndex >= (uint)projector.SourceCount
            || (uint)point.OtherSourceIndex >= (uint)projector.SourceCount)
            return;
        HoverScreenPoint screen = projector.Project(point).Screen;
        if (float.IsNaN(screen.X) || float.IsNaN(screen.Y)) return;
        float distance = Math.Abs(mouse.X - screen.X) + Math.Abs(mouse.Y - screen.Y);
        if (distance < bestDistance
            || distance == bestDistance && ordinal < bestOrdinal)
        {
            bestDistance = distance;
            bestOrdinal = ordinal;
            bestPoint = point;
            bestScreen = screen;
        }
    }

    private static double BlockLowerBound<TProjector>(HoverBlock block,
        HoverScreenPoint mouse, ref TProjector projector)
        where TProjector : struct, IHoverPointProjector
    {
        if (block.HasUncertainPoints) return 0.0;
        HoverProjectedBounds bounds = projector.ProjectBounds(
            block.Minimum, block.Maximum);
        if (bounds.Kind == HoverBoundsKind.WhollyBehind)
            return double.PositiveInfinity;
        if (bounds.Kind != HoverBoundsKind.Bounded
            || !double.IsFinite(bounds.MinimumX)
            || !double.IsFinite(bounds.MinimumY)
            || !double.IsFinite(bounds.MaximumX)
            || !double.IsFinite(bounds.MaximumY)
            || bounds.MinimumX > bounds.MaximumX
            || bounds.MinimumY > bounds.MaximumY)
            return 0.0;

        if (!TryOutwardFloat(bounds.MinimumX, lower: true, out float minX)
            || !TryOutwardFloat(bounds.MinimumY, lower: true, out float minY)
            || !TryOutwardFloat(bounds.MaximumX, lower: false, out float maxX)
            || !TryOutwardFloat(bounds.MaximumY, lower: false, out float maxY))
            return 0.0;
        float dx = mouse.X < minX ? minX - mouse.X
            : mouse.X > maxX ? mouse.X - maxX : 0.0f;
        float dy = mouse.Y < minY ? minY - mouse.Y
            : mouse.Y > maxY ? mouse.Y - maxY : 0.0f;
        if (!float.IsFinite(dx) || !float.IsFinite(dy)) return 0.0;
        if (dx > 0.0f) dx = MathF.BitDecrement(dx);
        if (dy > 0.0f) dy = MathF.BitDecrement(dy);
        float lower = dx + dy;
        if (!float.IsFinite(lower) || lower <= 0.0f) return 0.0;
        // The fine search's incumbent is float Manhattan distance. Directed float
        // rounding here protects against its subtraction/addition rounding down;
        // comparing a mathematically exact double lower bound would not.
        return Math.Max(0.0f, MathF.BitDecrement(lower));
    }

    private static bool TryOutwardFloat(
        double value, bool lower, out float rounded)
    {
        rounded = (float)value;
        if (!float.IsFinite(rounded)) return false;
        if (lower && rounded > value)
            rounded = MathF.BitDecrement(rounded);
        else if (!lower && rounded < value)
            rounded = MathF.BitIncrement(rounded);
        return float.IsFinite(rounded);
    }

    private static void ConsiderSegment(HoverPointRef a, HoverPointRef b,
        HoverScreenPoint pa, HoverScreenPoint pb, HoverScreenPoint mouse,
        ref HoverPointRef lo, ref HoverPointRef hi, ref double fraction,
        ref HoverScreenPoint projected, ref double bestSquared)
    {
        if (!TryCloserSegment(pa, pb, mouse, bestSquared,
                out double t, out HoverScreenPoint q, out double squared))
            return;
        bestSquared = squared;
        lo = a; hi = b; fraction = t;
        projected = q;
    }

    /// <summary>One implementation of the adjacent-chord rule shared
    /// by the dense adapter and the allocation-free decimated hover kernel.</summary>
    internal static bool TryCloserSegment(HoverScreenPoint a, HoverScreenPoint b,
        HoverScreenPoint mouse, double incumbentSquared, out double fraction,
        out HoverScreenPoint projected, out double squared)
    {
        fraction = 0.0;
        projected = default;
        squared = incumbentSquared;
        if (float.IsNaN(a.X) || float.IsNaN(a.Y)
            || float.IsNaN(b.X) || float.IsNaN(b.Y))
            return false;
        double abX = b.X - a.X, abY = b.Y - a.Y;
        double lengthSquared = abX * abX + abY * abY;
        if (lengthSquared <= 0.0) return false;
        double t = Math.Clamp(((mouse.X - a.X) * abX
            + (mouse.Y - a.Y) * abY) / lengthSquared, 0.0, 1.0);
        var q = new HoverScreenPoint(
            (float)(a.X + t * abX),
            (float)(a.Y + t * abY));
        double candidateSquared = SquaredDistance(q, mouse);
        if (candidateSquared >= incumbentSquared) return false;
        fraction = t;
        projected = q;
        squared = candidateSquared;
        return true;
    }

    private static double SquaredDistance(HoverScreenPoint a, HoverScreenPoint b)
    {
        double dx = a.X - b.X, dy = a.Y - b.Y;
        return dx * dx + dy * dy;
    }
}

/// <summary>The screen-space emit filter's worker-precomputed inputs over ONE
/// positions array: cumulative chord lengths (the density term) and per-vertex
/// Douglas-Peucker significance (the shape term). Built together so a draw site
/// can never pair arc lengths from one coordinate space with significance from
/// another — that mismatch starves or over-fires the filter and compiles fine.</summary>
public sealed record DecimationMetrics(
    double[] ArcCum,
    double[] Significance,
    int[] TraversalIndices)
{
    /// <summary>Immutable block index over <see cref="TraversalIndices"/> in the
    /// same source coordinate space as ArcCum/Significance. OverlaySamples publishes
    /// the coordinate and metric arrays once; mutating them afterward is unsupported
    /// and would invalidate both the decimation metrics and these retained bounds.</summary>
    internal HoverBlockPlan HoverPlan { get; init; } = HoverBlockPlan.Empty;

    /// <summary>Hard bound for render/input traversal. The adaptive sweep may retain
    /// hundreds of thousands of integration samples for analysis and precise marker
    /// interpolation, but a draw or mouse query must never inherit that cost. This
    /// worker-built ordered subset is then refined by the existing screen-space
    /// significance filter at draw time.</summary>
    public const int MaximumTraversalPoints = 8192;

    public static DecimationMetrics For(Vector3d[] points,
        int maximumTraversalPoints = MaximumTraversalPoints)
    {
        if (maximumTraversalPoints < 2)
            throw new ArgumentOutOfRangeException(nameof(maximumTraversalPoints));
        maximumTraversalPoints = Math.Min(maximumTraversalPoints, MaximumTraversalPoints);
        var significance = OverlayKernel.ChordSignificance(points);
        int[] traversal = OverlayKernel.BoundedTraversalIndices(
            significance, maximumTraversalPoints);
        return new DecimationMetrics(
            OverlayKernel.CumulativeArcLengths(points),
            significance,
            traversal)
        {
            HoverPlan = HoverHitTestKernel.BuildBlocks(points, traversal),
        };
    }
}

/// <summary>One surface-collision cut result (<see cref="OverlayKernel.CutAtFirstCollision"/>):
/// the sampled series truncated at the impact, whose FINAL sample is the bisected
/// surface crossing itself.</summary>
public sealed record CollisionCut(
    double[] Times, Vector3d[] Positions, double ImpactTimeSeconds, Vector3d ImpactCoordinate);

/// <summary>Map-layer line routing. Draw: batch is fresh and in the current frame MODE
/// (SOI-independence: a parent change never blinks the line; the
/// stage path re-anchors via <see cref="OverlayKernel.ParentShift"/> so the line stays
/// drawn straight across SOI boundaries). Blink: fresh but wrong frame mode (a
/// frame switch) — draw nothing AND suppress stock for the ≤1 tick until the
/// context-aware rebuild republishes; frame-space coordinates drawn under the wrong
/// pose would be geometry noise, and a stock flash would show the exact conic lines
/// the feature removes. Stock: stale (sampling stopped) or mod disabled — the
/// hand-back paths.</summary>
public enum LineRoute { Draw, Blink, Stock }

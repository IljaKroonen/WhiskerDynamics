using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Soi;

/// <summary>Rails-geometric SOI parenting decision (KSA-free; the registry translates
/// game state to these inputs). Stock re-parents an on-rails vessel only through its
/// flight plan's patch schedule (the Freefall branch's patch-EndTime jump,
/// decompiled VehicleUpdateTask.cs:845-867 — the geometric check,
/// PhysicsStates.CheckSoiTransitions at PhysicsStates.cs:487, runs on LIVE substeps
/// only, VehicleUpdateTask.cs:816/884), and those patches are
/// conic-extrapolation-vs-Kepler-body predictions — an n-body trajectory that bends
/// into an encounter the extrapolation misses keeps the stale parent arbitrarily deep
/// into the child's SOI (observed: Earth-parented at 2 km above Luna). This kernel
/// answers the geometric check's question — exit test first, then first child whose
/// SOI contains the vessel (PhysicsStates.cs:498-518) — but over RAILS positions, so
/// the registry can re-parent while still on rails.
///
/// Whisker Dynamics suppresses stock's conic/Kepler encounter and escape patch generation
/// for predictor-owned vessels, so this rails-geometric decision is authoritative and
/// uses the exact stock SOI radii. A NaN or infinite
/// parent SOI (the root star: StellarBody.cs:28 returns +inf) never exits — such
/// comparisons are false by construction.</summary>
public static class SoiReparentKernel
{
    public const double EnterFactor = 1.0;
    public const double ExitFactor = 1.0;

    public readonly record struct Candidate(string Id, Vector3d Position, double SoiRadius);

    /// <summary>One SOI expressed in the vessel-minus-body frame at the two ends of
    /// a checked time span. Relative endpoints make a moving SOI centre part of the
    /// sweep instead of treating the body as stationary.</summary>
    public readonly record struct SweptCandidate(
        string Id, Vector3d RelativeStart, Vector3d RelativeEnd, double SoiRadius);

    /// <summary>One child SOI's vessel-minus-body positions on the shared polyline
    /// sample grid accepted by SweepPolyline.</summary>
    public readonly record struct PolylineCandidate(
        string Id, IReadOnlyList<Vector3d> RelativeSamples, double SoiRadius);

    /// <summary>The first boundary crossed in a checked time span. Fraction is in
    /// [0, 1]; Escape distinguishes an upward parent exit from a child entry.</summary>
    public readonly record struct Crossing(string NewParentId, double Fraction, bool Escape);

    /// <summary>All authoritative parent transitions over a sampled interval, plus
    /// the parent that owns the vessel at the final sample.</summary>
    public readonly record struct SweepResult(
        string FinalParentId, IReadOnlyList<Crossing> Crossings,
        bool CrossingsTruncated);

    /// <summary>Maximum ordered transitions retained for diagnostics. State-machine
    /// traversal continues after this cap so the final parent remains authoritative.</summary>
    public const int MaxRecordedCrossings = 256;

    /// <summary>New parent id, or null to keep the current parent.
    /// <paramref name="grandparentId"/> null = the parent has no eligible parent
    /// (root body or unknown). Exit wins over entry, like stock's check —
    /// a vessel somehow satisfying both is re-expressed upward first and re-decided
    /// from there next check.</summary>
    public static string? Decide(Vector3d vesselAbsolute, Vector3d parentAbsolute, double parentSoi,
        string? grandparentId, IReadOnlyList<Candidate> children,
        double enterFactor = EnterFactor, double exitFactor = ExitFactor)
    {
        if (grandparentId is not null
            && (vesselAbsolute - parentAbsolute).Length() > parentSoi * exitFactor)
            return grandparentId;
        foreach (var child in children)
            if ((vesselAbsolute - child.Position).Length() <= child.SoiRadius * enterFactor)
                return child.Id;
        return null;
    }

    /// <summary>Finds the chronologically first SOI boundary crossed by the relative
    /// trajectory chord. Unlike endpoint classification, the quadratic sweep detects
    /// a complete child transit whose start and end are both outside. The current
    /// parent's escape wins an exact-time tie; child ties retain enumeration order.
    /// A tangent merely touches the boundary and is not a crossing.</summary>
    public static Crossing? FirstCrossing(
        Vector3d parentRelativeStart, Vector3d parentRelativeEnd, double parentSoi,
        string? grandparentId, IReadOnlyList<SweptCandidate> children)
    {
        Crossing? first = null;
        if (grandparentId is not null
            && BoundaryFraction(parentRelativeStart, parentRelativeEnd, parentSoi,
                entering: false) is { } escapeFraction)
        {
            first = new Crossing(grandparentId, escapeFraction, Escape: true);
        }

        foreach (var child in children)
        {
            if (BoundaryFraction(child.RelativeStart, child.RelativeEnd, child.SoiRadius,
                    entering: true) is not { } entryFraction)
                continue;
            // Strict comparison is intentional: an escape was considered first and
            // therefore owns an exact tie; earlier children likewise own child ties.
            if (first is null || entryFraction < first.Value.Fraction)
                first = new Crossing(child.Id, entryFraction, Escape: false);
        }
        return first;
    }

    /// <summary>Returns every parent transition over a shared relative-position
    /// polyline, in chronological order, and the parent at the final sample.</summary>
    public static SweepResult SweepPolyline(
        string parentId,
        IReadOnlyList<double> sampleFractions,
        IReadOnlyList<Vector3d> parentRelativeSamples,
        double parentSoi,
        string? grandparentId,
        IReadOnlyList<PolylineCandidate> children)
    {
        ArgumentNullException.ThrowIfNull(parentId);
        ArgumentNullException.ThrowIfNull(sampleFractions);
        ArgumentNullException.ThrowIfNull(parentRelativeSamples);
        ArgumentNullException.ThrowIfNull(children);
        ValidatePolyline(sampleFractions, parentRelativeSamples, children);
        if (sampleFractions.Count < 2)
            return new SweepResult(parentId, Array.Empty<Crossing>(),
                CrossingsTruncated: false);
        return SweepPolylineCore(parentId, sampleFractions, parentRelativeSamples,
            parentSoi, grandparentId, children);
    }

    private static void ValidatePolyline(
        // Shared sample grid.
        IReadOnlyList<double> fractions,
        IReadOnlyList<Vector3d> parentSamples,
        IReadOnlyList<PolylineCandidate> children)
    {
        int count = fractions.Count;
        if (parentSamples.Count != count)
            throw new ArgumentException();
        for (int i = 0; i < count; i++)
        {
            if (!double.IsFinite(fractions[i]))
                throw new ArgumentException();
            if (i > 0 && !(fractions[i] > fractions[i - 1]))
                throw new ArgumentException();
        }
        foreach (var child in children)
        {
            if (child.RelativeSamples is null)
                throw new ArgumentException();
            if (child.RelativeSamples.Count != count)
                throw new ArgumentException();
        }
    }

    private readonly record struct SegmentEvent(
        double LocalFraction, int ChildIndex, bool ParentEscape);

    private static SweepResult SweepPolylineCore(
        string parentId,
        IReadOnlyList<double> fractions,
        IReadOnlyList<Vector3d> parentSamples,
        double parentSoi,
        string? grandparentId,
        IReadOnlyList<PolylineCandidate> children)
    {
        List<Crossing> crossings = [];
        bool crossingsTruncated = false;
        string currentParentId = parentId;
        int activeChild = -1;
        bool underGrandparent = false;
        for (int segment = 0; segment + 1 < fractions.Count; segment++)
        {
            SweepSegment(parentId, fractions, parentSamples,
                parentSoi, grandparentId, children, segment, crossings,
                ref crossingsTruncated, ref currentParentId, ref activeChild,
                ref underGrandparent);
        }
        return new SweepResult(currentParentId, crossings.ToArray(),
            crossingsTruncated);
    }

    private static void SweepSegment(
        string parentId,
        IReadOnlyList<double> fractions,
        IReadOnlyList<Vector3d> parentSamples,
        double parentSoi,
        string? grandparentId,
        IReadOnlyList<PolylineCandidate> children,
        int segment,
        List<Crossing> crossings,
        ref bool crossingsTruncated,
        ref string currentParentId,
        ref int activeChild,
        ref bool underGrandparent)
    {
        double cursor = 0.0;
        int transitionBudget = 2 * children.Count + 2;
        while (cursor < 1.0 && transitionBudget-- > 0)
        {
            if (underGrandparent)
            {
                Vector3d start = Lerp(parentSamples[segment],
                    parentSamples[segment + 1], cursor);
                double? reentry = BoundaryFraction(
                    start, parentSamples[segment + 1], parentSoi, entering: true);
                if (reentry is null) break;
                double nextCursor = Advance(cursor, reentry.Value);
                if (DeferAtInteriorSample(nextCursor, segment, fractions.Count)) break;
                cursor = nextCursor;
                double reentryAt = GlobalFraction(fractions, segment, cursor);
                Record(crossings, ref crossingsTruncated,
                    new Crossing(parentId, reentryAt, Escape: false));
                currentParentId = parentId;
                underGrandparent = false;
                int contained = FirstStrictlyContainedChild(
                    children, segment, cursor, excludedChild: -1);
                if (contained >= 0)
                {
                    activeChild = contained;
                    currentParentId = children[contained].Id;
                    Record(crossings, ref crossingsTruncated,
                        new Crossing(currentParentId, reentryAt, Escape: false));
                }
                continue;
            }

            if (activeChild >= 0)
            {
                int exitedChild = activeChild;
                var child = children[activeChild];
                Vector3d start = Lerp(child.RelativeSamples[segment],
                    child.RelativeSamples[segment + 1], cursor);
                Vector3d end = child.RelativeSamples[segment + 1];
                double? exit = ExitFraction(start, end, child.SoiRadius);
                if (exit is null) break;
                double nextCursor = Advance(cursor, exit.Value);
                if (DeferAtInteriorSample(nextCursor, segment, fractions.Count)) break;
                // Stock's exit test is strict (> SOI). Reaching the boundary at
                // the final sample keeps the child parent; only a genuinely outside
                // endpoint may consume a root clamped to fraction one.
                if (nextCursor >= 1.0
                    && !StrictlyOutside(end, child.SoiRadius))
                    break;
                cursor = nextCursor;
                double exitAt = GlobalFraction(fractions, segment, cursor);
                Record(crossings, ref crossingsTruncated,
                    new Crossing(parentId, exitAt, Escape: true));
                currentParentId = parentId;
                activeChild = -1;

                Vector3d parentAtExit = Lerp(parentSamples[segment],
                    parentSamples[segment + 1], cursor);
                if (grandparentId is not null
                    && StrictlyOutside(parentAtExit, parentSoi))
                {
                    currentParentId = grandparentId;
                    underGrandparent = true;
                    Record(crossings, ref crossingsTruncated,
                        new Crossing(currentParentId, exitAt, Escape: true));
                    continue;
                }

                int contained = FirstStrictlyContainedChild(
                    children, segment, cursor, exitedChild);
                if (contained >= 0)
                {
                    activeChild = contained;
                    currentParentId = children[contained].Id;
                    Record(crossings, ref crossingsTruncated,
                        new Crossing(currentParentId, exitAt, Escape: false));
                }
                continue;
            }

            SegmentEvent? next = FindParentEvent(parentSamples, parentSoi,
                grandparentId, children, segment, cursor);
            if (next is null) break;
            double eventCursor = Advance(cursor, next.Value.LocalFraction);
            if (DeferAtInteriorSample(eventCursor, segment, fractions.Count)) break;
            // Parent escape is likewise strict at the final endpoint. Child entry
            // deliberately remains inclusive, matching Decide's <= child boundary.
            if (eventCursor >= 1.0 && next.Value.ParentEscape
                && !StrictlyOutside(parentSamples[segment + 1], parentSoi))
                break;
            cursor = eventCursor;
            double fraction = GlobalFraction(fractions, segment, cursor);
            if (next.Value.ParentEscape)
            {
                currentParentId = grandparentId!;
                underGrandparent = true;
                Record(crossings, ref crossingsTruncated,
                    new Crossing(currentParentId, fraction, Escape: true));
                continue;
            }
            activeChild = next.Value.ChildIndex;
            currentParentId = children[activeChild].Id;
            Record(crossings, ref crossingsTruncated,
                new Crossing(currentParentId, fraction, Escape: false));
        }
    }

    private static void Record(
        List<Crossing> crossings, ref bool truncated, Crossing crossing)
    {
        if (crossings.Count < MaxRecordedCrossings) crossings.Add(crossing);
        else truncated = true;
    }

    private static int FirstStrictlyContainedChild(
        IReadOnlyList<PolylineCandidate> children,
        int segment, double cursor, int excludedChild)
    {
        for (int i = 0; i < children.Count; i++)
        {
            if (i == excludedChild) continue;
            var child = children[i];
            Vector3d relative = Lerp(child.RelativeSamples[segment],
                child.RelativeSamples[segment + 1], cursor);
            if (StrictlyInside(relative, child.SoiRadius)) return i;
        }
        return -1;
    }

    private static double? ExitFraction(
        Vector3d start, Vector3d end, double radius)
    {
        double? direct = BoundaryFraction(start, end, radius, entering: false);
        if (direct is not null) return direct;
        // An entry root becomes the next exit when the chord is traversed backward.
        // This also tolerates the one-ulp outside reconstruction of a just-consumed
        // entry boundary without weakening ordinary direction classification.
        double? reverseEntry = BoundaryFraction(end, start, radius, entering: true);
        return reverseEntry is null ? null : 1.0 - reverseEntry.Value;
    }

    private static SegmentEvent? FindParentEvent(
        IReadOnlyList<Vector3d> parentSamples,
        double parentSoi,
        string? grandparentId,
        IReadOnlyList<PolylineCandidate> children,
        int segment,
        double cursor)
    {
        double? earliest = null;
        bool parentEscape = false;
        int childIndex = -1;
        if (grandparentId is not null)
        {
            Vector3d start = Lerp(parentSamples[segment],
                parentSamples[segment + 1], cursor);
            double? escape = BoundaryFraction(
                start, parentSamples[segment + 1], parentSoi, entering: false);
            if (escape is not null)
            {
                earliest = escape;
                parentEscape = true;
            }
        }

        for (int i = 0; i < children.Count; i++)
        {
            var child = children[i];
            Vector3d start = Lerp(child.RelativeSamples[segment],
                child.RelativeSamples[segment + 1], cursor);
            double? entry = BoundaryFraction(
                start, child.RelativeSamples[segment + 1],
                child.SoiRadius, entering: true);
            if (entry is null) continue;
            if (earliest is null || entry.Value < earliest.Value)
            {
                earliest = entry;
                parentEscape = false;
                childIndex = i;
            }
        }
        return earliest is null
            ? null
            : new SegmentEvent(earliest.Value, childIndex, parentEscape);
    }

    private static double Advance(double cursor, double localFraction) =>
        Math.Clamp(cursor + (1.0 - cursor) * localFraction, cursor, 1.0);

    /// <summary>An event exactly at a shared sample needs the next chord's direction
    /// before it can be called a crossing. Deferring preserves the current ownership;
    /// the following segment then emits fraction zero only when it actually proceeds
    /// through the boundary, while an outward cusp produces no transition.</summary>
    private static bool DeferAtInteriorSample(
        double cursor, int segment, int sampleCount) =>
        cursor >= 1.0 && segment + 2 < sampleCount;

    private static double GlobalFraction(
        IReadOnlyList<double> fractions, int segment, double localFraction) =>
        fractions[segment]
        + (fractions[segment + 1] - fractions[segment]) * localFraction;

    private static Vector3d Lerp(Vector3d start, Vector3d end, double fraction) =>
        start * (1.0 - fraction) + end * fraction;

    private static double? BoundaryFraction(
        Vector3d relativeStart, Vector3d relativeEnd, double radius, bool entering)
    {
        if (!(radius > 0) || !double.IsFinite(radius)
            || !Finite(relativeStart) || !Finite(relativeEnd))
            return null;

        // Normalize before subtraction so opposite near-double-limit endpoints do not
        // overflow the chord or discriminant. The roots are scale invariant.
        double scale = Math.Max(radius, Math.Max(MaxAbs(relativeStart), MaxAbs(relativeEnd)));
        if (!(scale > 0) || !double.IsFinite(scale)) return null;
        Vector3d start = relativeStart / scale;
        Vector3d end = relativeEnd / scale;
        Vector3d chord = end - start;
        double a = chord.LengthSquared();
        if (!(a > 0) || !double.IsFinite(a)) return null;

        double scaledRadius = radius / scale;
        double c = start.LengthSquared() - scaledRadius * scaledRadius;
        double b = start.Dot(chord);

        // No event if this segment begins on the post-crossing side. Boundary starts
        // count only when the chord heads through the sphere in the requested direction.
        if (entering)
        {
            if (c < 0) return null;
            if (c == 0) return b < 0 ? 0.0 : null;
        }
        else
        {
            if (c > 0) return null;
            if (c == 0 && b > 0) return 0.0;
            // From the boundary heading inward, continue to the later exit root. This
            // lets a catch-up sweep process entry then exit without rediscovering entry.
        }

        // Solve geometrically around the chord's closest approach. This avoids the
        // catastrophic cancellation in b*b-a*c when a high-warp chord is many orders
        // of magnitude longer than a small body's SOI.
        const double machineEpsilon = 2.2204460492503131e-16;
        double closestFraction = -b / a;
        Vector3d closest = start + chord * closestFraction;
        double radiusSquared = scaledRadius * scaledRadius;
        double closestSquared = closest.LengthSquared();
        double penetrationSquared = radiusSquared - closestSquared;
        double tangentTolerance = 32 * machineEpsilon
            * (radiusSquared + closestSquared);
        if (!(penetrationSquared > tangentTolerance)) return null;

        double halfSpan = Math.Sqrt(penetrationSquared / a);
        double lower = closestFraction - halfSpan;
        double upper = closestFraction + halfSpan;
        double fraction = entering ? lower : upper;

        const double endpointTolerance = 64 * machineEpsilon;
        if (fraction < -endpointTolerance || fraction > 1 + endpointTolerance)
            return null;
        return Math.Clamp(fraction, 0.0, 1.0);
    }

    private static bool Finite(Vector3d value) =>
        double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);

    private static bool StrictlyInside(Vector3d relative, double radius)
    {
        if (!(radius > 0) || !double.IsFinite(radius) || !Finite(relative))
            return false;
        double scale = Math.Max(radius, MaxAbs(relative));
        Vector3d normalized = relative / scale;
        double normalizedRadius = radius / scale;
        return normalized.LengthSquared() < normalizedRadius * normalizedRadius;
    }

    private static bool StrictlyOutside(Vector3d relative, double radius)
    {
        if (!(radius > 0) || !double.IsFinite(radius) || !Finite(relative))
            return false;
        double scale = Math.Max(radius, MaxAbs(relative));
        Vector3d normalized = relative / scale;
        double normalizedRadius = radius / scale;
        return normalized.LengthSquared() > normalizedRadius * normalizedRadius;
    }

    private static double MaxAbs(Vector3d value) =>
        Math.Max(Math.Abs(value.X), Math.Max(Math.Abs(value.Y), Math.Abs(value.Z)));

}

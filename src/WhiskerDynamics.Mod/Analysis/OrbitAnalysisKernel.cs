using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Analysis;

/// <summary>One scalar orbital element summarized over the analysed sampled arc.
/// Mean is time-weighted; Min/Max expose the real n-body excursion rather than
/// pretending the trajectory is one fixed conic. Current is the first usable
/// osculating value at or after the analysis epoch.</summary>
public readonly record struct OrbitElementStatistic(
    double Current, double Mean, double Minimum, double Maximum);

/// <summary>Osculating elements reduced to the quantities useful for flight design.
/// Angular values are radians. Undefined singular angles are NaN (not invented).</summary>
public sealed record OrbitElementSummary(
    OrbitElementStatistic SemiMajorAxis,
    OrbitElementStatistic Eccentricity,
    OrbitElementStatistic Inclination,
    OrbitElementStatistic LongitudeOfAscendingNode,
    OrbitElementStatistic ArgumentOfPeriapsis);

/// <summary>A measured event-to-event period. Spread is the population standard
/// deviation of the individual sampled cycles; CycleCount says how much evidence the
/// number rests on.</summary>
public readonly record struct OrbitPeriodEstimate(
    double MeanSeconds, double StandardDeviationSeconds,
    double MinimumSeconds, double MaximumSeconds, int CycleCount);

/// <summary>Least-squares secular angular rate over the sampled arc, with the RMS
/// residual around that line. Radians per second / radians respectively.</summary>
public readonly record struct OrbitPrecessionEstimate(
    double RadiansPerSecond, double ResidualRadians, int SampleCount);

/// <summary>Best observed closure of the body-fixed longitude at successive
/// ascending nodes. ObservedWindows states how many q-orbit comparisons support
/// the reported RMS ClosureErrorRadians.</summary>
public sealed record GroundTrackRecurrence(
    int OrbitCount, int RelativeTrackTurns, int ObservedWindows, double DurationSeconds,
    double LongitudeShiftPerOrbitRadians, double ClosureErrorRadians);

public readonly record struct OrbitTrendPoint(
    double TimeSeconds, double? PeriapsisAltitudeMeters, double? ApoapsisAltitudeMeters,
    double Eccentricity, double InclinationRadians);

/// <summary>Immutable worker-produced analysis attached to one overlay batch.</summary>
public sealed record OrbitAnalysisReport
{
    public required string BodyId { get; init; }
    public required double StartTimeSeconds { get; init; }
    public required double EndTimeSeconds { get; init; }
    public required int SampleCount { get; init; }
    public required double EstimatedRevolutions { get; init; }
    public required OrbitElementSummary Elements { get; init; }
    public OrbitElementStatistic? PeriapsisAltitude { get; init; }
    public OrbitElementStatistic? ApoapsisAltitude { get; init; }
    public double? LowestSampledAltitudeMeters { get; init; }
    public double? FirstSurfaceCrossingTimeSeconds { get; init; }
    public required IReadOnlyList<OrbitTrendPoint> Trend { get; init; }
    public OrbitPeriodEstimate? SiderealPeriod { get; init; }
    public OrbitPeriodEstimate? NodalPeriod { get; init; }
    public OrbitPeriodEstimate? AnomalisticPeriod { get; init; }
    public OrbitPrecessionEstimate? NodalPrecession { get; init; }
    public OrbitPrecessionEstimate? ArgumentOfPeriapsisPrecession { get; init; }
    public OrbitPrecessionEstimate? LongitudeOfPeriapsisPrecession { get; init; }
    public GroundTrackRecurrence? GroundTrack { get; init; }
    public required IReadOnlyList<string> Notes { get; init; }
}

/// <summary>
/// Pure sampled-series orbit analyser. It consumes the same parent-relative predictor
/// polyline the overlay publishes together with exact predictor-minus-parent
/// velocities (the position-only overload is an offline convenience). No game
/// object, conic patch, or new physics
/// enters the result. Event periods come from actual crossings/extrema, while secular
/// rates are regressions over osculating elements.
/// </summary>
public static class OrbitAnalysisKernel
{
    private const double Tau = 2 * Math.PI;
    private const double EccentricAngleFloor = 1e-5;
    private const double NodeAngleFloor = 1e-7;
    private const int MaxRecurrenceOrbits = 500;
    private const int MaxTrendPoints = 512;
    private const double MaxKeptTurnRadians = Math.PI / 36; // 5 degrees

    private readonly record struct NodePass(double Time, double Longitude);

    private readonly record struct Point(
        double Time, Vector3d R, Vector3d V, double Radius, double RadialVelocity,
        double SemiMajorAxis, double Eccentricity, double Inclination,
        double PeriapsisRadius, double? ApoapsisRadius,
        double Node, double PeriapsisArgument, double SiderealPhase);

    /// <summary>Analyse the interval [start,end] of a strictly chronological sampled
    /// parent-relative trajectory. Returns null when there are fewer than three valid
    /// points or the central-body/pole inputs cannot define honest elements.</summary>
    public static OrbitAnalysisReport? Analyze(
        string bodyId, double[] times, Vector3d[] positions,
        double startTimeSeconds, double endTimeSeconds, double mu,
        double meanRadiusMeters, Vector3d equatorialPole,
        double? angularVelocityRadiansPerSecond = null)
    {
        if (times.Length != positions.Length || times.Length < 3) return null;
        var velocities = new Vector3d[times.Length];
        for (int i = 0; i < times.Length; i++)
            velocities[i] = Derivative(times, positions, i);
        return Analyze(bodyId, times, positions, velocities, startTimeSeconds,
            endTimeSeconds, mu, meanRadiusMeters, equatorialPole,
            angularVelocityRadiansPerSecond);
    }

    /// <summary>Production overload: exact predictor-minus-parent velocities avoid
    /// manufacturing eccentricity and apsidal motion from polyline differentiation.</summary>
    public static OrbitAnalysisReport? Analyze(
        string bodyId, double[] times, Vector3d[] positions, Vector3d[] velocities,
        double startTimeSeconds, double endTimeSeconds, double mu,
        double meanRadiusMeters, Vector3d equatorialPole,
        double? angularVelocityRadiansPerSecond = null,
        Action<double>? progress = null,
        Func<bool>? shouldStop = null)
    {
        if (times.Length != positions.Length || times.Length != velocities.Length
            || times.Length < 3 || !double.IsFinite(mu) || mu <= 0
            || !double.IsFinite(startTimeSeconds) || !double.IsFinite(endTimeSeconds)
            || endTimeSeconds <= startTimeSeconds)
            return null;        double poleLength = equatorialPole.Length();
        if (!double.IsFinite(poleLength) || poleLength <= 0) return null;
        var pole = equatorialPole / poleLength;
        var reference = ReferenceAxis(pole);
        bool radiusKnown = double.IsFinite(meanRadiusMeters) && meanRadiusMeters > 0;

        int first = LowerBound(times, startTimeSeconds);
        int last = UpperBound(times, endTimeSeconds) - 1;
        if (last - first + 1 < 3) return null;
        int sourceCount = last - first + 1;
        var points = new List<Point>(Math.Min(sourceCount, 16384));
        Vector3d? lastKeptDirection = null;
        double previousRadial = double.NaN, previousPlane = double.NaN;
        double previousSurfaceAltitude = double.NaN;
        double previousSurfaceTime = double.NaN;
        double? lowestSampledAltitude = null;
        double? firstSurfaceCrossing = null;
        for (int i = first; i <= last; i++)
        {
            if ((i - first & 255) == 0)
            {
                if (shouldStop?.Invoke() == true)
                    throw new OperationCanceledException();
                progress?.Invoke((double)(i - first) / Math.Max(1, last - first));
            }
            double t = times[i];
            if (!double.IsFinite(t) || i > 0 && t <= times[i - 1]) continue;
            Vector3d r = positions[i], v = velocities[i];
            if (!Finite(r) || !Finite(v)) continue;
            double radius = r.Length();
            if (!(radius > 0) || !double.IsFinite(radius)) continue;
            var direction = r / radius;
            double radial = r.Dot(v) / radius;
            double plane = r.Dot(pole);
            double surfaceAltitude = radiusKnown ? radius - meanRadiusMeters : double.NaN;
            if (radiusKnown)
            {
                lowestSampledAltitude = lowestSampledAltitude is { } lowest
                    ? Math.Min(lowest, surfaceAltitude) : surfaceAltitude;
                if (firstSurfaceCrossing is null)
                {
                    if (!double.IsFinite(previousSurfaceTime) && surfaceAltitude <= 0)
                        firstSurfaceCrossing = t;
                    else if (previousSurfaceAltitude > 0 && surfaceAltitude <= 0)
                    {
                        double fraction = previousSurfaceAltitude
                            / (previousSurfaceAltitude - surfaceAltitude);
                        firstSurfaceCrossing = previousSurfaceTime
                            + fraction * (t - previousSurfaceTime);
                    }
                }
            }
            bool eventBracket = double.IsFinite(previousRadial)
                && (Math.Sign(previousRadial) != Math.Sign(radial)
                    || Math.Sign(previousPlane) != Math.Sign(plane)
                    || double.IsFinite(previousSurfaceAltitude)
                        && Math.Sign(previousSurfaceAltitude) != Math.Sign(surfaceAltitude));
            bool keep = points.Count == 0 || i == last || eventBracket
                || lastKeptDirection is not { } kept
                || Math.Acos(Math.Clamp(kept.Dot(direction), -1.0, 1.0)) >= MaxKeptTurnRadians;
            previousRadial = radial;
            previousPlane = plane;
            previousSurfaceAltitude = surfaceAltitude;
            previousSurfaceTime = t;
            if (!keep) continue;

            var h = r.Cross(v);
            double hLength = h.Length();
            if (!(hLength > 0) || !double.IsFinite(hLength)) continue;
            var hHat = h / hLength;
            var eVector = v.Cross(h) / mu - r / radius;
            double eccentricity = eVector.Length();
            double energy = 0.5 * v.LengthSquared() - mu / radius;
            double a = Math.Abs(energy) <= mu / radius * 1e-14
                ? double.PositiveInfinity : -mu / (2 * energy);
            double semiLatusRectum = hLength * hLength / mu;
            double periapsisRadius = semiLatusRectum / (1 + eccentricity);
            double? apoapsisRadius = eccentricity < 1
                ? semiLatusRectum / (1 - eccentricity)
                : null;
            double inclination = Math.Acos(Math.Clamp(hHat.Dot(pole), -1.0, 1.0));
            var nodeVector = pole.Cross(hHat);
            double nodeLength = nodeVector.Length();
            double node = nodeLength > NodeAngleFloor
                ? PositiveAngle(reference, nodeVector / nodeLength, pole)
                : double.NaN;
            double periapsis = eccentricity > EccentricAngleFloor && nodeLength > NodeAngleFloor
                ? PositiveAngle(nodeVector / nodeLength, eVector / eccentricity, hHat)
                : double.NaN;

            // Sidereal phase is inertial LAN plus argument of latitude. Circular
            // and equatorial orbits remain measurable although LAN and argument of
            // periapsis separately have no physical meaning there.
            double phase;
            if (nodeLength > NodeAngleFloor)
            {
                double u = PositiveAngle(nodeVector / nodeLength, direction, hHat);
                phase = node + u;
            }
            else
            {
                phase = PositiveAngle(reference, direction, hHat.Dot(pole) >= 0 ? pole : -pole);
            }
            points.Add(new Point(t, r, v, radius, radial,
                a, eccentricity, inclination, periapsisRadius, apoapsisRadius,
                node, periapsis, phase));
            lastKeptDirection = direction;
        }
        ThrowIfCancellationRequested(shouldStop);
        progress?.Invoke(1);
        if (points.Count < 3) return null;

        // Remove the stencil-only brackets from the reporting/fit window while
        // retaining them for the first crossing interpolation.
        int reportFirst = -1;
        for (int i = 0; i < points.Count; i++)
        {
            PollCancellation(shouldStop, i);
            if (points[i].Time < startTimeSeconds) continue;
            reportFirst = i;
            break;
        }
        if (reportFirst < 0) return null;
        int reportLast = -1;
        for (int i = points.Count - 1; i >= reportFirst; i--)
        {
            PollCancellation(shouldStop, points.Count - 1 - i);
            if (points[i].Time > endTimeSeconds) continue;
            reportLast = i;
            break;
        }
        if (reportLast - reportFirst + 1 < 2) return null;

        var rawPhases = new double[points.Count];
        for (int i = 0; i < points.Count; i++)
        {
            PollCancellation(shouldStop, i);
            rawPhases[i] = points[i].SiderealPhase;
        }
        double[] phases = Unwrap(rawPhases, shouldStop);
        double revolutions = Math.Abs(phases[reportLast] - phases[reportFirst]) / Tau;
        var siderealEvents = PhaseCrossings(
            points, phases, reportFirst, reportLast, shouldStop);
        var nodePasses = NodeCrossings(
            points, reportFirst, reportLast, pole, reference, shouldStop);
        var nodeEvents = new List<double>(nodePasses.Count);
        for (int i = 0; i < nodePasses.Count; i++)
        {
            PollCancellation(shouldStop, i);
            nodeEvents.Add(nodePasses[i].Time);
        }
        var apsisEvents = ZeroCrossings(points, reportFirst, reportLast,
            p => p.RadialVelocity, ascending: true, shouldStop);
        RemoveEventsOutside(siderealEvents,
            startTimeSeconds, endTimeSeconds, shouldStop);
        RemoveEventsOutside(nodeEvents,
            startTimeSeconds, endTimeSeconds, shouldStop);
        RemoveEventsOutside(apsisEvents,
            startTimeSeconds, endTimeSeconds, shouldStop);

        var valid = new List<Point>(reportLast - reportFirst + 1);
        for (int i = reportFirst; i <= reportLast; i++)
        {
            PollCancellation(shouldStop, i - reportFirst);
            valid.Add(points[i]);
        }
        var notes = new List<string>();
        OrbitElementSummary elements = new(
            Statistic(valid, p => p.SemiMajorAxis, circular: false, shouldStop),
            Statistic(valid, p => p.Eccentricity, circular: false, shouldStop),
            Statistic(valid, p => p.Inclination, circular: false, shouldStop),
            Statistic(valid, p => p.Node, circular: true, shouldStop),
            Statistic(valid, p => p.PeriapsisArgument, circular: true, shouldStop));
        OrbitElementStatistic? periapsisAltitude = radiusKnown
            ? OptionalStatistic(valid,
                p => p.PeriapsisRadius - meanRadiusMeters, shouldStop)
            : null;
        OrbitElementStatistic? apoapsisAltitude = radiusKnown
            ? OptionalStatistic(valid, p => p.ApoapsisRadius is { } radius
                ? radius - meanRadiusMeters : double.NaN, shouldStop)
            : null;
        if (!radiusKnown)
            notes.Add("altitudes and surface crossing unavailable: mean body radius was not captured");

        OrbitPrecessionEstimate? nodal = AngularRegression(
            valid, p => p.Node, shouldStop);
        OrbitPrecessionEstimate? argumentRate = AngularRegression(
            valid, p => p.PeriapsisArgument, shouldStop);
        OrbitPrecessionEstimate? longitudeRate = AngularRegression(valid, p =>
            double.IsFinite(p.Node) && double.IsFinite(p.PeriapsisArgument)
                ? p.Node + p.PeriapsisArgument : double.NaN, shouldStop);
        if (nodal is null) notes.Add("nodal precession undefined for an equatorial orbit");
        if (argumentRate is null)
            notes.Add("argument-of-periapsis precession undefined for a circular/equatorial orbit");
        if (longitudeRate is null)
            notes.Add("longitude-of-periapsis precession undefined for a circular/equatorial orbit");
        var sidereal = Periods(siderealEvents, shouldStop);
        var nodalPeriod = Periods(nodeEvents, shouldStop);
        var anomalistic = elements.Eccentricity.Mean >= EccentricAngleFloor
            ? Periods(apsisEvents, shouldStop)
            : null;
        if (sidereal is null) notes.Add("sidereal period needs at least two completed revolutions");
        if (nodalPeriod is null) notes.Add("nodal period needs at least two ascending-node passages");
        if (anomalistic is null) notes.Add("anomalistic period needs at least two periapsis passages");

        GroundTrackRecurrence? recurrence = null;
        if (angularVelocityRadiansPerSecond is { } spin && double.IsFinite(spin)
            && Math.Abs(spin) > 0 && nodeEvents.Count >= 2)
            recurrence = Recurrence(nodePasses, spin, shouldStop);
        else if (angularVelocityRadiansPerSecond is null || !double.IsFinite(angularVelocityRadiansPerSecond.Value))
            notes.Add("ground-track recurrence unavailable: body spin was not captured");
        else if (Math.Abs(angularVelocityRadiansPerSecond.Value) == 0)
            notes.Add("ground-track recurrence unavailable for a non-rotating body");
        else
            notes.Add("ground-track recurrence needs two ascending-node passages");

        var trend = Trend(valid, radiusKnown ? meanRadiusMeters : null, shouldStop);
        ThrowIfCancellationRequested(shouldStop);
        return new OrbitAnalysisReport
        {
            BodyId = bodyId,
            StartTimeSeconds = valid[0].Time,
            EndTimeSeconds = valid[^1].Time,
            SampleCount = valid.Count,
            EstimatedRevolutions = revolutions,
            Elements = elements,
            PeriapsisAltitude = periapsisAltitude,
            ApoapsisAltitude = apoapsisAltitude,
            LowestSampledAltitudeMeters = lowestSampledAltitude,
            FirstSurfaceCrossingTimeSeconds = firstSurfaceCrossing,
            Trend = trend,
            SiderealPeriod = sidereal,
            NodalPeriod = nodalPeriod,
            AnomalisticPeriod = anomalistic,
            NodalPrecession = nodal,
            ArgumentOfPeriapsisPrecession = argumentRate,
            LongitudeOfPeriapsisPrecession = longitudeRate,
            GroundTrack = recurrence,
            Notes = notes,
        };
    }

    private static OrbitElementStatistic? OptionalStatistic(
        IReadOnlyList<Point> points, Func<Point, double> selector,
        Func<bool>? shouldStop)
    {
        var statistic = Statistic(points, selector, circular: false, shouldStop);
        return double.IsFinite(statistic.Mean) ? statistic : null;
    }

    private static IReadOnlyList<OrbitTrendPoint> Trend(
        IReadOnlyList<Point> points, double? meanRadius, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        int count = Math.Min(points.Count, MaxTrendPoints);
        var result = new OrbitTrendPoint[count];
        for (int i = 0; i < count; i++)
        {
            PollCancellation(shouldStop, i);
            int source = count == 1 ? 0
                : (int)Math.Round((double)i * (points.Count - 1) / (count - 1));
            Point point = points[source];
            double? periapsis = meanRadius is { } radius
                ? point.PeriapsisRadius - radius : null;
            double? apoapsis = meanRadius is { } radius2 && point.ApoapsisRadius is { } ap
                ? ap - radius2 : null;
            result[i] = new(point.Time, periapsis, apoapsis,
                point.Eccentricity, point.Inclination);
        }
        return result;
    }

    private static OrbitElementStatistic Statistic(
        IReadOnlyList<Point> points, Func<Point, double> selector, bool circular,
        Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        var values = new double[points.Count];
        int currentIndex = -1;
        for (int i = 0; i < points.Count; i++)
        {
            PollCancellation(shouldStop, i);
            values[i] = selector(points[i]);
            if (currentIndex < 0 && double.IsFinite(values[i])) currentIndex = i;
        }
        if (currentIndex < 0) return new(double.NaN, double.NaN, double.NaN, double.NaN);
        double current = values[currentIndex];
        if (circular)
        {
            double sx = 0, sy = 0, weight = 0;
            for (int i = 0; i < values.Length; i++)
            {
                PollCancellation(shouldStop, i);
                if (!double.IsFinite(values[i])) continue;
                double w = TimeWeight(points, i);
                sx += w * Math.Cos(values[i]); sy += w * Math.Sin(values[i]); weight += w;
            }
            double mean = weight > 0 ? Positive(Math.Atan2(sy, sx)) : double.NaN;
            var unwrapped = UnwrapAround(values, mean, shouldStop);
            double circularMin = double.PositiveInfinity;
            double circularMax = double.NegativeInfinity;
            for (int i = 0; i < unwrapped.Length; i++)
            {
                PollCancellation(shouldStop, i);
                if (!double.IsFinite(unwrapped[i])) continue;
                circularMin = Math.Min(circularMin, unwrapped[i]);
                circularMax = Math.Max(circularMax, unwrapped[i]);
            }
            return new(current, mean, circularMin, circularMax);
        }
        double sum = 0, total = 0, min = double.PositiveInfinity, max = double.NegativeInfinity;
        for (int i = 0; i < values.Length; i++)
        {
            PollCancellation(shouldStop, i);
            double value = values[i];
            if (!double.IsFinite(value)) continue;
            double w = TimeWeight(points, i);
            sum += w * value; total += w;
            min = Math.Min(min, value); max = Math.Max(max, value);
        }
        return new(current, total > 0 ? sum / total : double.NaN, min, max);
    }

    private static double TimeWeight(IReadOnlyList<Point> points, int i)
    {
        if (points.Count == 1) return 1;
        if (i == 0) return (points[1].Time - points[0].Time) / 2;
        if (i == points.Count - 1) return (points[^1].Time - points[^2].Time) / 2;
        return (points[i + 1].Time - points[i - 1].Time) / 2;
    }

    private static OrbitPrecessionEstimate? AngularRegression(
        IReadOnlyList<Point> points, Func<Point, double> selector,
        Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        // Equatorial/circular arcs intentionally carry NaN for undefined angles.
        // Keep the first two finite observations inline and allocate only when a
        // regression is actually possible. This preserves the original one-pass
        // selector cost for defined angles while allocating nothing when <3 exist.
        Point firstPoint = default, secondPoint = default;
        double firstValue = 0, secondValue = 0;
        int finiteCount = 0;
        List<Point>? finitePoints = null;
        List<double>? values = null;
        for (int pointIndex = 0; pointIndex < points.Count; pointIndex++)
        {
            PollCancellation(shouldStop, pointIndex);
            Point point = points[pointIndex];
            double value = selector(point);
            if (!double.IsFinite(value)) continue;
            if (finiteCount == 0)
            {
                firstPoint = point;
                firstValue = value;
            }
            else if (finiteCount == 1)
            {
                secondPoint = point;
                secondValue = value;
            }
            else
            {
                if (finitePoints is null)
                {
                    finitePoints = new List<Point>(points.Count);
                    values = new List<double>(points.Count);
                    finitePoints.Add(firstPoint);
                    finitePoints.Add(secondPoint);
                    values.Add(firstValue);
                    values.Add(secondValue);
                }
                finitePoints.Add(point);
                values!.Add(value);
            }
            finiteCount++;
        }
        if (finitePoints is null) return null;
        double[] angles = Unwrap(values!, shouldStop);
        double t0 = finitePoints[0].Time;
        double totalWeight = 0, sumT = 0, sumA = 0;
        for (int i = 0; i < finitePoints.Count; i++)
        {
            PollCancellation(shouldStop, i);
            double weight = TimeWeight(finitePoints, i);
            totalWeight += weight;
            sumT += weight * (finitePoints[i].Time - t0);
            sumA += weight * angles[i];
        }
        if (!(totalWeight > 0)) return null;
        double meanT = sumT / totalWeight, meanA = sumA / totalWeight;
        double denominator = 0, numerator = 0;
        for (int i = 0; i < finitePoints.Count; i++)
        {
            PollCancellation(shouldStop, i);
            double weight = TimeWeight(finitePoints, i);
            double dt = finitePoints[i].Time - t0 - meanT;
            denominator += weight * dt * dt;
            numerator += weight * dt * (angles[i] - meanA);
        }
        if (!(denominator > 0)) return null;
        double slope = numerator / denominator;
        double residualSq = 0;
        for (int i = 0; i < finitePoints.Count; i++)
        {
            PollCancellation(shouldStop, i);
            double fit = meanA + slope * (finitePoints[i].Time - t0 - meanT);
            double residual = angles[i] - fit;
            residualSq += TimeWeight(finitePoints, i) * residual * residual;
        }
        return new(slope, Math.Sqrt(residualSq / totalWeight), finitePoints.Count);
    }
    private static List<double> PhaseCrossings(
        IReadOnlyList<Point> points, double[] unwrapped, int first, int last,
        Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        var events = new List<double>();
        double direction = unwrapped[last] >= unwrapped[first] ? 1 : -1;
        double PhaseAt(int index) => direction * unwrapped[index];
        double next = Math.Ceiling(PhaseAt(first) / Tau) * Tau;
        if (next <= PhaseAt(first) + 1e-12) next += Tau;
        for (int i = Math.Max(1, first); i <= last && next <= PhaseAt(last); i++)
        {
            PollCancellation(shouldStop, i - Math.Max(1, first));
            double phase = PhaseAt(i), previousPhase = PhaseAt(i - 1);
            if (phase < next || phase == previousPhase) continue;
            double f = (next - previousPhase) / (phase - previousPhase);
            events.Add(points[i - 1].Time + f * (points[i].Time - points[i - 1].Time));
            next += Tau;
        }
        return events;
    }

    private static List<double> ZeroCrossings(
        IReadOnlyList<Point> points, int first, int last,
        Func<Point, double> selector, bool ascending, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        var events = new List<double>();
        for (int i = Math.Max(1, first); i <= last; i++)
        {
            PollCancellation(shouldStop, i - Math.Max(1, first));
            double a = selector(points[i - 1]), b = selector(points[i]);
            bool crossed = ascending ? a <= 0 && b > 0 : a >= 0 && b < 0;
            if (!crossed || !double.IsFinite(a) || !double.IsFinite(b) || a == b) continue;
            double f = -a / (b - a);
            events.Add(points[i - 1].Time + f * (points[i].Time - points[i - 1].Time));
        }
        return events;
    }

    private static void RemoveEventsOutside(List<double> events,
        double startTimeSeconds, double endTimeSeconds, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        int kept = 0;
        for (int i = 0; i < events.Count; i++)
        {
            PollCancellation(shouldStop, i);
            double value = events[i];
            if (value < startTimeSeconds || value > endTimeSeconds) continue;
            events[kept++] = value;
        }
        if (kept < events.Count) events.RemoveRange(kept, events.Count - kept);
    }

    private static List<NodePass> NodeCrossings(IReadOnlyList<Point> points,
        int first, int last, Vector3d pole, Vector3d reference,
        Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        var events = new List<NodePass>();
        for (int i = Math.Max(1, first); i <= last; i++)
        {
            PollCancellation(shouldStop, i - Math.Max(1, first));
            double a = points[i - 1].R.Dot(pole), b = points[i].R.Dot(pole);
            if (!(a <= 0 && b > 0) || !double.IsFinite(a) || !double.IsFinite(b) || a == b)
                continue;
            double f = -a / (b - a);
            double time = points[i - 1].Time + f * (points[i].Time - points[i - 1].Time);
            var crossing = points[i - 1].R * (1 - f) + points[i].R * f;
            var projected = crossing - pole * crossing.Dot(pole);
            double length = projected.Length();
            if (!(length > 0) || !double.IsFinite(length)) continue;
            events.Add(new NodePass(time, PositiveAngle(reference, projected / length, pole)));
        }
        return events;
    }
    private static OrbitPeriodEstimate? Periods(
        IReadOnlyList<double> events, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        if (events.Count < 2) return null;
        int count = 0;
        double sum = 0, min = double.PositiveInfinity, max = double.NegativeInfinity;
        for (int i = 0; i < events.Count - 1; i++)
        {
            PollCancellation(shouldStop, i);
            double value = events[i + 1] - events[i];
            if (!double.IsFinite(value) || value <= 0) continue;
            count++;
            sum += value;
            min = Math.Min(min, value);
            max = Math.Max(max, value);
        }
        if (count == 0) return null;
        double mean = sum / count;
        double varianceSum = 0;
        for (int i = 0; i < events.Count - 1; i++)
        {
            PollCancellation(shouldStop, i);
            double value = events[i + 1] - events[i];
            if (!double.IsFinite(value) || value <= 0) continue;
            varianceSum += (value - mean) * (value - mean);
        }
        return new(mean, Math.Sqrt(varianceSum / count), min, max, count);
    }

    private static GroundTrackRecurrence Recurrence(
        IReadOnlyList<NodePass> nodes, double spin, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        var wrappedGround = new double[nodes.Count];
        for (int i = 0; i < nodes.Count; i++)
        {
            PollCancellation(shouldStop, i);
            wrappedGround[i] = Positive(nodes[i].Longitude - spin * nodes[i].Time);
        }
        double[] ground = Unwrap(wrappedGround, shouldStop);
        double shift = 0;
        for (int i = 1; i < ground.Length; i++)
        {
            PollCancellation(shouldStop, i - 1);
            shift += WrapSigned(ground[i] - ground[i - 1]);
        }
        shift /= ground.Length - 1;

        int bestOrbits = 1, bestCycles = 0, bestWindows = 0;
        double bestDuration = nodes[1].Time - nodes[0].Time;
        double bestError = double.PositiveInfinity;
        int limit = Math.Min(MaxRecurrenceOrbits, nodes.Count - 1);
        for (int q = 1; q <= limit; q++)
        {
            ThrowIfCancellationRequested(shouldStop);
            int windows = nodes.Count - q;
            double errorSq = 0, duration = 0, deltaSum = 0;
            for (int i = 0; i < windows; i++)
            {
                PollCancellation(shouldStop, i);
                double delta = ground[i + q] - ground[i];
                double closure = WrapSigned(delta);
                errorSq += closure * closure;
                duration += nodes[i + q].Time - nodes[i].Time;
                deltaSum += delta;
            }
            double error = Math.Sqrt(errorSq / windows);
            if (error < bestError - 1e-12)
            {
                bestError = error;
                bestOrbits = q;
                bestWindows = windows;
                bestDuration = duration / windows;
                bestCycles = (int)Math.Round(deltaSum / windows / Tau);
            }
        }
        return new(bestOrbits, bestCycles, bestWindows, bestDuration, shift, bestError);
    }
    private static Vector3d Derivative(double[] times, Vector3d[] positions, int index)
    {
        int a, b, c;
        if (index <= 0) { a = 0; b = 1; c = 2; }
        else if (index >= times.Length - 1) { a = times.Length - 3; b = times.Length - 2; c = times.Length - 1; }
        else { a = index - 1; b = index; c = index + 1; }
        double t = times[index], ta = times[a], tb = times[b], tc = times[c];
        double da = (ta - tb) * (ta - tc), db = (tb - ta) * (tb - tc), dc = (tc - ta) * (tc - tb);
        if (da == 0 || db == 0 || dc == 0) return new(double.NaN, double.NaN, double.NaN);
        double wa = (2 * t - tb - tc) / da;
        double wb = (2 * t - ta - tc) / db;
        double wc = (2 * t - ta - tb) / dc;
        return positions[a] * wa + positions[b] * wb + positions[c] * wc;
    }

    private static Vector3d ReferenceAxis(Vector3d pole)
    {
        var x = new Vector3d(1, 0, 0);
        var projected = x - pole * x.Dot(pole);
        if (projected.LengthSquared() < 1e-12)
        {
            var y = new Vector3d(0, 1, 0);
            projected = y - pole * y.Dot(pole);
        }
        return projected.Normalized();
    }

    private static double PositiveAngle(Vector3d from, Vector3d to, Vector3d normal) =>
        Positive(Math.Atan2(normal.Dot(from.Cross(to)), Math.Clamp(from.Dot(to), -1.0, 1.0)));

    private static double[] Unwrap(
        IReadOnlyList<double> values, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        if (values.Count == 0) return [];
        var result = new double[values.Count];
        result[0] = values[0];
        for (int i = 1; i < values.Count; i++)
        {
            PollCancellation(shouldStop, i);
            if (!double.IsFinite(values[i]) || !double.IsFinite(result[i - 1]))
            {
                result[i] = values[i];
                continue;
            }
            result[i] = result[i - 1] + WrapSigned(values[i] - values[i - 1]);
        }
        return result;
    }

    private static double[] UnwrapAround(
        double[] values, double center, Func<bool>? shouldStop)
    {
        ThrowIfCancellationRequested(shouldStop);
        var result = new double[values.Length];
        for (int i = 0; i < values.Length; i++)
        {
            PollCancellation(shouldStop, i);
            result[i] = double.IsFinite(values[i]) ? center + WrapSigned(values[i] - center) : double.NaN;
        }
        return result;
    }

    private static void PollCancellation(Func<bool>? shouldStop, int index)
    {
        if ((index & 255) == 0) ThrowIfCancellationRequested(shouldStop);
    }

    private static void ThrowIfCancellationRequested(Func<bool>? shouldStop)
    {
        if (shouldStop?.Invoke() == true)
            throw new OperationCanceledException();
    }

    private static double Positive(double angle)
    {
        angle %= Tau;
        return angle < 0 ? angle + Tau : angle;
    }

    private static double WrapSigned(double angle)
    {
        angle = Positive(angle);
        return angle > Math.PI ? angle - Tau : angle;
    }

    private static bool Finite(Vector3d v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);

    private static int LowerBound(double[] values, double target)
    {
        int lo = 0, hi = values.Length;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            if (values[mid] < target) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private static int UpperBound(double[] values, double target)
    {
        int lo = 0, hi = values.Length;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            if (values[mid] <= target) lo = mid + 1; else hi = mid;
        }
        return lo;
    }
}

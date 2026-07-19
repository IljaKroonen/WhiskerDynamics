namespace WhiskerDynamics.Mod.Ui;

internal enum OrbitDashboardWarningKind
{
    SurfaceCrossing,
    UnresolvedSurfaceCrossing,
}

internal sealed record OrbitDashboardWarning(
    OrbitDashboardWarningKind Kind, string Text);

internal sealed record OrbitDashboardMetric(string Label, string Value);

internal sealed record OrbitDashboardStatisticRow(
    string Label, string Current, string Mean, string Range);

internal sealed record OrbitDashboardValueRow(
    string Label, string Value, string Evidence);

internal sealed record OrbitDashboardPresentation(
    string Description,
    string RequestedInterval,
    string CoveredInterval,
    double CoverageFraction,
    OrbitDashboardWarning? Warning,
    IReadOnlyList<OrbitDashboardMetric> Summary,
    IReadOnlyList<OrbitDashboardStatisticRow> Elements,
    IReadOnlyList<OrbitDashboardValueRow> Periods,
    IReadOnlyList<OrbitDashboardValueRow> Precession,
    IReadOnlyList<OrbitDashboardValueRow> GroundTrack,
    IReadOnlyList<string> Limits);

internal static class OrbitAnalysisPresentationModel
{
    private const double RadiansToDegrees = 180.0 / Math.PI;
    private const double SecondsPerDay = 86400.0;

    internal static OrbitDashboardPresentation Create(
        OrbitAnalysisReport report, double requestedStartOffsetSeconds,
        double requestedSpanSeconds)
    {
        string requested = $"T+{Duration(requestedStartOffsetSeconds)}"
            + $" for {Duration(requestedSpanSeconds)}";
        double coveredSeconds = report.EndTimeSeconds - report.StartTimeSeconds;
        string covered = $"{Duration(coveredSeconds)} | {report.SampleCount:N0} samples"
            + $" | {report.EstimatedRevolutions:F2} rev";
        double coverage = double.IsFinite(requestedSpanSeconds) && requestedSpanSeconds > 0
            ? Math.Clamp(coveredSeconds / requestedSpanSeconds, 0, 1)
            : 1;

        return new(
            Classify(report.BodyId, report.Elements.Eccentricity.Mean,
                report.Elements.Inclination.Mean),
            requested,
            covered,
            coverage,
            Warning(report),
            [
                new("Mean Pe", Mean(report.PeriapsisAltitude, Length)),
                new("Mean Ap", Mean(report.ApoapsisAltitude, Length)),
                new("Lowest altitude", Length(report.LowestSampledAltitudeMeters)),
                new("Sidereal period", report.SiderealPeriod is { } sidereal
                    ? Duration(sidereal.MeanSeconds) : "not resolved"),
                new("Inclination", Angle(report.Elements.Inclination.Mean)),
                new("Eccentricity", Scalar(report.Elements.Eccentricity.Mean)),
            ],
            [
                Statistic("Pe altitude", report.PeriapsisAltitude, Length),
                Statistic("Ap altitude", report.ApoapsisAltitude, Length),
                Statistic("semi-major axis", report.Elements.SemiMajorAxis, Length),
                Statistic("eccentricity", report.Elements.Eccentricity, Scalar),
                Statistic("inclination", report.Elements.Inclination, Angle),
                Statistic("LAN", report.Elements.LongitudeOfAscendingNode, Angle),
                Statistic("argument of Pe", report.Elements.ArgumentOfPeriapsis, Angle),
            ],
            [
                Period("sidereal", report.SiderealPeriod),
                Period("nodal", report.NodalPeriod),
                Period("anomalistic", report.AnomalisticPeriod),
            ],
            [
                Rate("node", report.NodalPrecession),
                Rate("argument of Pe", report.ArgumentOfPeriapsisPrecession),
                Rate("longitude of Pe", report.LongitudeOfPeriapsisPrecession),
            ],
            GroundTrack(report.GroundTrack),
            report.Notes);
    }

    internal static string Classify(
        string bodyId, double eccentricity, double inclinationRadians)
    {
        var words = new List<string>();
        if (double.IsFinite(eccentricity))
        {
            words.Add(eccentricity < 0.01 ? "circular"
                : eccentricity > 0.5 ? "highly elliptical"
                : "elliptical");
        }
        if (double.IsFinite(inclinationRadians))
        {
            double degrees = inclinationRadians * RadiansToDegrees;
            double equatorDistance = Math.Min(degrees, Math.Abs(180 - degrees));
            if (equatorDistance <= 5 + 1e-10) words.Add("equatorial");
            else if (degrees >= 80 - 1e-10 && degrees <= 100 + 1e-10)
                words.Add("polar");
            else words.Add("inclined");
            if (degrees > 90) words.Add("retrograde");
        }
        words.Add("orbit");
        return $"{string.Join(' ', words)} relative to {bodyId}";
    }

    private static OrbitDashboardWarning? Warning(OrbitAnalysisReport report)
    {
        if (report.FirstSurfaceCrossingTimeSeconds is { } crossing)
        {
            double delay = Math.Max(0, crossing - report.StartTimeSeconds);
            string when = delay == 0 ? "at interval start" : $"in {Duration(delay)}";
            return new(OrbitDashboardWarningKind.SurfaceCrossing,
                $"PREDICTED MEAN-RADIUS SURFACE CROSSING {when}"
                + $" | lowest sampled altitude {Length(report.LowestSampledAltitudeMeters)}");
        }
        if (report.LowestSampledAltitudeMeters is <= 0)
        {
            return new(OrbitDashboardWarningKind.UnresolvedSurfaceCrossing,
                "SAMPLED PATH REACHES THE MEAN-RADIUS SURFACE"
                + " | crossing time was not resolved");
        }
        return null;
    }

    private static OrbitDashboardStatisticRow Statistic(
        string label, OrbitElementStatistic? statistic, Func<double, string> format)
    {
        if (statistic is not { } value)
            return new(label, "undefined", "undefined", "undefined");
        return new(label, format(value.Current), format(value.Mean),
            $"[{format(value.Minimum)}, {format(value.Maximum)}]");
    }

    private static string Mean(
        OrbitElementStatistic? statistic, Func<double, string> format) =>
        statistic is { } value ? format(value.Mean) : "undefined";

    private static OrbitDashboardValueRow Period(
        string label, OrbitPeriodEstimate? period)
    {
        if (period is not { } value)
            return new(label, "not resolved", "insufficient completed events");
        string cycles = $"{value.CycleCount} cycle{(value.CycleCount == 1 ? "" : "s")}";
        return new(label, Duration(value.MeanSeconds),
            $"sigma {Duration(value.StandardDeviationSeconds)} | {cycles}"
            + $" | [{Duration(value.MinimumSeconds)}, {Duration(value.MaximumSeconds)}]");
    }

    private static OrbitDashboardValueRow Rate(
        string label, OrbitPrecessionEstimate? estimate)
    {
        if (estimate is not { } value)
            return new(label, "undefined", "singular or insufficient samples");
        double degreesPerDay = value.RadiansPerSecond * RadiansToDegrees * SecondsPerDay;
        return new(label,
            $"{degreesPerDay:+0.000000;-0.000000;0.000000} deg/day",
            $"fit RMS {value.ResidualRadians * RadiansToDegrees:F4} deg"
            + $" | {value.SampleCount:N0} samples");
    }

    private static IReadOnlyList<OrbitDashboardValueRow> GroundTrack(
        GroundTrackRecurrence? recurrence)
    {
        if (recurrence is not { } value)
            return [new("recurrence", "not resolved", "see data-quality limits")];
        string direction = Math.Abs(value.RelativeTrackTurns) == 1 ? "cycle" : "cycles";
        return
        [
            new("best observed closure",
                $"{value.OrbitCount} orbits / {value.RelativeTrackTurns} ground-track {direction}",
                $"{Duration(value.DurationSeconds)} | closure "
                + $"{value.ClosureErrorRadians * RadiansToDegrees:F3} deg"
                + $" | {value.ObservedWindows} observed window"
                + $"{(value.ObservedWindows == 1 ? "" : "s")}"),
            new("longitude shift / nodal orbit",
                $"{value.LongitudeShiftPerOrbitRadians * RadiansToDegrees:+0.000;-0.000;0.000} deg",
                "measured in the body-fixed frame"),
        ];
    }

    private static string Length(double? meters) =>
        meters is { } value ? Length(value) : "undefined";

    private static string Length(double meters)
    {
        if (!double.IsFinite(meters)) return "undefined";
        double magnitude = Math.Abs(meters);
        if (magnitude >= 1e9) return $"{meters / 1e9:F4} Gm";
        if (magnitude >= 1e6) return $"{meters / 1e6:F4} Mm";
        if (magnitude >= 1e3) return $"{meters / 1e3:F3} km";
        return $"{meters:F1} m";
    }

    private static string Angle(double radians) => double.IsFinite(radians)
        ? $"{radians * RadiansToDegrees:F4} deg" : "undefined";

    private static string Scalar(double value) =>
        double.IsFinite(value) ? value.ToString("G7") : "undefined";

    private static string Duration(double seconds) =>
        double.IsFinite(seconds)
            ? TimeDisplayKernel.FormatDuration(seconds, years: true)
            : "undefined";
}

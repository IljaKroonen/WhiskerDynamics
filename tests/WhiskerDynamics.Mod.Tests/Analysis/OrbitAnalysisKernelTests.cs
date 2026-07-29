using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Ui;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Analysis;

public class OrbitAnalysisKernelTests
{
    private const double Mu = 3.986004418e14;
    private const double EarthRadius = 6_371_000;
    private static readonly Vector3d Pole = new(0, 0, 1);

    [Fact]
    public void Precessing_ellipse_resolves_elements_three_periods_and_secular_rates()
    {
        const double a = 7_200_000;
        const double e = 0.12;
        double inclination = Degrees(51.6);
        double n = Math.Sqrt(Mu / (a * a * a));
        double period = 2 * Math.PI / n;
        double nodeRate = Degrees(-0.8) / 86400.0;
        double apsisInPlaneRate = Degrees(1.1) / 86400.0;
        var (times, positions, velocities) = Ellipse(a, e, inclination, Degrees(20), Degrees(40),
            nodeRate, apsisInPlaneRate, 8 * period, 720 * 8);

        var report = OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities, 0, times[^1],
            Mu, EarthRadius, Pole, 2 * Math.PI / 86164.0905)!;

        Assert.InRange(report.Elements.SemiMajorAxis.Mean, a * 0.999, a * 1.001);
        Assert.InRange(report.Elements.Eccentricity.Mean, e - 0.001, e + 0.001);
        Assert.InRange(report.Elements.Inclination.Mean, inclination - Degrees(0.02), inclination + Degrees(0.02));
        double expectedPe = a * (1 - e) - EarthRadius;
        double expectedAp = a * (1 + e) - EarthRadius;
        Assert.NotNull(report.PeriapsisAltitude);
        Assert.NotNull(report.ApoapsisAltitude);
        Assert.InRange(report.PeriapsisAltitude.Value.Mean,
            expectedPe - 2_000, expectedPe + 2_000);
        Assert.InRange(report.ApoapsisAltitude.Value.Mean,
            expectedAp - 2_000, expectedAp + 2_000);
        Assert.InRange(report.LowestSampledAltitudeMeters!.Value,
            expectedPe - 2_000, expectedPe + 2_000);
        AssertPeriod(report.NodalPeriod, period, 0.002);
        AssertPeriod(report.AnomalisticPeriod, period, 0.002);
        // Inertial phase includes the node's secular drift.
        AssertPeriod(report.SiderealPeriod, 2 * Math.PI / (n + nodeRate + apsisInPlaneRate), 0.003);
        Assert.InRange(report.NodalPrecession!.Value.RadiansPerSecond,
            nodeRate - 2e-9, nodeRate + 2e-9);
        // The fixture rotates its geometric ellipse while the production kernel
        // intentionally derives velocity from positions; that derivative includes
        // the imposed frame motion, so the resulting osculating eccentricity vector
        // is not exactly the geometric axis. It must still recover the secular rate.
        Assert.InRange(report.LongitudeOfPeriapsisPrecession!.Value.RadiansPerSecond,
            nodeRate + apsisInPlaneRate - 1e-8, nodeRate + apsisInPlaneRate + 1e-8);
        Assert.True(report.EstimatedRevolutions > 7.9);
    }

    [Fact]
    public void Ground_track_finds_exact_fifteen_orbit_recurrence()
    {
        const double a = 7_000_000;
        double n = Math.Sqrt(Mu / (a * a * a));
        double period = 2 * Math.PI / n;
        // The orbital plane itself precesses one turn per 30 orbits. The body's
        // spin is that rate plus one relative ground-track turn per 15 orbits;
        // recurrence must use their difference, not rotation alone.
        double nodeRate = 2 * Math.PI / (30 * period);
        var (times, positions, velocities) = Ellipse(a, 0.02, Degrees(45), 0, 0,
            nodeRate, 0, 18 * period, 360 * 18);
        double spin = nodeRate + 2 * Math.PI / (15 * period);

        var report = OrbitAnalysisKernel.Analyze("Body", times, positions, velocities, 0, times[^1],
            Mu, EarthRadius, Pole, spin)!;

        var recurrence = Assert.IsType<GroundTrackRecurrence>(report.GroundTrack);
        Assert.Equal(15, recurrence.OrbitCount);
        Assert.Equal(-1, recurrence.RelativeTrackTurns);
        Assert.InRange(recurrence.ClosureErrorRadians, 0, Degrees(0.05));
        Assert.InRange(recurrence.DurationSeconds, 15 * period * 0.999, 15 * period * 1.001);
    }

    [Fact]
    public void Circular_equatorial_orbit_keeps_useful_quantities_and_marks_singular_angles()
    {
        const double a = 7_000_000;
        double n = Math.Sqrt(Mu / (a * a * a));
        double period = 2 * Math.PI / n;
        var (times, positions, velocities) = Ellipse(a, 0, 0, 0, 0,
            0, 0, 4 * period, 360 * 4);

        var report = OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities, 0, times[^1],
            Mu, EarthRadius, Pole)!;

        AssertPeriod(report.SiderealPeriod, period, 0.001);
        Assert.Null(report.NodalPrecession);
        Assert.Null(report.ArgumentOfPeriapsisPrecession);
        Assert.Null(report.LongitudeOfPeriapsisPrecession);
        Assert.True(double.IsNaN(report.Elements.LongitudeOfAscendingNode.Mean));
        Assert.True(double.IsNaN(report.Elements.ArgumentOfPeriapsis.Mean));
        Assert.Contains(report.Notes, n => n.Contains("equatorial", StringComparison.Ordinal));
    }

    [Fact]
    public void Circular_inclined_orbit_does_not_invent_apsidal_motion_at_default_density()
    {
        const double a = 7_000_000;
        double period = 2 * Math.PI / Math.Sqrt(Mu / (a * a * a));
        var (times, positions, velocities) = Ellipse(a, 0, Degrees(55), Degrees(20), 0,
            0, 0, 4 * period, 720 * 4); // production's default 0.5 degree turn density

        var report = OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities,
            0, times[^1], Mu, EarthRadius, Pole)!;

        Assert.InRange(report.Elements.Eccentricity.Mean, 0, 1e-8);
        Assert.Null(report.AnomalisticPeriod);
        Assert.Null(report.ArgumentOfPeriapsisPrecession);
        Assert.Null(report.LongitudeOfPeriapsisPrecession);
    }

    [Fact]
    public void Open_trajectory_has_periapsis_but_explicitly_undefined_apoapsis()
    {
        const double radius = EarthRadius + 500_000;
        double speed = 1.1 * Math.Sqrt(2 * Mu / radius);
        double[] times = [0, 1, 2, 3];
        var positions = new Vector3d[times.Length];
        var velocities = new Vector3d[times.Length];
        for (int i = 0; i < times.Length; i++)
        {
            double angle = Degrees(30 * i);
            positions[i] = new(radius * Math.Cos(angle), radius * Math.Sin(angle), 0);
            velocities[i] = new(-speed * Math.Sin(angle), speed * Math.Cos(angle), 0);
        }

        var report = OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities,
            0, times[^1], Mu, EarthRadius, Pole)!;

        Assert.NotNull(report.PeriapsisAltitude);
        Assert.Null(report.ApoapsisAltitude);
        Assert.All(report.Trend, point => Assert.Null(point.ApoapsisAltitudeMeters));
    }

    [Fact]
    public void Surface_metrics_use_sampled_altitude_and_interpolate_first_crossing()
    {
        double[] times = [0, 1, 2, 3];
        double[] radii = [110, 90, 80, 120];
        double[] angles = [0, 5, 10, 20];
        var positions = new Vector3d[times.Length];
        var velocities = new Vector3d[times.Length];
        for (int i = 0; i < times.Length; i++)
        {
            double angle = Degrees(angles[i]);
            positions[i] = new(radii[i] * Math.Cos(angle), radii[i] * Math.Sin(angle), 0);
            velocities[i] = new(-30 * Math.Sin(angle), 30 * Math.Cos(angle), 0);
        }

        var report = OrbitAnalysisKernel.Analyze("Test", times, positions, velocities,
            0, times[^1], 100_000, 100, Pole)!;

        Assert.Equal(-20, report.LowestSampledAltitudeMeters!.Value, 10);
        Assert.Equal(0.5, report.FirstSurfaceCrossingTimeSeconds!.Value, 12);
    }

    [Fact]
    public void Unknown_radius_leaves_altitudes_and_surface_warning_inputs_undefined()
    {
        double[] times = [0, 1, 2, 3];
        Vector3d[] positions =
        [
            new(100, 0, 0),
            new(90 * Math.Cos(Degrees(30)), 90 * Math.Sin(Degrees(30)), 0),
            new(80 * Math.Cos(Degrees(60)), 80 * Math.Sin(Degrees(60)), 0),
            new(0, 120, 0),
        ];
        Vector3d[] velocities =
        [
            new(0, 30, 0),
            new(-15, 26, 0),
            new(-26, 15, 0),
            new(-30, 0, 0),
        ];

        var report = OrbitAnalysisKernel.Analyze("Test", times, positions, velocities,
            0, times[^1], 100_000, 0, Pole)!;

        Assert.Null(report.PeriapsisAltitude);
        Assert.Null(report.ApoapsisAltitude);
        Assert.Null(report.LowestSampledAltitudeMeters);
        Assert.Null(report.FirstSurfaceCrossingTimeSeconds);
        Assert.All(report.Trend, point =>
        {
            Assert.Null(point.PeriapsisAltitudeMeters);
            Assert.Null(point.ApoapsisAltitudeMeters);
        });
        Assert.Contains(report.Notes, note => note.Contains("mean body radius"));
    }

    [Fact]
    public void Trend_is_chronological_includes_endpoints_and_is_bounded()
    {
        const double a = 7_000_000;
        double period = 2 * Math.PI / Math.Sqrt(Mu / (a * a * a));
        var (times, positions, velocities) = Ellipse(a, 0.03, Degrees(40), 0, 0,
            0, 0, 30 * period, 1200);
        var progress = new List<double>();

        var report = OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities,
            0, times[^1], Mu, EarthRadius, Pole, progress: progress.Add)!;

        Assert.True(report.SampleCount > 512);
        Assert.Equal(512, report.Trend.Count);
        Assert.Equal(report.StartTimeSeconds, report.Trend[0].TimeSeconds);
        Assert.Equal(report.EndTimeSeconds, report.Trend[^1].TimeSeconds);
        for (int i = 1; i < report.Trend.Count; i++)
            Assert.True(report.Trend[i].TimeSeconds > report.Trend[i - 1].TimeSeconds);
        Assert.NotEmpty(progress);
        Assert.Equal(0, progress[0]);
        Assert.Equal(1, progress[^1]);
        for (int i = 1; i < progress.Count; i++)
            Assert.True(progress[i] >= progress[i - 1]);
    }

    [Fact]
    public void Reduction_cooperatively_cancels_during_a_large_series()
    {
        const double a = 7_000_000;
        double period = 2 * Math.PI / Math.Sqrt(Mu / (a * a * a));
        var (times, positions, velocities) = Ellipse(a, 0.03, Degrees(40), 0, 0,
            0, 0, 30 * period, 1200);
        int polls = 0;

        Assert.Throws<OperationCanceledException>(() =>
            OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities,
                0, times[^1], Mu, EarthRadius, Pole,
                shouldStop: () => ++polls >= 2));
        Assert.Equal(2, polls);
    }

    [Fact]
    public void Report_reduction_keeps_polling_after_the_primary_point_scan()
    {
        const double a = 7_000_000;
        double period = 2 * Math.PI / Math.Sqrt(Mu / (a * a * a));
        var (times, positions, velocities) = Ellipse(a, 0.03, Degrees(40), 0, 0,
            0, 0, 30 * period, 1200);
        bool primaryScanComplete = false;
        int reductionPolls = 0;

        Assert.Throws<OperationCanceledException>(() =>
            OrbitAnalysisKernel.Analyze("Earth", times, positions, velocities,
                0, times[^1], Mu, EarthRadius, Pole,
                progress: progress => primaryScanComplete |= progress == 1,
                shouldStop: () =>
                {
                    if (!primaryScanComplete) return false;
                    reductionPolls++;
                    return true;
                }));

        Assert.Equal(1, reductionPolls);
    }

    [Fact]
    public void Refuses_degenerate_or_too_short_series()
    {
        Assert.Null(OrbitAnalysisKernel.Analyze("Earth", [0, 1],
            [new(1, 0, 0), new(0, 1, 0)], 0, 1, Mu, EarthRadius, Pole));
        Assert.Null(OrbitAnalysisKernel.Analyze("Earth", [0, 1, 2],
            [new(1, 0, 0), new(0, 1, 0), new(-1, 0, 0)],
            0, 2, 0, EarthRadius, Pole));
    }

    private static void AssertPeriod(OrbitPeriodEstimate? actual, double expected, double relativeTolerance)
    {
        Assert.True(actual.HasValue, "period should be resolved");
        Assert.InRange(actual.Value.MeanSeconds,
            expected * (1 - relativeTolerance), expected * (1 + relativeTolerance));
        Assert.True(actual.Value.CycleCount >= 2);
    }

    private static (double[] Times, Vector3d[] Positions, Vector3d[] Velocities) Ellipse(
        double a, double e, double inclination, double node0, double periapsis0,
        double nodeRate, double periapsisRate, double duration, int intervals)
    {
        double n = Math.Sqrt(Mu / (a * a * a));
        Vector3d PositionAt(double t)
        {
            double mean = n * t;
            double eccentric = SolveEccentricAnomaly(mean, e);
            double x = a * (Math.Cos(eccentric) - e);
            double y = a * Math.Sqrt(1 - e * e) * Math.Sin(eccentric);
            return Rotate(new Vector3d(x, y, 0), node0 + nodeRate * t,
                inclination, periapsis0 + periapsisRate * t);
        }

        var times = new double[intervals + 1];
        var positions = new Vector3d[intervals + 1];
        var velocities = new Vector3d[intervals + 1];
        const double derivativeStep = 0.01;
        for (int k = 0; k <= intervals; k++)
        {
            double t = duration * k / intervals;
            times[k] = t;
            positions[k] = PositionAt(t);
            velocities[k] = (PositionAt(t + derivativeStep) - PositionAt(t - derivativeStep))
                / (2 * derivativeStep);
        }
        return (times, positions, velocities);
    }
    private static Vector3d Rotate(Vector3d perifocal, double node, double inclination, double periapsis)
    {
        var afterPeriapsis = perifocal.RotateAbout(new Vector3d(0, 0, 1), periapsis);
        var afterInclination = afterPeriapsis.RotateAbout(new Vector3d(1, 0, 0), inclination);
        return afterInclination.RotateAbout(new Vector3d(0, 0, 1), node);
    }

    private static double SolveEccentricAnomaly(double mean, double eccentricity)
    {
        double reduced = Math.IEEERemainder(mean, 2 * Math.PI);
        double eccentric = reduced;
        for (int i = 0; i < 12; i++)
            eccentric -= (eccentric - eccentricity * Math.Sin(eccentric) - reduced)
                / (1 - eccentricity * Math.Cos(eccentric));
        return eccentric;
    }

    private static double Degrees(double value) => value * Math.PI / 180.0;
}
[Collection("orbit-analyser-statics")]
public class OrbitAnalyserRequestTests
{
    [Fact]
    public void Analysis_progress_is_monotone_versioned_and_counts_completed_passes()
    {
        OrbitAnalyserPanel.ResetSessionStatics();
        OrbitAnalyserPanel.Open();
        Assert.True(OrbitAnalyserPanel.TryGetRequest(out _, out _, out int version));

        int firstPass = OrbitAnalyserPanel.BeginAnalysisPass(version);
        Assert.Equal(1, firstPass);
        OrbitAnalyserPanel.ReportAnalysisProgress(version, firstPass, 0.4,
            OrbitAnalyserPanel.AnalysisPhase.Sampling);
        OrbitAnalyserPanel.ReportAnalysisProgress(version, firstPass, 0.2,
            OrbitAnalyserPanel.AnalysisPhase.Sampling);
        var running = OrbitAnalyserPanel.ReadAnalysisProgress(version);
        Assert.True(running.Running);
        Assert.Equal(0.4, running.Fraction);
        Assert.Equal(OrbitAnalyserPanel.AnalysisPhase.Sampling, running.Phase);

        OrbitAnalyserPanel.CompleteAnalysisPass(version, firstPass);
        var complete = OrbitAnalyserPanel.ReadAnalysisProgress(version);
        Assert.False(complete.Running);
        Assert.Equal(1, complete.Fraction);
        Assert.Equal(OrbitAnalyserPanel.AnalysisPhase.Complete, complete.Phase);

        Assert.Equal(2, OrbitAnalyserPanel.BeginAnalysisPass(version));
        OrbitAnalyserPanel.SetInterval(0, 86400);
        Assert.True(OrbitAnalyserPanel.TryGetRequest(out _, out _, out int nextVersion));
        OrbitAnalyserPanel.ReportAnalysisProgress(version, 2, 0.8,
            OrbitAnalyserPanel.AnalysisPhase.Reducing);
        var next = OrbitAnalyserPanel.ReadAnalysisProgress(nextVersion);
        Assert.Equal(0, next.Pass);
        Assert.Equal(0, next.Fraction);

        OrbitAnalyserPanel.Close();
        OrbitAnalyserPanel.ResetSessionStatics();
    }

    [Fact]
    public void Analysis_compute_request_is_atomic_clamped_idempotent_and_open_only()
    {
        OrbitAnalyserPanel.ResetSessionStatics();
        Assert.False(OrbitAnalyserPanel.TryGetRequest(out _, out _, out int resetVersion));
        Assert.False(OrbitAnalyserPanel.RequestMatches(resetVersion));

        OrbitAnalyserPanel.Open();
        Assert.True(OrbitAnalyserPanel.TryGetRequest(out double start, out double span,
            out int openVersion));
        Assert.Equal(0.0, start);
        Assert.Equal(7 * 86400.0, span);
        Assert.True(openVersion > resetVersion);
        Assert.True(OrbitAnalyserPanel.RequestMatches(openVersion));

        OrbitAnalyserPanel.Open();
        Assert.True(OrbitAnalyserPanel.TryGetRequest(out _, out _, out int repeatedOpenVersion));
        Assert.Equal(openVersion, repeatedOpenVersion);

        OrbitAnalyserPanel.SetInterval(123, 0);
        Assert.True(OrbitAnalyserPanel.TryGetRequest(out start, out span,
            out int minimumVersion));
        Assert.Equal(123.0, start);
        Assert.Equal(60.0, span);
        Assert.True(minimumVersion > openVersion);
        Assert.False(OrbitAnalyserPanel.RequestMatches(openVersion));
        Assert.True(OrbitAnalyserPanel.RequestMatches(minimumVersion));

        OrbitAnalyserPanel.SetInterval(-100, double.MaxValue);
        Assert.True(OrbitAnalyserPanel.TryGetRequest(out start, out span,
            out int maximumVersion));
        Assert.Equal(0.0, start);
        Assert.Equal(40 * 365.25 * 86400.0, span);
        Assert.True(maximumVersion > minimumVersion);

        OrbitAnalyserPanel.Close();
        Assert.False(OrbitAnalyserPanel.TryGetRequest(out _, out _, out int closeVersion));
        Assert.True(closeVersion > maximumVersion);
        Assert.False(OrbitAnalyserPanel.RequestMatches(maximumVersion));

        OrbitAnalyserPanel.Close();
        Assert.False(OrbitAnalyserPanel.TryGetRequest(out _, out _, out int repeatedCloseVersion));
        Assert.Equal(closeVersion, repeatedCloseVersion);
        OrbitAnalyserPanel.ResetSessionStatics();
    }
}

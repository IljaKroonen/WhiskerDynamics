using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.Mod.Tests.Analysis;

public class OrbitAnalysisPresentationModelTests
{
    [Theory]
    [InlineData(0.009999, 0, "circular equatorial orbit relative to Earth")]
    [InlineData(0.01, 5, "elliptical equatorial orbit relative to Earth")]
    [InlineData(0.5, 80, "elliptical polar orbit relative to Earth")]
    [InlineData(0.500001, 100, "highly elliptical polar retrograde orbit relative to Earth")]
    [InlineData(0.2, 100.001, "elliptical inclined retrograde orbit relative to Earth")]
    [InlineData(0.2, 90, "elliptical polar orbit relative to Earth")]
    [InlineData(0.2, 175, "elliptical equatorial retrograde orbit relative to Earth")]
    public void Classification_uses_shape_and_plane_boundaries(
        double eccentricity, double inclinationDegrees, string expected)
    {
        string actual = OrbitAnalysisPresentationModel.Classify(
            "Earth", eccentricity, Degrees(inclinationDegrees));

        Assert.Equal(expected, actual);
    }

    [Fact]
    public void Resolved_crossing_warning_has_priority_over_below_surface_fallback()
    {
        var report = Report() with
        {
            LowestSampledAltitudeMeters = -10,
            FirstSurfaceCrossingTimeSeconds = 130,
        };

        var presentation = OrbitAnalysisPresentationModel.Create(report, 0, 1000);

        Assert.Equal(OrbitDashboardWarningKind.SurfaceCrossing,
            presentation.Warning!.Kind);
        Assert.Contains("in 30s", presentation.Warning.Text);
        Assert.Contains("-10.0 m", presentation.Warning.Text);
    }

    [Fact]
    public void Undefined_values_remain_explicit_in_summary_and_detail_rows()
    {
        var presentation = OrbitAnalysisPresentationModel.Create(Report(), 0, 1000);

        Assert.Equal("Earth", presentation.BodyId);
        Assert.Equal("undefined",
            presentation.Summary.Single(row => row.Label == "Mean Pe").Value);
        Assert.Equal("not resolved",
            presentation.Summary.Single(row => row.Label == "Sidereal period").Value);
        var apoapsis = presentation.Elements.Single(row => row.Label == "Ap altitude");
        Assert.Equal("undefined", apoapsis.Current);
        Assert.Equal("undefined", apoapsis.Mean);
        Assert.Equal("undefined", apoapsis.Range);
        Assert.Equal("undefined",
            presentation.Elements.Single(row => row.Label == "LAN").Mean);
    }

    [Fact]
    public void Panel_title_names_the_reference_body_without_changing_window_identity()
    {
        const string hiddenId = "###WhiskerDynamicsOrbitAnalysis";

        string waiting = OrbitAnalyserPanel.WindowTitle(null);
        string earth = OrbitAnalyserPanel.WindowTitle("Earth");
        string moon = OrbitAnalyserPanel.WindowTitle("Moon");

        Assert.Equal("Orbit Analysis" + hiddenId, waiting);
        Assert.Equal("Orbit Analysis — Earth" + hiddenId, earth);
        Assert.Equal("Orbit Analysis — Moon" + hiddenId, moon);
        Assert.EndsWith(hiddenId, waiting);
        Assert.EndsWith(hiddenId, earth);
        Assert.EndsWith(hiddenId, moon);
    }

    [Fact]
    public void Summary_and_detail_rows_preserve_statistical_evidence()
    {
        var report = Report() with
        {
            PeriapsisAltitude = new(1_000, 2_000, 500, 2_500),
            SiderealPeriod = new(60, 2, 58, 63, 4),
            NodalPrecession = new(Degrees(1) / 86400, Degrees(0.25), 12),
        };

        var presentation = OrbitAnalysisPresentationModel.Create(report, 50, 1000);

        Assert.Equal("2.000 km",
            presentation.Summary.Single(row => row.Label == "Mean Pe").Value);
        var periapsis = presentation.Elements.Single(row => row.Label == "Pe altitude");
        Assert.Equal("1.000 km", periapsis.Current);
        Assert.Equal("2.000 km", periapsis.Mean);
        Assert.Equal("[500.0 m, 2.500 km]", periapsis.Range);
        var period = presentation.Periods.Single(row => row.Label == "sidereal");
        Assert.Equal("1m", period.Value);
        Assert.Equal("sigma 2s | 4 cycles | [58s, 1m 3s]", period.Evidence);
        var precession = presentation.Precession.Single(row => row.Label == "node");
        Assert.Equal("+1.000000 deg/day", precession.Value);
        Assert.Equal("fit RMS 0.2500 deg | 12 samples", precession.Evidence);
    }

    private static OrbitAnalysisReport Report()
    {
        OrbitElementStatistic defined = new(1, 1, 1, 1);
        OrbitElementStatistic undefined = new(
            double.NaN, double.NaN, double.NaN, double.NaN);
        return new()
        {
            BodyId = "Earth",
            StartTimeSeconds = 100,
            EndTimeSeconds = 600,
            SampleCount = 20,
            EstimatedRevolutions = 2,
            Elements = new(defined, defined, defined, undefined, undefined),
            Trend = [],
            Notes = [],
        };
    }

    private static double Degrees(double value) => value * Math.PI / 180;
}

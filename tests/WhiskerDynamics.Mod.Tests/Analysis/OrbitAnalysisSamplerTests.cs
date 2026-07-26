using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Analysis;

public class OrbitAnalysisSamplerTests
{
    [Fact]
    public void Coarse_analysis_sweep_covers_interval_after_dense_display_budget_exhausts()
    {
        const double mu = 3.986004418e14;
        const double radius = 7_000_000;
        double period = 2 * Math.PI * Math.Sqrt(radius * radius * radius / mu);
        double speed = Math.Sqrt(mu / radius);
        double end = 100 * period;

        (Vector3d Position, Vector3d Velocity) State(double time)
        {
            double phase = 2 * Math.PI * time / period;
            return (
                new(radius * Math.Cos(phase), radius * Math.Sin(phase), 0),
                new(-speed * Math.Sin(phase), speed * Math.Cos(phase), 0));
        }

        var display = AdaptiveSampler.Sample(
            time => State(time).Position, 0, end, 100_000,
            0.2 * Math.PI / 180, 1, period);
        var analysis = OrbitAnalysisSampler.Sample(State, 0, end, period);

        Assert.True(display.Truncated);
        Assert.False(analysis.Truncated);
        Assert.Equal(end, analysis.Times[^1]);
        Assert.InRange(analysis.Times.Length, 6_000, 20_000);
        int middle = analysis.Times.Length / 2;
        var expected = State(analysis.Times[middle]);
        Assert.Equal(expected.Position, analysis.Positions[middle]);
        Assert.Equal(expected.Velocity, analysis.Velocities[middle]);
    }

    [Fact]
    public void Explicit_test_budget_truncates_without_relaxing_scientific_quality()
    {
        const double period = 3000;
        const double radius = 7_000_000;
        const double revolutions = 20;
        double speed = 2 * Math.PI * radius / period;
        double end = revolutions * period;

        (Vector3d Position, Vector3d Velocity) State(double time)
        {
            double phase = 2 * Math.PI * time / period;
            return (
                new(radius * Math.Cos(phase), radius * Math.Sin(phase), 0),
                new(-speed * Math.Sin(phase), speed * Math.Cos(phase), 0));
        }

        var analysis = OrbitAnalysisSampler.Sample(
            State, 0, end, period, maximumPoints: 600);

        Assert.True(analysis.Truncated);
        Assert.Equal(600, analysis.Times.Length);
        Assert.True(analysis.Times[^1] < end);
        for (int i = 2; i < analysis.Positions.Length; i++)
            Assert.True(AdaptiveSampler.TurnAngle(
                analysis.Positions[i - 2], analysis.Positions[i - 1],
                analysis.Positions[i])
                <= OrbitAnalysisSampler.MaximumTurnRadians * 1.001);
    }

    [Fact]
    public void Production_resolution_relaxes_long_windows_without_defeating_antialiasing()
    {
        const double period = 6000;
        const double revolutions = 10_000;
        double end = revolutions * period;

        double turn = OrbitAnalysisSampler.ProductionTurnRadians(
            0, end, period, targetPoints: 100_000);

        Assert.Equal(OrbitAnalysisSampler.ProductionMaximumTurnRadians, turn);
        // AdaptiveSampler still independently caps its step to period / 8.
        Assert.Equal(Math.PI / 4, turn);
    }

    [Fact]
    public void Accepted_time_callback_keeps_streaming_predictor_history_bounded()
    {
        const double mu = 3.986004418e14;
        const double radius = 7_000_000;
        double period = 2 * Math.PI * Math.Sqrt(radius * radius * radius / mu);
        var body = new CelestialBody { Id = "Earth", Mu = mu };
        var predictor = new TrajectoryPredictor(
            new GravityModel(new Ephemerides([body])),
            new StateVector(
                new Vector3d(radius, 0, 0),
                new Vector3d(0, Math.Sqrt(mu / radius), 0)),
            0, new IntegratorOptions { RelTol = 1e-9 });
        int largestRetainedHistory = 0;

        (Vector3d Position, Vector3d Velocity) State(double time)
        {
            var state = predictor.StateAt(time);
            return (state.Position, state.Velocity);
        }

        var analysis = OrbitAnalysisSampler.Sample(
            State, 0, 200 * period, period,
            maximumTurnRadians: Math.PI / 4,
            acceptedTime: time =>
            {
                predictor.PruneBefore(time);
                largestRetainedHistory = Math.Max(
                    largestRetainedHistory, predictor.Nodes.Count);
            });

        Assert.False(analysis.Truncated);
        Assert.Equal(analysis.Times[^1], predictor.Horizon);
        Assert.Equal(analysis.Times.Length, analysis.Velocities.Length);
        Assert.InRange(largestRetainedHistory, 1, 100);
    }

    [Fact]
    public void Production_resolution_covers_ten_year_low_orbit_within_hard_budget()
    {
        const double mu = 3.986004418e14;
        const double radius = 7_000_000;
        double period = 2 * Math.PI * Math.Sqrt(radius * radius * radius / mu);
        double speed = Math.Sqrt(mu / radius);
        double end = 10 * 365.25 * 86400.0;

        (Vector3d Position, Vector3d Velocity) State(double time)
        {
            double phase = 2 * Math.PI * time / period;
            return (
                new(radius * Math.Cos(phase), radius * Math.Sin(phase), 0),
                new(-speed * Math.Sin(phase), speed * Math.Cos(phase), 0));
        }

        double turn = OrbitAnalysisSampler.ProductionTurnRadians(0, end, period);
        var analysis = OrbitAnalysisSampler.Sample(
            State, 0, end, period,
            maximumPoints: OrbitAnalysisSampler.ProductionMaximumPoints,
            maximumTurnRadians: turn);

        Assert.False(analysis.Truncated);
        Assert.Equal(end, analysis.Times[^1]);
        Assert.InRange(analysis.Times.Length, 400_000,
            OrbitAnalysisSampler.ProductionMaximumPoints);
    }
}

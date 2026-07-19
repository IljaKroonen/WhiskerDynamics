using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Overlay;

/// <summary>Tests angle-bounded adaptive sampling with closed-form paths.</summary>
public class AdaptiveSamplerTests
{
    private const double TwoPi = 2 * Math.PI;

    /// <summary>Returns a circle whose chord turn equals the arc angle per step.</summary>
    private static Func<double, Vector3d> Circle(double radius, double omega) =>
        t => new Vector3d(radius * Math.Cos(omega * t), radius * Math.Sin(omega * t), 0);

    [Fact]
    public void Turn_angle_bound_holds_on_a_circle()
    {
        double period = 5400.0; // LEO-ish
        double thetaMax = 0.5 * Math.PI / 180.0;
        var path = AdaptiveSampler.Sample(Circle(7e6, TwoPi / period), t0: 0, horizon: period,
            maxPoints: 2000, thetaMaxRad: thetaMax, dtMinSeconds: 1.0, periodHintSeconds: period);
        Assert.False(path.Truncated);
        for (int i = 2; i < path.Positions.Length; i++)
        {
            double turn = AdaptiveSampler.TurnAngle(
                path.Positions[i - 2], path.Positions[i - 1], path.Positions[i]);
            Assert.True(turn <= thetaMax * 1.001,
                $"turn {turn} rad at i={i} exceeds bound {thetaMax}");
        }
    }

    [Fact]
    public void Times_are_strictly_monotone_and_hit_both_endpoints_when_not_truncated()
    {
        double period = 5400.0;
        var path = AdaptiveSampler.Sample(Circle(7e6, TwoPi / period), t0: 100.0, horizon: 100.0 + period,
            maxPoints: 2000, thetaMaxRad: 0.5 * Math.PI / 180.0, dtMinSeconds: 1.0, periodHintSeconds: period);
        Assert.Equal(100.0, path.Times[0]);
        Assert.Equal(100.0 + period, path.Times[^1]);
        for (int i = 1; i < path.Times.Length; i++)
            Assert.True(path.Times[i] > path.Times[i - 1], $"times must strictly increase (i={i})");
    }

    [Fact]
    public void Budget_exhaustion_truncates_cleanly_before_the_horizon()
    {
        double period = 5400.0;
        // 10 laps at 0.5 deg/segment needs ~7200 points; cap at 500 -> truncation.
        var path = AdaptiveSampler.Sample(Circle(7e6, TwoPi / period), t0: 0, horizon: 10 * period,
            maxPoints: 500, thetaMaxRad: 0.5 * Math.PI / 180.0, dtMinSeconds: 1.0, periodHintSeconds: period);
        Assert.True(path.Truncated);
        Assert.False(path.WorkLimited);
        Assert.Equal(500, path.Times.Length);
        Assert.True(path.Times[^1] < 10 * period);
        for (int i = 2; i < path.Positions.Length; i++)
            Assert.True(AdaptiveSampler.TurnAngle(path.Positions[i - 2], path.Positions[i - 1], path.Positions[i])
                <= 0.5 * Math.PI / 180.0 * 1.001);
    }

    [Fact]
    public void Accepted_callback_matches_the_published_sample_times()
    {
        double period = 5400.0;
        var accepted = new List<double>();

        var path = AdaptiveSampler.Sample(Circle(7e6, TwoPi / period),
            0, 2 * period, 2000, Math.PI / 12, 1, period,
            accepted: accepted.Add);

        Assert.Equal(path.Times, accepted);
    }

    [Fact]
    public void Work_limit_returns_a_clean_prefix_without_restarting()
    {
        double period = 5400.0;
        int evaluations = 0;
        var circle = Circle(7e6, TwoPi / period);
        Vector3d Position(double t)
        {
            evaluations++;
            return circle(t);
        }

        var path = AdaptiveSampler.Sample(Position, 0, 10 * period,
            maxPoints: 16384, thetaMaxRad: 0.5 * Math.PI / 180.0,
            dtMinSeconds: 1.0, periodHintSeconds: period,
            shouldStop: () => evaluations >= 30);

        Assert.True(path.Truncated);
        Assert.True(path.WorkLimited);
        Assert.InRange(evaluations, 2, 30);
        Assert.True(path.Times.Length >= 2);
        Assert.True(path.Times[^1] < 10 * period);
        for (int i = 1; i < path.Times.Length; i++)
            Assert.True(path.Times[i] > path.Times[i - 1]);
    }

    [Fact]
    public void Straight_paths_are_cheap()
    {
        var path = AdaptiveSampler.Sample(t => new Vector3d(1000.0 * t, 0, 0), t0: 0, horizon: 86400.0,
            maxPoints: 2000, thetaMaxRad: 0.5 * Math.PI / 180.0, dtMinSeconds: 1.0,
            periodHintSeconds: double.PositiveInfinity);
        Assert.False(path.Truncated);
        Assert.True(path.Times.Length < 300, $"straight line used {path.Times.Length} points");
    }

    [Fact]
    public void A_genuine_corner_is_accepted_at_dtMin_instead_of_halving_forever()
    {
        // An irreducible 90-degree kink must not prevent reaching the horizon.
        Func<double, Vector3d> kink = t => t <= 500.0
            ? new Vector3d(1000.0 * t, 0, 0)
            : new Vector3d(500_000.0, 1000.0 * (t - 500.0), 0);
        var path = AdaptiveSampler.Sample(kink, t0: 0, horizon: 1000.0,
            maxPoints: 2000, thetaMaxRad: 0.5 * Math.PI / 180.0, dtMinSeconds: 1.0,
            periodHintSeconds: double.PositiveInfinity);
        Assert.False(path.Truncated);
        Assert.Equal(1000.0, path.Times[^1]);
    }

    [Fact]
    public void A_stationary_path_terminates_with_the_max_step()
    {
        var path = AdaptiveSampler.Sample(t => new Vector3d(1, 2, 3), t0: 0, horizon: 86400.0,
            maxPoints: 2000, thetaMaxRad: 0.5 * Math.PI / 180.0, dtMinSeconds: 1.0,
            periodHintSeconds: double.PositiveInfinity);
        Assert.False(path.Truncated);
        Assert.Equal(86400.0, path.Times[^1]);
        Assert.True(path.Times.Length <= 300);
    }

    [Fact]
    public void Period_hint_caps_the_step_against_aliasing()
    {
        // The period hint prevents collapsed chords from aliasing a full revolution.
        double period = 5400.0;
        var path = AdaptiveSampler.Sample(Circle(7e6, TwoPi / period), t0: 0, horizon: 4 * period,
            maxPoints: 2000, thetaMaxRad: 0.5 * Math.PI / 180.0, dtMinSeconds: 1.0, periodHintSeconds: period);
        for (int i = 1; i < path.Times.Length; i++)
            Assert.True(path.Times[i] - path.Times[i - 1] <= period / 8 + 1e-9);
    }

    [Fact]
    public void TurnAngle_is_zero_for_collinear_and_degenerate_chords()
    {
        var a = new Vector3d(0, 0, 0); var b = new Vector3d(1, 0, 0); var c = new Vector3d(2, 0, 0);
        Assert.Equal(0.0, AdaptiveSampler.TurnAngle(a, b, c), 12);
        Assert.Equal(0.0, AdaptiveSampler.TurnAngle(a, a, c), 12); // zero-length chord: no evidence of a turn
        Assert.Equal(Math.PI / 2, AdaptiveSampler.TurnAngle(a, b, new Vector3d(1, 1, 0)), 12);
    }

    [Theory]
    [InlineData(7e6)]
    [InlineData(4.2e7)]
    public void PeriodSeconds_matches_the_circular_closed_form(double r)
    {
        double mu = 3.986004418e14;
        double v = Math.Sqrt(mu / r);
        double expected = TwoPi * Math.Sqrt(r * r * r / mu);
        double period = AdaptiveSampler.PeriodSeconds(mu, new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        Assert.Equal(expected, period, expected * 1e-12);
    }

    [Fact]
    public void PeriodSeconds_is_infinite_for_unbound_orbits()
    {
        double mu = 3.986004418e14, r = 7e6;
        double vEscape = Math.Sqrt(2 * mu / r);
        Assert.True(double.IsPositiveInfinity(
            AdaptiveSampler.PeriodSeconds(mu, new Vector3d(r, 0, 0), new Vector3d(0, vEscape * 1.01, 0))));
        Assert.True(double.IsPositiveInfinity(
            AdaptiveSampler.PeriodSeconds(mu, new Vector3d(r, 0, 0), new Vector3d(0, vEscape, 0))));
    }

    [Fact]
    public void Integration_failure_returns_the_valid_prefix()
    {
        Vector3d Position(double t) => t >= 100
            ? throw new IntegrationFailureException(nameof(Position))
            : new Vector3d(t, 0, 0);
        var path = AdaptiveSampler.Sample(Position, 0, 1000, 2000, 0.01, 1,
            double.PositiveInfinity);
        Assert.True(path.Truncated);
        Assert.True(path.DynamicsLimited);
        Assert.InRange(path.Times[^1], 99, 100);
        Assert.All(path.Times, t => Assert.True(t < 100));
    }

    [Fact]
    public void Dynamics_limit_preserves_the_four_value_public_shape()
    {
        var path = new AdaptivePath([0], [default], true, false)
        {
            DynamicsLimited = true,
        };
        var (times, positions, truncated, workLimited) = path;
        Assert.Single(times);
        Assert.Single(positions);
        Assert.True(truncated);
        Assert.False(workLimited);
        Assert.True(path.DynamicsLimited);
    }

    [Fact]
    public void Dynamics_limit_preserves_a_valid_prefix()
    {
        Vector3d Position(double t) => t >= 600
            ? throw new IntegrationFailureException(nameof(Position))
            : new Vector3d(t, 0, 0);
        var path = AdaptiveSampler.Sample(Position, 0, 1000,
            maxPoints: 2000, thetaMaxRad: 0.01, dtMinSeconds: 1,
            periodHintSeconds: 100);
        Assert.True(path.Truncated);
        Assert.True(path.DynamicsLimited);
        Assert.InRange(path.Times[^1], 599, 600);
        for (int i = 1; i < path.Times.Length; i++)
            Assert.True(path.Times[i] > path.Times[i - 1]);
    }

    [Fact]
    public void Real_predictor_plunge_returns_a_dynamics_limited_line_prefix()
    {
        var body = new CelestialBody
        {
            Id = nameof(Real_predictor_plunge_returns_a_dynamics_limited_line_prefix),
            Mu = 3.986004418e14,
        };
        var gravity = new GravityModel(new Ephemerides([body]));
        const double t0 = 3e8;
        var predictor = new TrajectoryPredictor(gravity,
            new StateVector(new Vector3d(1e5, 0, 0), new Vector3d(-5000, 0, 0)), t0);

        var path = AdaptiveSampler.Sample(t => predictor.StateAt(t).Position,
            t0, t0 + 1000, 2000, 0.01, 1, double.PositiveInfinity);

        Assert.True(path.Truncated);
        Assert.True(path.DynamicsLimited);
        Assert.True(path.Times.Length >= 2);
        Assert.True(path.Times[^1] < t0 + 2);
        Assert.All(path.Positions, point => Assert.True(double.IsFinite(point.X)
            && double.IsFinite(point.Y) && double.IsFinite(point.Z)));
    }

    [Fact]
    public void Fixed_quality_truncates_tight_orbits_but_preserves_long_gentle_arcs()
    {
        const double horizon = 30 * 86400.0;
        double thetaMax = 0.4 * Math.PI / 180.0;
        var tight = AdaptiveSampler.Sample(Circle(7e6, TwoPi / 5400.0), 0, horizon,
            maxPoints: 4096, thetaMaxRad: thetaMax, dtMinSeconds: 1.0,
            periodHintSeconds: 5400.0);
        double gentlePeriod = 365 * 86400.0;
        var gentle = AdaptiveSampler.Sample(Circle(7e6, TwoPi / gentlePeriod),
            0, horizon, maxPoints: 4096, thetaMaxRad: thetaMax, dtMinSeconds: 1.0,
            periodHintSeconds: gentlePeriod);

        Assert.True(tight.Truncated);
        Assert.Equal(4096, tight.Times.Length);
        Assert.True(tight.Times[^1] < horizon);
        for (int i = 2; i < tight.Positions.Length; i++)
            Assert.True(AdaptiveSampler.TurnAngle(
                tight.Positions[i - 2], tight.Positions[i - 1], tight.Positions[i])
                <= thetaMax * 1.001);
        Assert.False(gentle.Truncated);
        Assert.Equal(horizon, gentle.Times[^1]);
    }
}

using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class DormandPrince54Tests
{
    private const double MuEarth = 3.986004418e14;

    private static Vector3d TwoBodyAccel(double t, StateVector s)
    {
        double r2 = s.Position.LengthSquared();
        return -s.Position * (MuEarth / (r2 * Math.Sqrt(r2)));
    }

    [Fact]
    public void Circular_orbit_returns_to_start_after_one_period()
    {
        double r = 7e6;
        double v = Math.Sqrt(MuEarth / r);
        double period = 2 * Math.PI * Math.Sqrt(r * r * r / MuEarth);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        var y1 = DormandPrince54.Propagate(TwoBodyAccel, y0, 0, period);
        Assert.True((y1.Position - y0.Position).Length() < 1.0,
            $"drift {(y1.Position - y0.Position).Length()} m after one orbit");
    }

    [Fact]
    public void Eccentric_orbit_matches_kepler_closed_form_after_ten_periods()
    {
        var el = new OrbitalElements(1e7, 0.7, 0.3, 0.5, 1.0, 0);
        double period = 2 * Math.PI * Math.Sqrt(Math.Pow(el.SemiMajorAxis, 3) / MuEarth);
        var y0 = Kepler.StateFromElements(el, MuEarth, 0);
        var integrated = DormandPrince54.Propagate(TwoBodyAccel, y0, 0, 10 * period);
        var exact = Kepler.StateFromElements(el, MuEarth, 10 * period);
        Assert.True((integrated.Position - exact.Position).Length() < 100.0,
            $"position error {(integrated.Position - exact.Position).Length()} m after 10 orbits");
    }

    [Fact]
    public void Energy_is_conserved_over_a_hundred_orbits()
    {
        double r = 7e6, v = Math.Sqrt(MuEarth / r);
        double period = 2 * Math.PI * Math.Sqrt(r * r * r / MuEarth);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        double E0 = v * v / 2 - MuEarth / r;
        var y1 = DormandPrince54.Propagate(TwoBodyAccel, y0, 0, 100 * period);
        double E1 = y1.Velocity.LengthSquared() / 2 - MuEarth / y1.Position.Length();
        Assert.True(Math.Abs((E1 - E0) / E0) < 1e-9, $"relative energy drift {(E1 - E0) / E0}");
    }

    [Fact]
    public void Accepted_steps_are_reported_in_order_and_reach_t1()
    {
        double r = 7e6, v = Math.Sqrt(MuEarth / r);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        var times = new List<double>();
        DormandPrince54.Propagate(TwoBodyAccel, y0, 0, 3600, onAcceptedStep: (t, _) => times.Add(t));
        Assert.NotEmpty(times);
        Assert.Equal(times.OrderBy(x => x), times);
        Assert.Equal(3600, times[^1], 9);
    }

    [Fact]
    public void One_body_system_matches_scalar_steps_and_state()
    {
        double r = 7e6, v = Math.Sqrt(MuEarth / r);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        var scalarTimes = new List<double>();
        var systemTimes = new List<double>();

        var scalar = DormandPrince54.Propagate(
            TwoBodyAccel, y0, 0, 3600,
            onAcceptedStep: (t, _) => scalarTimes.Add(t));
        var system = DormandPrince54.PropagateSystem(
            (t, states) => [TwoBodyAccel(t, states[0])], [y0], 0, 3600,
            onAcceptedStep: (t, _, _) => systemTimes.Add(t));

        Assert.Equal(scalarTimes, systemTimes);
        Assert.Equal(scalar, system[0]);
    }

    [Fact]
    public void Duplicating_identical_bodies_does_not_change_system_steps()
    {
        double r = 7e6, v = Math.Sqrt(MuEarth / r);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        var singleTimes = new List<double>();
        var duplicateTimes = new List<double>();

        DormandPrince54.PropagateSystem(
            (t, states) => [TwoBodyAccel(t, states[0])], [y0], 0, 3600,
            onAcceptedStep: (t, _, _) => singleTimes.Add(t));
        var duplicate = DormandPrince54.PropagateSystem(
            (t, states) => [TwoBodyAccel(t, states[0]), TwoBodyAccel(t, states[1])],
            [y0, y0], 0, 3600,
            onAcceptedStep: (t, _, _) => duplicateTimes.Add(t));

        Assert.Equal(singleTimes, duplicateTimes);
        Assert.Equal(duplicate[0], duplicate[1]);
    }

    [Fact]
    public void Zero_duration_system_returns_an_exact_independent_copy_without_rhs_work()
    {
        StateVector[] initial =
        [
            new(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6)),
            new(new Vector3d(7, 8, 9), new Vector3d(10, 11, 12)),
        ];
        int calls = 0;
        bool accepted = false;

        var result = DormandPrince54.PropagateSystem(
            (t, states) => { calls++; return new Vector3d[states.Length]; },
            initial, 42, 42, onAcceptedStep: (t, states, derivative) => accepted = true);

        Assert.Equal(initial, result);
        Assert.NotSame(initial, result);
        Assert.Equal(0, calls);
        Assert.False(accepted);
    }

    [Fact]
    public void Backward_integration_throws()
    {
        var y0 = new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 7500, 0));
        Assert.Throws<ArgumentException>(() => DormandPrince54.Propagate(TwoBodyAccel, y0, 100, 0));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Non_finite_endpoints_are_rejected_before_rhs_work(double invalid)
    {
        var y0 = new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 7500, 0));
        int scalarCalls = 0;
        Vector3d Scalar(double t, StateVector state)
        {
            scalarCalls++;
            return Vector3d.Zero;
        }
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            DormandPrince54.Propagate(Scalar, y0, invalid, 10));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            DormandPrince54.Propagate(Scalar, y0, 0, invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            DormandPrince54.PropagateSystem(
                (t, states) =>
                {
                    scalarCalls++;
                    return new Vector3d[states.Length];
                }, [y0], invalid, 10));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            DormandPrince54.PropagateSystem(
                (t, states) =>
                {
                    scalarCalls++;
                    return new Vector3d[states.Length];
                }, [y0], 0, invalid));
        Assert.Equal(0, scalarCalls);
    }

    [Fact]
    public void Plunge_to_singularity_at_large_t_throws_instead_of_hanging()
    {
        // Near the singularity, the adaptive step falls below one ulp of the large
        // absolute time. The integrator must fail instead of ceasing to advance.
        double t0 = 3e8;
        var y0 = new StateVector(new Vector3d(1e5, 0, 0), new Vector3d(-5000, 0, 0));
        Assert.Throws<IntegrationFailureException>(
            () => DormandPrince54.Propagate(TwoBodyAccel, y0, t0, t0 + 1000));
    }

    [Fact]
    public void Divergent_dynamics_throws_instead_of_returning_nan()
    {
        static Vector3d Bad(double t, StateVector s) =>
            new(double.NaN, 0, 0);
        var y0 = new StateVector(new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        Assert.Throws<IntegrationFailureException>(() => DormandPrince54.Propagate(Bad, y0, 0, 10));
    }
}

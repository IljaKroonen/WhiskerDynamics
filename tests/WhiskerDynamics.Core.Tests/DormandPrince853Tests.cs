using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

/// <summary>Correctness and evaluation-cost checks for the eighth-order
/// integrator.</summary>
public class DormandPrince853Tests
{
    private const double MuEarth = 3.986004418e14;

    [Fact]
    public void Zero_duration_returns_without_evaluating_rhs()
    {
        var initial = new StateVector(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        int calls = 0;
        bool accepted = false;

        var end = DormandPrince853.Propagate(
            (t, s) => { calls++; return new Vector3d(7, 8, 9); },
            initial, 42, 42, out long rhs, onAcceptedStep: (t, s) => accepted = true);

        Assert.Equal(initial, end);
        Assert.Equal(0, rhs);
        Assert.Equal(0, calls);
        Assert.False(accepted);
    }

    private static Vector3d PointMass(Vector3d position)
    {
        double r2 = position.LengthSquared();
        return position * (-MuEarth / (r2 * Math.Sqrt(r2)));
    }

    [Fact]
    public void Circular_orbit_stays_circular_over_ten_revolutions()
    {
        double r = 6.771e6;
        double v = Math.Sqrt(MuEarth / r);
        double period = 2 * Math.PI * Math.Sqrt(r * r * r / MuEarth);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));

        var end = DormandPrince853.Propagate((t, s) => PointMass(s.Position),
            y0, 0, 10 * period, out _, new IntegratorOptions { RelTol = 1e-12 });

        Assert.Equal(r, end.Position.Length(), r * 1e-9);      // radius held
        Assert.Equal(v, end.Velocity.Length(), v * 1e-9);      // speed held
        Assert.True((end.Position - y0.Position).Length() < 50.0,
            $"ten-orbit closure error {(end.Position - y0.Position).Length()} m");
    }

    [Fact]
    public void Matches_production_dp54_on_an_eccentric_orbit()
    {
        // Eccentric motion exercises step-size adaptation near periapsis.
        double a = 2.0e7;
        double rp = a * (1 - 0.7);
        double vp = Math.Sqrt(MuEarth * (2 / rp - 1 / a));
        var y0 = new StateVector(new Vector3d(rp, 0, 0), new Vector3d(0, vp, 0));
        double period = 2 * Math.PI * Math.Sqrt(a * a * a / MuEarth);
        var opt = new IntegratorOptions { RelTol = 1e-12 };

        var dp54 = DormandPrince54.Propagate((t, s) => PointMass(s.Position), y0, 0, 3 * period, opt);
        var dp853 = DormandPrince853.Propagate((t, s) => PointMass(s.Position), y0, 0, 3 * period, out _, opt);

        Assert.True((dp54.Position - dp853.Position).Length() < 100.0,
            $"integrators disagree by {(dp54.Position - dp853.Position).Length()} m after 3 eccentric orbits");
    }

    [Fact]
    public void Uses_far_fewer_rhs_evaluations_than_dp54_at_shipping_tolerance()
    {
        double r = 6.771e6;
        double v = Math.Sqrt(MuEarth / r);
        var y0 = new StateVector(new Vector3d(r, 0, 0), new Vector3d(0, v, 0));
        var opt = new IntegratorOptions { RelTol = 1e-11 };
        double horizon = 86400.0;

        long dp54Rhs = 0;
        DormandPrince54.Propagate((t, s) => { dp54Rhs++; return PointMass(s.Position); }, y0, 0, horizon, opt);
        DormandPrince853.Propagate((t, s) => PointMass(s.Position), y0, 0, horizon, out long dp853Rhs, opt);

        Assert.True(dp853Rhs * 3 < dp54Rhs,
            $"expected >=3x fewer RHS calls, got DP54 {dp54Rhs} vs DP853 {dp853Rhs}");
    }
}

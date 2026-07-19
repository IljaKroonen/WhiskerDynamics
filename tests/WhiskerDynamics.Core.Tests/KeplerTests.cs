using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class KeplerTests
{
    // Mercury fixture from the game catalog, converted to SI.
    private static readonly OrbitalElements Mercury = new(
        SemiMajorAxis: 5.790896153292818e7 * 1000,
        Eccentricity: 2.056462028967717e-1,
        Inclination: 7.003433958539783 * Math.PI / 180,
        LongitudeOfAscendingNode: 4.829884437379905e1 * Math.PI / 180,
        ArgumentOfPeriapsis: 2.919855011314794e1 * Math.PI / 180,
        TimeAtPeriapsis: -563615.3399035392);

    private const double MuSun = Constants.G * Constants.SolarMassKg;

    [Fact]
    public void SolveEccentricAnomaly_satisfies_keplers_equation()
    {
        double e = 0.7, M = 2.5;
        double E = Kepler.SolveEccentricAnomaly(M, e);
        Assert.Equal(M, E - e * Math.Sin(E), 12);
    }

    [Fact]
    public void State_at_periapsis_has_expected_radius_and_visviva_speed()
    {
        var s = Kepler.StateFromElements(Mercury, MuSun, Mercury.TimeAtPeriapsis);
        double a = Mercury.SemiMajorAxis, e = Mercury.Eccentricity;
        double rPeri = a * (1 - e);
        double vPeri = Math.Sqrt(MuSun * (2 / rPeri - 1 / a));
        Assert.Equal(rPeri, s.Position.Length(), rPeri * 1e-12);
        Assert.Equal(vPeri, s.Velocity.Length(), vPeri * 1e-12);
        Assert.Equal(0, s.Position.Normalized().Dot(s.Velocity.Normalized()), 10);
    }

    [Fact]
    public void State_one_full_period_later_returns_to_start()
    {
        double a = Mercury.SemiMajorAxis;
        double period = 2 * Math.PI * Math.Sqrt(a * a * a / MuSun);
        var s0 = Kepler.StateFromElements(Mercury, MuSun, 0);
        var s1 = Kepler.StateFromElements(Mercury, MuSun, period);
        Assert.True((s1.Position - s0.Position).Length() < 1e-3 * s0.Position.Length());
    }

    [Fact]
    public void ElementsFromState_roundtrips_mercury()
    {
        double t = 1_000_000; // arbitrary time, away from periapsis
        var state = Kepler.StateFromElements(Mercury, MuSun, t);
        var el = Kepler.ElementsFromState(state, MuSun, t);
        Assert.Equal(Mercury.SemiMajorAxis, el.SemiMajorAxis, Mercury.SemiMajorAxis * 1e-10);
        Assert.Equal(Mercury.Eccentricity, el.Eccentricity, 10);
        Assert.Equal(Mercury.Inclination, el.Inclination, 10);
        Assert.Equal(Mercury.LongitudeOfAscendingNode, el.LongitudeOfAscendingNode, 10);
        Assert.Equal(Mercury.ArgumentOfPeriapsis, el.ArgumentOfPeriapsis, 8);
        // TimeAtPeriapsis may differ by whole periods; compare modulo period.
        double period = 2 * Math.PI * Math.Sqrt(Math.Pow(Mercury.SemiMajorAxis, 3) / MuSun);
        double dt = Math.IEEERemainder(el.TimeAtPeriapsis - Mercury.TimeAtPeriapsis, period);
        Assert.True(Math.Abs(dt) < 1.0, $"periapsis time off by {dt}s");
    }

    // Hyperbolic fixture using the parser's negative semi-major-axis convention.
    private static readonly OrbitalElements Hyperbolic = new(
        SemiMajorAxis: 2.0e11 / (1 - 6.28), // negative
        Eccentricity: 6.28,
        Inclination: 0.3,
        LongitudeOfAscendingNode: 1.1,
        ArgumentOfPeriapsis: 2.4,
        TimeAtPeriapsis: 5.0e6);

    [Fact]
    public void Hyperbolic_state_at_periapsis_has_expected_radius_and_speed()
    {
        var s = Kepler.StateFromElements(Hyperbolic, MuSun, Hyperbolic.TimeAtPeriapsis);
        double q = Hyperbolic.SemiMajorAxis * (1 - Hyperbolic.Eccentricity); // > 0
        double vPeri = Math.Sqrt(MuSun * (1 + Hyperbolic.Eccentricity) / q);
        Assert.Equal(q, s.Position.Length(), q * 1e-10);
        Assert.Equal(vPeri, s.Velocity.Length(), vPeri * 1e-10);
        Assert.Equal(0, s.Position.Normalized().Dot(s.Velocity.Normalized()), 10);
    }

    [Theory]
    [InlineData(-9e7)] // deep inbound
    [InlineData(-3600)]
    [InlineData(0)]
    [InlineData(86400)]
    [InlineData(9e7)]  // far outbound
    public void Hyperbolic_evaluation_conserves_energy_and_angular_momentum(double dtFromPeriapsis)
    {
        var s = Kepler.StateFromElements(Hyperbolic, MuSun, Hyperbolic.TimeAtPeriapsis + dtFromPeriapsis);
        double a = Hyperbolic.SemiMajorAxis;
        double energy = s.Velocity.LengthSquared() / 2 - MuSun / s.Position.Length();
        Assert.Equal(-MuSun / (2 * a), energy, Math.Abs(MuSun / (2 * a)) * 1e-9);
        double q = a * (1 - Hyperbolic.Eccentricity);
        double hExpected = q * Math.Sqrt(MuSun * (1 + Hyperbolic.Eccentricity) / q);
        Assert.Equal(hExpected, s.Position.Cross(s.Velocity).Length(), hExpected * 1e-9);
    }

    [Fact]
    public void Hyperbolic_elements_roundtrip_through_state()
    {
        double t = Hyperbolic.TimeAtPeriapsis + 4.5e6; // outbound leg
        var state = Kepler.StateFromElements(Hyperbolic, MuSun, t);
        var el = Kepler.ElementsFromState(state, MuSun, t);
        Assert.Equal(Hyperbolic.SemiMajorAxis, el.SemiMajorAxis, Math.Abs(Hyperbolic.SemiMajorAxis) * 1e-9);
        Assert.Equal(Hyperbolic.Eccentricity, el.Eccentricity, 8);
        Assert.Equal(Hyperbolic.Inclination, el.Inclination, 9);
        Assert.Equal(Hyperbolic.LongitudeOfAscendingNode, el.LongitudeOfAscendingNode, 9);
        Assert.Equal(Hyperbolic.ArgumentOfPeriapsis, el.ArgumentOfPeriapsis, 8);
        Assert.Equal(Hyperbolic.TimeAtPeriapsis, el.TimeAtPeriapsis,
            Math.Abs(Hyperbolic.TimeAtPeriapsis) * 1e-6);
    }

    [Fact]
    public void Universal_path_agrees_with_the_classic_elliptic_solve()
    {
        // Cross-check universal propagation against the classical elliptic solver.
        var el = new OrbitalElements(2.5e11, 0.85, 0.4, 0.9, 1.7, 0.0);
        var s0 = Kepler.StateFromElements(el, MuSun, 0);
        foreach (double dt in new[] { 3600.0, 86400.0, 3e6, -86400.0 })
        {
            var classic = Kepler.StateFromElements(el, MuSun, dt);
            var universal = Kepler.PropagateUniversal(s0, MuSun, dt);
            Assert.True((classic.Position - universal.Position).Length()
                < 1e-6 * classic.Position.Length(),
                $"dt={dt}: positions diverge");
            Assert.True((classic.Velocity - universal.Velocity).Length()
                < 1e-6 * classic.Velocity.Length(),
                $"dt={dt}: velocities diverge");
        }
    }

    [Fact]
    public void Near_parabolic_live_log_pair_evaluates_through_the_universal_path()
    {
        // This near-parabolic state does not converge in the classical solver.
        double a = 1e12, e = 0.9999300000000018;
        double n = Math.Sqrt(MuSun / (a * a * a));
        var el = new OrbitalElements(a, e, 0, 0, 0, -1.0680879827075224E-06 / n);
        var s = Kepler.StateFromElements(el, MuSun, 0);
        double energy = s.Velocity.LengthSquared() / 2 - MuSun / s.Position.Length();
        Assert.Equal(-MuSun / (2 * a), energy, MuSun / (2 * a) * 1e-6);
        Assert.Throws<InvalidOperationException>(
            () => Kepler.SolveEccentricAnomaly(1.0680879827075224E-06, e));
    }

    [Fact]
    public void PropagateUniversal_zero_dt_is_identity_and_inverse_returns()
    {
        var s0 = new StateVector(new Vector3d(1.5e11, 2e10, -4e9), new Vector3d(-5e3, 2.9e4, 1e3));
        Assert.Equal(s0.Position, Kepler.PropagateUniversal(s0, MuSun, 0).Position);
        var forth = Kepler.PropagateUniversal(s0, MuSun, 5e6);
        var back = Kepler.PropagateUniversal(forth, MuSun, -5e6);
        Assert.True((back.Position - s0.Position).Length() < 1e-6 * s0.Position.Length());
        Assert.True((back.Velocity - s0.Velocity).Length() < 1e-6 * s0.Velocity.Length());
    }

    [Fact]
    public void Near_rectilinear_periapsis_pass_falls_back_without_throwing()
    {
        // Near-rectilinear periapsis can make the Newton step overflow. Bisection
        // must recover a finite, energy-conserving state.
        double mu = 3.986004418e14;
        var state = new StateVector(
            new Vector3d(1.82e8, 0, 0), new Vector3d(2859.99, 5.12, 0)); // hyperbolic, near-radial
        foreach (double dt in new[] { -27400.0, 27400.0 })
        {
            var s = Kepler.PropagateUniversal(state, mu, dt);
            double energy0 = state.Velocity.LengthSquared() / 2 - mu / state.Position.Length();
            double energy1 = s.Velocity.LengthSquared() / 2 - mu / s.Position.Length();
            Assert.Equal(energy0, energy1, Math.Abs(energy0) * 1e-6);
        }
    }

    [Fact]
    public void StateFromElements_rejects_corrupt_and_parabolic_element_pairs()
    {
        Assert.Throws<NotSupportedException>(() => Kepler.StateFromElements(
            new OrbitalElements(-1e11, 0.5, 0, 0, 0, 0), MuSun, 0));  // e<1, a<=0
        Assert.Throws<NotSupportedException>(() => Kepler.StateFromElements(
            new OrbitalElements(1e11, 1.0, 0, 0, 0, 0), MuSun, 0));   // e>=1, a>=0
    }

    [Fact]
    public void ElementsFromState_accepts_escape_states_and_rejects_near_circular()
    {
        double mu = 3.986004418e14;
        var escape = new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 12000, 0));
        var el = Kepler.ElementsFromState(escape, mu, 0);
        Assert.True(el.Eccentricity > 1);
        Assert.True(el.SemiMajorAxis < 0);
        var reproduced = Kepler.StateFromElements(el, mu, 0);
        Assert.True((reproduced.Position - escape.Position).Length() < 1.0);
        Assert.True((reproduced.Velocity - escape.Velocity).Length() < 1e-3);
        double vCirc = Math.Sqrt(mu / 7e6);
        var circular = new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, vCirc, 0));
        Assert.Throws<NotSupportedException>(() => Kepler.ElementsFromState(circular, mu, 0));
    }
}

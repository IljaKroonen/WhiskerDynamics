using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

/// <summary>
/// Headless numerical gate for the field shared by live physics and the absolute
/// vessel predictor. This intentionally tests a clean velocity-Verlet diagnostic;
/// the verified-build stock ordering is exercised by the manual fidelity probe.
/// </summary>
public sealed class PhysicsRailsFidelityTests
{
    [Fact]
    public void Exact_field_velocity_verlet_converges_to_absolute_rails_truth()
    {
        IReadOnlyList<CelestialBody> bodies = AstronomicalsParser.ParseFile(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"));
        var ephemerides = new NBodyEphemerides(bodies, 0.0,
            bodies.Select(body => body.Id).ToArray(),
            new IntegratorOptions { RelTol = 1e-12 });
        CelestialBody mercury = ephemerides["Mercury"];
        CelestialBody[] sources = [ephemerides["Sol"], mercury];
        var relative0 = new StateVector(
            new Vector3d(2.74e6, 0, 0),
            new Vector3d(0, Math.Sqrt(mercury.Mu / 2.74e6), 0));
        const double duration = 3_000.0;
        // Pin the immutable rails representation before any comparison and give
        // each path its own mutable GravityModel segment cache.
        _ = ephemerides.GetState(mercury, duration + 60.0);
        StateVector parent0 = ephemerides.GetState(mercury, 0.0);
        StateVector absolute0 = parent0 + relative0;
        var referenceGravity = new GravityModel(ephemerides, sources);
        StateVector referenceAbsolute = DormandPrince853.Propagate(
            (time, state) => referenceGravity.AccelerationAt(state.Position, time),
            absolute0, 0.0, duration, out _,
            new IntegratorOptions
            {
                RelTol = 1e-13,
                AbsTolPos = 1e-7,
                AbsTolVel = 1e-10,
                InitialStep = 1.0,
                MaxStep = 10.0,
            });
        StateVector truth = referenceAbsolute - ephemerides.GetState(mercury, duration);

        StateVector coarse = Integrate(new GravityModel(ephemerides, sources),
            mercury, relative0, duration, 2.0,
            includePerturbation: true);
        StateVector fine = Integrate(new GravityModel(ephemerides, sources),
            mercury, relative0, duration, 0.5,
            includePerturbation: true);
        StateVector parentOnly = Integrate(new GravityModel(ephemerides, sources),
            mercury, relative0, duration, 0.5,
            includePerturbation: false);

        double coarsePosition = (coarse.Position - truth.Position).Length();
        double finePosition = (fine.Position - truth.Position).Length();
        double coarseVelocity = (coarse.Velocity - truth.Velocity).Length();
        double fineVelocity = (fine.Velocity - truth.Velocity).Length();
        double parentOnlyPosition = (parentOnly.Position - truth.Position).Length();

        Assert.True(finePosition < coarsePosition / 8.0,
            $"position did not converge: h=2 {coarsePosition:E6} m, h=.5 {finePosition:E6} m");
        Assert.True(fineVelocity < coarseVelocity / 8.0,
            $"velocity did not converge: h=2 {coarseVelocity:E6} m/s, h=.5 {fineVelocity:E6} m/s");
        Assert.True(parentOnlyPosition > 10.0 * finePosition,
            $"shared field was not material: with {finePosition:E6} m, parent-only {parentOnlyPosition:E6} m");
    }

    private static StateVector Integrate(GravityModel gravity, CelestialBody parent,
        StateVector initial, double duration, double step, bool includePerturbation)
    {
        StateVector state = initial;
        double time = 0.0;
        while (time < duration)
        {
            double h = Math.Min(step, duration - time);
            Vector3d a0 = Central(parent.Mu, state.Position)
                + (includePerturbation
                    ? gravity.ThirdBodyDeltaAt(parent, state.Position, time)
                    : Vector3d.Zero);
            Vector3d position = state.Position + state.Velocity * h + a0 * (0.5 * h * h);
            Vector3d a1 = Central(parent.Mu, position)
                + (includePerturbation
                    ? gravity.ThirdBodyDeltaAt(parent, position, time + h)
                    : Vector3d.Zero);
            state = new StateVector(position, state.Velocity + (a0 + a1) * (0.5 * h));
            time += h;
        }
        return state;
    }

    private static Vector3d Central(double mu, Vector3d position)
    {
        double r2 = position.LengthSquared();
        return position * (-mu / (r2 * Math.Sqrt(r2)));
    }
}

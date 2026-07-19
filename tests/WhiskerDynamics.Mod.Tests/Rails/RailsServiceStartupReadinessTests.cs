using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Rails;

public sealed class RailsServiceStartupReadinessTests
{
    [Fact]
    public void Binding_at_epoch_zero_serves_the_first_positive_celestial_tick()
    {
        var root = new CelestialBody { Id = "Root", Mu = 1.0e20 };
        var planet = new CelestialBody
        {
            Id = "Planet",
            Mu = 1.0e14,
            Parent = root,
            Orbit = new OrbitalElements(
                SemiMajorAxis: 1.0e8,
                Eccentricity: 0.01,
                Inclination: 0.02,
                LongitudeOfAscendingNode: 0.03,
                ArgumentOfPeriapsis: 0.04,
                TimeAtPeriapsis: 0.0),
        };

        // Production binds at Universe time zero, then CelestialUpdateTask can run
        // a fraction of a microsecond later, before the background grower splices its
        // first chunk. A workerless service makes that horizon race deterministic.
        using var rails = RailsService.CreateForSyntheticCatalog(
            [root, planet], [root.Id, planet.Id]);
        rails.PrepareAuthorityAt(0.0);

        const double firstJobTime = 1.4e-7;
        Assert.False(rails.IsReadyAt(firstJobTime));
        Assert.True(rails.TryGetParentRelativeEcl(
            planet.Id, firstJobTime, out var position, out var velocity));
        Assert.True(rails.IsReadyAt(firstJobTime));
        Assert.True(double.IsFinite(position.X + position.Y + position.Z));
        Assert.True(double.IsFinite(velocity.X + velocity.Y + velocity.Z));
    }
}

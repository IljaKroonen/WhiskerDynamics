using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public sealed class SegmentedEphemeridesSnapshotTests
{
    private static (CelestialBody Sun, CelestialBody Planet, CelestialBody Moon) System()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = 1.32712440018e20 };
        var planet = new CelestialBody
        {
            Id = "Planet", Mu = 3.986004418e14, Parent = sun,
            Orbit = new OrbitalElements(1.4959787e11, 0.0167, 0.01, 0.2, 0.7, 0),
        };
        var moon = new CelestialBody
        {
            Id = "Moon", Mu = 4.9028e12, Parent = planet,
            Orbit = new OrbitalElements(3.844e8, 0.055, 0.09, 0.1, 0.4, 0),
        };
        return (sun, planet, moon);
    }

    [Fact]
    public void Snapshot_matches_every_modeled_body_across_window()
    {
        var (sun, planet, moon) = System();
        var rails = new NBodyEphemerides([sun, planet, moon], 0,
            ["Sun", "Planet", "Moon"],
            new IntegratorOptions { RelTol = 1e-11 });
        rails.GetState(planet, 20 * 86400.0);
        var snapshot = rails.CreateSnapshot(86400.0, 19 * 86400.0);

        foreach (double t in new[] { 86400.0, 7.25 * 86400.0, 18.9 * 86400.0 })
        foreach (var body in new[] { sun, planet, moon })
        {
            var expected = rails.GetState(body, t);
            var actual = snapshot.GetState(body, t);
            Assert.True((actual.Position - expected.Position).Length() < 1e-6);
            Assert.True((actual.Velocity - expected.Velocity).Length() < 1e-9);
        }
    }

    [Fact]
    public void Snapshot_survives_owner_extension_and_pruning()
    {
        var (sun, planet, moon) = System();
        var rails = new NBodyEphemerides([sun, planet, moon], 0,
            ["Sun", "Planet", "Moon"]);
        rails.GetState(moon, 20 * 86400.0);
        var snapshot = rails.CreateSnapshot(2 * 86400.0, 18 * 86400.0);
        var before = snapshot.GetState(moon, 3 * 86400.0);

        rails.GetState(moon, 45 * 86400.0);
        rails.Prune(30 * 86400.0);

        Assert.Equal(before, snapshot.GetState(moon, 3 * 86400.0));
    }

    [Fact]
    public void Snapshot_gravity_matches_owner_and_enforces_range()
    {
        var (sun, planet, moon) = System();
        var bodies = new[] { sun, planet, moon };
        var rails = new NBodyEphemerides(bodies, 0, ["Sun", "Planet", "Moon"]);
        rails.GetState(moon, 10 * 86400.0);
        var snapshot = rails.CreateSnapshot(86400.0, 9 * 86400.0);
        var ownerGravity = new GravityModel(rails, bodies);
        var snapshotGravity = new GravityModel(snapshot, bodies);
        double t = 5 * 86400.0;
        var point = rails.GetState(moon, t).Position + new Vector3d(2e7, -3e7, 1e7);

        Assert.True((ownerGravity.AccelerationAt(point, t)
            - snapshotGravity.AccelerationAt(point, t)).Length() < 1e-12);
        Assert.Throws<ArgumentOutOfRangeException>(() => snapshot.GetState(moon, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => snapshot.GetState(moon, 10 * 86400.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => snapshot.GetState(moon, double.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            rails.CreateSnapshot(double.NaN, 2 * 86400.0));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            rails.CreateSnapshot(86400.0, double.PositiveInfinity));
    }

    [Fact]
    public void Narrow_snapshot_range_selection_is_independent_of_retained_history()
    {
        var times = Enumerable.Range(0, 100_001).Select(i => (double)i).ToArray();

        var narrow = NBodyEphemerides.SnapshotSegmentRange(times, 50_000.25, 50_002.75);
        Assert.Equal(50_001, narrow.FirstHi);
        Assert.Equal(3, narrow.EndHiExclusive - narrow.FirstHi);

        var exact = NBodyEphemerides.SnapshotSegmentRange(times, 50_000, 50_002);
        Assert.Equal(50_000, exact.FirstHi);
        Assert.Equal(4, exact.EndHiExclusive - exact.FirstHi);
    }

    [Fact]
    public void Narrow_snapshot_after_long_growth_matches_owner()
    {
        var (sun, planet, moon) = System();
        var rails = new NBodyEphemerides([sun, planet, moon], 0,
            [sun.Id, planet.Id, moon.Id]);
        const double day = 86400.0;
        rails.GetState(moon, 180 * day);
        double from = 150 * day + 123.0, to = from + 900.0;
        var snapshot = rails.CreateSnapshot(from, to);

        foreach (double t in new[] { from, (from + to) / 2, to })
        foreach (var body in new[] { sun, planet, moon })
        {
            var expected = rails.GetState(body, t);
            var actual = snapshot.GetState(body, t);
            Assert.True((actual.Position - expected.Position).Length() < 1e-6);
            Assert.True((actual.Velocity - expected.Velocity).Length() < 1e-9);
        }
    }

    [Fact]
    public void Combined_snapshot_matches_one_shot_state_and_gravity_across_boundaries()
    {
        var (sun, planet, moon) = System();
        var bodies = new[] { sun, planet, moon };
        var rails = new NBodyEphemerides(bodies, 0,
            [sun.Id, planet.Id, moon.Id],
            new IntegratorOptions { RelTol = 1e-11 });
        const double day = 86_400.0;
        rails.GetState(moon, 12 * day);

        var oneShot = rails.CreateSnapshot(day, 11 * day);
        var combined = SegmentedEphemeridesSnapshot.Combine([
            rails.CreateSnapshot(day, 4 * day),
            rails.CreateSnapshot(4 * day, 7 * day),
            rails.CreateSnapshot(7 * day, 11 * day),
        ]);
        var oneShotGravity = new GravityModel(oneShot, bodies);
        var combinedGravity = new GravityModel(combined, bodies);

        foreach (double time in new[]
            { day, 4 * day, 4.5 * day, 7 * day, 10.75 * day, 11 * day })
        {
            foreach (var body in bodies)
                Assert.Equal(oneShot.GetState(body, time), combined.GetState(body, time));
            var point = oneShot.GetState(moon, time).Position
                + new Vector3d(2e7, -3e7, 1e7);
            Assert.Equal(oneShotGravity.AccelerationAt(point, time),
                combinedGravity.AccelerationAt(point, time));
        }

        Assert.Equal(day, combined.StartTime);
        Assert.Equal(11 * day, combined.Horizon);
        Assert.Throws<ArgumentException>(() =>
            SegmentedEphemeridesSnapshot.Combine([
                rails.CreateSnapshot(day, 2 * day),
                rails.CreateSnapshot(3 * day, 4 * day),
            ]));
    }

    [Fact]
    public void Position_segment_boundaries_advance_across_combined_windows()
    {
        var (sun, planet, moon) = System();
        var rails = new NBodyEphemerides([sun, planet, moon], 0,
            [sun.Id, planet.Id, moon.Id]);
        const double day = 86_400.0;
        rails.GetState(moon, 12 * day);
        var snapshot = SegmentedEphemeridesSnapshot.Combine([
            rails.CreateSnapshot(day, 4 * day),
            rails.CreateSnapshot(4 * day, 7 * day),
            rails.CreateSnapshot(7 * day, 11 * day),
        ]);

        double cursor = day;
        int spans = 0;
        while (cursor < snapshot.Horizon)
        {
            double next = snapshot.PositionSegmentEndAfter(moon, cursor);
            Assert.True(next > cursor);
            Assert.True(next <= snapshot.Horizon);
            cursor = next;
            Assert.True(++spans < 10_000);
        }
        Assert.Equal(snapshot.Horizon,
            snapshot.PositionSegmentEndAfter(moon, snapshot.Horizon));
    }
}

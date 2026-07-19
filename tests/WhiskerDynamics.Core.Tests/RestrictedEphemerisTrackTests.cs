using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

public sealed class RestrictedEphemerisTrackTests
{
    private const double Step = 3600.0;
    private const int LastNode = 120;

    [Fact]
    public void Sequential_resolve_matches_snapshot_binary_lookup()
    {
        var track = CreateTrack();
        var snapshot = track.CreateSnapshot(track.StartTime, track.Horizon);

        for (double time = track.StartTime; time < track.Horizon; time += 1379.25)
            AssertEquivalent(track, snapshot, time);
        AssertEquivalent(track, snapshot, track.Horizon);
    }

    [Fact]
    public void Nonlocal_resolve_matches_snapshot_binary_lookup()
    {
        var track = CreateTrack();
        var snapshot = track.CreateSnapshot(track.StartTime, track.Horizon);
        double stable = track.StableThrough;
        double[] times =
        [
            track.Horizon - 0.25,
            stable + 0.25,
            track.Horizon - Step - 0.5,
            stable + Step + 0.5,
            stable - 0.25,
            Step + 0.25,
            stable - Step - 0.5,
            2 * Step + 0.5,
            track.Horizon,
            track.StartTime,
        ];

        foreach (double time in times)
            AssertEquivalent(track, snapshot, time);
    }

    [Fact]
    public void Exact_endpoints_resolve_as_exact_hits()
    {
        var track = CreateTrack();

        foreach (double time in new[] { track.StartTime, track.StableThrough, track.Horizon })
        {
            var segment = track.Resolve(time);
            Assert.Equal(time, segment.T0);
            Assert.Equal(0, segment.Dt);
        }
    }

    [Fact]
    public void Local_backward_query_to_an_exact_knot_returns_the_knot()
    {
        var track = CreateTrack();
        const double knotTime = 10 * Step;
        Assert.Equal(0, track.Resolve(knotTime).Dt);

        var after = track.Resolve(knotTime + 0.25);
        Assert.Equal(knotTime, after.T0);
        Assert.True(after.Dt > 0);

        var exact = track.Resolve(knotTime);
        Assert.Equal(knotTime, exact.T0);
        Assert.Equal(0, exact.Dt);
        Assert.Equal(StateAt(10), exact.A);
    }

    [Fact]
    public void Duplicate_exact_query_returns_the_first_equal_node()
    {
        var initial = StateAt(0);
        var track = new RestrictedEphemerisTrack(0, initial, AccelerationAt(0));
        var growth = new RestrictedEphemerisGrowth(0, initial, track.Generation);
        growth.Times.AddRange([Step, Step, 2 * Step]);
        growth.States.AddRange([StateAt(1), StateAt(2), StateAt(3)]);
        growth.Accelerations.AddRange([AccelerationAt(1), AccelerationAt(2), AccelerationAt(3)]);
        track.Append(growth);

        _ = track.Resolve(1.5 * Step);
        var exact = track.Resolve(Step);

        Assert.Equal(Step, exact.T0);
        Assert.Equal(0, exact.Dt);
        Assert.Equal(StateAt(1), exact.A);
    }

    private static RestrictedEphemerisTrack CreateTrack()
    {
        var initial = StateAt(0);
        var track = new RestrictedEphemerisTrack(0, initial, AccelerationAt(0));
        var growth = new RestrictedEphemerisGrowth(0, initial, track.Generation);
        for (int i = 1; i <= LastNode; i++)
        {
            growth.Times.Add(i * Step);
            growth.States.Add(StateAt(i));
            growth.Accelerations.Add(AccelerationAt(i));
        }

        Assert.True(track.Append(growth));
        Assert.InRange(track.StableThrough, track.StartTime + Step, track.Horizon - Step);
        Assert.True(track.NodeCount > 1);
        return track;
    }

    private static void AssertEquivalent(RestrictedEphemerisTrack track,
        NBodyEphemerides.BodySegment[] snapshot, double time)
    {
        var expectedSegment = snapshot.First(segment =>
            segment.T0 <= time && time <= segment.T0 + segment.Dt);
        var expected = NBodyEphemerides.SegmentState(in expectedSegment, time);
        var actualSegment = track.Resolve(time);
        var actual = NBodyEphemerides.SegmentState(in actualSegment, time);

        Assert.Equal(expected, actual);
    }

    private static StateVector StateAt(int i)
    {
        double sign = (i & 1) == 0 ? 1 : -1;
        return new StateVector(
            new Vector3d(sign * 1000 * i, 10 * i * i, 17 * i),
            new Vector3d(0.3 * i, -0.2 * i, 0.1 * i));
    }

    private static Vector3d AccelerationAt(int i) =>
        new((i % 3) - 1, (i % 5) - 2, (i % 7) - 3);
}

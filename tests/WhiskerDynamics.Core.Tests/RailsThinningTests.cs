using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

/// <summary>Accuracy, determinism, and storage bounds for per-body rail
/// thinning.</summary>
public class RailsThinningTests
{
    private const double MuSun = 1.32712440018e20;
    private const double MuEarth = 3.986004418e14;
    private const double MuMoon = 4.9028e12;

    private static (CelestialBody sun, CelestialBody earth, CelestialBody moon) SunEarthMoon()
    {
        var sun = new CelestialBody { Id = "Sun", Mu = MuSun };
        var earth = new CelestialBody
        {
            Id = "Earth", Mu = MuEarth, Parent = sun,
            Orbit = new OrbitalElements(1.49598023e11, 0.0167086, 0, 0, 1.79676742, 0),
        };
        var moon = new CelestialBody
        {
            Id = "Moon", Mu = MuMoon, Parent = earth,
            Orbit = new OrbitalElements(3.844e8, 0.0549, 5.145 * Math.PI / 180, 0, 0, 0),
        };
        return (sun, earth, moon);
    }

    [Fact]
    public void Detached_growth_is_bit_identical_to_in_lock_extension()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var options = new IntegratorOptions { RelTol = 1e-11 };
        var direct = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        var grown = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        var grower = grown.CreateGrower();

        // Identical chunk boundaries and arithmetic must produce identical states.
        for (int chunk = 1; chunk <= 8; chunk++)
        {
            double end = chunk * 0.5 * 86400;
            direct.GetState(earth, end); // in-lock extension path
            double seed = grower.CaptureSeed();
            grower.Integrate(end);
            Assert.True(grower.TrySplice(), "uncontended splice must land");
            Assert.Equal(seed, end - 0.5 * 86400);
        }
        Assert.Equal(direct.Horizon, grown.Horizon);
        foreach (var body in new[] { sun, earth, moon })
            foreach (double t in new[] { 0.3 * 86400, 1.7 * 86400, 3.99 * 86400 })
            {
                Assert.Equal(direct.GetState(body, t).Position, grown.GetState(body, t).Position);
                Assert.Equal(direct.GetState(body, t).Velocity, grown.GetState(body, t).Velocity);
            }
        Assert.Equal(direct.KnotCount, grown.KnotCount);
        Assert.Equal(direct.NodeCount, grown.NodeCount);
    }

    [Fact]
    public void Refused_splice_after_a_safety_net_extension_discards_cleanly()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"],
            new IntegratorOptions { RelTol = 1e-11 });
        var grower = eph.CreateGrower();

        double seed = grower.CaptureSeed();
        grower.Integrate(seed + 0.5 * 86400);
        // Simulate a synchronous extension between detached capture and commit.
        eph.GetState(earth, seed + 0.2 * 86400 + 1.0);
        double movedHorizon = eph.Horizon;
        Assert.False(grower.TrySplice(), "a moved tip must refuse the stale chunk");
        Assert.Equal(movedHorizon, eph.Horizon); // nothing appended

        double seed2 = grower.CaptureSeed();
        Assert.Equal(movedHorizon, seed2);
        grower.Integrate(seed2 + 0.5 * 86400);
        Assert.True(grower.TrySplice());
        Assert.True(eph.Horizon >= seed2 + 0.5 * 86400);
        var state = eph.GetState(moon, seed2 + 0.4 * 86400); // spans the splice boundary
        Assert.True(double.IsFinite(state.Position.X));
    }

    [Fact]
    public void Detached_grower_integrates_only_once_per_capture()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var eph = new NBodyEphemerides([sun, earth, moon], 0,
            ["Sun", "Earth", "Moon"], new IntegratorOptions { RelTol = 1e-11 });
        var grower = eph.CreateGrower();

        double seed = grower.CaptureSeed();
        grower.Integrate(seed + 1000);
        Assert.Throws<InvalidOperationException>(() => grower.Integrate(seed + 2000));
        Assert.True(grower.TrySplice());

        double nextSeed = grower.CaptureSeed();
        grower.Integrate(nextSeed + 1000);
        Assert.True(grower.TrySplice());
        Assert.True(eph.Horizon >= nextSeed + 1000);
    }

    /// <summary>Dense one-shot reference using the ephemeris seeding and pairwise
    /// acceleration kernel.</summary>
    private static (List<double> times, List<StateVector[]> nodes) DenseReference(
        CelestialBody[] integrated, double endTime, IntegratorOptions options)
    {
        // Reuse production seeding so the comparison isolates interpolation.
        var initial = NBodyEphemerides.SeedBarycentric(integrated, new Ephemerides(integrated), 0.0);
        var pairwise = new PairwiseAccelerationKernel(integrated.Select(b => b.Mu).ToArray());
        var accBuffer = new Vector3d[integrated.Length];
        var times = new List<double> { 0.0 };
        var nodes = new List<StateVector[]> { (StateVector[])initial.Clone() };
        DormandPrince54.PropagateSystem(
            (t, states) => { pairwise.Compute(states, accBuffer); return accBuffer; },
            initial, 0, endTime, options,
            (t, y, _) => { times.Add(t); nodes.Add((StateVector[])y.Clone()); });
        return (times, nodes);
    }

    [Fact]
    public void Committed_quintic_reproduces_the_dense_integration_within_budget()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var options = new IntegratorOptions { RelTol = 1e-11 };
        double end = 30 * 86400;
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        eph.GetState(earth, end); // one-shot extension; commitment trails by the gap cap

        var (times, nodes) = DenseReference([sun, earth, moon], end, options);
        double committedEnd = end - 2 * NBodyEphemerides.KnotGapCapSeconds;
        int checked_ = 0;
        var bodies = new[] { sun, earth, moon };
        for (int i = 0; i < times.Count && times[i] <= committedEnd; i += 7)
        {
            for (int b = 0; b < bodies.Length; b++)
            {
                var interpolated = eph.GetState(bodies[b], times[i]);
                double posError = (interpolated.Position - nodes[i][b].Position).Length();
                Assert.True(posError <= NBodyEphemerides.KnotPositionBudget,
                    $"{bodies[b].Id} at t={times[i]:F0}: committed store off by {posError} m");
                double velError = (interpolated.Velocity - nodes[i][b].Velocity).Length();
                Assert.True(velError <= 0.05,
                    $"{bodies[b].Id} at t={times[i]:F0}: committed velocity off by {velError} m/s");
            }
            checked_++;
        }
        Assert.True(checked_ > 15, $"only {checked_} committed reference nodes checked");
    }

    [Fact]
    public void Uncommitted_tail_returns_dense_nodes_bit_exactly()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var options = new IntegratorOptions { RelTol = 1e-11 };
        double end = 30 * 86400;
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        eph.GetState(earth, end);

        var (times, nodes) = DenseReference([sun, earth, moon], end, options);
        var bodies = new[] { sun, earth, moon };
        int checked_ = 0;
        for (int b = 0; b < bodies.Length; b++)
        {
            double committedEnd = eph.LastKnotTime(eph.IntegratedIndexOf(bodies[b]));
            for (int i = times.Count - 1; i > 0 && times[i] > committedEnd; i--)
            {
                var state = eph.GetState(bodies[b], times[i]);
                Assert.Equal(nodes[i][b].Position, state.Position); // node-exact: bit-identical
                Assert.Equal(nodes[i][b].Velocity, state.Velocity);
                checked_++;
            }
        }
        // The horizon node remains in the uncommitted tail.
        Assert.True(checked_ >= bodies.Length, $"only {checked_} tail nodes checked");
    }

    [Fact]
    public void Knot_store_is_a_small_fraction_of_the_dense_sequence()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var options = new IntegratorOptions { RelTol = 1e-11 };
        double end = 60 * 86400;
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        eph.GetState(earth, end);

        var (times, _) = DenseReference([sun, earth, moon], end, options);
        // Even this small, fast-moon system must retain fewer knots than its dense
        // equivalent.
        long denseEquivalentBytes = (long)times.Count * (8 + 3 * (48 + 24));
        Assert.True(eph.ApproxBytes < denseEquivalentBytes,
            $"retained ~{eph.ApproxBytes} B vs dense-equivalent ~{denseEquivalentBytes} B "
            + $"({eph.KnotCount} knots + {eph.NodeCount} tail nodes vs {times.Count} dense nodes)");
        Assert.True(eph.NodeCount < times.Count / 4,
            $"dense tail {eph.NodeCount} should be gap-cap bounded, dense total {times.Count}");
    }

    [Fact]
    public void Region_transition_moves_a_value_by_no_more_than_the_budget()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var options = new IntegratorOptions { RelTol = 1e-11 };
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        eph.GetState(earth, 5 * 86400);
        int earthIndex = eph.IntegratedIndexOf(earth);
        // Any time after the last committed knot lies in the dense tail.
        double t = 0.5 * (eph.LastKnotTime(earthIndex) + eph.Horizon);
        var tail = eph.GetState(earth, t); // uncommitted: dense evaluation
        eph.GetState(earth, t + 5 * NBodyEphemerides.KnotGapCapSeconds);
        Assert.True(eph.LastKnotTime(earthIndex) > t, "extension should have committed past t");
        var committed = eph.GetState(earth, t); // now behind the knots: quintic evaluation
        // Between dense nodes, both interpolants contribute local error.
        Assert.True((tail.Position - committed.Position).Length()
                <= 2 * NBodyEphemerides.KnotPositionBudget,
            $"commit transition moved the value {(tail.Position - committed.Position).Length()} m");
    }

    [Fact]
    public void Gravity_simd_path_matches_scalar_segment_evaluation()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var options = new IntegratorOptions { RelTol = 1e-11 };
        var eph = new NBodyEphemerides([sun, earth, moon], 0, ["Sun", "Earth", "Moon"], options);
        var gravity = new GravityModel(eph, [sun, earth, moon]);
        eph.GetState(earth, 20 * 86400);

        // Cover the committed region, dense tail, and repeated segment rebuilds.
        foreach (double t in new[] { 3 * 86400.0, 19.9 * 86400.0, 5 * 86400.0, 5.01 * 86400.0, 12 * 86400.0 })
        {
            var p = eph.GetState(earth, t).Position + new Vector3d(7e6, 1e6, -2e6);
            var simd = gravity.AccelerationAt(p, t);
            var scalar = Vector3d.Zero;
            foreach (var body in new[] { sun, earth, moon })
            {
                var offset = p - eph.GetState(body, t).Position;
                double r2 = offset.LengthSquared();
                scalar -= offset * (body.Mu / (r2 * Math.Sqrt(r2)));
            }
            Assert.True((simd - scalar).Length() <= 1e-10 * scalar.Length(),
                $"SIMD vs scalar at t={t:F0}: {(simd - scalar).Length():E2} vs |a|={scalar.Length():E2}");
        }
    }

    [Fact]
    public void Gravity_cache_rejects_a_segment_after_ephemeris_pruning()
    {
        var (sun, earth, moon) = SunEarthMoon();
        var eph = new NBodyEphemerides([sun, earth, moon], 0,
            ["Sun", "Earth", "Moon"], new IntegratorOptions { RelTol = 1e-11 });
        var gravity = new GravityModel(eph, [sun, earth, moon]);
        double oldTime = 5 * 86400.0;
        eph.GetState(earth, 60 * 86400.0);
        Vector3d point = eph.GetState(earth, oldTime).Position
            + new Vector3d(7e6, 1e6, -2e6);
        _ = gravity.AccelerationAt(point, oldTime); // warm the old committed segment

        eph.Prune(20 * 86400.0);

        Assert.True(eph.StartTime > oldTime);
        Assert.Throws<ArgumentOutOfRangeException>(() => eph.GetState(earth, oldTime));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            gravity.AccelerationAt(point, oldTime));
    }
}

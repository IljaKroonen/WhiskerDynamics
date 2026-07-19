using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Planning.Rendezvous;

public class RendezvousKernelTests
{
    private const double Mu = 3.986004418e14;
    private const double Radius = 7_000_000.0;

    private readonly record struct RankedTransfer(string Id, double Score, double Duration);

    [Fact]
    public void Longer_window_does_not_evict_a_viable_shorter_duration_band()
    {
        var finalists = new DurationDiverseSet<RankedTransfer>(12, 4, 24, 5400,
            transfer => transfer.Score, transfer => transfer.Duration);

        // Many cheaper long arcs must not consume every duration band.
        for (int i = 0; i < 20; i++)
            finalists.Add(new RankedTransfer($"fragile-{i}", 3700 + i, 4 * 86400 + i));

        // Preserve a more expensive one-day candidate.
        finalists.Add(new RankedTransfer("robust-one-day", 4128, 23.25 * 3600));

        Assert.Contains(finalists.Values, transfer => transfer.Id == "robust-one-day");
        Assert.Equal(12, finalists.Values.Count(
            transfer => transfer.Id.StartsWith("fragile-")));
    }

    [Fact]
    public void Diverse_finalists_are_distinct_and_hard_capped()
    {
        var finalists = new DurationDiverseSet<RankedTransfer>(2, 2, 5, 100,
            transfer => transfer.Score, transfer => transfer.Duration);
        var first = new RankedTransfer("first", 1, 100);
        var second = new RankedTransfer("second", 2, 101);
        finalists.Add(first);
        finalists.Add(second);
        finalists.Add(first);
        for (int band = 1; band < 8; band++)
            finalists.Add(new RankedTransfer($"band-{band}", 10 + band,
                100 * Math.Pow(2, band)));

        Assert.Equal(5, finalists.Values.Count);
        Assert.Contains(first, finalists.Values);
        Assert.Contains(second, finalists.Values);
        Assert.Equal(finalists.Values.Count, finalists.Values.Distinct().Count());
    }

    [Fact]
    public void Diverse_finalists_reserve_the_global_ranking_at_capacity()
    {
        var finalists = new DurationDiverseSet<RankedTransfer>(3, 1, 5, 100,
            transfer => transfer.Score, transfer => transfer.Duration);
        var globals = Enumerable.Range(0, 3)
            .Select(i => new RankedTransfer($"global-{i}", i, 100)).ToArray();
        foreach (var transfer in globals) finalists.Add(transfer);
        for (int band = 1; band < 10; band++)
            finalists.Add(new RankedTransfer($"leader-{band}", 100 + band,
                100 * Math.Pow(2, band)));

        Assert.Equal(5, finalists.Values.Count);
        Assert.All(globals, transfer => Assert.Contains(transfer, finalists.Values));
    }

    [Fact]
    public void Lambert_short_way_reaches_the_requested_position()
    {
        double period = 2 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);
        var from = new Vector3d(Radius, 0, 0);
        var to = new Vector3d(Radius * 0.5, Radius * Math.Sqrt(3) / 2, 0);

        Assert.True(RendezvousKernel.TryLambert(from, to, period / 6, Mu,
            longWay: false, out var transfer));

        var arrived = Kepler.PropagateUniversal(
            new StateVector(from, transfer.DepartureVelocity), Mu, period / 6);
        Assert.True((arrived.Position - to).Length() < 0.1,
            $"Lambert miss was {(arrived.Position - to).Length():G6} m");
        Assert.True((arrived.Velocity - transfer.ArrivalVelocity).Length() < 1e-6);
    }

    [Fact]
    public void Lambert_long_way_reaches_the_requested_position()
    {
        double period = 2 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);
        var from = new Vector3d(Radius, 0, 0);
        var to = new Vector3d(Radius * 0.5, Radius * Math.Sqrt(3) / 2, 0);

        Assert.True(RendezvousKernel.TryLambert(from, to, 5 * period / 6, Mu,
            longWay: true, out var transfer));
        var arrived = Kepler.PropagateUniversal(
            new StateVector(from, transfer.DepartureVelocity), Mu, 5 * period / 6);
        Assert.True((arrived.Position - to).Length() < 0.1);
    }

    [Fact]
    public void Lambert_one_revolution_includes_the_circular_phasing_arc()
    {
        double period = 2 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);
        double circularSpeed = Math.Sqrt(Mu / Radius);
        var from = new Vector3d(Radius, 0, 0);
        var to = new Vector3d(Radius * 0.5, Radius * Math.Sqrt(3) / 2, 0);
        double tof = 7 * period / 6;

        var solutions = RendezvousKernel.SolveLambert(from, to, tof, Mu,
            longWay: false, revolutions: 1);

        Assert.NotEmpty(solutions);
        foreach (var solution in solutions)
        {
            var endpoint = Kepler.PropagateUniversal(
                new StateVector(from, solution.DepartureVelocity), Mu, tof);
            Assert.True((endpoint.Position - to).Length() < 0.1);
        }
        var circular = solutions.MinBy(s =>
            (s.DepartureVelocity - new Vector3d(0, circularSpeed, 0)).Length());
        Assert.Equal(1, circular.Revolutions);
        Assert.True((circular.DepartureVelocity - new Vector3d(0, circularSpeed, 0)).Length() < 1e-5);
        var arrived = Kepler.PropagateUniversal(
            new StateVector(from, circular.DepartureVelocity), Mu, tof);
        Assert.True((arrived.Position - to).Length() < 0.1,
            $"one-revolution Lambert miss was {(arrived.Position - to).Length():G6} m");
    }

    [Fact]
    public void Revolution_sampling_is_complete_when_small_and_spans_long_windows()
    {
        Assert.Equal([0, 1, 2, 3], RendezvousKernel.RevolutionSamples(3, 8));
        var sampled = RendezvousKernel.RevolutionSamples(1000, 16);
        Assert.Equal(16, sampled.Length);
        Assert.Equal(0, sampled[0]);
        Assert.Equal(1000, sampled[^1]);
        Assert.All(Enumerable.Range(0, 8), revolution => Assert.Contains(revolution, sampled));
        Assert.True(sampled.SequenceEqual(sampled.Order()));
        Assert.Equal(sampled.Length, sampled.Distinct().Count());
        Assert.Equal([0, 1000], RendezvousKernel.RevolutionSamples(1000, 2));
        Assert.Equal([0, 1, 1000], RendezvousKernel.RevolutionSamples(1000, 3));
    }

    [Fact]
    public void Lambert_two_revolutions_reaches_the_requested_position()
    {
        double period = 2 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);
        var from = new Vector3d(Radius, 0, 0);
        var to = new Vector3d(Radius * 0.5, Radius * Math.Sqrt(3) / 2, 0);
        double tof = 13 * period / 6;

        var solutions = RendezvousKernel.SolveLambert(from, to, tof, Mu,
            longWay: false, revolutions: 2);

        Assert.NotEmpty(solutions);
        Assert.All(solutions, solution =>
        {
            Assert.Equal(2, solution.Revolutions);
            var endpoint = Kepler.PropagateUniversal(
                new StateVector(from, solution.DepartureVelocity), Mu, tof);
            Assert.True((endpoint.Position - to).Length() < 0.1);
        });
    }

    [Fact]
    public void Lambert_rejects_degenerate_inputs()
    {
        Assert.False(RendezvousKernel.TryLambert(Vector3d.Zero, new Vector3d(1, 0, 0),
            100, Mu, false, out _));
        Assert.False(RendezvousKernel.TryLambert(new Vector3d(1, 0, 0), new Vector3d(2, 0, 0),
            100, Mu, false, out _));
        Assert.False(RendezvousKernel.TryLambert(new Vector3d(1, 0, 0), new Vector3d(0, 1, 0),
            -1, Mu, false, out _));
    }

    [Fact]
    public void OrbitalPeriod_matches_a_circular_orbit()
    {
        double speed = Math.Sqrt(Mu / Radius);
        double expected = 2 * Math.PI * Math.Sqrt(Radius * Radius * Radius / Mu);
        double actual = RendezvousKernel.OrbitalPeriod(
            new StateVector(new Vector3d(Radius, 0, 0), new Vector3d(0, speed, 0)), Mu);
        Assert.Equal(expected, actual, 8);
    }

    [Fact]
    public void PeriapsisDistance_matches_circular_and_eccentric_states()
    {
        double circularSpeed = Math.Sqrt(Mu / Radius);
        Assert.Equal(Radius, RendezvousKernel.PeriapsisDistance(
            new StateVector(new Vector3d(Radius, 0, 0),
                new Vector3d(0, circularSpeed, 0)), Mu), 6);

        const double apoapsis = 14_000_000.0;
        const double periapsis = 7_000_000.0;
        double semiMajor = (apoapsis + periapsis) / 2;
        double apoapsisSpeed = Math.Sqrt(Mu * (2 / apoapsis - 1 / semiMajor));
        Assert.Equal(periapsis, RendezvousKernel.PeriapsisDistance(
            new StateVector(new Vector3d(apoapsis, 0, 0),
                new Vector3d(0, apoapsisSpeed, 0)), Mu), 5);
    }

    [Fact]
    public void Linear_three_by_three_solver_recovers_the_vector()
    {
        var expected = new Vector3d(2, -3, 4);
        var c0 = new Vector3d(2, 1, 0);
        var c1 = new Vector3d(0, 3, 1);
        var c2 = new Vector3d(1, 0, 4);
        var rhs = c0 * expected.X + c1 * expected.Y + c2 * expected.Z;
        Assert.True(RendezvousKernel.TrySolveLinear3(c0, c1, c2, rhs, out var actual));
        Assert.True((actual - expected).Length() < 1e-12);
    }

    [Fact]
    public void Pivoted_linear_solver_recovers_a_six_state_correction()
    {
        var matrix = new double[,]
        {
            { 0, 2, 0, 0, 0, 1 },
            { 3, 0, 1, 0, 0, 0 },
            { 0, 1, 4, 0, 0, 0 },
            { 0, 0, 0, 5, 1, 0 },
            { 0, 0, 0, 0, 6, 1 },
            { 1, 0, 0, 0, 0, 7 },
        };
        double[] expected = [1, -2, 3, -4, 5, -6];
        var rhs = new double[6];
        for (int row = 0; row < 6; row++)
        for (int column = 0; column < 6; column++)
            rhs[row] += matrix[row, column] * expected[column];

        Assert.True(RendezvousKernel.TrySolveLinear(matrix, rhs, out var actual));
        for (int k = 0; k < 6; k++) Assert.Equal(expected[k], actual[k], 10);
    }
}

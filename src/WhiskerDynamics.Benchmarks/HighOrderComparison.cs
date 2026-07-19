using System.Diagnostics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Manual DP5(4)/DOP853 comparison over the shared vessel cases, reporting
/// runtime, RHS evaluations, accepted steps, and deviation from a tighter reference.</summary>
public static class HighOrderComparison
{
    public static void Run()
    {
        var (gravity, leo, highOrbit) = BenchmarkCatalog.CreateVesselCases();
        Compare("LEO 30 d (400 km circular)", gravity, leo);
        Compare("High orbit 30 d (100,000 km circular)", gravity, highOrbit);
    }

    private static void Compare(string label, GravityModel gravity, StateVector initial)
    {
        var shipping = new IntegratorOptions { RelTol = BenchmarkCatalog.ShippingRelTol };
        var reference = new IntegratorOptions { RelTol = BenchmarkCatalog.ShippingRelTol / 100 };

        Vector3d Rhs(double t, StateVector s) => gravity.AccelerationAt(s.Position, t);

        var truth = DormandPrince54.Propagate(
            Rhs, initial, 0, BenchmarkCatalog.VesselHorizonSeconds, reference);

        Console.WriteLine();
        Console.WriteLine($"== {label}, RelTol {shipping.RelTol:0e0}, reference = DP54 @ {reference.RelTol:0e0} ==");
        Report("DP5(4)7M (production)", truth, () =>
        {
            long rhs = 0;
            int steps = 0;
            var end = DormandPrince54.Propagate((t, s) => { rhs++; return Rhs(t, s); },
                initial, 0, BenchmarkCatalog.VesselHorizonSeconds, shipping, (t, s) => steps++);
            return (end, rhs, steps);
        });
        Report("DP853 8(5,3)", truth, () =>
        {
            int steps = 0;
            var end = DormandPrince853.Propagate(Rhs, initial, 0, BenchmarkCatalog.VesselHorizonSeconds,
                out long rhs, shipping, (t, s) => steps++);
            return (end, rhs, steps);
        });
    }

    private static void Report(string name, StateVector truth, Func<(StateVector End, long Rhs, int Steps)> run)
    {
        run(); // warmup (tier-up + rails segment-hint locality)
        var sw = new Stopwatch();
        const int reps = 5;
        (StateVector End, long Rhs, int Steps) last = default;
        double best = double.MaxValue;
        for (int i = 0; i < reps; i++)
        {
            sw.Restart();
            last = run();
            sw.Stop();
            best = Math.Min(best, sw.Elapsed.TotalMilliseconds);
        }
        double posErr = (last.End.Position - truth.Position).Length();
        double velErr = (last.End.Velocity - truth.Velocity).Length();
        Console.WriteLine(
            $"{name,-24} {best,9:F1} ms   RHS {last.Rhs,9:N0}   steps {last.Steps,7:N0}   " +
            $"vs ref: pos {posErr,10:F3} m  vel {velErr,9:F6} m/s");
    }
}

using System.Diagnostics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Measures mutual-backbone storage rates after thinning dense accepted nodes
/// to cubic or quintic Hermite knots within a position budget. Restricted-track storage
/// is measured by the composite ephemeris benchmarks instead.</summary>
public static class ThinningProbe
{
    private const int CubicKnotBytes = 8 + 24 + 24;        // t, pos, vel
    private const int QuinticKnotBytes = 8 + 24 + 24 + 24; // t, pos, vel, acc
    private static readonly double[] InteriorFractions =
        [0.21132486540518713, 0.5, 0.7886751345948129];

    internal const string ValidationScope =
        "validation scope: selected spans meet the position budget at every accepted "
        + "dense node and at three fixed interior times per dense step against the dense "
        + "cubic-Hermite reference; this finite sample does not prove a continuous-time maximum.";

    public static int Run(string systemXml, string astronomicals, double days)
    {
        var bodies = SystemManifestLoader.Load(systemXml, astronomicals);
        var backboneIds = IntegratedSetRule.Select(bodies, 0, out _);
        var backbone = bodies.Where(b => backboneIds.Contains(b.Id)).ToArray();
        int n = backbone.Length;
        Console.WriteLine($"catalog: {bodies.Count} modeled bodies, {n} massive backbone, "
            + $"{bodies.Count - n} restricted, window {days:G} d, "
            + $"gap cap {NBodyEphemerides.KnotGapCapSeconds / 86400:F0} d");

        var initial = NBodyEphemerides.SeedBarycentric(backbone, new Ephemerides(bodies), 0.0);

        var mus = backbone.Select(b => b.Mu).ToArray();
        var accBuffer = new Vector3d[n];
        var times = new List<double> { 0 };
        var nodes = new List<StateVector[]> { (StateVector[])initial.Clone() };

        var sw = Stopwatch.StartNew();
        DormandPrince54.PropagateSystem(
            (t, states) => { PairwiseScalar(mus, states, accBuffer); return accBuffer; },
            initial, 0, days * 86400, new IntegratorOptions { RelTol = 1e-11 },
            (t, y, _) => { times.Add(t); nodes.Add((StateVector[])y.Clone()); });
        sw.Stop();
        int dense = times.Count;
        long denseBytes = (long)dense * (8 + n * 48);
        Console.WriteLine($"dense fill: {sw.Elapsed.TotalSeconds:F2} s, {dense} nodes "
            + $"({dense / days:F0}/day), ~{denseBytes / (1024 * 1024)} MB "
            + $"({denseBytes / (1024.0 * 1024.0) / days:F2} MB/day)");
        Console.WriteLine("selection search: candidate endpoints inside the gap cap are "
            + "tested from latest to earliest; an adjacent dense node beyond the cap remains "
            + "the required fallback. The first passing endpoint is therefore the furthest "
            + "passing candidate, without assuming validity is monotone.");
        Console.WriteLine(ValidationScope);

        // Per-node accelerations (quintic knots carry them; validation interpolants need them).
        var accs = new Vector3d[dense][];
        for (int k = 0; k < dense; k++)
        {
            var a = new Vector3d[n];
            PairwiseScalar(mus, nodes[k], a);
            accs[k] = a;
        }

        foreach (bool quintic in new[] { false, true })
            foreach (double budget in new[] { 1.0, 10.0, 100.0 })
                Report(quintic, budget, backbone, times, nodes, accs, days);
        return 0;
    }

    private static void Report(bool quintic, double budget, CelestialBody[] backbone,
        List<double> times, List<StateVector[]> nodes, Vector3d[][] accs, double days)
    {
        int n = backbone.Length;
        int knotBytes = quintic ? QuinticKnotBytes : CubicKnotBytes;
        long totalKnots = 0;
        long candidateSpans = 0;
        long acceptedNodeChecks = 0;
        long interiorProbeChecks = 0;
        double worstPos = 0;
        double worstVel = 0;
        var perBody = new (string Id, int Knots)[n];
        for (int b = 0; b < n; b++)
        {
            var selection = SelectKnots(b, quintic, budget, times, nodes, accs);
            perBody[b] = (backbone[b].Id, selection.Knots.Count);
            totalKnots += selection.Knots.Count;
            candidateSpans += selection.CandidateSpans;
            acceptedNodeChecks += selection.AcceptedNodeChecks;
            interiorProbeChecks += selection.InteriorProbeChecks;
            worstPos = Math.Max(worstPos, selection.MaxPositionError);
            worstVel = Math.Max(worstVel, selection.MaxVelocityError);
        }
        double bytesPerDay = totalKnots * (double)knotBytes / days;
        Console.WriteLine($"\n== {(quintic ? "quintic" : "cubic")} Hermite, pos budget {budget:F0} m ==");
        Console.WriteLine($"total knots: {totalKnots} ({totalKnots / days:F0}/day), "
            + $"{bytesPerDay / 1024:F1} KB/day -> 1y {bytesPerDay * 365 / (1024 * 1024):F0} MB, "
            + $"10y {bytesPerDay * 3650 / (1024 * 1024):F0} MB, 40y {bytesPerDay * 14600 / (1024 * 1024):F0} MB");
        Console.WriteLine($"max position error at adopted validation samples: "
            + $"{worstPos:E2} m (budget {budget:F0} m)");
        Console.WriteLine($"max velocity error at the same samples (measured, not gated): "
            + $"{worstVel:E2} m/s");
        Console.WriteLine($"selection work: {candidateSpans:N0} candidate spans, "
            + $"{acceptedNodeChecks:N0} accepted-node checks, "
            + $"{interiorProbeChecks:N0} independent interior probes");
        Console.WriteLine("densest bodies: " + string.Join(", ", perBody
            .OrderByDescending(p => p.Knots).Take(8)
            .Select(p => $"{p.Id} {p.Knots / days:F0}/d")));
    }

    /// <summary>Greedy knot selection for one body. At each anchor it tests endpoints
    /// backward from the time-gap cap and adopts the first independently validated one,
    /// which is the furthest passing candidate even when validity oscillates.</summary>
    internal static SelectionResult SelectKnots(int body, bool quintic, double budget,
        IReadOnlyList<double> times, IReadOnlyList<StateVector[]> nodes, Vector3d[][] accs)
    {
        int last = times.Count - 1;
        var knots = new List<int> { 0 };
        double positionWorst = 0;
        double velocityWorst = 0;
        long candidateSpans = 0;
        long acceptedNodeChecks = 0;
        long interiorProbeChecks = 0;
        int k = 0;
        while (k < last)
        {
            int best = -1;
            SpanValidation bestValidation = default;
            int capEnd = k + 1;
            while (capEnd < last
                && times[capEnd + 1] - times[k] <= NBodyEphemerides.KnotGapCapSeconds)
                capEnd++;
            // Validity is not monotone. Descending order needs no such assumption:
            // the first passing endpoint is the furthest one inside the monotone time cap.
            for (int candidate = capEnd; candidate > k; candidate--)
            {
                var validation = EvaluateSpan(
                    body, quintic, k, candidate, times, nodes, accs, budget);
                candidateSpans++;
                acceptedNodeChecks += validation.AcceptedNodeChecks;
                interiorProbeChecks += validation.InteriorProbeChecks;
                if (validation.MaxPositionError <= budget)
                {
                    best = candidate;
                    bestValidation = validation;
                    break;
                }
            }
            if (best < 0)
                throw new InvalidOperationException(
                    $"no independently validated span advances dense node {k} "
                    + $"within the {budget:R} m position budget");

            knots.Add(best);
            positionWorst = Math.Max(positionWorst, bestValidation.MaxPositionError);
            velocityWorst = Math.Max(velocityWorst, bestValidation.MaxVelocityError);
            k = best;
        }
        return new SelectionResult(knots, positionWorst, velocityWorst,
            candidateSpans, acceptedNodeChecks, interiorProbeChecks);
    }

    internal static SpanValidation EvaluateSpan(int body, bool quintic, int a, int c,
        IReadOnlyList<double> times, IReadOnlyList<StateVector[]> nodes, Vector3d[][] accs,
        double positionBudget = double.PositiveInfinity)
    {
        double dt = times[c] - times[a];
        var start = nodes[a][body];
        var end = nodes[c][body];
        var startAcceleration = accs[a][body];
        var endAcceleration = accs[c][body];
        double maxPositionError = 0;
        double maxVelocityError = 0;
        int acceptedNodeChecks = 0;
        int interiorProbeChecks = 0;

        for (int i = a + 1; i < c; i++)
        {
            acceptedNodeChecks++;
            if (!Check(times[i], nodes[i][body])) return Result();
        }

        for (int i = a; i < c; i++)
        {
            double denseDt = times[i + 1] - times[i];
            var denseStart = nodes[i][body];
            var denseEnd = nodes[i + 1][body];
            foreach (double fraction in InteriorFractions)
            {
                interiorProbeChecks++;
                double time = times[i] + denseDt * fraction;
                var denseTruth = new StateVector(
                    NBodyEphemerides.CubicPosition(
                        in denseStart, in denseEnd, denseDt, fraction),
                    NBodyEphemerides.CubicVelocity(
                        in denseStart, in denseEnd, denseDt, fraction));
                if (!Check(time, denseTruth)) return Result();
            }
        }

        return Result();

        SpanValidation Result() => new(
            maxPositionError, maxVelocityError, acceptedNodeChecks, interiorProbeChecks);

        bool Check(double time, StateVector truth)
        {
            double u = (time - times[a]) / dt;
            var position = quintic
                ? NBodyEphemerides.QuinticPosition(in start, in startAcceleration,
                    in end, in endAcceleration, dt, u)
                : NBodyEphemerides.CubicPosition(in start, in end, dt, u);
            var velocity = quintic
                ? NBodyEphemerides.QuinticVelocity(in start, in startAcceleration,
                    in end, in endAcceleration, dt, u)
                : NBodyEphemerides.CubicVelocity(in start, in end, dt, u);
            maxPositionError = Math.Max(
                maxPositionError, (position - truth.Position).Length());
            maxVelocityError = Math.Max(
                maxVelocityError, (velocity - truth.Velocity).Length());
            return maxPositionError <= positionBudget;
        }
    }

    internal readonly record struct SpanValidation(
        double MaxPositionError, double MaxVelocityError,
        int AcceptedNodeChecks, int InteriorProbeChecks);

    internal sealed record SelectionResult(
        List<int> Knots, double MaxPositionError, double MaxVelocityError,
        long CandidateSpans, long AcceptedNodeChecks, long InteriorProbeChecks);

    /// <summary>Scalar O(n²/2) mutual gravity for the storage-geometry probe.</summary>
    private static void PairwiseScalar(double[] mus, StateVector[] states, Vector3d[] acc)
    {
        int n = states.Length;
        Array.Clear(acc);
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                var d = states[i].Position - states[j].Position;
                double r2 = d.LengthSquared();
                double invR3 = 1.0 / (r2 * Math.Sqrt(r2));
                acc[i] -= d * (mus[j] * invR3);
                acc[j] += d * (mus[i] * invR3);
            }
        }
    }
}

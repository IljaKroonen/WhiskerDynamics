using WhiskerDynamics.Benchmarks;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks.Tests;

public sealed class ThinningProbeTests
{
    [Fact]
    public void Periodic_span_validity_can_fail_and_then_pass_again()
    {
        const double step = 0.1;
        var times = Enumerable.Range(0, 54).Select(i => i * step).ToArray();
        var nodes = times.Select(time => new[]
        {
            new StateVector(
                new Vector3d(Math.Sin(time), 0.0, 0.0),
                new Vector3d(Math.Cos(time), 0.0, 0.0)),
        }).ToArray();
        var accelerations = times.Select(time => new[]
        {
            new Vector3d(-Math.Sin(time), 0.0, 0.0),
        }).ToArray();

        var before = ThinningProbe.EvaluateSpan(
            0, quintic: false, 0, 50, times, nodes, accelerations);
        var failure = ThinningProbe.EvaluateSpan(
            0, quintic: false, 0, 51, times, nodes, accelerations);
        var secondFailure = ThinningProbe.EvaluateSpan(
            0, quintic: false, 0, 52, times, nodes, accelerations);
        var laterPass = ThinningProbe.EvaluateSpan(
            0, quintic: false, 0, 53, times, nodes, accelerations);
        double passingMax = Math.Max(
            before.MaxPositionError, laterPass.MaxPositionError);
        double failingMin = Math.Min(
            failure.MaxPositionError, secondFailure.MaxPositionError);
        Assert.True(failingMin > passingMax);
        double budget = 0.5 * (failingMin + passingMax);

        Assert.True(before.MaxPositionError <= budget);
        Assert.True(failure.MaxPositionError > budget);
        Assert.True(secondFailure.MaxPositionError > budget);
        Assert.True(laterPass.MaxPositionError <= budget);

        var selectionAfterFailures = ThinningProbe.SelectKnots(
            0, quintic: false, budget,
            times[..53], nodes[..53], accelerations[..53]);
        Assert.Equal(50, selectionAfterFailures.Knots[1]);

        var selection = ThinningProbe.SelectKnots(
            0, quintic: false, budget, times, nodes, accelerations);
        Assert.Equal([0, 53], selection.Knots);
        Assert.Equal(1, selection.CandidateSpans);
    }

    [Fact]
    public void Interior_dense_truth_probes_reject_a_span_that_matches_every_dense_node()
    {
        double[] times = [0.0, 1.0, 2.0];
        StateVector[][] nodes =
        [
            [new StateVector(Vector3d.Zero, Vector3d.Zero)],
            [new StateVector(Vector3d.Zero, new Vector3d(10.0, 0.0, 0.0))],
            [new StateVector(Vector3d.Zero, Vector3d.Zero)],
        ];
        Vector3d[][] accelerations =
        [
            [Vector3d.Zero],
            [Vector3d.Zero],
            [Vector3d.Zero],
        ];

        var span = ThinningProbe.EvaluateSpan(
            0, quintic: false, 0, 2, times, nodes, accelerations);

        Assert.Equal(1, span.AcceptedNodeChecks);
        Assert.Equal(6, span.InteriorProbeChecks);
        Assert.True(span.MaxPositionError > 1.0);
        var selection = ThinningProbe.SelectKnots(
            0, quintic: false, budget: 0.1, times, nodes, accelerations);
        Assert.Equal([0, 1, 2], selection.Knots);
    }

    [Fact]
    public void A_single_dense_step_beyond_the_gap_cap_remains_selectable()
    {
        double[] times = [0.0, NBodyEphemerides.KnotGapCapSeconds + 1.0];
        StateVector[][] nodes =
        [
            [new StateVector(Vector3d.Zero, Vector3d.Zero)],
            [new StateVector(Vector3d.Zero, Vector3d.Zero)],
        ];
        Vector3d[][] accelerations =
        [
            [Vector3d.Zero],
            [Vector3d.Zero],
        ];

        var selection = ThinningProbe.SelectKnots(
            0, quintic: true, budget: 1.0, times, nodes, accelerations);

        Assert.Equal([0, 1], selection.Knots);
        Assert.Equal(1, selection.CandidateSpans);
        Assert.Equal(3, selection.InteriorProbeChecks);
    }

    [Fact]
    public void Validation_scope_describes_only_the_finite_sample_guarantee()
    {
        Assert.Contains("every accepted dense node", ThinningProbe.ValidationScope);
        Assert.Contains("three fixed interior times", ThinningProbe.ValidationScope);
        Assert.Contains("dense cubic-Hermite reference", ThinningProbe.ValidationScope);
        Assert.Contains("does not prove a continuous-time maximum",
            ThinningProbe.ValidationScope);
    }
}

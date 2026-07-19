using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning.Periapsis;

public class PeriapsisFiniteAdmissionTests
{
    private static readonly EngineScalars Engine = new(1000.0, 3000.0, 2.0);
    private static readonly FiniteBurnFold Model = new(Engine, 5.0, 32);

    [Fact]
    public void Node_after_prior_cutoff_is_rejected_when_centered_ignition_overlaps()
    {
        const double firstNode = 2000.0;
        const double candidateNode = 2015.0;
        const double magnitude = 150.0;
        var first = FiniteBurnKernel.Expand(firstNode, magnitude,
            Engine, Model.SliceSeconds, Model.MaxSlices)!;
        double priorCutoff = first.IgnitionSeconds + first.DurationSeconds;
        var chained = Engine with
        {
            MassKg = FiniteBurnKernel.MassAfterBurn(magnitude, Engine),
        };
        var candidate = FiniteBurnKernel.Expand(candidateNode, magnitude,
            chained, Model.SliceSeconds, Model.MaxSlices)!;

        Assert.True(candidateNode > priorCutoff); // The old node-only guard passes.
        Assert.True(candidate.IgnitionSeconds <= priorCutoff);
        Assert.True(candidate.IgnitionSeconds + candidate.DurationSeconds < 2100.0);

        var decision = PeriapsisFiniteAdmission.Decide(candidateNode, magnitude,
            Model, chained, priorCutoff, inclusiveHorizon: 2100.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectWindowStart, decision.Kind);
        Assert.Contains("overlaps", decision.Failure);
        Assert.Contains("preceding", decision.Failure);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Fact]
    public void Node_and_existing_one_second_guard_fit_but_finite_cutoff_overruns_horizon()
    {
        const double node = 7990.0;
        const double horizon = 8000.0;
        const double magnitude = 150.0;
        var expected = FiniteBurnKernel.Expand(node, magnitude,
            Engine, Model.SliceSeconds, Model.MaxSlices)!;

        Assert.True(node + 1.0 < horizon); // The old node-window guard passes.
        Assert.True(expected.IgnitionSeconds > 0.0);
        Assert.True(expected.IgnitionSeconds + expected.DurationSeconds > horizon);

        var decision = PeriapsisFiniteAdmission.Decide(node, magnitude,
            Model, Engine, exclusiveEarliestIgnition: 0.0, horizon);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectHorizon, decision.Kind);
        Assert.Contains("prediction horizon", decision.Failure);
        Assert.Contains("extend", decision.Failure);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Fact]
    public void Ignition_exactly_at_exclusive_preceding_bound_is_rejected()
    {
        var expansion = Expand();

        var decision = PeriapsisFiniteAdmission.Decide(5000.0, 150.0,
            Model, Engine, expansion.IgnitionSeconds,
            expansion.IgnitionSeconds + expansion.DurationSeconds + 1.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectWindowStart, decision.Kind);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Fact]
    public void Cutoff_exactly_at_inclusive_horizon_is_admitted()
    {
        var expected = Expand();
        double cutoff = expected.IgnitionSeconds + expected.DurationSeconds;

        var decision = PeriapsisFiniteAdmission.Decide(5000.0, 150.0,
            Model, Engine, expected.IgnitionSeconds - 1.0, cutoff);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Finite, decision.Kind);
        Assert.True(decision.TryGetAcceptedExpansion(out var accepted));
        AssertExpansionEqual(expected, accepted!);
        Assert.Equal(expected.IgnitionSeconds, decision.ModelStartSeconds);
        Assert.Equal(cutoff, decision.ModelEndSeconds);
    }

    [Fact]
    public void Fitting_finite_candidate_preserves_the_finite_expansion()
    {
        var expected = Expand();

        var decision = PeriapsisFiniteAdmission.Decide(5000.0, 150.0,
            Model, Engine, expected.IgnitionSeconds - 0.001,
            expected.IgnitionSeconds + expected.DurationSeconds + 0.001);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Finite, decision.Kind);
        Assert.Null(decision.Failure);
        Assert.True(decision.TryGetAcceptedExpansion(out var accepted));
        AssertExpansionEqual(expected, accepted!);
        Assert.Equal(expected.IgnitionSeconds, decision.ModelStartSeconds);
        Assert.Equal(expected.IgnitionSeconds + expected.DurationSeconds,
            decision.ModelEndSeconds);
    }

    [Fact]
    public void No_model_intentionally_dispatches_as_an_impulse()
    {
        var decision = PeriapsisFiniteAdmission.Decide(5000.0, 150.0,
            model: null, chainedEngine: default,
            exclusiveEarliestIgnition: 0.0, inclusiveHorizon: 6000.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Impulsive, decision.Kind);
        Assert.True(decision.TryGetAcceptedExpansion(out var expansion));
        Assert.Null(expansion);
        Assert.Equal(5000.0, decision.ModelStartSeconds);
        Assert.Equal(5000.0, decision.ModelEndSeconds);
        Assert.Null(decision.Failure);
    }

    [Fact]
    public void Zero_delta_v_dispatches_as_a_node_impulse()
    {
        var decision = PeriapsisFiniteAdmission.Decide(5000.0, magnitude: 0.0,
            Model, Engine, exclusiveEarliestIgnition: 0.0, inclusiveHorizon: 6000.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Impulsive, decision.Kind);
        Assert.True(decision.TryGetAcceptedExpansion(out var expansion));
        Assert.Null(expansion);
        Assert.Equal(5000.0, decision.ModelStartSeconds);
        Assert.Equal(5000.0, decision.ModelEndSeconds);
    }

    [Theory]
    [InlineData(60.0, 32)]
    [InlineData(5.0, 1)]
    public void Fitting_single_slice_representation_carries_physical_finite_window(
        double sliceSeconds, int maxSlices)
    {
        const double node = 5000.0;
        const double magnitude = 150.0;
        var model = new FiniteBurnFold(Engine, sliceSeconds, maxSlices);
        double duration = FiniteBurnKernel.BurnDurationSeconds(magnitude, Engine);
        double ignition = node - duration / 2.0;
        double cutoff = ignition + duration;
        Assert.Equal(1, FiniteBurnKernel.SliceCount(duration, sliceSeconds, maxSlices));

        var decision = PeriapsisFiniteAdmission.Decide(node, magnitude,
            model, Engine, ignition - 1.0, cutoff + 1.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Impulsive, decision.Kind);
        Assert.True(decision.TryGetAcceptedExpansion(out var expansion));
        Assert.Null(expansion);
        Assert.Equal(ignition, decision.ModelStartSeconds);
        Assert.Equal(cutoff, decision.ModelEndSeconds);
    }

    [Theory]
    [InlineData(60.0, 32)]
    [InlineData(5.0, 1)]
    public void Single_slice_representation_still_rejects_physical_window_overlap(
        double sliceSeconds, int maxSlices)
    {
        const double node = 5000.0;
        const double magnitude = 150.0;
        var model = new FiniteBurnFold(Engine, sliceSeconds, maxSlices);
        double duration = FiniteBurnKernel.BurnDurationSeconds(magnitude, Engine);
        double ignition = node - duration / 2.0;

        var decision = PeriapsisFiniteAdmission.Decide(node, magnitude,
            model, Engine, exclusiveEarliestIgnition: ignition,
            inclusiveHorizon: node + duration);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectWindowStart, decision.Kind);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Theory]
    [InlineData(60.0, 32)]
    [InlineData(5.0, 1)]
    public void Single_slice_representation_still_rejects_physical_horizon_overrun(
        double sliceSeconds, int maxSlices)
    {
        const double node = 5000.0;
        const double magnitude = 150.0;
        var model = new FiniteBurnFold(Engine, sliceSeconds, maxSlices);
        double duration = FiniteBurnKernel.BurnDurationSeconds(magnitude, Engine);
        double ignition = node - duration / 2.0;
        double cutoff = ignition + duration;

        var decision = PeriapsisFiniteAdmission.Decide(node, magnitude,
            model, Engine, exclusiveEarliestIgnition: ignition - 1.0,
            inclusiveHorizon: cutoff - 0.001);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectHorizon, decision.Kind);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Theory]
    [InlineData(60.0, 32)]
    [InlineData(5.0, 1)]
    public void Single_slice_cutoff_exactly_at_horizon_is_admitted(
        double sliceSeconds, int maxSlices)
    {
        const double node = 5000.0;
        const double magnitude = 150.0;
        var model = new FiniteBurnFold(Engine, sliceSeconds, maxSlices);
        double duration = FiniteBurnKernel.BurnDurationSeconds(magnitude, Engine);
        double ignition = node - duration / 2.0;
        double cutoff = ignition + duration;

        var decision = PeriapsisFiniteAdmission.Decide(node, magnitude,
            model, Engine, exclusiveEarliestIgnition: ignition - 0.001,
            inclusiveHorizon: cutoff);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Impulsive, decision.Kind);
        Assert.True(decision.TryGetAcceptedExpansion(out var expansion));
        Assert.Null(expansion);
        Assert.Equal(ignition, decision.ModelStartSeconds);
        Assert.Equal(cutoff, decision.ModelEndSeconds);
    }

    [Fact]
    public void Multi_slice_tank_emptying_failure_is_rejected_not_impulsed()
    {
        const double magnitude = 120_000.0;
        double duration = FiniteBurnKernel.BurnDurationSeconds(magnitude, Engine);
        Assert.True(FiniteBurnKernel.SliceCount(
            duration, Model.SliceSeconds, Model.MaxSlices) > 1);
        Assert.Null(FiniteBurnKernel.Expand(5000.0, magnitude,
            Engine, Model.SliceSeconds, Model.MaxSlices));

        var decision = PeriapsisFiniteAdmission.Decide(5000.0, magnitude,
            Model, Engine, exclusiveEarliestIgnition: 0.0, inclusiveHorizon: 6000.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectUnmodelable, decision.Kind);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Fact]
    public void Active_model_with_unusable_chained_engine_is_rejected_not_impulsed()
    {
        var decision = PeriapsisFiniteAdmission.Decide(5000.0, 150.0,
            Model, chainedEngine: default,
            exclusiveEarliestIgnition: 0.0, inclusiveHorizon: 6000.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectUnmodelable, decision.Kind);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    [Theory]
    [InlineData(double.NaN, 150.0, 0.0, 6000.0, 5.0)]
    [InlineData(5000.0, double.NaN, 0.0, 6000.0, 5.0)]
    [InlineData(5000.0, double.PositiveInfinity, 0.0, 6000.0, 5.0)]
    [InlineData(5000.0, -1.0, 0.0, 6000.0, 5.0)]
    [InlineData(5000.0, 150.0, double.NegativeInfinity, 6000.0, 5.0)]
    [InlineData(5000.0, 150.0, 0.0, double.PositiveInfinity, 5.0)]
    [InlineData(5000.0, 150.0, 0.0, 6000.0, double.NaN)]
    [InlineData(5000.0, 150.0, 0.0, 6000.0, double.PositiveInfinity)]
    public void Nonfinite_or_invalid_inputs_are_rejected_not_impulsed(
        double node, double magnitude, double bound, double horizon, double sliceSeconds)
    {
        var model = new FiniteBurnFold(Engine, sliceSeconds, 32);

        var decision = PeriapsisFiniteAdmission.Decide(node, magnitude,
            model, Engine, bound, horizon);

        Assert.Equal(PeriapsisFiniteAdmissionKind.RejectUnmodelable, decision.Kind);
        AssertRejectedCannotDispatchAsImpulse(decision);
    }

    private static FiniteBurnExpansion Expand()
        => FiniteBurnKernel.Expand(5000.0, 150.0,
            Engine, Model.SliceSeconds, Model.MaxSlices)!;

    private static void AssertRejectedCannotDispatchAsImpulse(
        PeriapsisFiniteAdmission decision)
    {
        Assert.StartsWith("rejected:", decision.Failure);
        Assert.False(decision.TryGetAcceptedExpansion(out var expansion));
        Assert.Null(expansion);
        Assert.Null(decision.ModelStartSeconds);
        Assert.Null(decision.ModelEndSeconds);
        Assert.NotEqual(PeriapsisFiniteAdmissionKind.Impulsive, decision.Kind);
    }

    private static void AssertExpansionEqual(
        FiniteBurnExpansion expected, FiniteBurnExpansion actual)
    {
        Assert.Equal(expected.IgnitionSeconds, actual.IgnitionSeconds);
        Assert.Equal(expected.DurationSeconds, actual.DurationSeconds);
        Assert.Equal(expected.Times, actual.Times);
        Assert.Equal(expected.Magnitudes, actual.Magnitudes);
    }
}

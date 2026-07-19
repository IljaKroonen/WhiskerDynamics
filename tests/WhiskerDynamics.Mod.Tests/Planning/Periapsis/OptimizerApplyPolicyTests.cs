using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.Mod.Tests.Planning.Periapsis;

public sealed class OptimizerApplyPolicyTests
{
    [Fact]
    public void Completed_result_is_only_consumed_by_its_owner()
    {
        Assert.Equal(CompletedOptimizeRoute.ApplyForOwner,
            OptimizeRoutingPolicy.RouteCompleted(ownerDrawn: true));
        Assert.Equal(CompletedOptimizeRoute.PreserveForOwner,
            OptimizeRoutingPolicy.RouteCompleted(ownerDrawn: false));
    }

    [Theory]
    [InlineData(false, false, 0)]
    [InlineData(true, false, 1)]
    [InlineData(false, true, 2)]
    [InlineData(true, true, 3)]
    public void Authority_loss_restores_every_completed_write(
        bool timeWritten, bool deltaVWritten, int expected)
    {
        Assert.Equal((OptimizeApplyRollback)expected,
            OptimizeApplyPolicy.ForAuthorityLoss(timeWritten, deltaVWritten));
    }

    [Fact]
    public void Apply_lead_is_measured_from_finite_ignition_not_the_future_node()
    {
        const double now = 100.0;
        const double lead = 10.0;
        const double futureNode = 140.0;
        const double alreadyOwnedIgnition = 109.0;

        Assert.True(OptimizeApplyPolicy.ModeledStartHasLead(futureNode, now, lead));
        Assert.False(OptimizeApplyPolicy.ModeledStartHasLead(
            alreadyOwnedIgnition, now, lead));
    }

    [Fact]
    public void K1_objective_impulse_still_applies_from_its_physical_ignition()
    {
        var engine = new EngineScalars(1000.0, 3000.0, 2.0);
        var model = new FiniteBurnFold(engine, SliceSeconds: 60.0, MaxSlices: 32);
        const double node = 140.0;
        var admission = PeriapsisFiniteAdmission.Decide(
            node, 150.0, model, engine,
            exclusiveEarliestIgnition: 0.0, inclusiveHorizon: 200.0);

        Assert.Equal(PeriapsisFiniteAdmissionKind.Impulsive, admission.Kind);
        double ignition = Assert.IsType<double>(admission.ModelStartSeconds);
        Assert.True(ignition < node);
        double now = ignition - 0.5;
        Assert.True(OptimizeApplyPolicy.ModeledStartHasLead(node, now, 1.0));
        Assert.False(OptimizeApplyPolicy.ModeledStartHasLead(ignition, now, 1.0));
    }

    [Theory]
    [InlineData(110.0, false)]
    [InlineData(110.0001, true)]
    [InlineData(double.NaN, false)]
    [InlineData(double.PositiveInfinity, false)]
    public void Apply_lead_gate_is_strict_and_fails_closed(double modeledStart, bool expected)
    {
        Assert.Equal(expected,
            OptimizeApplyPolicy.ModeledStartHasLead(modeledStart, 100.0, 10.0));
    }

    [Theory]
    [InlineData(true, true, true,
        PredictorAuthorityPolicy.Reason.ReseedPending)]
    [InlineData(false, false, true,
        PredictorAuthorityPolicy.Reason.NotFreefall)]
    [InlineData(false, true, false,
        PredictorAuthorityPolicy.Reason.PredictorReplaced)]
    public void Async_result_is_refused_after_pending_live_or_reseed(
        bool reseedPending,
        bool committedFreefall,
        bool samePredictor,
        PredictorAuthorityPolicy.Reason expected)
    {
        Assert.Equal(expected, PredictorAuthorityPolicy.Classify(new(
            EntryPresent: true,
            SameEntry: true,
            BoundVehicleAvailable: true,
            SameVehicle: true,
            ReseedPending: reseedPending,
            CommittedFreefall: committedFreefall,
            PredictorAvailable: true,
            SamePredictor: samePredictor)));
    }

    [Fact]
    public void Async_result_is_refused_after_same_id_entry_replacement()
    {
        Assert.Equal(PredictorAuthorityPolicy.Reason.EntryReplaced,
            PredictorAuthorityPolicy.Classify(new(
                EntryPresent: true,
                SameEntry: false,
                BoundVehicleAvailable: true,
                SameVehicle: false,
                ReseedPending: false,
                CommittedFreefall: true,
                PredictorAvailable: true,
                SamePredictor: false)));
    }
}

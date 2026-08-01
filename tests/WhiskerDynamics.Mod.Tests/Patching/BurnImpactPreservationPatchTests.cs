using KSA;
using WhiskerDynamics.Mod.Patches;

namespace WhiskerDynamics.Mod.Tests.Patching;

public class BurnImpactPreservationPatchTests
{
    [Fact]
    public void Missing_patch_after_impact_gets_a_burn_calculation_continuation()
    {
        Assert.True(BurnPlanCalculationContext.ShouldExtendPastImpact(
            PatchTransition.Impact,
            lastEndTime: new SimTime(100),
            requestedTime: new SimTime(200),
            currentTime: new SimTime(50)));
    }

    [Theory]
    [InlineData(PatchTransition.Final, 100, 200, 50)] // plan does not end in impact
    [InlineData(PatchTransition.Impact, 200, 100, 50)] // requested before the impact end
    [InlineData(PatchTransition.Impact, 100, 200, 300)] // completed-burn validity probe
    public void Extension_requires_a_future_time_past_an_impact_end(
        PatchTransition transition,
        double endTime,
        double requestedTime,
        double currentTime)
    {
        Assert.False(BurnPlanCalculationContext.ShouldExtendPastImpact(
            transition,
            new SimTime(endTime),
            new SimTime(requestedTime),
            new SimTime(currentTime)));
    }
}

using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Patches;

namespace WhiskerDynamics.Mod.Tests.Soi;

public class SoiPlanAuthorityKernelTests
{
    [Fact]
    public void First_seen_modeled_freefall_vessel_suppresses_stock_scheduler() =>
        Assert.Equal(SoiPlanAuthorityKernel.Disposition.SuppressStockScheduler,
            SoiPlanAuthorityKernel.Classify(
                enabled: true, bindingAvailable: true, parentModeled: true,
                committedFreefall: true, tracked: false, sameVehicle: false));

    [Fact]
    public void Multi_vehicle_modeled_freefall_path_also_suppresses_without_task_count_gate() =>
        Assert.Equal(SoiPlanAuthorityKernel.Disposition.SuppressStockScheduler,
            SoiPlanAuthorityKernel.Classify(
                enabled: true, bindingAvailable: true, parentModeled: true,
                committedFreefall: true, tracked: true, sameVehicle: true));

    [Fact]
    public void Legitimate_live_physics_keeps_a_stock_mirror_plan() =>
        Assert.Equal(SoiPlanAuthorityKernel.Disposition.LivePhysicsMirror,
            SoiPlanAuthorityKernel.Classify(
                enabled: true, bindingAvailable: true, parentModeled: true,
                committedFreefall: false, tracked: true, sameVehicle: true));

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void Inactive_or_unbound_mod_does_not_claim_scheduler(
        bool enabled, bool bindingAvailable) =>
        Assert.Equal(SoiPlanAuthorityKernel.Disposition.Inactive,
            SoiPlanAuthorityKernel.Classify(
                enabled, bindingAvailable, parentModeled: true,
                committedFreefall: true, tracked: false, sameVehicle: false));

    [Fact]
    public void Unknown_parent_and_contradictory_identity_are_fatal()
    {
        Assert.Equal(SoiPlanAuthorityKernel.Disposition.FatalUnknownParent,
            SoiPlanAuthorityKernel.Classify(
                enabled: true, bindingAvailable: true, parentModeled: false,
                committedFreefall: true, tracked: false, sameVehicle: false));
        Assert.Equal(SoiPlanAuthorityKernel.Disposition.FatalVehicleIdentity,
            SoiPlanAuthorityKernel.Classify(
                enabled: true, bindingAvailable: true, parentModeled: true,
                committedFreefall: true, tracked: true, sameVehicle: false));
    }
}

public class SoiHandoffInvariantPolicyTests
{
    [Fact]
    public void Enabled_prefix_inspection_failure_blocks_stock_conversion()
    {
        Assert.False(SoiHandoffInvariantPolicy.OriginalMayRun(
            enabled: true, inspectionSucceeded: false));
        Assert.True(SoiHandoffInvariantPolicy.OriginalMayRun(
            enabled: false, inspectionSucceeded: false));
    }

    [Theory]
    [InlineData(false, false, false,
        (int)SoiHandoffInvariantPolicy.Failure.NonAstronomicalParent)]
    [InlineData(true, false, true,
        (int)SoiHandoffInvariantPolicy.Failure.UnmodeledParent)]
    [InlineData(true, true, false,
        (int)SoiHandoffInvariantPolicy.Failure.CurrentOrbitParentMismatch)]
    [InlineData(true, true, true,
        (int)SoiHandoffInvariantPolicy.Failure.None)]
    public void Cross_parent_conversion_invariants_are_fail_closed(
        bool astronomical, bool modeled, bool currentOrbitOnClosest,
        int expected) =>
        Assert.Equal((SoiHandoffInvariantPolicy.Failure)expected,
            SoiHandoffInvariantPolicy.ClassifyCrossParent(
                astronomical, modeled, currentOrbitOnClosest));
}

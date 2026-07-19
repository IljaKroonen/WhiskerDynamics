using WhiskerDynamics.Mod.Patches;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.Mod.Tests;

public sealed class FaultPausePolicyTests
{
    [Theory]
    [InlineData(ModStatus.Active, 1000.0, 1000.0)]
    [InlineData(ModStatus.DisabledByUser, 1.0, 1.0)]
    [InlineData(ModStatus.DisabledFault, 1.0, 0.0)]
    [InlineData(ModStatus.DisabledFault, -1.0, 0.0)]
    public void Requested_speed_is_forced_to_zero_only_for_runtime_faults(
        ModStatus status, double requested, double expected) =>
        Assert.Equal(expected, FaultPausePolicy.RequestedSpeed(status, requested));

    [Fact]
    public void Scheduler_observes_zero_immediately_after_runtime_fault()
    {
        Assert.Equal(0.0,
            FaultPausePolicy.ObservedSpeed(ModStatus.DisabledFault, 100_000.0));
        Assert.Equal(100_000.0,
            FaultPausePolicy.ObservedSpeed(ModStatus.Active, 100_000.0));
    }

    [Fact]
    public void Main_thread_enforcer_invokes_the_real_speed_write_seam_with_zero()
    {
        var writes = new List<double>();

        Assert.True(FaultPauseEnforcer.TryEnforce(
            ModStatus.DisabledFault, writes.Add));
        Assert.Equal([0.0], writes);
        Assert.False(FaultPauseEnforcer.TryEnforce(
            ModStatus.Active, writes.Add));
        Assert.Equal([0.0], writes);
    }

    [Fact]
    public void Both_speed_gate_patch_classes_are_registered()
    {
        Assert.Contains(typeof(FaultPausePatch), GameplayPatchSet.PatchTypes);
        Assert.Contains(typeof(FaultPauseReadPatch), GameplayPatchSet.PatchTypes);
    }
}

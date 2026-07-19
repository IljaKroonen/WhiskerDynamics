using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class PausedOverlayRefreshKernelTests
{
    [Fact]
    public void Running_simulation_never_uses_the_ui_refresh()
    {
        Assert.False(PausedOverlayRefreshKernel.ShouldRefresh(
            1.0, 5000, 0, 2, 1, "Earth inertial", "Luna inertial"));
    }

    [Fact]
    public void Paused_keepalive_runs_once_per_second()
    {
        Assert.False(PausedOverlayRefreshKernel.ShouldRefresh(
            0.0, 1999, 1000, 4, 4, "Earth inertial", "Earth inertial"));
        Assert.True(PausedOverlayRefreshKernel.ShouldRefresh(
            0.0, 2000, 1000, 4, 4, "Earth inertial", "Earth inertial"));
    }

    [Fact]
    public void Plan_edit_bypasses_the_keepalive_period_while_paused()
    {
        Assert.True(PausedOverlayRefreshKernel.ShouldRefresh(
            0.0, 1100, 1000, 5, 4, null, null));
    }

    [Fact]
    public void Frame_change_bypasses_the_keepalive_period_while_paused()
    {
        Assert.True(PausedOverlayRefreshKernel.ShouldRefresh(
            0.0, 1100, 1000, 4, 4, "Luna inertial", "Earth inertial"));
    }

    [Fact]
    public void Deferred_edit_state_is_an_explicit_registry_contract()
    {
        Assert.NotNull(typeof(VesselRegistry).GetProperty(nameof(VesselRegistry.PausedEditsDeferred)));
    }

    [Fact]
    public void Deferred_edit_stays_sticky_across_unchanged_keepalives()
    {
        Assert.True(PausedOverlayRefreshKernel.AccumulateDeferral(
            alreadyDeferred: true, editDeferred: false));
        Assert.True(PausedOverlayRefreshKernel.AccumulateDeferral(
            alreadyDeferred: false, editDeferred: true));
        Assert.False(PausedOverlayRefreshKernel.AccumulateDeferral(
            alreadyDeferred: false, editDeferred: false));
    }
}

using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Patches;

public class NavigationTargetKernelTests
{
    [Fact]
    public void Relative_state_subtracts_parent_translation_and_velocity()
    {
        var parent = new StateVector(
            new Vector3d(100, -20, 7), new Vector3d(4, 5, -2));
        var target = new StateVector(
            new Vector3d(130, 10, -3), new Vector3d(9, 2, 6));

        var relative = NavigationTargetKernel.RelativeToParent(in target, in parent);

        Assert.Equal(new Vector3d(30, 30, -10), relative.Position);
        Assert.Equal(new Vector3d(5, -3, 8), relative.Velocity);
    }

    [Fact]
    public void Live_fallback_advances_the_committed_state_linearly()
    {
        var state = new StateVector(new Vector3d(10, 20, 30),
            new Vector3d(2, -3, 4));
        var advanced = NavigationTargetKernel.LinearStateAt(in state, 5);
        Assert.Equal(new Vector3d(20, 5, 50), advanced.Position);
        Assert.Equal(state.Velocity, advanced.Velocity);
    }

    [Fact]
    public void Live_fallback_advances_only_across_a_small_physics_gap()
    {
        var state = new StateVector(new Vector3d(10, 20, 30),
            new Vector3d(2, -3, 4));

        Assert.True(NavigationTargetKernel.TryBoundedLinearStateAt(
            in state, 0.5, 1.0, out var near));
        Assert.Equal(new Vector3d(11, 18.5, 32), near.Position);

        Assert.True(NavigationTargetKernel.TryBoundedLinearStateAt(
            in state, 1000, 1.0, out var far));
        Assert.Equal(state, far);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void Live_fallback_rejects_non_finite_time_gaps(double delta)
    {
        var state = new StateVector(Vector3d.Zero, Vector3d.Zero);
        Assert.False(NavigationTargetKernel.TryBoundedLinearStateAt(
            in state, delta, 1.0, out _));
    }
}

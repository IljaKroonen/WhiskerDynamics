using WhiskerDynamics.Core;
using WhiskerDynamics.GameTestDriver.Runtime;

namespace WhiskerDynamics.GameTestHost.Tests;

public class GameTestNrhoKernelTests
{
    [Fact]
    public void Reference_propagates_from_perilune_to_the_selected_Jpl_apolune()
    {
        var state = GameTestNrhoKernel.Propagate(
            GameTestNrhoKernel.PeriodNormalized / 2);

        Assert.Equal(1.01865929880526, state.Position.X, 7);
        Assert.Equal(0, state.Position.Y, 7);
        Assert.Equal(-0.179672100884756, state.Position.Z, 7);
        Assert.Equal(0, state.Velocity.X, 7);
        Assert.Equal(-0.0958140620387836, state.Velocity.Y, 7);
        Assert.Equal(0, state.Velocity.Z, 7);
    }

    [Fact]
    public void Reference_wraps_at_one_period()
    {
        Assert.Equal(GameTestNrhoKernel.Propagate(0),
            GameTestNrhoKernel.Propagate(GameTestNrhoKernel.PeriodNormalized));
    }

    [Fact]
    public void Embedding_combines_frame_motion_and_synodic_velocity()
    {
        var before = Pose(originX: -2, scale: 10);
        var at = Pose(originX: 0, scale: 10);
        var after = Pose(originX: 2, scale: 10);

        StateVector embedded = GameTestNrhoKernel.Embed(0, timeUnitSeconds: 5,
            before, at, after, poseStepSeconds: 2);
        var normalized = GameTestNrhoKernel.Propagate(0);

        Assert.Equal((normalized.Position.X + GameTestNrhoKernel.MassRatio) * 10,
            embedded.Position.X, 12);
        Assert.Equal(normalized.Position.Z * 10, embedded.Position.Z, 12);
        Assert.Equal(1, embedded.Velocity.X, 12);
        Assert.Equal(normalized.Velocity.Y * 2, embedded.Velocity.Y, 12);
    }

    [Fact]
    public void Feedback_and_vlf_projection_preserve_the_correction()
    {
        var current = new StateVector(
            new Vector3d(2, 0, 0), new Vector3d(0, 3, 0));
        var target = new StateVector(
            new Vector3d(8, 0, 6), new Vector3d(1, 5, 3));

        Vector3d correction = GameTestNrhoKernel.Feedback(current, target, 2);
        var vlf = GameTestNrhoKernel.ToVlf(current, correction);

        Assert.Equal(new Vector3d(4, 2, 6), correction);
        Assert.Equal(2, vlf.Prograde, 12);
        Assert.Equal(6, vlf.Normal, 12);
        Assert.Equal(4, vlf.Outward, 12);
    }

    private static FramePose Pose(double originX, double scale) => new(
        new Vector3d(originX, 0, 0),
        new Vector3d(1, 0, 0), new Vector3d(0, 1, 0),
        new Vector3d(0, 0, 1), scale);
}

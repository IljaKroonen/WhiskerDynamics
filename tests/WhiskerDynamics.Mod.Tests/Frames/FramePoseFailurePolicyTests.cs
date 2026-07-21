namespace WhiskerDynamics.Mod.Tests.Frames;

public class FramePoseFailurePolicyTests
{
    [Fact]
    public void Curve_failure_preserves_active_frame_and_stale_failure_cannot_clear_new_frame()
    {
        var target = new FrameSpec(FrameKind.TargetFixed, "Earth", "Station");
        var inertial = new FrameSpec(FrameKind.Inertial, "Earth", null);
        FrameSpec? active = target;
        long generation = 10;
        var targetSnapshot = new ActiveFrameSnapshot(
            target, default, 0, default, generation);

        void RetireTarget() => FrameActivationKernel.TryDeactivate(
            ref active, ref generation, targetSnapshot);

        FramePoseFailurePolicy.OnFailure(FramePoseQuery.CurveSample, RetireTarget);
        Assert.Equal(target, active);
        Assert.Equal(10, generation);

        active = inertial;
        generation++;
        FramePoseFailurePolicy.OnFailure(FramePoseQuery.CurrentDisplay, RetireTarget);
        Assert.Equal(inertial, active);
        Assert.Equal(11, generation);

        var inertialSnapshot = targetSnapshot with { Spec = inertial, Generation = generation };
        FramePoseFailurePolicy.OnFailure(FramePoseQuery.CurrentDisplay, () =>
            FrameActivationKernel.TryDeactivate(ref active, ref generation, inertialSnapshot));
        Assert.Null(active);
        Assert.Equal(12, generation);
    }
}

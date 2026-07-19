using Brutal.Numerics;
using WhiskerDynamics.Mod;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Frames;

public class MapPoseKernelTests
{
    private static void AssertClose(double3 a, double3 b, double tol = 1e-9) =>
        Assert.True((a - b).Length() < tol, $"expected {b}, got {a}");

    [Fact]
    public void QuatFromBasis_identity_basis_gives_identity_rotation()
    {
        var q = MapPoseKernel.QuatFromBasis(double3.UnitX, double3.UnitY, double3.UnitZ);
        AssertClose(double3.Transform(new double3(1, 2, 3), q), new double3(1, 2, 3));
    }

    [Fact]
    public void QuatFromBasis_maps_unit_axes_onto_the_given_basis()
    {
        var q = MapPoseKernel.QuatFromBasis(double3.UnitY, -double3.UnitX, double3.UnitZ);
        AssertClose(double3.Transform(double3.UnitX, q), double3.UnitY);
        AssertClose(double3.Transform(double3.UnitY, q), -double3.UnitX);
        AssertClose(double3.Transform(double3.UnitZ, q), double3.UnitZ);
    }

    [Fact]
    public void QuatFromBasis_handles_all_shepperd_branches()
    {
        // Exercise every trace <= 0 matrix-conversion branch.
        var aboutX = MapPoseKernel.QuatFromBasis(double3.UnitX, -double3.UnitY, -double3.UnitZ);
        AssertClose(double3.Transform(double3.UnitY, aboutX), -double3.UnitY);
        var aboutY = MapPoseKernel.QuatFromBasis(-double3.UnitX, double3.UnitY, -double3.UnitZ);
        AssertClose(double3.Transform(double3.UnitZ, aboutY), -double3.UnitZ);
        var aboutZ = MapPoseKernel.QuatFromBasis(-double3.UnitX, -double3.UnitY, double3.UnitZ);
        AssertClose(double3.Transform(double3.UnitX, aboutZ), -double3.UnitX);
    }

    [Fact]
    public void CounterPose_preserves_the_view_of_frame_corotating_points()
    {
        // Co-rotating a point and counter-posing the camera preserves its view vector.
        var center = new double3(1e8, -2e8, 3e7);
        var delta = doubleQuat.CreateFromAxisAngle(double3.UnitZ, 0.7);
        var camPos = new double3(1.5e8, -1.9e8, 5e7);
        var camRot = doubleQuat.CreateFromAxisAngle(new double3(0.6, 0.8, 0), 0.4);
        var p = new double3(0.9e8, -2.2e8, 2e7);

        var viewBefore = double3.Transform(p - camPos, doubleQuat.Inverse(camRot));
        var rotatedP = center + double3.Transform(p - center, delta);
        (double3 newPos, doubleQuat newRot) = MapPoseKernel.CounterPose(camPos, camRot, center, delta);
        var viewAfter = double3.Transform(rotatedP - newPos, doubleQuat.Inverse(newRot));

        AssertClose(viewAfter, viewBefore, 1e-6);
    }

    [Fact]
    public void CounterPose_with_identity_delta_is_a_no_op()
    {
        var camPos = new double3(1, 2, 3);
        var camRot = doubleQuat.CreateFromAxisAngle(double3.UnitY, 0.3);
        (double3 p, doubleQuat r) = MapPoseKernel.CounterPose(camPos, camRot, new double3(9, 9, 9), doubleQuat.Identity);
        AssertClose(p, camPos);
        AssertClose(double3.Transform(double3.UnitX, r), double3.Transform(double3.UnitX, camRot));
    }

    [Fact]
    public void FrameViewPose_keeps_an_off_origin_followed_target_pixel_invariant()
    {
        // Rotating about an off-origin follow anchor must preserve its camera-relative position.
        var frameOrigin = new double3(1.4711e11, -2.3e10, 5.1e6);       // Earth-ish (ecliptic)
        var anchor = frameOrigin + new double3(4.1e6, -5.2e6, 1.3e6);   // LEO vessel, |T-C| ~ 6.7e6 m
        var forward = double3.Normalize(new double3(0.5, 0.7, -0.2));
        double scope = 2.5e7;
        var camPos = anchor - forward * scope;
        var camRot = doubleQuat.CreateFromAxisAngle(new double3(0.6, 0.8, 0), 0.4);
        var delta = doubleQuat.CreateFromAxisAngle(double3.Normalize(new double3(0.1, -0.2, 0.97)), 0.5);

        var viewBefore = double3.Transform(anchor - camPos, doubleQuat.Inverse(camRot));
        (double3 newPos, doubleQuat newRot) = MapPoseKernel.FrameViewPose(
            camPos, camRot, frameOrigin, anchor, delta);
        var viewAfter = double3.Transform(anchor - newPos, doubleQuat.Inverse(newRot));

        // Tolerance is above floating-point noise and far below an origin-rotation error.
        AssertClose(viewAfter, viewBefore, 1e-3);
        // Orientation must still counter-rotate.
        AssertClose(double3.Transform(double3.UnitX, newRot),
            double3.Transform(double3.Transform(double3.UnitX, camRot), delta), 1e-9);
    }

    [Fact]
    public void FrameViewPose_without_a_follow_anchor_rotates_about_the_frame_origin()
    {
        // A missing follow target falls back to rotation about the frame origin.
        var frameOrigin = new double3(1e8, -2e8, 3e7);
        var camPos = new double3(1.5e8, -1.9e8, 5e7);
        var camRot = doubleQuat.CreateFromAxisAngle(new double3(0.6, 0.8, 0), 0.4);
        var delta = doubleQuat.CreateFromAxisAngle(double3.UnitZ, 0.7);

        (double3 expPos, doubleQuat expRot) = MapPoseKernel.CounterPose(camPos, camRot, frameOrigin, delta);
        (double3 gotPos, doubleQuat gotRot) = MapPoseKernel.FrameViewPose(camPos, camRot, frameOrigin, null, delta);

        AssertClose(gotPos, expPos);
        AssertClose(double3.Transform(double3.UnitX, gotRot), double3.Transform(double3.UnitX, expRot));
    }

    [Fact]
    public void FrameViewPose_with_anchor_at_the_origin_matches_the_origin_composition()
    {
        // Following the frame origin makes anchor and origin composition equivalent.
        var frameOrigin = new double3(1e8, -2e8, 3e7);
        var camPos = new double3(1.5e8, -1.9e8, 5e7);
        var camRot = doubleQuat.CreateFromAxisAngle(new double3(0, 0.6, 0.8), -0.3);
        var delta = doubleQuat.CreateFromAxisAngle(double3.UnitZ, 1.1);

        (double3 expPos, doubleQuat expRot) = MapPoseKernel.CounterPose(camPos, camRot, frameOrigin, delta);
        (double3 gotPos, doubleQuat gotRot) = MapPoseKernel.FrameViewPose(camPos, camRot, frameOrigin, frameOrigin, delta);

        AssertClose(gotPos, expPos);
        AssertClose(double3.Transform(double3.UnitX, gotRot), double3.Transform(double3.UnitX, expRot));
    }
}

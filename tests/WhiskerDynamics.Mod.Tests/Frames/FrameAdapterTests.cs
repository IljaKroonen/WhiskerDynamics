using Brutal.Numerics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Frames;

public class FrameAdapterTests
{
    [Fact]
    public void Roundtrip_between_core_and_game_vectors_is_exact()
    {
        var v = new Vector3d(1.5e11, -3.2e7, 42.0);
        Assert.Equal(v, FrameAdapter.ToCore(FrameAdapter.ToGame(v)));
    }

    [Fact]
    public void Identity_quaternion_leaves_components_unchanged()
    {
        var v = new Vector3d(1.0, 2.0, 3.0);
        var game = FrameAdapter.EclToCci(v, doubleQuat.Identity);
        Assert.Equal(1.0, game.X, 15);
        Assert.Equal(2.0, game.Y, 15);
        Assert.Equal(3.0, game.Z, 15);
    }

    [Fact]
    public void EclToCci_applies_the_active_rotation_convention()
    {
        // Active +90-degree rotation about +Z maps +X to +Y, pinning quaternion direction.
        double s = Math.Sqrt(0.5);
        var q = new doubleQuat(0.0, 0.0, s, s);
        var rotated = FrameAdapter.EclToCci(new Vector3d(1.0, 0.0, 0.0), q);
        Assert.Equal(0.0, rotated.X, 12);
        Assert.Equal(1.0, rotated.Y, 12);
        Assert.Equal(0.0, rotated.Z, 12);
    }

    [Fact]
    public void EclToCci_then_CciToEcl_with_inverse_quat_roundtrips()
    {
        // Build the same rotation from an explicit matrix.
        var rot = doubleQuat.CreateFromRotationMatrix(new double4x4(
            0, 1, 0, 0,
            -1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1));
        var v = new Vector3d(1.0, 2.0, 3.0);
        var back = FrameAdapter.CciToEcl(FrameAdapter.EclToCci(v, rot), doubleQuat.Inverse(rot));
        Assert.Equal(v.X, back.X, 12);
        Assert.Equal(v.Y, back.Y, 12);
        Assert.Equal(v.Z, back.Z, 12);
    }

    [Fact]
    public void BubToEcl_then_EclToBub_roundtrips_with_the_same_quaternion()
    {
        // EclToBub and BubToEcl apply inverse directions of one quaternion.
        double s = Math.Sqrt(0.5);
        var bub2Cce = new doubleQuat(0.0, 0.0, s, s); // +90 degrees about +Z
        var bub = new double3(1.0, 2.0, 3.0);
        var ecl = FrameAdapter.BubToEcl(bub, bub2Cce);
        Assert.Equal(-2.0, ecl.X, 12); // active-rotation direction pin
        Assert.Equal(1.0, ecl.Y, 12);
        var back = FrameAdapter.EclToBub(ecl, bub2Cce);
        Assert.Equal(bub.X, back.X, 12);
        Assert.Equal(bub.Y, back.Y, 12);
        Assert.Equal(bub.Z, back.Z, 12);
    }
}

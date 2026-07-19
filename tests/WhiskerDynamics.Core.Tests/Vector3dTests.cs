using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class Vector3dTests
{
    [Fact]
    public void Cross_of_unit_x_and_unit_y_is_unit_z()
    {
        var z = new Vector3d(1, 0, 0).Cross(new Vector3d(0, 1, 0));
        Assert.Equal(0, z.X, 15);
        Assert.Equal(0, z.Y, 15);
        Assert.Equal(1, z.Z, 15);
    }

    [Fact]
    public void Dot_and_length_are_consistent()
    {
        var v = new Vector3d(3, 4, 12);
        Assert.Equal(169, v.Dot(v), 12);
        Assert.Equal(13, v.Length(), 12);
        Assert.Equal(169, v.LengthSquared(), 12);
        Assert.Equal(1, v.Normalized().Length(), 12);
    }

    [Fact]
    public void Operators_compose_linearly()
    {
        var a = new Vector3d(1, 2, 3);
        var b = new Vector3d(-4, 0, 5);
        var r = a + b * 2.0 - (-a) / 2.0;
        Assert.Equal(1 - 8 + 0.5, r.X, 12);
        Assert.Equal(2 + 0 + 1.0, r.Y, 12);
        Assert.Equal(3 + 10 + 1.5, r.Z, 12);
    }

    [Fact]
    public void RotateAbout_z_axis_by_90_degrees_maps_x_to_y()
    {
        var r = new Vector3d(1, 0, 0).RotateAbout(new Vector3d(0, 0, 1), Math.PI / 2);
        Assert.Equal(0, r.X, 12);
        Assert.Equal(1, r.Y, 12);
        Assert.Equal(0, r.Z, 12);
    }

    [Fact]
    public void StateVector_arithmetic_scales_both_components()
    {
        var s = new StateVector(new Vector3d(1, 0, 0), new Vector3d(0, 2, 0));
        var r = s + s * 0.5;
        Assert.Equal(1.5, r.Position.X, 12);
        Assert.Equal(3.0, r.Velocity.Y, 12);
    }
}

using WhiskerDynamics.Core;
using WhiskerDynamics.Mod.Ui;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Ui;

public class LagrangeOverlayTests
{
    [Theory]
    [InlineData(3.986004418e14, 4.9028e12)]
    [InlineData(4.9028e12, 3.986004418e14)]
    public void Named_points_preserve_standard_labels_for_either_mass_order(
        double primaryMu, double secondaryMu)
    {
        var direct = LagrangePotential.Equilibria(
            LagrangePotential.MassRatio(primaryMu, secondaryMu));
        var actual = LagrangeOverlay.NamedPoints(primaryMu, secondaryMu);

        for (int pointIndex = 0; pointIndex < 5; pointIndex++)
        {
            int mapped = primaryMu >= secondaryMu ? pointIndex : pointIndex switch
            {
                1 => 2,
                2 => 1,
                3 => 4,
                4 => 3,
                _ => 0,
            };
            Assert.Equal(direct[mapped], actual[pointIndex]);
        }
    }

    [Fact]
    public void Potential_control_is_available_only_for_body_pair_fixed_frames()
    {
        Assert.True(LagrangeOverlay.AvailableFor(
            new FrameSpec(FrameKind.TwoBodyFixed, "Luna", "Earth")));
        Assert.False(LagrangeOverlay.AvailableFor(
            new FrameSpec(FrameKind.Inertial, "Earth", null)));
        Assert.False(LagrangeOverlay.AvailableFor(
            new FrameSpec(FrameKind.Surface, "Earth", null)));
        Assert.False(LagrangeOverlay.AvailableFor(
            new FrameSpec(FrameKind.TargetFixed, "Earth", "Target")));
        Assert.False(LagrangeOverlay.AvailableFor(null));
    }
}

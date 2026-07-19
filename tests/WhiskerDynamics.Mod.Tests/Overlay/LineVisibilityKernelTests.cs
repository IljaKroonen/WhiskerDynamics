using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class LineVisibilityKernelTests
{
    [Fact]
    public void Stock_opt_in_always_rules_both_kinds()
    {
        Assert.False(LineVisibilityKernel.CelestialLineVisible(
            stockOptIn: false, showAstralBodyLines: true));
        Assert.False(LineVisibilityKernel.VesselLineVisible(
            stockOptIn: false, isControlled: true));
    }

    [Theory]
    [InlineData(true, true)]
    [InlineData(false, false)]
    public void Astral_body_toggle_controls_every_opted_in_celestial(
        bool showAstralBodyLines, bool expected)
    {
        Assert.Equal(expected, LineVisibilityKernel.CelestialLineVisible(
            stockOptIn: true, showAstralBodyLines));
    }

    [Theory]
    [InlineData(true, true)]
    [InlineData(false, false)]
    public void Only_the_controlled_vessel_is_visible(bool isControlled, bool expected)
    {
        Assert.Equal(expected, LineVisibilityKernel.VesselLineVisible(
            stockOptIn: true, isControlled));
    }

    [Theory]
    [InlineData(true, true, true)]
    [InlineData(true, false, false)]
    [InlineData(false, true, false)]
    [InlineData(false, false, false)]
    public void Only_active_map_vessel_bypasses_stock_orbit_visibility_check(
        bool isMapView, bool isActive, bool expected)
    {
        Assert.Equal(expected,
            LineVisibilityKernel.BypassOrbitVisibilityCheck(isMapView, isActive));
    }
}

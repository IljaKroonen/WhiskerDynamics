using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class DenseLineDrawTests
{
    [Fact]
    public void Stock_style_fade_preserves_rgb_and_applies_alpha_floor()
    {
        var source = new byte4(240, 90, 30, 255);

        var styled = DenseLineDraw.StyledColor(source, sampleIndex: 2, nowIndex: 3,
            invFadeCount: 0.1f, fadeOpacity: true);

        Assert.Equal(source.R, styled.R);
        Assert.Equal(source.G, styled.G);
        Assert.Equal(source.B, styled.B);
        Assert.Equal((byte)127, styled.A);
    }
}

using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Tests.Overlay;

public class DenseLineDrawTests
{
    [Fact]
    public void Actual_past_samples_are_muted_below_the_future_fade_floor()
    {
        var source = new byte4(240, 90, 30, 255);

        var past = DenseLineDraw.StyledColor(source, sampleIndex: 2, nowIndex: 3,
            invFadeCount: 0.1f, fadeOpacity: true, stylePastTrajectory: true);
        var future = DenseLineDraw.StyledColor(source, sampleIndex: 3, nowIndex: 3,
            invFadeCount: 0.1f, fadeOpacity: true, stylePastTrajectory: true);

        Assert.Equal((byte)158, past.R);
        Assert.Equal((byte)108, past.G);
        Assert.Equal((byte)88, past.B);
        Assert.Equal((byte)88, past.A);
        Assert.Equal(source, future);
    }

    [Fact]
    public void Other_lines_keep_the_stock_style_fade()
    {
        var source = new byte4(240, 90, 30, 255);

        var past = DenseLineDraw.StyledColor(source, sampleIndex: 2, nowIndex: 3,
            invFadeCount: 0.1f, fadeOpacity: true, stylePastTrajectory: false);

        Assert.Equal(source.R, past.R);
        Assert.Equal(source.G, past.G);
        Assert.Equal(source.B, past.B);
        Assert.Equal((byte)127, past.A);
    }

    [Fact]
    public void Entirely_elapsed_actual_lines_are_all_styled_as_past()
    {
        var source = new byte4(60, 120, 180, 64);

        var past = DenseLineDraw.StyledColor(source, sampleIndex: 4, nowIndex: -1,
            invFadeCount: 0.1f, fadeOpacity: true, stylePastTrajectory: true);

        Assert.Equal((byte)64, past.A);
        Assert.NotEqual(source, past);
    }
}

namespace WhiskerDynamics.Mod.Tests.Ui;

public class FramesPanelTimingTests
{
    [Fact]
    public void Readout_refreshes_initially_and_then_at_half_second_intervals()
    {
        Assert.True(FramesPanel.ReadoutRefreshDue(lastRefreshMs: 0, nowMs: 100));
        Assert.False(FramesPanel.ReadoutRefreshDue(lastRefreshMs: 100, nowMs: 599));
        Assert.True(FramesPanel.ReadoutRefreshDue(lastRefreshMs: 100, nowMs: 600));
        Assert.True(FramesPanel.ReadoutRefreshDue(lastRefreshMs: 600, nowMs: 100));
    }
}

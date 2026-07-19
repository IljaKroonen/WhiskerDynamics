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

    [Fact]
    public void History_coverage_refreshes_immediately_after_reset_and_duration_edit()
    {
        FramesPanel.ResetSessionStatics();
        try
        {
            Assert.True(FramesPanel.TryBeginHistoryCoverageRefresh(nowMs: 100));
            Assert.False(FramesPanel.TryBeginHistoryCoverageRefresh(nowMs: 599));
            Assert.True(FramesPanel.TryBeginHistoryCoverageRefresh(nowMs: 600));

            FramesPanel.InvalidateHistoryCoverageReadout();

            Assert.True(FramesPanel.TryBeginHistoryCoverageRefresh(nowMs: 600));
        }
        finally
        {
            FramesPanel.ResetSessionStatics();
        }
    }
}

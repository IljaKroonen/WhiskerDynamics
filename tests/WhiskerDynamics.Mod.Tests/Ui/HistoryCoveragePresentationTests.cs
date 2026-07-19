namespace WhiskerDynamics.Mod.Tests.Ui;

public class HistoryCoveragePresentationTests
{
    [Fact]
    public void Render_budget_truncation_is_a_point_limit_warning()
    {
        string text = FramesPanel.HistoryCoverageText(
            sampleT0: 365 * 86400.0,
            displaySeconds: 365 * 86400.0,
            requestedStartSeconds: 0,
            oldestRecordedStartSeconds: 0,
            oldestRenderedStartSeconds: 282 * 86400.0,
            renderBudgetTruncated: true);

        Assert.Equal(
            "showing 83d of requested 1y — trajectory point limit reached", text);
    }

    [Fact]
    public void Short_session_is_neutral_recorded_history_text()
    {
        string text = FramesPanel.HistoryCoverageText(
            sampleT0: 20 * 86400.0,
            displaySeconds: 30 * 86400.0,
            requestedStartSeconds: -10 * 86400.0,
            oldestRecordedStartSeconds: 8 * 86400.0,
            oldestRenderedStartSeconds: 8 * 86400.0,
            renderBudgetTruncated: false);

        Assert.Equal("recorded history available: 12d", text);
    }
}

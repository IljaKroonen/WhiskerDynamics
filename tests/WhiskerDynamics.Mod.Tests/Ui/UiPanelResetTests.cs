using System.Reflection;
using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.Mod.Tests.Ui;

[CollectionDefinition(nameof(UiPanelResetTestCollection), DisableParallelization = true)]
public sealed class UiPanelResetTestCollection
{
}

[Collection(nameof(UiPanelResetTestCollection))]
public class UiPanelResetTests
{
    [Fact]
    public void Frames_panel_reset_clears_fault_latch() =>
        AssertResetClearsFaultLatch(typeof(FramesPanel), FramesPanel.ResetSessionStatics);

    [Fact]
    public void Settings_panel_reset_clears_fault_latch() =>
        AssertResetClearsFaultLatch(typeof(SettingsPanel), SettingsPanel.ResetSessionStatics);

    [Fact]
    public void Burn_planner_panel_reset_clears_fault_latch() =>
        AssertResetClearsFaultLatch(typeof(BurnPlannerPanel), BurnPlannerPanel.ResetSessionStatics);

    [Fact]
    public void Burn_planner_window_is_closed_for_each_session() =>
        AssertWindowLifecycle(typeof(BurnPlannerPanel), BurnPlannerPanel.Open,
            BurnPlannerPanel.ResetSessionStatics);

    [Fact]
    public void Status_window_is_closed_for_each_session() =>
        AssertWindowLifecycle(typeof(StatusPanel), StatusPanel.Open,
            StatusPanel.ResetSessionStatics);

    [Fact]
    public void Status_panel_reset_hides_stock_patched_conic_diagnostic()
    {
        try
        {
            DiagnosticDisplay.ShowStockPatchedConics = true;

            StatusPanel.ResetSessionStatics();

            Assert.False(DiagnosticDisplay.ShowStockPatchedConics);
        }
        finally
        {
            StatusPanel.ResetSessionStatics();
        }
    }

    [Fact]
    public void Settings_window_is_closed_for_each_session() =>
        AssertWindowLifecycle(typeof(SettingsPanel), SettingsPanel.Open,
            SettingsPanel.ResetSessionStatics);

    private static void AssertResetClearsFaultLatch(Type panelType, Action reset)
    {
        FieldInfo errors = panelType.GetField(
            string.Concat('_', 'e', 'r', 'r', 'o', 'r', 's'),
            BindingFlags.NonPublic | BindingFlags.Static)!;

        try
        {
            errors.SetValue(null, 2);
            reset();
            Assert.Equal(0, (int)errors.GetValue(null)!);

            errors.SetValue(null, 3);
            reset();
            Assert.Equal(0, (int)errors.GetValue(null)!);
        }
        finally
        {
            reset();
        }
    }

    private static void AssertWindowLifecycle(Type panelType, Action open, Action reset)
    {
        FieldInfo openState = panelType.GetField("_open",
            BindingFlags.NonPublic | BindingFlags.Static)!;

        try
        {
            reset();
            Assert.False((bool)openState.GetValue(null)!);

            open();
            Assert.True((bool)openState.GetValue(null)!);

            reset();
            Assert.False((bool)openState.GetValue(null)!);
        }
        finally
        {
            reset();
        }
    }
}

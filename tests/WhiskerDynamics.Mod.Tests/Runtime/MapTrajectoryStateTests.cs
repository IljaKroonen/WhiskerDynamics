namespace WhiskerDynamics.Mod.Tests.Runtime;

public sealed class MapTrajectoryStateTests
{
    [Fact]
    public void New_game_state_uses_the_hardcoded_display_defaults()
    {
        var state = new MapTrajectoryState();

        Assert.Equal(30, state.HistoryDisplayDays);
        Assert.True(state.ShowAstralBodyLines);
    }

    [Fact]
    public void Display_choices_are_mutable_runtime_state()
    {
        var state = new MapTrajectoryState
        {
            HistoryDisplayDays = 7,
            ShowAstralBodyLines = false,
        };

        Assert.Equal(7, state.HistoryDisplayDays);
        Assert.False(state.ShowAstralBodyLines);
    }
}

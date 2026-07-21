namespace WhiskerDynamics.Mod.Tests.Runtime;

public sealed class MapTrajectoryStateTests
{
    [Fact]
    public void New_game_state_shows_astral_body_lines()
    {
        var state = new MapTrajectoryState();

        Assert.True(state.ShowAstralBodyLines);
    }

    [Fact]
    public void Astral_body_line_visibility_is_mutable_runtime_state()
    {
        var state = new MapTrajectoryState
        {
            ShowAstralBodyLines = false,
        };

        Assert.False(state.ShowAstralBodyLines);
    }
}

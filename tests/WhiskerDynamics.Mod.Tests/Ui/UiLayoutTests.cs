using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.Mod.Tests.Ui;

public class UiLayoutTests
{
    [Theory]
    [InlineData(500f, 3, 150f)]
    [InlineData(320f, 3, 92f)]
    [InlineData(200f, 3, 1f)]
    [InlineData(-10f, 3, 1f)]
    [InlineData(100f, 0, 100f)]
    public void Step_field_uses_remaining_width_up_to_its_target(
        float availableWidth, int stepCount, float expected) =>
        Assert.Equal(expected,
            UiLayout.StepFieldWidth(availableWidth, stepCount));
}

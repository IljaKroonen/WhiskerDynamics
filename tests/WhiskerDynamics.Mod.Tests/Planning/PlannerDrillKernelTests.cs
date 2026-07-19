using System.Globalization;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning;

public class PlannerDrillKernelTests
{
    [Fact]
    public void Valid_command_uses_invariant_numbers_under_comma_decimal_culture()
    {
        CultureInfo previous = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");

            Assert.True(PlannerDrillKernel.TryParse(
                " 90.5, 1.25, -2.5, 3e2 ", out var command, out string error));

            Assert.Equal("", error);
            Assert.Equal(90.5, command.OffsetSeconds);
            Assert.Equal(1.25, command.Prograde);
            Assert.Equal(-2.5, command.Normal);
            Assert.Equal(300, command.Outward);
        }
        finally
        {
            CultureInfo.CurrentCulture = previous;
        }
    }

    [Theory]
    [InlineData(null)]
    [InlineData("")]
    [InlineData("1,2,3")]
    [InlineData("1,2,3,4,5")]
    [InlineData("1,5,2,3,4")]
    public void Wrong_component_count_is_rejected(string? raw)
    {
        Assert.False(PlannerDrillKernel.TryParse(raw, out _, out string error));
        Assert.Contains("exactly 4", error);
    }

    [Theory]
    [InlineData("bad,1,2,3", "offsetSeconds")]
    [InlineData("1,bad,2,3", "prograde")]
    [InlineData("1,2,bad,3", "normal")]
    [InlineData("1,2,3,bad", "outward")]
    public void Unparseable_component_is_rejected_with_its_name(string raw, string component)
    {
        Assert.False(PlannerDrillKernel.TryParse(raw, out _, out string error));
        Assert.Contains(component, error);
        Assert.Contains("invariant-culture number", error);
    }

    [Fact]
    public void Offset_is_bounded_to_the_operational_future_window()
    {
        Assert.False(PlannerDrillKernel.TryParse("0,0,0,0", out _, out string zeroError));
        Assert.Contains("offsetSeconds", zeroError);
        Assert.False(PlannerDrillKernel.TryParse("0.999,0,0,0", out _, out string lowError));
        Assert.Contains("future", lowError);

        string minimum = PlannerDrillKernel.MinOffsetSeconds.ToString(
            "R", CultureInfo.InvariantCulture);
        Assert.True(PlannerDrillKernel.TryParse(minimum + ",0,0,0", out _, out _));

        string maximum = PlannerDrillKernel.MaxOffsetSeconds.ToString(
            "R", CultureInfo.InvariantCulture);
        Assert.True(PlannerDrillKernel.TryParse(maximum + ",0,0,0", out _, out _));
        string above = (PlannerDrillKernel.MaxOffsetSeconds + 1).ToString(
            "R", CultureInfo.InvariantCulture);
        Assert.False(PlannerDrillKernel.TryParse(above + ",0,0,0", out _,
            out string highError));
        Assert.Contains("offsetSeconds", highError);
    }

    [Theory]
    [InlineData("1e200")]
    [InlineData("1e308")]
    public void Individually_finite_delta_v_whose_combined_norm_overflows_is_rejected(
        string component)
    {
        Assert.False(PlannerDrillKernel.TryParse("10," + component + ",0,0",
            out _, out string error));
        Assert.Contains("combined VLF length-squared", error);
    }

    [Fact]
    public void Large_delta_v_with_finite_combined_norm_is_accepted()
    {
        Assert.True(PlannerDrillKernel.TryParse(
            "10,1e150,-1e150,1e150", out var command, out string error));
        Assert.Equal("", error);
        Assert.True(PlannerKernel.ValidateDv(
            command.Prograde, command.Normal, command.Outward));
    }

    public static IEnumerable<object[]> NonFiniteComponents()
    {
        string[] literals = ["NaN", "Infinity", "-Infinity"];
        string[] names = ["offsetSeconds", "prograde", "normal", "outward"];
        for (int component = 0; component < names.Length; component++)
        {
            foreach (string literal in literals)
            {
                string[] parts = ["1", "2", "3", "4"];
                parts[component] = literal;
                yield return [string.Join(',', parts), names[component]];
            }
        }
    }

    [Theory]
    [MemberData(nameof(NonFiniteComponents))]
    public void Every_nonfinite_component_is_rejected(string raw, string component)
    {
        Assert.False(PlannerDrillKernel.TryParse(raw, out var command, out string error));
        Assert.Equal(default, command);
        Assert.Contains(component, error);
        Assert.Contains("finite", error);
    }

    [Fact]
    public void Every_accepted_result_has_only_finite_values()
    {
        string[] corpus =
        [
            "1,0,0,0",
            "10,1,-2,3",
            "1e20,-1e-20,2.5,3",
            "NaN,1,2,3",
            "1,Infinity,2,3",
            "1,2,-Infinity,3",
            "10,1e200,0,0",
        ];

        foreach (string raw in corpus)
        {
            if (!PlannerDrillKernel.TryParse(raw, out var command, out _)) continue;
            Assert.True(double.IsFinite(command.OffsetSeconds));
            Assert.True(double.IsFinite(command.Prograde));
            Assert.True(double.IsFinite(command.Normal));
            Assert.True(double.IsFinite(command.Outward));
        }
    }
}

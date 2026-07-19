using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Ui;

/// <summary>The human-readable duration text the panels' time fields speak:
/// compact descending d/h/m/s formatting on a millisecond grid (no "60s"
/// rounding artifacts), the matching parser (descending unit tokens, bare
/// number = seconds, one leading minus for the whole value), and the
/// format-then-parse round trip across the magnitudes a plan can hold.</summary>
public class TimeDisplayKernelTests
{
    // ---- Formatting: compact descending units, zero omission.

    [Theory]
    [InlineData(0.0, "0s")]
    [InlineData(45.5, "45.5s")]
    [InlineData(42.0, "42s")]
    [InlineData(0.125, "0.125s")]
    [InlineData(90.0, "1m 30s")]
    [InlineData(5400.0, "1h 30m")]
    [InlineData(86400.0 * 7, "7d")]
    [InlineData(86400.0 * 400, "400d")] // days is the largest unit: no years
    [InlineData(86400.0 * 3 + 3600.0 * 4 + 60.0 * 5 + 6.25, "3d 4h 5m 6.25s")]
    public void Formats_compact_descending_units(double seconds, string expected)
        => Assert.Equal(expected, TimeDisplayKernel.FormatDuration(seconds));

    [Fact]
    public void Interior_zero_components_are_omitted()
    {
        Assert.Equal("1d 5s", TimeDisplayKernel.FormatDuration(86400.0 + 5.0));
        Assert.Equal("2h 30s", TimeDisplayKernel.FormatDuration(7200.0 + 30.0));
        Assert.Equal("1d 3m", TimeDisplayKernel.FormatDuration(86400.0 + 180.0));
    }

    [Fact]
    public void Seconds_decimals_trim_trailing_zeros()
    {
        Assert.Equal("42.5s", TimeDisplayKernel.FormatDuration(42.5));
        Assert.Equal("42.25s", TimeDisplayKernel.FormatDuration(42.25));
        Assert.Equal("1.5s", TimeDisplayKernel.FormatDuration(1.500));
    }

    [Fact]
    public void Sub_millisecond_residue_carries_instead_of_rendering_60s()
    {
        // 59.9999 s rounds up on the millisecond grid: must become a minute,
        // never the "60s" a per-component rounding would print.
        Assert.Equal("1m", TimeDisplayKernel.FormatDuration(59.9999));
        Assert.Equal("1h", TimeDisplayKernel.FormatDuration(3599.9999));
        Assert.Equal("1d", TimeDisplayKernel.FormatDuration(86399.9999));
        Assert.Equal("2m", TimeDisplayKernel.FormatDuration(119.99999));
    }

    [Fact]
    public void Negative_values_carry_one_leading_minus()
    {
        Assert.Equal("-45.5s", TimeDisplayKernel.FormatDuration(-45.5));
        Assert.Equal("-1h 30m", TimeDisplayKernel.FormatDuration(-5400.0));
        Assert.Equal("-3d 4h 5m 6.25s",
            TimeDisplayKernel.FormatDuration(-(86400.0 * 3 + 3600.0 * 4 + 60.0 * 5 + 6.25)));
    }

    [Fact]
    public void Values_that_round_to_zero_are_0s_without_sign()
    {
        Assert.Equal("0s", TimeDisplayKernel.FormatDuration(0.0001));
        Assert.Equal("0s", TimeDisplayKernel.FormatDuration(-0.0001));
        Assert.Equal("0s", TimeDisplayKernel.FormatDuration(-0.0));
    }

    [Fact]
    public void Forty_year_magnitudes_format_in_days()
    {
        double fortyYears = 40 * 365.25 * 86400.0;
        Assert.Equal("14610d", TimeDisplayKernel.FormatDuration(fortyYears));
        Assert.Equal("14610d 1m 1.5s", TimeDisplayKernel.FormatDuration(fortyYears + 61.5));
    }

    [Fact]
    public void Years_mode_promotes_whole_display_years()
    {
        // 365-day display years (the window controls' vocabulary), opt-in only:
        // the default stays day-capped for plan/burn fields.
        Assert.Equal("1y 35d", TimeDisplayKernel.FormatDuration(400 * 86400.0, years: true));
        Assert.Equal("40y", TimeDisplayKernel.FormatDuration(14600 * 86400.0, years: true));
        Assert.Equal("2y 1h", TimeDisplayKernel.FormatDuration(730 * 86400.0 + 3600, years: true));
        Assert.Equal("30d", TimeDisplayKernel.FormatDuration(30 * 86400.0, years: true));
        Assert.Equal("400d", TimeDisplayKernel.FormatDuration(400 * 86400.0));
    }

    [Theory]
    [InlineData(0.0, "0s")]
    [InlineData(1.49, "1s")]
    [InlineData(1.5, "2s")]
    [InlineData(59.5, "1m 0s")]
    [InlineData(3600.0, "1h 0m 0s")]
    [InlineData(86400.0, "1d 0h 0m 0s")]
    [InlineData(90061.5, "1d 1h 1m 2s")]
    public void Countdown_rounds_to_whole_seconds_and_keeps_lower_fields_stable(
        double seconds, string expected)
        => Assert.Equal(expected, TimeDisplayKernel.FormatCountdown(seconds));

    [Fact]
    public void Year_countdown_keeps_every_lower_field()
        => Assert.Equal("1y 0d 0h 0m 0s",
            TimeDisplayKernel.FormatCountdown(365 * 86400.0, years: true));

    // ---- Parsing: accepted forms.

    [Theory]
    [InlineData("3d 4h 5m 6.25s", 86400.0 * 3 + 3600.0 * 4 + 60.0 * 5 + 6.25)]
    [InlineData("3d4h", 86400.0 * 3 + 3600.0 * 4)]
    [InlineData("90m", 5400.0)]
    [InlineData("1.5d", 129600.0)]
    [InlineData("10m 30s", 630.0)]
    [InlineData("7d", 604800.0)]
    [InlineData("0s", 0.0)]
    [InlineData("42.5s", 42.5)]
    [InlineData("  1h  30m  ", 5400.0)] // whitespace is optional everywhere
    [InlineData("3 d 4 h", 86400.0 * 3 + 3600.0 * 4)] // space before the unit letter
    [InlineData("1D 2H 3M 4S", 86400.0 + 7200.0 + 180.0 + 4.0)] // units case-insensitive
    [InlineData("2y 30d", (2 * 365 + 30) * 86400.0)] // y = 365 display days
    [InlineData("40y", 40 * 365 * 86400.0)]
    [InlineData("1.5y", 1.5 * 365 * 86400.0)]
    public void Parses_descending_unit_tokens(string text, double expected)
    {
        Assert.True(TimeDisplayKernel.TryParseDuration(text, out double seconds));
        Assert.Equal(expected, seconds, precision: 9);
    }

    [Theory]
    [InlineData("120", 120.0)]
    [InlineData("0.5", 0.5)]
    [InlineData("120.25", 120.25)]
    public void Bare_number_means_seconds(string text, double expected)
    {
        Assert.True(TimeDisplayKernel.TryParseDuration(text, out double seconds));
        Assert.Equal(expected, seconds, precision: 9);
    }

    [Fact]
    public void One_leading_minus_negates_the_whole_value()
    {
        Assert.True(TimeDisplayKernel.TryParseDuration("-1h 30m", out double s));
        Assert.Equal(-5400.0, s, precision: 9);
        Assert.True(TimeDisplayKernel.TryParseDuration("-120", out double bare));
        Assert.Equal(-120.0, bare, precision: 9);
        Assert.True(TimeDisplayKernel.TryParseDuration("- 45.5s", out double spaced));
        Assert.Equal(-45.5, spaced, precision: 9);
    }

    // ---- Parsing: rejections.

    [Theory]
    [InlineData("")]
    [InlineData("   ")]
    [InlineData("garbage")]
    [InlineData("3x")] // unknown unit
    [InlineData("4h 3d")] // ascending units
    [InlineData("3d 3d")] // repeated unit
    [InlineData("30d 1y")] // years must lead
    [InlineData("1y 1y")]
    [InlineData("--5s")] // only one leading minus for the whole value
    [InlineData("3d -4h")] // negative component
    [InlineData("-")]
    [InlineData("3d 120")] // trailing number without a unit
    [InlineData("d")] // unit without a number
    [InlineData("1.2.3s")]
    [InlineData("5s 3")] // trailing garbage after a valid token
    [InlineData("5 s s")]
    public void Rejects_malformed_text(string text)
    {
        Assert.False(TimeDisplayKernel.TryParseDuration(text, out double seconds));
        Assert.Equal(0.0, seconds);
    }

    [Fact]
    public void Rejects_null()
        => Assert.False(TimeDisplayKernel.TryParseDuration(null, out _));

    // ---- Round trip: parse(format(x)) recovers x within the millisecond grid.

    [Theory]
    [InlineData(0.0)]
    [InlineData(0.125)]
    [InlineData(42.5)]
    [InlineData(59.9999)]
    [InlineData(61.25)]
    [InlineData(3600.5)]
    [InlineData(86400.0 * 1.5)]
    [InlineData(86400.0 * 400 + 3661.125)]
    [InlineData(-86400.0 * 3 - 4.75)]
    [InlineData(40 * 365.25 * 86400.0)]
    [InlineData(40 * 365.25 * 86400.0 + 59.999)]
    public void Round_trip_recovers_within_a_millisecond(double seconds)
    {
        string text = TimeDisplayKernel.FormatDuration(seconds);
        Assert.True(TimeDisplayKernel.TryParseDuration(text, out double parsed));
        Assert.Equal(seconds, parsed, tolerance: 1e-3);
    }

    [Fact]
    public void Round_trip_holds_over_a_magnitude_spread()
    {
        // Deterministic spread from sub-second to 40 years: mantissas chosen to
        // exercise fractional seconds at every scale.
        var random = new Random(42);
        double max = 40 * 365.25 * 86400.0;
        for (int i = 0; i < 500; i++)
        {
            double magnitude = Math.Pow(10, random.NextDouble() * Math.Log10(max / 0.001));
            double value = 0.001 * magnitude * (random.Next(2) == 0 ? 1 : -1);
            string text = TimeDisplayKernel.FormatDuration(value);
            Assert.True(TimeDisplayKernel.TryParseDuration(text, out double parsed),
                $"unparseable: '{text}' from {value:R}");
            Assert.True(Math.Abs(parsed - value) <= 1e-3,
                $"round trip drifted: {value:R} -> '{text}' -> {parsed:R}");
        }
    }
}

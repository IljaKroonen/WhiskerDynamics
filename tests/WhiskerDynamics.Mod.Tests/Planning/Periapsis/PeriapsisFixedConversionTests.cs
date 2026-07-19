using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning.Periapsis;

public sealed class PeriapsisFixedConversionTests
{
    [Fact]
    public void Production_choreography_renews_around_bounded_extension_before_conversion()
    {
        var events = new List<string>();

        int value = PeriapsisFixedConversionOrchestration.Run(
            () => { events.Add("poll"); return false; },
            () => events.Add("renew"),
            () => events.Add("extend"),
            () => { events.Add("convert"); return 42; });

        Assert.Equal(42, value);
        Assert.Equal(
            ["poll", "renew", "extend", "poll", "renew", "convert"],
            events);
    }

    [Fact]
    public void Cancellation_before_extension_runs_no_work()
    {
        var events = new List<string>();

        var error = Assert.Throws<OperationCanceledException>(() =>
            PeriapsisFixedConversionOrchestration.Run(
                () => true,
                () => events.Add("renew"),
                () => events.Add("extend"),
                () => { events.Add("convert"); return 42; }));

        Assert.Contains("cancelled", error.Message);
        Assert.Empty(events);
    }

    [Fact]
    public void Cancellation_during_extension_never_reaches_relstate_conversion()
    {
        bool stopped = false;
        var events = new List<string>();

        var error = Assert.Throws<OperationCanceledException>(() =>
            PeriapsisFixedConversionOrchestration.Run(
                () => stopped,
                () => events.Add("renew"),
                () => { events.Add("extend"); stopped = true; },
                () => { events.Add("convert"); return 42; }));

        Assert.Contains("cancelled", error.Message);
        Assert.Equal(["renew", "extend"], events);
        Assert.DoesNotContain("convert", events);
    }
}

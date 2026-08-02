using WhiskerDynamics.Mod.Planning;

namespace WhiskerDynamics.Mod.Tests.Planning;

[Collection("basis-urgency-statics")]
public class BasisReconversionUrgencyTests : IDisposable
{
    public BasisReconversionUrgencyTests() => BasisReconversionUrgency.Reset();

    public void Dispose() => BasisReconversionUrgency.Reset();

    [Fact]
    public void Raise_is_idempotent_and_visible_until_cleared()
    {
        Assert.False(BasisReconversionUrgency.Any);
        Assert.False(BasisReconversionUrgency.IsUrgent("Rocket"));

        BasisReconversionUrgency.Raise("Rocket");
        BasisReconversionUrgency.Raise("Rocket");
        Assert.True(BasisReconversionUrgency.Any);
        Assert.True(BasisReconversionUrgency.IsUrgent("Rocket"));
        Assert.Equal(["Rocket"], BasisReconversionUrgency.Snapshot());

        BasisReconversionUrgency.Clear("Rocket");
        Assert.False(BasisReconversionUrgency.Any);
        Assert.False(BasisReconversionUrgency.IsUrgent("Rocket"));
    }

    [Fact]
    public void Vessels_are_independent_and_ordinal_keyed()
    {
        BasisReconversionUrgency.Raise("Rocket");
        BasisReconversionUrgency.Raise("Gemini7");

        Assert.False(BasisReconversionUrgency.IsUrgent("rocket"));
        BasisReconversionUrgency.Clear("Rocket");
        Assert.True(BasisReconversionUrgency.IsUrgent("Gemini7"));
        Assert.True(BasisReconversionUrgency.Any);
    }

    [Fact]
    public void Clear_of_unknown_vessel_is_a_no_op()
    {
        BasisReconversionUrgency.Clear("NeverRaised");
        Assert.False(BasisReconversionUrgency.Any);
    }

    [Fact]
    public void Generation_clear_removes_only_the_observed_raise()
    {
        BasisReconversionUrgency.Raise("Rocket");
        long? observed = BasisReconversionUrgency.Observe("Rocket");
        Assert.NotNull(observed);

        BasisReconversionUrgency.Raise("Rocket");
        BasisReconversionUrgency.Clear("Rocket", observed!.Value);
        Assert.True(BasisReconversionUrgency.IsUrgent("Rocket"));

        long? fresh = BasisReconversionUrgency.Observe("Rocket");
        Assert.NotEqual(observed, fresh);
        BasisReconversionUrgency.Clear("Rocket", fresh!.Value);
        Assert.False(BasisReconversionUrgency.IsUrgent("Rocket"));
        Assert.Null(BasisReconversionUrgency.Observe("Rocket"));
    }

    [Fact]
    public void Reset_drops_every_pending_flag()
    {
        BasisReconversionUrgency.Raise("Rocket");
        BasisReconversionUrgency.Raise("Gemini7");
        BasisReconversionUrgency.Reset();
        Assert.False(BasisReconversionUrgency.Any);
        Assert.Empty(BasisReconversionUrgency.Snapshot());
    }

    [Fact]
    public async Task Concurrent_raise_and_clear_never_corrupt_the_store()
    {
        var raisers = Enumerable.Range(0, 4).Select(_ => Task.Run(() =>
        {
            for (int i = 0; i < 1000; i++)
                BasisReconversionUrgency.Raise($"vessel-{i % 8}");
        }));
        var clearers = Enumerable.Range(0, 2).Select(_ => Task.Run(() =>
        {
            for (int i = 0; i < 1000; i++)
                BasisReconversionUrgency.Clear($"vessel-{i % 8}");
        }));
        await Task.WhenAll([.. raisers, .. clearers]);

        foreach (string id in BasisReconversionUrgency.Snapshot())
            Assert.True(BasisReconversionUrgency.IsUrgent(id));
        BasisReconversionUrgency.Reset();
        Assert.False(BasisReconversionUrgency.Any);
    }
}

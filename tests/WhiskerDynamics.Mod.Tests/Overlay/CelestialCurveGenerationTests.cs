using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Overlay;

[CollectionDefinition("celestial-curve-generation", DisableParallelization = true)]
public sealed class CelestialCurveGenerationCollection;

[Collection("celestial-curve-generation")]
public sealed class CelestialCurveGenerationTests
{
    [Fact]
    public async Task Reset_rejects_parked_old_publication_without_replacing_new_curve()
    {
        string bodyId = nameof(Reset_rejects_parked_old_publication_without_replacing_new_curve);
        using var oldPublishParked = new ManualResetEventSlim();
        using var releaseOldPublish = new ManualResetEventSlim();
        Task<bool>? oldPublish = null;
        CelestialCurves.ResetSessionStatics();
        long oldGeneration = CelestialCurves.CurrentGeneration;
        try
        {
            oldPublish = Task.Run(() => CelestialCurves.TryPublishSentinelForTest(
                bodyId, "old", oldGeneration, () =>
                {
                    oldPublishParked.Set();
                    releaseOldPublish.Wait();
                }));
            Assert.True(oldPublishParked.Wait(5000));

            CelestialCurves.ResetSessionStatics();
            long currentGeneration = CelestialCurves.CurrentGeneration;
            Assert.NotEqual(oldGeneration, currentGeneration);
            Assert.True(CelestialCurves.TryPublishSentinelForTest(
                bodyId, "new", currentGeneration));
            Assert.Equal("new", CelestialCurves.PublishedSentinelForTest(bodyId));

            releaseOldPublish.Set();
            Assert.False(await oldPublish.WaitAsync(TimeSpan.FromSeconds(5)));
            Assert.Equal("new", CelestialCurves.PublishedSentinelForTest(bodyId));
        }
        finally
        {
            releaseOldPublish.Set();
            if (oldPublish is not null)
            {
                try { await oldPublish.WaitAsync(TimeSpan.FromSeconds(5)); }
                catch { }
            }
            CelestialCurves.ResetSessionStatics();
        }
    }

    [Fact]
    public void Sampling_completion_is_frame_specific_and_cleared_by_reset()
    {
        CelestialCurves.ResetSessionStatics();
        try
        {
            Assert.True(CelestialCurves.TryPublishCompletionForTest(
                400, "Earth-Centred Inertial", CelestialCurves.CurrentGeneration));
            Assert.Equal(400, CelestialCurves.CompletedWindowDays("Earth-Centred Inertial"));
            Assert.Equal(0, CelestialCurves.CompletedWindowDays("Earth Surface"));

            CelestialCurves.ResetSessionStatics();
            Assert.Equal(0, CelestialCurves.CompletedWindowDays("Earth-Centred Inertial"));
        }
        finally
        {
            CelestialCurves.ResetSessionStatics();
        }
    }
}

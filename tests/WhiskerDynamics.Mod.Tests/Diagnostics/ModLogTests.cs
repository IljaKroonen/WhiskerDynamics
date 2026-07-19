using System.Diagnostics;

namespace WhiskerDynamics.Mod.Tests.Diagnostics;

[CollectionDefinition(nameof(ModLogTestCollection), DisableParallelization = true)]
public sealed class ModLogTestCollection { }

[Collection(nameof(ModLogTestCollection))]
public class ModLogTests
{
    [Fact]
    public void Snapshot_is_versioned_and_reused_between_writes()
    {
        long before = ModLog.Version;
        ModLog.Info("snapshot-version-probe");
        var first = ModLog.Snapshot();
        var second = ModLog.Snapshot();
        Assert.True(ModLog.Version > before);
        Assert.Same(first, second);
        Assert.Contains(first, line => line.Contains("snapshot-version-probe"));
    }

    [Fact]
    public void Producer_burst_is_bounded_and_does_not_wait_for_disk()
    {
        var sw = Stopwatch.StartNew();
        for (int i = 0; i < 10_000; i++) ModLog.Info("burst-" + i);
        sw.Stop();
        Assert.True(sw.ElapsedMilliseconds < 5000, $"producer burst took {sw.ElapsedMilliseconds} ms");
        Assert.True(ModLog.Snapshot().Count <= 200);
    }
}

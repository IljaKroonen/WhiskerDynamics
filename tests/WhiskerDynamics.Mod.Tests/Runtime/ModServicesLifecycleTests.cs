using WhiskerDynamics.Mod;
using System.Runtime.CompilerServices;

namespace WhiskerDynamics.Mod.Tests.Runtime;

public sealed class ModServicesLifecycleTests
{
    private sealed class BindingSlot(ModServices.Binding initial)
    {
        internal ModServices.Binding Value = initial;
    }

    [Fact]
    public void Producer_binding_token_rejects_a_new_session_even_if_captured_work_resumes_late()
    {
        object oldRails = new();
        object newRails = new();

        Assert.True(ModServices.BindingTokenMatches(
            capturedGeneration: 4, currentGeneration: 4,
            oldRails, oldRails, faulted: false));
        Assert.False(ModServices.BindingTokenMatches(
            capturedGeneration: 4, currentGeneration: 5,
            oldRails, newRails, faulted: false));
        Assert.False(ModServices.BindingTokenMatches(
            capturedGeneration: 4, currentGeneration: 4,
            oldRails, newRails, faulted: false));
    }

    [Fact]
    public void Fault_latch_invalidates_an_otherwise_matching_binding_token()
    {
        object rails = new();

        Assert.False(ModServices.BindingTokenMatches(
            capturedGeneration: 7, currentGeneration: 7,
            rails, rails, faulted: true));
    }

    [Fact]
    public async Task Stale_failure_side_effect_is_suppressed_when_rebind_wins_the_gate()
    {
        object gate = new();
        int currentGeneration = 1;
        int sideEffects = 0;
        using var attempted = new ManualResetEventSlim();

        System.Threading.Monitor.Enter(gate);
        Task<bool> stale = Task.Run(() =>
        {
            attempted.Set();
            return BindingSideEffectPolicy.TryRun(
                gate,
                () => Volatile.Read(ref currentGeneration) == 1,
                () => Interlocked.Increment(ref sideEffects));
        });
        try
        {
            Assert.True(attempted.Wait(TimeSpan.FromSeconds(5)));
            Volatile.Write(ref currentGeneration, 2);
        }
        finally
        {
            System.Threading.Monitor.Exit(gate);
        }

        Assert.False(await stale.WaitAsync(TimeSpan.FromSeconds(5)));
        Assert.Equal(0, Volatile.Read(ref sideEffects));
        Assert.True(BindingSideEffectPolicy.TryRun(
            gate,
            () => Volatile.Read(ref currentGeneration) == 2,
            () => Interlocked.Increment(ref sideEffects)));
        Assert.Equal(1, Volatile.Read(ref sideEffects));
    }

    [Fact]
    public async Task Immutable_binding_snapshot_never_exposes_a_cross_generation_pair()
    {
        var railsA = (RailsService)RuntimeHelpers.GetUninitializedObject(typeof(RailsService));
        var railsB = (RailsService)RuntimeHelpers.GetUninitializedObject(typeof(RailsService));
        var vesselsA = (VesselRegistry)RuntimeHelpers.GetUninitializedObject(typeof(VesselRegistry));
        var vesselsB = (VesselRegistry)RuntimeHelpers.GetUninitializedObject(typeof(VesselRegistry));
        var a = new ModServices.Binding(new object(), 10, railsA, vesselsA);
        var b = new ModServices.Binding(new object(), 11, railsB, vesselsB);
        var slot = new BindingSlot(a);
        using var start = new ManualResetEventSlim();

        Task writer = Task.Run(() =>
        {
            start.Wait();
            for (int i = 0; i < 100_000; i++)
                Volatile.Write(ref slot.Value, (i & 1) == 0 ? b : a);
        });
        Task[] readers = Enumerable.Range(0, 4).Select(_ => Task.Run(() =>
        {
            start.Wait();
            for (int i = 0; i < 100_000; i++)
            {
                ModServices.BoundServices snapshot =
                    Volatile.Read(ref slot.Value).Services;
                bool pairA = snapshot.Generation == 10
                    && ReferenceEquals(snapshot.Rails, railsA)
                    && ReferenceEquals(snapshot.Vessels, vesselsA);
                bool pairB = snapshot.Generation == 11
                    && ReferenceEquals(snapshot.Rails, railsB)
                    && ReferenceEquals(snapshot.Vessels, vesselsB);
                Assert.True(pairA || pairB);
            }
        })).ToArray();

        start.Set();
        await Task.WhenAll(readers.Append(writer)).WaitAsync(TimeSpan.FromSeconds(10));
    }
}

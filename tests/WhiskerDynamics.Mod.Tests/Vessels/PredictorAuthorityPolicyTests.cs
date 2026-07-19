using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Vessels;

public sealed class PredictorAuthorityPolicyTests
{
    private sealed class PublicationState
    {
        internal bool Authoritative = true;
        internal bool SideEffect;
        internal List<string> Order { get; } = [];
    }

    private static PredictorAuthorityPolicy.Reason Classify(
        bool entryPresent = true,
        bool sameEntry = true,
        bool boundVehicleAvailable = true,
        bool sameVehicle = true,
        bool reseedPending = false,
        bool committedFreefall = true,
        bool predictorAvailable = true,
        bool samePredictor = true) =>
        PredictorAuthorityPolicy.Classify(new(
            entryPresent, sameEntry, boundVehicleAvailable, sameVehicle,
            reseedPending, committedFreefall, predictorAvailable, samePredictor));

    [Theory]
    [InlineData(true, true, true, true, false, true, true, true,
        PredictorAuthorityPolicy.Reason.Authoritative)]
    [InlineData(false, true, true, true, false, true, true, true,
        PredictorAuthorityPolicy.Reason.MissingEntry)]
    [InlineData(true, false, true, true, false, true, true, true,
        PredictorAuthorityPolicy.Reason.EntryReplaced)]
    [InlineData(true, true, false, true, false, true, true, true,
        PredictorAuthorityPolicy.Reason.BoundVehicleUnavailable)]
    [InlineData(true, true, true, false, false, true, true, true,
        PredictorAuthorityPolicy.Reason.VehicleReplaced)]
    [InlineData(true, true, true, true, true, true, true, true,
        PredictorAuthorityPolicy.Reason.ReseedPending)]
    [InlineData(true, true, true, true, false, false, true, true,
        PredictorAuthorityPolicy.Reason.NotFreefall)]
    [InlineData(true, true, true, true, false, true, false, true,
        PredictorAuthorityPolicy.Reason.PredictorUnavailable)]
    [InlineData(true, true, true, true, false, true, true, false,
        PredictorAuthorityPolicy.Reason.PredictorReplaced)]
    public void Lifecycle_matrix_reports_the_first_strict_failure(
        bool entryPresent, bool sameEntry, bool boundVehicleAvailable,
        bool sameVehicle, bool reseedPending, bool committedFreefall,
        bool predictorAvailable, bool samePredictor,
        PredictorAuthorityPolicy.Reason expected)
    {
        var reason = Classify(
            entryPresent, sameEntry, boundVehicleAvailable, sameVehicle,
            reseedPending, committedFreefall, predictorAvailable, samePredictor);

        Assert.Equal(expected, reason);
        Assert.Equal(expected == PredictorAuthorityPolicy.Reason.Authoritative,
            PredictorAuthorityPolicy.IsAuthoritative(reason));
    }

    [Fact]
    public void Every_verdict_has_precise_user_facing_detail()
    {
        foreach (var reason in Enum.GetValues<PredictorAuthorityPolicy.Reason>())
            Assert.False(string.IsNullOrWhiteSpace(
                PredictorAuthorityPolicy.Describe(reason)));
    }

    [Fact]
    public async Task Final_validation_and_side_effect_are_atomic_against_publication()
    {
        var gate = new object();
        var state = new PublicationState();
        using var validationEntered = new ManualResetEventSlim();
        using var releaseValidation = new ManualResetEventSlim();
        using var publicationStarted = new ManualResetEventSlim();

        Task<(bool Executed, string Result)> operation = Task.Run(() =>
        {
            bool executed = RailsAuthoritySynchronization.TryExecute(
                gate,
                () =>
                {
                    validationEntered.Set();
                    if (!releaseValidation.Wait(TimeSpan.FromSeconds(5)))
                        throw new TimeoutException("test did not release final validation");
                    return state.Authoritative;
                },
                () =>
                {
                    state.SideEffect = true;
                    state.Order.Add("add");
                    return "queued";
                },
                out string result);
            return (executed, result);
        });

        Assert.True(validationEntered.Wait(TimeSpan.FromSeconds(5)));
        Task publication = Task.Run(() =>
        {
            publicationStarted.Set();
            RailsAuthoritySynchronization.Publish(
                gate, state,
                static current =>
                {
                    current.Authoritative = false;
                    current.Order.Add("pending");
                });
        });

        Assert.True(publicationStarted.Wait(TimeSpan.FromSeconds(5)));
        releaseValidation.Set();
        var outcome = await operation;
        await publication;

        Assert.True(outcome.Executed);
        Assert.Equal("queued", outcome.Result);
        Assert.True(state.SideEffect);
        Assert.Equal(["add", "pending"], state.Order);
    }

    [Fact]
    public void Publication_that_wins_refuses_the_side_effect()
    {
        var gate = new object();
        var state = new PublicationState();
        RailsAuthoritySynchronization.Publish(
            gate, state,
            static current => current.Authoritative = false);

        bool executed = RailsAuthoritySynchronization.TryExecute(
            gate,
            () => state.Authoritative,
            () =>
            {
                state.SideEffect = true;
                return "queued";
            },
            out string result);

        Assert.False(executed);
        Assert.Null(result);
        Assert.False(state.SideEffect);
    }
}

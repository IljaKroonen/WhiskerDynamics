using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Vessels;

/// <summary>Offline tests for the complete KSA-free policy seam called by
/// VesselRegistry.VerifyCommit.</summary>
public sealed class VesselRegistryCanaryTests
{
    private enum FaultPoint
    {
        None,
        BeforeEligibility,
        DuringEvaluation,
    }

    private enum RacePoint
    {
        None,
        BeforeEligibility,
        DuringTolerance,
        DuringEvaluation,
    }

    private sealed class ProbeFaultException(string message) : Exception(message);

    private struct Probe : IVesselRegistryCanaryProbe
    {
        private readonly VesselLifecycle.CommitCanaryEligibility _eligibility;
        private readonly double _residual;
        private readonly double _tolerance;
        private readonly FaultPoint _fault;
        private readonly RacePoint _race;
        private readonly CanaryCounter? _raceCanary;

        internal Probe(
            VesselLifecycle.CommitCanaryEligibility eligibility =
                VesselLifecycle.CommitCanaryEligibility.Comparable,
            double residual = 0.0,
            double tolerance = 10.0,
            FaultPoint fault = FaultPoint.None,
            RacePoint race = RacePoint.None,
            CanaryCounter? raceCanary = null)
        {
            _eligibility = eligibility;
            _residual = residual;
            _tolerance = tolerance;
            _fault = fault;
            _race = race;
            _raceCanary = raceCanary;
        }

        public VesselLifecycle.CommitCanaryEligibility CaptureAndClassify()
        {
            if (_race == RacePoint.BeforeEligibility) AdvanceToReplacementLineage();
            if (_fault == FaultPoint.BeforeEligibility)
                throw new ProbeFaultException("fault before eligibility");
            return _eligibility;
        }

        public double CommitTime => 1001.0;

        public double EvaluateResidual()
        {
            if (_race == RacePoint.DuringEvaluation) AdvanceToReplacementLineage();
            if (_fault == FaultPoint.DuringEvaluation)
                throw new ProbeFaultException("fault during evaluation");
            return _residual;
        }

        public double ToleranceMeters
        {
            get
            {
                if (_race == RacePoint.DuringTolerance) AdvanceToReplacementLineage();
                return _tolerance;
            }
        }

        private void AdvanceToReplacementLineage()
        {
            CanaryCounter canary = _raceCanary
                ?? throw new InvalidOperationException("Race injection needs a counter.");
            canary.BeginContinuityTransition();
            canary.EndContinuityTransition();

            // Record the replacement lineage's first miss through this exact policy.
            var replacement = new Probe(residual: 11.0, tolerance: 10.0);
            var recorded = VesselRegistryCanary.Verify(canary, ref replacement);
            if (recorded.Completion.Kind != CanaryCounter.CompletionKind.Recorded
                || recorded.Completion.Strikes != 1)
                throw new InvalidOperationException("Replacement miss was not recorded.");
        }
    }

    private struct BlockingProbe(
        ManualResetEventSlim ready,
        ManualResetEventSlim release) : IVesselRegistryCanaryProbe
    {
        public VesselLifecycle.CommitCanaryEligibility CaptureAndClassify() =>
            VesselLifecycle.CommitCanaryEligibility.Comparable;

        public double CommitTime => 1001.0;

        public double EvaluateResidual()
        {
            ready.Set();
            release.Wait();
            return 11.0;
        }

        public double ToleranceMeters => 10.0;
    }

    private static VesselRegistryCanary.Verification Run(
        CanaryCounter canary,
        VesselLifecycle.CommitCanaryEligibility eligibility =
            VesselLifecycle.CommitCanaryEligibility.Comparable,
        double residual = 0.0,
        double tolerance = 10.0,
        FaultPoint fault = FaultPoint.None,
        RacePoint race = RacePoint.None,
        CanaryCounter? raceCanary = null)
    {
        var probe = new Probe(eligibility, residual, tolerance, fault, race, raceCanary);
        return VesselRegistryCanary.Verify(canary, ref probe);
    }

    [Theory]
    [InlineData(true, true, true, 1001.0, 1000.0,
        VesselLifecycle.CommitCanaryEligibility.ReseedPending)]
    [InlineData(false, false, true, 1001.0, 1000.0,
        VesselLifecycle.CommitCanaryEligibility.NotFreefall)]
    [InlineData(false, true, false, 1001.0, 1000.0,
        VesselLifecycle.CommitCanaryEligibility.ReplacementVehicle)]
    [InlineData(false, true, true, 1000.0, 1000.0,
        VesselLifecycle.CommitCanaryEligibility.SeedOrReseedTick)]
    [InlineData(false, true, true, double.NaN, 1000.0,
        VesselLifecycle.CommitCanaryEligibility.SeedOrReseedTick)]
    [InlineData(false, true, true, double.PositiveInfinity, 1000.0,
        VesselLifecycle.CommitCanaryEligibility.SeedOrReseedTick)]
    public void Classifies_every_observed_skip_reason(
        bool reseedPending, bool isFreefall, bool sameVehicle,
        double committedTime, double seedTime,
        VesselLifecycle.CommitCanaryEligibility expected)
    {
        Assert.Equal(expected, VesselLifecycle.ClassifyCommitCanary(
            reseedPending, isFreefall, sameVehicle, committedTime, seedTime));
    }

    [Theory]
    [InlineData(VesselLifecycle.CommitCanaryEligibility.ReseedPending)]
    [InlineData(VesselLifecycle.CommitCanaryEligibility.NotFreefall)]
    [InlineData(VesselLifecycle.CommitCanaryEligibility.ReplacementVehicle)]
    [InlineData(VesselLifecycle.CommitCanaryEligibility.SeedOrReseedTick)]
    public void Exact_policy_breaks_streak_for_every_ineligible_commit(
        VesselLifecycle.CommitCanaryEligibility eligibility)
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var skipped = Run(canary, eligibility);

        Assert.Equal(CanaryCounter.CompletionKind.Discarded, skipped.Completion.Kind);
        Assert.Equal(0, canary.Strikes);
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
    }

    [Fact]
    public void No_callback_suspends_but_uninterrupted_three_misses_are_fatal()
    {
        var canary = new CanaryCounter();
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
        Assert.Equal(2, Run(canary, residual: 11.0).Completion.Strikes);

        // Pause/no task result: VerifyCommit and the policy are not called.

        var third = Run(canary, residual: 11.0).Completion;
        Assert.Equal(CanaryCounter.CompletionKind.Fatal, third.Kind);
        Assert.Equal(3, third.Strikes);
    }

    [Fact]
    public void Comparable_hit_resets_the_miss_streak()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var hit = Run(canary, residual: 10.0);

        Assert.False(hit.Miss);
        Assert.Equal(0, hit.Completion.Strikes);
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
    }

    [Fact]
    public void Repeated_observed_skips_are_idempotent()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        for (int i = 0; i < 3; i++)
            Assert.Equal(CanaryCounter.CompletionKind.Discarded,
                Run(canary, VesselLifecycle.CommitCanaryEligibility.NotFreefall)
                    .Completion.Kind);

        Assert.Equal(0, canary.Strikes);
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
    }

    [Fact]
    public void Composite_reseed_and_bind_has_no_available_gap_between_nested_transitions()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        canary.BeginContinuityTransition(); // outer ReseedAndBind
        canary.BeginContinuityTransition(); // Reseed
        canary.EndContinuityTransition();
        // A throwing probe proves unavailable observations return before snapshot work.
        var between = Run(canary, fault: FaultPoint.BeforeEligibility);
        canary.BeginContinuityTransition(); // identity-changing BindVehicle
        canary.EndContinuityTransition();
        canary.EndContinuityTransition();

        Assert.Equal(CanaryCounter.CompletionKind.Discarded, between.Completion.Kind);
        Assert.Equal(0, canary.Strikes);
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
    }

    [Fact]
    public void Fault_before_eligibility_is_contained_and_breaks_the_current_streak()
    {
        AssertCurrentFaultBreaks(FaultPoint.BeforeEligibility);
    }

    [Fact]
    public void Transient_comparable_probe_failure_is_contained_and_resets_on_success()
    {
        var canary = new CanaryCounter();

        var failed = Run(canary, fault: FaultPoint.DuringEvaluation);

        Assert.IsType<ProbeFaultException>(failed.Failure);
        Assert.Equal(CanaryCounter.CompletionKind.Recorded, failed.Completion.Kind);
        Assert.Equal(1, failed.Completion.ProbeFailures);
        Assert.Equal(1, canary.ProbeFailures);

        var recovered = Run(canary, residual: 0.0);

        Assert.Null(recovered.Failure);
        Assert.Equal(0, recovered.Completion.ProbeFailures);
        Assert.Equal(0, canary.ProbeFailures);
        Assert.Equal(CanaryCounter.CompletionKind.Recorded,
            Run(canary, fault: FaultPoint.DuringEvaluation).Completion.Kind);
    }

    private static void AssertCurrentFaultBreaks(FaultPoint fault)
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var failed = Run(canary, fault: fault);

        Assert.IsType<ProbeFaultException>(failed.Failure);
        Assert.Equal(CanaryCounter.CompletionKind.Discarded, failed.Completion.Kind);
        Assert.Equal(0, failed.Completion.ProbeFailures);
        Assert.Equal(0, canary.Strikes);
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
    }

    [Fact]
    public void Permanently_broken_comparable_probe_reaches_bounded_fatal_threshold()
    {
        var canary = new CanaryCounter();

        for (int failure = 1; failure < 3; failure++)
        {
            var result = Run(canary, fault: FaultPoint.DuringEvaluation);
            Assert.IsType<ProbeFaultException>(result.Failure);
            Assert.Equal(CanaryCounter.CompletionKind.Recorded, result.Completion.Kind);
            Assert.Equal(failure, result.Completion.ProbeFailures);
        }

        var fatal = Run(canary, fault: FaultPoint.DuringEvaluation);

        Assert.IsType<ProbeFaultException>(fatal.Failure);
        Assert.Equal(CanaryCounter.CompletionKind.Fatal, fatal.Completion.Kind);
        Assert.Equal(3, fatal.Completion.ProbeFailures);
        Assert.Equal(0, fatal.Completion.Strikes);
    }

    [Fact]
    public void Miss_and_probe_failure_streaks_do_not_combine()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var failure = Run(canary, fault: FaultPoint.DuringEvaluation);

        Assert.Equal(0, failure.Completion.Strikes);
        Assert.Equal(1, failure.Completion.ProbeFailures);

        var miss = Run(canary, residual: 11.0);

        Assert.Equal(1, miss.Completion.Strikes);
        Assert.Equal(0, miss.Completion.ProbeFailures);
    }

    [Fact]
    public void Stale_ineligible_callback_cannot_reset_a_new_lineage_miss()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var stale = Run(canary,
            VesselLifecycle.CommitCanaryEligibility.NotFreefall,
            race: RacePoint.BeforeEligibility,
            raceCanary: canary);

        Assert.Equal(CanaryCounter.CompletionKind.Discarded, stale.Completion.Kind);
        Assert.Equal(1, stale.Completion.Strikes);
        Assert.Equal(1, canary.Strikes);
    }

    [Fact]
    public void Stale_evaluation_failure_cannot_reset_a_new_lineage_miss()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var stale = Run(canary,
            fault: FaultPoint.DuringEvaluation,
            race: RacePoint.DuringEvaluation,
            raceCanary: canary);

        Assert.IsType<ProbeFaultException>(stale.Failure);
        Assert.Equal(CanaryCounter.CompletionKind.Discarded, stale.Completion.Kind);
        Assert.Equal(0, stale.Completion.ProbeFailures);
        Assert.Equal(1, canary.Strikes);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Non_finite_residual_is_always_a_miss(double residual)
    {
        var result = Run(new CanaryCounter(), residual: residual, tolerance: 10.0);

        Assert.True(result.Miss);
        Assert.Equal(1, result.Completion.Strikes);
    }

    [Fact]
    public void Negative_residual_is_conservatively_a_miss()
    {
        var result = Run(new CanaryCounter(), residual: -1.0, tolerance: 10.0);

        Assert.True(result.Miss);
        Assert.Equal(1, result.Completion.Strikes);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    [InlineData(-1.0)]
    public void Invalid_tolerance_discards_and_breaks_the_current_streak(double tolerance)
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var result = Run(canary, residual: 0.0, tolerance: tolerance);

        Assert.Equal(CanaryCounter.CompletionKind.Discarded, result.Completion.Kind);
        Assert.False(result.Miss);
        Assert.True(double.IsNaN(result.Residual));
        Assert.Equal(0, canary.Strikes);
        Assert.Equal(1, Run(canary, residual: 11.0).Completion.Strikes);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    [InlineData(-1.0)]
    public void Perfect_residuals_with_invalid_tolerance_never_become_fatal(double tolerance)
    {
        var canary = new CanaryCounter();

        for (int i = 0; i < 3; i++)
        {
            var result = Run(canary, residual: 0.0, tolerance: tolerance);
            Assert.Equal(CanaryCounter.CompletionKind.Discarded, result.Completion.Kind);
            Assert.False(result.Miss);
        }

        Assert.Equal(0, canary.Strikes);
    }

    [Fact]
    public void Stale_invalid_tolerance_cannot_reset_a_new_lineage_miss()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);

        var stale = Run(canary,
            residual: 0.0,
            tolerance: double.NaN,
            race: RacePoint.DuringTolerance,
            raceCanary: canary);

        Assert.Equal(CanaryCounter.CompletionKind.Discarded, stale.Completion.Kind);
        Assert.False(stale.Miss);
        Assert.Equal(1, stale.Completion.Strikes);
        Assert.Equal(1, canary.Strikes);
    }

    [Fact]
    public void Zero_tolerance_remains_valid_for_a_perfect_residual()
    {
        var result = Run(new CanaryCounter(), residual: 0.0, tolerance: 0.0);

        Assert.Equal(CanaryCounter.CompletionKind.Recorded, result.Completion.Kind);
        Assert.False(result.Miss);
        Assert.Equal(0, result.Completion.Strikes);
    }

    [Fact]
    public async Task Concurrent_reseed_discards_old_completion_without_resetting_new_miss()
    {
        var canary = new CanaryCounter();
        Run(canary, residual: 11.0);
        Run(canary, residual: 11.0);
        using var ready = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        Task<VesselRegistryCanary.Verification> oldVerification = Task.Run(() =>
        {
            var probe = new BlockingProbe(ready, release);
            return VesselRegistryCanary.Verify(canary, ref probe);
        });

        Assert.True(ready.Wait(TimeSpan.FromSeconds(5)));
        VesselRegistryCanary.Verification replacement;
        try
        {
            canary.BeginContinuityTransition();
            canary.EndContinuityTransition();
            replacement = Run(canary, residual: 11.0);
        }
        finally
        {
            release.Set();
        }

        var stale = await oldVerification;
        Assert.Equal(CanaryCounter.CompletionKind.Recorded, replacement.Completion.Kind);
        Assert.Equal(1, replacement.Completion.Strikes);
        Assert.Equal(CanaryCounter.CompletionKind.Discarded, stale.Completion.Kind);
        Assert.Equal(1, canary.Strikes);
    }
}

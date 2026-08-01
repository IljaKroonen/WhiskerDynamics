using Brutal.Numerics;
using WhiskerDynamics.Mod;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Planning;

public class PlannerKernelTests
{
    [Fact]
    public void ComposeVlf_pins_the_stock_component_convention()
    {
        // VLF components are prograde, normal, and outward in X, Y, and Z respectively.
        var dv = PlannerKernel.ComposeVlf(prograde: 1.0, normal: 2.0, outward: 3.0);
        Assert.Equal(1.0, dv.X);
        Assert.Equal(2.0, dv.Y);
        Assert.Equal(3.0, dv.Z);
    }

    [Fact]
    public void DecomposeVlf_round_trips_compose()
    {
        var (p, n, o) = PlannerKernel.DecomposeVlf(new double3(-0.5, 4.25, 9.0));
        Assert.Equal(-0.5, p);
        Assert.Equal(4.25, n);
        Assert.Equal(9.0, o);
    }

    [Theory]
    [InlineData(11_999, 10_000, false)]
    [InlineData(12_000, 10_000, true)]
    [InlineData(12_001, 10_000, true)]
    [InlineData(9_999, 10_000, false)]
    public void Live_delta_v_must_stay_quiet_for_the_full_rebase_grace(
        long nowMs, long lastWitnessMs, bool expected) =>
        Assert.Equal(expected,
            PlannerKernel.LiveDeltaVHasSettled(nowMs, lastWitnessMs));

    [Fact]
    public void ValidateAdd_accepts_a_clean_future_burn()
    {
        var verdict = PlannerKernel.ValidateAdd(
            burnTime: 1000.0, now: 100.0, existingBurnTimes: [500.0, 2000.0], patchFound: true);
        Assert.Equal(PlannerKernel.Verdict.Ok, verdict);
    }

    [Fact]
    public void ValidateAdd_rejects_past_nonfinite_duplicate_and_patchless()
    {
        Assert.Equal(PlannerKernel.Verdict.NotAhead,
            PlannerKernel.ValidateAdd(99.0, 100.0, [], patchFound: true));
        Assert.Equal(PlannerKernel.Verdict.NotAhead,       // inside the minimum lead window
            PlannerKernel.ValidateAdd(100.5, 100.0, [], patchFound: true));
        Assert.Equal(PlannerKernel.Verdict.NotFinite,
            PlannerKernel.ValidateAdd(double.NaN, 100.0, [], patchFound: true));
        Assert.Equal(PlannerKernel.Verdict.DuplicateTime,
            PlannerKernel.ValidateAdd(500.0, 100.0, [500.0], patchFound: true));
        Assert.Equal(PlannerKernel.Verdict.DuplicateTime,
            PlannerKernel.ValidateAdd(
                500.0 + BurnIdentityPolicy.ToleranceSeconds * 0.5,
                100.0, [500.0], patchFound: true));
        Assert.Equal(PlannerKernel.Verdict.NoPatch,
            PlannerKernel.ValidateAdd(1000.0, 100.0, [], patchFound: false));
    }

    [Fact]
    public void ValidateAdd_rejects_addition_overflow_before_patch_availability()
    {
        double baseTime = double.MaxValue;
        double offset = double.MaxValue;
        double overflowedBurnTime = baseTime + offset;
        Assert.Equal(double.PositiveInfinity, overflowedBurnTime);

        Assert.Equal(PlannerKernel.Verdict.NotFinite,
            PlannerKernel.ValidateAdd(overflowedBurnTime, 100.0, [], patchFound: false));
        Assert.Equal(PlannerKernel.Verdict.NotFinite,
            PlannerKernel.ValidateAddTiming(overflowedBurnTime, 100.0, []));
    }

    [Fact]
    public void ValidateTimeEdit_excludes_the_edited_burns_own_slot()
    {
        // Moving a burn to a NEW clean time is fine even though its old time is in the list
        // the caller passes (the caller passes OTHER burns' times only — pinned here).
        Assert.Equal(PlannerKernel.Verdict.Ok,
            PlannerKernel.ValidateTimeEdit(1500.0, 100.0, otherBurnTimes: [500.0]));
        Assert.Equal(PlannerKernel.Verdict.DuplicateTime,
            PlannerKernel.ValidateTimeEdit(500.0, 100.0, otherBurnTimes: [500.0]));
        Assert.Equal(PlannerKernel.Verdict.DuplicateTime,
            PlannerKernel.ValidateTimeEdit(
                500.0 + BurnIdentityPolicy.ToleranceSeconds * 0.5,
                100.0, otherBurnTimes: [500.0]));
        Assert.Equal(PlannerKernel.Verdict.NotAhead,
            PlannerKernel.ValidateTimeEdit(50.0, 100.0, otherBurnTimes: []));
    }

    [Fact]
    public void ValidateDv_requires_finite_components_and_stock_compatible_length_squared()
    {
        Assert.True(PlannerKernel.ValidateDv(0, 0, 0));
        Assert.True(PlannerKernel.ValidateDv(-3000, 250.5, 1e4));
        Assert.True(PlannerKernel.ValidateDv(1e150, -1e150, 1e150));
        Assert.False(PlannerKernel.ValidateDv(double.NaN, 0, 0));
        Assert.False(PlannerKernel.ValidateDv(0, double.PositiveInfinity, 0));
        Assert.False(PlannerKernel.ValidateDv(1e200, 0, 0));
        Assert.False(PlannerKernel.ValidateDv(0, 1e308, 0));
    }

    [Theory]
    [InlineData(1000.1, 1000.0, true)]
    [InlineData(1000.0, 1000.0, false)]
    [InlineData(999.9, 1000.0, false)]
    [InlineData(0.0, 0.0, false)]
    public void Delta_v_budget_warns_only_when_plan_strictly_exceeds_vessel(
        double planTotal, double vesselAvailable, bool expected)
        => Assert.Equal(expected,
            PlannerKernel.IsDeltaVOverBudget(planTotal, vesselAvailable));

    [Fact]
    public void Delta_v_budget_does_not_warn_for_invalid_readouts()
    {
        Assert.False(PlannerKernel.IsDeltaVOverBudget(double.NaN, 1000.0));
        Assert.False(PlannerKernel.IsDeltaVOverBudget(1000.0, double.PositiveInfinity));
        Assert.False(PlannerKernel.IsDeltaVOverBudget(-1.0, 1000.0));
        Assert.False(PlannerKernel.IsDeltaVOverBudget(1000.0, -1.0));
    }

    [Fact]
    public void Describe_names_every_verdict()
    {
        foreach (PlannerKernel.Verdict verdict in Enum.GetValues<PlannerKernel.Verdict>())
            Assert.False(string.IsNullOrWhiteSpace(PlannerKernel.Describe(verdict)));
    }

    [Fact]
    public void BurnBasisParent_prefers_the_burn_time_patch_parent()
    {
        // Burn delta-v is expressed in the burn-time patch parent's frame.
        Assert.Equal("Luna", PlannerKernel.BurnBasisParent("Luna", "Earth"));
        // The panel-time orbit parent is only the no-patch fallback.
        Assert.Equal("Earth", PlannerKernel.BurnBasisParent(null, "Earth"));
    }

    [Theory]
    [InlineData(10_000, 0, true)]
    [InlineData(30, 0, false)] // too close to ignition to touch
    [InlineData(double.NaN, 0, false)] // hostile stock burn time
    public void Automatic_rewrites_require_a_burn_safely_before_ignition(
        double burnTime, double now, bool expected)
    {
        Assert.Equal(expected, PlannerKernel.SafelyAheadForRewrite(burnTime, now));
    }
}

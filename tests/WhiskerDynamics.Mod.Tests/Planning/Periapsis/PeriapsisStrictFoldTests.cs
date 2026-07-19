using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning.Periapsis;

public sealed class PeriapsisStrictFoldTests
{
    private static readonly EngineScalars Engine = new(1000.0, 3000.0, 2.0);
    private static readonly FiniteBurnFold Finite = new(Engine, 5.0, 32);

    [Fact]
    public void Finite_burn_applies_exact_admitted_slices_and_returns_chain_state()
    {
        var burn = Burn(2000.0, new Vector3d(150.0, 0.0, 0.0));
        var applied = new List<(double Time, Vector3d Dv)>();

        var result = PeriapsisStrictFold.Fold([burn], 0.0, 3000.0, Finite,
            _ => new Vector3d(0.0, 150.0, 0.0),
            (time, dv) => applied.Add((time, dv)));

        var expected = FiniteBurnKernel.Expand(
            burn.Time, 150.0, Engine, Finite.SliceSeconds, Finite.MaxSlices)!;
        Assert.True(result.Success);
        Assert.Null(result.Failure);
        Assert.Equal(1, result.AppliedBurns);
        Assert.Equal(expected.Times, applied.Select(item => item.Time));
        Assert.Equal(expected.Magnitudes,
            applied.Select(item => item.Dv.Length()), new DoubleToleranceComparer(1e-12));
        Assert.All(applied, item => Assert.Equal(0.0, item.Dv.X));
        Assert.All(applied, item => Assert.Equal(0.0, item.Dv.Z));
        Assert.Equal(expected.IgnitionSeconds + expected.DurationSeconds,
            result.LastBoundSeconds);
        Assert.Equal(FiniteBurnKernel.MassAfterBurn(150.0, Engine),
            result.EngineAtTarget.MassKg);
    }

    [Fact]
    public void K1_impulse_still_reserves_its_physical_cutoff_and_debits_mass()
    {
        var model = new FiniteBurnFold(Engine, 60.0, 32);
        var burn = Burn(2000.0, new Vector3d(150.0, 0.0, 0.0));
        var applied = new List<(double Time, Vector3d Dv)>();

        var result = PeriapsisStrictFold.Fold([burn], 0.0, 3000.0, model,
            _ => burn.DvVlf,
            (time, dv) => applied.Add((time, dv)));

        double duration = FiniteBurnKernel.BurnDurationSeconds(150.0, Engine);
        var impulse = Assert.Single(applied);
        Assert.True(result.Success);
        Assert.Equal(burn.Time, impulse.Time);
        Assert.Equal(burn.DvVlf, impulse.Dv);
        Assert.Equal(burn.Time + duration / 2.0, result.LastBoundSeconds, 12);
        Assert.Equal(FiniteBurnKernel.MassAfterBurn(150.0, Engine),
            result.EngineAtTarget.MassKg);
    }

    [Fact]
    public void Null_conversion_fails_without_applying_or_committing_mass_and_bound()
    {
        int adds = 0;
        var result = PeriapsisStrictFold.Fold(
            [Burn(2000.0, new Vector3d(150.0, 0.0, 0.0))],
            100.0, 3000.0, Finite,
            _ => null,
            (_, _) => adds++);

        Assert.False(result.Success);
        Assert.Contains("degenerate VLF", result.Failure);
        Assert.Equal(0, result.AppliedBurns);
        Assert.Equal(0, adds);
        Assert.Equal(100.0, result.LastBoundSeconds);
        Assert.Equal(Engine, result.EngineAtTarget);
    }

    [Fact]
    public void Duplicate_node_is_neither_converted_applied_nor_debited_twice()
    {
        var first = Burn(2000.0, new Vector3d(150.0, 0.0, 0.0));
        var duplicate = Burn(
            2000.0 + BurnIdentityPolicy.ToleranceSeconds * 0.5,
            new Vector3d(300.0, 0.0, 0.0));
        int conversions = 0;
        var applied = new List<(double Time, Vector3d Dv)>();

        var result = PeriapsisStrictFold.Fold([first, duplicate], 0.0, 3000.0, Finite,
            index => { conversions++; return index == 0 ? first.DvVlf : duplicate.DvVlf; },
            (time, dv) => applied.Add((time, dv)));

        var expected = FiniteBurnKernel.Expand(
            first.Time, 150.0, Engine, Finite.SliceSeconds, Finite.MaxSlices)!;
        Assert.True(result.Success);
        Assert.Equal(1, result.AppliedBurns);
        Assert.Equal(1, conversions);
        Assert.Equal(expected.Times.Length, applied.Count);
        Assert.Equal(FiniteBurnKernel.MassAfterBurn(150.0, Engine),
            result.EngineAtTarget.MassKg);
        Assert.Equal(expected.IgnitionSeconds + expected.DurationSeconds,
            result.LastBoundSeconds);
    }

    [Fact]
    public void Overlap_rejects_second_burn_without_an_impulse_fallback()
    {
        var first = Burn(2000.0, new Vector3d(150.0, 0.0, 0.0));
        var second = Burn(2015.0, new Vector3d(150.0, 0.0, 0.0));
        var applied = new List<(double Time, Vector3d Dv)>();
        var expectedFirst = FiniteBurnKernel.Expand(
            first.Time, 150.0, Engine, Finite.SliceSeconds, Finite.MaxSlices)!;

        var result = PeriapsisStrictFold.Fold([first, second], 0.0, 3000.0, Finite,
            index =>
            {
                if (index == 1)
                    Assert.Equal(expectedFirst.Times.Length, applied.Count);
                return index == 0 ? first.DvVlf : second.DvVlf;
            },
            (time, dv) => applied.Add((time, dv)));

        Assert.False(result.Success);
        Assert.Contains("overlaps", result.Failure);
        Assert.Equal(1, result.AppliedBurns);
        Assert.Equal(expectedFirst.Times.Length, applied.Count);
        Assert.DoesNotContain(applied, item => item.Time == second.Time);
        Assert.Equal(FiniteBurnKernel.MassAfterBurn(150.0, Engine),
            result.EngineAtTarget.MassKg);
        Assert.Equal(expectedFirst.IgnitionSeconds + expectedFirst.DurationSeconds,
            result.LastBoundSeconds);
    }

    [Fact]
    public void Horizon_rejection_never_calls_the_impulse_fallback()
    {
        const double horizon = 2005.0;
        int adds = 0;
        var result = PeriapsisStrictFold.Fold(
            [Burn(2000.0, new Vector3d(150.0, 0.0, 0.0))],
            0.0, horizon, Finite,
            _ => new Vector3d(150.0, 0.0, 0.0),
            (_, _) => adds++);

        Assert.False(result.Success);
        Assert.Contains("prediction horizon", result.Failure);
        Assert.Equal(0, result.AppliedBurns);
        Assert.Equal(0, adds);
        Assert.Equal(Engine, result.EngineAtTarget);
        Assert.Equal(0.0, result.LastBoundSeconds);
    }

    [Fact]
    public void Admission_uses_authored_norm_once_even_when_conversion_norm_drifts()
    {
        var burn = Burn(2000.0, new Vector3d(150.0, 0.0, 0.0));
        var authored = FiniteBurnKernel.Expand(
            burn.Time, 150.0, Engine, Finite.SliceSeconds, Finite.MaxSlices)!;
        double exactHorizon = authored.IgnitionSeconds + authored.DurationSeconds;
        var applied = new List<Vector3d>();

        var result = PeriapsisStrictFold.Fold([burn], 0.0, exactHorizon, Finite,
            _ => new Vector3d(0.0, 300.0, 0.0),
            (_, dv) => applied.Add(dv));

        Assert.True(result.Success);
        Assert.Equal(authored.Times.Length, applied.Count);
        Assert.Equal(150.0, applied.Sum(dv => dv.Length()), 9);
        Assert.Equal(exactHorizon, result.LastBoundSeconds);
    }

    [Fact]
    public void Out_of_window_burns_do_not_participate_in_any_chain_state()
    {
        int conversions = 0, adds = 0;
        var result = PeriapsisStrictFold.Fold(
            [Burn(100.0, new Vector3d(150.0, 0.0, 0.0)),
             Burn(4000.0, new Vector3d(150.0, 0.0, 0.0))],
            100.0, 3000.0, Finite,
            _ => { conversions++; return new Vector3d(150.0, 0.0, 0.0); },
            (_, _) => adds++);

        Assert.True(result.Success);
        Assert.Equal(0, result.AppliedBurns);
        Assert.Equal(0, conversions);
        Assert.Equal(0, adds);
        Assert.Equal(Engine, result.EngineAtTarget);
        Assert.Equal(100.0, result.LastBoundSeconds);
    }

    private static (double Time, Vector3d DvVlf, string BasisParentId) Burn(
        double time, Vector3d dv) => (time, dv, "parent");

    private sealed class DoubleToleranceComparer(double tolerance) : IEqualityComparer<double>
    {
        public bool Equals(double x, double y) => Math.Abs(x - y) <= tolerance;
        public int GetHashCode(double obj) => obj.GetHashCode();
    }
}

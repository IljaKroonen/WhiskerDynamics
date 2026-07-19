using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Planning;

/// <summary>Tests finite-burn duration, mass, slicing, and impulse invariants.</summary>
public class FiniteBurnKernelTests
{
    /// <summary>1000 kg ship, 3000 m/s exhaust velocity, 2 kg/s flow — closed-form
    /// numbers: a 300 m/s burn consumes 1000·(1−e^(−0.1)) ≈ 95.1626 kg over
    /// ≈ 47.5813 s.</summary>
    private static EngineScalars Engine(double mass = 1000, double ve = 3000, double mdot = 2)
        => new(mass, ve, mdot);

    [Fact]
    public void Rcs_reduction_uses_net_axial_force_and_total_selected_flow()
    {
        var engine = RcsPerformanceKernel.FromSelectedJets(1000,
        [
            (100.0, 0.10), // forward
            (50.0, 0.10),  // canted jet's axial contribution
            (-10.0, 0.05), // imperfect opposing component still spends propellant
        ]);
        Assert.Equal(1000, engine.MassKg);
        Assert.Equal(0.25, engine.MassFlowRate, 12);
        Assert.Equal(560.0, engine.ExhaustVelocity, 12);
    }

    [Fact]
    public void Rcs_reduction_rejects_unusable_or_nonfinite_jet_sets()
    {
        Assert.False(RcsPerformanceKernel.FromSelectedJets(1000, []).Usable);
        Assert.False(RcsPerformanceKernel.FromSelectedJets(1000, [(0.0, 0.1)]).Usable);
        Assert.False(RcsPerformanceKernel.FromSelectedJets(1000, [(1.0, double.NaN)]).Usable);
        Assert.False(RcsPerformanceKernel.FromSelectedJets(0, [(1.0, 0.1)]).Usable);
    }


    [Theory]
    [InlineData(1000, 3000, 2, true)]
    [InlineData(0, 3000, 2, false)]      // massless: nothing to burn
    [InlineData(1000, 0, 2, false)]      // no exhaust velocity: no engine
    [InlineData(1000, 3000, 0, false)]   // no flow: infinite duration
    [InlineData(-1, 3000, 2, false)]
    [InlineData(double.NaN, 3000, 2, false)]
    [InlineData(1000, double.PositiveInfinity, 2, false)]
    public void Usable_requires_all_three_scalars_finite_and_positive(
        double mass, double ve, double mdot, bool usable)
        => Assert.Equal(usable, new EngineScalars(mass, ve, mdot).Usable);


    [Fact]
    public void Duration_is_the_flight_computers_rocket_equation()
    {
        double expectedPropellant = 1000.0 * (1.0 - Math.Exp(-300.0 / 3000.0));
        Assert.Equal(expectedPropellant / 2.0,
            FiniteBurnKernel.BurnDurationSeconds(300.0, Engine()), 12);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-5.0)]
    public void Duration_is_zero_for_non_positive_delta_v(double dv)
        => Assert.Equal(0.0, FiniteBurnKernel.BurnDurationSeconds(dv, Engine()));

    [Fact]
    public void Duration_is_zero_for_an_unusable_engine()
        => Assert.Equal(0.0, FiniteBurnKernel.BurnDurationSeconds(300.0, Engine(mdot: 0)));

    [Fact]
    public void Mass_after_burn_is_the_rocket_equation_and_chains()
    {
        var engine = Engine();
        double after1 = FiniteBurnKernel.MassAfterBurn(300.0, engine);
        Assert.Equal(1000.0 * Math.Exp(-0.1), after1, 9);
        // The same delta-v takes less propellant and time after the ship lightens.
        double duration1 = FiniteBurnKernel.BurnDurationSeconds(300.0, engine);
        double duration2 = FiniteBurnKernel.BurnDurationSeconds(300.0, engine with { MassKg = after1 });
        Assert.True(duration2 < duration1);
        double after2 = FiniteBurnKernel.MassAfterBurn(300.0, engine with { MassKg = after1 });
        Assert.Equal(FiniteBurnKernel.MassAfterBurn(600.0, engine), after2, 9);
    }


    [Theory]
    [InlineData(47.6, 20.0, 32, 3)]   // ceil(47.6/20) = 3
    [InlineData(47.6, 60.0, 32, 1)]   // shorter than one slice: stays an impulse
    [InlineData(1000.0, 20.0, 32, 32)] // capped
    [InlineData(47.6, 0.0, 32, 1)]    // feature off
    [InlineData(0.0, 20.0, 32, 1)]    // degenerate duration
    [InlineData(47.6, 20.0, 0, 1)]    // cap floor is 1
    public void Slice_count_is_ceiling_capped_and_degenerate_safe(
        double duration, double sliceSeconds, int maxSlices, int expected)
        => Assert.Equal(expected, FiniteBurnKernel.SliceCount(duration, sliceSeconds, maxSlices));


    [Fact]
    public void Expand_returns_null_when_one_slice_suffices()
    {
        Assert.Null(FiniteBurnKernel.Expand(5000.0, 300.0, Engine(), sliceSeconds: 60.0, maxSlices: 32));
        Assert.Null(FiniteBurnKernel.Expand(5000.0, 300.0, Engine(mdot: 0), 20.0, 32));
        Assert.Null(FiniteBurnKernel.Expand(5000.0, 0.0, Engine(), 20.0, 32));
    }

    [Fact]
    public void Expand_centers_the_window_on_the_node_with_midpoint_slice_times()
    {
        double node = 5000.0;
        var expansion = FiniteBurnKernel.Expand(node, 300.0, Engine(), 20.0, 32)!;
        double duration = FiniteBurnKernel.BurnDurationSeconds(300.0, Engine());
        Assert.Equal(3, expansion.Times.Length);
        Assert.Equal(duration, expansion.DurationSeconds, 12);
        Assert.Equal(node - duration / 2.0, expansion.IgnitionSeconds, 12); // FC centering (:735)
        double sliceDt = duration / 3.0;
        for (int i = 0; i < 3; i++)
            Assert.Equal(expansion.IgnitionSeconds + (i + 0.5) * sliceDt, expansion.Times[i], 12);
        Assert.True(expansion.Times[0] < expansion.Times[1] && expansion.Times[1] < expansion.Times[2]);
        Assert.Equal(node, (expansion.Times[0] + expansion.Times[^1]) / 2.0, 9);
    }

    [Fact]
    public void Expand_returns_null_when_the_burn_would_empty_the_tank()
    {
        // Rocket-equation underflow must not produce non-finite impulses.
        Assert.Null(FiniteBurnKernel.Expand(5000.0, 120_000.0, Engine(), 20.0, 32));
        var expansion = FiniteBurnKernel.Expand(5000.0, 9000.0, Engine(), 20.0, 32);
        if (expansion is not null)
            Assert.All(expansion.Magnitudes, m => Assert.True(double.IsFinite(m)));
    }

    [Fact]
    public void Expand_magnitudes_telescope_exactly_to_the_total_and_grow_as_the_ship_lightens()
    {
        var expansion = FiniteBurnKernel.Expand(5000.0, 300.0, Engine(), 5.0, 32)!;
        Assert.Equal(10, expansion.Times.Length); // ceil(47.58/5)
        Assert.Equal(300.0, expansion.Magnitudes.Sum(), 9); // ln-telescoping: exact by construction
        for (int i = 1; i < expansion.Magnitudes.Length; i++)
            Assert.True(expansion.Magnitudes[i] > expansion.Magnitudes[i - 1],
                $"slice {i} should out-accelerate slice {i - 1} (constant thrust, falling mass)");
    }

    [Theory]
    [InlineData(60.0, 32)]
    [InlineData(5.0, 1)]
    public void One_impulse_representation_preserves_the_nonzero_physical_window(
        double sliceSeconds, int maxSlices)
    {
        const double node = 5000.0;
        Assert.True(FiniteBurnKernel.TryResolveCommand(
            node, 300.0, Engine(), sliceSeconds, maxSlices, out var command));

        Assert.Null(command.Expansion);
        Assert.True(command.Window.IgnitionSeconds < node);
        Assert.True(command.Window.CutoffSeconds > node);
        Assert.Equal(FiniteBurnKernel.BurnDurationSeconds(300.0, Engine()),
            command.Window.DurationSeconds, 12);
    }

    [Fact]
    public void Multi_slice_command_preserves_one_physical_window_and_expansion()
    {
        Assert.True(FiniteBurnKernel.TryResolveCommand(
            5000.0, 300.0, Engine(), 5.0, 32, out var command));

        var expansion = Assert.IsType<FiniteBurnExpansion>(command.Expansion);
        Assert.Equal(command.Window.IgnitionSeconds, expansion.IgnitionSeconds);
        Assert.Equal(command.Window.CutoffSeconds,
            expansion.IgnitionSeconds + expansion.DurationSeconds);
    }

    [Theory]
    [InlineData(double.NaN, 300.0, 20.0)]
    [InlineData(5000.0, double.PositiveInfinity, 20.0)]
    [InlineData(5000.0, 300.0, 0.0)]
    public void Command_resolution_rejects_invalid_inputs(
        double node, double magnitude, double sliceSeconds) =>
        Assert.False(FiniteBurnKernel.TryResolveCommand(
            node, magnitude, Engine(), sliceSeconds, 32, out _));
}

using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Planning;

/// <summary>An integrator-free oracle for the finite-burn midpoint discretization.</summary>
public sealed class FiniteBurnFidelityTests
{
    [Fact]
    public void Midpoint_impulses_converge_at_second_order_to_continuous_thrust()
    {
        const double deltaV = 300.0;
        var engine = new EngineScalars(1_000.0, 3_000.0, 2.0);
        double duration = StableDuration(deltaV, engine);
        double finalMass = engine.MassKg - engine.MassFlowRate * duration;
        double exactDisplacement = engine.ExhaustVelocity / engine.MassFlowRate
            * (engine.MassKg
                - finalMass * (1.0 + Math.Log(engine.MassKg / finalMass)));

        FiniteBurnExpansion coarse = ExpansionWithSlices(8);
        FiniteBurnExpansion fine = ExpansionWithSlices(16);
        double coarseError = DisplacementError(coarse, exactDisplacement);
        double fineError = DisplacementError(fine, exactDisplacement);
        double ratio = coarseError / fineError;

        Assert.InRange(ratio, 3.9, 4.1);
        Assert.InRange(Math.Abs(coarse.Magnitudes.Sum() - deltaV), 0.0, 1e-10);
        Assert.InRange(Math.Abs(fine.Magnitudes.Sum() - deltaV), 0.0, 1e-10);

        var defaults = new ModConfig();
        FiniteBurnExpansion shipping = FiniteBurnKernel.Expand(
            0.0, deltaV, engine, defaults.FiniteBurnSliceSeconds,
            defaults.FiniteBurnMaxSlices)!;
        FiniteBurnExpansion refined = FiniteBurnKernel.Expand(
            0.0, deltaV, engine, sliceSeconds: 5.0,
            maxSlices: ModConfig.MaxFiniteBurnMaxSlices)!;
        Assert.Equal(3, shipping.Times.Length);
        Assert.Equal(10, refined.Times.Length);
        Assert.True(DisplacementError(refined, exactDisplacement)
            < DisplacementError(shipping, exactDisplacement) / 8.0);

        FiniteBurnExpansion ExpansionWithSlices(int slices) =>
            FiniteBurnKernel.Expand(0.0, deltaV, engine,
                duration / (slices - 0.25), slices)!;
    }

    private static double DisplacementError(FiniteBurnExpansion expansion,
        double exactDisplacement)
    {
        double cutoff = expansion.IgnitionSeconds + expansion.DurationSeconds;
        double displacement = 0.0;
        for (int i = 0; i < expansion.Times.Length; i++)
            displacement += (cutoff - expansion.Times[i]) * expansion.Magnitudes[i];
        return Math.Abs(displacement - exactDisplacement);
    }

    private static double StableDuration(double deltaV, EngineScalars engine)
    {
        double x = -deltaV / engine.ExhaustVelocity;
        double expm1 = Math.Abs(x) < 1e-5
            ? x * (1.0 + x * (0.5 + x * (1.0 / 6.0 + x * (1.0 / 24.0 + x / 120.0))))
            : Math.Exp(x) - 1.0;
        return -engine.MassKg * expm1 / engine.MassFlowRate;
    }
}

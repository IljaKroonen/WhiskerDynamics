namespace WhiskerDynamics.Core;

/// <summary>Unit-conversion constants used when parsing Astronomicals.xml. The defaults
/// are WhiskerDynamics.Core's built-in values; the mod overrides them with the running game
/// build's own KSA.Constants values so every mu matches the game exactly
/// (a 3.0e-5 relative solar-mass mismatch measures as 400-3800 km epoch errors).</summary>
public sealed record MassConstants
{
    public double G { get; init; } = Constants.G;
    public double SolarMassKg { get; init; } = Constants.SolarMassKg;
    public double EarthMassKg { get; init; } = 5.97219e24;
    public double LunarMassKg { get; init; } = 7.349e22;
    public double JupiterMassKg { get; init; } = 1.89819e27;
}

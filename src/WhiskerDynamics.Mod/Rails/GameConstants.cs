using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Rails;

/// <summary>The running game build's own physical constants, read via reflection
/// (GetRawConstantValue) so nothing is baked in at the mod's compile time.</summary>
public sealed record GameConstants(
    double G, double SolarMassKg, double EarthMassKg, double LunarMassKg, double JupiterMassKg)
{
    public static GameConstants ReadFromGame()
    {
        static double Const(string name) =>
            (double)typeof(KSA.Constants).GetField(name)!.GetRawConstantValue()!;
        return new GameConstants(
            Const("GRAVITATIONAL_CONSTANT"),
            Const("SOLAR_MASS"),
            Const("EARTH_MASS"),
            Const("LUNAR_MASS"),
            Const("JUPITER_MASS"));
    }

    public MassConstants ToMassConstants() => new()
    {
        G = G,
        SolarMassKg = SolarMassKg,
        EarthMassKg = EarthMassKg,
        LunarMassKg = LunarMassKg,
        JupiterMassKg = JupiterMassKg,
    };
}

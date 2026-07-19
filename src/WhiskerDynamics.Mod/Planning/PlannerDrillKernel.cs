using System.Globalization;

namespace WhiskerDynamics.Mod.Planning;

/// <summary>One validated planner-drill command from whiskerdynamics.toml. Kept free
/// of KSA/Brutal types so malformed verification scaffolding is rejected offline,
/// before the main-thread adapter can inspect or mutate a stock flight plan.</summary>
internal readonly record struct PlannerDrillCommand(
    double OffsetSeconds, double Prograde, double Normal, double Outward);

/// <summary>Strict parser for drill_planner_burn's documented
/// "offsetSeconds,prograde,normal,outward" wire format. TOML uses a comma as the
/// component separator, so numbers are always invariant-culture and decimal commas
/// are deliberately not accepted.</summary>
internal static class PlannerDrillKernel
{
    internal const double MinOffsetSeconds = PlannerKernel.MinLeadSeconds;
    internal const double MaxOffsetSeconds =
        ModConfig.MaxWorkloadDays * ModConfig.SecondsPerDay;

    private static readonly string[] ComponentNames =
        ["offsetSeconds", "prograde", "normal", "outward"];

    internal static bool TryParse(string? raw, out PlannerDrillCommand command,
        out string error)
    {
        command = default;
        string[] parts = raw?.Split(',', StringSplitOptions.None) ?? [];
        if (parts.Length != ComponentNames.Length)
        {
            error = "expected exactly 4 comma-separated values "
                + "(offsetSeconds,prograde,normal,outward); got " + parts.Length;
            return false;
        }

        Span<double> values = stackalloc double[ComponentNames.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            if (!double.TryParse(parts[i], NumberStyles.Float,
                    CultureInfo.InvariantCulture, out double value))
            {
                error = $"invalid {ComponentNames[i]} '{parts[i].Trim()}': "
                    + "expected an invariant-culture number";
                return false;
            }
            if (!double.IsFinite(value))
            {
                error = $"invalid {ComponentNames[i]} '{parts[i].Trim()}': "
                    + "value must be finite";
                return false;
            }
            values[i] = value;
        }

        if (values[0] < MinOffsetSeconds || values[0] > MaxOffsetSeconds)
        {
            error = $"invalid offsetSeconds '{parts[0].Trim()}': must be within ["
                + MinOffsetSeconds.ToString("R", CultureInfo.InvariantCulture) + ", "
                + MaxOffsetSeconds.ToString("R", CultureInfo.InvariantCulture)
                + "] seconds in the future";
            return false;
        }
        if (!PlannerKernel.ValidateDv(values[1], values[2], values[3]))
        {
            // Every component was proven finite above, so failure here specifically
            // means stock-compatible x*x+y*y+z*z overflowed.
            error = "invalid delta-v: combined VLF length-squared is not finite; "
                + "use smaller components";
            return false;
        }

        command = new PlannerDrillCommand(values[0], values[1], values[2], values[3]);
        error = "";
        return true;
    }
}

using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Planning;

/// <summary>KSA-free planner rules. VLF component convention (decompiled
/// evidence): Burn.DeltaVVlf.X = prograde, .Y = orbit-normal,
/// .Z = radial-out — the stock editor labels Prograde/Normal/Outward write exactly
/// these components (Burn.cs:718/732/746) and the VLF basis is
/// [velocityDir, angularMomentumDir, radialDir] (StateVectors.GetVlf2GivenFrame,
/// StateVectors.cs:81). Validation mirrors stock ProcessBurnClick where stock validates
/// (duplicate exact time -> rejected, Program.cs:1819-1823) and adds the finite/ahead
/// guards stock leaves to the user. Burn times use the same tolerance-sized identity
/// slots as flight-plan metadata and snapshots.</summary>
public static class PlannerKernel
{
    /// <summary>Minimum lead: a burn closer than this to 'now' is stock's to execute,
    /// not ours to plan (mirrors the overlay's burn-window rule).</summary>
    public const double MinLeadSeconds = 1.0;

    /// <summary>Rebase coast gate, wall ms: a vessel can linger OFF-RAILS long after
    /// its engine stops (SAS actuators or an armed flight computer keep it in live
    /// physics), so "back on rails" is the wrong availability signal — "no live
    /// delta-v witnessed for this long" is the honest one. During thrust the witness
    /// stamps every tick, so the gate stays closed exactly while the trajectory is
    /// actively changing.</summary>
    public const long RebaseCoastGraceMs = 2000;

    public enum Verdict { Ok, NotFinite, NotAhead, DuplicateTime, NoPatch }

    public static double3 ComposeVlf(double prograde, double normal, double outward) =>
        new(prograde, normal, outward);

    public static (double Prograde, double Normal, double Outward) DecomposeVlf(double3 dvVlf) =>
        (dvVlf.X, dvVlf.Y, dvVlf.Z);

    public static Verdict ValidateAdd(double burnTime, double now,
        IReadOnlyList<double> existingBurnTimes, bool patchFound)
    {
        Verdict timing = ValidateAddTiming(burnTime, now, existingBurnTimes);
        if (timing != Verdict.Ok) return timing;
        return patchFound ? Verdict.Ok : Verdict.NoPatch;
    }

    /// <summary>Admission checks that do not require constructing a stock SimTime or
    /// asking the stock plan for a patch. TryAdd runs these first so NaN/infinite,
    /// too-near, and occupied time slots never enter stock lookup code.</summary>
    public static Verdict ValidateAddTiming(double burnTime, double now,
        IReadOnlyList<double> existingBurnTimes)
    {
        if (!double.IsFinite(burnTime)) return Verdict.NotFinite;
        if (burnTime < now + MinLeadSeconds) return Verdict.NotAhead;
        for (int i = 0; i < existingBurnTimes.Count; i++)
            if (BurnIdentityPolicy.SameBurn(existingBurnTimes[i], burnTime))
                return Verdict.DuplicateTime;
        return Verdict.Ok;
    }

    /// <summary>Time-edit rules: same ahead/duplicate guards, against the OTHER burns'
    /// times (the edited burn's own slot is legitimately being vacated).</summary>
    public static Verdict ValidateTimeEdit(double newTime, double now,
        IReadOnlyList<double> otherBurnTimes)
    {
        if (!double.IsFinite(newTime)) return Verdict.NotFinite;
        if (newTime < now + MinLeadSeconds) return Verdict.NotAhead;
        for (int i = 0; i < otherBurnTimes.Count; i++)
            if (BurnIdentityPolicy.SameBurn(otherBurnTimes[i], newTime))
                return Verdict.DuplicateTime;
        return Verdict.Ok;
    }

    /// <summary>Stock-compatible VLF vector admission. Individual finite components
    /// are insufficient: Brutal's length-squared path is x*x+y*y+z*z, so very large
    /// finite components can still overflow the magnitude used throughout burn
    /// planning. Require that exact combined scalar to remain finite too.</summary>
    public static bool ValidateDv(double prograde, double normal, double outward) =>
        double.IsFinite(prograde) && double.IsFinite(normal) && double.IsFinite(outward)
        && double.IsFinite(prograde * prograde + normal * normal + outward * outward);

    /// <summary>Whether the authored plan exceeds the vessel's stock vacuum delta-v
    /// readout. Non-finite inputs are not actionable budgets and therefore never
    /// trigger the warning colour.</summary>
    public static bool IsDeltaVOverBudget(double planTotal, double vesselAvailable) =>
        double.IsFinite(planTotal) && planTotal >= 0
        && double.IsFinite(vesselAvailable) && vesselAvailable >= 0
        && planTotal > vesselAvailable;

    /// <summary>The parent whose VLF basis a burn EXECUTES in (decompiled
    /// evidence): stock resolves the burn's PATCH at the burn time —
    /// BurnPlan.TryGetValidTimeLinePatch / FlightPlan.TryFindPatch, the resolution
    /// BurnPlanWriter.TryAdd mirrors (BurnPlanWriter.cs:88-89) — and transforms
    /// DeltaVVlf by that patch orbit's GetVlf2ParentCci at execution
    /// (BurnTarget.UpdateFromBurn, BurnTarget.cs:47-49) and for plan chaining
    /// (FlightPlan.CalculateBurnPatch, FlightPlan.cs:267-273). So a cross-SOI burn
    /// belongs to the POST-transition patch's parent, never the vessel's panel-time
    /// orbit parent; the current parent is only the fallback when no patch resolves
    /// (execution itself needs a patch, so stock has no better answer either).</summary>
    public static string BurnBasisParent(string? patchParentId, string currentParentId) =>
        patchParentId ?? currentParentId;

    public static string Describe(Verdict verdict) => verdict switch
    {
        Verdict.Ok => "ok",
        Verdict.NotFinite => "rejected: value is not finite",
        Verdict.NotAhead => "rejected: burn time is not ahead of now",
        Verdict.DuplicateTime => "rejected: a burn already occupies this time slot",
        Verdict.NoPatch => "rejected: no flight-plan patch covers this time",
        _ => "rejected: unknown verdict",
    };
}

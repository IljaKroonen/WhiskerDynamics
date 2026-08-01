using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

internal static class BurnPlanCalculationContext
{
    [ThreadStatic] private static int _depth;
    [ThreadStatic] private static SimTime _currentTime;

    /// <summary>Wall-clock throttle so a persistent game-read fault (one contained
    /// error per recalculation per vessel) cannot flood the log.</summary>
    private static long _nextEnterErrorMs;

    internal static bool Active => _depth > 0;

    /// <summary>The "now" for the completed-burn gate. Vessel scopes use the
    /// committed state epoch, not the elapsed clock: at high warp the elapsed clock
    /// leads by up to a tick, which would let stock delete a preserved node while the
    /// vessel is still warping toward ignition.</summary>
    internal static SimTime CurrentTime => _currentTime;

    internal readonly record struct Scope(bool Entered, SimTime PreviousCurrentTime)
        : IDisposable
    {
        public void Dispose() => Exit(this);
    }

    /// <summary>Preservation applies only while the mod owns the vessel's trajectory
    /// forecast (mirrors the SOI plan-authority classification); otherwise stock
    /// keeps full conic authority. Never throws — a contained failure degrades to
    /// stock behavior.</summary>
    internal static Scope EnterForVehicle(Vehicle? vehicle)
    {
        try
        {
            if (vehicle is null
                || !ModServices.Enabled
                || !ModServices.TryGetBound(out var services)
                || vehicle.Props.Situation != Situation.Freefall)
                return default;
            var tracked = services.Vessels.TryGetTracked(vehicle.Id);
            if (tracked is not null && !tracked.IsSameVehicle(vehicle))
                return default;
            return Enter(vehicle.Orbit?.StateVectors.StateTime
                ?? Universe.GetElapsedSimTime());
        }
        catch (Exception e)
        {
            return Contain("classification", e);
        }
    }

    /// <summary>Resolves the vehicle from its flight plan the same way stock does
    /// inside the patched method (Burn.cs:1717/1727).</summary>
    internal static Scope EnterForVehiclePlan(FlightPlan vehicleFlightPlan)
    {
        try
        {
            return EnterForVehicle(
                Universe.CurrentSystem?.All.Get(vehicleFlightPlan.IdHash) as Vehicle);
        }
        catch (Exception e)
        {
            return Contain("vehicle resolution", e);
        }
    }

    private static Scope Contain(string what, Exception e)
    {
        if (Environment.TickCount64 >= _nextEnterErrorMs)
        {
            _nextEnterErrorMs = Environment.TickCount64 + 5000;
            ModLog.Error($"burn preservation {what} contained: {e}");
        }
        return default;
    }

    /// <summary>Save restore rebuilds burns before the mod can bind (the vessel
    /// registry is still empty), so this gate is Enabled alone. KSA restores elapsed
    /// time before burn plans deserialize, so the elapsed clock is correct here.</summary>
    internal static Scope EnterForDeserialize() =>
        ModServices.Enabled ? Enter(Universe.GetElapsedSimTime()) : default;

    private static Scope Enter(SimTime currentTime)
    {
        var scope = new Scope(true, _currentTime);
        _depth++;
        _currentTime = currentTime;
        return scope;
    }

    internal static void Exit(Scope scope)
    {
        if (!scope.Entered) return;
        _depth--;
        _currentTime = scope.PreviousCurrentTime;
    }

    /// <summary>Extend when the probed plan ends in an impact before the requested
    /// burn time and the burn is still in the future. The currentTime gate rejects
    /// completed-burn validity probes (Burn.cs:1739): answering those would keep
    /// spent nodes alive past stock's cleanup.</summary>
    internal static bool ShouldExtendPastImpact(
        PatchTransition lastTransition,
        SimTime lastEndTime,
        SimTime requestedTime,
        SimTime currentTime) =>
        lastTransition == PatchTransition.Impact
        && requestedTime > lastEndTime
        && requestedTime > currentTime;
}

[HarmonyPatch(typeof(BurnPlan),
    nameof(BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly))]
internal static class BurnPlanCalculationScopePatch
{
    static void Prefix(FlightPlan vehicleFlightPlan,
        out BurnPlanCalculationContext.Scope __state) =>
        __state = BurnPlanCalculationContext.EnterForVehiclePlan(vehicleFlightPlan);

    static Exception? Finalizer(Exception? __exception,
        BurnPlanCalculationContext.Scope __state)
    {
        BurnPlanCalculationContext.Exit(__state);
        return __exception;
    }
}

/// <summary>Save restore drops a burn whose patch probe fails (Burn.cs:1324), so a
/// node preserved in-session past a stock impact must also be preserved here.</summary>
[HarmonyPatch(typeof(BurnPlan), nameof(BurnPlan.DeserializeSave))]
internal static class BurnPlanDeserializeScopePatch
{
    static void Prefix(out BurnPlanCalculationContext.Scope __state) =>
        __state = BurnPlanCalculationContext.EnterForDeserialize();

    static Exception? Finalizer(Exception? __exception,
        BurnPlanCalculationContext.Scope __state)
    {
        BurnPlanCalculationContext.Exit(__state);
        return __exception;
    }
}

[HarmonyPatch(typeof(FlightPlan), nameof(FlightPlan.TryFindPatch))]
internal static class BurnImpactPreservationPatch
{
    static void Postfix(FlightPlan __instance, SimTime time,
        ref PatchedConic? __result)
    {
        if (__result is not null || !BurnPlanCalculationContext.Active) return;
        if (__instance.Patches.Count == 0) return;
        PatchedConic last = __instance.Patches[^1];
        if (!BurnPlanCalculationContext.ShouldExtendPastImpact(
                last.EndTransition, last.EndTime, time,
                BurnPlanCalculationContext.CurrentTime))
            return;

        // Extend only the copy for burn calculation. Keep the active impact unchanged.
        __result = new PatchedConic(last) { EndTime = time };
    }
}

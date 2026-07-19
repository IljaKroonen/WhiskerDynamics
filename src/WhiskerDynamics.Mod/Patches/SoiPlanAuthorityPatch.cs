using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

internal static class SoiPlanAuthorityContext
{
    [ThreadStatic] private static int _depth;

    internal static bool Active => _depth > 0;

    internal readonly record struct Scope(bool Entered, bool RunOriginal);

    internal static Scope Begin(VehicleUpdateState state)
    {
        if (!ModServices.Enabled) return new(false, true);
        ModServices.BoundServices services = default;
        bool bindingCaptured = false;
        try
        {
            if (!ModServices.TryGetBound(out services))
            {
                if (ModServices.Status == ModStatus.DisabledFault) return new(false, false);
                if (ModServices.Status == ModStatus.Active)
                {
                    ModServices.FatalDisable(
                        $"SOI scheduler authority unavailable for '{state.Id}'");
                    return new(false, false);
                }
                return new(false, true);
            }
            bindingCaptured = true;
            var rails = services.Rails;
            var tracked = services.Vessels.TryGetTracked(state.Id);
            bool sameVehicle = tracked?.IsSameVehicle(state.ReadOnlyVehicle) ?? false;
            bool committedFreefall =
                state.ReadOnlyVehicle.Props.Situation == Situation.Freefall;
            bool parentModeled = state.CurrentOrbit.Parent is Astronomical parent
                && rails.IsModeled(parent.Id);
            var disposition = SoiPlanAuthorityKernel.Classify(
                enabled: ModServices.Enabled,
                bindingAvailable: true,
                parentModeled,
                committedFreefall,
                tracked: tracked is not null,
                sameVehicle);
            switch (disposition)
            {
                case SoiPlanAuthorityKernel.Disposition.SuppressStockScheduler:
                    _depth++;
                    return new(true, true);
                case SoiPlanAuthorityKernel.Disposition.LivePhysicsMirror:
                case SoiPlanAuthorityKernel.Disposition.Inactive:
                    return new(false, true);
                case SoiPlanAuthorityKernel.Disposition.FatalUnknownParent:
                    ModServices.RunIfBindingCurrent(services,
                        () => ModServices.FatalDisable(
                            $"SOI scheduler found an unmodeled parent for '{state.Id}'"));
                    return new(false, false);
                case SoiPlanAuthorityKernel.Disposition.FatalVehicleIdentity:
                    ModServices.RunIfBindingCurrent(services,
                        () => ModServices.FatalDisable(
                            $"SOI scheduler found a contradictory tracked identity for '{state.Id}'"));
                    return new(false, false);
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }
        catch (Exception e)
        {
            if (bindingCaptured)
                ModServices.RunIfBindingCurrent(services,
                    () => ModServices.FatalDisable(
                        $"SOI scheduler authority discovery failed for '{state.Id}': {e}"));
            else if (ModServices.Status == ModStatus.Active)
                ModServices.FatalDisable(
                    $"SOI scheduler authority discovery failed before binding capture: {e}");
            return new(false, false);
        }
    }

    internal static void End(Scope scope)
    {
        if (scope.Entered) _depth--;
    }
}

[HarmonyPatch(typeof(PatchedConic), nameof(PatchedConic.CheckUpdateEncounter))]
internal static class SoiEncounterPlanAuthorityPatch
{
    private static int _pathLogged;

    internal static void ResetSessionStatics() =>
        System.Threading.Volatile.Write(ref _pathLogged, 0);

    static bool Prefix(PatchedConic __instance, ref SimTime expiryGameTime, ref bool __result)
    {
        if (!SoiPlanAuthorityContext.Active) return true;
        // Keep a finite verification horizon so an unexpected global disable cannot
        // strand this transition-free plan forever.
        expiryGameTime = __instance.StartTime
            + Math.Max(10.0, ModServices.Config.OsculationRefreshSeconds);
        __result = false;
        if (System.Threading.Interlocked.CompareExchange(ref _pathLogged, 1, 0) == 0)
            ModLog.Info("SOI authority active: stock Kepler encounter patches suppressed");
        return false;
    }

    internal static bool HasParentTransition(FlightPlan plan)
    {
        for (int i = 1; i < plan.Patches.Count; i++)
            if (!ReferenceEquals(plan.Patches[i - 1].Orbit.Parent,
                    plan.Patches[i].Orbit.Parent))
                return true;
        return false;
    }
}

[HarmonyPatch(typeof(FlightPlan), "CalculateEscapePatch")]
internal static class SoiEscapePlanAuthorityPatch
{
    static bool Prefix(PatchedConic patch, ref PatchedConic nextPatch, ref bool error,
        ref bool __result)
    {
        if (!SoiPlanAuthorityContext.Active) return true;
        nextPatch = null!;
        error = false;
        __result = false;
        return false;
    }
}

[HarmonyPatch(typeof(VehicleUpdateState), nameof(VehicleUpdateState.RecalculateFlightPlan))]
internal static class SoiRecalculateFlightPlanScopePatch
{
    static bool Prefix(VehicleUpdateState __instance,
        out SoiPlanAuthorityContext.Scope __state)
    {
        __state = SoiPlanAuthorityContext.Begin(__instance);
        return __state.RunOriginal;
    }

    static Exception? Finalizer(
        Exception? __exception, SoiPlanAuthorityContext.Scope __state)
    {
        SoiPlanAuthorityContext.End(__state);
        return __exception;
    }
}

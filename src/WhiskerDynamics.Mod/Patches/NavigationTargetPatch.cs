using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;
using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Patches;

[HarmonyPatch(typeof(NavigationTarget), nameof(NavigationTarget.Create))]
internal static class NavigationTargetPatch
{
    private static int _pathLogged;
    private static long _nextErrorLogMs;

    internal static void ResetSessionStatics()
    {
        System.Threading.Volatile.Write(ref _pathLogged, 0);
        System.Threading.Interlocked.Exchange(ref _nextErrorLogMs, 0);
    }

    static void Postfix(IOrbiter? target, Part? targetPart, IParentBody myParent, UniverseTime time,
        ref NavigationTarget? __result)
    {
        if (!__result.HasValue || target is null || !ModServices.Enabled) return;
        try
        {
            if (!ModServices.EnsureBound(out var services)) return;
            double t = time.Seconds();
            if (!TryCorrect(services.Vessels, services.Rails, target, myParent,
                    t, __result.Value, out var corrected,
                    preservePartOffset: targetPart is not null)) return;
            __result = corrected;

            if (System.Threading.Interlocked.CompareExchange(ref _pathLogged, 1, 0) == 0)
                ModLog.Info($"navigation target re-anchored to rails: '{target.Id}' relative to '{myParent.Id}' at t={t:F1} s");
        }
        catch (Exception e)
        {
            long now = Environment.TickCount64;
            long next = System.Threading.Interlocked.Read(ref _nextErrorLogMs);
            if (now >= next
                && System.Threading.Interlocked.CompareExchange(ref _nextErrorLogMs, now + 30_000, next) == next)
                ModLog.Warn($"navigation target rails correction skipped: {e.Message}");
        }
    }

    private static double3 StockPositionCci(
        IOrbiter target, IParentBody myParent, UniverseTime time)
    {
        StateVectors state = target.Orbit.GetStateVectorsAt(time);
        if (target.Parent == myParent) return state.PositionCci;
        double3 targetParentPosition = target.Parent is IOrbiter targetParent
            ? targetParent.GetPositionEcl(time)
            : target.Parent.GetPositionEcl();
        double3 myParentPosition = myParent is IOrbiter parent
            ? parent.GetPositionEcl(time)
            : myParent.GetPositionEcl();
        double3 targetPosition = targetParentPosition
            + state.PositionCci.Transform(target.Parent.GetCci2Cce());
        return (targetPosition - myParentPosition).Transform(myParent.GetCce2Cci());
    }

    internal static bool TryCorrect(
        VesselRegistry vessels, RailsService rails, IOrbiter target,
        IParentBody myParent, double time, in NavigationTarget original,
        out NavigationTarget corrected, bool preservePartOffset = false)
    {
        corrected = original;
        if (!double.IsFinite(time)
            || myParent is not Astronomical parent
            || !rails.CanEvaluate(parent.Id))
            return false;

        StateVector targetAbsolute;
        StateVector parentAbsolute;
        if (target is Vehicle vehicle)
        {
            if (!vessels.TryReadAuthoritativePredictorState(
                    vehicle, time, out targetAbsolute, out _)
                && !TryCurrentVehicleAbsolute(rails, vehicle, time, out targetAbsolute))
                return false;
            parentAbsolute = rails.GetAbsolute(parent.Id, time);
        }
        else
        {
            if (!rails.CanEvaluate(target.Id)) return false;
            (targetAbsolute, parentAbsolute) =
                rails.GetAbsolutePair(target.Id, parent.Id, time);
        }

        StateVector relative = NavigationTargetKernel.RelativeToParent(
            in targetAbsolute, in parentAbsolute);
        doubleQuat cce2Cci = myParent.GetCce2Cci();
        corrected.PositionCci = FrameAdapter.EclToCci(relative.Position, cce2Cci);
        if (preservePartOffset)
            corrected.PositionCci += original.PositionCci
                - StockPositionCci(target, myParent, new UniverseTime(time));
        corrected.VelocityCci = FrameAdapter.EclToCci(relative.Velocity, cce2Cci);
        return true;
    }

    private static bool TryCurrentVehicleAbsolute(RailsService rails, Vehicle vehicle,
        double requestedTime, out StateVector absolute)
    {
        absolute = default;
        Orbit orbit = vehicle.Orbit;
        if (orbit.Parent is not Astronomical targetParent
            || !rails.CanEvaluate(targetParent.Id))
            return false;
        ref readonly StateVectors state = ref orbit.StateVectors;
        double stateTime = state.StateTime.Seconds();
        double dt = requestedTime - stateTime;
        if (!double.IsFinite(stateTime) || !double.IsFinite(dt)) return false;
        doubleQuat cci2Cce = orbit.Parent.GetCci2Cce();
        var relativeAtStateTime = new StateVector(
            FrameAdapter.CciToEcl(state.PositionCci, cci2Cce),
            FrameAdapter.CciToEcl(state.VelocityCci, cci2Cce));
        if (!NavigationTargetKernel.TryBoundedLinearStateAt(
                in relativeAtStateTime, dt, 1.0, out StateVector relative))
            return false;
        StateVector parent = rails.GetAbsolute(targetParent.Id, requestedTime);
        absolute = new StateVector(parent.Position + relative.Position,
            parent.Velocity + relative.Velocity);
        return NavigationTargetKernel.IsFinite(in absolute);
    }
}

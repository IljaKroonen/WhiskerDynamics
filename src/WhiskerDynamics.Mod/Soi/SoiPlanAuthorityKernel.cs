namespace WhiskerDynamics.Mod.Soi;

public static class SoiPlanAuthorityKernel
{
    public enum Disposition
    {
        Inactive,
        LivePhysicsMirror,
        SuppressStockScheduler,
        FatalUnknownParent,
        FatalVehicleIdentity,
    }

    /// <summary>Stock conics may mirror live-physics truth, but they never schedule
    /// transitions for a modeled Freefall vessel. First-seen and clustered vessels
    /// need no predictor token to suppress encounter/escape generation.</summary>
    public static Disposition Classify(
        bool enabled,
        bool bindingAvailable,
        bool parentModeled,
        bool committedFreefall,
        bool tracked,
        bool sameVehicle)
    {
        if (!enabled || !bindingAvailable) return Disposition.Inactive;
        if (!parentModeled) return Disposition.FatalUnknownParent;
        if (tracked && !sameVehicle) return Disposition.FatalVehicleIdentity;
        return committedFreefall
            ? Disposition.SuppressStockScheduler
            : Disposition.LivePhysicsMirror;
    }
}

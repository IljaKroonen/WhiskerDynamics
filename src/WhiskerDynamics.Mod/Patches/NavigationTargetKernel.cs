using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Pure geometry behind the stock navigation-target correction.</summary>
public static class NavigationTargetKernel
{
    public static StateVector RelativeToParent(in StateVector targetAbsolute,
        in StateVector parentAbsolute) => new(
            targetAbsolute.Position - parentAbsolute.Position,
            targetAbsolute.Velocity - parentAbsolute.Velocity);

    public static StateVector LinearStateAt(in StateVector state, double deltaSeconds) =>
        new(state.Position + state.Velocity * deltaSeconds, state.Velocity);

    /// <summary>Advance a committed live state only across a small physics-step gap.
    /// Across warp-sized gaps, retain its last parent-relative position: that is safer
    /// for surface vessels than inventing a long inertial tangent trajectory.</summary>
    public static bool TryBoundedLinearStateAt(in StateVector state,
        double deltaSeconds, double maxLinearSeconds, out StateVector result)
    {
        result = default;
        if (!IsFinite(in state) || !double.IsFinite(deltaSeconds)
            || !double.IsFinite(maxLinearSeconds) || maxLinearSeconds < 0)
            return false;
        result = Math.Abs(deltaSeconds) <= maxLinearSeconds
            ? LinearStateAt(in state, deltaSeconds)
            : state;
        return IsFinite(in result);
    }

    public static bool IsFinite(in StateVector state) =>
        double.IsFinite(state.Position.X) && double.IsFinite(state.Position.Y)
        && double.IsFinite(state.Position.Z) && double.IsFinite(state.Velocity.X)
        && double.IsFinite(state.Velocity.Y) && double.IsFinite(state.Velocity.Z);
}

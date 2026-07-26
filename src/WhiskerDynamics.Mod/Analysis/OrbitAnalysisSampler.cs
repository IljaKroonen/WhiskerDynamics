using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Analysis;

internal sealed record OrbitAnalysisSeries(
    double[] Times,
    Vector3d[] Positions,
    Vector3d[] Velocities,
    bool Truncated,
    bool WorkLimited,
    bool DynamicsLimited);

internal static class OrbitAnalysisSampler
{
    internal const int MaximumPoints = int.MaxValue;
    internal const double MaximumTurnRadians = Math.PI / 36;
    internal const int ProductionTargetPoints = 500_000;
    internal const int ProductionMaximumPoints = 1_000_000;
    internal const double ProductionMaximumTurnRadians = Math.PI / 4;

    /// <summary>Long analysis intervals cannot retain five-degree samples for every
    /// revolution without eventually consuming gigabytes. Relax only the geometric
    /// turn target needed to stay near the production point target; the adaptive
    /// sampler's eight-samples-per-period anti-aliasing floor remains in force, and
    /// the hard point cap remains a last-resort honest truncation.</summary>
    internal static double ProductionTurnRadians(
        double startTimeSeconds, double endTimeSeconds, double periodHintSeconds,
        int targetPoints = ProductionTargetPoints)
    {
        if (targetPoints < 2
            || !double.IsFinite(startTimeSeconds)
            || !double.IsFinite(endTimeSeconds)
            || endTimeSeconds <= startTimeSeconds
            || !double.IsFinite(periodHintSeconds)
            || periodHintSeconds <= 0)
            return MaximumTurnRadians;

        double revolutions = (endTimeSeconds - startTimeSeconds) / periodHintSeconds;
        // AdaptiveSampler grows only below half the turn bound, so 4*pi rather than
        // 2*pi is the conservative points-per-revolution estimate.
        double targetTurn = 4 * Math.PI * revolutions / targetPoints;
        return Math.Clamp(
            Math.Max(MaximumTurnRadians, targetTurn),
            MaximumTurnRadians, ProductionMaximumTurnRadians);
    }

    internal static OrbitAnalysisSeries Sample(
        Func<double, (Vector3d Position, Vector3d Velocity)> relativeState,
        double startTimeSeconds, double endTimeSeconds, double periodHintSeconds,
        Func<bool>? shouldStop = null, int maximumPoints = MaximumPoints,
        Action<double>? progress = null,
        double maximumTurnRadians = MaximumTurnRadians,
        Action<double>? acceptedTime = null)
    {
        (Vector3d Position, Vector3d Velocity) lastState = default;
        Vector3d PositionAt(double time)
        {
            lastState = relativeState(time);
            return lastState.Position;
        }

        var velocities = new List<Vector3d>();
        void Accepted(double time)
        {
            velocities.Add(lastState.Velocity);
            acceptedTime?.Invoke(time);
            progress?.Invoke((time - startTimeSeconds)
                / (endTimeSeconds - startTimeSeconds));
        }

        var sampled = AdaptiveSampler.Sample(
            PositionAt, startTimeSeconds, endTimeSeconds, maximumPoints,
            maximumTurnRadians, dtMinSeconds: 1, periodHintSeconds, shouldStop,
            Accepted);
        return new(sampled.Times, sampled.Positions, [.. velocities],
            sampled.Truncated, sampled.WorkLimited, sampled.DynamicsLimited);
    }
}

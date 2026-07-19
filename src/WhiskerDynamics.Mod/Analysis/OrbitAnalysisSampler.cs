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

    internal static OrbitAnalysisSeries Sample(
        Func<double, (Vector3d Position, Vector3d Velocity)> relativeState,
        double startTimeSeconds, double endTimeSeconds, double periodHintSeconds,
        Func<bool>? shouldStop = null, int maximumPoints = MaximumPoints,
        Action<double>? progress = null)
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
            progress?.Invoke((time - startTimeSeconds)
                / (endTimeSeconds - startTimeSeconds));
        }

        var sampled = AdaptiveSampler.Sample(
            PositionAt, startTimeSeconds, endTimeSeconds, maximumPoints,
            MaximumTurnRadians, dtMinSeconds: 1, periodHintSeconds, shouldStop,
            Accepted);
        return new(sampled.Times, sampled.Positions, [.. velocities],
            sampled.Truncated, sampled.WorkLimited, sampled.DynamicsLimited);
    }
}

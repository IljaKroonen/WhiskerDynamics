namespace WhiskerDynamics.Mod.Planning;

/// <summary>The single logical identity rule for stock burns and their time-keyed
/// metadata, snapshots, admission checks, and fold guards.</summary>
public static class BurnIdentityPolicy
{
    public const double ToleranceSeconds = 1e-3;

    public static bool TryMatch(double candidateSeconds, double requestedSeconds,
        out double distanceSeconds)
    {
        distanceSeconds = Math.Abs(candidateSeconds - requestedSeconds);
        return distanceSeconds <= ToleranceSeconds;
    }

    public static bool SameBurn(double firstSeconds, double secondSeconds) =>
        TryMatch(firstSeconds, secondSeconds, out _);

    public static bool DifferentBurn(double firstSeconds, double secondSeconds) =>
        !SameBurn(firstSeconds, secondSeconds);

    public static bool ContainsBurn(
        IEnumerable<double> burnTimes, double timeSeconds)
    {
        foreach (double candidate in burnTimes)
            if (SameBurn(candidate, timeSeconds)) return true;
        return false;
    }
}

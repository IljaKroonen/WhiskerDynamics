namespace WhiskerDynamics.Mod.Overlay;

/// <summary>KSA-free policy for the pause-only overlay producer.</summary>
internal static class PausedOverlayRefreshKernel
{
    internal const long KeepAlivePeriodMs = 1000;

    internal static bool ShouldRefresh(double simulationSpeed, long wallMs,
        long lastRefreshMs, long editStamp, long lastEditStamp,
        string? frameLabel, string? lastFrameLabel) =>
        simulationSpeed == 0.0
        && (wallMs - lastRefreshMs >= KeepAlivePeriodMs
            || editStamp != lastEditStamp
            || !string.Equals(frameLabel, lastFrameLabel, StringComparison.Ordinal));

    internal static bool AccumulateDeferral(bool alreadyDeferred, bool editDeferred) =>
        alreadyDeferred || editDeferred;
}

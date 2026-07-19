namespace WhiskerDynamics.Mod.Runtime;

internal sealed class MapTrajectoryState
{
    internal const double DefaultHistoryDisplayDays = 30.0;

    internal double HistoryDisplayDays { get; set; } = DefaultHistoryDisplayDays;
    internal bool ShowAstralBodyLines { get; set; } = true;
}

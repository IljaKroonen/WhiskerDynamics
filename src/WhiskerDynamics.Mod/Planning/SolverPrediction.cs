using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning;

/// <summary>One background solver's detached prediction view. The immutable body
/// snapshot may be shared, but the context's gravity cache and every predictor passed
/// here belong exclusively to this solver thread. No method touches RailsService.Gate.</summary>
internal sealed class SolverPrediction
{
    private readonly RailsService.PredictionContext _context;
    private readonly Func<bool> _cancelled;

    internal SolverPrediction(RailsService.PredictionContext context, Func<bool> cancelled)
    {
        _context = context;
        _cancelled = cancelled;
    }

    internal GravityModel Gravity => _context.Gravity;

    internal StateVector GetAbsolute(string bodyId, double time)
    {
        ThrowIfCancelled();
        return _context.GetAbsolute(bodyId, time);
    }

    internal double GetAbsolutePositionSegmentEndAfter(string bodyId, double time)
    {
        ThrowIfCancelled();
        return _context.GetAbsolutePositionSegmentEndAfter(bodyId, time);
    }

    /// <summary>Extends a caller-owned predictor in the caller-selected historical
    /// chunk cadence. Chunking is retained for cancellation latency and numerical
    /// compatibility; cooperative yields keep the below-normal worker courteous.</summary>
    internal StateVector StateAt(
        TrajectoryPredictor predictor, double time, double chunkSeconds)
    {
        if (!(chunkSeconds > 0) || !double.IsFinite(chunkSeconds))
            throw new ArgumentOutOfRangeException(nameof(chunkSeconds));
        while (predictor.Horizon < time)
        {
            ThrowIfCancelled();
            predictor.ExtendTo(Math.Min(time, predictor.Horizon + chunkSeconds));
            Thread.Yield();
        }
        ThrowIfCancelled();
        return predictor.StateAt(time);
    }

    internal (Vector3d RRel, Vector3d VRel) RelativeState(
        TrajectoryPredictor predictor, string parentId, double time, double chunkSeconds)
    {
        var absolute = StateAt(predictor, time, chunkSeconds);
        var parent = GetAbsolute(parentId, time);
        return (absolute.Position - parent.Position, absolute.Velocity - parent.Velocity);
    }

    private void ThrowIfCancelled()
    {
        if (_cancelled()) throw new OperationCanceledException();
    }
}

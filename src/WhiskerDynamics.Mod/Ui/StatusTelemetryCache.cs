namespace WhiskerDynamics.Mod.Ui;

/// <summary>Wall-clock cadence and last-complete publication for the status panel's
/// informational providers. The caller supplies the clock so cadence behavior stays
/// deterministic in tests and benchmarks. A refresh builds privately and is published
/// only after the callback completes; failures therefore retain the last complete
/// snapshot while still consuming the cadence window.</summary>
internal sealed class StatusTelemetryCache
{
    private readonly object _gate = new();
    private readonly long _refreshIntervalMs;
    private IReadOnlyList<string> _lines = [];
    private long _nextRefreshMs;
    private long _lastObservedMs;
    private long _generation;
    private long _admissionToken;
    private bool _hasObservedClock;

    public StatusTelemetryCache(long refreshIntervalMs)
    {
        if (refreshIntervalMs <= 0)
            throw new ArgumentOutOfRangeException(nameof(refreshIntervalMs));
        _refreshIntervalMs = refreshIntervalMs;
    }

    /// <summary>Returns the same published snapshot reference until the cadence is due.
    /// A wall-clock regression and an explicit <see cref="Reset"/> both make the next
    /// read refresh immediately. The deadline is advanced before invoking
    /// <paramref name="refresh"/>, preserving failure throttling.</summary>
    public IReadOnlyList<string> Read(long nowMs, Func<IReadOnlyList<string>> refresh)
    {
        ArgumentNullException.ThrowIfNull(refresh);
        long generation;
        long admissionToken;
        lock (_gate)
        {
            bool clockRegressed = _hasObservedClock && nowMs < _lastObservedMs;
            bool refreshDue = !_hasObservedClock || clockRegressed || nowMs >= _nextRefreshMs;
            _hasObservedClock = true;
            _lastObservedMs = nowMs;
            if (!refreshDue) return _lines;

            // Consume the cadence before running providers. If one throws, callers keep
            // the previous complete snapshot and the next rendered frame does not retry.
            _nextRefreshMs = NextDeadline(nowMs);
            generation = _generation;
            admissionToken = ++_admissionToken;
        }

        // Providers can trigger EnsureBound, whose session sweep calls Reset while
        // holding ModServices.BindGate. Never invoke arbitrary provider code under the
        // cache gate: doing so would invert BindGate -> cache gate against a concurrent
        // panel refresh and deadlock save-load/rebind.
        IReadOnlyList<string> refreshed = refresh()
            ?? throw new InvalidOperationException("status telemetry refresh returned null");

        lock (_gate)
        {
            // Reset and a later due admission both supersede this private result.
            // Only the latest admitted refresh in the same session may publish.
            if (generation == _generation && admissionToken == _admissionToken)
                _lines = refreshed;
            return _lines;
        }
    }

    /// <summary>Clears every displayed line and makes the next read immediately due.</summary>
    public void Reset()
    {
        lock (_gate)
        {
            _generation++;
            _admissionToken++;
            _lines = [];
            _nextRefreshMs = 0;
            _lastObservedMs = 0;
            _hasObservedClock = false;
        }
    }

    private long NextDeadline(long nowMs) =>
        nowMs > long.MaxValue - _refreshIntervalMs
            ? long.MaxValue
            : nowMs + _refreshIntervalMs;
}

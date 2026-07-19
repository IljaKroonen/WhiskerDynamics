using System.Collections.Concurrent;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Decides which whiskerdynamics.log line an override emits. KSA-free by design so the
/// offline suite covers it: strings and milliseconds in, a line kind out. Thread-safe —
/// the celestial patch calls this from game job threads.
/// <para>First override of a body => <see cref="Line.Epoch"/> (one-shot: the
/// epoch-equality gate polls whiskerdynamics.log for exactly one line per body). Afterwards a
/// wall-clock throttle per body => <see cref="Line.Drift"/> at most once per period
/// (stock-Kepler vs rails divergence, observable over warp without unbounded log growth).</para></summary>
internal sealed class RailsTelemetry(long driftPeriodMs)
{
    internal enum Line { None, Epoch, Drift }

    /// <summary>Per body: earliest wall-clock ms at which the next drift line is due.</summary>
    private readonly ConcurrentDictionary<string, long> _nextDriftDueMs = new();

    internal Line Classify(string bodyId, long nowMs)
    {
        if (_nextDriftDueMs.TryAdd(bodyId, nowMs + driftPeriodMs)) return Line.Epoch;
        if (!_nextDriftDueMs.TryGetValue(bodyId, out long due) || nowMs < due) return Line.None;
        // Exactly one concurrent caller wins the slot; losers stay silent.
        return _nextDriftDueMs.TryUpdate(bodyId, nowMs + driftPeriodMs, due) ? Line.Drift : Line.None;
    }

    /// <summary>Statics sweep: re-arm the per-body one-shots after a rebind /
    /// save load, so each bound sim re-evidences its frame conventions in whiskerdynamics.log.
    /// Thread-safe (concurrent Classify calls race benignly: worst case one extra or
    /// one dropped log line around the reset instant).</summary>
    internal void Reset() => _nextDriftDueMs.Clear();
}

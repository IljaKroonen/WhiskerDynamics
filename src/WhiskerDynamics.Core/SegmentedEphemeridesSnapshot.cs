namespace WhiskerDynamics.Core;

/// <summary>Immutable range copy of an NBodyEphemerides interpolation store. Segment
/// endpoint values are deep-copied value types; later owner growth, commit, pooling,
/// and pruning cannot affect this snapshot.</summary>
public sealed class SegmentedEphemeridesSnapshot : IEphemerides, ISegmentedEphemerides
{
    private readonly Ephemerides _kepler;
    private readonly Dictionary<CelestialBody, int> _integratedIndex;
    private readonly HashSet<CelestialBody> _backbone;
    private readonly NBodyEphemerides.BodySegment[][]? _segments;
    private readonly SegmentedEphemeridesSnapshot[]? _windows;

    internal SegmentedEphemeridesSnapshot(IReadOnlyList<CelestialBody> bodies,
        CelestialBody[] integrated, NBodyEphemerides.BodySegment[][] segments,
        int backboneCount, double startTime, double horizon)
    {
        Bodies = bodies;
        _kepler = new Ephemerides(bodies);
        _integratedIndex = integrated.Select((body, index) => (body, index))
            .ToDictionary(pair => pair.body, pair => pair.index);
        _backbone = new HashSet<CelestialBody>(
            integrated.Take(backboneCount), ReferenceEqualityComparer.Instance);
        _segments = segments;
        StartTime = startTime;
        Horizon = horizon;
    }

    private SegmentedEphemeridesSnapshot(SegmentedEphemeridesSnapshot[] windows)
    {
        var first = windows[0];
        Bodies = first.Bodies;
        _kepler = new Ephemerides(Bodies);
        _integratedIndex = first._integratedIndex;
        _backbone = first._backbone;
        _windows = windows;
        StartTime = first.StartTime;
        Horizon = windows[^1].Horizon;
    }

    /// <summary>Builds one immutable contiguous view over independently captured
    /// windows. Segment arrays stay in their original windows, so joining a large
    /// incrementally prepared range copies only the small window-reference array.</summary>
    public static SegmentedEphemeridesSnapshot Combine(
        IReadOnlyList<SegmentedEphemeridesSnapshot> windows)
    {
        ArgumentNullException.ThrowIfNull(windows);
        if (windows.Count == 0)
            throw new ArgumentException("At least one snapshot window is required.",
                nameof(windows));

        var leaves = new List<SegmentedEphemeridesSnapshot>(windows.Count);
        foreach (var window in windows)
        {
            ArgumentNullException.ThrowIfNull(window);
            if (window._windows is { } nested) leaves.AddRange(nested);
            else leaves.Add(window);
        }

        var first = leaves[0];
        double expectedStart = first.StartTime;
        for (int i = 0; i < leaves.Count; i++)
        {
            var window = leaves[i];
            if (window.StartTime != expectedStart)
                throw new ArgumentException(
                    $"Snapshot windows must be exactly contiguous; window {i} starts "
                    + $"at {window.StartTime:R}, expected {expectedStart:R}.", nameof(windows));
            if (!SameSystem(first, window))
                throw new ArgumentException(
                    "Snapshot windows must describe the same celestial system.", nameof(windows));
            expectedStart = window.Horizon;
        }
        return leaves.Count == 1 ? first : new([.. leaves]);
    }

    private static bool SameSystem(
        SegmentedEphemeridesSnapshot a, SegmentedEphemeridesSnapshot b)
    {
        if (a.Bodies.Count != b.Bodies.Count
            || a._integratedIndex.Count != b._integratedIndex.Count)
            return false;
        for (int i = 0; i < a.Bodies.Count; i++)
            if (!ReferenceEquals(a.Bodies[i], b.Bodies[i])) return false;
        foreach (var pair in a._integratedIndex)
            if (!b._integratedIndex.TryGetValue(pair.Key, out int index)
                || index != pair.Value)
                return false;
        if (!a._backbone.SetEquals(b._backbone)) return false;
        return true;
    }

    public IReadOnlyList<CelestialBody> Bodies { get; }
    public CelestialBody this[string id] => _kepler[id];
    public double StartTime { get; }
    public double Horizon { get; }

    public bool IsBackbone(CelestialBody body) => _backbone.Contains(body);

    /// <inheritdoc />
    public bool FeelsGravityFrom(CelestialBody body, CelestialBody source)
    {
        if (ReferenceEquals(body, source) || source.Mu == 0
            || !_integratedIndex.ContainsKey(body) || !_integratedIndex.ContainsKey(source))
            return false;
        if (_backbone.Contains(source)) return true;
        if (_backbone.Contains(body)) return false;
        for (var ancestor = body.Parent; ancestor is not null && !_backbone.Contains(ancestor);
            ancestor = ancestor.Parent)
            if (ReferenceEquals(ancestor, source)) return true;
        return false;
    }

    public StateVector GetState(CelestialBody body, double time)
    {
        if (!double.IsFinite(time) || time < StartTime || time > Horizon)
            throw new ArgumentOutOfRangeException(nameof(time),
                $"Snapshot covers [{StartTime:R}, {Horizon:R}].");
        if (!_integratedIndex.TryGetValue(body, out int index))
            throw new ArgumentException(
                $"Body '{body.Id}' does not belong to this snapshot.", nameof(body));
        var segment = Resolve(index, time);
        return NBodyEphemerides.SegmentState(in segment, time);
    }

    /// <summary>Returns the end of the exact cubic or quintic position span that
    /// contains times immediately after <paramref name="time"/>. The result is
    /// strictly later unless <paramref name="time"/> is the snapshot horizon.</summary>
    public double PositionSegmentEndAfter(CelestialBody body, double time)
    {
        if (!double.IsFinite(time) || time < StartTime || time > Horizon)
            throw new ArgumentOutOfRangeException(nameof(time));
        if (!_integratedIndex.TryGetValue(body, out int index))
            throw new ArgumentException(
                $"Body '{body.Id}' does not belong to this snapshot.", nameof(body));
        if (time == Horizon) return Horizon;
        if (_windows is not null)
        {
            int lo = 0, hi = _windows.Length;
            while (lo < hi)
            {
                int mid = lo + ((hi - lo) >> 1);
                if (_windows[mid].Horizon <= time) lo = mid + 1;
                else hi = mid;
            }
            return _windows[lo].PositionSegmentEndAfter(body, time);
        }

        var segments = _segments![index];
        int first = 0, end = segments.Length;
        while (first < end)
        {
            int mid = first + ((end - first) >> 1);
            if (segments[mid].T0 + segments[mid].Dt <= time) first = mid + 1;
            else end = mid;
        }
        if (first == segments.Length)
            throw new ArgumentOutOfRangeException(nameof(time),
                "No copied position segment extends beyond this time.");
        return Math.Min(Horizon, segments[first].T0 + segments[first].Dt);
    }

    private NBodyEphemerides.BodySegment Resolve(int bodyIndex, double time)
    {
        if (!double.IsFinite(time) || time < StartTime || time > Horizon)
            throw new ArgumentOutOfRangeException(nameof(time));
        if (_windows is not null) return WindowAt(time).Resolve(bodyIndex, time);
        var segments = _segments![bodyIndex];
        int lo = 0, hi = segments.Length - 1;
        while (lo < hi)
        {
            int mid = (lo + hi) / 2;
            double end = segments[mid].T0 + segments[mid].Dt;
            if (end < time) lo = mid + 1; else hi = mid;
        }
        var result = segments[lo];
        if (time < result.T0 || time > result.T0 + result.Dt)
            throw new ArgumentOutOfRangeException(nameof(time), "No copied segment covers this time.");
        return result;
    }

    private SegmentedEphemeridesSnapshot WindowAt(double time)
    {
        var windows = _windows!;
        int lo = 0, hi = windows.Length - 1;
        while (lo < hi)
        {
            int mid = lo + ((hi - lo) >> 1);
            if (time <= windows[mid].Horizon) hi = mid;
            else lo = mid + 1;
        }
        return windows[lo];
    }

    int ISegmentedEphemerides.CommitStamp => 0;
    int ISegmentedEphemerides.IntegratedIndexOf(CelestialBody body) =>
        _integratedIndex.TryGetValue(body, out int index) ? index : -1;
    NBodyEphemerides.BodySegment ISegmentedEphemerides.ResolveBodySegment(
        int bodyIndex, double time) => Resolve(bodyIndex, time);
    bool ISegmentedEphemerides.InCommittedRegion(int bodyIndex, double time) => true;
    Vector3d ISegmentedEphemerides.BodyPositionAt(int bodyIndex, double time)
    {
        var segment = Resolve(bodyIndex, time);
        return NBodyEphemerides.SegmentPosition(in segment, time);
    }
    bool ISegmentedEphemerides.TryResolveDenseSegment(
        double time, out int hi, out double t0, out double dt)
    {
        hi = 0; t0 = 0; dt = 0;
        return false;
    }
    StateVector ISegmentedEphemerides.DenseNodeState(int nodeIndex, int bodyIndex) =>
        throw new NotSupportedException("Immutable snapshots expose copied body segments.");
}

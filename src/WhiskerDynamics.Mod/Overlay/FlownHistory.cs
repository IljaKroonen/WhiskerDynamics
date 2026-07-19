using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

internal readonly record struct FlownHistorySettings(
    double RetentionSeconds, double InitialToleranceMeters, int MaxPoints)
{
    internal const double DefaultRetentionSeconds =
        ModConfig.MaxWorkloadDays * ModConfig.SecondsPerDay;
    internal const double DefaultInitialToleranceMeters = 10.0;
    internal const int DefaultMaxPoints = 262_144;

    internal static FlownHistorySettings Default => new(
        DefaultRetentionSeconds, DefaultInitialToleranceMeters, DefaultMaxPoints);
}

internal readonly record struct FlownSample(double TimeSeconds, Vector3d AbsolutePosition);

internal readonly record struct FlownHistoryCoverage(
    double RequestedStartSeconds,
    double? OldestRecordedStartSeconds,
    double? OldestRenderedStartSeconds,
    bool RenderBudgetTruncated);

internal readonly record struct FlownHistorySnapshot(
    FlownSample[] Samples, FlownHistoryCoverage Coverage);

internal sealed class FlownHistory
{
    private const int PendingSimplificationPoints = 4096;

    private readonly List<FlownSample> _samples = [];
    private readonly List<FlownSample> _pending = [];
    private readonly object _gate = new();
    private FlownSample[]? _snapshot;
    private double _lastTime = double.NegativeInfinity;
    private double _oldestTime = double.NaN;
    private double _initialToleranceMeters =
        FlownHistorySettings.DefaultInitialToleranceMeters;
    private double _effectiveToleranceMeters =
        FlownHistorySettings.DefaultInitialToleranceMeters;
    private int _count;

    internal int Count => Volatile.Read(ref _count);
    internal double OldestTimeSeconds => Volatile.Read(ref _oldestTime);
    internal double LatestTimeSeconds => Volatile.Read(ref _lastTime);
    internal double EffectiveToleranceMeters =>
        Volatile.Read(ref _effectiveToleranceMeters);

    internal void Configure(double nowSeconds, in FlownHistorySettings settings)
    {
        lock (_gate)
        {
            _initialToleranceMeters = settings.InitialToleranceMeters;
            if (_samples.Count == 0)
            {
                _effectiveToleranceMeters = settings.InitialToleranceMeters;
                return;
            }

            double cutoff = nowSeconds - settings.RetentionSeconds;
            int maxPoints = Math.Max(2, settings.MaxPoints);
            if (_samples[0].TimeSeconds >= cutoff
                && StoredPointCountLocked() <= maxPoints) return;
            FinalizePendingLocked();
            PruneLocked(cutoff);
            EnforceCapLocked(maxPoints);
            ResetPendingLocked();
            ChangedLocked();
        }
    }

    internal bool Wants(double timeSeconds, double nowSeconds,
        in FlownHistorySettings settings)
    {
        if (!double.IsFinite(timeSeconds)) return false;
        if (timeSeconds < nowSeconds - settings.RetentionSeconds || timeSeconds > nowSeconds)
            return false;
        return timeSeconds > Volatile.Read(ref _lastTime);
    }

    internal bool Append(double timeSeconds, Vector3d absolutePosition, double nowSeconds,
        in FlownHistorySettings settings)
    {
        lock (_gate)
        {
            if (!WantsLocked(timeSeconds, nowSeconds, in settings)) return false;
            if (!double.IsFinite(absolutePosition.X)
                || !double.IsFinite(absolutePosition.Y)
                || !double.IsFinite(absolutePosition.Z)) return false;

            var sample = new FlownSample(timeSeconds, absolutePosition);
            if (_samples.Count == 0)
            {
                _initialToleranceMeters = settings.InitialToleranceMeters;
                _effectiveToleranceMeters = settings.InitialToleranceMeters;
                _samples.Add(sample);
                _pending.Add(sample);
            }
            else
            {
                _pending.Add(sample);
            }

            int maxPoints = Math.Max(2, settings.MaxPoints);
            if (_pending.Count >= PendingSimplificationPoints
                || StoredPointCountLocked() > maxPoints)
            {
                FinalizePendingLocked();
                EnforceCapLocked(maxPoints);
                ResetPendingLocked();
            }
            ChangedLocked();
            return true;
        }
    }

    internal FlownHistorySnapshot SnapshotRange(
        double requestedStartSeconds, double endSeconds, int maxPoints)
    {
        FlownSample[] source = Snapshot();
        double? oldestRecorded = source.Length == 0 ? null : source[0].TimeSeconds;
        if (!(endSeconds > requestedStartSeconds))
            return new([], new(requestedStartSeconds, oldestRecorded, null, false));

        int first = LowerBound(source, requestedStartSeconds);
        int end = LowerBound(source, endSeconds);
        bool hasStartBoundary = first > 0 && first < source.Length
            && source[first].TimeSeconds > requestedStartSeconds;
        int available = Math.Max(0, end - first) + (hasStartBoundary ? 1 : 0);
        if (maxPoints <= 0)
            return new([], new(
                requestedStartSeconds, oldestRecorded, null, available > 0));

        int take = Math.Min(maxPoints, available);
        int takeFrom = available - take;
        var result = new FlownSample[take];
        for (int i = 0; i < take; i++)
        {
            int candidate = takeFrom + i;
            if (hasStartBoundary && candidate == 0)
            {
                FlownSample before = source[first - 1];
                FlownSample after = source[first];
                double fraction = (requestedStartSeconds - before.TimeSeconds)
                    / (after.TimeSeconds - before.TimeSeconds);
                result[i] = new FlownSample(requestedStartSeconds,
                    before.AbsolutePosition
                    + (after.AbsolutePosition - before.AbsolutePosition) * fraction);
            }
            else
            {
                int sourceIndex = first + candidate - (hasStartBoundary ? 1 : 0);
                result[i] = source[sourceIndex];
            }
        }
        return new(result, new(
            requestedStartSeconds,
            oldestRecorded,
            take == 0 ? null : result[0].TimeSeconds,
            take < available));
    }

    internal void Clear()
    {
        lock (_gate)
        {
            _samples.Clear();
            _pending.Clear();
            _effectiveToleranceMeters = _initialToleranceMeters;
            ChangedLocked();
        }
    }

    internal static FlownSample[] Simplify(
        IReadOnlyList<FlownSample> source, double toleranceMeters)
    {
        if (source.Count <= 2) return source.ToArray();

        double toleranceSquared = toleranceMeters * toleranceMeters;
        var keep = new bool[source.Count];
        keep[0] = true;
        keep[^1] = true;
        var ranges = new Stack<(int Start, int End)>();
        ranges.Push((0, source.Count - 1));
        while (ranges.Count > 0)
        {
            var (start, end) = ranges.Pop();
            FlownSample a = source[start];
            FlownSample b = source[end];
            double span = b.TimeSeconds - a.TimeSeconds;
            int furthest = -1;
            double maximumErrorSquared = toleranceSquared;
            for (int i = start + 1; i < end; i++)
            {
                double fraction = (source[i].TimeSeconds - a.TimeSeconds) / span;
                Vector3d interpolated = a.AbsolutePosition
                    + (b.AbsolutePosition - a.AbsolutePosition) * fraction;
                double errorSquared =
                    (source[i].AbsolutePosition - interpolated).LengthSquared();
                if (errorSquared <= maximumErrorSquared) continue;
                maximumErrorSquared = errorSquared;
                furthest = i;
            }
            if (furthest < 0) continue;
            keep[furthest] = true;
            ranges.Push((start, furthest));
            ranges.Push((furthest, end));
        }

        var simplified = new List<FlownSample>();
        for (int i = 0; i < source.Count; i++)
            if (keep[i]) simplified.Add(source[i]);
        return [.. simplified];
    }

    private FlownSample[] Snapshot()
    {
        var source = Volatile.Read(ref _snapshot);
        if (source is not null) return source;
        lock (_gate)
        {
            source = _snapshot;
            if (source is not null) return source;
            if (_samples.Count == 0)
            {
                source = [];
            }
            else
            {
                FlownSample[] suffix = Simplify(
                    _pending, _effectiveToleranceMeters);
                source = new FlownSample[_samples.Count + suffix.Length - 1];
                _samples.CopyTo(source, 0);
                if (suffix.Length > 1)
                    Array.Copy(suffix, 1, source, _samples.Count, suffix.Length - 1);
            }
            Volatile.Write(ref _snapshot, source);
            return source;
        }
    }

    private void FinalizePendingLocked()
    {
        if (_pending.Count <= 1) return;
        FlownSample[] suffix = Simplify(_pending, _effectiveToleranceMeters);
        for (int i = 1; i < suffix.Length; i++) _samples.Add(suffix[i]);
    }

    private void ResetPendingLocked()
    {
        _pending.Clear();
        if (_samples.Count > 0) _pending.Add(_samples[^1]);
    }

    private void PruneLocked(double cutoffSeconds)
    {
        int first = LowerBound(_samples, cutoffSeconds);
        if (first == 0) return;
        if (first == _samples.Count)
        {
            _samples.Clear();
            _effectiveToleranceMeters = _initialToleranceMeters;
            return;
        }
        if (_samples[first].TimeSeconds == cutoffSeconds)
        {
            _samples.RemoveRange(0, first);
            return;
        }

        FlownSample before = _samples[first - 1];
        FlownSample after = _samples[first];
        double fraction = (cutoffSeconds - before.TimeSeconds)
            / (after.TimeSeconds - before.TimeSeconds);
        var boundary = new FlownSample(cutoffSeconds,
            before.AbsolutePosition + (after.AbsolutePosition - before.AbsolutePosition) * fraction);
        _samples.RemoveRange(0, first);
        _samples.Insert(0, boundary);
    }

    private void EnforceCapLocked(int maxPoints)
    {
        while (_samples.Count > maxPoints)
        {
            double previousTolerance = _effectiveToleranceMeters;
            double nextTolerance = previousTolerance * 2.0;
            double additionalTolerance = nextTolerance - previousTolerance;
            FlownSample[] simplified = Simplify(_samples, additionalTolerance);
            _samples.Clear();
            _samples.AddRange(simplified);
            _effectiveToleranceMeters = nextTolerance;
        }
    }

    private int StoredPointCountLocked() =>
        _samples.Count + Math.Max(0, _pending.Count - 1);

    private bool WantsLocked(double timeSeconds, double nowSeconds,
        in FlownHistorySettings settings)
    {
        if (!double.IsFinite(timeSeconds)) return false;
        if (timeSeconds < nowSeconds - settings.RetentionSeconds || timeSeconds > nowSeconds)
            return false;
        return _samples.Count == 0 || timeSeconds > _pending[^1].TimeSeconds;
    }

    private static int LowerBound(IReadOnlyList<FlownSample> source, double timeSeconds)
    {
        int lo = 0;
        int hi = source.Count;
        while (lo < hi)
        {
            int mid = lo + ((hi - lo) >> 1);
            if (source[mid].TimeSeconds < timeSeconds) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }

    private void ChangedLocked()
    {
        Volatile.Write(ref _snapshot, null);
        int count = StoredPointCountLocked();
        Volatile.Write(ref _count, count);
        Volatile.Write(ref _lastTime,
            count == 0 ? double.NegativeInfinity : _pending[^1].TimeSeconds);
        Volatile.Write(ref _oldestTime,
            count == 0 ? double.NaN : _samples[0].TimeSeconds);
    }
}

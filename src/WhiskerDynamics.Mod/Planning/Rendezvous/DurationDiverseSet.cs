namespace WhiskerDynamics.Mod.Planning.Rendezvous;

/// <summary>
/// Streaming bounded finalist set combining a global score ranking with small
/// rankings in logarithmic duration bands.
/// </summary>
internal sealed class DurationDiverseSet<T>
{
    private readonly int _globalCount;
    private readonly int _perDurationBand;
    private readonly int _maximumCount;
    private readonly double _baseDuration;
    private readonly Func<T, double> _score;
    private readonly Func<T, double> _duration;
    private readonly List<T> _global = [];
    private readonly Dictionary<int, List<T>> _bands = [];

    public DurationDiverseSet(int globalCount, int perDurationBand, int maximumCount,
        double baseDuration, Func<T, double> score, Func<T, double> duration)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(globalCount);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(perDurationBand);
        if (maximumCount < globalCount)
            throw new ArgumentOutOfRangeException(nameof(maximumCount));
        if (!(baseDuration > 0) || !double.IsFinite(baseDuration))
            throw new ArgumentOutOfRangeException(nameof(baseDuration));
        _globalCount = globalCount;
        _perDurationBand = perDurationBand;
        _maximumCount = maximumCount;
        _baseDuration = baseDuration;
        _score = score;
        _duration = duration;
    }

    public IReadOnlyList<T> BandLeaders => _bands.OrderBy(pair => pair.Key)
        .Select(pair => pair.Value[0]).ToArray();

    public IReadOnlyList<T> Values
    {
        get
        {
            // Reserve the configured global ranking, then spend the remaining budget
            // on duration leaders and runners-up. Emit leaders early so a robust
            // shorter family reaches correction promptly without sacrificing the
            // global fallbacks the constructor promises to retain.
            var selected = new HashSet<T>(_global);
            foreach (var leader in BandLeaders)
                if (selected.Count < _maximumCount) selected.Add(leader);
            foreach (var value in _bands.Values.SelectMany(values => values).OrderBy(_score))
                if (selected.Count < _maximumCount) selected.Add(value);

            var result = new List<T>(_maximumCount);
            if (_global.Count > 0) Add(_global[0]);
            foreach (var leader in BandLeaders) Add(leader);
            foreach (var value in _global.Skip(1)) Add(value);
            foreach (var value in selected.OrderBy(_score)) Add(value);
            return result;

            void Add(T value)
            {
                if (selected.Contains(value) && !result.Contains(value)) result.Add(value);
            }
        }
    }

    public void Add(T value)
    {
        double score = _score(value);
        double duration = _duration(value);
        if (!double.IsFinite(score) || !double.IsFinite(duration) || !(duration > 0)) return;

        Keep(_global, value, _globalCount);
        int band = duration <= _baseDuration
            ? 0
            : Math.Max(0, (int)Math.Floor(Math.Log2(duration / _baseDuration)) + 1);
        if (!_bands.TryGetValue(band, out var values))
            _bands.Add(band, values = []);
        Keep(values, value, _perDurationBand);
    }

    private void Keep(List<T> values, T value, int count)
    {
        if (values.Contains(value)) return;
        values.Add(value);
        values.Sort((a, b) => _score(a).CompareTo(_score(b)));
        if (values.Count > count) values.RemoveRange(count, values.Count - count);
    }
}

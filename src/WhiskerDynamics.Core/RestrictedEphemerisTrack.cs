namespace WhiskerDynamics.Core;

/// <summary>
/// One independently stepped, nonbackreacting trajectory. The body feels the mutual
/// backbone and any massive restricted ancestors supplied by its owner, but never
/// contributes to those sources or to a restricted peer/descendant. Its catalog mu is
/// retained for external gravity consumers.
/// </summary>
internal sealed class RestrictedEphemerisTrack
{
    private readonly List<double> _times = [];
    private readonly List<StateVector> _states = [];
    private readonly List<Vector3d> _accels = [];
    private readonly List<double> _knotTimes = [];
    private readonly List<Knot> _knots = [];
    private int _lastKnotDenseIndex;
    private int _knotHint = 1;
    private int _denseHint = 1;

    private readonly struct Knot(Vector3d position, Vector3d velocity, Vector3d acceleration)
    {
        public readonly Vector3d Position = position;
        public readonly Vector3d Velocity = velocity;
        public readonly Vector3d Acceleration = acceleration;
    }

    public RestrictedEphemerisTrack(double startTime, in StateVector initial,
        in Vector3d acceleration)
    {
        _times.Add(startTime);
        _states.Add(initial);
        _accels.Add(acceleration);
        _knotTimes.Add(startTime);
        _knots.Add(new Knot(initial.Position, initial.Velocity, acceleration));
    }

    public double StartTime => _knotTimes[0];
    public double Horizon => _times[^1];
    public double StableThrough => _knotTimes[^1];
    public int NodeCount => _times.Count;
    public int KnotCount => _knots.Count;
    public StateVector TipState => _states[^1];
    public long Generation { get; private set; }
    public long ApproxBytes => (long)KnotCount * (8 + 72) + (long)NodeCount * (8 + 48 + 24);

    public bool CanAppend(RestrictedEphemerisGrowth growth, long expectedGeneration) =>
        Generation == expectedGeneration && Horizon == growth.SeedTime
        && TipState == growth.SeedState;

    /// <summary>Appends an already validated private suffix. Callers preflight every
    /// track before appending any, making composite publication all-or-nothing.</summary>
    public bool Append(RestrictedEphemerisGrowth growth)
    {
        if (growth.NodeCount == 0) return false;
        _times.AddRange(growth.Times);
        _states.AddRange(growth.States);
        _accels.AddRange(growth.Accelerations);
        Generation++;
        return CommitKnots();
    }

    public bool CommitKnots()
    {
        bool committed = false;
        while (Horizon - _knotTimes[^1] > NBodyEphemerides.KnotGapCapSeconds)
        {
            int end = MaxValidSpanEnd(_lastKnotDenseIndex);
            _knotTimes.Add(_times[end]);
            _knots.Add(new Knot(_states[end].Position, _states[end].Velocity, _accels[end]));
            _lastKnotDenseIndex = end;
            committed = true;
        }
        if (committed) PruneDenseTail();
        return committed;
    }

    public NBodyEphemerides.BodySegment Resolve(double time)
    {
        if (time < StartTime || time > Horizon)
            throw new ArgumentOutOfRangeException(nameof(time),
                $"Restricted trajectory covers [{StartTime:R}, {Horizon:R}].");
        if (time <= _knotTimes[^1])
        {
            int hi = LowerBoundWithHint(_knotTimes, time, ref _knotHint);
            if (_knotTimes[hi] == time)
            {
                var knot = _knots[hi];
                return new NBodyEphemerides.BodySegment(time, 0,
                    new StateVector(knot.Position, knot.Velocity), default,
                    knot.Acceleration, default, quintic: true);
            }
            var a = _knots[hi - 1];
            var b = _knots[hi];
            return new NBodyEphemerides.BodySegment(_knotTimes[hi - 1],
                _knotTimes[hi] - _knotTimes[hi - 1],
                new StateVector(a.Position, a.Velocity),
                new StateVector(b.Position, b.Velocity),
                a.Acceleration, b.Acceleration, quintic: true);
        }

        int denseHi = LowerBoundWithHint(_times, time, ref _denseHint);
        if (_times[denseHi] == time)
            return new NBodyEphemerides.BodySegment(time, 0, _states[denseHi], default,
                default, default, quintic: false);
        return new NBodyEphemerides.BodySegment(_times[denseHi - 1],
            _times[denseHi] - _times[denseHi - 1], _states[denseHi - 1],
            _states[denseHi], default, default, quintic: false);
    }

    public NBodyEphemerides.BodySegment[] CreateSnapshot(double fromTime, double toTime)
    {
        var (committedFirst, committedEndExclusive) =
            NBodyEphemerides.SnapshotSegmentRange(_knotTimes, fromTime, toTime);
        double committedTail = _knotTimes[^1];
        var (denseFirst, denseEnd) =
            NBodyEphemerides.SnapshotSegmentRange(_times, fromTime, toTime);
        denseFirst = Math.Max(denseFirst, UpperBound(_times, committedTail));
        var result = new List<NBodyEphemerides.BodySegment>(
            Math.Max(0, committedEndExclusive - committedFirst)
            + Math.Max(0, denseEnd - denseFirst));
        for (int hi = committedFirst; hi < committedEndExclusive; hi++)
        {
            var a = _knots[hi - 1];
            var b = _knots[hi];
            double t0 = _knotTimes[hi - 1], t1 = _knotTimes[hi];
            result.Add(new NBodyEphemerides.BodySegment(t0, t1 - t0,
                new StateVector(a.Position, a.Velocity),
                new StateVector(b.Position, b.Velocity),
                a.Acceleration, b.Acceleration, quintic: true));
        }
        for (int hi = denseFirst; hi < denseEnd; hi++)
            result.Add(new NBodyEphemerides.BodySegment(_times[hi - 1],
                _times[hi] - _times[hi - 1], _states[hi - 1], _states[hi],
                default, default, quintic: false));
        return [.. result];
    }

    public void Prune(double keepFromTime)
    {
        int keepIndex = 0;
        while (keepIndex + 1 < _knotTimes.Count && _knotTimes[keepIndex + 1] <= keepFromTime)
            keepIndex++;
        if (keepIndex == 0) return;
        _knotTimes.RemoveRange(0, keepIndex);
        _knots.RemoveRange(0, keepIndex);
        _knotHint = Math.Max(1, _knotHint - keepIndex);
        Generation++;
    }

    private int MaxValidSpanEnd(int from)
    {
        int lo0 = from + 1, hi0 = _times.Count - 1;
        double capTime = _times[from] + NBodyEphemerides.KnotGapCapSeconds;
        while (lo0 < hi0)
        {
            int mid = lo0 + (hi0 - lo0 + 1) / 2;
            if (_times[mid] <= capTime) lo0 = mid; else hi0 = mid - 1;
        }
        int maxCandidate = lo0;
        int best = from + 1;
        int probe = Math.Min(from + 2, maxCandidate);
        while (probe > best && SpanValid(from, probe))
        {
            best = probe;
            if (probe == maxCandidate) break;
            probe = Math.Min(from + 2 * (probe - from), maxCandidate);
        }
        if (best < probe)
        {
            int lo = best, hi = probe;
            while (hi - lo > 1)
            {
                int mid = lo + (hi - lo) / 2;
                if (SpanValid(from, mid)) lo = mid; else hi = mid;
            }
            best = lo;
        }
        return best;
    }

    private bool SpanValid(int from, int end)
    {
        double dt = _times[end] - _times[from];
        if (dt > NBodyEphemerides.KnotGapCapSeconds) return false;
        var segment = new NBodyEphemerides.BodySegment(_times[from], dt,
            _states[from], _states[end], _accels[from], _accels[end], quintic: true);
        for (int i = from + 1; i < end; i++)
            if ((NBodyEphemerides.SegmentPosition(in segment, _times[i])
                - _states[i].Position).Length() > NBodyEphemerides.KnotPositionBudget)
                return false;
        return true;
    }

    private void PruneDenseTail()
    {
        int keep = _lastKnotDenseIndex;
        if (keep <= 0) return;
        _times.RemoveRange(0, keep);
        _states.RemoveRange(0, keep);
        _accels.RemoveRange(0, keep);
        _lastKnotDenseIndex = 0;
        _denseHint = Math.Max(1, _denseHint - keep);
    }

    private static int LowerBoundWithHint(List<double> times, double time, ref int hint)
    {
        int count = times.Count;
        int hi = hint;
        if (hi < 1 || hi >= count)
        {
            hi = LowerBoundIn(times, 0, count - 1, time);
        }
        else if (times[hi] < time)
        {
            int lo = hi, step = 1;
            hi = Math.Min(hi + step, count - 1);
            while (times[hi] < time)
            {
                lo = hi;
                step <<= 1;
                hi = Math.Min(hi + step, count - 1);
            }
            hi = LowerBoundIn(times, lo + 1, hi, time);
        }
        else if (times[hi - 1] >= time)
        {
            hi = LowerBoundIn(times, 0, count - 1, time);
        }
        hint = Math.Max(hi, 1);
        return hi;
    }

    private static int LowerBoundIn(List<double> times, int lo, int hi, double time)
    {
        while (lo < hi)
        {
            int mid = lo + ((hi - lo) >> 1);
            if (times[mid] < time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private static int UpperBound(IReadOnlyList<double> times, double time)
    {
        int lo = 0, hi = times.Count;
        while (lo < hi)
        {
            int mid = lo + ((hi - lo) >> 1);
            if (times[mid] <= time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }
}

/// <summary>Private append-only suffix for one restricted track. It contains only
/// accepted nodes after the captured tip, never retained owner history.</summary>
internal sealed class RestrictedEphemerisGrowth(
    double seedTime, StateVector seedState, long seedGeneration)
{
    public double SeedTime { get; } = seedTime;
    public StateVector SeedState { get; } = seedState;
    public long SeedGeneration { get; } = seedGeneration;
    public List<double> Times { get; } = [];
    public List<StateVector> States { get; } = [];
    public List<Vector3d> Accelerations { get; } = [];
    public int NodeCount => Times.Count;

    public void Integrate(double toTime, Func<double, StateVector, Vector3d> acceleration,
        IntegratorOptions options, CancellationToken cancellationToken = default)
    {
        if (toTime <= SeedTime) return;
        _ = DormandPrince54.Propagate(acceleration, SeedState, SeedTime, toTime, options,
            (time, state) =>
            {
                Times.Add(time);
                States.Add(state);
                Accelerations.Add(acceleration(time, state));
            }, cancellationToken);
    }
}

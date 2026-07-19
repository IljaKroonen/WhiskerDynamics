namespace WhiskerDynamics.Core;

public readonly record struct TrajectoryNode(double Time, StateVector State);

/// <summary>Incrementally integrated trajectory with impulsive maneuvers and
/// cubic-Hermite state lookup between accepted integrator steps.</summary>
public sealed class TrajectoryPredictor
{
    private readonly GravityModel _gravity;
    private readonly IntegratorOptions _options;
    private readonly Func<double, StateVector, Vector3d> _acceleration;
    private readonly Action<double, StateVector> _acceptedStep;
    private Action<double>? _extensionProgress;
    private readonly List<TrajectoryNode> _nodes = [];
    private sealed class Impulse
    {
        public required double Time { get; init; }
        public required Vector3d DeltaV { get; init; }
        public bool Applied { get; set; }
        public StateVector? PreImpulseState { get; set; }
    }

    // Invariant: an unapplied impulse can only sit at Time >= Horizon; extension always
    // stops at unapplied impulse boundaries, so ApplyDueImpulses only ever applies an
    // impulse to a tip node at exactly that impulse's time. An impulse applied exactly
    // at Horizon keeps its node (post-burn state) through a truncation at time >= Horizon,
    // so Applied stays true; truncation strictly behind an applied impulse removes its
    // node and correctly resets Applied = false for re-application on re-extension.
    private readonly List<Impulse> _impulses = []; // sorted by Time

    public TrajectoryPredictor(GravityModel gravity, StateVector initialState, double initialTime,
        IntegratorOptions? options = null)
    {
        if (!double.IsFinite(initialTime))
            throw new ArgumentOutOfRangeException(nameof(initialTime),
                "Trajectory start time must be finite.");
        if (!initialState.IsFinite())
            throw new ArgumentException("Initial state must be finite.", nameof(initialState));
        _gravity = gravity;
        _options = IntegratorOptions.Validate(options);
        _acceleration = AccelerationAt;
        _acceptedStep = AcceptStep;
        _nodes.Add(new TrajectoryNode(initialTime, initialState));
    }

    public double StartTime => _nodes[0].Time;
    public double Horizon => _nodes[^1].Time;
    public IReadOnlyList<TrajectoryNode> Nodes => _nodes;

    /// <summary>Maximum memoized nodes per prediction, preventing distant point queries
    /// from retaining an unbounded number of accepted steps.</summary>
    public const int MaxNodes = 2_000_000;

    public void AddImpulse(double time, Vector3d deltaV)
    {
        if (!double.IsFinite(time))
            throw new ArgumentOutOfRangeException(nameof(time), "Impulse time must be finite.");
        if (!deltaV.IsFinite())
            throw new ArgumentException("Impulse delta-v must be finite.", nameof(deltaV));
        if (time < StartTime)
            throw new ArgumentOutOfRangeException(nameof(time), "Impulse before trajectory start.");
        int at = ImpulseLowerBound(time);
        if (at < _impulses.Count && _impulses[at].Time == time)
            throw new ArgumentException("An impulse already exists at this exact time.", nameof(time));
        _impulses.Insert(at, new Impulse { Time = time, DeltaV = deltaV });
        TruncateAfter(time);
    }

    public void ExtendTo(double time, Action<double>? progress = null)
    {
        if (!double.IsFinite(time))
            throw new ArgumentOutOfRangeException(nameof(time), "Extension time must be finite.");
        double before = Horizon;
        var previousProgress = _extensionProgress;
        _extensionProgress = progress;
        try
        {
            ExtendToCore(time);
        }
        catch (InvalidOperationException)
        {
            // Discard partial work from a refused extension and re-arm later impulses.
            TruncateAfter(before);
            throw;
        }
        finally
        {
            _extensionProgress = previousProgress;
        }
    }

    private void ExtendToCore(double time)
    {
        while (true)
        {
            // Apply impulses due at the current tip BEFORE integrating past them —
            // including one added exactly at the horizon.
            ApplyDueImpulses();
            if (Horizon >= time) break;

            var last = _nodes[^1];
            double target = time;
            int nextImpulse = ImpulseUpperBound(last.Time);
            if (nextImpulse < _impulses.Count)
                target = Math.Min(target, _impulses[nextImpulse].Time);

            var end = DormandPrince54.Propagate(
                _acceleration,
                last.State, last.Time, target, _options, _acceptedStep);

            if (_nodes[^1].Time < target)
            {
                _nodes.Add(new TrajectoryNode(target, end));
                _extensionProgress?.Invoke(target);
            }
        }
    }

    private Vector3d AccelerationAt(double time, StateVector state) =>
        _gravity.AccelerationAt(state.Position, time);

    private void AcceptStep(double time, StateVector state)
    {
        if (_nodes.Count >= MaxNodes)
            throw new InvalidOperationException(
                $"Prediction node budget exceeded ({MaxNodes} nodes at t={time:F0} s) — "
                + "the target time is too many orbits ahead for this trajectory's step density.");
        _nodes.Add(new TrajectoryNode(time, state));
        _extensionProgress?.Invoke(time);
    }

    private void ApplyDueImpulses()
    {
        foreach (var impulse in _impulses)
        {
            if (impulse.Applied) continue;
            if (impulse.Time > Horizon) break;
            var tip = _nodes[^1];
            impulse.PreImpulseState = tip.State;
            _nodes[^1] = tip with { State = tip.State with { Velocity = tip.State.Velocity + impulse.DeltaV } };
            impulse.Applied = true;
        }
    }

    public StateVector StateAt(double time)
    {
        if (!double.IsFinite(time))
            throw new ArgumentOutOfRangeException(nameof(time), "Query time must be finite.");
        if (time < StartTime)
            throw new ArgumentOutOfRangeException(nameof(time), "Query before trajectory start.");
        if (time > Horizon) ExtendTo(time);
        else if (time == Horizon) ApplyDueImpulses();

        int hi = LowerBound(time);
        if (_nodes[hi].Time == time) return _nodes[hi].State;
        var a = _nodes[hi - 1];
        var b = _nodes[hi];
        int atImpulse = ImpulseLowerBound(b.Time);
        if (atImpulse < _impulses.Count
            && _impulses[atImpulse] is { Applied: true, PreImpulseState: { } preImpulseState }
            && _impulses[atImpulse].Time == b.Time)
            b = b with { State = preImpulseState };
        return Hermite(a, b, time);
    }

    private int LowerBound(double time)
    {
        int lo = 0, hi = _nodes.Count - 1;
        while (lo < hi)
        {
            int mid = (lo + hi) / 2;
            if (_nodes[mid].Time < time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private int NodeUpperBound(double time)
    {
        int lo = 0, hi = _nodes.Count;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            if (_nodes[mid].Time <= time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private int ImpulseLowerBound(double time)
    {
        int lo = 0, hi = _impulses.Count;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            if (_impulses[mid].Time < time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private int ImpulseUpperBound(double time)
    {
        int lo = 0, hi = _impulses.Count;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            if (_impulses[mid].Time <= time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private void TruncateAfter(double time)
    {
        int keep = Math.Max(1, NodeUpperBound(time));
        _nodes.RemoveRange(keep, _nodes.Count - keep);
        for (int i = ImpulseUpperBound(Horizon); i < _impulses.Count; i++)
        {
            _impulses[i].Applied = false;
            _impulses[i].PreImpulseState = null;
        }
    }

    /// <summary>Drops trajectory nodes strictly before <paramref name="time"/>, keeping the
    /// last node at or before it. Impulse bookkeeping is unaffected: pruning never removes
    /// the tip node, so Horizon and every Applied flag stay valid; impulses whose burn nodes
    /// were pruned are in the past and stay Applied. Not thread-safe: callers serialize.</summary>
    public void PruneBefore(double time)
    {
        if (!double.IsFinite(time))
            throw new ArgumentOutOfRangeException(nameof(time), "Prune time must be finite.");
        int keepIndex = NodeUpperBound(time) - 1;
        if (keepIndex <= 0) return;
        _nodes.RemoveRange(0, keepIndex);
    }

    /// <summary>Cubic Hermite between accepted integrator steps, through the shared
    /// basis home (<see cref="NBodyEphemerides"/>) — vessel-line interpolation and
    /// rails interpolation cannot fork.</summary>
    private static StateVector Hermite(TrajectoryNode a, TrajectoryNode b, double time)
    {
        double dt = b.Time - a.Time;
        double u = (time - a.Time) / dt;
        var stateA = a.State;
        var stateB = b.State;
        return new StateVector(
            NBodyEphemerides.CubicPosition(in stateA, in stateB, dt, u),
            NBodyEphemerides.CubicVelocity(in stateA, in stateB, dt, u));
    }
}

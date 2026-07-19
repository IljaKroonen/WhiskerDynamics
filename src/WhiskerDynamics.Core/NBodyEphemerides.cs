namespace WhiskerDynamics.Core;

/// <summary>Celestial ephemerides computed by one n-body dynamics model. The selected
/// positive-mass set forms a mutually coupled backbone. Every other body is independently
/// stepped in that same time-varying field plus its massive restricted ancestors,
/// without backreaction or restricted-peer coupling. Accepted steps
/// form dense tails, while older nodes are thinned to
/// per-body quintic Hermite knots within <see cref="KnotPositionBudget"/>.</summary>
public sealed class NBodyEphemerides : IEphemerides, ISegmentedEphemerides
{
    /// <summary>Maximum position deviation, in metres, at a dense node replaced by a
    /// committed quintic segment.</summary>
    public const double KnotPositionBudget = 1.0;

    /// <summary>Maximum committed-knot gap in seconds, except when a single accepted
    /// integrator step is longer.</summary>
    public const double KnotGapCapSeconds = 86400.0;

    private readonly Ephemerides _kepler;
    // Segment indices put the shared mutual backbone first. Restricted bodies follow
    // and expose the same query/snapshot surface without entering the quadratic clock.
    private readonly CelestialBody[] _integrated;
    private readonly int _backboneCount;
    private readonly Dictionary<CelestialBody, int> _integratedIndex;
    private readonly PairwiseAccelerationKernel _pairwise;
    private readonly Vector3d[] _accBuffer;
    private readonly IntegratorOptions _options;
    private readonly IntegratorOptions _restrictedOptions;
    private readonly RestrictedEphemerisTrack[] _restrictedTracks;
    // Per restricted track, indices into _restrictedTracks for each positive-mu
    // restricted ancestor whose one-way gravity that descendant receives.
    private readonly int[][] _restrictedAncestorSources;

    // Dense uncommitted tail: shared adaptive-step nodes, newest window only.
    private readonly List<double> _times = [];
    private readonly List<StateVector[]> _states = [];
    private readonly List<Vector3d[]> _accels = [];

    /// <summary>One committed knot: a bit-exact accepted integrator state plus its RHS
    /// acceleration — the quintic Hermite's endpoint data.</summary>
    private readonly struct Knot(Vector3d position, Vector3d velocity, Vector3d acceleration)
    {
        public readonly Vector3d Position = position;
        public readonly Vector3d Velocity = velocity;
        public readonly Vector3d Acceleration = acceleration;
    }

    private sealed class BodyTrack
    {
        public readonly List<double> KnotTimes = [];
        public readonly List<Knot> Knots = [];
        /// <summary>Dense-tail index of the last committed knot (adjusted when the tail
        /// is pruned) — the greedy span search starts here.</summary>
        public int LastKnotDenseIndex;
        /// <summary>Knot segment hint — see <see cref="_segmentHint"/>.</summary>
        public int Hint = 1;
    }

    private readonly BodyTrack[] _tracks;

    /// <summary>Bumped whenever committing or pruning moves a queryable region boundary.
    /// <see cref="GravityModel"/> drops its per-slot segment caches on a change, so a
    /// cached segment can never answer outside the ephemeris's retained window.</summary>
    internal int CommitStamp { get; private set; }
    private long _backboneGeneration;

    public NBodyEphemerides(IReadOnlyList<CelestialBody> bodies, double startTime,
        IReadOnlyCollection<string>? integratedIds = null, IntegratorOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(bodies);
        if (!double.IsFinite(startTime))
            throw new ArgumentOutOfRangeException(nameof(startTime),
                "Ephemeris start time must be finite.");
        _options = IntegratorOptions.Validate(options);
        Bodies = bodies;

        var duplicateIds = bodies.GroupBy(body => body.Id, StringComparer.Ordinal)
            .Where(group => group.Count() > 1).Select(group => group.Key).ToArray();
        if (duplicateIds.Length > 0)
            throw new ArgumentException("Modeled body ids must be unique: "
                + string.Join(", ", duplicateIds), nameof(bodies));

        var roots = bodies.Where(b => b.Parent is null).ToArray();
        if (roots.Length != 1)
            throw new ArgumentException(
                $"The celestial system must contain exactly one root body; found {roots.Length}.",
                nameof(bodies));
        var bodySet = new HashSet<CelestialBody>(bodies, ReferenceEqualityComparer.Instance);
        var missingParents = bodies
            .Where(body => body.Parent is not null && !bodySet.Contains(body.Parent))
            .Select(body => $"{body.Id}->{body.Parent!.Id}")
            .ToArray();
        if (missingParents.Length > 0)
            throw new ArgumentException("Every modeled parent must belong to the catalog: "
                + string.Join(", ", missingParents), nameof(bodies));
        var invalidMasses = bodies
            .Where(b => !double.IsFinite(b.Mu) || b.Mu < 0)
            .Select(b => b.Id)
            .ToArray();
        if (invalidMasses.Length > 0)
            throw new ArgumentException("Modeled bodies require finite nonnegative mu: "
                + string.Join(", ", invalidMasses), nameof(bodies));

        _kepler = new Ephemerides(bodies);

        var selectedIds = integratedIds is null
            ? bodies.Where(body => body.Mu > 0 || body.Parent is null)
                .Select(body => body.Id).ToHashSet(StringComparer.Ordinal)
            : integratedIds.ToHashSet(StringComparer.Ordinal);
        var bodyIds = bodies.Select(body => body.Id).ToHashSet(StringComparer.Ordinal);
        var unknownIds = selectedIds.Where(id => !bodyIds.Contains(id)).ToArray();
        if (unknownIds.Length > 0)
            throw new ArgumentException("The backbone contains unknown body ids: "
                + string.Join(", ", unknownIds), nameof(integratedIds));
        if (!selectedIds.Contains(roots[0].Id))
            throw new ArgumentException(
                "The backbone must include the celestial system's root body.",
                nameof(integratedIds));
        var graph = ParentGraphAnalyzer.AnalyzeBodies(bodies, out int[] bodyIndices);
        var backbone = bodies.Where(body => selectedIds.Contains(body.Id)).ToArray();
        var nonbackreactingSelections = backbone
            .Where(body => body.Parent is not null && body.Mu == 0)
            .Select(body => body.Id).ToArray();
        if (nonbackreactingSelections.Length > 0)
            throw new ArgumentException("Zero-mu bodies must use restricted trajectories: "
                + string.Join(", ", nonbackreactingSelections), nameof(integratedIds));
        var missingBackboneParents = backbone
            .Where(body => body.Parent is not null && !selectedIds.Contains(body.Parent.Id))
            .Select(body => $"{body.Id}->{body.Parent!.Id}").ToArray();
        if (missingBackboneParents.Length > 0)
            throw new ArgumentException("Backbone bodies require backbone parents: "
                + string.Join(", ", missingBackboneParents), nameof(integratedIds));
        _backboneCount = backbone.Length;
        // A descendant can depend on a restricted parent. Keep the caller's order
        // everywhere else, but put restricted tracks in stable parent-before-child
        // order so their one-way ancestor trajectories are complete before use.
        var restricted = bodies.Select((body, index) => (Body: body, Index: index))
            .Where(entry => !selectedIds.Contains(entry.Body.Id))
            .OrderBy(entry => graph.Depths[bodyIndices[entry.Index]])
            .ThenBy(entry => entry.Index)
            .Select(entry => entry.Body);
        _integrated = [.. backbone, .. restricted];
        _integratedIndex = _integrated
            .Select((b, i) => (b, i))
            .ToDictionary(x => x.b, x => x.i);
        double totalMu = backbone.Sum(b => b.Mu);
        if (!(totalMu > 0) || !double.IsFinite(totalMu))
            throw new ArgumentException(
                "The integrated system must have a finite positive total mu.",
                nameof(integratedIds));
        // The kernel owns this array; do not retain a mutable alias.
        ValidateSeeds(bodies, _kepler, startTime);

        var mus = backbone.Select(b => b.Mu).ToArray();
        _pairwise = new PairwiseAccelerationKernel(mus);
        _accBuffer = new Vector3d[_backboneCount];

        var initial = SeedBarycentric(backbone, _kepler, startTime, out var velocityDrift);

        _times.Add(startTime);
        _states.Add(initial);
        _pairwise.Compute(initial, _accBuffer);
        _accels.Add((Vector3d[])_accBuffer.Clone());

        _tracks = new BodyTrack[_backboneCount];
        for (int i = 0; i < _backboneCount; i++)
        {
            var track = new BodyTrack();
            track.KnotTimes.Add(startTime);
            track.Knots.Add(new Knot(initial[i].Position, initial[i].Velocity, _accels[0][i]));
            _tracks[i] = track;
        }

        _restrictedOptions = _options with
        {
            MaxStep = Math.Min(_options.MaxStep, KnotGapCapSeconds),
        };
        _restrictedTracks = new RestrictedEphemerisTrack[_integrated.Length - _backboneCount];
        _restrictedAncestorSources = new int[_restrictedTracks.Length][];
        for (int i = 0; i < _restrictedTracks.Length; i++)
        {
            var restrictedBody = _integrated[_backboneCount + i];
            var ancestorSources = new List<int>();
            for (var ancestor = restrictedBody.Parent; ancestor is not null;
                ancestor = ancestor.Parent)
            {
                if (!_integratedIndex.TryGetValue(ancestor, out int ancestorIndex))
                    throw new ArgumentException(
                        $"Restricted body '{restrictedBody.Id}' has an unmodeled ancestor "+
                        $"'{ancestor.Id}'.", nameof(bodies));
                if (ancestorIndex < _backboneCount) break;
                int restrictedAncestor = ancestorIndex - _backboneCount;
                if (restrictedAncestor >= i)
                    throw new InvalidOperationException(
                        "Restricted tracks must be ordered parent before child.");
                if (ancestor.Mu > 0) ancestorSources.Add(restrictedAncestor);
            }
            _restrictedAncestorSources[i] = [.. ancestorSources];

            var seed = _kepler.GetState(restrictedBody, startTime);
            seed = seed with { Velocity = seed.Velocity - velocityDrift };
            var acceleration = InitialRestrictedAcceleration(i, seed);
            _restrictedTracks[i] = new RestrictedEphemerisTrack(startTime, seed, acceleration);
        }
    }

    private Vector3d InitialRestrictedAcceleration(int restrictedIndex, in StateVector state)
    {
        var acceleration = Vector3d.Zero;
        for (int body = 0; body < _backboneCount; body++)
        {
            var offset = _states[0][body].Position - state.Position;
            double r2 = offset.LengthSquared();
            acceleration += offset * (_integrated[body].Mu / (r2 * Math.Sqrt(r2)));
        }
        foreach (int ancestor in _restrictedAncestorSources[restrictedIndex])
        {
            var offset = _restrictedTracks[ancestor].TipState.Position - state.Position;
            double r2 = offset.LengthSquared();
            acceleration += offset * (_integrated[_backboneCount + ancestor].Mu
                / (r2 * Math.Sqrt(r2)));
        }
        return acceleration;
    }

    private static void ValidateSeeds(
        IReadOnlyList<CelestialBody> bodies, Ephemerides kepler, double startTime)
    {
        foreach (var body in bodies)
        {
            StateVector seed;
            try { seed = kepler.GetState(body, startTime); }
            catch (Exception e) when (e is InvalidOperationException
                or NotSupportedException or NullReferenceException)
            {
                throw new ArgumentException(
                    $"Body '{body.Id}' has no valid n-body seed: {e.Message}", nameof(bodies), e);
            }
            if (!IsFinite(seed.Position) || !IsFinite(seed.Velocity))
                throw new ArgumentException(
                    $"Body '{body.Id}' has a non-finite n-body seed.", nameof(bodies));
        }
    }

    private static bool IsFinite(in Vector3d value) =>
        double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);

    /// <summary>Creates Kepler states at the start epoch and removes their net
    /// momentum so the system barycentre remains stationary.</summary>
    internal static StateVector[] SeedBarycentric(
        IReadOnlyList<CelestialBody> integrated, Ephemerides kepler, double startTime)
        => SeedBarycentric(integrated, kepler, startTime, out _);

    private static StateVector[] SeedBarycentric(
        IReadOnlyList<CelestialBody> integrated, Ephemerides kepler, double startTime,
        out Vector3d drift)
    {
        var initial = new StateVector[integrated.Count];
        double totalMu = 0;
        var momentum = Vector3d.Zero;
        for (int i = 0; i < integrated.Count; i++)
        {
            initial[i] = kepler.GetState(integrated[i], startTime);
            totalMu += integrated[i].Mu;
            momentum += initial[i].Velocity * integrated[i].Mu;
        }
        drift = momentum / totalMu;
        for (int i = 0; i < integrated.Count; i++)
            initial[i] = initial[i] with { Velocity = initial[i].Velocity - drift };
        return initial;
    }

    public IReadOnlyList<CelestialBody> Bodies { get; }

    public CelestialBody this[string id] => _kepler[id];

    /// <summary>True only when the body participates in mutual backbone gravity.
    /// False does not mean unmodeled: every catalog body has a numerical track.</summary>
    public bool IsBackbone(CelestialBody body) =>
        _integratedIndex.TryGetValue(body, out int index) && index < _backboneCount;

    public bool IsBackbone(string id) => IsBackbone(_kepler[id]);

    /// <inheritdoc />
    public bool FeelsGravityFrom(CelestialBody body, CelestialBody source)
    {
        if (ReferenceEquals(body, source) || source.Mu == 0
            || !_integratedIndex.TryGetValue(body, out int bodyIndex)
            || !_integratedIndex.TryGetValue(source, out int sourceIndex))
            return false;
        if (sourceIndex < _backboneCount) return true;
        if (bodyIndex < _backboneCount) return false;
        int restrictedSource = sourceIndex - _backboneCount;
        return Array.IndexOf(
            _restrictedAncestorSources[bodyIndex - _backboneCount], restrictedSource) >= 0;
    }

    /// <summary>Earliest instant EVERY integrated body can answer for (bodies commit
    /// knots at independent paces, so retained starts differ per body by up to a gap;
    /// this is the latest of them — queries at or after it never throw).</summary>
    public double StartTime
    {
        get
        {
            double start = double.NegativeInfinity;
            foreach (var track in _tracks) start = Math.Max(start, track.KnotTimes[0]);
            foreach (var track in _restrictedTracks)
                if (track is not null) start = Math.Max(start, track.StartTime);
            return start;
        }
    }

    private double BackboneHorizon => _times[^1];

    /// <summary>Latest instant every modeled trajectory can answer.</summary>
    public double Horizon
    {
        get
        {
            double horizon = BackboneHorizon;
            foreach (var track in _restrictedTracks)
                if (track is not null) horizon = Math.Min(horizon, track.Horizon);
            return horizon;
        }
    }

    /// <summary>Latest instant whose interpolation representation is committed for
    /// every integrated body. Snapshots ending at or before this frontier cannot be
    /// changed by later horizon growth; the remaining dense tail is bounded by
    /// <see cref="KnotGapCapSeconds"/>.</summary>
    public double SnapshotStableThrough
    {
        get
        {
            double stable = double.PositiveInfinity;
            foreach (var track in _tracks) stable = Math.Min(stable, track.KnotTimes[^1]);
            foreach (var track in _restrictedTracks)
                if (track is not null) stable = Math.Min(stable, track.StableThrough);
            return stable;
        }
    }

    /// <summary>Dense (uncommitted tail) node count — bounded by the gap cap's worth of
    /// shared adaptive steps, not the horizon (bench/telemetry observability).</summary>
    public int NodeCount => _times.Count + _restrictedTracks.Sum(track => track.NodeCount);

    /// <summary>Committed knots across all integrated bodies (bench/telemetry
    /// observability) — the horizon-proportional part of the store.</summary>
    public int KnotCount
    {
        get
        {
            int count = 0;
            foreach (var track in _tracks) count += track.Knots.Count;
            foreach (var track in _restrictedTracks) count += track.KnotCount;
            return count;
        }
    }

    /// <summary>Approximate retained bytes: per-body knots plus the dense tail
    /// (bench gates and telemetry — capacity slack and pooling excluded).</summary>
    public long ApproxBytes =>
        (long)_tracks.Sum(track => track.Knots.Count) * (8 + 72)
        + (long)_times.Count * (8 + _backboneCount * (48 + 24))
        + _restrictedTracks.Sum(track => track.ApproxBytes);

    public StateVector GetState(CelestialBody body, double time)
    {
        if (!double.IsFinite(time))
            throw new ArgumentOutOfRangeException(nameof(time), "Query time must be finite.");
        if (!_integratedIndex.TryGetValue(body, out int index))
            throw new ArgumentException(
                $"Body '{body.Id}' does not belong to this ephemeris.", nameof(body));
        var segment = ResolveBodySegment(index, time);
        return SegmentState(in segment, time);
    }

    /// <summary>Reads without extending the shared horizon. Game-thread callers use
    /// this seam so a distant query cannot become an unbounded integration.</summary>
    public bool TryGetState(CelestialBody body, double time, out StateVector state)
    {
        state = default;
        if (!double.IsFinite(time) || time < StartTime || time > Horizon) return false;
        state = GetState(body, time); // the range check makes this non-extending
        return true;
    }

    /// <summary>Dense-tail segment hint for time-local queries. Bracketing is rechecked
    /// on every use, so pruning or extension can only invalidate its performance benefit.</summary>
    private int _segmentHint = 1;

    /// <summary>A body's resolved interpolation segment: quintic between committed
    /// knots, cubic between dense nodes, or an exact hit when Dt is zero.</summary>
    internal readonly struct BodySegment
    {
        public readonly double T0;
        public readonly double Dt;
        public readonly StateVector A;
        public readonly StateVector B;
        public readonly Vector3d AccA;
        public readonly Vector3d AccB;
        public readonly bool Quintic;

        internal BodySegment(double t0, double dt, in StateVector a, in StateVector b,
            in Vector3d accA, in Vector3d accB, bool quintic)
        {
            T0 = t0; Dt = dt; A = a; B = b; AccA = accA; AccB = accB; Quintic = quintic;
        }
    }

    /// <summary>Deep-copies exact interpolation segments overlapping the requested
    /// range. Callers serialize this mutable owner while capturing.</summary>
    public SegmentedEphemeridesSnapshot CreateSnapshot(double fromTime, double toTime)
    {
        if (!double.IsFinite(fromTime) || !double.IsFinite(toTime)
            || fromTime < StartTime || toTime > Horizon || toTime <= fromTime)
            throw new ArgumentOutOfRangeException(nameof(fromTime),
                $"Snapshot [{fromTime:R}, {toTime:R}] must lie inside [{StartTime:R}, {Horizon:R}].");
        var copied = new BodySegment[_integrated.Length][];
        for (int body = 0; body < _integrated.Length; body++)
        {
            if (body >= _backboneCount)
            {
                var restricted = _restrictedTracks[body - _backboneCount];
                copied[body] = restricted.CreateSnapshot(fromTime, toTime);
                if (copied[body].Length == 0)
                    throw new InvalidOperationException(
                        $"No segment copied for '{_integrated[body].Id}' over the requested range.");
                continue;
            }
            var track = _tracks[body];
            var (committedFirst, committedEndExclusive) =
                SnapshotSegmentRange(track.KnotTimes, fromTime, toTime);
            double committedTail = track.KnotTimes[^1];
            var (denseFirst, denseEnd) = SnapshotSegmentRange(_times, fromTime, toTime);
            denseFirst = Math.Max(denseFirst, UpperBound(_times, committedTail));
            int capacity = Math.Max(0, committedEndExclusive - committedFirst)
                + Math.Max(0, denseEnd - denseFirst);
            var result = new List<BodySegment>(capacity);
            for (int hi = committedFirst; hi < committedEndExclusive; hi++)
            {
                double t0 = track.KnotTimes[hi - 1], t1 = track.KnotTimes[hi];
                var a = track.Knots[hi - 1];
                var b = track.Knots[hi];
                result.Add(new BodySegment(t0, t1 - t0,
                    new StateVector(a.Position, a.Velocity),
                    new StateVector(b.Position, b.Velocity),
                    a.Acceleration, b.Acceleration, quintic: true));
            }
            for (int hi = denseFirst; hi < denseEnd; hi++)
            {
                double t0 = _times[hi - 1], t1 = _times[hi];
                result.Add(new BodySegment(t0, t1 - t0,
                    _states[hi - 1][body], _states[hi][body],
                    default, default, quintic: false));
            }
            if (result.Count == 0)
                throw new InvalidOperationException(
                    $"No segment copied for '{_integrated[body].Id}' over the requested range.");
            // Both sources are already chronological, and dense segments begin at
            // the committed tail. Avoid sorting the copied window while Rails.Gate
            // is held by the capture caller.
            copied[body] = [.. result];
        }
        return new SegmentedEphemeridesSnapshot(
            Bodies, _integrated, copied, _backboneCount, fromTime, toTime);
    }

    int ISegmentedEphemerides.CommitStamp => CommitStamp;
    int ISegmentedEphemerides.IntegratedIndexOf(CelestialBody body) => IntegratedIndexOf(body);
    BodySegment ISegmentedEphemerides.ResolveBodySegment(int bodyIndex, double time) =>
        ResolveBodySegment(bodyIndex, time);
    bool ISegmentedEphemerides.InCommittedRegion(int bodyIndex, double time) =>
        InCommittedRegion(bodyIndex, time);
    Vector3d ISegmentedEphemerides.BodyPositionAt(int bodyIndex, double time) =>
        BodyPositionAt(bodyIndex, time);
    bool ISegmentedEphemerides.TryResolveDenseSegment(
        double time, out int hi, out double t0, out double dt) =>
        TryResolveDenseSegment(time, out hi, out t0, out dt);
    StateVector ISegmentedEphemerides.DenseNodeState(int nodeIndex, int bodyIndex) =>
        DenseNodeState(nodeIndex, bodyIndex);

    /// <summary>Locates the segment bracketing <paramref name="time"/> for one
    /// integrated body, extending (and committing) past the horizon first. Exact node
    /// hits return Dt == 0 with the state in A.</summary>
    internal BodySegment ResolveBodySegment(int bodyIndex, double time)
    {
        if (time > Horizon)
            GrowCompositeTo(time);
        if (bodyIndex >= _backboneCount)
            return _restrictedTracks[bodyIndex - _backboneCount].Resolve(time);
        var track = _tracks[bodyIndex];
        var knotTimes = track.KnotTimes;
        if (time < knotTimes[0])
            throw new ArgumentOutOfRangeException(nameof(time), "Query before ephemeris start.");

        if (InCommittedRegion(bodyIndex, time))
        {
            int hint = track.Hint;
            int hi = LowerBoundWithHint(knotTimes, time, ref hint);
            track.Hint = hint;
            var knots = track.Knots;
            if (knotTimes[hi] == time)
            {
                var k = knots[hi];
                return new BodySegment(time, 0,
                    new StateVector(k.Position, k.Velocity), default, k.Acceleration, default, true);
            }
            var a = knots[hi - 1];
            var b = knots[hi];
            return new BodySegment(knotTimes[hi - 1], knotTimes[hi] - knotTimes[hi - 1],
                new StateVector(a.Position, a.Velocity), new StateVector(b.Position, b.Velocity),
                a.Acceleration, b.Acceleration, quintic: true);
        }

        // Non-quintic segments never read accelerations — the cubic basis is
        // position/velocity only, so the acc fields stay default rather than
        // shuttling values no consumer evaluates.
        int denseHint = _segmentHint;
        int denseHi = LowerBoundWithHint(_times, time, ref denseHint);
        _segmentHint = denseHint;
        if (_times[denseHi] == time)
            return new BodySegment(time, 0, _states[denseHi][bodyIndex], default,
                default, default, quintic: false);
        return new BodySegment(_times[denseHi - 1], _times[denseHi] - _times[denseHi - 1],
            _states[denseHi - 1][bodyIndex], _states[denseHi][bodyIndex],
            default, default, quintic: false);
    }

    /// <summary>Whether a time belongs to a body's committed quintic region.</summary>
    internal bool InCommittedRegion(int bodyIndex, double time) =>
        bodyIndex < _backboneCount
            ? time <= _tracks[bodyIndex].KnotTimes[^1]
            : time <= _restrictedTracks[bodyIndex - _backboneCount].StableThrough;

    /// <summary>Index into the modeled set, or -1 when <paramref name="body"/> does
    /// not belong to this ephemeris.</summary>
    internal int IntegratedIndexOf(CelestialBody body) =>
        _integratedIndex.TryGetValue(body, out int i) ? i : -1;

    /// <summary>One body's committed-region end (its last knot's time) — queries at or
    /// before it evaluate the quintic knots, later ones the dense tail. Test/telemetry
    /// observability for the region boundary.</summary>
    internal double LastKnotTime(int bodyIndex) => bodyIndex < _backboneCount
        ? _tracks[bodyIndex].KnotTimes[^1]
        : _restrictedTracks[bodyIndex - _backboneCount].StableThrough;

    /// <summary>Position-only segment evaluation for the scalar gravity paths — routes
    /// through the same segment resolve and basis as <see cref="GetState"/>.</summary>
    internal Vector3d BodyPositionAt(int bodyIndex, double time)
    {
        var segment = ResolveBodySegment(bodyIndex, time);
        return SegmentPosition(in segment, time);
    }

    /// <summary>Resolves the shared dense-tail bracket for a batched gravity-cache
    /// rebuild. Returns false for exact hits or times outside the dense window.</summary>
    internal bool TryResolveDenseSegment(double time, out int hi, out double t0, out double dt)
    {
        hi = 0; t0 = 0; dt = 0;
        if (time <= _times[0] || time > _times[^1]) return false;
        int denseHint = _segmentHint;
        hi = LowerBoundWithHint(_times, time, ref denseHint);
        _segmentHint = denseHint;
        if (_times[hi] == time) return false;
        t0 = _times[hi - 1];
        dt = _times[hi] - t0;
        return true;
    }

    /// <summary>Direct dense-node row access for the batched tail rebuild — the same
    /// values <see cref="ResolveBodySegment"/> packages, without the struct copy.</summary>
    internal StateVector DenseNodeState(int nodeIndex, int bodyIndex)
    {
        if (bodyIndex >= _backboneCount)
            throw new ArgumentOutOfRangeException(nameof(bodyIndex),
                "Restricted tracks do not share the backbone dense-node table.");
        return _states[nodeIndex][bodyIndex];
    }

    /// <summary>Transactional safety-net growth for synchronous callers. It uses the
    /// same private composite builder as the worker path, so schedule cannot change the
    /// restricted-body equations or interpolation.</summary>
    private void GrowCompositeTo(double time)
    {
        var grower = new DetachedGrower(this);
        grower.CaptureSeed();
        grower.Integrate(time);
        if (!grower.TrySplice())
            throw new InvalidOperationException("Composite ephemeris changed during serialized growth.");
    }

    /// <summary>Integrates horizon-growth chunks in private scratch so callers need the
    /// owner lock only to capture a seed and splice results. A splice fails if the owner
    /// horizon changed after capture. One caller may use a grower at a time.</summary>
    public sealed class DetachedGrower
    {
        private readonly NBodyEphemerides _owner;
        private readonly PairwiseAccelerationKernel _kernel;
        private readonly Vector3d[] _accBuffer;
        private readonly StateVector[] _seed;
        private double _seedTime = double.NaN;
        private long _seedBackboneGeneration;
        private bool _seedCaptured;
        private bool _integratedSinceCapture;
        private bool _readyToSplice;
        private readonly List<double> _times = [];
        private readonly List<StateVector[]> _states = [];
        private readonly List<Vector3d[]> _accels = [];
        private RestrictedEphemerisGrowth[] _restricted = [];
        private readonly Stack<StateVector[]> _statePool = new();
        private readonly Stack<Vector3d[]> _accPool = new();
        private int _activeRestrictedIndex = -1;

        internal DetachedGrower(NBodyEphemerides owner)
        {
            _owner = owner;
            _kernel = new PairwiseAccelerationKernel(owner._integrated
                .Take(owner._backboneCount).Select(b => b.Mu).ToArray());
            _accBuffer = new Vector3d[owner._backboneCount];
            _seed = new StateVector[owner._backboneCount];
        }

        /// <summary>Captures the current tip while the caller holds the owner lock.</summary>
        public double CaptureSeed()
        {
            Array.Copy(_owner._states[^1], _seed, _seed.Length);
            _seedTime = _owner._times[^1];
            _seedBackboneGeneration = _owner._backboneGeneration;
            while (_statePool.Count < 512 && _owner._statePool.TryPop(out var s)) _statePool.Push(s);
            while (_accPool.Count < 512 && _owner._accPool.TryPop(out var a)) _accPool.Push(a);
            ReclaimScratch();
            if (_owner._restrictedTracks.Any(track => track.Horizon != _seedTime))
                throw new InvalidOperationException(
                    "Every restricted track must reach the composite tip before detached growth.");
            _restricted = _owner._restrictedTracks.Select(track =>
                new RestrictedEphemerisGrowth(
                    _seedTime, track.TipState, track.Generation)).ToArray();
            _seedCaptured = true;
            _integratedSinceCapture = false;
            _readyToSplice = false;
            return _seedTime;
        }

        /// <summary>Integrates from the captured seed to <paramref name="toTime"/> in
        /// private scratch. A target at or behind the seed is a no-op.</summary>
        public void Integrate(double toTime, CancellationToken cancellationToken = default)
        {
            if (!_seedCaptured)
                throw new InvalidOperationException("CaptureSeed must be called before detached integration.");
            if (!double.IsFinite(toTime))
                throw new ArgumentOutOfRangeException(nameof(toTime),
                    "Detached integration target must be finite.");
            if (toTime <= _seedTime) { _readyToSplice = true; return; }
            if (_integratedSinceCapture)
                throw new InvalidOperationException(
                    "A detached grower can integrate only once per captured seed.");
            _integratedSinceCapture = true;
            var end = DormandPrince54.PropagateSystem(
                (t, states) => { _kernel.Compute(states, _accBuffer); return _accBuffer; },
                _seed, _seedTime, toTime, _owner._options,
                (t, states, derivatives) =>
                {
                    _times.Add(t);
                    var node = _statePool.TryPop(out var pooled) && pooled.Length == states.Length
                        ? pooled
                        : new StateVector[states.Length];
                    Array.Copy(states, node, states.Length);
                    _states.Add(node);
                    var acc = _accPool.TryPop(out var pooledAcc) && pooledAcc.Length == derivatives.Length
                        ? pooledAcc
                        : new Vector3d[derivatives.Length];
                    for (int b = 0; b < derivatives.Length; b++) acc[b] = derivatives[b].Velocity;
                    _accels.Add(acc);
                }, cancellationToken);
            if (_times.Count == 0 || _times[^1] < toTime)
            {
                _times.Add(toTime);
                _states.Add((StateVector[])end.Clone());
                _kernel.Compute(end, _accBuffer);
                _accels.Add((Vector3d[])_accBuffer.Clone());
            }
            try
            {
                for (int i = 0; i < _restricted.Length; i++)
                {
                    _activeRestrictedIndex = i;
                    _restricted[i].Integrate(toTime, ScratchRestrictedAccelerationAt,
                        _owner._restrictedOptions, cancellationToken);
                }
            }
            finally { _activeRestrictedIndex = -1; }
            _readyToSplice = true;
        }

        /// <summary>Splices the chunk and commits knots while the caller holds the owner
        /// lock. Returns false if the owner horizon changed after capture.</summary>
        public bool TrySplice()
        {
            if (!_readyToSplice
                || _owner._times[^1] != _seedTime
                || _owner._backboneGeneration != _seedBackboneGeneration
                || _owner._restrictedTracks.Length != _restricted.Length)
                return false;
            for (int i = 0; i < _restricted.Length; i++)
                if (!_owner._restrictedTracks[i].CanAppend(
                    _restricted[i], _restricted[i].SeedGeneration))
                    return false;

            for (int i = 0; i < _times.Count; i++)
            {
                _owner._times.Add(_times[i]);
                _owner._states.Add(_states[i]);
                _owner._accels.Add(_accels[i]);
            }
            _times.Clear();
            _states.Clear();
            _accels.Clear();
            bool restrictedCommitted = false;
            for (int i = 0; i < _restricted.Length; i++)
                restrictedCommitted |= _owner._restrictedTracks[i].Append(_restricted[i]);
            _restricted = [];
            _readyToSplice = false;
            _owner._backboneGeneration++;
            if (restrictedCommitted) _owner.CommitStamp++;
            _owner.CommitKnots();
            _seedCaptured = false;
            return true;
        }

        /// <summary>Accepted restricted suffix nodes currently held in private scratch.
        /// Zero immediately after capture proves retained histories were not copied.</summary>
        internal int RestrictedScratchNodeCount =>
            _restricted.Sum(growth => growth.NodeCount);

        internal int RestrictedScratchTrackCount => _restricted.Length;

        private Vector3d ScratchRestrictedAccelerationAt(double time, StateVector state)
        {
            var acceleration = Vector3d.Zero;
            for (int body = 0; body < _owner._backboneCount; body++)
            {
                var offset = ScratchBackbonePositionAt(body, time) - state.Position;
                double r2 = offset.LengthSquared();
                acceleration += offset
                    * (_owner._integrated[body].Mu / (r2 * Math.Sqrt(r2)));
            }
            foreach (int ancestor in _owner._restrictedAncestorSources[_activeRestrictedIndex])
            {
                var offset = ScratchRestrictedPositionAt(ancestor, time) - state.Position;
                double r2 = offset.LengthSquared();
                acceleration += offset * (_owner._integrated[
                    _owner._backboneCount + ancestor].Mu / (r2 * Math.Sqrt(r2)));
            }
            return acceleration;
        }

        private Vector3d ScratchRestrictedPositionAt(int restrictedIndex, double time)
        {
            var growth = _restricted[restrictedIndex];
            if (time == _seedTime) return growth.SeedState.Position;
            if (time < _seedTime || growth.Times.Count == 0 || time > growth.Times[^1])
                throw new ArgumentOutOfRangeException(nameof(time));
            int hi = LowerBound(growth.Times, time);
            if (growth.Times[hi] == time) return growth.States[hi].Position;
            double t0;
            StateVector a;
            if (hi == 0) { t0 = _seedTime; a = growth.SeedState; }
            else { t0 = growth.Times[hi - 1]; a = growth.States[hi - 1]; }
            var segment = new BodySegment(t0, growth.Times[hi] - t0,
                a, growth.States[hi], default, default, quintic: false);
            return SegmentPosition(in segment, time);
        }

        private Vector3d ScratchBackbonePositionAt(int bodyIndex, double time)
        {
            if (time == _seedTime) return _seed[bodyIndex].Position;
            if (time < _seedTime || _times.Count == 0 || time > _times[^1])
                throw new ArgumentOutOfRangeException(nameof(time));
            int hi = LowerBound(_times, time);
            if (_times[hi] == time) return _states[hi][bodyIndex].Position;
            double t0;
            StateVector a;
            if (hi == 0) { t0 = _seedTime; a = _seed[bodyIndex]; }
            else { t0 = _times[hi - 1]; a = _states[hi - 1][bodyIndex]; }
            var segment = new BodySegment(t0, _times[hi] - t0,
                a, _states[hi][bodyIndex], default, default, quintic: false);
            return SegmentPosition(in segment, time);
        }

        /// <summary>Returns unspliced node arrays to the private pools (a refused
        /// splice, or a stale chunk from an aborted round).</summary>
        private void ReclaimScratch()
        {
            for (int i = 0; i < _states.Count; i++)
            {
                if (_statePool.Count < NodePoolCap) _statePool.Push(_states[i]);
                if (_accPool.Count < NodePoolCap) _accPool.Push(_accels[i]);
            }
            _times.Clear();
            _states.Clear();
            _accels.Clear();
            _restricted = [];
        }
    }

    /// <summary>One grower per growth-owning thread — see <see cref="DetachedGrower"/>.</summary>
    public DetachedGrower CreateGrower() => new(this);

    /// <summary>Commits knots for bodies with a full gap cap of lookahead, then recycles
    /// dense nodes that every body has committed past.</summary>
    private void CommitKnots()
    {
        double horizon = _times[^1];
        bool committed = false;
        for (int i = 0; i < _tracks.Length; i++)
        {
            var track = _tracks[i];
            while (horizon - track.KnotTimes[^1] > KnotGapCapSeconds)
            {
                int end = MaxValidSpanEnd(i, track.LastKnotDenseIndex);
                track.KnotTimes.Add(_times[end]);
                track.Knots.Add(new Knot(_states[end][i].Position, _states[end][i].Velocity,
                    _accels[end][i]));
                track.LastKnotDenseIndex = end;
                committed = true;
            }
        }
        if (!committed) return;
        CommitStamp++;
        _backboneGeneration++;
        PruneDenseTail();
    }

    /// <summary>Finds the farthest within-cap dense node whose quintic reproduces all
    /// interior nodes within budget. The search always admits the adjacent node.</summary>
    private int MaxValidSpanEnd(int bodyIndex, int from)
    {
        // Largest candidate index whose gap stays within the cap — the probe wall.
        int lo0 = from + 1, hi0 = _times.Count - 1;
        double capTime = _times[from] + KnotGapCapSeconds;
        while (lo0 < hi0)
        {
            int mid = lo0 + (hi0 - lo0 + 1) / 2;
            if (_times[mid] <= capTime) lo0 = mid; else hi0 = mid - 1;
        }
        int maxCandidate = lo0;

        int best = from + 1;
        int probe = Math.Min(from + 2, maxCandidate);
        while (probe > best && SpanValid(bodyIndex, from, probe))
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
                if (SpanValid(bodyIndex, from, mid)) lo = mid; else hi = mid;
            }
            best = lo;
        }
        return best;
    }

    private bool SpanValid(int bodyIndex, int from, int end)
    {
        if (_times[end] - _times[from] > KnotGapCapSeconds) return false;
        double t0 = _times[from];
        double dt = _times[end] - t0;
        var a = _states[from][bodyIndex];
        var b = _states[end][bodyIndex];
        var accA = _accels[from][bodyIndex];
        var accB = _accels[end][bodyIndex];
        for (int i = from + 1; i < end; i++)
        {
            var p = QuinticPosition(in a, in accA, in b, in accB, dt, (_times[i] - t0) / dt);
            if ((p - _states[i][bodyIndex].Position).Length() > KnotPositionBudget) return false;
        }
        return true;
    }

    /// <summary>Drops dense tail nodes strictly before the earliest last-committed knot
    /// (its node stays — it IS a knot and the tail evaluation's left bracket for the
    /// body that owns it), recycling their arrays into the extension pools.</summary>
    private void PruneDenseTail()
    {
        int keep = int.MaxValue;
        foreach (var track in _tracks) keep = Math.Min(keep, track.LastKnotDenseIndex);
        if (keep <= 0) return;
        for (int i = 0; i < keep; i++)
        {
            if (_statePool.Count < NodePoolCap) _statePool.Push(_states[i]);
            if (_accPool.Count < NodePoolCap) _accPool.Push(_accels[i]);
        }
        _times.RemoveRange(0, keep);
        _states.RemoveRange(0, keep);
        _accels.RemoveRange(0, keep);
        foreach (var track in _tracks) track.LastKnotDenseIndex -= keep;
        _segmentHint = Math.Max(1, _segmentHint - keep);
    }

    /// <summary>Recycled node arrays, reused by extension so the steady-state
    /// extend-and-commit loop (the in-game rails pattern) allocates nothing.
    /// Capped so a one-off deep prune cannot pin more than a few MB of dead arrays.</summary>
    private readonly Stack<StateVector[]> _statePool = new();
    private readonly Stack<Vector3d[]> _accPool = new();
    private const int NodePoolCap = 1024;

    private StateVector[] CopyNode(StateVector[] source)
    {
        var node = _statePool.TryPop(out var pooled) && pooled.Length == source.Length
            ? pooled
            : new StateVector[source.Length];
        Array.Copy(source, node, source.Length);
        return node;
    }

    private Vector3d[] RentAccNode() =>
        _accPool.TryPop(out var pooled) && pooled.Length == _backboneCount
            ? pooled
            : new Vector3d[_backboneCount];

    /// <summary>Drops committed knots strictly before <paramref name="keepFromTime"/>,
    /// keeping each body's last knot at or before it so interpolation across the
    /// boundary stays exact. Queries earlier than the retained window then throw
    /// ArgumentOutOfRangeException (<see cref="StartTime"/> rises with the retained
    /// knots). The dense tail rides the horizon end and is untouched here; it recycles
    /// through <see cref="PruneDenseTail"/> as bodies commit past its nodes.
    /// Not thread-safe: callers serialize.</summary>
    public void Prune(double keepFromTime)
    {
        if (!double.IsFinite(keepFromTime))
            throw new ArgumentOutOfRangeException(nameof(keepFromTime),
                "Retention boundary must be finite.");
        bool pruned = false;
        bool backbonePruned = false;
        foreach (var track in _tracks)
        {
            var times = track.KnotTimes;
            int keepIndex = 0;
            while (keepIndex + 1 < times.Count && times[keepIndex + 1] <= keepFromTime) keepIndex++;
            if (keepIndex == 0) continue;
            times.RemoveRange(0, keepIndex);
            track.Knots.RemoveRange(0, keepIndex);
            track.Hint = Math.Max(1, track.Hint - keepIndex);
            pruned = true;
            backbonePruned = true;
        }
        foreach (var track in _restrictedTracks)
        {
            double before = track.StartTime;
            track.Prune(keepFromTime);
            pruned |= track.StartTime != before;
        }
        if (backbonePruned) _backboneGeneration++;
        if (pruned) CommitStamp++;
    }

    /// <summary>O(n²/2) pairwise gravity via Newton's third law, SIMD inner loop —
    /// see <see cref="PairwiseAccelerationKernel"/>. Returns a per-instance buffer
    /// reused across calls — the integrator consumes it immediately; callers must
    /// not retain it. Not thread-safe (like all of this class).</summary>
    private Vector3d[] MutualAccelerations(StateVector[] states)
    {
        _pairwise.Compute(states, _accBuffer);
        return _accBuffer;
    }

    /// <summary>Smallest index with times[index] &gt;= time, hint-accelerated: gallop
    /// right from the hint (the dominant forward-sweep pattern), full binary search on
    /// a backward jump. Callers guarantee times[0] &lt;= time &lt;= times[^1]; a return
    /// of 0 is necessarily an exact hit on times[0] (the caller's exact-hit check
    /// handles it — only non-exact returns are guaranteed &gt;= 1 for bracketing).</summary>
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
            while (times[hi] < time) // terminates: time <= times[count - 1]
            {
                lo = hi;
                step <<= 1;
                hi = Math.Min(hi + step, count - 1);
            }
            hi = LowerBoundIn(times, lo + 1, hi, time);
        }
        else if (times[hi - 1] > time)
        {
            hi = LowerBoundIn(times, 0, count - 1, time); // backward jump: rare, full search
        }
        hint = Math.Max(hi, 1);
        return hi;
    }

    /// <summary>Smallest index in [lo, hi] with times[index] &gt;= time, given
    /// times[hi] &gt;= time.</summary>
    private static int LowerBoundIn(List<double> times, int lo, int hi, double time)
    {
        while (lo < hi)
        {
            int mid = (lo + hi) / 2;
            if (times[mid] < time) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    internal static (int FirstHi, int EndHiExclusive) SnapshotSegmentRange(
        IReadOnlyList<double> times, double fromTime, double toTime)
    {
        int count = times.Count;
        if (count < 2) return (1, 1);
        int first = Math.Max(1, LowerBound(times, fromTime));
        if (first >= count) return (count, count);
        int upper = UpperBound(times, toTime);
        int end = upper >= count ? count : upper + 1;
        return (first, Math.Max(first, end));
    }

    private static int LowerBound(IReadOnlyList<double> times, double time)
    {
        int lo = 0, hi = times.Count;
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

    // Shared Hermite basis for state lookup, gravity, prediction, and thinning probes.
    // GravityModel uses algebraically equivalent monomial forms for its SIMD tables.

    internal static StateVector SegmentState(in BodySegment s, double time)
    {
        if (s.Dt == 0) return s.A;
        double u = (time - s.T0) / s.Dt;
        // Keep one normalized-time evaluation for both halves of the state.
        var position = s.Quintic
            ? QuinticPosition(in s.A, in s.AccA, in s.B, in s.AccB, s.Dt, u)
            : CubicPosition(in s.A, in s.B, s.Dt, u);
        return new StateVector(position, s.Quintic
            ? QuinticVelocity(in s.A, in s.AccA, in s.B, in s.AccB, s.Dt, u)
            : CubicVelocity(in s.A, in s.B, s.Dt, u));
    }

    internal static Vector3d SegmentPosition(in BodySegment s, double time)
    {
        if (s.Dt == 0) return s.A.Position;
        double u = (time - s.T0) / s.Dt;
        return s.Quintic
            ? QuinticPosition(in s.A, in s.AccA, in s.B, in s.AccB, s.Dt, u)
            : CubicPosition(in s.A, in s.B, s.Dt, u);
    }

    [System.Runtime.CompilerServices.MethodImpl(
        System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    internal static Vector3d CubicPosition(in StateVector a, in StateVector b, double dt, double u)
    {
        double u2 = u * u, u3 = u2 * u;
        double h00 = 2 * u3 - 3 * u2 + 1;
        double h10 = u3 - 2 * u2 + u;
        double h01 = -2 * u3 + 3 * u2;
        double h11 = u3 - u2;
        return a.Position * h00 + a.Velocity * (h10 * dt)
             + b.Position * h01 + b.Velocity * (h11 * dt);
    }

    internal static Vector3d CubicVelocity(in StateVector a, in StateVector b, double dt, double u)
    {
        double u2 = u * u;
        double d00 = (6 * u2 - 6 * u) / dt;
        double d10 = 3 * u2 - 4 * u + 1;
        double d01 = (-6 * u2 + 6 * u) / dt;
        double d11 = 3 * u2 - 2 * u;
        return a.Position * d00 + a.Velocity * d10 + b.Position * d01 + b.Velocity * d11;
    }

    [System.Runtime.CompilerServices.MethodImpl(
        System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    internal static Vector3d QuinticPosition(in StateVector a, in Vector3d accA,
        in StateVector b, in Vector3d accB, double dt, double u)
    {
        double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
        double h0 = 1 - 10 * u3 + 15 * u4 - 6 * u5;
        double h1 = u - 6 * u3 + 8 * u4 - 3 * u5;
        double h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
        double h3 = 0.5 * u3 - u4 + 0.5 * u5;
        double h4 = -4 * u3 + 7 * u4 - 3 * u5;
        double h5 = 10 * u3 - 15 * u4 + 6 * u5;
        return a.Position * h0 + a.Velocity * (h1 * dt) + accA * (h2 * dt * dt)
             + accB * (h3 * dt * dt) + b.Velocity * (h4 * dt) + b.Position * h5;
    }

    internal static Vector3d QuinticVelocity(in StateVector a, in Vector3d accA,
        in StateVector b, in Vector3d accB, double dt, double u)
    {
        double u2 = u * u, u3 = u2 * u, u4 = u3 * u;
        double d0 = (-30 * u2 + 60 * u3 - 30 * u4) / dt;
        double d1 = 1 - 18 * u2 + 32 * u3 - 15 * u4;
        double d2 = (u - 4.5 * u2 + 6 * u3 - 2.5 * u4) * dt;
        double d3 = (1.5 * u2 - 4 * u3 + 2.5 * u4) * dt;
        double d4 = -12 * u2 + 28 * u3 - 15 * u4;
        double d5 = (30 * u2 - 60 * u3 + 30 * u4) / dt;
        return a.Position * d0 + a.Velocity * d1 + accA * d2
             + accB * d3 + b.Velocity * d4 + b.Position * d5;
    }
}

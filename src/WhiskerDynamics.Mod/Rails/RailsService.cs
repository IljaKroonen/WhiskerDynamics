using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Rails;

/// <summary>Owns the mod's celestial rails: one NBodyEphemerides integrated from the
/// live game system's celestial catalog
/// with the game's own constants, always anchored at t=0
/// (the game epoch, JD 2461009.5 by data convention) so trajectories are reproducible
/// across sessions. A background worker keeps the horizon ahead of sim time and prunes
/// behind it. ALL ephemerides/gravity access goes through <see cref="Gate"/> — game job
/// threads query concurrently and WhiskerDynamics.Core is not thread-safe.</summary>
public sealed class RailsService : IDisposable
{
    public object Gate { get; } = new();

    private readonly NBodyEphemerides _ephemerides;
    /// <summary>Worker-thread-only detached growth scratch — see the growth loop.</summary>
    private readonly NBodyEphemerides.DetachedGrower _grower;
    private readonly GravityModel _gravity;
    private readonly CelestialBody[] _vesselGravitySources;
    private readonly CelestialBody[] _thirdBodyBodies;
    private readonly Dictionary<string, int> _thirdBodyBodyIndex;
    private readonly int[] _thirdBodySourceBodyIndices;
    private readonly double[] _thirdBodySourceMu;
    private readonly bool[] _thirdBodyFeelsSource;
    private readonly Dictionary<string, CelestialBody> _bodyById;
    private readonly IReadOnlyDictionary<string, string[]> _soiChildrenByParent;
    private IReadOnlyDictionary<string, Vector3d> _equatorialPoles =
        new Dictionary<string, Vector3d>(StringComparer.Ordinal);
    private IReadOnlyDictionary<string, double> _angularVelocities =
        new Dictionary<string, double>(StringComparer.Ordinal);
    private int _equatorialPoleCaptureStarted;
    private long _nextEquatorialPoleCaptureMs;
    private readonly HashSet<string> _modeledIds;
    private readonly HashSet<string> _backboneIds;
    private readonly HashSet<string> _vesselGravitySourceIds;
    private readonly string[] _curveBodyPriority;
    private readonly string _rootId;
    private readonly ModConfig _config;
    private readonly Thread _worker;
    private readonly bool _workerStarted;
    private readonly CancellationTokenSource _stop = new();
    private readonly object _disposeGate = new();
    private readonly object _thirdBodyRefreshLifecycleGate = new();
    private readonly ManualResetEventSlim _thirdBodyRefreshesDrained = new(initialState: true);
    private readonly long _celestialCurveGeneration;
    private readonly Action<CancellationToken>? _celestialSamplingForTest;
    private int _acceptedThirdBodyRefreshes;
    private volatile bool _stopping;
    private bool _disposeCompleted;
    private const int ShutdownTimeoutMs = 2_000;
    private double _lastSimTime;
    private readonly object _authorityFaultGate = new();
    private Exception? _authorityFailure;
    private Action<RailsService, Exception>? _authorityFaultHandler;
    private long _thirdBodySnapshotBuildCount;
    private long _thirdBodyDynamicsBuildCount;
    private long _thirdBodyDynamicsPairEvaluationCount;
    internal Action<string, double>? ThirdBodyRefreshOwnerBeforeBuildForTest { get; set; }
    internal Action<string, double>? ThirdBodyRefreshFlightAcquiredForTest { get; set; }
    /// <summary>Test-only pause after a new flight is published and its direct-owner
    /// admission is recorded, while the lifecycle gate still excludes Dispose.</summary>
    internal Action? ThirdBodyRefreshLifecycleBoundaryForTest { get; set; }
    private SegmentedEphemeridesSnapshot? _predictionSnapshot;
    private PredictionContext? _predictionContext;
    private double _predictionRequestedFrom = double.PositiveInfinity;
    private double _predictionRequestedTo = double.NegativeInfinity;
    private PredictionSnapshotBuild? _predictionSnapshotBuild;
    private const long PredictionSnapshotBudgetMsPerCycle = 2;
    private const int PredictionSnapshotCycleHandoffMs = 1;
    private const double PredictionSnapshotChunkSeconds =
        NBodyEphemerides.KnotGapCapSeconds;
    internal double PredictionSnapshotChunkSecondsForTest { get; set; } =
        PredictionSnapshotChunkSeconds;
    internal Action<int>? PredictionSnapshotChunkCapturedForTest { get; set; }
    internal Action? PredictionSnapshotBeforePublishForTest { get; set; }
    /// <summary>Test/benchmark observation point for the dominant-attractor folded
    /// read. Invoked once after entering <see cref="Gate"/> and before sampling the
    /// batch; null in production.</summary>
    internal Action? AbsoluteManyGateEnteredForTest { get; set; }

    internal void SetAuthorityFaultHandler(Action<RailsService, Exception> handler)
    {
        ArgumentNullException.ThrowIfNull(handler);
        Exception? replay;
        lock (_authorityFaultGate)
        {
            Volatile.Write(ref _authorityFaultHandler, handler);
            replay = Volatile.Read(ref _authorityFailure);
        }
        if (replay is not null) QueueAuthorityFaultHandler(handler, replay);
    }

    internal void ThrowIfAuthorityFaulted()
    {
        if (Volatile.Read(ref _authorityFailure) is { } failure)
            throw new InvalidOperationException("authoritative rails service faulted", failure);
    }

    internal bool AuthorityFaulted => Volatile.Read(ref _authorityFailure) is not null;

    private void ReportAuthorityFailure(Exception failure)
    {
        Action<RailsService, Exception>? handler;
        lock (_authorityFaultGate)
        {
            if (Volatile.Read(ref _authorityFailure) is not null) return;
            Volatile.Write(ref _authorityFailure, failure);
            handler = Volatile.Read(ref _authorityFaultHandler);
        }
        if (handler is not null) QueueAuthorityFaultHandler(handler, failure);
    }

    private void QueueAuthorityFaultHandler(
        Action<RailsService, Exception> handler, Exception failure) =>
        ThreadPool.UnsafeQueueUserWorkItem(
            static state => state.Handler(state.Service, state.Failure),
            (Handler: handler, Service: this, Failure: failure), preferLocal: false);

    /// <summary>Final activation transaction. Failure latching shares this gate, so a
    /// service cannot become published between a healthy check and a concurrent worker
    /// failure. A later failure observes an already-published binding and is delivered
    /// through the replayable handler.</summary>
    internal void PublishIfAuthorityHealthy(Action publication)
    {
        ArgumentNullException.ThrowIfNull(publication);
        lock (_authorityFaultGate)
        {
            ThrowIfAuthorityFaulted();
            publication();
        }
    }

    internal void ReportAuthorityFailureForTest(Exception failure) =>
        ReportAuthorityFailure(failure);

    private sealed class PredictionSnapshotBuild(double from, double captureTo)
    {
        public readonly double From = from;
        public double CaptureTo = captureTo;
        public double Next = from;
        public readonly List<SegmentedEphemeridesSnapshot> Chunks = [];
    }

    public static RailsService CreateFromGameData(ModConfig config, GameConstants constants) =>
        CreateFromGameData(config, constants, celestialSamplingForTest: null);

    internal static RailsService CreateFromGameData(ModConfig config, GameConstants constants,
        Action<CancellationToken>? celestialSamplingForTest)
    {
        ModConfig.LogRepairs(config.NormalizeWorkload(), "rails service creation");
        // The running system is the only production catalog authority.
        var catalog = LiveCatalog.SnapshotCurrentSystem(out string systemId);
        var bodies = CatalogKernel.Build(catalog, constants.G, out var skipped,
            config.SelectedLunarGravityFidelity);
        if (skipped.Count != 0)
            throw new FormatException("live celestial catalog cannot be modeled completely: "
                + string.Join("; ", skipped));
        string catalogSource = $"live system '{systemId}'";
        ModLog.Info($"lunar gravity model: {config.LunarGravityModel}");
        var root = bodies.Single(b => b.Parent is null);

        // Every eligible finite-mass body joins the mutually coupled system. Zero-mu
        // bodies, and descendants whose ancestor cannot join it, remain numerical
        // restricted tracks.
        var backbone = IntegratedSetRule.Select(bodies, 0.0, out var restricted);
        LogCatalogSelection(catalogSource, bodies, backbone, restricted);
        var ephemerides = new NBodyEphemerides(bodies, 0.0, backbone,
            new IntegratorOptions { RelTol = config.RailsRelTol });
        return new RailsService(config, bodies, ephemerides, backbone, root.Id,
            celestialSamplingForTest, CelestialCurves.CurrentGeneration,
            startWorker: true);
    }

    internal static RailsService CreateForModeledCatalog(
        ModConfig config, IReadOnlyList<CelestialBody> bodies,
        Action<CancellationToken>? celestialSamplingForTest = null)
    {
        var root = bodies.Single(body => body.Parent is null);
        var backbone = IntegratedSetRule.Select(bodies, 0.0, out var restricted);
        LogCatalogSelection("test modeled catalog", bodies, backbone, restricted);
        var ephemerides = new NBodyEphemerides(bodies, 0.0, backbone,
            new IntegratorOptions { RelTol = config.RailsRelTol });
        return new RailsService(config, bodies, ephemerides, backbone, root.Id,
            celestialSamplingForTest, celestialCurveGeneration: 0, startWorker: true);
    }

    private static void LogCatalogSelection(
        string catalogSource,
        IReadOnlyList<CelestialBody> bodies,
        IReadOnlySet<string> backbone,
        IReadOnlyList<RestrictedClassification> restricted)
    {
        ModLog.Info($"rails catalog: {catalogSource}, {bodies.Count} modeled bodies "
            + $"({backbone.Count} mutually coupled, {restricted.Count} restricted)");
        ModLog.Info($"rails mutual backbone: {string.Join(", ", backbone)}");
        foreach (var group in restricted
            .GroupBy(classification => classification.Kind)
            .OrderBy(group => group.Key))
            ModLog.Info($"rails restricted [{RestrictedKind(group.Key)}]: {group.Count()} bodies");

        const int classificationSampleLimit = 32;
        foreach (var classification in restricted
            .OrderBy(classification => classification.Id, StringComparer.Ordinal)
            .Take(classificationSampleLimit))
            ModLog.Info($"rails restricted [{RestrictedKind(classification.Kind)}]: "
                + $"{classification.Id}: {classification.Reason}");
        if (restricted.Count > classificationSampleLimit)
            ModLog.Info($"rails restricted: {restricted.Count - classificationSampleLimit} "
                + "additional classifications omitted");

        static string RestrictedKind(RestrictedClassificationKind kind) => kind switch
        {
            RestrictedClassificationKind.NonBackreacting => "zero-mu",
            RestrictedClassificationKind.Ancestor => "ancestor",
            _ => kind.ToString(),
        };
    }

    /// <summary>Builds the real rails read stack over a caller-owned synthetic
    /// catalog without probing game data or starting background work. Tests and
    /// benchmarks use this seam to exercise <see cref="TryGetAbsoluteMany"/> and its
    /// Gate discipline in a KSA-free process.</summary>
    internal static RailsService CreateForSyntheticCatalog(
        IReadOnlyList<CelestialBody> bodies,
        IReadOnlyCollection<string> backboneIds,
        double startTime = 0.0)
    {
        ArgumentNullException.ThrowIfNull(bodies);
        ArgumentNullException.ThrowIfNull(backboneIds);
        var root = bodies.Single(body => body.Parent is null);
        var backbone = backboneIds.ToHashSet(StringComparer.Ordinal);
        var config = new ModConfig();
        var ephemerides = new NBodyEphemerides(bodies, startTime, backbone,
            new IntegratorOptions { RelTol = config.RailsRelTol });
        return new RailsService(config, bodies, ephemerides, backbone, root.Id,
            celestialSamplingForTest: null, celestialCurveGeneration: 0,
            startWorker: false);
    }

    private RailsService(ModConfig config, IReadOnlyList<CelestialBody> bodies,
        NBodyEphemerides ephemerides, HashSet<string> backboneIds, string rootId,
        Action<CancellationToken>? celestialSamplingForTest,
        long celestialCurveGeneration, bool startWorker)
    {
        _config = config;
        _ephemerides = ephemerides;
        _grower = ephemerides.CreateGrower();
        _modeledIds = bodies.Select(body => body.Id)
            .ToHashSet(StringComparer.Ordinal);
        _backboneIds = new HashSet<string>(backboneIds, StringComparer.Ordinal);
        // Every parented modeled body is curve-eligible, mutually coupled or
        // restricted, including hyperbolic tracks. Bind has already rejected invalid
        // seeds; AdaptiveSampler treats non-periodic tracks as infinite-period.
        // Live catalog:
        // ordered by OverlayKernel.CurvePriority. The live config applies the cap at
        // read time so the settings panel can change the worker budget immediately.
        var eligible = bodies
            .Where(b => b.Parent is not null && b.Orbit is not null)
            .Select(b => (b.Id, b.Mu, Backbone: backboneIds.Contains(b.Id)))
            .ToArray();
        CurveEligibleCount = eligible.Length;
        _curveBodyPriority = [.. OverlayKernel.CurvePriority(eligible, eligible.Length)];
        _rootId = rootId;
        _bodyById = bodies.ToDictionary(b => b.Id);
        // Every modeled nonzero-mu body pulls vessels. A positive-mu descendant of a
        // restricted zero-mu ancestor remains restricted but still exerts its force;
        // zero-mu bodies remain modeled and exert none.
        _vesselGravitySources = SelectVesselGravitySources(bodies);
        _gravity = new GravityModel(ephemerides, _vesselGravitySources);
        _thirdBodyBodies = bodies.ToArray();
        _thirdBodyBodyIndex = _thirdBodyBodies
            .Select((body, index) => (body.Id, index))
            .ToDictionary(pair => pair.Id, pair => pair.index, StringComparer.Ordinal);
        _thirdBodySourceBodyIndices = _vesselGravitySources
            .Select(source => _thirdBodyBodyIndex[source.Id]).ToArray();
        _thirdBodySourceMu = _vesselGravitySources.Select(source => source.Mu).ToArray();
        _thirdBodyFeelsSource = new bool[_thirdBodyBodies.Length * _vesselGravitySources.Length];
        for (int bodyIndex = 0; bodyIndex < _thirdBodyBodies.Length; bodyIndex++)
        {
            int row = bodyIndex * _vesselGravitySources.Length;
            for (int sourceIndex = 0; sourceIndex < _vesselGravitySources.Length; sourceIndex++)
                _thirdBodyFeelsSource[row + sourceIndex] = ephemerides.FeelsGravityFrom(
                    _thirdBodyBodies[bodyIndex], _vesselGravitySources[sourceIndex]);
        }
        _vesselGravitySourceIds = _gravity.Sources.Select(b => b.Id)
            .ToHashSet(StringComparer.Ordinal);
        _soiChildrenByParent = bodies
            .Where(body => body.Parent is not null && _vesselGravitySourceIds.Contains(body.Id)
                && double.IsFinite(body.SphereOfInfluence) && body.SphereOfInfluence > 0)
            .GroupBy(body => body.Parent!.Id, StringComparer.Ordinal)
            .ToDictionary(group => group.Key, group => group.Select(body => body.Id).ToArray(),
                StringComparer.Ordinal);
        // Fixed ownership captured before the worker starts. A rebind advances the
        // CelestialCurves generation; this service can never adopt that later session.
        _celestialCurveGeneration = celestialCurveGeneration;
        _celestialSamplingForTest = celestialSamplingForTest;
        _worker = new Thread(WorkerLoop)
        {
            IsBackground = true,
            Name = "whiskerdynamics-rails",
            Priority = ThreadPriority.BelowNormal,
        };
        _workerStarted = startWorker;
        if (startWorker) _worker.Start();
    }

    internal bool WorkerAliveForTest => _worker.IsAlive;
    internal ThreadPriority WorkerPriorityForTest => _worker.Priority;
    internal long PredictionSnapshotBudgetMsForTest =>
        PredictionSnapshotBudgetMsPerCycle;
    internal int PredictionSnapshotCycleHandoffMsForTest =>
        PredictionSnapshotCycleHandoffMs;
    internal double PredictionRetainedStartForTest
    {
        get { lock (Gate) return _ephemerides.StartTime; }
    }
    internal double PredictionStableThroughForTest
    {
        get { lock (Gate) return _ephemerides.SnapshotStableThrough; }
    }

    internal static CelestialBody[] SelectVesselGravitySources(
        IReadOnlyList<CelestialBody> bodies) =>
        bodies.Where(body => body.Mu > 0.0).ToArray();

    /// <summary>Shared by Seam 1 predictors and Seam 2 — ONE field model for everything.
    /// Every use must hold <see cref="Gate"/>: the shared ephemerides underneath are not thread-safe.</summary>
    public GravityModel VesselGravity
    {
        get
        {
            ThrowIfAuthorityFaulted();
            return _gravity;
        }
    }
    internal long ThirdBodySnapshotBuildCount =>
        Interlocked.Read(ref _thirdBodySnapshotBuildCount);
    internal long ThirdBodyDynamicsBuildCount =>
        Interlocked.Read(ref _thirdBodyDynamicsBuildCount);
    internal long ThirdBodyDynamicsPairEvaluationCount =>
        Interlocked.Read(ref _thirdBodyDynamicsPairEvaluationCount);
    internal int ThirdBodyRefreshPendingCount => _thirdBodyRefreshPending.Count;
    internal int ThirdBodyRefreshFlightCount => _thirdBodyRefreshFlights.Count;
    internal bool StoppingForTest => _stopping;

    public IReadOnlyCollection<string> ModeledIds => _modeledIds;
    public IReadOnlyCollection<string> BackboneIds => _backboneIds;
    /// <summary>Every modeled body that can carry an honest arc: anything with a
    /// parent (root excluded), any eccentricity. Membership is parse-time constant.
    /// Built once in the constructor next to the modeled/backbone sets. Live catalog:
    /// ordered by <see cref="OverlayKernel.CurvePriority"/> (backbone first, µ
    /// descending, ordinal-id ties), with celestial_curve_max_bodies selecting a live
    /// prefix of that stable order.</summary>
    public IReadOnlyList<string> CurveBodyIds
    {
        get
        {
            int count = Math.Min(_curveBodyPriority.Length,
                Math.Clamp(_config.CelestialCurveMaxBodies, 1, 256));
            return count == _curveBodyPriority.Length
                ? _curveBodyPriority
                : new ArraySegment<string>(_curveBodyPriority, 0, count);
        }
    }
    /// <summary>Count of curve-ELIGIBLE parsed bodies BEFORE the cap (the same filter
    /// that feeds <see cref="CurveBodyIds"/>). CelestialCurves compares it against
    /// CurveBodyIds.Count for the one-shot cap log.</summary>
    public int CurveEligibleCount { get; }
    /// <summary>The parsed root body's id (game origin body). Consumers use
    /// it to convert mod-frame absolutes to game-convention positions (GetGameEcl
    /// contract: game position = mod absolute - mod root absolute).</summary>
    public string RootId => _rootId;
    public bool IsModeled(string gameBodyId) => _modeledIds.Contains(gameBodyId);
    public bool IsBackbone(string gameBodyId) => _backboneIds.Contains(gameBodyId);
    public bool IsVesselGravitySource(string gameBodyId) =>
        _vesselGravitySourceIds.Contains(gameBodyId);
    /// <summary>True when <see cref="GetAbsolute"/> can serve this modeled id.
    /// Unknown ids are the only exclusions. Parse-time constant, no Gate.</summary>
    public bool CanEvaluate(string gameBodyId) => _modeledIds.Contains(gameBodyId);
    public double MuOf(string gameBodyId) => _bodyById[gameBodyId].Mu;

    /// <summary>Main-thread-only one-shot: captures live orientation state after a
    /// job-thread caller may already have created this service, then atomically publishes
    /// immutable plain pole data to overlay workers. Wrong-thread calls defer silently;
    /// the main-thread UI calls EnsureBound every frame and completes the capture.</summary>
    internal static bool BeginEquatorialPoleCapture(ref int captureStarted) =>
        BurnPlanWriter.IsMainThread
        && Interlocked.CompareExchange(ref captureStarted, 1, 0) == 0;

    internal static void RearmEquatorialPoleCaptureAfterFailure(
        ref int captureStarted, ref long nextAttemptMs, long nowMs)
    {
        Volatile.Write(ref nextAttemptMs, nowMs + 1000);
        Volatile.Write(ref captureStarted, 0);
    }

    internal void CaptureEquatorialPolesOnMainThread()
    {
        if (Environment.TickCount64 < Volatile.Read(ref _nextEquatorialPoleCaptureMs)) return;
        if (!BeginEquatorialPoleCapture(ref _equatorialPoleCaptureStarted)) return;
        try
        {
            var poles = BodyRotationReader.SnapshotPoles(out var rates, out var diagnostics);
            Volatile.Write(ref _equatorialPoles, poles);
            Volatile.Write(ref _angularVelocities, rates);
            foreach (string diagnostic in diagnostics)
                ModLog.Warn($"equatorial marker pole: {diagnostic}");
        }
        catch (Exception e)
        {
            RearmEquatorialPoleCaptureAfterFailure(
                ref _equatorialPoleCaptureStarted,
                ref _nextEquatorialPoleCaptureMs,
                Environment.TickCount64);
            // Rails remain usable. AN/DN are omitted rather than falling back to a
            // different reference plane and presenting a false node.
            ModLog.Warn($"equatorial marker pole capture failed: {e.Message}");
        }
    }

    /// <summary>Bind-captured unit spin pole in game-ecliptic axes. Immutable plain
    /// data: safe for overlay-worker reads without the rails Gate or game objects.</summary>
    public bool TryGetEquatorialPole(string gameBodyId, out Vector3d pole) =>
        Volatile.Read(ref _equatorialPoles).TryGetValue(gameBodyId, out pole);

    /// <summary>Bind-captured constant body spin rate (rad/s), paired with the pole
    /// snapshot and safe for overlay-worker ground-track analysis.</summary>
    public bool TryGetAngularVelocity(string gameBodyId, out double radiansPerSecond) =>
        Volatile.Read(ref _angularVelocities).TryGetValue(gameBodyId, out radiansPerSecond);

    /// <summary>Mean radius (m) for marker altitude labels; 0 for unknown ids or
    /// bodies whose catalog entry carries none (labels then read as distance from
    /// the body's center — still truthful). Parse-time constant, no Gate.</summary>
    public double MeanRadiusOf(string gameBodyId) =>
        _bodyById.TryGetValue(gameBodyId, out var body) ? body.MeanRadius : 0.0;

    /// <summary>Game SOI radius captured with the live catalog. Synthetic/test
    /// catalogs may carry a finite Laplace estimate. NaN means this body has no
    /// usable closed SOI.</summary>
    public double SphereOfInfluenceOf(string gameBodyId) =>
        _bodyById.TryGetValue(gameBodyId, out var body) ? body.SphereOfInfluence : double.NaN;

    /// <summary>Vessel-gravity-source children whose SOIs can produce the next geometric
    /// encounter while the vessel is parented to <paramref name="parentId"/>.</summary>
    public IReadOnlyList<string> SoiChildrenOf(string parentId) =>
        _soiChildrenByParent.TryGetValue(parentId, out var children) ? children : [];

    internal bool IsSoiChildCandidate(string bodyId, double sphereOfInfluence) =>
        _vesselGravitySourceIds.Contains(bodyId)
        && double.IsFinite(sphereOfInfluence) && sphereOfInfluence > 0;

    /// <summary>Parent body id of a parsed body; null for the root (no orbit line) or an
    /// unknown id. Hierarchy is parse-time constant — no Gate needed.</summary>
    public string? ParentIdOf(string gameBodyId) =>
        _bodyById.TryGetValue(gameBodyId, out var body) ? body.Parent?.Id : null;

    /// <summary>Defining parent-relative Keplerian elements of a parsed body; null for
    /// the root or an unknown id. Feeds the picker's distance-from-primary sibling
    /// ordering (FrameCatalog.SiblingSortKey). Parse-time constant — no Gate.</summary>
    public OrbitalElements? OrbitOf(string gameBodyId) =>
        _bodyById.TryGetValue(gameBodyId, out var body) ? body.Orbit : null;
    public double Horizon
    {
        get
        {
            ThrowIfAuthorityFaulted();
            lock (Gate) return _ephemerides.Horizon;
        }
    }

    public bool IsReadyAt(double time)
    {
        ThrowIfAuthorityFaulted();
        if (!double.IsFinite(time)) return false;
        lock (Gate) return time >= _ephemerides.StartTime && time <= _ephemerides.Horizon;
    }

    /// <summary>Binding readiness barrier: do not publish Active services until the
    /// authoritative celestial state covers the loaded simulation epoch.</summary>
    internal void PrepareAuthorityAt(double time)
    {
        ThrowIfAuthorityFaulted();
        NoteSimTime(time);
        lock (Gate)
        {
            if (time < _ephemerides.StartTime)
                throw new InvalidOperationException("simulation time precedes rails start");
            if (time > _ephemerides.Horizon)
            {
                var body = _bodyById.Values.First();
                _ = _ephemerides.GetState(body, time);
            }
            if (time > _ephemerides.Horizon)
                throw new InvalidOperationException("rails readiness barrier did not reach simulation time");
        }
        ThrowIfAuthorityFaulted();
    }

    private StateVector ReadStateOrThrow(CelestialBody body, double time)
    {
        if (_ephemerides.TryGetState(body, time, out var state)) return state;
        throw new InvalidOperationException();
    }

    /// <summary>THE rails-availability clamp (one home — every display/authoring path
    /// that promises trajectory data must go through it): how many days ahead of
    /// <paramref name="nowSeconds"/> the rails have ACTUALLY integrated, capped at the
    /// config target. While a raised orbits window grows chunk by chunk this trails
    /// the target and the consumers grow with it — sampling or authoring past the
    /// reached horizon would demand a synchronous Gate-held extension instead.</summary>
    public double AvailableAheadDays(double nowSeconds)
    {
        ThrowIfAuthorityFaulted();
        double horizon;
        lock (Gate) horizon = _ephemerides.Horizon;
        return Math.Min(_config.RailsAheadDays, Math.Max(0.0, (horizon - nowSeconds) / 86400.0));
    }

    /// <summary>A lock-free view for one background prediction owner. Capture is short
    /// and Gate-bound; every later body/gravity query is detached from live rails. The
    /// ephemerides snapshot is immutable and shareable, but <see cref="Gravity"/> has
    /// mutable segment caches: the cached context belongs to the serialized overlay
    /// worker, while concurrent solvers must use <see cref="ForkForConcurrentUse"/>.</summary>
    public sealed class PredictionContext
    {
        private readonly SegmentedEphemeridesSnapshot _ephemerides;
        private readonly IReadOnlyDictionary<string, CelestialBody> _bodies;
        private readonly string _rootId;
        private readonly CelestialBody[] _gravitySources;

        internal PredictionContext(SegmentedEphemeridesSnapshot ephemerides,
            IReadOnlyDictionary<string, CelestialBody> bodies, string rootId,
            CelestialBody[] gravitySources)
        {
            _ephemerides = ephemerides;
            _bodies = bodies;
            _rootId = rootId;
            _gravitySources = gravitySources;
            Gravity = new GravityModel(ephemerides, gravitySources);
        }

        public double StartTime => _ephemerides.StartTime;
        public double Horizon => _ephemerides.Horizon;
        public string RootId => _rootId;
        public GravityModel Gravity { get; }

        public StateVector GetAbsolute(string id, double time) =>
            _ephemerides.GetState(_bodies[id], time);

        internal double GetAbsolutePositionSegmentEndAfter(string id, double time) =>
            _ephemerides.PositionSegmentEndAfter(_bodies[id], time);

        public (StateVector A, StateVector B) GetAbsolutePair(string a, string b, double time) =>
            (GetAbsolute(a, time), GetAbsolute(b, time));

        public StateVector[] GetAbsoluteMany(IReadOnlyList<string> ids, double time)
        {
            var states = new StateVector[ids.Count];
            for (int i = 0; i < ids.Count; i++) states[i] = GetAbsolute(ids[i], time);
            return states;
        }

        public StateVector GetGameEcl(string id, double time)
        {
            var body = GetAbsolute(id, time);
            var root = GetAbsolute(_rootId, time);
            return body - root;
        }

        public (StateVector A, StateVector B) GetGameEclPair(string a, string b, double time)
        {
            var root = GetAbsolute(_rootId, time);
            return (GetAbsolute(a, time) - root, GetAbsolute(b, time) - root);
        }

        /// <summary>Shares the immutable ephemerides/body window while giving a
        /// concurrent consumer its own mutable gravity segment cache. A context's
        /// <see cref="Gravity"/> remains single-owner even though its snapshot may be
        /// read by any number of forks.</summary>
        internal PredictionContext ForkForConcurrentUse() =>
            new(_ephemerides, _bodies, _rootId, _gravitySources);
    }

    /// <summary>Returns a covering immutable window, or records a request for the rails
    /// worker and returns null. Display callers retain the historical clamp to the
    /// currently retained/live rails range. Reuse avoids copying once per vessel; the
    /// bounded copy never runs on this vehicle-update caller.</summary>
    public PredictionContext? TryCapturePredictionContext(double fromTime, double toTime)
    {
        ThrowIfAuthorityFaulted();
        ValidatePredictionRange(fromTime, toTime);
        lock (Gate)
        {
            double from = Math.Max(fromTime, _ephemerides.StartTime);
            double to = Math.Min(toTime, _ephemerides.Horizon);
            if (!(to > from)) throw new ArgumentOutOfRangeException(nameof(toTime));
            return TryCapturePredictionContextUnderGate(from, to);
        }
    }

    /// <summary>Returns a job-private prediction context when the cached immutable
    /// window covers the request. A miss uses the same asynchronous rails-worker
    /// request as overlay capture and returns null; no snapshot copying or gravity
    /// construction occurs while <see cref="Gate"/> is held.</summary>
    internal PredictionContext? TryCaptureSolverPredictionContext(
        double fromTime, double toTime)
    {
        ThrowIfAuthorityFaulted();
        ValidatePredictionRange(fromTime, toTime);
        PredictionContext? shared;
        lock (Gate)
        {
            // Solvers seed from these exact endpoints. A retained-history clamp or a
            // live-horizon clamp would manufacture a context smaller than the job's
            // declared domain and merely defer the failure to a background thread.
            if (fromTime < _ephemerides.StartTime || toTime > _ephemerides.Horizon)
                return null;
            shared = TryCapturePredictionContextUnderGate(fromTime, toTime);
        }
        return shared?.ForkForConcurrentUse();
    }

    private static void ValidatePredictionRange(double fromTime, double toTime)
    {
        if (!double.IsFinite(fromTime) || !double.IsFinite(toTime))
            throw new ArgumentOutOfRangeException(nameof(fromTime));
        if (!(toTime > fromTime)) throw new ArgumentOutOfRangeException(nameof(toTime));
    }

    /// <summary>Gate-held cache lookup/request aggregation over already-normalized
    /// exact bounds.</summary>
    private PredictionContext? TryCapturePredictionContextUnderGate(double from, double to)
    {
        var snapshot = _predictionSnapshot;
        if (snapshot is not null && snapshot.StartTime <= from && snapshot.Horizon >= to)
            return _predictionContext;
        _predictionRequestedFrom = Math.Min(_predictionRequestedFrom, from);
        _predictionRequestedTo = Math.Max(_predictionRequestedTo, to);
        return null;
    }

    /// <summary>Rails-worker-only incremental snapshot refresh. Each Gate acquisition
    /// deep-copies at most one day: committed slices use the stable-knot frontier and
    /// the final dense-tail slice is bounded by the same knot-gap cap. Slices accumulate
    /// immutably and the complete composite is built outside the Gate, then published
    /// atomically. A two-millisecond per-cycle budget is the soft burst backstop when
    /// Monitor scheduling is unfair. The below-normal worker relinquishes its remaining
    /// quantum between slices and sleeps for one millisecond when the budget expires,
    /// giving gameplay threads a real handoff window before later worker activity. A
    /// 40-year request advances over successive cycles; one indivisible one-day slice
    /// may overrun the soft wall-clock budget.</summary>
    private void RefreshPredictionSnapshot()
    {
        var budget = System.Diagnostics.Stopwatch.StartNew();
        while (!_stop.IsCancellationRequested)
        {
            PredictionSnapshotBuild? completed;
            bool captured;
            int chunkCount;
            lock (Gate)
                (completed, captured, chunkCount) = CapturePredictionSnapshotChunkUnderGate();

            if (captured) PredictionSnapshotChunkCapturedForTest?.Invoke(chunkCount);
            if (completed is not null) PublishPredictionSnapshot(completed);
            if (!captured) return;
            if (budget.ElapsedMilliseconds >= PredictionSnapshotBudgetMsPerCycle)
            {
                Thread.Sleep(PredictionSnapshotCycleHandoffMs);
                return;
            }
            Thread.Sleep(0);
        }
    }

    /// <summary>Captures one bounded slice and returns the build once all of its
    /// requested coverage (including reuse margin) is present. Caller holds Gate.</summary>
    private (PredictionSnapshotBuild? Completed, bool Captured, int ChunkCount)
        CapturePredictionSnapshotChunkUnderGate()
    {
        if (!(_predictionRequestedTo > _predictionRequestedFrom))
            return (null, false, 0);

        double from = Math.Max(_predictionRequestedFrom, _ephemerides.StartTime);
        double to = Math.Min(_predictionRequestedTo, _ephemerides.Horizon);
        if (!(to > from))
        {
            if (_predictionRequestedTo <= _ephemerides.StartTime)
            {
                // A warp/prune moved the whole queued display request out of
                // retention. Drop its partial immutable chunks; the next display
                // frame will enqueue its newly clamped live window.
                _predictionRequestedFrom = double.PositiveInfinity;
                _predictionRequestedTo = double.NegativeInfinity;
                _predictionSnapshotBuild = null;
            }
            return (null, false, 0);
        }
        double margin = Math.Max(3600.0, (to - from) * 0.05);
        double captureTo = Math.Min(_ephemerides.Horizon, to + margin);

        var build = _predictionSnapshotBuild;
        if (build is null || from < build.From
            || build.Next < _ephemerides.StartTime)
        {
            // Retention may advance while a very large build spans worker cycles.
            // Already-copied chunks are immutable, but if the next uncopied instant
            // was pruned there is no contiguous completion path; restart at the same
            // clamp a fresh display caller observes. Exact solver callers reject the
            // now-unavailable original bound before consulting this cache.
            build = new PredictionSnapshotBuild(from, captureTo);
            _predictionSnapshotBuild = build;
        }
        else if (captureTo > build.CaptureTo)
        {
            build.CaptureTo = captureTo;
        }

        if (build.Next >= build.CaptureTo)
            return (build, false, build.Chunks.Count);

        double configuredChunk = PredictionSnapshotChunkSecondsForTest;
        double chunkSeconds = double.IsFinite(configuredChunk) && configuredChunk > 0
            ? Math.Min(configuredChunk, PredictionSnapshotChunkSeconds)
            : PredictionSnapshotChunkSeconds;
        double stableThrough = Math.Min(
            _ephemerides.SnapshotStableThrough, build.CaptureTo);
        double chunkTo;
        if (build.Next < stableThrough)
        {
            // Every body's interpolation is committed in this slice, so later tip
            // commits cannot change its representation.
            chunkTo = Math.Min(stableThrough,
                Math.Min(build.CaptureTo, build.Next + chunkSeconds));
        }
        else
        {
            // This is the only uncommitted slice. CommitKnots guarantees that the
            // shared dense tail is no longer than KnotGapCapSeconds; copy it in one
            // atomic Gate hold so a later commit cannot split its representation.
            chunkTo = build.CaptureTo;
        }

        var chunk = _ephemerides.CreateSnapshot(build.Next, chunkTo);
        build.Chunks.Add(chunk);
        build.Next = chunkTo;
        return (build.Next >= build.CaptureTo ? build : null, true, build.Chunks.Count);
    }

    /// <summary>Joins immutable slice references without Gate, then atomically swaps
    /// the published cache if this is still the worker's active build.</summary>
    private void PublishPredictionSnapshot(PredictionSnapshotBuild build)
    {
        var snapshot = SegmentedEphemeridesSnapshot.Combine(build.Chunks);
        var context = new PredictionContext(
            snapshot, _bodyById, _rootId, _vesselGravitySources);
        PredictionSnapshotBeforePublishForTest?.Invoke();
        lock (Gate)
        {
            if (!ReferenceEquals(_predictionSnapshotBuild, build)) return;
            _predictionSnapshot = snapshot;
            _predictionContext = context;
            _predictionSnapshotBuild = null;

            double from = Math.Max(_predictionRequestedFrom, _ephemerides.StartTime);
            double to = Math.Min(_predictionRequestedTo, _ephemerides.Horizon);
            if (!(to > from)
                || (snapshot.StartTime <= from && snapshot.Horizon >= to))
            {
                _predictionRequestedFrom = double.PositiveInfinity;
                _predictionRequestedTo = double.NegativeInfinity;
            }
        }
    }

    /// <summary>Point query on a caller-owned predictor with BOUNDED Gate holds: a far
    /// target (a burn years out on a fast orbit — legal under long rails windows)
    /// extends in trajectory-time chunks, releasing the <see cref="Gate"/> between
    /// them so rails readers (physics substeps included) interleave instead of
    /// stalling for the whole grind. Only the predictor's RHS touches the shared
    /// ephemerides — the predictor itself stays the caller's, single-threaded. The
    /// chunk is sized like the growth loop's: a few ms of low-orbit integration per
    /// hold, frame-invisible; TrajectoryPredictor.MaxNodes still backstops the
    /// total.</summary>
    public StateVector ChunkedStateAt(TrajectoryPredictor predictor, double time)
    {
        ThrowIfAuthorityFaulted();
        const double ChunkSeconds = 3 * 86400.0;
        while (predictor.Horizon < time)
        {
            double next = Math.Min(time, predictor.Horizon + ChunkSeconds);
            lock (Gate) predictor.ExtendTo(next);
            Thread.Yield(); // see the growth loop: give blocked readers a window
        }
        lock (Gate) return predictor.StateAt(time);
    }

    /// <summary>Absolute mod-frame state of any modeled body. Throws for unknown ids
    /// or unavailable authoritative state.</summary>
    public StateVector GetAbsolute(string gameBodyId, double time)
    {
        ThrowIfAuthorityFaulted();
        var body = _bodyById[gameBodyId];
        lock (Gate) return ReadStateOrThrow(body, time);
    }

    public bool TryGetAbsolute(string gameBodyId, double time, out StateVector state)
    {
        ThrowIfAuthorityFaulted();
        state = default;
        if (!_bodyById.TryGetValue(gameBodyId, out var body)) return false;
        lock (Gate) return _ephemerides.TryGetState(body, time, out state);
    }

    /// <summary>Two bodies' absolute mod-frame states at one time under a SINGLE
    /// <see cref="Gate"/> acquisition (Gate discipline: fold multi-body ephemerides
    /// reads instead of stacking one acquisition per body). Throws like
    /// <see cref="GetAbsolute"/>.</summary>
    public (StateVector A, StateVector B) GetAbsolutePair(string aId, string bId, double time)
    {
        ThrowIfAuthorityFaulted();
        var a = _bodyById[aId];
        var b = _bodyById[bId];
        lock (Gate) return (ReadStateOrThrow(a, time), ReadStateOrThrow(b, time));
    }

    /// <summary>Absolute mod-frame states for a batch of ids at one time under a SINGLE
    /// <see cref="Gate"/> acquisition — the folded read for candidate sweeps (SOI
    /// re-parenting samples a parent plus every eligible child). Throws like
    /// <see cref="GetAbsolute"/>.</summary>
    public StateVector[] GetAbsoluteMany(IReadOnlyList<string> ids, double time)
    {
        ThrowIfAuthorityFaulted();
        var states = new StateVector[ids.Count];
        lock (Gate)
            for (int i = 0; i < ids.Count; i++)
                states[i] = ReadStateOrThrow(_bodyById[ids[i]], time);
        return states;
    }

    public bool TryGetAbsoluteMany(IReadOnlyList<CelestialBody> bodies, double time,
        out StateVector[] states)
    {
        ThrowIfAuthorityFaulted();
        states = new StateVector[bodies.Count];
        lock (Gate)
        {
            AbsoluteManyGateEnteredForTest?.Invoke();
            for (int i = 0; i < bodies.Count; i++)
                if (!_ephemerides.TryGetState(bodies[i], time, out states[i]))
                {
                    states = [];
                    return false;
                }
        }
        return true;
    }

    /// <summary>Game-world-convention state of a body: the mod's absolute state minus
    /// the root's. The game pins the root (Sol) at the origin while the mod's frame
    /// carries the root's barycentric wobble; relative geometry is identical, so
    /// game position == mod absolute − mod root absolute. Display frames MUST use this
    /// (an F1 frame built on raw GetAbsolute would displace everything by the root
    /// wobble). Throws for unknown ids (contained by callers).</summary>
    public StateVector GetGameEcl(string gameBodyId, double time)
    {
        ThrowIfAuthorityFaulted();
        var body = _bodyById[gameBodyId];
        var root = _bodyById[_rootId];
        lock (Gate)
        {
            var bodyState = ReadStateOrThrow(body, time);
            var rootState = ReadStateOrThrow(root, time);
            return new StateVector(bodyState.Position - rootState.Position,
                bodyState.Velocity - rootState.Velocity);
        }
    }

    /// <summary>Two bodies' game-convention states at one time under a SINGLE
    /// <see cref="Gate"/> acquisition with the root sampled once — the folded read for
    /// rotating-frame poses (Gate discipline: fold multi-body ephemerides reads instead
    /// of stacking one acquisition per body). Throws like <see cref="GetGameEcl"/>.</summary>
    public (StateVector A, StateVector B) GetGameEclPair(string aId, string bId, double time)
    {
        ThrowIfAuthorityFaulted();
        var a = _bodyById[aId];
        var b = _bodyById[bId];
        var root = _bodyById[_rootId];
        lock (Gate)
        {
            var rootState = ReadStateOrThrow(root, time);
            var aState = ReadStateOrThrow(a, time);
            var bState = ReadStateOrThrow(b, time);
            return (new StateVector(aState.Position - rootState.Position, aState.Velocity - rootState.Velocity),
                new StateVector(bState.Position - rootState.Position, bState.Velocity - rootState.Velocity));
        }
    }

    /// <summary>Modeled non-root bodies only: exact parent-relative state in Ecl axes.
    /// This is the live celestial-authority seam, so it synchronously closes any small
    /// worker-horizon gap before reading. False only for unknown ids or the root;
    /// numerical growth failures propagate to the fail-closed caller.</summary>
    public bool TryGetParentRelativeEcl(string gameBodyId, double time,
        out Vector3d position, out Vector3d velocity)
    {
        ThrowIfAuthorityFaulted();
        position = default;
        velocity = default;
        if (!_modeledIds.Contains(gameBodyId)) return false;
        var body = _bodyById[gameBodyId];
        // The root's mod-frame wobble must never be applied game-side (game origin stays Sol).
        if (body.Parent is null) return false;
        NoteSimTime(time);
        lock (Gate)
        {
            // GetState performs the transactional safety-net growth when the background
            // worker has not yet spliced this exact staged time. Keep growth and both
            // reads in one Gate transaction so pruning cannot open a readiness gap.
            var child = _ephemerides.GetState(body, time);
            var parent = _ephemerides.GetState(body.Parent, time);
            position = child.Position - parent.Position;
            velocity = child.Velocity - parent.Velocity;
        }
        return true;
    }

    /// <summary>Full multi-body acceleration at an absolute mod-frame position.</summary>
    public Vector3d Acceleration(Vector3d absolutePosition, double time)
    {
        ThrowIfAuthorityFaulted();
        lock (Gate)
        {
            if (time < _ephemerides.StartTime || time > _ephemerides.Horizon)
                throw new InvalidOperationException();
            return _gravity.AccelerationAt(absolutePosition, time);
        }
    }

    /// <summary>Seam 2 HOT path (ComputeDerivatives: every physics substep, job threads).
    /// Third-body correction for a point at
    /// <paramref name="parentRelativePositionEcl"/> from the modeled parent — the
    /// same mixed direct/tidal correction <see cref="GravityModel.ThirdBodyDeltaAt"/> defines over the ONE
    /// shared <see cref="VesselGravity"/> source set, but served from a per-parent
    /// view over shared exact-time body dynamics: cache hits take no lock and allocate nothing;
    /// the <see cref="Gate"/> is acquired only to refresh a snapshot older than
    /// <see cref="ThirdBodySnapshotToleranceSeconds"/>. Positions advance quadratically
    /// from relative velocity and acceleration; an analytic jerk estimate refreshes
    /// whenever the omitted remainder exceeds a fixed fraction of vessel-source distance.
    /// Callers gate on <see cref="IsModeled"/>; unknown ids throw (contained).</summary>
    public Vector3d ThirdBodyDelta(string parentGameBodyId, Vector3d parentRelativePositionEcl, double time)
    {
        ThrowIfAuthorityFaulted();
        // Some exact-reference and predictor callers already own Gate. They must not
        // join a flight whose owner may itself be waiting to capture ephemeris state
        // under Gate: that lock inversion would strand both callers. A fresh snapshot
        // remains the allocation-free fast path; a miss evaluates the exact shared field
        // while the caller's reentrant Gate ownership makes that evaluation safe.
        if (Monitor.IsEntered(Gate))
        {
            if (_thirdBodyCache.TryGetValue(parentGameBodyId, out var cached)
                && Math.Abs(time - cached.Time) <= ThirdBodySnapshotToleranceSeconds
                && !CacheRemainderTooLarge(cached, parentRelativePositionEcl, time))
                return EvaluateThirdBodySnapshot(cached, parentRelativePositionEcl, time);
            return ExactThirdBodyDeltaUnderGate(
                parentGameBodyId, parentRelativePositionEcl, time);
        }
        if (!TryGetThirdBodySnapshot(parentGameBodyId, parentRelativePositionEcl, time,
                waitForRefresh: true, out var snapshot))
            throw new InvalidOperationException();
        return EvaluateThirdBodySnapshot(snapshot, parentRelativePositionEcl, time);
    }

    /// <summary>Exact cache-miss evaluation for a caller that already owns
    /// <see cref="Gate"/>. Joining a refresh flight here can deadlock when its owner is
    /// waiting for this Gate, while calling an extending ephemeris API can silently grow
    /// the rails horizon. Validate the current retained window in place, read only
    /// non-extending states, and reproduce the snapshot evaluator's point-mass plus
    /// non-parent extended-gravity terms in the same source order.</summary>
    private Vector3d ExactThirdBodyDeltaUnderGate(string parentGameBodyId,
        Vector3d parentRelativePosition, double time)
    {
        if (!Monitor.IsEntered(Gate))
            throw new InvalidOperationException("exact third-body evaluation requires Rails Gate");
        if (!double.IsFinite(time)
            || time < _ephemerides.StartTime || time > _ephemerides.Horizon)
            throw new InvalidOperationException("third-body query is outside the retained rails window");

        var parent = _bodyById[parentGameBodyId];
        if (!_modeledIds.Contains(parent.Id))
            throw new ArgumentException(
                "third-body evaluation requires a modeled parent", nameof(parentGameBodyId));
        Vector3d parentPosition = ReadStateOrThrow(parent, time).Position;
        var delta = _gravity.ParentRelativeCorrectionAt(
            parent, parentRelativePosition, time);
        foreach (var body in _gravity.Sources)
        {
            if (ReferenceEquals(body, parent)) continue;
            Vector3d parentToBody = ReadStateOrThrow(body, time).Position - parentPosition;
            if (body.Geopotential is { } field)
                delta += GravityModel.ExtendedBodyDirectTerm(
                    field, body.Mu, parentToBody, parentRelativePosition, time);
        }
        return delta;
    }

    private static Vector3d EvaluateThirdBodySnapshot(
        ThirdBodySnapshot snapshot, Vector3d parentRelativePositionEcl, double time)
    {
        var delta = Vector3d.Zero;
        double dt = time - snapshot.Time;
        double halfDt2 = 0.5 * dt * dt;
        var dynamics = snapshot.Dynamics;
        int parentIndex = snapshot.ParentBodyIndex;
        var parentState = dynamics.States[parentIndex];
        var parentAcceleration = dynamics.Accelerations[parentIndex];
        int feelsRow = parentIndex * dynamics.SourceBodyIndices.Length;
        for (int i = 0; i < dynamics.SourceBodyIndices.Length; i++)
        {
            int bodyIndex = dynamics.SourceBodyIndices[i];
            if (bodyIndex == parentIndex) continue;
            var bodyState = dynamics.States[bodyIndex];
            var parentToBody = bodyState.Position - parentState.Position
                + (bodyState.Velocity - parentState.Velocity) * dt
                + (dynamics.Accelerations[bodyIndex] - parentAcceleration) * halfDt2;
            delta += dynamics.SourceMu[i] * (dynamics.FeelsSource[feelsRow + i]
                ? GravityModel.TidalTerm(parentToBody, parentRelativePositionEcl)
                : GravityModel.DirectPointMassTerm(parentToBody, parentRelativePositionEcl));
            if (dynamics.SourceGeopotential[i] is { } field)
                delta += GravityModel.ExtendedBodyDirectTerm(
                    field, dynamics.SourceMu[i], parentToBody, parentRelativePositionEcl, time);
        }
        return delta;
    }

    private static bool CacheRemainderTooLarge(
        ThirdBodySnapshot snapshot, Vector3d parentRelativePosition, double time)
    {
        double dt = time - snapshot.Time;
        if (dt == 0) return false;
        double halfDt2 = 0.5 * dt * dt;
        var dynamics = snapshot.Dynamics;
        int parentIndex = snapshot.ParentBodyIndex;
        var parentState = dynamics.States[parentIndex];
        var parentAcceleration = dynamics.Accelerations[parentIndex];
        var parentJerk = dynamics.Jerks[parentIndex];
        for (int i = 0; i < dynamics.SourceBodyIndices.Length; i++)
        {
            int bodyIndex = dynamics.SourceBodyIndices[i];
            if (bodyIndex == parentIndex) continue;
            var bodyState = dynamics.States[bodyIndex];
            var curvature = (dynamics.Accelerations[bodyIndex] - parentAcceleration) * halfDt2;
            var parentToBody = bodyState.Position - parentState.Position
                + (bodyState.Velocity - parentState.Velocity) * dt + curvature;
            double distance = (parentToBody - parentRelativePosition).Length();
            var estimatedRemainder = (dynamics.Jerks[bodyIndex] - parentJerk)
                * (dt * dt * dt / 6.0);
            if (!double.IsFinite(distance)
                || !Finite(estimatedRemainder)
                || estimatedRemainder.Length()
                    > MaxCachedRemainderFraction * Math.Max(distance, 1.0))
                return true;
        }
        return false;

        static bool Finite(Vector3d v) =>
            double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
    }

    /// <summary>Complete vessel-only perturbation on top of stock parent point-mass
    /// gravity: n-body tides plus the parent's optional extended-body field.</summary>
    public Vector3d VesselPerturbation(
        string parentGameBodyId, Vector3d parentRelativePositionEcl, double time)
    {
        var perturbation = ThirdBodyDelta(parentGameBodyId, parentRelativePositionEcl, time);
        var parent = _bodyById[parentGameBodyId];
        return parent.Geopotential is { } field
            ? perturbation + field.AccelerationCorrection(parentRelativePositionEcl, parent.Mu, time)
            : perturbation;
    }

    public bool TryVesselPerturbation(string parentGameBodyId,
        Vector3d parentRelativePositionEcl, double time, out Vector3d perturbation)
    {
        ThrowIfAuthorityFaulted();
        if (Monitor.IsEntered(Gate))
        {
            perturbation = VesselPerturbation(
                parentGameBodyId, parentRelativePositionEcl, time);
            return true;
        }
        perturbation = default;
        if (!TryGetThirdBodySnapshot(parentGameBodyId, parentRelativePositionEcl, time,
                waitForRefresh: false, out var snapshot)) return false;
        perturbation = EvaluateThirdBodySnapshot(snapshot, parentRelativePositionEcl, time);
        var parent = _bodyById[parentGameBodyId];
        if (parent.Geopotential is { } field)
            perturbation += field.AccelerationCorrection(parentRelativePositionEcl, parent.Mu, time);
        return true;
    }

    /// <summary>Schedules the immutable live-force snapshot before a tracked vessel
    /// can transition to live ownership. This never builds on the caller's thread.</summary>
    internal void RequestThirdBodyRefresh(string parentGameBodyId, double time)
    {
        if (!_bodyById.ContainsKey(parentGameBodyId) || !IsReadyAt(time)) return;
        QueueThirdBodyRefresh(parentGameBodyId, Vector3d.Zero, time);
    }

    internal bool HasThirdBodySnapshot(string parentGameBodyId, double time) =>
        Volatile.Read(ref _authorityFailure) is null
        && _thirdBodyCache.TryGetValue(parentGameBodyId, out var snapshot)
        && Math.Abs(time - snapshot.Time) <= ThirdBodySnapshotToleranceSeconds;

    private ThirdBodySnapshot SnapshotThirdBodies(string parentGameBodyId, double time)
    {
        Interlocked.Increment(ref _thirdBodySnapshotBuildCount);
        int parentBodyIndex = _thirdBodyBodyIndex[parentGameBodyId];
        return new ThirdBodySnapshot
        {
            Dynamics = SnapshotThirdBodyDynamics(time),
            ParentBodyIndex = parentBodyIndex,
        };
    }

    private ThirdBodyDynamicsSnapshot SnapshotThirdBodyDynamics(double time)
    {
        var cached = Volatile.Read(ref _thirdBodyDynamicsCache);
        if (cached is not null && cached.Time == time) return cached;

        lock (_thirdBodyDynamicsGate)
        {
            cached = Volatile.Read(ref _thirdBodyDynamicsCache);
            if (cached is not null && cached.Time == time) return cached;

            var states = new StateVector[_thirdBodyBodies.Length];
            lock (Gate)
            {
                for (int i = 0; i < _thirdBodyBodies.Length; i++)
                {
                    if (!_ephemerides.TryGetState(_thirdBodyBodies[i], time, out states[i]))
                        throw new InvalidOperationException();
                }
            }

            var accelerations = new Vector3d[_thirdBodyBodies.Length];
            var jerks = new Vector3d[_thirdBodyBodies.Length];
            int sourceCount = _thirdBodySourceBodyIndices.Length;
            for (int bodyIndex = 0; bodyIndex < _thirdBodyBodies.Length; bodyIndex++)
            {
                var state = states[bodyIndex];
                var acceleration = Vector3d.Zero;
                var jerk = Vector3d.Zero;
                int feelsRow = bodyIndex * sourceCount;
                for (int sourceIndex = 0; sourceIndex < sourceCount; sourceIndex++)
                {
                    if (!_thirdBodyFeelsSource[feelsRow + sourceIndex]) continue;
                    var sourceState = states[_thirdBodySourceBodyIndices[sourceIndex]];
                    acceleration += _thirdBodySourceMu[sourceIndex]
                        * GravityModel.DirectPointMassTerm(
                            sourceState.Position - state.Position, Vector3d.Zero);
                    jerk += _thirdBodySourceMu[sourceIndex] * PointMassJerk(
                        sourceState.Position - state.Position,
                        sourceState.Velocity - state.Velocity);
                }
                accelerations[bodyIndex] = acceleration;
                jerks[bodyIndex] = jerk;
            }

            var built = new ThirdBodyDynamicsSnapshot
            {
                Time = time,
                States = states,
                Accelerations = accelerations,
                Jerks = jerks,
                SourceBodyIndices = _thirdBodySourceBodyIndices,
                SourceMu = _thirdBodySourceMu,
                SourceGeopotential = _vesselGravitySources
                    .Select(source => source.Geopotential).ToArray(),
                FeelsSource = _thirdBodyFeelsSource,
            };
            Interlocked.Increment(ref _thirdBodyDynamicsBuildCount);
            Interlocked.Add(ref _thirdBodyDynamicsPairEvaluationCount,
                (long)_thirdBodyBodies.Length * sourceCount);
            Volatile.Write(ref _thirdBodyDynamicsCache, built);
            return built;
        }

        static Vector3d PointMassJerk(Vector3d offset, Vector3d relativeVelocity)
        {
            double r2 = offset.LengthSquared();
            double r = Math.Sqrt(r2);
            double invR3 = 1.0 / (r2 * r);
            return (relativeVelocity
                - offset * (3.0 * offset.Dot(relativeVelocity) / r2)) * invR3;
        }
    }

    /// <summary>Immutable, published whole via the concurrent map.</summary>
    private sealed class ThirdBodySnapshot
    {
        public required ThirdBodyDynamicsSnapshot Dynamics { get; init; }
        public required int ParentBodyIndex { get; init; }
        public double Time => Dynamics.Time;
    }

    /// <summary>One immutable exact-time generation. Parent snapshots retain their
    /// generation after the service publishes a newer time.</summary>
    private sealed class ThirdBodyDynamicsSnapshot
    {
        public required double Time { get; init; }
        public required StateVector[] States { get; init; }
        public required Vector3d[] Accelerations { get; init; }
        public required Vector3d[] Jerks { get; init; }
        public required int[] SourceBodyIndices { get; init; }
        public required double[] SourceMu { get; init; }
        public required Geopotential?[] SourceGeopotential { get; init; }
        public required bool[] FeelsSource { get; init; }
    }

    private readonly record struct ThirdBodyRefreshOutcome(
        ThirdBodySnapshot? Snapshot, Exception? Failure, bool Canceled)
    {
        public static ThirdBodyRefreshOutcome Succeeded(ThirdBodySnapshot snapshot) =>
            new(snapshot, null, false);
        public static ThirdBodyRefreshOutcome Failed(Exception failure) =>
            new(null, failure, false);
        public static ThirdBodyRefreshOutcome Cancelled => new(null, null, true);
    }

    /// <summary>One ephemeral per-parent refresh generation. Completion always carries
    /// an explicit non-faulted outcome, so an owner failure cannot leave an unobserved
    /// faulted Task. Asynchronous continuations keep waiter work off the publishing
    /// rails or physics caller.</summary>
    private sealed class ThirdBodyRefreshFlight
    {
        public TaskCompletionSource<ThirdBodyRefreshOutcome> Completion { get; } =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
    }

    private const double ThirdBodySnapshotToleranceSeconds = 1.0;
    private const double MaxCachedRemainderFraction = 1e-6;
    private readonly object _thirdBodyDynamicsGate = new();
    private ThirdBodyDynamicsSnapshot? _thirdBodyDynamicsCache;
    private readonly System.Collections.Concurrent.ConcurrentDictionary<string, ThirdBodySnapshot> _thirdBodyCache = new();
    private readonly System.Collections.Concurrent.ConcurrentDictionary<string, ThirdBodyRefreshFlight> _thirdBodyRefreshFlights = new(StringComparer.Ordinal);
    private readonly System.Collections.Concurrent.ConcurrentDictionary<string, byte> _thirdBodyRefreshPending = new();

    private bool TryGetThirdBodySnapshot(string parentGameBodyId,
        Vector3d parentRelativePosition, double time, bool waitForRefresh,
        out ThirdBodySnapshot snapshot, bool refreshAlreadyAccepted = false)
    {
        if (Monitor.IsEntered(Gate))
            throw new InvalidOperationException(
                "third-body snapshot refresh cannot run while the Rails Gate is held");
        if (TryReadUsableThirdBodySnapshot(
                parentGameBodyId, parentRelativePosition, time, out snapshot)) return true;
        if (!IsReadyAt(time)) return false;
        if (!waitForRefresh)
        {
            QueueThirdBodyRefresh(parentGameBodyId, parentRelativePosition, time);
            return _thirdBodyCache.TryGetValue(parentGameBodyId, out snapshot!)
                && Math.Abs(time - snapshot.Time) <= ThirdBodySnapshotToleranceSeconds
                && !CacheRemainderTooLarge(snapshot, parentRelativePosition, time);
        }

        while (true)
        {
            if (TryReadUsableThirdBodySnapshot(
                    parentGameBodyId, parentRelativePosition, time, out snapshot)) return true;
            if (!IsReadyAt(time)) return false;

            var candidate = new ThirdBodyRefreshFlight();
            if (!TryAcquireThirdBodyRefreshFlight(parentGameBodyId, candidate,
                    refreshAlreadyAccepted, out var flight, out bool ownsFlight,
                    out bool completeDirectAdmission, out Exception? acquisitionFailure))
                throw new OperationCanceledException();
            ThirdBodyRefreshOutcome outcome;
            if (ownsFlight)
            {
                try
                {
                    if (acquisitionFailure is not null) throw acquisitionFailure;
                    // This hook is deliberately inside the owner's cleanup envelope:
                    // even a throwing test callback completes and removes the flight.
                    ThirdBodyRefreshFlightAcquiredForTest?.Invoke(parentGameBodyId, time);
                    if (_stop.IsCancellationRequested)
                    {
                        outcome = ThirdBodyRefreshOutcome.Cancelled;
                    }
                    else
                    {
                        ThirdBodyRefreshOwnerBeforeBuildForTest?.Invoke(
                            parentGameBodyId, time);
                        if (_stop.IsCancellationRequested)
                            outcome = ThirdBodyRefreshOutcome.Cancelled;
                        else
                        {
                            var built = SnapshotThirdBodies(parentGameBodyId, time);
                            _thirdBodyCache[parentGameBodyId] = built;
                            outcome = ThirdBodyRefreshOutcome.Succeeded(built);
                        }
                    }
                }
                catch (Exception e)
                {
                    outcome = e is OperationCanceledException
                            && _stop.IsCancellationRequested
                        ? ThirdBodyRefreshOutcome.Cancelled
                        : ThirdBodyRefreshOutcome.Failed(e);
                }

                // Remove this exact generation before waking waiters. A waiter whose
                // time is outside this result's window can elect the next generation
                // without an ABA removal race or a completed-flight spin.
                RemoveThirdBodyRefreshFlight(parentGameBodyId, flight);
                flight.Completion.TrySetResult(outcome);
                if (completeDirectAdmission) CompleteThirdBodyRefresh();
            }
            else
            {
                ThirdBodyRefreshFlightAcquiredForTest?.Invoke(parentGameBodyId, time);
                outcome = flight.Completion.Task.GetAwaiter().GetResult();
            }

            if (outcome.Canceled) throw new OperationCanceledException();
            if (outcome.Failure is { } failure)
            {
                System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(failure).Throw();
                throw failure;
            }
            if (outcome.Snapshot is { } refreshed
                && Math.Abs(time - refreshed.Time) <= ThirdBodySnapshotToleranceSeconds
                && !CacheRemainderTooLarge(refreshed, parentRelativePosition, time))
            {
                snapshot = refreshed;
                return true;
            }
            // This successful shared build belonged to another request window. Loop
            // so every caller in this later window converges on one next generation.
        }
    }

    private bool TryReadUsableThirdBodySnapshot(string parentGameBodyId,
        Vector3d parentRelativePosition, double time, out ThirdBodySnapshot snapshot) =>
        _thirdBodyCache.TryGetValue(parentGameBodyId, out snapshot!)
        && Math.Abs(time - snapshot.Time) <= ThirdBodySnapshotToleranceSeconds
        && !CacheRemainderTooLarge(snapshot, parentRelativePosition, time);

    private void RemoveThirdBodyRefreshFlight(
        string parentGameBodyId, ThirdBodyRefreshFlight flight) =>
        ((ICollection<KeyValuePair<string, ThirdBodyRefreshFlight>>)_thirdBodyRefreshFlights)
            .Remove(new KeyValuePair<string, ThirdBodyRefreshFlight>(parentGameBodyId, flight));

    private bool TryAcquireThirdBodyRefreshFlight(string parentGameBodyId,
        ThirdBodyRefreshFlight candidate, bool refreshAlreadyAccepted,
        out ThirdBodyRefreshFlight flight, out bool ownsFlight,
        out bool completeDirectAdmission, out Exception? acquisitionFailure)
    {
        lock (_thirdBodyRefreshLifecycleGate)
        {
            if (_thirdBodyRefreshFlights.TryGetValue(parentGameBodyId, out flight!))
            {
                ownsFlight = false;
                completeDirectAdmission = false;
                acquisitionFailure = null;
                return true;
            }
            if (!refreshAlreadyAccepted && _stopping)
            {
                flight = candidate;
                ownsFlight = false;
                completeDirectAdmission = false;
                acquisitionFailure = null;
                return false;
            }

            if (!_thirdBodyRefreshFlights.TryAdd(parentGameBodyId, candidate))
                throw new InvalidOperationException();
            flight = candidate;
            ownsFlight = true;
            completeDirectAdmission = !refreshAlreadyAccepted;
            if (completeDirectAdmission) AcceptThirdBodyRefreshUnderLifecycleGate();

            // Publication and direct admission are now one lifecycle-gate transaction.
            // A throwing boundary callback is returned to the owner's normal cleanup
            // path so this test seam can never strand a published flight.
            try
            {
                ThirdBodyRefreshLifecycleBoundaryForTest?.Invoke();
                acquisitionFailure = null;
            }
            catch (Exception e)
            {
                acquisitionFailure = e;
            }
            return true;
        }
    }

    private void QueueThirdBodyRefresh(string parentGameBodyId,
        Vector3d parentRelativePosition, double time)
    {
        if (!_thirdBodyRefreshPending.TryAdd(parentGameBodyId, 0)) return;
        if (!TryAcceptThirdBodyRefresh())
        {
            _thirdBodyRefreshPending.TryRemove(parentGameBodyId, out _);
            return;
        }

        bool queued = false;
        try
        {
            queued = ThreadPool.UnsafeQueueUserWorkItem(state =>
            {
                try
                {
                    if (!_stop.IsCancellationRequested)
                        TryGetThirdBodySnapshot(parentGameBodyId, parentRelativePosition, time,
                            waitForRefresh: true, out _, refreshAlreadyAccepted: true);
                }
                catch (OperationCanceledException) when (_stop.IsCancellationRequested)
                {
                }
                catch (Exception e)
                {
                    ReportAuthorityFailure(e);
                }
                finally
                {
                    _thirdBodyRefreshPending.TryRemove(parentGameBodyId, out _);
                    CompleteThirdBodyRefresh();
                }
            }, null);
        }
        finally
        {
            if (!queued)
            {
                _thirdBodyRefreshPending.TryRemove(parentGameBodyId, out _);
                CompleteThirdBodyRefresh();
            }
        }
    }

    private bool TryAcceptThirdBodyRefresh()
    {
        lock (_thirdBodyRefreshLifecycleGate)
        {
            if (_stopping) return false;
            AcceptThirdBodyRefreshUnderLifecycleGate();
            return true;
        }
    }

    private void AcceptThirdBodyRefreshUnderLifecycleGate()
    {
        if (++_acceptedThirdBodyRefreshes == 1)
            _thirdBodyRefreshesDrained.Reset();
    }

    private void CompleteThirdBodyRefresh()
    {
        lock (_thirdBodyRefreshLifecycleGate)
        {
            if (--_acceptedThirdBodyRefreshes == 0)
                _thirdBodyRefreshesDrained.Set();
        }
    }

    public void NoteSimTime(double simSeconds)
    {
        ThrowIfAuthorityFaulted();
        if (!double.IsFinite(simSeconds))
            throw new ArgumentOutOfRangeException(nameof(simSeconds),
                "Simulation time must be finite.");
        lock (Gate) if (simSeconds > _lastSimTime) _lastSimTime = simSeconds;
    }

    /// <summary>The one retention-boundary conversion used by the worker. A positive
    /// keep-behind duration can only move the cutoff backward from the present.</summary>
    internal static double RetentionCutoffSeconds(double nowSeconds, double keepBehindDays) =>
        nowSeconds - keepBehindDays * ModConfig.SecondsPerDay;

    /// <summary>Catch-up chunk size (days of horizon per detached-growth round) and
    /// the per-cycle growth budget. The INTEGRATION runs off the Gate entirely
    /// (NBodyEphemerides.DetachedGrower); the Gate is held only for the seed capture
    /// (microseconds) and the splice+knot-commitment for a quarter-day chunk,
    /// so growth occupies the Gate ~a quarter of the burst in few-ms slices — a
    /// game-thread rails read loses a small slice of one frame at worst. An earlier
    /// in-Gate design starved readers a chunk at a time at ~full duty and read as a
    /// hitch per step during long catch-ups.</summary>
    private const double GrowthChunkDays = 0.25;
    private const long GrowthBudgetMsPerCycle = 350;
    private const long GrowthLogPeriodMs = 5000;
    private long _lastGrowthLogMs; // worker-thread only

    private void WorkerLoop()
    {
        while (!_stop.IsCancellationRequested)
        {
            bool catchingUp = false;
            try
            {
                double now;
                lock (Gate) now = _lastSimTime;
                // RailsAheadDays is re-read every cycle from the SHARED config instance.
                // Celestial ephemerides always stay at least 40 display-years ahead,
                // regardless of a shorter in-game orbits-window request.
                // Catch-up toward a raised target is DETACHED-CHUNKED: the integration
                // (~16 ms per day of horizon at shipping scale) runs off the Gate on
                // this thread; only the seed capture and the splice+knot-commitment
                // hold it briefly per quarter-day chunk, so readers interleave freely
                // however large the window jump. The loop stops when the
                // per-cycle budget is spent. Maintenance runs between bursts, then
                // catch-up resumes immediately while the worker remains behind.
                // Display windows
                // clamp to the horizon actually reached, so lines grow live while the
                // worker catches up. Steady state (target moved by one cycle of sim
                // time) is a single sub-chunk round.
                double predictionAheadDays = Math.Max(_config.RailsAheadDays, 40.0 * 365.0);
                double target = now + predictionAheadDays * 86400;
                var growth = System.Diagnostics.Stopwatch.StartNew();
                double horizon;
                int knots = 0;
                long approxBytes = 0;
                lock (Gate) horizon = _ephemerides.Horizon;
                while (horizon < target && !_stop.IsCancellationRequested)
                {
                    // Detached growth: integrate the chunk OFF the Gate, splice it on
                    // under a few-ms hold. A refused splice means a reader's safety-net
                    // extension moved the tip mid-chunk — discard and re-capture.
                    double seedTime;
                    lock (Gate) seedTime = _grower.CaptureSeed();
                    _grower.Integrate(Math.Min(target, seedTime + GrowthChunkDays * 86400), _stop.Token);
                    lock (Gate)
                    {
                        _grower.TrySplice();
                        horizon = _ephemerides.Horizon;
                        knots = _ephemerides.KnotCount;      // telemetry captured under the
                        approxBytes = _ephemerides.ApproxBytes; // Gate: readers can extend too
                    }
                    if (growth.ElapsedMilliseconds >= GrowthBudgetMsPerCycle) break;
                    // Lock acquisition is not fair: a yield between chunks gives blocked
                    // rails readers a scheduling window inside a growth burst instead of
                    // gambling on winning the reacquisition race.
                    Thread.Yield();
                }
                if (horizon < target && Environment.TickCount64 - _lastGrowthLogMs >= GrowthLogPeriodMs)
                {
                    _lastGrowthLogMs = Environment.TickCount64;
                    ModLog.Info($"rails growing: {(horizon - now) / 86400.0:F1} of "
                        + $"{predictionAheadDays:F0} d ahead "
                        + $"({knots} knots, ~{approxBytes / (1024 * 1024)} MB)");
                }
                catchingUp = horizon < target;
                lock (Gate)
                    _ephemerides.Prune(RetentionCutoffSeconds(
                        now, _config.RailsKeepBehindDays));
                RefreshPredictionSnapshot();
                // Honest orbit lines: celestial curve sampling (~1 Hz internally,
                // always-on while enabled; frame mode iff a frame is active). Strictly
                // OUTSIDE the Gate: the sampling path takes it internally per lookup —
                // holding it across an adaptive multi-point sweep would starve the
                // render thread. Never throws (contains locally).
                if (_celestialSamplingForTest is { } sampleForTest)
                    sampleForTest(_stop.Token);
                else
                    CelestialCurves.MaybeSample(this, _config,
                        _celestialCurveGeneration, _stop.Token);
            }
            catch (OperationCanceledException) when (_stop.IsCancellationRequested)
            {
                break;
            }
            catch (Exception e)
            {
                ReportAuthorityFailure(e);
                break;
            }
            if (!catchingUp && _stop.Token.WaitHandle.WaitOne(500)) break;
        }
    }

    public void Dispose()
    {
        var shutdown = System.Diagnostics.Stopwatch.StartNew();
        lock (_disposeGate)
        {
            if (_disposeCompleted) return;
            lock (_thirdBodyRefreshLifecycleGate) _stopping = true;
            _stop.Cancel();
            // A worker-initiated stop cannot join itself. Leave completion unset so
            // a later external Dispose still performs the authoritative join.
            if (_workerStarted && ReferenceEquals(Thread.CurrentThread, _worker)) return;
            if (_workerStarted && !_worker.Join(RemainingShutdownMilliseconds(shutdown)))
                throw new TimeoutException(
                    $"rails worker did not stop within {ShutdownTimeoutMs} ms");
            if (!_thirdBodyRefreshesDrained.Wait(RemainingShutdownMilliseconds(shutdown)))
                throw new TimeoutException(
                    $"rails background refreshes did not stop within {ShutdownTimeoutMs} ms");
            // Synchronize with the final callback's Set before disposing the event.
            lock (_thirdBodyRefreshLifecycleGate)
            {
                if (_acceptedThirdBodyRefreshes != 0)
                    throw new InvalidOperationException(
                        "rails refresh drain signaled with work still accepted");
                _thirdBodyRefreshesDrained.Dispose();
            }
            _disposeCompleted = true;
            _stop.Dispose();
        }
    }

    private static int RemainingShutdownMilliseconds(
        System.Diagnostics.Stopwatch shutdown)
    {
        long remaining = ShutdownTimeoutMs - shutdown.ElapsedMilliseconds;
        return (int)Math.Clamp(remaining, 0, ShutdownTimeoutMs);
    }
}

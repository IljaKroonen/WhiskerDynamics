using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning.Periapsis;

/// <summary>Allocation-free rollback decision for the optimizer's two stock writes.
/// The UI performs the writes; this KSA-free seam makes every authority checkpoint's
/// transaction consequence explicit and offline-testable.</summary>
[Flags]
internal enum OptimizeApplyRollback
{
    None = 0,
    Time = 1,
    DeltaV = 2,
}

internal static class OptimizeApplyPolicy
{
    internal static OptimizeApplyRollback ForAuthorityLoss(
        bool timeWritten, bool deltaVWritten) =>
        (timeWritten ? OptimizeApplyRollback.Time : OptimizeApplyRollback.None) |
        (deltaVWritten ? OptimizeApplyRollback.DeltaV : OptimizeApplyRollback.None);

    /// <summary>KSA-free final time-warp gate. A finite burn belongs to the flight
    /// computer from ignition, not from its centered maneuver-node time, so apply
    /// authority is measured from the solver's accepted model start.</summary>
    internal static bool ModeledStartHasLead(
        double modeledStartSeconds, double nowSeconds, double minLeadSeconds) =>
        double.IsFinite(modeledStartSeconds)
        && double.IsFinite(nowSeconds)
        && double.IsFinite(minLeadSeconds)
        && minLeadSeconds >= 0.0
        && modeledStartSeconds > nowSeconds + minLeadSeconds;
}

/// <summary>KSA-free ordering around one strict fixed-burn conversion. The production
/// pre-extension is bounded and polls cancellation between chunks; only after it has
/// reached the burn time may RelState perform its now-local point query.</summary>
internal static class PeriapsisFixedConversionOrchestration
{
    internal static T Run<T>(
        Func<bool> stopped,
        Action keepAlive,
        Action preExtend,
        Func<T> convert)
    {
        ThrowIfStopped(stopped);
        keepAlive();
        preExtend();
        ThrowIfStopped(stopped);
        keepAlive();
        return convert();
    }

    private static void ThrowIfStopped(Func<bool> stopped)
    {
        if (stopped())
            throw new OperationCanceledException("optimizer cancelled");
    }
}

/// <summary>The constrained burn optimize as a background job: minimize the LAST
/// burn's total |dv| at fixed periapsis, then improve optional inclination within
/// a bounded delta-v premium. Inputs are
/// captured on the MAIN thread as plain data — the solver thread never touches game
/// objects. Propagation uses a private gravity cache over an immutable celestial
/// snapshot, so the thousands of objective reads never acquire the live Rails Gate. The
/// dominant per-candidate cost is amortized by a SPLIT predictor: one base display
/// predictor carries the trajectory with every OTHER burn folded (candidate-invariant
/// — the target is the last burn and candidate times stay past the others), and each
/// candidate only integrates a cheap clone seeded at the burn time with the candidate
/// dv applied through the objective measurement. Long integrations retain bounded
/// chunks for cancellation and cooperative scheduling. Results are published through
/// the volatile <see cref="Done"/> flag;
/// the panel applies them on the main thread through BurnPlanWriter with its own
/// staleness guards (plan shape, vessel, predictor seed lineage, warp).</summary>
internal sealed class PeriapsisSolveJob
{
    /// <summary>Probe/bracket bounds for the inner constrained-component solve: 1 m/s initial
    /// probe (fine burns stay fine), doubling out to ±4096 m/s (past any single
    /// plan-correction burn; a transfer that needs more is not a local tweak).</summary>
    private const double ProbeStepMps = 1.0;
    private const double MaxOffsetMps = 4096.0;

    /// <summary>Outer pattern-search scales: normal/outward start at 8 m/s and
    /// refine to 0.01 m/s; the time step starts at min(600 s, a quarter of the
    /// movable window) and refines to 1 s.</summary>
    private const double InitialDvStepMps = 8.0;
    private const double DvStepFloorMps = 0.01;
    private const double InitialTimeStepCapSeconds = 600.0;
    private const double TimeStepFloorSeconds = 1.0;

    /// <summary>Hard ceiling on trajectory integrations for one solve — the
    /// worst-case compass search (256 iterations × 6 probes × a full inner
    /// bracket each) is three orders of magnitude past any useful budget, and an
    /// unattended background thread must not saturate a core for minutes. Hitting
    /// the budget stops the search at its best on-constraint point (reported as
    /// such), it does not discard the work.</summary>
    private const int EvaluationBudget = 4000;

    /// <summary>Inclination is secondary to periapsis and may spend only a local
    /// correction premium over the minimum-Pe solution: 25%, with a useful 25 m/s
    /// floor and a hard 250 m/s ceiling.</summary>
    private const double InclinationPremiumFraction = 0.25;
    private const double InclinationPremiumFloorMps = 25.0;
    private const double InclinationPremiumCeilingMps = 250.0;

    /// <summary>Historical base-predictor chunk span, retained so detachment does not
    /// change integration boundaries and cancellation stays responsive.</summary>
    private const double ExtendChunkSeconds = 6.0 * 3600.0;

    public required TrackedVessel Tracked { get; init; }
    /// <summary>The authoritative predictor instance at capture — the seed lineage.
    /// A reseed (live-physics dip, save/load) replaces the instance, so the panel
    /// refuses to apply when the reference no longer matches: the solve optimized a
    /// trajectory that no longer exists.</summary>
    public required TrajectoryPredictor SeedLineage { get; init; }
    public required RailsService.PredictionContext Prediction { get; init; }
    public required StateVector SeedState { get; init; }
    public required string TargetBodyId { get; init; }
    /// <summary>Center-distance Pe (m) and optional inclination (rad), with tolerances.</summary>
    public required double TargetPeriapsis { get; init; }
    public required double PeriapsisTolerance { get; init; }
    public required double? TargetInclination { get; init; }
    public required double InclinationTolerance { get; init; }
    public required Vector3d EquatorialPole { get; init; }
    public required double NowSeconds { get; init; }
    /// <summary>min(plan end, rails-ahead window, rails horizon) at capture — the
    /// scan bound. The rails horizon only ever grows, so the captured value stays
    /// valid; re-reading it live would let a mid-solve extension change the
    /// objective between probes of the same search.</summary>
    public required double HorizonSeconds { get; init; }
    /// <summary>Movable range for the burn time (past the previous burn and the
    /// minimum lead; the top end includes the authored baseline time even when it
    /// sits close to the horizon).</summary>
    public required double TimeLo { get; init; }
    public required double TimeHi { get; init; }
    public required double BaselineTime { get; init; }
    public required Vector3d BaselineDvVlf { get; init; }
    /// <summary>Every burn EXCEPT the target, times fixed, with the main-thread
    /// resolved basis parent per burn (BurnParentId walks game patches).</summary>
    public required (double Time, Vector3d DvVlf, string BasisParentId)[] OtherBurns { get; init; }
    /// <summary>Basis parent for the TARGET burn, resolved at the baseline time.
    /// Candidate times reuse it (the pre-burn patch); the panel refuses the apply
    /// when the patch parent at the SOLVED time disagrees (an SOI boundary moved
    /// under the burn — the solved VLF numbers would execute in a different basis).</summary>
    public required string TargetBasisParentId { get; init; }
    /// <summary>The display fold's finite-burn model (engine scalars + slice
    /// config), or null for the impulsive fold (finite estimation off / engine
    /// unusable). The objective MUST predict the trajectory the flight computer
    /// will fly — for a multi-minute burn the impulsive and finite arrival
    /// geometries diverge grossly (a lunar Pe misses by hundreds of km), so the
    /// candidate burn expands into the FC's centered thrust arc exactly like the
    /// drawn planned line. The written VLF numbers stay impulsive-defined (stock's
    /// semantic); this is about which trajectory those numbers PRODUCE.</summary>
    public required FiniteBurnFold? Finite { get; init; }

    private volatile string _statusLine = "optimizing...";
    private volatile bool _stop;
    private volatile bool _cancelRequested;
    private volatile bool _budgetExhausted;
    private volatile bool _done;
    private int _evaluations;
    private long _nextOverlayKeepAliveMs;
    private OverlayBuffer.LineLease? _lineLease;
    private SolverPrediction _solverPrediction = null!;

    public string StatusLine => _statusLine;
    public bool Done => _done;
    public bool Cancelled => _cancelRequested;
    public bool BudgetExhausted => _budgetExhausted;
    public void Cancel()
    {
        _cancelRequested = true;
        _stop = true;
    }

    // Written on the solver thread before the volatile _done store publishes them.
    public DvMinimum? Result { get; private set; }
    /// <summary>Start of the physical model accepted for <see cref="Result"/>:
    /// finite ignition for every active nonzero finite model (including an intentional
    /// K=1 objective impulse), otherwise the no-model/zero-dv node time.
    /// Published with the result so the main-thread apply gate cannot mistake a
    /// future node for a finite burn whose ignition has already entered the lead
    /// window. NaN until a final candidate is re-admitted successfully.</summary>
    public double AcceptedModelStartSeconds { get; private set; } = double.NaN;
    public string? Failure { get; private set; }
    public int Evaluations => _evaluations;
    public long ElapsedMs { get; private set; }

    public void Start()
    {
        _lineLease = OverlayBuffer.BeginLineLease(
            Tracked.Id, OverlayWorker.CurrentGeneration, Environment.TickCount64);
        new Thread(Run)
        {
            IsBackground = true,
            Name = "whiskerdynamics-peopt",
            Priority = ThreadPriority.BelowNormal,
        }.Start();
    }

    private void Run()
    {
        var stopwatch = System.Diagnostics.Stopwatch.StartNew();
        try
        {
            RunCore();
        }
        catch (OperationCanceledException)
        {
            Failure = "optimizer cancelled - nothing applied";
        }
        catch (ArgumentOutOfRangeException)
        {
            // Capture guarantees the advertised range. A failure therefore means
            // a stale/malformed job; contain it as a retry instead of publishing.
            Failure = "optimizer: detached prediction window was unavailable - try again";
        }
        catch (Exception e)
        {
            Failure = $"optimizer failed: {e.Message}";
            ModLog.Warn($"planner: periapsis optimize contained: {e}");
        }
        finally
        {
            if (_lineLease is { } lease) OverlayBuffer.EndLineLease(lease);
            ElapsedMs = stopwatch.ElapsedMilliseconds;
            _done = true;
        }
    }

    private void RunCore()
    {
        var rails = Tracked.Rails;

        // Base predictor: the display trajectory with every FIXED burn folded —
        // candidate-invariant and wholly owned by this detached solver.
        _solverPrediction = new SolverPrediction(Prediction, () => _stop);
        var display = new TrajectoryPredictor(
            _solverPrediction.Gravity, SeedState, NowSeconds,
            new IntegratorOptions { RelTol = 1e-9 });

        void KeepOverlayAlive()
        {
            long now = Environment.TickCount64;
            if (now < _nextOverlayKeepAliveMs) return;
            _nextOverlayKeepAliveMs = now + 1_000;
            if (_lineLease is { } lease) OverlayBuffer.RenewLineLease(lease, now);
        }

        ExtendChunked(display, OtherBurns.Length > 0 ? OtherBurns[0].Time : TimeLo);
        if (_stop) { Failure = "optimizer cancelled - nothing applied"; return; }
        // One strict sequential pipeline: conversion sees the predictor after every
        // earlier admitted burn, and mass/window state commits only after the exact
        // admitted impulses land. There is no display-style impulse fallback here.
        var fixedFold = PeriapsisStrictFold.Fold(
            OtherBurns,
            NowSeconds + PlannerKernel.MinLeadSeconds,
            HorizonSeconds,
            Finite,
            index =>
            {
                var burn = OtherBurns[index];
                return PeriapsisFixedConversionOrchestration.Run(
                    () => _stop,
                    KeepOverlayAlive,
                    () => ExtendChunked(display, burn.Time),
                    () =>
                    {
                        var (rRel, vRel) = _solverPrediction.RelativeState(
                            display, burn.BasisParentId, burn.Time, 3 * 86400.0);
                        return BurnFrameKernel.VlfToEcl(burn.DvVlf, rRel, vRel);
                    });
            },
            display.AddImpulse);
        if (!fixedFold.Success)
        {
            if (_stop)
            {
                Failure = "optimizer cancelled - nothing applied";
                return;
            }
            Failure = "optimizer: "
                + (fixedFold.Failure ?? "fixed-burn fold failed safely");
            return;
        }
        EngineScalars engineAtTarget = fixedFold.EngineAtTarget;
        double othersLastBound = fixedFold.LastBoundSeconds;
        ExtendChunked(display, TimeHi);
        if (_stop) { Failure = "optimizer cancelled - nothing applied"; return; }

        string? lastFailure = null;

        // One candidate: clone the base at the burn time with the dv applied and
        // find the first periapsis past it. Null = refused (reason kept for the
        // final report). Contained per candidate — one diverged probe must not
        // kill the whole solve, the search just avoids it.
        (double Periapsis, double? Inclination, bool Accepted)? Evaluate(
            double burnTime, Vector3d dvVlf, bool measureInclination = false)
        {
            if (_stop) return null;
            KeepOverlayAlive();
            if (Interlocked.Increment(ref _evaluations) > EvaluationBudget)
            {
                lastFailure = $"evaluation budget exhausted ({EvaluationBudget} integrations)";
                _budgetExhausted = true;
                _stop = true;
                return null;
            }
            try
            {
                if (burnTime <= NowSeconds + PlannerKernel.MinLeadSeconds)
                { lastFailure = "rejected: burn time is not ahead of now"; return null; }
                if (burnTime < othersLastBound)
                { lastFailure = "rejected: burn overlaps the preceding finite burn"; return null; }
                if (HorizonSeconds <= burnTime + 1.0)
                { lastFailure = "no window after the burn - extend the plan length"; return null; }

                var (rRel, vRel) = _solverPrediction.RelativeState(
                    display, TargetBasisParentId, burnTime, 3 * 86400.0);
                if (BurnFrameKernel.VlfToEcl(dvVlf, rRel, vRel) is not { } dvEcl)
                { lastFailure = "degenerate VLF basis at the burn time"; return null; }

                // The candidate burn folds the way the FC will FLY it. Unlike the
                // display pipeline, the objective may not silently replace a
                // material finite arc with a node impulse: typed admission either
                // accepts the intentional no-model/K<=1 impulse, accepts the whole
                // finite window, or rejects before either predictor branch below.
                // Duration is defined by the authored VLF magnitude. Use that same
                // scalar here and in final publication so re-admission is bit-stable;
                // normalize the transformed vector separately for slice direction.
                double magnitude = dvVlf.Length();
                var admission = PeriapsisFiniteAdmission.Decide(
                    burnTime, magnitude, Finite, engineAtTarget,
                    othersLastBound, HorizonSeconds);
                FiniteBurnExpansion? expansion;
                switch (admission.Kind)
                {
                    case PeriapsisFiniteAdmissionKind.Impulsive:
                        if (!admission.TryGetAcceptedExpansion(out _))
                        {
                            lastFailure = "rejected: impulsive admission has no physical window";
                            return null;
                        }
                        expansion = null;
                        break;
                    case PeriapsisFiniteAdmissionKind.Finite:
                        if (!admission.TryGetAcceptedExpansion(out var acceptedExpansion)
                            || acceptedExpansion is null)
                        {
                            lastFailure = "rejected: finite admission returned no expansion";
                            return null;
                        }
                        expansion = acceptedExpansion;
                        break;
                    case PeriapsisFiniteAdmissionKind.RejectWindowStart:
                    case PeriapsisFiniteAdmissionKind.RejectHorizon:
                    case PeriapsisFiniteAdmissionKind.RejectUnmodelable:
                        lastFailure = admission.Failure
                            ?? "rejected: finite burn could not be admitted";
                        return null;
                    default:
                        lastFailure = "rejected: unknown finite-burn admission result";
                        return null;
                }
                if (admission.ModelStartSeconds is not { } modelStart
                    || admission.ModelEndSeconds is not { } modelEnd
                    || !double.IsFinite(modelStart)
                    || !double.IsFinite(modelEnd))
                {
                    lastFailure = "rejected: admitted burn has no safe physical window";
                    return null;
                }
                TrajectoryPredictor clone;
                double scanStart;
                double postBurnTime;
                if (expansion is not null)
                {
                    var preIgnition = _solverPrediction.StateAt(
                        display, modelStart, 3 * 86400.0);
                    clone = new TrajectoryPredictor(_solverPrediction.Gravity, preIgnition,
                        modelStart, new IntegratorOptions { RelTol = 1e-9 });
                    double transformedMagnitude = dvEcl.Length();
                    if (!(transformedMagnitude > 0.0)
                        || !double.IsFinite(transformedMagnitude))
                    {
                        lastFailure = "degenerate finite-burn direction";
                        return null;
                    }
                    var direction = dvEcl * (1.0 / transformedMagnitude);
                    for (int s = 0; s < expansion.Times.Length; s++)
                        clone.AddImpulse(expansion.Times[s], direction * expansion.Magnitudes[s]);
                    // Scan from IGNITION, not the cutoff: a capture burn centered
                    // near a flyby periapsis has its true closest approach INSIDE
                    // the thrust arc — the drawn line's Pe marker detects it there,
                    // and the objective must agree with what the map reports.
                    scanStart = modelStart + 1.0;
                    postBurnTime = modelEnd + 1.0;
                }
                else
                {
                    var preBurn = _solverPrediction.StateAt(
                        display, burnTime, 3 * 86400.0);
                    clone = new TrajectoryPredictor(_solverPrediction.Gravity,
                        preBurn with { Velocity = preBurn.Velocity + dvEcl }, burnTime,
                        new IntegratorOptions { RelTol = 1e-9 });
                    scanStart = burnTime + 1.0;
                    // K=1 is intentionally represented by a node impulse, but the
                    // FC still owns its physical centered thrust interval.
                    postBurnTime = modelEnd + 1.0;
                }
                double requiredTime = measureInclination
                    ? Math.Max(postBurnTime, scanStart) : scanStart;
                if (HorizonSeconds <= requiredTime)
                { lastFailure = "no window after the burn - extend the plan length"; return null; }

                // Sequential samples extend only this private clone; the helper keeps
                // the prior three-day cadence and checks cancellation between chunks.
                double DistanceAt(double t)
                {
                    KeepOverlayAlive();
                    return _solverPrediction.RelativeState(
                        clone, TargetBodyId, t, 3 * 86400.0).RRel.Length();
                }
                var (postR, postV) = _solverPrediction.RelativeState(
                    clone, TargetBodyId, scanStart, 3 * 86400.0);
                double period = AdaptiveSampler.PeriodSeconds(rails.MuOf(TargetBodyId), postR, postV);
                double scanEnd = double.IsFinite(period) && period > 0
                    ? Math.Min(HorizonSeconds, scanStart + 2.0 * period)
                    : HorizonSeconds;
                var (periapsis, interior) =
                    PeriapsisKernel.ScanFirstPeriapsis(DistanceAt, scanStart, scanEnd);
                if (!double.IsFinite(periapsis))
                { lastFailure = "trajectory prediction diverged for a candidate dv"; return null; }
                double? inclination = null;
                if (measureInclination)
                {
                    var state = _solverPrediction.RelativeState(
                        clone, TargetBodyId, postBurnTime, 3 * 86400.0);
                    inclination = PeriapsisKernel.InclinationRadians(
                        state.RRel, state.VRel, EquatorialPole);
                    if (inclination is null) return null;
                }
                return (periapsis, inclination, interior);
            }
            catch (Exception e)
            {
                lastFailure = $"optimizer evaluation failed: {e.Message}";
                return null;
            }
        }

        // Inner projection: solve prograde so the point HITS the target periapsis.
        // The window-edge fallback keeps the bracketing objective continuous exactly
        // like the 1-D optimizer, but an accepted point must be a REAL pass within
        // tolerance — anything else is a non-move for the outer search. The warm
        // start is the kernel's hint: always the current BEST point's prograde
        // (the baseline's on the first call), never a rejected probe's, so the
        // search cannot wander onto a different constraint branch.
        (double Constrained, double Achieved)? SolvePeAt(
            double time, double normal, double outward, double progradeHint)
        {
            if (_stop) return null;
            double x0 = double.IsNaN(progradeHint) ? BaselineDvVlf.X : progradeHint;
            bool lastAccepted = false;
            double? EvaluatePrograde(double prograde)
            {
                var evaluated = Evaluate(time, new Vector3d(prograde, normal, outward));
                if (evaluated is not { } value) return null;
                lastAccepted = value.Accepted;
                return value.Periapsis;
            }
            var solved = PeriapsisKernel.SolveForTarget(
                EvaluatePrograde, x0, TargetPeriapsis,
                ProbeStepMps, MaxOffsetMps, PeriapsisTolerance, () => _stop);
            if (solved is not { } result) return null;
            if (EvaluatePrograde(result.X) is not { } achieved || !lastAccepted
                || Math.Abs(achieved - TargetPeriapsis) > PeriapsisTolerance)
                return null;
            _statusLine = $"optimizing: {_evaluations} integrations...";
            return (result.X, achieved);
        }

        double inclinationProgradeSeed = BaselineDvVlf.X;
        (double Prograde, double AchievedPeriapsis, double AchievedInclination)?
            SolvePeAndInclinationAt(double time, double normal, double outward, double hint)
        {
            if (_stop || TargetInclination is null) return null;
            double x0 = double.IsNaN(hint) ? inclinationProgradeSeed : hint;
            double? Pe(double prograde) =>
                Evaluate(time, new Vector3d(prograde, normal, outward))?.Periapsis;
            var solved = PeriapsisKernel.SolveForTarget(
                Pe, x0, TargetPeriapsis, ProbeStepMps, MaxOffsetMps,
                PeriapsisTolerance, () => _stop);
            if (solved is not { } result) return null;
            var measured = Evaluate(time, new Vector3d(result.X, normal, outward),
                measureInclination: true);
            if (measured is not { Accepted: true, Inclination: { } inclination })
                return null;
            if (Math.Abs(measured.Value.Periapsis - TargetPeriapsis) > PeriapsisTolerance)
                return null;
            return (result.X, measured.Value.Periapsis, inclination);
        }
        double initialTimeStep = Math.Max(2.0 * TimeStepFloorSeconds,
            Math.Min(InitialTimeStepCapSeconds, (TimeHi - TimeLo) / 4.0));
        DvMinimum? best = PeriapsisKernel.MinimizeDeltaV(SolvePeAt,
            BaselineTime, BaselineDvVlf.Y, BaselineDvVlf.Z,
            TimeLo, TimeHi, initialTimeStep, InitialDvStepMps,
            TimeStepFloorSeconds, DvStepFloorMps, () => _stop);
        if (best is not null && TargetInclination is { } targetI && !_stop)
        {
            inclinationProgradeSeed = best.Prograde;
            double premium = Math.Min(InclinationPremiumCeilingMps,
                Math.Max(InclinationPremiumFloorMps,
                    InclinationPremiumFraction * best.Magnitude));
            best = PeriapsisKernel.ImproveInclinationAtFixedPeriapsis(
                SolvePeAndInclinationAt,
                best.TimeSeconds, best.Normal, best.Outward, targetI, InclinationTolerance,
                best.Magnitude + premium, TimeLo, TimeHi,
                initialTimeStep, InitialDvStepMps,
                TimeStepFloorSeconds, DvStepFloorMps, () => _stop) ?? best;
        }

        if (Cancelled)
        {
            Failure = "optimizer cancelled - nothing applied";
            return;
        }
        if (best is null)
        {
            Failure = lastFailure is not null
                ? $"optimizer: {lastFailure}"
                : $"optimizer: no dv within ±{MaxOffsetMps:F0} m/s reaches the target "
                    + "Pe at the initial burn time (try another burn time or extend the plan)";
            return;
        }

        // Re-admit the exact result that will be published. Search probes already
        // passed the same policy, but this closes any future path that constructs a
        // best point without Evaluate and gives apply the authoritative ignition or
        // node start. Any inconsistency fails closed.
        var finalAdmission = PeriapsisFiniteAdmission.Decide(
            best.TimeSeconds, best.Magnitude, Finite, engineAtTarget,
            othersLastBound, HorizonSeconds);
        switch (finalAdmission.Kind)
        {
            case PeriapsisFiniteAdmissionKind.Impulsive:
            case PeriapsisFiniteAdmissionKind.Finite:
                if (!finalAdmission.TryGetAcceptedExpansion(out _)
                    || finalAdmission.ModelStartSeconds is not { } acceptedStart
                    || finalAdmission.ModelEndSeconds is not { } acceptedEnd
                    || !double.IsFinite(acceptedStart)
                    || !double.IsFinite(acceptedEnd))
                {
                    Failure = "optimizer: final burn has no safe physical execution window";
                    return;
                }
                // Includes the physical ignition of an intentional K=1 impulse.
                AcceptedModelStartSeconds = acceptedStart;
                break;
            case PeriapsisFiniteAdmissionKind.RejectWindowStart:
            case PeriapsisFiniteAdmissionKind.RejectHorizon:
            case PeriapsisFiniteAdmissionKind.RejectUnmodelable:
                Failure = "optimizer: final candidate "
                    + (finalAdmission.Failure ?? "failed finite-burn admission");
                return;
            default:
                Failure = "optimizer: final candidate has an unknown finite-burn admission result";
                return;
        }
        if (!double.IsFinite(AcceptedModelStartSeconds))
        {
            Failure = "optimizer: final modeled burn start is not finite";
            return;
        }
        Result = best;
    }

    /// <summary>Extends <paramref name="predictor"/> to <paramref name="to"/> in the
    /// historical six-hour cadence. Slices now bound cancellation and line-lease
    /// latency; the private predictor never acquires Rails Gate.</summary>
    private void ExtendChunked(TrajectoryPredictor predictor, double to)
    {
        for (double t = NowSeconds; t < to && !_stop; t += ExtendChunkSeconds)
        {
            long now = Environment.TickCount64;
            if (now >= _nextOverlayKeepAliveMs)
            {
                _nextOverlayKeepAliveMs = now + 1_000;
                if (_lineLease is { } lease) OverlayBuffer.RenewLineLease(lease, now);
            }
            _ = _solverPrediction.StateAt(
                predictor, Math.Min(t + ExtendChunkSeconds, to), ExtendChunkSeconds);
        }
    }
}

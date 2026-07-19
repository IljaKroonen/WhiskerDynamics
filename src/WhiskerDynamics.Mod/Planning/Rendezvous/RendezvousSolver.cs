using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning.Rendezvous;

internal readonly record struct RendezvousSolution(double DepartureTime, double ArrivalTime,
    Vector3d DepartureDvEcl, Vector3d ArrivalDvEcl, Vector3d DepartureDvVlf,
    Vector3d ArrivalDvVlf, double MissDistance, int Revolutions)
{
    public double TotalDv => DepartureDvEcl.Length() + ArrivalDvEcl.Length();
}

/// <summary>Background two-impulse rendezvous search. Lambert samples short/long
/// zero- and multi-revolution departure/flight-time windows; finalists are shot through the
/// real n-body field, then both commands are corrected through centered finite thrust
/// arcs to match the target's terminal position and velocity.</summary>
internal sealed class RendezvousSolveJob
{
    private readonly record struct Candidate(double Departure, double Arrival,
        RendezvousKernel.LambertSolution Lambert, double Score);

    public required TrackedVessel Chaser { get; init; }
    public required TrackedVessel Target { get; init; }
    public required TrajectoryPredictor ChaserLineage { get; init; }
    public required TrajectoryPredictor TargetLineage { get; init; }
    public required RailsService.PredictionContext Prediction { get; init; }
    public required StateVector ChaserSeed { get; init; }
    public required StateVector TargetSeed { get; init; }
    public required string ParentId { get; init; }
    public required FiniteBurnFold? Finite { get; init; }
    public required double NowSeconds { get; init; }
    public required double HorizonSeconds { get; init; }

    private volatile bool _cancel;
    private volatile bool _done;
    private volatile string _status = "searching transfer windows...";
    private TrajectoryPredictor? _chaserCoast;
    private TrajectoryPredictor? _targetCoast;
    private SolverPrediction _solverPrediction = null!;
    public bool Done => _done;
    public bool Cancelled => _cancel;
    public string StatusLine => _status;
    public RendezvousSolution? Result { get; private set; }
    public string? Failure { get; private set; }
    public long ElapsedMs { get; private set; }
    public void Cancel() => _cancel = true;
    public void Start() => new Thread(Run)
    {
        IsBackground = true,
        Name = "whiskerdynamics-rendezvous",
        Priority = ThreadPriority.BelowNormal,
    }.Start();

    private void Run()
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        try { RunCore(); }
        catch (OperationCanceledException)
        {
            Failure = "rendezvous cancelled";
        }
        catch (Exception e)
        {
            Failure = $"rendezvous solve failed: {e.Message}";
            ModLog.Warn($"planner: rendezvous solve contained: {e}");
        }
        finally { ElapsedMs = clock.ElapsedMilliseconds; _done = true; }
    }

    private void RunCore()
    {
        var rails = Chaser.Rails;
        // Every dynamic read below is served by this job-private gravity cache over
        // an immutable ephemerides snapshot. The captured seeds keep authoritative
        // vessel predictors untouched, and no propagation competes for Rails.Gate.
        _solverPrediction = new SolverPrediction(Prediction, () => _cancel);
        _chaserCoast = new TrajectoryPredictor(
            _solverPrediction.Gravity, ChaserSeed, NowSeconds,
            new IntegratorOptions { RelTol = 1e-9 });
        _targetCoast = new TrajectoryPredictor(
            _solverPrediction.Gravity, TargetSeed, NowSeconds,
            new IntegratorOptions { RelTol = 1e-9 });
        var chaserNow = ChaserSeed;
        var targetNow = TargetSeed;
        double mu = rails.MuOf(ParentId);
        var parentNow = _solverPrediction.GetAbsolute(ParentId, NowSeconds);
        double p1 = RendezvousKernel.OrbitalPeriod(chaserNow - parentNow, mu);
        double p2 = RendezvousKernel.OrbitalPeriod(targetNow - parentNow, mu);
        double period = new[] { p1, p2 }.Where(double.IsFinite).DefaultIfEmpty(6 * 3600.0).Average();
        period = Math.Clamp(period, 10 * 60.0, Math.Max(10 * 60.0, HorizonSeconds - NowSeconds));
        // Match the manual planner's ten-minute placement lead. Besides leaving the
        // background solve/apply transaction room under warp, this keeps ordinary
        // finite burns' centered ignition windows ahead of the click time.
        double lead = Math.Max(600.0, PlannerKernel.MinLeadSeconds + 5.0);
        double tofLo = Math.Max(60.0, 0.04 * period);
        double tofHi = HorizonSeconds - NowSeconds - lead;
        if (tofHi <= tofLo) { Failure = "rendezvous: prediction window is too short"; return; }
        double depLo = NowSeconds + lead;
        // Searching an arbitrary 30-day window at uniform 25-point spacing aliases
        // low orbits completely. Relative phase repeats after the synodic period (or
        // one representative orbit when the mean motions agree), so search one such
        // cycle with a resolution tied to the faster orbit.
        double fasterPeriod = new[] { p1, p2 }.Where(double.IsFinite)
            .DefaultIfEmpty(period).Min();
        double maxRadius = Math.Max((chaserNow - parentNow).Position.Length(),
            (targetNow - parentNow).Position.Length());
        double minimumTransferPeriod = 2.0 * Math.PI
            * Math.Sqrt(Math.Pow(0.5 * maxRadius, 3) / mu);
        if (!double.IsFinite(minimumTransferPeriod) || !(minimumTransferPeriod > 0))
            minimumTransferPeriod = 0.25 * fasterPeriod;
        double relativeCyclesPerSecond = double.IsFinite(p1) && double.IsFinite(p2)
            ? Math.Abs(1.0 / p1 - 1.0 / p2) : 0.0;
        double phaseCycle = relativeCyclesPerSecond > 1e-12
            ? 1.0 / relativeCyclesPerSecond : Math.Max(p1, p2);
        if (!double.IsFinite(phaseCycle) || !(phaseCycle > 0)) phaseCycle = period;
        double depHi = Math.Min(HorizonSeconds - tofLo, depLo + phaseCycle);
        if (depHi <= depLo) { Failure = "rendezvous: no departure window is available"; return; }

        // Preserve a few strong candidates from every logarithmic flight-duration
        // band as well as the globally cheapest set. Otherwise a longer window can
        // fill all finalist slots with delicate multi-day arcs and evict a robust
        // shorter transfer before high-fidelity shooting gets a chance to try it.
        var best = new DurationDiverseSet<Candidate>(
            globalCount: 12, perDurationBand: 4, maximumCount: 24,
            baseDuration: fasterPeriod,
            static candidate => candidate.Score,
            static candidate => candidate.Arrival - candidate.Departure);
        var chaserRelativeCache = new Dictionary<double, StateVector>();
        var targetRelativeCache = new Dictionary<double, StateVector>();
        int departureSamples = Math.Clamp(
            (int)Math.Ceiling((depHi - depLo) / Math.Max(60.0, fasterPeriod / 8.0)) + 1,
            25, 193);
        int maximumRevolutions = Math.Max(0,
            (int)Math.Floor(tofHi / minimumTransferPeriod));
        int[] revolutionCounts = RendezvousKernel.RevolutionSamples(maximumRevolutions, 128);
        if (maximumRevolutions > 0) departureSamples = Math.Min(departureSamples, 49);
        for (int di = 0; di < departureSamples && !_cancel; di++)
        {
            double departure = depLo + (depHi - depLo) * di / (departureSamples - 1);
            double localHi = Math.Min(tofHi, HorizonSeconds - departure);
            if (localHi <= tofLo) continue;
            for (int ri = 0; ri < revolutionCounts.Length && !_cancel; ri++)
            {
                int revolutions = revolutionCounts[ri];
                _status = $"searching transfer windows ({di + 1}/{departureSamples}, "
                    + $"rev {revolutions}/{maximumRevolutions})...";
                double revLo = revolutions == 0
                    ? tofLo : Math.Max(tofLo, revolutions * minimumTransferPeriod + tofLo);
                double revHi = localHi;
                if (revHi <= revLo) continue;
                foreach (double tof in TransferTimeSamples(revolutions, revLo, revHi))
                {
                    foreach (var candidate in CheapCandidates(departure, tof, mu, revolutions))
                        best.Add(candidate);
                }
            }
        }
        if (_cancel) { Failure = "rendezvous cancelled"; return; }
        var finalists = best.Values.ToList();
        if (finalists.Count == 0)
        {
            Failure = "rendezvous: no feasible transfer was found";
            return;
        }

        // Refine the global leader and every retained duration-band leader. Fallback
        // transfers should not reach high-fidelity shooting on a coarser grid merely
        // because a fragile long-duration arc had the best preliminary score.
        double depStep = (depHi - depLo) / (departureSamples - 1);
        var refinementSeeds = best.BandLeaders.Where(finalists.Contains)
            .Prepend(finalists[0]).Distinct().ToArray();
        foreach (var seed in refinementSeeds)
        {
            Candidate incumbent = seed;
            double localDepStep = depStep;
            double tofStep = Math.Min(period / 4.0,
                Math.Max(60.0, 0.1 * (incumbent.Arrival - incumbent.Departure)));
            for (int iteration = 0; iteration < 7 && !_cancel; iteration++)
            {
                foreach (int dd in new[] { -1, 0, 1 })
                foreach (int dt in new[] { -1, 0, 1 })
                {
                    if (dd == 0 && dt == 0) continue;
                    double departure = Math.Clamp(
                        incumbent.Departure + dd * localDepStep, depLo, depHi);
                    double tof = Math.Clamp(
                        incumbent.Arrival - incumbent.Departure + dt * tofStep,
                        tofLo, Math.Min(tofHi, HorizonSeconds - departure));
                    if (!(tof >= tofLo)) continue;
                    foreach (var candidate in CheapCandidates(departure, tof, mu,
                        incumbent.Lambert.Revolutions))
                        if (candidate.Score < incumbent.Score) incumbent = candidate;
                }
                localDepStep *= 0.5;
                tofStep *= 0.5;
            }
            int index = finalists.IndexOf(seed);
            if (index >= 0) finalists[index] = incumbent;
        }
        finalists = finalists.Distinct().ToList();

        _status = "differential-correcting in the n-body field...";
        RendezvousSolution? winner = null;
        foreach (var candidate in finalists)
        {
            if (_cancel) { Failure = "rendezvous cancelled"; return; }
            if (Correct(candidate) is not { } corrected) continue;
            if (Finite is not null)
            {
                _status = "correcting finite thrust arcs...";
                if (CorrectFinite(corrected) is not { } finiteCorrected) continue;
                corrected = finiteCorrected;
            }
            if (winner is null || corrected.TotalDv < winner.Value.TotalDv) winner = corrected;
        }
        if (winner is null)
        {
            Failure = "rendezvous: transfer shooting did not converge; try a longer orbit window";
            return;
        }
        Result = winner;

        double[] TransferTimeSamples(int revolutions, double lo, double hi)
        {
            var values = new SortedSet<double> { lo, hi };
            // Log spacing represents the entire selected duration without spending
            // every probe at its far end when the window spans hundreds of orbits.
            const int logarithmicSamples = 8;
            double ratio = hi / lo;
            for (int k = 1; k < logarithmicSamples - 1; k++)
                values.Add(lo * Math.Pow(ratio, (double)k / (logarithmicSamples - 1)));
            // Dense probes around current-orbit phasing times retain sensitivity to
            // the low-dv family even though transfer-orbit periods can be much lower.
            foreach (double fraction in new[] { 0.05, 0.25, 0.5, 0.75, 1.0 })
            {
                double expected = (revolutions + fraction) * period;
                if (expected > lo && expected < hi) values.Add(expected);
            }
            return [.. values];
        }

        List<Candidate> CheapCandidates(double departure, double tof,
            double centralMu, int revolutions)
        {
            var candidates = new List<Candidate>(4);
            double arrival = departure + tof;
            var cr = RelativeAt(chaserRelativeCache, departure, ChaserStateAt);
            var tr = RelativeAt(targetRelativeCache, arrival, TargetStateAt);
            foreach (bool longWay in new[] { false, true })
            foreach (var lambert in RendezvousKernel.SolveLambert(
                cr.Position, tr.Position, tof, centralMu, longWay, revolutions))
            {
                double score = (lambert.DepartureVelocity - cr.Velocity).Length()
                    + (tr.Velocity - lambert.ArrivalVelocity).Length();
                candidates.Add(new Candidate(departure, arrival, lambert, score));
            }
            return candidates;
        }

        StateVector RelativeAt(Dictionary<double, StateVector> cache, double time,
            Func<double, StateVector> stateAt)
        {
            if (cache.TryGetValue(time, out var relative)) return relative;
            relative = stateAt(time) - _solverPrediction.GetAbsolute(ParentId, time);
            cache.Add(time, relative);
            return relative;
        }
    }

    private RendezvousSolution? Correct(Candidate candidate)
    {
        var chaser = ChaserStateAt(candidate.Departure);
        var parent = _solverPrediction.GetAbsolute(ParentId, candidate.Departure);
        var target = TargetStateAt(candidate.Arrival);
        Vector3d velocity = parent.Velocity + candidate.Lambert.DepartureVelocity;
        StateVector flown = default;
        double miss = double.PositiveInfinity;
        for (int iteration = 0; iteration < 8; iteration++)
        {
            if (_cancel) return null;
            flown = Propagate(chaser.Position, velocity, candidate.Departure, candidate.Arrival);
            Vector3d residual = target.Position - flown.Position;
            miss = residual.Length();
            if (miss <= 5.0) break;
            const double epsilon = 0.1;
            var j0 = (Propagate(chaser.Position, velocity + new Vector3d(epsilon, 0, 0),
                candidate.Departure, candidate.Arrival).Position - flown.Position) / epsilon;
            if (_cancel) return null;
            var j1 = (Propagate(chaser.Position, velocity + new Vector3d(0, epsilon, 0),
                candidate.Departure, candidate.Arrival).Position - flown.Position) / epsilon;
            if (_cancel) return null;
            var j2 = (Propagate(chaser.Position, velocity + new Vector3d(0, 0, epsilon),
                candidate.Departure, candidate.Arrival).Position - flown.Position) / epsilon;
            if (!RendezvousKernel.TrySolveLinear3(j0, j1, j2, residual, out var correction)) return null;
            double magnitude = correction.Length();
            if (!double.IsFinite(magnitude) || magnitude > 3000.0) return null;
            velocity += correction;
        }
        // The loop may apply its eighth correction on the final iteration. Always
        // evaluate the actually returned command once more so miss, arrival velocity,
        // and both burns describe the same trajectory.
        if (_cancel) return null;
        flown = Propagate(chaser.Position, velocity, candidate.Departure, candidate.Arrival);
        miss = (target.Position - flown.Position).Length();
        if (miss > 25.0) return null;
        if (!PathClearsBodies(chaser.Position, velocity, candidate.Departure, candidate.Arrival))
            return null;
        Vector3d dv1 = velocity - chaser.Velocity;
        Vector3d dv2 = target.Velocity - flown.Velocity;
        var parentArrival = _solverPrediction.GetAbsolute(ParentId, candidate.Arrival);
        var vlf1 = BurnFrameKernel.EclToVlf(dv1,
            chaser.Position - parent.Position, chaser.Velocity - parent.Velocity);
        var vlf2 = BurnFrameKernel.EclToVlf(dv2,
            flown.Position - parentArrival.Position, flown.Velocity - parentArrival.Velocity);
        if (vlf1 is null || vlf2 is null) return null;
        return new RendezvousSolution(candidate.Departure, candidate.Arrival, dv1, dv2,
            vlf1.Value, vlf2.Value, miss, candidate.Lambert.Revolutions);
    }

    /// <summary>Six-state differential correction of both commanded burns through
    /// the FC's centered finite-thrust model. The terminal condition is evaluated at
    /// arrival-burn cutoff, so both position and velocity actually match after the
    /// maneuver rather than only at an idealized impulse.</summary>
    private RendezvousSolution? CorrectFinite(RendezvousSolution impulsive)
    {
        var values = new[]
        {
            impulsive.DepartureDvEcl.X, impulsive.DepartureDvEcl.Y, impulsive.DepartureDvEcl.Z,
            impulsive.ArrivalDvEcl.X, impulsive.ArrivalDvEcl.Y, impulsive.ArrivalDvEcl.Z,
        };
        const double epsilon = 0.05;
        double timeScale = Math.Max(1.0, impulsive.ArrivalTime - impulsive.DepartureTime);
        for (int iteration = 0; iteration < 7; iteration++)
        {
            if (_cancel || !TryEvaluateFinite(values, impulsive.DepartureTime,
                    impulsive.ArrivalTime, out var baseline)) return null;
            var delta = baseline.Target - baseline.Flown;
            if (delta.Position.Length() <= 5.0 && delta.Velocity.Length() <= 0.005) break;
            var rhs = new[]
            {
                delta.Position.X / timeScale, delta.Position.Y / timeScale,
                delta.Position.Z / timeScale, delta.Velocity.X, delta.Velocity.Y, delta.Velocity.Z,
            };
            var jacobian = new double[6, 6];
            for (int column = 0; column < 6; column++)
            {
                if (_cancel) return null;
                var perturbed = (double[])values.Clone();
                perturbed[column] += epsilon;
                if (!TryEvaluateFinite(perturbed, impulsive.DepartureTime,
                        impulsive.ArrivalTime, out var probe)) return null;
                // Burn duration (therefore cutoff time) changes with command
                // magnitude, so the target moves between probes too. Differentiate
                // the relative terminal state, not the vessel state alone.
                var change = (probe.Flown - probe.Target) - (baseline.Flown - baseline.Target);
                jacobian[0, column] = change.Position.X / (epsilon * timeScale);
                jacobian[1, column] = change.Position.Y / (epsilon * timeScale);
                jacobian[2, column] = change.Position.Z / (epsilon * timeScale);
                jacobian[3, column] = change.Velocity.X / epsilon;
                jacobian[4, column] = change.Velocity.Y / epsilon;
                jacobian[5, column] = change.Velocity.Z / epsilon;
            }
            if (!RendezvousKernel.TrySolveLinear(jacobian, rhs, out var correction)) return null;
            var firstCorrection = new Vector3d(correction[0], correction[1], correction[2]);
            var secondCorrection = new Vector3d(correction[3], correction[4], correction[5]);
            if (firstCorrection.Length() > 2000.0 || secondCorrection.Length() > 2000.0) return null;
            for (int k = 0; k < 6; k++) values[k] += correction[k];
        }
        if (!TryEvaluateFinite(values, impulsive.DepartureTime, impulsive.ArrivalTime,
                out var final)) return null;
        var terminalError = final.Target - final.Flown;
        double miss = terminalError.Position.Length();
        if (miss > 25.0 || terminalError.Velocity.Length() > 0.02
            || !PathClearsBodies(final.Path)) return null;

        var dv1 = new Vector3d(values[0], values[1], values[2]);
        var dv2 = new Vector3d(values[3], values[4], values[5]);
        var chaserAtNode = ChaserStateAt(impulsive.DepartureTime);
        var parentAtDeparture = _solverPrediction.GetAbsolute(ParentId, impulsive.DepartureTime);
        if (!TryImpulsiveFirstBurnStateAt(dv1, impulsive.DepartureTime, impulsive.ArrivalTime,
                out var beforeSecond)) return null;
        var parentAtArrival = _solverPrediction.GetAbsolute(ParentId, impulsive.ArrivalTime);
        var vlf1 = BurnFrameKernel.EclToVlf(dv1,
            chaserAtNode.Position - parentAtDeparture.Position,
            chaserAtNode.Velocity - parentAtDeparture.Velocity);
        var vlf2 = BurnFrameKernel.EclToVlf(dv2,
            beforeSecond.Position - parentAtArrival.Position,
            beforeSecond.Velocity - parentAtArrival.Velocity);
        if (vlf1 is null || vlf2 is null) return null;
        return new RendezvousSolution(impulsive.DepartureTime, impulsive.ArrivalTime,
            dv1, dv2, vlf1.Value, vlf2.Value, miss, impulsive.Revolutions);
    }

    private bool TryEvaluateFinite(double[] values, double departure, double arrival,
        out RendezvousFiniteEvaluation evaluation)
    {
        evaluation = null!;
        if (Finite is not { } finite) return false;
        var dv1 = new Vector3d(values[0], values[1], values[2]);
        var dv2 = new Vector3d(values[3], values[4], values[5]);
        if (!RendezvousFiniteAdmission.TryAdmit(
                departure, dv1.Length(), arrival, dv2.Length(), finite,
                NowSeconds + PlannerKernel.MinLeadSeconds, HorizonSeconds,
                out var commands))
            return false;
        evaluation = RendezvousFiniteEvaluator.Evaluate(commands,
            _solverPrediction.Gravity, ChaserStateAt(commands.PredictionStartSeconds),
            departure, dv1, arrival, dv2,
            StateAtCancellable, TargetStateAt);
        return true;
    }

    /// <summary>Stock interprets the second node's VLF numbers against its impulsive
    /// chained plan. Use the same earlier-burn semantics for authoring even though the
    /// terminal objective above models the FC's finite execution.</summary>
    private bool TryImpulsiveFirstBurnStateAt(Vector3d dv, double departure, double time,
        out StateVector state)
    {
        var path = new TrajectoryPredictor(_solverPrediction.Gravity,
            ChaserStateAt(departure), departure,
            new IntegratorOptions { RelTol = 1e-10 });
        if (dv.LengthSquared() > 0) path.AddImpulse(departure, dv);
        state = StateAtCancellable(path, time);
        return true;
    }

    private StateVector Propagate(Vector3d position, Vector3d velocity, double from, double to)
    {
        var predictor = new TrajectoryPredictor(_solverPrediction.Gravity,
            new StateVector(position, velocity), from, new IntegratorOptions { RelTol = 1e-10 });
        return StateAtCancellable(predictor, to);
    }

    private StateVector ChaserStateAt(double time) =>
        StateAtCancellable(_chaserCoast!, time);

    private StateVector TargetStateAt(double time) =>
        StateAtCancellable(_targetCoast!, time);

    /// <summary>Endpoint convergence is not enough: mathematical gravity happily
    /// integrates through a planet. Verify every accepted integration knot against
    /// every modeled massive body's surface before accepting the corrected transfer.</summary>
    private bool PathClearsBodies(Vector3d position, Vector3d velocity, double from, double to)
    {
        var predictor = new TrajectoryPredictor(_solverPrediction.Gravity,
            new StateVector(position, velocity), from, new IntegratorOptions { RelTol = 1e-10 });
        _ = StateAtCancellable(predictor, to);
        return PathClearsBodies(predictor);
    }

    private StateVector StateAtCancellable(TrajectoryPredictor predictor, double time) =>
        _solverPrediction.StateAt(predictor, time, 6 * 3600.0);

    private bool PathClearsBodies(TrajectoryPredictor predictor)
    {
        var rails = Chaser.Rails;
        double parentSoi = rails.SphereOfInfluenceOf(ParentId);
        var children = rails.SoiChildrenOf(ParentId);
        foreach (var node in predictor.Nodes)
        {
            if (_cancel) return false;
            var parent = _solverPrediction.GetAbsolute(ParentId, node.Time);
            if (double.IsFinite(parentSoi) && parentSoi > 0
                && (node.State.Position - parent.Position).Length() >= parentSoi) return false;
            foreach (string childId in children)
            {
                double soi = rails.SphereOfInfluenceOf(childId);
                if (!(soi > 0) || !double.IsFinite(soi)) continue;
                if ((node.State.Position
                    - _solverPrediction.GetAbsolute(childId, node.Time).Position).Length() <= soi)
                    return false;
            }
        }
        foreach (var node in predictor.Nodes)
        foreach (string bodyId in rails.ModeledIds)
        {
            if (_cancel) return false;
            double radius = rails.MeanRadiusOf(bodyId);
            if (!(radius > 0)) continue;
            var body = _solverPrediction.GetAbsolute(bodyId, node.Time);
            if ((node.State.Position - body.Position).Length()
                <= radius + Math.Max(100.0, radius * 1e-6)) return false;
        }
        for (int i = 1; i < predictor.Nodes.Count; i++)
        {
            if (_cancel) return false;
            var a = predictor.Nodes[i - 1];
            var b = predictor.Nodes[i];
            foreach (string bodyId in rails.ModeledIds)
            {
                double radius = rails.MeanRadiusOf(bodyId);
                if (!(radius > 0)) continue;
                double clearance = radius + Math.Max(100.0, radius * 1e-6);
                if (!SegmentClearsSphere(predictor, bodyId, clearance, a, b, depth: 6))
                    return false;
            }
            foreach (string childId in children)
            {
                double soi = rails.SphereOfInfluenceOf(childId);
                if (soi > 0 && double.IsFinite(soi)
                    && !SegmentClearsSphere(predictor, childId, soi, a, b, depth: 6))
                    return false;
            }
        }
        return true;
    }

    private bool SegmentClearsSphere(TrajectoryPredictor predictor, string bodyId,
        double radius, TrajectoryNode a, TrajectoryNode b, int depth)
    {
        Vector3d CenterAt(double time) =>
            _solverPrediction.GetAbsolute(bodyId, time).Position;
        StateVector StateAt(TrajectoryPredictor path, double time) =>
            _solverPrediction.StateAt(path, time, 6 * 3600.0);
        bool Cancelled() => _cancel;
        Func<double, Vector3d> centerAt = CenterAt;
        Func<TrajectoryPredictor, double, StateVector> stateAt = StateAt;
        Func<bool> cancelled = Cancelled;

        var left = a;
        while (left.Time < b.Time)
        {
            if (_cancel) return false;
            double spanEnd = Math.Min(b.Time,
                _solverPrediction.GetAbsolutePositionSegmentEndAfter(bodyId, left.Time));
            if (!(spanEnd > left.Time)) return false;
            var right = spanEnd == b.Time
                ? b
                : new TrajectoryNode(spanEnd, StateAtCancellable(predictor, spanEnd));
            if (!RendezvousFiniteEvaluator.SegmentClearsSphere(
                    predictor, radius, left, right, depth,
                    centerAt, stateAt, cancelled))
                return false;
            left = right;
        }
        return true;
    }

}

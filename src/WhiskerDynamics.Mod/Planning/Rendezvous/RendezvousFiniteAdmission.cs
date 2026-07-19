using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning.Rendezvous;

/// <summary>The two safely representable finite commands for a rendezvous solve.
/// Numerical impulses/expansions remain attached to their physical windows so no
/// caller can accidentally use node time for lead, overlap, horizon, collision-path,
/// or terminal-state boundaries.</summary>
internal readonly record struct RendezvousFiniteCommands(
    FiniteBurnCommand Departure, FiniteBurnCommand Arrival)
{
    public double PredictionStartSeconds => Departure.Window.IgnitionSeconds;
    public double TerminalSeconds => Arrival.Window.CutoffSeconds;
}

/// <summary>KSA-free admission for the automatic rendezvous finite-burn pair.</summary>
internal static class RendezvousFiniteAdmission
{
    /// <summary>Resolves both commands, chaining mass into the arrival burn, and
    /// admits them only when both complete physical windows fit the solve bounds.
    /// The earliest ignition and inter-burn bounds are exclusive; the horizon is
    /// inclusive.</summary>
    public static bool TryAdmit(double departureNodeSeconds, double departureMagnitude,
        double arrivalNodeSeconds, double arrivalMagnitude, FiniteBurnFold finite,
        double exclusiveEarliestIgnition, double inclusiveHorizon,
        out RendezvousFiniteCommands commands)
    {
        commands = default;
        if (!double.IsFinite(exclusiveEarliestIgnition)
            || !double.IsFinite(inclusiveHorizon)
            || !FiniteBurnKernel.TryResolveCommand(
                departureNodeSeconds, departureMagnitude, finite.Engine,
                finite.SliceSeconds, finite.MaxSlices, out var departure))
            return false;
        var arrivalEngine = finite.Engine with
        {
            MassKg = FiniteBurnKernel.MassAfterBurn(departureMagnitude, finite.Engine),
        };
        if (!FiniteBurnKernel.TryResolveCommand(
                arrivalNodeSeconds, arrivalMagnitude, arrivalEngine,
                finite.SliceSeconds, finite.MaxSlices, out var arrival))
            return false;
        if (departure.Window.IgnitionSeconds <= exclusiveEarliestIgnition
            || departure.Window.CutoffSeconds >= arrival.Window.IgnitionSeconds
            || arrival.Window.CutoffSeconds > inclusiveHorizon)
            return false;
        commands = new RendezvousFiniteCommands(departure, arrival);
        return true;
    }
}

internal sealed record RendezvousFiniteEvaluation(
    StateVector Flown, StateVector Target, TrajectoryPredictor Path);

/// <summary>KSA-free execution seam for an admitted rendezvous pair. The production
/// solver and regression tests share the exact trajectory boundaries, numerical
/// commands, terminal target epoch, and recursive sphere traversal.</summary>
internal static class RendezvousFiniteEvaluator
{
    public static RendezvousFiniteEvaluation Evaluate(
        RendezvousFiniteCommands commands, GravityModel gravity,
        StateVector chaserAtPredictionStart,
        double departureNodeSeconds, Vector3d departureDeltaV,
        double arrivalNodeSeconds, Vector3d arrivalDeltaV,
        Func<TrajectoryPredictor, double, StateVector> stateAt,
        Func<double, StateVector> targetAt)
    {
        var path = new TrajectoryPredictor(gravity, chaserAtPredictionStart,
            commands.PredictionStartSeconds,
            new IntegratorOptions { RelTol = 1e-10 });
        AddCommand(path, departureNodeSeconds, departureDeltaV,
            commands.Departure.Expansion);
        AddCommand(path, arrivalNodeSeconds, arrivalDeltaV,
            commands.Arrival.Expansion);
        var flown = stateAt(path, commands.TerminalSeconds);
        return new RendezvousFiniteEvaluation(
            flown, targetAt(commands.TerminalSeconds), path);
    }

    /// <summary>Conservatively admits one relative interpolation span. The chaser is
    /// cubic and the body center is at most quintic, so seven Chebyshev-Lobatto samples
    /// determine the relative position polynomial; their Lebesgue bound encloses its
    /// distance from the endpoint chord. Unproved spans subdivide, and depth exhaustion
    /// or cancellation rejects.</summary>
    public static bool SegmentClearsSphere(TrajectoryPredictor predictor,
        double radius, TrajectoryNode a, TrajectoryNode b, int depth,
        Func<double, Vector3d> centerAt,
        Func<TrajectoryPredictor, double, StateVector> stateAt,
        Func<bool>? isCancelled = null)
    {
        if (!(radius > 0.0) || !double.IsFinite(radius)
            || depth <= 0 || (isCancelled?.Invoke() ?? false)) return false;
        var ra = a.State.Position - centerAt(a.Time);
        if (isCancelled?.Invoke() ?? false) return false;
        var rb = b.State.Position - centerAt(b.Time);
        if (isCancelled?.Invoke() ?? false) return false;
        return SegmentClearsSphere(predictor, radius,
            new RelativeSample(a, ra), new RelativeSample(b, rb), depth,
            centerAt, stateAt, isCancelled, knownMidpoint: null);
    }

    private static bool SegmentClearsSphere(TrajectoryPredictor predictor,
        double radius, RelativeSample a, RelativeSample b, int depth,
        Func<double, Vector3d> centerAt,
        Func<TrajectoryPredictor, double, StateVector> stateAt,
        Func<bool>? isCancelled, RelativeSample? knownMidpoint)
    {
        if (a.Relative.Length() <= radius || b.Relative.Length() <= radius) return false;
        if (depth <= 0 || (isCancelled?.Invoke() ?? false)) return false;

        double dt = b.Node.Time - a.Node.Time;
        if (!(dt > 0.0) || !double.IsFinite(dt)) return false;
        if (!TrySample(0.5 - Math.Sqrt(3.0) / 4.0, out var nearA)
            || !TrySample(0.25, out var quarter)
            || !TrySample(0.5, out var midpoint)
            || !TrySample(0.75, out var threeQuarter)
            || !TrySample(0.5 + Math.Sqrt(3.0) / 4.0, out var nearB))
            return false;

        var chord = b.Relative - a.Relative;
        double chordLengthSquared = chord.LengthSquared();
        double u = chordLengthSquared > 0
            ? Math.Clamp(-a.Relative.Dot(chord) / chordLengthSquared, 0.0, 1.0) : 0.0;
        double chordDistance = (a.Relative + chord * u).Length();
        double maxDeviation = Math.Max(
            Math.Max(Deviation(nearA), Deviation(quarter)),
            Math.Max(Deviation(midpoint),
                Math.Max(Deviation(threeQuarter), Deviation(nearB))));
        // The seven-node Chebyshev-Lobatto Lebesgue constant is below 3. This
        // bounds every point of the degree-five relative polynomial from its chord.
        if (chordDistance > radius + 3.0 * maxDeviation) return true;

        return SegmentClearsSphere(predictor, radius, a, midpoint, depth - 1,
                centerAt, stateAt, isCancelled, quarter)
            && SegmentClearsSphere(predictor, radius, midpoint, b, depth - 1,
                centerAt, stateAt, isCancelled, threeQuarter);

        double Deviation(RelativeSample sample)
        {
            double sampleU = (sample.Node.Time - a.Node.Time) / dt;
            return (sample.Relative - (a.Relative + chord * sampleU)).Length();
        }

        bool TrySample(double sampleU, out RelativeSample sample)
        {
            if (isCancelled?.Invoke() ?? false)
            {
                sample = default;
                return false;
            }
            if (sampleU == 0.5 && knownMidpoint is { } known)
            {
                sample = known;
                return sample.Relative.Length() > radius;
            }
            double time = a.Node.Time + dt * sampleU;
            var state = stateAt(predictor, time);
            if (isCancelled?.Invoke() ?? false)
            {
                sample = default;
                return false;
            }
            var relative = state.Position - centerAt(time);
            sample = new RelativeSample(new TrajectoryNode(time, state), relative);
            return !(isCancelled?.Invoke() ?? false) && relative.Length() > radius;
        }
    }

    private readonly record struct RelativeSample(TrajectoryNode Node, Vector3d Relative);

    private static void AddCommand(TrajectoryPredictor path, double nodeTime,
        Vector3d deltaV, FiniteBurnExpansion? expansion)
    {
        double magnitude = deltaV.Length();
        if (!(magnitude > 0)) return;
        var direction = deltaV / magnitude;
        if (expansion is null)
        {
            path.AddImpulse(nodeTime, deltaV);
            return;
        }
        for (int k = 0; k < expansion.Times.Length; k++)
            path.AddImpulse(expansion.Times[k], direction * expansion.Magnitudes[k]);
    }
}

internal enum RendezvousApplyLeadVerdict
{
    Allowed,
    PhysicallyUnmodelable,
    InsufficientLead,
}

/// <summary>KSA-free solve-to-apply lead policy. A centered finite burn is owned from
/// physical ignition, even when its one-slice numerical representation remains at the
/// later node.</summary>
internal static class RendezvousApplyPolicy
{
    public static RendezvousApplyLeadVerdict CheckDepartureLead(
        double departureNodeSeconds, double departureMagnitude,
        FiniteBurnFold? finite, double nowSeconds, double leadSeconds)
    {
        double modeledStart = departureNodeSeconds;
        if (finite is { } model)
        {
            if (!FiniteBurnKernel.TryGetPhysicalWindow(
                    departureNodeSeconds, departureMagnitude, model.Engine,
                    out var window))
                return RendezvousApplyLeadVerdict.PhysicallyUnmodelable;
            modeledStart = window.IgnitionSeconds;
        }
        return OptimizeApplyPolicy.ModeledStartHasLead(modeledStart, nowSeconds, leadSeconds)
            ? RendezvousApplyLeadVerdict.Allowed
            : RendezvousApplyLeadVerdict.InsufficientLead;
    }
}

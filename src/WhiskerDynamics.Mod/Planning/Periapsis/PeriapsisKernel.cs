namespace WhiskerDynamics.Mod.Planning.Periapsis;

/// <summary>One accepted point of the constrained 4-D burn optimize: burn time,
/// full VLF delta-v, and the achieved objective value in its native unit.</summary>
public sealed record DvMinimum(
    double TimeSeconds, double Prograde, double Normal, double Outward, double AchievedObjective,
    double? AchievedInclination = null)
{
    public double Magnitude => Math.Sqrt(Prograde * Prograde + Normal * Normal + Outward * Outward);
    /// <summary>Periapsis-objective entry point.</summary>
    public double AchievedPeriapsis => AchievedObjective;
}

public readonly record struct CoupledTargetSolution(
    double Prograde, double Normal, double AchievedPeriapsis, double AchievedInclination);

/// <summary>The VLF component projected by the optimizer's inner scalar solve.</summary>
public enum OptimizerConstraint
{
    Prograde,
    Normal,
}

public enum OptimizerObjective
{
    Periapsis,
    Inclination,
}

/// <summary>KSA-free optimizer rules (offline-tested; the panel and solver translate
/// game state to these inputs): target-body selection, inclination measurement,
/// scalar constraint solves, first-periapsis scanning, and the 4-D minimum-delta-v
/// outer search.</summary>
public static class PeriapsisKernel
{
    /// <summary>Jointly solves prograde/normal for Pe and inclination; only accepted
    /// (interior-periapsis) solutions are returned.</summary>
    public static CoupledTargetSolution? SolveCoupledTargets(
        Func<double, double, (double Periapsis, double Inclination, bool Accepted)?> evaluate,
        double prograde0, double normal0, double periapsisTarget, double inclinationTarget,
        double probeStep, double maxOffset, double periapsisTolerance,
        double inclinationTolerance, int iterations = 40, Func<bool>? cancelled = null)
    {
        cancelled ??= () => false;
        if (!(probeStep > 0) || !(maxOffset >= probeStep)
            || !(periapsisTolerance > 0) || !(inclinationTolerance > 0))
            return null;

        double ResidualNorm((double Periapsis, double Inclination, bool Accepted) value)
        {
            double pe = (value.Periapsis - periapsisTarget) / periapsisTolerance;
            double inc = (value.Inclination - inclinationTarget) / inclinationTolerance;
            return Math.Sqrt(pe * pe + inc * inc);
        }
        bool Meets((double Periapsis, double Inclination, bool Accepted) value) =>
            value.Accepted
            && Math.Abs(value.Periapsis - periapsisTarget) <= periapsisTolerance
            && Math.Abs(value.Inclination - inclinationTarget) <= inclinationTolerance;

        double p = prograde0, n = normal0;
        if (cancelled()) return null;
        var current = evaluate(p, n);
        if (current is null) return null;
        double currentNorm = ResidualNorm(current.Value);
        double trust = Math.Min(maxOffset, 4.0 * probeStep);
        double damping = 1e-3;

        for (int iteration = 0; iteration < iterations && !cancelled(); iteration++)
        {
            if (Meets(current.Value))
                return new CoupledTargetSolution(
                    p, n, current.Value.Periapsis, current.Value.Inclination);

            double rPe = (current.Value.Periapsis - periapsisTarget) / periapsisTolerance;
            double rInc = (current.Value.Inclination - inclinationTarget) / inclinationTolerance;
            bool Derivative(double dp, double dn, out double dPe, out double dInc)
            {
                if (cancelled()) { dPe = dInc = 0; return false; }
                double pp = Math.Clamp(p + dp, prograde0 - maxOffset, prograde0 + maxOffset);
                double nn = Math.Clamp(n + dn, normal0 - maxOffset, normal0 + maxOffset);
                double delta = dp != 0 ? pp - p : nn - n;
                var probe = delta != 0 ? evaluate(pp, nn) : null;
                if (probe is null)
                {
                    if (cancelled()) { dPe = dInc = 0; return false; }
                    pp = Math.Clamp(p - dp, prograde0 - maxOffset, prograde0 + maxOffset);
                    nn = Math.Clamp(n - dn, normal0 - maxOffset, normal0 + maxOffset);
                    delta = dp != 0 ? pp - p : nn - n;
                    probe = delta != 0 ? evaluate(pp, nn) : null;
                }
                if (probe is null || delta == 0)
                { dPe = dInc = 0; return false; }
                dPe = ((probe.Value.Periapsis - current.Value.Periapsis)
                    / periapsisTolerance) / delta;
                dInc = ((probe.Value.Inclination - current.Value.Inclination)
                    / inclinationTolerance) / delta;
                return double.IsFinite(dPe) && double.IsFinite(dInc);
            }

            if (!Derivative(probeStep, 0, out double j00, out double j10)
                || !Derivative(0, probeStep, out double j01, out double j11))
                return null;

            // Levenberg-Marquardt normal equations: (J'J + lambda I) step = -J'r.
            double a = j00 * j00 + j10 * j10 + damping;
            double b = j00 * j01 + j10 * j11;
            double d = j01 * j01 + j11 * j11 + damping;
            double gp = j00 * rPe + j10 * rInc;
            double gn = j01 * rPe + j11 * rInc;
            double determinant = a * d - b * b;
            if (!(Math.Abs(determinant) > 1e-18) || !double.IsFinite(determinant)) return null;
            double stepP = (-d * gp + b * gn) / determinant;
            double stepN = (b * gp - a * gn) / determinant;
            double stepLength = Math.Sqrt(stepP * stepP + stepN * stepN);
            if (!double.IsFinite(stepLength) || stepLength == 0) return null;
            if (stepLength > trust)
            {
                double scale = trust / stepLength;
                stepP *= scale;
                stepN *= scale;
            }

            bool improved = false;
            for (double scale = 1.0; scale >= 0.125; scale *= 0.5)
            {
                if (cancelled()) return null;
                double candidateP = Math.Clamp(
                    p + scale * stepP, prograde0 - maxOffset, prograde0 + maxOffset);
                double candidateN = Math.Clamp(
                    n + scale * stepN, normal0 - maxOffset, normal0 + maxOffset);
                var candidate = evaluate(candidateP, candidateN);
                if (candidate is null) continue;
                double candidateNorm = ResidualNorm(candidate.Value);
                if (!(candidateNorm < currentNorm)) continue;
                p = candidateP;
                n = candidateN;
                current = candidate;
                currentNorm = candidateNorm;
                trust = Math.Min(maxOffset, trust * 2.0);
                damping = Math.Max(1e-12, damping * 0.25);
                improved = true;
                break;
            }
            if (improved) continue;
            trust *= 0.5;
            damping *= 10.0;
            if (trust < probeStep / 64.0) return null;
        }
        return Meets(current.Value)
            ? new CoupledTargetSolution(p, n, current.Value.Periapsis, current.Value.Inclination)
            : null;
    }

    /// <summary>Low-angle inclination changes are projected through normal delta-v;
    /// high-angle changes constrain prograde so polar and retrograde solutions can
    /// cross the target instead of approaching it asymptotically.</summary>
    public static OptimizerConstraint ConstraintFor(
        OptimizerObjective objective, double targetValue) =>
        objective == OptimizerObjective.Inclination && targetValue <= Math.PI / 4.0
            ? OptimizerConstraint.Normal
            : OptimizerConstraint.Prograde;

    /// <summary>Find a feasible local-search seed: authored value first, then
    /// exponentially expanding positive/negative probes.</summary>
    public static double? FindFeasibleOuter(Func<double, bool> feasible,
        double authored, double probeStep, double maxOffset, Func<bool> cancelled)
    {
        if (feasible(authored)) return authored;
        for (double step = probeStep; step <= maxOffset && !cancelled(); step *= 2)
        {
            if (feasible(authored + step)) return authored + step;
            if (feasible(authored - step)) return authored - step;
        }
        return null;
    }

    /// <summary>Inclination is measured after all finite-burn slices have executed;
    /// impulsive candidates are measurable one second after their node.</summary>
    public static double InclinationMeasurementTime(
        double burnTime, FiniteBurnExpansion? expansion) => expansion is null
            ? burnTime + 1.0
            : expansion.IgnitionSeconds + expansion.DurationSeconds + 1.0;

    /// <summary>Osculating inclination of a relative state against a body's equatorial
    /// pole, in radians. Null when position, velocity, or pole cannot define an orbit
    /// plane. This is the same angular-momentum convention used by the orbit analyser.</summary>
    public static double? InclinationRadians(
        WhiskerDynamics.Core.Vector3d relativePosition, WhiskerDynamics.Core.Vector3d relativeVelocity,
        WhiskerDynamics.Core.Vector3d equatorialPole)
    {
        var angularMomentum = relativePosition.Cross(relativeVelocity);
        double hLength = angularMomentum.Length();
        double poleLength = equatorialPole.Length();
        if (!(hLength > 0) || !(poleLength > 0)
            || !double.IsFinite(hLength) || !double.IsFinite(poleLength))
            return null;
        return Math.Acos(Math.Clamp(
            angularMomentum.Dot(equatorialPole) / (hLength * poleLength), -1.0, 1.0));
    }

    /// <summary>The body a periapsis optimization targets under the ACTIVE map
    /// display frame: the frame's PRIMARY body for every body-anchored kind — the transfer target
    /// in a two-body fixed frame (the catalog builds pairs child-first, so
    /// "Luna-Earth Fixed" targets Luna), the centre body in body-centred inertial
    /// ("Sol-Centred Inertial" targets Sol), surface frames, and target-fixed frames
    /// ("Earth-Target Fixed" targets Earth). No active frame falls back to the
    /// controlled vessel's orbit parent. Null only when neither exists.</summary>
    public static string? TargetBodyId(FrameSpec? activeFrame, string? orbitParentId) =>
        activeFrame?.PrimaryId ?? orbitParentId;

    /// <summary>Golden-section refinement of a distance minimum inside a bracketing
    /// interval (the coarse scan's neighbors around its first interior local
    /// minimum): the continuous closest-approach time and distance. Unimodality
    /// inside the bracket is the coarse scan's contract; iterations cap the work,
    /// the 1 ms interval floor stops early (far below any orbital timescale), and
    /// <paramref name="distanceTolerance"/> stops once the probes agree within it —
    /// the caller's precision need, so refinement never out-resolves its consumer.</summary>
    public static (double Time, double Distance) RefineMinimum(Func<double, double> distanceAt,
        double lo, double hi, int iterations = 60, double distanceTolerance = 0.0)
    {
        const double phi = 0.6180339887498949; // golden-ratio conjugate
        double a = lo, b = hi;
        double x1 = b - phi * (b - a), x2 = a + phi * (b - a);
        double f1 = distanceAt(x1), f2 = distanceAt(x2);
        for (int i = 0; i < iterations && b - a > 1e-3
             && !(distanceTolerance > 0 && Math.Abs(f1 - f2) <= distanceTolerance && i >= 2); i++)
        {
            if (f1 <= f2)
            {
                b = x2; x2 = x1; f2 = f1;
                x1 = b - phi * (b - a); f1 = distanceAt(x1);
            }
            else
            {
                a = x1; x1 = x2; f1 = f2;
                x2 = a + phi * (b - a); f2 = distanceAt(x2);
            }
        }
        return f1 <= f2 ? (x1, f1) : (x2, f2);
    }

    /// <summary>1-D solve for a target periapsis: vary x (the burn's prograde VLF
    /// component) until <paramref name="evaluate"/> — the achieved periapsis
    /// distance, null when that candidate is invalid — meets <paramref name="target"/> within
    /// <paramref name="tolerance"/>. Expanding probe (doubling steps) until the
    /// objective changes sign against the nearest evaluated point on that side, then
    /// a regula-falsi/bisection hybrid inside the bracket. The probe walks the side
    /// the ±probeStep slope says approaches the target FIRST, and only then the
    /// other (whether prograde raises or lowers the periapsis depends on where the
    /// burn sits, but the wrong side's deep candidates — suicide plunges, ejections —
    /// are expensive integrations for the caller, so they are the fallback, not half
    /// of every search). An invalid probe blocks only its direction, so the opposite
    /// side remains available. Null when the baseline is invalid, the solve is
    /// cancelled, or no bracket exists within <paramref name="maxOffset"/> on either
    /// side; a bracket
    /// that runs out of iterations returns its best endpoint (the closest achieved
    /// answer, honestly reported by the caller).</summary>
    public static (double X, double Achieved)? SolveForTarget(Func<double, double?> evaluate,
        double x0, double target, double probeStep, double maxOffset, double tolerance,
        Func<bool>? cancelled = null)
    {
        cancelled ??= static () => false;
        if (cancelled() || evaluate(x0) is not { } f0 || cancelled()) return null;
        if (Math.Abs(f0 - target) <= tolerance) return (x0, f0);

        double? plus = evaluate(x0 + probeStep);
        if (cancelled()) return null;
        if (plus is { } fPlus)
        {
            if (Math.Abs(fPlus - target) <= tolerance) return (x0 + probeStep, fPlus);
            if ((fPlus - target) * (f0 - target) <= 0)
            {
                var result = Bracketed(evaluate, x0, f0, x0 + probeStep, fPlus,
                    target, tolerance, cancelled);
                if (result is not null || cancelled()) return result;
                plus = null;
            }
        }

        double? minus = evaluate(x0 - probeStep);
        if (cancelled()) return null;
        if (minus is { } fMinus)
        {
            if (Math.Abs(fMinus - target) <= tolerance) return (x0 - probeStep, fMinus);
            if ((fMinus - target) * (f0 - target) <= 0)
            {
                var result = Bracketed(evaluate, x0 - probeStep, fMinus, x0, f0,
                    target, tolerance, cancelled);
                if (result is not null || cancelled()) return result;
                minus = null;
            }
        }

        if (plus is null && minus is null) return null;

        // Promising side first: the one whose unit-step slope moves toward the target
        // (ties/zero slope: positive first). A missing near probe blocks that side.
        bool positiveFirst = minus is null
            || plus is not null && (plus.Value - minus.Value) * (target - f0) >= 0;
        var firstNear = positiveFirst ? plus : minus;
        var first = firstNear is { } firstF
            ? ProbeSide(evaluate, x0, firstF, positive: positiveFirst,
                probeStep, maxOffset, target, tolerance, cancelled)
            : null;
        if (first is not null || cancelled()) return first;

        var secondNear = positiveFirst ? minus : plus;
        return secondNear is { } secondF
            ? ProbeSide(evaluate, x0, secondF, positive: !positiveFirst,
                probeStep, maxOffset, target, tolerance, cancelled)
            : null;
    }

    /// <summary>Target solve with a minimum-error fallback for tangent roots. The
    /// sign-change solve remains the fast path; when it cannot bracket, exponentially
    /// spaced samples locate the best basin and golden-section refinement minimizes
    /// |evaluate(x)-target| inside its neighboring samples.</summary>
    public static (double X, double Achieved)? SolveForTargetIncludingTangencies(
        Func<double, double?> evaluate, double x0, double target,
        double probeStep, double maxOffset, double tolerance)
    {
        var samples = new SortedDictionary<double, double>();
        double? Cached(double x)
        {
            if (samples.TryGetValue(x, out double value)) return value;
            if (evaluate(x) is not { } achieved) return null;
            samples[x] = achieved;
            return achieved;
        }
        if (SolveForTarget(Cached, x0, target, probeStep, maxOffset, tolerance) is { } root)
            return root;
        for (double step = probeStep; step <= maxOffset; step *= 2)
        {
            Cached(x0 - step);
            Cached(x0 + step);
        }
        if (samples.Count < 3) return null;
        var points = samples.ToArray();
        int best = 0;
        for (int i = 1; i < points.Length; i++)
            if (Math.Abs(points[i].Value - target) < Math.Abs(points[best].Value - target))
                best = i;
        if (Math.Abs(points[best].Value - target) <= tolerance)
            return (points[best].Key, points[best].Value);
        const double phi = 0.6180339887498949;
        double a = points[Math.Max(0, best - 1)].Key;
        double b = points[Math.Min(points.Length - 1, best + 1)].Key;
        double x1 = b - phi * (b - a), x2 = a + phi * (b - a);
        double Error(double x, out double achieved)
        {
            achieved = Cached(x) ?? double.NaN;
            return double.IsFinite(achieved) ? Math.Abs(achieved - target) : double.PositiveInfinity;
        }
        double e1 = Error(x1, out double f1), e2 = Error(x2, out double f2);
        for (int i = 0; i < 60 && b - a > 1e-9 && Math.Min(e1, e2) > tolerance; i++)
        {
            if (e1 <= e2)
            {
                b = x2; x2 = x1; e2 = e1; f2 = f1;
                x1 = b - phi * (b - a); e1 = Error(x1, out f1);
            }
            else
            {
                a = x1; x1 = x2; e1 = e2; f1 = f2;
                x2 = a + phi * (b - a); e2 = Error(x2, out f2);
            }
        }
        return Math.Min(e1, e2) <= tolerance
            ? e1 <= e2 ? (x1, f1) : (x2, f2)
            : null;
    }

    /// <summary>Doubling probe along one side from |x - x0| = 2·probeStep outward
    /// (±probeStep was already evaluated by the caller — its value seeds the
    /// nearest-point bracket anchor). An invalid probe blocks this direction.</summary>
    private static (double X, double Achieved)? ProbeSide(Func<double, double?> evaluate,
        double x0, double nearProbeF, bool positive, double probeStep, double maxOffset,
        double target, double tolerance, Func<bool> cancelled)
    {
        double sign = positive ? 1.0 : -1.0;
        double nearX = x0 + sign * probeStep, nearF = nearProbeF;
        for (double step = probeStep * 2; step <= maxOffset && !cancelled(); step *= 2)
        {
            double x = x0 + sign * step;
            if (evaluate(x) is not { } f || cancelled()) return null;
            if ((f - target) * (nearF - target) <= 0)
                return Bracketed(evaluate, nearX, nearF, x, f, target, tolerance, cancelled);
            nearX = x;
            nearF = f;
        }
        return null;
    }

    /// <summary>First-periapsis extraction shared by the optimizer objectives: a
    /// coarse uniform scan over the window finds the first interior local minimum of
    /// <paramref name="distanceAt"/> and golden-section refines it (Interior true —
    /// a real periapsis PASS); a monotone window falls back to the closest sampled
    /// approach (Interior false — a bracketing-continuity value, never a reportable
    /// periapsis).</summary>
    public static (double PeriapsisMeters, bool Interior) ScanFirstPeriapsis(
        Func<double, double> distanceAt, double scanStart, double scanEnd, int samples = 512)
    {
        Span<double> distances = (uint)samples <= 1024u
            ? stackalloc double[samples]
            : new double[samples];
        double step = (scanEnd - scanStart) / (samples - 1);
        for (int i = 0; i < samples; i++)
        {
            double time = scanStart + i * step;
            distances[i] = distanceAt(time);
        }
        int minimum = FirstLocalMinimum(distances);
        if (minimum >= 1)
        {
            (_, double refined) = RefineMinimum(
                distanceAt,
                scanStart + (minimum - 1) * step,
                scanStart + (minimum + 1) * step,
                distanceTolerance: 1.0);
            return (refined, true);
        }
        int best = 0;
        for (int i = 1; i < samples; i++)
            if (distances[i] < distances[best]) best = i;
        return (distances[best], false);
    }

    private static int FirstLocalMinimum(ReadOnlySpan<double> values)
    {
        for (int i = 1; i < values.Length - 1; i++)
        {
            if (values[i] >= values[i - 1]) continue;
            int plateauEnd = i;
            while (plateauEnd + 1 < values.Length
                && values[plateauEnd + 1] == values[i]) plateauEnd++;
            if (plateauEnd + 1 < values.Length
                && values[plateauEnd + 1] > values[i])
                return i;
            i = plateauEnd;
        }
        return -1;
    }

    /// <summary>Minimum-delta-v periapsis optimize (the 4-D upgrade): minimize the
    /// burn's TOTAL |dv| over (time, normal, outward) while the inner 1-D prograde
    /// solve — <paramref name="solveAt"/>(time, normal, outward, progradeHint),
    /// returning the prograde component that hits the target periapsis at that
    /// point, or null where it cannot — keeps every evaluated point ON the
    /// constraint. The hint is always the CURRENT BEST point's prograde (never a
    /// rejected probe's), so the inner solve warm-starts near the constraint branch
    /// the search is walking and the objective stays evaluation-order independent.
    /// Constraint projection through the dominant control makes the outer objective
    /// smooth-ish and 3-D; the outer search is a derivative-free compass/pattern
    /// search (tiny ± probes per axis, move to the best improvement, halve the
    /// steps when none improves), which needs no gradients and treats refused
    /// points (no bracket, window-edge Pe, diverged candidates) simply as
    /// non-moves — the same refusal honesty the 1-D solve ships with. Null when
    /// even the starting point has no inner solution. The start and every time
    /// probe clamp to [<paramref name="timeLo"/>, <paramref name="timeHi"/>]; steps
    /// clamp at their floors (an axis whose floor is reached first stops refining
    /// while the other continues), and the search exits when no axis improves at
    /// both floors. <paramref name="cancelled"/> stops it at the best point so far.</summary>
    public static DvMinimum? MinimizeDeltaV(
        Func<double, double, double, double, (double Prograde, double Achieved)?> solveAt,
        double time0, double normal0, double outward0, double timeLo, double timeHi,
        double timeStep, double dvStep, double timeStepFloor, double dvStepFloor,
        Func<bool> cancelled)
        => MinimizeDeltaV(solveAt, OptimizerConstraint.Prograde,
            time0, normal0, outward0, timeLo, timeHi,
            timeStep, dvStep, timeStepFloor, dvStepFloor, cancelled);

    /// <summary>Generalized constrained optimize. For a prograde constraint the two
    /// outer components are (normal, outward); for a normal constraint they are
    /// (prograde, outward). The returned point always uses actual VLF component names.</summary>
    public static DvMinimum? MinimizeDeltaV(
        Func<double, double, double, double, (double Constrained, double Achieved)?> solveAt,
        OptimizerConstraint constraint,
        double time0, double outer0, double outward0, double timeLo, double timeHi,
        double timeStep, double dvStep, double timeStepFloor, double dvStepFloor,
        Func<bool> cancelled)
    {
        // NaN hint at the start: the caller supplies its own x0 (the authored
        // prograde) for the first solve; every later hint is the best point's.
        time0 = Math.Clamp(time0, timeLo, timeHi);
        if (solveAt(time0, outer0, outward0, double.NaN) is not { } start) return null;
        DvMinimum Point(double time, double outer, double outward,
            double constrained, double achieved) => constraint == OptimizerConstraint.Prograde
                ? new DvMinimum(time, constrained, outer, outward, achieved)
                : new DvMinimum(time, outer, constrained, outward, achieved);
        double Outer(DvMinimum point) => constraint == OptimizerConstraint.Prograde
            ? point.Normal : point.Prograde;
        double Constrained(DvMinimum point) => constraint == OptimizerConstraint.Prograde
            ? point.Prograde : point.Normal;
        var best = Point(time0, outer0, outward0, start.Constrained, start.Achieved);
        const int maxIterations = 256; // backstop; the step floors are the real exit
        Span<(double Time, double Outer, double Outward)> probes =
            stackalloc (double, double, double)[6];
        for (int i = 0; i < maxIterations && !cancelled(); i++)
        {
            DvMinimum? move = null;
            double bestOuter = Outer(best);
            probes[0] = (Math.Clamp(best.TimeSeconds + timeStep, timeLo, timeHi), bestOuter, best.Outward);
            probes[1] = (Math.Clamp(best.TimeSeconds - timeStep, timeLo, timeHi), bestOuter, best.Outward);
            probes[2] = (best.TimeSeconds, bestOuter + dvStep, best.Outward);
            probes[3] = (best.TimeSeconds, bestOuter - dvStep, best.Outward);
            probes[4] = (best.TimeSeconds, bestOuter, best.Outward + dvStep);
            probes[5] = (best.TimeSeconds, bestOuter, best.Outward - dvStep);
            foreach (var (t, n, o) in probes)
            {
                if (cancelled()) break;
                if (t == best.TimeSeconds && n == bestOuter && o == best.Outward) continue;
                if (solveAt(t, n, o, Constrained(best)) is not { } solved) continue;
                var candidate = Point(t, n, o, solved.Constrained, solved.Achieved);
                if (candidate.Magnitude < (move?.Magnitude ?? best.Magnitude)) move = candidate;
            }
            if (move is not null)
            {
                best = move;
                continue;
            }
            // No axis improves at the current scale: refine, each axis clamped at
            // its own floor (never probe below it), and exit once BOTH sit there.
            bool timeAtFloor = timeStep <= timeStepFloor;
            bool dvAtFloor = dvStep <= dvStepFloor;
            if (timeAtFloor && dvAtFloor) break;
            if (!timeAtFloor) timeStep = Math.Max(timeStep * 0.5, timeStepFloor);
            if (!dvAtFloor) dvStep = Math.Max(dvStep * 0.5, dvStepFloor);
        }
        return best;
    }

    /// <summary>Minimizes delta-v over time/outward after Pe/inclination projection.</summary>
    public static DvMinimum? MinimizeDeltaVWithInclination(
        Func<double, double, double, double, CoupledTargetSolution?> solveAt,
        double time0, double outward0, double timeLo, double timeHi,
        double timeStep, double dvStep, double timeStepFloor, double dvStepFloor,
        Func<bool> cancelled)
    {
        time0 = Math.Clamp(time0, timeLo, timeHi);
        if (solveAt(time0, outward0, double.NaN, double.NaN) is not { } start) return null;
        DvMinimum Point(double time, double outward, CoupledTargetSolution solved) =>
            new(time, solved.Prograde, solved.Normal, outward,
                solved.AchievedPeriapsis, solved.AchievedInclination);
        var best = Point(time0, outward0, start);
        const int maxIterations = 256;
        Span<(double Time, double Outward)> probes =
            stackalloc (double, double)[4];
        for (int i = 0; i < maxIterations && !cancelled(); i++)
        {
            DvMinimum? move = null;
            probes[0] = (Math.Clamp(best.TimeSeconds + timeStep, timeLo, timeHi), best.Outward);
            probes[1] = (Math.Clamp(best.TimeSeconds - timeStep, timeLo, timeHi), best.Outward);
            probes[2] = (best.TimeSeconds, best.Outward + dvStep);
            probes[3] = (best.TimeSeconds, best.Outward - dvStep);
            foreach (var (time, outward) in probes)
            {
                if (cancelled()) break;
                if (time == best.TimeSeconds && outward == best.Outward) continue;
                if (solveAt(time, outward, best.Prograde, best.Normal) is not { } solved) continue;
                var candidate = Point(time, outward, solved);
                if (candidate.Magnitude < (move?.Magnitude ?? best.Magnitude)) move = candidate;
            }
            if (move is not null)
            {
                best = move;
                continue;
            }
            bool timeAtFloor = timeStep <= timeStepFloor;
            bool dvAtFloor = dvStep <= dvStepFloor;
            if (timeAtFloor && dvAtFloor) break;
            if (!timeAtFloor) timeStep = Math.Max(timeStep * 0.5, timeStepFloor);
            if (!dvAtFloor) dvStep = Math.Max(dvStep * 0.5, dvStepFloor);
        }
        return best;
    }

    /// <summary>Improves inclination while keeping every candidate projected onto
    /// the already-established periapsis constraint. Inclination is a best-effort
    /// secondary objective: accept a point whenever it is closer to the target and
    /// remains below the caller's delta-v ceiling. Cancellation returns the best
    /// fixed-periapsis point found so far.</summary>
    public static DvMinimum? ImproveInclinationAtFixedPeriapsis(
        Func<double, double, double, double,
            (double Prograde, double AchievedPeriapsis, double AchievedInclination)?> solveAt,
        double time0, double normal0, double outward0, double inclinationTarget,
        double inclinationTolerance,
        double maxMagnitude, double timeLo, double timeHi,
        double timeStep, double dvStep, double timeStepFloor, double dvStepFloor,
        Func<bool> cancelled)
    {
        time0 = Math.Clamp(time0, timeLo, timeHi);
        if (solveAt(time0, normal0, outward0, double.NaN) is not { } start) return null;
        DvMinimum Point(double time, double normal, double outward,
            (double Prograde, double AchievedPeriapsis, double AchievedInclination) solved) =>
            new(time, solved.Prograde, normal, outward,
                solved.AchievedPeriapsis, solved.AchievedInclination);
        var best = Point(time0, normal0, outward0, start);
        if (best.Magnitude > maxMagnitude) return null;
        double bestError = Math.Abs(start.AchievedInclination - inclinationTarget);
        const double angularNoiseFloor = 1e-12;
        const int maxIterations = 4096;
        Span<(double Time, double Normal, double Outward)> probes =
            stackalloc (double, double, double)[6];
        for (int i = 0; i < maxIterations && bestError > inclinationTolerance
             && !cancelled(); i++)
        {
            DvMinimum? move = null;
            double moveError = bestError;
            probes[0] = (Math.Clamp(best.TimeSeconds + timeStep, timeLo, timeHi), best.Normal, best.Outward);
            probes[1] = (Math.Clamp(best.TimeSeconds - timeStep, timeLo, timeHi), best.Normal, best.Outward);
            probes[2] = (best.TimeSeconds, best.Normal + dvStep, best.Outward);
            probes[3] = (best.TimeSeconds, best.Normal - dvStep, best.Outward);
            probes[4] = (best.TimeSeconds, best.Normal, best.Outward + dvStep);
            probes[5] = (best.TimeSeconds, best.Normal, best.Outward - dvStep);
            foreach (var (time, normal, outward) in probes)
            {
                if (cancelled()) break;
                if (time == best.TimeSeconds && normal == best.Normal && outward == best.Outward)
                    continue;
                if (solveAt(time, normal, outward, best.Prograde) is not { } solved) continue;
                var candidate = Point(time, normal, outward, solved);
                if (candidate.Magnitude > maxMagnitude) continue;
                double error = Math.Abs(solved.AchievedInclination - inclinationTarget);
                if (!(error < bestError - angularNoiseFloor)) continue;
                if (move is null || error < moveError - angularNoiseFloor
                    || (Math.Abs(error - moveError) <= angularNoiseFloor
                        && candidate.Magnitude < move.Magnitude))
                {
                    move = candidate;
                    moveError = error;
                }
            }
            if (move is not null)
            {
                best = move;
                bestError = moveError;
                continue;
            }
            bool timeAtFloor = timeStep <= timeStepFloor;
            bool dvAtFloor = dvStep <= dvStepFloor;
            if (timeAtFloor && dvAtFloor) break;
            if (!timeAtFloor) timeStep = Math.Max(timeStep * 0.5, timeStepFloor);
            if (!dvAtFloor) dvStep = Math.Max(dvStep * 0.5, dvStepFloor);
        }
        return best;
    }

    /// <summary>Root refinement inside a sign-change bracket: regula falsi for
    /// speed alternated with bisection for guaranteed interval shrinkage (plain
    /// false position stalls on one stagnant endpoint when the objective is steep
    /// near the body). Exits on the achieved-distance tolerance; iteration or fp
    /// exhaustion returns the best endpoint seen.</summary>
    private static (double X, double Achieved)? Bracketed(Func<double, double?> evaluate,
        double aX, double aF, double bX, double bF, double target, double tolerance,
        Func<bool> cancelled)
    {
        for (int i = 0; i < 60; i++)
        {
            if (cancelled()) return null;
            if (Math.Abs(aF - target) <= tolerance) return (aX, aF);
            if (Math.Abs(bF - target) <= tolerance) return (bX, bF);
            double x;
            double denominator = bF - aF;
            if (i % 2 == 0 && Math.Abs(denominator) > 0)
            {
                x = bX - (bF - target) * (bX - aX) / denominator; // regula falsi
                double lo = Math.Min(aX, bX), hi = Math.Max(aX, bX);
                if (!(x > lo && x < hi)) x = 0.5 * (aX + bX);     // degenerate secant
            }
            else
            {
                x = 0.5 * (aX + bX);                              // bisection turn
            }
            if (x == aX || x == bX) break; // fp resolution exhausted
            if (evaluate(x) is not { } f || cancelled()) return null;
            if ((f - target) * (aF - target) <= 0) { bX = x; bF = f; }
            else { aX = x; aF = f; }
        }
        return Math.Abs(aF - target) <= Math.Abs(bF - target) ? (aX, aF) : (bX, bF);
    }
}

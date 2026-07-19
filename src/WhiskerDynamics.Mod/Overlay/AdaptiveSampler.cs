using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

/// <summary>One sampled display path: strictly increasing times, matching positions in
/// the DRAWN coordinates, whether it ended before the horizon, whether the caller's
/// optional work slice caused that early end, and whether integration dynamics rather
/// than the point budget caused it.</summary>
public sealed record AdaptivePath(
    double[] Times, Vector3d[] Positions, bool Truncated, bool WorkLimited = false)
{
    public bool DynamicsLimited { get; init; }
}

/// <summary>Honest-orbit-lines smoothness kernel (KSA-free): walks a trajectory
/// with an adaptive time step so the TURN ANGLE between consecutive chords stays under a
/// bound — the n-body analog of stock's true-anomaly-spaced points (UpdateTaskUtils.
/// GenerateSpacedPoints: 2000 points, angle-uniform). Uniform-time sampling starves
/// periapsis and aliases fast orbits; bounding the chord turn puts points exactly where
/// the drawn line curves. The criterion runs in whatever coordinates the caller's
/// closure returns (frame coordinates when a display frame is active — re-embedding at
/// the now-pose is a similarity, rotation plus one uniform scale, so angles are
/// preserved even for rotating-pulsating two-body frames; parent-relative otherwise).</summary>
public static class AdaptiveSampler
{
    /// <summary>Growth hysteresis: dt doubles only when the accepted turn was under
    /// bound/2, so a grown step's turn lands at most at the bound (turn is linear in
    /// dt at these angles) and the step does not oscillate (grow, fail, halve, grow)
    /// — every proposal is still checked directly, so this is an evaluation-count
    /// guard, not the correctness argument. The step therefore PARKS with turns in
    /// [bound/2, bound], so real density can reach 2× the bound's nominal.</summary>
    private const double GrowWhenUnder = 2.0;

    public static AdaptivePath Sample(Func<double, Vector3d> position, double t0, double horizon,
        int maxPoints, double thetaMaxRad, double dtMinSeconds, double periodHintSeconds,
        Func<bool>? shouldStop = null, Action<double>? accepted = null)
    {
        if (horizon <= t0) throw new ArgumentOutOfRangeException(nameof(horizon), "horizon must exceed t0");
        if (maxPoints < 2) throw new ArgumentOutOfRangeException(nameof(maxPoints));
        if (!(dtMinSeconds > 0)) throw new ArgumentOutOfRangeException(nameof(dtMinSeconds), "dtMinSeconds must be positive");
        double span = horizon - t0;
        // Max step: the 16-point floor of the uniform sampler, AND >= 8 samples per
        // revolution when a period is known — the anti-aliasing cap (a step near one
        // period collapses chords and the turn test alone would accept it).
        double dtMax = span / 16.0;
        if (double.IsFinite(periodHintSeconds) && periodHintSeconds > 0)
            dtMax = Math.Min(dtMax, periodHintSeconds / 8.0);
        dtMax = Math.Max(dtMax, dtMinSeconds);
        // Seed at the floor: the FIRST segment has no previous chord to measure a turn
        // against, so it is accepted unchecked — seeding at dtMin makes that chord the
        // shortest the sampler can ever emit (an oversized unchecked first chord would
        // force every following turn over the bound however small the next step gets).
        // Doubling recovers the right step in O(log) accepted segments.
        double dt = dtMinSeconds;

        // Presize toward the budget (capped so short celestial arcs don't pay for
        // the vessel budget): dense sweeps at 16k+ would otherwise double the
        // backing arrays ~7 times, with the late copies landing on the LOH.
        // Cancellable sweeps can stop after only a few dozen probes. Reserving the
        // full dense-sweep capacity there would waste a large allocation.
        int capacity = Math.Min(maxPoints, shouldStop is null ? 8192 : 128);
        var times = new List<double>(capacity) { t0 };
        var points = new List<Vector3d>(capacity) { position(t0) };
        accepted?.Invoke(t0);
        bool truncated = false;
        bool workLimited = false;
        bool dynamicsLimited = false;
        while (times[^1] < horizon)
        {
            if (times.Count >= maxPoints) { truncated = true; break; }
            // Observe work budgets between trajectory probes. StateAt(t) is the
            // indivisible unit; always leave downstream consumers at least one chord.
            if (times.Count >= 2 && shouldStop?.Invoke() == true)
            {
                truncated = workLimited = true;
                break;
            }
            double tNext = Math.Min(times[^1] + dt, horizon);
            Vector3d pNext;
            try
            {
                pNext = position(tNext);
            }
            catch (IntegrationFailureException)
            {
                // A point-mass plunge can make the requested interval cross a
                // singularity. Refine toward the last integrable point so the line
                // ends at the body instead of disappearing with the failed batch.
                if (dt > dtMinSeconds)
                {
                    dt = Math.Max(dt / 2.0, dtMinSeconds);
                    continue;
                }
                truncated = dynamicsLimited = true;
                break;
            }
            if (points.Count >= 2)
            {
                double turn = TurnAngle(points[^2], points[^1], pNext);
                if (turn > thetaMaxRad && dt > dtMinSeconds)
                {
                    dt = Math.Max(dt / 2.0, dtMinSeconds);
                    continue; // re-propose the shorter segment
                }
                // At dt == dtMin an over-bound turn is a genuine corner no subdivision
                // can smooth: accept it and keep walking. Growth runs even from dtMin so
                // the step recovers after the corner instead of crawling to the horizon.
                if (turn < thetaMaxRad / GrowWhenUnder) dt = Math.Min(dt * 2.0, dtMax);
            }
            times.Add(tNext);
            points.Add(pNext);
            accepted?.Invoke(tNext);
            if (times[^1] < horizon && times.Count >= 2 && shouldStop?.Invoke() == true)
            {
                truncated = workLimited = true;
                break;
            }
        }
        return new AdaptivePath([.. times], [.. points], truncated, workLimited)
        {
            DynamicsLimited = dynamicsLimited,
        };
    }

    /// <summary>Angle between chords (a->b) and (b->c); 0 when either chord is
    /// degenerate (no evidence of a turn — a stationary path must not halve forever).</summary>
    public static double TurnAngle(Vector3d a, Vector3d b, Vector3d c)
    {
        var u = b - a;
        var v = c - b;
        // Only zero/non-zero is needed here. Taking two square roots merely to
        // discard their magnitudes dominated this per-proposal hot path.
        if (u.LengthSquared() <= 0 || v.LengthSquared() <= 0) return 0.0;
        double cross = u.Cross(v).Length();
        double dot = u.Dot(v);
        return Math.Atan2(cross, dot);
    }

    /// <summary>Orbital period from one relative state via vis-viva (1/a = 2/r − v²/µ);
    /// PositiveInfinity for parabolic/hyperbolic states — callers use this as the
    /// anti-aliasing step hint and the celestial window clamp.</summary>
    public static double PeriodSeconds(double mu, Vector3d relPosition, Vector3d relVelocity)
    {
        double r = relPosition.Length();
        if (r <= 0 || mu <= 0) return double.PositiveInfinity;
        double invA = 2.0 / r - relVelocity.LengthSquared() / mu;
        if (invA <= 0) return double.PositiveInfinity;
        double a = 1.0 / invA;
        return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
    }
}

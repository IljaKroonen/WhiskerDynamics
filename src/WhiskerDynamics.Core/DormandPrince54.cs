namespace WhiskerDynamics.Core;

/// <summary>Adaptive Dormand–Prince RK5(4)7M integrator specialised to orbital state vectors.</summary>
public static class DormandPrince54
{
    private static readonly double[] C = [0, 1.0 / 5, 3.0 / 10, 4.0 / 5, 8.0 / 9, 1, 1];

    private static readonly double[][] A =
    [
        [],
        [1.0 / 5],
        [3.0 / 40, 9.0 / 40],
        [44.0 / 45, -56.0 / 15, 32.0 / 9],
        [19372.0 / 6561, -25360.0 / 2187, 64448.0 / 6561, -212.0 / 729],
        [9017.0 / 3168, -355.0 / 33, 46732.0 / 5247, 49.0 / 176, -5103.0 / 18656],
        [35.0 / 384, 0, 500.0 / 1113, 125.0 / 192, -2187.0 / 6784, 11.0 / 84],
    ];

    private static readonly double[] B4 = [5179.0 / 57600, 0, 7571.0 / 16695, 393.0 / 640, -92097.0 / 339200, 187.0 / 2100, 1.0 / 40];

    public static StateVector Propagate(
        Func<double, StateVector, Vector3d> acceleration,
        StateVector y0, double t0, double t1,
        IntegratorOptions? options = null,
        Action<double, StateVector>? onAcceptedStep = null,
        CancellationToken cancellationToken = default)
    {
        if (!double.IsFinite(t0))
            throw new ArgumentOutOfRangeException(nameof(t0), "Initial time must be finite.");
        if (!double.IsFinite(t1))
            throw new ArgumentOutOfRangeException(nameof(t1), "Final time must be finite.");
        if (t1 < t0) throw new ArgumentException("Backward integration is not supported (t1 < t0).");
        if (!y0.IsFinite())
            throw new ArgumentException("Initial state must be finite.", nameof(y0));
        var opt = IntegratorOptions.Validate(options);

        double t = t0;
        var y = y0;
        double h = Math.Min(Math.Min(opt.InitialStep, opt.MaxStep), Math.Max(t1 - t0, double.Epsilon));
        var k = new StateVector[7];
        bool k0Valid = false; // k[0] = f(t, y); stays valid across rejections (t, y unchanged)
        int consecutiveRejections = 0;

        while (t < t1)
        {
            cancellationToken.ThrowIfCancellationRequested();
            h = Math.Min(h, t1 - t);

            if (!k0Valid) { k[0] = Derivative(acceleration, t, y); k0Valid = true; }
            var y5 = y; // stage 7 uses the fifth-order weights, so it is the fifth-order solution
            for (int s = 1; s < 7; s++)
            {
                var ys = y;
                for (int j = 0; j < s; j++)
                    if (A[s][j] != 0)
                        ys += k[j] * (h * A[s][j]);
                if (s == 6) y5 = ys;
                k[s] = Derivative(acceleration, t + C[s] * h, ys);
            }

            var y4 = y;
            for (int j = 0; j < 7; j++)
                if (B4[j] != 0) y4 += k[j] * (h * B4[j]);

            double err = ErrorNorm(y, y5, y4, opt);

            if (double.IsNaN(err) || double.IsInfinity(err))
            {
                if (++consecutiveRejections > 60)
                    throw new IntegrationFailureException($"Integration diverged (NaN) at t={t}.");
                h *= 0.2;
                if (h < MinStep(t))
                    throw new IntegrationFailureException($"Step size underflow at t={t} — dynamics too stiff or divergent.");
                continue;
            }

            if (err <= 1.0)
            {
                consecutiveRejections = 0;
                t += h;
                y = y5;
                k[0] = k[6]; // FSAL: k[6] = f(t + h, y5) = f at the new (t, y)
                onAcceptedStep?.Invoke(t, y);
                if (t >= t1) return y; // final step needs no next-step controller work
            }
            else if (++consecutiveRejections > 60)
            {
                throw new IntegrationFailureException($"Integration failed: 60 consecutive step rejections at t={t}.");
            }

            double factor = err <= 0 ? 5.0 : 0.9 * Math.Pow(1.0 / err, 0.2);
            h *= Math.Clamp(factor, 0.2, 5.0);
            h = Math.Min(h, opt.MaxStep);
            if (h < MinStep(t) && t < t1)
                throw new IntegrationFailureException($"Step size underflow at t={t} — dynamics too stiff or divergent.");
        }

        return y;
    }

    /// <summary>System overload. <paramref name="onAcceptedStep"/> receives the accepted
    /// state AND its derivative (per body: d = (velocity, acceleration) — the FSAL
    /// stage evaluated at the accepted node, so exposing it costs nothing); both arrays
    /// are reused internal buffers, valid only for the duration of the callback — copy
    /// them if kept.</summary>
    public static StateVector[] PropagateSystem(
        Func<double, StateVector[], Vector3d[]> accelerations,
        StateVector[] y0, double t0, double t1,
        IntegratorOptions? options = null,
        Action<double, StateVector[], StateVector[]>? onAcceptedStep = null,
        CancellationToken cancellationToken = default)
    {
        if (!double.IsFinite(t0))
            throw new ArgumentOutOfRangeException(nameof(t0), "Initial time must be finite.");
        if (!double.IsFinite(t1))
            throw new ArgumentOutOfRangeException(nameof(t1), "Final time must be finite.");
        if (t1 < t0) throw new ArgumentException("Backward integration is not supported (t1 < t0).");
        ArgumentNullException.ThrowIfNull(y0);
        if (Array.Exists(y0, state => !state.IsFinite()))
            throw new ArgumentException("Initial states must be finite.", nameof(y0));
        var opt = IntegratorOptions.Validate(options);
        if (t1 == t0) return (StateVector[])y0.Clone();
        int n = y0.Length;

        double t = t0;
        var y = (StateVector[])y0.Clone();
        double h = Math.Min(Math.Min(opt.InitialStep, opt.MaxStep), Math.Max(t1 - t0, double.Epsilon));

        // Per-call scratch, reused across steps: zero per-step allocation.
        var k = new StateVector[7][];
        for (int s = 0; s < 7; s++) k[s] = new StateVector[n];
        var ys = new StateVector[n];
        var y4 = new StateVector[n];
        bool k0Valid = false; // k[0] = f(t, y); stays valid across rejections (t, y unchanged)
        int consecutiveRejections = 0;

        while (t < t1)
        {
            cancellationToken.ThrowIfCancellationRequested();
            h = Math.Min(h, t1 - t);

            if (!k0Valid) { DerivativeSystem(accelerations, t, y, k[0]); k0Valid = true; }
            for (int s = 1; s < 7; s++)
            {
                Array.Copy(y, ys, n);
                for (int j = 0; j < s; j++)
                    if (A[s][j] != 0)
                    {
                        double w = h * A[s][j];
                        var kj = k[j];
                        for (int b = 0; b < n; b++)
                            ys[b] += kj[b] * w;
                    }
                DerivativeSystem(accelerations, t + C[s] * h, ys, k[s]);
            }
            // Stage 7 uses the fifth-order weights, so ys is the fifth-order solution,
            // and k[6] = f(t + h, y5) — the FSAL derivative.
            var y5 = ys;

            Array.Copy(y, y4, n);
            for (int j = 0; j < 7; j++)
                if (B4[j] != 0) { double w = h * B4[j]; var kj = k[j]; for (int b = 0; b < n; b++) y4[b] += kj[b] * w; }

            double errSq = 0;
            for (int b = 0; b < n; b++)
                errSq += ErrorSquaredSum(y[b], y5[b], y4[b], opt);
            // Each body's ErrorNorm is sqrt(sum / 6). The old formulation
            // squared those per-body roots again before taking the system RMS;
            // accumulate the component squares directly and pay for one sqrt.
            double err = Math.Sqrt(errSq / (6.0 * n));

            if (double.IsNaN(err) || double.IsInfinity(err))
            {
                if (++consecutiveRejections > 60)
                    throw new IntegrationFailureException($"System integration diverged (NaN) at t={t}.");
                h *= 0.2;
                if (h < MinStep(t))
                    throw new IntegrationFailureException($"Step size underflow at t={t} — dynamics too stiff or divergent.");
                continue;
            }

            if (err <= 1.0)
            {
                consecutiveRejections = 0;
                t += h;
                (y, ys) = (ys, y);         // adopt the 5th-order solution; the old y becomes scratch
                (k[0], k[6]) = (k[6], k[0]); // FSAL: k[6] was f at the new (t, y)
                onAcceptedStep?.Invoke(t, y, k[0]);
                if (t >= t1) return y; // final step needs no next-step controller work
            }
            else if (++consecutiveRejections > 60)
            {
                throw new IntegrationFailureException($"System integration failed: 60 consecutive step rejections at t={t}.");
            }

            double factor = err <= 0 ? 5.0 : 0.9 * Math.Pow(1.0 / err, 0.2);
            h *= Math.Clamp(factor, 0.2, 5.0);
            h = Math.Min(h, opt.MaxStep);
            if (h < MinStep(t) && t < t1)
                throw new IntegrationFailureException($"Step size underflow at t={t} — dynamics too stiff or divergent.");
        }

        return y;
    }

    /// <summary>Smallest step that can still make progress: 4 ulp of t, floored at 1e-9.
    /// Below one ulp of t, t + h == t and the loop would not advance. Internal so
    /// <see cref="DormandPrince853"/> can use the same underflow behavior.</summary>
    internal static double MinStep(double t)
    {
        double ulp = Math.BitIncrement(Math.Abs(t)) - Math.Abs(t);
        return Math.Max(1e-9, 4 * ulp);
    }

    private static void DerivativeSystem(
        Func<double, StateVector[], Vector3d[]> accelerations, double t, StateVector[] y, StateVector[] d)
    {
        var acc = accelerations(t, y);
        for (int b = 0; b < y.Length; b++)
            d[b] = new StateVector(y[b].Velocity, acc[b]);
    }

    private static StateVector Derivative(Func<double, StateVector, Vector3d> acceleration, double t, StateVector y) =>
        new(y.Velocity, acceleration(t, y));

    private static double ErrorNorm(StateVector y, StateVector y5, StateVector y4, IntegratorOptions opt)
    {
        return Math.Sqrt(ErrorSquaredSum(y, y5, y4, opt) / 6);
    }

    private static double ErrorSquaredSum(
        StateVector y, StateVector y5, StateVector y4, IntegratorOptions opt)
    {
        double rel = opt.RelTol;
        double pos = opt.AbsTolPos;
        double vel = opt.AbsTolVel;
        return ErrorTerm(y.Position.X, y5.Position.X, y4.Position.X, pos, rel)
             + ErrorTerm(y.Position.Y, y5.Position.Y, y4.Position.Y, pos, rel)
             + ErrorTerm(y.Position.Z, y5.Position.Z, y4.Position.Z, pos, rel)
             + ErrorTerm(y.Velocity.X, y5.Velocity.X, y4.Velocity.X, vel, rel)
             + ErrorTerm(y.Velocity.Y, y5.Velocity.Y, y4.Velocity.Y, vel, rel)
             + ErrorTerm(y.Velocity.Z, y5.Velocity.Z, y4.Velocity.Z, vel, rel);
    }

    private static double ErrorTerm(double now, double hi, double lo, double absTol, double relTol)
    {
        double scale = absTol + relTol * Math.Max(Math.Abs(now), Math.Abs(hi));
        double e = (hi - lo) / scale;
        return e * e;
    }
}

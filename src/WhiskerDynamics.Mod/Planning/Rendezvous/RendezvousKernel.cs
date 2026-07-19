using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning.Rendezvous;

/// <summary>Game-free orbital mathematics used by the automatic rendezvous planner.
/// The Lambert solve supplies short/long transfers at every requested revolution
/// count for a specified departure/arrival pair; the runtime job searches those pairs and then
/// differential-corrects the winner in the mod's full n-body gravity field.</summary>
public static class RendezvousKernel
{
    public readonly record struct LambertSolution(Vector3d DepartureVelocity,
        Vector3d ArrivalVelocity, int Revolutions = 0, bool HighPath = false);

    /// <summary>Solves the zero-revolution case. Both the short and long way
    /// are available; multi-revolution callers use <see cref="SolveLambert"/>.</summary>
    public static bool TryLambert(Vector3d from, Vector3d to, double timeOfFlight,
        double mu, bool longWay, out LambertSolution solution)
    {
        var solutions = SolveLambert(from, to, timeOfFlight, mu, longWay, 0);
        solution = solutions.Count > 0 ? solutions[0] : default;
        return solutions.Count > 0;
    }

    /// <summary>Universal-variable Lambert roots for one exact revolution count.
    /// Revolution zero has one root per short/long geometry. Every higher interval
    /// can have a low- and high-path root; both are returned when the requested time
    /// of flight exceeds that interval's minimum feasible time.</summary>
    public static IReadOnlyList<LambertSolution> SolveLambert(Vector3d from, Vector3d to,
        double timeOfFlight, double mu, bool longWay, int revolutions)
    {
        var solutions = new List<LambertSolution>(revolutions == 0 ? 1 : 2);
        if (revolutions < 0) return solutions;
        double r1 = from.Length(), r2 = to.Length();
        if (!(r1 > 0) || !(r2 > 0) || !(timeOfFlight > 0) || !(mu > 0))
            return solutions;
        double cosTheta = Math.Clamp(from.Dot(to) / (r1 * r2), -1.0, 1.0);
        double sinMagnitude = Math.Sqrt(Math.Max(0.0, 1.0 - cosTheta * cosTheta));
        double sinTheta = longWay ? -sinMagnitude : sinMagnitude;
        double denominator = 1.0 - cosTheta;
        if (denominator < 1e-12 || Math.Abs(sinTheta) < 1e-12) return solutions;
        double a = sinTheta * Math.Sqrt(r1 * r2 / denominator);
        if (!double.IsFinite(a) || Math.Abs(a) < 1e-12) return solutions;
        double requiredTime = Math.Sqrt(mu) * timeOfFlight;

        bool Evaluate(double z, out double residual, out double y)
        {
            var (c, s) = Stumpff(z);
            y = r1 + r2 + a * (z * s - 1.0) / Math.Sqrt(c);
            if (!(c > 0) || !(y >= 0) || !double.IsFinite(y))
            {
                residual = double.NaN;
                return false;
            }
            double x = Math.Sqrt(y / c);
            residual = x * x * x * s + a * Math.Sqrt(y) - requiredTime;
            return double.IsFinite(residual);
        }

        // C(z) vanishes at z=(2*pi*n)^2. The interval below the first singularity
        // contains the zero-revolution root; interval n contains the n-revolution
        // low/high pair. Uniform sampling in sqrt(z) gives every revolution equal
        // anomaly resolution even at large n, then bisection makes each root exact.
        int samples = revolutions == 0 ? 320 : 256;
        double zLoBound, zHiBound;
        double sqrtZLoBound = 0.0, sqrtZHiBound = 0.0;
        if (revolutions == 0)
        {
            zLoBound = -4.0 * Math.PI * Math.PI + 1e-7;
            zHiBound = 4.0 * Math.PI * Math.PI - 1e-7;
        }
        else
        {
            double qLo = 2.0 * Math.PI * revolutions + 1e-7;
            double qHi = 2.0 * Math.PI * (revolutions + 1) - 1e-7;
            zLoBound = qLo * qLo;
            zHiBound = qHi * qHi;
            sqrtZLoBound = Math.Sqrt(zLoBound);
            sqrtZHiBound = Math.Sqrt(zHiBound);
        }
        bool havePrevious = false;
        double previousZ = 0, previousF = 0;
        for (int i = 0; i <= samples; i++)
        {
            double z;
            if (revolutions == 0)
                z = zLoBound + (zHiBound - zLoBound) * i / samples;
            else
            {
                double q = sqrtZLoBound
                    + (sqrtZHiBound - sqrtZLoBound) * i / samples;
                z = q * q;
            }
            if (!Evaluate(z, out double f, out _)) { havePrevious = false; continue; }
            if (Math.Abs(f) < 1e-8)
            {
                AddSolution(z);
                havePrevious = false;
                continue;
            }
            if (havePrevious && Math.Sign(f) != Math.Sign(previousF))
            {
                double lo = previousZ, hi = z, flo = previousF;
                for (int iteration = 0; iteration < 100; iteration++)
                {
                    double mid = 0.5 * (lo + hi);
                    if (!Evaluate(mid, out double fm, out _)) break;
                    if (Math.Abs(fm) < 1e-8) { lo = hi = mid; break; }
                    if (Math.Sign(fm) == Math.Sign(flo)) { lo = mid; flo = fm; }
                    else hi = mid;
                }
                AddSolution(0.5 * (lo + hi));
                if (solutions.Count >= (revolutions == 0 ? 1 : 2)) break;
            }
            havePrevious = true; previousZ = z; previousF = f;
        }

        return solutions;

        void AddSolution(double root)
        {
            if (!Evaluate(root, out _, out double yFinal)) return;
            double fCoeff = 1.0 - yFinal / r1;
            double g = a * Math.Sqrt(yFinal / mu);
            double gDot = 1.0 - yFinal / r2;
            if (Math.Abs(g) < 1e-12 || !double.IsFinite(g)) return;
            var v1 = (to - from * fCoeff) / g;
            var v2 = (to * gDot - from) / g;
            if (!Finite(v1) || !Finite(v2)) return;
            if (solutions.Any(s => (s.DepartureVelocity - v1).Length() < 1e-7)) return;
            solutions.Add(new LambertSolution(v1, v2, revolutions, solutions.Count > 0));
        }
    }

    public static double OrbitalPeriod(in StateVector relativeState, double mu)
    {
        double r = relativeState.Position.Length();
        if (!(r > 0) || !(mu > 0)) return double.PositiveInfinity;
        double energy = 0.5 * relativeState.Velocity.LengthSquared() - mu / r;
        if (!(energy < 0)) return double.PositiveInfinity;
        double semiMajor = -mu / (2.0 * energy);
        return 2.0 * Math.PI * Math.Sqrt(semiMajor * semiMajor * semiMajor / mu);
    }

    /// <summary>All revolution counts when they fit the budget. Longer ranges retain
    /// a dense low-revolution prefix, then sample the remaining tail end-to-end. A
    /// longer user duration therefore cannot remove ordinary short transfers.</summary>
    internal static int[] RevolutionSamples(int maximum, int budget)
    {
        if (maximum <= 0) return [0];
        budget = Math.Max(2, budget);
        if (maximum + 1 <= budget) return Enumerable.Range(0, maximum + 1).ToArray();
        var samples = new SortedSet<int>();
        int prefixCount = Math.Max(1, budget / 2);
        for (int revolution = 0; revolution < prefixCount; revolution++)
            samples.Add(revolution);
        int tailCount = budget - samples.Count;
        int tailStart = samples.Max + 1;
        if (tailCount == 1) samples.Add(maximum);
        else
            for (int k = 0; k < tailCount; k++)
                samples.Add((int)Math.Round(tailStart
                    + (double)k * (maximum - tailStart) / (tailCount - 1)));
        return [.. samples];
    }

    /// <summary>Osculating-conic periapsis p/(1+e), valid for elliptic, parabolic,
    /// and hyperbolic states. NaN denotes a degenerate radial/non-finite state.</summary>
    public static double PeriapsisDistance(in StateVector relativeState, double mu)
    {
        if (!(mu > 0)) return double.NaN;
        var r = relativeState.Position;
        var v = relativeState.Velocity;
        double rMagnitude = r.Length();
        if (!(rMagnitude > 0)) return double.NaN;
        var h = r.Cross(v);
        double h2 = h.LengthSquared();
        if (!(h2 > 0)) return double.NaN;
        double eccentricity = (v.Cross(h) / mu - r / rMagnitude).Length();
        double periapsis = h2 / (mu * (1.0 + eccentricity));
        return double.IsFinite(periapsis) ? periapsis : double.NaN;
    }

    internal static bool TrySolveLinear3(Vector3d c0, Vector3d c1, Vector3d c2,
        Vector3d rhs, out Vector3d solution)
    {
        double det = c0.Dot(c1.Cross(c2));
        if (!double.IsFinite(det) || Math.Abs(det) < 1e-12)
        {
            solution = default;
            return false;
        }
        solution = new Vector3d(rhs.Dot(c1.Cross(c2)) / det,
            c0.Dot(rhs.Cross(c2)) / det, c0.Dot(c1.Cross(rhs)) / det);
        return Finite(solution);
    }

    /// <summary>Pivoted Gaussian solve used by finite-burn six-state shooting.</summary>
    internal static bool TrySolveLinear(double[,] coefficients, double[] rhs, out double[] solution)
    {
        int n = rhs.Length;
        solution = new double[n];
        if (coefficients.GetLength(0) != n || coefficients.GetLength(1) != n) return false;
        var a = (double[,])coefficients.Clone();
        var b = (double[])rhs.Clone();
        for (int column = 0; column < n; column++)
        {
            int pivot = column;
            double largest = Math.Abs(a[column, column]);
            for (int row = column + 1; row < n; row++)
                if (Math.Abs(a[row, column]) > largest)
                { pivot = row; largest = Math.Abs(a[row, column]); }
            if (!(largest > 1e-14) || !double.IsFinite(largest)) return false;
            if (pivot != column)
            {
                for (int k = column; k < n; k++) (a[column, k], a[pivot, k]) = (a[pivot, k], a[column, k]);
                (b[column], b[pivot]) = (b[pivot], b[column]);
            }
            for (int row = column + 1; row < n; row++)
            {
                double factor = a[row, column] / a[column, column];
                for (int k = column; k < n; k++) a[row, k] -= factor * a[column, k];
                b[row] -= factor * b[column];
            }
        }
        for (int row = n - 1; row >= 0; row--)
        {
            double value = b[row];
            for (int k = row + 1; k < n; k++) value -= a[row, k] * solution[k];
            solution[row] = value / a[row, row];
            if (!double.IsFinite(solution[row])) return false;
        }
        return true;
    }

    private static bool Finite(Vector3d v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);

    private static (double C, double S) Stumpff(double z)
    {
        if (z > 1e-3)
        {
            double q = Math.Sqrt(z);
            return ((1.0 - Math.Cos(q)) / z, (q - Math.Sin(q)) / (q * q * q));
        }
        if (z < -1e-3)
        {
            double q = Math.Sqrt(-z);
            return ((Math.Cosh(q) - 1.0) / (-z), (Math.Sinh(q) - q) / (q * q * q));
        }
        double z2 = z * z, z3 = z2 * z, z4 = z3 * z;
        return (0.5 - z / 24.0 + z2 / 720.0 - z3 / 40320.0 + z4 / 3628800.0,
            1.0 / 6.0 - z / 120.0 + z2 / 5040.0 - z3 / 362880.0 + z4 / 39916800.0);
    }
}

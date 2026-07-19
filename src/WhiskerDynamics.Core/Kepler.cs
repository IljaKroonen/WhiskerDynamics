namespace WhiskerDynamics.Core;

public static class Kepler
{
    /// <summary>Eccentricity at which element evaluation switches to universal-variable
    /// propagation because the elliptic anomaly solve becomes ill-conditioned.</summary>
    public const double UniversalPathEccentricity = 0.99;

    /// <summary>Solves M = E - e·sin(E) for E (elliptic, e &lt; 1). Newton with Danby's starter.</summary>
    public static double SolveEccentricAnomaly(double meanAnomaly, double eccentricity)
    {
        double M = Math.IEEERemainder(meanAnomaly, 2 * Math.PI);
        double sinM = Math.Sin(M);
        double E = M + 0.85 * eccentricity * Math.Sign(sinM == 0 ? 1 : sinM);
        for (int i = 0; i < 50; i++)
        {
            double f = E - eccentricity * Math.Sin(E) - M;
            double dE = f / (1 - eccentricity * Math.Cos(E));
            E -= dE;
            if (Math.Abs(dE) < 1e-14) return E;
        }
        throw new InvalidOperationException(
            $"Kepler solver failed to converge (M={meanAnomaly}, e={eccentricity})");
    }

    /// <summary>All-conic element evaluation. Elliptic below
    /// <see cref="UniversalPathEccentricity"/> keeps the classic anomaly solve;
    /// near-parabolic ellipses and hyperbolae (a &lt; 0, e &gt; 1) build the exact
    /// periapsis state from the elements and propagate it with
    /// <see cref="PropagateUniversal"/>. Corrupt data (e &lt; 1 with a &lt;= 0,
    /// e &gt; 1 with a &gt;= 0, an exact parabola — unrepresentable as (a, e)) is
    /// rejected with <see cref="NotSupportedException"/>.</summary>
    public static StateVector StateFromElements(in OrbitalElements el, double mu, double time)
    {
        double a = el.SemiMajorAxis, e = el.Eccentricity;
        if (e < 1 && a <= 0)
            throw new NotSupportedException($"corrupt elliptic elements (e={e:G6} < 1 with a={a:G6} <= 0)");
        if (e >= 1 && a >= 0)
            throw new NotSupportedException($"parabolic or corrupt elements (e={e:G6} >= 1 with a={a:G6} >= 0)");
        if (e > UniversalPathEccentricity)
        {
            // Periapsis state, exact from the elements: q = a(1-e) > 0 for both
            // signs of a on the branches admitted above; the velocity there is
            // purely transverse with v² = µ(1+e)/q (vis-viva at r = q).
            double q = PeriapsisDistance(el);
            if (!(q > 0) || !double.IsFinite(q))
                throw new NotSupportedException($"degenerate periapsis distance ({q:G6})");
            var periapsis = PerifocalStateToReference(
                new Vector3d(q, 0, 0),
                new Vector3d(0, PeriapsisSpeed(el, mu), 0), el);
            double dt = time - el.TimeAtPeriapsis;
            // Bound the universal anomaly for closed orbits: whole revolutions are
            // exact no-ops, and a many-period dt would leave Newton a needle to find.
            if (e < 1) dt = Math.IEEERemainder(dt, EllipticPeriod(a, mu));
            return PropagateUniversal(periapsis, mu, dt);
        }
        double n = MeanMotion(mu, a);
        double E = SolveEccentricAnomaly(n * (time - el.TimeAtPeriapsis), e);
        double cosE = Math.Cos(E), sinE = Math.Sin(E);
        double sqrt1mE2 = Math.Sqrt(1 - e * e);
        double radialFactor = 1 - e * cosE;
        double r = a * radialFactor;
        var posPerifocal = new Vector3d(a * (cosE - e), a * sqrt1mE2 * sinE, 0);
        double muTimesA = mu * a;
        double vScale = double.NaN;
        if (muTimesA > 0 && double.IsFinite(muTimesA))
            vScale = Math.Sqrt(muTimesA) / r;
        if (!(vScale > 0) || !double.IsFinite(vScale))
            vScale = Math.Sqrt(mu / a) / radialFactor;
        var velPerifocal = new Vector3d(-vScale * sinE, vScale * sqrt1mE2 * cosE, 0);
        return PerifocalStateToReference(posPerifocal, velPerifocal, el);
    }

    /// <summary>Periapsis distance q = a(1−e), positive for valid elliptic and
    /// hyperbolic elements.</summary>
    public static double PeriapsisDistance(in OrbitalElements el) =>
        el.SemiMajorAxis * (1 - el.Eccentricity);

    /// <summary>Parent-relative speed at periapsis: v² = µ(1+e)/q with q = a(1−e),
    /// one identity for elliptic and hyperbolic conics alike (vis-viva at r = q).
    /// Returns NaN for degenerate elements (q &lt;= 0).</summary>
    public static double PeriapsisSpeed(in OrbitalElements el, double mu)
    {
        double q = PeriapsisDistance(el);
        return q > 0 ? Math.Sqrt(mu * (1 + el.Eccentricity) / q) : double.NaN;
    }

    /// <summary>Two-body propagation of a state by <paramref name="dt"/> seconds via
    /// universal variables (Stumpff-function f-and-g solution, Vallado's formulation):
    /// one algorithm for elliptic, near-parabolic, and hyperbolic motion, convergent
    /// where the anomaly solvers are not. Newton on the universal Kepler equation —
    /// whose derivative is the orbital radius, so it is monotone — with a bisection
    /// fallback bracket for full robustness.</summary>
    public static StateVector PropagateUniversal(in StateVector state, double mu, double dt)
    {
        if (dt == 0) return state;
        var r0 = state.Position;
        var v0 = state.Velocity;
        double r0Mag = r0.Length();
        double sqrtMu = Math.Sqrt(mu);
        double alpha = 2.0 / r0Mag - v0.LengthSquared() / mu; // 1/a: >0 elliptic, <0 hyperbolic
        double rDotV = r0.Dot(v0);

        // Universal Kepler equation F(chi) = 0, F monotone increasing (F' = r >= 0).
        double F(double chi, out double dF)
        {
            double z = alpha * chi * chi;
            var (c2, c3) = Stumpff(z);
            double chi2 = chi * chi;
            double value = rDotV / sqrtMu * chi2 * c2
                + (1 - alpha * r0Mag) * chi2 * chi * c3
                + r0Mag * chi - sqrtMu * dt;
            dF = rDotV / sqrtMu * chi * (1 - z * c3)
                + (1 - alpha * r0Mag) * chi2 * c2 + r0Mag;
            return value;
        }

        // Starters (Vallado): elliptic from the mean rate; hyperbolic from the
        // asymptotic log form; near-parabolic from the dominant cubic term.
        double chi;
        if (alpha > 1e-12 / r0Mag)
        {
            chi = sqrtMu * dt * alpha;
        }
        else if (alpha < -1e-12 / r0Mag)
        {
            double aNeg = 1.0 / alpha;
            chi = Math.Sign(dt) * Math.Sqrt(-aNeg) * Math.Log(
                -2 * mu * alpha * dt
                / (rDotV + Math.Sign(dt) * Math.Sqrt(-mu * aNeg) * (1 - r0Mag * alpha)));
            if (!double.IsFinite(chi)) chi = Math.Sign(dt) * Math.Sqrt(-aNeg);
        }
        else
        {
            chi = Math.Cbrt(6 * sqrtMu * dt); // parabolic-dominant: chi^3·(1/6) ≈ sqrtMu·dt
        }

        bool converged = false;
        if (!double.IsFinite(chi)) chi = Math.Sign(dt);
        for (int i = 0; i < 60; i++)
        {
            double value = F(chi, out double dF);
            if (!(dF > 0) || !double.IsFinite(value)) break; // dF = r: 0 only at a
                                                             // rectilinear periapsis
            double step = value / dF;
            chi -= step;
            if (!double.IsFinite(chi)) break;
            if (Math.Abs(step) < 1e-12 * (1 + Math.Abs(chi))) { converged = true; break; }
        }
        if (!converged || !double.IsFinite(chi))
        {
            // Bisection fallback on the monotone F: bracket outward from 0 — F(0) =
            // -sqrtMu·dt carries dt's opposite sign — then halve. F overflowing to
            // ±Inf/NaN far past the root (near-rectilinear periapsis passes drive
            // Newton there) counts as the root side: F is monotone and finite at the
            // root, so a non-finite value can only lie beyond it. Unconditionally
            // convergent, and no Math.Sign(NaN) throw.
            bool loNegative = dt > 0; // sign of F(0)
            bool SameSideAsLo(double x)
            {
                double value = F(x, out _);
                if (!double.IsFinite(value)) return false;
                return value < 0 == loNegative;
            }
            double lo = 0;
            double hi = Math.Sign(dt) * (double.IsFinite(chi) && chi != 0
                ? Math.Abs(chi)
                : Math.Max(sqrtMu * Math.Abs(dt) / r0Mag, 1.0));
            int guard = 0;
            while (SameSideAsLo(hi) && guard++ < 200) hi *= 2;
            for (int i = 0; i < 200; i++)
            {
                double mid = 0.5 * (lo + hi);
                if (SameSideAsLo(mid)) lo = mid; else hi = mid;
            }
            chi = 0.5 * (lo + hi);
        }

        double zFinal = alpha * chi * chi;
        var (c2f, c3f) = Stumpff(zFinal);
        double f = 1 - chi * chi / r0Mag * c2f;
        double g = dt - chi * chi * chi / sqrtMu * c3f;
        var position = r0 * f + v0 * g;
        double rMag = position.Length();
        double fDot = sqrtMu / (rMag * r0Mag) * chi * (zFinal * c3f - 1);
        double gDot = 1 - chi * chi / rMag * c2f;
        return new StateVector(position, r0 * fDot + v0 * gDot);
    }

    /// <summary>Stumpff functions C(z) and S(z), using series near zero to avoid
    /// cancellation in the closed trigonometric and hyperbolic forms.</summary>
    private static (double C2, double C3) Stumpff(double z)
    {
        if (z > 1e-2)
        {
            double s = Math.Sqrt(z);
            return ((1 - Math.Cos(s)) / z, (s - Math.Sin(s)) / (s * s * s));
        }
        if (z < -1e-2)
        {
            double s = Math.Sqrt(-z);
            return ((1 - Math.Cosh(s)) / z, (Math.Sinh(s) - s) / (s * s * s));
        }
        // |z| <= 1e-2: truncation ~z^5/13! (< 3e-20 relative).
        double c2 = 1.0 / 2 - z / 24 + z * z / 720 - z * z * z / 40320 + z * z * z * z / 3628800;
        double c3 = 1.0 / 6 - z / 120 + z * z / 5040 - z * z * z / 362880 + z * z * z * z / 39916800;
        return (c2, c3);
    }

    /// <summary>Element derivation for any non-parabolic conic: elliptic states give
    /// a &gt; 0, e &lt; 1; hyperbolic states (positive specific energy) give a &lt; 0,
    /// e &gt; 1 with TimeAtPeriapsis from the hyperbolic anomaly. An exact parabola
    /// (zero energy — a is unrepresentable) and near-circular states are refused.</summary>
    public static OrbitalElements ElementsFromState(in StateVector state, double mu, double time)
    {
        var r = state.Position;
        var v = state.Velocity;
        double rSquared = r.LengthSquared();
        double rMag = MagnitudeWithScaledFallback(r, rSquared);
        double speedSquared = v.LengthSquared();
        double speed = MagnitudeWithScaledFallback(v, speedSquared);
        bool speedSquaredUnderflowed = speedSquared == 0 && speed != 0;
        double kineticEnergy = !double.IsFinite(speedSquared) || speedSquaredUnderflowed
            ? speed * 0.5 * speed
            : speedSquared / 2;
        double energy = kineticEnergy - mu / rMag;

        var h = r.Cross(v);
        var eVec = v.Cross(h) / mu - r / rMag;
        double eSquared = eVec.LengthSquared();
        double e = MagnitudeWithScaledFallback(eVec, eSquared);
        if (e < 1e-8)
            throw new NotSupportedException("Near-circular orbits are not supported by ElementsFromState in v1.");

        // Within ~1e-12 of zero energy the (a, e) representation is ill-conditioned
        // (a explodes, q = a(1-e) cancels catastrophically): refuse the parabola and
        // its immediate numerical neighborhood rather than emit garbage elements.
        if (Math.Abs(energy) < 1e-12 * (mu / rMag))
            throw new NotSupportedException("exactly parabolic defining conic (zero specific energy)");
        double a = -mu / (2 * energy);
        double hSquared = h.LengthSquared();
        double hLength = MagnitudeWithScaledFallback(h, hSquared);
        double inc = Math.Acos(Math.Clamp(h.Z / hLength, -1, 1));

        var nodeVec = new Vector3d(-h.Y, h.X, 0); // ẑ × h
        double nodeSquared = nodeVec.LengthSquared();
        double nodeLength = MagnitudeWithScaledFallback(nodeVec, nodeSquared);
        double raan, argPe;
        if (nodeLength < 1e-12 * hLength)
        {
            raan = 0; // equatorial: node undefined, measure argPe from +X
            argPe = Math.Atan2(eVec.Y, eVec.X);
        }
        else
        {
            raan = Math.Atan2(nodeVec.Y, nodeVec.X);
            argPe = Math.Acos(Math.Clamp((nodeVec / nodeLength).Dot(eVec / e), -1, 1));
            if (eVec.Z < 0) argPe = 2 * Math.PI - argPe;
        }

        double nu = Math.Acos(Math.Clamp(eVec.Dot(r) / (e * rMag), -1, 1));
        if (r.Dot(v) < 0) nu = 2 * Math.PI - nu;
        double timeAtPe;
        if (e < 1)
        {
            double E = Math.Atan2(Math.Sqrt(1 - e * e) * Math.Sin(nu), e + Math.Cos(nu));
            double M = E - e * Math.Sin(E);
            double n = MeanMotion(mu, a);
            timeAtPe = time - M / n;
        }
        else
        {
            // Hyperbolic anomaly via asinh (no tan(nu/2) blowup toward the asymptote);
            // 1 + e·cos(nu) > 0 on the physical branch by construction.
            double sinhH = Math.Sqrt(e * e - 1) * Math.Sin(nu) / (1 + e * Math.Cos(nu));
            double H = Math.Asinh(sinhH);
            double M = e * sinhH - H;
            double n = MeanMotion(mu, -a);
            timeAtPe = time - M / n;
        }

        return new OrbitalElements(a, e, inc, NormalizeAngle(raan), NormalizeAngle(argPe), timeAtPe);
    }

    /// <summary>Computes sqrt(mu / |a|^3) directly when its intermediates are
    /// representable, with an exponent-scaled path for extreme magnitudes.</summary>
    private static double MeanMotion(double mu, double semiMajorAxisMagnitude)
    {
        double cube = semiMajorAxisMagnitude * semiMajorAxisMagnitude
            * semiMajorAxisMagnitude;
        if (cube > 0 && double.IsFinite(cube))
        {
            double direct = Math.Sqrt(mu / cube);
            if (direct > 0 && double.IsFinite(direct)) return direct;
        }
        return ScaledMeanMotion(mu, semiMajorAxisMagnitude);
    }

    /// <summary>Elliptic period with a direct ordinary-scale path and an
    /// exponent-scaled path for extreme magnitudes.</summary>
    private static double EllipticPeriod(double semiMajorAxis, double mu)
    {
        double cube = semiMajorAxis * semiMajorAxis * semiMajorAxis;
        if (cube > 0 && double.IsFinite(cube))
        {
            double direct = 2 * Math.PI * Math.Sqrt(cube / mu);
            if (direct > 0 && double.IsFinite(direct)) return direct;
        }
        return 2 * Math.PI / ScaledMeanMotion(mu, semiMajorAxis);
    }

    /// <summary>Exponent-scaled equivalent of sqrt(mu / a^3). Mantissas stay near
    /// unity so no intermediate needs the dimensional range of the final quotient.</summary>
    private static double ScaledMeanMotion(double mu, double semiMajorAxisMagnitude)
    {
        if (!(mu > 0) || !double.IsFinite(mu)
            || !(semiMajorAxisMagnitude > 0) || !double.IsFinite(semiMajorAxisMagnitude))
            return Math.Sqrt(mu / semiMajorAxisMagnitude) / semiMajorAxisMagnitude;

        int muExponent = Math.ILogB(mu);
        int axisExponent = Math.ILogB(semiMajorAxisMagnitude);
        double muMantissa = Math.ScaleB(mu, -muExponent);
        double axisMantissa = Math.ScaleB(semiMajorAxisMagnitude, -axisExponent);
        int combinedExponent = muExponent - 3 * axisExponent;
        int halfExponent = combinedExponent >> 1;
        int oddExponent = combinedExponent - 2 * halfExponent;
        double mantissa = Math.Sqrt(muMantissa
            / (axisMantissa * axisMantissa * axisMantissa) * (oddExponent == 0 ? 1 : 2));
        return Math.ScaleB(mantissa, halfExponent);
    }

    /// <summary>Uses sqrt(squared norm) for finite positive inputs and scales
    /// components when that intermediate is zero or non-finite.</summary>
    private static double MagnitudeWithScaledFallback(
        in Vector3d vector, double squaredMagnitude)
    {
        if (squaredMagnitude > 0 && double.IsFinite(squaredMagnitude))
            return Math.Sqrt(squaredMagnitude);

        double scale = Math.Max(Math.Abs(vector.X),
            Math.Max(Math.Abs(vector.Y), Math.Abs(vector.Z)));
        if (scale == 0 || !double.IsFinite(scale)) return scale;
        double x = vector.X / scale;
        double y = vector.Y / scale;
        double z = vector.Z / scale;
        return scale * Math.Sqrt(x * x + y * y + z * z);
    }

    private static double NormalizeAngle(double angle)
    {
        double r = angle % (2 * Math.PI);
        return r < 0 ? r + 2 * Math.PI : r;
    }

    /// <summary>Transforms a complete perifocal state while evaluating each fixed
    /// orientation angle once. The arithmetic for each vector deliberately retains
    /// the nested Z-X-Z rotation order used throughout the orbital conversion.</summary>
    private static StateVector PerifocalStateToReference(
        Vector3d position, Vector3d velocity, in OrbitalElements el)
    {
        double cPe = Math.Cos(el.ArgumentOfPeriapsis), sPe = Math.Sin(el.ArgumentOfPeriapsis);
        double cInc = Math.Cos(el.Inclination), sInc = Math.Sin(el.Inclination);
        double cNode = Math.Cos(el.LongitudeOfAscendingNode),
            sNode = Math.Sin(el.LongitudeOfAscendingNode);

        Vector3d Transform(Vector3d v) =>
            RotZ(RotX(RotZ(v, cPe, sPe), cInc, sInc), cNode, sNode);
        return new StateVector(Transform(position), Transform(velocity));
    }

    private static Vector3d RotZ(Vector3d v, double c, double s)
    {
        return new Vector3d(c * v.X - s * v.Y, s * v.X + c * v.Y, v.Z);
    }

    private static Vector3d RotX(Vector3d v, double c, double s)
    {
        return new Vector3d(v.X, c * v.Y - s * v.Z, s * v.Y + c * v.Z);
    }
}

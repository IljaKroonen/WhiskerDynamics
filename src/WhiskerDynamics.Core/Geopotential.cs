namespace WhiskerDynamics.Core;

/// <summary>An unnormalised coefficient. Fully-normalised source models must enter
/// through <see cref="Geopotential.FromFullyNormalized"/>.</summary>
public readonly record struct SphericalHarmonicCoefficient(int Degree, int Order, double Cosine, double Sine);

public sealed class Geopotential
{
    public const int MaximumDegree = 50;
    // Far-field screening tapers each degree smoothly so the potential and its first
    // derivative remain continuous. A degree is exact below its conservative s0 threshold,
    // tapered in potential over [s0, 3*s0], and zero beyond 3*s0.
    private const double DampingTolerance = 1.0 / (1 << 24);
    private readonly SphericalHarmonicCoefficient[] _coefficients;
    private readonly IReadOnlyList<SphericalHarmonicCoefficient> _coefficientView;
    private readonly double[] _dampingRadius;
    private readonly int _maximumDegree;
    public double ReferenceRadius { get; }
    public BodyRotation Rotation { get; }
    public BodyFixedToModelRotation? BodyFixedToModel { get; }
    public IReadOnlyList<SphericalHarmonicCoefficient> Coefficients => _coefficientView;
    public int Degree => _maximumDegree;
    public Geopotential(double radius, BodyRotation rotation,
        IEnumerable<SphericalHarmonicCoefficient> coefficients,
        BodyFixedToModelRotation? bodyFixedToModel = null)
    {
        if (!double.IsFinite(radius) || radius <= 0) throw new ArgumentOutOfRangeException(nameof(radius));
        ValidateRotation(rotation);
        ReferenceRadius = radius;
        Rotation = rotation;
        BodyFixedToModel = bodyFixedToModel;
        ArgumentNullException.ThrowIfNull(coefficients);
        _coefficients = coefficients.OrderBy(c => c.Degree).ThenBy(c => c.Order).ToArray();
        _coefficientView = Array.AsReadOnly(_coefficients);
        if (_coefficients.Length == 0) throw new ArgumentException("No coefficients.", nameof(coefficients));
        var seen = new HashSet<(int, int)>();
        foreach (var c in _coefficients)
        {
            if (c.Degree is < 2 or > MaximumDegree || c.Order < 0 || c.Order > c.Degree)
                throw new ArgumentOutOfRangeException(nameof(coefficients));
            if (!double.IsFinite(c.Cosine) || !double.IsFinite(c.Sine) || c.Order == 0 && c.Sine != 0)
                throw new ArgumentException("Invalid coefficient.", nameof(coefficients));
            if (!seen.Add((c.Degree, c.Order))) throw new ArgumentException("Duplicate coefficient.");
        }
        _maximumDegree = _coefficients[^1].Degree;
        _dampingRadius = new double[_maximumDegree + 1];
        foreach (var c in _coefficients)
            _dampingRadius[c.Degree] += (Math.Abs(c.Cosine) + Math.Abs(c.Sine))
                / Normalization(c.Degree, c.Order) * Math.Sqrt(2 * c.Degree + 1);
        for (int n = 2; n <= _maximumDegree; n++)
            _dampingRadius[n] = radius * Math.Pow(
                (n + 1) * _dampingRadius[n] / DampingTolerance, 1.0 / n);
    }

    public static Geopotential FromJ2(double radius, BodyRotation rotation, double j2)
    {
        if (!double.IsFinite(j2) || j2 < 0) throw new ArgumentOutOfRangeException(nameof(j2));
        return new(radius, rotation, [new(2, 0, -j2, 0)]);
    }

    public static Geopotential FromJ2(double radius, Vector3d pole, double j2) =>
        FromJ2(radius, NonRotatingBasis(pole), j2);

    public static Geopotential FromFullyNormalized(double radius, BodyRotation rotation,
        IEnumerable<SphericalHarmonicCoefficient> coefficients,
        BodyFixedToModelRotation? bodyFixedToModel = null) =>
        new(radius, rotation, coefficients.Select(c => c with
        {
            Cosine = c.Cosine * Normalization(c.Degree, c.Order),
            Sine = c.Sine * Normalization(c.Degree, c.Order),
        }), bodyFixedToModel);

    /// <summary>Analytic, pole-safe non-point-mass acceleration.</summary>
    public Vector3d AccelerationCorrection(Vector3d position, double mu, double time)
    {
        double r2 = position.LengthSquared();
        if (r2 == 0) return Vector3d.Zero;
        if (!double.IsFinite(mu) || mu < 0) throw new ArgumentOutOfRangeException(nameof(mu));
        double r = Math.Sqrt(r2);
        double angle = Rotation.AngularVelocity * (time - Rotation.ReferenceTime);
        // X, Y, Pole are a right-handed orthonormal basis, so rotating both
        // equatorial axes only needs one trigonometric evaluation:
        // pole x X = Y and pole x Y = -X.
        var (sinAngle, cosAngle) = Math.SinCos(angle);
        var bx = Rotation.XAxisEcl * cosAngle + Rotation.YAxisEcl * sinAngle;
        var by = Rotation.YAxisEcl * cosAngle - Rotation.XAxisEcl * sinAngle;
        var bz = Rotation.PoleEcl;
        var bodyFixed = new Vector3d(
            position.Dot(bx), position.Dot(by), position.Dot(bz));
        var model = BodyFixedToModel?.ToModelCoordinates(bodyFixed) ?? bodyFixed;
        double x = model.X, y = model.Y, z = model.Z;
        double rho = Math.Sqrt(x * x + y * y);
        double sinB = z / r, cosB = rho / r;
        double cosL = rho == 0 ? 1 : x / rho, sinL = rho == 0 ? 0 : y / rho;
        int limit = LimitingDegree(r), w = limit + 1;

        // d[n,m] = the mth derivative of Pn at sin(latitude). The recurrence
        // only reads degrees n-1 and n-2, so retain three rows instead of
        // clearing a 51x51 matrix on every acceleration evaluation.
        Span<double> dNm2 = stackalloc double[w];
        Span<double> dNm1 = stackalloc double[w];
        Span<double> dN = stackalloc double[w];
        Span<double> cm = stackalloc double[w];
        Span<double> sm = stackalloc double[w];
        Span<double> cbm = stackalloc double[w];
        // The recurrence deliberately reads the zero triangular fringe m > n.
        dNm2.Clear(); dNm1.Clear(); dN.Clear();
        dNm2[0] = 1;
        dNm1[0] = sinB; dNm1[1] = 1;
        cm[0] = 1; sm[0] = 0; cbm[0] = 1;
        cm[1] = cosL; sm[1] = sinL; cbm[1] = cosB;
        for (int m = 2; m <= limit; m++)
        {
            cm[m] = cm[m - 1] * cosL - sm[m - 1] * sinL;
            sm[m] = sm[m - 1] * cosL + cm[m - 1] * sinL;
            cbm[m] = cbm[m - 1] * cosB;
        }

        var er = new Vector3d(x / r, y / r, z / r);
        var eb = new Vector3d(-sinB * cosL, -sinB * sinL, cosB);
        var el = new Vector3d(-sinL, cosL, 0);
        double ar = 0, ab = 0, al = 0;
        double muOverR2 = mu / r2;
        double q = ReferenceRadius / r, qn = q;
        int coefficient = 0;
        for (int n = 2; n <= limit; n++)
        {
            for (int m = 0; m <= n; m++)
                dN[m] = ((2 * n - 1) * (sinB * dNm1[m]
                    + (m == 0 ? 0 : m * dNm1[m - 1]))
                    - (n - 1) * dNm2[m]) / n;

            qn *= q;
            var (sigma, rSigmaPrime) = Damping(n, r);
            double scale = muOverR2 * qn;
            while (coefficient < _coefficients.Length
                && _coefficients[coefficient].Degree == n)
            {
                var c = _coefficients[coefficient++];
                int m = c.Order;
                double dm = dN[m];
                double b = cbm[m] * dm;
                double longitude = c.Cosine * cm[m] + c.Sine * sm[m];
                double db = m < n ? cbm[m] * cosB * dN[m + 1] : 0;
                if (m > 0) db -= m * sinB * cbm[m - 1] * dm;
                ar += (-(n + 1) * sigma + rSigmaPrime) * b * longitude * scale;
                ab += sigma * db * longitude * scale;
                if (m > 0)
                    al += sigma * cbm[m - 1] * dm * m
                        * (c.Sine * cm[m] - c.Cosine * sm[m]) * scale;
            }

            var recycle = dNm2;
            dNm2 = dNm1;
            dNm1 = dN;
            dN = recycle;
        }
        var a = er * ar + eb * ab + el * al;
        if (BodyFixedToModel is { } bodyFixedToModel)
            a = bodyFixedToModel.ToBodyFixedCoordinates(a);
        return bx * a.X + by * a.Y + bz * a.Z;
    }

    private int LimitingDegree(double r)
    {
        for (int n = _maximumDegree; n >= 2; n--)
            if (r < 3 * _dampingRadius[n]) return n;
        return 2;
    }

    private (double, double) Damping(int n, double r)
    {
        double s0 = _dampingRadius[n];
        if (r <= s0) return (1, 0);
        if (r >= 3 * s0) return (0, 0);
        double u = r / s0;
        return (u * (u - 3) * (u - 3) / 4,
            u * (3 * u * u - 12 * u + 9) / 4);
    }

    private static double Normalization(int n, int m)
    {
        double ratio = 1;
        for (int k = n - m + 1; k <= n + m; k++) ratio /= k;
        return Math.Sqrt((m == 0 ? 1 : 2) * (2 * n + 1) * ratio);
    }

    private static BodyRotation NonRotatingBasis(Vector3d pole)
    {
        double length = pole.Length();
        if (!double.IsFinite(length) || length == 0)
            throw new ArgumentException("Geopotential pole must be finite and non-zero.", nameof(pole));
        var z = pole / length;
        var reference = Math.Abs(z.Z) < 0.9 ? new Vector3d(0, 0, 1) : new Vector3d(1, 0, 0);
        var x = (reference - z * reference.Dot(z)).Normalized();
        return new(z, x, z.Cross(x), 0, 0);
    }

    private static void ValidateRotation(in BodyRotation r)
    {
        static bool Finite(in Vector3d v) =>
            double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
        if (!Finite(r.PoleEcl) || !Finite(r.XAxisEcl) || !Finite(r.YAxisEcl)
            || !double.IsFinite(r.AngularVelocity) || !double.IsFinite(r.ReferenceTime))
            throw new ArgumentException("Geopotential rotation must be finite.", nameof(r));
        var x = r.XAxisEcl; var y = r.YAxisEcl; var z = r.PoleEcl;
        double e = Math.Max(Math.Max(Math.Abs(x.LengthSquared() - 1), Math.Abs(y.LengthSquared() - 1)),
            Math.Max(Math.Abs(z.LengthSquared() - 1), Math.Max(Math.Abs(x.Dot(y)),
                Math.Max(Math.Abs(x.Dot(z)), Math.Abs(y.Dot(z))))));
        e = Math.Max(e, (x.Cross(y) - z).Length());
        if (e > 1e-9) throw new ArgumentException(
            $"Geopotential rotation must be a right-handed orthonormal basis (error {e:E2}).", nameof(r));
    }
}

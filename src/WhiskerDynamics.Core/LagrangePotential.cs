namespace WhiskerDynamics.Core;

/// <summary>A line segment of an iso-potential contour in primary-centred,
/// separation-normalized rotating-frame coordinates.</summary>
public readonly record struct PotentialSegment(Vector3d A, Vector3d B);

/// <summary>The five equilibria of the circular restricted three-body problem, in
/// the same primary-centred coordinates used by <see cref="FrameKernel.Rotating"/>:
/// the primary is (0,0,0), the secondary is (1,0,0), and the orbital plane is z=0.</summary>
public readonly record struct LagrangePoints(
    Vector3d L1, Vector3d L2, Vector3d L3, Vector3d L4, Vector3d L5)
{
    public Vector3d this[int index] => index switch
    {
        0 => L1,
        1 => L2,
        2 => L3,
        3 => L4,
        4 => L5,
        _ => throw new ArgumentOutOfRangeException(nameof(index)),
    };
}

/// <summary>Pure CR3BP display kernel. It deliberately uses normalized coordinates:
/// total mass, pair separation, and angular rate are one. This matches the
/// rotating-pulsating display pose and makes one cached contour mesh valid while an
/// eccentric pair breathes. The result is a visualization of the instantaneous
/// circular restricted-three-body effective potential, not an extra force model.</summary>
public static class LagrangePotential
{
    /// <summary>Secondary mass fraction mu = m2/(m1+m2). A supported pair has two
    /// positive finite gravitational parameters and is ordered primary then secondary.</summary>
    public static double MassRatio(double primaryMu, double secondaryMu)
    {
        if (!(primaryMu > 0) || !(secondaryMu > 0)
            || !double.IsFinite(primaryMu) || !double.IsFinite(secondaryMu))
            throw new ArgumentOutOfRangeException(nameof(primaryMu),
                "Both bodies need positive finite gravitational parameters.");
        double ratio = secondaryMu / (primaryMu + secondaryMu);
        if (!(ratio > 0 && ratio < 1) || !double.IsFinite(ratio))
            throw new ArgumentOutOfRangeException(nameof(primaryMu),
                "The pair's mass ratio is not representable.");
        return ratio;
    }

    /// <summary>Positive CR3BP pseudo-potential Omega in primary-centred coordinates.
    /// Singular at either body. Its planar stationary points are L1-L5.</summary>
    public static double At(double massRatio, double x, double y)
    {
        ValidateRatio(massRatio);
        return AtUnchecked(massRatio, x, y);
    }

    private static double AtUnchecked(double massRatio, double x, double y)
    {
        double r1 = Math.Sqrt(x * x + y * y);
        double dx2 = x - 1.0;
        double r2 = Math.Sqrt(dx2 * dx2 + y * y);
        double baryX = x - massRatio;
        return 0.5 * (baryX * baryX + y * y)
            + (1.0 - massRatio) / r1 + massRatio / r2;
    }

    /// <summary>Analytic planar gradient of <see cref="At"/>.</summary>
    public static (double X, double Y) Gradient(double massRatio, double x, double y)
    {
        ValidateRatio(massRatio);
        double r1Sq = x * x + y * y;
        double dx2 = x - 1.0;
        double r2Sq = dx2 * dx2 + y * y;
        double r1Cubed = r1Sq * Math.Sqrt(r1Sq);
        double r2Cubed = r2Sq * Math.Sqrt(r2Sq);
        return (
            x - massRatio - (1.0 - massRatio) * x / r1Cubed - massRatio * dx2 / r2Cubed,
            y - (1.0 - massRatio) * y / r1Cubed - massRatio * y / r2Cubed);
    }

    /// <summary>Finds L1-L3 by safeguarded bisection of dOmega/dx and returns the
    /// analytic equilateral L4/L5. Bisection is used instead of Newton iteration so
    /// extreme but representable mass ratios cannot jump through a body singularity.</summary>
    public static LagrangePoints Equilibria(double massRatio)
    {
        ValidateRatio(massRatio);
        const double gap = 1e-12;
        double l1 = BisectRoot(massRatio, gap, 1.0 - gap);
        double l2 = BisectRoot(massRatio, 1.0 + gap, 2.0);
        double l3 = BisectRoot(massRatio, -1.0, -gap);
        double h = Math.Sqrt(3.0) / 2.0;
        return new LagrangePoints(
            new Vector3d(l1, 0, 0), new Vector3d(l2, 0, 0), new Vector3d(l3, 0, 0),
            new Vector3d(0.5, h, 0), new Vector3d(0.5, -h, 0));
    }

    /// <summary>Marching-squares iso-line segments over a fixed normalized map box.
    /// Ambiguous saddle cells use the centre value to choose connectivity. Invalid
    /// or body-singular samples remain finite-capped on the high side, preventing a
    /// NaN from punching arbitrary holes through otherwise valid contours.</summary>
    public static PotentialSegment[] Contour(double massRatio, double level,
        int columns = 192, int rows = 160,
        double minX = -1.5, double maxX = 2.0,
        double minY = -1.5, double maxY = 1.5) =>
        ContourCore(massRatio, level, columns, rows,
            minX, maxX, minY, maxY, validateEachSample: true);

    internal static PotentialSegment[] ContourWithHoistedValidation(
        double massRatio, double level,
        int columns = 192, int rows = 160,
        double minX = -1.5, double maxX = 2.0,
        double minY = -1.5, double maxY = 1.5) =>
        ContourCore(massRatio, level, columns, rows,
            minX, maxX, minY, maxY, validateEachSample: false);

    private static PotentialSegment[] ContourCore(
        double massRatio, double level, int columns, int rows,
        double minX, double maxX, double minY, double maxY,
        bool validateEachSample)
    {
        ValidateRatio(massRatio);
        if (!double.IsFinite(level)) throw new ArgumentOutOfRangeException(nameof(level));
        if (columns < 2 || rows < 2) throw new ArgumentOutOfRangeException(nameof(columns));
        if (!(minX < maxX) || !(minY < maxY)) throw new ArgumentOutOfRangeException(nameof(minX));

        double dx = (maxX - minX) / columns;
        double dy = (maxY - minY) / rows;
        var values = new double[(columns + 1) * (rows + 1)];
        for (int iy = 0; iy <= rows; iy++)
        for (int ix = 0; ix <= columns; ix++)
        {
            double x = minX + ix * dx;
            double y = minY + iy * dy;
            double value = validateEachSample
                ? At(massRatio, x, y)
                : AtUnchecked(massRatio, x, y);
            values[iy * (columns + 1) + ix] = double.IsFinite(value) ? value : double.MaxValue;
        }

        var segments = new List<PotentialSegment>();
        for (int iy = 0; iy < rows; iy++)
        for (int ix = 0; ix < columns; ix++)
        {
            double x0 = minX + ix * dx;
            double y0 = minY + iy * dy;
            int stride = columns + 1;
            double v0 = values[iy * stride + ix];             // bottom-left
            double v1 = values[iy * stride + ix + 1];         // bottom-right
            double v2 = values[(iy + 1) * stride + ix + 1];   // top-right
            double v3 = values[(iy + 1) * stride + ix];       // top-left
            int mask = (v0 >= level ? 1 : 0) | (v1 >= level ? 2 : 0)
                | (v2 >= level ? 4 : 0) | (v3 >= level ? 8 : 0);
            if (mask is 0 or 15) continue;

            Vector3d Edge(int edge) => edge switch
            {
                0 => new Vector3d(x0 + dx * Fraction(v0, v1, level), y0, 0),
                1 => new Vector3d(x0 + dx, y0 + dy * Fraction(v1, v2, level), 0),
                2 => new Vector3d(x0 + dx * Fraction(v3, v2, level), y0 + dy, 0),
                _ => new Vector3d(x0, y0 + dy * Fraction(v0, v3, level), 0),
            };
            void Add(int a, int b) => segments.Add(new PotentialSegment(Edge(a), Edge(b)));

            switch (mask)
            {
                case 1: case 14: Add(3, 0); break;
                case 2: case 13: Add(0, 1); break;
                case 3: case 12: Add(3, 1); break;
                case 4: case 11: Add(1, 2); break;
                case 6: case 9: Add(0, 2); break;
                case 7: case 8: Add(3, 2); break;
                case 5:
                {
                    bool centreHigh = (v0 + v1 + v2 + v3) * 0.25 >= level;
                    if (centreHigh) { Add(0, 1); Add(2, 3); }
                    else { Add(3, 0); Add(1, 2); }
                    break;
                }
                case 10:
                {
                    bool centreHigh = (v0 + v1 + v2 + v3) * 0.25 >= level;
                    if (centreHigh) { Add(3, 0); Add(1, 2); }
                    else { Add(0, 1); Add(2, 3); }
                    break;
                }
            }
        }
        return [.. segments];
    }

    /// <summary>Critical levels through the five equilibria, de-duplicated because
    /// symmetry gives L4 and L5 the same value.</summary>
    public static double[] CriticalLevels(double massRatio)
    {
        var points = Equilibria(massRatio);
        var levels = new List<double>(4);
        for (int i = 0; i < 5; i++)
        {
            double value = At(massRatio, points[i].X, points[i].Y);
            if (!levels.Any(existing => Math.Abs(existing - value) <= 1e-12 * Math.Max(1.0, value)))
                levels.Add(value);
        }
        levels.Sort();
        return [.. levels];
    }

    private static double Fraction(double a, double b, double level)
    {
        double span = b - a;
        if (span == 0 || !double.IsFinite(span)) return 0.5;
        return Math.Clamp((level - a) / span, 0.0, 1.0);
    }

    private static double BisectRoot(double massRatio, double lo, double hi)
    {
        double flo = Gradient(massRatio, lo, 0).X;
        double fhi = Gradient(massRatio, hi, 0).X;
        if (Math.Sign(flo) == Math.Sign(fhi))
            throw new InvalidOperationException("Failed to bracket a collinear Lagrange point.");
        for (int i = 0; i < 100; i++)
        {
            double mid = lo + (hi - lo) * 0.5;
            double fm = Gradient(massRatio, mid, 0).X;
            if (fm == 0) return mid;
            if (Math.Sign(fm) == Math.Sign(flo)) { lo = mid; flo = fm; }
            else hi = mid;
        }
        return lo + (hi - lo) * 0.5;
    }

    private static void ValidateRatio(double massRatio)
    {
        if (!(massRatio > 0 && massRatio < 1) || !double.IsFinite(massRatio))
            throw new ArgumentOutOfRangeException(nameof(massRatio), "Mass ratio must be finite and in (0, 1).");
    }
}

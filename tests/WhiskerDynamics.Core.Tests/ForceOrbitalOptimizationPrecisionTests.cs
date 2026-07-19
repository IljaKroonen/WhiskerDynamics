using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

public class ForceOrbitalOptimizationPrecisionTests
{
    private const double SolarMu = 1.32712440018e20;

    [Fact]
    public void Cached_anomaly_starter_is_bitwise_identical_to_repeated_sine()
    {
        foreach (double e in new[] { 0.0, 0.01, 0.2, 0.75, 0.98 })
        for (int i = -128; i <= 128; i++)
        {
            double meanAnomaly = i * 0.123456789012345;
            AssertBits(ReferenceEccentricAnomaly(meanAnomaly, e),
                Kepler.SolveEccentricAnomaly(meanAnomaly, e));
        }
    }

    [Fact]
    public void Shared_perifocal_orientation_is_bitwise_identical_for_all_conics()
    {
        var cases = new[]
        {
            (new OrbitalElements(5.790896153292818e10, 0.2056462028967717,
                0.122232, 0.843, 0.5096, -563615.3399), 1_000_000.0),
            (new OrbitalElements(2.5e11, 0.75, 0.4, 0.9, 1.7, 0), -4.2e6),
            (new OrbitalElements(1e12, 0.99993, 0.73, 2.4, 5.1, 10), 86410.0),
            (new OrbitalElements(-8e10, 1.7, 1.2, 4.7, 0.3, -20), 1.4e6),
        };

        foreach (var (elements, time) in cases)
            AssertStateBits(ReferenceStateFromElements(elements, SolarMu, time),
                Kepler.StateFromElements(elements, SolarMu, time));
    }

    [Fact]
    public void Cached_angular_momentum_lengths_leave_elements_bitwise_identical()
    {
        var elements = new[]
        {
            new OrbitalElements(5.79e10, 0.21, 0.12, 0.84, 0.51, -560000),
            new OrbitalElements(2.5e11, 0.75, 0.4, 0.9, 1.7, 0),
            new OrbitalElements(-8e10, 1.7, 1.2, 4.7, 0.3, -20),
        };
        foreach (var el in elements)
        {
            double time = 1.234567e6;
            var state = Kepler.StateFromElements(el, SolarMu, time);
            AssertElementsBits(ReferenceElementsFromState(state, SolarMu, time),
                Kepler.ElementsFromState(state, SolarMu, time));
        }
    }

    [Fact]
    public void Shared_surface_trigonometry_is_bitwise_identical_to_two_rotations()
    {
        var body = new StateVector(new Vector3d(1e11, -2e10, 3e9), new Vector3d(1, 2, 3));
        var rotations = new[]
        {
            new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
                new Vector3d(0, 1, 0), 7.2921159e-5, 42),
            new BodyRotation(new Vector3d(0.3, -0.4, 0.5), new Vector3d(2, 3, -1),
                new Vector3d(-4, 0.25, 7), -1.234e-3, -8),
        };

        foreach (var rotation in rotations)
        foreach (double time in new[] { -1e9, -1.0, 0.0, 1.0, 12345.6789, 1e9 })
        {
            double angle = rotation.AngularVelocity * (time - rotation.ReferenceTime);
            var expected = new FramePose(body.Position,
                rotation.XAxisEcl.RotateAbout(rotation.PoleEcl, angle),
                rotation.YAxisEcl.RotateAbout(rotation.PoleEcl, angle), rotation.PoleEcl);
            var actual = FrameKernel.Surface(body, rotation, time);
            AssertFrameBits(expected, actual);
        }
    }

    [Fact]
    public void Unchecked_cr3bp_inner_loops_retain_bitwise_values_and_roots()
    {
        foreach (double ratio in new[] { 1e-12, 0.012150585609624, 0.1, 0.499999999 })
        {
            for (int iy = -10; iy <= 10; iy++)
            for (int ix = -10; ix <= 10; ix++)
            {
                double x = ix * 0.137 + 0.013;
                double y = iy * 0.119 + 0.017;
                AssertBits(ReferencePotential(ratio, x, y), LagrangePotential.At(ratio, x, y));
                var expected = ReferenceGradient(ratio, x, y);
                var actual = LagrangePotential.Gradient(ratio, x, y);
                AssertBits(expected.X, actual.X);
                AssertBits(expected.Y, actual.Y);
            }

            var expectedPoints = ReferenceEquilibria(ratio);
            var actualPoints = LagrangePotential.Equilibria(ratio);
            for (int i = 0; i < 5; i++) AssertVectorBits(expectedPoints[i], actualPoints[i]);
        }
    }

    [Fact]
    public void Combined_sine_cosine_matches_separate_calls_bitwise_on_target_runtime()
    {
        for (int exponent = -200; exponent <= 200; exponent += 8)
        for (int i = -1000; i <= 1000; i++)
        {
            double angle = Math.ScaleB(i * 0.00123456789012345, exponent);
            var pair = Math.SinCos(angle);
            AssertBits(Math.Sin(angle), pair.Sin);
            AssertBits(Math.Cos(angle), pair.Cos);
        }
    }

    private static double ReferenceEccentricAnomaly(double meanAnomaly, double eccentricity)
    {
        double m = Math.IEEERemainder(meanAnomaly, 2 * Math.PI);
        double e = m + 0.85 * eccentricity
            * Math.Sign(Math.Sin(m) == 0 ? 1 : Math.Sin(m));
        for (int i = 0; i < 50; i++)
        {
            double f = e - eccentricity * Math.Sin(e) - m;
            double de = f / (1 - eccentricity * Math.Cos(e));
            e -= de;
            if (Math.Abs(de) < 1e-14) return e;
        }
        throw new InvalidOperationException();
    }

    private static StateVector ReferenceStateFromElements(
        in OrbitalElements el, double mu, double time)
    {
        double a = el.SemiMajorAxis, e = el.Eccentricity;
        if (e > Kepler.UniversalPathEccentricity)
        {
            double q = a * (1 - e);
            var periapsis = new StateVector(
                ReferenceTransform(new Vector3d(q, 0, 0), el),
                ReferenceTransform(new Vector3d(0, Math.Sqrt(mu * (1 + e) / q), 0), el));
            double dt = time - el.TimeAtPeriapsis;
            if (e < 1) dt = Math.IEEERemainder(dt, 2 * Math.PI * Math.Sqrt(a * a * a / mu));
            return Kepler.PropagateUniversal(periapsis, mu, dt);
        }
        double n = Math.Sqrt(mu / (a * a * a));
        double anomaly = ReferenceEccentricAnomaly(n * (time - el.TimeAtPeriapsis), e);
        double cosE = Math.Cos(anomaly), sinE = Math.Sin(anomaly);
        double sqrt1mE2 = Math.Sqrt(1 - e * e);
        double r = a * (1 - e * cosE);
        var p = new Vector3d(a * (cosE - e), a * sqrt1mE2 * sinE, 0);
        double vScale = Math.Sqrt(mu * a) / r;
        var v = new Vector3d(-vScale * sinE, vScale * sqrt1mE2 * cosE, 0);
        return new StateVector(ReferenceTransform(p, el), ReferenceTransform(v, el));
    }

    private static Vector3d ReferenceTransform(Vector3d v, in OrbitalElements el) =>
        ReferenceRotZ(ReferenceRotX(ReferenceRotZ(v, el.ArgumentOfPeriapsis),
            el.Inclination), el.LongitudeOfAscendingNode);

    private static Vector3d ReferenceRotZ(Vector3d v, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new Vector3d(c * v.X - s * v.Y, s * v.X + c * v.Y, v.Z);
    }

    private static Vector3d ReferenceRotX(Vector3d v, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new Vector3d(v.X, c * v.Y - s * v.Z, s * v.Y + c * v.Z);
    }

    private static OrbitalElements ReferenceElementsFromState(
        in StateVector state, double mu, double time)
    {
        var r = state.Position;
        var v = state.Velocity;
        double rMag = r.Length();
        double energy = v.LengthSquared() / 2 - mu / rMag;
        var h = r.Cross(v);
        var eVec = v.Cross(h) / mu - r / rMag;
        double e = eVec.Length();
        double a = -mu / (2 * energy);
        double inc = Math.Acos(Math.Clamp(h.Z / h.Length(), -1, 1));
        var nodeVec = new Vector3d(-h.Y, h.X, 0);
        double raan, argPe;
        if (nodeVec.Length() < 1e-12 * h.Length())
        {
            raan = 0;
            argPe = Math.Atan2(eVec.Y, eVec.X);
        }
        else
        {
            raan = Math.Atan2(nodeVec.Y, nodeVec.X);
            argPe = Math.Acos(Math.Clamp(nodeVec.Normalized().Dot(eVec / e), -1, 1));
            if (eVec.Z < 0) argPe = 2 * Math.PI - argPe;
        }

        double nu = Math.Acos(Math.Clamp(eVec.Dot(r) / (e * rMag), -1, 1));
        if (r.Dot(v) < 0) nu = 2 * Math.PI - nu;
        double timeAtPe;
        if (e < 1)
        {
            double anomaly = Math.Atan2(Math.Sqrt(1 - e * e) * Math.Sin(nu),
                e + Math.Cos(nu));
            double mean = anomaly - e * Math.Sin(anomaly);
            double n = Math.Sqrt(mu / (a * a * a));
            timeAtPe = time - mean / n;
        }
        else
        {
            double sinhH = Math.Sqrt(e * e - 1) * Math.Sin(nu) / (1 + e * Math.Cos(nu));
            double anomaly = Math.Asinh(sinhH);
            double mean = e * sinhH - anomaly;
            double n = Math.Sqrt(mu / (-a * a * a));
            timeAtPe = time - mean / n;
        }
        return new OrbitalElements(a, e, inc, Normalize(raan), Normalize(argPe), timeAtPe);
    }

    private static double Normalize(double angle)
    {
        double result = angle % (2 * Math.PI);
        return result < 0 ? result + 2 * Math.PI : result;
    }

    private static double ReferencePotential(double massRatio, double x, double y)
    {
        double r1 = Math.Sqrt(x * x + y * y);
        double dx2 = x - 1.0;
        double r2 = Math.Sqrt(dx2 * dx2 + y * y);
        double baryX = x - massRatio;
        return 0.5 * (baryX * baryX + y * y)
            + (1.0 - massRatio) / r1 + massRatio / r2;
    }

    private static (double X, double Y) ReferenceGradient(double massRatio, double x, double y)
    {
        double r1Sq = x * x + y * y;
        double dx2 = x - 1.0;
        double r2Sq = dx2 * dx2 + y * y;
        double r1Cubed = r1Sq * Math.Sqrt(r1Sq);
        double r2Cubed = r2Sq * Math.Sqrt(r2Sq);
        return (
            x - massRatio - (1.0 - massRatio) * x / r1Cubed - massRatio * dx2 / r2Cubed,
            y - (1.0 - massRatio) * y / r1Cubed - massRatio * y / r2Cubed);
    }

    private static LagrangePoints ReferenceEquilibria(double massRatio)
    {
        const double gap = 1e-12;
        double l1 = ReferenceBisect(massRatio, gap, 1.0 - gap);
        double l2 = ReferenceBisect(massRatio, 1.0 + gap, 2.0);
        double l3 = ReferenceBisect(massRatio, -1.0, -gap);
        double h = Math.Sqrt(3.0) / 2.0;
        return new LagrangePoints(new Vector3d(l1, 0, 0), new Vector3d(l2, 0, 0),
            new Vector3d(l3, 0, 0), new Vector3d(0.5, h, 0), new Vector3d(0.5, -h, 0));
    }

    private static double ReferenceBisect(double massRatio, double lo, double hi)
    {
        double flo = ReferenceGradient(massRatio, lo, 0).X;
        _ = ReferenceGradient(massRatio, hi, 0).X;
        for (int i = 0; i < 100; i++)
        {
            double mid = lo + (hi - lo) * 0.5;
            double fm = ReferenceGradient(massRatio, mid, 0).X;
            if (fm == 0) return mid;
            if (Math.Sign(fm) == Math.Sign(flo)) { lo = mid; flo = fm; }
            else hi = mid;
        }
        return lo + (hi - lo) * 0.5;
    }

    private static void AssertFrameBits(in FramePose expected, in FramePose actual)
    {
        AssertVectorBits(expected.Origin, actual.Origin);
        AssertVectorBits(expected.XAxis, actual.XAxis);
        AssertVectorBits(expected.YAxis, actual.YAxis);
        AssertVectorBits(expected.ZAxis, actual.ZAxis);
        AssertBits(expected.Scale, actual.Scale);
    }

    private static void AssertElementsBits(in OrbitalElements expected, in OrbitalElements actual)
    {
        AssertBits(expected.SemiMajorAxis, actual.SemiMajorAxis);
        AssertBits(expected.Eccentricity, actual.Eccentricity);
        AssertBits(expected.Inclination, actual.Inclination);
        AssertBits(expected.LongitudeOfAscendingNode, actual.LongitudeOfAscendingNode);
        AssertBits(expected.ArgumentOfPeriapsis, actual.ArgumentOfPeriapsis);
        AssertBits(expected.TimeAtPeriapsis, actual.TimeAtPeriapsis);
    }

    private static void AssertStateBits(in StateVector expected, in StateVector actual)
    {
        AssertVectorBits(expected.Position, actual.Position);
        AssertVectorBits(expected.Velocity, actual.Velocity);
    }

    private static void AssertVectorBits(in Vector3d expected, in Vector3d actual)
    {
        AssertBits(expected.X, actual.X);
        AssertBits(expected.Y, actual.Y);
        AssertBits(expected.Z, actual.Z);
    }

    private static void AssertBits(double expected, double actual) =>
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected), BitConverter.DoubleToInt64Bits(actual));
}

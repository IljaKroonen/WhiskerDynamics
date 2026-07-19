using WhiskerDynamics.Core;
using Xunit.Abstractions;

namespace WhiskerDynamics.Core.Tests;

public class NumericalPrecisionTradeoffTests(ITestOutputHelper output)
{
    [Fact]
    public void Quantify_rejected_reciprocal_and_lunar_degree_tradeoffs()
    {
        ulong vectorMaxUlp = 0;
        double vectorMaxRelative = 0;
        for (int exponent = -200; exponent <= 200; exponent += 5)
        for (int i = 1; i <= 100; i++)
        {
            var v = new Vector3d(Math.ScaleB(Math.Sin(i * 0.71), exponent),
                Math.ScaleB(Math.Cos(i * 1.13), exponent - 3),
                Math.ScaleB(Math.Sin(i * 1.91), exponent - 7));
            var exact = v / v.Length();
            var reciprocal = v * (1.0 / v.Length());
            Measure(exact.X, reciprocal.X, ref vectorMaxUlp, ref vectorMaxRelative);
            Measure(exact.Y, reciprocal.Y, ref vectorMaxUlp, ref vectorMaxRelative);
            Measure(exact.Z, reciprocal.Z, ref vectorMaxUlp, ref vectorMaxRelative);
        }

        double pointMassReciprocalMaxRelative = 0;
        double pointMassNewtonMaxRelative = 0;
        ulong pointMassReciprocalMaxUlp = 0;
        ulong pointMassNewtonMaxUlp = 0;
        for (int i = 1; i <= 100_000; i++)
        {
            double r2 = 1e4 + i * 1.234567890123e17 + Math.Sin(i * 0.13) * 1e8;
            double mu = 1e9 + i * 7.654321098765e14;
            double exact = mu / (r2 * Math.Sqrt(r2));
            double inverseRadius = 1.0 / Math.Sqrt(r2);
            double reciprocal = mu * inverseRadius * inverseRadius * inverseRadius;
            Measure(exact, reciprocal, ref pointMassReciprocalMaxUlp,
                ref pointMassReciprocalMaxRelative);

            inverseRadius = Math.ReciprocalSqrtEstimate(r2);
            inverseRadius *= 1.5 - 0.5 * r2 * inverseRadius * inverseRadius;
            inverseRadius *= 1.5 - 0.5 * r2 * inverseRadius * inverseRadius;
            double newton = mu * inverseRadius * inverseRadius * inverseRadius;
            Measure(exact, newton, ref pointMassNewtonMaxUlp, ref pointMassNewtonMaxRelative);
        }

        ulong fmaMaxUlpFromBaseline = 0, reassociatedMaxUlpFromBaseline = 0;
        int fmaMoreAccurate = 0, fmaLessAccurate = 0;
        int reassociatedMoreAccurate = 0, reassociatedLessAccurate = 0;
        double fmaMaxAbsoluteError = 0, reassociatedMaxAbsoluteError = 0;
        for (int i = 1; i <= 100_000; i++)
        {
            var a = new Vector3d(Math.Sin(i * 0.71) * 1e12,
                Math.Cos(i * 1.13) * 1e9, Math.Sin(i * 1.91) * 1e6);
            var b = new Vector3d(Math.Cos(i * 0.37) * 1e7,
                Math.Sin(i * 1.31) * 1e10, Math.Cos(i * 1.73) * 1e13);
            double baseline = a.Dot(b);
            double fma = Math.FusedMultiplyAdd(a.X, b.X,
                Math.FusedMultiplyAdd(a.Y, b.Y, a.Z * b.Z));
            double reassociated = a.X * b.X + (a.Y * b.Y + a.Z * b.Z);
            double oracle = (double)((decimal)a.X * (decimal)b.X
                + (decimal)a.Y * (decimal)b.Y + (decimal)a.Z * (decimal)b.Z);
            double baselineError = Math.Abs(baseline - oracle);
            double fmaError = Math.Abs(fma - oracle);
            double reassociatedError = Math.Abs(reassociated - oracle);
            fmaMaxUlpFromBaseline = Math.Max(fmaMaxUlpFromBaseline, UlpDistance(baseline, fma));
            reassociatedMaxUlpFromBaseline = Math.Max(reassociatedMaxUlpFromBaseline,
                UlpDistance(baseline, reassociated));
            fmaMaxAbsoluteError = Math.Max(fmaMaxAbsoluteError, fmaError);
            reassociatedMaxAbsoluteError = Math.Max(reassociatedMaxAbsoluteError, reassociatedError);
            if (fmaError < baselineError) fmaMoreAccurate++;
            else if (fmaError > baselineError) fmaLessAccurate++;
            if (reassociatedError < baselineError) reassociatedMoreAccurate++;
            else if (reassociatedError > baselineError) reassociatedLessAccurate++;
        }

        var rotation = new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 2.6616995e-6, 0);
        var degree50 = LunarGravityModel.Create(rotation);
        var degree20 = new Geopotential(degree50.ReferenceRadius, rotation,
            degree50.Coefficients.Where(c => c.Degree <= 20));
        var degree10 = new Geopotential(degree50.ReferenceRadius, rotation,
            degree50.Coefficients.Where(c => c.Degree <= 10));
        double degree20MaxAbsolute = 0, degree20MaxRelative = 0, degree20SquareSum = 0;
        double degree10MaxAbsolute = 0, degree10MaxRelative = 0, degree10SquareSum = 0;
        const int lunarCases = 2048;
        for (int i = 0; i < lunarCases; i++)
        {
            double longitude = i * 2 * Math.PI / lunarCases;
            double latitude = Math.Sin(i * 0.6180339887498949) * 1.45;
            double r = 1_788_000 + i % 9 * 50_000;
            var p = new Vector3d(r * Math.Cos(latitude) * Math.Cos(longitude),
                r * Math.Cos(latitude) * Math.Sin(longitude), r * Math.Sin(latitude));
            double time = i * 12345.6789;
            var exact = degree50.AccelerationCorrection(p, 4.9028000661637961e12, time);
            MeasureFieldError(exact,
                degree20.AccelerationCorrection(p, 4.9028000661637961e12, time),
                ref degree20MaxAbsolute, ref degree20MaxRelative, ref degree20SquareSum);
            MeasureFieldError(exact,
                degree10.AccelerationCorrection(p, 4.9028000661637961e12, time),
                ref degree10MaxAbsolute, ref degree10MaxRelative, ref degree10SquareSum);
        }
        double degree20RmsAbsolute = Math.Sqrt(degree20SquareSum / lunarCases);
        double degree10RmsAbsolute = Math.Sqrt(degree10SquareSum / lunarCases);

        output.WriteLine(nameof(vectorMaxUlp) + '=' + vectorMaxUlp);
        output.WriteLine(nameof(vectorMaxRelative) + '=' + vectorMaxRelative);
        output.WriteLine(nameof(pointMassReciprocalMaxUlp) + '=' + pointMassReciprocalMaxUlp);
        output.WriteLine(nameof(pointMassReciprocalMaxRelative) + '=' + pointMassReciprocalMaxRelative);
        output.WriteLine(nameof(pointMassNewtonMaxUlp) + '=' + pointMassNewtonMaxUlp);
        output.WriteLine(nameof(pointMassNewtonMaxRelative) + '=' + pointMassNewtonMaxRelative);
        output.WriteLine(nameof(fmaMaxUlpFromBaseline) + '=' + fmaMaxUlpFromBaseline);
        output.WriteLine(nameof(fmaMoreAccurate) + '=' + fmaMoreAccurate);
        output.WriteLine(nameof(fmaLessAccurate) + '=' + fmaLessAccurate);
        output.WriteLine(nameof(fmaMaxAbsoluteError) + '=' + fmaMaxAbsoluteError);
        output.WriteLine(nameof(reassociatedMaxUlpFromBaseline) + '=' + reassociatedMaxUlpFromBaseline);
        output.WriteLine(nameof(reassociatedMoreAccurate) + '=' + reassociatedMoreAccurate);
        output.WriteLine(nameof(reassociatedLessAccurate) + '=' + reassociatedLessAccurate);
        output.WriteLine(nameof(reassociatedMaxAbsoluteError) + '=' + reassociatedMaxAbsoluteError);
        output.WriteLine(nameof(degree20MaxAbsolute) + '=' + degree20MaxAbsolute);
        output.WriteLine(nameof(degree20MaxRelative) + '=' + degree20MaxRelative);
        output.WriteLine(nameof(degree20RmsAbsolute) + '=' + degree20RmsAbsolute);
        output.WriteLine(nameof(degree10MaxAbsolute) + '=' + degree10MaxAbsolute);
        output.WriteLine(nameof(degree10MaxRelative) + '=' + degree10MaxRelative);
        output.WriteLine(nameof(degree10RmsAbsolute) + '=' + degree10RmsAbsolute);

        Assert.True(vectorMaxUlp > 0);
        Assert.True(pointMassReciprocalMaxUlp > 0);
        Assert.True(pointMassNewtonMaxUlp > 0);
        Assert.True(fmaMaxUlpFromBaseline > 0);
        Assert.True(fmaLessAccurate > 0);
        Assert.True(reassociatedMaxUlpFromBaseline > 0);
        Assert.True(reassociatedLessAccurate > 0);
        Assert.True(degree20MaxAbsolute > 0);
        Assert.True(degree10MaxAbsolute > degree20MaxAbsolute);
    }

    private static void Measure(double exact, double candidate,
        ref ulong maxUlp, ref double maxRelative)
    {
        maxUlp = Math.Max(maxUlp, UlpDistance(exact, candidate));
        if (exact != 0)
            maxRelative = Math.Max(maxRelative, Math.Abs(candidate - exact) / Math.Abs(exact));
    }

    private static void MeasureFieldError(Vector3d exact, Vector3d candidate,
        ref double maxAbsolute, ref double maxRelative, ref double squareSum)
    {
        double error = (candidate - exact).Length();
        maxAbsolute = Math.Max(maxAbsolute, error);
        if (exact.Length() > 0) maxRelative = Math.Max(maxRelative, error / exact.Length());
        squareSum += error * error;
    }

    private static ulong UlpDistance(double a, double b)
    {
        static ulong Ordered(double value)
        {
            ulong bits = (ulong)BitConverter.DoubleToInt64Bits(value);
            return (bits & (1UL << 63)) == 0 ? bits | (1UL << 63) : ~bits;
        }
        ulong x = Ordered(a), y = Ordered(b);
        return x >= y ? x - y : y - x;
    }
}

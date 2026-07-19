using System.Numerics;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Core.Tests;

public class PairwiseMaskOptimizationPrecisionTests
{
    [Fact]
    public void Massive_fast_path_is_bitwise_identical_to_always_masked_simd()
    {
        var mus = Enumerable.Range(0, 50).Select(i => 1e10 * (i + 1)).ToArray();
        var states = CreateStates(mus.Length);
        AssertKernelBits(mus, states);
    }

    [Fact]
    public void Massless_path_retains_mask_and_is_bitwise_identical()
    {
        var mus = Enumerable.Range(0, 13)
            .Select(i => i is 3 or 4 or 8 ? 0.0 : 2e9 * (i + 1)).ToArray();
        var states = CreateStates(mus.Length);
        states[4] = states[3];
        AssertKernelBits(mus, states);
    }

    private static StateVector[] CreateStates(int count)
    {
        var states = new StateVector[count];
        for (int i = 0; i < count; i++)
            states[i] = new StateVector(new Vector3d(
                1e8 * (i + 1) + Math.Sin(i * 0.7) * 1e6,
                -3e8 * (i + 2) + Math.Cos(i * 1.1) * 2e6,
                7e7 * (i * i + 1) - Math.Sin(i * 0.3) * 3e6), Vector3d.Zero);
        return states;
    }

    private static void AssertKernelBits(double[] mus, StateVector[] states)
    {
        var expected = new Vector3d[states.Length];
        var actual = new Vector3d[states.Length];
        ReferenceCompute(mus, states, expected);
        new PairwiseAccelerationKernel(mus).Compute(states, actual);
        for (int i = 0; i < states.Length; i++)
        {
            AssertBits(expected[i].X, actual[i].X);
            AssertBits(expected[i].Y, actual[i].Y);
            AssertBits(expected[i].Z, actual[i].Z);
        }
    }

    private static void ReferenceCompute(double[] mus, StateVector[] states, Vector3d[] acc)
    {
        int n = states.Length;
        var px = new double[n]; var py = new double[n]; var pz = new double[n];
        var ax = new double[n]; var ay = new double[n]; var az = new double[n];
        for (int i = 0; i < n; i++)
        {
            var p = states[i].Position;
            px[i] = p.X; py[i] = p.Y; pz[i] = p.Z;
        }

        int w = Vector<double>.Count;
        for (int i = 0; i < n; i++)
        {
            double pix = px[i], piy = py[i], piz = pz[i];
            double mui = mus[i];
            double aix = 0, aiy = 0, aiz = 0;
            int j = i + 1;
            if (n - j >= w)
            {
                var vpix = new Vector<double>(pix);
                var vpiy = new Vector<double>(piy);
                var vpiz = new Vector<double>(piz);
                var vmui = new Vector<double>(mui);
                Vector<double> vaix = default, vaiy = default, vaiz = default;
                for (; j <= n - w; j += w)
                {
                    var dx = vpix - new Vector<double>(px, j);
                    var dy = vpiy - new Vector<double>(py, j);
                    var dz = vpiz - new Vector<double>(pz, j);
                    var r2 = dx * dx + dy * dy + dz * dz;
                    var vmuj = new Vector<double>(mus, j);
                    var zeroZero = Vector.Equals(vmui, Vector<double>.Zero)
                        & Vector.Equals(vmuj, Vector<double>.Zero);
                    r2 = Vector.ConditionalSelect(zeroZero, Vector<double>.One, r2);
                    var invR3 = Vector<double>.One / (r2 * Vector.SquareRoot(r2));
                    var sj = vmuj * invR3;
                    vaix -= dx * sj; vaiy -= dy * sj; vaiz -= dz * sj;
                    var si = vmui * invR3;
                    (new Vector<double>(ax, j) + dx * si).CopyTo(ax, j);
                    (new Vector<double>(ay, j) + dy * si).CopyTo(ay, j);
                    (new Vector<double>(az, j) + dz * si).CopyTo(az, j);
                }
                aix = Vector.Sum(vaix); aiy = Vector.Sum(vaiy); aiz = Vector.Sum(vaiz);
            }
            for (; j < n; j++)
            {
                if (mui == 0 && mus[j] == 0) continue;
                double dx = pix - px[j], dy = piy - py[j], dz = piz - pz[j];
                double r2 = dx * dx + dy * dy + dz * dz;
                double invR3 = 1.0 / (r2 * Math.Sqrt(r2));
                double sj = mus[j] * invR3;
                aix -= dx * sj; aiy -= dy * sj; aiz -= dz * sj;
                double si = mui * invR3;
                ax[j] += dx * si; ay[j] += dy * si; az[j] += dz * si;
            }
            ax[i] += aix; ay[i] += aiy; az[i] += aiz;
        }
        for (int i = 0; i < n; i++) acc[i] = new Vector3d(ax[i], ay[i], az[i]);
    }

    private static void AssertBits(double expected, double actual) =>
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected), BitConverter.DoubleToInt64Bits(actual));
}

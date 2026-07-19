using System.Numerics;

namespace WhiskerDynamics.Core;

/// <summary>Restricted point-mass gravity kernel for the rails RHS. Massive bodies
/// interact mutually. Zero-mu catalog entries feel every massive body but neither
/// back-react nor interact with one another. This is the exact zero-mass limit and
/// avoids the false 0 * infinity singularity of coincident massless catalog records.
/// The scratch is per-instance mutable state: not thread-safe, callers serialize
/// (same contract as <see cref="NBodyEphemerides"/>, which owns one instance).</summary>
internal sealed class PairwiseAccelerationKernel
{
    private readonly double[] _mus;
    private readonly bool _hasMasslessBody;
    private readonly double[] _px, _py, _pz;
    private readonly double[] _ax, _ay, _az;

    internal PairwiseAccelerationKernel(double[] mus)
    {
        _mus = mus;
        _hasMasslessBody = Array.IndexOf(mus, 0.0) >= 0;
        int n = mus.Length;
        _px = new double[n]; _py = new double[n]; _pz = new double[n];
        _ax = new double[n]; _ay = new double[n]; _az = new double[n];
    }

    /// <summary>Overwrites <paramref name="acc"/> with the mutual gravitational
    /// acceleration of every body in <paramref name="states"/> (positions used;
    /// velocities ignored). Lengths must equal the constructor's mu count.</summary>
    internal void Compute(StateVector[] states, Vector3d[] acc)
    {
        int n = states.Length;
        var mus = _mus;
        var px = _px; var py = _py; var pz = _pz;
        var ax = _ax; var ay = _ay; var az = _az;

        for (int i = 0; i < n; i++)
        {
            var p = states[i].Position;
            px[i] = p.X; py[i] = p.Y; pz[i] = p.Z;
            ax[i] = 0; ay[i] = 0; az[i] = 0;
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
                    if (_hasMasslessBody)
                    {
                        var zeroZero = Vector.Equals(vmui, Vector<double>.Zero)
                            & Vector.Equals(vmuj, Vector<double>.Zero);
                        // A zero/zero pair exerts identically zero force. Give those
                        // lanes a finite dummy distance so 0 * infinity cannot form.
                        r2 = Vector.ConditionalSelect(zeroZero, Vector<double>.One, r2);
                    }
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

        for (int i = 0; i < n; i++)
            acc[i] = new Vector3d(ax[i], ay[i], az[i]);
    }
}

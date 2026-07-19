using System.Numerics;

namespace WhiskerDynamics.Core;

/// <summary>Summed point-mass gravitational acceleration from celestial bodies on rails.
/// Over an <see cref="NBodyEphemerides"/> this instance carries mutable per-segment
/// caches (SIMD coefficient tables), so it shares the ephemerides' thread-safety
/// contract: not thread-safe, callers serialize.</summary>
public sealed class GravityModel
{
    private readonly IEphemerides _ephemerides;
    private readonly CelestialBody[] _sources;
    private readonly CelestialBody[] _extendedSources;
    private readonly ISegmentedEphemerides? _segmented; // rails/snapshot fast path
    private readonly int[] _integratedIndex = []; // modeled segment index per source

    // SIMD fast path over the integrated sources. Rails segments are PER BODY
    // (committed quintic knots or dense tail nodes — see NBodyEphemerides), so each
    // source slot caches its own segment: its validity window [T0, T1], the u mapping
    // 1/dt, and the segment position refactored into monomial coefficients
    // position(u) = ((((c5·u + c4)·u + c3)·u + c2)·u + c1)·u + c0 per component
    // (a tail segment's cubic is the same polynomial with c4 = c5 = 0), stored SoA and
    // padded to the vector width with zero-mass far-away entries, so one RHS call is a
    // single vector sweep with a per-lane u computed in the sweep itself. The
    // steady-state gate is ONE scalar range check: the intersection window
    // [_coveredFrom, _coveredTo] of every slot's segment — inside it nothing can be
    // stale. On a miss only the expired slots rebuild: in the dense tail (where every
    // body's segment expires together) they refactor from direct node rows in one
    // batched pass; committed-region slots rebuild individually and rarely (slow
    // bodies keep their coefficients for days). All slots drop on the ephemerides'
    // CommitStamp: a cached tail segment must not keep answering for an instant that
    // has committed.
    private readonly int[] _integratedSlots = []; // integrated body index per compact slot
    private readonly double[] _smu = [];
    private readonly double[] _segT0 = [], _segT1 = [], _segInvDt = [];
    private readonly double[] _c0x = [], _c0y = [], _c0z = [];
    private readonly double[] _c1x = [], _c1y = [], _c1z = [];
    private readonly double[] _c2x = [], _c2y = [], _c2z = [];
    private readonly double[] _c3x = [], _c3y = [], _c3z = [];
    private readonly double[] _c4x = [], _c4y = [], _c4z = [];
    private readonly double[] _c5x = [], _c5y = [], _c5z = [];
    private double _coveredFrom = double.MaxValue; // intersection of slot windows
    private double _coveredTo = double.MinValue;
    private int _cachedCommitStamp = -1;

    public GravityModel(IEphemerides ephemerides, IEnumerable<CelestialBody>? sources = null)
    {
        _ephemerides = ephemerides;
        // Integrated zero-mu trajectories are evaluable, but never gravity sources.
        // Filtering avoids wasted vessel-RHS slots and a false 0 * infinity
        // singularity if a query position coincides with a massless catalog entry.
        _sources = (sources ?? ephemerides.Bodies).Where(body => body.Mu != 0).ToArray();
        _extendedSources = _sources.Where(body => body.Geopotential is not null).ToArray();
        _segmented = ephemerides as ISegmentedEphemerides;
        if (_segmented is not null)
        {
            _integratedIndex = _sources.Select(_segmented.IntegratedIndexOf).ToArray();

            var slots = new List<int>();
            var slotMus = new List<double>();
            for (int s = 0; s < _sources.Length; s++)
            {
                if (_integratedIndex[s] < 0)
                    throw new ArgumentException(
                        $"Gravity source '{_sources[s].Id}' is not modeled by this ephemeris.",
                        nameof(sources));
                slots.Add(_integratedIndex[s]);
                slotMus.Add(_sources[s].Mu);
            }
            _integratedSlots = slots.ToArray();

            int w = Vector<double>.Count;
            int padded = (slots.Count + w - 1) / w * w;
            _smu = new double[padded];
            _segT0 = new double[padded]; _segT1 = new double[padded];
            _segInvDt = new double[padded];
            _c0x = new double[padded]; _c0y = new double[padded]; _c0z = new double[padded];
            _c1x = new double[padded]; _c1y = new double[padded]; _c1z = new double[padded];
            _c2x = new double[padded]; _c2y = new double[padded]; _c2z = new double[padded];
            _c3x = new double[padded]; _c3y = new double[padded]; _c3z = new double[padded];
            _c4x = new double[padded]; _c4y = new double[padded]; _c4z = new double[padded];
            _c5x = new double[padded]; _c5y = new double[padded]; _c5z = new double[padded];
            for (int k = 0; k < slots.Count; k++) _smu[k] = slotMus[k];
            for (int k = slots.Count; k < padded; k++)
            {
                // Padding lanes: mu = 0 and a position so remote that r² stays finite
                // and huge — the lane contributes exactly 0/huge = 0, never NaN. The
                // rebuild loops never reach padding slots; T0 stays FINITE so the
                // per-lane u = (time − T0)·invDt is exactly 0·finite = 0, never the
                // ∞·0 NaN an infinite sentinel would make.
                _c0x[k] = 1e30; _c0y[k] = 1e30; _c0z[k] = 1e30;
            }
        }
    }

    /// <summary>The bodies whose gravity this model sums.</summary>
    public IReadOnlyList<CelestialBody> Sources => _sources;

    public Vector3d AccelerationAt(Vector3d position, double time)
    {
        var acceleration = Vector3d.Zero;
        if (_segmented is { } nbody)
        {
            if (_integratedSlots.Length > 0)
            {
                if (nbody.CommitStamp != _cachedCommitStamp) InvalidateSlots(nbody.CommitStamp);
                if (time < _coveredFrom || time > _coveredTo) RebuildExpiredSlots(nbody, time);
                acceleration = SumIntegratedSimd(position, time);
            }
            return acceleration + ExtendedBodyAcceleration(position, time);
        }
        foreach (var body in _sources)
        {
            var offset = position - _ephemerides.GetState(body, time).Position;
            double r2 = offset.LengthSquared();
            acceleration -= offset * (body.Mu / (r2 * Math.Sqrt(r2)));
        }
        return acceleration + ExtendedBodyAcceleration(position, time);
    }

    private Vector3d ExtendedBodyAcceleration(Vector3d position, double time)
    {
        var correction = Vector3d.Zero;
        foreach (var body in _extendedSources)
        {
            var relative = position - _ephemerides.GetState(body, time).Position;
            correction += body.Geopotential!.AccelerationCorrection(relative, body.Mu, time);
        }
        return correction;
    }

    private void InvalidateSlots(int commitStamp)
    {
        _cachedCommitStamp = commitStamp;
        for (int k = 0; k < _integratedSlots.Length; k++) _segT0[k] = double.NaN;
        _coveredFrom = double.MaxValue;
        _coveredTo = double.MinValue;
    }

    /// <summary>The miss path: rebuild every slot whose window does not cover
    /// <paramref name="time"/>, then re-derive the intersection window. Tail-region
    /// backbone slots share ONE dense bracket (resolved once, refactored from direct
    /// node rows); restricted and committed-region slots resolve individually. A query past the horizon
    /// extends (and commits) through the general resolve first, re-syncing the commit
    /// stamp so no slot keeps a pre-commit segment.</summary>
    private void RebuildExpiredSlots(ISegmentedEphemerides nbody, double time)
    {
        // Poison the window FIRST: a mid-loop throw (a query behind some body's
        // retained start — contained by callers) must leave the next call re-entering
        // this rebuild, never sweeping half-rebuilt tables behind a stale window.
        // Partially rebuilt slots are harmless in themselves: a slot's window is only
        // ever written together with coefficients that are correct for it.
        _coveredFrom = double.MaxValue;
        _coveredTo = double.MinValue;
        if (time > nbody.Horizon)
        {
            _ = nbody.ResolveBodySegment(_integratedSlots[0], time); // extend + commit once
            if (nbody.CommitStamp != _cachedCommitStamp) InvalidateSlots(nbody.CommitStamp);
        }
        bool haveDense = nbody.TryResolveDenseSegment(time, out int hi, out double t0, out double dt);
        for (int k = 0; k < _integratedSlots.Length; k++)
        {
            if (time >= _segT0[k] && time <= _segT1[k]) continue;
            int body = _integratedSlots[k];
            if (haveDense && nbody.IsBackbone(_sources[k])
                && !nbody.InCommittedRegion(body, time))
                SetCubic(k, t0, dt, nbody.DenseNodeState(hi - 1, body), nbody.DenseNodeState(hi, body));
            else
                RebuildSlot(nbody, k, time);
        }
        double from = double.NegativeInfinity, to = double.PositiveInfinity;
        for (int k = 0; k < _integratedSlots.Length; k++)
        {
            from = Math.Max(from, _segT0[k]);
            to = Math.Min(to, _segT1[k]);
        }
        _coveredFrom = from;
        _coveredTo = to;
    }

    /// <summary>Evaluates cached segment polynomials and accumulates point-mass gravity
    /// over SIMD-padded source tables.</summary>
    private Vector3d SumIntegratedSimd(Vector3d position, double time)
    {
        var vtime = new Vector<double>(time);
        var vpx = new Vector<double>(position.X);
        var vpy = new Vector<double>(position.Y);
        var vpz = new Vector<double>(position.Z);
        Vector<double> ax = default, ay = default, az = default;
        int w = Vector<double>.Count;
        for (int j = 0; j < _smu.Length; j += w)
        {
            var vu = (vtime - new Vector<double>(_segT0, j)) * new Vector<double>(_segInvDt, j);
            var sx = ((((new Vector<double>(_c5x, j) * vu + new Vector<double>(_c4x, j)) * vu
                      + new Vector<double>(_c3x, j)) * vu + new Vector<double>(_c2x, j)) * vu
                      + new Vector<double>(_c1x, j)) * vu + new Vector<double>(_c0x, j);
            var sy = ((((new Vector<double>(_c5y, j) * vu + new Vector<double>(_c4y, j)) * vu
                      + new Vector<double>(_c3y, j)) * vu + new Vector<double>(_c2y, j)) * vu
                      + new Vector<double>(_c1y, j)) * vu + new Vector<double>(_c0y, j);
            var sz = ((((new Vector<double>(_c5z, j) * vu + new Vector<double>(_c4z, j)) * vu
                      + new Vector<double>(_c3z, j)) * vu + new Vector<double>(_c2z, j)) * vu
                      + new Vector<double>(_c1z, j)) * vu + new Vector<double>(_c0z, j);
            var dx = vpx - sx;
            var dy = vpy - sy;
            var dz = vpz - sz;
            var r2 = dx * dx + dy * dy + dz * dz;
            var s = new Vector<double>(_smu, j) / (r2 * Vector.SquareRoot(r2));
            ax -= dx * s; ay -= dy * s; az -= dz * s;
        }
        return new Vector3d(Vector.Sum(ax), Vector.Sum(ay), Vector.Sum(az));
    }


    /// <summary>General per-slot rebuild through the ephemerides' segment resolve —
    /// the committed-region and exact-hit path (the batched tail path bypasses it).</summary>
    private void RebuildSlot(ISegmentedEphemerides nbody, int k, double time)
    {
        var seg = nbody.ResolveBodySegment(_integratedSlots[k], time);
        if (seg.Dt == 0) SetConstant(k, seg.T0, seg.A.Position);
        else if (seg.Quintic) SetQuintic(k, seg.T0, seg.Dt, seg.A, seg.AccA, seg.B, seg.AccB);
        else SetCubic(k, seg.T0, seg.Dt, seg.A, seg.B);
    }

    private void SetConstant(int k, double t, Vector3d position)
    {
        _segT0[k] = t; _segT1[k] = t; _segInvDt[k] = 0;
        _c0x[k] = position.X; _c0y[k] = position.Y; _c0z[k] = position.Z;
        _c1x[k] = 0; _c1y[k] = 0; _c1z[k] = 0;
        _c2x[k] = 0; _c2y[k] = 0; _c2z[k] = 0;
        _c3x[k] = 0; _c3y[k] = 0; _c3z[k] = 0;
        _c4x[k] = 0; _c4y[k] = 0; _c4z[k] = 0;
        _c5x[k] = 0; _c5y[k] = 0; _c5z[k] = 0;
    }

    /// <summary>Cubic (dense tail) segment refactored into the monomial tables:
    /// c0 = Pa, c1 = Va·dt, c2 = 3(Pb−Pa) − (2Va+Vb)·dt, c3 = 2(Pa−Pb) + (Va+Vb)·dt
    /// (the same polynomial as <see cref="NBodyEphemerides.SegmentPosition"/>'s cubic
    /// branch, re-associated — last-ulp differences only).</summary>
    private void SetCubic(int k, double t0, double dt, in StateVector a, in StateVector b)
    {
        _segT0[k] = t0; _segT1[k] = t0 + dt; _segInvDt[k] = 1.0 / dt;
        var pa = a.Position; var pb = b.Position;
        var vadt = a.Velocity * dt; var vbdt = b.Velocity * dt;
        _c0x[k] = pa.X; _c0y[k] = pa.Y; _c0z[k] = pa.Z;
        _c1x[k] = vadt.X; _c1y[k] = vadt.Y; _c1z[k] = vadt.Z;
        _c2x[k] = 3 * (pb.X - pa.X) - (2 * vadt.X + vbdt.X);
        _c2y[k] = 3 * (pb.Y - pa.Y) - (2 * vadt.Y + vbdt.Y);
        _c2z[k] = 3 * (pb.Z - pa.Z) - (2 * vadt.Z + vbdt.Z);
        _c3x[k] = 2 * (pa.X - pb.X) + (vadt.X + vbdt.X);
        _c3y[k] = 2 * (pa.Y - pb.Y) + (vadt.Y + vbdt.Y);
        _c3z[k] = 2 * (pa.Z - pb.Z) + (vadt.Z + vbdt.Z);
        _c4x[k] = 0; _c4y[k] = 0; _c4z[k] = 0;
        _c5x[k] = 0; _c5y[k] = 0; _c5z[k] = 0;
    }

    /// <summary>Quintic (committed knot) segment refactored into the monomial tables:
    /// c2 = Aa·dt²/2,
    /// c3 = 10(Pb−Pa) − (6Va+4Vb)·dt − (1.5·Aa − 0.5·Ab)·dt²,
    /// c4 = 15(Pa−Pb) + (8Va+7Vb)·dt + (1.5·Aa − Ab)·dt²,
    /// c5 = 6(Pb−Pa) − 3(Va+Vb)·dt − 0.5·(Aa − Ab)·dt²
    /// (the same polynomial as <see cref="NBodyEphemerides.SegmentPosition"/>'s quintic
    /// branch, re-associated — last-ulp differences only).</summary>
    private void SetQuintic(int k, double t0, double dt, in StateVector a, in Vector3d accA,
        in StateVector b, in Vector3d accB)
    {
        _segT0[k] = t0; _segT1[k] = t0 + dt; _segInvDt[k] = 1.0 / dt;
        var pa = a.Position; var pb = b.Position;
        var vadt = a.Velocity * dt; var vbdt = b.Velocity * dt;
        var aadt2 = accA * (dt * dt); var abdt2 = accB * (dt * dt);
        _c0x[k] = pa.X; _c0y[k] = pa.Y; _c0z[k] = pa.Z;
        _c1x[k] = vadt.X; _c1y[k] = vadt.Y; _c1z[k] = vadt.Z;
        _c2x[k] = 0.5 * aadt2.X; _c2y[k] = 0.5 * aadt2.Y; _c2z[k] = 0.5 * aadt2.Z;
        _c3x[k] = 10 * (pb.X - pa.X) - (6 * vadt.X + 4 * vbdt.X) - (1.5 * aadt2.X - 0.5 * abdt2.X);
        _c3y[k] = 10 * (pb.Y - pa.Y) - (6 * vadt.Y + 4 * vbdt.Y) - (1.5 * aadt2.Y - 0.5 * abdt2.Y);
        _c3z[k] = 10 * (pb.Z - pa.Z) - (6 * vadt.Z + 4 * vbdt.Z) - (1.5 * aadt2.Z - 0.5 * abdt2.Z);
        _c4x[k] = 15 * (pa.X - pb.X) + (8 * vadt.X + 7 * vbdt.X) + (1.5 * aadt2.X - abdt2.X);
        _c4y[k] = 15 * (pa.Y - pb.Y) + (8 * vadt.Y + 7 * vbdt.Y) + (1.5 * aadt2.Y - abdt2.Y);
        _c4z[k] = 15 * (pa.Z - pb.Z) + (8 * vadt.Z + 7 * vbdt.Z) + (1.5 * aadt2.Z - abdt2.Z);
        _c5x[k] = 6 * (pb.X - pa.X) - 3 * (vadt.X + vbdt.X) - 0.5 * (aadt2.X - abdt2.X);
        _c5y[k] = 6 * (pb.Y - pa.Y) - 3 * (vadt.Y + vbdt.Y) - 0.5 * (aadt2.Y - abdt2.Y);
        _c5z[k] = 6 * (pb.Z - pa.Z) - 3 * (vadt.Z + vbdt.Z) - 0.5 * (aadt2.Z - abdt2.Z);
    }

    /// <summary>Third-body correction for a point at
    /// <paramref name="parentRelativePosition"/> from <paramref name="parent"/>:
    /// A source felt by the parent's numerical trajectory contributes a tidal term
    /// because that trajectory received g_j(parent). Every other source contributes a
    /// direct term. Restricted descendants therefore treat massive restricted
    /// ancestors as tides while restricted peers remain direct.
    ///
    /// This is exactly the correction that turns parent-relative single-mu dynamics
    /// into full n-body dynamics relative to a parent that itself accelerates along
    /// its n-body rail: rel'' = g_parent(point) + delta. It is NOT the direct sum
    /// gN(point) - g_parent(point) — that form omits the parent's own rails
    /// acceleration and would double-count the star's direct pull (~6e-3 m/s^2 near
    /// 1 AU) instead of its tide (~1e-6..1e-5 m/s^2 in LEO..GEO). Exactly zero in a
    /// pure two-body world. <paramref name="parent"/> is matched by reference against
    /// <see cref="Sources"/>. Same thread-safety contract as
    /// <see cref="AccelerationAt"/>: callers serialize access to the ephemerides.
    /// Over <see cref="NBodyEphemerides"/>, <paramref name="parent"/> must be modeled.</summary>
    public Vector3d ParentRelativeCorrectionAt(
        CelestialBody parent, Vector3d parentRelativePosition, double time)
    {
        var delta = Vector3d.Zero;
        if (_segmented is { } nbody)
        {
            int parentIndex = nbody.IntegratedIndexOf(parent);
            if (parentIndex < 0)
                throw new ArgumentException(
                    "Parent-relative correction requires a modeled parent.",
                    nameof(parent));
            var parentPos = nbody.BodyPositionAt(parentIndex, time);
            // Preserve source order for bitwise parity with consumers that cache these terms.
            for (int s = 0; s < _sources.Length; s++)
            {
                var body = _sources[s];
                if (ReferenceEquals(body, parent)) continue;
                int index = _integratedIndex[s];
                var bodyPosition = nbody.BodyPositionAt(index, time);
                var parentToBody = bodyPosition - parentPos;
                delta += body.Mu * (nbody.FeelsGravityFrom(parent, body)
                    ? TidalTerm(parentToBody, parentRelativePosition)
                    : DirectPointMassTerm(parentToBody, parentRelativePosition));
            }
            return delta;
        }
        var parentPosition = _ephemerides.GetState(parent, time).Position;
        foreach (var body in _sources)
        {
            if (ReferenceEquals(body, parent)) continue;
            var parentToBody = _ephemerides.GetState(body, time).Position - parentPosition;
            delta += body.Mu * TidalTerm(parentToBody, parentRelativePosition);
        }
        return delta;
    }

    /// <summary>Compatibility name for <see cref="ParentRelativeCorrectionAt"/>.</summary>
    public Vector3d ThirdBodyDeltaAt(
        CelestialBody parent, Vector3d parentRelativePosition, double time) =>
        ParentRelativeCorrectionAt(parent, parentRelativePosition, time);

    /// <summary>Per-unit-mu tidal kernel: (s - r)/|s - r|^3 - s/|s|^3 for a third body
    /// at parent-relative <paramref name="parentToBody"/> (s) and a point at
    /// parent-relative <paramref name="parentRelativePosition"/> (r). Linearizes to the
    /// classic tide (3*s_hat*(s_hat . r) - r)/|s|^3 for |r| &lt;&lt; |s|.</summary>
    public static Vector3d TidalTerm(Vector3d parentToBody, Vector3d parentRelativePosition)
    {
        var pointToBody = parentToBody - parentRelativePosition;
        double a2 = pointToBody.LengthSquared();
        double b2 = parentToBody.LengthSquared();
        return pointToBody / (a2 * Math.Sqrt(a2)) - parentToBody / (b2 * Math.Sqrt(b2));
    }

    /// <summary>Per-unit-mu direct acceleration at a point. Parent-relative dynamics
    /// uses this for restricted sources, which do not accelerate the parent's track.</summary>
    public static Vector3d DirectPointMassTerm(
        Vector3d parentToBody, Vector3d parentRelativePosition)
    {
        var pointToBody = parentToBody - parentRelativePosition;
        double r2 = pointToBody.LengthSquared();
        return pointToBody / (r2 * Math.Sqrt(r2));
    }

    /// <summary>Direct extended-body acceleration from a non-parent source.  Unlike
    /// point-mass gravity this is NOT differenced at the parent: celestial rails are
    /// intentionally point-mass and therefore never received a term to subtract.</summary>
    public static Vector3d ExtendedBodyDirectTerm(Geopotential field, double mu,
        Vector3d parentToBody, Vector3d parentRelativePosition, double time) =>
        field.AccelerationCorrection(parentRelativePosition - parentToBody, mu, time);
}

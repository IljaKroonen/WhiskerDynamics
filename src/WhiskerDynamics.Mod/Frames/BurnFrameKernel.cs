using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Frames;

/// <summary>Converts delta-v among stock VLF, ecliptic, and catalog-frame bases.
/// </summary>
public static class BurnFrameKernel
{
    public const double MinVlfSine = FrameCatalog.MinRotationSine;

    /// <summary>The stock VLF basis from a parent-relative state, in the axes of the
    /// inputs: X = prograde (velocity direction), Y = orbit normal (r-hat x v-hat),
    /// Z = radial-out (v-hat x h-hat). False on degenerate geometry (zero vectors or
    /// (nearly) radial motion) — mirrors GetVlf2GivenFrame returning null.</summary>
    public static bool TryVlfBasis(Vector3d rRel, Vector3d vRel,
        out Vector3d x, out Vector3d y, out Vector3d z)
    {
        x = y = z = default;
        double rLength = rRel.Length();
        double vLength = vRel.Length();
        if (rLength == 0 || vLength == 0) return false;
        var rHat = rRel / rLength;
        var vHat = vRel / vLength;
        var h = rHat.Cross(vHat);
        double hLength = h.Length();
        if (hLength < MinVlfSine) return false;
        x = vHat;
        y = h / hLength;
        z = x.Cross(y).Normalized();
        return true;
    }

    /// <summary>VLF components (prograde, normal, outward) -> the same delta-v in the
    /// axes rRel/vRel are expressed in. Null on degenerate geometry.</summary>
    public static Vector3d? VlfToEcl(Vector3d dvVlf, Vector3d rRel, Vector3d vRel) =>
        TryVlfBasis(rRel, vRel, out var x, out var y, out var z)
            ? x * dvVlf.X + y * dvVlf.Y + z * dvVlf.Z
            : null;

    /// <summary>Inverse of <see cref="VlfToEcl"/>: axis projections (the basis is
    /// orthonormal). Null on degenerate geometry.</summary>
    public static Vector3d? EclToVlf(Vector3d dvEcl, Vector3d rRel, Vector3d vRel) =>
        TryVlfBasis(rRel, vRel, out var x, out var y, out var z)
            ? new Vector3d(dvEcl.Dot(x), dvEcl.Dot(y), dvEcl.Dot(z))
            : null;

    /// <summary>Authored components along a frame pose's UNIT axes -> ecliptic delta-v.
    /// Axes only — a delta-v is a direction quantity, so neither the pose ORIGIN nor its
    /// SCALE ever enters (FramePose.FromFrame would add the origin AND multiply by Scale;
    /// rotating-pulsating: a two-body pose carries Scale = the pair's
    /// separation, and pulsation must NOT scale delta-v — authored m/s are m/s). Assumes
    /// orthonormal axes (FrameKernel's constructors guarantee unit axes for every kind;
    /// FrameCatalog gates live data). Pinned by BurnFrameKernelTests'
    /// scale-blindness test.</summary>
    public static Vector3d FrameToEcl(Vector3d components, in FramePose pose) =>
        pose.XAxis * components.X + pose.YAxis * components.Y + pose.ZAxis * components.Z;

    /// <summary>Inverse of <see cref="FrameToEcl"/>: axis projections — equally
    /// origin- and Scale-blind.</summary>
    public static Vector3d EclToFrame(Vector3d dvEcl, in FramePose pose) =>
        new(dvEcl.Dot(pose.XAxis), dvEcl.Dot(pose.YAxis), dvEcl.Dot(pose.ZAxis));

    /// <summary>A burn authored in a catalog frame is expressed as
    /// PROGRADE / RADIAL / NORMAL components of
    /// the vessel's FRAME-RELATIVE trajectory at burn time — not raw frame axes (an
    /// "X (ecl)" component is meaningless to fly). The basis is the same construction
    /// as stock's VLF (<see cref="TryVlfBasis"/>) applied to the frame-space state:
    /// prograde = frame-relative velocity direction (the drawn line's tangent — in a
    /// rotating-pulsating frame that is the direction the vessel MOVES IN THE FRAME),
    /// normal = frame-space r x v direction,
    /// radial = prograde x normal (outward). Component order matches the panel
    /// labels: X=prograde, Y=radial, Z=normal. Null on degenerate
    /// geometry (vessel (nearly) stationary in the frame — e.g. hovering at a
    /// libration point — or (nearly) radial motion).</summary>
    public static Vector3d? FrenetToFrame(Vector3d authoredPrn, Vector3d rFrame, Vector3d vFrame) =>
        TryVlfBasis(rFrame, vFrame, out var prograde, out var normal, out var radial)
            ? prograde * authoredPrn.X + radial * authoredPrn.Y + normal * authoredPrn.Z
            : null;

    /// <summary>Inverse of <see cref="FrenetToFrame"/>: (prograde, radial, normal)
    /// projections of a frame-space delta-v. Null on degenerate geometry.</summary>
    public static Vector3d? FrameToFrenet(Vector3d dvFrame, Vector3d rFrame, Vector3d vFrame) =>
        TryVlfBasis(rFrame, vFrame, out var prograde, out var normal, out var radial)
            ? new Vector3d(dvFrame.Dot(prograde), dvFrame.Dot(radial), dvFrame.Dot(normal))
            : null;

    /// <summary>Staleness rule for a frame-authored burn: when the plan around a burn
    /// changes (an earlier burn edited, the burn dragged in time), the predicted
    /// pre-burn state — and with it the VLF basis — shifts, so the stock VLF components
    /// no longer realize the authored frame components. Stale when the freshly
    /// converted VLF differs from the stored stock VLF by more than
    /// <paramref name="toleranceMps"/> (m/s, magnitude of the vector difference).</summary>
    public static bool IsStale(Vector3d freshDvVlf, Vector3d currentDvVlf, double toleranceMps) =>
        (freshDvVlf - currentDvVlf).Length() > toleranceMps;
}

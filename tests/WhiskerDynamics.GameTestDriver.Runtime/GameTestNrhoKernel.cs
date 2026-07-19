using WhiskerDynamics.Core;

namespace WhiskerDynamics.GameTestDriver.Runtime;

/// <summary>KSA-free reference math for the opt-in NRHO game scenario. The selected
/// orbit is the 6.501-day Earth-Moon L2 southern halo record from NASA/JPL's Poincare
/// catalog (API v1.0, queried as halo/libr=2/branch=S). The catalog state is at
/// apolune; the state below is the same record propagated by half a period to the
/// close lunar passage so stock SOI bookkeeping can execute the insertion burn in
/// Luna VLF.</summary>
internal static class GameTestNrhoKernel
{
    internal const double MassRatio = 0.01215058560962404;
    internal const double PeriodNormalized = 1.46669510795117;

    // Barycentric synodic state at perilune, derived from JPL catalog record
    // [x,z,vy] = [1.01865929880526,-0.179672100884756,-0.0958140620387836].
    private static readonly Cr3bpState Perilune = new(
        0.9874616352932665, 0, 0.007128057076522584,
        0, 1.8181630479961242, 0);

    internal static (Vector3d Position, Vector3d Velocity) Propagate(double normalizedTime)
    {
        if (!double.IsFinite(normalizedTime))
            throw new ArgumentOutOfRangeException(nameof(normalizedTime));
        double time = normalizedTime % PeriodNormalized;
        if (time < 0) time += PeriodNormalized;

        Cr3bpState state = Perilune;
        // Perilune is fast and the family is unstable; 1e-4 TU keeps a full
        // revolution reversible to the catalog state without making this sparse
        // game-test-only calculation material to frame time.
        int steps = Math.Max(1, (int)Math.Ceiling(time / 0.0001));
        double h = time / steps;
        for (int i = 0; i < steps; i++)
        {
            Cr3bpState k1 = Derivative(state);
            Cr3bpState k2 = Derivative(state.Add(k1, h * 0.5));
            Cr3bpState k3 = Derivative(state.Add(k2, h * 0.5));
            Cr3bpState k4 = Derivative(state.Add(k3, h));
            state = state.Add(k1.Add(k2.Scale(2)).Add(k3.Scale(2)).Add(k4), h / 6);
        }
        return (new Vector3d(state.X, state.Y, state.Z),
            new Vector3d(state.Vx, state.Vy, state.Vz));
    }

    /// <summary>Embeds the normalized synodic reference into the game's root-ecliptic
    /// frame. FramePose is position-only, so fixed-coordinate motion is differentiated
    /// from adjacent poses and CR3BP coordinate velocity is added explicitly.</summary>
    internal static StateVector Embed(double normalizedTime, double timeUnitSeconds,
        in FramePose before, in FramePose at, in FramePose after, double poseStepSeconds)
    {
        if (!(timeUnitSeconds > 0) || !double.IsFinite(timeUnitSeconds))
            throw new ArgumentOutOfRangeException(nameof(timeUnitSeconds));
        if (!(poseStepSeconds > 0) || !double.IsFinite(poseStepSeconds))
            throw new ArgumentOutOfRangeException(nameof(poseStepSeconds));

        var state = Propagate(normalizedTime);
        // JPL is barycentric; FramePose.Rotating is primary-centred.
        var coordinate = state.Position + new Vector3d(MassRatio, 0, 0);
        Vector3d position = at.FromFrame(coordinate);
        Vector3d velocity = (after.FromFrame(coordinate) - before.FromFrame(coordinate))
            / (2 * poseStepSeconds)
            + at.XAxis * (state.Velocity.X * at.Scale / timeUnitSeconds)
            + at.YAxis * (state.Velocity.Y * at.Scale / timeUnitSeconds)
            + at.ZAxis * (state.Velocity.Z * at.Scale / timeUnitSeconds);
        return new StateVector(position, velocity);
    }

    internal static Vector3d Feedback(StateVector current, StateVector target,
        double correctionTimescaleSeconds)
    {
        if (!(correctionTimescaleSeconds > 0)
            || !double.IsFinite(correctionTimescaleSeconds))
            throw new ArgumentOutOfRangeException(nameof(correctionTimescaleSeconds));
        return target.Velocity - current.Velocity
            + (target.Position - current.Position) / correctionTimescaleSeconds;
    }

    internal static (double Prograde, double Normal, double Outward) ToVlf(
        StateVector current, Vector3d deltaV)
    {
        double speed = current.Velocity.Length();
        Vector3d angularMomentum = current.Position.Cross(current.Velocity);
        double angularMomentumMagnitude = angularMomentum.Length();
        if (!(speed > 0) || !(angularMomentumMagnitude > 0))
            throw new InvalidOperationException("NRHO state has no usable VLF basis");
        Vector3d prograde = current.Velocity / speed;
        Vector3d normal = angularMomentum / angularMomentumMagnitude;
        Vector3d outward = prograde.Cross(normal).Normalized();
        return (deltaV.Dot(prograde), deltaV.Dot(normal), deltaV.Dot(outward));
    }

    private static Cr3bpState Derivative(Cr3bpState state)
    {
        double r1 = Math.Sqrt((state.X + MassRatio) * (state.X + MassRatio)
            + state.Y * state.Y + state.Z * state.Z);
        double r2 = Math.Sqrt((state.X - 1 + MassRatio) * (state.X - 1 + MassRatio)
            + state.Y * state.Y + state.Z * state.Z);
        double r13 = r1 * r1 * r1;
        double r23 = r2 * r2 * r2;
        return new Cr3bpState(
            state.Vx, state.Vy, state.Vz,
            2 * state.Vy + state.X
                - (1 - MassRatio) * (state.X + MassRatio) / r13
                - MassRatio * (state.X - 1 + MassRatio) / r23,
            -2 * state.Vx + state.Y
                - (1 - MassRatio) * state.Y / r13 - MassRatio * state.Y / r23,
            -(1 - MassRatio) * state.Z / r13 - MassRatio * state.Z / r23);
    }

    private readonly record struct Cr3bpState(
        double X, double Y, double Z, double Vx, double Vy, double Vz)
    {
        internal Cr3bpState Add(Cr3bpState other, double scale = 1) => new(
            X + other.X * scale, Y + other.Y * scale, Z + other.Z * scale,
            Vx + other.Vx * scale, Vy + other.Vy * scale, Vz + other.Vz * scale);

        internal Cr3bpState Scale(double scale) => new(
            X * scale, Y * scale, Z * scale,
            Vx * scale, Vy * scale, Vz * scale);
    }
}

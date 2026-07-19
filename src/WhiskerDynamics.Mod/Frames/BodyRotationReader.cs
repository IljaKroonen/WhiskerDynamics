using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Frames;

/// <summary>Activation-time reader of a live body's spin model — the body-surface
/// frame's only game seam. Reads the PUBLIC time-parameterized orientation surface:
/// <c>IParentBody.GetCcf2Cce(SimTime)</c> (IParentBody.cs:31 — Celestial.cs:560
/// implements the spin-then-tilt composition SurfaceFrameKernel documents;
/// StellarBody.cs:126 returns Identity) and <c>IParentBody.GetAngularVelocity()</c>
/// (IParentBody.cs:68 — Celestial.cs:196 returns the template spin rate,
/// StellarBody.cs:137 returns 0), then reconstructs the KSA-free
/// <see cref="BodyRotation"/> and VERIFIES the reconstruction against the game's own
/// quaternion at a second sample time (tRef + 1 h): any drift in the assumed-constant
/// tilt, a changed game formula, or a quaternion-convention surprise refuses activation
/// with a panel-ready reason instead of rendering a wrong frame. The SimTime-
/// parameterized overload is verified rather than the cached GetBodyFixed2Ecl()
/// (Celestial.cs:186-189): the cache is stamped at Orbit.StateVectors.StateTime
/// (Celestial.cs:589-590), which lags 'now' under warp — same composition, stale time.
/// The root star reads Identity/0, so a root Surface frame degenerates honestly to the
/// root-centred inertial pose. Every member touched is registered in GameplayTargets.
/// May throw (KSA-less process, game drift); the caller contains.</summary>
internal static class BodyRotationReader
{
    /// <summary>1 h: Earth-class spin sweeps ~0.26 rad, tidally locked moons ~1e-2 rad —
    /// both many orders above <see cref="SurfaceFrameKernel.VerifyTolerance"/>, so a
    /// wrong pole/rate/composition cannot pass, while an exact reconstruction sits at
    /// fp round-off. Comfortably inside the rails horizon and free of angle aliasing
    /// (no body spins anywhere near 2*pi per hour).</summary>
    private const double VerifyIntervalSeconds = 3600.0;

    /// <summary>Bind-time capture of every live body's fixed spin pole as plain core
    /// vectors. This main-thread-only seam feeds the overlay worker's
    /// equatorial AN/DN scan: the worker receives only the immutable dictionary and
    /// never touches <see cref="IParentBody"/>. Individual bad bodies are omitted and
    /// reported without costing the rest of the system its equatorial markers.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    internal static IReadOnlyDictionary<string, Vector3d> SnapshotPoles(
        out IReadOnlyDictionary<string, double> angularVelocities,
        out IReadOnlyList<string> diagnostics)
    {
        if (!BurnPlanWriter.IsMainThread)
            throw new InvalidOperationException("spin-pole snapshot requires the main thread");
        var system = Universe.CurrentSystem
            ?? throw new InvalidOperationException("no CurrentSystem loaded");
        var poles = new Dictionary<string, Vector3d>(StringComparer.Ordinal);
        var rates = new Dictionary<string, double>(StringComparer.Ordinal);
        var failures = new List<string>();
        int count = system.Count;
        for (int i = 0; i < count; i++)
        {
            if (system.GetIndex(i) is not Astronomical astronomical
                || astronomical is not IParentBody parentBody) continue;
            try
            {
                var q = parentBody.GetCcf2Cce(new SimTime(CatalogKernel.ReferenceEpochSeconds));
                var pole = FrameAdapter.ToCore(Brutal.Numerics.double3.Transform(
                    Brutal.Numerics.double3.UnitZ, q));
                double length = pole.Length();
                if (!double.IsFinite(length) || length == 0)
                {
                    failures.Add($"'{astronomical.Id}': invalid spin pole");
                    continue;
                }
                poles[astronomical.Id] = pole / length;
                double rate = parentBody.GetAngularVelocity();
                if (double.IsFinite(rate)) rates[astronomical.Id] = rate;
            }
            catch (Exception e)
            {
                failures.Add($"'{astronomical.Id}': {e.Message}");
            }
        }
        angularVelocities = rates;
        diagnostics = failures;
        return poles;
    }

    /// <summary>Null on success (model out is verified); otherwise a panel-ready
    /// refusal reason.</summary>
    // NoInlining: the KSA-less-process containment seam (FrameManager.Activate's
    // catch-all) must not depend on inliner policy — KSA tokens JIT only at THIS call
    // (the LiveCatalog.SnapshotCurrentSystem precedent).
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    internal static string? TryRead(string bodyId, double referenceTime, out BodyRotation model)
    {
        model = default;
        var system = Universe.CurrentSystem;
        if (system is null) return "no live system";
        IParentBody? body = null;
        int count = system.Count;
        for (int i = 0; i < count; i++)
        {
            // IParentBody covers exactly the celestial population (LiveCatalog reading):
            // Celestial and the root StellarBody; vehicles are never IParentBody.
            if (system.GetIndex(i) is Astronomical astronomical
                && astronomical is IParentBody parentBody
                && string.Equals(astronomical.Id, bodyId, StringComparison.Ordinal))
            {
                body = parentBody;
                break;
            }
        }
        if (body is null) return $"'{bodyId}' has no live rotation state";
        double angularVelocity = body.GetAngularVelocity();
        var qRef = body.GetCcf2Cce(new SimTime(referenceTime));
        model = SurfaceFrameKernel.ModelFromGameQuat(qRef, angularVelocity, referenceTime);
        // Tolerance gate first (FrameKernel.Surface throws on an EXACT zero pole only —
        // a degenerate live quaternion must refuse with a reason, not throw).
        if (FrameCatalog.ValidateRotation(model) is { } modelReason) return modelReason;
        // Arbitrary-t reconstruction check against the game's own formula at a second
        // time — the property every frame consumer (counter-pose at 'now', curve
        // re-embedding at sample times) relies on.
        double tCheck = referenceTime + VerifyIntervalSeconds;
        return SurfaceFrameKernel.VerifyReconstruction(
            model, body.GetCcf2Cce(new SimTime(tCheck)), tCheck);
    }
}

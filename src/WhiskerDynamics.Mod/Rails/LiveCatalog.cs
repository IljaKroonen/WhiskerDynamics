using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Rails;

/// <summary>Bind-time reader of the RUNNING game's celestial catalog (the live
/// <see cref="CelestialSystem"/>), replacing the Astronomicals.xml guess — the game's
/// objects are ground truth for whatever catalog the save loaded (SolSystem.xml,
/// SolSystemDense.xml, ...). DETERMINISM: only the DEFINING conic is read —
/// <c>Orbit.GetStateVectorsAt(SimTime)</c> is pure element math off the conic's
/// OrbitData (Orbit.cs:1966: time→mean→true anomaly, perifocal state, Orb2ParentCci),
/// never the propagated <c>Orbit.StateVectors</c> cache — evaluated at the fixed
/// reference epoch t = 0, so every rebind snapshots identical records and rails from
/// t = 0 stay bit-reproducible across sessions. Every game member touched here is
/// pinned in <see cref="WhiskerDynamics.Compatibility.Patching.GameplayTargets"/>.</summary>
internal static class LiveCatalog
{
    /// <summary>Snapshots the authoritative running system. Any failure aborts binding;
    /// production never substitutes a second catalog model.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    internal static IReadOnlyList<CatalogBody> SnapshotCurrentSystem(out string systemId)
    {
        var system = Universe.CurrentSystem
            ?? throw new InvalidOperationException("no CurrentSystem loaded");
        systemId = system.Id;
        return Snapshot(system);
    }

    /// <summary>Snapshots every celestial (vessels excluded) as a plain record. The
    /// root (Sol — a StellarBody, which is an
    /// Astronomical + IParentBody but NOT a Celestial and carries no Orbit) yields a
    /// null ParentId and null state; Celestials always carry an Orbit, and its Parent
    /// identifies theirs.</summary>
    internal static IReadOnlyList<CatalogBody> Snapshot(CelestialSystem system)
    {
        int count = system.Count;
        var catalog = new List<CatalogBody>(count);
        for (int i = 0; i < count; i++)
        {
            // IParentBody covers exactly the celestial population: Celestial (planets,
            // moons, comets) and StellarBody (the root). Vehicles are Astronomical but
            // never IParentBody.
            if (system.GetIndex(i) is not Astronomical astronomical
                || astronomical is not IParentBody parentBody) continue;

            string? parentId = null;
            Vector3d? relPositionEcl = null;
            Vector3d? relVelocityEcl = null;
            // The zonal field needs only the fixed spin pole.  Reading it from the
            // game's own body-fixed orientation avoids duplicating tilt conventions.
            var rotation = SurfaceFrameKernel.ModelFromGameQuat(
                parentBody.GetCcf2Cce(new SimTime(CatalogKernel.ReferenceEpochSeconds)),
                parentBody.GetAngularVelocity(), CatalogKernel.ReferenceEpochSeconds);
            if (astronomical is Celestial celestial
                && celestial.Orbit is { } orbit && orbit.Parent is { } parent)
            {
                parentId = parent.Id;
                // Defining conic at the fixed epoch; parent-relative Cci axes.
                var sv = orbit.GetStateVectorsAt(new SimTime(CatalogKernel.ReferenceEpochSeconds));
                // Cci -> ecliptic axes: the same registered conversion TrackedVessel.Reseed
                // uses (GetCci2Cce is a fixed composition of defining-conic orientations,
                // constant in time — root Cce axes ARE the mod's ecliptic axes).
                var cci2Cce = parent.GetCci2Cce();
                relPositionEcl = FrameAdapter.CciToEcl(sv.PositionCci, cci2Cce);
                relVelocityEcl = FrameAdapter.CciToEcl(sv.VelocityCci, cci2Cce);
            }

            catalog.Add(new CatalogBody(
                astronomical.Id, parentBody.Mass, parentId, astronomical.MeanRadius,
                relPositionEcl, relVelocityEcl, rotation, parentBody.SphereOfInfluence));
        }
        return catalog;
    }
}

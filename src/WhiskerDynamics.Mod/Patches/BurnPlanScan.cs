using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Shared burn-plan membership scan for the two draw-suppression patches:
/// <see cref="VesselLinePatch"/> asks "is this FlightPlan one of the vessel's planned
/// burns' plans?" and <see cref="PatchMarkerPatch"/> asks "is this PatchedConic the
/// patch at this index of one of them?". The enumeration scaffolding — BurnPlan
/// resolve, BurnCount loop, TryGetBurn null-guard (BurnPlan.cs:161), Burn.FlightPlan
/// read (Burn.cs:36) — lives once in the allocation-free struct enumerator below;
/// each public method reduces to its predicate. Every game member touched is an
/// existing registry entry.</summary>
internal static class BurnPlanScan
{
    /// <summary>True when <paramref name="plan"/> is the flight plan of one of the
    /// vessel's planned burns — the post-burn conic predictions both patched draw
    /// methods route through (BurnPlan.cs:369-382 for lines, BurnPlan.cs:468 for
    /// markers).</summary>
    public static bool ContainsPlan(Vehicle vehicle, FlightPlan plan)
    {
        foreach (var planned in new PlannedPlans(vehicle))
            if (ReferenceEquals(planned, plan)) return true;
        return false;
    }

    /// <summary>True when <paramref name="patch"/> is the patch at
    /// <paramref name="index"/> of one of the vessel's planned burns' flight plans
    /// (identity-at-index, see <see cref="PatchMarkerPatch.IsPatchOfPlanAt"/>).</summary>
    public static bool ContainsPatchAt(Vehicle vehicle, PatchedConic patch, int index)
    {
        foreach (var planned in new PlannedPlans(vehicle))
            if (PatchMarkerPatch.IsPatchOfPlanAt(patch, index, planned)) return true;
        return false;
    }

    /// <summary>True when <paramref name="patch"/> is ANY patch of one of the vessel's
    /// planned burns' flight plans (identity, index-free — the hover router has only
    /// the patch instance). Small lists: burns × patches-per-plan.</summary>
    public static bool ContainsPatch(Vehicle vehicle, PatchedConic patch)
    {
        foreach (var planned in new PlannedPlans(vehicle))
        {
            var patches = planned.Patches;
            for (int i = 0; i < patches.Count; i++)
                if (ReferenceEquals(patches[i], patch)) return true;
        }
        return false;
    }

    /// <summary>True when <paramref name="patch"/> is patch 0 of one of the vessel's
    /// burn plans. Two-line display: EVERY burn plan's first patch lies along the one
    /// drawn PLANNED line (burn k's pre-burn trajectory with earlier burns applied is
    /// the planned path at its time — impulses are position-continuous), so the hover
    /// substitute serves them all; later patches are suppressed post-burn SOI conics.</summary>
    public static bool IsFirstPatchOfAnyBurnPlan(Vehicle vehicle, PatchedConic patch)
    {
        foreach (var planned in new PlannedPlans(vehicle))
        {
            var patches = planned.Patches;
            if (patches.Count > 0 && ReferenceEquals(patches[0], patch)) return true;
        }
        return false;
    }

    /// <summary>The vessel's earliest planned burn by time, or null when it has none —
    /// the owner of the two-line display's PLANNED-line canvas (the planned batch is
    /// sampled from the first in-window burn and staged into this burn's plan orbit).
    /// A min-scan, not _burns[0]: stock sorts the list, but this must hold even
    /// mid-mutation.</summary>
    public static Burn? EarliestBurn(Vehicle vehicle)
    {
        var burnPlan = vehicle.FlightComputer.BurnPlan;
        Burn? earliest = null;
        for (int i = 0; i < burnPlan.BurnCount; i++)
        {
            if (!burnPlan.TryGetBurn(i, out Burn? burn) || burn is null) continue;
            if (earliest is null || burn.Time.Seconds() < earliest.Time.Seconds()) earliest = burn;
        }
        return earliest;
    }

    /// <summary>Allocation-free enumeration of the vessel's planned burns' flight
    /// plans (foreach binds to the struct enumerator pattern — no IEnumerable
    /// boxing on these per-frame draw paths).</summary>
    private readonly struct PlannedPlans
    {
        private readonly Vehicle _vehicle;
        public PlannedPlans(Vehicle vehicle) => _vehicle = vehicle;
        public Enumerator GetEnumerator() => new(_vehicle.FlightComputer.BurnPlan);

        internal struct Enumerator
        {
            private readonly BurnPlan _burnPlan;
            private int _next;
            private FlightPlan? _current;

            public Enumerator(BurnPlan burnPlan)
            {
                _burnPlan = burnPlan;
                _next = 0;
                _current = null;
            }

            public FlightPlan Current => _current!;

            public bool MoveNext()
            {
                while (_next < _burnPlan.BurnCount)
                {
                    if (!_burnPlan.TryGetBurn(_next++, out Burn? burn) || burn is null) continue;
                    _current = burn.FlightPlan;
                    return true;
                }
                return false;
            }
        }
    }
}

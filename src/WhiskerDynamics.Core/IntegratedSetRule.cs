namespace WhiskerDynamics.Core;

/// <summary>Why a valid modeled body was classified as a restricted trajectory
/// instead of a member of the mutually coupled backbone.</summary>
public enum RestrictedClassificationKind { NonBackreacting, Ancestor }

public sealed record RestrictedClassification(
    string Id, string Reason, RestrictedClassificationKind Kind);

/// <summary>Selects the mutually coupled positive-mass backbone. Bodies outside the returned
/// set remain modeled as independently stepped restricted n-body trajectories; they
/// never revert to a prescribed conic. Invalid catalogs or seeds reject selection.</summary>
public static class IntegratedSetRule
{
    public static HashSet<string> Select(
        IReadOnlyList<CelestialBody> bodies, double startTime,
        out IReadOnlyList<RestrictedClassification> restrictedClassifications)
    {
        ArgumentNullException.ThrowIfNull(bodies);
        if (!double.IsFinite(startTime))
            throw new ArgumentOutOfRangeException(nameof(startTime),
                "Ephemeris start time must be finite.");
        ValidateCatalogAndSeeds(bodies, startTime);
        var graph = ParentGraphAnalyzer.AnalyzeBodies(bodies, out int[] bodyIndices);
        var ordered = bodies.Select((body, index) =>
                (Body: body, Depth: graph.Depths[bodyIndices[index]], Index: index))
            .OrderBy(entry => entry.Depth)
            .ThenBy(entry => entry.Index)
            .Select(entry => entry.Body)
            .ToArray();

        var candidates = new HashSet<string>(StringComparer.Ordinal);
        var classified = new List<RestrictedClassification>();
        foreach (var body in ordered)
        {
            if (body.Mu == 0)
            {
                classified.Add(new RestrictedClassification(body.Id,
                    "restricted because a zero-mu body has no backreaction",
                    RestrictedClassificationKind.NonBackreacting));
                continue;
            }
            if (body.Parent is null)
            {
                candidates.Add(body.Id);
                continue;
            }
            if (!candidates.Contains(body.Parent.Id))
            {
                classified.Add(new RestrictedClassification(body.Id,
                    $"restricted because ancestor '{body.Parent.Id}' is outside the mutual backbone",
                    RestrictedClassificationKind.Ancestor));
                continue;
            }
            candidates.Add(body.Id);
        }

        restrictedClassifications = classified;
        return candidates;
    }

    private static void ValidateCatalogAndSeeds(
        IReadOnlyList<CelestialBody> bodies, double startTime)
    {
        var duplicateIds = bodies.GroupBy(body => body.Id, StringComparer.Ordinal)
            .Where(group => group.Count() > 1).Select(group => group.Key).ToArray();
        if (duplicateIds.Length > 0)
            throw new ArgumentException("Modeled body ids must be unique: "
                + string.Join(", ", duplicateIds), nameof(bodies));
        var graph = ParentGraphAnalyzer.AnalyzeBodies(bodies, out _);
        if (graph.Cycles.Length > 0)
            throw new ArgumentException("Parent cycle detected: "
                + graph.FormatCycles() + ".", nameof(bodies));
        var roots = bodies.Where(body => body.Parent is null).ToArray();
        if (roots.Length != 1)
            throw new ArgumentException(
                $"The celestial system must contain exactly one root body; found {roots.Length}.",
                nameof(bodies));
        var bodySet = new HashSet<CelestialBody>(bodies, ReferenceEqualityComparer.Instance);
        var missingParents = bodies
            .Where(body => body.Parent is not null && !bodySet.Contains(body.Parent))
            .Select(body => $"{body.Id}->{body.Parent!.Id}").ToArray();
        if (missingParents.Length > 0)
            throw new ArgumentException("Every modeled parent must belong to the catalog: "
                + string.Join(", ", missingParents), nameof(bodies));
        var invalidMasses = bodies.Where(body => !double.IsFinite(body.Mu) || body.Mu < 0)
            .Select(body => body.Id).ToArray();
        if (invalidMasses.Length > 0)
            throw new ArgumentException("Modeled bodies require finite nonnegative mu: "
                + string.Join(", ", invalidMasses), nameof(bodies));

        ValidateSeeds(bodies, startTime);
    }

    private static void ValidateSeeds(IReadOnlyList<CelestialBody> bodies, double startTime)
    {
        var seedEphemerides = new Ephemerides(bodies);
        foreach (var body in bodies)
        {
            if (body.Parent is not null && body.Orbit is null)
                throw new ArgumentException(
                    $"Body '{body.Id}' has no defining conic for its numerical seed.",
                    nameof(bodies));
            StateVector seed;
            try { seed = seedEphemerides.GetState(body, startTime); }
            catch (Exception error) when (error is InvalidOperationException
                or NotSupportedException or NullReferenceException)
            {
                throw new ArgumentException(
                    $"Body '{body.Id}' has no valid n-body seed: {error.Message}",
                    nameof(bodies), error);
            }
            if (!Finite(seed.Position) || !Finite(seed.Velocity))
                throw new ArgumentException(
                    $"Body '{body.Id}' has a non-finite n-body seed.", nameof(bodies));
        }
    }

    private static bool Finite(in Vector3d value) =>
        double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);
}

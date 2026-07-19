using System.Buffers;

namespace WhiskerDynamics.Core;

/// <summary>Evaluates celestial body states at any time by walking parent chains.
/// Frame: root body (Sol) fixed at the origin, axes = the game's ecliptic frame.</summary>
public sealed class Ephemerides : IEphemerides
{
    private const int InlineStepCapacity = 8;
    private static readonly StateVector RootState =
        new(Vector3d.Zero, Vector3d.Zero);

    /// <summary>An unmanaged snapshot of one child-to-parent Kepler edge. Orbit stays
    /// nullable until the root-to-child evaluation pass so malformed shallow orbits
    /// are reported before deeper ones.</summary>
    private readonly record struct KeplerStep(OrbitalElements? Orbit, double ParentMu);

    private readonly Dictionary<string, CelestialBody> _byId;

    public Ephemerides(IReadOnlyList<CelestialBody> bodies)
    {
        // Generic ephemerides deliberately permits an empty set or a multi-root
        // forest, but no parent cycle: recursive state composition could otherwise
        // overflow before a caller receives a diagnostic. Include external ancestors
        // in the analysis to preserve the class's existing forest semantics.
        var graph = ParentGraphAnalyzer.AnalyzeBodies(bodies, out _);
        if (graph.Cycles.Length > 0)
            throw new ArgumentException("Parent cycle detected: "
                + graph.FormatCycles() + ".", nameof(bodies));
        Bodies = bodies;
        _byId = bodies.ToDictionary(b => b.Id);
    }

    public IReadOnlyList<CelestialBody> Bodies { get; }

    public CelestialBody this[string id] => _byId[id];

    public StateVector GetState(CelestialBody body, double time)
    {
        // Allocation-free fast paths for the real catalog's root, planets, and moons.
        // Keep the explicit RootState addition for consistent signed-zero and
        // floating-point operation order across depths.
        if (body.Parent is not { } parent) return RootState;
        if (parent.Parent is not { } grandparent)
            return RootState
                + Kepler.StateFromElements(body.Orbit!.Value, parent.Mu, time);
        if (grandparent.Parent is null)
        {
            var parentLocal =
                Kepler.StateFromElements(parent.Orbit!.Value, grandparent.Mu, time);
            var local = Kepler.StateFromElements(body.Orbit!.Value, parent.Mu, time);
            return (RootState + parentLocal) + local;
        }

        return GetDeepState(body, time);
    }

    /// <summary>Iterative deep-chain evaluation. The upward pass snapshots nullable
    /// orbit inputs while Floyd's reference-identity walk proves acyclicity; only then
    /// does the downward pass evaluate and add root-to-child, preserving both Kepler
    /// exception order and every floating-point association of the recursive code.</summary>
    private static StateVector GetDeepState(CelestialBody body, double time)
    {
        Span<KeplerStep> steps = stackalloc KeplerStep[InlineStepCapacity];
        KeplerStep[]? rented = null;
        int count = 0;
        CelestialBody current = body;
        CelestialBody? fast = body;
        try
        {
            while (current.Parent is { } parent)
            {
                if (count == steps.Length)
                {
                    var replacement = ArrayPool<KeplerStep>.Shared.Rent(
                        checked(steps.Length * 2));
                    steps.CopyTo(replacement);
                    if (rented is not null)
                        ArrayPool<KeplerStep>.Shared.Return(rented);
                    rented = replacement;
                    steps = replacement;
                }
                steps[count++] = new KeplerStep(current.Orbit, parent.Mu);
                current = parent;

                fast = fast?.Parent?.Parent;
                if (fast is not null && ReferenceEquals(current, fast))
                    ThrowParentCycle(body);
            }

            var state = RootState;
            for (int i = count - 1; i >= 0; i--)
            {
                ref readonly var step = ref steps[i];
                state += Kepler.StateFromElements(
                    step.Orbit!.Value, step.ParentMu, time);
            }
            return state;
        }
        finally
        {
            // KeplerStep is unmanaged, so pooled slots retain no object graph.
            if (rented is not null) ArrayPool<KeplerStep>.Shared.Return(rented);
        }
    }

    [System.Diagnostics.CodeAnalysis.DoesNotReturn]
    private static void ThrowParentCycle(CelestialBody body)
    {
        // Error-only reconstruction reuses the canonical formatter. The steady-state
        // path pays only Floyd's reference comparisons and allocates nothing.
        var graph = ParentGraphAnalyzer.AnalyzeBodies([body], out _);
        string closure = graph.Cycles.Length == 0
            ? $"'{body.Id}'"
            : graph.FormatCycles();
        throw new InvalidOperationException(
            $"Parent cycle detected while evaluating state for '{body.Id}': {closure}.");
    }
}

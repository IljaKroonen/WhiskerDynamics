namespace WhiskerDynamics.Core;

/// <summary>One body from a live-system snapshot, without game-specific types.
/// The state is the body's defining conic evaluated at
/// <see cref="CatalogKernel.ReferenceEpochSeconds"/>, parent-relative, ecliptic axes,
/// SI units; null state for the root (and only the root has a null ParentId).</summary>
public sealed record CatalogBody(
    string Id,
    double MassKg,
    string? ParentId,
    double MeanRadiusM,
    Vector3d? RelPositionEcl,
    Vector3d? RelVelocityEcl,
    BodyRotation? Rotation = null,
    double SphereOfInfluenceM = double.NaN);

/// <summary>Converts a live-system catalog snapshot into a linked celestial graph,
/// deriving gravitational parameters and orbital elements.</summary>
public static class CatalogKernel
{
    /// <summary>The catalog states are the defining conics evaluated at t = 0 game
    /// seconds (the game epoch the rails anchor at), so the derived TimeAtPeriapsis
    /// lands on the same clock the ephemerides integrate on. A fixed epoch keeps the
    /// snapshot bind-time-invariant: rebinding at any sim time reads the same conic.</summary>
    public const double ReferenceEpochSeconds = 0.0;

    /// <summary>Below this eccentricity <see cref="Kepler.ElementsFromState"/> refuses
    /// the state (periapsis direction ill-defined); the kernel switches to the circular
    /// convention instead. Matches the guard inside ElementsFromState.</summary>
    private const double NearCircularEccentricity = 1e-8;

    /// <summary>Builds the celestial graph. Invalid individual bodies are skipped and
    /// reported through <paramref name="diagnostics"/>. Duplicate ids or an invalid root
    /// count throw <see cref="FormatException"/>. Output preserves catalog order.</summary>
    public static IReadOnlyList<CelestialBody> Build(
        IReadOnlyList<CatalogBody> catalog, double gravitationalConstant,
        out IReadOnlyList<string> diagnostics) =>
        Build(catalog, gravitationalConstant, out diagnostics, LunarGravityFidelity.Degree50);

    public static IReadOnlyList<CelestialBody> Build(
        IReadOnlyList<CatalogBody> catalog, double gravitationalConstant,
        out IReadOnlyList<string> diagnostics, LunarGravityFidelity lunarGravityFidelity)
    {
        var diags = new List<string>();
        diagnostics = diags;

        var seen = new HashSet<string>();
        foreach (var body in catalog)
            if (!seen.Add(body.Id))
                throw new FormatException($"duplicate body id '{body.Id}' in the catalog");

        var roots = catalog.Where(b => b.ParentId is null).ToList();
        if (roots.Count != 1)
            throw new FormatException(
                $"expected exactly one parentless body, found {roots.Count}"
                + (roots.Count == 0 ? "" : $" ({string.Join(", ", roots.Select(b => b.Id))})"));

        var root = roots[0];
        if (!double.IsFinite(root.MassKg) || root.MassKg <= 0)
            throw new FormatException(
                $"root body '{root.Id}' has a non-positive or non-finite mass ({root.MassKg} kg)");
        double rootMu = gravitationalConstant * root.MassKg;
        if (!double.IsFinite(rootMu) || rootMu <= 0)
            throw new FormatException(
                $"root body '{root.Id}' has a non-positive or non-finite derived mu ({rootMu})");
        var built = new Dictionary<string, CelestialBody>
        {
            [root.Id] = new CelestialBody
            {
                Id = root.Id,
                Mu = rootMu,
                MeanRadius = root.MeanRadiusM,
                SphereOfInfluence = root.SphereOfInfluenceM,
                Geopotential = KnownGeopotentials.ForBody(
                    root.Id, root.Rotation, lunarGravityFidelity),
            },
        };

        // Parent-before-child conversion regardless of catalog order: walk the id tree
        // from the root. Bodies never reached (parent not in the catalog, or an ancestor
        // skipped, or a parent cycle) are diagnosed afterwards.
        var childrenOf = catalog.Where(b => b.ParentId is not null).ToLookup(b => b.ParentId!);
        var pending = new Queue<string>();
        pending.Enqueue(root.Id);
        var failed = new HashSet<string>(); // conversion failures, already diagnosed
        while (pending.TryDequeue(out var parentId))
        {
            var parent = built[parentId];
            foreach (var child in childrenOf[parentId])
            {
                if (TryConvert(child, parent, gravitationalConstant, diags,
                    lunarGravityFidelity) is { } body)
                {
                    built[child.Id] = body;
                    pending.Enqueue(child.Id);
                }
                else
                {
                    failed.Add(child.Id);
                }
            }
        }

        foreach (var body in catalog)
        {
            if (built.ContainsKey(body.Id) || failed.Contains(body.Id)) continue;
            diags.Add(seen.Contains(body.ParentId!)
                ? $"body '{body.Id}' skipped: its parent '{body.ParentId}' was skipped"
                : $"body '{body.Id}' skipped: unknown parent '{body.ParentId}'");
        }

        return catalog.Where(b => built.ContainsKey(b.Id)).Select(b => built[b.Id]).ToList();
    }

    private static CelestialBody? TryConvert(
        CatalogBody body, CelestialBody parent, double gravitationalConstant, List<string> diags,
        LunarGravityFidelity lunarGravityFidelity)
    {
        if (body.RelPositionEcl is not { } position || body.RelVelocityEcl is not { } velocity)
        {
            diags.Add($"body '{body.Id}' skipped: no defining-conic state");
            return null;
        }
        if (!double.IsFinite(body.MassKg))
        {
            diags.Add($"body '{body.Id}' skipped: non-finite mass ({body.MassKg} kg)");
            return null;
        }
        // Zero-mass catalog entries are supported test particles. Negative zero is
        // deliberately accepted as the same exact restricted-body limit.
        if (body.MassKg < 0)
        {
            diags.Add($"body '{body.Id}' skipped: negative mass ({body.MassKg} kg)");
            return null;
        }
        double bodyMu = gravitationalConstant * body.MassKg;
        if (!double.IsFinite(bodyMu) || bodyMu < 0
            || (body.MassKg != 0 && bodyMu == 0))
        {
            diags.Add($"body '{body.Id}' skipped: non-finite, negative, or underflowed gravitational "
                + $"parameter derived from mass (mu={bodyMu})");
            return null;
        }
        if (!double.IsFinite(parent.Mu) || parent.Mu <= 0)
        {
            diags.Add($"body '{body.Id}' skipped: parent mu must be finite and positive ({parent.Mu})");
            return null;
        }
        if (!IsFinite(position) || !IsFinite(velocity))
        {
            diags.Add($"body '{body.Id}' skipped: non-finite defining-conic state");
            return null;
        }

        double radius = Magnitude(position);
        if (!(radius > 0) || !double.IsFinite(radius))
        {
            diags.Add($"body '{body.Id}' skipped: degenerate defining-conic radius ({radius})");
            return null;
        }
        double speed = Magnitude(velocity);
        if (!double.IsFinite(speed))
        {
            diags.Add($"body '{body.Id}' skipped: non-finite defining-conic speed ({speed})");
            return null;
        }
        var angularMomentum = position.Cross(velocity);
        double angularMomentumMagnitude = Magnitude(angularMomentum);
        if (!(angularMomentumMagnitude > 0) || !double.IsFinite(angularMomentumMagnitude))
        {
            diags.Add($"body '{body.Id}' skipped: degenerate defining-conic angular momentum "
                + $"({angularMomentumMagnitude})");
            return null;
        }
        double kineticEnergy = 0.5 * speed * speed;
        double potentialMagnitude = parent.Mu / radius;
        if (!double.IsFinite(kineticEnergy) || !double.IsFinite(potentialMagnitude))
        {
            diags.Add($"body '{body.Id}' skipped: non-finite specific orbital energy");
            return null;
        }

        var state = new StateVector(position, velocity);
        OrbitalElements elements;
        try
        {
            elements = ElementsFor(state, parent.Mu);
        }
        catch (NotSupportedException e)
        {
            diags.Add($"body '{body.Id}' skipped: {e.Message}");
            return null;
        }

        if (!HasFiniteElements(elements))
        {
            diags.Add($"body '{body.Id}' skipped: non-finite derived orbital elements");
            return null;
        }
        if (!HasPhysicallyConsistentElements(elements, parent.Mu))
        {
            diags.Add($"body '{body.Id}' skipped: physically inconsistent derived orbital "
                + $"elements (a={elements.SemiMajorAxis:G6}, e={elements.Eccentricity:G6})");
            return null;
        }
        StateVector evaluated;
        try
        {
            evaluated = Kepler.StateFromElements(elements, parent.Mu, ReferenceEpochSeconds);
        }
        catch (Exception e) when (
            e is NotSupportedException or InvalidOperationException or ArithmeticException)
        {
            diags.Add($"body '{body.Id}' skipped: derived orbit is not evaluable: {e.Message}");
            return null;
        }
        if (!IsFinite(evaluated.Position) || !IsFinite(evaluated.Velocity))
        {
            diags.Add($"body '{body.Id}' skipped: derived orbit evaluated to a non-finite state");
            return null;
        }

        return new CelestialBody
        {
            Id = body.Id,
            Mu = bodyMu,
            MeanRadius = body.MeanRadiusM,
            SphereOfInfluence = body.SphereOfInfluenceM,
            Geopotential = KnownGeopotentials.ForBody(
                body.Id, body.Rotation, lunarGravityFidelity),
            Parent = parent,
            Orbit = elements,
        };
    }

    /// <summary>Derives elements, using a circular convention when the periapsis
    /// direction is numerically undefined: e = 0, periapsis at the reference-epoch
    /// position, and TimeAtPeriapsis at the reference epoch.</summary>
    private static OrbitalElements ElementsFor(in StateVector state, double mu)
    {
        var r = state.Position;
        var v = state.Velocity;
        double rMag = Magnitude(r);

        var h = r.Cross(v);
        var eVec = v.Cross(h) / mu - r / rMag;
        if (Magnitude(eVec) >= NearCircularEccentricity)
            return Kepler.ElementsFromState(state, mu, ReferenceEpochSeconds);

        // A circular conic must pass through the defining state at the reference
        // epoch. The energy-derived semi-major axis differs from the current radius
        // by O(ae) for a merely near-circular input, producing an avoidable absolute
        // position jump when the catalog state is rebuilt as e = 0.
        double a = rMag;
        double hMagnitude = Magnitude(h);
        double inc = Math.Acos(Math.Clamp(h.Z / hMagnitude, -1, 1));
        var nodeVec = new Vector3d(-h.Y, h.X, 0); // ẑ × h
        double nodeMagnitude = Magnitude(nodeVec);
        double raan;
        Vector3d nodeDir;
        if (nodeMagnitude < 1e-12 * hMagnitude)
        {
            raan = 0; // equatorial: node undefined, measure from +X
            nodeDir = new Vector3d(1, 0, 0);
        }
        else
        {
            raan = Math.Atan2(nodeVec.Y, nodeVec.X);
            nodeDir = nodeVec / nodeMagnitude;
        }
        // In-plane angle from the node line to the position, signed around h.
        var hDir = h / hMagnitude;
        double argPe = Math.Atan2(nodeDir.Cross(r / rMag).Dot(hDir), nodeDir.Dot(r / rMag));
        return new OrbitalElements(a, 0, inc, NormalizeAngle(raan), NormalizeAngle(argPe),
            ReferenceEpochSeconds);
    }

    private static double NormalizeAngle(double angle)
    {
        double r = angle % (2 * Math.PI);
        return r < 0 ? r + 2 * Math.PI : r;
    }

    private static bool IsFinite(in Vector3d v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);

    private static bool HasFiniteElements(in OrbitalElements elements) =>
        double.IsFinite(elements.SemiMajorAxis)
        && double.IsFinite(elements.Eccentricity)
        && double.IsFinite(elements.Inclination)
        && double.IsFinite(elements.LongitudeOfAscendingNode)
        && double.IsFinite(elements.ArgumentOfPeriapsis)
        && double.IsFinite(elements.TimeAtPeriapsis);

    private static bool HasPhysicallyConsistentElements(
        in OrbitalElements elements, double parentMu)
    {
        double a = elements.SemiMajorAxis;
        double e = elements.Eccentricity;
        double node = elements.LongitudeOfAscendingNode;
        double periapsisArgument = elements.ArgumentOfPeriapsis;
        double fullTurn = 2 * Math.PI;
        if (e < 0
            || elements.Inclination < 0 || elements.Inclination > Math.PI
            || node < 0 || node >= fullTurn
            || periapsisArgument < 0 || periapsisArgument >= fullTurn)
            return false;
        // A finite-a parabola has no representation in OrbitalElements. These are
        // the only two supported physical branches, including e = 0 circles.
        if (!((e < 1 && a > 0) || (e > 1 && a < 0))) return false;

        double periapsisDistance = Kepler.PeriapsisDistance(elements);
        if (!(periapsisDistance > 0) || !double.IsFinite(periapsisDistance)) return false;
        double periapsisSpeed = Kepler.PeriapsisSpeed(elements, parentMu);
        return periapsisSpeed > 0 && double.IsFinite(periapsisSpeed);
    }

    /// <summary>Euclidean magnitude without squaring the vector's dimensional scale.
    /// This distinguishes exact zero from a representable subnormal vector and avoids
    /// overflowing merely because a finite component is larger than sqrt(double.MaxValue).</summary>
    private static double Magnitude(in Vector3d v)
    {
        double scale = Math.Max(Math.Abs(v.X), Math.Max(Math.Abs(v.Y), Math.Abs(v.Z)));
        if (scale == 0 || !double.IsFinite(scale)) return scale;
        double x = v.X / scale, y = v.Y / scale, z = v.Z / scale;
        return scale * Math.Sqrt(x * x + y * y + z * z);
    }
}

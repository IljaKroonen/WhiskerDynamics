using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Frames;

/// <summary>The supported display-frame taxonomy (barycentric Lagrange-centred frames
/// are not yet supported): body-centred inertial,
/// two-body fixed (origin on the primary, BOTH bodies pinned — mathematically
/// FrameKernel.Rotating, which is rotating-pulsating:
/// frame coordinates are normalized by the pair's instantaneous separation and
/// re-dimensionalized at the current separation, so an eccentric secondary is literally
/// fixed instead of sliding radially; distances near the pair breathe with the true
/// separation. Presented as a transfer-planning "X-Y Fixed" frame, NOT
/// "rotating/synodic"), body surface (axes spin with the body), and target-fixed (the
/// target vessel substitutes for the secondary in a two-body fixed frame). Persisted by
/// enum NAME in the sidecar for each vessel's display selection, using the same stable
/// identity convention as burn-authoring metadata. Persisted names are a stable
/// serialization contract.</summary>
public enum FrameKind { Inertial, TwoBodyFixed, Surface, TargetFixed }

public sealed record FrameSpec(FrameKind Kind, string PrimaryId, string? SecondaryId)
{
    /// <summary>Human-readable display-frame label, also used by the map caches as a
    /// session-local staleness key (CelestialCurves/VesselLinePatch/TrajectoryOverlay).
    /// It is never persisted, so the wording may evolve.</summary>
    public string Label => Kind switch
    {
        FrameKind.Inertial => $"{PrimaryId}-Centred Inertial",
        FrameKind.TwoBodyFixed => $"{PrimaryId}-{SecondaryId} Fixed",
        FrameKind.Surface => $"{PrimaryId} Surface",
        FrameKind.TargetFixed => $"{PrimaryId}-Target Fixed ({SecondaryId})",
        // Exhaustive on purpose: Label doubles as the map caches' staleness key, so an
        // out-of-range kind (the enum doc promises future members) must fail loudly
        // here — contained by the panels — rather than silently collide with the
        // genuine Surface frame's key and let wrong-frame curves pass modeMatches.
        _ => throw new ArgumentOutOfRangeException(nameof(Kind), Kind, "unknown FrameKind"),
    };
}

/// <summary>The one session-live synthetic row injected into the body hierarchy when
/// the controlled vessel targets another tracked vessel. Its spec fixes the target as
/// the secondary of its current parent body, exactly like a body-body fixed frame.</summary>
public sealed record TargetFrameCandidate(string VesselId, string ParentBodyId)
{
    public FrameSpec Spec { get; } = new(FrameKind.TargetFixed, ParentBodyId, VesselId);
}

internal enum FramePoseQuery { CurrentDisplay, CurveSample }

internal readonly record struct ActiveFrameSnapshot(
    FrameSpec Spec, FramePose ActivationPose, double ActivationTime,
    BodyRotation SurfaceRotation, long Generation);

internal static class FramePoseFailurePolicy
{
    internal static void OnFailure(FramePoseQuery query, Action retireCurrent)
    {
        if (query == FramePoseQuery.CurrentDisplay) retireCurrent();
    }
}

internal static class FrameActivationKernel
{
    internal static bool IsCurrent(
        FrameSpec? active, long generation, ActiveFrameSnapshot snapshot) =>
        generation == snapshot.Generation && active == snapshot.Spec;

    internal static bool TryDeactivate(
        ref FrameSpec? active, ref long generation, ActiveFrameSnapshot snapshot)
    {
        if (!IsCurrent(active, generation, snapshot)) return false;
        active = null;
        generation++;
        return true;
    }
}

/// <summary>Pure catalog/validation rules (KSA-free, offline-tested).</summary>
public static class FrameCatalog
{
    private const double BasisTolerance = 1e-6;

    /// <summary>Whether a session-local frame choice may remain active/restorable under
    /// the currently selected target. Catalog frames are independent of targeting;
    /// target-fixed frames exist only while their exact synthetic candidate exists.</summary>
    public static bool TargetChoiceIsCurrent(FrameSpec spec, FrameSpec? currentTarget) =>
        spec.Kind != FrameKind.TargetFixed || spec == currentTarget;

    /// <summary>Whether a target-fixed choice is no longer valid. An unknown
    /// target state (fresh registry still populating after load) is transient and must
    /// not erase a persisted preference.</summary>
    public static bool TargetChoiceShouldRetire(
        FrameSpec spec, bool targetStateKnown, FrameSpec? currentTarget) =>
        targetStateKnown && !TargetChoiceIsCurrent(spec, currentTarget);

    /// <summary>Floor on sin(angle between separation and relative velocity) —
    /// |r x v| / (|r| |v|) — below which a two-body fixed frame is refused. Non-dimensional,
    /// so scale-free. Rationale: cross-product cancellation noise is ~1e-16·|r||v|, so
    /// at 1e-6 the z-axis direction still carries <=~1e-10 relative error (far below
    /// display precision), while no BOUND pair ever approaches the floor (|r x v| = h
    /// is an orbit constant; even e~0.995 keeps sin(theta) above ~5e-2).
    /// CONJUNCTION HANDOFF: h-conservation bounds this gate away
    /// from firing only for BOUND pairs — a generic planet-planet pair's relative h is
    /// NOT conserved, and near a near-coplanar conjunction crossing it can fall through
    /// the floor while the frame z-axis swings ~180 deg rapidly. The frame may then
    /// legitimately self-deactivate mid-warp (TrySamplePose's gate) with a throttled
    /// WARN — swept by design, never silent. The camera counter-pose renders the swing
    /// as a fast map rotation (expected physics, not a bug); a generic planet-planet
    /// pair (e.g. Sol-Jupiter) may exhibit it.</summary>
    public const double MinRotationSine = 1e-6;

    /// <summary>Null when the spec is valid over the given modeled-body set;
    /// otherwise a panel-ready rejection reason.</summary>
    public static string? ValidateSpec(FrameSpec spec, IReadOnlyCollection<string> modeledIds)
    {
        if (!modeledIds.Contains(spec.PrimaryId))
            return $"'{spec.PrimaryId}' is not a modeled body";
        switch (spec.Kind)
        {
            case FrameKind.Inertial:
                return spec.SecondaryId is null ? null : "inertial frames take one body";
            case FrameKind.Surface:
                return spec.SecondaryId is null ? null : "surface frames take one body";
            case FrameKind.TwoBodyFixed:
                if (spec.SecondaryId is null) return "two-body fixed frames need a reference body";
                if (!modeledIds.Contains(spec.SecondaryId))
                    return $"'{spec.SecondaryId}' is not a modeled body";
                if (spec.SecondaryId == spec.PrimaryId) return "primary and reference must differ";
                return null;
            case FrameKind.TargetFixed:
                if (spec.SecondaryId is null) return "target-fixed frames need a target vessel";
                return spec.SecondaryId == spec.PrimaryId
                    ? "primary body and target vessel must differ"
                    : null;
            default:
                // Refuse, never fall through to a real kind's rules: every gated path
                // (Activate/SampleSpecPose) funnels through here, so an out-of-range
                // kind — e.g. a corrupted sidecar's numeric enum string — fails
                // identically everywhere with an honest reason.
                return $"unknown frame kind '{spec.Kind}'";
        }
    }

    /// <summary>Surface-frame twin of <see cref="ValidateGeometry"/>: the activation
    /// gate on a live-read spin model. FrameKernel.Surface throws on an EXACT zero pole
    /// only, so live consumers gate the basis here: null when the model is a finite,
    /// right-handed orthonormal basis with a finite rate; otherwise a panel-ready
    /// rejection reason. Tolerance 1e-6 sits far above quaternion round-off (~1e-15)
    /// and far below any real conditioning cliff.</summary>
    public static string? ValidateRotation(BodyRotation rotation)
    {
        if (!IsFinite(rotation.PoleEcl)
            || !IsFinite(rotation.XAxisEcl)
            || !IsFinite(rotation.YAxisEcl))
            return "degenerate spin: basis contains non-finite components";
        if (!double.IsFinite(rotation.AngularVelocity))
            return "degenerate spin: angular velocity is not finite";
        if (!double.IsFinite(rotation.ReferenceTime))
            return "degenerate spin: reference time is not finite";
        double poleLength = rotation.PoleEcl.Length();
        double xLength = rotation.XAxisEcl.Length();
        double yLength = rotation.YAxisEcl.Length();
        if (!double.IsFinite(poleLength)
            || !double.IsFinite(xLength)
            || !double.IsFinite(yLength))
            return "degenerate spin: basis axis length is not finite";
        double maximumLengthError = Math.Max(Math.Abs(poleLength - 1),
            Math.Max(Math.Abs(xLength - 1), Math.Abs(yLength - 1)));
        if (!double.IsFinite(maximumLengthError))
            return "degenerate spin: basis length error is not finite";
        if (maximumLengthError > BasisTolerance)
            return "degenerate spin: non-unit basis axes";
        var handednessResidual = rotation.XAxisEcl.Cross(rotation.YAxisEcl) - rotation.PoleEcl;
        if (!IsFinite(handednessResidual))
            return "degenerate spin: handedness residual is not finite";
        double handednessError = handednessResidual.Length();
        if (!double.IsFinite(handednessError))
            return "degenerate spin: handedness error is not finite";
        if (handednessError > BasisTolerance)
            return "degenerate spin: basis is not right-handed orthonormal";
        return null;
    }

    /// <summary>Eccentricity at and above which a body sorts in the trailing "comets"
    /// group of <see cref="SiblingSortKey"/> (the catalogs carry no body-type field, so
    /// eccentricity is the classifier). 0.9 splits the game's population cleanly:
    /// its comets (Halley 0.97, Lovejoy ~0.998, the hyperbolic interstellars) sit
    /// above it, the most eccentric dwarf planets (Sedna ~0.85) below. A low-e
    /// short-period comet (Encke-class, e < 0.9) would sort with the planets by its
    /// SMA — accepted: such a body RIDES like a planet, and mid-list is where a
    /// distance-ordered eye looks for it.</summary>
    public const double CometEccentricity = 0.9;

    /// <summary>Sibling ordering key for <see cref="HierarchyOrder"/>: bound, sub-comet
    /// eccentricity bodies (planets, dwarf planets, moons) group first, ordered by
    /// semi-major axis — "in order from the sun" under a star, by orbital distance
    /// under a planet — then comets (e >= <see cref="CometEccentricity"/> or
    /// hyperbolic), ordered by periapsis q = a(1-e), which is finite and positive for
    /// every conic (a hyperbolic SMA is negative, so SMA cannot order that group).
    /// Roots and unknown orbits key to the front of the first group.</summary>
    public static (int Group, double Distance) SiblingSortKey(OrbitalElements? orbit)
    {
        if (orbit is not { } o) return (0, 0.0);
        return o.Eccentricity >= CometEccentricity || o.SemiMajorAxis < 0
            ? (1, Kepler.PeriapsisDistance(o))
            : (0, o.SemiMajorAxis);
    }

    /// <summary>Hierarchy (DFS) ordering of body ids for the picker: parents precede
    /// children, siblings sort by <paramref name="sortKeyOf"/> — group ascending, then
    /// distance, ordinal-id ties — or plain ordinal when no key is given. Depth is the
    /// depth in the REDUCED tree (each body hangs under its nearest ancestor within
    /// the set, so a body whose direct parent is not offered still lands under the
    /// grandparent instead of floating).
    /// ParentId IS that nearest in-set ancestor (null for roots) — THE pair-partner
    /// rule for two-body fixed frames, consumed by both the frames tree and the burn
    /// planner's frame picker so the two panels can never offer different pair frames
    /// for the same body (and, over the integrated set, never a pair partner that
    /// ValidateSpec would refuse). Pure over the injected parent map and key —
    /// offline-tested; rails hierarchy and elements are parse-time constant
    /// (RailsService.ParentIdOf/OrbitOf).</summary>
    public static IReadOnlyList<(string Id, int Depth, string? ParentId)> HierarchyOrder(
        IReadOnlyCollection<string> ids, Func<string, string?> parentOf,
        Func<string, (int Group, double Distance)>? sortKeyOf = null)
    {
        var set = new HashSet<string>(ids, StringComparer.Ordinal);
        var children = new Dictionary<string, List<string>>(StringComparer.Ordinal);
        var roots = new List<string>();
        foreach (string id in set)
        {
            string? ancestor = parentOf(id);
            // Walk to the nearest ancestor IN the set; guard caps a malformed cycle.
            for (int guard = 0; ancestor is not null && !set.Contains(ancestor) && guard < set.Count; guard++)
                ancestor = parentOf(ancestor);
            if (ancestor is null || !set.Contains(ancestor) || ancestor == id)
            {
                roots.Add(id);
                continue;
            }
            if (!children.TryGetValue(ancestor, out var siblings))
                children[ancestor] = siblings = [];
            siblings.Add(id);
        }
        // No key = one constant key: the keyed comparator's ordinal tie-break IS the
        // keyless ordering, so there is only ONE ordering policy to maintain.
        sortKeyOf ??= static _ => (0, 0.0);
        Comparison<string> siblingOrder = (a, b) =>
        {
            var (aGroup, aDistance) = sortKeyOf(a);
            var (bGroup, bDistance) = sortKeyOf(b);
            if (aGroup != bGroup) return aGroup.CompareTo(bGroup);
            // CompareTo (not <): a NaN distance from a malformed orbit still
            // yields a total order — List.Sort throws on an inconsistent one.
            int byDistance = aDistance.CompareTo(bDistance);
            return byDistance != 0 ? byDistance : string.CompareOrdinal(a, b);
        };
        var ordered = new List<(string Id, int Depth, string? ParentId)>(set.Count);
        roots.Sort(siblingOrder);
        void Visit(string id, int depth, string? parent)
        {
            ordered.Add((id, depth, parent));
            if (!children.TryGetValue(id, out var kids)) return;
            kids.Sort(siblingOrder);
            foreach (string kid in kids) Visit(kid, depth + 1, id);
        }
        foreach (string root in roots) Visit(root, 0, null);
        return ordered;
    }

    /// <summary>Tolerance gate for building a two-body fixed frame
    /// (FrameKernel.Rotating) from LIVE data. FrameKernel's near-degeneracy contract
    /// throws on EXACT zero only — near-degenerate inputs return a finite but
    /// ill-conditioned pose, so live-data consumers must gate |h| themselves: this is
    /// that gate. Null when the pair's geometry is well-conditioned; otherwise a
    /// panel-ready rejection reason.</summary>
    public static string? ValidateGeometry(StateVector primary, StateVector secondary)
        => ValidateRotatingGeometry(primary, secondary);

    /// <summary>Checks every component of one sampled state before any subtraction or
    /// frame construction can turn an invalid source into an invalid pose.</summary>
    internal static string? ValidateState(StateVector state, string role)
    {
        if (!IsFinite(state.Position))
            return $"non-finite frame state: {role} position";
        if (!IsFinite(state.Velocity))
            return $"non-finite frame state: {role} velocity";
        return null;
    }

    /// <summary>Final admission check shared by every frame kind. Null means the pose
    /// is finite, positively scaled, and carries a right-handed orthonormal basis.</summary>
    public static string? ValidatePose(FramePose pose)
    {
        if (!IsFinite(pose.Origin)) return "non-finite frame pose: origin";
        if (!double.IsFinite(pose.Scale)) return "non-finite frame pose: scale";
        if (!(pose.Scale > 0)) return "degenerate frame pose: scale must be positive";
        return ValidateBasis(pose.XAxis, pose.YAxis, pose.ZAxis, "frame pose");
    }

    /// <summary>Validates, constructs, and post-validates a rotating pose. The out value
    /// remains default unless every stage succeeds.</summary>
    internal static string? TryCreateRotatingPose(
        StateVector primary, StateVector secondary, out FramePose pose)
    {
        pose = default;
        if (ValidateRotatingGeometry(primary, secondary) is { } reason) return reason;

        FramePose candidate;
        try
        {
            candidate = FrameKernel.Rotating(primary, secondary);
        }
        catch (ArgumentException e)
        {
            return $"degenerate frame geometry: rotating-frame construction failed: {e.Message}";
        }

        if (ValidatePose(candidate) is { } poseReason) return poseReason;
        pose = candidate;
        return null;
    }

    /// <summary>Publishes a constructed pose only after the shared final postcondition.</summary>
    internal static string? TryAcceptPose(FramePose candidate, out FramePose pose)
    {
        pose = default;
        if (ValidatePose(candidate) is { } reason) return reason;
        pose = candidate;
        return null;
    }

    private static string? ValidateRotatingGeometry(
        StateVector primary, StateVector secondary)
    {
        if (ValidateState(primary, "primary") is { } primaryReason) return primaryReason;
        if (ValidateState(secondary, "secondary") is { } secondaryReason) return secondaryReason;
        var r = secondary.Position - primary.Position;
        var v = secondary.Velocity - primary.Velocity;
        if (!IsFinite(r)) return "non-finite frame geometry: relative position";
        if (!IsFinite(v)) return "non-finite frame geometry: relative velocity";

        // The raw cross itself is part of the acceptance contract: a non-finite
        // component must never be hidden by direction-first normalization.
        var rawCross = r.Cross(v);
        if (!IsFinite(rawCross))
            return "non-finite frame geometry: angular-momentum cross product";
        double rawCrossLength = StableLength(rawCross, out _);
        if (!double.IsFinite(rawCrossLength))
            return "non-finite frame geometry: angular-momentum length";

        double separationLength = StableLength(r, out var separationDirection);
        double velocityLength = StableLength(v, out var velocityDirection);
        if (!double.IsFinite(separationLength))
            return "non-finite frame geometry: separation length";
        if (!double.IsFinite(velocityLength))
            return "non-finite frame geometry: relative-speed length";
        if (!(separationLength > 0)) return "degenerate geometry: bodies coincide";
        if (!(velocityLength > 0)) return "degenerate geometry: no relative motion";
        if (!IsFinite(separationDirection) || !IsFinite(velocityDirection))
            return "non-finite frame geometry: normalized direction";

        // Scale-safe sin(theta): no |r||v| product and no raw-cross magnitude is used
        // for conditioning, so finite 1e300 and 1e-200 geometries behave identically.
        var normalizedCross = separationDirection.Cross(velocityDirection);
        if (!IsFinite(normalizedCross))
            return "non-finite frame geometry: normalized cross product";
        double rotationSine = StableLength(normalizedCross, out var z);
        if (!double.IsFinite(rotationSine))
            return "non-finite frame geometry: rotation sine";
        if (!(rotationSine >= MinRotationSine))
            return "degenerate geometry: relative motion (nearly) radial";

        var y = z.Cross(separationDirection);
        if (ValidateBasis(separationDirection, y, z, "frame geometry") is { } basisReason)
            return basisReason;
        return null;
    }

    /// <summary>Overflow/underflow-safe Euclidean length and normalization. The input is
    /// expected to have finite components; a mathematically unrepresentable magnitude
    /// is returned as infinity and rejected by the caller.</summary>
    private static double StableLength(Vector3d value, out Vector3d normalized)
    {
        normalized = default;
        double scale = Math.Max(Math.Abs(value.X),
            Math.Max(Math.Abs(value.Y), Math.Abs(value.Z)));
        if (!(scale > 0)) return scale;
        var scaled = value / scale;
        double scaledLength = scaled.Length();
        if (!(scaledLength > 0) || !double.IsFinite(scaledLength))
            return double.NaN;
        normalized = scaled / scaledLength;
        return scale * scaledLength;
    }

    private static string? ValidateBasis(
        Vector3d x, Vector3d y, Vector3d z, string subject)
    {
        if (!IsFinite(x) || !IsFinite(y) || !IsFinite(z))
            return $"non-finite {subject}: basis components";
        double xLength = x.Length();
        double yLength = y.Length();
        double zLength = z.Length();
        if (!double.IsFinite(xLength)
            || !double.IsFinite(yLength)
            || !double.IsFinite(zLength))
            return $"non-finite {subject}: basis axis length";
        double maximumLengthError = Math.Max(Math.Abs(xLength - 1),
            Math.Max(Math.Abs(yLength - 1), Math.Abs(zLength - 1)));
        if (!double.IsFinite(maximumLengthError))
            return $"non-finite {subject}: basis length error";
        if (maximumLengthError > BasisTolerance)
            return $"degenerate {subject}: non-unit basis axes";
        var handednessResidual = x.Cross(y) - z;
        if (!IsFinite(handednessResidual))
            return $"non-finite {subject}: handedness residual";
        double handednessError = handednessResidual.Length();
        if (!double.IsFinite(handednessError))
            return $"non-finite {subject}: handedness error";
        if (handednessError > BasisTolerance)
            return $"degenerate {subject}: basis is not right-handed orthonormal";
        return null;
    }

    private static bool IsFinite(Vector3d value) =>
        double.IsFinite(value.X)
        && double.IsFinite(value.Y)
        && double.IsFinite(value.Z);
}

/// <summary>Owns the active DISPLAY frame. Strictly display-side: consumers
/// are the map camera counter-pose and the curve re-embedding; no
/// simulation surface ever reads it. Pose sampling uses game-convention states
/// (RailsService.GetGameEcl) so frames land exactly where the game draws the bodies.
/// Deactivated by the session statics sweep on every rebind/save load; each vessel's
/// last selection is restored separately from the matched sidecar. Never throws:
/// current-display sampling failures deactivate the matching activation with a warning;
/// arbitrary-time curve failures degrade only their batch.</summary>
public static class FrameManager
{
    private static readonly object Gate = new();
    private static FrameSpec? _active;
    private static long _activationGeneration;
    /// <summary>Last successfully ACTIVATED spec — survives self-deactivation (sampling
    /// failures), so <see cref="EnsureActiveOrDefault"/> restores the frame the user
    /// chose once it samples again instead of silently swapping to the default.</summary>
    private static FrameSpec? _lastActivated;
    /// <summary>Last user-selected frame per controlled vessel. The active pose remains
    /// global because only one vessel is controlled/drawn at a time; this map supplies
    /// the desired spec when control changes and is the sidecar persistence source.</summary>
    private static readonly Dictionary<string, FrameSpec> VesselSelections =
        new(StringComparer.Ordinal);
    private static string? _activeVesselId;
    /// <summary>Invalidates activation work that sampled across vessel destruction,
    /// id reuse, control switching, import, or a session reset.</summary>
    private static long _selectionGeneration;
    /// <summary>Wall-clock backoff between ensure-active attempts: a persistent
    /// activation failure (rails horizon miss during a long load, a conjunction-
    /// degenerate pair) must retry, but not once per rendered frame.</summary>
    private static long _nextEnsureAttemptMs;
    private static double _activationTime;
    private static FramePose _activationPose;
    // Surface frames only: the spin model captured (and verified against the game's own
    // orientation) at activation by BodyRotationReader. Meaningful iff _active is a
    // Surface spec — always written together with _active under Gate.
    private static BodyRotation _surfaceRotation;
    private static long _nextWarnMs;

    public static FrameSpec? Active { get { lock (Gate) return _active; } }

    /// <summary>Pose snapshotted at activation (the camera counter-pose rotates by the
    /// delta between the current pose's axes and these).</summary>
    public static FramePose ActivationPose { get { lock (Gate) return _activationPose; } }
    public static double ActivationTime { get { lock (Gate) return _activationTime; } }

    /// <summary>Sidecar projection for one vessel. Null selects the default and is
    /// also returned for vessels that were never controlled.</summary>
    internal static SidecarFrame? SelectedFrameForSidecar(string vesselId)
    {
        lock (Gate)
            return VesselSelections.TryGetValue(vesselId, out var spec)
                ? FrameSelectionSidecar.ToSidecar(spec)
                : null;
    }

    /// <summary>Stable snapshot of every remembered selection. This intentionally does
    /// not consult VesselRegistry: UI preferences survive even when exact trajectory
    /// state is ineligible for persistence.</summary>
    internal static List<SidecarFrameSelection> FrameSelectionsForSidecar()
    {
        lock (Gate)
            return VesselSelections
                .OrderBy(pair => pair.Key, StringComparer.Ordinal)
                .Select(pair => new SidecarFrameSelection
                {
                    VesselId = pair.Key,
                    Frame = FrameSelectionSidecar.ToSidecar(pair.Value),
                })
                .ToList();
    }

    /// <summary>Restores all sane per-vessel choices from a matched sidecar after the
    /// rebind sweep. Invalid entries are ignored independently.</summary>
    internal static int ImportFrameSelections(SidecarFile sidecar)
    {
        int restored = 0;
        lock (Gate)
        {
            foreach (var selection in sidecar.FrameSelections ?? [])
            {
                if (selection is null || string.IsNullOrEmpty(selection.VesselId)
                    || FrameSelectionSidecar.FromSidecar(selection.Frame) is not { } spec)
                    continue;
                VesselSelections[selection.VesselId] = spec;
                restored++;
            }
        }
        return restored;
    }

    /// <summary>Vessel-lifecycle seam: a destroyed/evicted vessel must not donate its
    /// preference to a later vehicle that recycles the same string id.</summary>
    internal static void ForgetSelection(string vesselId)
    {
        lock (Gate)
        {
            _selectionGeneration++;
            VesselSelections.Remove(vesselId);
            if (!string.Equals(_activeVesselId, vesselId, StringComparison.Ordinal)) return;
            _active = null;
            _activationGeneration++;
            _lastActivated = null;
            _activeVesselId = null;
        }
        _nextEnsureAttemptMs = 0;
    }

    private static void DiscardCurrentPreference(FrameSpec spec)
    {
        lock (Gate)
        {
            if (_lastActivated == spec) _lastActivated = null;
            if (_activeVesselId is { } current
                && VesselSelections.TryGetValue(current, out var selected)
                && selected == spec)
                VesselSelections.Remove(current);
        }
    }

    private static bool IsPermanentlyInvalid(FrameSpec spec) =>
        ModServices.Rails is { } rails
        && FrameCatalog.ValidateSpec(spec, rails.ModeledIds) is not null;

    /// <summary>Atomic activation snapshot — every value needed to sample or
    /// counter-pose one exact activation, including a generation identity that prevents
    /// stale workers from clearing or publishing against a later selection.</summary>
    internal static bool TryCaptureActive(out ActiveFrameSnapshot snapshot)
    {
        lock (Gate)
        {
            if (_active is null)
            {
                snapshot = default;
                return false;
            }
            snapshot = new ActiveFrameSnapshot(_active, _activationPose, _activationTime,
                _surfaceRotation, _activationGeneration);
            return true;
        }
    }

    public static bool TryGetActive(out FrameSpec spec, out FramePose pose, out double activationTime)
    {
        if (!TryCaptureActive(out var snapshot))
        {
            spec = null!;
            pose = default;
            activationTime = 0;
            return false;
        }
        spec = snapshot.Spec;
        pose = snapshot.ActivationPose;
        activationTime = snapshot.ActivationTime;
        return true;
    }

    /// <summary>Cached picker ordering: the rails hierarchy and integrated set are
    /// parse-time constant per bind (RailsService doc), yet FramesPanel and
    /// BurnPlannerPanel ask for this every UI frame — rebuilding HierarchyOrder's
    /// HashSet/Dictionary/DFS/sorts per frame is pure render-thread garbage. One
    /// immutable snapshot, swapped whole (volatile) so UI-draw reads race safely
    /// against rebinds on other paths; keyed on the bound RailsService plus the
    /// integrated-set identity (reference + count) as a cheap drift guard.</summary>
    private sealed record CandidateCache(
        RailsService Rails, IReadOnlyCollection<string> Ids, int Count,
        IReadOnlyList<(string Id, int Depth, string? ParentId)> Ordered);

    private static volatile CandidateCache? _candidates;
    private static volatile TargetFrameCandidate? _targetCandidate;

    /// <summary>Modeled bodies for the picker, hierarchy-ordered (parents precede
    /// children; siblings by distance from their primary, comets last —
    /// FrameCatalog.SiblingSortKey) with the reduced-tree depth for indentation and the
    /// nearest in-set ancestor as the two-body pair partner (HierarchyOrder doc).
    /// Rebuilt only when the rails bind (or its modeled-set identity) changes and
    /// on the session statics sweep — identical between rebinds by construction.</summary>
    public static IReadOnlyList<(string Id, int Depth, string? ParentId)> CandidateBodies()
    {
        if (ModServices.Rails is not { } rails) return [];
        var ids = rails.ModeledIds;
        var cache = _candidates;
        if (cache is null || !ReferenceEquals(cache.Rails, rails)
            || !ReferenceEquals(cache.Ids, ids) || cache.Count != ids.Count)
        {
            cache = new CandidateCache(rails, ids, ids.Count,
                FrameCatalog.HierarchyOrder(ids, rails.ParentIdOf,
                    id => FrameCatalog.SiblingSortKey(rails.OrbitOf(id))));
            _candidates = cache;
        }
        return cache.Ordered;
    }

    /// <summary>The controlled vehicle's current vessel target, when both vehicles have
    /// live registry entries and the target therefore has an authoritative predictor.
    /// The Program.ControlledVehicle reference can be stale after boot-loads, so its Id
    /// is resolved through VesselRegistry before reading Target.</summary>
    private enum TargetCandidateState { Unavailable, Known }

    public static TargetFrameCandidate? CandidateTargetVessel()
    {
        ResolveTargetCandidate(out var candidate);
        return candidate;
    }

    private static TargetCandidateState ResolveTargetCandidate(out TargetFrameCandidate? candidate)
    {
        candidate = null;
        if (!ModServices.TryGetBound(out var services)
            || KSA.Program.ControlledVehicle is not { } marker)
            return TargetCandidateState.Unavailable;
        var vessels = services.Vessels;
        var rails = services.Rails;
        if (vessels.TryGetLiveVehicle(marker.Id) is not { } controlled)
            return TargetCandidateState.Unavailable;
        if (controlled.Target is not KSA.Vehicle target)
            return TargetCandidateState.Known;
        if (string.Equals(target.Id, marker.Id, StringComparison.Ordinal))
            return TargetCandidateState.Known;
        if (!vessels.TryCaptureRailsAuthority(target, out var authority, out _)
            || authority.Tracked.LastParentId is not { } parentId)
            return TargetCandidateState.Unavailable;
        if (!rails.IsModeled(parentId))
            return TargetCandidateState.Known;
        var cached = _targetCandidate;
        if (cached is not null
            && string.Equals(cached.VesselId, target.Id, StringComparison.Ordinal)
            && string.Equals(cached.ParentBodyId, parentId, StringComparison.Ordinal))
        {
            candidate = cached;
            return TargetCandidateState.Known;
        }
        candidate = _targetCandidate = new TargetFrameCandidate(target.Id, parentId);
        return TargetCandidateState.Known;
    }

    private static bool TargetChoiceShouldRetire(FrameSpec spec)
    {
        var state = ResolveTargetCandidate(out var candidate);
        return FrameCatalog.TargetChoiceShouldRetire(
            spec, state == TargetCandidateState.Known, candidate?.Spec);
    }

    // (The pair partner rides on CandidateBodies' ParentId — the ONE reduced-tree
    // rule — not a separate raw ParentIdOf read.)

    /// <summary>Activate a frame (panel-ready status returned). Snapshot rule: the
    /// activation pose anchors the camera counter-pose delta at identity, so the map
    /// view is visually unchanged at the moment of activation.</summary>
    public static string Activate(FrameSpec spec) => Activate(spec, rememberSelection: true);

    private static string Activate(FrameSpec spec, bool rememberSelection)
    {
        try
        {
            string? vesselId = ControlledVesselId();
            long selectionGeneration;
            lock (Gate) selectionGeneration = _selectionGeneration;
            double now = KSA.Universe.GetElapsedSimTime().Seconds();
            if (GatedSample(spec, now, out var pose, out var rotation) is { } reason)
                return reason; // gates refuse — don't activate
            if (!string.Equals(vesselId, ControlledVesselId(), StringComparison.Ordinal))
                return "controlled vessel changed during activation";
            lock (Gate)
            {
                if (selectionGeneration != _selectionGeneration)
                    return "vessel selection changed during activation";
                _active = spec;
                _activationGeneration++;
                _activeVesselId = vesselId;
                if (rememberSelection)
                {
                    _lastActivated = spec;
                    if (vesselId is not null) VesselSelections[vesselId] = spec;
                }
                _activationTime = now;
                _activationPose = pose;
                _surfaceRotation = rotation;
            }
            ModLog.Info($"frame activated: {spec.Label} at t={now:F1} s");
            return "active";
        }
        catch (Exception e)
        {
            ModLog.Warn($"frame activation contained: {e.Message}");
            return $"error: {e.Message}";
        }
    }

    public static void Deactivate()
    {
        lock (Gate)
        {
            _active = null;
            _activationGeneration++;
        }
        ModLog.Info("frame deactivated");
    }

    /// <summary>The always-a-frame policy: the map runs in one of the mod's frames at
    /// all times (there is deliberately no stock/no-frame UI). Null-active means only
    /// "not activated YET" — a fresh session, or a frame that self-deactivated on a
    /// sampling failure — and this restores it: the last user-activated frame first
    /// (a conjunction-degenerate pair frame comes BACK once its geometry recovers,
    /// instead of being silently replaced), the kernel default
    /// (FrameSelectorKernel.DefaultFrame) otherwise. Returns the active spec, or null
    /// while activation keeps failing (retried on a wall-clock backoff — never once
    /// per rendered frame). Main-thread callers (panel draw phase).</summary>
    public static FrameSpec? EnsureActiveOrDefault()
    {
        string? vesselId = ControlledVesselId();
        bool vesselChanged;
        lock (Gate)
        {
            vesselChanged = !string.Equals(_activeVesselId, vesselId, StringComparison.Ordinal);
            if (vesselChanged)
            {
                _selectionGeneration++;
                _active = null;
                _activationGeneration++;
                _activeVesselId = vesselId;
                _lastActivated = vesselId is not null
                    && VesselSelections.TryGetValue(vesselId, out var selected)
                        ? selected
                        : null;
            }
        }
        if (vesselChanged) _nextEnsureAttemptMs = 0;
        if (Active is { } active)
        {
            // A target-fixed choice exists only while that exact target remains
            // selected under the same parent. Untargeting, retargeting, or an SOI
            // parent change removes the synthetic row and retires its frame.
            if (active.Kind != FrameKind.TargetFixed
                || !TargetChoiceShouldRetire(active))
            {
                FrameSpec? desired;
                lock (Gate) desired = _lastActivated;
                // A non-preferred active frame is the automatic fallback. Keep drawing
                // it, but periodically fall through to retry the remembered preference.
                if (desired is null || desired == active
                    || Environment.TickCount64 < _nextEnsureAttemptMs)
                    return active;
            }
            else
            {
                // Unlike a transient sampling failure, selection loss is terminal for
                // this session choice: forget it so the restore-last path below cannot
                // immediately resurrect a now-hidden target frame.
                lock (Gate)
                {
                    if (_active == active)
                    {
                        _active = null;
                        _activationGeneration++;
                    }
                }
                DiscardCurrentPreference(active);
                _nextEnsureAttemptMs = 0;
                ModLog.Info("target-fixed frame retired (target selection or parent changed)");
            }
        }
        if (Environment.TickCount64 < _nextEnsureAttemptMs) return null;
        _nextEnsureAttemptMs = Environment.TickCount64 + 2000;
        FrameSpec? last;
        lock (Gate) last = _lastActivated;
        if (last is { Kind: FrameKind.TargetFixed }
            && TargetChoiceShouldRetire(last))
        {
            DiscardCurrentPreference(last);
            last = null;
        }
        if (last is not null)
        {
            if (Activate(last, rememberSelection: false) == "active") return last;
            if (IsPermanentlyInvalid(last)) DiscardCurrentPreference(last);
            // A fallback already on screen remains correctly anchored; don't reactivate
            // it every retry and reset its camera pose.
            if (Active is { } existingFallback) return existingFallback;
        }
        if (FrameSelectorKernel.DefaultFrame(CandidateBodies()) is { } fallback
            && Activate(fallback, rememberSelection: false) == "active")
            return fallback;
        return null;
    }

    /// <summary>True while the map camera is NOT counter-posed: no active frame, or a
    /// body-centred INERTIAL frame (both are TryGetCameraDelta identity cases). THE
    /// gate for the stale-window stock fallbacks (patch-0 line, conic markers, hover,
    /// gizmo positions): stock-position geometry renders sanely in these views — the
    /// same pairing stock always had — while rotating/surface frames counter-pose the
    /// camera, where stock-position geometry is exactly the wrong-frame artifact those
    /// fallbacks refuse to draw.</summary>
    public static bool InertialView => Active is not { } spec || spec.Kind == FrameKind.Inertial;

    /// <summary>Current display pose at time t; false when no frame is active or sampling
    /// fails. A current-pose failure compare-retires that exact activation, never a frame
    /// selected while the sample was running.</summary>
    public static bool TrySamplePose(double t, out FramePose pose)
    {
        if (!TryCaptureActive(out var snapshot))
        {
            pose = default;
            return false;
        }
        return TrySamplePose(snapshot, t, FramePoseQuery.CurrentDisplay, out pose);
    }

    /// <summary>Pose used to transform one past or future curve sample. The caller
    /// supplies its batch's activation snapshot: a switch makes the sample fail instead
    /// of mixing coordinates, and a time-range failure invalidates only the batch.</summary>
    internal static bool TrySamplePoseForCurve(
        ActiveFrameSnapshot snapshot, double t, out FramePose pose) =>
        TrySamplePose(snapshot, t, FramePoseQuery.CurveSample, out pose);

    // snapshot-backed background curve sampling
    internal static bool TrySamplePoseForCurve(ActiveFrameSnapshot snapshot,
        RailsService.PredictionContext prediction, Func<double, StateVector?>? targetStateAt,
        double t, out FramePose pose)
    {
        pose = default;
        return TrySamplePoseFromPrediction(snapshot, prediction, targetStateAt, t, out pose);
    }

    /// <summary>Current-time twin for render staging that already captured and mode-
    /// checked a batch's activation. Failures retire only that exact activation.</summary>
    internal static bool TrySamplePoseForDisplay(
        ActiveFrameSnapshot snapshot, double t, out FramePose pose) =>
        TrySamplePose(snapshot, t, FramePoseQuery.CurrentDisplay, out pose);

    private static bool TrySamplePoseFromPrediction(ActiveFrameSnapshot snapshot,
        RailsService.PredictionContext prediction, Func<double, StateVector?>? targetStateAt,
        double t, out FramePose pose)
    {
        pose = default;
        try
        {
            var reason = SamplePose(prediction, snapshot.Spec,
                snapshot.SurfaceRotation, targetStateAt, t, out pose);
            // The activation snapshot is immutable. A frame change during this sweep is
            // rejected once by the batch label/generation consumers; checking FrameManager
            // Gate for every one of up to 262k points would turn a display lock into the
            // new hot path.
            if (reason is null) return true;
            pose = default;
            // Detached arbitrary-time failures invalidate only this immutable curve batch:
            // a transient bad future target sample must not retire a healthy current frame.
            // The shared throttle keeps a dense sweep from becoming a log flood.
            NoteContained("frame curve pose sampling", reason);
            return false;
        }
        catch (Exception e)
        {
            // Detached curve work is batch-local. Unlike a current-display sample,
            // an arbitrary-time exception must neither escape the worker nor retire
            // the still-healthy active frame that supplied this immutable snapshot.
            pose = default;
            NoteContained("frame curve pose sampling", e);
            return false;
        }
    }

    private static bool TrySamplePose(
        ActiveFrameSnapshot snapshot, double t, FramePoseQuery query, out FramePose pose)
    {
        pose = default;
        if (!ModServices.TryGetBound(out var services)) return false;
        try
        {
            if (SamplePose(services.Rails, services.Vessels,
                    snapshot.Spec, snapshot.SurfaceRotation, t, out pose) is { } reason)
            {
                pose = default;
                FramePoseFailurePolicy.OnFailure(query, () => DeactivateIfCurrent(snapshot));
                NoteContained("frame pose sampling", reason);
                return false;
            }
            // Sampling happens outside Gate. Never hand a superseded frame pose to a caller
            // after a concurrent deactivate/re-activate or selection change.
            if (IsCurrent(snapshot)) return true;
            pose = default;
            return false;
        }
        catch (Exception e)
        {
            pose = default;
            FramePoseFailurePolicy.OnFailure(query, () => DeactivateIfCurrent(snapshot));
            NoteContained("frame pose sampling", e);
            return false;
        }
    }

    private static bool IsCurrent(ActiveFrameSnapshot snapshot)
    {
        lock (Gate)
            return FrameActivationKernel.IsCurrent(_active, _activationGeneration, snapshot);
    }

    private static void DeactivateIfCurrent(ActiveFrameSnapshot snapshot)
    {
        bool deactivated = false;
        lock (Gate)
        {
            deactivated = FrameActivationKernel.TryDeactivate(
                ref _active, ref _activationGeneration, snapshot);
        }
        if (deactivated) ModLog.Info("frame deactivated");
    }

    /// <summary>Burn-authoring seam (flight-plan feature): pose of an ARBITRARY catalog
    /// frame at time t, independent of the ACTIVE display frame — the planner interprets
    /// a burn's authored components along these axes at the burn time. THE same gate
    /// pipeline as <see cref="Activate"/> (both call <see cref="GatedSample"/>, so the
    /// display and burn-authoring paths cannot drift), but never touches the
    /// active-frame state: authoring a burn must not disturb the map display. Null
    /// reason on success; panel-ready reason otherwise (rails horizon misses, degenerate
    /// geometry, KSA-less process — all contained here). Main-thread callers only for
    /// Surface specs (BodyRotationReader walks the live system).</summary>
    public static string? SampleSpecPose(FrameSpec spec, double t, out FramePose pose)
    {
        try
        {
            return GatedSample(spec, t, out pose, out _);
        }
        catch (Exception e)
        {
            pose = default;
            return $"frame pose at t={t:F0} s failed: {e.Message}";
        }
    }

    /// <summary>The ONE gated sampler behind both <see cref="Activate"/> (display) and
    /// <see cref="SampleSpecPose"/> (burn authoring): rails bind check, spec validation
    /// over the integrated set, the Surface live-seam gate — read the body's spin model
    /// from the game, tolerance-gate it (FrameCatalog.ValidateRotation), verify the
    /// arbitrary-t reconstruction against the game's own orientation
    /// (BodyRotationReader) — then the pose with its geometry tolerance gate. A single
    /// pipeline so the planner can never author in a frame the display would refuse (or
    /// vice versa). Null reason on success, with the pose and (for Surface specs) the
    /// verified spin model outed; panel-ready rejection otherwise. May throw on rails
    /// horizon misses etc. — callers contain per their own framing.</summary>
    private static string? GatedSample(
        FrameSpec spec, double t, out FramePose pose, out BodyRotation rotation)
    {
        pose = default;
        rotation = default;
        if (!double.IsFinite(t)) return "non-finite frame sample time";
        if (!ModServices.EnsureBound(out var services))
            return "rails unavailable";
        var rails = services.Rails;
        if (FrameCatalog.ValidateSpec(spec, rails.ModeledIds) is { } specReason)
            return specReason;
        if (spec.Kind == FrameKind.Surface
            && BodyRotationReader.TryRead(spec.PrimaryId, t, out rotation) is { } spinReason)
            return spinReason;
        return SamplePose(rails, services.Vessels, spec, rotation, t, out pose);
    }

    /// <summary>The camera counter-pose inputs — the current frame origin (game
    /// convention) and the rotation from the activation axes to the current axes. Kind-
    /// agnostic over the pose axes, so it serves two-body fixed frames (counter-rotate
    /// with the pair line) and surface frames (counter-rotate with the body spin) alike.
    /// Identity-delta cases (no frame / inertial frame / sampling failure) return false
    /// and the map stays stock-posed. Consumes TryGetActive (the atomic spec+activation
    /// snapshot) — never separate Active/ActivationPose reads, which could interleave
    /// with a deactivate/re-activate and pair a fresh spec with a stale pose.
    /// SCALE-BLIND by design (rotating-pulsating): only the pose AXES (unit
    /// length always — FramePose.Scale carries the pulsation separately) and ORIGIN feed
    /// the delta, so the game camera counter-ROTATES but never zooms with the two-body
    /// separation. At 'now' the drawn true positions are unchanged regardless of Scale
    /// (staging re-embeds via ToFrame-then-FromFrame, which cancels at the same pose),
    /// so a rigid camera counter-pose stays exactly consistent with the curves.</summary>
    public static bool TryGetCameraDelta(out Brutal.Numerics.double3 center, out Brutal.Numerics.doubleQuat delta)
    {
        center = default;
        delta = Brutal.Numerics.doubleQuat.Identity;
        if (!TryCaptureActive(out var snapshot)) return false;
        if (snapshot.Spec.Kind == FrameKind.Inertial) return false;
        double now = KSA.Universe.GetElapsedSimTime().Seconds();
        if (!TrySamplePose(snapshot, now, FramePoseQuery.CurrentDisplay, out var pose)) return false;
        var activation = snapshot.ActivationPose;
        var qNow = MapPoseKernel.QuatFromBasis(
            FrameAdapter.ToGame(pose.XAxis), FrameAdapter.ToGame(pose.YAxis), FrameAdapter.ToGame(pose.ZAxis));
        var qActivation = MapPoseKernel.QuatFromBasis(
            FrameAdapter.ToGame(activation.XAxis), FrameAdapter.ToGame(activation.YAxis), FrameAdapter.ToGame(activation.ZAxis));
        // delta must satisfy Transform(v, delta) == Transform(Transform(v, Inverse(qActivation)), qNow)
        // — axes-at-activation mapped onto axes-now. Concatenate(a, b) = b * a ("a then b",
        // pinned by MapPoseKernelTests offline), so this is qNow * Inverse(qActivation).
        delta = Brutal.Numerics.doubleQuat.Concatenate(Brutal.Numerics.doubleQuat.Inverse(qActivation), qNow);
        center = FrameAdapter.ToGame(pose.Origin);
        return IsCurrent(snapshot);
    }

    /// <summary>Pose at time t in game-convention states. Null reason on success;
    /// otherwise the tolerance-gate rejection (FrameCatalog.ValidateGeometry — the
    /// kernel itself throws on exact zero only, which callers contain). Two-body pairs
    /// read both bodies under ONE rails Gate acquisition (fold discipline). Surface
    /// frames combine the rails origin with the activation-captured spin model — exact
    /// at arbitrary t (constant-rate spin about a fixed pole; reconstruction verified
    /// against the game at activation), which is what curve re-embedding needs for
    /// past/future sample times.</summary>
    internal static string? SamplePose(
        RailsService rails, VesselRegistry vessels, FrameSpec spec,
        in BodyRotation surfaceRotation, double t, out FramePose pose)
    {
        pose = default;
        if (!double.IsFinite(t)) return "non-finite frame sample time";
        switch (spec.Kind)
        {
            case FrameKind.Inertial:
            {
                var body = rails.GetGameEcl(spec.PrimaryId, t);
                if (FrameCatalog.ValidateState(body,
                        $"inertial body '{spec.PrimaryId}'") is { } stateReason)
                    return stateReason;
                return FrameCatalog.TryAcceptPose(FrameKernel.Inertial(body), out pose);
            }
            case FrameKind.Surface:
            {
                var body = rails.GetGameEcl(spec.PrimaryId, t);
                if (FrameCatalog.ValidateState(body,
                        $"surface body '{spec.PrimaryId}'") is { } stateReason)
                    return stateReason;
                return FrameCatalog.TryAcceptPose(
                    FrameKernel.Surface(body, surfaceRotation, t), out pose);
            }
            case FrameKind.TwoBodyFixed:
            {
                var (primary, secondary) = rails.GetGameEclPair(spec.PrimaryId, spec.SecondaryId!, t);
                return FrameCatalog.TryCreateRotatingPose(primary, secondary, out pose);
            }
            case FrameKind.TargetFixed:
            {
                if (KSA.Program.ControlledVehicle is not { } marker
                    || vessels.TryGetLiveVehicle(marker.Id) is not { } controlled
                    || controlled.Target is not KSA.Vehicle target
                    || !string.Equals(target.Id, spec.SecondaryId, StringComparison.Ordinal))
                    return $"target vessel '{spec.SecondaryId}' is not the current target";
                if (!vessels.TryReadAuthoritativePredictorState(
                        target, t, out var absolute, out var authorityReason))
                    return $"target vessel '{spec.SecondaryId}' predictor unavailable: "
                        + PredictorAuthorityPolicy.Describe(authorityReason);
                // Vessel predictors use the mod's barycentric absolute convention;
                // display poses use the game's root-pinned convention (GetGameEcl).
                if (FrameCatalog.ValidateState(absolute,
                        $"target vessel '{spec.SecondaryId}' absolute") is { } absoluteReason)
                    return absoluteReason;
                var root = rails.GetAbsolute(rails.RootId, t);
                if (FrameCatalog.ValidateState(root,
                        $"root body '{rails.RootId}' absolute") is { } rootReason)
                    return rootReason;
                var targetState = new StateVector(
                    absolute.Position - root.Position,
                    absolute.Velocity - root.Velocity);
                var primaryBody = rails.GetGameEcl(spec.PrimaryId, t);
                return FrameCatalog.TryCreateRotatingPose(primaryBody, targetState, out pose);
            }
            default:
                // Unreachable through the gated paths (ValidateSpec refuses unknown
                // kinds first); throwing — contained by every caller — keeps a future
                // FrameKind from silently posing as a plain two-body Rotating frame.
                throw new ArgumentOutOfRangeException(nameof(spec), spec.Kind, "unknown FrameKind");
        }
    }

    private static string? SamplePose(RailsService.PredictionContext prediction,
        FrameSpec spec, in BodyRotation rotation, Func<double, StateVector?>? targetStateAt,
        double t, out FramePose pose)
    {
        pose = default;
        if (!double.IsFinite(t)) return "non-finite frame sample time";
        switch (spec.Kind)
        {
            case FrameKind.Inertial:
            {
                var body = prediction.GetGameEcl(spec.PrimaryId, t);
                if (FrameCatalog.ValidateState(body,
                        $"inertial body '{spec.PrimaryId}'") is { } stateReason)
                    return stateReason;
                return FrameCatalog.TryAcceptPose(FrameKernel.Inertial(body), out pose);
            }
            case FrameKind.Surface:
            {
                var body = prediction.GetGameEcl(spec.PrimaryId, t);
                if (FrameCatalog.ValidateState(body,
                        $"surface body '{spec.PrimaryId}'") is { } stateReason)
                    return stateReason;
                return FrameCatalog.TryAcceptPose(
                    FrameKernel.Surface(body, rotation, t), out pose);
            }
            case FrameKind.TwoBodyFixed:
            {
                var pair = prediction.GetGameEclPair(spec.PrimaryId, spec.SecondaryId!, t);
                return FrameCatalog.TryCreateRotatingPose(pair.A, pair.B, out pose);
            }
            case FrameKind.TargetFixed:
            {
                var absolute = targetStateAt?.Invoke(t);
                if (absolute is null)
                    return $"target vessel '{spec.SecondaryId}' has no captured trajectory";
                if (FrameCatalog.ValidateState(absolute.Value,
                        $"target vessel '{spec.SecondaryId}' absolute") is { } absoluteReason)
                    return absoluteReason;
                var root = prediction.GetAbsolute(prediction.RootId, t);
                if (FrameCatalog.ValidateState(root,
                        $"root body '{prediction.RootId}' absolute") is { } rootReason)
                    return rootReason;
                var target = absolute.Value - root;
                var primary = prediction.GetGameEcl(spec.PrimaryId, t);
                return FrameCatalog.TryCreateRotatingPose(primary, target, out pose);
            }
            default:
                pose = default;
                return "unknown frame kind";
        }
    }

    /// <summary>Throttled containment reporter shared by the frame patches.</summary>
    public static void NoteContained(string where, Exception e) => NoteContained(where, e.Message);

    public static void NoteContained(string where, string message)
    {
        long now = Environment.TickCount64;
        if (now < System.Threading.Interlocked.Read(ref _nextWarnMs)) return;
        System.Threading.Interlocked.Exchange(ref _nextWarnMs, now + 30_000);
        ModLog.Warn($"{where} contained: {message}");
    }

    /// <summary>Session statics sweep: a save/load or rebind replaces the sim under the
    /// frame — deactivate rather than carry a stale activation snapshot (the
    /// last-activated memory goes too: its body may not exist in the new catalog),
    /// and drop the picker-order cache (the new bind's RailsService owns a fresh
    /// hierarchy). The ensure backoff resets so the new session activates its
    /// default immediately.</summary>
    internal static void ResetSessionStatics()
    {
        lock (Gate)
        {
            _active = null;
            _activationGeneration++;
            _lastActivated = null;
            _selectionGeneration++;
            _activeVesselId = null;
            VesselSelections.Clear();
        }
        _nextEnsureAttemptMs = 0;
        _candidates = null;
        _targetCandidate = null;
    }

    private static string? ControlledVesselId()
    {
        try { return ReadControlledVesselId(); }
        catch (FileNotFoundException) { return null; } // KSA-free offline suite
        catch (TypeLoadException) { return null; }
    }

    // Keep the KSA token out of ControlledVesselId's JIT body: the offline suite loads
    // FrameManager without copying KSA.dll, just like BodyRotationReader's live seam.
    [System.Runtime.CompilerServices.MethodImpl(
        System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    private static string? ReadControlledVesselId() => KSA.Program.ControlledVehicle?.Id;
}

/// <summary>Pure selected-frame sidecar codec. Kept separate from live frame validation:
/// restore checks the serialized shape here, then activation checks the current catalog,
/// target and geometry before a choice can affect the display.</summary>
internal static class FrameSelectionSidecar
{
    internal static SidecarFrame ToSidecar(FrameSpec spec) => new()
    {
        FrameKind = spec.Kind.ToString(),
        PrimaryId = spec.PrimaryId,
        SecondaryId = spec.SecondaryId,
    };

    internal static FrameSpec? FromSidecar(SidecarFrame? dto)
    {
        if (dto is null
            || !Enum.TryParse<FrameKind>(dto.FrameKind, ignoreCase: false, out var kind)
            || !Enum.IsDefined(kind)
            || string.IsNullOrEmpty(dto.PrimaryId))
            return null;
        bool needsSecondary = kind is FrameKind.TwoBodyFixed or FrameKind.TargetFixed;
        if (needsSecondary ? string.IsNullOrEmpty(dto.SecondaryId) : dto.SecondaryId is not null)
            return null;
        return new FrameSpec(kind, dto.PrimaryId, dto.SecondaryId);
    }
}

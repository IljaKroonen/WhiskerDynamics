namespace WhiskerDynamics.Mod.Frames;

/// <summary>One visible row of the frames-panel body tree: the body, its depth in the
/// reduced hierarchy (indentation), its pair partner (FrameCatalog.HierarchyOrder's
/// ParentId — the nearest offered ancestor, exactly the body the row visually hangs
/// under), whether it has children to collapse, and its current expansion.</summary>
public readonly record struct FrameTreeRow(
    string Id, int Depth, string? ParentId, bool HasChildren, bool Expanded,
    FrameSpec? TargetFrame = null);

public enum TrajectoryComputationPhase
{
    IntegratingRails,
    SamplingCelestialCurves,
}

public readonly record struct TrajectoryComputationProgress(
    TrajectoryComputationPhase Phase, double CompletedDays, double RequestedDays)
{
    public float Fraction => (float)Math.Clamp(CompletedDays / RequestedDays, 0.0, 1.0);
    public string PhaseLabel => Phase switch
    {
        TrajectoryComputationPhase.IntegratingRails => "Computing future trajectories...",
        TrajectoryComputationPhase.SamplingCelestialCurves => "Updating planet trajectories...",
        _ => throw new ArgumentOutOfRangeException(nameof(Phase), Phase, "unknown computation phase"),
    };
    public string CoverageLabel => $"{Math.Max(0.0, CompletedDays):F0} of {RequestedDays:F0} d";
}

/// <summary>KSA-free decision rules for the hierarchical frame selector
/// (FramesPanel is the thin ImGui adapter; these rules are the offline-tested part):
/// the collapsible-tree row walk over FrameCatalog.HierarchyOrder's DFS ordering, the
/// default-expansion policy, the ECI-style frame acronyms the row buttons carry, and
/// the orbits-window growth/cap captions. The free-form duration field is clamped by
/// `SettingsKernel`; rails grow in chunks and consumers clamp to the reached horizon.
/// The rails horizon follows the display window through
/// `SettingsKernel.ApplyPrediction`.</summary>
public static class FrameSelectorKernel
{
    /// <summary>Effective displayed future duration when the adaptive vessel-line
    /// sampler stopped specifically at its point budget. Work/dynamics limits and a
    /// physical surface cut have their own meanings and must not be presented as a
    /// point-budget cap.</summary>
    public static double? PointBudgetEffectiveDuration(bool truncated, bool workLimited,
        bool dynamicsLimited, bool physicallyCut, double startSeconds, double displayedEndSeconds)
    {
        if (!truncated || workLimited || dynamicsLimited || physicallyCut
            || !double.IsFinite(startSeconds) || !double.IsFinite(displayedEndSeconds)
            || displayedEndSeconds < startSeconds)
            return null;
        return displayedEndSeconds - startSeconds;
    }

    /// <summary>Current background stage for the Frames-panel readiness indicator.
    /// The one-day slack tolerates the steady-state lag behind "now + target" without
    /// flicker. Rails must cover every visible trajectory window; after that, the
    /// indicator remains until a complete celestial sampling pass covers its window.</summary>
    public static TrajectoryComputationProgress? FutureComputationProgress(
        double reachedAheadDays, double sampledCelestialDays, double railsTargetDays,
        double overlayDays, double celestialDays, bool celestialCurvesShown)
    {
        double visibleDays = Math.Max(overlayDays, celestialCurvesShown ? celestialDays : 0.0);
        double railsPromised = Math.Min(railsTargetDays, visibleDays);
        if (reachedAheadDays + 1.0 < railsPromised)
            return new(TrajectoryComputationPhase.IntegratingRails,
                Math.Max(0.0, reachedAheadDays), railsPromised);

        double celestialPromised = Math.Min(railsTargetDays, celestialDays);
        if (celestialCurvesShown && sampledCelestialDays + 1.0 < celestialPromised)
            return new(TrajectoryComputationPhase.SamplingCelestialCurves,
                Math.Max(0.0, sampledCelestialDays), celestialPromised);
        return null;
    }

    /// <summary>Default expansion policy (no user override yet): roots open, everything
    /// deeper closed — planets start with their moons hidden. THE one definition; the
    /// panel's IsExpanded and the offline tests both call it.</summary>
    public static bool DefaultExpanded(int depth) => depth == 0;

    /// <summary>The body whose centred-inertial frame is the session default when it is
    /// offered (the home world — where every mission starts).</summary>
    public const string DefaultFrameBodyId = "Earth";

    /// <summary>The default display frame: the map always runs in one of the mod's
    /// frames (there is deliberately NO stock/no-frame choice in the panel), so a fresh
    /// session needs one picked for it — <see cref="DefaultFrameBodyId"/>-Centred
    /// Inertial when that body is in the catalog, else the first root's (an unfamiliar
    /// catalog defaults to its star — visually the stock view, since inertial frames
    /// apply no camera counter-pose). Null only for an empty catalog (rails not bound
    /// yet); the panel retries.</summary>
    public static FrameSpec? DefaultFrame(
        IReadOnlyList<(string Id, int Depth, string? ParentId)> ordered)
    {
        if (ordered.Count == 0) return null;
        foreach (var (id, _, _) in ordered)
            if (string.Equals(id, DefaultFrameBodyId, StringComparison.Ordinal))
                return new FrameSpec(FrameKind.Inertial, id, null);
        return new FrameSpec(FrameKind.Inertial, ordered[0].Id, null);
    }

    /// <summary>Short coordinate-frame acronym for a row button: "ECI" (Earth-Centred
    /// Inertial), "ELF" (Earth-Luna Fixed), "ECEF" (Earth surface — the Earth-Centred
    /// Earth-Fixed convention), or "ETF" for an Earth-target fixed synthetic row.
    /// Body acronyms are generated from id initials, so they can repeat across same-
    /// initial bodies — the panel disambiguates with the full-label tooltip and per-row
    /// ImGui IDs, never with the acronym itself.</summary>
    public static string Abbreviate(FrameSpec spec)
    {
        char p = Initial(spec.PrimaryId);
        return spec.Kind switch
        {
            FrameKind.Inertial => $"{p}CI",
            FrameKind.TwoBodyFixed => $"{p}{Initial(spec.SecondaryId ?? "")}F",
            FrameKind.Surface => $"{p}C{p}F",
            FrameKind.TargetFixed => $"{p}TF",
            // Exhaustive like FrameSpec.Label: a future kind must fail loudly (into the
            // panel's 3-strike containment) rather than ship an unlabeled button.
            _ => throw new ArgumentOutOfRangeException(nameof(spec), spec.Kind, "unknown FrameKind"),
        };
    }

    private static char Initial(string id) =>
        id.Length > 0 ? char.ToUpperInvariant(id[0]) : '?';

    /// <summary>The rows the tree draws, in display order: walks the DFS hierarchy
    /// ordering, skipping every descendant of a row <paramref name="isExpanded"/>
    /// answers false for (subtrees collapse whole — a collapsed planet hides its
    /// moons). Leaves are never asked: a body without children has nothing to expand,
    /// so its row reports Expanded=false and draws no toggle.</summary>
    public static List<FrameTreeRow> VisibleRows(
        IReadOnlyList<(string Id, int Depth, string? ParentId)> ordered,
        Func<string, int, bool> isExpanded,
        TargetFrameCandidate? target = null)
    {
        // Inject the selected target as the final direct child of its current parent:
        // Earth > Luna, Target: Station, then the next planet. Inserting after the
        // parent's whole existing subtree preserves the catalog's DFS invariant.
        var candidates = new List<(string Id, int Depth, string? ParentId, FrameSpec? TargetFrame)>(
            ordered.Count + (target is null ? 0 : 1));
        int insertAt = -1;
        int targetDepth = 0;
        if (target is not null)
        {
            for (int i = 0; i < ordered.Count; i++)
            {
                if (!string.Equals(ordered[i].Id, target.ParentBodyId, StringComparison.Ordinal))
                    continue;
                targetDepth = ordered[i].Depth + 1;
                insertAt = i + 1;
                while (insertAt < ordered.Count && ordered[insertAt].Depth > ordered[i].Depth)
                    insertAt++;
                break;
            }
        }
        for (int i = 0; i <= ordered.Count; i++)
        {
            if (i == insertAt && target is not null)
                candidates.Add(($"Target: {target.VesselId}", targetDepth,
                    target.ParentBodyId, target.Spec));
            if (i < ordered.Count)
            {
                var (id, depth, parentId) = ordered[i];
                candidates.Add((id, depth, parentId, null));
            }
        }

        var rows = new List<FrameTreeRow>(candidates.Count);
        int? collapsedAt = null; // depth of the nearest collapsed ancestor row
        for (int i = 0; i < candidates.Count; i++)
        {
            var (id, depth, parentId, targetFrame) = candidates[i];
            if (collapsedAt is { } c && depth > c) continue; // inside a collapsed subtree
            collapsedAt = null;
            bool hasChildren = i + 1 < candidates.Count && candidates[i + 1].Depth > depth;
            bool expanded = hasChildren && isExpanded(id, depth);
            rows.Add(new FrameTreeRow(id, depth, parentId, hasChildren, expanded, targetFrame));
            if (hasChildren && !expanded) collapsedAt = depth;
        }
        return rows;
    }
}

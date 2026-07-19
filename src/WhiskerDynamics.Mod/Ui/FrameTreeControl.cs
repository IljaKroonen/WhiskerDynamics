using Brutal.ImGuiApi;
using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>A reusable collapsible body-frame tree control. The tree follows hierarchy
/// order and is depth-indented; a collapsed planet hides its moons. Each row carries
/// small acronym buttons — "ECI" (body-centred inertial), "ELF"
/// (body-parent two-body fixed), "ECEF" (body surface) — plus a selected target vessel
/// as a synthetic child of its current primary with an "ETF" target-fixed button. The
/// full frame name is the tooltip and the given active frame has a strong native highlight. Selection policy is
/// the CALLER's: <see cref="Draw"/> returns the clicked spec (the frames panel
/// activates the display frame; the burn planner's picker re-authors a burn). One
/// INSTANCE per host panel, so each keeps its own expansion state and caches; decision
/// rules live in the KSA-free <see cref="FrameSelectorKernel"/>.</summary>
internal sealed class FrameTreeControl
{
    private const float NameColumnWidth = 170f;
    private const float IndentPerDepth = 14f;

    /// <summary>Per-body expansion overrides; unset bodies use
    /// FrameSelectorKernel.DefaultExpanded (roots open, moons hidden).</summary>
    private readonly Dictionary<string, bool> _expandOverrides = new(StringComparer.Ordinal);

    /// <summary>Per-bind cache of each row's activatable frames (spec + acronym +
    /// tooltip label): pure rails-catalog data, parse-time constant per bind
    /// (FrameCatalog.HierarchyOrder doc) — rebuilding the strings every rendered frame
    /// would be steady render-thread garbage. Cleared by <see cref="Reset"/> with the
    /// host's session statics: a rebind is the only event that changes the catalog.</summary>
    private readonly record struct FrameButtonEntry(FrameSpec Spec, string Acronym, string Label);

    private readonly Dictionary<string, FrameButtonEntry[]> _rowCache = new(StringComparer.Ordinal);
    private FrameSpec? _targetEntrySpec;
    private FrameButtonEntry[]? _targetEntry;

    /// <summary>Visible tree rows, rebuilt only when the tree actually changes (an
    /// expand toggle or <see cref="Reset"/>) — the walk allocates a catalog-sized list,
    /// which at 60 Hz over a dense catalog would be pure render-thread garbage.</summary>
    private List<FrameTreeRow>? _visibleRows;
    private TargetFrameCandidate? _visibleTarget;

    /// <summary>Statics-sweep hook for the host panel: fresh tree state for the new
    /// session (the catalog underneath is about to be rebuilt by the rebind).</summary>
    public void Reset()
    {
        _expandOverrides.Clear();
        _rowCache.Clear();
        _targetEntrySpec = null;
        _targetEntry = null;
        _visibleRows = null;
        _visibleTarget = null;
    }

    /// <summary>Draws the tree; returns the frame spec clicked this frame (null when
    /// nothing was clicked). <paramref name="active"/> highlights that frame's button.</summary>
    public FrameSpec? Draw(FrameSpec? active)
    {
        var bodies = FrameManager.CandidateBodies();
        if (bodies.Count == 0) return null;
        var target = FrameManager.CandidateTargetVessel();
        if (target != _visibleTarget)
        {
            _visibleTarget = target;
            _visibleRows = null;
            _targetEntrySpec = null;
            _targetEntry = null;
        }
        var rows = _visibleRows ??= FrameSelectorKernel.VisibleRows(bodies, IsExpanded, target);
        FrameSpec? clicked = null;
        var cellPadding = new float2(8f, 0f);
        ImGui.PushStyleVar(ImGuiStyleVar.CellPadding, in cellPadding);
        if (!ImGui.BeginTable("##frame-tree"u8, 2,
                ImGuiTableFlags.SizingStretchProp))
        {
            ImGui.PopStyleVar();
            return null;
        }
        try
        {
            ImGui.TableSetupColumn("##body"u8, ImGuiTableColumnFlags.WidthFixed,
                NameColumnWidth);
            ImGui.TableSetupColumn("##frames"u8, ImGuiTableColumnFlags.WidthStretch);
            foreach (var row in rows)
            {
                ImGui.PushID(row.Id);
                try
                {
                    // Every row draws every frame — a ??= on the call would skip
                    // rendering the rest of the tree on the frame a button is clicked.
                    var rowClicked = DrawRow(row, active);
                    clicked ??= rowClicked;
                }
                finally
                {
                    ImGui.PopID();
                }
            }
        }
        finally
        {
            ImGui.EndTable();
            ImGui.PopStyleVar();
        }
        return clicked;
    }

    private bool IsExpanded(string id, int depth) =>
        _expandOverrides.TryGetValue(id, out bool expanded)
            ? expanded
            : FrameSelectorKernel.DefaultExpanded(depth)
                || string.Equals(id, _visibleTarget?.ParentBodyId, StringComparison.Ordinal);

    private FrameSpec? DrawRow(in FrameTreeRow row, FrameSpec? active)
    {
        ImGui.TableNextRow();
        ImGui.TableNextColumn();
        float indent = row.Depth * IndentPerDepth;
        if (indent > 0f) ImGui.Indent(indent);
        float rowControlSize = ImGui.GetTextLineHeight();
        if (row.HasChildren)
        {
            if (SmallArrowButton("##expand"u8,
                    row.Expanded ? ImGuiDir.Down : ImGuiDir.Right))
            {
                _expandOverrides[row.Id] = !row.Expanded;
                _visibleRows = null; // tree changed: rebuild the row cache next frame
            }
        }
        else
        {
            var spacer = new float2(rowControlSize, 1f); // keeps names aligned with siblings
            ImGui.Dummy(in spacer);
        }
        ImGui.SameLine(0f);
        ImGui.Text(row.Id);
        if (indent > 0f) ImGui.Unindent(indent);

        ImGui.TableNextColumn();
        FrameSpec? clicked = null;
        var entries = RowEntriesFor(row);
        for (int i = 0; i < entries.Length; i++)
        {
            if (i > 0) ImGui.SameLine(0f);
            var entry = entries[i];
            if (FrameChoice(entry.Acronym, entry.Spec == active))
                clicked = entry.Spec;
            // Acronyms are initials-generated and may repeat across same-initial
            // bodies; the full-name tooltip is the disambiguator.
            ImGui.SetItemTooltip(entry.Label);
        }
        return clicked;
    }

    private static bool SmallArrowButton(ImString id, ImGuiDir direction)
    {
        var framePadding = new float2(8f, 0f);
        ImGui.PushStyleVar(ImGuiStyleVar.FramePadding, in framePadding);
        try
        {
            return ImGui.ArrowButton(id, direction);
        }
        finally
        {
            ImGui.PopStyleVar();
        }
    }

    private FrameButtonEntry[] RowEntriesFor(in FrameTreeRow row)
    {
        if (row.TargetFrame is { } target)
        {
            if (_targetEntry is null || _targetEntrySpec != target)
            {
                _targetEntrySpec = target;
                _targetEntry =
                [new FrameButtonEntry(target, FrameSelectorKernel.Abbreviate(target), target.Label)];
            }
            return _targetEntry;
        }
        if (_rowCache.TryGetValue(row.Id, out var cached)) return cached;
        var specs = row.ParentId is { } parent
            ? new[]
            {
                new FrameSpec(FrameKind.Inertial, row.Id, null),
                new FrameSpec(FrameKind.TwoBodyFixed, row.Id, parent),
                new FrameSpec(FrameKind.Surface, row.Id, null),
            }
            : [new(FrameKind.Inertial, row.Id, null), new(FrameKind.Surface, row.Id, null)];
        var entries = new FrameButtonEntry[specs.Length];
        for (int i = 0; i < specs.Length; i++)
            entries[i] = new FrameButtonEntry(specs[i], FrameSelectorKernel.Abbreviate(specs[i]), specs[i].Label);
        _rowCache[row.Id] = entries;
        return entries;
    }

    /// <summary>Native compact button with a clear selected state.</summary>
    internal static bool FrameChoice(ImString label, bool selected) =>
        UiTheme.FrameChoice(label, selected);
}

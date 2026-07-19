using Brutal.ImGuiApi;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>The shared duration control: aligned decrement steppers, a d/h/m/s text
/// field, and aligned increment steppers arranged on one line. Steppers nudge the
/// value in seconds immediately;
/// the field shows TimeDisplayKernel.FormatDuration and commits
/// TryParseDuration when editing ends (Enter or focus loss — deactivation after an
/// edit). An unparseable commit reverts to the formatted value and hands the error
/// back for the caller's status line. One widget home serves both the planner's time
/// rows and the frames panel's orbits window, so the edit protocol (shared buffer,
/// mid-typing guard, stepper/commit composition) cannot fork between panels.</summary>
internal static class DurationField
{
    /// <summary>Shared text buffer for every duration field: only one ImGui item is
    /// ever active, inactive fields get the buffer refreshed to their own formatted
    /// value right before drawing, and the typed text is captured into
    /// <see cref="_editText"/> on every edited frame — so one buffer serves all rows
    /// without cross-talk.</summary>
    // Lazy nested holder: session/edit cancellation remains KSA-free for offline tests;
    // the Brutal.ImGui buffer initializes only when a row is actually drawn.
    private static class ImGuiState
    {
        internal static readonly ImInputString Buffer = new(64, ImString.Empty);
    }

    /// <summary>Which duration row is being typed in (row kind + per-row id, e.g.
    /// the burn's absolute time), or null when none: that row must NOT have its
    /// buffer refreshed mid-typing — ImGui detects an externally changed buffer and
    /// reloads its edit state, clobbering the user's text.</summary>
    private static (string Row, double Id)? _editKey;

    /// <summary>The duration field's text as last typed. Parsed on commit instead
    /// of the buffer: by deactivation time a LATER row's refresh has already reused
    /// the shared buffer, and ImGui only writes the buffer on frames it edited.</summary>
    private static string _editText = "";

    /// <summary>Clears any edit in progress when the session changes.</summary>
    internal static void ResetSessionStatics() => _editKey = null;

    /// <summary>Draws one row; true when the value changed. rowKey/id identify the
    /// row across frames (several rows can share one widget id under their PushID
    /// scopes, so the ImGui id alone cannot name the row being edited).
    /// <paramref name="parseError"/> is the status-line message for a failed commit,
    /// null otherwise. <paramref name="years"/> selects the window-scale vocabulary
    /// ("2y 30d") for display — parsing accepts "y" either way.</summary>
    internal static bool Row(ReadOnlySpan<byte> rowId, string rowKey, double id,
        ref double seconds, (double Step, string Minus, string Plus)[] steps,
        out string? parseError, bool years = false)
    {
        parseError = null;
        bool changed = false;
        // Stepper nudges accumulate separately and apply after the commit check, so a
        // click that defocuses an edited field composes with that field's commit.
        double stepDelta = 0;
        ImGui.PushID(rowId);
        try
        {
            float fieldWidth = UiLayout.MeasureStepFieldWidth(steps.Length);
            stepDelta += UiLayout.StepDecrements(steps);
            bool editing = _editKey is { } key
                && string.Equals(key.Row, rowKey, StringComparison.Ordinal) && key.Id == id;
            if (!editing) ImGuiState.Buffer.SetValue(TimeDisplayKernel.FormatDuration(seconds, years));
            ImGui.SetNextItemWidth(fieldWidth);
            if (ImGui.InputText("##value"u8, ImGuiState.Buffer))
                _editText = ImGuiState.Buffer.ToString();
            if (ImGui.IsItemActive()) _editKey = (rowKey, id);
            else if (editing) _editKey = null;
            if (ImGui.IsItemDeactivatedAfterEdit())
            {
                if (TimeDisplayKernel.TryParseDuration(_editText, out double parsed))
                {
                    seconds = parsed;
                    changed = true;
                }
                else
                {
                    parseError = $"invalid duration '{_editText}' - "
                        + "use y/d/h/m/s (e.g. 3d 4h 5m 6.25s) or bare seconds";
                }
            }
            stepDelta += UiLayout.StepIncrements(steps);
            changed |= stepDelta != 0;
            seconds += stepDelta;
        }
        finally
        {
            ImGui.PopID();
        }
        return changed;
    }
}

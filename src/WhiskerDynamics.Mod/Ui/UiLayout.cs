using Brutal.ImGuiApi;
using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Ui;

internal static class UiLayout
{
    internal const float StepFieldTargetWidth = 150f;
    internal const float StepButtonWidth = 38f;

    internal static bool BeginProperties(ImString id, float labelWidth)
    {
        if (!ImGui.BeginTable(id, 2, ImGuiTableFlags.SizingStretchProp)) return false;
        ImGui.TableSetupColumn("##property-label"u8, ImGuiTableColumnFlags.WidthFixed,
            labelWidth);
        ImGui.TableSetupColumn("##property-value"u8, ImGuiTableColumnFlags.WidthStretch);
        return true;
    }

    internal static void NextProperty(string label)
    {
        ImGui.TableNextRow();
        ImGui.TableNextColumn();
        ImGui.AlignTextToFramePadding();
        UiTheme.MutedText(label, wrapped: false);
        ImGui.TableNextColumn();
    }

    internal static float MeasureStepFieldWidth(int stepCount) =>
        StepFieldWidth(ImGui.GetContentRegionAvail().X, stepCount);

    internal static float StepFieldWidth(float availableWidth, int stepCount) =>
        Math.Clamp(availableWidth - Math.Max(0, stepCount) * 2f * StepButtonWidth,
            1f, StepFieldTargetWidth);

    internal static double StepDecrements(
        (double Step, string Minus, string Plus)[] steps)
    {
        double delta = 0;
        for (int k = steps.Length - 1; k >= 0; k--)
        {
            if (ImGui.Button(steps[k].Minus, new float2(StepButtonWidth, 0f)))
                delta -= steps[k].Step;
            ImGui.SameLine(0f);
        }
        return delta;
    }

    internal static double StepIncrements(
        (double Step, string Minus, string Plus)[] steps)
    {
        double delta = 0;
        for (int k = 0; k < steps.Length; k++)
        {
            ImGui.SameLine(0f);
            if (ImGui.Button(steps[k].Plus, new float2(StepButtonWidth, 0f)))
                delta += steps[k].Step;
        }
        return delta;
    }
}

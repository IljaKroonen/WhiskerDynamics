using Brutal.ImGuiApi;
using Brutal.Numerics;

namespace WhiskerDynamics.Mod.Ui;

internal static class UiTheme
{
    internal const float PropertyLabelWidth = 132f;
    internal const float SettingsLabelWidth = 218f;

    internal static void PrepareWindow(float width, float height, float minWidth, float minHeight)
    {
        ImGui.SetNextWindowSize(new float2(width, height), ImGuiCond.FirstUseEver);
        var minimum = new float2(minWidth, minHeight);
        var maximum = new float2(float.MaxValue, float.MaxValue);
        ImGui.SetNextWindowSizeConstraints(in minimum, in maximum);
    }

    internal static bool FrameChoice(ImString label, bool selected)
    {
        if (selected)
        {
            var style = ImGui.GetStyle();
            var selectedColor = style.Colors[(int)ImGuiCol.HeaderActive];
            var hoveredColor = style.Colors[(int)ImGuiCol.HeaderHovered];
            var borderColor = style.Colors[(int)ImGuiCol.CheckMark];
            ImGui.PushStyleColor(ImGuiCol.Button, in selectedColor);
            ImGui.PushStyleColor(ImGuiCol.ButtonHovered, in hoveredColor);
            ImGui.PushStyleColor(ImGuiCol.ButtonActive, in selectedColor);
            ImGui.PushStyleColor(ImGuiCol.Border, in borderColor);
            ImGui.PushStyleVar(ImGuiStyleVar.FrameBorderSize, 2f);
        }
        try
        {
            return ImGui.SmallButton(label);
        }
        finally
        {
            if (selected)
            {
                ImGui.PopStyleVar();
                ImGui.PopStyleColor(4);
            }
        }
    }

    internal static void MutedText(string text, bool wrapped = true)
    {
        if (wrapped) ImGui.PushTextWrapPos(0f);
        try
        {
            ImGui.TextDisabled(text);
        }
        finally
        {
            if (wrapped) ImGui.PopTextWrapPos();
        }
    }

    internal static void RightAlignedText(string text)
    {
        float available = ImGui.GetContentRegionAvail().X;
        float width = ImGui.CalcTextSize(text).X;
        if (width < available) ImGui.SetCursorPosX(ImGui.GetCursorPosX() + available - width);
        ImGui.Text(text);
    }
}

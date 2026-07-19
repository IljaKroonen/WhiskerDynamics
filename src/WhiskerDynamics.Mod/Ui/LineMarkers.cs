using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>Honest line markers on the map (in place of stock's
/// conic Ap/Pe markers, which PatchMarkerPatch suppresses): the
/// controlled vessel's actual and planned lines each show the first upcoming Ap/Pe per
/// frame-relevant body, AN/DN vs the mode's natural plane, predictor SOI
/// encounter/escape events, and closest approach to the selected target — computed from the
/// sampled batches at rebuild (TrajectoryOverlay.ComputeMarkers), positioned at render
/// time by interpolating the DRAWN line at the marker's time
/// (TrajectoryOverlay.TryDrawnPositionAt), so markers ride the re-embedded curve in
/// frame views and disappear with the line through blinks and stale windows. Drawn as
/// small filled dots + text into the background draw list (stock's own map-marker
/// surface, CelestialPosition.DrawUi precedent) from the same UI-draw phase as the
/// panels (StatusPanelPatch). Actual-line markers use stock's own first-patch line
/// color, planned-line markers the planned-burn color — a marker's tint says which
/// line owns it. Contained with the throttled reporter (a bad camera read must not
/// take the panels down; no 3-strike — draw-list writes hold no state).</summary>
public static class LineMarkers
{
    private const float DotRadius = 4f;

    public static void Draw()
    {
        if (!ModServices.Enabled) return;
        try
        {
            // A marker must never exist for a line that is not drawn: every gate the
            // LINE honors gates the markers too — the HUD toggle (stock draws all map
            // UI under Program.DrawUI), the vessel's own ShowOrbit, and
            // the mod's line-visibility policy in place of stock's distance cull
            // (VesselLinePatch draws the line under the same rule).
            if (!KSA.Program.DrawUI) return;
            if (KSA.Program.ControlledVehicle is not { } vehicle) return;
            if (!vehicle.ShowOrbit && !vehicle.TargetOfControlledVehicle) return;
            var viewport = KSA.Program.MainViewport;
            if (viewport.Mode != CameraMode.Map) return;
            if (vehicle.Orbit?.Parent is null || !Patches.LineVisibility.ForVessel(vehicle, viewport)) return;
            var camera = viewport.GetCamera();
            var drawList = ImGui.GetBackgroundDrawList();
            long nowMs = Environment.TickCount64;
            double nowSimSeconds = Universe.GetElapsedSimTime().Seconds();
            DrawBatch(OverlayBuffer.ReadFresh(vehicle.Id, nowMs, nowSimSeconds),
                nowSimSeconds, viewport, camera, drawList,
                (byte4)FlightPlan.FirstPatchColor);
            DrawBatch(OverlayBuffer.ReadPlannedFresh(vehicle.Id, nowMs, nowSimSeconds),
                nowSimSeconds, viewport, camera, drawList,
                (byte4)BurnPlan.BurnPatchColor);
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("line marker draw", e);
        }
    }

    private static void DrawBatch(OverlaySamples? samples, double nowSimSeconds,
        Viewport viewport, Camera camera, ImDrawListPtr drawList, byte4 color)
    {
        if (samples is null || samples.Markers.Count == 0) return;
        if (!TrajectoryOverlay.TryBuildMarkerDrawContext(samples, out var context)) return;
        int2 size = viewport.Size;
        foreach (var marker in samples.Markers)
        {
            if (marker.TimeSeconds < nowSimSeconds) continue;
            // Rides the drawn line: re-embedded per frame in frame views; false while
            // the line itself is not drawn (blink/mode mismatch) — markers go with it.
            if (!TrajectoryOverlay.TryDrawnPositionAt(
                    samples, marker.TimeSeconds, in context, out var world))
                continue;
            float2 screen = camera.EclToScreen(FrameAdapter.ToGame(world));
            if (float.IsNaN(screen.X) || float.IsNaN(screen.Y)) continue; // behind the camera
            if (screen.X < -DotRadius || screen.Y < -DotRadius
                || screen.X > size.X + DotRadius || screen.Y > size.Y + DotRadius)
                continue;
            float2 position = viewport.Position + screen;
            drawList.AddCircleFilled(position, DotRadius, color);
            ImGuiHelper.DrawTextOnScreen(drawList, position + new float2(7f, -7f),
                OverlayKernel.MarkerLabelAt(marker, nowSimSeconds), Color.White.AsByte4);
        }
    }
}

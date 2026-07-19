using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Thin adapter over <see cref="LineVisibilityKernel"/> for the three draw
/// surfaces (CelestialLinePatch, VesselLinePatch, LineMarkers). The global celestial
/// toggle and controlled-vessel-only rule apply in every view; map view replaces the
/// remaining stock distance culls while other views retain them.</summary>
internal static class LineVisibility
{
    internal static bool IsControlled(Vehicle vehicle) =>
        string.Equals(KSA.Program.ControlledVehicle?.Id, vehicle.Id, StringComparison.Ordinal);

    /// <summary>Only the controlled vessel can pass the stock line opt-in.</summary>
    internal static bool ForVessel(Vehicle vehicle, Viewport viewport)
    {
        bool visible = LineVisibilityKernel.VesselLineVisible(
            stockOptIn: vehicle.ShowOrbit || vehicle.TargetOfControlledVehicle,
            isControlled: IsControlled(vehicle));
        if (!visible) return false;
        if (viewport.Mode != CameraMode.Map)
            return vehicle.Orbit?.Parent is { } parent
                && Astronomical.ShouldDrawUiOrLines(parent, viewport, vehicle.Orbit);
        return true;
    }

    internal static bool BypassOrbitVisibilityCheck(Viewport viewport, bool isActive) =>
        LineVisibilityKernel.BypassOrbitVisibilityCheck(
            isMapView: viewport.Mode == CameraMode.Map,
            isActive: isActive);

    /// <summary>The global toggle gates all stock-enabled celestial lines.</summary>
    internal static bool ForCelestial(Celestial celestial, Viewport viewport)
    {
        bool visible = LineVisibilityKernel.CelestialLineVisible(
            stockOptIn: celestial.ShowOrbit || celestial.TargetOfControlledVehicle,
            showAstralBodyLines: ModServices.MapTrajectory.ShowAstralBodyLines);
        if (!visible) return false;
        if (viewport.Mode != CameraMode.Map)
            return Astronomical.ShouldDrawLines((Astronomical)celestial, viewport, null);
        return true;
    }
}

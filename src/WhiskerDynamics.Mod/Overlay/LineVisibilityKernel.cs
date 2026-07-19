namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Orbit-line visibility rules. Celestial lines have one global toggle and
/// retain stock's per-body opt-in. Vessel lines are restricted to the controlled
/// vessel. The active map vessel alone bypasses stock's final FOV/5-pixel cull so its
/// trajectory remains useful for navigation. KSA-free; the draw patches and
/// LineMarkers are the thin adapters (Patches.LineVisibility).</summary>
public static class LineVisibilityKernel
{
    /// <summary>A celestial line is shown only when both controls allow it.</summary>
    public static bool CelestialLineVisible(bool stockOptIn, bool showAstralBodyLines) =>
        stockOptIn && showAstralBodyLines;

    /// <summary>A vessel's actual, planned, and marker lines share one rule.</summary>
    public static bool VesselLineVisible(bool stockOptIn, bool isControlled) =>
        stockOptIn && isControlled;

    /// <summary>Stock exposes its final FOV/5-pixel orbit visibility check as one
    /// indivisible bypass. Suppress it only for the active (camera-followed) vessel
    /// in map view; other lines and flight view retain the stock check.</summary>
    public static bool BypassOrbitVisibilityCheck(bool isMapView, bool isActive) =>
        isMapView && isActive;
}

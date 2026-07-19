using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

internal readonly record struct TerrainHeightSnapshot(
    double MaximumSurfaceRadius,
    Func<Vector3d, double> HeightFromDirectionCcf);

/// <summary>Main-thread capture seam for KSA public CPU terrain sampler. The
/// generation guard prevents a worker retaining a celestial across save load.</summary>
internal static class TerrainHeightReader
{
    [System.Runtime.CompilerServices.MethodImpl(
        System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    internal static TerrainHeightSnapshot? TryCapture(string bodyId)
    {
        long generation = ModServices.BindingGeneration;
        var system = Universe.CurrentSystem;
        if (system is null) return null;
        Celestial? body = null;
        for (int i = 0; i < system.Count; i++)
            if (system.GetIndex(i) is Celestial candidate && candidate.Id == bodyId)
            {
                body = candidate;
                break;
            }
        if (body is null) return null;

        if (ModServices.BindingGeneration != generation
            || !ReferenceEquals(Universe.CurrentSystem, system))
            return null;
        double meanRadius = body.MeanRadius;
        // The exact heightmap maximum plus the game's accurate 16k-direction
        // procedural estimate forms the broad phase. Pad the estimate so narrow
        // procedural features do not sit exactly on its sampled boundary.
        double maximumHeight = Math.Max(
            body.MaxTerrainRadius - meanRadius, body.MaxTerrainHeightApprox);
        double maximumSurfaceRadius = meanRadius
            + Math.Max(0.0, maximumHeight) * 1.1 + 100.0;

        double Height(Vector3d direction)
        {
            if (ModServices.BindingGeneration != generation
                || !ReferenceEquals(Universe.CurrentSystem, system))
                return double.NaN;
            double length = direction.Length();
            if (!(length > 0) || !double.IsFinite(length)) return double.NaN;
            try
            {
                var dir = new double3(
                    direction.X / length, direction.Y / length, direction.Z / length);
                return body.GetTerrainHeightFromDirCcf(dir, accurate: true);
            }
            catch { return double.NaN; }
        }
        return new TerrainHeightSnapshot(maximumSurfaceRadius, Height);
    }
}

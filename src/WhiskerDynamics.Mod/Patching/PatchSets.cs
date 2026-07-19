using System.Reflection;
using System.Runtime.CompilerServices;
using HarmonyLib;
using WhiskerDynamics.Compatibility.Patching;

namespace WhiskerDynamics.Mod.Patching;

/// <summary>A named, ordered patch transaction and the game-member contracts that
/// must all validate before any of its patch classes are applied.</summary>
internal sealed record PatchSetDefinition(
    string Name,
    IReadOnlyList<TargetSpec> Targets,
    IReadOnlyList<Type> PatchTypes);

/// <summary>The disabled-status panel remains independently applicable before the
/// gameplay compatibility transaction runs.</summary>
internal static class PanelPatchSet
{
    internal static PatchSetDefinition Create() => new(
        "panel",
        Array.AsReadOnly(PanelTargets.Panel),
        Array.AsReadOnly<Type>([typeof(Patches.StatusPanelPatch)]));
}

/// <summary>The existing all-or-nothing gameplay transaction. Keep the first call to
/// <see cref="Create"/> inside ModMain's type-drift guard: resolving any target type may
/// run <see cref="GameplayTargets"/>' static initializer.</summary>
internal static class GameplayPatchSet
{
    internal static IReadOnlyList<Type> PatchTypes { get; } = Array.AsReadOnly<Type>(
    [
        typeof(Patches.CelestialRailsPatch),
        typeof(Patches.VesselRailsPatch),
        typeof(Patches.ClusterFollowerRailsPatch),
        typeof(Patches.CommitCanaryPatch),
        typeof(Patches.LiveGravityPatch),
        typeof(Patches.FaultPausePatch),
        typeof(Patches.FaultPauseReadPatch),
        typeof(Patches.SoiHandoffPatch),
        typeof(Patches.SoiEncounterPlanAuthorityPatch),
        typeof(Patches.SoiEscapePlanAuthorityPatch),
        typeof(Patches.SoiRecalculateFlightPlanScopePatch),
        typeof(Patches.NavigationTargetPatch),
        typeof(Patches.SaveSidecarWritePatch),
        typeof(Patches.SaveSidecarRestorePatch),
        typeof(Patches.SaveDrillPatch),
        typeof(Patches.MapFramePatch),
        typeof(Patches.OrbitCacheUpdatePatch),
        typeof(Patches.VesselLinePatch),
        typeof(Patches.CelestialLinePatch),
        typeof(Patches.PatchMarkerPatch),
        typeof(Patches.SoiIndicatorPatch),
        typeof(Patches.OrbitHoverPatch),
        typeof(Patches.BurnNodePatch),
        typeof(Patches.BurnGizmoPatch),
        typeof(Patches.BurnClickPatch),
    ]);

    internal static PatchSetDefinition Create() => new(
        "gameplay",
        Array.AsReadOnly(GameplayTargets.Gameplay),
        PatchTypes);
}

/// <summary>Harmony application plus the eager patch-method JIT check used by the
/// compatibility transaction.</summary>
internal static class HarmonyPatchActivation
{
    internal static void Apply(Harmony harmony, Type patchType) =>
        harmony.CreateClassProcessor(patchType).Patch();

    internal static void ApplyAndWarm(Harmony harmony, IReadOnlyList<Type> patchTypes) =>
        ApplyAndWarm(
            patchTypes,
            patchType => Apply(harmony, patchType),
            method => RuntimeHelpers.PrepareMethod(method.MethodHandle));

    internal static void ApplyAndWarm(
        IReadOnlyList<Type> patchTypes,
        Action<Type> apply,
        Action<MethodInfo> prepare)
    {
        foreach (var patchType in patchTypes)
        {
            apply(patchType);
            foreach (var method in PatchMethods(patchType))
            {
                // Open generic helpers cannot be prepared without supplying a concrete
                // instantiation. Their non-generic patch callers compile them on demand.
                if (!method.ContainsGenericParameters) prepare(method);
            }
        }
    }

    private static MethodInfo[] PatchMethods(Type patchType) => patchType.GetMethods(
        BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
}

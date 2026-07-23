using System.Reflection;
using WhiskerDynamics.Compatibility;
using WhiskerDynamics.Compatibility.Patching;
using WhiskerDynamics.Mod.Patches;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.Mod.Tests.Patching;

public class PatchSetTests
{
    [Fact]
    public void Compatibility_contract_is_owned_by_the_dedicated_assembly()
    {
        Assembly modAssembly = typeof(ModMain).Assembly;
        Assembly compatibilityAssembly = typeof(GameBuildPolicy).Assembly;

        Assert.NotSame(modAssembly, compatibilityAssembly);
        Assert.Equal("WhiskerDynamics.Compatibility", compatibilityAssembly.GetName().Name);
        Assert.Same(compatibilityAssembly, typeof(TargetSpec).Assembly);
        Assert.Same(compatibilityAssembly, typeof(PatchValidator).Assembly);
        Assert.Same(compatibilityAssembly, typeof(EnumContract).Assembly);
        Assert.Contains(modAssembly.GetReferencedAssemblies(),
            reference => reference.Name == "WhiskerDynamics.Compatibility");
        var metadata = compatibilityAssembly
            .GetCustomAttributes<AssemblyMetadataAttribute>()
            .Where(attribute => attribute.Key == "VerifiedKsaBuild");
        var verifiedBuild = Assert.Single(metadata);
        Assert.Equal("2026.7.8.4980", verifiedBuild.Value);
        Assert.Equal(verifiedBuild.Value, GameBuildPolicy.VerifiedBuild);
        Assert.True(GameBuildPolicy.IsVerified(verifiedBuild.Value!));
        Assert.False(GameBuildPolicy.IsVerified("2026.7.5.4892"));
    }

    [Fact]
    public void Compatibility_catalog_shape_is_characterized()
    {
        Assert.Single(PanelTargets.Panel);
        Assert.Equal(263, GameplayTargets.Gameplay.Length);
        Assert.True(EnumContract.Validate(out var mismatches));
        Assert.Empty(mismatches);
    }

    [Fact]
    public void Panel_patch_set_keeps_its_independent_target_and_patch()
    {
        var panel = PanelPatchSet.Create();

        Assert.Equal("panel", panel.Name);
        Assert.Equal(PanelTargets.Panel, panel.Targets);
        Assert.Equal([typeof(StatusPanelPatch)], panel.PatchTypes);
    }

    [Fact]
    public void Gameplay_patch_order_is_characterized()
    {
        Type[] expected =
        [
            typeof(CelestialRailsPatch),
            typeof(VesselRailsPatch),
            typeof(ClusterFollowerRailsPatch),
            typeof(CommitCanaryPatch),
            typeof(LiveGravityPatch),
            typeof(FaultPausePatch),
            typeof(FaultPauseReadPatch),
            typeof(SoiHandoffPatch),
            typeof(SoiEncounterPlanAuthorityPatch),
            typeof(SoiEscapePlanAuthorityPatch),
            typeof(SoiRecalculateFlightPlanScopePatch),
            typeof(NavigationTargetPatch),
            typeof(SaveSidecarWritePatch),
            typeof(SaveSidecarRestorePatch),
            typeof(SaveDrillPatch),
            typeof(MapFramePatch),
            typeof(OrbitCacheUpdatePatch),
            typeof(VesselLinePatch),
            typeof(CelestialLinePatch),
            typeof(PatchMarkerPatch),
            typeof(SoiIndicatorPatch),
            typeof(OrbitHoverPatch),
            typeof(BurnNodePatch),
            typeof(BurnGizmoPatch),
            typeof(BurnClickPatch),
        ];

        Assert.Equal(25, GameplayPatchSet.PatchTypes.Count);
        Assert.Equal(expected, GameplayPatchSet.PatchTypes);
        Assert.Equal(expected.Length, expected.Distinct().Count());
        Assert.DoesNotContain(typeof(StatusPanelPatch), GameplayPatchSet.PatchTypes);
    }

    [Fact]
    public void Activation_applies_each_type_before_warming_it_and_preserves_type_order()
    {
        Type[] patchTypes = [typeof(FirstPatch), typeof(SecondPatch)];
        var events = new List<(string Action, Type Type, string? Method)>();

        HarmonyPatchActivation.ApplyAndWarm(
            patchTypes,
            type => events.Add(("apply", type, null)),
            method => events.Add(("prepare", method.DeclaringType!, method.Name)));

        int firstApply = events.FindIndex(entry =>
            entry is ("apply", var type, null) && type == typeof(FirstPatch));
        int secondApply = events.FindIndex(entry =>
            entry is ("apply", var type, null) && type == typeof(SecondPatch));
        Assert.Equal(0, firstApply);
        Assert.True(secondApply > firstApply);
        Assert.All(
            events.Where(entry => entry.Action == "prepare" && entry.Type == typeof(FirstPatch)),
            entry => Assert.InRange(events.IndexOf(entry), firstApply + 1, secondApply - 1));
        Assert.All(
            events.Where(entry => entry.Action == "prepare" && entry.Type == typeof(SecondPatch)),
            entry => Assert.True(events.IndexOf(entry) > secondApply));

        Assert.Contains(events, entry =>
            entry is ("prepare", var type, nameof(FirstPatch.PublicPatch))
            && type == typeof(FirstPatch));
        Assert.Contains(events, entry =>
            entry is ("prepare", var type, "PrivatePatch")
            && type == typeof(FirstPatch));
        Assert.DoesNotContain(events, entry => entry.Method == nameof(FirstPatch.GenericHelper));
        Assert.DoesNotContain(events, entry => entry.Method == nameof(FirstPatch.InstanceHelper));
    }

    [Fact]
    public void Activation_stops_when_apply_throws()
    {
        var applied = new List<Type>();

        Assert.Throws<InvalidOperationException>(() =>
            HarmonyPatchActivation.ApplyAndWarm(
                [typeof(FirstPatch), typeof(SecondPatch)],
                type =>
                {
                    applied.Add(type);
                    throw new InvalidOperationException("apply failed");
                },
                _ => throw new Xunit.Sdk.XunitException("warming should not run")));

        Assert.Equal([typeof(FirstPatch)], applied);
    }

    [Fact]
    public void Activation_stops_before_the_next_type_when_warming_throws()
    {
        var applied = new List<Type>();

        Assert.Throws<InvalidOperationException>(() =>
            HarmonyPatchActivation.ApplyAndWarm(
                [typeof(FirstPatch), typeof(SecondPatch)],
                applied.Add,
                _ => throw new InvalidOperationException("warm failed")));

        Assert.Equal([typeof(FirstPatch)], applied);
    }

    private sealed class FirstPatch
    {
        public static void PublicPatch() { }
        private static void PrivatePatch() { }
        public static void GenericHelper<T>() { }
        public void InstanceHelper() { }
    }

    private static class SecondPatch
    {
        public static void Patch() { }
    }
}

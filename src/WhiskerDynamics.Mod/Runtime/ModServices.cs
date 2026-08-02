using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Runtime;

public enum ModStatus { Booting, DisabledByUser, DisabledIncompatible, WaitingForSystem, Active, DisabledFault }

public static class ModServices
{
    internal readonly record struct BoundServices(
        long Generation, RailsService Rails, VesselRegistry Vessels);

    internal sealed record Binding(
        object System, long Generation, RailsService Rails, VesselRegistry Vessels)
    {
        internal BoundServices Services => new(Generation, Rails, Vessels);
    }

    private static readonly object BindGate = new();
    private static object? _boundSystem;
    private static Binding? _binding;
    private static long _bindingGeneration;

    internal static long BindingGeneration =>
        System.Threading.Volatile.Read(ref _bindingGeneration);

    private static int _enabled;
    private static int _status = (int)ModStatus.Booting;
    private static int _faultPauseLogged;
    private static long _nextFaultPauseErrorMs;
    public static bool Enabled
    {
        get => Volatile.Read(ref _enabled) != 0;
        internal set => Volatile.Write(ref _enabled, value ? 1 : 0);
    }
    public static ModStatus Status
    {
        get => (ModStatus)Volatile.Read(ref _status);
        internal set => Volatile.Write(ref _status, (int)value);
    }
    public static ModConfig Config { get; internal set; } = new();
    internal static BodySettingsCatalog BodySettings { get; set; } = BodySettingsCatalog.Empty;
    internal static MapTrajectoryState MapTrajectory { get; } = new();
    public static IReadOnlyList<string> Mismatches { get; internal set; } = [];
    public static RailsService? Rails => Volatile.Read(ref _binding)?.Rails;
    public static VesselRegistry? Vessels => Volatile.Read(ref _binding)?.Vessels;

    internal static bool TryGetBound(out BoundServices services)
    {
        var binding = Volatile.Read(ref _binding);
        if (binding is not null
            && binding.Generation == BindingGeneration
            && ReferenceEquals(Volatile.Read(ref _boundSystem), binding.System)
            && Status == ModStatus.Active
            && !binding.Rails.AuthorityFaulted)
        {
            services = binding.Services;
            return true;
        }
        services = default;
        return false;
    }

    internal static bool IsBindingCurrent(long generation, RailsService rails)
    {
        var binding = Volatile.Read(ref _binding);
        return BindingTokenMatches(
            generation, BindingGeneration, rails, binding?.Rails,
            Status == ModStatus.DisabledFault || rails.AuthorityFaulted)
            && binding!.Generation == generation;
    }

    /// <summary>Runs a containment/fatal side effect only while the exact immutable
    /// service pair that observed the failure is still published. The bind gate makes
    /// the validation and side effect one transaction against rebind/fatal-disable.</summary>
    internal static bool RunIfBindingCurrent(BoundServices captured, Action sideEffect) =>
        BindingSideEffectPolicy.TryRun(BindGate, () =>
        {
            var binding = Volatile.Read(ref _binding);
            return binding is not null
                && binding.Generation == captured.Generation
                && binding.Generation == BindingGeneration
                && ReferenceEquals(binding.Rails, captured.Rails)
                && ReferenceEquals(binding.Vessels, captured.Vessels)
                && ReferenceEquals(_boundSystem, binding.System)
                && Status == ModStatus.Active;
        }, sideEffect);

    internal static bool BindingTokenMatches(
        long capturedGeneration, long currentGeneration,
        object capturedService, object? currentService, bool faulted) =>
        !faulted
        && capturedGeneration == currentGeneration
        && ReferenceEquals(capturedService, currentService);

    /// <summary>Bind physics services to the loaded CelestialSystem; rebinds automatically
    /// after a save/new-game replaces Universe.CurrentSystem. Returns false while no system
    /// is loaded. Binding at a positive elapsed time integrates rails from epoch to
    /// the current simulation time.</summary>
    public static bool EnsureBound() => EnsureBound(out _);

    internal static bool EnsureBound(out BoundServices services)
    {
        services = default;
        // A fault latch is permanent for the session; consumers gate on EnsureBound()'s return.
        if (Status == ModStatus.DisabledFault) return false;
        var system = Universe.CurrentSystem;
        if (system is null) return false;
        var fastBinding = Volatile.Read(ref _binding);
        if (ReferenceEquals(Volatile.Read(ref _boundSystem), system)
            && fastBinding is not null
            && !fastBinding.Rails.AuthorityFaulted)
        {
            fastBinding.Rails.CaptureEquatorialPolesOnMainThread();
            if (ReferenceEquals(Volatile.Read(ref _binding), fastBinding)
                && ReferenceEquals(Volatile.Read(ref _boundSystem), system)
                && fastBinding.Generation == BindingGeneration
                && Status == ModStatus.Active
                && !fastBinding.Rails.AuthorityFaulted)
            {
                services = fastBinding.Services;
                return true;
            }
        }
        lock (BindGate)
        {
            if (Status == ModStatus.DisabledFault) return false;
            system = Universe.CurrentSystem;
            if (system is null) return false;
            var rebound = Volatile.Read(ref _binding);
            if (ReferenceEquals(_boundSystem, system) && rebound is not null)
            {
                if (rebound.Rails.AuthorityFaulted)
                {
                    FatalDisable("published authoritative rails service faulted");
                    return false;
                }
                rebound.Rails.CaptureEquatorialPolesOnMainThread();
                services = rebound.Services;
                return true;
            }
            RailsService? replacement = null;
            Binding? retiring = null;
            try
            {
                retiring = Interlocked.Exchange(ref _binding, null);
                retiring?.Rails.Dispose();
                // Dispose joins the old worker. Only after it is quiescent may the
                // binding/session generations advance and their shared state reset;
                // the replacement worker therefore starts in the fresh generation.
                System.Threading.Interlocked.Increment(ref _bindingGeneration);
                ResetSessionStatics();
                var sw = System.Diagnostics.Stopwatch.StartNew();
                ModConfig.LogRepairs(Config.NormalizeWorkload(), "system bind");
                replacement = RailsService.CreateFromGameData(
                    Config, GameConstants.ReadFromGame(), BodySettings);
                replacement.SetAuthorityFaultHandler(HandleRailsAuthorityFailure);
                replacement.CaptureEquatorialPolesOnMainThread();
                replacement.PrepareAuthorityAt(Universe.GetElapsedSimTime().Seconds());
                replacement.ThrowIfAuthorityFaulted();
                var replacementVessels = new VesselRegistry(Config, replacement);
                long generation = BindingGeneration;
                var published = new Binding(system, generation, replacement, replacementVessels);
                replacement.PublishIfAuthorityHealthy(() =>
                {
                    Volatile.Write(ref _boundSystem, system);
                    Volatile.Write(ref _binding, published);
                    Status = ModStatus.Active;
                });
                services = published.Services;
                replacement = null;
                ModLog.Info($"bound to system in {sw.ElapsedMilliseconds} ms");
                return true;
            }
            catch (Exception e)
            {
                if (retiring is not null)
                {
                    try { retiring.Rails.Dispose(); }
                    catch (Exception shutdownError)
                    {
                        ModLog.Error("failed old-service shutdown contained: " + shutdownError);
                    }
                }
                if (replacement is not null)
                {
                    try { replacement.Dispose(); }
                    catch (Exception shutdownError)
                    {
                        ModLog.Error($"failed replacement shutdown contained: {shutdownError}");
                    }
                }
                FatalDisable($"system bind failed: {e}");
                return false;
            }
        }
    }

    /// <summary>Forces the next EnsureBound to rebind. A save load keeps the
    /// CurrentSystem OBJECT (the game deserializes into it) while replacing the vessel
    /// population and moving sim time in either direction — semantically a system swap,
    /// so it gets the same response: fresh rails (bit-reproducible from t=0, and the
    /// keep-behind pruning of the old rails cannot strand a backward jump), a fresh
    /// vessel registry, and the session statics sweep.</summary>
    public static void InvalidateBinding(string reason)
    {
        lock (BindGate)
        {
            if (_boundSystem is null) return;
            _boundSystem = null;
            System.Threading.Interlocked.Increment(ref _bindingGeneration);
            ModLog.Info($"binding invalidated ({reason}) - rebinding");
        }
    }

    /// <summary>Session statics sweep: one-shot / telemetry statics that must re-arm
    /// when the sim is replaced under the process (system swap or save load). Everything
    /// here is log/panel material — resetting is observability hygiene, never dynamics.</summary>
    private static void ResetSessionStatics()
    {
        Patches.CelestialRailsPatch.ResetSessionStatics();
        Patches.VesselRailsPatch.ResetSessionStatics();
        Patches.ClusterFollowerRailsPatch.ResetSessionStatics();
        Patches.LiveGravityPatch.ResetSessionStatics();
        Patches.SoiHandoffPatch.ResetSessionStatics();
        Patches.SoiEncounterPlanAuthorityPatch.ResetSessionStatics();
        Patches.NavigationTargetPatch.ResetSessionStatics();
        TrajectoryOverlay.ResetSessionStatics();
        Ui.StatusPanel.ResetSessionStatics();
        ModMain.ResetSessionStatics();
        Patches.MapFramePatch.ResetSessionStatics();
        Patches.VesselLinePatch.ResetSessionStatics();
        OverlayBuffer.ResetSessionStatics();
        Ui.BurnPlannerPanel.ResetSessionStatics();
        Ui.OrbitAnalyserPanel.ResetSessionStatics();
        FlightPlans.ResetSessionStatics();
        FrameManager.ResetSessionStatics();
        Ui.FramesPanel.ResetSessionStatics();
        Ui.SettingsPanel.ResetSessionStatics();
        CelestialCurves.ResetSessionStatics();
        Ui.LagrangeOverlay.ResetSessionStatics();
        Patches.CelestialLinePatch.ResetSessionStatics();
        Patches.PatchMarkerPatch.ResetSessionStatics();
        Patches.SoiIndicatorPatch.ResetSessionStatics();
        Patches.OrbitHoverPatch.ResetSessionStatics();
        Patches.BurnNodePatch.ResetSessionStatics();
        Patches.BurnGizmoPatch.ResetSessionStatics();
        Patches.BurnClickPatch.ResetSessionStatics();
        Patches.KittenRemovalPatch.ResetSessionStatics();
    }

    public static void FatalDisable(string reason)
    {
        lock (BindGate)
        {
            Enabled = false;
            if (Status == ModStatus.DisabledFault) return;
            Status = ModStatus.DisabledFault;
            Interlocked.Increment(ref _bindingGeneration);
            var binding = Volatile.Read(ref _binding);
            FatalShutdownPolicy.Execute(
                binding is null ? static () => { } : binding.Rails.Dispose,
                static (phase, error) =>
                    ModLog.Error($"fatal-disable {phase} contained: {error}"));
            ModLog.Error("MOD DISABLED: " + reason);
        }
    }

    private static void HandleRailsAuthorityFailure(
        RailsService failedService, Exception failure)
    {
        lock (BindGate)
        {
            var binding = Volatile.Read(ref _binding);
            if (binding is null
                || !ReferenceEquals(binding.Rails, failedService)
                || binding.Generation != BindingGeneration
                || Status == ModStatus.DisabledFault)
                return;
            FatalDisable("authoritative rails background worker failed: " + failure);
        }
    }

    /// <summary>Main-thread render seam for the runtime fault latch. Faulted dynamics
    /// never transition to stock propagation: the game remains paused, and the speed
    /// setter patch rejects later attempts to resume the simulation.</summary>
    internal static void EnforceFaultPauseOnMainThread()
    {
        try
        {
            if (!Patches.FaultPauseEnforcer.TryEnforce(
                    Status, speed => Universe.SetSimulationSpeed(speed)))
                return;
            if (Interlocked.CompareExchange(ref _faultPauseLogged, 1, 0) == 0)
                ModLog.Error("simulation paused: authoritative dynamics faulted; reload is required");
        }
        catch (Exception e)
        {
            long now = Environment.TickCount64;
            long next = Interlocked.Read(ref _nextFaultPauseErrorMs);
            if (now >= next && Interlocked.CompareExchange(
                    ref _nextFaultPauseErrorMs, now + 5000, next) == next)
                ModLog.Error("failed to enforce dynamics fault pause: " + e);
        }
    }
}

/// <summary>KSA-free lock/validation policy shared by every failure side-effect seam.</summary>
internal static class BindingSideEffectPolicy
{
    internal static bool TryRun(object gate, Func<bool> stillCurrent, Action sideEffect)
    {
        lock (gate)
        {
            if (!stillCurrent()) return false;
            sideEffect();
            return true;
        }
    }
}

/// <summary>One fatal-shutdown ordering: revoke display publications, retain the last
/// authoritative staged caches, then stop the rails worker.</summary>
internal static class FatalShutdownPolicy
{
    internal static void Execute(
        Action stopRails,
        Action<string, Exception> reportContained)
    {
        OverlayWorker.ResetSessionStatics();
        OverlayBuffer.ResetSessionStatics();
        try { stopRails(); }
        catch (Exception error) { reportContained("rails shutdown", error); }
    }
}

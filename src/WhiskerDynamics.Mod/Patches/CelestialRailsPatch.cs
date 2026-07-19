using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Seam 3: after CelestialUpdateTask.Run stages the stock Kepler state into
/// the public NewStateVectors field, overwrite position/velocity for modeled bodies
/// with the precomputed n-body rail state (converted parent-relative, Ecl->Cci axes).
/// Runs on game job threads — RailsService serializes internally.
/// TrueAnomaly is reused from the stock evaluation (map/UI cosmetic).</summary>
[HarmonyPatch(typeof(CelestialUpdateTask), nameof(CelestialUpdateTask.Run))]
internal static class CelestialRailsPatch
{
    internal static long OverrideCount; // panel telemetry

    /// <summary>In-game verification telemetry (whiskerdynamics.log is the only observable while
    /// the game runs): one epoch-equality line per body at first override — at t~0 the
    /// rails ARE the Kepler positions by construction, so |dr| must be tiny or the frame
    /// conversion is wrong — then stock-vs-rails drift lines throttled to one per body
    /// per 30 s wall clock (drift growth stays observable over warp, log growth bounded).</summary>
    private static readonly RailsTelemetry Telemetry = new(driftPeriodMs: 30_000);

    // One-shot consumption probe: Celestial.UpdateFromTaskResults writes NewStateVectors
    // into Orbit._stateVectors (readable via public `ref readonly Orbit.StateVectors`), so
    // one tick after we stage an override the committed state must equal it bit-for-bit.
    // Proves in-game that the game consumes the staged overrides.
    // Probe state deliberately uses NO KSA types (double/double3): a KSA-typed static
    // field would force KSA.dll to load with the type, breaking the offline suite's
    // KSA-free reflection over this class (and the lazy-load containment shape).
    private static string? _probeBodyId;
    private static double _probeTime;
    private static Brutal.Numerics.double3 _probePos, _probeVel;
    private static int _probeState; // 0=unarmed, 1=armed, 2=done

    /// <summary>Statics sweep: a rebind / save load replaces the
    /// sim under this patch — re-arm the per-body epoch/drift telemetry and the one-shot
    /// consumption probe so the new session evidences itself. Races with in-flight job
    /// ticks are benign (worst case one duplicate or one dropped log line). KSA-free,
    /// like every static here.</summary>
    internal static void ResetSessionStatics()
    {
        Telemetry.Reset();
        _probeBodyId = null;
        System.Threading.Volatile.Write(ref _probeState, 0);
    }

    private static void ProbeConsumption(Celestial celestial, in StateVectors staged)
    {
        if (_probeState == 2) return;
        if (_probeState == 0)
        {
            if (System.Threading.Interlocked.CompareExchange(ref _probeState, 1, 0) != 0) return;
            _probeTime = staged.StateTime.Seconds();
            _probePos = staged.PositionCci;
            _probeVel = staged.VelocityCci;
            _probeBodyId = celestial.Id; // written last: subsequent ticks key on the id
            return;
        }
        if (!string.Equals(celestial.Id, _probeBodyId, StringComparison.Ordinal)) return;
        ref readonly var committed = ref celestial.Orbit.StateVectors;
        if (committed.StateTime.Seconds() == _probeTime)
        {
            double dr = (committed.PositionCci - _probePos).Length();
            double dv = (committed.VelocityCci - _probeVel).Length();
            ModLog.Info($"consumption check '{_probeBodyId}': committed Orbit.StateVectors at "
                + $"t={committed.StateTime.Seconds():F1} s vs last staged rail override: |dr|={dr:E3} m, "
                + $"|dv|={dv:E3} m/s (0 = consumed verbatim)");
            _probeState = 2;
            return;
        }
        // Commit window not aligned yet: re-arm on this tick's staged value.
        _probeTime = staged.StateTime.Seconds();
        _probePos = staged.PositionCci;
        _probeVel = staged.VelocityCci;
    }

    static void Postfix(CelestialUpdateTask __instance, Celestial ____readOnlyCelestial)
    {
        if (!ModServices.Enabled) return;
        ModServices.BoundServices services = default;
        bool bindingCaptured = false;
        try
        {
            if (!ModServices.EnsureBound(out services)) return;
            bindingCaptured = true;
            if (__instance.NewStateVectors is not { } stock) return; // zero-dt tick staged nothing
            var rails = services.Rails;
            double t = stock.StateTime.Seconds();
            if (!rails.TryGetParentRelativeEcl(____readOnlyCelestial.Id, t, out var relPos, out var relVel))
                throw new InvalidOperationException(
                    $"no authoritative celestial state for '{____readOnlyCelestial.Id}' at t={t:R}");
            var cce2Cci = ____readOnlyCelestial.Orbit.Parent.GetCce2Cci();
            var railPosCci = FrameAdapter.EclToCci(relPos, cce2Cci);
            var railVelCci = FrameAdapter.EclToCci(relVel, cce2Cci);
            var staged = new StateVectors(stock.StateTime, railPosCci, railVelCci, stock.TrueAnomaly);
            __instance.NewStateVectors = staged;
            System.Threading.Interlocked.Increment(ref OverrideCount);
            ProbeConsumption(____readOnlyCelestial, in staged);

            var line = Telemetry.Classify(____readOnlyCelestial.Id, Environment.TickCount64);
            if (line != RailsTelemetry.Line.None)
            {
                double dr = (railPosCci - stock.PositionCci).Length();
                double dv = (railVelCci - stock.VelocityCci).Length();
                ModLog.Info($"{(line == RailsTelemetry.Line.Epoch ? "epoch check" : "drift")} "
                    + $"'{____readOnlyCelestial.Id}': |dr|={dr:E3} m, |dv|={dv:E3} m/s vs stock Kepler "
                    + $"at t={t:F1} s ({t / 86400.0:F2} d)");
                // Frame diagnostic (once per body per bind, with the epoch line): the
                // parent's Ecl->Cci quaternion. Frame conventions are the top silent-
                // wrongness risk; this makes the game's actual transform auditable
                // offline (and lets scenario tooling convert Ecl states to save-file
                // Cci states without re-deriving the game's rotation chain).
                if (line == RailsTelemetry.Line.Epoch)
                    ModLog.Info($"frame check '{____readOnlyCelestial.Id}': parent cce2Cci = "
                        + $"({cce2Cci.X:R}, {cce2Cci.Y:R}, {cce2Cci.Z:R}, {cce2Cci.W:R})");
            }
        }
        catch (Exception e)
        {
            // Celestial rails are systemic: a failure here means every consumer sees
            // inconsistent positions. Disable the whole mod, not one body.
            if (bindingCaptured)
                ModServices.RunIfBindingCurrent(services,
                    () => ModServices.FatalDisable(
                        $"celestial rails failed for '{____readOnlyCelestial.Id}': {e}"));
        }
    }
}

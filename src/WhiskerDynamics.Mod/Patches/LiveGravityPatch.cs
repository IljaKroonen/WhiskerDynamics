using System.Reflection;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Seam 2: postfix on the static PhysicsStates.ComputeDerivatives — the one
/// non-inline-marked surface every live path funnels through (the velocity-Verlet
/// unconstrained step twice per substep AND the Bepu constrained prestep, via the
/// delegating instance overload). Adds the third-body correction in bubble axes:
/// coupled sources use g_j(vessel) - g_j(parent), while prescribed one-way sources
/// use direct g_j(vessel). Both come from the same GravityModel source set Seam 1 flies
/// (by design: ONE field model for both seams). Thrust, drag, buoyancy, collisions, joints
/// stay stock.
///
/// Physics: live states are parent-relative; stock applies
/// single-mu gravity at the vessel and, in Cci frames, subtracts the single-mu term at
/// the bubble origin — that subtraction stays STOCK on purpose, because the live origin
/// is propagated on a Kepler rail whose actual acceleration IS that term. The parent
/// itself flies an n-body rail (Seam 3), so the true parent-relative acceleration is
/// gN(vessel) - a_parent; adding the delta above turns stock's g1(vessel) into exactly
/// that. Coupled sources use the tidal form because they accelerate the parent rail;
/// prescribed sources do not, so their vessel acceleration stays direct.
///
/// HOT PATH: runs on job threads at physics substep rate. RailsService.ThirdBodyDelta
/// serves it from a per-parent third-body snapshot — zero allocations and zero Gate
/// acquisitions on cache hits (refresh at most once per sim-second per parent). Cached
/// source positions advance quadratically; a jerk-estimated remainder-to-distance
/// bound refreshes close encounters. origin.Time matches stock's environment sampling cadence. Only by-`in`/
/// by-value parameters are requested by
/// name; the ReadOnlySpan parameter is never touched.</summary>
[HarmonyPatch]
internal static class LiveGravityPatch
{
    /// <summary>Panel telemetry (m/s^2). Written by job threads, read by the render
    /// thread; plain double (stale-after-rebind or torn-read-free on x64 is fine for
    /// a display value — the next live substep overwrites it).</summary>
    internal static double LastDeltaMagnitude;

    // One-shot + 30 s-throttled magnitude lines: whiskerdynamics.log is the only observable
    // while the game runs (gate 1 reads the delta magnitude from here). The one-shot
    // is re-armed per bound sim by the statics sweep; the wall-clock throttle
    // needs no reset.
    private static int _pathLogged;
    private static long _nextLogMs;

    /// <summary>Statics sweep: re-arm the one-shot path-evidence line and zero
    /// the stale panel readout (the new sim overwrites it on its first live substep).</summary>
    internal static void ResetSessionStatics()
    {
        System.Threading.Volatile.Write(ref _pathLogged, 0);
        LastDeltaMagnitude = 0;
    }

    static MethodBase TargetMethod() =>
        typeof(PhysicsStates)
            .GetMethods(BindingFlags.Public | BindingFlags.Static)
            .Single(m => m.Name == "ComputeDerivatives");

    static void Postfix(ref Disturbances __result, in BubbleOrigin origin,
        in KinematicStates kinematic, double3 positionOriginBub)
    {
        if (!ModServices.Enabled) return;
        ModServices.BoundServices services = default;
        bool bindingCaptured = false;
        try
        {
            if (!ModServices.EnsureBound(out services)) return;
            bindingCaptured = true;
            var rails = services.Rails;
            // The frame anchor is the body the ORIGIN orbits: its stock mirror defines
            // the origin subtraction the mixed direct/tidal delta composes with. Every
            // modeled parent, backbone or restricted, has an authoritative numerical
            // rail and uses this correction.
            if (origin.Parent is not Astronomical parentBody)
                throw new InvalidOperationException("live gravity origin has no modeled astronomical parent");
            if (!rails.IsModeled(parentBody.Id))
                throw new InvalidOperationException(
                    $"live gravity parent '{parentBody.Id}' has no authoritative modeled state");

            // Vessel position relative to the parent, bubble axes -> Ecl axes. Uses the
            // caller-provided origin position (current or next half-step) exactly like
            // the stock gravity term; GetBub2Cce covers both Cci and rotating Ccf axes.
            double3 vesselParentBub = positionOriginBub + kinematic.PositionPhys;
            doubleQuat bub2Cce = origin.GetBub2Cce();
            Vector3d relEcl = FrameAdapter.BubToEcl(vesselParentBub, bub2Cce);

            Vector3d deltaEcl = ResolvePerturbation(
                rails, parentBody.Id, relEcl, origin.Time.Seconds());

            __result.AddAccelPhys(FrameAdapter.EclToBub(deltaEcl, bub2Cce));
            double magnitude = deltaEcl.Length();
            LastDeltaMagnitude = magnitude;

            if (System.Threading.Interlocked.CompareExchange(ref _pathLogged, 1, 0) == 0)
            {
                ModLog.Info($"seam2 live third-body field active (first |delta|={magnitude:E2} m/s^2, "
                    + $"parent {parentBody.Id}, t={origin.Time.Seconds():F1} s)");
            }
            else
            {
                long now = Environment.TickCount64;
                long next = System.Threading.Interlocked.Read(ref _nextLogMs);
                if (now >= next
                    && System.Threading.Interlocked.CompareExchange(ref _nextLogMs, now + 30_000, next) == next)
                    ModLog.Info($"seam2 |delta|={magnitude:E2} m/s^2 "
                        + $"(parent {parentBody.Id}, t={origin.Time.Seconds():F1} s)");
            }
        }
        catch (Exception e)
        {
            // Systemic surface (every live vessel flows through here): disable loudly.
            if (bindingCaptured)
                ModServices.RunIfBindingCurrent(services,
                    () => ModServices.FatalDisable($"live gravity patch failed: {e}"));
        }
    }

    /// <summary>A cold or contended hot-path miss joins the single authoritative
    /// refresh flight; the current substep never retains stock-only gravity.</summary>
    internal static Vector3d ResolvePerturbation(
        RailsService rails, string parentId, Vector3d relativePosition, double time) =>
        rails.TryVesselPerturbation(parentId, relativePosition, time, out var delta)
            ? delta
            : rails.VesselPerturbation(parentId, relativePosition, time);
}

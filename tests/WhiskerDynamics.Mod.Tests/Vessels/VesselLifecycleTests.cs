using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Patches;
using WhiskerDynamics.Mod.Patching;

namespace WhiskerDynamics.Mod.Tests.Vessels;

/// <summary>
/// Tests vessel reseeding, teleport detection, eviction, persistence eligibility, and
/// commit-canary decisions without game types.
/// </summary>
public class VesselLifecycleTests
{
    // --- Seeding decision -------------------------------------------------------
    //
    // The patches run AFTER the stock freefall branch staged Situation.Freefall into
    // the new props, so the kernel receives the PRE-TICK (last committed) situation:
    // "was this vessel already on rails when the tick began?"

    [Fact]
    public void First_seen_vessel_is_seeded()
    {
        Assert.Equal(VesselLifecycle.Seeding.Seed,
            VesselLifecycle.Decide(alreadyTracked: false, wasFreefallAtTickStart: true, reseedPending: false, sameVehicleInstance: true));
        Assert.Equal(VesselLifecycle.Seeding.Seed,
            VesselLifecycle.Decide(alreadyTracked: false, wasFreefallAtTickStart: false, reseedPending: false, sameVehicleInstance: true));
    }

    [Fact]
    public void Vessel_that_stayed_on_rails_keeps_its_predictor()
    {
        Assert.Equal(VesselLifecycle.Seeding.Keep,
            VesselLifecycle.Decide(alreadyTracked: true, wasFreefallAtTickStart: true, reseedPending: false, sameVehicleInstance: true));
    }

    [Fact]
    public void Vessel_returning_from_live_physics_is_reseeded()
    {
        // Burns, collisions, docking all end with a non-Freefall committed situation;
        // the first rails tick after that must discard the stale predictor.
        Assert.Equal(VesselLifecycle.Seeding.Reseed,
            VesselLifecycle.Decide(alreadyTracked: true, wasFreefallAtTickStart: false, reseedPending: false, sameVehicleInstance: true));
    }

    [Fact]
    public void Pending_reseed_forces_a_reseed_even_in_uninterrupted_freefall()
    {
        // Set while live physics owned the vessel and the predictor was stale:
        // continuous Freefall alone is insufficient evidence that the predictor is current.
        Assert.Equal(VesselLifecycle.Seeding.Reseed,
            VesselLifecycle.Decide(alreadyTracked: true, wasFreefallAtTickStart: true, reseedPending: true, sameVehicleInstance: true));
    }

    [Fact]
    public void A_new_vehicle_instance_with_a_tracked_id_is_reseeded_not_inherited()
    {
        // Stale-Id teleport guard: a vessel can be destroyed and a NEW one spawn in
        // Freefall under the same Id before eviction fires. Pre-tick situation and
        // staged situation are both Freefall, and the canary is structurally blind
        // (committed == predictor by construction) — instance identity is the only
        // reliable signal, and it must force a reseed instead of staging the dead
        // vessel's trajectory onto the newcomer.
        Assert.Equal(VesselLifecycle.Seeding.Reseed,
            VesselLifecycle.Decide(alreadyTracked: true, wasFreefallAtTickStart: true, reseedPending: false, sameVehicleInstance: false));
    }

    // --- Eviction (registry hygiene) ----------------------------------------------

    [Fact]
    public void Vessel_seen_recently_is_not_evicted()
    {
        Assert.False(VesselLifecycle.ShouldEvict(nowMs: 10_000, lastSeenMs: 9_000, evictAfterMs: 30_000));
        Assert.False(VesselLifecycle.ShouldEvict(nowMs: 40_000, lastSeenMs: 10_000, evictAfterMs: 30_000)); // exactly at bound: keep
    }

    [Fact]
    public void Vessel_unseen_past_the_bound_is_evicted()
    {
        Assert.True(VesselLifecycle.ShouldEvict(nowMs: 40_001, lastSeenMs: 10_000, evictAfterMs: 30_000));
    }

    // --- Same-instance teleport guard (Keep path) -----------------------------------

    [Fact]
    public void Smooth_conic_drift_growth_is_not_a_teleport()
    {
        // Large accumulated drift is not a teleport when it grows continuously.
        Assert.False(VesselLifecycle.IsTeleportJump(previousDrift: 1.0e7, drift: 1.0e7 + 500, jumpMeters: 1e6));
    }

    [Fact]
    public void Stock_plan_rebuild_sawtooth_reset_is_not_a_teleport()
    {
        // Stock rebuilds its flight plan from our committed states, resetting the drift
        // reference DOWNWARD — only upward jumps can mean a discontinuity.
        Assert.False(VesselLifecycle.IsTeleportJump(previousDrift: 1.0e7, drift: 12.0, jumpMeters: 1e6));
    }

    [Fact]
    public void Single_tick_upward_jump_is_a_teleport()
    {
        // Vehicle.Teleport-class events move the staged stock state discontinuously
        // under the SAME vehicle instance; staging the old predictor would snap the
        // vessel back, and the canary would verify the snap-back as consistent.
        Assert.True(VesselLifecycle.IsTeleportJump(previousDrift: 40.0, drift: 3.0e6, jumpMeters: 1e6));
        Assert.True(VesselLifecycle.IsTeleportJump(previousDrift: 0.0, drift: 1.1e6, jumpMeters: 1e6)); // right after seed
    }

    [Fact]
    public void Jump_on_a_parent_transition_tick_is_overridden_not_adopted()
    {
        // SOI patch transitions may introduce a Kepler-vs-rails discontinuity; the
        // continuous predictor remains authoritative during the transition.
        Assert.False(VesselLifecycle.AdoptStagedJump(parentTransitionTick: true));
        Assert.True(VesselLifecycle.AdoptStagedJump(parentTransitionTick: false));
    }

    // --- FlightPlan re-osculation trigger ------------------------------------------
    //
    // Conic drift resets when the plan is re-derived, so decisions use instantaneous
    // drift rather than a remembered peak.

    [Fact]
    public void No_refresh_while_drift_is_small_and_period_has_not_elapsed()
    {
        Assert.False(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 12.0, driftThresholdMeters: 1000,
            time: 100.0, lastRefreshTime: 0.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Drift_beyond_the_threshold_triggers_a_refresh()
    {
        Assert.True(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 1000.5, driftThresholdMeters: 1000,
            time: 100.0, lastRefreshTime: 0.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Elapsed_period_triggers_a_refresh_even_with_tiny_drift()
    {
        // The periodic path re-arms FlightPlan.ExpiryGameTime so a vessel left alone
        // for sim-months keeps a valid, non-expired plan.
        Assert.True(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 0.001, driftThresholdMeters: 1000,
            time: 600.5, lastRefreshTime: 0.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Sawtooth_reset_reading_does_not_trigger()
    {
        // Tick n-1 read 9.9e2 m; stock rebuilt its plan from our committed state and
        // the readout snapped to ~0. The kernel must see "fresh conic, nothing to do",
        // not chase the previous peak.
        Assert.False(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 0.4, driftThresholdMeters: 1000,
            time: 300.0, lastRefreshTime: 250.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Thresholds_are_strict_bounds()
    {
        // Exactly AT either bound: keep — the readout snapping to ~threshold on the
        // refresh tick itself must not re-trigger forever.
        Assert.False(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 1000.0, driftThresholdMeters: 1000,
            time: 0.0, lastRefreshTime: 0.0, refreshPeriodSeconds: 600));
        Assert.False(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 0.0, driftThresholdMeters: 1000,
            time: 600.0, lastRefreshTime: 0.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Time_before_the_last_refresh_never_periodic_triggers()
    {
        // Reseed stamps LastRefreshTime = seed time; a clock that reads earlier than
        // the stamp (paused sim, defensive) must not arm the periodic path.
        Assert.False(VesselLifecycle.ShouldRefreshOsculation(
            conicDrift: 0.0, driftThresholdMeters: 1000,
            time: -50.0, lastRefreshTime: 0.0, refreshPeriodSeconds: 600));
    }

    // --- Eviction process-liveness guard ---------------------------------------------

    [Fact]
    public void Normal_sweep_cadence_is_not_a_stall()
    {
        // Sweeps run at 1 s cadence while anything stages; even a landed-only stretch
        // where GetOrSeed goes quiet for exactly the eviction bound is not a stall.
        Assert.False(VesselLifecycle.SweepGapMeansStall(nowMs: 11_000, lastSweepMs: 10_000, evictAfterMs: 30_000));
        Assert.False(VesselLifecycle.SweepGapMeansStall(nowMs: 40_000, lastSweepMs: 10_000, evictAfterMs: 30_000)); // at bound
    }

    [Fact]
    public void Sweep_gap_beyond_the_eviction_bound_means_the_process_stalled()
    {
        // LastSeenMs stamps stop for EVERY entry while the process stalls (debugger
        // break, OS suspend, long hitch) — wall clock keeps running, so the first
        // post-stall sweep would otherwise mass-evict live vessels.
        Assert.True(VesselLifecycle.SweepGapMeansStall(nowMs: 40_001, lastSweepMs: 10_000, evictAfterMs: 30_000));
    }

    // --- Sidecar write eligibility -------------------------------------------------
    //
    // Only vessels whose predictor is CURRENT may persist to the sidecar: a landed or
    // stock-owned vessel's predictor froze at its last rails stretch, and persisting
    // that ghost state would "restore" the vessel onto a stale trajectory. The recency
    // signal is max(LastRefreshTime, LastStagedTime): refresh stamps cover the
    // single-vessel path, staging stamps cover cluster followers (which never book
    // re-osculation refreshes — on refresh stamps alone, live followers like
    // Gemini7/Hunter would be dropped from the sidecar). 2x the refresh period
    // separates "actively on rails" from "frozen".

    [Fact]
    public void Fresh_rails_vessel_is_sidecar_eligible()
    {
        Assert.True(VesselLifecycle.SidecarEligible(reseedPending: false, hasPredictor: true,
            seedTime: 0.0, lastActiveTime: 2000.0, elapsedSeconds: 2200.0, refreshPeriodSeconds: 600));
        // Boundary: exactly 2x the period behind still counts as current.
        Assert.True(VesselLifecycle.SidecarEligible(reseedPending: false, hasPredictor: true,
            seedTime: 0.0, lastActiveTime: 1000.0, elapsedSeconds: 2200.0, refreshPeriodSeconds: 600));
        // Cluster-follower shape: no refresh ever booked, but staged THIS tick — the
        // caller passes max(refresh, staged), which is the staging time itself.
        Assert.True(VesselLifecycle.SidecarEligible(reseedPending: false, hasPredictor: true,
            seedTime: 0.0, lastActiveTime: 172804.0, elapsedSeconds: 172804.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Pending_or_predictorless_vessels_are_not_eligible()
    {
        // ReseedPending = predictor known stale after a live-physics interval.
        Assert.False(VesselLifecycle.SidecarEligible(reseedPending: true, hasPredictor: true,
            seedTime: 0.0, lastActiveTime: 1000.0, elapsedSeconds: 1000.0, refreshPeriodSeconds: 600));
        Assert.False(VesselLifecycle.SidecarEligible(reseedPending: false, hasPredictor: false,
            seedTime: 0.0, lastActiveTime: 1000.0, elapsedSeconds: 1000.0, refreshPeriodSeconds: 600));
    }

    [Fact]
    public void Stale_activity_or_future_seed_is_not_eligible()
    {
        // Landed vessel: both stamps froze when it left rails; sim time ran on.
        Assert.False(VesselLifecycle.SidecarEligible(reseedPending: false, hasPredictor: true,
            seedTime: 0.0, lastActiveTime: 1000.0, elapsedSeconds: 2201.0, refreshPeriodSeconds: 600));
        // Seeded after the save instant: StateAt(elapsed) would throw (query before start).
        Assert.False(VesselLifecycle.SidecarEligible(reseedPending: false, hasPredictor: true,
            seedTime: 3000.0, lastActiveTime: 3000.0, elapsedSeconds: 2000.0, refreshPeriodSeconds: 600));
    }

    // --- Sidecar restore guard -------------------------------------------------------
    //
    // Applied on a vessel's FIRST GetOrSeed after a load, right after the stock seed.
    // The restored save's vessels stage within a tick of the sidecar epoch, so a real
    // restore has seed ~ epoch; a LATER new vessel under a recycled id must never
    // inherit the saved trajectory. The 1 ms tolerance admits save-time rounding while
    // remaining far below a simulation tick.

    [Fact]
    public void Restore_applies_at_and_near_the_seed_time()
    {
        Assert.True(VesselLifecycle.ShouldRestoreFromSidecar(epochSeconds: 1000.0, seedTime: 1000.0, windowSeconds: 30));
        Assert.True(VesselLifecycle.ShouldRestoreFromSidecar(epochSeconds: 970.0, seedTime: 1000.0, windowSeconds: 30)); // at window
        // The in-game case: XML rounded the restored time DOWN by ~4.8e-5 s.
        Assert.True(VesselLifecycle.ShouldRestoreFromSidecar(epochSeconds: 172804.188447685, seedTime: 172804.1884, windowSeconds: 30));
    }

    [Fact]
    public void Restore_is_refused_outside_the_window()
    {
        // A new vessel launched long after the save under the same id.
        Assert.False(VesselLifecycle.ShouldRestoreFromSidecar(epochSeconds: 1000.0, seedTime: 5000.0, windowSeconds: 30));
        Assert.False(VesselLifecycle.ShouldRestoreFromSidecar(epochSeconds: 969.9, seedTime: 1000.0, windowSeconds: 30));
        // An epoch meaningfully in the FUTURE of the seed would make the restored
        // predictor throw on its first staging query — refuse anything beyond the
        // XML-rounding allowance.
        Assert.False(VesselLifecycle.ShouldRestoreFromSidecar(epochSeconds: 1000.01, seedTime: 1000.0, windowSeconds: 30));
    }

}

/// <summary>Pins the registration step: all three Seam 1 patch classes must stay in
/// the gameplay patch set (applied inside the guarded try only after ALL gameplay
/// targets validate). Reflection-only on mod types — no KSA type is loaded offline.</summary>
public class Seam1RegistrationTests
{
    [Fact]
    public void Seam1_patches_are_registered_as_gameplay_patches()
    {
        Assert.Contains(typeof(VesselRailsPatch), GameplayPatchSet.PatchTypes);
        Assert.Contains(typeof(ClusterFollowerRailsPatch), GameplayPatchSet.PatchTypes);
        Assert.Contains(typeof(CommitCanaryPatch), GameplayPatchSet.PatchTypes);
    }
}

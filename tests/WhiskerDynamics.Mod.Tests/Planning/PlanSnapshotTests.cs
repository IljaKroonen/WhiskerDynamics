using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Planning;

/// <summary>Plan snapshot ("the plan is a snapshot"): capture ordering,
/// the diverged flag's transitions, the panel-edit mirrors (set/move/remove with the
/// meta-style time tolerance), the reconcile match rule, the sidecar round-trip with
/// per-field sanitation, and the overlay's snapshot kernels (sample/fold start, the
/// restamp-vs-resample rule). KSA-free; BurnPlannerPanel and TrajectoryOverlay are
/// the thin adapters over these rules.</summary>
public class PlanSnapshotTests
{
    private static FlightPlanModel NewPlan(double createdAt = 1000, double length = 86400) =>
        new() { CreatedAtSeconds = createdAt, LengthSeconds = length };

    private static PlanSnapshot Snap(double epoch = 1000, params (double T, Vector3d Dv)[] burns) =>
        PlanSnapshot.Capture(epoch,
            new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 7500, 0)),
            "Earth", burns.Select(b => Burn(b.T, b.Dv)));

    private static Vector3d Dv(double x) => new(x, 0, 0);
    private static PlanSnapshotBurn Burn(double time, Vector3d dv,
        string? parent = "Earth") => new(time, dv, parent);

    // ---- capture / diverged transitions ----

    [Fact]
    public void Capture_sorts_burns_ascending_and_precomputes_times()
    {
        var snapshot = PlanSnapshot.Capture(1000, default, "Earth",
        [
            Burn(5000.0, Dv(2), "Luna"),
            Burn(2000.0, Dv(1), "Earth"),
            Burn(9000.0, Dv(3), "Mars"),
        ]);
        Assert.Equal([2000.0, 5000.0, 9000.0], snapshot.Burns.Select(b => b.TimeSeconds));
        Assert.Equal([2000.0, 5000.0, 9000.0], snapshot.BurnTimes);
        Assert.Equal(1, snapshot.Burns[0].DeltaVVlf.X);
        Assert.Equal(["Earth", "Luna", "Mars"],
            snapshot.Burns.Select(b => b.BasisParentId));
        Assert.Equal("Earth", snapshot.AnchorParentId);
    }

    [Fact]
    public void Hourly_reanchor_matches_geometry_but_burn_or_engine_changes_do_not()
    {
        var original = Snap(1000, (5000.0, Dv(1)));
        var reanchored = PlanSnapshot.Capture(5000,
            new StateVector(new Vector3d(8e6, 0, 0), new Vector3d(0, 7000, 0)),
            "Luna", [Burn(5000.0, Dv(1))]);
        var edited = Snap(5000, (5000.0, Dv(2)));
        var parentChanged = PlanSnapshot.Capture(5000, default, "Earth",
            [Burn(5000.0, Dv(1), "Luna")]);
        var engineChanged = PlanSnapshot.Capture(5000, default, "Earth",
            [Burn(5000.0, Dv(1))], new EngineScalars(1000, 3000, 2));

        Assert.True(original.GeometryMatches(reanchored));
        Assert.False(original.GeometryMatches(edited));
        Assert.False(original.GeometryMatches(parentChanged));
        Assert.False(original.GeometryMatches(engineChanged));
    }

    [Fact]
    public void SetSnapshot_installs_and_resets_diverged_MarkDiverged_flags()
    {
        var plan = NewPlan();
        Assert.Null(plan.Snapshot);
        Assert.False(plan.Diverged);
        plan.MarkDiverged(); // divergence may precede the first capture
        Assert.True(plan.Diverged);
        var snapshot = Snap();
        plan.SetSnapshot(snapshot);
        Assert.Same(snapshot, plan.Snapshot);
        Assert.False(plan.Diverged); // capture/rebase clears the flag
        plan.MarkDiverged();
        Assert.True(plan.Diverged);
        Assert.Same(snapshot, plan.Snapshot); // divergence never mutates the snapshot itself
    }

    // ---- panel-edit mirrors ----

    [Fact]
    public void SnapshotSetBurn_adds_and_replaces_at_the_time_slot()
    {
        var plan = NewPlan();
        plan.SnapshotSetBurn(5000.0, Dv(1)); // no snapshot yet: no-op, not a crash
        Assert.Null(plan.Snapshot);
        plan.SetSnapshot(Snap(1000, (5000.0, Dv(1))));
        plan.SnapshotSetBurn(5000.0 + BurnIdentityPolicy.ToleranceSeconds * 0.5,
            Dv(9), "Luna");
        var burn = Assert.Single(plan.Snapshot!.Burns); // replaced, not duplicated
        Assert.Equal(9, burn.DeltaVVlf.X);
        Assert.Equal("Luna", burn.BasisParentId);
        plan.SnapshotSetBurn(7000.0, Dv(4), "Mars");
        Assert.Equal(2, plan.Snapshot!.Burns.Count);
        Assert.Equal("Mars", plan.Snapshot.Burns[1].BasisParentId);
        // A failed re-resolution is represented by null and must keep the matched
        // record's known parent rather than erasing it.
        plan.SnapshotSetBurn(7000.0, Dv(5), basisParentId: null);
        Assert.Equal("Mars", plan.Snapshot.Burns[1].BasisParentId);
        Assert.Equal(1000, plan.Snapshot!.EpochSeconds); // the anchor never moves on a mirror
    }

    [Fact]
    public void SnapshotSetBurn_records_and_an_ordinary_write_clears_the_display_vector()
    {
        var plan = NewPlan();
        plan.SetSnapshot(Snap(1000, (5000.0, Dv(1))));
        plan.SnapshotSetBurn(5000.0, Dv(2), "Earth",
            displayDvVlf: new Vector3d(2, 1, 0));
        Assert.Equal(new Vector3d(2, 1, 0),
            Assert.Single(plan.Snapshot!.Burns).DisplayDvVlf);
        plan.SnapshotSetBurn(5000.0, Dv(3), "Earth");
        Assert.Null(Assert.Single(plan.Snapshot!.Burns).DisplayDvVlf);
    }

    [Fact]
    public void SnapshotSetDisplayDv_refreshes_only_a_material_change_and_never_churns_the_version()
    {
        var plan = NewPlan();
        plan.SnapshotSetDisplayDv(5000.0, Dv(1), 0.01); // no snapshot yet: no-op, not a crash
        plan.SetSnapshot(Snap(1000, (5000.0, Dv(1))));
        long installed = plan.Version;

        // Within tolerance of the stock components: no churn.
        plan.SnapshotSetDisplayDv(5000.0, new Vector3d(1, 0.005, 0), 0.01);
        Assert.Null(Assert.Single(plan.Snapshot!.Burns).DisplayDvVlf);
        Assert.Equal(installed, plan.Version);

        plan.SnapshotSetDisplayDv(5000.0, new Vector3d(1, 5, 0), 0.01);
        Assert.Equal(new Vector3d(1, 5, 0),
            Assert.Single(plan.Snapshot!.Burns).DisplayDvVlf);
        Assert.True(plan.Version > installed);
        long refreshed = plan.Version;

        // Steady state now compares against the recorded vector.
        plan.SnapshotSetDisplayDv(5000.0, new Vector3d(1, 5, 0.005), 0.01);
        Assert.Equal(new Vector3d(1, 5, 0),
            Assert.Single(plan.Snapshot!.Burns).DisplayDvVlf);
        Assert.Equal(refreshed, plan.Version);

        plan.SnapshotSetDisplayDv(9999.0, Dv(9), 0.01); // no burn at the slot: no-op
        Assert.Equal(refreshed, plan.Version);
    }

    [Fact]
    public void SnapshotDisplayDvFor_is_guarded_by_the_live_stock_components()
    {
        var plan = NewPlan();
        Assert.Null(plan.SnapshotDisplayDvFor(5000.0, Dv(1))); // no snapshot yet
        plan.SetSnapshot(Snap(1000, (5000.0, Dv(1))));
        Assert.Null(plan.SnapshotDisplayDvFor(5000.0, Dv(1))); // none recorded
        plan.SnapshotSetBurn(5000.0, Dv(1), "Earth",
            displayDvVlf: new Vector3d(0.5, 0.8, 0));
        Assert.Equal(new Vector3d(0.5, 0.8, 0),
            plan.SnapshotDisplayDvFor(5000.0, Dv(1)));
        Assert.Null(plan.SnapshotDisplayDvFor(5000.0, Dv(2))); // stock-side edit: guard drops it
        Assert.Null(plan.SnapshotDisplayDvFor(7000.0, Dv(1))); // no burn at the slot
    }

    [Fact]
    public void Snapshot_move_drops_the_display_vector_with_its_time_slot()
    {
        var plan = NewPlan();
        plan.SetSnapshot(Snap(1000, (5000.0, Dv(1))));
        plan.SnapshotSetBurn(5000.0, Dv(1), "Earth",
            displayDvVlf: new Vector3d(0.9, 0.4, 0));
        plan.SnapshotMoveBurn(5000.0, 6000.0);
        var moved = Assert.Single(plan.Snapshot!.Burns);
        Assert.Equal(6000.0, moved.TimeSeconds);
        Assert.Null(moved.DisplayDvVlf); // the realization is time-dependent
    }

    [Fact]
    public void SnapshotMoveBurn_rekeys_keeps_dv_and_evicts_the_target_slot()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(5000.0, Dv(7), "Earth"), Burn(6000.0, Dv(8), "Mars")]));
        plan.SnapshotMoveBurn(5000.0, 6000.0, "Luna");
        var moved = Assert.Single(plan.Snapshot!.Burns); // the old occupant of 6000 s is gone
        Assert.Equal(6000.0, moved.TimeSeconds);
        Assert.Equal(7, moved.DeltaVVlf.X);
        Assert.Equal("Luna", moved.BasisParentId);
        // A within-tolerance move must not delete the moved burn itself.
        plan.SnapshotMoveBurn(6000.0,
            6000.0 + BurnIdentityPolicy.ToleranceSeconds * 0.5, basisParentId: null);
        Assert.Equal("Luna", Assert.Single(plan.Snapshot!.Burns).BasisParentId);
        plan.SnapshotMoveBurn(123.0, 456.0); // no burn at the source: no-op
        Assert.Single(plan.Snapshot!.Burns);
    }

    [Fact]
    public void Deferred_time_move_preserves_parent_through_same_frame_dv_until_capture_resolves()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(5000.0, Dv(7), "Earth")]));
        plan.MarkDiverged();

        plan.SnapshotMoveBurnDeferred(5000.0, 7000.0);
        var moved = Assert.Single(plan.Snapshot!.Burns);
        Assert.Equal(7000.0, moved.TimeSeconds);
        Assert.Equal("Earth", moved.BasisParentId);
        Assert.True(plan.SnapshotParentRefreshPending(7000.0));

        // A DV edit can land in the same UI frame. Its burn-aware resolver still sees
        // the old stock chain, so pending ownership must defeat the stale answer.
        plan.SnapshotSetBurn(7000.0, Dv(9), "Mars");
        moved = Assert.Single(plan.Snapshot.Burns);
        Assert.Equal(9, moved.DeltaVVlf.X);
        Assert.Equal("Earth", moved.BasisParentId);
        Assert.True(plan.SnapshotParentRefreshPending(7000.0));

        // A non-null answer from a dirty stock patch chain is still stale. The
        // whole-scan readiness barrier must preserve both parent and pending key.
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            7000.0, "Mars", patchChainReady: false));
        Assert.True(plan.SnapshotParentRefreshPending(7000.0));
        Assert.Equal("Luna", plan.SnapshotParentFromCapture(
            7000.0, "Luna", patchChainReady: true));
        Assert.False(plan.SnapshotParentRefreshPending(7000.0));
        Assert.Equal("Luna", Assert.Single(plan.Snapshot.Burns).BasisParentId);
        Assert.True(plan.Diverged);
    }

    [Fact]
    public void Add_refresh_schedule_survives_divergence_before_stock_rebuild()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(2000, Dv(1)), Burn(4000, Dv(4))]));
        plan.SnapshotSetBurn(3000, Dv(3), "Luna",
            markDownstreamParentsPending: true);

        Assert.False(plan.Diverged);
        Assert.False(plan.SnapshotParentRefreshPending(2000));
        Assert.False(plan.SnapshotParentRefreshPending(3000));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(2000));
        Assert.Equal("Luna", plan.SnapshotKnownParentAt(3000));
        plan.MarkDiverged();
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            4000, "Venus", patchChainReady: false));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal("Mars", plan.SnapshotParentFromCapture(4000, "Mars"));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(2000));
        Assert.False(plan.SnapshotParentRefreshPending(4000));
    }

    [Fact]
    public void Dv_refresh_schedule_survives_divergence_before_stock_rebuild()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(2000, Dv(1)), Burn(3000, Dv(2)), Burn(4000, Dv(3))]));
        plan.SnapshotSetBurn(3000, Dv(9), "Luna",
            markDownstreamParentsPending: true);

        Assert.False(plan.Diverged);
        Assert.False(plan.SnapshotParentRefreshPending(2000));
        Assert.False(plan.SnapshotParentRefreshPending(3000));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal("Luna", plan.SnapshotKnownParentAt(3000));
        plan.MarkDiverged();
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            4000, "Venus", patchChainReady: false));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal("Mars", plan.SnapshotParentFromCapture(4000, "Mars"));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(2000));
        Assert.False(plan.SnapshotParentRefreshPending(4000));
    }

    [Fact]
    public void Remove_refresh_schedule_survives_divergence_before_stock_rebuild()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(2000, Dv(1)), Burn(3000, Dv(2)), Burn(4000, Dv(3))]));
        plan.SnapshotRemoveBurn(3000);

        Assert.False(plan.Diverged);
        Assert.False(plan.SnapshotParentRefreshPending(2000));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        plan.MarkDiverged();
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            4000, "Venus", patchChainReady: false));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal("Luna", plan.SnapshotParentFromCapture(4000, "Luna"));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(2000));
        Assert.False(plan.SnapshotParentRefreshPending(4000));
    }

    [Fact]
    public void Move_refresh_schedule_survives_divergence_and_evicts_target_slot()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
        [
            Burn(2000, Dv(1), "Earth"),
            Burn(3000, Dv(2), "Earth"),
            Burn(3500, Dv(3), "Mars"),
            Burn(4000, Dv(4), "Earth"),
        ]));
        plan.SnapshotMoveBurnDeferred(3000, 3500);

        Assert.False(plan.Diverged);
        Assert.Equal(3, plan.Snapshot!.Burns.Count); // target-slot occupant was evicted
        Assert.False(plan.SnapshotParentRefreshPending(2000));
        Assert.True(plan.SnapshotParentRefreshPending(3500));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal(2, plan.Snapshot.Burns.Single(b => b.TimeSeconds == 3500).DeltaVVlf.X);
        plan.MarkDiverged();
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            3500, "Venus", patchChainReady: false));
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            4000, "Venus", patchChainReady: false));
        Assert.True(plan.SnapshotParentRefreshPending(3500));
        Assert.True(plan.SnapshotParentRefreshPending(4000));
        Assert.Equal("Luna", plan.SnapshotParentFromCapture(3500, "Luna"));
        Assert.Equal("Mars", plan.SnapshotParentFromCapture(4000, "Mars"));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(2000));
        Assert.False(plan.SnapshotParentRefreshPending(3500));
        Assert.False(plan.SnapshotParentRefreshPending(4000));
    }

    [Fact]
    public void Deferred_parent_key_follows_moves_and_is_cleared_by_remove_or_snapshot_replace()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(5000.0, Dv(1), "Earth")]));
        plan.SnapshotMoveBurnDeferred(5000.0, 6000.0);
        Assert.False(plan.SnapshotParentRefreshPending(5000.0));
        Assert.True(plan.SnapshotParentRefreshPending(6000.0));

        plan.SnapshotMoveBurnDeferred(6000.0, 7000.0);
        Assert.False(plan.SnapshotParentRefreshPending(6000.0));
        Assert.True(plan.SnapshotParentRefreshPending(7000.0));
        Assert.Equal("Earth", Assert.Single(plan.Snapshot!.Burns).BasisParentId);

        plan.SnapshotRemoveBurn(7000.0);
        Assert.False(plan.SnapshotParentRefreshPending(7000.0));
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(8000.0, Dv(1), "Earth")]));
        plan.SnapshotMoveBurnDeferred(8000.0, 9000.0);
        Assert.True(plan.SnapshotParentRefreshPending(9000.0));
        plan.SetSnapshot(Snap());
        Assert.False(plan.SnapshotParentRefreshPending(9000.0));
    }

    [Fact]
    public void Rebase_snapshot_can_rearm_provisional_parents_before_immediate_divergence()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
        [
            Burn(4000, Dv(1), "Earth"),
            Burn(5000, Dv(2), "Earth"),
            Burn(6000, Dv(3), "Earth"),
        ]));
        plan.SnapshotMarkParentRefresh([5000]);
        PlanSnapshotPersistenceState preRebase =
            plan.CaptureSnapshotPersistenceState();
        Assert.True(preRebase.ParentRefreshPendingAt(5000));
        // Even a non-null resolver answer remains provisional for a captured pending
        // key; Rebase retains the last known parent and carries the key across replace.
        Assert.Equal("Earth", preRebase.ParentForReplacement(5000, "Venus"));

        var rebased = PlanSnapshot.Capture(2000, default, "Earth",
        [
            Burn(4000, Dv(1), preRebase.ParentForReplacement(4000, "Earth")),
            Burn(5000, Dv(2), preRebase.ParentForReplacement(5000, "Venus")),
            Burn(6000, Dv(3), preRebase.ParentForReplacement(6000, "Mars")),
        ]);

        plan.SetSnapshot(rebased);
        plan.SnapshotMarkParentRefresh(preRebase.PendingParentRefreshTimes);
        Assert.False(plan.SnapshotParentRefreshPending(4000));
        Assert.True(plan.SnapshotParentRefreshPending(5000));
        Assert.False(plan.SnapshotParentRefreshPending(6000));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(5000));
        Assert.Equal("Mars", plan.SnapshotKnownParentAt(6000));

        plan.MarkDiverged();
        Assert.Equal("Earth", plan.SnapshotParentFromCapture(
            5000, "Venus", patchChainReady: false));
        Assert.Equal("Luna", plan.SnapshotParentFromCapture(5000, "Luna"));
        Assert.Equal("Earth", plan.SnapshotKnownParentAt(4000));
        Assert.False(plan.SnapshotParentRefreshPending(5000));
        Assert.False(plan.SnapshotParentRefreshPending(6000));
    }

    [Fact]
    public void SnapshotRemoveBurn_drops_the_slot()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [Burn(5000.0, Dv(1), "Earth"), Burn(6000.0, Dv(2), "Luna")]));
        plan.SnapshotRemoveBurn(5000.0);
        var survivor = Assert.Single(plan.Snapshot!.Burns);
        Assert.Equal(6000.0, survivor.TimeSeconds);
        Assert.Equal("Luna", survivor.BasisParentId);
    }

    // ---- reconcile match rule ----

    [Fact]
    public void SnapshotBurnsMatch_requires_same_set_within_tolerances()
    {
        var plan = NewPlan();
        Assert.False(plan.SnapshotBurnsMatch([])); // no snapshot: capture is due
        plan.SetSnapshot(Snap(1000, (5000.0, Dv(1)), (6000.0, Dv(2))));
        // Order-insensitive: the stock scan may hand burns in any order.
        Assert.True(plan.SnapshotBurnsMatch([Burn(6000.0, Dv(2)), Burn(5000.0, Dv(1))]));
        Assert.False(plan.SnapshotBurnsMatch([Burn(5000.0, Dv(1))])); // count
        Assert.False(plan.SnapshotBurnsMatch(
            [Burn(5000.0, Dv(1)), Burn(6000.0 + BurnIdentityPolicy.ToleranceSeconds * 2, Dv(2))])); // time
        Assert.False(plan.SnapshotBurnsMatch(
            [Burn(5000.0, Dv(1)), Burn(6000.0, Dv(2 + FlightPlanModel.DvMatchTolerance * 2))])); // dv
        Assert.True(plan.SnapshotBurnsMatch(
            [Burn(5000.0, Dv(1)), Burn(6000.0, Dv(2 + FlightPlanModel.DvMatchTolerance * 0.5))]));
        Assert.False(plan.SnapshotBurnsMatch(
            [Burn(5000.0, Dv(1)), Burn(6000.0, Dv(2), "Luna")]));
    }

    [Fact]
    public void Multi_parent_evidence_signature_is_stable_throttled_and_bounded()
    {
        PlanSnapshotBurn[] first =
        [
            Burn(1000, Dv(1), "Earth"),
            Burn(2000, Dv(2), "Luna"),
            Burn(3000, Dv(3), "Luna"),
        ];
        string signature = Assert.IsType<string>(
            FlightPlanModel.SnapshotParentSignature(first));
        Assert.Equal(signature, FlightPlanModel.SnapshotParentSignature(
        [
            Burn(1100, Dv(99), "Earth"),
            Burn(2200, Dv(88), "Luna"),
            Burn(3300, Dv(77), "Luna"),
        ]));
        Assert.NotEqual(signature, FlightPlanModel.SnapshotParentSignature(
            [Burn(1000, Dv(1), "Earth"), Burn(2000, Dv(2), "Mars")]));
        Assert.Null(FlightPlanModel.SnapshotParentSignature(
            [Burn(1000, Dv(1), "Earth"), Burn(2000, Dv(2), "Earth")]));

        Assert.True(FlightPlanModel.SnapshotParentEvidenceDue(
            nowMs: 500, lastLogMs: 0, signature, lastSignature: null));
        Assert.False(FlightPlanModel.SnapshotParentEvidenceDue(
            nowMs: 500, lastLogMs: 0, signature, lastSignature: signature));
        Assert.True(FlightPlanModel.SnapshotParentEvidenceDue(
            nowMs: 500, lastLogMs: 0, signature, lastSignature: signature,
            samePlan: false));
        // The caller records null even though the single-parent capture is throttled;
        // that transition re-arms the same multi-parent pattern within the same second.
        Assert.False(FlightPlanModel.SnapshotParentEvidenceDue(
            nowMs: 600, lastLogMs: 0, signature: null, lastSignature: signature));
        Assert.True(FlightPlanModel.SnapshotParentEvidenceDue(
            nowMs: 700, lastLogMs: 0, signature, lastSignature: null));
        Assert.True(FlightPlanModel.SnapshotParentEvidenceDue(
            nowMs: 1000, lastLogMs: 0, signature, lastSignature: signature));

        var many = Enumerable.Range(0, 12)
            .Select(i => Burn(1000 + i, Dv(i), i % 2 == 0 ? "Earth" : "Luna"))
            .ToArray();
        string evidence = FlightPlanModel.SnapshotParentEvidence(many, maximumBurns: 1000);
        Assert.Contains("1000.0:Earth", evidence);
        Assert.Contains("1007.0:Luna", evidence);
        Assert.DoesNotContain("1008.0:Earth", evidence);
        Assert.Contains("+4 more", evidence);
        Assert.Equal(string.Empty, FlightPlanModel.SnapshotParentEvidence(
            [Burn(1000, Dv(1), "Earth")]));
    }

    // ---- sidecar round-trip ----

    [Fact]
    public void Sidecar_roundtrips_anchor_burns_and_diverged()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(4321,
            new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 7500, 0)),
            "Earth",
            [
                Burn(5000.0, new Vector3d(1, 2, 3), "Earth"),
                Burn(6000.0, new Vector3d(4, 5, 6), "Luna"),
            ]));
        plan.MarkDiverged();
        var dto = FlightPlans.ToSidecar(plan)!;
        Assert.True(dto.Diverged);
        Assert.Equal(4321, dto.Anchor!.EpochSeconds);
        Assert.Equal("Earth", dto.Anchor.ParentId);
        Assert.Equal(2, dto.SnapshotBurns.Count);
        Assert.Equal(["Earth", "Luna"],
            dto.SnapshotBurns.Select(b => b.BasisParentId));

        var restored = FlightPlans.FromSidecar(dto, nowMs: 0)!;
        Assert.True(restored.Diverged);
        var snapshot = restored.Snapshot!;
        Assert.Equal(4321, snapshot.EpochSeconds);
        Assert.Equal(7e6, snapshot.State.Position.X);
        Assert.Equal(7500, snapshot.State.Velocity.Y);
        Assert.Equal("Earth", snapshot.AnchorParentId);
        Assert.Equal(2, snapshot.Burns.Count);
        Assert.Equal(new Vector3d(4, 5, 6), snapshot.Burns[1].DeltaVVlf);
        Assert.Equal(["Earth", "Luna"],
            snapshot.Burns.Select(b => b.BasisParentId));
    }

    [Fact]
    public void Sidecar_roundtrips_diverged_parent_refresh_schedule_before_stock_recovers()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
        [
            Burn(4000, Dv(1), "Earth"),
            Burn(5000, Dv(2), "Earth"),
            Burn(6000, Dv(3), "Earth"),
        ]));
        plan.MarkDiverged();
        plan.SnapshotMoveBurnDeferred(5000, 5500);

        PlanSnapshotPersistenceState captured =
            plan.CaptureSnapshotPersistenceState();
        Assert.True(captured.Diverged);
        Assert.Equal([5500.0, 6000.0],
            captured.PendingParentRefreshTimes.OrderBy(t => t));
        Assert.All(captured.PendingParentRefreshTimes, pendingTime =>
            Assert.Contains(captured.Snapshot!.Burns,
            b => BurnIdentityPolicy.SameBurn(b.TimeSeconds, pendingTime)));
        Assert.Equal("Earth", captured.KnownParentAt(5500));

        var dto = FlightPlans.ToSidecar(plan)!;
        Assert.False(dto.SnapshotBurns.Single(b => b.TimeSeconds == 4000)
            .BasisParentRefreshPending);
        Assert.True(dto.SnapshotBurns.Single(b => b.TimeSeconds == 5500)
            .BasisParentRefreshPending);
        Assert.True(dto.SnapshotBurns.Single(b => b.TimeSeconds == 6000)
            .BasisParentRefreshPending);

        // The atomic export view owns a copied key set and immutable snapshot
        // reference: a concurrent-style later model replacement cannot tear it or the
        // already-built DTO into mismatched snapshot/divergence/pending pieces.
        plan.SetSnapshot(null);
        Assert.True(captured.Diverged);
        Assert.NotNull(captured.Snapshot);
        Assert.Equal([5500.0, 6000.0],
            captured.PendingParentRefreshTimes.OrderBy(t => t));
        Assert.True(dto.Diverged);
        Assert.Equal(3, dto.SnapshotBurns.Count);

        var restored = FlightPlans.FromSidecar(dto, nowMs: 0)!;
        Assert.True(restored.Diverged);
        Assert.False(restored.SnapshotParentRefreshPending(4000));
        Assert.True(restored.SnapshotParentRefreshPending(5500));
        Assert.True(restored.SnapshotParentRefreshPending(6000));
        Assert.Equal("Earth", restored.SnapshotKnownParentAt(5500));

        // The first clean scan after loading can resolve the complete affected suffix.
        Assert.Equal("Luna", restored.SnapshotParentFromCapture(5500, "Luna"));
        Assert.Equal("Mars", restored.SnapshotParentFromCapture(6000, "Mars"));
        Assert.Equal("Earth", restored.SnapshotKnownParentAt(4000));
        Assert.False(restored.SnapshotParentRefreshPending(5500));
        Assert.False(restored.SnapshotParentRefreshPending(6000));
    }

    [Fact]
    public void Sidecar_without_anchor_loads_null_snapshot_but_keeps_diverged()
    {
        // A missing Anchor property produces a null snapshot.
        var dto = new SidecarPlan { CreatedAtSeconds = 1000, LengthSeconds = 86400, Diverged = true };
        var restored = FlightPlans.FromSidecar(dto, nowMs: 0)!;
        Assert.Null(restored.Snapshot); // lazy capture re-arms on the first on-rails rebuild
        Assert.True(restored.Diverged);
    }

    [Theory]
    [InlineData(double.NaN, 1, 1)] // epoch
    [InlineData(0, double.NaN, 1)] // position component
    [InlineData(0, 1, double.NaN)] // burn dv component
    public void Sidecar_with_broken_anchor_or_burn_drops_the_whole_snapshot(
        double epoch, double posX, double burnX)
    {
        var dto = new SidecarPlan
        {
            CreatedAtSeconds = 1000,
            LengthSeconds = 86400,
            Anchor = new SidecarPlanAnchor
            {
                EpochSeconds = epoch,
                PositionEcl = [posX, 0, 0],
                VelocityEcl = [0, 0, 0],
            },
            SnapshotBurns = [new SidecarSnapshotBurn { TimeSeconds = 5000, X = burnX }],
        };
        Assert.Null(FlightPlans.FromSidecar(dto, nowMs: 0)!.Snapshot);
    }

    [Fact]
    public void Sidecar_with_wrong_length_anchor_arrays_drops_the_snapshot()
    {
        var dto = new SidecarPlan
        {
            CreatedAtSeconds = 1000,
            LengthSeconds = 86400,
            Anchor = new SidecarPlanAnchor { PositionEcl = [1.0, 2.0], VelocityEcl = [0, 0, 0] },
        };
        Assert.Null(FlightPlans.FromSidecar(dto, nowMs: 0)!.Snapshot);
    }

    // ---- propulsion selection / engine scalars (finite-burn estimation) ----

    [Fact]
    public void Propulsion_source_defaults_to_main_and_versions_only_real_switches()
    {
        var plan = NewPlan();
        Assert.Equal(PropulsionSource.MainEngines, plan.PropulsionSource);
        long version = plan.Version;
        plan.PropulsionSource = PropulsionSource.MainEngines;
        Assert.Equal(version, plan.Version);
        plan.PropulsionSource = PropulsionSource.RcsForward;
        Assert.True(plan.Version > version);
    }

    [Fact]
    public void Propulsion_source_roundtrips_for_plan_and_frozen_snapshot()
    {
        var plan = NewPlan();
        plan.PropulsionSource = PropulsionSource.RcsForward;
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, null, [],
            new EngineScalars(1000, 500, 0.2), PropulsionSource.RcsForward));

        var dto = FlightPlans.ToSidecar(plan)!;
        Assert.Equal(nameof(PropulsionSource.RcsForward), dto.PropulsionSource);
        Assert.Equal(nameof(PropulsionSource.RcsForward), dto.Anchor!.PropulsionSource);
        var restored = FlightPlans.FromSidecar(dto, nowMs: 0)!;
        Assert.Equal(PropulsionSource.RcsForward, restored.PropulsionSource);
        Assert.Equal(PropulsionSource.RcsForward, restored.Snapshot!.PropulsionSource);

        // Unknown and absent values fall back to main engines.
        dto.PropulsionSource = 999.ToString();
        dto.Anchor.PropulsionSource = string.Empty;
        restored = FlightPlans.FromSidecar(dto, nowMs: 0)!;
        Assert.Equal(PropulsionSource.MainEngines, restored.PropulsionSource);
        Assert.Equal(PropulsionSource.MainEngines, restored.Snapshot!.PropulsionSource);
    }

    [Fact]
    public void Propulsion_switch_updates_a_diverged_snapshot_but_keeps_plan_world_mass()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, null,
            [Burn(5000, Dv(1), "Luna")],
            new EngineScalars(1200, 3000, 2)));
        plan.MarkDiverged();
        long version = plan.Version;

        plan.SetPropulsionSource(PropulsionSource.RcsForward,
            new EngineScalars(900, 500, 0.2));

        Assert.True(plan.Diverged);
        Assert.True(plan.Version > version);
        Assert.Equal(PropulsionSource.RcsForward, plan.PropulsionSource);
        Assert.Equal(PropulsionSource.RcsForward, plan.Snapshot!.PropulsionSource);
        Assert.Equal(new EngineScalars(1200, 500, 0.2), plan.Snapshot.Engine);
        Assert.Equal("Luna", Assert.Single(plan.Snapshot.Burns).BasisParentId);
    }

    [Fact]
    public void Reapplying_the_same_unavailable_propulsion_is_a_version_noop()
    {
        var plan = NewPlan();
        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, null, []));
        long version = plan.Version;

        plan.SetPropulsionSource(PropulsionSource.MainEngines, default);

        Assert.Equal(version, plan.Version);
        Assert.Null(plan.Snapshot!.Engine);
    }

    private static PlanSnapshot SnapWithEngine(EngineScalars? engine) =>
        PlanSnapshot.Capture(1000,
            new StateVector(new Vector3d(7e6, 0, 0), new Vector3d(0, 7500, 0)),
            "Earth", [Burn(5000.0, Dv(1))], engine);

    [Fact]
    public void Capture_stores_usable_engine_scalars_and_nulls_unusable_ones()
    {
        Assert.Equal(new EngineScalars(1000, 3000, 2),
            SnapWithEngine(new EngineScalars(1000, 3000, 2)).Engine);
        Assert.Null(SnapWithEngine(null).Engine);
        Assert.Null(SnapWithEngine(new EngineScalars(0, 0, 0)).Engine);       // engineless vessel
        Assert.Null(SnapWithEngine(new EngineScalars(1000, double.NaN, 2)).Engine);
    }

    [Fact]
    public void Writer_mirrors_preserve_the_captured_engine()
    {
        var plan = NewPlan();
        plan.SetSnapshot(SnapWithEngine(new EngineScalars(1000, 3000, 2)));
        plan.MarkDiverged();
        plan.SnapshotSetBurn(7000.0, Dv(2), "Luna");
        plan.SnapshotMoveBurn(7000.0, 7500.0, basisParentId: null);
        plan.SnapshotRemoveBurn(5000.0);
        Assert.Equal(new EngineScalars(1000, 3000, 2), plan.Snapshot!.Engine);
        Assert.Equal(PropulsionSource.MainEngines, plan.Snapshot.PropulsionSource);
        Assert.True(plan.Diverged);
        Assert.Equal("Luna", Assert.Single(plan.Snapshot.Burns).BasisParentId);
    }

    [Fact]
    public void Sidecar_roundtrips_engine_scalars_and_no_engine_state()
    {
        var plan = NewPlan();
        plan.SetSnapshot(SnapWithEngine(new EngineScalars(1000, 3000, 2)));
        var dto = FlightPlans.ToSidecar(plan)!;
        Assert.Equal(1000, dto.Anchor!.MassKg);
        Assert.Equal(3000, dto.Anchor.ExhaustVelocity);
        Assert.Equal(2, dto.Anchor.MassFlowRate);
        Assert.Equal(new EngineScalars(1000, 3000, 2),
            FlightPlans.FromSidecar(dto, nowMs: 0)!.Snapshot!.Engine);

        // Zero engine scalars restore as no engine and remain stable through serialization.
        dto.Anchor.MassKg = 0;
        dto.Anchor.ExhaustVelocity = 0;
        dto.Anchor.MassFlowRate = 0;
        var restored = FlightPlans.FromSidecar(dto, nowMs: 0)!;
        Assert.Null(restored.Snapshot!.Engine);
        Assert.Equal(0, FlightPlans.ToSidecar(restored)!.Anchor!.MassKg);
    }

    // ---- overlay kernels ----

    [Fact]
    public void SnapshotSampleStart_branches_at_first_upcoming_burn_or_now_when_diverged()
    {
        double[] burns = [2000.0, 5000.0];
        // Not diverged: PlannedWindowStart semantics (first strictly-future in-window burn).
        Assert.Equal(5000.0, OverlayKernel.SnapshotSampleStart(burns, 3000, 9000, diverged: false));
        Assert.Null(OverlayKernel.SnapshotSampleStart(burns, 6000, 9000, diverged: false));
        Assert.Null(OverlayKernel.SnapshotSampleStart([], 3000, 9000, diverged: false));
        // Diverged: the whole ghost from now, burns or not.
        Assert.Equal(3000.0, OverlayKernel.SnapshotSampleStart(burns, 3000, 9000, diverged: true));
        Assert.Equal(6000.0, OverlayKernel.SnapshotSampleStart([], 6000, 9000, diverged: true));
    }

    [Fact]
    public void Snapshot_reconcile_waits_for_a_clean_parent_scan_before_wholesale_capture()
    {
        Assert.False(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: false, diverged: false, hasSnapshot: false));
        Assert.False(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: false, diverged: false, hasSnapshot: true));
        Assert.False(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: false, diverged: true, hasSnapshot: false));
        Assert.False(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: true, diverged: true, hasSnapshot: true));
        Assert.True(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: true, diverged: false, hasSnapshot: false));
        Assert.True(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: true, diverged: false, hasSnapshot: true));
        Assert.True(OverlayKernel.SnapshotReconcileAllowed(
            patchChainReady: true, diverged: true, hasSnapshot: false));
    }

    [Fact]
    public void SnapshotFoldStart_is_anchor_when_diverged_now_otherwise()
    {
        Assert.Equal(1000.0, OverlayKernel.SnapshotFoldStart(diverged: true, 1000, 3000));
        Assert.Equal(3000.0, OverlayKernel.SnapshotFoldStart(diverged: false, 1000, 3000));
    }

    [Fact]
    public void PlannedResampleDue_restamps_only_while_every_input_holds()
    {
        // The restamp fast path: everything unchanged and geometry remains ahead.
        Assert.False(OverlayKernel.PlannedResampleDue(
            sameSnapshot: true, sameDiverged: true, diverged: false,
            startSeconds: 5000, lastStartSeconds: 5000, sameParent: true, sameFrame: true,
            sameGeometryInputs: true, t0Seconds: 3005, batchT0Seconds: 3000,
            batchEndSeconds: 9000));
        // Any identity change forces the resample (snapshot, diverged flag, parent,
        // frame mode, horizon inputs — a plan-length edit must show while paused).
        Assert.True(OverlayKernel.PlannedResampleDue(false, true, false, 5000, 5000, true, true, true, 3005, 3000, 9000));
        Assert.True(OverlayKernel.PlannedResampleDue(true, false, false, 5000, 5000, true, true, true, 3005, 3000, 9000));
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, false, 5000, 5000, false, true, true, 3005, 3000, 9000));
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, false, 5000, 5000, true, false, true, 3005, 3000, 9000));
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, false, 5000, 5000, true, true, false, 3005, 3000, 9000));
        // Not diverged, the branch point moved (burn crossed/edited): resample now.
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, false, 6000, 5000, true, true, true, 3005, 3000, 9000));
        // Diverged, the start is just "now" and slides every call. Even a warp-sized
        // advance restamps while the deterministic geometry still extends ahead.
        Assert.False(OverlayKernel.PlannedResampleDue(true, true, true, 8000, 3000, true, true, true, 8000, 3000, 9000));
        // Exhausted geometry, invalid geometry, or a backwards clock (load) resamples.
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, true, 9000, 3000, true, true, true, 9000, 3000, 9000));
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, true, 8000, 3000, true, true, true, 8000, 3000, double.NaN));
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, false, 5000, 5000, true, true, true, 2999, 3000, 9000));
        // NaN lastStart (no planned batch cached yet) never restamps.
        Assert.True(OverlayKernel.PlannedResampleDue(true, true, false, 5000, double.NaN, true, true, true, 3005, 3000, 9000));
    }

    [Fact]
    public void Planned_rails_coverage_grows_only_at_geometric_high_water_marks()
    {
        Assert.False(OverlayKernel.PlannedCoverageExpansionDue(8, 7, 30));
        Assert.False(OverlayKernel.PlannedCoverageExpansionDue(8, 12, 30));
        Assert.True(OverlayKernel.PlannedCoverageExpansionDue(8, 16, 30));
        Assert.True(OverlayKernel.PlannedCoverageExpansionDue(16, 30, 30));
        Assert.False(OverlayKernel.PlannedCoverageExpansionDue(30, 29, 30));
        // Tiny startup windows grow at one day first rather than resampling every
        // quarter-day rails-worker increment.
        Assert.False(OverlayKernel.PlannedCoverageExpansionDue(0.25, 0.75, 30));
        Assert.True(OverlayKernel.PlannedCoverageExpansionDue(0.25, 1.0, 30));
    }

    [Fact]
    public void OffRails_retains_unchanged_planned_geometry_when_the_burn_marks_divergence()
    {
        var sampled = Snap(1000, (5000.0, Dv(1)));
        // Divergence is deliberately absent from this policy: MarkDiverged leaves
        // the exact snapshot object in place, so its sampled map geometry survives.
        Assert.True(OverlayKernel.PlannedOffRailsRestampAllowed(
            sampled, sampled, sameContext: true));
        // Rebase replaces the snapshot and its plan-world anchor. Even identical
        // burn geometry must not license reuse of the pre-Rebase trajectory.
        var rebased = PlanSnapshot.Capture(2000,
            new StateVector(new Vector3d(8e6, 0, 0), new Vector3d(0, 7000, 0)),
            "Earth", [Burn(5000.0, Dv(1))]);
        Assert.True(sampled.GeometryMatches(rebased));
        Assert.False(OverlayKernel.PlannedOffRailsRestampAllowed(
            rebased, sampled, sameContext: true));
        Assert.False(OverlayKernel.PlannedOffRailsRestampAllowed(
            sampled, sampled, sameContext: false));
        Assert.False(OverlayKernel.PlannedOffRailsRestampAllowed(
            null, sampled, sameContext: true));
    }

    [Fact]
    public void Planned_geometry_key_invalidates_every_sampling_and_finite_burn_knob()
    {
        var baseline = new PlannedGeometryKey(
            PlanEnd: 9000,
            ConfigHorizonDays: 30, ConfigRailsAheadDays: 30,
            ThetaMax: 0.01, MaxDensePoints: 65536,
            FiniteBurnSliceSeconds: 20, FiniteBurnMaxSlices: 32);

        Assert.Equal(baseline, baseline with { });
        Assert.NotEqual(baseline, baseline with { PlanEnd = 9001 });
        Assert.NotEqual(baseline, baseline with { ConfigHorizonDays = 31 });
        Assert.NotEqual(baseline, baseline with { ConfigRailsAheadDays = 20 });
        Assert.NotEqual(baseline, baseline with { ThetaMax = 0.02 });
        Assert.NotEqual(baseline, baseline with { MaxDensePoints = 32768 });
        Assert.NotEqual(baseline, baseline with { FiniteBurnSliceSeconds = 10 });
        Assert.NotEqual(baseline, baseline with { FiniteBurnMaxSlices = 64 });
    }

    [Fact]
    public void Target_fixed_policy_forces_planned_resample_even_while_paused()
    {
        var target = new FrameSpec(FrameKind.TargetFixed, "Earth", "Station");
        Assert.True(OverlayKernel.PlannedResampleDue(
            sameSnapshot: true, sameDiverged: true, diverged: false,
            startSeconds: 5000, lastStartSeconds: 5000, sameParent: true,
            sameFrame: OverlayKernel.FrameAllowsPlannedRestamp(target),
            sameGeometryInputs: true, t0Seconds: 3000, batchT0Seconds: 3000,
            batchEndSeconds: 9000));
    }
}

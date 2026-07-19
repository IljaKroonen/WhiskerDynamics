using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Planning;

/// <summary>Tests flight-plan metadata, validation, horizons, storage, and serialization.</summary>
[Collection("flightplans-statics")]
public class FlightPlanModelTests
{
    private static FlightPlanBurnMeta Meta(double t, long stampMs = 0, FrameSpec? frame = null,
        Vector3d? authored = null) => new()
    {
        TimeSeconds = t,
        Frame = frame ?? new FrameSpec(FrameKind.Inertial, "Earth", null),
        Authored = authored ?? new Vector3d(1, 2, 3),
        StampMs = stampMs,
    };

    private static FlightPlanModel NewPlan(double createdAt = 1000, double length = 86400) =>
        new() { CreatedAtSeconds = createdAt, LengthSeconds = length };

    [Fact]
    public void Version_ticks_exactly_on_planned_line_relevant_edits()
    {
        // Version changes only when an edit changes displayed plan state.
        var plan = NewPlan(createdAt: 1000, length: 86400);
        long v = plan.Version;
        Assert.True(v > 0); // a fresh plan always registers against the zero stamp

        plan.SnapshotSetBurn(2000, new Vector3d(1, 0, 0)); // no snapshot yet: no-op
        Assert.Equal(v, plan.Version);

        plan.MarkDiverged(); // flip: bump
        Assert.True(plan.Version > v);
        v = plan.Version;
        plan.MarkDiverged(); // already diverged: no bump
        Assert.Equal(v, plan.Version);

        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth", []), diverged: false);
        Assert.True(plan.Version > v);
        v = plan.Version;

        plan.SnapshotSetBurn(2000, new Vector3d(1, 0, 0)); // captured burn list edit
        Assert.True(plan.Version > v);
        v = plan.Version;

        plan.LengthSeconds = 2 * 86400; // window edit: the drawn horizon moves
        Assert.True(plan.Version > v);
        v = plan.Version;

        var current = plan.Snapshot;
        var fresh = PlanSnapshot.Capture(1500, default, "Earth", []);
        Assert.True(plan.TryReconcileSnapshot(current, fresh));
        Assert.True(plan.Version > v);
        v = plan.Version;
        Assert.False(plan.TryReconcileSnapshot(current, fresh)); // stale expected: refused
        Assert.Equal(v, plan.Version); // a refused reconcile writes nothing
    }

    [Fact]
    public void Reconcile_atomically_refuses_a_stale_propulsion_capture()
    {
        var plan = NewPlan();
        var oldSourceCapture = PlanSnapshot.Capture(1000, default, null, [],
            new EngineScalars(1000, 3000, 2), PropulsionSource.MainEngines);
        plan.PropulsionSource = PropulsionSource.RcsForward;

        Assert.False(plan.TryReconcileSnapshot(null, oldSourceCapture,
            PropulsionSource.MainEngines));
        Assert.Null(plan.Snapshot);
        Assert.Equal(PropulsionSource.RcsForward, plan.PropulsionSource);
    }

    [Fact]
    public void Meta_matches_by_time_within_tolerance_only()
    {
        var plan = NewPlan();
        plan.SetMeta(Meta(5000.0));
        Assert.NotNull(plan.TryGetMetaAt(5000.0));
        Assert.NotNull(plan.TryGetMetaAt(5000.0 + BurnIdentityPolicy.ToleranceSeconds * 0.9));
        Assert.Null(plan.TryGetMetaAt(5000.0 + BurnIdentityPolicy.ToleranceSeconds * 2));
    }

    [Fact]
    public void SetMeta_replaces_the_same_time_slot()
    {
        var plan = NewPlan();
        plan.SetMeta(Meta(5000.0, authored: new Vector3d(1, 0, 0)));
        plan.SetMeta(Meta(5000.0, authored: new Vector3d(9, 0, 0)));
        Assert.Single(plan.Meta);
        Assert.Equal(9, plan.TryGetMetaAt(5000.0)!.Authored.X);
    }

    [Fact]
    public void Submillisecond_times_share_identity_from_admission_through_plan_mutations()
    {
        const double first = 5000.0;
        double aliased = first + BurnIdentityPolicy.ToleranceSeconds * 0.5;
        double distinct = first + BurnIdentityPolicy.ToleranceSeconds * 2;

        Assert.Equal(PlannerKernel.Verdict.DuplicateTime,
            PlannerKernel.ValidateAddTiming(aliased, now: 100, [first]));
        Assert.Equal(PlannerKernel.Verdict.DuplicateTime,
            PlannerKernel.ValidateTimeEdit(aliased, now: 100, [first]));
        Assert.Equal(PlannerKernel.Verdict.Ok,
            PlannerKernel.ValidateAddTiming(distinct, now: 100, [first]));

        var plan = NewPlan();
        plan.SetMeta(Meta(first, authored: new Vector3d(1, 0, 0)));
        plan.SetMeta(Meta(aliased, authored: new Vector3d(2, 0, 0)));
        var aliasedMeta = Assert.Single(plan.Meta);
        Assert.Equal(aliased, aliasedMeta.TimeSeconds);
        Assert.Equal(2, aliasedMeta.Authored.X);
        plan.SetMeta(Meta(distinct, authored: new Vector3d(3, 0, 0)));
        Assert.Equal(2, plan.Meta.Count);

        plan.SetSnapshot(PlanSnapshot.Capture(1000, default, "Earth",
            [new PlanSnapshotBurn(first, new Vector3d(1, 0, 0))]));
        plan.SnapshotSetBurn(aliased, new Vector3d(2, 0, 0));
        var aliasedSnapshot = Assert.Single(plan.Snapshot!.Burns);
        Assert.Equal(aliased, aliasedSnapshot.TimeSeconds);
        Assert.Equal(2, aliasedSnapshot.DeltaVVlf.X);
        plan.SnapshotSetBurn(distinct, new Vector3d(3, 0, 0));
        Assert.Equal(2, plan.Snapshot.Burns.Count);
    }

    [Fact]
    public void MoveMeta_rekeys_refreshes_the_stamp_and_evicts_the_target_slot()
    {
        var plan = NewPlan();
        plan.SetMeta(Meta(5000.0, stampMs: 111, authored: new Vector3d(7, 0, 0)));
        plan.SetMeta(Meta(6000.0, authored: new Vector3d(8, 0, 0)));
        plan.MoveMeta(5000.0, 6000.0, nowMs: 999);
        var moved = Assert.Single(plan.Meta);
        Assert.Equal(6000.0, moved.TimeSeconds);
        Assert.Equal(7, moved.Authored.X);
        Assert.Equal(999, moved.StampMs);
        plan.MoveMeta(123.0, 456.0, 0); // no meta at the source: no-op
        Assert.Single(plan.Meta);
    }

    [Fact]
    public void MoveMeta_within_tolerance_keeps_the_meta_it_moves()
    {
        // A within-tolerance move updates the key without evicting its metadata.
        var plan = NewPlan();
        plan.SetMeta(Meta(5000.0, stampMs: 111, authored: new Vector3d(7, 0, 0)));
        plan.MoveMeta(5000.0, 5000.0005, nowMs: 999);
        var moved = Assert.Single(plan.Meta);
        Assert.Equal(5000.0005, moved.TimeSeconds);
        Assert.Equal(7, moved.Authored.X);
        Assert.Equal(999, moved.StampMs);
        Assert.NotNull(plan.TryGetMetaAt(5000.0005));
        plan.MoveMeta(5000.0005, 5000.0005, nowMs: 1234);
        Assert.Equal(1234, Assert.Single(plan.Meta).StampMs);
    }

    [Fact]
    public void Prune_drops_orphans_only_after_the_grace_window()
    {
        var plan = NewPlan();
        plan.SetMeta(Meta(5000.0, stampMs: 100_000));              // orphan, old
        plan.SetMeta(Meta(6000.0, stampMs: 100_000));              // matched, old
        plan.SetMeta(Meta(7000.0, stampMs: 100_000 + FlightPlanModel.MetaGraceMs - 1)); // orphan, young
        plan.PruneOrphanedMeta([6000.0], nowMs: 100_000 + FlightPlanModel.MetaGraceMs);
        Assert.Equal(2, plan.Meta.Count);
        Assert.Null(plan.TryGetMetaAt(5000.0));   // orphaned past grace: pruned
        Assert.NotNull(plan.TryGetMetaAt(6000.0)); // its burn exists: kept
        Assert.NotNull(plan.TryGetMetaAt(7000.0)); // queued add within grace: kept
    }

    [Fact]
    public void Burns_outside_the_plan_window_are_rejected()
    {
        var plan = NewPlan(createdAt: 1000, length: 86400);
        // Rails window comfortably beyond the plan end: the plan-end rule decides.
        Assert.Contains("plan end",
            plan.RejectOutsideWindow(1000 + 86400, nowSeconds: 1000, railsAheadDays: 30));
        Assert.Null(plan.RejectOutsideWindow(
            1000 + 86400 - FlightPlanModel.MinimumPostBurnSeconds,
            nowSeconds: 1000, railsAheadDays: 30));
        Assert.Contains("plan end", plan.RejectOutsideWindow(
            1000 + 86400 - FlightPlanModel.MinimumPostBurnSeconds + 1e-6,
            nowSeconds: 1000, railsAheadDays: 30));
        Assert.Null(plan.RejectOutsideWindow(
            1000 + 86400 - FlightPlanModel.MinimumPostBurnSeconds - 1e-6,
            nowSeconds: 1000, railsAheadDays: 30));
        Assert.Contains("plan end",
            plan.RejectOutsideWindow(1000 + 86400 + 1, nowSeconds: 1000, railsAheadDays: 30));
    }

    [Fact]
    public void Burns_beyond_the_rails_horizon_are_rejected_with_the_frames_panel_remedy()
    {
        // Plan windows cannot demand ephemerides beyond the maintained rails horizon.
        var plan = NewPlan(createdAt: 1000, length: 86400);
        // Inside the plan end (t=87400) but past now + 0.5 d of rails (t=44200).
        var reason = plan.RejectOutsideWindow(50_000, nowSeconds: 1000, railsAheadDays: 0.5);
        Assert.NotNull(reason);
        Assert.Contains("rails horizon", reason);
        Assert.Contains("N-Body Frames", reason);
        // Exactly on the rails horizon: allowed.
        Assert.Null(plan.RejectOutsideWindow(1000 + 0.5 * 86400, nowSeconds: 1000, railsAheadDays: 0.5));
    }

    [Fact]
    public void Plan_length_validation_guards_finiteness_and_the_last_burn()
    {
        var plan = NewPlan(createdAt: 1000, length: 86400);
        Assert.Null(plan.ValidateLength(2 * 86400, [50_000.0]));
        Assert.NotNull(plan.ValidateLength(double.NaN, []));
        Assert.NotNull(plan.ValidateLength(0, []));
        Assert.NotNull(plan.ValidateLength(-5, []));
        // Shrinking below the last planned burn is refused.
        Assert.Contains("last burn", plan.ValidateLength(10_000, [1000 + 20_000.0]));
        Assert.Contains("last burn", plan.ValidateLength(20_000, [1000 + 20_000.0]));
        Assert.Null(plan.ValidateLength(
            20_000 + FlightPlanModel.MinimumPostBurnSeconds, [1000 + 20_000.0]));
    }

    [Fact]
    public void Plan_length_is_capped_at_the_rails_ceiling()
    {
        // Plans cannot exceed the maximum attainable rails horizon.
        var plan = NewPlan(createdAt: 1000, length: 86400);
        double cap = SettingsKernel.MaxRailsDays * 86400.0;
        Assert.Null(plan.ValidateLength(cap, []));
        Assert.Contains("capped", plan.ValidateLength(cap + 1, []));
        Assert.Contains("positive", plan.ValidateLength(double.PositiveInfinity, []));
    }

    [Fact]
    public void Effective_horizon_extends_to_the_plan_end_clamped_to_rails_ahead()
    {
        double now = 5_000_000;
        Assert.Equal(30, FlightPlans.EffectiveHorizonDays(30, 30, null, now));
        // Display horizon follows the rails already available while the worker catches up.
        Assert.Equal(12, FlightPlans.EffectiveHorizonDays(100, 12, null, now));
        Assert.Equal(30, FlightPlans.EffectiveHorizonDays(30, 30, now + 10 * 86400, now));
        Assert.Equal(10, FlightPlans.EffectiveHorizonDays(5, 30, now + 10 * 86400, now));
        Assert.Equal(30, FlightPlans.EffectiveHorizonDays(5, 30, now + 100 * 86400, now));
        Assert.Equal(5, FlightPlans.EffectiveHorizonDays(5, 30, now - 1, now));
        Assert.Equal(5, FlightPlans.EffectiveHorizonDays(5, 30, double.NaN, now));
    }

    [Fact]
    public void Initial_length_covers_adopted_stock_burns()
    {
        double now = 5_000_000;
        Assert.Equal(FlightPlans.DefaultLengthSeconds, FlightPlans.InitialLengthSeconds(now, []));
        Assert.Equal(FlightPlans.DefaultLengthSeconds,
            FlightPlans.InitialLengthSeconds(now, [now + 3600.0]));
        // A late burn extends the plan to include its safety margin.
        double late = now + 10 * 86400.0;
        double length = FlightPlans.InitialLengthSeconds(now, [now + 3600.0, late]);
        Assert.Equal(10 * 86400.0 + FlightPlans.AdoptedBurnMarginSeconds, length);
        var plan = new FlightPlanModel { CreatedAtSeconds = now, LengthSeconds = length };
        Assert.Null(plan.ValidateLength(length, [now + 3600.0, late]));
        Assert.Null(plan.RejectOutsideWindow(late, now, railsAheadDays: 30));
        // Extension is capped by the maximum rails horizon.
        Assert.Equal(SettingsKernel.MaxRailsDays * 86400.0,
            FlightPlans.InitialLengthSeconds(now, [now + (SettingsKernel.MaxRailsDays + 100) * 86400.0]));
        Assert.Equal(FlightPlans.DefaultLengthSeconds,
            FlightPlans.InitialLengthSeconds(now, [double.NaN]));
    }

    [Fact]
    public void Store_creates_reads_removes_and_sweeps()
    {
        FlightPlans.ResetSessionStatics();
        string id = "store-" + Guid.NewGuid().ToString("N");
        Assert.Null(FlightPlans.TryGet(id));
        var plan = FlightPlans.Create(id, nowSeconds: 42.0);
        Assert.Equal(42.0, plan.CreatedAtSeconds);
        Assert.Equal(FlightPlans.DefaultLengthSeconds, plan.LengthSeconds);
        Assert.Same(plan, FlightPlans.TryGet(id));
        FlightPlans.Remove(id);
        Assert.Null(FlightPlans.TryGet(id));
        FlightPlans.Create(id, 1.0);
        FlightPlans.ResetSessionStatics();
        Assert.Null(FlightPlans.TryGet(id));
    }

    [Fact]
    public void Vessel_session_cleanup_forgets_plan_and_frame_state_for_recycled_ids()
    {
        FlightPlans.ResetSessionStatics();
        FrameManager.ResetSessionStatics();
        string id = "forgotten-" + Guid.NewGuid().ToString("N");
        try
        {
            FlightPlans.Create(id, nowSeconds: 42.0);
            Assert.Equal(1, FrameManager.ImportFrameSelections(new SidecarFile
            {
                FrameSelections =
                [
                    new SidecarFrameSelection
                    {
                        VesselId = id,
                        Frame = new SidecarFrame
                        {
                            FrameKind = "Surface",
                            PrimaryId = "Earth",
                        },
                    },
                ],
            }));
            Assert.Single(FlightPlans.PlansForSidecar());
            Assert.NotNull(FrameManager.SelectedFrameForSidecar(id));

            VesselRegistry.ForgetVesselSessionState(id);

            Assert.Null(FlightPlans.TryGet(id));
            Assert.Empty(FlightPlans.PlansForSidecar());
            Assert.Null(FrameManager.SelectedFrameForSidecar(id));
        }
        finally
        {
            FlightPlans.ResetSessionStatics();
            FrameManager.ResetSessionStatics();
        }
    }

    [Fact]
    public void Sidecar_bridge_round_trips_every_frame_kind()
    {
        var plan = NewPlan(createdAt: 777, length: 3 * 86400);
        plan.SetMeta(Meta(1000, frame: new FrameSpec(FrameKind.Inertial, "Earth", null),
            authored: new Vector3d(1.5, -2.5, 3.5)));
        plan.SetMeta(Meta(2000, frame: new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Luna"),
            authored: new Vector3d(-4, 5, -6)));
        plan.SetMeta(Meta(3000, frame: new FrameSpec(FrameKind.Surface, "Luna", null),
            authored: new Vector3d(0.25, 0, -0.125)));
        plan.SetMeta(Meta(4000, frame: new FrameSpec(FrameKind.TargetFixed, "Earth", "Rendezvous Target"),
            authored: new Vector3d(7, -8, 9)));

        var restored = FlightPlans.FromSidecar(FlightPlans.ToSidecar(plan), nowMs: 5);
        Assert.NotNull(restored);
        Assert.Equal(777, restored.CreatedAtSeconds);
        Assert.Equal(3 * 86400, restored.LengthSeconds);
        Assert.Equal(4, restored.Meta.Count);
        var pair = restored.TryGetMetaAt(2000)!;
        Assert.Equal(new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Luna"), pair.Frame);
        Assert.Equal(new Vector3d(-4, 5, -6), pair.Authored);
        Assert.Equal(5, pair.StampMs); // restored metas get the import-time grace stamp
        Assert.Equal(new FrameSpec(FrameKind.Surface, "Luna", null), restored.TryGetMetaAt(3000)!.Frame);
        Assert.Equal(new FrameSpec(FrameKind.TargetFixed, "Earth", "Rendezvous Target"),
            restored.TryGetMetaAt(4000)!.Frame);
        Assert.Equal(1.5, restored.TryGetMetaAt(1000)!.Authored.X);
    }

    [Fact]
    public void Sidecar_bridge_drops_broken_burns_and_refuses_broken_plans()
    {
        Assert.Null(FlightPlans.ToSidecar(null));
        Assert.Null(FlightPlans.FromSidecar(null, 0));
        Assert.Null(FlightPlans.FromSidecar(
            new SidecarPlan { CreatedAtSeconds = double.NaN, LengthSeconds = 100 }, 0));
        Assert.Null(FlightPlans.FromSidecar(
            new SidecarPlan { CreatedAtSeconds = 0, LengthSeconds = 0 }, 0));
        double maximumLength = SettingsKernel.MaxRailsDays * 86400.0;
        Assert.Null(FlightPlans.FromSidecar(
            new SidecarPlan
            {
                CreatedAtSeconds = 0,
                LengthSeconds = maximumLength + 1.0,
            }, 0));
        Assert.NotNull(FlightPlans.FromSidecar(
            new SidecarPlan
            {
                CreatedAtSeconds = 0,
                LengthSeconds = maximumLength,
            }, 0));

        var dto = new SidecarPlan
        {
            CreatedAtSeconds = 0,
            LengthSeconds = 86400,
            Burns =
            [
                new SidecarPlanBurn { TimeSeconds = 100, FrameKind = "Inertial", PrimaryId = "Earth", Basis = "prn", X = 1 },
                // Entries without an explicit basis cannot be interpreted safely.
                new SidecarPlanBurn { TimeSeconds = 150, FrameKind = "Inertial", PrimaryId = "Earth", X = 1 },
                new SidecarPlanBurn { TimeSeconds = 200, FrameKind = "Warp9", PrimaryId = "Earth" },          // unknown kind
                // Numeric strings may parse to undefined enum values and must be rejected.
                new SidecarPlanBurn { TimeSeconds = 250, FrameKind = "42", PrimaryId = "Earth" },             // undefined numeric kind
                new SidecarPlanBurn { TimeSeconds = 300, FrameKind = "Inertial", PrimaryId = "" },            // no body
                new SidecarPlanBurn { TimeSeconds = 400, FrameKind = "TwoBodyFixed", PrimaryId = "Earth" },   // pair without reference
                new SidecarPlanBurn { TimeSeconds = 500, FrameKind = "Surface", PrimaryId = "Luna", SecondaryId = "Earth" }, // stray reference
                new SidecarPlanBurn { TimeSeconds = 550, FrameKind = "TargetFixed", PrimaryId = "Earth" },    // target frame without vessel
                new SidecarPlanBurn { TimeSeconds = 600, FrameKind = "Inertial", PrimaryId = "Earth", X = double.NaN },      // non-finite
                new SidecarPlanBurn { TimeSeconds = double.PositiveInfinity, FrameKind = "Inertial", PrimaryId = "Earth" },  // non-finite key
            ],
        };
        var plan = FlightPlans.FromSidecar(dto, 0);
        Assert.NotNull(plan);
        var kept = Assert.Single(plan.Meta); // broken metas dropped; their burns stay VLF
        Assert.Equal(100, kept.TimeSeconds);
    }

    [Fact]
    public void ImportSidecar_repopulates_the_store_per_vessel()
    {
        FlightPlans.ResetSessionStatics();
        string withPlan = "import-" + Guid.NewGuid().ToString("N");
        string without = "import-" + Guid.NewGuid().ToString("N");
        string invalid = "import-" + Guid.NewGuid().ToString("N");
        string duplicate = "import-" + Guid.NewGuid().ToString("N");
        var sidecar = new SidecarFile
        {
            Plans =
            {
                new SidecarPlanRecord
                {
                    VesselId = withPlan,
                    Plan = new SidecarPlan { CreatedAtSeconds = 10, LengthSeconds = 20 },
                },
                new SidecarPlanRecord { VesselId = without, Plan = null },
                new SidecarPlanRecord
                {
                    VesselId = invalid,
                    Plan = new SidecarPlan { CreatedAtSeconds = 10, LengthSeconds = 0 },
                },
                new SidecarPlanRecord
                {
                    VesselId = duplicate,
                    Plan = new SidecarPlan { CreatedAtSeconds = 10, LengthSeconds = 20 },
                },
                new SidecarPlanRecord
                {
                    VesselId = duplicate,
                    Plan = new SidecarPlan { CreatedAtSeconds = 30, LengthSeconds = 40 },
                },
            },
        };
        Assert.Equal(1, FlightPlans.ImportSidecar(sidecar));
        Assert.NotNull(FlightPlans.TryGet(withPlan));
        Assert.Null(FlightPlans.TryGet(without));
        Assert.Null(FlightPlans.TryGet(invalid));
        Assert.Null(FlightPlans.TryGet(duplicate));
        FlightPlans.Remove(withPlan);
    }
}

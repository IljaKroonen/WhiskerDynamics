using Brutal.Numerics;
using KSA;

namespace WhiskerDynamics.Mod.Planning;

/// <summary>The mod's ONLY write path into the game's maneuver plan. Mirrors
/// the stock mutation discipline exactly (decompiled Program.ProcessBurnClick and the
/// Burn editor): structural changes (add/remove) are enqueued as
/// InputEvents.BurnUpdateData — the game's own InputEvents.ApplyInputEvents applies them
/// on the main thread inside Program.PrepareFrame — and in-place edits write
/// Burn.Time / Burn.DeltaVVlf then call burn.Update(fc), which enqueues the BurnUpdated
/// event that sets BurnPlan.FlightPlansOutOfDate for the worker recompute. MUST be called
/// only from main-thread ImGui draw code (the same phase stock's editor windows mutate
/// from) — ENFORCED: every mutating method rejects
/// wrong-thread callers with a panel-visible string (thread id captured at EarlyInit).
/// Never throws: every method returns a panel-ready status string.</summary>
public static class BurnPlanWriter
{
    /// <summary>Main managed thread id, captured at EarlyInit.
    /// StarMap's BeforeMain shim runs on the process main thread — the same thread that
    /// then runs Program.Main and every ImGui draw phase. -1 = never captured: every
    /// write is rejected (fail-safe and panel-visible, never silent corruption).</summary>
    private static int _mainThreadId = -1;

    /// <summary>Called once, first thing in ModMain.EarlyInit.</summary>
    internal static void CaptureMainThread() => _mainThreadId = Environment.CurrentManagedThreadId;

    /// <summary>Shared read-only main-thread identity for game-state seams that must
    /// defer work captured from job-thread callers.</summary>
    internal static bool IsMainThread => Environment.CurrentManagedThreadId == _mainThreadId;

    /// <summary>KSA-free guard seam (offline-testable): null on the captured main thread,
    /// a panel-ready rejection anywhere else. Every mutating method calls this FIRST —
    /// InputEvents.BurnUpdateBuffer and the Burn fields are main-thread state, and a
    /// wrong-thread caller must get a visible string instead of a silent torn write.</summary>
    internal static string? RejectIfOffMainThread() =>
        IsMainThread ? null : "rejected: wrong thread";

    /// <summary>Stable snapshot of the plan's burns for one panel frame. Never throws:
    /// a faulting read degrades to an empty list + one WARN.</summary>
    public static IReadOnlyList<Burn> Snapshot(Vehicle vehicle) => ContainSnapshot(
        static v =>
        {
            var plan = v.FlightComputer.BurnPlan;
            var burns = new List<Burn>(plan.BurnCount);
            for (int i = 0; i < plan.BurnCount; i++)
                if (plan.TryGetBurn(i, out Burn? burn) && burn is not null)
                    burns.Add(burn);
            return (IReadOnlyList<Burn>)burns;
        }, vehicle);

    /// <summary>Never-throw containment seam for Snapshot. Generic so the offline suite
    /// can prove containment with a throwing fake — Vehicle itself cannot be faked
    /// offline (tests are KSA-free by repo convention, and FlightComputer is
    /// non-virtual, so no fake subclass either). Static lambda + state = no per-frame
    /// closure allocation on the panel's once-per-frame call.</summary>
    internal static IReadOnlyList<T> ContainSnapshot<TState, T>(Func<TState, IReadOnlyList<T>> read, TState state)
    {
        try
        {
            return read(state);
        }
        catch (Exception e)
        {
            if (Environment.TickCount64 >= _nextSnapshotWarnMs)
            {
                // 5 s-wall floor: Snapshot runs every panel frame, so a persistently
                // faulting read must not flood the log (SOI-shim throttle precedent;
                // wall-clock throttles are deliberately not reset by the statics sweep).
                _nextSnapshotWarnMs = Environment.TickCount64 + 5000;
                ModLog.Warn($"planner: snapshot contained: {e}");
            }
            return [];
        }
    }

    private static long _nextSnapshotWarnMs;

    /// <summary>Applies an in-place stock edit as one transaction. Burn.Update may
    /// throw after the live field has been assigned; restore the exact prior field
    /// before propagating the exception to the writer's containment boundary.</summary>
    internal static void ApplyTransactional<T>(T previous, T replacement,
        Action<T> assign, Action update)
    {
        try
        {
            assign(replacement);
            update();
        }
        catch
        {
            assign(previous);
            throw;
        }
    }

    /// <summary>Plan-snapshot mirror: every successful stock mutation
    /// reflects into the plan's captured burn list HERE — the one write path — so no
    /// edit affordance can forget it. While NOT diverged the on-rails reconcile would
    /// self-heal a miss anyway; while DIVERGED (reconcile off by design) this mirror
    /// is the only thing keeping panel edits visible on the frozen ghost. Queued
    /// adds/removes mirror optimistically — stock applies them next frame, and a
    /// stock-side rejection of an already-validated queued write has no known path.</summary>
    private static void MirrorSnapshotSet(Vehicle vehicle, double timeSeconds,
        WhiskerDynamics.Core.Vector3d dvVlf, string? basisParentId,
        WhiskerDynamics.Core.Vector3d? displayDvVlf = null) =>
        FlightPlans.TryGet(vehicle.Id)?.SnapshotSetBurn(
            timeSeconds, dvVlf, basisParentId,
            markDownstreamParentsPending: true,
            displayDvVlf: displayDvVlf);

    /// <summary>THE planning-side patch resolution, mirroring stock click-to-place.
    /// Runs inside the burn preservation scope so every planning seam resolves the
    /// same extended conic past a stock impact prediction.</summary>
    internal static PatchedConic? ResolvePlanningPatch(Vehicle vehicle, SimTime time)
    {
        using (Patches.BurnPlanCalculationContext.EnterForVehicle(vehicle))
            return vehicle.FlightComputer.BurnPlan.TryGetValidTimeLinePatch(time)
                ?? vehicle.FlightPlan.TryFindPatch(time);
    }

    public static string TryAdd(Vehicle vehicle, double burnTimeSeconds, double3 dvVlf)
    {
        try
        {
            if (RejectIfOffMainThread() is { } wrongThread) return wrongThread;
            // Final admission boundary for every add route (drills, automatic
            // solvers, and the panel). A non-finite vector must be
            // rejected before even resolving a stock patch, let alone constructing
            // or queueing a Burn; TryEditDv enforces the same invariant.
            if (!PlannerKernel.ValidateDv(dvVlf.X, dvVlf.Y, dvVlf.Z))
                return PlannerKernel.Describe(PlannerKernel.Verdict.NotFinite);
            double now = Universe.GetElapsedSimTime().Seconds();
            var existing = Snapshot(vehicle);
            var times = new List<double>(existing.Count);
            foreach (var b in existing) times.Add(b.Time.Seconds());
            var verdict = PlannerKernel.ValidateAddTiming(burnTimeSeconds, now, times);
            if (verdict != PlannerKernel.Verdict.Ok)
                return PlannerKernel.Describe(verdict);

            // Only a time that passed every KSA-free admission check may enter
            // SimTime construction and stock patch resolution.
            var time = new SimTime(burnTimeSeconds);
            PatchedConic? patch = ResolvePlanningPatch(vehicle, time);
            if (patch is null)
                return PlannerKernel.Describe(PlannerKernel.Verdict.NoPatch);

            // Stock creation recipe (Program.cs:1835-1842). The OrbitPointCce argument is
            // required by the factory signature but unused by the constructor body
            // (Burn.cs:137-146); stock passes the clicked point, we pass the patch orbit's
            // point at the burn time. Burn.Create with a nonzero dv has stock precedent:
            // BurnPlan.DeserializeSave rebuilds saved burns exactly this way.
            var burn = Burn.Create(patch.Orbit.GetPointAt(time), burnTimeSeconds, dvVlf, patch, vehicle);
            InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
            {
                FlightComputer = vehicle.FlightComputer,
                Burn = burn,
                AddBurn = true,
            });
            string? basisParentId = patch.Orbit.Parent is Astronomical patchParent
                ? patchParent.Id
                : (vehicle.Orbit.Parent as Astronomical)?.Id;
            MirrorSnapshotSet(vehicle, burnTimeSeconds, FrameAdapter.ToCore(dvVlf),
                basisParentId);
            ModLog.Info($"planner: queued add burn for '{vehicle.Id}' at t={burnTimeSeconds:F1} s "
                + $"({(burnTimeSeconds - now) / 60.0:F1} min ahead), dvVlf=({dvVlf.X:F2}, {dvVlf.Y:F2}, {dvVlf.Z:F2}) m/s");
            return "queued";
        }
        catch (Exception e)
        {
            ModLog.Warn($"planner: add contained: {e}");
            return $"error: {e.Message}";
        }
    }

    public static string TryEditTime(Vehicle vehicle, Burn burn, double newTimeSeconds)
    {
        try
        {
            if (RejectIfOffMainThread() is { } wrongThread) return wrongThread;
            double now = Universe.GetElapsedSimTime().Seconds();
            var others = new List<double>();
            foreach (var b in Snapshot(vehicle))
                if (!ReferenceEquals(b, burn)) others.Add(b.Time.Seconds());
            var verdict = PlannerKernel.ValidateTimeEdit(newTimeSeconds, now, others);
            if (verdict != PlannerKernel.Verdict.Ok) return PlannerKernel.Describe(verdict);
            double oldTimeSeconds = burn.Time.Seconds();
            SimTime oldTime = burn.Time;
            ApplyTransactional(oldTime, new SimTime(newTimeSeconds),
                value => burn.Time = value,
                () => burn.Update(vehicle.FlightComputer)); // queues BurnUpdated dirty event
            FlightPlans.TryGet(vehicle.Id)?.SnapshotMoveBurnDeferred(
                oldTimeSeconds, newTimeSeconds);
            return "applied";
        }
        catch (Exception e)
        {
            ModLog.Warn($"planner: time edit contained: {e}");
            return $"error: {e.Message}";
        }
    }

    /// <summary><paramref name="displayDvVlf"/>: predictor-basis realization of an
    /// execution-basis write (upkeep only); ordinary edits leave it null.</summary>
    public static string TryEditDv(Vehicle vehicle, Burn burn, double prograde, double normal, double outward,
        WhiskerDynamics.Core.Vector3d? displayDvVlf = null)
    {
        try
        {
            if (RejectIfOffMainThread() is { } wrongThread) return wrongThread;
            if (!PlannerKernel.ValidateDv(prograde, normal, outward))
                return PlannerKernel.Describe(PlannerKernel.Verdict.NotFinite);
            double3 oldDeltaV = burn.DeltaVVlf;
            ApplyTransactional(oldDeltaV,
                PlannerKernel.ComposeVlf(prograde, normal, outward),
                value => burn.DeltaVVlf = value,
                () => burn.Update(vehicle.FlightComputer));
            MirrorSnapshotSet(vehicle, burn.Time.Seconds(),
                FrameAdapter.ToCore(burn.DeltaVVlf),
                PlannedBurnConverter.ExistingBurnParentId(vehicle, burn),
                displayDvVlf);
            return "applied";
        }
        catch (Exception e)
        {
            ModLog.Warn($"planner: dv edit contained: {e}");
            return $"error: {e.Message}";
        }
    }

    public static string TryRemove(Vehicle vehicle, Burn burn)
    {
        try
        {
            if (RejectIfOffMainThread() is { } wrongThread) return wrongThread;
            // Stock delete recipe (Burn.cs:630-636): enqueue, let ApplyInputEvents route it
            // through FlightComputer.RemoveBurn (which also unloads an active BurnTarget).
            InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
            {
                FlightComputer = vehicle.FlightComputer,
                Burn = burn,
                DeleteBurn = true,
            });
            FlightPlans.TryGet(vehicle.Id)?.SnapshotRemoveBurn(burn.Time.Seconds());
            ModLog.Info($"planner: queued remove burn for '{vehicle.Id}' at t={burn.Time.Seconds():F1} s");
            return "queued";
        }
        catch (Exception e)
        {
            ModLog.Warn($"planner: remove contained: {e}");
            return $"error: {e.Message}";
        }
    }
}

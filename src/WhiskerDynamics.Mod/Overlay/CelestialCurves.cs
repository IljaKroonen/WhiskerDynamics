using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Honest orbit lines: each selected modeled celestial's line is sampled
/// from its authoritative composite rails track — inertial
/// parent-relative arcs when no display frame is active (replacing the fossil stock
/// Kepler ellipse), frame-space curves when one is (same semantics as the vessel
/// overlay). The bounded display set is <see cref="RailsService.CurveBodyIds"/> and
/// may contain both mutual-backbone and restricted-track bodies. Sampling
/// runs on the rails
/// worker (~1 Hz, adaptive turn-angle density; each body contained on its own so one
/// throwing body cannot starve the rest; a frame-label change bypasses the throttle so
/// the frame-change blink lasts ≤~1 s); the render-phase prefix (CelestialLinePatch) routes
/// three ways via <see cref="Route"/> and re-embeds/stages cached coordinates through
/// <see cref="StageFresh"/>, drawing with joinEnds:false. DEACTIVATION RESTORE
/// unchanged: stock regenerates celestial points only when MISSING (Orbit.cs:2263), so
/// every line the mod overwrote is handed back once via Celestial.RegenerateOrbitLines
/// (Celestial.cs:1761) whenever staging is not happening — including after a
/// mid-session FatalDisable.</summary>
public static class CelestialCurves
{
    internal const int MaximumTraversalPoints = 2048;
    private static readonly object GenerationGate = new();
    private static long _generation;
    private sealed class CurveSamples
    {
        public required string BodyId { get; init; }
        public required string ParentId { get; init; }
        public required long SampleWallMs { get; init; }
        public required double[] Times { get; init; }
        /// <summary>Frame coordinates when <see cref="FrameLabel"/> is non-null;
        /// inertial parent-relative Cce otherwise.</summary>
        public required Vector3d[] Coordinates { get; init; }
        public required string? FrameLabel { get; init; }
        public required double SampleT0 { get; init; }
        // Honest-density arc: the FULL adaptive sweep, unpadded —
        // the dense OrbitLinePass draw reads these; Times/Coordinates above are a
        // decimated stock-length subset kept for the staged pick buffer.
        public required double[] DenseTimes { get; init; }
        public required Vector3d[] DenseCoordinates { get; init; }
        /// <summary>The screen-space emit filter's inputs (arc + DP significance),
        /// worker-computed over <see cref="DenseCoordinates"/>.</summary>
        public required DecimationMetrics DenseMetrics { get; init; }
    }

    /// <summary>Everything the draw prefix needs for one staged celestial arc: the
    /// dense sweep plus the READY staging context — the SAME per-frame pose/parent
    /// resolution the staged buffer was built with, assembled HERE where the staging
    /// math lives (a hand-built mirror in the patch is how a new context field ships
    /// default-valued). ParentShift is identity by construction: a celestial's parent
    /// is parse-time constant, the vessel-side SOI re-anchor cannot arise.</summary>
    internal readonly struct StagedCelestial
    {
        public required double[] DenseTimes { get; init; }
        public required Vector3d[] DenseCoordinates { get; init; }
        public required DecimationMetrics DenseMetrics { get; init; }
        public required TrajectoryOverlay.StagingContext Context { get; init; }
        public required double NowSeconds { get; init; }
    }

    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, CurveSamples> Curves = new();
    /// <summary>Ids whose orbit-line cache this class overwrote and has not yet handed
    /// back. Once-only restore comes from TryRemove's atomicity.</summary>
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, byte> StagedIds = new();
    /// <summary>Per-frame re-stage skip: the curve whose
    /// points are CURRENTLY in each body's orbit cache. Inertial payloads
    /// (t − SampleT0) are time-independent, so rebuilding 2000 points per celestial
    /// per frame is pure waste — dozens of bodies deep, milliseconds of every
    /// frame. Framed payloads (t − now, re-embedded at the now pose) must restage
    /// per frame and never enter this cache.</summary>
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, CurveSamples> LastStagedCurve = new();
    private sealed record StageMemo(CurveSamples Curve, double NowSeconds,
        StagedCelestial Staged, Brutal.Numerics.double3 FirstPosition);
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, StageMemo>
        LastStagedFrame = new();
    private static long _lastSampleWallMs;
    /// <summary>Frame label the last COMPLETED sampling loop ran under (worker-thread
    /// only). When the active label differs, the 1 Hz throttle is bypassed (worker
    /// half): the draw prefix blinks every celestial while curves carry the old label,
    /// and the worker cycles every 500 ms, so the bypass caps the blink at ≤~1 s.</summary>
    private static string? _lastSampledFrameLabel;
    private sealed record SamplingCompletion(double WindowDays, string? FrameLabel);
    private static SamplingCompletion? _samplingCompletion;
    /// <summary>One-shot latch for the curve-budget cap log (worker-thread only, like
    /// the throttle statics). Re-armed per bind by the session statics sweep: a rebind
    /// may load a different catalog, so the cap fact must be re-announced for it.</summary>
    private static bool _capLogged;
    /// <summary>Once-per-BODY latch (same re-arm pattern) for point-budget arc
    /// truncation — a silently short arc would read as "the orbits setting is
    /// ignored". Per body, not one-shot: fixed-quality sampling routinely truncates
    /// fast moons, and a single latch would
    /// let the first fast moon consume the whole bind's diagnostic while slower
    /// bodies truncate unreported at longer window presets. Bounded by the curve
    /// body cap; worker-thread only.</summary>
    private static readonly HashSet<string> _truncationLogged = [];

    internal static long CurrentGeneration
    {
        get { lock (GenerationGate) return _generation; }
    }

    private static bool IsGenerationCurrent(long generation)
    {
        lock (GenerationGate) return generation == _generation;
    }

    /// <summary>Runs a shared-state mutation only while its worker still belongs to the
    /// current celestial generation. Reset and publication use this same gate, so an old
    /// worker can neither race a reset nor publish after it.</summary>
    private static bool TryMutateCurrentGeneration(long generation,
        CancellationToken cancellationToken, Action mutation)
    {
        lock (GenerationGate)
        {
            if (cancellationToken.IsCancellationRequested || generation != _generation)
                return false;
            mutation();
            return true;
        }
    }

    /// <summary>The one publication funnel for completed curve samples. The optional
    /// callback is a deterministic test seam immediately before the reset/publication
    /// gate; production never supplies it.</summary>
    private static bool TryPublishCurve(string bodyId, CurveSamples curve, long generation,
        CancellationToken cancellationToken, Action? beforeGenerationGate = null)
    {
        beforeGenerationGate?.Invoke();
        return TryMutateCurrentGeneration(generation, cancellationToken,
            () => Curves[bodyId] = curve);
    }

    internal static bool TryPublishSentinelForTest(string bodyId, string sentinel,
        long generation, Action? beforeGenerationGate = null)
    {
        Vector3d[] points = [Vector3d.Zero, Vector3d.Zero];
        return TryPublishCurve(bodyId, new CurveSamples
        {
            BodyId = bodyId,
            ParentId = sentinel,
            SampleWallMs = Environment.TickCount64,
            Times = [0.0, 1.0],
            Coordinates = points,
            FrameLabel = null,
            SampleT0 = 0.0,
            DenseTimes = [0.0, 1.0],
            DenseCoordinates = points,
            DenseMetrics = DecimationMetrics.For(points, MaximumTraversalPoints),
        }, generation, CancellationToken.None, beforeGenerationGate);
    }

    internal static string? PublishedSentinelForTest(string bodyId) =>
        Curves.TryGetValue(bodyId, out var curve) ? curve.ParentId : null;

    internal static double CompletedWindowDays(string? frameLabel)
    {
        var completion = Volatile.Read(ref _samplingCompletion);
        return completion is not null
            && string.Equals(completion.FrameLabel, frameLabel, StringComparison.Ordinal)
                ? completion.WindowDays
                : 0.0;
    }

    internal static bool TryPublishCompletionForTest(
        double windowDays, string? frameLabel, long generation) =>
        TryMutateCurrentGeneration(generation, CancellationToken.None,
            () => _samplingCompletion = new(windowDays, frameLabel));

    internal static void ResetSessionStatics()
    {
        lock (GenerationGate)
        {
            _generation = unchecked(_generation + 1);
            Curves.Clear();
            // Deliberately NOT restoring here: a rebind/save-load
            // rebuilds orbit state; stock's IsMissingPoints() regeneration covers fresh
            // orbits, and touching possibly-stale Celestial instances is a crash hazard.
            StagedIds.Clear();
            LastStagedCurve.Clear();
            LastStagedFrame.Clear();
            _lastSampleWallMs = 0;
            _lastSampledFrameLabel = null;
            _samplingCompletion = null;
            _capLogged = false;
            _truncationLogged.Clear();
        }
    }

    /// <summary>Rails worker, each cycle (~1 Hz internally). Always-on while the mod is
    /// enabled; frame mode iff a display frame is active at sampling time.</summary>
    public static void MaybeSample(RailsService rails, ModConfig config, long generation,
        CancellationToken cancellationToken)
    {
        bool ShouldStop() =>
            cancellationToken.IsCancellationRequested || !IsGenerationCurrent(generation);

        try
        {
            if (ShouldStop()) return;
            if (!ModServices.Enabled)
            {
                TryMutateCurrentGeneration(generation, cancellationToken, Curves.Clear);
                return;
            }
            IReadOnlyList<string> curveBodyIds = rails.CurveBodyIds;
            // Cap log, one-shot per bind (latch re-armed by ResetSessionStatics): a
            // catalog denser than the curve budget is a config-visible fact, announced
            // once. Unselected bodies retain stock compatibility conic displays only;
            // those displays are never an input to the authoritative dynamics.
            bool logCap = false;
            if (!TryMutateCurrentGeneration(generation, cancellationToken, () =>
                {
                    if (!_capLogged
                        && rails.CurveEligibleCount > curveBodyIds.Count)
                    {
                        _capLogged = true;
                        logCap = true;
                    }
                }))
                return;
            if (logCap)
            {
                int cap = Math.Max(1, config.CelestialCurveMaxBodies); // kernel's clamp, mirrored
                ModLog.Info($"celestial curves: catalog {rails.CurveEligibleCount} bodies exceeds "
                    + $"celestial_curve_max_bodies={cap} - modeled arcs cover the top {cap}; "
                    + "the rest retain display-only stock conics");
            }
            long nowWall = Environment.TickCount64;
            bool framed = FrameManager.TryCaptureActive(out var frameSnapshot);
            var active = framed ? frameSnapshot.Spec : null;
            // Label-change throttle bypass (worker half): while curves carry the
            // previous frame label the draw prefix BLINKS every celestial, so a frame switch
            // must not also wait out the 1 Hz window — resample this cycle.
            bool throttled = false;
            if (!TryMutateCurrentGeneration(generation, cancellationToken, () =>
                {
                    bool labelChanged = !string.Equals(active?.Label,
                        _lastSampledFrameLabel, StringComparison.Ordinal);
                    if (!labelChanged && nowWall - _lastSampleWallMs < 1000)
                        throttled = true;
                    else
                        _lastSampleWallMs = nowWall;
                }))
                return;
            if (throttled) return;

            if (ShouldStop()) return;
            double now = KSA.Universe.GetElapsedSimTime().Seconds();
            double thetaMax = OverlayKernel.SamplingThetaRadians(
                config.OverlayMaxTurnDeg);
            // Clamp to the rails window ACTUALLY integrated: while the worker grows a
            // raised preset chunk by chunk the arcs grow with it — sampling past the
            // reached horizon would synchronously extend the ephemerides mid-lookup.
            double availableAheadDays = rails.AvailableAheadDays(now);
            if (availableAheadDays <= 0) return; // rails still catching up to now (a load)
            double windowDays = OverlayKernel.CelestialWindowSeconds(
                config.CelestialCurveDays, availableAheadDays) / 86400.0;
            foreach (string bodyId in curveBodyIds)
            {
                if (ShouldStop()) return;
                try
                {
                    if (rails.ParentIdOf(bodyId) is not { } parentId) continue; // root: no line
                    var (bodyNow, parentNow) = rails.GetGameEclPair(bodyId, parentId, now);
                    double period = AdaptiveSampler.PeriodSeconds(rails.MuOf(parentId),
                        bodyNow.Position - parentNow.Position, bodyNow.Velocity - parentNow.Velocity);
                    double window = windowDays * 86400.0;

                    Vector3d SampleDrawn(double t)
                    {
                        if (!framed)
                        {
                            var (body, parent) = rails.GetGameEclPair(bodyId, parentId, t);
                            return body.Position - parent.Position; // inertial: parent-relative Cce
                        }
                        if (!FrameManager.TrySamplePoseForCurve(frameSnapshot, t, out var pose))
                            throw new InvalidOperationException("frame pose unavailable");
                        return pose.ToFrame(rails.GetGameEcl(bodyId, t).Position);
                    }

                    // Every segment uses the configured turn bound. Multi-period
                    // windows retain that quality and truncate at the dense budget.
                    int maxDense = Math.Clamp(config.CelestialMaxPoints,
                        OverlayKernel.StockPointBufferLength, 65536);
                    var sampled = AdaptiveSampler.Sample(SampleDrawn, now, now + window,
                        maxDense, thetaMax, dtMinSeconds: 1.0, period,
                        shouldStop: ShouldStop);
                    if (ShouldStop()) return;
                    // Celestial ephemerides are precomputed and should not hit a
                    // point-mass singularity during display sampling. Do not publish a
                    // potentially one-point partial curve; this cosmetic body's previous
                    // curve ages out to its display-only stock route.
                    if (sampled.DynamicsLimited)
                        throw new IntegrationFailureException(
                            $"celestial trajectory integration failed for '{bodyId}'");
                    bool logTruncation = false;
                    if (sampled.Truncated
                        && !TryMutateCurrentGeneration(generation, cancellationToken,
                            () => logTruncation = _truncationLogged.Add(bodyId)))
                        return;
                    if (logTruncation)
                    {
                        ModLog.Info($"celestial curves: point budget truncated '{bodyId}' at "
                            + $"{(sampled.Times[^1] - now) / 86400.0:F1} d of the {window / 86400.0:F0} d "
                            + $"window (celestial_max_points={maxDense}; affects fast orbits "
                            + "under long windows — raise it or overlay_max_turn_deg "
                            + "for more coverage)");
                    }
                    // Decimated stock-length subset for the staged pick buffer; the
                    // dense sweep publishes unpadded alongside (the draw's surface).
                    var indices = OverlayKernel.DecimateIndices(
                        sampled.Times.Length, OverlayKernel.StockPointBufferLength);
                    var times = OverlayKernel.TakeAt(sampled.Times, indices);
                    var coordinates = OverlayKernel.TakeAt(sampled.Positions, indices);
                    var curve = new CurveSamples
                    {
                        BodyId = bodyId,
                        ParentId = parentId,
                        SampleWallMs = nowWall,
                        Times = OverlayKernel.PadToStockLength(times),
                        Coordinates = OverlayKernel.PadToStockLength(coordinates),
                        FrameLabel = active?.Label,
                        SampleT0 = now,
                        DenseTimes = sampled.Times,
                        DenseCoordinates = sampled.Positions,
                        DenseMetrics = DecimationMetrics.For(
                            sampled.Positions, MaximumTraversalPoints),
                    };
                    if (!TryPublishCurve(bodyId, curve, generation, cancellationToken))
                        return;
                }
                catch (Exception e)
                {
                    if (ShouldStop()) return;
                    // Per-body containment: one throwing body (frame pose gone
                    // mid-loop, an ephemerides edge) must not starve the rest of the
                    // set. The body's prior curve ages out; the outer catch stays
                    // as the backstop for pre/post-loop failures.
                    FrameManager.NoteContained("celestial curve sampling '" + bodyId + "'", e);
                    continue;
                }
            }
            // Updated only after a COMPLETED loop: a mid-loop throw (outer catch) must
            // leave the label bypass armed so the next 500 ms cycle retries at once.
            TryMutateCurrentGeneration(generation, cancellationToken, () =>
            {
                _lastSampledFrameLabel = active?.Label;
                _samplingCompletion = new(windowDays, active?.Label);
            });
        }
        catch (Exception e)
        {
            if (ShouldStop()) return;
            FrameManager.NoteContained("celestial curve sampling", e);
        }
    }

    /// <summary>Context routing for one celestial (no side effects): Draw when a fresh
    /// matching-mode curve exists; Blink when the curve is fresh but the frame MODE
    /// changed since sampling (the ≤1 s until the worker resamples); Stock when
    /// there is no usable curve at all (never sampled / stale; disabled is the
    /// caller's gate). A celestial's parent is parse-time constant, so the vessel-side
    /// SOI re-anchor case cannot arise here (RouteLine takes no parent input
    /// at all — SOI-independence).</summary>
    public static LineRoute Route(KSA.Celestial celestial)
    {
        if (!Curves.TryGetValue(celestial.Id, out var curve)) return LineRoute.Stock;
        bool fresh = OverlayKernel.SamplesUsable(curve.SampleWallMs, Environment.TickCount64);
        bool modeMatches = OverlayKernel.ModeMatches(curve.FrameLabel, FrameManager.Active?.Label);
        return OverlayKernel.RouteLine(true, fresh, modeMatches);
    }

    /// <summary>Render-phase staging for one celestial (called by CelestialLinePatch on
    /// the Draw route — <see cref="Route"/> already answered freshness and mode): stage
    /// its sampled arc as the body's orbit-line points (parent-anchored Cce, stock
    /// payload shapes). Frame batches re-embed at the current pose; inertial batches are
    /// parent-relative already. <paramref name="staged"/> hands the prefix everything
    /// its dense OrbitLinePass draw needs — the unpadded sweep plus the SAME pose/parent
    /// resolution the pick buffer was just staged with. Returns false when a stage-time
    /// surprise (curve pruned since Route, parentless orbit, frame pose gone) means
    /// nothing was staged: the caller's terminal stock path.</summary>
    internal static bool StageFresh(KSA.Celestial celestial, RailsService rails, out StagedCelestial staged)
    {
        staged = default;
        if (!Curves.TryGetValue(celestial.Id, out var curve)) return false;
        if (celestial.Orbit?.Parent is not KSA.Astronomical) return false;

        // Frame mode reads its pose and current parent BEFORE the buffer exists (the
        // MemoryOwner ownership rule: allocate -> UpdateCachedPoints handoff, with no
        // bail-out path in between — never both dispose and hand off).
        double now = KSA.Universe.GetElapsedSimTime().Seconds();
        if (LastStagedFrame.TryGetValue(celestial.Id, out var memo)
            && ReferenceEquals(memo.Curve, curve)
            && memo.NowSeconds.Equals(now)
            && celestial.Orbit.LineCount == curve.Times.Length
            && celestial.Orbit.CachedPoints.Length > 0
            && double.IsNaN(celestial.Orbit.CachedPoints[0].CompassTrueAnomaly)
            && celestial.Orbit.CachedPoints[0].PositionCce.Equals(memo.FirstPosition))
        {
            staged = memo.Staged;
            return true;
        }

        // Per-frame re-stage skip: the SAME inertial curve is already in the cache
        // (LastStagedCurve reference), the restore debt is still recorded, and the
        // cache still holds OUR points — verified by BOTH the NaN true anomaly (mod
        // payloads only; stock fills write finite TAs) AND slot 0's position being
        // bit-identical to the curve's first staged point (a stock refill cannot
        // reproduce it; closes the exotic case of a partial stock fill leaving
        // recycled pooled NaN-TA slots untouched). Framed curves restage every
        // frame (their payloads and embed are now-anchored).
        if (curve.FrameLabel is null
            && StagedIds.ContainsKey(celestial.Id)
            && LastStagedCurve.TryGetValue(celestial.Id, out var lastStaged)
            && ReferenceEquals(lastStaged, curve)
            && celestial.Orbit.LineCount == curve.Times.Length
            && double.IsNaN(celestial.Orbit.CachedPoints[0].CompassTrueAnomaly)
            && celestial.Orbit.CachedPoints[0].PositionCce.Equals(FrameAdapter.ToGame(curve.Coordinates[0])))
        {
            staged = new StagedCelestial
            {
                DenseTimes = curve.DenseTimes,
                DenseCoordinates = curve.DenseCoordinates,
                DenseMetrics = curve.DenseMetrics,
                Context = new TrajectoryOverlay.StagingContext
                {
                    Framed = false,
                    NowPose = default,
                    FrameParentNow = default,
                    ParentShift = Vector3d.Zero,
                    AnchorPeSeconds = double.NaN,
                },
                NowSeconds = now,
            };
            return true;
        }
        FramePose nowPose = default;
        Vector3d parentNow = default;
        if (curve.FrameLabel is not null)
        {
            if (!FrameManager.TryCaptureActive(out var frameSnapshot)
                || !OverlayKernel.ModeMatches(curve.FrameLabel, frameSnapshot.Spec.Label)
                || !FrameManager.TrySamplePoseForDisplay(frameSnapshot, now, out nowPose))
                return false;
            parentNow = rails.GetGameEcl(curve.ParentId, now).Position;
        }

        int n = curve.Times.Length;
        var points = CommunityToolkit.HighPerformance.Buffers.MemoryOwner<KSA.OrbitPointCce>.Allocate(n);
        var span = points.Span;
        if (curve.FrameLabel is null)
        {
            // Inertial: TimeSincePe/RemainingTimeTo use the batch's own epoch (SampleT0)
            // so values are stable across the batch's ~1 s of restaging.
            for (int i = 0; i < n; i++)
                span[i] = new KSA.OrbitPointCce(
                    FrameAdapter.ToGame(curve.Coordinates[i]),
                    new KSA.SimTime(curve.Times[i] - curve.SampleT0),
                    new KSA.SimTime(curve.Times[i] - curve.SampleT0),
                    KSA.TrueAnomaly.NaN);
        }
        else
        {
            for (int i = 0; i < n; i++)
            {
                var image = nowPose.FromFrame(curve.Coordinates[i]);
                span[i] = new KSA.OrbitPointCce(
                    FrameAdapter.ToGame(image - parentNow),
                    new KSA.SimTime(curve.Times[i] - now),
                    new KSA.SimTime(curve.Times[i] - now),
                    KSA.TrueAnomaly.NaN);
            }
        }
        // Restore debt recorded BEFORE the overwrite so it is unconditional: a throw
        // between the two leaves a restore owed for an un-overwritten line — a harmless
        // one-time RegenerateOrbitLines.
        StagedIds[celestial.Id] = 1;
        var firstPosition = span[0].PositionCce;
        celestial.Orbit.UpdateCachedPoints(points);
        if (curve.FrameLabel is null) LastStagedCurve[celestial.Id] = curve;
        else LastStagedCurve.TryRemove(celestial.Id, out _);
        staged = new StagedCelestial
        {
            DenseTimes = curve.DenseTimes,
            DenseCoordinates = curve.DenseCoordinates,
            DenseMetrics = curve.DenseMetrics,
            Context = new TrajectoryOverlay.StagingContext
            {
                Framed = curve.FrameLabel is not null,
                NowPose = nowPose,
                FrameParentNow = parentNow,
                ParentShift = Vector3d.Zero,
                AnchorPeSeconds = double.NaN,
            },
            NowSeconds = now,
        };
        LastStagedFrame[celestial.Id] = new StageMemo(curve, now, staged, firstPosition);
        return true;
    }

    /// <summary>Deactivation hand-back (see class doc): restore each overwritten line
    /// once, through the game's own path, whenever staging is not happening.</summary>
    public static void TryRestoreStock(KSA.Celestial celestial)
    {
        if (!StagedIds.TryRemove(celestial.Id, out _)) return;
        LastStagedCurve.TryRemove(celestial.Id, out _); // skip cache dies with the debt
        LastStagedFrame.TryRemove(celestial.Id, out _);
        if (celestial.Orbit is null) return;
        celestial.RegenerateOrbitLines();
        ModLog.Info($"celestial '{celestial.Id}': stock orbit line restored (mod line inactive)");
    }
}

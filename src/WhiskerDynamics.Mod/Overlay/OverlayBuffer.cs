using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Honest line markers: what kind of point a
/// marker flags on a drawn vessel trajectory. Collision is the surface-frame
/// impact point — the sample where the drawn line was cut at the frame body's
/// surface, its label carrying the time remaining and surface-relative impact speed.
/// Encounter/Escape
/// are predictor SOI crossings; ClosestApproach is a local separation minimum to the
/// controlled vessel's selected vessel/body target.</summary>
public enum OverlayMarkerKind
{
    Apoapsis, Periapsis, AscendingNode, DescendingNode, Collision,
    Encounter, Escape, ClosestApproach,
}

/// <summary>One marker on a drawn vessel line, computed from the SAMPLED batch (never
/// a conic): the first upcoming point of its kind relative to <paramref name="BodyId"/>.
/// The render side derives the screen position from the batch at the marker's TIME
/// (TrajectoryOverlay.TryDrawnPositionAt), so markers ride the re-embedded line in
/// frame views. AltitudeMeters is distance-to-center minus the body's mean radius
/// for Ap/Pe, closest separation for closest approach, and 0 for nodes/transitions.
/// Label is precomposed at computation time and refreshed at render time for its
/// countdown. RelativeSpeedMetersPerSecond is relative to the marker's referenced
/// body/target; collision retains its separate surface-relative speed.
/// ImpactSpeedMetersPerSecond retains collision data needed to refresh its countdown
/// when deterministic planned geometry is restamped.</summary>
public sealed record OverlayMarker(
    OverlayMarkerKind Kind, string BodyId, double TimeSeconds, double AltitudeMeters, string Label,
    double? ImpactSpeedMetersPerSecond = null,
    double? RelativeSpeedMetersPerSecond = null);

/// <summary>One immutable batch of overlay samples: everything the render-phase restage
/// needs to rebuild the point cache without touching the predictor or the rails. The
/// OrbitPointCce payloads (PositionsCce/TimesSincePe/RemainingTimesTo) reproduce the
/// overlay staging bit-for-bit; every array is padded to stock length at publish, with
/// PointCount preserving the real pre-padding sample count (the hover substitute scans
/// only the real prefix). Immutability is also the thread-safety story: the hover job
/// (a ConcurrentWorkers worker) reads these arrays while the render thread stages —
/// safe because nothing ever mutates a published batch.</summary>
public sealed record OverlaySamples
{
    public required string VesselId { get; init; }
    public required double SampleT0 { get; init; }
    /// <summary>Epoch of the sampled sweep. Unlike SampleT0 this survives geometry
    /// reuse and therefore drives its age cap.</summary>
    public required double FutureStartSeconds { get; init; }
    public required long SampleWallMs { get; init; }
    /// <summary>Universe simulation epoch read by the producer during this rebuild's
    /// capture/enqueue phase. It is deliberately distinct from SampleT0/state time:
    /// at high warp the committed vessel state may lag Universe while the captured
    /// inputs are nevertheless current.</summary>
    public required double CaptureSimSeconds { get; init; }
    /// <summary>Fresh absolute physical coast state at <see cref="SampleT0"/>.
    /// Target-vessel CA prediction seeds from actual batches rather than from display
    /// samples, whose density can collapse when the target is stationary in its own
    /// frame. A planned batch retained during live divergence instead keeps its last
    /// honest plan-world anchor while its immutable future geometry is cheaply
    /// restamped; planned batches are never target-prediction seeds.</summary>
    public required StateVector AnchorState { get; init; }
    public required double[] Times { get; init; }
    public required double[] TimesSincePe { get; init; }
    public required double[] RemainingTimesTo { get; init; }
    public required Vector3d[] PositionsCce { get; init; }
    public required string ParentId { get; init; }
    /// <summary>Real sample count before pad-to-stock-length (the padded tail repeats
    /// the last sample — harmless to draw, waste to scan).</summary>
    public required int PointCount { get; init; }
    /// <summary>The adaptive sweep ran out of point budget before the horizon
    /// (surfaced in the overlay status note).</summary>
    public required bool Truncated { get; init; }
    /// <summary>The sampler stopped at its work limit. Geometry up to the last valid
    /// sample remains drawable.</summary>
    public bool WorkLimited { get; init; }
    /// <summary>The integrator could not advance through the next point. Geometry up
    /// to the last valid sample remains drawable.</summary>
    public bool DynamicsLimited { get; init; }
    /// <summary>The horizon the sweep was BUILT for. The reuse rule compares the
    /// current horizon against it: a truncated batch under an UNCHANGED window is
    /// reusable (the budget already ran out under this horizon's capped bound — a
    /// resample reproduces the same coverage), but a GROWN window resamples once so
    /// the batch identity reflects the new request.</summary>
    public required double HorizonSeconds { get; init; }
    /// <summary>Sampling policy that produced the immutable geometry. Live edits to
    /// any value invalidate actual-line reuse immediately.</summary>
    public required double SamplingThetaMax { get; init; }
    public required int SamplingMaxDensePoints { get; init; }
    /// <summary>Honest line markers for this batch (first upcoming Ap/Pe per
    /// frame-relevant body + AN/DN vs the mode's natural plane) — see
    /// TrajectoryOverlay's marker computation. Empty when none resolved.</summary>
    public required IReadOnlyList<OverlayMarker> Markers { get; init; }
    /// <summary>All deterministic events found over the immutable geometry. Markers is
    /// the cheap now-filtered projection (first upcoming Ap/Pe/nodes, all future
    /// transitions/closest approaches). Planned restamps reuse this list verbatim.</summary>
    public IReadOnlyList<OverlayMarker> MarkerCandidates { get; init; } = [];
    /// <summary>Identity of the marker-work state and target trajectory used for candidates.
    /// Celestial targets are stable by id; vessel targets key to their immutable dense
    /// geometry array, which survives ordinary actual-batch restamps.</summary>
    public object? MarkerCacheKey { get; init; }
    /// <summary>Worker-side sampled-series orbit analysis frozen to the SOI body at
    /// the requested interval's first timestamp. Null until its equatorial pole and
    /// enough honest future samples are available.</summary>
    public OrbitAnalysisReport? Analysis { get; init; }
    /// <summary>Panel-ready reason when Analysis is unavailable; null when analysis
    /// was not requested for this batch (non-controlled/planned trajectories).</summary>
    public string? AnalysisUnavailableReason { get; init; }
    /// <summary>UI request identity this report/absence answers; zero when analysis was not requested.</summary>
    public int AnalysisRequestVersion { get; init; }
    /// <summary>Whether this controlled-vessel batch answers the attached analyser
    /// request. The analyser has its own coarse sweep and does not retain that
    /// temporary series after producing the report.</summary>
    public bool AnalysisRequested { get; init; }

    // Frame-curve source data. Non-null only when a display frame was active at
    // sampling time. FrameLabel identifies WHICH frame the coordinates belong to:
    // a label mismatch at the draw site BLINKS the line
    // (LineRoute.Blink) and triggers an immediate context-change rebuild (~1 tick);
    // TrajectoryOverlay.Stage's inertial fallback remains for the job-thread staging
    // call, where a frame switch can land between sampling and staging.
    public Vector3d[]? FrameCoordinates { get; init; }
    public string? FrameLabel { get; init; }

    // Honest-density lines: the FULL adaptive sweep, unpadded —
    // the mod-owned OrbitLinePass draw and the burn-node/marker time interpolation
    // read these; the padded stock-length arrays above are a DECIMATED SUBSET of
    // them (OverlayKernel.DecimateIndices) kept for every stock-shaped reader
    // (hover job, click payloads, ground track). Same immutability contract.
    public required double[] DenseTimes { get; init; }
    public required Vector3d[] DensePositionsCce { get; init; }
    /// <summary>Dense frame-curve coordinates; null exactly when
    /// <see cref="FrameCoordinates"/> is null (one sweep, one mode).</summary>
    public Vector3d[]? DenseFrameCoordinates { get; init; }
    /// <summary>Emit-filter metrics (cumulative arc + DP significance,
    /// <see cref="DecimationMetrics"/>) over the dense sweep in its DRAWN coordinate
    /// space, computed once on the worker — the per-frame screen-space emit
    /// filter's inputs.</summary>
    public required DecimationMetrics DenseMetrics { get; init; }
    /// <summary>Metrics over the Cce array specifically — the SAME reference as
    /// <see cref="DenseMetrics"/> for inertial batches, a separate meters-space
    /// pair for framed ones: a framed batch drawn through the pose-failure
    /// INERTIAL fallback reads DensePositionsCce, and measuring meter chords
    /// against separation-normalized values would starve the emit filter (the
    /// line would thin toward its endpoints).</summary>
    public required DecimationMetrics DenseMetricsCce { get; init; }

    /// <summary>Republish with a fresh wall stamp while retaining CaptureSimSeconds.
    /// This is a wall-liveness restamp, never a claim that old geometry was captured
    /// at the current Universe epoch. Planned restamps receive a new capture epoch
    /// explicitly at their call site.</summary>
    public OverlaySamples WithFreshStamp(long nowWallMs) => this with { SampleWallMs = nowWallMs };

    /// <summary>Drop every analyser-only payload while preserving draw geometry.</summary>
    public OverlaySamples WithoutAnalysis() => this with
    {
        Analysis = null,
        AnalysisUnavailableReason = null,
        AnalysisRequestVersion = 0,
        AnalysisRequested = false,
    };
}

/// <summary>Per-vessel publish/read handoff (honest orbit lines: EVERY tracked vessel
/// keeps a batch, not just the controlled one). Values are immutable after
/// construction, so replacing a slot's reference can never tear; the dictionary itself
/// is the only shared state (ConcurrentDictionary).</summary>
public static class OverlayBuffer
{
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, OverlaySamples> Current =
        new(StringComparer.Ordinal);
    private static readonly object AnalysisPublishGate = new();
    private static readonly object SessionPublishGate = new();

    /// <summary>Two-line display: the PLANNED trajectory batch —
    /// the vessel's path with all in-window burns folded in, sampled from the first
    /// burn onward — published alongside the actual batch by the same rebuild and
    /// drawn in the planned-burn color by VesselLinePatch. Absent whenever the vessel
    /// has no in-window burns (the actual line is then the whole story).</summary>
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, OverlaySamples> Planned =
        new(StringComparer.Ordinal);

    public static void Publish(OverlaySamples samples) => PublishGeometry(samples);

    /// <summary>Publishes fresh geometry without allowing it to erase a concurrently
    /// completed analyser result. The compare-and-swap loop linearizes geometry
    /// replacement against <see cref="TryPublishAnalysis"/>; whichever writer wins
    /// first, the other retries against that immutable value.</summary>
    internal static void PublishGeometry(OverlaySamples samples)
    {
        lock (AnalysisPublishGate)
        {
            while (true)
            {
                Current.TryGetValue(samples.VesselId, out var current);
                var merged = CarryAnalysisForward(samples, current);
                bool published = current is null
                    ? Current.TryAdd(samples.VesselId, merged)
                    : Current.TryUpdate(samples.VesselId, merged, current);
                if (!published) continue;
                AdoptLinePublication(merged, planned: false);
                return;
            }
        }
    }

    internal static OverlaySamples CarryAnalysisForward(
        OverlaySamples geometry, OverlaySamples? current)
    {
        if (geometry.AnalysisRequestVersion != 0
            || geometry.AnalysisRequested
            || geometry.Analysis is not null
            || geometry.AnalysisUnavailableReason is not null)
            geometry = geometry.WithoutAnalysis();
        if (current is null
            || current.AnalysisRequestVersion == 0
            || !string.Equals(current.VesselId, geometry.VesselId, StringComparison.Ordinal)
            || !Ui.OrbitAnalyserPanel.RequestMatches(current.AnalysisRequestVersion))
            return geometry;
        return geometry with
        {
            Analysis = current.Analysis,
            AnalysisUnavailableReason = current.AnalysisUnavailableReason,
            AnalysisRequestVersion = current.AnalysisRequestVersion,
            AnalysisRequested = true,
        };
    }

    /// <summary>Attaches an analyser-only result to the newest vessel geometry.
    /// Request and worker generation are rechecked at the atomic handoff. Geometry
    /// parent is intentionally independent: the report freezes and names the SOI
    /// body that owned the vessel at its requested start timestamp.</summary>
    internal static bool TryPublishAnalysis(string vesselId, int requestVersion,
        OrbitAnalysisReport? report, string? reason, int generation)
    {
        lock (SessionPublishGate)
        {
            if (generation != OverlayWorker.CurrentGeneration) return false;
            lock (AnalysisPublishGate)
            {
                if (!Ui.OrbitAnalyserPanel.RequestMatches(requestVersion)) return false;
                while (Current.TryGetValue(vesselId, out var current))
                {
                    var updated = current with
                    {
                        Analysis = report,
                        AnalysisUnavailableReason = reason,
                        AnalysisRequestVersion = requestVersion,
                        AnalysisRequested = true,
                    };
                    if (Current.TryUpdate(vesselId, updated, current))
                    {
                        AdoptLinePublication(updated, planned: false);
                        return true;
                    }
                }
                return false;
            }
        }
    }

    /// <summary>Worker publication guarded by the session generation. The check and
    /// slot replacement share a gate with the session clear, so a rebuild captured
    /// before a load can neither republish after that clear nor retract a newer
    /// session's line.</summary>
    internal static bool PublishIfCurrent(OverlaySamples samples, int generation)
    {
        lock (SessionPublishGate)
        {
            if (generation != OverlayWorker.CurrentGeneration) return false;
            Publish(samples);
            return true;
        }
    }

    public static OverlaySamples? Read(string vesselId) =>
        Current.TryGetValue(vesselId, out var samples) ? samples : null;

    public static void PublishPlanned(OverlaySamples samples)
    {
        Planned[samples.VesselId] = samples;
        AdoptLinePublication(samples, planned: true);
    }

    internal static bool PublishPlannedIfCurrent(OverlaySamples samples, int generation)
    {
        lock (SessionPublishGate)
        {
            if (generation != OverlayWorker.CurrentGeneration) return false;
            PublishPlanned(samples);
            return true;
        }
    }

    internal sealed record RebuildLease(
        string VesselId, long Id, int Generation, long StartedMs, long ExpiresMs);

    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, RebuildLease>
        RebuildLeases = new(StringComparer.Ordinal);
    private static long _nextRebuildLeaseId;
    internal const long RebuildLeaseMaxAgeMs = 30_000;

    /// <summary>Bounded stale-while-revalidate lease. Repeated latest-wins captures
    /// retain the first request's deadline, so a stuck worker cannot keep fossil
    /// geometry alive forever. Only the newest request may end the lease.</summary>
    internal static RebuildLease? BeginRebuildLease(string vesselId, int generation, long nowMs)
    {
        lock (SessionPublishGate)
        {
            if (generation != OverlayWorker.CurrentGeneration) return null;
            long id = Interlocked.Increment(ref _nextRebuildLeaseId);
            while (true)
            {
                if (RebuildLeases.TryGetValue(vesselId, out var current)
                    && current.Generation == generation)
                {
                    var replacement = current with { Id = id };
                    if (RebuildLeases.TryUpdate(vesselId, replacement, current)) return replacement;
                    continue;
                }
                var created = new RebuildLease(vesselId, id, generation, nowMs,
                    nowMs + RebuildLeaseMaxAgeMs);
                if (current is null)
                {
                    if (RebuildLeases.TryAdd(vesselId, created)) return created;
                }
                else if (RebuildLeases.TryUpdate(vesselId, created, current))
                {
                    return created;
                }
            }
        }
    }

    internal static void EndRebuildLease(RebuildLease lease)
    {
        if (!RebuildLeases.TryGetValue(lease.VesselId, out var current)
            || current.Id != lease.Id) return;
        ((ICollection<KeyValuePair<string, RebuildLease>>)RebuildLeases)
            .Remove(new KeyValuePair<string, RebuildLease>(lease.VesselId, current));
    }

    internal static bool IsRebuildLeased(string vesselId, long nowMs) =>
        RebuildLeases.TryGetValue(vesselId, out var lease)
        && lease.Generation == OverlayWorker.CurrentGeneration
        && nowMs <= lease.ExpiresMs;

    internal sealed record LineLease(
        string VesselId, long Id, int Generation,
        OverlaySamples? Actual, OverlaySamples? Planned, long ExpiresMs);

    private static readonly System.Collections.Concurrent.ConcurrentDictionary<string, LineLease>
        LineLeases = new(StringComparer.Ordinal);
    private static long _nextLineLeaseId;

    /// <summary>Optimizer-only display lease for the exact batches visible when the
    /// solve starts. It affects line drawing only; hover, markers, burn nodes, and
    /// analysis retain the normal freshness contract. Official publications in the
    /// same session advance the leased identity.</summary>
    internal static LineLease BeginLineLease(string vesselId, int generation, long nowMs)
    {
        var lease = new LineLease(vesselId, Interlocked.Increment(ref _nextLineLeaseId),
            generation, Read(vesselId), ReadPlanned(vesselId),
            nowMs + OverlayKernel.RestageMaxAgeMs);
        LineLeases[vesselId] = lease;
        // Close the capture/install race: a publication between either Read above
        // and the dictionary write saw no lease to adopt. Re-read after installation;
        // publications after these reads adopt themselves in Publish*.
        if (Read(vesselId) is { } actual) AdoptLinePublication(actual, planned: false);
        if (ReadPlanned(vesselId) is { } planned) AdoptLinePublication(planned, planned: true);
        return lease;
    }

    internal static void RenewLineLease(LineLease lease, long nowMs)
    {
        if (lease.Generation != OverlayWorker.CurrentGeneration) return;
        if (!LineLeases.TryGetValue(lease.VesselId, out var current)
            || current.Id != lease.Id) return;
        LineLeases.TryUpdate(lease.VesselId,
            current with { ExpiresMs = nowMs + OverlayKernel.RestageMaxAgeMs }, current);
    }

    internal static void EndLineLease(LineLease lease)
    {
        if (!LineLeases.TryGetValue(lease.VesselId, out var current)
            || current.Id != lease.Id) return;
        ((ICollection<KeyValuePair<string, LineLease>>)LineLeases)
            .Remove(new KeyValuePair<string, LineLease>(lease.VesselId, current));
    }

    private static void AdoptLinePublication(OverlaySamples samples, bool planned)
    {
        while (LineLeases.TryGetValue(samples.VesselId, out var current))
        {
            if (current.Generation != OverlayWorker.CurrentGeneration) return;
            var updated = planned
                ? current with { Planned = samples }
                : current with { Actual = samples };
            if (LineLeases.TryUpdate(samples.VesselId, updated, current)) return;
        }
    }

    internal static bool IsLineLeased(
        string vesselId, OverlaySamples samples, bool planned, long nowMs) =>
        LineLeases.TryGetValue(vesselId, out var lease)
        && lease.Generation == OverlayWorker.CurrentGeneration
        && nowMs <= lease.ExpiresMs
        && ReferenceEquals(samples, planned ? lease.Planned : lease.Actual);

    internal static bool LineSamplesUsable(
        string vesselId, OverlaySamples samples, bool planned, long nowMs,
        double nowSimSeconds)
    {
        // Lines are absolute sampled trajectories. Keep the last complete geometry
        // only while it still reaches now and while wall freshness or a bounded
        // display/rebuild lease owns it.
        if (!OverlayKernel.LineGeometryCovers(samples, nowSimSeconds))
            return false;
        if (OverlayKernel.SamplesUsable(samples.SampleWallMs, nowMs)) return true;
        return IsLineLeased(vesselId, samples, planned, nowMs)
            || IsRebuildLeased(vesselId, nowMs);
    }

    /// <summary>Interactive/marker freshness follows the same coverage and wall-
    /// liveness contract as the line. The optimizer-only line lease is deliberately
    /// excluded: it freezes display geometry without keeping click payloads live.</summary>
    internal static bool ConsumerSamplesUsable(
        string vesselId, OverlaySamples samples, long nowMs, double nowSimSeconds) =>
        OverlayKernel.LineGeometryCovers(samples, nowSimSeconds)
        && (OverlayKernel.SamplesUsable(samples.SampleWallMs, nowMs)
            || IsRebuildLeased(vesselId, nowMs));

    public static void ClearPlanned(string vesselId) => Planned.TryRemove(vesselId, out _);

    /// <summary>Revokes every display-ownership artifact for one vessel. Callers first
    /// cancel both worker tickets; publication already inside either ticket's atomic
    /// gate finishes before this transaction. A continuity transition may retain the
    /// last drawable geometry, but never its coast-specific analyser payload.</summary>
    internal static void RevokeVessel(string vesselId, bool clearSamples)
    {
        lock (SessionPublishGate)
        {
            RebuildLeases.TryRemove(vesselId, out _);
            LineLeases.TryRemove(vesselId, out _);
            lock (AnalysisPublishGate)
            {
                if (clearSamples)
                {
                    Current.TryRemove(vesselId, out _);
                }
                else if (Current.TryGetValue(vesselId, out var current))
                {
                    Current.TryUpdate(vesselId, current.WithoutAnalysis(), current);
                }
            }
            if (!clearSamples) return;
            Planned.TryRemove(vesselId, out _);
        }
    }

    internal static bool ClearPlannedIfCurrent(string vesselId, int generation)
    {
        lock (SessionPublishGate)
        {
            if (generation != OverlayWorker.CurrentGeneration) return false;
            ClearPlanned(vesselId);
            return true;
        }
    }

    public static OverlaySamples? ReadPlanned(string vesselId) =>
        Planned.TryGetValue(vesselId, out var samples) ? samples : null;

    /// <summary>The interactive-consumer gate, fused with the read: null when absent,
    /// outside sampled coverage, or past wall liveness without a rebuild lease.
    /// Hover, burn-node positions, and line markers share it; the draw path adds only
    /// the optimizer-specific line lease through <see cref='LineSamplesUsable'/>.</summary>
    public static OverlaySamples? ReadFresh(
        string vesselId, long nowMs, double nowSimSeconds)
    {
        var samples = Read(vesselId);
        return samples is not null
            && ConsumerSamplesUsable(vesselId, samples, nowMs, nowSimSeconds)
            ? samples : null;
    }

    /// <inheritdoc cref="ReadFresh"/>
    public static OverlaySamples? ReadPlannedFresh(
        string vesselId, long nowMs, double nowSimSeconds)
    {
        var samples = ReadPlanned(vesselId);
        return samples is not null
            && ConsumerSamplesUsable(vesselId, samples, nowMs, nowSimSeconds)
            ? samples : null;
    }

    /// <summary>Panel/log evidence: how many vessels currently hold a published batch.</summary>
    public static int PublishedCount => Current.Count;

    /// <summary>Close-window cleanup. Publish performs the same request check, so a
    /// racing in-flight batch cannot reintroduce the payload after this sweep.</summary>
    internal static void StripAnalysis()
    {
        lock (AnalysisPublishGate)
        {
            foreach (var pair in Current)
                if (pair.Value.AnalysisRequestVersion != 0)
                    Current.TryUpdate(pair.Key, pair.Value.WithoutAnalysis(), pair.Value);
        }
    }

    /// <summary>Session statics sweep: cross-session samples must never restage.</summary>
    internal static void ResetSessionStatics()
    {
        lock (SessionPublishGate)
        {
            Current.Clear();
            Planned.Clear();
            LineLeases.Clear();
            RebuildLeases.Clear();
        }
    }
}

namespace WhiskerDynamics.Mod.Soi;

/// <summary>Pure admission policy for the rails SOI sweep. Keeping cursor identity and
/// retention decisions outside KSA makes time reversal, reseeds and parent changes
/// independently testable.</summary>
internal static class SoiSweepAdmissionKernel
{
    internal enum Mode
    {
        Throttled,
        EndpointOnly,
        Sweep,
    }

    internal readonly record struct Decision(Mode CheckMode, double FromTime)
    {
        internal bool ShouldCheck => CheckMode != Mode.Throttled;
        internal bool CanSweep => CheckMode == Mode.Sweep;
    }

    internal static Decision Decide(double now, double periodSeconds,
        string parentId, object lineage,
        double cursorTime, string? cursorParentId, object? cursorLineage,
        double retainedPredictorStart, bool railsRetainsCursor)
    {
        bool sameCursor = double.IsFinite(cursorTime)
            && ReferenceEquals(cursorLineage, lineage)
            && string.Equals(cursorParentId, parentId, StringComparison.Ordinal);
        if (!sameCursor)
            return new Decision(Mode.EndpointOnly, now);

        double elapsed = now - cursorTime;
        if (elapsed >= 0 && elapsed < periodSeconds)
            return new Decision(Mode.Throttled, cursorTime);

        if (elapsed > 0 && cursorTime >= retainedPredictorStart && railsRetainsCursor)
            return new Decision(Mode.Sweep, cursorTime);

        // Reversed time and expired predictor/celestial history must never reuse an
        // interval with mismatched endpoints. The current endpoint remains safe.
        return new Decision(Mode.EndpointOnly, now);
    }
}

/// <summary>Index-only retained-node selection. Binary search bounds the lookup cost
/// even when a predictor has reached its two-million-node safety ceiling; only the
/// fixed output grid is subsequently visited.</summary>
internal static class SoiSweepGridKernel
{
    internal readonly record struct InteriorSelection(
        int FirstIndex, int InteriorCount, int SelectedCount)
    {
        internal int IndexAt(int selectedOffset)
        {
            if ((uint)selectedOffset >= (uint)SelectedCount)
                throw new ArgumentOutOfRangeException(nameof(selectedOffset));
            if (SelectedCount == 1)
                return FirstIndex + (InteriorCount - 1) / 2;
            return FirstIndex + (int)((long)selectedOffset * (InteriorCount - 1)
                / (SelectedCount - 1));
        }
    }

    internal static InteriorSelection SelectInterior(
        int nodeCount, Func<int, double> timeAt,
        double from, double to, int maxBaseSamples)
    {
        ArgumentNullException.ThrowIfNull(timeAt);
        if (nodeCount <= 0 || !(to > from) || maxBaseSamples <= 2)
            return default;

        int first = Bound(nodeCount, timeAt, from, strictlyGreater: true);
        int endExclusive = Bound(nodeCount, timeAt, to, strictlyGreater: false);
        int interiorCount = Math.Max(0, endExclusive - first);
        int capacity = maxBaseSamples - 2;
        return new InteriorSelection(first, interiorCount,
            Math.Min(interiorCount, capacity));
    }

    private static int Bound(int count, Func<int, double> timeAt,
        double boundary, bool strictlyGreater)
    {
        int lo = 0, hi = count;
        while (lo < hi)
        {
            int mid = lo + (hi - lo) / 2;
            double time = timeAt(mid);
            bool remainsLeft = strictlyGreater ? time <= boundary : time < boundary;
            if (remainsLeft) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }
}

/// <summary>A true sampling-work budget: every adaptive midpoint probe consumes one
/// unit whether or not that probe is retained in the final polyline.</summary>
internal struct SoiSweepWorkBudget
{
    internal SoiSweepWorkBudget(int available) => Remaining = Math.Max(0, available);

    internal int Remaining { get; private set; }
    internal int Consumed { get; private set; }

    internal bool TryConsume()
    {
        if (Remaining <= 0) return false;
        Remaining--;
        Consumed++;
        return true;
    }
}

/// <summary>Pure post-sweep policy. Sampled crossings provide ordered evidence, while
/// the exact final sample remains authoritative for the parent applied to the game and
/// for the next cursor identity.</summary>
internal static class SoiSweepReconciliationKernel
{
    internal readonly record struct Decision(
        string FinalParentId,
        string CursorParentId,
        double CursorTime,
        int EvidenceCrossingCount,
        bool ShouldLogEvidence,
        bool KernelTruncated,
        bool EvidenceTruncated,
        bool EndpointOverrodeSample);

    internal static Decision Reconcile(
        string sampledFinalParentId,
        string endpointParentId,
        double cursorTime,
        int crossingCount,
        bool kernelTruncated,
        int maxEvidenceCrossings)
    {
        ArgumentNullException.ThrowIfNull(sampledFinalParentId);
        ArgumentNullException.ThrowIfNull(endpointParentId);
        if (crossingCount < 0)
            throw new ArgumentOutOfRangeException(nameof(crossingCount));

        int shown = Math.Min(crossingCount, Math.Max(0, maxEvidenceCrossings));
        bool evidenceTruncated = kernelTruncated || shown < crossingCount;
        return new Decision(
            FinalParentId: endpointParentId,
            CursorParentId: endpointParentId,
            CursorTime: cursorTime,
            EvidenceCrossingCount: shown,
            ShouldLogEvidence: crossingCount > 0 || kernelTruncated,
            KernelTruncated: kernelTruncated,
            EvidenceTruncated: evidenceTruncated,
            EndpointOverrodeSample: !string.Equals(sampledFinalParentId,
                endpointParentId, StringComparison.Ordinal));
    }
}

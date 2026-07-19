using WhiskerDynamics.Mod;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Soi;

public class SoiSweepAdmissionTests
{
    private static readonly object Lineage = new();

    private static SoiSweepAdmissionKernel.Decision Decide(
        double now = 110, double cursor = 100,
        string parent = nameof(Decide), string? cursorParent = nameof(Decide),
        object? lineage = null, object? cursorLineage = null,
        double retainedStart = 0, bool railsRetainsCursor = true) =>
        SoiSweepAdmissionKernel.Decide(now, 10, parent, lineage ?? Lineage,
            cursor, cursorParent, cursorLineage ?? Lineage,
            retainedStart, railsRetainsCursor);

    [Fact]
    public void MatchingForwardCursor_AtCadence_Sweeps()
    {
        var decision = Decide();
        Assert.Equal(SoiSweepAdmissionKernel.Mode.Sweep, decision.CheckMode);
        Assert.Equal(100, decision.FromTime);
    }

    [Fact]
    public void MatchingForwardCursor_InsideCadence_Throttles()
    {
        var decision = Decide(now: 109.999);
        Assert.Equal(SoiSweepAdmissionKernel.Mode.Throttled, decision.CheckMode);
        Assert.False(decision.ShouldCheck);
    }

    [Fact]
    public void MissingInitialCursor_ChecksEndpoint()
    {
        var decision = Decide(cursor: double.NegativeInfinity, cursorParent: null,
            cursorLineage: new object());
        Assert.Equal(SoiSweepAdmissionKernel.Mode.EndpointOnly, decision.CheckMode);
    }

    [Fact]
    public void ParentChange_InvalidatesHistoricalInterval()
    {
        var decision = Decide(parent: nameof(ParentChange_InvalidatesHistoricalInterval));
        Assert.Equal(SoiSweepAdmissionKernel.Mode.EndpointOnly, decision.CheckMode);
    }

    [Fact]
    public void PredictorReplacement_InvalidatesHistoricalInterval()
    {
        var decision = Decide(lineage: new object());
        Assert.Equal(SoiSweepAdmissionKernel.Mode.EndpointOnly, decision.CheckMode);
    }

    [Fact]
    public void TimeReversal_ChecksEndpointInsteadOfThrottlingOrSweeping()
    {
        var decision = Decide(now: 90);
        Assert.Equal(SoiSweepAdmissionKernel.Mode.EndpointOnly, decision.CheckMode);
        Assert.Equal(90, decision.FromTime);
    }

    [Fact]
    public void PredictorHistoryPrunedPastCursor_ChecksEndpoint()
    {
        var decision = Decide(retainedStart: 100.001);
        Assert.Equal(SoiSweepAdmissionKernel.Mode.EndpointOnly, decision.CheckMode);
    }

    [Fact]
    public void CelestialHistoryPrunedPastCursor_ChecksEndpoint()
    {
        var decision = Decide(railsRetainsCursor: false);
        Assert.Equal(SoiSweepAdmissionKernel.Mode.EndpointOnly, decision.CheckMode);
    }

    [Fact]
    public void TwoMillionNodeHistory_UsesLogarithmicLookupAndFixedGrid()
    {
        int calls = 0;
        double TimeAt(int index)
        {
            calls++;
            return index;
        }

        var selection = SoiSweepGridKernel.SelectInterior(
            2_000_000, TimeAt, 100, 1_999_900, maxBaseSamples: 257);

        Assert.Equal(101, selection.FirstIndex);
        Assert.Equal(1_999_799, selection.InteriorCount);
        Assert.Equal(255, selection.SelectedCount);
        Assert.Equal(101, selection.IndexAt(0));
        Assert.Equal(1_999_899, selection.IndexAt(254));
        Assert.InRange(calls, 1, 44);
        int previous = -1;
        for (int i = 0; i < selection.SelectedCount; i++)
        {
            int current = selection.IndexAt(i);
            Assert.True(current > previous);
            previous = current;
        }
    }

    [Fact]
    public void SmallInteriorRange_RetainsEveryNodeStrictlyInsideEndpoints()
    {
        var selection = SoiSweepGridKernel.SelectInterior(
            10, index => index, 2, 7, maxBaseSamples: 257);

        Assert.Equal(4, selection.InteriorCount);
        Assert.Equal(4, selection.SelectedCount);
        Assert.Equal([3, 4, 5, 6],
            Enumerable.Range(0, selection.SelectedCount)
                .Select(selection.IndexAt).ToArray());
    }

    [Fact]
    public void DegenerateWindow_SelectsNoHistoricalNodes()
    {
        Assert.Equal(default, SoiSweepGridKernel.SelectInterior(
            2_000_000, index => index, 100, 100, maxBaseSamples: 257));
    }

    [Fact]
    public void MidpointWorkBudget_StopsExactlyAtCapacity()
    {
        var budget = new SoiSweepWorkBudget(2);

        Assert.True(budget.TryConsume());
        Assert.True(budget.TryConsume());
        Assert.False(budget.TryConsume());
        Assert.Equal(2, budget.Consumed);
        Assert.Equal(0, budget.Remaining);
    }

    [Fact]
    public void NonPositiveMidpointBudget_IsImmediatelyExhausted()
    {
        var budget = new SoiSweepWorkBudget(-10);
        Assert.False(budget.TryConsume());
        Assert.Equal(0, budget.Consumed);
        Assert.Equal(0, budget.Remaining);
    }

    [Fact]
    public void MatchingSampleAndEndpoint_StampsThatFinalParentAndEvidence()
    {
        var decision = SoiSweepReconciliationKernel.Reconcile(
            "Moon", "Moon", cursorTime: 120,
            crossingCount: 2, kernelTruncated: false, maxEvidenceCrossings: 64);

        Assert.Equal("Moon", decision.FinalParentId);
        Assert.Equal("Moon", decision.CursorParentId);
        Assert.Equal(120, decision.CursorTime);
        Assert.Equal(2, decision.EvidenceCrossingCount);
        Assert.True(decision.ShouldLogEvidence);
        Assert.False(decision.EvidenceTruncated);
        Assert.False(decision.EndpointOverrodeSample);
    }

    [Fact]
    public void ExactEndpoint_OverridesSampledFinalParentAndOwnsCursor()
    {
        var decision = SoiSweepReconciliationKernel.Reconcile(
            sampledFinalParentId: "Moon", endpointParentId: "Earth",
            cursorTime: 130, crossingCount: 2,
            kernelTruncated: false, maxEvidenceCrossings: 64);

        Assert.Equal("Earth", decision.FinalParentId);
        Assert.Equal("Earth", decision.CursorParentId);
        Assert.Equal(130, decision.CursorTime);
        Assert.True(decision.EndpointOverrodeSample);
        Assert.True(decision.ShouldLogEvidence);
    }

    [Fact]
    public void EndpointOnlyResult_StampsCursorWithoutInventingEvidence()
    {
        var decision = SoiSweepReconciliationKernel.Reconcile(
            "Earth", "Earth", cursorTime: 140,
            crossingCount: 0, kernelTruncated: false, maxEvidenceCrossings: 64);

        Assert.Equal("Earth", decision.FinalParentId);
        Assert.Equal("Earth", decision.CursorParentId);
        Assert.Equal(0, decision.EvidenceCrossingCount);
        Assert.False(decision.ShouldLogEvidence);
        Assert.False(decision.EvidenceTruncated);
    }

    [Fact]
    public void EvidenceCount_IsCappedWithoutChangingFinalOrCursorParent()
    {
        var decision = SoiSweepReconciliationKernel.Reconcile(
            "Earth", "Earth", cursorTime: 150,
            crossingCount: 100, kernelTruncated: false, maxEvidenceCrossings: 64);

        Assert.Equal("Earth", decision.FinalParentId);
        Assert.Equal("Earth", decision.CursorParentId);
        Assert.Equal(64, decision.EvidenceCrossingCount);
        Assert.True(decision.ShouldLogEvidence);
        Assert.True(decision.EvidenceTruncated);
        Assert.False(decision.KernelTruncated);
    }

    [Fact]
    public void KernelTruncation_ProducesEvidenceEvenWhenRecordedListIsEmpty()
    {
        var decision = SoiSweepReconciliationKernel.Reconcile(
            "Earth", "Earth", cursorTime: 160,
            crossingCount: 0, kernelTruncated: true, maxEvidenceCrossings: 64);

        Assert.Equal(0, decision.EvidenceCrossingCount);
        Assert.True(decision.ShouldLogEvidence);
        Assert.True(decision.KernelTruncated);
        Assert.True(decision.EvidenceTruncated);
    }
}

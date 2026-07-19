using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Mod.Tests.Soi;

/// <summary>Tests rails-based SOI reparenting: check parent exit first, then enter the
/// first child whose SOI contains the vessel.</summary>
public class SoiReparentKernelTests
{
    // A Luna-like child 384,400 km out with a 66,000 km SOI under an Earth-like
    // parent with a 924,000 km SOI.
    private static readonly Vector3d ParentPos = new(1e9, 2e9, 3e8);
    private const double ParentSoi = 9.24e8;
    private static readonly Vector3d ChildPos = ParentPos + new Vector3d(3.844e8, 0, 0);
    private const double ChildSoi = 6.6e7;
    private const string SweepParent = nameof(SweepParent);
    private const string SweepGrandparent = nameof(SweepGrandparent);
    private const string SweepChildA = nameof(SweepChildA);
    private const string SweepChildB = nameof(SweepChildB);

    private static SoiReparentKernel.Candidate Child(string id = "Luna") => new(id, ChildPos, ChildSoi);

    [Fact]
    public void AuthorityFactors_UseTheExactStockBoundary()
    {
        var justInside = ChildPos + new Vector3d(ChildSoi * 0.999999, 0, 0);
        Assert.Equal("Luna", SoiReparentKernel.Decide(
            justInside, ParentPos, ParentSoi, "Sol", [Child()], 1.0, 1.0));

        var justOutside = ParentPos + new Vector3d(0, ParentSoi * 1.000001, 0);
        Assert.Equal("Sol", SoiReparentKernel.Decide(
            justOutside, ParentPos, ParentSoi, "Sol", [Child()], 1.0, 1.0));
    }

    [Fact]
    public void DeepInsideChildSoi_ReturnsChild()
    {
        var vessel = ChildPos + new Vector3d(2e6, 0, 0); // 2,000 km from the child
        Assert.Equal("Luna", SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    [Fact]
    public void ExactlyAtEnterMargin_ReturnsChild()
    {
        var vessel = ChildPos + new Vector3d(ChildSoi * SoiReparentKernel.EnterFactor, 0, 0);
        Assert.Equal("Luna", SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    [Fact]
    public void InsideChildBoundary_Enters()
    {
        var vessel = ChildPos + new Vector3d(ChildSoi * 0.95, 0, 0);
        Assert.Equal("Luna", SoiReparentKernel.Decide(
            vessel, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    [Fact]
    public void OutsideChildBoundary_KeepsCurrentParent()
    {
        var vessel = ChildPos + new Vector3d(ChildSoi * 1.01, 0, 0);
        Assert.Null(SoiReparentKernel.Decide(
            vessel, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    [Fact]
    public void DecisivelyOutsideParentSoi_ReturnsGrandparent()
    {
        var vessel = ParentPos + new Vector3d(0, ParentSoi * 1.2, 0);
        Assert.Equal("Sol", SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    [Fact]
    public void OutsideParentSoi_Exits()
    {
        var vessel = ParentPos + new Vector3d(0, ParentSoi * 1.05, 0);
        Assert.Equal("Sol", SoiReparentKernel.Decide(
            vessel, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    [Fact]
    public void NoGrandparent_NeverExits()
    {
        var vessel = ParentPos + new Vector3d(0, ParentSoi * 100, 0);
        Assert.Null(SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, null, [Child()]));
    }

    /// <summary>Infinite root SOI and other non-finite radii must not produce an exit.</summary>
    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void NonFiniteParentSoi_NeverExits(double soi)
    {
        var vessel = ParentPos + new Vector3d(0, 1e15, 0);
        Assert.Null(SoiReparentKernel.Decide(vessel, ParentPos, soi, "Ghost", []));
    }

    /// <summary>Overlapping child SOIs select the first candidate.</summary>
    [Fact]
    public void FirstMatchingChildWins()
    {
        var other = new SoiReparentKernel.Candidate("First", ChildPos, ChildSoi * 2);
        var vessel = ChildPos + new Vector3d(1e6, 0, 0);
        Assert.Equal("First", SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", [other, Child()]));
    }

    /// <summary>Parent exit takes precedence over child entry.</summary>
    [Fact]
    public void ExitWinsOverEntry()
    {
        var farChildPos = ParentPos + new Vector3d(0, ParentSoi * 1.2, 0);
        var farChild = new SoiReparentKernel.Candidate("Far", farChildPos, ChildSoi);
        Assert.Equal("Sol", SoiReparentKernel.Decide(farChildPos, ParentPos, ParentSoi, "Sol", [farChild]));
    }

    [Fact]
    public void NoChildrenInsideParent_Keeps()
    {
        var vessel = ParentPos + new Vector3d(1e8, 0, 0);
        Assert.Null(SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", []));
    }

    [Fact]
    public void CustomFactors_MoveTheDecisionBoundaries()
    {
        // At enterFactor 0.5, 60% of the child SOI is a keep; at the default it enters.
        var vessel = ChildPos + new Vector3d(ChildSoi * 0.6, 0, 0);
        Assert.Null(SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", [Child()],
            enterFactor: 0.5, exitFactor: 2.0));
        Assert.Equal("Luna", SoiReparentKernel.Decide(vessel, ParentPos, ParentSoi, "Sol", [Child()]));
        // At exitFactor 2.0, 1.5x the parent SOI is a keep; at the default it exits.
        var far = ParentPos + new Vector3d(0, ParentSoi * 1.5, 0);
        Assert.Null(SoiReparentKernel.Decide(far, ParentPos, ParentSoi, "Sol", [Child()],
            enterFactor: 0.5, exitFactor: 2.0));
        Assert.Equal("Sol", SoiReparentKernel.Decide(far, ParentPos, ParentSoi, "Sol", [Child()]));
    }

    private static SoiReparentKernel.SweptCandidate SweptChild(
        Vector3d start, Vector3d end, double radius = 5, string id = "Luna") =>
        new(id, start, end, radius);

    private static SoiReparentKernel.PolylineCandidate PolylineChild(
        string id, double radius, params Vector3d[] samples) =>
        new(id, samples, radius);

    [Fact]
    public void CompleteShortTransit_WithBothEndpointsOutside_IsDetected()
    {
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, "Sol",
            [SweptChild(new Vector3d(-20, 0, 0), new Vector3d(20, 0, 0))]);

        Assert.Equal(new SoiReparentKernel.Crossing("Luna", 0.375, Escape: false), crossing);
    }

    [Fact]
    public void HighWarpAstronomicalSegment_RemainsNumericallyStable()
    {
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 2e12, null,
            [SweptChild(new Vector3d(-1e12, 0, 0), new Vector3d(1e12, 0, 0), 1e6)]);

        Assert.NotNull(crossing);
        Assert.Equal(0.4999995, crossing.Value.Fraction, precision: 12);
    }

    [Fact]
    public void MovingSoiCentre_IsRepresentedByRelativeEndpoints()
    {
        var vesselStart = new Vector3d(0, 0, 0);
        var vesselEnd = new Vector3d(100, 0, 0);
        var bodyStart = new Vector3d(40, 0, 0);
        var bodyEnd = new Vector3d(60, 0, 0);
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 1_000, null,
            [SweptChild(vesselStart - bodyStart, vesselEnd - bodyEnd, radius: 10)]);

        Assert.NotNull(crossing);
        Assert.Equal(0.375, crossing.Value.Fraction, precision: 12);
    }

    [Fact]
    public void ChronologicallyEarlierChildEntry_BeatsLaterParentEscape()
    {
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, new Vector3d(20, 0, 0), 10, "Sol",
            [SweptChild(new Vector3d(-8, 0, 0), new Vector3d(12, 0, 0), radius: 5)]);

        Assert.NotNull(crossing);
        Assert.Equal("Luna", crossing.Value.NewParentId);
        Assert.False(crossing.Value.Escape);
        Assert.Equal(0.15, crossing.Value.Fraction, precision: 12);
    }

    [Fact]
    public void ExactTimeTie_PrefersParentEscape()
    {
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, new Vector3d(20, 0, 0), 10, "Sol",
            [SweptChild(new Vector3d(-15, 0, 0), new Vector3d(5, 0, 0), radius: 5)]);

        Assert.Equal(new SoiReparentKernel.Crossing("Sol", 0.5, Escape: true), crossing);
    }

    [Fact]
    public void EqualTimeChildEntries_PreserveEnumerationOrder()
    {
        var children = new[]
        {
            SweptChild(new Vector3d(-15, 0, 0), new Vector3d(5, 0, 0), id: "First"),
            SweptChild(new Vector3d(0, -15, 0), new Vector3d(0, 5, 0), id: "Second"),
        };
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null, children);

        Assert.Equal("First", crossing!.Value.NewParentId);
    }

    [Fact]
    public void TangentChildSweep_IsNotACrossing()
    {
        var crossing = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null,
            [SweptChild(new Vector3d(-10, 5, 0), new Vector3d(10, 5, 0))]);

        Assert.Null(crossing);
    }

    [Fact]
    public void ZeroLengthSweep_IsSafeAndHasNoCrossing()
    {
        var point = new Vector3d(20, 0, 0);
        Assert.Null(SoiReparentKernel.FirstCrossing(
            point, point, 10, "Sol", [SweptChild(point, point)]));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void InvalidChildRadius_IsIgnoredBySweep(double radius)
    {
        Assert.Null(SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null,
            [SweptChild(new Vector3d(-20, 0, 0), new Vector3d(20, 0, 0), radius)]));
    }

    [Fact]
    public void NonFiniteEndpoint_IsIgnoredBySweep()
    {
        Assert.Null(SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null,
            [SweptChild(new Vector3d(double.NaN, 0, 0), new Vector3d(20, 0, 0))]));
    }

    [Fact]
    public void ChildExit_IsNotAnEntryEvent()
    {
        Assert.Null(SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null,
            [SweptChild(Vector3d.Zero, new Vector3d(20, 0, 0))]));
    }

    [Fact]
    public void ParentEntry_IsNotAnEscapeEvent()
    {
        Assert.Null(SoiReparentKernel.FirstCrossing(
            new Vector3d(20, 0, 0), Vector3d.Zero, 10, "Sol", []));
    }

    [Fact]
    public void CatchUpFromEntryBoundary_FindsLaterExitInsteadOfRepeatingEntry()
    {
        var exit = SoiReparentKernel.FirstCrossing(
            new Vector3d(-5, 0, 0), new Vector3d(20, 0, 0), 5, "Earth", []);

        Assert.NotNull(exit);
        Assert.True(exit.Value.Escape);
        Assert.Equal(0.4, exit.Value.Fraction, precision: 12);
    }

    [Fact]
    public void StartingAtChildBoundary_MovingInward_EntersAtZero()
    {
        var entry = SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null,
            [SweptChild(new Vector3d(-5, 0, 0), Vector3d.Zero)]);

        Assert.Equal(0.0, entry!.Value.Fraction);
        Assert.False(entry.Value.Escape);
    }

    [Fact]
    public void StartingAtParentBoundary_MovingOutward_ExitsAtZero()
    {
        var exit = SoiReparentKernel.FirstCrossing(
            new Vector3d(5, 0, 0), new Vector3d(10, 0, 0), 5, "Earth", []);

        Assert.Equal(0.0, exit!.Value.Fraction);
        Assert.True(exit.Value.Escape);
    }

    [Fact]
    public void CurvedPolyline_DetectsTransitWhenOverallChordIsClear()
    {
        var start = new Vector3d(-10, 10, 0);
        var middle = Vector3d.Zero;
        var end = new Vector3d(10, 10, 0);
        Assert.Null(SoiReparentKernel.FirstCrossing(
            Vector3d.Zero, Vector3d.Zero, 100, null,
            [new SoiReparentKernel.SweptCandidate(SweepChildA, start, end, 5)]));

        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 0.5, 1.0],
            [Vector3d.Zero, Vector3d.Zero, Vector3d.Zero],
            100, SweepGrandparent,
            [PolylineChild(SweepChildA, 5, start, middle, end)]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Equal(2, result.Crossings.Count);
        Assert.Equal(SweepChildA, result.Crossings[0].NewParentId);
        Assert.False(result.Crossings[0].Escape);
        Assert.InRange(result.Crossings[0].Fraction, 0.32, 0.33);
        Assert.Equal(SweepParent, result.Crossings[1].NewParentId);
        Assert.True(result.Crossings[1].Escape);
        Assert.InRange(result.Crossings[1].Fraction, 0.67, 0.68);
    }

    [Fact]
    public void CompleteHighWarpTransit_EndsOnOriginalParentInOneSweep()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero],
            2e12, null,
            [PolylineChild(SweepChildA, 1e6,
                new Vector3d(-1e12, 0, 0), new Vector3d(1e12, 0, 0))]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Equal(2, result.Crossings.Count);
        Assert.Equal(SweepChildA, result.Crossings[0].NewParentId);
        Assert.False(result.Crossings[0].Escape);
        Assert.Equal(0.4999995, result.Crossings[0].Fraction, precision: 12);
        Assert.Equal(SweepParent, result.Crossings[1].NewParentId);
        Assert.True(result.Crossings[1].Escape);
        Assert.Equal(0.5000005, result.Crossings[1].Fraction, precision: 12);
    }

    [Fact]
    public void MovingCentreRelativePolyline_OrdersEntryAndExit()
    {
        var vesselStart = new Vector3d(0, 0, 0);
        var vesselEnd = new Vector3d(100, 0, 0);
        var bodyStart = new Vector3d(40, 0, 0);
        var bodyEnd = new Vector3d(60, 0, 0);
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero], 1_000, null,
            [PolylineChild(SweepChildA, 10,
                vesselStart - bodyStart, vesselEnd - bodyEnd)]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Equal(2, result.Crossings.Count);
        Assert.Equal(0.375, result.Crossings[0].Fraction, precision: 12);
        Assert.Equal(0.625, result.Crossings[1].Fraction, precision: 12);
    }

    [Fact]
    public void MultipleSiblingTransits_AreChronologicalAndReturnToParent()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero], 100, null,
            [
                PolylineChild(SweepChildA, 2,
                    new Vector3d(-20, 0, 0), new Vector3d(20, 0, 0)),
                PolylineChild(SweepChildB, 2,
                    new Vector3d(-30, 0, 0), new Vector3d(10, 0, 0)),
            ]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Equal(4, result.Crossings.Count);
        Assert.Equal(SweepChildA, result.Crossings[0].NewParentId);
        Assert.Equal(SweepParent, result.Crossings[1].NewParentId);
        Assert.Equal(SweepChildB, result.Crossings[2].NewParentId);
        Assert.Equal(SweepParent, result.Crossings[3].NewParentId);
        Assert.Equal(0.45, result.Crossings[0].Fraction, precision: 12);
        Assert.Equal(0.55, result.Crossings[1].Fraction, precision: 12);
        Assert.Equal(0.70, result.Crossings[2].Fraction, precision: 12);
        Assert.Equal(0.80, result.Crossings[3].Fraction, precision: 12);
    }

    [Fact]
    public void PolylineTie_PreservesChildEnumerationOrder()
    {
        var path = new[]
        {
            new Vector3d(-10, 0, 0),
            new Vector3d(10, 0, 0),
        };
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero], 100, null,
            [
                PolylineChild(SweepChildA, 2, path),
                PolylineChild(SweepChildB, 2, path),
            ]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Equal(2, result.Crossings.Count);
        Assert.Equal(SweepChildA, result.Crossings[0].NewParentId);
        Assert.Equal(SweepParent, result.Crossings[1].NewParentId);
    }

    [Fact]
    public void PolylineTie_PrefersParentEscapeAndTerminates()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, new Vector3d(20, 0, 0)], 10,
            SweepGrandparent,
            [PolylineChild(SweepChildA, 5,
                new Vector3d(-15, 0, 0), new Vector3d(5, 0, 0))]);

        Assert.Equal(SweepGrandparent, result.FinalParentId);
        Assert.Single(result.Crossings);
        Assert.Equal(SweepGrandparent, result.Crossings[0].NewParentId);
        Assert.True(result.Crossings[0].Escape);
        Assert.Equal(0.5, result.Crossings[0].Fraction, precision: 12);
    }

    [Fact]
    public void OverlappingSiblingTransit_HandsOffAtActiveChildExit()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero], 100, null,
            [
                // A owns .2-.4. B's geometric entry is .3 while A owns the vessel,
                // so B becomes authoritative immediately when A exits at .4.
                PolylineChild(SweepChildA, 1,
                    new Vector3d(-3, 0, 0), new Vector3d(7, 0, 0)),
                PolylineChild(SweepChildB, 1,
                    new Vector3d(-2.2, 0, 0), new Vector3d(1.8, 0, 0)),
            ]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Collection(result.Crossings,
            enterA =>
            {
                Assert.Equal(SweepChildA, enterA.NewParentId);
                Assert.False(enterA.Escape);
                Assert.Equal(0.2, enterA.Fraction, precision: 12);
            },
            exitA =>
            {
                Assert.Equal(SweepParent, exitA.NewParentId);
                Assert.True(exitA.Escape);
                Assert.Equal(0.4, exitA.Fraction, precision: 12);
            },
            enterB =>
            {
                Assert.Equal(SweepChildB, enterB.NewParentId);
                Assert.False(enterB.Escape);
                Assert.Equal(0.4, enterB.Fraction, precision: 12);
            },
            exitB =>
            {
                Assert.Equal(SweepParent, exitB.NewParentId);
                Assert.True(exitB.Escape);
                Assert.Equal(0.8, exitB.Fraction, precision: 12);
            });
    }

    [Fact]
    public void CurvedParentExitThenReentry_ReturnsToOriginalParent()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 0.5, 1.0],
            [
                Vector3d.Zero,
                new Vector3d(20, 0, 0),
                Vector3d.Zero,
            ],
            10, SweepGrandparent, []);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Collection(result.Crossings,
            escape =>
            {
                Assert.Equal(SweepGrandparent, escape.NewParentId);
                Assert.True(escape.Escape);
                Assert.Equal(0.25, escape.Fraction, precision: 12);
            },
            reentry =>
            {
                Assert.Equal(SweepParent, reentry.NewParentId);
                Assert.False(reentry.Escape);
                Assert.Equal(0.75, reentry.Fraction, precision: 12);
            });
    }

    [Fact]
    public void CrossingHistoryCap_DoesNotStopFinalStateResolution()
    {
        int completeSegments = SoiReparentKernel.MaxRecordedCrossings / 2;
        int segments = completeSegments + 1;
        var fractions = new double[segments + 1];
        var parent = new Vector3d[segments + 1];
        var child = new Vector3d[segments + 1];
        for (int i = 0; i <= completeSegments; i++)
        {
            fractions[i] = i / (double)segments;
            child[i] = new Vector3d(i % 2 == 0 ? -2 : 2, 0, 0);
        }
        fractions[^1] = 1.0;
        child[^1] = Vector3d.Zero; // unrecorded final entry must still change state

        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, fractions, parent, 100, null,
            [PolylineChild(SweepChildA, 1, child)]);

        Assert.Equal(SweepChildA, result.FinalParentId);
        Assert.Equal(SoiReparentKernel.MaxRecordedCrossings,
            result.Crossings.Count);
        Assert.True(result.CrossingsTruncated);
    }

    [Fact]
    public void PolylineTangencyAcrossSampleBoundary_IsNotATransition()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 0.5, 1.0],
            [Vector3d.Zero, Vector3d.Zero, Vector3d.Zero],
            100, null,
            [PolylineChild(SweepChildA, 5,
                new Vector3d(-10, 5, 0),
                new Vector3d(0, 5, 0),
                new Vector3d(10, 5, 0))]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Empty(result.Crossings);
    }

    [Fact]
    public void PolylineBoundaryCusp_DoesNotEmitZeroDurationEntryAndExit()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 0.5, 1.0],
            [Vector3d.Zero, Vector3d.Zero, Vector3d.Zero],
            100, null,
            [PolylineChild(SweepChildA, 5,
                new Vector3d(-10, 0, 0),
                new Vector3d(-5, 0, 0),
                new Vector3d(-10, 0, 0))]);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Empty(result.Crossings);
    }

    [Fact]
    public void PolylineBoundaryThenInward_StillEntersAtSharedSample()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 0.5, 1.0],
            [Vector3d.Zero, Vector3d.Zero, Vector3d.Zero],
            100, null,
            [PolylineChild(SweepChildA, 5,
                new Vector3d(-10, 0, 0),
                new Vector3d(-5, 0, 0),
                Vector3d.Zero)]);

        Assert.Equal(SweepChildA, result.FinalParentId);
        var entry = Assert.Single(result.Crossings);
        Assert.Equal(SweepChildA, entry.NewParentId);
        Assert.False(entry.Escape);
        Assert.Equal(0.5, entry.Fraction, precision: 12);
    }

    [Fact]
    public void ParentReachingFinalBoundaryOutward_DoesNotEscape()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, new Vector3d(10, 0, 0)],
            10, SweepGrandparent, []);

        Assert.Equal(SweepParent, result.FinalParentId);
        Assert.Empty(result.Crossings);
    }

    [Fact]
    public void ActiveChildReachingFinalBoundaryOutward_DoesNotExit()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 0.5, 1.0],
            [Vector3d.Zero, Vector3d.Zero, Vector3d.Zero],
            100, null,
            [PolylineChild(SweepChildA, 5,
                new Vector3d(-10, 0, 0),
                Vector3d.Zero,
                new Vector3d(5, 0, 0))]);

        Assert.Equal(SweepChildA, result.FinalParentId);
        var entry = Assert.Single(result.Crossings);
        Assert.Equal(SweepChildA, entry.NewParentId);
        Assert.False(entry.Escape);
        Assert.Equal(0.25, entry.Fraction, precision: 12);
    }

    [Fact]
    public void ChildReachingFinalBoundaryInward_StillEnters()
    {
        var result = SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero],
            100, null,
            [PolylineChild(SweepChildA, 5,
                new Vector3d(-10, 0, 0),
                new Vector3d(-5, 0, 0))]);

        Assert.Equal(SweepChildA, result.FinalParentId);
        var entry = Assert.Single(result.Crossings);
        Assert.Equal(SweepChildA, entry.NewParentId);
        Assert.False(entry.Escape);
        Assert.Equal(1.0, entry.Fraction, precision: 12);
    }

    [Fact]
    public void PolylineGridMismatch_IsRejected()
    {
        Assert.Throws<ArgumentException>(() => SoiReparentKernel.SweepPolyline(
            SweepParent, [0.0, 1.0],
            [Vector3d.Zero, Vector3d.Zero],
            100, null,
            [PolylineChild(SweepChildA, 5, Vector3d.Zero)]));
    }

}

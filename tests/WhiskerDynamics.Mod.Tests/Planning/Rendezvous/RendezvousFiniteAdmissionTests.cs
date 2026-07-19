using WhiskerDynamics.Mod;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Planning.Rendezvous;

public class RendezvousFiniteAdmissionTests
{
    private static readonly EngineScalars Engine = new(1000.0, 3000.0, 2.0);
    private static GravityModel ZeroGravity() => new(new Ephemerides([]));

    [Theory]
    [InlineData(20.0, 32)]
    [InlineData(2.0, 1)]
    public void One_slice_pair_uses_physical_ignition_cutoff_and_terminal_bounds(
        double sliceSeconds, int maxSlices)
    {
        const double departure = 100.0;
        const double arrival = 140.0;
        const double magnitude = 60.0;
        var finite = new FiniteBurnFold(Engine, sliceSeconds, maxSlices);
        Assert.True(FiniteBurnKernel.TryGetPhysicalWindow(
            departure, magnitude, Engine, out var firstWindow));
        var secondEngine = Engine with
        {
            MassKg = FiniteBurnKernel.MassAfterBurn(magnitude, Engine),
        };
        Assert.True(FiniteBurnKernel.TryGetPhysicalWindow(
            arrival, magnitude, secondEngine, out var secondWindow));

        Assert.True(RendezvousFiniteAdmission.TryAdmit(
            departure, magnitude, arrival, magnitude, finite,
            firstWindow.IgnitionSeconds - 1.0, secondWindow.CutoffSeconds,
            out var commands));

        Assert.Null(commands.Departure.Expansion);
        Assert.Null(commands.Arrival.Expansion);
        Assert.Equal(firstWindow.IgnitionSeconds, commands.PredictionStartSeconds);
        Assert.Equal(secondWindow.CutoffSeconds, commands.TerminalSeconds);
        Assert.True(commands.PredictionStartSeconds < departure);
        Assert.True(commands.TerminalSeconds > arrival);
    }

    [Fact]
    public void One_slice_pair_rejects_when_physical_ignition_loses_its_lead()
    {
        const double departure = 100.0;
        const double magnitude = 60.0;
        var finite = new FiniteBurnFold(Engine, 20.0, 32);
        Assert.True(FiniteBurnKernel.TryGetPhysicalWindow(
            departure, magnitude, Engine, out var window));
        double now = window.IgnitionSeconds - 0.5;
        Assert.True(departure > now + PlannerKernel.MinLeadSeconds);
        Assert.False(OptimizeApplyPolicy.ModeledStartHasLead(
            window.IgnitionSeconds, now, PlannerKernel.MinLeadSeconds));

        Assert.False(RendezvousFiniteAdmission.TryAdmit(
            departure, magnitude, 140.0, magnitude, finite,
            exclusiveEarliestIgnition: now + PlannerKernel.MinLeadSeconds,
            inclusiveHorizon: 200.0, out _));

        Assert.Equal(RendezvousApplyLeadVerdict.Allowed,
            RendezvousApplyPolicy.CheckDepartureLead(
                departure, magnitude, finite: null, now,
                PlannerKernel.MinLeadSeconds));
        Assert.Equal(RendezvousApplyLeadVerdict.InsufficientLead,
            RendezvousApplyPolicy.CheckDepartureLead(
                departure, magnitude, finite, now,
                PlannerKernel.MinLeadSeconds));
    }

    [Fact]
    public void One_slice_evaluator_traverses_pre_node_window_and_matches_target_at_cutoff()
    {
        const double departure = 100.0;
        const double arrival = 140.0;
        const double magnitude = 60.0;
        var finite = new FiniteBurnFold(Engine, SliceSeconds: 20.0, MaxSlices: 32);
        var departureDv = new Vector3d(0.0, magnitude, 0.0);
        var arrivalDv = -departureDv;
        Assert.True(RendezvousFiniteAdmission.TryAdmit(
            departure, magnitude, arrival, magnitude, finite,
            exclusiveEarliestIgnition: 0.0,
            inclusiveHorizon: 200.0, out var commands));
        Assert.Null(commands.Departure.Expansion);
        Assert.Null(commands.Arrival.Expansion);

        double start = commands.PredictionStartSeconds;
        var seed = new StateVector(
            new Vector3d(-5.0, 0.0, 0.0),
            new Vector3d(1.0, 0.0, 0.0));
        double targetQuery = double.NaN;
        StateVector TargetAt(double time)
        {
            targetQuery = time;
            var position = seed.Position;
            var velocity = seed.Velocity;
            double cursor = start;
            position += velocity * (departure - cursor);
            cursor = departure;
            velocity += departureDv;
            position += velocity * (arrival - cursor);
            cursor = arrival;
            velocity += arrivalDv;
            position += velocity * (time - cursor);
            return new StateVector(position, velocity);
        }

        var evaluation = RendezvousFiniteEvaluator.Evaluate(
            commands, ZeroGravity(), seed,
            departure, departureDv, arrival, arrivalDv,
            static (path, time) => path.StateAt(time), TargetAt);

        Assert.Equal(commands.PredictionStartSeconds, evaluation.Path.StartTime);
        Assert.Equal(commands.TerminalSeconds, evaluation.Path.Horizon);
        Assert.Equal(commands.TerminalSeconds, targetQuery);
        Assert.InRange((evaluation.Target.Position - evaluation.Flown.Position).Length(),
            0.0, 1e-8);
        Assert.InRange((evaluation.Target.Velocity - evaluation.Flown.Velocity).Length(),
            0.0, 1e-10);

        // This sphere is crossed before the numerical K=1 impulse at the node. A path
        // incorrectly starting at the node could never observe this physical traversal.
        double impactTime = 0.5 * (commands.PredictionStartSeconds + departure);
        Assert.True(impactTime < departure);
        Vector3d center = seed.Position + seed.Velocity * (impactTime - start);
        bool clears = true;
        for (int i = 1; i < evaluation.Path.Nodes.Count; i++)
        {
            clears &= RendezvousFiniteEvaluator.SegmentClearsSphere(
                evaluation.Path, radius: 0.1,
                evaluation.Path.Nodes[i - 1], evaluation.Path.Nodes[i], depth: 6,
                _ => center, static (path, time) => path.StateAt(time));
        }
        Assert.False(clears);
    }

    [Fact]
    public void Curved_path_intersection_is_rejected_when_endpoint_chord_clears()
    {
        var primary = new CelestialBody { Id = "Primary", Mu = 1000.0 };
        var gravity = new GravityModel(new Ephemerides([primary]));
        var initial = new StateVector(
            new Vector3d(10.0, 0.0, 0.0), new Vector3d(0.0, 10.0, 0.0));
        var predictor = new TrajectoryPredictor(gravity, initial, 0.0,
            new IntegratorOptions { RelTol = 1e-12 });
        double endTime = Math.PI / 2.0;
        _ = predictor.StateAt(endTime);
        int segmentEnd = 1;
        for (int i = 2; i < predictor.Nodes.Count; i++)
            if (predictor.Nodes[i].Time - predictor.Nodes[i - 1].Time
                > predictor.Nodes[segmentEnd].Time - predictor.Nodes[segmentEnd - 1].Time)
                segmentEnd = i;
        var a = predictor.Nodes[segmentEnd - 1];
        var b = predictor.Nodes[segmentEnd];
        double midTime = 0.5 * (a.Time + b.Time);
        Vector3d center = predictor.StateAt(midTime).Position;
        var ra = a.State.Position - center;
        var rb = b.State.Position - center;
        var chord = rb - ra;
        double u = Math.Clamp(-ra.Dot(chord) / chord.LengthSquared(), 0.0, 1.0);
        double chordClearance = (ra + chord * u).Length();
        Assert.True(chordClearance > 0.0);
        double collisionRadius = chordClearance / 2.0;

        int midpointCalls = 0;
        StateVector PropagatedState(TrajectoryPredictor path, double time)
        {
            midpointCalls++;
            return path.StateAt(time);
        }

        Assert.False(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, collisionRadius, a, b, depth: 6,
            _ => center, PropagatedState));
        Assert.Equal(3, midpointCalls);
    }

    [Fact]
    public void Clear_midpoint_does_not_hide_an_interior_cubic_collision()
    {
        var predictor = new TrajectoryPredictor(ZeroGravity(), default, 0.0);
        const double collisionTime = 0.2;
        const double cubicScale = -125.0 / 3.0;
        static Vector3d PositionAt(double time)
        {
            double x = 5.0 * (time - collisionTime);
            double y = 2.0 + cubicScale * time * (time - 0.5) * (time - 1.0);
            return new Vector3d(x, y, 0.0);
        }
        var a = new TrajectoryNode(0.0, new StateVector(PositionAt(0.0), default));
        var b = new TrajectoryNode(1.0, new StateVector(PositionAt(1.0), default));

        Assert.Equal(2.0, PositionAt(0.5).Y, 12);
        Assert.Equal(0.0, PositionAt(collisionTime).Length(), 12);
        Assert.True(PositionAt(0.25).Length() > 0.1);
        Assert.False(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, radius: 0.1, a, b, depth: 6,
            static _ => Vector3d.Zero,
            static (_, time) => new StateVector(PositionAt(time), default)));
    }

    [Fact]
    public void Curved_clear_path_is_proved_by_subdivided_polynomial_bounds()
    {
        var predictor = new TrajectoryPredictor(ZeroGravity(), default, 0.0);
        var a = new TrajectoryNode(0.0,
            new StateVector(new Vector3d(-2.0, 3.0, 0.0), default));
        var b = new TrajectoryNode(2.0,
            new StateVector(new Vector3d(2.0, 3.0, 0.0), default));
        int midpointCalls = 0;
        StateVector CurvedState(TrajectoryPredictor _, double time)
        {
            midpointCalls++;
            double x = 2.0 * (time - 1.0);
            double y = 2.0 + (time - 1.0) * (time - 1.0);
            return new StateVector(new Vector3d(x, y, 0.0), default);
        }

        Assert.True(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, radius: 1.0, a, b, depth: 6,
            static _ => Vector3d.Zero, CurvedState));
        Assert.Equal(13, midpointCalls);
    }

    [Fact]
    public void Collision_subdivision_honors_cancellation_and_depth_bound()
    {
        var predictor = new TrajectoryPredictor(ZeroGravity(), default, 0.0);
        var a = new TrajectoryNode(0.0,
            new StateVector(new Vector3d(-2.0, 2.0, 0.0), default));
        var b = new TrajectoryNode(2.0,
            new StateVector(new Vector3d(2.0, 2.0, 0.0), default));
        StateVector UnexpectedState(TrajectoryPredictor _, double __) =>
            throw new InvalidOperationException();

        Assert.False(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, radius: 1.0, a, b, depth: 6,
            static _ => Vector3d.Zero, UnexpectedState, static () => true));
        Assert.False(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, radius: 1.0, a, b, depth: 0,
            _ => throw new InvalidOperationException(), UnexpectedState));

        bool cancelled = false;
        int stateCalls = 0;
        int centerCalls = 0;
        Vector3d CenterAt(double _)
        {
            centerCalls++;
            return Vector3d.Zero;
        }
        StateVector CancelAfterSample(TrajectoryPredictor _, double __)
        {
            stateCalls++;
            cancelled = true;
            return new StateVector(new Vector3d(0.0, 2.0, 0.0), default);
        }
        Assert.False(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, radius: 1.0, a, b, depth: 6,
            CenterAt, CancelAfterSample, () => cancelled));
        Assert.Equal(1, stateCalls);
        Assert.Equal(2, centerCalls);
    }

    [Fact]
    public void Collision_subdivision_has_a_hard_callback_bound_and_reuses_midpoints()
    {
        var predictor = new TrajectoryPredictor(ZeroGravity(), default, 0.0);
        var a = new TrajectoryNode(0.0,
            new StateVector(new Vector3d(-1.0, 1.0, 0.0), default));
        var b = new TrajectoryNode(1.0,
            new StateVector(new Vector3d(2.0, 1.0, 0.0), default));
        int stateCalls = 0;

        Assert.False(RendezvousFiniteEvaluator.SegmentClearsSphere(
            predictor, radius: 1.0, a, b, depth: 6,
            static _ => Vector3d.Zero,
            (_, time) =>
            {
                stateCalls++;
                return new StateVector(new Vector3d(3.0 * time - 1.0, 1.0, 0.0), default);
            }));

        Assert.InRange(stateCalls, 1, 253);
    }

    [Fact]
    public void Node_separation_does_not_admit_overlapping_physical_windows()
    {
        const double departure = 100.0;
        const double arrival = 108.0;
        const double magnitude = 60.0;
        var finite = new FiniteBurnFold(Engine, 20.0, 32);
        Assert.True(departure < arrival);

        Assert.False(RendezvousFiniteAdmission.TryAdmit(
            departure, magnitude, arrival, magnitude, finite,
            exclusiveEarliestIgnition: 0.0,
            inclusiveHorizon: 200.0, out _));
    }

    [Fact]
    public void One_slice_pair_rejects_physical_cutoff_past_horizon()
    {
        const double departure = 100.0;
        const double arrival = 140.0;
        const double magnitude = 60.0;
        var finite = new FiniteBurnFold(Engine, 20.0, 32);
        var secondEngine = Engine with
        {
            MassKg = FiniteBurnKernel.MassAfterBurn(magnitude, Engine),
        };
        Assert.True(FiniteBurnKernel.TryGetPhysicalWindow(
            arrival, magnitude, secondEngine, out var secondWindow));

        Assert.False(RendezvousFiniteAdmission.TryAdmit(
            departure, magnitude, arrival, magnitude, finite,
            exclusiveEarliestIgnition: 0.0,
            inclusiveHorizon: secondWindow.CutoffSeconds - 0.001, out _));
    }

    [Fact]
    public void Multi_slice_pair_keeps_expansions_inside_the_same_physical_bounds()
    {
        const double departure = 100.0;
        const double arrival = 200.0;
        const double magnitude = 300.0;
        var finite = new FiniteBurnFold(Engine, 5.0, 32);

        Assert.True(RendezvousFiniteAdmission.TryAdmit(
            departure, magnitude, arrival, magnitude, finite,
            exclusiveEarliestIgnition: 0.0,
            inclusiveHorizon: 300.0, out var commands));

        var first = Assert.IsType<FiniteBurnExpansion>(commands.Departure.Expansion);
        var second = Assert.IsType<FiniteBurnExpansion>(commands.Arrival.Expansion);
        Assert.Equal(commands.PredictionStartSeconds, first.IgnitionSeconds);
        Assert.Equal(commands.Departure.Window.CutoffSeconds,
            first.IgnitionSeconds + first.DurationSeconds);
        Assert.Equal(commands.TerminalSeconds,
            second.IgnitionSeconds + second.DurationSeconds);
    }
}

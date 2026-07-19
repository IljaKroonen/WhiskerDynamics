using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.GameTestDriver.Runtime;

internal readonly record struct GameTestLunarTransferSolution(
    double DepartureTime, Vector3d DeltaVVlf, double MissDistanceMeters);

internal sealed class GameTestLunarTransferSolveJob
{
    private readonly record struct Candidate(
        double DepartureTime, double ArrivalTime, StateVector DepartureState,
        StateVector EarthDeparture, Vector3d Velocity, double DeltaV);

    public required RailsService.PredictionContext Prediction { get; init; }
    public required StateVector DepartureState { get; init; }
    public required double DepartureTime { get; init; }
    public required double DepartureSearchDuration { get; init; }
    public required double FlightDuration { get; init; }
    public required double EarthMu { get; init; }
    public required double TargetLunarRadiusMeters { get; init; }
    public required double EarthClearanceRadiusMeters { get; init; }
    public required double LunaClearanceRadiusMeters { get; init; }

    private volatile bool _done;
    public bool Done => _done;
    public GameTestLunarTransferSolution? Result { get; private set; }
    public string? Failure { get; private set; }

    public void Start() => new Thread(Run)
    {
        IsBackground = true,
        Name = "whiskerdynamics-game-test-lunar-transfer",
        Priority = ThreadPriority.BelowNormal,
    }.Start();

    private void Run()
    {
        try { Solve(); }
        catch (Exception e)
        {
            Failure = $"lunar transfer solve failed: {e.Message}";
            ModLog.Warn($"game test: lunar transfer solve contained: {e}");
        }
        finally { _done = true; }
    }

    private void Solve()
    {
        var solver = new SolverPrediction(Prediction, static () => false);
        if (!(DepartureSearchDuration > 0)
            || !double.IsFinite(DepartureSearchDuration))
            throw new InvalidOperationException(
                "parking orbit has no finite departure-search period");
        if (!(FlightDuration > 0) || !double.IsFinite(FlightDuration))
            throw new InvalidOperationException(
                "lunar transfer has no finite flight duration");

        var coast = new TrajectoryPredictor(solver.Gravity,
            DepartureState, DepartureTime,
            new IntegratorOptions { RelTol = 1e-10 });
        var candidates = new List<Candidate>();
        const int departureSamples = 25;
        for (int i = 0; i < departureSamples; i++)
        {
            var (departure, arrival) = CandidateTimes(DepartureTime,
                DepartureSearchDuration, FlightDuration, i, departureSamples);
            StateVector vesselDeparture = solver.StateAt(
                coast, departure, 3600);
            StateVector earthDeparture = solver.GetAbsolute("Earth", departure);
            StateVector earthArrival = solver.GetAbsolute("Earth", arrival);
            StateVector lunaArrival = solver.GetAbsolute("Luna", arrival);
            StateVector lunaRelative = lunaArrival - earthArrival;
            Vector3d radial = lunaRelative.Position
                / lunaRelative.Position.Length();
            Vector3d tangentVelocity = lunaRelative.Velocity
                - radial * lunaRelative.Velocity.Dot(radial);
            double tangentSpeed = tangentVelocity.Length();
            if (!(tangentSpeed > 0) || !double.IsFinite(tangentSpeed))
                throw new InvalidOperationException(
                    "Luna has no finite transverse velocity");
            Vector3d tangent = tangentVelocity / tangentSpeed;
            Vector3d r1 = vesselDeparture.Position
                - earthDeparture.Position;
            foreach (double scale in new[] { 0.75, 1.0, 1.25 })
            foreach (double sign in new[] { -1.0, 1.0 })
            {
                Vector3d targetPosition = lunaArrival.Position
                    + tangent * (sign * scale * TargetLunarRadiusMeters);
                Vector3d r2 = targetPosition - earthArrival.Position;
                foreach (bool longWay in new[] { false, true })
                foreach (var lambert in RendezvousKernel.SolveLambert(
                    r1, r2, FlightDuration, EarthMu, longWay,
                    revolutions: 0))
                {
                    Vector3d velocity = earthDeparture.Velocity
                        + lambert.DepartureVelocity;
                    double deltaV = (velocity
                        - vesselDeparture.Velocity).Length();
                    if (deltaV is >= 2_000 and <= 4_500)
                        candidates.Add(new Candidate(
                            departure, arrival, vesselDeparture,
                            earthDeparture, velocity, deltaV));
                }
            }
        }
        if (candidates.Count == 0)
            throw new InvalidOperationException(
                "parking-orbit search found no low-delta-v Lambert departure");

        Candidate? best = null;
        double bestClosestApproach = double.PositiveInfinity;
        double bestScore = double.PositiveInfinity;
        int propagationRejected = 0;
        int clearanceRejected = 0;
        int farRejected = 0;
        foreach (Candidate candidate in candidates
            .OrderBy(static candidate => candidate.DeltaV)
            .Take(120))
        {
            double? closest;
            try { closest = ClosestLunarApproach(solver, candidate); }
            catch (InvalidOperationException)
            {
                propagationRejected++;
                continue;
            }
            if (closest is null)
            {
                clearanceRejected++;
                continue;
            }
            if (closest > 2 * TargetLunarRadiusMeters)
            {
                farRejected++;
                continue;
            }
            double score = candidate.DeltaV
                + Math.Abs(closest.Value - TargetLunarRadiusMeters) / 10_000;
            if (score >= bestScore)
                continue;
            best = candidate;
            bestClosestApproach = closest.Value;
            bestScore = score;
        }
        if (best is not { } solved)
            throw new InvalidOperationException(
                $"no safe n-body lunar passage among {candidates.Count} "
                + $"low-delta-v departures; propagation {propagationRejected}, "
                + $"clearance {clearanceRejected}, far {farRejected}");

        Vector3d deltaVEcl = solved.Velocity
            - solved.DepartureState.Velocity;
        StateVector earthRelative = solved.DepartureState
            - solved.EarthDeparture;
        Vector3d? vlf = BurnFrameKernel.EclToVlf(deltaVEcl,
            earthRelative.Position, earthRelative.Velocity);
        if (vlf is null)
            throw new InvalidOperationException("departure VLF basis is degenerate");
        Result = new GameTestLunarTransferSolution(
            solved.DepartureTime, vlf.Value, bestClosestApproach);
    }

    internal static (double DepartureTime, double ArrivalTime) CandidateTimes(
        double departureStart, double departureSearchDuration,
        double flightDuration, int index, int sampleCount)
    {
        double departure = departureStart
            + departureSearchDuration * index / (sampleCount - 1);
        return (departure, departure + flightDuration);
    }

    private double? ClosestLunarApproach(
        SolverPrediction solver, Candidate candidate)
    {
        var predictor = new TrajectoryPredictor(solver.Gravity,
            new StateVector(candidate.DepartureState.Position,
                candidate.Velocity),
            candidate.DepartureTime,
            new IntegratorOptions { RelTol = 1e-10 });
        double closest = double.PositiveInfinity;
        double earthRadialAtClosest = double.NaN;
        const int samples = 256;
        for (int i = 0; i <= samples; i++)
        {
            double time = candidate.DepartureTime
                + (candidate.ArrivalTime - candidate.DepartureTime)
                * i / samples;
            StateVector vessel = solver.StateAt(predictor, time, 6 * 3600);
            StateVector earth = solver.GetAbsolute("Earth", time);
            double earthDistance = (vessel.Position
                - earth.Position).Length();
            double lunaDistance = (vessel.Position
                - solver.GetAbsolute("Luna", time).Position).Length();
            if (earthDistance <= EarthClearanceRadiusMeters
                || lunaDistance <= LunaClearanceRadiusMeters)
                return null;
            if (lunaDistance < closest)
            {
                Vector3d earthPosition = vessel.Position - earth.Position;
                Vector3d earthVelocity = vessel.Velocity - earth.Velocity;
                earthRadialAtClosest = earthPosition.Dot(earthVelocity)
                    / earthPosition.Length();
            }
            closest = Math.Min(closest, lunaDistance);
        }
        if (earthRadialAtClosest < 100)
            return null;
        return closest;
    }
}

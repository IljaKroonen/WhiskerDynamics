using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.GameTestDriver.Runtime;

internal readonly record struct GameTestPlayerLunarTransferSolution(
    double DepartureTime, Vector3d DeltaVVlf, double PeriluneRadiusMeters);

internal sealed class GameTestPlayerLunarTransferSolveJob
{
    public required RailsService.PredictionContext Prediction { get; init; }
    public required StateVector CoastSeedState { get; init; }
    public required double CoastSeedTime { get; init; }
    public required double DepartureStartTime { get; init; }
    public required double DepartureSearchDuration { get; init; }
    public required double FlightDuration { get; init; }
    public required double ProgradeDeltaV { get; init; }
    public required double DesiredPeriluneRadiusMeters { get; init; }
    public required double LunaRadiusMeters { get; init; }

    private volatile bool _done;
    public bool Done => _done;
    public GameTestPlayerLunarTransferSolution? Result { get; private set; }
    public string? Failure { get; private set; }

    public void Start() => new Thread(Run)
    {
        IsBackground = true,
        Name = "whiskerdynamics-game-test-player-lunar-transfer",
        Priority = ThreadPriority.BelowNormal,
    }.Start();

    private void Run()
    {
        try { Solve(); }
        catch (Exception e)
        {
            Failure = $"player-style lunar transfer solve failed: {e.Message}";
            ModLog.Warn($"game test: player-style lunar transfer solve contained: {e}");
        }
        finally { _done = true; }
    }

    private void Solve()
    {
        if (!(DepartureSearchDuration > 0)
            || !double.IsFinite(DepartureSearchDuration))
            throw new InvalidOperationException(
                "parking orbit has no finite departure-search period");
        if (!(FlightDuration > 0) || !double.IsFinite(FlightDuration))
            throw new InvalidOperationException(
                "lunar transfer has no finite prediction duration");
        if (!(ProgradeDeltaV > 0) || !double.IsFinite(ProgradeDeltaV))
            throw new InvalidOperationException(
                "lunar transfer has no finite prograde delta-v");

        var solver = new SolverPrediction(Prediction, static () => false);
        var evaluated = new Dictionary<double, double>();
        double bestTime = double.NaN;
        double bestPerilune = double.PositiveInfinity;
        double bestError = double.PositiveInfinity;

        void Consider(double departureTime)
        {
            departureTime = Math.Clamp(departureTime, DepartureStartTime,
                DepartureStartTime + DepartureSearchDuration);
            if (evaluated.ContainsKey(departureTime)) return;
            double perilune = EvaluatePerilune(solver, departureTime);
            evaluated[departureTime] = perilune;
            double error = Math.Abs(perilune - DesiredPeriluneRadiusMeters);
            if (error >= bestError) return;
            bestTime = departureTime;
            bestPerilune = perilune;
            bestError = error;
        }

        void Sweep(double start, double end, double step)
        {
            int count = Math.Max(1, (int)Math.Ceiling((end - start) / step));
            for (int i = 0; i <= count; i++)
                Consider(Math.Min(end, start + i * step));
        }

        const double coarseStepSeconds = 60;
        Sweep(DepartureStartTime,
            DepartureStartTime + DepartureSearchDuration,
            coarseStepSeconds);
        if (!double.IsFinite(bestTime))
            throw new InvalidOperationException(
                "departure-time sweep produced no lunar prediction");
        Sweep(bestTime - coarseStepSeconds, bestTime + coarseStepSeconds, 10);
        Sweep(bestTime - 10, bestTime + 10, 1);

        Result = new GameTestPlayerLunarTransferSolution(
            bestTime, new Vector3d(ProgradeDeltaV, 0, 0), bestPerilune);
    }

    private double EvaluatePerilune(
        SolverPrediction solver, double departureTime)
    {
        var coast = new TrajectoryPredictor(solver.Gravity,
            CoastSeedState, CoastSeedTime,
            new IntegratorOptions { RelTol = 1e-10 });
        StateVector departureState = solver.StateAt(coast, departureTime, 3600);
        StateVector earth = solver.GetAbsolute("Earth", departureTime);
        StateVector earthRelative = departureState - earth;
        Vector3d deltaV = BurnFrameKernel.VlfToEcl(
                new Vector3d(ProgradeDeltaV, 0, 0),
                earthRelative.Position, earthRelative.Velocity)
            ?? throw new InvalidOperationException(
                "parking-orbit prograde basis is degenerate");
        var transfer = new TrajectoryPredictor(solver.Gravity,
            departureState with { Velocity = departureState.Velocity + deltaV },
            departureTime, new IntegratorOptions { RelTol = 1e-10 });
        return FindPeriluneRadius(solver, transfer, departureTime,
            departureTime + FlightDuration, LunaRadiusMeters);
    }

    internal static double FindPeriluneRadius(
        SolverPrediction solver, TrajectoryPredictor predictor,
        double startTime, double endTime, double lunaRadiusMeters)
    {
        const int samples = 256;
        var distances = new double[samples];
        double step = (endTime - startTime) / (samples - 1);
        double DistanceAt(double time) =>
            solver.RelativeState(predictor, "Luna", time, 6 * 3600).RRel.Length();

        int minimum = 0;
        for (int i = 0; i < samples; i++)
        {
            double time = startTime + i * step;
            try { distances[i] = DistanceAt(time); }
            catch (InvalidOperationException) when (i > 0
                && distances[i - 1] <= 5 * lunaRadiusMeters)
            {
                return 0;
            }
            if (distances[i] <= lunaRadiusMeters)
                return distances[i];
            if (distances[i] < distances[minimum])
                minimum = i;
        }
        if (minimum == 0 || minimum == samples - 1)
            return distances[minimum];
        return PeriapsisKernel.RefineMinimum(DistanceAt,
            startTime + (minimum - 1) * step,
            startTime + (minimum + 1) * step,
            distanceTolerance: 10).Distance;
    }
}

internal readonly record struct GameTestPlayerLunarCorrectionSolution(
    double BurnTime, Vector3d DeltaVVlf, double PeriluneRadiusMeters);

internal sealed class GameTestPlayerLunarCorrectionSolveJob
{
    private static readonly Vector3d[] Directions = BuildDirections();
    private static readonly double[] Magnitudes =
        [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100];

    public required RailsService.PredictionContext Prediction { get; init; }
    public required StateVector SeedState { get; init; }
    public required double StartTime { get; init; }
    public required double BurnTime { get; init; }
    public required double EndTime { get; init; }
    public required double DesiredPeriluneRadiusMeters { get; init; }
    public required double LunaRadiusMeters { get; init; }

    private volatile bool _done;
    public bool Done => _done;
    public GameTestPlayerLunarCorrectionSolution? Result { get; private set; }
    public string? Failure { get; private set; }

    public void Start() => new Thread(Run)
    {
        IsBackground = true,
        Name = "whiskerdynamics-game-test-player-lunar-correction",
        Priority = ThreadPriority.BelowNormal,
    }.Start();

    private void Run()
    {
        try { Solve(); }
        catch (Exception e)
        {
            Failure = $"player-style lunar correction solve failed: {e.Message}";
            ModLog.Warn($"game test: player-style lunar correction solve contained: {e}");
        }
        finally { _done = true; }
    }

    private void Solve()
    {
        var solver = new SolverPrediction(Prediction, static () => false);
        var coast = new TrajectoryPredictor(solver.Gravity,
            SeedState, StartTime, new IntegratorOptions { RelTol = 1e-10 });
        StateVector burnState = solver.StateAt(coast, BurnTime, 3600);
        StateVector earth = solver.GetAbsolute("Earth", BurnTime);
        StateVector earthRelative = burnState - earth;
        Vector3d bestVlf = default;
        double bestPerilune = Evaluate(default);
        double bestError = Math.Abs(bestPerilune - DesiredPeriluneRadiusMeters);

        double Evaluate(Vector3d deltaVVlf)
        {
            Vector3d deltaVEcl = BurnFrameKernel.VlfToEcl(deltaVVlf,
                    earthRelative.Position, earthRelative.Velocity)
                ?? throw new InvalidOperationException(
                    "correction-burn VLF basis is degenerate");
            var corrected = new TrajectoryPredictor(solver.Gravity,
                burnState with { Velocity = burnState.Velocity + deltaVEcl },
                BurnTime, new IntegratorOptions { RelTol = 1e-10 });
            return GameTestPlayerLunarTransferSolveJob.FindPeriluneRadius(
                solver, corrected, BurnTime, EndTime, LunaRadiusMeters);
        }

        void Consider(Vector3d deltaVVlf, double perilune)
        {
            double error = Math.Abs(perilune - DesiredPeriluneRadiusMeters);
            if (error >= bestError) return;
            bestVlf = deltaVVlf;
            bestPerilune = perilune;
            bestError = error;
        }

        foreach (Vector3d direction in Directions)
        {
            double lowerMagnitude = 0;
            double lowerPerilune = Evaluate(default);
            foreach (double magnitude in Magnitudes)
            {
                Vector3d deltaV = direction * magnitude;
                double perilune = Evaluate(deltaV);
                Consider(deltaV, perilune);
                double lowerOffset = lowerPerilune - DesiredPeriluneRadiusMeters;
                double upperOffset = perilune - DesiredPeriluneRadiusMeters;
                if (lowerOffset * upperOffset <= 0
                    && lowerOffset != upperOffset)
                {
                    double low = lowerMagnitude;
                    double high = magnitude;
                    for (int i = 0; i < 16; i++)
                    {
                        double mid = 0.5 * (low + high);
                        Vector3d midDeltaV = direction * mid;
                        double midPerilune = Evaluate(midDeltaV);
                        Consider(midDeltaV, midPerilune);
                        if ((midPerilune - DesiredPeriluneRadiusMeters)
                            * lowerOffset <= 0)
                            high = mid;
                        else
                        {
                            low = mid;
                            lowerOffset = midPerilune
                                - DesiredPeriluneRadiusMeters;
                        }
                    }
                    break;
                }
                lowerMagnitude = magnitude;
                lowerPerilune = perilune;
            }
        }
        if (!(bestVlf.Length() >= 0.01) || bestVlf.Length() > 100)
            throw new InvalidOperationException(
                $"no practical correction found; best predicted perilune "
                + $"{bestPerilune - LunaRadiusMeters:F1} m altitude");
        Result = new GameTestPlayerLunarCorrectionSolution(
            BurnTime, bestVlf, bestPerilune);
    }

    private static Vector3d[] BuildDirections()
    {
        var directions = new List<Vector3d>();
        for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
        for (int z = -1; z <= 1; z++)
        {
            if (x == 0 && y == 0 && z == 0) continue;
            directions.Add(new Vector3d(x, y, z).Normalized());
        }
        return [.. directions];
    }
}

internal readonly record struct GameTestPlayerLunarCircularizationSolution(
    double BurnTime, Vector3d LunaFrameDeltaVPrn,
    double PeriluneRadiusMeters);

internal sealed class GameTestPlayerLunarCircularizationSolveJob
{
    public required RailsService.PredictionContext Prediction { get; init; }
    public required StateVector SeedState { get; init; }
    public required double StartTime { get; init; }
    public required double EndTime { get; init; }
    public required double LunaMu { get; init; }
    public required double LunaRadiusMeters { get; init; }

    private volatile bool _done;
    public bool Done => _done;
    public GameTestPlayerLunarCircularizationSolution? Result { get; private set; }
    public string? Failure { get; private set; }

    public void Start() => new Thread(Run)
    {
        IsBackground = true,
        Name = "whiskerdynamics-game-test-earth-soi-lunar-circularization",
        Priority = ThreadPriority.BelowNormal,
    }.Start();

    private void Run()
    {
        try { Solve(); }
        catch (Exception e)
        {
            Failure = $"Earth-SOI lunar circularization solve failed: {e.Message}";
            ModLog.Warn($"game test: Earth-SOI lunar circularization solve contained: {e}");
        }
        finally { _done = true; }
    }

    private void Solve()
    {
        var solver = new SolverPrediction(Prediction, static () => false);
        var predictor = new TrajectoryPredictor(solver.Gravity,
            SeedState, StartTime, new IntegratorOptions { RelTol = 1e-10 });
        const int samples = 512;
        var distances = new double[samples];
        double step = (EndTime - StartTime) / (samples - 1);
        double DistanceAt(double time) =>
            solver.RelativeState(predictor, "Luna", time, 6 * 3600).RRel.Length();
        for (int i = 0; i < samples; i++)
        {
            distances[i] = DistanceAt(StartTime + i * step);
            if (distances[i] <= LunaRadiusMeters)
                throw new InvalidOperationException(
                    "predicted transfer intersects Luna before circularization");
        }
        int minimum = -1;
        for (int i = 1; i < samples - 1; i++)
        {
            if (distances[i] < distances[i - 1]
                && distances[i] <= distances[i + 1])
            {
                minimum = i;
                break;
            }
        }
        if (minimum < 1)
            throw new InvalidOperationException(
                "no interior lunar perilune exists in the prediction window");
        var (burnTime, periluneRadius) = PeriapsisKernel.RefineMinimum(
            DistanceAt, StartTime + (minimum - 1) * step,
            StartTime + (minimum + 1) * step,
            distanceTolerance: 1);
        var (position, velocity) = solver.RelativeState(
            predictor, "Luna", burnTime, 6 * 3600);
        double circularSpeed = Math.Sqrt(LunaMu / position.Length());
        double retrogradeDeltaV = velocity.Length() - circularSpeed;
        if (!(retrogradeDeltaV > 1 && retrogradeDeltaV <= 2_000))
            throw new InvalidOperationException(
                $"circularization is not a practical pure retrograde burn: "
                + $"speed={velocity.Length():R} m/s, "
                + $"circular speed={circularSpeed:R} m/s");
        var deltaVPrn = new Vector3d(-retrogradeDeltaV, 0, 0);
        Result = new GameTestPlayerLunarCircularizationSolution(
            burnTime, deltaVPrn, periluneRadius);
    }
}

using System.Text.Json;
using System.Text.Json.Serialization;

namespace WhiskerDynamics.GameTesting;

public static class GameTestProtocol
{
    public const string RequestFileName = "game-test-request.json";
    public const string ResultFileName = "game-test-result.json";

    public static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true,
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        WriteIndented = true,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
    };
}

public sealed class GameTestScenario
{
    public string RunId { get; set; } = "";
    public string Name { get; set; } = "";
    public double TimeoutSeconds { get; set; } = 300;
    public List<GameTestStep> Steps { get; set; } = [];
}

public sealed class GameTestStep
{
    public string Action { get; set; } = "";
    public string? Vessel { get; set; }
    public string? Expected { get; set; }
    public int? OrbitOffset { get; set; }
    public double? TimeoutSeconds { get; set; }
    public double? DurationSeconds { get; set; }
    public double? OffsetSeconds { get; set; }
    public double? DeltaVMetersPerSecond { get; set; }
    public double? CorrectionTimescaleSeconds { get; set; }
    public double? TargetRadiusMeters { get; set; }
    public double? MinPeriluneAltitudeMeters { get; set; }
    public double? MaxPeriluneAltitudeMeters { get; set; }
    public double? MinEccentricity { get; set; }
}

public sealed class GameTestResult
{
    public string RunId { get; set; } = "";
    public string Scenario { get; set; } = "";
    public bool Passed { get; set; }
    public string? Error { get; set; }
    public double ElapsedWallSeconds { get; set; }
    public double? FinalSimulationTime { get; set; }
    public List<GameTestStepResult> Steps { get; set; } = [];
}

public sealed class GameTestStepResult
{
    public int Index { get; set; }
    public string Action { get; set; } = "";
    public bool Passed { get; set; }
    public string Detail { get; set; } = "";
    public double? SimulationTime { get; set; }
}

/// <summary>A compiled, user-authored scenario discovered by the game-test host.</summary>
public interface IGameTestScenario
{
    string Id { get; }
    GameTestScenario Create();
}

/// <summary>Fluent C# authoring surface. The built object is serialized only as the
/// private host-to-game transport envelope.</summary>
public sealed class GameScenarioBuilder
{
    private readonly GameTestScenario _scenario;

    private GameScenarioBuilder(string name) => _scenario = new GameTestScenario
    {
        Name = name,
    };

    public static GameScenarioBuilder Named(string name) => new(name);

    public GameScenarioBuilder WithTimeout(double wallSeconds)
    {
        _scenario.TimeoutSeconds = wallSeconds;
        return this;
    }

    public GameScenarioBuilder WaitUntilReady(double? timeoutSeconds = null) =>
        Add(new GameTestStep { Action = "wait-ready", TimeoutSeconds = timeoutSeconds });

    public GameScenarioBuilder PlanAndExecuteLunarTransfer(double offsetSeconds,
        double flightDurationSeconds, double targetPeriluneAltitudeMeters,
        string? vessel = null, double? timeoutSeconds = null,
        int departureOrbitOffset = 0) => Add(new GameTestStep
        {
            Action = "plan-and-execute-lunar-transfer",
            Vessel = vessel,
            OffsetSeconds = offsetSeconds,
            DurationSeconds = flightDurationSeconds,
            TargetRadiusMeters = targetPeriluneAltitudeMeters,
            DeltaVMetersPerSecond = 3_150,
            CorrectionTimescaleSeconds = 3_600,
            TimeoutSeconds = timeoutSeconds,
            OrbitOffset = departureOrbitOffset,
        });

    public GameScenarioBuilder AssertPeriluneAltitudeMetersBetween(
        double minimumMeters, double maximumMeters,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "assert-perilune-altitude-between",
            Vessel = vessel,
            MinPeriluneAltitudeMeters = minimumMeters,
            MaxPeriluneAltitudeMeters = maximumMeters,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder PlanLunarCircularizationFromEarthSoi(
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "plan-lunar-circularization-from-earth-soi",
            Vessel = vessel,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder AssertBadLunarCircularization(
        double minimumEccentricity = 0.05,
        string? vessel = null) => Add(new GameTestStep
        {
            Action = "assert-bad-lunar-circularization",
            Vessel = vessel,
            MinEccentricity = minimumEccentricity,
        });

    public GameScenarioBuilder ExecuteBurns(string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "execute-burns",
            Vessel = vessel,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder AssertParent(string expected, string? vessel = null) =>
        Add(new GameTestStep
        {
            Action = "assert-parent",
            Expected = expected,
            Vessel = vessel,
        });

    public GameTestScenario Build() => _scenario;

    private GameScenarioBuilder Add(GameTestStep step)
    {
        _scenario.Steps.Add(step);
        return this;
    }
}

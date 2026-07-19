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
    public string Save { get; set; } = "";
    public double TimeoutSeconds { get; set; } = 300;
    public List<GameTestStep> Steps { get; set; } = [];
}

public sealed class GameTestStep
{
    public string Action { get; set; } = "";
    public string? Vessel { get; set; }
    public string? Target { get; set; }
    public string? Expected { get; set; }
    public int? Count { get; set; }
    public double? TimeoutSeconds { get; set; }
    public double? Speed { get; set; }
    public double? DurationSeconds { get; set; }
    public double? OffsetSeconds { get; set; }
    public double? Prograde { get; set; }
    public double? Normal { get; set; }
    public double? Outward { get; set; }
    public double? MaxDistanceMeters { get; set; }
    public double? DeltaVMetersPerSecond { get; set; }
    public double? CorrectionTimescaleSeconds { get; set; }
    public double? MaxDeltaVMetersPerSecond { get; set; }
    public double? MaxPeriluneRadiusMeters { get; set; }
    public double? MinApoluneRadiusMeters { get; set; }
    public double? MaxApoluneRadiusMeters { get; set; }
    public double? TargetRadiusMeters { get; set; }
    public double? MaxPositionErrorMeters { get; set; }
    public double? MaxVelocityErrorMetersPerSecond { get; set; }
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

    public GameScenarioBuilder FromSave(string save)
    {
        _scenario.Save = save;
        return this;
    }

    public GameScenarioBuilder FromDefaultSystem()
    {
        _scenario.Save = "";
        return this;
    }

    public GameScenarioBuilder WithTimeout(double wallSeconds)
    {
        _scenario.TimeoutSeconds = wallSeconds;
        return this;
    }

    public GameScenarioBuilder WaitUntilReady(double? timeoutSeconds = null) =>
        Add(new GameTestStep { Action = "wait-ready", TimeoutSeconds = timeoutSeconds });

    public GameScenarioBuilder AssertModActive() =>
        Add(new GameTestStep { Action = "assert-mod-active" });

    public GameScenarioBuilder SetTarget(string target, string? vessel = null) =>
        Add(new GameTestStep { Action = "set-target", Target = target, Vessel = vessel });

    public GameScenarioBuilder PlanRendezvous(string target, double planDurationSeconds,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "plan-rendezvous",
            Target = target,
            Vessel = vessel,
            DurationSeconds = planDurationSeconds,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder ControlVessel(string vessel) => Add(new GameTestStep
    {
        Action = "control-vessel",
        Vessel = vessel,
    });

    public GameScenarioBuilder Refill(string vessel) => Add(new GameTestStep
    {
        Action = "refill",
        Vessel = vessel,
    });

    public GameScenarioBuilder SaveAs(string saveName) => Add(new GameTestStep
    {
        Action = "save-as",
        Target = saveName,
    });

    public GameScenarioBuilder AddBurn(double offsetSeconds, double prograde,
        double normal = 0, double outward = 0, string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "add-burn",
            Vessel = vessel,
            OffsetSeconds = offsetSeconds,
            Prograde = prograde,
            Normal = normal,
            Outward = outward,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder PlanLunarTransfer(double offsetSeconds,
        double flightDurationSeconds, double targetLunarRadiusMeters,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "plan-lunar-transfer",
            Vessel = vessel,
            OffsetSeconds = offsetSeconds,
            DurationSeconds = flightDurationSeconds,
            TargetRadiusMeters = targetLunarRadiusMeters,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder PlanLunarOrbitInsertion(double targetApoluneRadiusMeters,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "plan-lunar-orbit-insertion",
            Vessel = vessel,
            TargetRadiusMeters = targetApoluneRadiusMeters,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder WarpFor(double speed, double simulationSeconds,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "warp-for",
            Speed = speed,
            DurationSeconds = simulationSeconds,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder ExecuteBurns(string? vessel = null,
        double? timeoutSeconds = null, double burnWarpSpeed = 10) => Add(new GameTestStep
        {
            Action = "execute-burns",
            Vessel = vessel,
            TimeoutSeconds = timeoutSeconds,
            Speed = burnWarpSpeed,
        });

    public GameScenarioBuilder ExecuteBurnsWithRcs(string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "execute-burns-rcs",
            Vessel = vessel,
            TimeoutSeconds = timeoutSeconds,
        });

    /// <summary>Moves the vessel, through KSA's stock teleport input, onto the
    /// pre-insertion side of the deterministic Earth-Luna L2 southern NRHO used by
    /// the integration scenario. The approach state is faster than the reference by
    /// <paramref name="insertionDeltaVMetersPerSecond"/>.</summary>
    public GameScenarioBuilder TeleportToNrhoApproach(
        double insertionDeltaVMetersPerSecond = 20, string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "teleport-nrho-approach",
            Vessel = vessel,
            DeltaVMetersPerSecond = insertionDeltaVMetersPerSecond,
            TimeoutSeconds = timeoutSeconds,
        });

    /// <summary>Warps from the close lunar passage to the distant NRHO lobe and
    /// stops just after the local maximum radius.</summary>
    public GameScenarioBuilder WarpToNrhoApolune(double speed,
        double minApoluneRadiusMeters, double maxApoluneRadiusMeters,
        string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "warp-nrho-apolune",
            Vessel = vessel,
            Speed = speed,
            MinApoluneRadiusMeters = minApoluneRadiusMeters,
            MaxApoluneRadiusMeters = maxApoluneRadiusMeters,
            TimeoutSeconds = timeoutSeconds,
        });

    /// <summary>Warps from the distant lobe to the close lunar passage and stops
    /// just after the local minimum radius.</summary>
    public GameScenarioBuilder WarpToNrhoPerilune(double speed,
        double maxPeriluneRadiusMeters, string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "warp-nrho-perilune",
            Vessel = vessel,
            Speed = speed,
            MaxPeriluneRadiusMeters = maxPeriluneRadiusMeters,
            TimeoutSeconds = timeoutSeconds,
        });

    /// <summary>Plans a bounded position-plus-velocity feedback correction against
    /// the scenario's propagated NRHO reference at the burn epoch.</summary>
    public GameScenarioBuilder AddNrhoStationKeepingBurn(double offsetSeconds,
        double correctionTimescaleSeconds, double maxDeltaVMetersPerSecond,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "add-nrho-station-keeping-burn",
            Vessel = vessel,
            OffsetSeconds = offsetSeconds,
            CorrectionTimescaleSeconds = correctionTimescaleSeconds,
            MaxDeltaVMetersPerSecond = maxDeltaVMetersPerSecond,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder Pause() => Add(new GameTestStep { Action = "pause" });

    public GameScenarioBuilder AssertNrhoTracking(string phase,
        double maxPositionErrorMeters, double maxVelocityErrorMetersPerSecond,
        string? vessel = null) =>
        Add(new GameTestStep
        {
            Action = "assert-nrho-tracking",
            Vessel = vessel,
            Expected = phase,
            MaxPositionErrorMeters = maxPositionErrorMeters,
            MaxVelocityErrorMetersPerSecond = maxVelocityErrorMetersPerSecond,
        });

    public GameScenarioBuilder AssertParent(string expected, string? vessel = null) =>
        Add(new GameTestStep
        {
            Action = "assert-parent",
            Expected = expected,
            Vessel = vessel,
        });

    public GameScenarioBuilder WaitForParent(string expected, double? warpSpeed = null,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "wait-parent",
            Expected = expected,
            Speed = warpSpeed,
            Vessel = vessel,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder WaitForOutboundLunarEncounter(double warpSpeed,
        string? vessel = null, double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "wait-outbound-lunar-encounter",
            Vessel = vessel,
            Speed = warpSpeed,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder AssertBoundLunarOrbit(double maxApoluneRadiusMeters,
        string? vessel = null) => Add(new GameTestStep
        {
            Action = "assert-bound-lunar-orbit",
            Vessel = vessel,
            MaxApoluneRadiusMeters = maxApoluneRadiusMeters,
        });

    public GameScenarioBuilder AssertBurnCount(int expected, string? vessel = null) =>
        Add(new GameTestStep
        {
            Action = "assert-burn-count",
            Count = expected,
            Vessel = vessel,
        });

    public GameScenarioBuilder WaitForBurnCount(int expected, string? vessel = null,
        double? timeoutSeconds = null) => Add(new GameTestStep
        {
            Action = "wait-burn-count",
            Count = expected,
            Vessel = vessel,
            TimeoutSeconds = timeoutSeconds,
        });

    public GameScenarioBuilder AssertDistanceToTarget(string target,
        double maxDistanceMeters, string? vessel = null) => Add(new GameTestStep
        {
            Action = "assert-distance-to-target",
            Target = target,
            MaxDistanceMeters = maxDistanceMeters,
            Vessel = vessel,
        });

    public GameTestScenario Build() => _scenario;

    private GameScenarioBuilder Add(GameTestStep step)
    {
        _scenario.Steps.Add(step);
        return this;
    }
}

using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

public sealed class MoonTransferScenario : IGameTestScenario
{
    public string Id => "moon-transfer";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("Earth to Luna transfer and capture")
        .FromSave("integration-lunar-transfer-v3")
        .WithTimeout(900)
        .WaitUntilReady()
        .Refill("Rocket")
        .AssertParent("Earth")
        .PlanLunarTransfer(offsetSeconds: 600,
            flightDurationSeconds: 3 * 86400,
            targetLunarRadiusMeters: 8_000_000,
            vessel: "Rocket", timeoutSeconds: 180)
        .ExecuteBurns(timeoutSeconds: 600, burnWarpSpeed: 1)
        .WaitForOutboundLunarEncounter(warpSpeed: 10_000,
            vessel: "Rocket", timeoutSeconds: 300)
        .PlanLunarOrbitInsertion(targetApoluneRadiusMeters: 30_000_000,
            vessel: "Rocket")
        .ExecuteBurns(vessel: "Rocket", timeoutSeconds: 600, burnWarpSpeed: 1)
        .AssertBoundLunarOrbit(maxApoluneRadiusMeters: 50_000_000, vessel: "Rocket")
        .WarpFor(speed: 10_000, simulationSeconds: 2 * 86400, timeoutSeconds: 120)
        .AssertBoundLunarOrbit(maxApoluneRadiusMeters: 50_000_000, vessel: "Rocket")
        .Build();
}

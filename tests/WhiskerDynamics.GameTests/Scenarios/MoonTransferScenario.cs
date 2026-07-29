using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

public sealed class MoonTransferScenario : IGameTestScenario
{
    public string Id => "moon-transfer";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("Pre-SOI lunar circularization Auto Warp repro")
        .WithTimeout(1500)
        .WaitUntilReady()
        .AssertParent("Earth")
        .PlanAndExecuteLunarTransfer(offsetSeconds: 600,
            flightDurationSeconds: 3 * 86400,
            targetPeriluneAltitudeMeters: 500_000,
            vessel: "Rocket", timeoutSeconds: 600,
            departureOrbitOffset: 1)
        .AssertPeriluneAltitudeMetersBetween(200_000, 800_000)
        .PlanLunarCircularizationFromEarthSoi(
            vessel: "Rocket", timeoutSeconds: 300)
        .ExecuteBurns(vessel: "Rocket", timeoutSeconds: 600)
        .AssertBadLunarCircularization(
            minimumEccentricity: 0.02, vessel: "Rocket")
        .Build();
}

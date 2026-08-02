using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

public sealed class MoonTransferFastWarpScenario : IGameTestScenario
{
    public string Id => "moon-transfer-fast-warp";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("lunar circularization warped across the SOI boundary by built-in Auto Warp")
        .WithTimeout(1500)
        .WaitUntilReady()
        .AutoStage(count: 2, vessel: "Rocket", timeoutSeconds: 30)
        .AssertParent("Earth")
        .PlanAndExecuteLunarTransfer(offsetSeconds: 600,
            flightDurationSeconds: 3 * 86400,
            targetPeriluneAltitudeMeters: 200_000,
            vessel: "Rocket", timeoutSeconds: 600,
            departureOrbitOffset: 1)
        .PlanLunarCircularizationFromEarthSoi(
            minimumPeriluneAltitudeMeters: 100_000,
            maximumPeriluneAltitudeMeters: 300_000,
            vessel: "Rocket", timeoutSeconds: 300)
        .ExecuteBurns(vessel: "Rocket", timeoutSeconds: 600,
            lunarCircularization: true, expectSoiCrossing: true)
        .CompleteLunarOrbit(
            maximumEccentricity: 0.25,
            vessel: "Rocket", timeoutSeconds: 300)
        .Build();
}

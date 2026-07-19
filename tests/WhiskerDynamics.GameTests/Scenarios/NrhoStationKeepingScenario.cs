using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

public sealed class NrhoStationKeepingScenario : IGameTestScenario
{
    public string Id => "nrho-station-keeping";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("NRHO insertion and station keeping")
        .FromSave("integration-lunar-transfer-v3")
        .WithTimeout(600)
        .WaitUntilReady()
        .Refill("Rocket")
        .TeleportToNrhoApproach(insertionDeltaVMetersPerSecond: 2,
            vessel: "Rocket", timeoutSeconds: 60)
        .AssertParent("Luna", vessel: "Rocket")
        .AddBurn(offsetSeconds: 60, prograde: -2, vessel: "Rocket")
        .ExecuteBurnsWithRcs(vessel: "Rocket", timeoutSeconds: 180)
        .WarpToNrhoApolune(speed: 10_000,
            minApoluneRadiusMeters: 50_000_000,
            maxApoluneRadiusMeters: 90_000_000,
            vessel: "Rocket", timeoutSeconds: 180)
        .AssertNrhoTracking("apolune", maxPositionErrorMeters: 10_000_000,
            maxVelocityErrorMetersPerSecond: 40,
            vessel: "Rocket")
        .AddNrhoStationKeepingBurn(offsetSeconds: 60,
            correctionTimescaleSeconds: 10 * 86400,
            maxDeltaVMetersPerSecond: 50,
            vessel: "Rocket", timeoutSeconds: 60)
        .ExecuteBurns(vessel: "Rocket", timeoutSeconds: 120)
        .WarpToNrhoPerilune(speed: 10_000,
            maxPeriluneRadiusMeters: 6_000_000,
            vessel: "Rocket", timeoutSeconds: 180)
        .WarpToNrhoApolune(speed: 10_000,
            minApoluneRadiusMeters: 50_000_000,
            maxApoluneRadiusMeters: 90_000_000,
            vessel: "Rocket", timeoutSeconds: 180)
        .AssertNrhoTracking("apolune", maxPositionErrorMeters: 12_000_000,
            maxVelocityErrorMetersPerSecond: 40,
            vessel: "Rocket")
        .AssertBurnCount(0, vessel: "Rocket")
        .Build();
}

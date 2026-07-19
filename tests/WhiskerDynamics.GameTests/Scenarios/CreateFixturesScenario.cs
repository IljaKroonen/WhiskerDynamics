using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

/// <summary>Creates current-build baseline saves from KSA's deterministic default
/// system. Run explicitly when the integration fixtures need to be regenerated.</summary>
public sealed class CreateFixturesScenario : IGameTestScenario
{
    public string Id => "create-fixtures";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("create integration fixtures")
        .FromDefaultSystem()
        .WithTimeout(180)
        .WaitUntilReady()
        .AssertModActive()
        // Capture after a committed update rather than at the default system's
        // initialization boundary.
        .WarpFor(speed: 1, simulationSeconds: 2)
        .Refill("Rocket")
        .Refill("Gemini7")
        .SaveAs("integration-lunar-transfer-v3")
        .SaveAs("integration-rendezvous-v3")
        .Build();
}

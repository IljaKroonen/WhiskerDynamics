using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

public sealed class SmokeScenario : IGameTestScenario
{
    public string Id => "smoke";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("mod smoke test")
        .FromDefaultSystem()
        .WithTimeout(120)
        .WaitUntilReady()
        .Refill("Rocket")
        .AssertModActive()
        .AssertParent("Earth")
        .WarpFor(speed: 100, simulationSeconds: 600)
        .AssertParent("Earth")
        .Build();
}

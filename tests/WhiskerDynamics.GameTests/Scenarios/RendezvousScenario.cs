using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Scenarios;

public sealed class RendezvousScenario : IGameTestScenario
{
    public string Id => "rendezvous";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("two-burn rendezvous")
        .FromSave("integration-rendezvous-v3")
        .WithTimeout(900)
        .WaitUntilReady()
        .ControlVessel("Gemini7")
        .Refill("Gemini7")
        .SetTarget("Hunter", vessel: "Gemini7")
        // Keep the fixture's close formation as a short-transfer problem. A multi-day
        // window selects millimetres-per-second, many-revolution solutions below this
        // vessel's stock flight-computer/engine control resolution.
        .PlanRendezvous("Hunter", planDurationSeconds: 30 * 60,
            vessel: "Gemini7", timeoutSeconds: 300)
        .ExecuteBurnsWithRcs(vessel: "Gemini7", timeoutSeconds: 600)
        .AssertBurnCount(0, vessel: "Gemini7")
        .AssertDistanceToTarget("Hunter", maxDistanceMeters: 1000,
            vessel: "Gemini7")
        .Build();
}

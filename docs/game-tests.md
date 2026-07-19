# In-game scenario tests

`WhiskerDynamics.GameTests` launches the real game through StarMap, loads a named
save, drives KSA and the mod on the game's main thread, and exits non-zero when a
scenario assertion fails. It complements the offline unit suite; it does not mock
KSA physics or the stock flight computer.

## Run a scenario

1. Create a deterministic save under
   `Documents\My Games\Kitten Space Agency\saves`. Treat the save as the fixture:
   vessel names, target names, engine state, fuel, orbit, epoch, and existing burns
   are all scenario inputs.
2. Add an `IGameTestScenario` class under
   `tests/WhiskerDynamics.GameTests/Scenarios`, using `GameScenarioBuilder` to define
   the fixture, actions, and assertions. The host discovers it automatically.
3. Close any running KSA or StarMap process, then run:

   ```powershell
   dotnet run --project tests/WhiskerDynamics.GameTests -- --list
   dotnet publish -c Release .\src\WhiskerDynamics.Mod --disable-build-servers -m:1
   dotnet run --project tests/WhiskerDynamics.GameTests -- smoke
   ```

The host builds the selected C# scenario, deploys the existing production Release
publish output, publishes and deploys the scenario patches as a separate
`WhiskerDynamics.GameTestDriver` StarMap mod, writes a private transport request
beside the production mod, starts StarMap, waits for the atomic result, prints each
step, and closes only
the isolated KSA/StarMap session it started. Use `--keep-game-running` for visual
inspection, `--no-deploy` to reuse an existing deployment, or `--timeout N` to set
the outer wall-clock timeout.

The deployed `game-test-result.json` and `whiskerdynamics.log` provide machine- and
human-readable evidence. The normal mod publish and deployment contain neither the
driver patches nor `WhiskerDynamics.GameTesting.dll`. When the separate test driver
is installed but no `game-test-request.json` exists, it is inactive apart from that
file-existence check.

JSON is not an authoring surface. It is only the generated inter-process envelope
between the .NET host and the separate KSA process.

`FromDefaultSystem()` uses KSA's freshly loaded default system and is useful for
portable boot/smoke checks. Dynamics scenarios should use `FromSave(...)` with a
dedicated fixture.

## Author a scenario

```csharp
public sealed class EarthOrbitSmoke : IGameTestScenario
{
    public string Id => "earth-orbit-smoke";

    public GameTestScenario Create() => GameScenarioBuilder
        .Named("Earth orbit remains stable")
        .FromSave("integration-earth-orbit")
        .WithTimeout(120)
        .WaitUntilReady()
        .AssertModActive()
        .AssertParent("Earth")
        .WarpFor(speed: 100, simulationSeconds: 600)
        .AssertParent("Earth")
        .Build();
}
```

## Step reference

- `WaitUntilReady()`: wait for the save, mod binding, controlled vessel, and n-body rails.
- `AssertModActive()`: require the mod's lifecycle status to be `Active`.
- `SetTarget(...)`: call KSA's real vehicle target setter.
- `PlanRendezvous(...)`: run the same automatic Whisker Dynamics rendezvous solve
  and transactional two-node creation used by the planner panel.
- `PlanLunarTransfer(...)`: search departure times across the live parking orbit,
  solve Lambert/B-plane candidates, evaluate them in the captured n-body field, and
  queue the lowest-cost outbound lunar encounter that clears Earth and Luna.
- `WaitForOutboundLunarEncounter(...)`: require Luna SOI authority while the vessel
  is still outbound from Earth and approaching Luna, rejecting a return-leg crossing.
- `PlanLunarOrbitInsertion(...)`: use the committed stock lunar osculating orbit to
  queue a retrograde periapsis burn for the requested apolune.
- `AssertBoundLunarOrbit(...)`: require Luna authority, negative lunar orbital energy,
  a safe perilune, and an apolune below the configured limit. Run it again after warp
  to prove the capture survives propagation.
- `ControlVessel(...)` / `Refill(...)`: queue KSA's stock input events and wait
  until the selected vessel is active before executing its flight computer.
- `SaveAs(...)`: create a named save from the current committed KSA state.
- `AddBurn(...)`: queue a stock maneuver node through `BurnPlanWriter`. Its offset is
  relative to current simulation time; VLF components are in m/s. The step waits for
  stock to accept the node.
- `WarpFor(...)`: request a speed, advance by the given simulation duration, then
  return to 1x.
- `ExecuteBurns(...)`: put KSA's flight computer into Auto, use stock
  `WarpToNextBurn` to approach each calculated ignition time, fly it at the requested
  `burnWarpSpeed` (10x by default), verify its residual delta-v, remove the completed
  node, return to 1x, and repeat.
- `ExecuteBurnsWithRcs(...)`: orient to each node with stock RCS attitude control,
  refill after the attitude slew, keep attitude tracking active during forward-RCS
  translation, and require the main engines to remain off throughout. It verifies
  that the committed residual decreases to the measured fine-burn tolerance, removes
  the completed node, refills, and repeats. This is intended for maneuver nodes below
  a vessel's practical main-engine resolution.
- `TeleportToNrhoApproach(...)`: use KSA's stock teleport input to initialize the
  deterministic pre-insertion state for the NASA/JPL-derived Earth-Luna L2 southern
  NRHO scenario.
- `WarpToNrhoApolune(...)` / `WarpToNrhoPerilune(...)`: warp to the next local
  maximum/minimum lunar radius, require the configured distant/close NRHO corridor,
  reject an escaping apolune, and reject a perilune that intersects Luna.
- `AssertNrhoTracking(...)`: compare the committed 3D position and velocity with the
  NASA/JPL-derived reference at the detected apolune or perilune phase.
- `AddNrhoStationKeepingBurn(...)`: propagate the NRHO reference to the burn epoch,
  calculate a bounded position-plus-velocity feedback correction from the actual
  apolune phase, and queue it as a stock VLF maneuver node.
- `Pause()`: set simulation speed to zero.
- `AssertParent(...)` / `WaitForParent(...)`: require an SOI parent id. The waiting
  form can warp and always returns to 1x when matched.
- `AssertBurnCount(...)` / `WaitForBurnCount(...)`: compare the stock burn count.
- `AssertDistanceToTarget(...)`: require a same-parent target within a distance.

Most actions accept an optional vessel id; otherwise they operate on KSA's controlled
vessel. Waiting actions accept a wall-clock timeout. `WithTimeout(...)` bounds the
complete in-game run.

Run `create-fixtures` to generate `integration-lunar-transfer-v3` and
`integration-rendezvous-v3` from the current KSA build. It refills the scenario
vessels before saving and fails if either save
already exists; remove those two fixture directories explicitly when intentionally
regenerating them.

## Moon, NRHO, and rendezvous fixtures

`MoonTransferScenario` solves from the fixture's live epoch and parking orbit. It
requires a genuinely outbound lunar encounter, plans capture from the committed
stock osculating perilune, then checks a safe bound lunar orbit immediately and after
two propagated days. The fixture supplies the vessel and epoch, but the scenario no
longer carries hard-coded transfer or insertion vectors.

`RendezvousScenario` invokes the production automatic solver, waits for both stock
nodes, executes them with forward RCS while the main engines remain off, and asserts
the final separation.

`NrhoStationKeepingScenario` reuses the lunar fixture's `Rocket`, initializes a
2 m/s pre-insertion approach through the stock teleport buffer, performs that fine
insertion with forward RCS, and proves the main engines remained off. It validates a
bounded distant lobe, applies one phase-aligned station-keeping correction, requires
a safe close lunar passage, and then requires a second bounded distant lobe with a
3D reference-state check and no remaining burns. Its normalized reference is the
6.501-day Earth-Moon L2 southern halo record from the NASA/JPL Poincare periodic-orbit
catalog; the game driver scales it to the live Earth-Luna frame rather than assuming
the catalog's dimensional length and time units.

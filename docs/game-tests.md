# In-game moon-transfer repro

This test starts the default `Rocket` in low Earth orbit and uses the real mod
planner and KSA flight computer. No save fixture is required.

## Run it

Close KSA and StarMap, then run:

```powershell
dotnet publish -c Release .\src\WhiskerDynamics.Mod --disable-build-servers -m:1
dotnet run --project tests/WhiskerDynamics.GameTests -- moon-transfer
```

Use `--keep-game-running`, `--no-deploy`, or `--timeout N` as needed.

## Expected result

The transfer must reach a 200–800 km predicted perilune. The test then plans
circularization before entering Luna's SOI and passes only if Auto Warp/Burn loses
the node or target at the transition, or leaves an orbit with eccentricity at least
0.02.

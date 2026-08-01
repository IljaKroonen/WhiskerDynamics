# In-game moon-transfer repro

This test starts the default `Rocket` in low Earth orbit and uses the real mod
planner and KSA flight computer. No save fixture is required.

## Run it

Close KSA and StarMap, then run:

```powershell
dotnet publish -c Release .\src\WhiskerDynamics.Mod
dotnet run --project tests/WhiskerDynamics.GameTests -- moon-transfer
```

Use `--keep-game-running`, `--no-deploy`, or `--timeout N` as needed.

## Expected result

The driver first activates two staging sequences so the default vessel flies the
scenario with its single-stage engine setup. The transfer targets a 200 km lunar
perilune and must predict one in the 100–300 km range without refinement burns. It
plans, immediately after the correction burn, one pure retrograde Luna-centered
inertial (LCI) burn whose magnitude aims for circular speed at perilune. The driver
arms Auto Burn, presses the built-in Auto Warp button, and makes no further plan
edits or simulation-speed changes before the LCI burn completes. A missing node or
automatic-execution target is never recreated. No post-capture cleanup burn is
allowed: that single LCI burn must produce a safe bound orbit with eccentricity at
most 0.25, after which the vessel completes one full lunar revolution.

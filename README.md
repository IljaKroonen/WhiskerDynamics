# Whisker Dynamics

N-body flight dynamics for Kitten Space Agency. Heavily inspired by KSP mod [Principia](https://github.com/mockingbirdnest/Principia).

Whisker Dynamics replaces patched-conic gravity with numerical trajectories for
celestial bodies and vessels. While active, it is the sole authority for gravity
and on-rails propagation; KSA continues to handle thrust, drag, buoyancy,
collisions, and joints. Stock conics and SOIs remain compatibility and UI
mechanisms, not alternate dynamics.

The mod is experimental and build-specific. Each release supports exactly the
KSA build it was verified against. A mismatch disables the mod and leaves the
game running stock.

## Capabilities

- N-body celestial and vessel propagation, including Earth J2 and configurable
  Luna gravity harmonics.
- Sampled celestial, actual, planned, and flown paths with event markers, orbit
  analysis, and selectable reference frames.
- Finite-burn plan prediction, maneuver optimization, and automatic rendezvous
  searches.
- Stock-compatible saves, with exact mod state kept in a sidecar.

The governing constraints are in [the design cornerstones](docs/design.md).

## Build and test

Mod-layer projects require a KSA installation. The default path is
`C:\Program Files\Kitten Space Agency`; override it with `KsaInstallDir` or
`KSA_INSTALL_DIR`. StarMap defaults to `%LOCALAPPDATA%\StarMap`; override it with
`StarMapDir` or `STARMAP_DIR`.

```powershell
dotnet test
```

Real-game scenarios and the benchmark harness provide the focused integration
and numerical validation workflows.

## Install

Install [StarMap Mod Loader](https://github.com/StarMapLoader/StarMap) and the
.NET 10 runtime, then publish, deploy, and launch:

```powershell
dotnet publish -c Release .\src\WhiskerDynamics.Mod
dotnet run --file .\scripts\deploy-mod.cs
& (Join-Path $env:LOCALAPPDATA 'StarMap\StarMap.exe')
```

Deployment writes to
`Documents\My Games\Kitten Space Agency\mods\WhiskerDynamics` and does not
modify the game installation. Configuration and logs live in that directory.

Launch `KSA.exe` directly to run without the mod. To uninstall, remove the
deployed `WhiskerDynamics` directory and its optional `manifest.toml` entry.

## Package

Create a versioned SpaceDock archive with:

```powershell
dotnet run --file .\scripts\create-spacedock-bundle.cs -- 0.1.0
```

The script publishes the mod and writes
`artifacts\WhiskerDynamics-0.1.0.zip`. The archive contains a
`WhiskerDynamics` directory and can be extracted directly into
`Documents\My Games\Kitten Space Agency\mods`.

## Documentation

- [Configuration](docs/configuration.md)
- [Design cornerstones](docs/design.md)
- [In-game scenario tests](docs/game-tests.md)
- [Publishing releases](docs/releases.md)

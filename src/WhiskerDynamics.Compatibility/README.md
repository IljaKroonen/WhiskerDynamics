# Compatibility contract

This directory owns the game-build policy, reflected member registry, validator,
and enum-value contract used by both the mod and the compatibility CLI.

`WhiskerDynamics.Compatibility.csproj` is an ordinary class library referenced
by the mod and the compatibility CLI. It is deployed beside the mod with the
other Whisker Dynamics dependencies.

Contract types live under `WhiskerDynamics.Compatibility` and
`WhiskerDynamics.Compatibility.Patching`. Internal contract types are exposed
only to the Mod, Mod.Tests, and Compatibility.Cli assemblies through explicit
friend access.

Keep the separate `PanelTargets`/`GameplayTargets` static initializers unchanged
unless the activation failure behavior is being changed deliberately.

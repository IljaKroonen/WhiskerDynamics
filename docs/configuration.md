# Configuration

The generated `whiskerdynamics.toml` contains persistent settings. The Orbit look-ahead
control is session-only and starts at 30 days on every process launch. These are the main
user-facing configuration keys:

| Key | Default | Effect |
|---|---:|---|
| `enabled` | `true` | Enables mod activation |
| `lunar_gravity_model` | `degree30` | GRGM1200A truncation: `degree10`, `degree20`, `degree30`, `degree40`, or `degree50` |
| `overlay_max_turn_deg` | `0.4` | Maximum angular turn per vessel or celestial segment, in degrees |
| `overlay_max_points` | `65536` | Maximum drawn points per vessel path |
| `celestial_max_points` | `8192` | Maximum drawn points per celestial path |
| `celestial_curve_max_bodies` | `128` | Maximum number of mod-drawn celestial paths |

A KSA version other than the release's verified build logs a warning and proceeds
through startup compatibility checks. Concrete API, enum, or patch-activation
incompatibilities still disable the mod's gameplay patches and leave the game
running stock.

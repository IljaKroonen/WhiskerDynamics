# Configuration

The generated `whiskerdynamics.toml` contains persistent settings. The Orbit look-ahead
control is session-only and starts at 30 days on every process launch. These are the main
user-facing configuration keys:

| Key | Default | Effect |
|---|---:|---|
| `enabled` | `true` | Enables mod activation |
| `overlay_max_turn_deg` | `0.4` | Maximum angular turn per vessel or celestial segment, in degrees |
| `overlay_max_points` | `65536` | Maximum drawn points per vessel path |
| `celestial_max_points` | `8192` | Maximum drawn points per celestial path |
| `celestial_curve_max_bodies` | `128` | Maximum number of mod-drawn celestial paths |

## Body gravity settings

Extended gravity is configured separately from `whiskerdynamics.toml`. The shipped
`body-settings` directory contains one JSON file per configured body. At system bind,
Whisker Dynamics first snapshots every body from the running game's catalog. Mass,
mean radius, hierarchy, defining orbit, sphere of influence, and body rotation remain
game-catalog values. A matching body-settings entry only attaches the configured
extended gravity model; an unmatched body keeps point-mass gravity.

Matches use the game's body `id` and may also require `parent_id`. Both comparisons are
exact and ordinal. Overlapping matches, unknown properties, unsupported models, and
invalid model values reject the settings catalog at startup rather than silently
changing the intended gravity field.

For example, the complete shipped Earth file is:

```json
{
  "schema_version": 1,
  "match": {
    "id": "Earth",
    "parent_id": "Sol"
  },
  "gravity_model": {
    "model": "spherical_harmonics",
    "name": "Earth J2",
    "normalization": "unnormalized",
    "reference_radius_m": 6378137.0,
    "maximum_degree": 2,
    "coefficients": [
      [2, 0, -0.00108262668, 0]
    ]
  }
}
```

The generic `spherical_harmonics` model stores each coefficient as
`[degree, order, cosine, sine]`. `normalization` is either `unnormalized` or
`fully_normalized`; fully normalized source values are converted once while the
catalog is bound. `maximum_degree` selects the runtime truncation from 2 through
50. The file may retain coefficients above that degree, making it possible to tune
runtime cost without maintaining another copy of the source model. The shipped Luna
file therefore contains all 1,323 GRGM1200A coefficients through degree 50 while
selecting degree 30 by default.

`reference_radius_m` is optional and otherwise comes from the matched game-catalog
body. Body-fixed models always use the rotation captured from the game catalog.

A KSA version other than the release's verified build logs a warning and proceeds
through startup compatibility checks. Concrete API, enum, or patch-activation
incompatibilities still disable the mod's gameplay patches and leave the game
running stock.

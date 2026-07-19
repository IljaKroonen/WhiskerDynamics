# Design

These are the design cornerstones of Whisker Dynamics. Features belong in the
[README](../README.md), and game validation procedures belong in
[game tests](game-tests.md).

## One gravitational truth

While the mod is active, numerical n-body propagation is the only authority for
celestial and vessel gravity. The game continues to own thrust, drag, buoyancy,
collisions, and joints.

Every catalog body has a modeled trajectory. Eligible finite-mass bodies form a
mutually coupled backbone; bodies outside that backbone use restricted,
one-way-coupled trajectories. That approximation is explicit and stable: it is
never selected dynamically to save time.

Stock conics seed the model and mirror its state for compatibility. SOIs support
game bookkeeping and handoffs. Neither conics nor SOIs determine gravity or
provide an alternate propagation path.

## Authority is all or nothing

Activation requires the supported game build, a complete valid catalog, valid
seeds, and authoritative rails covering the current epoch. If those conditions
are not met, the mod does not activate and the game remains stock. It never
silently drops bodies or selects a reduced dynamics model.

After activation, a dynamics failure is session-wide. The mod stops time and
requires a reload instead of mixing mod and stock propagation or continuing
from partially valid state.

## Keep physics independent of the game

The physics and trajectory engine is deterministic, game-independent code. Game
types, patches, persistence, and UI adapters stay outside it. Integration code
translates between the game and the engine; it does not duplicate the engine's
physical rules.

## Presentation derives from simulation

Trajectory lines, event markers, orbit analysis, and plan previews are derived
from propagated states. Reference frames transform how those states are shown;
they never change the underlying dynamics. Display sampling and point budgets
may limit what is visible, but never what is simulated.

Background work uses private or immutable trajectory state and publishes only
complete results for the current session. A stale, partial, or cancelled result
must not become authoritative.

## Preserve stock boundaries without creating a second authority

The stock burn plan remains the execution contract; the mod predicts its effect
without replacing it with a separate executable plan.

Saves retain stock-valid osculating elements, while exact mod state and metadata
live in a sidecar. These stock representations keep saves and game systems
operable, but remain compatibility mirrors while the mod is active.

# iPhysics Debug

Minimal native diagnostics for the circle-only fixed-point physics engine. It
uses the same `eframe`/`egui` stack as the neighboring iShape Rust debug apps.

Run from the repository root:

```sh
cargo run --manifest-path DebugApp/physics_debug/Cargo.toml
```

Use the scenario selector for free fall, an elastic collision, sleeping on a
static support, a small circle pile, and deterministic replay comparison.
Space pauses, `N` advances one tick while paused, and `R` resets the scenario.
The mouse wheel zooms; right or middle drag pans.

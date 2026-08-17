# iPhysics Debug

Minimal native diagnostics for the circle-only fixed-point physics engine. It
uses the same `eframe`/`egui` stack as the neighboring iShape Rust debug apps.

Run from the repository root:

```sh
cargo run --manifest-path DebugApp/physics_debug/Cargo.toml
```

Use the scenario selector for free fall, elastic circle collision, sleeping,
a small circle pile, circle/convex and convex/convex contacts, a multi-part
static playground, and deterministic replay comparison. Space pauses, `N`
advances one tick while paused, and `R` resets the scenario. The mouse wheel
zooms; right or middle drag pans.

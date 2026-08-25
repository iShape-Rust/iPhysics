# iPhysics Debug

Minimal native diagnostics for the fixed-point circle/convex physics engine.
It uses the same `eframe`/`egui` stack as neighboring iShape Rust debug apps.

Run from the repository root:

```sh
cargo run --manifest-path DebugApp/physics_debug/Cargo.toml
```

Use the scenario selector for free fall, elastic circle collision, sleeping,
a small circle pile, a seven-row square pyramid, circle/convex and convex/convex
contacts, a multi-part static playground, a nested-box deep-penetration diagnostic, five
`DistanceJoint`/`RopeJoint` demonstrations (including a gravity-driven harpoon), and
deterministic replay comparison. Joint scenes draw their current world anchors;
rope scenes use a dashed gray line while slack and a solid orange line while
taut. Their sidebar controls reel the target or maximum length in and out.

Space pauses, `N` advances one tick while paused, and `R` resets the scenario.
Left drag moves a dynamic body through a `MouseJoint`; the mouse wheel zooms,
and right or middle drag pans.

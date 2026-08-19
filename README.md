# i_physics

Deterministic fixed-point 2D physics engine for games.

The engine intentionally targets gameplay rather than physically exact
simulation: worlds are bounded, dynamic body counts are expected to be small,
and values outside gameplay limits saturate instead of making a simulation
step fail. Simulation uses integer arithmetic and a fixed 64 Hz tick so the
same initial state and inputs produce the same result on every supported
platform.

## Physical quantity ranges

In this document, `Q16`, `Q24`, and `Q30` mean 16, 24, and 30 fractional bits.
For example, a signed Q16 raw value represents `raw / 2^16`. All ranges are
inclusive unless an upper bound is explicitly marked as exclusive.

| Quantity / Rust type | Storage | Resolution | Enforced range | Rationale |
| --- | ---: | ---: | ---: | --- |
| Body position [`Position`](iPhysics/src/quantity/position.rs) | 2 × `i32`, Q16 | `2^-16 m` = `0.0000152588 m` (about `0.0153 mm`) | Approximately `-8,192 m` to `8,192 m` per axis | The symmetric raw bound `±(2^29 - 1)` guarantees that position sums and differences fit in `i32`. |
| Derived world point [`GeometryPoint`](iPhysics/src/geometry/point.rs) | 2 × `i32`, Q16 | `2^-16 m` | Approximately `-16,384 m` to `16,384 m` per axis | The symmetric raw bound `±(2^30 - 1)` covers translated collider geometry while guaranteeing that the sum or difference of any two points fits in `i32`. |
| Non-negative length [`Length`](iPhysics/src/quantity/length.rs) | `u32`, Q16 | `2^-16 m` | `0` to `16,384 m` exclusive | The raw maximum `2^30 - 1` covers penetration up to the sum of two maximum collider radii. Circle and convex radii are additionally limited to `2^29 - 1` raw units. |
| Linear velocity [`LinearVelocity`](iPhysics/src/quantity/linear_velocity.rs) | 2 × `i32`, Q24 | `2^-24 m/s` = `5.96046e-8 m/s` | `-128` inclusive to `128 m/s` exclusive per component | Uses the full underlying `i32` Q24 range; integration and solver operations use widened intermediates. |
| Linear acceleration [`LinearAcceleration`](iPhysics/src/quantity/linear_acceleration.rs) | 2 × `i32`, Q24 | `2^-24 m/s²` = `5.96046e-8 m/s²` | `-128` inclusive to `128 m/s²` exclusive per component | Uses the full underlying `i32` Q24 range; acceleration-to-velocity integration widens intermediates to `i64`. |
| Orientation [`Angle`](iPhysics/src/quantity/angle.rs) | `u32` binary angle | `2π / 2^32` = `1.46292e-9 rad` (about `8.38e-8°`) | One complete turn, wrapping | Overflow performs exact angle normalization. Quarter, half, and full turns are exact powers of two. Deterministic sine/cosine uses non-expanding integer Q30 CORDIC. |
| Signed angle difference `AngleDelta` | `i32` binary angle | Same as `Angle` | `-π` to `π` exclusive | A subtraction interpreted as `i32` directly yields the shortest wrapped angular difference. |
| Angular velocity [`AngularVelocity`](iPhysics/src/quantity/angular_velocity.rs) | `i32`, Q24 | `2^-24 rad/s` = `5.96046e-8 rad/s` | `-128` inclusive to `128 rad/s` exclusive | Uses the full underlying `i32` Q24 range; conversion to binary angle units and integration use widened intermediates. |
| Angular acceleration [`AngularAcceleration`](iPhysics/src/quantity/angular_acceleration.rs) | `i32`, Q24 | `2^-24 rad/s²` = `5.96046e-8 rad/s²` | `-128` inclusive to `128 rad/s²` exclusive | Uses the full underlying `i32` Q24 range; integration widens intermediates to `i64`. |
| Body mass [`Mass`](iPhysics/src/quantity/mass.rs) | `u32`, Q14 | `2^-14 kg` = `0.0000610352 kg` | `0.0000610352` to `262,143.999939 kg` | Covers the intended approximate range from a `0.1 × 0.1 m` body at `1 kg/m²` (`0.01 kg`) through a `100 × 100 m` body at `10 kg/m²` (`100,000 kg`). Zero is rejected. |
| Restitution [`Material`](iPhysics/src/body/material.rs) | `u32`, Q16 | `2^-16` = `0.0000152588` | `0` to `1` | Dimensionless collision elasticity; values outside the physical gameplay interval are rejected. |
| Simulation time | fixed tick, no stored scalar | `1 / 64 s` = `0.015625 s` | Integer number of ticks | `64` is a power of two, so acceleration-to-velocity and velocity-to-position integration use deterministic shifts rather than division. |

Ranges for vector quantities are specified **per component**. Thus the largest
representable velocity magnitude is approximately `181.019 m/s` near the
corners of the component range. If the design requires a smaller strict
magnitude limit, it should be a separate gameplay invariant rather than an
undocumented side effect of component clamping.

### Effective precision during integration

Storage resolution is not always the same as the smallest value observable in
another quantity after one tick:

| Operation at 64 Hz | Smallest input producing a one-raw-unit output change |
| --- | ---: |
| Velocity → position | `2^-11 m/s` = `0.00048828125 m/s`, producing one Q16 position step |
| Acceleration → velocity | `2^-19 m/s²` = `0.00000190735 m/s²`, producing one Q24 velocity step |
| Angular acceleration → angular velocity | `2^-19 rad/s²` = `0.00000190735 rad/s²` |
| Angular velocity → angle | One Q24 angular-velocity unit already rounds to a non-zero binary-angle step |

Sub-threshold linear velocities remain stored and deterministic but do not
move a body until some input changes them. This is intentional for the target
gameplay scale and also helps small debris settle.

Mass is converted once to unsigned Q24 inverse mass for the solver. Masses up
to approximately `0.00390625 kg` all saturate to the maximum inverse mass;
this remains below the intended minimum gameplay mass of roughly `0.01 kg`.

## Geometry invariants

- Body centers are always bounded `Position` values and saturate at the world
  edge during integration.
- Circle radius must be non-zero and no greater than `2^29 - 1` raw Q16 units.
- Every local convex vertex must be within the same radial limit and a convex
  has between 3 and 6 vertices.
- Integer CORDIC rotation is conservatively non-expanding. Consequently, a
  valid local vertex plus any valid body center fits in the bounded
  `GeometryPoint` range without runtime clamp.
- `Aabb` internally reuses `i_float::IntRect<i32>` while enforcing the same
  bounded Q16 range as `GeometryPoint`.

These bounds are deliberately generous for the expected `0.1–1,000 m`
gameplay scale while keeping common geometry products in `i64`.

## Not implemented yet

Moment of inertia and torque do not currently have a stored physical type.
Angular velocity can be integrated explicitly, but collision impulses do not
yet generate angular response from contact lever arms. A future inertia format
should be selected together with that solver work rather than documenting a
range the engine does not enforce.

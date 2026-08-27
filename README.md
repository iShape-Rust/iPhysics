# i_physics

Deterministic fixed-point 2D physics engine for games.

The engine intentionally targets gameplay rather than physically exact
simulation: worlds are bounded, dynamic body counts are expected to be small,
and values outside gameplay limits saturate instead of making a simulation
step fail. Simulation uses integer arithmetic and a fixed 64 Hz tick so the
same initial state and inputs produce the same result on every supported
platform.

## Physical quantity ranges

In this document, `Qn` means a fixed-point value with `n` fractional bits. For
example, a signed Q16 raw value represents `raw / 2^16`. All ranges are
inclusive unless an upper bound is explicitly marked as exclusive.

### Space

**Body position [`Position`](iPhysics/src/quantity/position.rs)** — two `i32`
components in Q16.

- Resolution: `2^-16 m`, or `0.0000152588 m` (about `0.0153 mm`).
- Raw range per axis: `-(2^29 - 1)..=(2^29 - 1)`.
- Physical range per axis: approximately `-8,192 m..8,192 m`.

The symmetric raw bound guarantees that position sums and differences fit in
`i32`.

**Derived world point
[`GeometryPoint`](iPhysics/src/geometry/point.rs)** — two `i32` components in
Q16.

- Resolution: `2^-16 m`.
- Raw range per axis: `-(2^30 - 1)..=(2^30 - 1)`.
- Physical range per axis: approximately `-16,384 m..16,384 m`.

This wider domain covers translated collider geometry while keeping the sum
or difference of any two points inside `i32`.

**Non-negative [`Length`](iPhysics/src/quantity/length.rs)** — one `u32` in
Q16.

- Resolution: `2^-16 m`.
- Raw range: `0..=2^30 - 1`.
- Physical range: `0 m..16,384 m` (exclusive upper bound).

The range covers penetration up to the sum of two maximum collider radii.
Circle and convex radii are additionally limited to `2^29 - 1` raw units.

### Linear motion

**[`LinearVelocity`](iPhysics/src/quantity/linear_velocity.rs)** — two `i32`
components in Q10.

- Resolution: `2^-10 m/s`, or `0.0009765625 m/s` (`0.9765625 mm/s`).
- Raw range per component: `-(2^20)..=2^20`.
- Physical range per component: `-1,024 m/s..=1,024 m/s`.

The component bound guarantees that the difference of two velocities fits in
`RawVec2` and its projection onto a Q30 contact normal fits in `i32`. The
largest representable vector magnitude is approximately `1,448.155 m/s` near
a corner of the component range. Even under the conservative component-wise
normal bound, relative normal speed is at most `2^22` raw units and the fully
elastic velocity change is at most `2^23` raw units. Collision impulse
magnitudes and inverse-mass weighting use `u64`; signed vector updates use
`i64`.

**[`LinearAcceleration`](iPhysics/src/quantity/linear_acceleration.rs)** — two
`i32` components in Q4.

- Resolution: `2^-4 m/s²`, or `0.0625 m/s²`.
- Raw range per component: `-(2^20)..=2^20`.
- Physical range per component: `-65,536 m/s²..=65,536 m/s²`.

At `64 Hz`, one Q4 acceleration unit changes velocity by exactly one Q10 unit
per tick. The upper bound can therefore move a component from zero to the
maximum velocity in one tick. The raw velocity and acceleration bounds are
identical, and the resulting velocity saturates at its physical limit.

Vector ranges are per component. A smaller strict magnitude limit should be a
separate gameplay invariant rather than a side effect of component clamping.

### Angular motion

**Orientation [`Angle`](iPhysics/src/quantity/angle.rs)** — one `u32` binary
angle covering a complete wrapping turn.

- Resolution: `2π / 2^32`, or approximately `1.46292e-9 rad`
  (`8.38e-8°`).
- Quarter, half, and full turns are exact powers of two.

Overflow performs exact angle normalization. Sine and cosine are calculated
with deterministic, non-expanding integer Q30 CORDIC.

**Signed angle difference `AngleDelta`** — one `i32` binary angle.

- Resolution: the same as `Angle`.
- Range: `-π..π` with an exclusive upper bound.

Interpreting an angle subtraction as `i32` directly produces the shortest
wrapped difference.

**[`AngularVelocity`](iPhysics/src/quantity/angular_velocity.rs)** — one `i32`
in Q16.

- Resolution: `2^-16 rad/s`, approximately `0.0000152588 rad/s`.
- Raw range: `-2^23..=2^23 - 1`.
- Physical range: `-128 rad/s` inclusive to `128 rad/s` exclusive.

**[`AngularAcceleration`](iPhysics/src/quantity/angular_acceleration.rs)** —
one `i32` in Q24.

- Resolution: `2^-24 rad/s²`, approximately `5.96046e-8 rad/s²`.
- Raw range: the full `i32` range.
- Physical range: `-128 rad/s²` inclusive to `128 rad/s²` exclusive.

Conversion and integration use `i64` intermediates. Angular velocity
saturates at its bounded Q16 range.

### Mass and material

**Body [`Mass`](iPhysics/src/quantity/mass.rs)** — one `u32` in Q14.

- Resolution and minimum non-zero value: `2^-14 kg`, or `0.0000610352 kg`.
- Maximum value: `262,143.999939 kg`.

Zero is rejected. The range covers the intended gameplay scale from roughly
`0.01 kg` for a small body through `100,000 kg` for a large body.

Mass is converted once to unsigned Q24 inverse mass for the solver. Masses up
to approximately `0.00390625 kg` saturate to the maximum inverse mass; this is
below the intended minimum gameplay mass of roughly `0.01 kg`.

**[`Force`](iPhysics/src/quantity/force.rs)** — a non-negative force stored as
unsigned Q16 newtons.

- Resolution: `2^-16 N`, or approximately `0.0000153 N`.
- Range: `0 N..65,536 N` (exclusive upper bound).

At 64 Hz, a force limit converts to the mouse-joint impulse limit with an
exact power-of-two scale change: `max_impulse = max_force / 64`.

**Inverse moment of inertia** is derived once from a body's mass and collider
and cached privately on the body as unsigned Q40 in `(kg·m²)⁻¹`.

- Resolution: `2^-40 (kg·m²)⁻¹`, approximately `9.09e-13 (kg·m²)⁻¹`.
- Range: `0..16,777,216 (kg·m²)⁻¹` (exclusive upper bound).

Circles use `I / m = r² / 2`. Convex colliders use the uniform-polygon area
integral about the body origin, so an offset collider automatically includes
the parallel-axis contribution. Values outside the fixed-point range saturate;
zero represents an angularly immovable body at solver precision.

Composite colliders distribute the body's mass between their simple parts in
proportion to part area. Every part contributes its full area, including when
parts overlap, and offset parts include the parallel-axis contribution.

**Material coefficients [`Material`](iPhysics/src/body/material.rs)** —
restitution and Coulomb friction stored as unsigned Q16 values.

- Resolution: `2^-16`, or `0.0000152588` for both coefficients.
- Restitution range: `0..=1`.
- Friction range: `0..65,536` (exclusive upper bound); values greater than one
  are allowed.

Restitution outside its physical interval and negative friction are rejected.
Contact friction uses the arithmetic mean of the two material coefficients.
The velocity solver accumulates normal and tangent impulses across its contact
iterations and clamps the tangent impulse to `|jt| <= friction * jn`.
`Material::INELASTIC` and `Material::ELASTIC` both use friction `0.5`.

### Simulation time and effective precision

Simulation advances at a fixed `64 Hz` tick: `1 / 64 s`, or `0.015625 s`.
The selected linear formats differ by six fractional bits at each stage:

```text
Position Q16 ← LinearVelocity Q10 ← LinearAcceleration Q4
```

Since `64 = 2^6`, semi-implicit linear integration requires no rescaling or
rounding:

```rust
velocity_raw += acceleration_raw;
position_raw += velocity_raw;
```

Consequently, the smallest stored values remain observable across quantities:

- One Q4 acceleration unit (`0.0625 m/s²`) produces one Q10 velocity unit per
  tick.
- One Q10 velocity unit (`0.0009765625 m/s`) produces one Q16 position unit
  per tick.
- Because angular acceleration Q24 is converted to angular velocity Q16 at
  `64 Hz`, the smallest acceleration that rounds to a non-zero velocity change
  per tick is `2^-11 rad/s²`, or `0.00048828125 rad/s²`.
- One Q16 angular-velocity unit already rounds to a non-zero binary-angle
  step.

Every non-zero stored linear velocity moves the body. Small debris still
settles through the explicit sleep thresholds rather than through discarded
sub-position motion.

### Velocity damping

`WorldSettings::linear_damping` and `WorldSettings::angular_damping` specify
the fraction of velocity lost during each fixed `1 / 64 s` tick. A coefficient
of zero preserves velocity, while one removes it completely. Both coefficients
default to approximately `0.001` per tick.

The complementary retention multiplier is stored internally as unsigned Q16.
Damping is applied before gravity and the constraint solvers, with fixed-point
results truncated toward zero so the smallest velocities cannot persist
indefinitely because of rounding.

## Mouse joints

`MouseJoint` pulls a body-local anchor toward a mutable world-space target.
The constraint participates in the iterative velocity solver, accounts for
both mass and rotational inertia, wakes its body, and limits the accumulated
impulse to `max_force / 64` on every tick.

```rust
let body_id = BodyId::new(1);
let pointer = Position::from_meters(2.0, 3.0).unwrap();
let transform = world.body_by_id(body_id).unwrap().transform();
let joint = MouseJoint::at_world_point(
    body_id,
    transform,
    pointer,
    Force::from_newtons(100.0).unwrap(),
);
world.add_mouse_joint(joint).unwrap();

// Before subsequent fixed ticks:
world.mouse_joint_mut(body_id).unwrap().set_target(pointer);

// On pointer release:
world.remove_mouse_joint(body_id);
```

The debug application implements this flow with left-button dragging and
draws the active anchor-to-target constraint.

## Geometry invariants

- Body centers are always bounded `Position` values and saturate at the world
  edge during integration.
- Circle radius must be non-zero. Its body-local center and radius together
  must fit within the `2^29 - 1` raw Q16 collider-radius limit.
- Every local convex vertex must be within the same radial limit and a convex
  has between 3 and 6 vertices.
- A composite collider contains at least one circle or convex. Debug builds
  flag composites above 16 parts because composite-pair narrow phase can grow
  as `O(n × m)`; release builds impose no part-count limit.
- Integer CORDIC rotation is conservatively non-expanding. Consequently, a
  valid local vertex plus any valid body center fits in the bounded
  `GeometryPoint` range without runtime clamp.
- `Aabb` internally reuses `i_float::IntRect<i32>` while enforcing the same
  bounded Q16 range as `GeometryPoint`.

These bounds are deliberately generous for the expected `0.1–1,000 m`
gameplay scale while keeping common geometry products in `i64`.

## Not implemented yet

External torque does not yet have a stored physical type or public force API.
Collision impulses do account for angular contact velocity, moment of inertia,
and contact lever arms.

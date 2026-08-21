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

**[`AngularVelocity`](iPhysics/src/quantity/angular_velocity.rs)** and
**[`AngularAcceleration`](iPhysics/src/quantity/angular_acceleration.rs)** —
one `i32` in Q24 each.

- Resolution: `2^-24 rad/s` or `2^-24 rad/s²`, approximately `5.96046e-8` in
  the corresponding unit.
- Range: `-128` inclusive to `128` exclusive in the corresponding unit.

Both use the full underlying `i32` range. Conversion and integration use
`i64` intermediates.

### Mass and material

**Body [`Mass`](iPhysics/src/quantity/mass.rs)** — one `u32` in Q14.

- Resolution and minimum non-zero value: `2^-14 kg`, or `0.0000610352 kg`.
- Maximum value: `262,143.999939 kg`.

Zero is rejected. The range covers the intended gameplay scale from roughly
`0.01 kg` for a small body through `100,000 kg` for a large body.

Mass is converted once to unsigned Q24 inverse mass for the solver. Masses up
to approximately `0.00390625 kg` saturate to the maximum inverse mass; this is
below the intended minimum gameplay mass of roughly `0.01 kg`.

**Restitution [`Material`](iPhysics/src/body/material.rs)** — one `u32` in
Q16.

- Resolution: `2^-16`, or `0.0000152588`.
- Range: `0..=1`.

Values outside the dimensionless physical interval are rejected.

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
- Angular acceleration to angular velocity: `2^-19 rad/s²`, or
  `0.00000190735 rad/s²`.
- One Q24 angular-velocity unit already rounds to a non-zero binary-angle
  step.

Every non-zero stored linear velocity moves the body. Small debris still
settles through the explicit sleep thresholds rather than through discarded
sub-position motion.

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

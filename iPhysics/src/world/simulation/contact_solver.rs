use super::constraint::{
    MAX_RELATIVE_CONTACT_SPEED_RAW, add_angular_velocity, add_position, add_velocity,
    contact_inverse_mass_q24, contact_lever_cross_axis, relative_speed_along_levers,
    two_bodies_mut,
};
use crate::body::Body;
use crate::world::{ActiveContact, ContactBodyIndex, World};
use crate::{AngularVelocity, UnitVector};
use alloc::vec::Vec;

const POSITION_SLOP_RAW: u32 = 128; // 1/512 m
const MAX_POSITION_CORRECTION_RAW: u32 = 16_384; // 0.25 m
const MAX_VELOCITY_CHANGE_RAW: u64 = 2 * MAX_RELATIVE_CONTACT_SPEED_RAW as u64;
const ANGULAR_RESPONSE_FRACTION_BITS: u32 = 17;
const MAX_ANGULAR_RESPONSE_Q17: u64 = AngularVelocity::MAX_CHANGE << ANGULAR_RESPONSE_FRACTION_BITS;
const LINEAR_RESPONSE_FRACTION_BITS: u32 = 31;
const ONE_LINEAR_RESPONSE_Q31: u64 = 1 << LINEAR_RESPONSE_FRACTION_BITS;
const FRICTION_RESPONSE_FRACTION_BITS: u32 = 31;
// One contact can be visited at most 255 times per step, and each visit can
// change its tangent accumulator by at most one bounded relative speed.
const MAX_TANGENT_ACCUMULATOR_RAW: u64 =
    (u8::MAX as u64 + 1) * MAX_RELATIVE_CONTACT_SPEED_RAW as u64;
const MAX_FRICTION_RESPONSE_Q31: u64 =
    MAX_TANGENT_ACCUMULATOR_RAW << FRICTION_RESPONSE_FRACTION_BITS;

#[derive(Debug, Clone, Copy, Default)]
struct AxisConstraint {
    inverse_sum_q24: u64,
    angular_response_a_q17: i64,
    angular_response_b_q17: i64,
    linear_response_a_q31: u32,
    linear_response_b_q31: u32,
    lever_a_q16: i32,
    lever_b_q16: i32,
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct ContactConstraint {
    // These are impulse numerators: dividing each by the corresponding Q24
    // effective inverse mass yields the scalar contact impulse. Keeping Q10
    // numerators avoids introducing another stored fixed-point format.
    normal_velocity_change_q10: u64,
    tangent_velocity_change_q10: i64,
    normal_target_speed_q10: u32,
    normal: AxisConstraint,
    tangent: AxisConstraint,
    friction_response_q31: u64,
    restitution_q16: u32,
}

pub(super) fn prepare_constraints(world: &World) -> Vec<ContactConstraint> {
    world
        .active_contacts
        .iter()
        .map(|contact| {
            let a = &world.bodies[contact.body_a];
            let material_a = a.material();
            let (b, material_b) = match contact.body_b {
                ContactBodyIndex::Static(static_index) => {
                    (None, world.static_bodies[static_index].material())
                }
                ContactBodyIndex::Dynamic(index_b) => {
                    let b = &world.bodies[index_b];
                    (Some(b), b.material())
                }
            };
            let normal = prepare_axis_constraint(a, b, contact, contact.normal);
            let friction_q16 = material_a.combined_friction_raw(material_b);
            let tangent = if friction_q16 == 0 {
                AxisConstraint::default()
            } else {
                prepare_axis_constraint(a, b, contact, contact.normal.perpendicular())
            };
            let friction_response_q31 = friction_response_q31(
                friction_q16,
                normal.inverse_sum_q24,
                tangent.inverse_sum_q24,
            );
            ContactConstraint {
                normal,
                tangent,
                friction_response_q31,
                restitution_q16: material_a.combined_restitution_raw(material_b),
                ..ContactConstraint::default()
            }
        })
        .collect()
}

fn prepare_axis_constraint(
    a: &Body,
    b: Option<&Body>,
    contact: &ActiveContact,
    axis: UnitVector,
) -> AxisConstraint {
    let lever_a_q16 = contact_lever_cross_axis(a, contact.point, axis);
    let lever_b_q16 = b
        .map(|body| contact_lever_cross_axis(body, contact.point, axis))
        .unwrap_or(0);
    let inverse_sum_q24 = contact_inverse_mass_q24(a, b, lever_a_q16, lever_b_q16);
    if inverse_sum_q24 == 0 {
        return AxisConstraint {
            lever_a_q16,
            lever_b_q16,
            ..AxisConstraint::default()
        };
    }

    AxisConstraint {
        inverse_sum_q24,
        angular_response_a_q17: angular_response_q17(
            a.inverse_inertia_q40(),
            lever_a_q16,
            inverse_sum_q24,
        ),
        angular_response_b_q17: b
            .map(|body| {
                angular_response_q17(body.inverse_inertia_q40(), lever_b_q16, inverse_sum_q24)
            })
            .unwrap_or(0),
        linear_response_a_q31: linear_response_q31(a.inverse_mass_q24(), inverse_sum_q24),
        linear_response_b_q31: b
            .map(|body| linear_response_q31(body.inverse_mass_q24(), inverse_sum_q24))
            .unwrap_or(0),
        lever_a_q16,
        lever_b_q16,
    }
}

pub(super) fn solve_velocities(
    world: &mut World,
    constraints: &mut [ContactConstraint],
    initialize_targets: bool,
) {
    debug_assert_eq!(constraints.len(), world.active_contacts.len());

    for (index, constraint) in constraints.iter_mut().enumerate() {
        solve_velocity(world, index, constraint, initialize_targets);
    }
}

fn solve_velocity(
    world: &mut World,
    index: usize,
    constraint: &mut ContactConstraint,
    initialize_target: bool,
) {
    let contact = world.active_contacts[index];
    match contact.body_b {
        ContactBodyIndex::Static(_) => {
            solve_contact_velocity(
                &mut world.bodies[contact.body_a],
                None,
                &contact,
                constraint,
                initialize_target,
            );
        }
        ContactBodyIndex::Dynamic(index_b) => {
            let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
            solve_contact_velocity(a, Some(b), &contact, constraint, initialize_target);
        }
    }
}

pub(super) fn correct_positions(world: &mut World) {
    for contact in world.active_contacts.iter().copied() {
        if !contact.correct_position {
            continue;
        }
        let correction = contact
            .penetration
            .raw()
            .saturating_sub(POSITION_SLOP_RAW)
            .saturating_mul(4)
            / 5;
        let correction = correction.min(MAX_POSITION_CORRECTION_RAW);
        if correction == 0 {
            continue;
        }

        if let ContactBodyIndex::Static(_) = contact.body_b {
            let [move_x, move_y] = contact.normal.scaled_wide_raw(correction as u64);
            add_position(&mut world.bodies[contact.body_a], -move_x, -move_y);
            continue;
        }

        let ContactBodyIndex::Dynamic(index_b) = contact.body_b else {
            unreachable!()
        };
        let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
        let inverse_a = a.inverse_mass_q24() as u64;
        let inverse_b = b.inverse_mass_q24() as u64;
        let inverse_sum = inverse_a + inverse_b;
        if inverse_sum == 0 {
            continue;
        }

        let move_a = div_round(correction as u64 * inverse_a, inverse_sum);
        let move_b = div_round(correction as u64 * inverse_b, inverse_sum);
        if inverse_a != 0 {
            let [move_x, move_y] = contact.normal.scaled_wide_raw(move_a);
            add_position(a, -move_x, -move_y);
        }
        if inverse_b != 0 {
            let [move_x, move_y] = contact.normal.scaled_wide_raw(move_b);
            add_position(b, move_x, move_y);
        }
    }
}

fn solve_contact_velocity(
    a: &mut Body,
    mut b: Option<&mut Body>,
    contact: &ActiveContact,
    constraint: &mut ContactConstraint,
    initialize_target: bool,
) {
    let normal = contact.normal;
    if constraint.normal.inverse_sum_q24 == 0 {
        return;
    }

    let normal_speed = relative_speed_along_levers(
        a,
        b.as_deref(),
        normal,
        constraint.normal.lever_a_q16,
        constraint.normal.lever_b_q16,
    );
    if initialize_target {
        constraint.normal_target_speed_q10 =
            restitution_target_speed(normal_speed, constraint.restitution_q16) as u32;
    }
    let previous_normal = constraint.normal_velocity_change_q10;
    let candidate_normal =
        previous_normal as i64 + constraint.normal_target_speed_q10 as i64 - normal_speed as i64;
    let accumulated_normal = candidate_normal.max(0) as u64;
    let normal_velocity_change = accumulated_normal as i64 - previous_normal as i64;
    constraint.normal_velocity_change_q10 = accumulated_normal;
    if normal_velocity_change != 0 {
        apply_contact_impulse(
            a,
            b.as_deref_mut(),
            normal,
            normal_velocity_change,
            constraint.normal.linear_response_a_q31,
            constraint.normal.linear_response_b_q31,
            constraint.normal.angular_response_a_q17,
            constraint.normal.angular_response_b_q17,
        );
    }

    if constraint.friction_response_q31 == 0 || constraint.normal_velocity_change_q10 == 0 {
        return;
    }

    let tangent = normal.perpendicular();
    if constraint.tangent.inverse_sum_q24 == 0 {
        return;
    }

    let tangent_speed = relative_speed_along_levers(
        a,
        b.as_deref(),
        tangent,
        constraint.tangent.lever_a_q16,
        constraint.tangent.lever_b_q16,
    );
    let previous = constraint.tangent_velocity_change_q10;
    let candidate = previous.saturating_sub(tangent_speed as i64);
    let limit = friction_velocity_change_limit_q10(
        constraint.normal_velocity_change_q10,
        constraint.friction_response_q31,
    );
    let accumulated = candidate.clamp(-limit, limit);
    let velocity_change = accumulated - previous;
    constraint.tangent_velocity_change_q10 = accumulated;

    apply_contact_impulse(
        a,
        b,
        tangent,
        velocity_change,
        constraint.tangent.linear_response_a_q31,
        constraint.tangent.linear_response_b_q31,
        constraint.tangent.angular_response_a_q17,
        constraint.tangent.angular_response_b_q17,
    );
}

fn apply_contact_impulse(
    a: &mut Body,
    b: Option<&mut Body>,
    mut axis: UnitVector,
    impulse_numerator_q10: i64,
    linear_response_a_q31: u32,
    linear_response_b_q31: u32,
    angular_response_a_q17: i64,
    angular_response_b_q17: i64,
) {
    if impulse_numerator_q10 == 0 {
        return;
    }

    if impulse_numerator_q10 < 0 {
        axis = -axis;
    }
    let magnitude = impulse_numerator_q10.unsigned_abs();
    debug_assert!(magnitude <= MAX_VELOCITY_CHANGE_RAW);
    let change_a = linear_velocity_change_raw(magnitude, linear_response_a_q31);
    if change_a != 0 {
        let [change_x, change_y] = axis.scaled_wide_raw(change_a);
        add_velocity(a, -change_x, -change_y);
    }
    let angular_change_a =
        angular_velocity_change_raw(impulse_numerator_q10, angular_response_a_q17);
    add_angular_velocity(a, -angular_change_a);

    if let Some(body) = b {
        let change_b = linear_velocity_change_raw(magnitude, linear_response_b_q31);
        if change_b != 0 {
            let [change_x, change_y] = axis.scaled_wide_raw(change_b);
            add_velocity(body, change_x, change_y);
        }
        let angular_change_b =
            angular_velocity_change_raw(impulse_numerator_q10, angular_response_b_q17);
        add_angular_velocity(body, angular_change_b);
    }
}

#[inline(always)]
fn linear_response_q31(inverse_mass_q24: u32, inverse_sum_q24: u64) -> u32 {
    if inverse_mass_q24 == 0 {
        return 0;
    }

    debug_assert!(inverse_sum_q24 >= inverse_mass_q24 as u64);
    let numerator = (inverse_mass_q24 as u64) << LINEAR_RESPONSE_FRACTION_BITS;
    div_round(numerator, inverse_sum_q24).min(ONE_LINEAR_RESPONSE_Q31) as u32
}

#[inline(always)]
fn linear_velocity_change_raw(magnitude_q10: u64, linear_response_q31: u32) -> u64 {
    round_shift(
        magnitude_q10 * linear_response_q31 as u64,
        LINEAR_RESPONSE_FRACTION_BITS,
    )
}

#[inline(always)]
fn friction_response_q31(
    friction_q16: u32,
    normal_inverse_sum_q24: u64,
    tangent_inverse_sum_q24: u64,
) -> u64 {
    if friction_q16 == 0 || normal_inverse_sum_q24 == 0 || tangent_inverse_sum_q24 == 0 {
        return 0;
    }

    // friction is Q16, so shifting the numerator by 15 produces a Q31
    // multiplier for normal_velocity_change_q10.
    let numerator = (friction_q16 as u128 * tangent_inverse_sum_q24 as u128) << 15;
    let denominator = normal_inverse_sum_q24 as u128;
    ((numerator + (denominator >> 1)) / denominator).min(MAX_FRICTION_RESPONSE_Q31 as u128) as u64
}

#[inline(always)]
fn friction_velocity_change_limit_q10(
    normal_velocity_change_q10: u64,
    friction_response_q31: u64,
) -> i64 {
    let product = normal_velocity_change_q10 as u128 * friction_response_q31 as u128;
    ((product >> FRICTION_RESPONSE_FRACTION_BITS).min(MAX_TANGENT_ACCUMULATOR_RAW as u128)) as i64
}

#[inline(always)]
fn angular_response_q17(inverse_inertia_q40: u64, lever_q16: i32, inverse_sum_q24: u64) -> i64 {
    if inverse_inertia_q40 == 0 || lever_q16 == 0 {
        return 0;
    }

    // Q17 is the most precise response format whose maximum value multiplied
    // by the maximum Q10 solver impulse is guaranteed to fit u64.
    let numerator = inverse_inertia_q40 as u128 * lever_q16.unsigned_abs() as u128;
    let denominator = (inverse_sum_q24 as u128) << (26 - ANGULAR_RESPONSE_FRACTION_BITS);
    let magnitude = ((numerator + (denominator >> 1)) / denominator)
        .min(MAX_ANGULAR_RESPONSE_Q17 as u128) as i64;
    if lever_q16 < 0 { -magnitude } else { magnitude }
}

#[inline(always)]
fn angular_velocity_change_raw(velocity_change_q10: i64, angular_response_q17: i64) -> i64 {
    if velocity_change_q10 == 0 || angular_response_q17 == 0 {
        return 0;
    }

    let negative = (velocity_change_q10 < 0) ^ (angular_response_q17 < 0);
    let product = velocity_change_q10.unsigned_abs() * angular_response_q17.unsigned_abs();
    let magnitude = round_shift(product, ANGULAR_RESPONSE_FRACTION_BITS)
        .min(AngularVelocity::MAX_CHANGE) as i64;
    if negative { -magnitude } else { magnitude }
}

#[inline(always)]
fn restitution_target_speed(normal_speed: i32, restitution: u32) -> u64 {
    debug_assert!(restitution <= 1 << 16);
    if normal_speed >= 0 {
        return 0;
    }
    let closing_speed = normal_speed.unsigned_abs() as u64;
    let result = round_shift(closing_speed * restitution as u64, 16);
    debug_assert!(result <= MAX_RELATIVE_CONTACT_SPEED_RAW as u64);
    result
}

#[inline(always)]
fn round_shift(value: u64, shift: u32) -> u64 {
    (value + (1_u64 << (shift - 1))) >> shift
}

#[inline(always)]
fn div_round(numerator: u64, denominator: u64) -> u64 {
    debug_assert!(denominator > 0);
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    quotient + u64::from(remainder >= denominator - remainder)
}

#[cfg(test)]
mod tests {
    use super::super::constraint::{relative_normal_speed, relative_speed_along};
    use super::*;
    use crate::body::{BodyId, BodyState, Material, StaticBody};
    use crate::collider::{Circle, Convex};
    use crate::geometry::GeometryPoint;
    use crate::quantity::{
        Angle, AngularVelocity, Damping, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
    use crate::transform::Transform;
    use crate::world::WorldSettings;

    fn circle_body(id: u64, x: f64, velocity: f64, material: Material) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
            Mass::ONE,
            material,
            BodyState::new(
                Transform::new(Position::from_meters(x, 0.0).unwrap(), Angle::ZERO),
                LinearVelocity::from_meters_per_second(velocity, 0.0).unwrap(),
                AngularVelocity::ZERO,
            ),
        )
    }

    fn zero_gravity_world() -> World {
        let mut settings = WorldSettings::new(LinearAcceleration::ZERO);
        settings.linear_damping = Damping::NONE;
        settings.angular_damping = Damping::NONE;
        World::new(settings)
    }

    fn rectangle(half_width: f64, half_height: f64) -> Convex {
        Convex::new(&[
            Position::from_meters(-half_width, -half_height).unwrap(),
            Position::from_meters(half_width, -half_height).unwrap(),
            Position::from_meters(half_width, half_height).unwrap(),
            Position::from_meters(-half_width, half_height).unwrap(),
        ])
        .unwrap()
    }

    fn active_contact(
        body_a: usize,
        body_b: ContactBodyIndex,
        point: GeometryPoint,
        normal: UnitVector,
    ) -> ActiveContact {
        ActiveContact {
            body_a,
            body_b,
            point,
            normal,
            penetration: Length::ZERO,
            correct_position: true,
        }
    }

    #[test]
    fn relative_speed_subtracts_extreme_velocities_without_overflow() {
        let mut a = circle_body(1, 0.0, 0.0, Material::INELASTIC);
        let mut b = circle_body(2, 0.0, 0.0, Material::INELASTIC);
        a.state_mut()
            .set_linear_velocity(LinearVelocity::from_raw(i32::MIN, i32::MIN));
        b.state_mut()
            .set_linear_velocity(LinearVelocity::from_raw(i32::MAX, i32::MAX));
        let point = GeometryPoint::ZERO;
        let normal = UnitVector::from_raw(1 << 30, 1 << 30);

        assert_eq!(
            relative_normal_speed(&a, Some(&b), point, normal),
            MAX_RELATIVE_CONTACT_SPEED_RAW
        );
    }

    #[test]
    fn maximum_solver_impulse_fits_u64_chain() {
        let normal_speed = -MAX_RELATIVE_CONTACT_SPEED_RAW;
        let target = restitution_target_speed(normal_speed, Material::ELASTIC.restitution_raw());
        let impulse = target + normal_speed.unsigned_abs() as u64;
        let inverse_mass = u32::MAX as u64;
        let inverse_sum = 2 * inverse_mass;

        assert_eq!(impulse, MAX_VELOCITY_CHANGE_RAW);
        assert!(impulse <= u32::MAX as u64);
        assert_eq!(div_round(impulse * inverse_mass, inverse_sum), impulse / 2);
    }

    #[test]
    fn cached_linear_response_tracks_division_reference() {
        let magnitudes = [0, 1, 17, 1 << 10, 1 << 20, MAX_VELOCITY_CHANGE_RAW];
        let inverse_masses = [0, 1, 1 << 16, 1 << 24, u32::MAX];

        for magnitude in magnitudes {
            for inverse_mass in inverse_masses {
                let inverse_mass_wide = inverse_mass as u64;
                let inverse_sums = [
                    inverse_mass_wide.max(1),
                    inverse_mass_wide.saturating_add(1),
                    inverse_mass_wide.saturating_mul(2).max(1),
                    1 << 32,
                    1 << 48,
                    u64::MAX,
                ];

                for inverse_sum in inverse_sums {
                    if inverse_sum < inverse_mass_wide {
                        continue;
                    }
                    let expected = div_round(magnitude * inverse_mass_wide, inverse_sum);
                    let response = linear_response_q31(inverse_mass, inverse_sum);
                    let actual = linear_velocity_change_raw(magnitude, response);

                    assert!(
                        actual.abs_diff(expected) <= 1,
                        "expected {expected}, got {actual} for {magnitude}, \
                         {inverse_mass}, {inverse_sum}",
                    );
                }
            }
        }
    }

    #[test]
    fn cached_q17_angular_response_tracks_full_width_reference() {
        assert!(
            MAX_VELOCITY_CHANGE_RAW
                .checked_mul(MAX_ANGULAR_RESPONSE_Q17)
                .is_some()
        );
        let velocity_changes = [
            -(MAX_VELOCITY_CHANGE_RAW as i64),
            -(1 << 10),
            -1,
            1,
            17,
            1 << 10,
            1 << 20,
            MAX_VELOCITY_CHANGE_RAW as i64,
        ];
        let inverse_inertias = [1, 1 << 24, 1 << 40, 6 << 40, u64::MAX];
        let levers = [
            i32::MIN,
            -(1 << 26),
            -1,
            1,
            1 << 11,
            1 << 16,
            1 << 26,
            i32::MAX,
        ];
        let inverse_sums = [1, 1 << 16, 1 << 24, 1 << 32, 1 << 48, u64::MAX];

        for velocity_change in velocity_changes {
            for inverse_inertia in inverse_inertias {
                for lever in levers {
                    for inverse_sum in inverse_sums {
                        let numerator = velocity_change.unsigned_abs() as u128
                            * inverse_inertia as u128
                            * lever.unsigned_abs() as u128;
                        let denominator = (inverse_sum as u128) << 26;
                        let expected_magnitude = ((numerator + (denominator >> 1)) / denominator)
                            .min(AngularVelocity::MAX_CHANGE as u128)
                            as i64;
                        let expected = if (velocity_change < 0) ^ (lever < 0) {
                            -expected_magnitude
                        } else {
                            expected_magnitude
                        };
                        let response = angular_response_q17(inverse_inertia, lever, inverse_sum);
                        let actual = angular_velocity_change_raw(velocity_change, response);

                        assert!(
                            actual.abs_diff(expected) <= 32,
                            "expected {expected}, got {actual} for {velocity_change}, \
                             {inverse_inertia}, {lever}, {inverse_sum}",
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn prepared_contact_constraint_size_is_bounded() {
        assert_eq!(core::mem::size_of::<ContactConstraint>(), 112);
    }

    #[test]
    fn elastic_equal_mass_circles_exchange_velocity() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, -0.5, 1.0, Material::ELASTIC))
            .unwrap();
        world
            .add_body(circle_body(2, 0.5, -1.0, Material::ELASTIC))
            .unwrap();

        let stats = world.step();

        assert_eq!(stats.contacts, 1);
        assert_eq!(
            world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .to_meters_per_second(),
            [-1.0, 0.0]
        );
        assert_eq!(
            world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
                .to_meters_per_second(),
            [1.0, 0.0]
        );
    }

    #[test]
    fn off_center_contact_transfers_linear_momentum_into_spin() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, -0.5, 1.0, Material::INELASTIC))
            .unwrap();
        world
            .add_body(circle_body(2, 0.5, -1.0, Material::INELASTIC))
            .unwrap();
        let contact = active_contact(
            0,
            ContactBodyIndex::Dynamic(1),
            Position::from_meters(0.0, 0.25).unwrap().into(),
            UnitVector::X,
        );
        world.active_contacts.push(contact);

        let mut constraints = prepare_constraints(&world);
        solve_velocities(&mut world, &mut constraints, true);

        let a = world.body(BodyId::new(1)).unwrap();
        let b = world.body(BodyId::new(2)).unwrap();
        assert_eq!(a.state().linear_velocity().raw(), [341, 0]);
        assert_eq!(b.state().linear_velocity().raw(), [-341, 0]);
        assert!(a.state().angular_velocity().raw() > 0);
        assert!(b.state().angular_velocity().raw() < 0);
        assert!(relative_normal_speed(a, Some(b), contact.point, contact.normal).abs() <= 1);
    }

    #[test]
    fn tangent_impulse_stops_sliding_at_a_rough_static_contact() {
        let material = Material::new(0.0, 1.0).unwrap();
        let mut body = circle_body(1, 0.0, 1.0, material);
        body.state_mut()
            .set_linear_velocity(LinearVelocity::from_meters_per_second(1.0, -1.0).unwrap());

        let mut world = zero_gravity_world();
        world.add_body(body).unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::new(Position::from_meters(0.0, -1.0).unwrap(), Angle::ZERO),
                Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
                material,
            ))
            .unwrap();
        let contact = active_contact(
            0,
            ContactBodyIndex::Static(0),
            Position::from_meters(0.0, -0.5).unwrap().into(),
            UnitVector::from_raw(0, -(1 << 30)),
        );
        world.active_contacts.push(contact);

        let mut constraints = prepare_constraints(&world);
        solve_velocities(&mut world, &mut constraints, true);

        let body = world.body(BodyId::new(1)).unwrap();
        assert_eq!(body.state().linear_velocity().raw(), [683, 0]);
        assert!(body.state().angular_velocity().raw() < 0);
        assert!(
            relative_speed_along(body, None, contact.point, contact.normal.perpendicular()).abs()
                <= 1
        );

        let after_first_solve = *body.state();
        solve_velocities(&mut world, &mut constraints, false);
        assert_eq!(
            *world.body(BodyId::new(1)).unwrap().state(),
            after_first_solve
        );
    }

    #[test]
    fn tangent_impulse_affects_both_dynamic_bodies() {
        let material = Material::new(0.0, 1.0).unwrap();
        let mut a = circle_body(1, 0.0, 1.0, material);
        a.state_mut()
            .set_linear_velocity(LinearVelocity::from_meters_per_second(1.0, -1.0).unwrap());
        let mut b = circle_body(2, 0.0, 0.0, material);
        b.state_mut().set_transform(Transform::new(
            Position::from_meters(0.0, -1.0).unwrap(),
            Angle::ZERO,
        ));

        let mut world = zero_gravity_world();
        world.add_body(a).unwrap();
        world.add_body(b).unwrap();
        let contact = active_contact(
            0,
            ContactBodyIndex::Dynamic(1),
            Position::from_meters(0.0, -0.5).unwrap().into(),
            UnitVector::from_raw(0, -(1 << 30)),
        );
        world.active_contacts.push(contact);

        let mut constraints = prepare_constraints(&world);
        solve_velocities(&mut world, &mut constraints, true);

        let a = world.body(BodyId::new(1)).unwrap();
        let b = world.body(BodyId::new(2)).unwrap();
        assert_eq!(a.state().linear_velocity().raw(), [853, -512]);
        assert_eq!(b.state().linear_velocity().raw(), [171, -512]);
        assert!(a.state().angular_velocity().raw() < 0);
        assert!(b.state().angular_velocity().raw() < 0);
        assert!(
            relative_speed_along(a, Some(b), contact.point, contact.normal.perpendicular(),).abs()
                <= 1
        );
    }

    #[test]
    fn coulomb_limit_scales_with_normal_impulse() {
        let inverse_mass = 1_u64 << 24;
        let response = friction_response_q31(1 << 13, inverse_mass, 3 * inverse_mass);

        assert_eq!(friction_velocity_change_limit_q10(1 << 10, response), 384);
    }

    #[test]
    fn cached_friction_response_tracks_full_width_reference() {
        let friction_values = [0, 1, 1 << 13, 1 << 16, u32::MAX];
        let normal_changes = [
            0,
            1,
            17,
            1 << 10,
            1 << 20,
            u8::MAX as u64 * MAX_VELOCITY_CHANGE_RAW,
        ];
        let inverse_sums = [1, 1 << 16, 1 << 24, 1 << 32, 1 << 48, u64::MAX];

        for friction_q16 in friction_values {
            for normal_change in normal_changes {
                for normal_inverse_sum in inverse_sums {
                    for tangent_inverse_sum in inverse_sums {
                        let numerator = friction_q16 as u128
                            * normal_change as u128
                            * tangent_inverse_sum as u128;
                        let denominator = (normal_inverse_sum as u128) << 16;
                        let expected = (numerator / denominator)
                            .min(MAX_TANGENT_ACCUMULATOR_RAW as u128)
                            as i64;
                        let response = friction_response_q31(
                            friction_q16,
                            normal_inverse_sum,
                            tangent_inverse_sum,
                        );
                        let actual = friction_velocity_change_limit_q10(normal_change, response);

                        assert!(
                            actual.abs_diff(expected) <= 1,
                            "expected {expected}, got {actual} for {friction_q16}, \
                             {normal_change}, {normal_inverse_sum}, {tangent_inverse_sum}",
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn aligned_box_does_not_gain_spin_on_a_rough_inclined_plane() {
        let material = Material::new(0.0, 0.8).unwrap();
        let angle = Angle::from_radians(20_f64.to_radians()).unwrap();
        let mut world = World::default();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(1),
                Transform::new(Position::ZERO, angle),
                rectangle(5.0, 0.2),
                material,
            ))
            .unwrap();
        world
            .add_body(Body::dynamic(
                BodyId::new(2),
                rectangle(0.65, 0.4),
                Mass::ONE,
                material,
                BodyState::new(
                    Transform::new(Position::from_meters(0.8, 0.88).unwrap(), angle),
                    LinearVelocity::ZERO,
                    AngularVelocity::ZERO,
                ),
            ))
            .unwrap();

        let first_stats = world.step();
        for _ in 1..128 {
            world.step();
        }

        let body = world.body(BodyId::new(2)).unwrap();
        assert!(
            body.state()
                .angular_velocity()
                .to_radians_per_second()
                .abs()
                < 0.02,
            "unexpected spin: {} rad/s",
            body.state().angular_velocity().to_radians_per_second(),
        );
        assert!(
            (body.state().transform().angle.to_radians() - angle.to_radians()).abs() < 0.01,
            "box rotated away from the plane: {} rad",
            body.state().transform().angle.to_radians(),
        );
        assert_eq!(first_stats.contacts, 2);
    }
}

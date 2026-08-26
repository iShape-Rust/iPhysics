use super::constraint::{
    add_angular_velocity, add_position, add_velocity, contact_inverse_mass_q24,
    contact_lever_cross_axis, relative_speed_along_levers, two_bodies_mut,
    MAX_RELATIVE_CONTACT_SPEED_RAW,
};
use crate::body::Body;
use crate::world::contact_cache::{ContactIdentity, HotContact, HotContacts};
use crate::world::{ActiveContact, ContactBodyIndex, World};
use crate::{AngularVelocity, UnitVector};
use alloc::vec::Vec;

const POSITION_SLOP_RAW: u32 = 128; // 1/512 m
const MAX_POSITION_CORRECTION_RAW: u32 = 16_384; // 0.25 m
const POSITION_PARENT_SCALE_Q30: u64 = 1 << 28; // 0.25
const MAX_VELOCITY_CHANGE_RAW: u64 = 2 * MAX_RELATIVE_CONTACT_SPEED_RAW as u64;
const MAX_ACCUMULATED_NORMAL_RAW: u64 = (u8::MAX as u64 + 1) * MAX_VELOCITY_CHANGE_RAW;
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
const SUPPORT_ALIGNMENT_MIN_Q30: i64 = 1 << 28; // 0.25
const SUPPORT_SCALE_Q30: u64 = 1 << 30;
const SHOCK_STRENGTH_Q30: u64 = 1 << 30; // 1.0
const ANGULAR_SUPPORT_ROOT_Q30: u32 = 1 << 28; // 0.25
const ANGULAR_SUPPORT_PROPAGATION_Q30: u32 = 3 << 28; // 0.75

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

#[derive(Debug, Clone, Copy)]
struct SupportState {
    depth: u16,
    axis: Option<UnitVector>,
    root_score: i64,
}

impl Default for SupportState {
    fn default() -> Self {
        Self {
            depth: u16::MAX,
            axis: None,
            root_score: i64::MIN,
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum ShockParent {
    Static,
    BodyA,
    BodyB,
}

#[derive(Debug, Clone, Copy)]
struct ShockConstraint {
    contact_index: usize,
    depth: u16,
    normal: AxisConstraint,
}

#[derive(Debug, Default)]
pub(super) struct ShockSet {
    constraints: Vec<ShockConstraint>,
    angular_support_q30: Vec<u32>,
    position_parents: Vec<Option<ShockParent>>,
}

impl ShockSet {
    #[inline(always)]
    pub(super) fn is_empty(&self) -> bool {
        self.constraints.is_empty()
    }

    #[inline(always)]
    pub(super) fn len(&self) -> usize {
        self.constraints.len()
    }
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

pub(super) fn prepare_shock_constraints(
    world: &World,
    regular: &[ContactConstraint],
) -> ShockSet {
    debug_assert_eq!(regular.len(), world.active_contacts.len());
    let mut result = ShockSet {
        constraints: Vec::new(),
        angular_support_q30: alloc::vec![0; world.bodies.len()],
        position_parents: alloc::vec![None; world.active_contacts.len()],
    };
    if world.settings.gravity.is_zero() || world.active_contacts.is_empty() {
        return result;
    }

    // Build the current contact graph once, then run a multi-source BFS from
    // every static (or still-sleeping) support. The propagated axis is the
    // normal of the root support, not the normals of intermediate contacts.
    let mut incident = alloc::vec![Vec::new(); world.bodies.len()];
    let mut support = alloc::vec![SupportState::default(); world.bodies.len()];
    let gravity = world.settings.gravity.raw();
    for (contact_index, contact) in world.active_contacts.iter().copied().enumerate() {
        incident[contact.body_a].push(contact_index);
        match contact.body_b {
            ContactBodyIndex::Static(_) => {
                seed_support(&mut support[contact.body_a], -contact.normal, gravity);
            }
            ContactBodyIndex::Dynamic(index_b) => {
                incident[index_b].push(contact_index);
                let sleeping_a = world.bodies[contact.body_a].state().is_sleeping();
                let sleeping_b = world.bodies[index_b].state().is_sleeping();
                if sleeping_a && !sleeping_b {
                    seed_support(&mut support[index_b], contact.normal, gravity);
                } else if sleeping_b && !sleeping_a {
                    seed_support(&mut support[contact.body_a], -contact.normal, gravity);
                }
            }
        }
    }

    let mut queue = Vec::new();
    for (body_index, state) in support.iter().enumerate() {
        if state.depth == 0 {
            queue.push(body_index);
        }
    }
    let mut head = 0;
    while head < queue.len() {
        let parent_index = queue[head];
        head += 1;
        let parent = support[parent_index];
        let Some(parent_axis) = parent.axis else {
            continue;
        };
        for &contact_index in &incident[parent_index] {
            let contact = world.active_contacts[contact_index];
            let ContactBodyIndex::Dynamic(index_b) = contact.body_b else {
                continue;
            };
            let (child_index, normal_on_child) = if contact.body_a == parent_index {
                (index_b, contact.normal)
            } else {
                (contact.body_a, -contact.normal)
            };
            if world.bodies[child_index].state().is_sleeping()
                || axis_dot_q30(normal_on_child, parent_axis) < SUPPORT_ALIGNMENT_MIN_Q30
            {
                continue;
            }
            let child_depth = parent.depth.saturating_add(1);
            if child_depth < support[child_index].depth
                || (child_depth == support[child_index].depth
                    && parent.root_score > support[child_index].root_score)
            {
                support[child_index].depth = child_depth;
                support[child_index].axis = Some(parent_axis);
                support[child_index].root_score = parent.root_score;
                queue.push(child_index);
            }
        }
    }

    result.angular_support_q30 = prepare_angular_support(world, regular, &support, &incident);

    for (contact_index, contact) in world.active_contacts.iter().copied().enumerate() {
        if regular[contact_index].normal_target_speed_q10 != 0 {
            continue;
        }
        let Some((parent, parent_axis, normal_on_child, depth)) =
            shock_parent(world, contact, &support)
        else {
            continue;
        };
        let alignment_q30 = axis_dot_q30(normal_on_child, parent_axis);
        if alignment_q30 < SUPPORT_ALIGNMENT_MIN_Q30 {
            continue;
        }
        let normal = match parent {
            ShockParent::Static => regular[contact_index].normal,
            ShockParent::BodyA | ShockParent::BodyB => prepare_shock_axis_constraint(
                world,
                contact,
                regular[contact_index].normal,
                parent,
                alignment_q30 as u32,
            ),
        };
        if normal.inverse_sum_q24 == 0 {
            continue;
        }
        let child_index = match (parent, contact.body_b) {
            (ShockParent::Static, _) => contact.body_a,
            (ShockParent::BodyA, ContactBodyIndex::Dynamic(index_b)) => index_b,
            (ShockParent::BodyB, ContactBodyIndex::Dynamic(_)) => contact.body_a,
            _ => continue,
        };
        if result.angular_support_q30[child_index] != 0 {
            result.position_parents[contact_index] = Some(parent);
        }
        result.constraints.push(ShockConstraint {
            contact_index,
            depth,
            normal,
        });
    }
    result
        .constraints
        .sort_unstable_by_key(|constraint| (constraint.depth, constraint.contact_index));
    result
}

fn seed_support(state: &mut SupportState, axis: UnitVector, gravity: [i32; 2]) {
    let [axis_x, axis_y] = axis.raw();
    let score = -(axis_x as i64 * gravity[0] as i64 + axis_y as i64 * gravity[1] as i64);
    if state.depth != 0 || score > state.root_score {
        state.depth = 0;
        state.axis = Some(axis);
        state.root_score = score;
    }
}

fn shock_parent(
    world: &World,
    contact: ActiveContact,
    support: &[SupportState],
) -> Option<(ShockParent, UnitVector, UnitVector, u16)> {
    match contact.body_b {
        ContactBodyIndex::Static(_) => {
            let state = support[contact.body_a];
            Some((ShockParent::Static, state.axis?, -contact.normal, 0))
        }
        ContactBodyIndex::Dynamic(index_b) => {
            let state_a = support[contact.body_a];
            let state_b = support[index_b];
            let sleeping_a = world.bodies[contact.body_a].state().is_sleeping();
            let sleeping_b = world.bodies[index_b].state().is_sleeping();
            if sleeping_a && !sleeping_b {
                return Some((ShockParent::BodyA, state_b.axis?, contact.normal, 0));
            }
            if sleeping_b && !sleeping_a {
                return Some((ShockParent::BodyB, state_a.axis?, -contact.normal, 0));
            }
            if state_a.depth < state_b.depth {
                Some((
                    ShockParent::BodyA,
                    state_a.axis?,
                    contact.normal,
                    state_b.depth,
                ))
            } else if state_b.depth < state_a.depth {
                Some((
                    ShockParent::BodyB,
                    state_b.axis?,
                    -contact.normal,
                    state_a.depth,
                ))
            } else {
                None
            }
        }
    }
}

fn prepare_angular_support(
    world: &World,
    regular: &[ContactConstraint],
    support: &[SupportState],
    incident: &[Vec<usize>],
) -> Vec<u32> {
    let mut angular_support = world
        .bodies
        .iter()
        .map(|body| {
            if body.state().is_sleeping() {
                SUPPORT_SCALE_Q30 as u32
            } else {
                0
            }
        })
        .collect::<Vec<_>>();
    let mut order = support
        .iter()
        .enumerate()
        .filter_map(|(body_index, state)| {
            (state.depth != u16::MAX).then_some((state.depth, body_index))
        })
        .collect::<Vec<_>>();
    order.sort_unstable();

    for (_, body_index) in order {
        if world.bodies[body_index].state().is_sleeping() {
            continue;
        }
        let Some(support_axis) = support[body_index].axis else {
            continue;
        };
        let mut negative_strength = 0;
        let mut positive_strength = 0;
        for &contact_index in &incident[body_index] {
            if regular[contact_index].normal_target_speed_q10 != 0 {
                continue;
            }
            let contact = world.active_contacts[contact_index];
            let Some((parent, _, normal_on_child, _)) =
                shock_parent(world, contact, support)
            else {
                continue;
            };
            let child_index = match (parent, contact.body_b) {
                (ShockParent::Static, _) => contact.body_a,
                (ShockParent::BodyA, ContactBodyIndex::Dynamic(index_b)) => index_b,
                (ShockParent::BodyB, ContactBodyIndex::Dynamic(_)) => contact.body_a,
                _ => continue,
            };
            if child_index != body_index {
                continue;
            }
            let alignment_q30 = axis_dot_q30(normal_on_child, support_axis);
            if alignment_q30 < SUPPORT_ALIGNMENT_MIN_Q30 {
                continue;
            }
            let (parent_strength, propagate) = match parent {
                ShockParent::Static => (ANGULAR_SUPPORT_ROOT_Q30, false),
                ShockParent::BodyA => {
                    let parent_index = contact.body_a;
                    (
                        angular_support[parent_index],
                        !world.bodies[parent_index].state().is_sleeping(),
                    )
                }
                ShockParent::BodyB => {
                    let ContactBodyIndex::Dynamic(parent_index) = contact.body_b else {
                        continue;
                    };
                    (
                        angular_support[parent_index],
                        !world.bodies[parent_index].state().is_sleeping(),
                    )
                }
            };
            if parent_strength == 0 {
                continue;
            }
            let alignment_squared_q30 = square_q30(alignment_q30 as u32);
            let mut strength = multiply_q30(parent_strength, alignment_squared_q30);
            if propagate {
                strength = multiply_q30(strength, ANGULAR_SUPPORT_PROPAGATION_Q30);
            }
            let tangent_offset_q16 =
                contact_lever_cross_axis(&world.bodies[body_index], contact.point, support_axis);
            if tangent_offset_q16 <= -(POSITION_SLOP_RAW as i32) {
                negative_strength = negative_strength.max(strength);
            } else if tangent_offset_q16 >= POSITION_SLOP_RAW as i32 {
                positive_strength = positive_strength.max(strength);
            }
        }
        angular_support[body_index] = negative_strength.min(positive_strength);
    }

    angular_support
}

fn prepare_shock_axis_constraint(
    world: &World,
    contact: ActiveContact,
    regular: AxisConstraint,
    parent: ShockParent,
    alignment_q30: u32,
) -> AxisConstraint {
    let a = &world.bodies[contact.body_a];
    let b = match contact.body_b {
        ContactBodyIndex::Dynamic(index_b) => Some(&world.bodies[index_b]),
        ContactBodyIndex::Static(_) => None,
    };
    let alignment_squared_q30 = square_q30(alignment_q30) as u64;
    let shock_q30 =
        (alignment_squared_q30 * SHOCK_STRENGTH_Q30 + (1 << 29)) >> 30;
    let retention_q30 = SUPPORT_SCALE_Q30 - shock_q30;
    // Only the parent becomes heavier. With a fully aligned support normal its
    // inverse mass and inertia reach zero, so the correction travels upward.
    let scale_a = if matches!(parent, ShockParent::BodyA) {
        retention_q30
    } else {
        SUPPORT_SCALE_Q30
    };
    let scale_b = if matches!(parent, ShockParent::BodyB) {
        retention_q30
    } else {
        SUPPORT_SCALE_Q30
    };
    let inverse_mass_a = scale_u32_q30(a.inverse_mass_q24(), scale_a);
    let inverse_inertia_a = scale_u64_q30(a.inverse_inertia_q40(), scale_a);
    let inverse_mass_b = b
        .map(|body| scale_u32_q30(body.inverse_mass_q24(), scale_b))
        .unwrap_or(0);
    let inverse_inertia_b = b
        .map(|body| scale_u64_q30(body.inverse_inertia_q40(), scale_b))
        .unwrap_or(0);
    let inverse_sum_q24 = inverse_mass_a as u64
        + inverse_mass_b as u64
        + rotational_inverse_mass_q24(regular.lever_a_q16, inverse_inertia_a)
        + rotational_inverse_mass_q24(regular.lever_b_q16, inverse_inertia_b);
    if inverse_sum_q24 == 0 {
        return AxisConstraint {
            lever_a_q16: regular.lever_a_q16,
            lever_b_q16: regular.lever_b_q16,
            ..AxisConstraint::default()
        };
    }
    AxisConstraint {
        inverse_sum_q24,
        angular_response_a_q17: angular_response_q17(
            inverse_inertia_a,
            regular.lever_a_q16,
            inverse_sum_q24,
        ),
        angular_response_b_q17: angular_response_q17(
            inverse_inertia_b,
            regular.lever_b_q16,
            inverse_sum_q24,
        ),
        linear_response_a_q31: linear_response_q31(inverse_mass_a, inverse_sum_q24),
        linear_response_b_q31: linear_response_q31(inverse_mass_b, inverse_sum_q24),
        lever_a_q16: regular.lever_a_q16,
        lever_b_q16: regular.lever_b_q16,
    }
}

#[inline(always)]
fn square_q30(value_q30: u32) -> u32 {
    ((value_q30 as u64 * value_q30 as u64 + (1 << 29)) >> 30)
        .min(SUPPORT_SCALE_Q30) as u32
}

#[inline(always)]
fn multiply_q30(a_q30: u32, b_q30: u32) -> u32 {
    ((a_q30 as u64 * b_q30 as u64 + (1 << 29)) >> 30)
        .min(SUPPORT_SCALE_Q30) as u32
}

#[inline(always)]
fn axis_dot_q30(a: UnitVector, b: UnitVector) -> i64 {
    let [ax, ay] = a.raw();
    let [bx, by] = b.raw();
    (ax as i64 * bx as i64 + ay as i64 * by as i64 + (1 << 29)) >> 30
}

#[inline(always)]
fn scale_u32_q30(value: u32, scale_q30: u64) -> u32 {
    ((value as u64 * scale_q30 + (1 << 29)) >> 30).min(u32::MAX as u64) as u32
}

#[inline(always)]
fn scale_u64_q30(value: u64, scale_q30: u64) -> u64 {
    (((value as u128 * scale_q30 as u128) + (1 << 29)) >> 30).min(u64::MAX as u128) as u64
}

#[inline(always)]
fn scale_signed_i32_q30(value: i32, scale_q30: u32) -> i32 {
    let magnitude = ((value.unsigned_abs() as u64 * scale_q30 as u64 + (1 << 29)) >> 30) as i32;
    if value < 0 {
        -magnitude
    } else {
        magnitude
    }
}

#[inline(always)]
fn rotational_inverse_mass_q24(lever_q16: i32, inverse_inertia_q40: u64) -> u64 {
    let lever = lever_q16.unsigned_abs() as u128;
    let product = lever * lever * inverse_inertia_q40 as u128;
    ((product + (1_u128 << 47)) >> 48).min(u64::MAX as u128) as u64
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
    if initialize_targets {
        initialize_restitution_targets(world, constraints);
    }

    for (index, constraint) in constraints.iter_mut().enumerate() {
        solve_velocity(world, index, constraint);
    }
}

pub(super) fn solve_shock_velocities(
    world: &mut World,
    constraints: &mut [ContactConstraint],
    shock: &ShockSet,
    accumulators: &mut [u64],
) {
    debug_assert_eq!(constraints.len(), world.active_contacts.len());
    debug_assert_eq!(accumulators.len(), shock.constraints.len());
    for (shock_index, shock_constraint) in shock.constraints.iter().copied().enumerate() {
        let contact_index = shock_constraint.contact_index;
        solve_shock_velocity(
            world,
            contact_index,
            shock_constraint.normal,
            &mut accumulators[shock_index],
        );
        solve_friction_velocity(world, contact_index, &mut constraints[contact_index]);
    }
}

pub(super) fn apply_angular_support(world: &mut World, shock: &ShockSet) {
    debug_assert_eq!(shock.angular_support_q30.len(), world.bodies.len());
    for (body, support_q30) in world
        .bodies
        .iter_mut()
        .zip(shock.angular_support_q30.iter().copied())
    {
        if support_q30 == 0 || body.state().is_sleeping() {
            continue;
        }
        let retention_q30 = SUPPORT_SCALE_Q30 as u32 - support_q30;
        let angular_velocity = body.state().angular_velocity().raw();
        let retained = scale_signed_i32_q30(angular_velocity, retention_q30);
        body.state_mut().angular_velocity = AngularVelocity::from_raw(retained);
    }
}

pub(super) fn prepare_warm_start(world: &mut World, constraints: &mut [ContactConstraint]) {
    debug_assert_eq!(constraints.len(), world.active_contacts.len());
    initialize_restitution_targets(world, constraints);

    for (index, constraint) in constraints.iter_mut().enumerate() {
        if constraint.normal_target_speed_q10 != 0 {
            continue;
        }
        let contact = world.active_contacts[index];
        let Some(identity) = contact_identity(world, contact) else {
            continue;
        };
        let cached = world.hot_contacts[contact.body_a]
            .find(identity)
            .or_else(|| match contact.body_b {
                ContactBodyIndex::Dynamic(index_b) => world.hot_contacts[index_b].find(identity),
                ContactBodyIndex::Static(_) => None,
            });
        if let Some(cached) = cached {
            constraint.normal_velocity_change_q10 = cached.normal_velocity_change_q10;
            let tangent_limit = friction_velocity_change_limit_q10(
                cached.normal_velocity_change_q10,
                constraint.friction_response_q31,
            );
            constraint.tangent_velocity_change_q10 = cached
                .tangent_velocity_change_q10
                .clamp(-tangent_limit, tangent_limit);
        }
    }

    for (index, constraint) in constraints.iter().enumerate() {
        apply_cached_velocity(world, index, constraint);
    }
}

pub(super) fn rebuild_contact_cache(world: &mut World, constraints: &[ContactConstraint]) {
    debug_assert_eq!(constraints.len(), world.active_contacts.len());
    let mut next = alloc::vec![HotContacts::EMPTY; world.bodies.len()];
    for (index, constraint) in constraints.iter().copied().enumerate() {
        if constraint.normal_target_speed_q10 != 0 || constraint.normal_velocity_change_q10 == 0 {
            continue;
        }
        let contact = world.active_contacts[index];
        let Some(identity) = contact_identity(world, contact) else {
            continue;
        };
        let hot = HotContact {
            identity,
            normal_velocity_change_q10: constraint.normal_velocity_change_q10,
            tangent_velocity_change_q10: constraint.tangent_velocity_change_q10,
        };
        next[contact.body_a].insert(hot);
        if let ContactBodyIndex::Dynamic(index_b) = contact.body_b {
            next[index_b].insert(hot);
        }
    }
    world.hot_contacts = next;
}

fn initialize_restitution_targets(world: &World, constraints: &mut [ContactConstraint]) {
    for (index, constraint) in constraints.iter_mut().enumerate() {
        let contact = world.active_contacts[index];
        let a = &world.bodies[contact.body_a];
        let b = match contact.body_b {
            ContactBodyIndex::Dynamic(index_b) => Some(&world.bodies[index_b]),
            ContactBodyIndex::Static(_) => None,
        };
        let normal_speed = relative_speed_along_levers(
            a,
            b,
            contact.normal,
            constraint.normal.lever_a_q16,
            constraint.normal.lever_b_q16,
        );
        constraint.normal_target_speed_q10 =
            restitution_target_speed(normal_speed, constraint.restitution_q16) as u32;
    }
}

fn contact_identity(world: &World, contact: ActiveContact) -> Option<ContactIdentity> {
    let key = contact.key.cache_key()?;
    let body_b = match contact.body_b {
        ContactBodyIndex::Dynamic(index_b) => world.bodies[index_b].id(),
        ContactBodyIndex::Static(index_b) => world.static_bodies[index_b].id(),
    };
    Some(ContactIdentity {
        body_a: world.bodies[contact.body_a].id(),
        body_b,
        key,
    })
}

fn apply_cached_velocity(world: &mut World, index: usize, constraint: &ContactConstraint) {
    if constraint.normal_velocity_change_q10 == 0 && constraint.tangent_velocity_change_q10 == 0 {
        return;
    }
    let contact = world.active_contacts[index];
    match contact.body_b {
        ContactBodyIndex::Static(_) => apply_cached_contact_velocity(
            &mut world.bodies[contact.body_a],
            None,
            &contact,
            constraint,
        ),
        ContactBodyIndex::Dynamic(index_b) => {
            let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
            apply_cached_contact_velocity(a, Some(b), &contact, constraint);
        }
    }
}

fn apply_cached_contact_velocity(
    a: &mut Body,
    mut b: Option<&mut Body>,
    contact: &ActiveContact,
    constraint: &ContactConstraint,
) {
    apply_contact_impulse(
        a,
        b.as_deref_mut(),
        contact.normal,
        constraint.normal_velocity_change_q10 as i64,
        constraint.normal.linear_response_a_q31,
        constraint.normal.linear_response_b_q31,
        constraint.normal.angular_response_a_q17,
        constraint.normal.angular_response_b_q17,
    );
    apply_contact_impulse(
        a,
        b,
        contact.normal.perpendicular(),
        constraint.tangent_velocity_change_q10,
        constraint.tangent.linear_response_a_q31,
        constraint.tangent.linear_response_b_q31,
        constraint.tangent.angular_response_a_q17,
        constraint.tangent.angular_response_b_q17,
    );
}

fn solve_velocity(world: &mut World, index: usize, constraint: &mut ContactConstraint) {
    let contact = world.active_contacts[index];
    match contact.body_b {
        ContactBodyIndex::Static(_) => {
            solve_contact_velocity(
                &mut world.bodies[contact.body_a],
                None,
                &contact,
                constraint,
            );
        }
        ContactBodyIndex::Dynamic(index_b) => {
            let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
            solve_contact_velocity(a, Some(b), &contact, constraint);
        }
    }
}

fn solve_shock_velocity(
    world: &mut World,
    index: usize,
    normal_constraint: AxisConstraint,
    accumulator: &mut u64,
) {
    let contact = world.active_contacts[index];
    match contact.body_b {
        ContactBodyIndex::Static(_) => solve_shock_contact_velocity(
            &mut world.bodies[contact.body_a],
            None,
            &contact,
            normal_constraint,
            accumulator,
        ),
        ContactBodyIndex::Dynamic(index_b) => {
            let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
            solve_shock_contact_velocity(
                a,
                Some(b),
                &contact,
                normal_constraint,
                accumulator,
            );
        }
    }
}

fn solve_friction_velocity(
    world: &mut World,
    index: usize,
    constraint: &mut ContactConstraint,
) {
    let contact = world.active_contacts[index];
    match contact.body_b {
        ContactBodyIndex::Static(_) => solve_contact_friction(
            &mut world.bodies[contact.body_a],
            None,
            &contact,
            constraint,
        ),
        ContactBodyIndex::Dynamic(index_b) => {
            let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
            solve_contact_friction(a, Some(b), &contact, constraint);
        }
    }
}

pub(super) fn correct_positions(world: &mut World, shock: &ShockSet) {
    debug_assert_eq!(shock.position_parents.len(), world.active_contacts.len());
    // Supported contacts are already ordered by BFS depth, so positional
    // correction also travels from the static field toward the island edge.
    for constraint in &shock.constraints {
        let contact_index = constraint.contact_index;
        let Some(parent) = shock.position_parents[contact_index] else {
            continue;
        };
        correct_supported_position(world, world.active_contacts[contact_index], parent);
    }
    for contact_index in 0..world.active_contacts.len() {
        if shock.position_parents[contact_index].is_none() {
            let contact = world.active_contacts[contact_index];
            correct_symmetric_position(world, contact);
        }
    }
}

fn position_correction(contact: ActiveContact) -> u32 {
    if !contact.key.correct_position() {
        return 0;
    }
    (contact
        .penetration
        .raw()
        .saturating_sub(POSITION_SLOP_RAW)
        .saturating_mul(4)
        / 5)
        .min(MAX_POSITION_CORRECTION_RAW)
}

fn correct_supported_position(world: &mut World, contact: ActiveContact, parent: ShockParent) {
    let correction = position_correction(contact);
    if correction == 0 {
        return;
    }
    if matches!(parent, ShockParent::Static) {
        let [move_x, move_y] = contact.normal.scaled_wide_raw(correction as u64);
        add_position(&mut world.bodies[contact.body_a], -move_x, -move_y);
        return;
    }
    let ContactBodyIndex::Dynamic(index_b) = contact.body_b else {
        unreachable!()
    };
    let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
    let inverse_a = if matches!(parent, ShockParent::BodyA) {
        scale_u32_q30(a.inverse_mass_q24(), POSITION_PARENT_SCALE_Q30) as u64
    } else {
        a.inverse_mass_q24() as u64
    };
    let inverse_b = if matches!(parent, ShockParent::BodyB) {
        scale_u32_q30(b.inverse_mass_q24(), POSITION_PARENT_SCALE_Q30) as u64
    } else {
        b.inverse_mass_q24() as u64
    };
    apply_position_correction(a, b, contact.normal, correction, inverse_a, inverse_b);
}

fn correct_symmetric_position(world: &mut World, contact: ActiveContact) {
    let correction = position_correction(contact);
    if correction == 0 {
        return;
    }
    if let ContactBodyIndex::Static(_) = contact.body_b {
        let [move_x, move_y] = contact.normal.scaled_wide_raw(correction as u64);
        add_position(&mut world.bodies[contact.body_a], -move_x, -move_y);
        return;
    }
    let ContactBodyIndex::Dynamic(index_b) = contact.body_b else {
        unreachable!()
    };
    let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
    apply_position_correction(
        a,
        b,
        contact.normal,
        correction,
        a.inverse_mass_q24() as u64,
        b.inverse_mass_q24() as u64,
    );
}

fn apply_position_correction(
    a: &mut Body,
    b: &mut Body,
    normal: UnitVector,
    correction: u32,
    inverse_a: u64,
    inverse_b: u64,
) {
    let inverse_sum = inverse_a + inverse_b;
    if inverse_sum == 0 {
        return;
    }
    let move_a = div_round(correction as u64 * inverse_a, inverse_sum);
    let move_b = div_round(correction as u64 * inverse_b, inverse_sum);
    if inverse_a != 0 {
        let [move_x, move_y] = normal.scaled_wide_raw(move_a);
        add_position(a, -move_x, -move_y);
    }
    if inverse_b != 0 {
        let [move_x, move_y] = normal.scaled_wide_raw(move_b);
        add_position(b, move_x, move_y);
    }
}

fn solve_contact_velocity(
    a: &mut Body,
    mut b: Option<&mut Body>,
    contact: &ActiveContact,
    constraint: &mut ContactConstraint,
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

    solve_contact_friction(a, b, contact, constraint);
}

fn solve_shock_contact_velocity(
    a: &mut Body,
    b: Option<&mut Body>,
    contact: &ActiveContact,
    normal_constraint: AxisConstraint,
    accumulator: &mut u64,
) {
    let normal_speed = relative_speed_along_levers(
        a,
        b.as_deref(),
        contact.normal,
        normal_constraint.lever_a_q16,
        normal_constraint.lever_b_q16,
    );
    let previous = *accumulator;
    let candidate = previous as i64 - normal_speed as i64;
    let accumulated = candidate.max(0) as u64;
    let velocity_change = accumulated as i64 - previous as i64;
    *accumulator = accumulated;
    apply_contact_impulse(
        a,
        b,
        contact.normal,
        velocity_change,
        normal_constraint.linear_response_a_q31,
        normal_constraint.linear_response_b_q31,
        normal_constraint.angular_response_a_q17,
        normal_constraint.angular_response_b_q17,
    );
}

fn solve_contact_friction(
    a: &mut Body,
    b: Option<&mut Body>,
    contact: &ActiveContact,
    constraint: &mut ContactConstraint,
) {
    let normal = contact.normal;

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
    debug_assert!(magnitude <= MAX_ACCUMULATED_NORMAL_RAW.max(MAX_TANGENT_ACCUMULATOR_RAW));
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
    if lever_q16 < 0 {
        -magnitude
    } else {
        magnitude
    }
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
    if negative {
        -magnitude
    } else {
        magnitude
    }
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
            key: crate::collision::ContactKey::new(
                crate::collision::ColliderFeature::Circle,
                crate::collision::ColliderFeature::Circle,
            )
            .with_correct_position(true),
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
        assert!(MAX_VELOCITY_CHANGE_RAW
            .checked_mul(MAX_ANGULAR_RESPONSE_Q17)
            .is_some());
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
        assert_eq!(world.hot_contacts[0].len(), 0);
        assert_eq!(world.hot_contacts[1].len(), 0);
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
    fn final_tangent_accumulator_is_persisted() {
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

        world.step();

        assert_eq!(world.hot_contacts[0].len(), 1);
        assert!(world.hot_contacts[0].has_tangent());
    }

    #[test]
    fn convex_stack_is_stable_with_one_velocity_iteration() {
        let settings = WorldSettings {
            velocity_iterations: 1,
            linear_damping: Damping::NONE,
            angular_damping: Damping::NONE,
            sleep: crate::body::SleepConfig::from_raw(0, 0, u8::MAX),
            ..WorldSettings::default()
        };
        let mut world = World::new(settings);
        for (id, y) in [(1, 0.5), (2, 1.5), (3, 2.5), (4, 3.5)] {
            world
                .add_body(Body::dynamic(
                    BodyId::new(id),
                    rectangle(0.5, 0.5),
                    Mass::ONE,
                    Material::INELASTIC,
                    BodyState::new(
                        Transform::new(Position::from_meters(0.0, y).unwrap(), Angle::ZERO),
                        LinearVelocity::ZERO,
                        AngularVelocity::ZERO,
                    ),
                ))
                .unwrap();
        }
        world
            .add_static_body(StaticBody::new(
                BodyId::new(100),
                Transform::new(Position::from_meters(0.0, -0.5).unwrap(), Angle::ZERO),
                rectangle(5.0, 0.5),
                Material::INELASTIC,
            ))
            .unwrap();

        for _ in 0..192 {
            world.step();
        }

        for (index, body) in world.bodies().iter().enumerate() {
            let expected_y = index as f64 + 0.5;
            let actual_y = body.state().transform().position.to_meters()[1];
            let vertical_speed = body.state().linear_velocity().to_meters_per_second()[1].abs();
            assert!(
                (actual_y - expected_y).abs() < 0.06,
                "body {index}: y={actual_y}"
            );
            assert!(vertical_speed < 0.5, "body {index}: vy={vertical_speed}");
        }
    }

    #[test]
    fn shock_constraints_follow_support_depth_and_freeze_the_parent_response() {
        let mut world = World::default();
        for id in 1..=3 {
            world
                .add_body(circle_body(id, 0.0, 0.0, Material::INELASTIC))
                .unwrap();
        }
        world
            .add_static_body(StaticBody::new(
                BodyId::new(100),
                Transform::default(),
                Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
                Material::INELASTIC,
            ))
            .unwrap();

        let down = UnitVector::from_raw(0, -(1 << 30));
        world.active_contacts.push(active_contact(
            0,
            ContactBodyIndex::Static(0),
            GeometryPoint::ZERO,
            down,
        ));
        world.active_contacts.push(active_contact(
            1,
            ContactBodyIndex::Dynamic(0),
            GeometryPoint::ZERO,
            down,
        ));
        world.active_contacts.push(active_contact(
            2,
            ContactBodyIndex::Dynamic(1),
            GeometryPoint::ZERO,
            down,
        ));

        let regular = prepare_constraints(&world);
        let shock = prepare_shock_constraints(&world, &regular);

        assert_eq!(shock.constraints.len(), 3);
        assert_eq!(
            shock
                .constraints
                .iter()
                .map(|constraint| (constraint.contact_index, constraint.depth))
                .collect::<Vec<_>>(),
            [(0, 0), (1, 1), (2, 2)]
        );
        for constraint in &shock.constraints[1..] {
            assert_eq!(constraint.normal.linear_response_a_q31, ONE_LINEAR_RESPONSE_Q31 as u32);
            assert_eq!(constraint.normal.linear_response_b_q31, 0);
            assert_eq!(constraint.normal.angular_response_b_q17, 0);
        }
        assert_eq!(shock.angular_support_q30, [0, 0, 0]);

    }

    #[test]
    fn angular_support_requires_two_sided_contacts_and_attenuates_upward() {
        let mut world = World::default();
        for id in 1..=3 {
            world
                .add_body(circle_body(id, 0.0, 0.0, Material::INELASTIC))
                .unwrap();
        }
        world
            .add_static_body(StaticBody::new(
                BodyId::new(100),
                Transform::default(),
                Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
                Material::INELASTIC,
            ))
            .unwrap();

        let down = UnitVector::from_raw(0, -(1 << 30));
        let contact_points = [
            Position::from_meters(-0.25, 0.0).unwrap().into(),
            Position::from_meters(0.25, 0.0).unwrap().into(),
        ];
        for point in contact_points {
            world.active_contacts.push(active_contact(
                0,
                ContactBodyIndex::Static(0),
                point,
                down,
            ));
            world.active_contacts.push(active_contact(
                1,
                ContactBodyIndex::Dynamic(0),
                point,
                down,
            ));
            world.active_contacts.push(active_contact(
                2,
                ContactBodyIndex::Dynamic(1),
                point,
                down,
            ));
        }

        let regular = prepare_constraints(&world);
        let shock = prepare_shock_constraints(&world, &regular);

        assert_eq!(
            shock.angular_support_q30,
            [1 << 28, 3 << 26, 9 << 24]
        );
        for body in &mut world.bodies {
            body.state_mut().angular_velocity =
                AngularVelocity::from_radians_per_second(1.0).unwrap();
        }
        apply_angular_support(&mut world, &shock);
        assert_eq!(world.bodies[0].state().angular_velocity().raw(), 3 << 14);
        assert_eq!(world.bodies[1].state().angular_velocity().raw(), 13 << 12);
        assert_eq!(world.bodies[2].state().angular_velocity().raw(), 55 << 10);
    }

    #[test]
    fn supported_position_correction_moves_the_parent_four_times_less() {
        let mut world = World::default();
        world
            .add_body(circle_body(1, 0.0, 0.0, Material::INELASTIC))
            .unwrap();
        world
            .add_body(circle_body(2, 0.0, 0.0, Material::INELASTIC))
            .unwrap();
        let mut contact = active_contact(
            0,
            ContactBodyIndex::Dynamic(1),
            GeometryPoint::ZERO,
            UnitVector::from_raw(0, 1 << 30),
        );
        contact.penetration = Length::from_meters(0.1).unwrap();
        world.active_contacts.push(contact);
        let shock = ShockSet {
            constraints: alloc::vec![ShockConstraint {
                contact_index: 0,
                depth: 1,
                normal: AxisConstraint::default(),
            }],
            angular_support_q30: alloc::vec![0; 2],
            position_parents: alloc::vec![Some(ShockParent::BodyA)],
        };

        correct_positions(&mut world, &shock);

        let lower_y = world.bodies[0].state().transform().position.raw()[1];
        let upper_y = world.bodies[1].state().transform().position.raw()[1];
        assert!(lower_y < 0);
        assert!(upper_y > 0);
        assert!(upper_y >= 3 * lower_y.unsigned_abs() as i32);
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

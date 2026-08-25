mod brute_force;
mod grid;

use super::StepStats;
use super::constraint::{relative_normal_speed, two_bodies_mut};
use crate::body::{Body, StaticBody};
use crate::collision::collide;
use crate::geometry::Aabb;
use crate::world::{ActiveContact, BroadPhase, ContactBodyIndex, World};
use alloc::vec::Vec;

const WAKE_SPEED_RAW: i32 = 205; // approximately 0.2 m/s in Q10
const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16
const AUTO_BRUTE_FORCE_LIMIT: usize = 64;

#[derive(Debug, Clone, Copy)]
struct AabbProxy {
    aabb: Aabb,
    body: ContactBodyIndex,
}

#[derive(Debug, Clone, Default)]
pub(in crate::world) struct BroadPhaseScratch {
    proxies: Vec<AabbProxy>,
    grid: grid::Scratch,
}

impl BroadPhaseScratch {
    pub(in crate::world) const fn new() -> Self {
        Self {
            proxies: Vec::new(),
            grid: grid::Scratch::new(),
        }
    }

    fn clear(&mut self) {
        self.proxies.clear();
        self.grid.clear();
    }
}

pub(super) fn build_contacts(world: &mut World) -> StepStats {
    world.active_contacts.clear();
    let mut stats = StepStats::default();

    {
        let bodies = &world.bodies;
        let static_bodies = &world.static_bodies;
        let active_contacts = &mut world.active_contacts;
        let scratch = &mut world.broad_phase_scratch;
        build_proxies(bodies, static_bodies, &mut scratch.proxies);

        match world.settings.broad_phase {
            BroadPhase::BruteForce => brute_force::detect(
                bodies,
                static_bodies,
                active_contacts,
                &scratch.proxies,
                &mut stats,
            ),
            BroadPhase::Grid(settings) => grid::detect(
                bodies,
                static_bodies,
                active_contacts,
                &scratch.proxies,
                &mut scratch.grid,
                settings,
                &mut stats,
            ),
            BroadPhase::Auto(_) if scratch.proxies.len() <= AUTO_BRUTE_FORCE_LIMIT => {
                brute_force::detect(
                    bodies,
                    static_bodies,
                    active_contacts,
                    &scratch.proxies,
                    &mut stats,
                )
            }
            BroadPhase::Auto(settings) => grid::detect(
                bodies,
                static_bodies,
                active_contacts,
                &scratch.proxies,
                &mut scratch.grid,
                settings,
                &mut stats,
            ),
        }

        scratch.clear();
    }

    sort_top_down(world);
    stats.contacts = world.active_contacts.len();
    stats
}

fn build_proxies(bodies: &[Body], static_bodies: &[StaticBody], proxies: &mut Vec<AabbProxy>) {
    proxies.clear();
    proxies.reserve(bodies.len() + static_bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        proxies.push(AabbProxy {
            aabb: body.collider().aabb(body.state().transform()),
            body: ContactBodyIndex::Dynamic(index),
        });
    }
    for (index, body) in static_bodies.iter().enumerate() {
        proxies.push(AabbProxy {
            aabb: body.aabb(),
            body: ContactBodyIndex::Static(index),
        });
    }
}

fn detect_pair(
    bodies: &[Body],
    static_bodies: &[StaticBody],
    active_contacts: &mut Vec<ActiveContact>,
    a: AabbProxy,
    b: AabbProxy,
    stats: &mut StepStats,
) {
    match (a.body, b.body) {
        (ContactBodyIndex::Dynamic(index_a), ContactBodyIndex::Dynamic(index_b)) => {
            let (index_a, index_b, aabb_a, aabb_b) = if index_a < index_b {
                (index_a, index_b, a.aabb, b.aabb)
            } else {
                (index_b, index_a, b.aabb, a.aabb)
            };
            let body_a = &bodies[index_a];
            let body_b = &bodies[index_b];
            if body_a.state().is_sleeping() && body_b.state().is_sleeping() {
                return;
            }

            stats.tested_pairs += 1;
            if !aabb_a.intersects(aabb_b) {
                return;
            }
            stats.aabb_pairs += 1;
            if let Some(manifold) = collide(
                body_a.id(),
                body_a.collider(),
                body_a.state().transform(),
                body_b.id(),
                body_b.collider(),
                body_b.state().transform(),
            ) {
                for (point_index, contact) in manifold.into_contacts().enumerate() {
                    active_contacts.push(ActiveContact {
                        body_a: index_a,
                        body_b: ContactBodyIndex::Dynamic(index_b),
                        point: contact.point,
                        normal: contact.normal,
                        penetration: contact.penetration,
                        correct_position: point_index == 0,
                    });
                }
            }
        }
        (ContactBodyIndex::Dynamic(index), ContactBodyIndex::Static(static_index)) => {
            detect_dynamic_static(
                bodies,
                static_bodies,
                active_contacts,
                index,
                static_index,
                a.aabb,
                b.aabb,
                stats,
            );
        }
        (ContactBodyIndex::Static(static_index), ContactBodyIndex::Dynamic(index)) => {
            detect_dynamic_static(
                bodies,
                static_bodies,
                active_contacts,
                index,
                static_index,
                b.aabb,
                a.aabb,
                stats,
            );
        }
        (ContactBodyIndex::Static(_), ContactBodyIndex::Static(_)) => {}
    }
}

#[allow(clippy::too_many_arguments)]
fn detect_dynamic_static(
    bodies: &[Body],
    static_bodies: &[StaticBody],
    active_contacts: &mut Vec<ActiveContact>,
    index: usize,
    static_index: usize,
    aabb: Aabb,
    static_aabb: Aabb,
    stats: &mut StepStats,
) {
    let body = &bodies[index];
    if body.state().is_sleeping() {
        return;
    }

    stats.tested_pairs += 1;
    if !aabb.intersects(static_aabb) {
        return;
    }
    stats.aabb_pairs += 1;

    let static_body = &static_bodies[static_index];
    if let Some(manifold) = collide(
        body.id(),
        body.collider(),
        body.state().transform(),
        static_body.id(),
        static_body.collider(),
        static_body.transform(),
    ) {
        for (point_index, contact) in manifold.into_contacts().enumerate() {
            active_contacts.push(ActiveContact {
                body_a: index,
                body_b: ContactBodyIndex::Static(static_index),
                point: contact.point,
                normal: contact.normal,
                penetration: contact.penetration,
                correct_position: point_index == 0,
            });
        }
    }
}

fn sort_top_down(world: &mut World) {
    world.active_contacts.sort_by(|a, b| {
        let [ax, ay] = a.point.raw();
        let [bx, by] = b.point.raw();
        by.cmp(&ay)
            .then_with(|| ax.cmp(&bx))
            .then_with(|| a.body_a.cmp(&b.body_a))
    });
}

pub(super) fn wake_impacted_bodies(world: &mut World) {
    for contact in world.active_contacts.iter().copied() {
        let normal_speed = match contact.body_b {
            ContactBodyIndex::Dynamic(index_b) => relative_normal_speed(
                &world.bodies[contact.body_a],
                Some(&world.bodies[index_b]),
                contact.point,
                contact.normal,
            ),
            ContactBodyIndex::Static(_) => relative_normal_speed(
                &world.bodies[contact.body_a],
                None,
                contact.point,
                contact.normal,
            ),
        };
        let strong =
            normal_speed < -WAKE_SPEED_RAW || contact.penetration.raw() > WAKE_PENETRATION_RAW;
        if !strong {
            continue;
        }

        match contact.body_b {
            ContactBodyIndex::Dynamic(index_b) => {
                let (a, b) = two_bodies_mut(&mut world.bodies, contact.body_a, index_b);
                a.state_mut().wake();
                b.state_mut().wake();
            }
            ContactBodyIndex::Static(_) => {
                world.bodies[contact.body_a].state_mut().wake();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::UnitVector;
    use crate::body::{BodyId, BodyState, Material};
    use crate::collider::Circle;
    use crate::quantity::{
        Angle, AngularVelocity, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
    use crate::transform::Transform;
    use crate::world::{GridBroadPhase, WorldSettings};

    fn zero_gravity_world() -> World {
        World::new(WorldSettings::new(LinearAcceleration::ZERO))
    }

    fn circle_body(id: u64, x: f64, y: f64, radius: f64) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(radius).unwrap()).unwrap(),
            Mass::ONE,
            Material::INELASTIC,
            BodyState::new(
                Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ),
        )
    }

    fn static_circle(id: u64, x: f64, y: f64, radius: f64) -> StaticBody {
        StaticBody::new(
            BodyId::new(id),
            Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
            Circle::new(Length::from_meters(radius).unwrap()).unwrap(),
            Material::INELASTIC,
        )
    }

    fn contacts_with(world: &World, broad_phase: BroadPhase) -> (Vec<ActiveContact>, StepStats) {
        let mut world = world.clone();
        world.settings.broad_phase = broad_phase;
        let stats = build_contacts(&mut world);
        (world.active_contacts, stats)
    }

    #[test]
    fn active_contacts_are_sorted_top_down_with_deterministic_ties() {
        let mut world = zero_gravity_world();
        for (pair_index, x, y) in [
            (0, 0.0, 0.0),
            (1, 1.0, 2.0),
            (2, 0.0, 1.0),
            (3, -1.0, 2.0),
            (4, -1.0, 2.0),
        ] {
            world.active_contacts.push(ActiveContact {
                body_a: pair_index as usize,
                body_b: ContactBodyIndex::Static(0),
                point: Position::from_meters(x, y).unwrap().into(),
                normal: UnitVector::X,
                penetration: Length::ZERO,
                correct_position: true,
            });
        }

        sort_top_down(&mut world);

        let body_indices = world
            .active_contacts
            .iter()
            .map(|contact| contact.body_a)
            .collect::<alloc::vec::Vec<_>>();
        assert_eq!(body_indices, [3, 4, 1, 2, 0]);
    }

    #[test]
    fn grid_matches_brute_force_for_dynamic_and_static_bodies() {
        let mut world = zero_gravity_world();
        for (id, x, y, radius) in [
            (1, -4.0, 0.0, 2.0),
            (2, -1.0, 0.0, 2.0),
            (3, 1.5, 0.0, 1.0),
            (4, 8.0, 1.0, 1.5),
            (5, 10.0, 1.0, 1.0),
        ] {
            world.add_body(circle_body(id, x, y, radius)).unwrap();
        }
        world
            .add_static_body(static_circle(100, -2.0, -2.5, 1.0))
            .unwrap();
        world
            .add_static_body(static_circle(101, 9.0, -1.0, 1.25))
            .unwrap();

        let brute = contacts_with(&world, BroadPhase::BruteForce);
        let grid = contacts_with(&world, BroadPhase::Grid(GridBroadPhase::new(0).unwrap()));

        assert_eq!(grid.0, brute.0);
        assert_eq!(grid.1.aabb_pairs, brute.1.aabb_pairs);
        assert_eq!(grid.1.contacts, brute.1.contacts);
        assert!(grid.1.tested_pairs < brute.1.tested_pairs);
    }

    #[test]
    fn pair_spanning_many_columns_is_detected_once() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1, 0.0, 0.0, 4.0)).unwrap();
        world.add_body(circle_body(2, 1.0, 0.0, 4.0)).unwrap();
        world.settings.broad_phase = BroadPhase::Grid(GridBroadPhase::new(0).unwrap());

        let stats = build_contacts(&mut world);

        assert_eq!(stats.tested_pairs, 1);
        assert_eq!(stats.aabb_pairs, 1);
        assert_eq!(stats.contacts, 1);
    }

    #[test]
    fn touching_on_column_boundary_remains_a_candidate() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1, 0.5, 0.0, 0.5)).unwrap();
        world.add_body(circle_body(2, 1.5, 0.0, 0.5)).unwrap();
        world.settings.broad_phase = BroadPhase::Grid(GridBroadPhase::new(0).unwrap());

        let stats = build_contacts(&mut world);

        assert_eq!(stats.tested_pairs, 1);
        assert_eq!(stats.aabb_pairs, 1);
        assert_eq!(stats.contacts, 1);
    }

    #[test]
    fn auto_uses_grid_above_the_brute_force_limit() {
        let mut world = zero_gravity_world();
        for index in 0..=AUTO_BRUTE_FORCE_LIMIT {
            let x = -3_200.0 + 100.0 * index as f64;
            world
                .add_body(circle_body(index as u64 + 1, x, 0.0, 0.5))
                .unwrap();
        }
        world.settings.broad_phase = BroadPhase::Auto(GridBroadPhase::default());

        let stats = build_contacts(&mut world);

        assert_eq!(stats.tested_pairs, 0);
        assert_eq!(stats.contacts, 0);
    }

    #[test]
    fn grid_matches_brute_force_across_deterministic_worlds() {
        let mut random = 0x6d2b_79f5_u32;
        for _ in 0..32 {
            let mut world = zero_gravity_world();
            for index in 0..20 {
                random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let x = (random % 513) as f64 * 0.25 - 64.0;
                random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let y = (random % 129) as f64 * 0.25 - 16.0;
                random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let radius = (random % 16 + 1) as f64 * 0.25;
                world
                    .add_body(circle_body(index + 1, x, y, radius))
                    .unwrap();
            }
            for index in 0..5 {
                random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let x = (random % 513) as f64 * 0.25 - 64.0;
                random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let y = (random % 129) as f64 * 0.25 - 16.0;
                random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let radius = (random % 16 + 1) as f64 * 0.25;
                world
                    .add_static_body(static_circle(100 + index, x, y, radius))
                    .unwrap();
            }

            let brute = contacts_with(&world, BroadPhase::BruteForce);
            for power in [0, 4, 8] {
                let grid = contacts_with(
                    &world,
                    BroadPhase::Grid(GridBroadPhase::new(power).unwrap()),
                );
                assert_eq!(grid.0, brute.0);
                assert_eq!(grid.1.aabb_pairs, brute.1.aabb_pairs);
                assert_eq!(grid.1.contacts, brute.1.contacts);
            }
        }
    }
}

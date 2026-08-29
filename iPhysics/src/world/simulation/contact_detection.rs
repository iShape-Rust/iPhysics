mod brute_force;
mod grid;

use super::StepStats;
use crate::body::{Body, StaticBody};
use crate::collision::CollisionSolver;
use crate::geometry::Aabb;
use crate::world::{
    ActiveContactData, ActiveContactDynamic, ActiveContactStatic, BroadPhase, World,
};
use alloc::vec::Vec;
use core::ops::Deref;
use i_key_sort::sort::two_keys_cmp::TwoKeysAndCmpSort;

const AUTO_BRUTE_FORCE_LIMIT: usize = 64;

#[derive(Debug, Clone, Copy)]
struct AabbProxy {
    aabb: Aabb,
    body: ProxyBodyIndex,
}

#[derive(Debug, Clone, Copy)]
enum ProxyBodyIndex {
    Dynamic(usize),
    Static(usize),
}

#[derive(Debug, Clone, Default)]
pub(in crate::world) struct BroadPhaseScratch {
    proxies: Vec<AabbProxy>,
    grid: grid::Scratch,
    static_contact_sort_buffer: Vec<ActiveContactStatic>,
    dynamic_contact_sort_buffer: Vec<ActiveContactDynamic>,
}

impl BroadPhaseScratch {
    pub(in crate::world) const fn new() -> Self {
        Self {
            proxies: Vec::new(),
            grid: grid::Scratch::new(),
            static_contact_sort_buffer: Vec::new(),
            dynamic_contact_sort_buffer: Vec::new(),
        }
    }

    fn clear(&mut self) {
        self.proxies.clear();
        self.grid.clear();
        self.static_contact_sort_buffer.clear();
        self.dynamic_contact_sort_buffer.clear();
    }
}

impl World {
    pub(super) fn build_contacts(&mut self) -> StepStats {
        self.active_static_contacts.clear();
        self.active_dynamic_contacts.clear();

        let scratch = &mut self.broad_phase_scratch;
        build_proxies(&self.bodies, &self.static_bodies, &mut scratch.proxies);
        let detector = Detector {
            bodies: &self.bodies,
            static_bodies: &self.static_bodies,
            active_static_contacts: &mut self.active_static_contacts,
            active_dynamic_contacts: &mut self.active_dynamic_contacts,
            proxies: &scratch.proxies,
            stats: StepStats::default(),
            collision_solver: CollisionSolver::new(),
        };

        let stats = detector.detect(
            self.settings.broad_phase,
            &mut scratch.grid,
            &mut scratch.static_contact_sort_buffer,
            &mut scratch.dynamic_contact_sort_buffer,
        );
        scratch.clear();
        stats
    }
}

struct Detector<'a> {
    bodies: &'a [Body],
    static_bodies: &'a [StaticBody],
    active_static_contacts: &'a mut Vec<ActiveContactStatic>,
    active_dynamic_contacts: &'a mut Vec<ActiveContactDynamic>,
    proxies: &'a [AabbProxy],
    stats: StepStats,
    collision_solver: CollisionSolver,
}

fn build_proxies(bodies: &[Body], static_bodies: &[StaticBody], proxies: &mut Vec<AabbProxy>) {
    proxies.clear();
    proxies.reserve(bodies.len() + static_bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        proxies.push(AabbProxy {
            aabb: body.collider().aabb(body.state().transform()),
            body: ProxyBodyIndex::Dynamic(index),
        });
    }
    for (index, body) in static_bodies.iter().enumerate() {
        proxies.push(AabbProxy {
            aabb: body.aabb(),
            body: ProxyBodyIndex::Static(index),
        });
    }
}

impl Detector<'_> {
    fn detect(
        mut self,
        broad_phase: BroadPhase,
        grid_scratch: &mut grid::Scratch,
        static_contact_sort_buffer: &mut Vec<ActiveContactStatic>,
        dynamic_contact_sort_buffer: &mut Vec<ActiveContactDynamic>,
    ) -> StepStats {
        match broad_phase {
            BroadPhase::BruteForce => self.detect_brute_force(),
            BroadPhase::Grid(settings) => self.detect_grid(grid_scratch, settings),
            BroadPhase::Auto(_) if self.proxies.len() <= AUTO_BRUTE_FORCE_LIMIT => {
                self.detect_brute_force()
            }
            BroadPhase::Auto(settings) => self.detect_grid(grid_scratch, settings),
        }

        sort_top_down(self.active_static_contacts, static_contact_sort_buffer);
        sort_top_down(self.active_dynamic_contacts, dynamic_contact_sort_buffer);
        self.stats.contacts =
            self.active_static_contacts.len() + self.active_dynamic_contacts.len();
        self.stats
    }

    fn detect_pair(&mut self, a: AabbProxy, b: AabbProxy) {
        match (a.body, b.body) {
            (ProxyBodyIndex::Dynamic(index_a), ProxyBodyIndex::Dynamic(index_b)) => {
                let (index_a, index_b, aabb_a, aabb_b) = if index_a < index_b {
                    (index_a, index_b, a.aabb, b.aabb)
                } else {
                    (index_b, index_a, b.aabb, a.aabb)
                };
                let body_a = &self.bodies[index_a];
                let body_b = &self.bodies[index_b];
                if body_a.state().is_sleeping() && body_b.state().is_sleeping() {
                    return;
                }

                self.stats.tested_pairs += 1;
                if !aabb_a.intersects(aabb_b) {
                    return;
                }
                self.stats.aabb_pairs += 1;
                let active_contacts = &mut self.active_dynamic_contacts;
                self.collision_solver.collide(
                    body_a.id(),
                    body_a.collider(),
                    body_a.state().transform(),
                    body_b.id(),
                    body_b.collider(),
                    body_b.state().transform(),
                    |manifold| {
                        for (point_index, contact) in manifold.into_contacts().enumerate() {
                            active_contacts.push(ActiveContactDynamic {
                                body_b: index_b,
                                data: ActiveContactData {
                                    body_a: index_a,
                                    point: contact.point,
                                    normal: contact.normal,
                                    penetration: contact.penetration,
                                    key: contact.key.with_correct_position(point_index == 0),
                                },
                            });
                        }
                    },
                );
            }
            (ProxyBodyIndex::Dynamic(index), ProxyBodyIndex::Static(static_index)) => {
                self.detect_dynamic_static(index, static_index, a.aabb, b.aabb);
            }
            (ProxyBodyIndex::Static(static_index), ProxyBodyIndex::Dynamic(index)) => {
                self.detect_dynamic_static(index, static_index, b.aabb, a.aabb);
            }
            (ProxyBodyIndex::Static(_), ProxyBodyIndex::Static(_)) => {}
        }
    }

    fn detect_dynamic_static(
        &mut self,
        index: usize,
        static_index: usize,
        aabb: Aabb,
        static_aabb: Aabb,
    ) {
        let body = &self.bodies[index];
        if body.state().is_sleeping() {
            return;
        }

        self.stats.tested_pairs += 1;
        if !aabb.intersects(static_aabb) {
            return;
        }
        self.stats.aabb_pairs += 1;

        let static_body = &self.static_bodies[static_index];
        let active_contacts = &mut self.active_static_contacts;
        self.collision_solver.collide(
            body.id(),
            body.collider(),
            body.state().transform(),
            static_body.id(),
            static_body.collider(),
            static_body.transform(),
            |manifold| {
                for (point_index, contact) in manifold.into_contacts().enumerate() {
                    active_contacts.push(ActiveContactStatic {
                        body_b: static_index,
                        data: ActiveContactData {
                            body_a: index,
                            point: contact.point,
                            normal: contact.normal,
                            penetration: contact.penetration,
                            key: contact.key.with_correct_position(point_index == 0),
                        },
                    });
                }
            },
        );
    }
}

fn sort_top_down<T>(active_contacts: &mut [T], buffer: &mut Vec<T>)
where
    T: Copy + Deref<Target = ActiveContactData>,
{
    active_contacts.sort_by_two_keys_then_by_and_buffer(
        false,
        buffer,
        |contact| -contact.point.raw()[1],
        |contact| contact.point.raw()[0],
        |a, b| a.body_a.cmp(&b.body_a),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::UnitVector;
    use crate::body::{BodyId, BodyState, Material};
    use crate::collider::{Circle, CompositeCollider};
    use crate::quantity::{
        Angle, AngularVelocity, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
    use crate::transform::Transform;
    use crate::world::{GridBroadPhase, WorldSettings};
    use alloc::vec;

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

    impl World {
        fn contacts_with(
            &self,
            broad_phase: BroadPhase,
        ) -> (
            Vec<ActiveContactStatic>,
            Vec<ActiveContactDynamic>,
            StepStats,
        ) {
            let mut world = self.clone();
            world.settings.broad_phase = broad_phase;
            let stats = world.build_contacts();
            (
                world.active_static_contacts,
                world.active_dynamic_contacts,
                stats,
            )
        }
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
            world.active_static_contacts.push(ActiveContactStatic {
                body_b: 0,
                data: ActiveContactData {
                    body_a: pair_index as usize,
                    point: Position::from_meters(x, y).unwrap().into(),
                    normal: UnitVector::X,
                    penetration: Length::ZERO,
                    key: crate::collision::ContactKey::new(
                        crate::collision::ColliderFeature::Circle,
                        crate::collision::ColliderFeature::Circle,
                    )
                    .with_correct_position(true),
                },
            });
        }

        let mut buffer = Vec::new();
        sort_top_down(&mut world.active_static_contacts, &mut buffer);

        let body_indices = world
            .active_static_contacts
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

        let brute = world.contacts_with(BroadPhase::BruteForce);
        let grid = world.contacts_with(BroadPhase::Grid(GridBroadPhase::new(0).unwrap()));

        assert_eq!(grid.0, brute.0);
        assert_eq!(grid.1, brute.1);
        assert_eq!(grid.2.aabb_pairs, brute.2.aabb_pairs);
        assert_eq!(grid.2.contacts, brute.2.contacts);
        assert!(grid.2.tested_pairs < brute.2.tested_pairs);
    }

    #[test]
    fn pair_spanning_many_columns_is_detected_once() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1, 0.0, 0.0, 4.0)).unwrap();
        world.add_body(circle_body(2, 1.0, 0.0, 4.0)).unwrap();
        world.settings.broad_phase = BroadPhase::Grid(GridBroadPhase::new(0).unwrap());

        let stats = world.build_contacts();

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

        let stats = world.build_contacts();

        assert_eq!(stats.tested_pairs, 1);
        assert_eq!(stats.aabb_pairs, 1);
        assert_eq!(stats.contacts, 1);
    }

    #[test]
    fn composite_body_pair_keeps_contacts_from_separate_parts() {
        let radius = Length::from_meters(0.5).unwrap();
        let composite = |y: f64| {
            CompositeCollider::new(vec![
                Circle::with_center(Position::from_meters(-1.0, y).unwrap(), radius)
                    .unwrap()
                    .into(),
                Circle::with_center(Position::from_meters(1.0, y).unwrap(), radius)
                    .unwrap()
                    .into(),
            ])
        };
        let mut world = zero_gravity_world();
        world
            .add_body(Body::dynamic(
                BodyId::new(1),
                composite(0.0),
                Mass::ONE,
                Material::INELASTIC,
                BodyState::new(
                    Transform::IDENTITY,
                    LinearVelocity::ZERO,
                    AngularVelocity::ZERO,
                ),
            ))
            .unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::IDENTITY,
                composite(0.75),
                Material::INELASTIC,
            ))
            .unwrap();

        let stats = world.build_contacts();

        assert_eq!(stats.contacts, 2);
        assert_eq!(world.contacts().len(), 2);
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

        let stats = world.build_contacts();

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

            let brute = world.contacts_with(BroadPhase::BruteForce);
            for power in [0, 4, 8] {
                let grid =
                    world.contacts_with(BroadPhase::Grid(GridBroadPhase::new(power).unwrap()));
                assert_eq!(grid.0, brute.0);
                assert_eq!(grid.1, brute.1);
                assert_eq!(grid.2.aabb_pairs, brute.2.aabb_pairs);
                assert_eq!(grid.2.contacts, brute.2.contacts);
            }
        }
    }
}

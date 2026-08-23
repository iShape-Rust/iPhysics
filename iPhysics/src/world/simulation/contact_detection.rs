use super::StepStats;
use super::constraint::{relative_normal_speed, two_bodies_mut};
use crate::collision::collide;
use crate::world::{ActiveContact, ContactBodyIndex, World};

const WAKE_SPEED_RAW: i32 = 205; // approximately 0.2 m/s in Q10
const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16

pub(super) fn build_contacts(world: &mut World) -> StepStats {
    world.active_contacts.clear();
    let mut stats = StepStats::default();

    for index_a in 0..world.bodies.len() {
        let a = &world.bodies[index_a];
        let aabb_a = a.collider().aabb(a.state().transform());

        for index_b in index_a + 1..world.bodies.len() {
            let b = &world.bodies[index_b];
            if a.state().is_sleeping() && b.state().is_sleeping() {
                continue;
            }

            stats.tested_pairs += 1;
            let aabb_b = b.collider().aabb(b.state().transform());
            if !aabb_a.intersects(aabb_b) {
                continue;
            }
            stats.aabb_pairs += 1;

            if let Some(manifold) = collide(
                a.id(),
                a.collider(),
                a.state().transform(),
                b.id(),
                b.collider(),
                b.state().transform(),
            ) {
                for (point_index, contact) in manifold.into_contacts().enumerate() {
                    world.active_contacts.push(ActiveContact {
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

        if a.state().is_sleeping() {
            continue;
        }

        for (static_index, static_body) in world.static_bodies.iter().enumerate() {
            if !aabb_a.intersects(static_body.aabb()) {
                continue;
            }

            for part in static_body.collider().parts() {
                stats.tested_pairs += 1;
                let part_transform = static_body.transform().compose(part.local_transform());
                let part_aabb = part.collider().aabb(part_transform);
                if !aabb_a.intersects(part_aabb) {
                    continue;
                }
                stats.aabb_pairs += 1;

                if let Some(manifold) = collide(
                    a.id(),
                    a.collider(),
                    a.state().transform(),
                    static_body.id(),
                    part.collider(),
                    part_transform,
                ) {
                    for (point_index, contact) in manifold.into_contacts().enumerate() {
                        world.active_contacts.push(ActiveContact {
                            body_a: index_a,
                            body_b: ContactBodyIndex::Static(static_index),
                            point: contact.point,
                            normal: contact.normal,
                            penetration: contact.penetration,
                            correct_position: point_index == 0,
                        });
                    }
                }
            }
        }
    }

    sort_top_down(world);
    stats.contacts = world.active_contacts.len();
    stats
}

fn sort_top_down(world: &mut World) {
    world.active_contacts.sort_unstable_by(|a, b| {
        let [ax, ay] = a.point.raw();
        let [bx, by] = b.point.raw();
        by.cmp(&ay)
            .then_with(|| ax.cmp(&bx))
            .then_with(|| a.body_a.cmp(&b.body_a))
            .then_with(|| contact_body_key(a.body_b).cmp(&contact_body_key(b.body_b)))
            .then_with(|| a.normal.raw().cmp(&b.normal.raw()))
            .then_with(|| a.penetration.raw().cmp(&b.penetration.raw()))
            .then_with(|| b.correct_position.cmp(&a.correct_position))
    });
}

#[inline(always)]
fn contact_body_key(body: ContactBodyIndex) -> (u8, usize) {
    match body {
        ContactBodyIndex::Dynamic(index) => (0, index),
        ContactBodyIndex::Static(index) => (1, index),
    }
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
    use crate::body::{Body, BodyId, BodyState, Material, StaticBody};
    use crate::collider::{Circle, ColliderPart, CompositeCollider};
    use crate::quantity::{
        Angle, AngularVelocity, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
    use crate::transform::Transform;
    use crate::world::WorldSettings;
    use alloc::vec;

    fn circle_body(id: u64, x: f64) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
            Mass::ONE,
            Material::INELASTIC,
            BodyState::new(
                Transform::new(Position::from_meters(x, 0.0).unwrap(), Angle::ZERO),
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ),
        )
    }

    fn zero_gravity_world() -> World {
        World::new(WorldSettings::new(LinearAcceleration::ZERO))
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
    fn composite_part_identity_is_discarded_after_narrow_phase() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1, 3.0)).unwrap();
        let small_circle = Circle::new(Length::from_meters(0.5).unwrap()).unwrap();
        let composite = CompositeCollider::new(vec![
            ColliderPart::new(
                Transform::new(Position::from_meters(-3.0, 0.0).unwrap(), Angle::ZERO),
                small_circle.into(),
            ),
            ColliderPart::new(
                Transform::new(Position::from_meters(3.5, 0.0).unwrap(), Angle::ZERO),
                small_circle.into(),
            ),
        ])
        .unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::IDENTITY,
                composite,
                Material::INELASTIC,
            ))
            .unwrap();

        let stats = world.step();

        assert_eq!(stats.contacts, 1);
        let contact = world.contacts().next().unwrap();
        assert_eq!(contact.body_a, BodyId::new(1));
        assert_eq!(contact.body_b, BodyId::new(2));
    }
}

use criterion::{BatchSize, BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use i_physics::{
    Angle, AngularVelocity, Body, BodyId, BodyState, BroadPhase, Circle, Convex, Damping,
    GridBroadPhase, Length, LinearAcceleration, LinearVelocity, Mass, Material, Position,
    SleepConfig, Transform, World, WorldSettings,
};
use std::hint::black_box;
use std::time::Duration;

const RADIUS_METERS: f64 = 0.5;
const SPACING_METERS: f64 = 0.9;

fn circle_body(id: u64, x: f64, y: f64, velocity_sign: f64) -> Body {
    Body::dynamic(
        BodyId::new(id),
        Circle::new(Length::from_meters(RADIUS_METERS).unwrap()).unwrap(),
        Mass::ONE,
        Material::INELASTIC,
        BodyState::new(
            Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
            LinearVelocity::from_meters_per_second(velocity_sign, -velocity_sign).unwrap(),
            AngularVelocity::ZERO,
        ),
    )
}

fn dense_world(side: usize, velocity_iterations: u8) -> World {
    let mut settings = WorldSettings::new(LinearAcceleration::ZERO);
    settings.broad_phase = BroadPhase::Grid(GridBroadPhase::new(0).unwrap());
    settings.linear_damping = Damping::NONE;
    settings.angular_damping = Damping::NONE;
    settings.velocity_iterations = velocity_iterations;
    settings.sleep = SleepConfig::from_raw(0, 0, u8::MAX);

    let mut world = World::new(settings);
    let first = -0.5 * SPACING_METERS * (side - 1) as f64;
    for row in 0..side {
        for column in 0..side {
            let id = (row * side + column + 1) as u64;
            let x = first + SPACING_METERS * column as f64;
            let y = first + SPACING_METERS * row as f64;
            let velocity_sign = if (row + column) & 1 == 0 { 1.0 } else { -1.0 };
            world
                .add_body(circle_body(id, x, y, velocity_sign))
                .unwrap();
        }
    }

    world
}

fn square() -> Convex {
    let half_extent = 0.5;
    Convex::new(&[
        Position::from_meters(-half_extent, -half_extent).unwrap(),
        Position::from_meters(half_extent, -half_extent).unwrap(),
        Position::from_meters(half_extent, half_extent).unwrap(),
        Position::from_meters(-half_extent, half_extent).unwrap(),
    ])
    .unwrap()
}

fn convex_body(id: u64, x: f64, y: f64, velocity_sign: f64) -> Body {
    Body::dynamic(
        BodyId::new(id),
        square(),
        Mass::ONE,
        Material::INELASTIC,
        BodyState::new(
            Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
            LinearVelocity::from_meters_per_second(velocity_sign, -velocity_sign).unwrap(),
            AngularVelocity::ZERO,
        ),
    )
}

fn dense_convex_world(side: usize, velocity_iterations: u8) -> World {
    let mut settings = WorldSettings::new(LinearAcceleration::ZERO);
    settings.broad_phase = BroadPhase::Grid(GridBroadPhase::new(0).unwrap());
    settings.linear_damping = Damping::NONE;
    settings.angular_damping = Damping::NONE;
    settings.velocity_iterations = velocity_iterations;
    settings.sleep = SleepConfig::from_raw(0, 0, u8::MAX);

    let mut world = World::new(settings);
    let first = -0.5 * SPACING_METERS * (side - 1) as f64;
    for row in 0..side {
        for column in 0..side {
            let id = (row * side + column + 1) as u64;
            let x = first + SPACING_METERS * column as f64;
            let y = first + SPACING_METERS * row as f64;
            let velocity_sign = if (row + column) & 1 == 0 { 1.0 } else { -1.0 };
            world
                .add_body(convex_body(id, x, y, velocity_sign))
                .unwrap();
        }
    }

    world
}

fn expected_contacts(side: usize) -> usize {
    2 * side * (side - 1)
}

fn bench_dense_contacts(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("step/dense_contacts");
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(3));
    group.sample_size(20);

    for side in [8, 16, 32] {
        let body_count = side * side;
        let contact_count = expected_contacts(side);
        group.throughput(Throughput::Elements(contact_count as u64));

        for velocity_iterations in [1, 8] {
            let world = dense_world(side, velocity_iterations);
            let stats = world.clone().step();
            assert_eq!(stats.contacts, contact_count);

            group.bench_with_input(
                BenchmarkId::new(format!("iterations_{velocity_iterations}"), body_count),
                &world,
                |bencher, world| {
                    bencher.iter_batched(
                        || world.clone(),
                        |mut world| black_box(world.step()),
                        BatchSize::SmallInput,
                    );
                },
            );
        }
    }

    group.finish();

    let mut group = criterion.benchmark_group("step/dense_convex_contacts");
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(3));
    group.sample_size(20);

    for side in [8, 16, 32] {
        let body_count = side * side;

        for velocity_iterations in [1, 8] {
            let world = dense_convex_world(side, velocity_iterations);
            let stats = world.clone().step();
            group.throughput(Throughput::Elements(stats.contacts as u64));

            group.bench_with_input(
                BenchmarkId::new(format!("iterations_{velocity_iterations}"), body_count),
                &world,
                |bencher, world| {
                    bencher.iter_batched(
                        || world.clone(),
                        |mut world| black_box(world.step()),
                        BatchSize::SmallInput,
                    );
                },
            );
        }
    }

    group.finish();
}

criterion_group!(benches, bench_dense_contacts);
criterion_main!(benches);

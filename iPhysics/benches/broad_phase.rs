use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use i_physics::{
    Angle, AngularVelocity, Body, BodyId, BodyState, BroadPhase, Circle, GridBroadPhase, Length,
    LinearAcceleration, LinearVelocity, Mass, Material, Position, Transform, World, WorldSettings,
};
use std::hint::black_box;
use std::time::Duration;

const GRID: GridBroadPhase = match GridBroadPhase::new(5) {
    Some(grid) => grid,
    None => unreachable!(),
};

fn circle_body(id: u64, x: f64, y: f64) -> Body {
    Body::dynamic(
        BodyId::new(id),
        Circle::new(Length::from_meters(1.5).unwrap()).unwrap(),
        Mass::ONE,
        Material::INELASTIC,
        BodyState::new(
            Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
            LinearVelocity::ZERO,
            AngularVelocity::ZERO,
        ),
    )
}

fn world_with_positions(
    broad_phase: BroadPhase,
    positions: impl Iterator<Item = [f64; 2]>,
) -> World {
    let mut settings = WorldSettings::new(LinearAcceleration::ZERO);
    settings.broad_phase = broad_phase;
    settings.velocity_iterations = 1;
    let mut world = World::new(settings);
    for (index, [x, y]) in positions.enumerate() {
        world.add_body(circle_body(index as u64 + 1, x, y)).unwrap();
    }
    world
}

fn spread_x(count: usize) -> impl Iterator<Item = [f64; 2]> {
    let first_x = -3.0 * (count - 1) as f64;
    (0..count).map(move |index| [first_x + 6.0 * index as f64, 0.0])
}

fn same_x(count: usize) -> impl Iterator<Item = [f64; 2]> {
    let first_y = -3.0 * (count - 1) as f64;
    (0..count).map(move |index| [0.0, first_y + 6.0 * index as f64])
}

fn square_grid(side: usize) -> impl Iterator<Item = [f64; 2]> {
    let first = -3.0 * (side - 1) as f64;
    (0..side * side).map(move |index| {
        let x = first + 6.0 * (index % side) as f64;
        let y = first + 6.0 * (index / side) as f64;
        [x, y]
    })
}

fn warm_up(world: &mut World) {
    let stats = world.step();
    assert_eq!(stats.aabb_pairs, 0);
    assert_eq!(stats.contacts, 0);
}

fn bench_spread_x(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("broad_phase/spread_x");
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(2));
    group.sample_size(20);

    for count in [16, 32, 64, 128, 256, 512, 1_024, 2_048] {
        group.throughput(Throughput::Elements(count as u64));
        for (name, broad_phase) in [
            ("brute_force", BroadPhase::BruteForce),
            ("grid_32m", BroadPhase::Grid(GRID)),
        ] {
            group.bench_with_input(BenchmarkId::new(name, count), &count, |bencher, &count| {
                let mut world = world_with_positions(broad_phase, spread_x(count));
                warm_up(&mut world);
                bencher.iter(|| black_box(world.step()));
            });
        }
    }

    group.finish();
}

fn bench_same_x(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("broad_phase/same_x");
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(2));
    group.sample_size(20);

    for count in [16, 32, 64, 128, 256, 512] {
        group.throughput(Throughput::Elements(count as u64));
        for (name, broad_phase) in [
            ("brute_force", BroadPhase::BruteForce),
            ("grid_32m", BroadPhase::Grid(GRID)),
        ] {
            group.bench_with_input(BenchmarkId::new(name, count), &count, |bencher, &count| {
                let mut world = world_with_positions(broad_phase, same_x(count));
                warm_up(&mut world);
                bencher.iter(|| black_box(world.step()));
            });
        }
    }

    group.finish();
}

fn bench_square_grid(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("broad_phase/square_grid");
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(2));
    group.sample_size(20);

    for side in [4, 8, 16, 32, 64] {
        let count = side * side;
        group.throughput(Throughput::Elements(count as u64));
        for (name, broad_phase) in [
            ("brute_force", BroadPhase::BruteForce),
            ("grid_32m", BroadPhase::Grid(GRID)),
        ] {
            group.bench_with_input(BenchmarkId::new(name, count), &side, |bencher, &side| {
                let mut world = world_with_positions(broad_phase, square_grid(side));
                warm_up(&mut world);
                bencher.iter(|| black_box(world.step()));
            });
        }
    }

    group.finish();
}

criterion_group!(benches, bench_spread_x, bench_same_x, bench_square_grid);
criterion_main!(benches);

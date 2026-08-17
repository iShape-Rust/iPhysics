mod camera;
mod grid;

use camera::Camera;
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use grid::Grid;
use i_physics::{
    Aabb, Angle, AngularVelocity, Body, BodyId, BodyState, Circle, Collider, ColliderPart,
    CompositeCollider, Convex, Length, LinearAcceleration, LinearVelocity, Mass, Material,
    Position, StaticBody, StepStats, Transform, World, WorldSettings,
};
use std::time::{Duration, Instant};

const TICK_DURATION: Duration = Duration::from_nanos(15_625_000);
const CHECKPOINT_INTERVAL: u64 = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    FreeFall,
    ElasticCircles,
    SleepOnSupport,
    CirclePile,
    CircleVsConvex,
    ConvexVsConvex,
    CompositePlayground,
    ReplayRollback,
}

impl Scenario {
    const ALL: [Self; 8] = [
        Self::FreeFall,
        Self::ElasticCircles,
        Self::SleepOnSupport,
        Self::CirclePile,
        Self::CircleVsConvex,
        Self::ConvexVsConvex,
        Self::CompositePlayground,
        Self::ReplayRollback,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::FreeFall => "Free fall",
            Self::ElasticCircles => "Two elastic circles",
            Self::SleepOnSupport => "Sleep on static support",
            Self::CirclePile => "Circle pile / pyramid",
            Self::CircleVsConvex => "Circle vs convex",
            Self::ConvexVsConvex => "Convex vs convex",
            Self::CompositePlayground => "Composite static playground",
            Self::ReplayRollback => "Replay / rollback comparison",
        }
    }

    const fn description(self) -> &'static str {
        match self {
            Self::FreeFall => "One dynamic circle under the fixed 64 Hz gravity step.",
            Self::ElasticCircles => "Equal masses and restitution 1 exchange velocities.",
            Self::SleepOnSupport => "A falling circle settles on a large static circle.",
            Self::CirclePile => "Six circles start in a small pyramid above a curved support.",
            Self::CircleVsConvex => "A circle and a rotated box collide with zero gravity.",
            Self::ConvexVsConvex => "A triangle and a hexagon exercise convex SAT contacts.",
            Self::CompositePlayground => {
                "Circles and convex bodies fall onto a multi-part static collider."
            }
            Self::ReplayRollback => {
                "A cloned checkpoint advances independently and is compared every tick."
            }
        }
    }
}

struct ReplayState {
    world: World,
    checkpoint_tick: u64,
    matched: bool,
    mismatch_tick: Option<u64>,
}

struct PhysicsDebugApp {
    scenario: Scenario,
    world: World,
    replay: Option<ReplayState>,
    camera: Camera,
    grid: Grid,
    tick: u64,
    stats: StepStats,
    running: bool,
    speed: f32,
    accumulator: Duration,
    last_frame: Instant,
    error: Option<String>,
}

impl Default for PhysicsDebugApp {
    fn default() -> Self {
        let scenario = Scenario::FreeFall;
        let world = build_world(scenario);
        Self {
            scenario,
            world,
            replay: None,
            camera: Camera::default(),
            grid: Grid::default(),
            tick: 0,
            stats: StepStats::default(),
            running: true,
            speed: 1.0,
            accumulator: Duration::ZERO,
            last_frame: Instant::now(),
            error: None,
        }
    }
}

impl eframe::App for PhysicsDebugApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.advance_clock();
        self.handle_shortcuts(ui);

        egui::Panel::left("physics_controls")
            .resizable(false)
            .default_size(280.0)
            .frame(egui::Frame::default().fill(Color32::from_rgb(24, 27, 32)))
            .show_inside(ui, |ui| self.controls(ui));

        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(self.grid.background))
            .show_inside(ui, |ui| self.canvas(ui));

        ui.ctx().request_repaint_after(Duration::from_millis(8));
    }
}

impl PhysicsDebugApp {
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.spacing_mut().item_spacing = Vec2::new(6.0, 7.0);
        ui.add_space(8.0);
        ui.heading("iPhysics debug");
        ui.small("egui/wgpu · fixed simulation tick 64 Hz");
        ui.add_space(8.0);

        let previous = self.scenario;
        egui::ComboBox::from_label("Scenario")
            .selected_text(self.scenario.label())
            .width(230.0)
            .show_ui(ui, |ui| {
                for scenario in Scenario::ALL {
                    ui.selectable_value(&mut self.scenario, scenario, scenario.label());
                }
            });
        if self.scenario != previous {
            self.reset();
        }
        ui.small(self.scenario.description());

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui
                .button(if self.running { "Pause" } else { "Run" })
                .clicked()
            {
                self.running = !self.running;
            }
            if ui
                .add_enabled(!self.running, egui::Button::new("Step"))
                .clicked()
            {
                self.step_once();
            }
            if ui.button("Reset").clicked() {
                self.reset();
            }
        });
        ui.add(egui::Slider::new(&mut self.speed, 0.25..=4.0).text("speed"));
        ui.small("Space: run/pause   N: step   R: reset");

        ui.separator();
        ui.monospace(format!("tick              {}", self.tick));
        ui.monospace(format!("dynamic bodies    {}", self.world.body_count()));
        ui.monospace(format!(
            "static bodies     {}",
            self.world.static_body_count()
        ));
        ui.monospace(format!("tested pairs      {}", self.stats.tested_pairs));
        ui.monospace(format!("AABB pairs        {}", self.stats.aabb_pairs));
        ui.monospace(format!("contacts          {}", self.stats.contacts));
        ui.monospace(format!("sleeping bodies   {}", self.stats.sleeping_bodies));

        if let Some(error) = &self.error {
            ui.colored_label(Color32::from_rgb(240, 118, 118), error);
        }

        if let Some(replay) = &self.replay {
            ui.separator();
            ui.label("Rollback diagnostic");
            let color = if replay.matched {
                Color32::from_rgb(87, 214, 141)
            } else {
                Color32::from_rgb(255, 93, 117)
            };
            ui.colored_label(
                color,
                if replay.matched {
                    "MATCH: bodies, contacts and stats"
                } else {
                    "MISMATCH"
                },
            );
            ui.monospace(format!("checkpoint tick   {}", replay.checkpoint_tick));
            ui.monospace(format!(
                "replayed ticks    {}",
                self.tick.saturating_sub(replay.checkpoint_tick)
            ));
            if let Some(tick) = replay.mismatch_tick {
                ui.monospace(format!("first mismatch    {tick}"));
            }
        }

        ui.separator();
        ui.label("Legend");
        legend(ui, Color32::from_rgb(72, 161, 255), "dynamic");
        legend(ui, Color32::from_rgb(78, 211, 183), "sleeping");
        legend(ui, Color32::from_rgb(126, 132, 145), "static");
        legend(ui, Color32::from_rgb(255, 205, 86), "transient contact");
        legend(ui, Color32::from_rgb(106, 226, 125), "AABB");
        ui.small("Mouse wheel: zoom · right/middle drag: pan");
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        self.grid
            .handle_input(ui, &response, rect, &mut self.camera);
        self.grid.paint(&painter, rect, &self.camera);

        for body in self.world.bodies() {
            let color = if body.state().is_sleeping() {
                Color32::from_rgb(78, 211, 183)
            } else {
                Color32::from_rgb(72, 161, 255)
            };

            paint_collider(
                &painter,
                rect,
                &self.camera,
                body.collider(),
                body.state().transform(),
                color,
                2.0,
            );
            paint_body_id(
                &painter,
                rect,
                &self.camera,
                body.id(),
                body.state().transform(),
                color,
            );

            if let Some(aabb) = body.collider().aabb(body.state().transform()) {
                paint_aabb(&painter, rect, &self.camera, aabb);
            }
        }

        for body in self.world.static_bodies() {
            let color = Color32::from_rgb(126, 132, 145);
            for part in body.collider().parts() {
                let Some(transform) = body.transform().checked_compose(part.local_transform())
                else {
                    continue;
                };
                paint_collider(
                    &painter,
                    rect,
                    &self.camera,
                    part.collider(),
                    transform,
                    color,
                    2.0,
                );
            }
            paint_body_id(
                &painter,
                rect,
                &self.camera,
                body.id(),
                body.transform(),
                color,
            );
            paint_aabb(&painter, rect, &self.camera, body.aabb());
        }

        for contact in self.world.contacts() {
            let [x, y] = contact.point.to_meters();
            let [nx, ny] = contact.normal.raw();
            let point = Pos2::new(x as f32, y as f32);
            let normal = Vec2::new(
                nx as f32 / (1_u64 << 30) as f32,
                ny as f32 / (1_u64 << 30) as f32,
            );
            let screen_point = self.camera.screen_from_world(rect, point);
            let screen_tip = self.camera.screen_from_world(rect, point + normal * 0.7);
            let color = Color32::from_rgb(255, 205, 86);
            painter.circle_filled(screen_point, 4.5, color);
            painter.arrow(
                screen_point,
                screen_tip - screen_point,
                Stroke::new(2.0_f32, color),
            );
        }

        if let Some(replay) = &self.replay {
            let color = if replay.matched {
                Color32::from_rgba_unmultiplied(235, 240, 248, 130)
            } else {
                Color32::from_rgb(255, 93, 117)
            };
            for body in replay.world.bodies() {
                paint_collider(
                    &painter,
                    rect,
                    &self.camera,
                    body.collider(),
                    body.state().transform(),
                    color,
                    1.0,
                );
            }

            painter.text(
                rect.right_top() + Vec2::new(-12.0, 12.0),
                Align2::RIGHT_TOP,
                if replay.matched {
                    "ROLLBACK MATCH"
                } else {
                    "ROLLBACK MISMATCH"
                },
                FontId::monospace(14.0),
                color,
            );
        }
    }

    fn handle_shortcuts(&mut self, ui: &egui::Ui) {
        let (toggle, step, reset) = ui.input(|input| {
            (
                input.key_pressed(egui::Key::Space),
                input.key_pressed(egui::Key::N),
                input.key_pressed(egui::Key::R),
            )
        });
        if toggle {
            self.running = !self.running;
        }
        if step && !self.running {
            self.step_once();
        }
        if reset {
            self.reset();
        }
    }

    fn advance_clock(&mut self) {
        let now = Instant::now();
        let elapsed = now
            .duration_since(self.last_frame)
            .min(Duration::from_millis(250));
        self.last_frame = now;

        if !self.running || self.error.is_some() {
            self.accumulator = Duration::ZERO;
            return;
        }

        self.accumulator += elapsed.mul_f32(self.speed);
        let mut steps = 0;
        while self.accumulator >= TICK_DURATION && steps < 32 {
            self.accumulator -= TICK_DURATION;
            self.step_once();
            steps += 1;
            if self.error.is_some() {
                break;
            }
        }
    }

    fn step_once(&mut self) {
        match self.world.step() {
            Ok(stats) => self.stats = stats,
            Err(error) => {
                self.error = Some(format!("step failed: {error:?}"));
                self.running = false;
                return;
            }
        }
        self.tick += 1;

        if let Some(replay) = &mut self.replay {
            match replay.world.step() {
                Ok(replay_stats) => {
                    replay.matched = self.world.bodies() == replay.world.bodies()
                        && self.world.contacts() == replay.world.contacts()
                        && self.stats == replay_stats;
                    if !replay.matched && replay.mismatch_tick.is_none() {
                        replay.mismatch_tick = Some(self.tick);
                    }
                }
                Err(error) => {
                    replay.matched = false;
                    replay.mismatch_tick.get_or_insert(self.tick);
                    self.error = Some(format!("replay step failed: {error:?}"));
                    self.running = false;
                }
            }

            if replay.matched && self.tick.is_multiple_of(CHECKPOINT_INTERVAL) {
                replay.world = self.world.clone();
                replay.checkpoint_tick = self.tick;
            }
        }
    }

    fn reset(&mut self) {
        self.world = build_world(self.scenario);
        self.replay = (self.scenario == Scenario::ReplayRollback).then(|| ReplayState {
            world: self.world.clone(),
            checkpoint_tick: 0,
            matched: true,
            mismatch_tick: None,
        });
        self.tick = 0;
        self.stats = StepStats::default();
        self.accumulator = Duration::ZERO;
        self.last_frame = Instant::now();
        self.error = None;
        self.camera = Camera::default();
    }
}

fn paint_collider(
    painter: &egui::Painter,
    rect: Rect,
    camera: &Camera,
    collider: Collider,
    transform: Transform,
    color: Color32,
    stroke_width: f32,
) {
    let stroke = Stroke::new(stroke_width, color);
    let fill = color.gamma_multiply(0.30);

    match collider {
        Collider::Circle(circle) => {
            let center = screen_position(camera, rect, transform.position);
            let radius = circle.radius().to_meters() as f32 * camera.zoom;
            painter.circle_filled(center, radius, fill);
            painter.circle_stroke(center, radius, stroke);
            painter.circle_filled(center, 2.5, color);
        }
        Collider::Convex(convex) => {
            let points = convex
                .vertices()
                .iter()
                .filter_map(|vertex| transform.checked_apply(*vertex))
                .map(|position| screen_position(camera, rect, position))
                .collect::<Vec<_>>();
            if points.len() == convex.len() {
                painter.add(egui::Shape::convex_polygon(points, fill, stroke));
            }
            painter.circle_filled(
                screen_position(camera, rect, transform.position),
                2.5,
                color,
            );
        }
    }
}

fn paint_aabb(painter: &egui::Painter, rect: Rect, camera: &Camera, aabb: Aabb) {
    let screen_min = screen_position(camera, rect, aabb.min());
    let screen_max = screen_position(camera, rect, aabb.max());
    painter.rect_stroke(
        Rect::from_two_pos(screen_min, screen_max),
        0.0,
        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(106, 226, 125, 150)),
        StrokeKind::Inside,
    );
}

fn paint_body_id(
    painter: &egui::Painter,
    rect: Rect,
    camera: &Camera,
    id: BodyId,
    transform: Transform,
    color: Color32,
) {
    painter.text(
        screen_position(camera, rect, transform.position) + Vec2::new(0.0, -8.0),
        Align2::CENTER_BOTTOM,
        id.raw().to_string(),
        FontId::monospace(11.0),
        color,
    );
}

fn screen_position(camera: &Camera, rect: Rect, position: Position) -> Pos2 {
    let [x, y] = position.to_meters();
    camera.screen_from_world(rect, Pos2::new(x as f32, y as f32))
}

fn build_world(scenario: Scenario) -> World {
    match scenario {
        Scenario::FreeFall => {
            let mut world = World::default();
            add(
                &mut world,
                dynamic(1, 0.0, 5.0, 0.5, 0.0, 0.0, Material::INELASTIC),
            );
            world
        }
        Scenario::ElasticCircles => {
            let mut world = zero_gravity_world();
            add(
                &mut world,
                dynamic(1, -3.0, 1.0, 0.6, 4.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic(2, 3.0, 1.0, 0.6, -4.0, 0.0, Material::ELASTIC),
            );
            world
        }
        Scenario::SleepOnSupport => {
            let mut world = World::default();
            add_static(&mut world, static_support(1));
            add(
                &mut world,
                dynamic(2, 0.0, 4.0, 0.5, 0.0, 0.0, Material::INELASTIC),
            );
            world
        }
        Scenario::CirclePile => {
            let mut world = World::default();
            add_static(&mut world, static_support(1));
            for (offset, (x, y)) in [
                (-1.05, 0.15),
                (0.0, 0.15),
                (1.05, 0.15),
                (-0.53, 1.10),
                (0.53, 1.10),
                (0.0, 2.05),
            ]
            .into_iter()
            .enumerate()
            {
                add(
                    &mut world,
                    dynamic(offset as u64 + 2, x, y, 0.5, 0.0, 0.0, Material::INELASTIC),
                );
            }
            world
        }
        Scenario::CircleVsConvex => {
            let mut world = zero_gravity_world();
            add(
                &mut world,
                dynamic(1, -3.0, 1.0, 0.6, 3.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic_convex(
                    2,
                    3.0,
                    1.0,
                    angle_degrees(20.0),
                    rectangle(0.75, 0.55),
                    -3.0,
                    0.0,
                    Material::ELASTIC,
                ),
            );
            world
        }
        Scenario::ConvexVsConvex => {
            let mut world = zero_gravity_world();
            add(
                &mut world,
                dynamic_convex(
                    1,
                    -3.0,
                    1.0,
                    angle_degrees(-15.0),
                    triangle(0.85),
                    3.0,
                    0.0,
                    Material::ELASTIC,
                ),
            );
            add(
                &mut world,
                dynamic_convex(
                    2,
                    3.0,
                    1.0,
                    angle_degrees(12.0),
                    hexagon(0.75),
                    -3.0,
                    0.0,
                    Material::ELASTIC,
                ),
            );
            world
        }
        Scenario::CompositePlayground => {
            let mut world = World::default();
            add_static(&mut world, composite_playground(1));
            add(
                &mut world,
                dynamic(2, -2.4, 4.0, 0.55, 0.0, 0.0, Material::INELASTIC),
            );
            add(
                &mut world,
                dynamic_convex(
                    3,
                    0.0,
                    5.2,
                    angle_degrees(18.0),
                    rectangle(0.65, 0.5),
                    0.0,
                    0.0,
                    Material::INELASTIC,
                ),
            );
            add(
                &mut world,
                dynamic_convex(
                    4,
                    2.4,
                    4.6,
                    angle_degrees(-12.0),
                    triangle(0.7),
                    0.0,
                    0.0,
                    Material::INELASTIC,
                ),
            );
            world
        }
        Scenario::ReplayRollback => {
            let mut world = zero_gravity_world();
            add(
                &mut world,
                dynamic(1, -3.0, 1.0, 0.6, 4.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic(2, 3.0, 1.0, 0.6, -4.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic(3, 0.0, 3.0, 0.45, 0.0, -1.25, Material::ELASTIC),
            );
            world
        }
    }
}

fn zero_gravity_world() -> World {
    World::new(WorldSettings::new(LinearAcceleration::ZERO))
}

fn dynamic(id: u64, x: f64, y: f64, radius: f64, vx: f64, vy: f64, material: Material) -> Body {
    let collider = Circle::new(Length::from_meters(radius).expect("scenario radius must fit"))
        .expect("scenario radius must be positive");
    dynamic_collider(id, x, y, Angle::ZERO, collider, vx, vy, material)
}

#[allow(clippy::too_many_arguments)]
fn dynamic_convex(
    id: u64,
    x: f64,
    y: f64,
    angle: Angle,
    collider: Convex,
    vx: f64,
    vy: f64,
    material: Material,
) -> Body {
    dynamic_collider(id, x, y, angle, collider, vx, vy, material)
}

#[allow(clippy::too_many_arguments)]
fn dynamic_collider(
    id: u64,
    x: f64,
    y: f64,
    angle: Angle,
    collider: impl Into<Collider>,
    vx: f64,
    vy: f64,
    material: Material,
) -> Body {
    Body::dynamic(
        BodyId::new(id),
        collider,
        Mass::ONE,
        material,
        BodyState::new(
            Transform::new(
                Position::from_meters(x, y).expect("scenario position must fit"),
                angle,
            ),
            LinearVelocity::from_meters_per_second(vx, vy).expect("scenario velocity must fit"),
            AngularVelocity::ZERO,
        ),
    )
}

fn rectangle(half_width: f64, half_height: f64) -> Convex {
    convex(&[
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ])
}

fn triangle(radius: f64) -> Convex {
    convex(&[
        (0.0, radius),
        (-radius, -radius * 0.75),
        (radius, -radius * 0.75),
    ])
}

fn hexagon(radius: f64) -> Convex {
    let h = radius * 0.866_025_403_784_438_6;
    convex(&[
        (radius, 0.0),
        (radius * 0.5, h),
        (-radius * 0.5, h),
        (-radius, 0.0),
        (-radius * 0.5, -h),
        (radius * 0.5, -h),
    ])
}

fn convex(vertices: &[(f64, f64)]) -> Convex {
    let vertices = vertices
        .iter()
        .map(|&(x, y)| Position::from_meters(x, y).expect("convex vertex must fit"))
        .collect::<Vec<_>>();
    Convex::new(&vertices).expect("scenario vertices must form a strict convex")
}

fn angle_degrees(degrees: f64) -> Angle {
    Angle::from_radians(degrees.to_radians()).expect("scenario angle must be finite")
}

fn static_support(id: u64) -> StaticBody {
    StaticBody::new(
        BodyId::new(id),
        Transform::new(Position::from_meters(0.0, -100.5).unwrap(), Angle::ZERO),
        CompositeCollider::single(
            Circle::new(Length::from_meters(100.0).unwrap())
                .unwrap()
                .into(),
        )
        .expect("static support collider must fit"),
        Material::INELASTIC,
    )
    .expect("static support boundary must fit")
}

fn composite_playground(id: u64) -> StaticBody {
    let parts = vec![
        ColliderPart::new(
            Transform::new(Position::from_meters(0.0, -0.65).unwrap(), Angle::ZERO),
            rectangle(5.5, 0.3).into(),
        ),
        ColliderPart::new(
            Transform::new(
                Position::from_meters(-3.3, 0.25).unwrap(),
                angle_degrees(14.0),
            ),
            rectangle(2.0, 0.18).into(),
        ),
        ColliderPart::new(
            Transform::new(
                Position::from_meters(3.3, 0.25).unwrap(),
                angle_degrees(-14.0),
            ),
            rectangle(2.0, 0.18).into(),
        ),
        ColliderPart::new(
            Transform::new(Position::from_meters(0.0, 0.15).unwrap(), Angle::ZERO),
            Circle::new(Length::from_meters(0.7).unwrap())
                .unwrap()
                .into(),
        ),
    ];

    StaticBody::new(
        BodyId::new(id),
        Transform::IDENTITY,
        CompositeCollider::new(parts).expect("playground parts must fit"),
        Material::INELASTIC,
    )
    .expect("playground boundary must fit")
}

fn add(world: &mut World, body: Body) {
    world
        .add_body(body)
        .expect("scenario body IDs must be unique")
}

fn add_static(world: &mut World, body: StaticBody) {
    world
        .add_static_body(body)
        .expect("scenario body IDs must be unique")
}

fn legend(ui: &mut egui::Ui, color: Color32, text: &str) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::splat(10.0), Sense::hover());
        ui.painter().circle_filled(rect.center(), 4.0, color);
        ui.small(text);
    });
}

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("iPhysics Debug")
            .with_inner_size(Vec2::new(1100.0, 780.0)),
        ..eframe::NativeOptions::default()
    };

    eframe::run_native(
        "iPhysics Debug",
        native_options,
        Box::new(|_cc| Ok(Box::new(PhysicsDebugApp::default()))),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_scenario_builds_and_steps() {
        for scenario in Scenario::ALL {
            let mut world = build_world(scenario);
            for _ in 0..4 {
                world.step().unwrap();
            }
        }
    }

    #[test]
    fn new_collider_scenes_generate_contacts() {
        for scenario in [
            Scenario::CircleVsConvex,
            Scenario::ConvexVsConvex,
            Scenario::CompositePlayground,
        ] {
            let mut world = build_world(scenario);
            let mut contact_seen = false;
            for _ in 0..256 {
                contact_seen |= world.step().unwrap().contacts > 0;
            }
            assert!(contact_seen, "{} produced no contacts", scenario.label());
        }
    }

    #[test]
    fn replay_scenario_is_bit_exact_from_clone() {
        let mut reference = build_world(Scenario::ReplayRollback);
        let mut replay = reference.clone();

        for tick in 0..256 {
            let reference_stats = reference.step().unwrap();
            let replay_stats = replay.step().unwrap();
            assert!(
                reference.bodies() == replay.bodies(),
                "body mismatch at tick {tick}"
            );
            assert_eq!(
                reference.contacts(),
                replay.contacts(),
                "contact mismatch at tick {tick}"
            );
            assert_eq!(
                reference_stats, replay_stats,
                "stats mismatch at tick {tick}"
            );
        }
    }

    #[test]
    fn support_scenario_reaches_sleep() {
        let mut world = build_world(Scenario::SleepOnSupport);

        for _ in 0..512 {
            world.step().unwrap();
            if world.bodies().iter().any(|body| body.state().is_sleeping()) {
                return;
            }
        }

        panic!("dynamic circle did not reach sleep within 512 ticks");
    }
}

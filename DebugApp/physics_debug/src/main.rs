mod camera;
mod grid;

use camera::Camera;
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use grid::Grid;
use i_physics::{
    Angle, AngularVelocity, Body, BodyId, BodyState, Circle, Length, LinearAcceleration,
    LinearVelocity, Mass, Material, Position, StepStats, Transform, World, WorldSettings,
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
    ReplayRollback,
}

impl Scenario {
    const ALL: [Self; 5] = [
        Self::FreeFall,
        Self::ElasticCircles,
        Self::SleepOnSupport,
        Self::CirclePile,
        Self::ReplayRollback,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::FreeFall => "Free fall",
            Self::ElasticCircles => "Two elastic circles",
            Self::SleepOnSupport => "Sleep on static support",
            Self::CirclePile => "Circle pile / pyramid",
            Self::ReplayRollback => "Replay / rollback comparison",
        }
    }

    const fn description(self) -> &'static str {
        match self {
            Self::FreeFall => "One dynamic circle under the fixed 64 Hz gravity step.",
            Self::ElasticCircles => "Equal masses and restitution 1 exchange velocities.",
            Self::SleepOnSupport => "A falling circle settles on a large static circle.",
            Self::CirclePile => "Six circles start in a small pyramid above a curved support.",
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
        ui.monospace(format!("bodies            {}", self.world.body_count()));
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

        for (id, body) in self.world.bodies() {
            let [x, y] = body.state().transform().position.to_meters();
            let center = self
                .camera
                .screen_from_world(rect, Pos2::new(x as f32, y as f32));
            let radius = body.collider().radius().to_meters() as f32 * self.camera.zoom;
            let color = if !body.is_dynamic() {
                Color32::from_rgb(126, 132, 145)
            } else if body.state().is_sleeping() {
                Color32::from_rgb(78, 211, 183)
            } else {
                Color32::from_rgb(72, 161, 255)
            };

            painter.circle_filled(center, radius, color.gamma_multiply(0.30));
            painter.circle_stroke(center, radius, Stroke::new(2.0_f32, color));
            painter.circle_filled(center, 2.5, color);
            painter.text(
                center + Vec2::new(0.0, -radius - 5.0),
                Align2::CENTER_BOTTOM,
                format!("{}:{}", id.index(), id.revision()),
                FontId::monospace(11.0),
                color,
            );

            if let Some(aabb) = body.collider().aabb(body.state().transform().position) {
                let [min_x, min_y] = aabb.min().to_meters();
                let [max_x, max_y] = aabb.max().to_meters();
                let screen_a = self
                    .camera
                    .screen_from_world(rect, Pos2::new(min_x as f32, min_y as f32));
                let screen_b = self
                    .camera
                    .screen_from_world(rect, Pos2::new(max_x as f32, max_y as f32));
                painter.rect_stroke(
                    Rect::from_two_pos(screen_a, screen_b),
                    0.0,
                    Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(106, 226, 125, 150)),
                    StrokeKind::Inside,
                );
            }
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
            for (_id, body) in replay.world.bodies() {
                let [x, y] = body.state().transform().position.to_meters();
                let center = self
                    .camera
                    .screen_from_world(rect, Pos2::new(x as f32, y as f32));
                let radius = body.collider().radius().to_meters() as f32 * self.camera.zoom + 3.0;
                painter.circle_stroke(center, radius, Stroke::new(1.0_f32, color));
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
                    replay.matched = self.world.bodies().eq(replay.world.bodies())
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

fn build_world(scenario: Scenario) -> World {
    match scenario {
        Scenario::FreeFall => {
            let mut world = World::default();
            add(
                &mut world,
                dynamic(0.0, 5.0, 0.5, 0.0, 0.0, Material::INELASTIC),
            );
            world
        }
        Scenario::ElasticCircles => {
            let mut world = zero_gravity_world();
            add(
                &mut world,
                dynamic(-3.0, 1.0, 0.6, 4.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic(3.0, 1.0, 0.6, -4.0, 0.0, Material::ELASTIC),
            );
            world
        }
        Scenario::SleepOnSupport => {
            let mut world = World::default();
            add(&mut world, static_support());
            add(
                &mut world,
                dynamic(0.0, 4.0, 0.5, 0.0, 0.0, Material::INELASTIC),
            );
            world
        }
        Scenario::CirclePile => {
            let mut world = World::default();
            add(&mut world, static_support());
            for (x, y) in [
                (-1.05, 0.15),
                (0.0, 0.15),
                (1.05, 0.15),
                (-0.53, 1.10),
                (0.53, 1.10),
                (0.0, 2.05),
            ] {
                add(
                    &mut world,
                    dynamic(x, y, 0.5, 0.0, 0.0, Material::INELASTIC),
                );
            }
            world
        }
        Scenario::ReplayRollback => {
            let mut world = zero_gravity_world();
            add(
                &mut world,
                dynamic(-3.0, 1.0, 0.6, 4.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic(3.0, 1.0, 0.6, -4.0, 0.0, Material::ELASTIC),
            );
            add(
                &mut world,
                dynamic(0.0, 3.0, 0.45, 0.0, -1.25, Material::ELASTIC),
            );
            world
        }
    }
}

fn zero_gravity_world() -> World {
    World::new(WorldSettings::new(LinearAcceleration::ZERO))
}

fn dynamic(x: f64, y: f64, radius: f64, vx: f64, vy: f64, material: Material) -> Body {
    Body::dynamic(
        Circle::new(Length::from_meters(radius).expect("scenario radius must fit"))
            .expect("scenario radius must be positive"),
        Mass::ONE,
        material,
        BodyState::new(
            Transform::new(
                Position::from_meters(x, y).expect("scenario position must fit"),
                Angle::ZERO,
            ),
            LinearVelocity::from_meters_per_second(vx, vy).expect("scenario velocity must fit"),
            AngularVelocity::ZERO,
        ),
    )
}

fn static_support() -> Body {
    Body::static_body(
        Circle::new(Length::from_meters(100.0).unwrap()).unwrap(),
        Material::INELASTIC,
        BodyState::new(
            Transform::new(Position::from_meters(0.0, -100.5).unwrap(), Angle::ZERO),
            LinearVelocity::ZERO,
            AngularVelocity::ZERO,
        ),
    )
}

fn add(world: &mut World, body: Body) -> BodyId {
    world
        .add_body(body)
        .expect("scenario must fit body store capacity")
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
    fn replay_scenario_is_bit_exact_from_clone() {
        let mut reference = build_world(Scenario::ReplayRollback);
        let mut replay = reference.clone();

        for tick in 0..256 {
            let reference_stats = reference.step().unwrap();
            let replay_stats = replay.step().unwrap();
            assert!(
                reference.bodies().eq(replay.bodies()),
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
            if world
                .bodies()
                .any(|(_, body)| body.is_dynamic() && body.state().is_sleeping())
            {
                return;
            }
        }

        panic!("dynamic circle did not reach sleep within 512 ticks");
    }
}

mod camera;
mod grid;

use camera::Camera;
use eframe::egui::{
    self, Align2, Color32, FontId, PointerButton, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2,
};
use grid::Grid;
use i_physics::{
    Aabb, Angle, AngularVelocity, Body, BodyId, BodyState, Circle, Collider, Contact, Convex,
    DistanceJoint, Force, Length, LinearAcceleration, LinearVelocity, Mass, Material, MouseJoint,
    Position, RopeJoint, SimpleCollider, StaticBody, StepStats, Transform, World, WorldSettings,
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
    FrictionComparison,
    SpinningCircle,
    InclinedPlane,
    RestitutionComparison,
    OffCenterImpact,
    BoxStack,
    BoxPyramid,
    DominoPyramid,
    CircleVsConvex,
    ConvexVsConvex,
    CompositePlayground,
    DeepBoxPenetration,
    DistanceDynamicPair,
    DistanceStaticHarpoon,
    RopeDynamicPair,
    RopeStaticHarpoon,
    RopeGravityHarpoon,
    ReplayRollback,
}

impl Scenario {
    const ALL: [Self; 22] = [
        Self::FreeFall,
        Self::ElasticCircles,
        Self::SleepOnSupport,
        Self::CirclePile,
        Self::FrictionComparison,
        Self::SpinningCircle,
        Self::InclinedPlane,
        Self::RestitutionComparison,
        Self::OffCenterImpact,
        Self::BoxStack,
        Self::BoxPyramid,
        Self::DominoPyramid,
        Self::CircleVsConvex,
        Self::ConvexVsConvex,
        Self::CompositePlayground,
        Self::DeepBoxPenetration,
        Self::DistanceDynamicPair,
        Self::DistanceStaticHarpoon,
        Self::RopeDynamicPair,
        Self::RopeStaticHarpoon,
        Self::RopeGravityHarpoon,
        Self::ReplayRollback,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::FreeFall => "Free fall",
            Self::ElasticCircles => "Two elastic circles",
            Self::SleepOnSupport => "Sleep on static support",
            Self::CirclePile => "Circle pile / pyramid",
            Self::FrictionComparison => "Friction comparison",
            Self::SpinningCircle => "Spinning circle on rough floor",
            Self::InclinedPlane => "Circle + box on inclined plane",
            Self::RestitutionComparison => "Restitution comparison",
            Self::OffCenterImpact => "Off-center impact",
            Self::BoxStack => "Box stack stability",
            Self::BoxPyramid => "Box pyramid (base 7)",
            Self::DominoPyramid => "Domino pyramid (base 7)",
            Self::CircleVsConvex => "Circle vs convex",
            Self::ConvexVsConvex => "Convex vs convex",
            Self::CompositePlayground => "Composite static playground",
            Self::DeepBoxPenetration => "Deep penetration: nested boxes",
            Self::DistanceDynamicPair => "DistanceJoint: two dynamic bodies",
            Self::DistanceStaticHarpoon => "DistanceJoint: fixed harpoon",
            Self::RopeDynamicPair => "RopeJoint: two dynamic bodies",
            Self::RopeStaticHarpoon => "RopeJoint: harpoon pull",
            Self::RopeGravityHarpoon => "RopeJoint: gravity harpoon",
            Self::ReplayRollback => "Replay / rollback comparison",
        }
    }

    const fn description(self) -> &'static str {
        match self {
            Self::FreeFall => "One dynamic circle under the fixed 64 Hz gravity step.",
            Self::ElasticCircles => "Equal masses and restitution 1 exchange velocities.",
            Self::SleepOnSupport => "A falling circle settles on a large static circle.",
            Self::CirclePile => "Six circles start in a small pyramid above a curved support.",
            Self::FrictionComparison => {
                "IDs 2/3/4 slide right with friction 0.0/0.5/2.0 on separate tracks."
            }
            Self::SpinningCircle => {
                "A spinning circle starts at rest; rough contact converts spin into rolling."
            }
            Self::InclinedPlane => "A circle and a box descend a rough 20 degree ramp.",
            Self::RestitutionComparison => "IDs 2/3/4 fall with restitution 0.0/0.5/1.0.",
            Self::OffCenterImpact => {
                "A circle strikes above a box center to exercise angular impulse response."
            }
            Self::BoxStack => "Six slightly rotated boxes test resting-contact stability.",
            Self::BoxPyramid => "Twenty-eight squares form a seven-row pyramid on a flat floor.",
            Self::DominoPyramid => {
                "Pi-shaped domino arches form a seven-row pyramid with seven arches at the base."
            }
            Self::CircleVsConvex => "A circle and a rotated box collide with zero gravity.",
            Self::ConvexVsConvex => "A triangle and a hexagon exercise convex SAT contacts.",
            Self::CompositePlayground => {
                "Circles and convex bodies fall onto a multi-part static collider."
            }
            Self::DeepBoxPenetration => {
                "A small dynamic box starts fully embedded in a larger one. Pause, R, then N to inspect each correction tick."
            }
            Self::DistanceDynamicPair => {
                "Off-center anchors keep a fixed separation while both boxes translate and rotate."
            }
            Self::DistanceStaticHarpoon => {
                "A moving body stays tethered at a fixed distance from a static wall anchor."
            }
            Self::RopeDynamicPair => {
                "The bodies separate freely while slack, then the maximum distance becomes taut."
            }
            Self::RopeStaticHarpoon => {
                "A wall-mounted rope arrests and pulls the moving shooter when it becomes taut."
            }
            Self::RopeGravityHarpoon => {
                "Gravity drops the shooter through a slack phase before the wall rope catches it."
            }
            Self::ReplayRollback => {
                "A cloned checkpoint advances independently and is compared every tick."
            }
        }
    }

    const fn hint(self) -> Option<&'static str> {
        match self {
            Self::DistanceDynamicPair => {
                Some("DISTANCE · drag either box; off-center anchors transmit rotation")
            }
            Self::DistanceStaticHarpoon => {
                Some("DISTANCE HARPOON · drag the blue body or reel the tether")
            }
            Self::RopeDynamicPair => Some("ROPE · gray dashed = slack, orange solid = taut"),
            Self::RopeStaticHarpoon => {
                Some("ROPE HARPOON · reel in to pull the shooter toward the wall")
            }
            Self::RopeGravityHarpoon => {
                Some("GRAVITY HARPOON · fall through slack, then swing from the wall")
            }
            _ => None,
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
    dragged_body: Option<BodyId>,
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
            dragged_body: None,
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
            .show_inside(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.controls(ui));
            });

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
        ui.label("Scenario");
        ui.group(|ui| {
            ui.set_width(ui.available_width());
            for scenario in Scenario::ALL {
                ui.selectable_value(&mut self.scenario, scenario, scenario.label());
            }
        });
        if self.scenario != previous {
            self.reset();
        }
        ui.small(self.scenario.description());

        if self.scenario.hint().is_some() {
            self.joint_controls(ui);
        }

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
        legend(
            ui,
            Color32::from_rgb(255, 205, 86),
            "contact + response normal on A",
        );
        legend(ui, Color32::from_rgb(106, 226, 125), "AABB");
        if self.scenario.hint().is_some() {
            legend(ui, Color32::from_rgb(255, 126, 182), "distance constraint");
            legend(ui, Color32::from_rgb(255, 174, 66), "taut rope");
            legend(ui, Color32::from_rgb(135, 143, 158), "slack rope");
        }
        ui.small("Left drag: move body · wheel: zoom · right/middle drag: pan");
    }

    fn joint_controls(&mut self, ui: &mut egui::Ui) {
        let distance = self.world.distance_joints().first().copied();
        let rope = self.world.rope_joints().first().copied();
        let Some((body_a, body_b, current, is_rope)) = distance
            .map(|joint| (joint.body_a(), joint.body_b(), joint.length(), false))
            .or_else(|| {
                rope.map(|joint| (joint.body_a(), joint.body_b(), joint.max_length(), true))
            })
        else {
            return;
        };

        ui.add_space(6.0);
        ui.separator();
        ui.label(if is_rope {
            "Rope max length"
        } else {
            "Distance target length"
        });

        let mut requested = current.to_meters();
        let mut changed = ui
            .add(
                egui::Slider::new(&mut requested, 1.0..=8.0)
                    .text("metres")
                    .fixed_decimals(2),
            )
            .changed();
        ui.horizontal(|ui| {
            if ui.button("Reel in −0.25").clicked() {
                requested = (requested - 0.25).max(1.0);
                changed = true;
            }
            if ui.button("Pay out +0.25").clicked() {
                requested = (requested + 0.25).min(8.0);
                changed = true;
            }
        });
        ui.small("Length changes are quantized before the next fixed simulation tick.");

        if !changed {
            return;
        }
        let length = Length::from_meters(requested).expect("joint control range must fit Length");
        if length == current {
            return;
        }
        if is_rope {
            self.world
                .rope_joint_mut(body_a, body_b)
                .expect("displayed rope joint must still exist")
                .set_max_length(length);
        } else {
            self.world
                .distance_joint_mut(body_a, body_b)
                .expect("displayed distance joint must still exist")
                .set_length(length);
        }
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        self.grid
            .handle_input(ui, &response, rect, &mut self.camera);
        self.handle_mouse_drag(ui, &response, rect);
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

            paint_aabb(
                &painter,
                rect,
                &self.camera,
                body.collider().aabb(body.state().transform()),
            );
        }

        for body in self.world.static_bodies() {
            let color = Color32::from_rgb(126, 132, 145);
            paint_collider(
                &painter,
                rect,
                &self.camera,
                body.collider(),
                body.transform(),
                color,
                2.0,
            );
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

        for joint in self.world.mouse_joints() {
            let Some(body) = self.world.body(joint.body()) else {
                continue;
            };
            let [anchor_x, anchor_y] = joint.world_anchor(body.state().transform()).raw();
            let anchor = screen_raw_position(&self.camera, rect, anchor_x, anchor_y);
            let target = screen_position(&self.camera, rect, joint.target());
            let color = Color32::from_rgb(255, 126, 182);
            painter.line_segment([anchor, target], Stroke::new(2.0_f32, color));
            painter.circle_filled(anchor, 4.0, color);
            painter.circle_stroke(target, 6.0, Stroke::new(2.0_f32, color));
        }

        self.paint_two_body_joints(&painter, rect);

        for contact in self.world.contacts() {
            let [x, y] = contact.point.to_meters();
            let point = Pos2::new(x as f32, y as f32);
            let normal = response_normal_on_body_a(&contact);
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

        if let Some(hint) = self.scenario.hint() {
            painter.text(
                rect.left_top() + Vec2::new(12.0, 12.0),
                Align2::LEFT_TOP,
                hint,
                FontId::monospace(13.0),
                Color32::from_rgb(220, 226, 236),
            );
        }
    }

    fn paint_two_body_joints(&self, painter: &egui::Painter, rect: Rect) {
        for joint in self.world.distance_joints() {
            let Some((anchor_a, anchor_b, current)) = joint_screen_anchors(
                &self.world,
                &self.camera,
                rect,
                joint.body_a(),
                joint.body_b(),
                |transform| joint.world_anchor_a(transform).raw(),
                |transform| joint.world_anchor_b(transform).raw(),
            ) else {
                continue;
            };
            let color = Color32::from_rgb(255, 126, 182);
            painter.line_segment([anchor_a, anchor_b], Stroke::new(3.0_f32, color));
            paint_joint_anchors(painter, anchor_a, anchor_b, color);
            painter.text(
                anchor_a.lerp(anchor_b, 0.5) + Vec2::new(0.0, -8.0),
                Align2::CENTER_BOTTOM,
                format!("{current:.2} / {:.2} m", joint.length().to_meters()),
                FontId::monospace(12.0),
                color,
            );
        }

        for joint in self.world.rope_joints() {
            let Some((anchor_a, anchor_b, current)) = joint_screen_anchors(
                &self.world,
                &self.camera,
                rect,
                joint.body_a(),
                joint.body_b(),
                |transform| joint.world_anchor_a(transform).raw(),
                |transform| joint.world_anchor_b(transform).raw(),
            ) else {
                continue;
            };
            let max_length = joint.max_length().to_meters();
            let taut = rope_is_visually_taut(current, max_length);
            let color = if taut {
                Color32::from_rgb(255, 174, 66)
            } else {
                Color32::from_rgb(135, 143, 158)
            };
            let stroke = Stroke::new(if taut { 3.0_f32 } else { 2.0_f32 }, color);
            if taut {
                painter.line_segment([anchor_a, anchor_b], stroke);
            } else {
                paint_dashed_line(painter, anchor_a, anchor_b, stroke);
            }
            paint_joint_anchors(painter, anchor_a, anchor_b, color);
            painter.text(
                anchor_a.lerp(anchor_b, 0.5) + Vec2::new(0.0, -8.0),
                Align2::CENTER_BOTTOM,
                format!(
                    "{} · {current:.2} / {max_length:.2} m",
                    if taut { "TAUT" } else { "slack" }
                ),
                FontId::monospace(12.0),
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

    fn handle_mouse_drag(&mut self, ui: &egui::Ui, response: &egui::Response, rect: Rect) {
        if response.drag_started_by(PointerButton::Primary)
            && let Some(pointer) = ui.input(|input| input.pointer.press_origin())
            && let Some(target) = pointer_position(&self.camera, rect, pointer)
            && let Some(body_id) = body_at_point(&self.world, target)
        {
            let transform = self.world.body(body_id).unwrap().state().transform();
            let max_force = Force::from_newtons(100.0).unwrap();
            self.world
                .add_mouse_joint(MouseJoint::at_world_point(
                    body_id, transform, target, max_force,
                ))
                .expect("selected body cannot already have a mouse joint");
            if let Some(replay) = &mut self.replay {
                let transform = replay.world.body(body_id).unwrap().state().transform();
                replay
                    .world
                    .add_mouse_joint(MouseJoint::at_world_point(
                        body_id, transform, target, max_force,
                    ))
                    .expect("replay body cannot already have a mouse joint");
            }
            self.dragged_body = Some(body_id);
        }

        if let Some(body_id) = self.dragged_body
            && ui.input(|input| input.pointer.button_down(PointerButton::Primary))
            && let Some(pointer) = response.interact_pointer_pos()
            && let Some(target) = pointer_position(&self.camera, rect, pointer)
        {
            self.world
                .mouse_joint_mut(body_id)
                .expect("dragged body must retain its mouse joint")
                .set_target(target);
            if let Some(replay) = &mut self.replay {
                replay
                    .world
                    .mouse_joint_mut(body_id)
                    .expect("replay body must retain its mouse joint")
                    .set_target(target);
            }
        }

        if response.drag_stopped_by(PointerButton::Primary) {
            self.stop_mouse_drag();
        }
    }

    fn stop_mouse_drag(&mut self) {
        let Some(body_id) = self.dragged_body.take() else {
            return;
        };
        self.world.remove_mouse_joint(body_id);
        if let Some(replay) = &mut self.replay {
            replay.world.remove_mouse_joint(body_id);
        }
    }

    fn advance_clock(&mut self) {
        let now = Instant::now();
        let elapsed = now
            .duration_since(self.last_frame)
            .min(Duration::from_millis(250));
        self.last_frame = now;

        if !self.running {
            self.accumulator = Duration::ZERO;
            return;
        }

        self.accumulator += elapsed.mul_f32(self.speed);
        let mut steps = 0;
        while self.accumulator >= TICK_DURATION && steps < 32 {
            self.accumulator -= TICK_DURATION;
            self.step_once();
            steps += 1;
        }
    }

    fn step_once(&mut self) {
        self.stats = self.world.step();
        self.tick += 1;

        if let Some(replay) = &mut self.replay {
            let replay_stats = replay.world.step();
            replay.matched = self.world.bodies() == replay.world.bodies()
                && self.world.contacts() == replay.world.contacts()
                && self.stats == replay_stats;
            if !replay.matched && replay.mismatch_tick.is_none() {
                replay.mismatch_tick = Some(self.tick);
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
        self.camera = Camera::default();
        self.dragged_body = None;
    }
}

fn pointer_position(camera: &Camera, rect: Rect, pointer: Pos2) -> Option<Position> {
    let world = camera.world_from_screen(rect, pointer);
    Position::from_meters(world.x as f64, world.y as f64)
}

fn body_at_point(world: &World, point: Position) -> Option<BodyId> {
    world
        .bodies()
        .iter()
        .rev()
        .find(|body| collider_contains(body.collider(), body.state().transform(), point))
        .map(Body::id)
}

fn collider_contains(collider: &Collider, transform: Transform, point: Position) -> bool {
    match collider {
        Collider::Circle(circle) => {
            simple_collider_contains(SimpleCollider::Circle(*circle), transform, point)
        }
        Collider::Convex(convex) => {
            simple_collider_contains(SimpleCollider::Convex(*convex), transform, point)
        }
        Collider::Composite(composite) => composite
            .simple_colliders()
            .iter()
            .copied()
            .any(|collider| simple_collider_contains(collider, transform, point)),
    }
}

fn simple_collider_contains(
    collider: SimpleCollider,
    transform: Transform,
    point: Position,
) -> bool {
    let point = point.raw_point();
    match collider {
        SimpleCollider::Circle(circle) => {
            let center = transform.apply(circle.center()).raw_point();
            let dx = point.x as i64 - center.x as i64;
            let dy = point.y as i64 - center.y as i64;
            let radius = (circle.radius().to_meters() * Position::SCALE as f64) as i64;
            dx * dx + dy * dy <= radius * radius
        }
        SimpleCollider::Convex(convex) => {
            let vertices = convex.transformed_vertices(transform);
            let mut has_positive = false;
            let mut has_negative = false;
            for index in 0..vertices.len() {
                let a = vertices[index].raw();
                let b = vertices[(index + 1) % vertices.len()].raw();
                let edge_x = b[0] as i64 - a[0] as i64;
                let edge_y = b[1] as i64 - a[1] as i64;
                let point_x = point.x as i64 - a[0] as i64;
                let point_y = point.y as i64 - a[1] as i64;
                let cross = edge_x * point_y - edge_y * point_x;
                has_positive |= cross > 0;
                has_negative |= cross < 0;
                if has_positive && has_negative {
                    return false;
                }
            }
            true
        }
    }
}

fn paint_collider(
    painter: &egui::Painter,
    rect: Rect,
    camera: &Camera,
    collider: &Collider,
    transform: Transform,
    color: Color32,
    stroke_width: f32,
) {
    match collider {
        Collider::Circle(circle) => paint_simple_collider(
            painter,
            rect,
            camera,
            SimpleCollider::Circle(*circle),
            transform,
            color,
            stroke_width,
        ),
        Collider::Convex(convex) => paint_simple_collider(
            painter,
            rect,
            camera,
            SimpleCollider::Convex(*convex),
            transform,
            color,
            stroke_width,
        ),
        Collider::Composite(composite) => {
            for &collider in composite.simple_colliders() {
                paint_simple_collider(
                    painter,
                    rect,
                    camera,
                    collider,
                    transform,
                    color,
                    stroke_width,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn paint_simple_collider(
    painter: &egui::Painter,
    rect: Rect,
    camera: &Camera,
    collider: SimpleCollider,
    transform: Transform,
    color: Color32,
    stroke_width: f32,
) {
    let stroke = Stroke::new(stroke_width, color);
    let fill = color.gamma_multiply(0.30);

    match collider {
        SimpleCollider::Circle(circle) => {
            let center = screen_position(camera, rect, transform.apply(circle.center()));
            let radius = circle.radius().to_meters() as f32 * camera.zoom;
            painter.circle_filled(center, radius, fill);
            painter.circle_stroke(center, radius, stroke);
            let angle = transform.angle.to_radians() as f32;
            let radius_tip = center + Vec2::new(angle.cos(), -angle.sin()) * radius;
            painter.line_segment([center, radius_tip], stroke);
            painter.circle_filled(center, 2.5, color);
        }
        SimpleCollider::Convex(convex) => {
            let points = convex
                .transformed_vertices(transform)
                .iter()
                .map(|point| {
                    let [x, y] = point.raw();
                    screen_raw_position(camera, rect, x, y)
                })
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
    let min = aabb.min();
    let max = aabb.max();
    let [min_x, min_y] = min.raw();
    let [max_x, max_y] = max.raw();
    let screen_min = screen_raw_position(camera, rect, min_x, min_y);
    let screen_max = screen_raw_position(camera, rect, max_x, max_y);
    painter.rect_stroke(
        Rect::from_two_pos(screen_min, screen_max),
        0.0,
        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(106, 226, 125, 150)),
        StrokeKind::Inside,
    );
}

fn screen_raw_position(camera: &Camera, rect: Rect, x: i32, y: i32) -> Pos2 {
    let scale = Position::SCALE as f32;
    camera.screen_from_world(rect, Pos2::new(x as f32 / scale, y as f32 / scale))
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

fn joint_screen_anchors(
    world: &World,
    camera: &Camera,
    rect: Rect,
    body_a: BodyId,
    body_b: BodyId,
    anchor_a: impl FnOnce(Transform) -> [i32; 2],
    anchor_b: impl FnOnce(Transform) -> [i32; 2],
) -> Option<(Pos2, Pos2, f64)> {
    let transform_a = endpoint_transform(world, body_a)?;
    let transform_b = endpoint_transform(world, body_b)?;
    let [a_x, a_y] = anchor_a(transform_a);
    let [b_x, b_y] = anchor_b(transform_b);
    let dx = (b_x as i64 - a_x as i64) as f64 / Position::SCALE as f64;
    let dy = (b_y as i64 - a_y as i64) as f64 / Position::SCALE as f64;
    Some((
        screen_raw_position(camera, rect, a_x, a_y),
        screen_raw_position(camera, rect, b_x, b_y),
        dx.hypot(dy),
    ))
}

fn endpoint_transform(world: &World, id: BodyId) -> Option<Transform> {
    world
        .body(id)
        .map(|body| body.state().transform())
        .or_else(|| world.static_body(id).map(StaticBody::transform))
}

fn paint_joint_anchors(painter: &egui::Painter, a: Pos2, b: Pos2, color: Color32) {
    painter.circle_filled(a, 4.5, color);
    painter.circle_filled(b, 4.5, color);
    painter.circle_stroke(a, 7.0, Stroke::new(1.0_f32, color));
    painter.circle_stroke(b, 7.0, Stroke::new(1.0_f32, color));
}

fn paint_dashed_line(painter: &egui::Painter, a: Pos2, b: Pos2, stroke: Stroke) {
    let delta = b - a;
    let length = delta.length();
    if length <= f32::EPSILON {
        return;
    }
    let direction = delta / length;
    let dash = 9.0;
    let gap = 6.0;
    let mut start = 0.0;
    while start < length {
        let end = (start + dash).min(length);
        painter.line_segment([a + direction * start, a + direction * end], stroke);
        start += dash + gap;
    }
}

fn rope_is_visually_taut(current: f64, max_length: f64) -> bool {
    current >= max_length * 0.98
}

/// Contact normals are stored A -> B. The displayed arrow is the solver
/// response direction applied to A, so static surfaces point outwards.
fn response_normal_on_body_a(contact: &Contact) -> Vec2 {
    let [nx, ny] = contact.normal.raw();
    let scale = (1_u64 << 30) as f32;
    Vec2::new(-(nx as f32) / scale, -(ny as f32) / scale)
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
        Scenario::FrictionComparison => {
            let mut world = World::default();
            for track in parallel_tracks(10) {
                add_static(&mut world, track);
            }
            for (id, y, friction) in [(2, 3.2, 0.0), (3, 0.2, 0.5), (4, -2.8, 2.0)] {
                add(
                    &mut world,
                    dynamic(
                        id,
                        -4.0,
                        y,
                        0.5,
                        4.0,
                        0.0,
                        Material::new(0.0, friction).unwrap(),
                    ),
                );
            }
            world
        }
        Scenario::SpinningCircle => {
            let mut world = World::default();
            add_static(&mut world, flat_floor(1, Material::new(0.0, 1.5).unwrap()));
            add(
                &mut world,
                dynamic_spinning_circle(
                    2,
                    0.0,
                    -0.43,
                    0.55,
                    0.0,
                    0.0,
                    8.0,
                    Material::new(0.0, 1.5).unwrap(),
                ),
            );
            world
        }
        Scenario::InclinedPlane => {
            let mut world = World::default();
            let rough = Material::new(0.0, 0.8).unwrap();
            add_static(&mut world, inclined_floor(1, 20.0, rough));
            add(&mut world, dynamic(2, 2.7, 1.67, 0.5, 0.0, 0.0, rough));
            add(
                &mut world,
                dynamic_convex(
                    3,
                    0.8,
                    0.88,
                    angle_degrees(20.0),
                    rectangle(0.65, 0.4),
                    0.0,
                    0.0,
                    rough,
                ),
            );
            world
        }
        Scenario::RestitutionComparison => {
            let mut world = World::default();
            add_static(&mut world, flat_floor(1, Material::new(0.0, 0.0).unwrap()));
            for (id, x, restitution) in [(2, -3.0, 0.0), (3, 0.0, 0.5), (4, 3.0, 1.0)] {
                add(
                    &mut world,
                    dynamic(
                        id,
                        x,
                        4.0,
                        0.5,
                        0.0,
                        0.0,
                        Material::new(restitution, 0.0).unwrap(),
                    ),
                );
            }
            world
        }
        Scenario::OffCenterImpact => {
            let mut world = zero_gravity_world();
            let material = Material::new(0.5, 0.0).unwrap();
            add(&mut world, dynamic(1, -4.0, 1.6, 0.4, 6.0, 0.0, material));
            add(
                &mut world,
                dynamic_convex(
                    2,
                    1.0,
                    1.0,
                    Angle::ZERO,
                    rectangle(0.75, 1.0),
                    0.0,
                    0.0,
                    material,
                ),
            );
            world
        }
        Scenario::BoxStack => {
            let mut world = World::default();
            add_static(&mut world, flat_floor(1, Material::INELASTIC));
            for level in 0..6 {
                let angle = if level % 2 == 0 { 1.5 } else { -1.5 };
                add(
                    &mut world,
                    dynamic_convex(
                        level + 2,
                        0.0,
                        -0.63 + level as f64 * 0.72,
                        angle_degrees(angle),
                        rectangle(0.65, 0.35),
                        0.0,
                        0.0,
                        Material::INELASTIC,
                    ),
                );
            }
            world
        }
        Scenario::BoxPyramid => block_pyramid_world(0.32, 0.32, 0.68, 0.68),
        Scenario::DominoPyramid => domino_pyramid_world(),
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
            for part in composite_playground(10) {
                add_static(&mut world, part);
            }
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
        Scenario::DeepBoxPenetration => deep_box_penetration_world(),
        Scenario::DistanceDynamicPair => distance_dynamic_pair_world(),
        Scenario::DistanceStaticHarpoon => distance_static_harpoon_world(),
        Scenario::RopeDynamicPair => rope_dynamic_pair_world(),
        Scenario::RopeStaticHarpoon => rope_static_harpoon_world(),
        Scenario::RopeGravityHarpoon => rope_gravity_harpoon_world(),
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

fn block_pyramid_world(
    half_width: f64,
    half_height: f64,
    horizontal_spacing: f64,
    vertical_spacing: f64,
) -> World {
    const BASE_COUNT: usize = 7;
    const FLOOR_TOP: f64 = -1.0;
    const INITIAL_GAP: f64 = 0.02;

    let mut world = World::default();
    let material = Material::new(0.0, 0.8).unwrap();
    add_static(&mut world, flat_floor(1, material));
    let block = rectangle(half_width, half_height);
    let bottom_y = FLOOR_TOP + half_height + INITIAL_GAP;
    let mut id = 2;
    for row in 0..BASE_COUNT {
        let count = BASE_COUNT - row;
        let y = bottom_y + row as f64 * vertical_spacing;
        for column in 0..count {
            let x = (column as f64 - (count - 1) as f64 * 0.5) * horizontal_spacing;
            add(
                &mut world,
                dynamic_convex(id, x, y, Angle::ZERO, block, 0.0, 0.0, material),
            );
            id += 1;
        }
    }
    world
}

fn domino_pyramid_world() -> World {
    const BASE_COUNT: usize = 7;
    const DOMINO_LENGTH: f64 = 0.75;
    const DOMINO_THICKNESS: f64 = 0.1875;
    const FLOOR_TOP: f64 = -1.0;

    let mut world = World::default();
    let material = Material::new(0.0, 0.8).unwrap();
    add_static(&mut world, flat_floor(1, material));

    let vertical = rectangle(DOMINO_THICKNESS * 0.5, DOMINO_LENGTH * 0.5);
    let horizontal = rectangle(DOMINO_LENGTH * 0.5, DOMINO_THICKNESS * 0.5);
    let level_spacing = DOMINO_LENGTH + DOMINO_THICKNESS;
    let bottom_leg_y = FLOOR_TOP + DOMINO_LENGTH * 0.5;
    let beam_offset_y = (DOMINO_LENGTH + DOMINO_THICKNESS) * 0.5;
    let mut id = 2;

    for row in 0..BASE_COUNT {
        let count = BASE_COUNT - row;
        let leg_y = bottom_leg_y + row as f64 * level_spacing;
        let beam_y = leg_y + beam_offset_y;
        for support in 0..=count {
            let x = (support as f64 - count as f64 * 0.5) * DOMINO_LENGTH;
            add(
                &mut world,
                dynamic_convex(id, x, leg_y, Angle::ZERO, vertical, 0.0, 0.0, material),
            );
            id += 1;
        }
        for beam in 0..count {
            let x = (beam as f64 - (count - 1) as f64 * 0.5) * DOMINO_LENGTH;
            add(
                &mut world,
                dynamic_convex(id, x, beam_y, Angle::ZERO, horizontal, 0.0, 0.0, material),
            );
            id += 1;
        }
    }
    world
}

fn deep_box_penetration_world() -> World {
    let mut world = zero_gravity_world();
    let material = Material::new(0.0, 0.4).unwrap();
    add(
        &mut world,
        dynamic_convex(
            1,
            0.0,
            1.0,
            Angle::ZERO,
            rectangle(2.4, 1.8),
            0.0,
            0.0,
            material,
        ),
    );
    add(
        &mut world,
        dynamic_convex(
            2,
            0.35,
            1.15,
            angle_degrees(12.0),
            rectangle(0.75, 0.55),
            0.0,
            0.0,
            material,
        ),
    );
    world
}

fn distance_dynamic_pair_world() -> World {
    let mut world = zero_gravity_world();
    let material = Material::new(0.15, 0.2).unwrap();
    add(
        &mut world,
        dynamic_convex(
            1,
            -2.5,
            1.0,
            angle_degrees(-12.0),
            rectangle(0.8, 0.5),
            0.0,
            -1.7,
            material,
        ),
    );
    add(
        &mut world,
        dynamic_convex(
            2,
            2.5,
            1.0,
            angle_degrees(15.0),
            rectangle(0.8, 0.5),
            0.0,
            1.7,
            material,
        ),
    );

    let anchor_a = Position::from_meters(-1.8, 1.55).unwrap();
    let anchor_b = Position::from_meters(1.8, 0.45).unwrap();
    let joint = DistanceJoint::between_world_points(
        BodyId::new(1),
        world.body(BodyId::new(1)).unwrap().state().transform(),
        anchor_a,
        BodyId::new(2),
        world.body(BodyId::new(2)).unwrap().state().transform(),
        anchor_b,
        joint_force(),
    )
    .expect("distance joint anchors must fit Length");
    world
        .add_distance_joint(joint)
        .expect("distance scenario endpoints must exist");
    world
}

fn distance_static_harpoon_world() -> World {
    let mut world = zero_gravity_world();
    add_static(&mut world, harpoon_wall(1));
    add(
        &mut world,
        dynamic_convex(
            2,
            -0.4,
            1.2,
            angle_degrees(-18.0),
            rectangle(0.9, 0.45),
            0.2,
            -2.4,
            Material::new(0.1, 0.4).unwrap(),
        ),
    );

    let wall_anchor = Position::from_meters(4.25, 2.6).unwrap();
    let body_anchor = Position::from_meters(0.25, 1.5).unwrap();
    let joint = DistanceJoint::between_world_points(
        BodyId::new(1),
        world.static_body(BodyId::new(1)).unwrap().transform(),
        wall_anchor,
        BodyId::new(2),
        world.body(BodyId::new(2)).unwrap().state().transform(),
        body_anchor,
        joint_force(),
    )
    .expect("distance harpoon anchors must fit Length");
    world
        .add_distance_joint(joint)
        .expect("distance harpoon endpoints must exist");
    world
}

fn rope_dynamic_pair_world() -> World {
    let mut world = zero_gravity_world();
    let material = Material::new(0.0, 0.1).unwrap();
    add(
        &mut world,
        dynamic(1, -1.5, 1.0, 0.65, -2.0, 0.35, material),
    );
    add(&mut world, dynamic(2, 1.5, 1.0, 0.65, 2.0, -0.35, material));
    let joint = RopeJoint::at_world_points(
        BodyId::new(1),
        world.body(BodyId::new(1)).unwrap().state().transform(),
        Position::from_meters(-1.5, 1.0).unwrap(),
        BodyId::new(2),
        world.body(BodyId::new(2)).unwrap().state().transform(),
        Position::from_meters(1.5, 1.0).unwrap(),
        Length::from_meters(4.5).unwrap(),
        joint_force(),
    );
    world
        .add_rope_joint(joint)
        .expect("rope scenario endpoints must exist");
    world
}

fn rope_static_harpoon_world() -> World {
    let mut world = zero_gravity_world();
    add_static(&mut world, harpoon_wall(1));
    add(
        &mut world,
        dynamic_convex(
            2,
            -0.2,
            0.5,
            angle_degrees(8.0),
            rectangle(0.9, 0.55),
            -2.4,
            -0.7,
            Material::new(0.0, 0.5).unwrap(),
        ),
    );

    let joint = RopeJoint::at_world_points(
        BodyId::new(1),
        world.static_body(BodyId::new(1)).unwrap().transform(),
        Position::from_meters(4.25, 2.2).unwrap(),
        BodyId::new(2),
        world.body(BodyId::new(2)).unwrap().state().transform(),
        Position::from_meters(0.55, 0.8).unwrap(),
        Length::from_meters(4.6).unwrap(),
        joint_force(),
    );
    world
        .add_rope_joint(joint)
        .expect("rope harpoon endpoints must exist");
    world
}

fn rope_gravity_harpoon_world() -> World {
    let mut world = World::default();
    add_static(&mut world, harpoon_wall(1));
    add(
        &mut world,
        dynamic_convex(
            2,
            0.5,
            3.4,
            angle_degrees(-8.0),
            rectangle(0.9, 0.55),
            -0.5,
            0.0,
            Material::new(0.0, 0.5).unwrap(),
        ),
    );

    let joint = RopeJoint::at_world_points(
        BodyId::new(1),
        world.static_body(BodyId::new(1)).unwrap().transform(),
        Position::from_meters(4.25, 4.2).unwrap(),
        BodyId::new(2),
        world.body(BodyId::new(2)).unwrap().state().transform(),
        Position::from_meters(1.2, 3.7).unwrap(),
        Length::from_meters(4.2).unwrap(),
        joint_force(),
    );
    world
        .add_rope_joint(joint)
        .expect("gravity harpoon endpoints must exist");
    world
}

fn joint_force() -> Force {
    Force::from_newtons(240.0).unwrap()
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
fn dynamic_spinning_circle(
    id: u64,
    x: f64,
    y: f64,
    radius: f64,
    vx: f64,
    vy: f64,
    angular_velocity: f64,
    material: Material,
) -> Body {
    let collider = Circle::new(Length::from_meters(radius).expect("scenario radius must fit"))
        .expect("scenario radius must be positive");
    dynamic_collider_with_spin(
        id,
        x,
        y,
        Angle::ZERO,
        collider,
        vx,
        vy,
        angular_velocity,
        material,
    )
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
    dynamic_collider_with_spin(id, x, y, angle, collider, vx, vy, 0.0, material)
}

#[allow(clippy::too_many_arguments)]
fn dynamic_collider_with_spin(
    id: u64,
    x: f64,
    y: f64,
    angle: Angle,
    collider: impl Into<Collider>,
    vx: f64,
    vy: f64,
    angular_velocity: f64,
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
            AngularVelocity::from_radians_per_second(angular_velocity)
                .expect("scenario angular velocity must fit"),
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

fn flat_floor(id: u64, material: Material) -> StaticBody {
    StaticBody::new(
        BodyId::new(id),
        Transform::new(Position::from_meters(0.0, -1.25).unwrap(), Angle::ZERO),
        rectangle(5.5, 0.25),
        material,
    )
}

fn inclined_floor(id: u64, degrees: f64, material: Material) -> StaticBody {
    StaticBody::new(
        BodyId::new(id),
        Transform::new(
            Position::from_meters(0.0, 0.0).unwrap(),
            angle_degrees(degrees),
        ),
        rectangle(5.0, 0.2),
        material,
    )
}

fn harpoon_wall(id: u64) -> StaticBody {
    StaticBody::new(
        BodyId::new(id),
        Transform::new(Position::from_meters(4.55, 1.0).unwrap(), Angle::ZERO),
        rectangle(0.3, 4.5),
        Material::INELASTIC,
    )
}

fn parallel_tracks(first_id: u64) -> [StaticBody; 3] {
    let ys = [2.5, -0.5, -3.5];
    core::array::from_fn(|index| {
        StaticBody::new(
            BodyId::new(first_id + index as u64),
            Transform::new(Position::from_meters(0.0, ys[index]).unwrap(), Angle::ZERO),
            rectangle(5.5, 0.18),
            Material::new(0.0, 0.0).unwrap(),
        )
    })
}

fn static_support(id: u64) -> StaticBody {
    StaticBody::new(
        BodyId::new(id),
        Transform::new(Position::from_meters(0.0, -100.5).unwrap(), Angle::ZERO),
        Circle::new(Length::from_meters(100.0).unwrap()).unwrap(),
        Material::INELASTIC,
    )
}

fn composite_playground(first_id: u64) -> [StaticBody; 4] {
    [
        StaticBody::new(
            BodyId::new(first_id),
            Transform::new(Position::from_meters(0.0, -0.65).unwrap(), Angle::ZERO),
            rectangle(5.5, 0.3),
            Material::INELASTIC,
        ),
        StaticBody::new(
            BodyId::new(first_id + 1),
            Transform::new(
                Position::from_meters(-3.3, 0.25).unwrap(),
                angle_degrees(14.0),
            ),
            rectangle(2.0, 0.18),
            Material::INELASTIC,
        ),
        StaticBody::new(
            BodyId::new(first_id + 2),
            Transform::new(
                Position::from_meters(3.3, 0.25).unwrap(),
                angle_degrees(-14.0),
            ),
            rectangle(2.0, 0.18),
            Material::INELASTIC,
        ),
        StaticBody::new(
            BodyId::new(first_id + 3),
            Transform::new(Position::from_meters(0.0, 0.15).unwrap(), Angle::ZERO),
            Circle::new(Length::from_meters(0.7).unwrap()).unwrap(),
            Material::INELASTIC,
        ),
    ]
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
                world.step();
            }
        }
    }

    #[test]
    fn nested_boxes_generate_and_resolve_a_deep_contact() {
        let mut world = build_world(Scenario::DeepBoxPenetration);
        let before = body_center_distance(&world, BodyId::new(1), BodyId::new(2));

        let first_step = world.step();
        assert!(first_step.contacts > 0);
        for _ in 0..15 {
            world.step();
        }

        let after = body_center_distance(&world, BodyId::new(1), BodyId::new(2));
        assert!(
            after > before + 0.5,
            "nested boxes did not separate: {before:.3} m -> {after:.3} m"
        );
    }

    fn body_center_distance(world: &World, body_a: BodyId, body_b: BodyId) -> f64 {
        let [a_x, a_y] = world
            .body(body_a)
            .unwrap()
            .state()
            .transform()
            .position
            .to_meters();
        let [b_x, b_y] = world
            .body(body_b)
            .unwrap()
            .state()
            .transform()
            .position
            .to_meters();
        (b_x - a_x).hypot(b_y - a_y)
    }

    #[test]
    fn joint_scenarios_build_with_the_expected_endpoints() {
        for (scenario, distance_count, rope_count, static_count) in [
            (Scenario::DistanceDynamicPair, 1, 0, 0),
            (Scenario::DistanceStaticHarpoon, 1, 0, 1),
            (Scenario::RopeDynamicPair, 0, 1, 0),
            (Scenario::RopeStaticHarpoon, 0, 1, 1),
            (Scenario::RopeGravityHarpoon, 0, 1, 1),
        ] {
            let world = build_world(scenario);
            assert_eq!(world.distance_joints().len(), distance_count);
            assert_eq!(world.rope_joints().len(), rope_count);
            assert_eq!(world.static_body_count(), static_count);
            for joint in world.distance_joints() {
                assert!(endpoint_transform(&world, joint.body_a()).is_some());
                assert!(endpoint_transform(&world, joint.body_b()).is_some());
            }
            for joint in world.rope_joints() {
                assert!(endpoint_transform(&world, joint.body_a()).is_some());
                assert!(endpoint_transform(&world, joint.body_b()).is_some());
            }
        }
    }

    #[test]
    fn dynamic_rope_scene_transitions_from_slack_to_taut() {
        let mut world = build_world(Scenario::RopeDynamicPair);
        let initial = rope_distance(&world);
        let max_length = world.rope_joints()[0].max_length().to_meters();
        assert!(initial < max_length - 1.0);

        let mut maximum_seen = initial;
        for _ in 0..64 {
            world.step();
            maximum_seen = maximum_seen.max(rope_distance(&world));
        }

        assert!(
            maximum_seen >= max_length,
            "rope never reached its {max_length:.4} m limit; max was {maximum_seen:.4} m"
        );
        assert!(rope_distance(&world) <= max_length + 0.001);
    }

    #[test]
    fn gravity_harpoon_falls_from_slack_to_the_rope_limit() {
        let mut world = build_world(Scenario::RopeGravityHarpoon);
        let initial_y = world
            .body(BodyId::new(2))
            .unwrap()
            .state()
            .transform()
            .position;
        let initial_distance = rope_distance(&world);
        let max_length = world.rope_joints()[0].max_length().to_meters();
        assert!(initial_distance < max_length - 1.0);

        let mut maximum_seen = initial_distance;
        for _ in 0..128 {
            world.step();
            maximum_seen = maximum_seen.max(rope_distance(&world));
        }

        let final_y = world
            .body(BodyId::new(2))
            .unwrap()
            .state()
            .transform()
            .position;
        assert!(final_y.to_meters()[1] < initial_y.to_meters()[1] - 1.0);
        assert!(
            maximum_seen >= max_length,
            "gravity rope never reached its {max_length:.4} m limit; max was {maximum_seen:.4} m"
        );
    }

    #[test]
    fn resetting_each_joint_scene_recreates_a_clean_world() {
        let mut app = PhysicsDebugApp::default();
        for scenario in [
            Scenario::DistanceDynamicPair,
            Scenario::DistanceStaticHarpoon,
            Scenario::RopeDynamicPair,
            Scenario::RopeStaticHarpoon,
            Scenario::RopeGravityHarpoon,
        ] {
            app.scenario = scenario;
            app.reset();
            let dynamic_id = app.world.bodies()[0].id();
            app.world
                .add_mouse_joint(MouseJoint::new(
                    dynamic_id,
                    Position::ZERO,
                    Position::ZERO,
                    Force::from_newtons(10.0).unwrap(),
                ))
                .unwrap();

            app.reset();

            assert!(app.world.mouse_joints().is_empty());
            assert_eq!(
                app.world.distance_joints().len() + app.world.rope_joints().len(),
                1
            );
            for _ in 0..8 {
                app.step_once();
            }
        }
    }

    fn rope_distance(world: &World) -> f64 {
        let joint = world.rope_joints()[0];
        let [a_x, a_y] = joint
            .world_anchor_a(endpoint_transform(world, joint.body_a()).unwrap())
            .to_meters();
        let [b_x, b_y] = joint
            .world_anchor_b(endpoint_transform(world, joint.body_b()).unwrap())
            .to_meters();
        (b_x - a_x).hypot(b_y - a_y)
    }

    #[test]
    fn diagnostic_scenes_generate_contacts() {
        for scenario in [
            Scenario::FrictionComparison,
            Scenario::SpinningCircle,
            Scenario::InclinedPlane,
            Scenario::RestitutionComparison,
            Scenario::OffCenterImpact,
            Scenario::BoxStack,
            Scenario::BoxPyramid,
            Scenario::DominoPyramid,
            Scenario::CircleVsConvex,
            Scenario::ConvexVsConvex,
            Scenario::CompositePlayground,
            Scenario::DeepBoxPenetration,
        ] {
            let mut world = build_world(scenario);
            let mut contact_seen = false;
            for _ in 0..256 {
                contact_seen |= world.step().contacts > 0;
            }
            assert!(contact_seen, "{} produced no contacts", scenario.label());
        }
    }

    #[test]
    fn box_pyramid_has_seven_squares_at_its_base() {
        let world = build_world(Scenario::BoxPyramid);
        assert_pyramid_layout(&world);
    }

    #[test]
    fn domino_pyramid_has_seven_arches_at_its_base() {
        let world = build_world(Scenario::DominoPyramid);
        let material = Material::new(0.0, 0.8).unwrap();
        assert_eq!(world.static_body_count(), 1);
        assert_eq!(world.body_count(), 2 * (7 + 6 + 5 + 4 + 3 + 2 + 1) + 7);
        assert_eq!(world.static_bodies()[0].material(), material);
        assert!(
            world
                .bodies()
                .iter()
                .all(|body| body.material() == material)
        );

        let base_y = world.bodies()[0].state().transform().position.to_meters()[1];
        let base_legs = world
            .bodies()
            .iter()
            .filter(|body| body.state().transform().position.to_meters()[1] == base_y)
            .count();
        assert_eq!(base_legs, 8);

        let body_aabb = |index: usize| {
            let body = &world.bodies()[index];
            body.collider().aabb(body.state().transform())
        };
        let floor = world.static_bodies()[0].aabb();
        let first_leg = body_aabb(0);
        let first_beam = body_aabb(8);
        let second_beam = body_aabb(9);
        let first_upper_leg = body_aabb(15);
        assert_eq!(floor.max().raw()[1], first_leg.min().raw()[1]);
        assert_eq!(first_leg.max().raw()[1], first_beam.min().raw()[1]);
        assert_eq!(first_beam.max().raw()[0], second_beam.min().raw()[0]);
        assert_eq!(first_beam.max().raw()[1], first_upper_leg.min().raw()[1]);
    }

    fn assert_pyramid_layout(world: &World) {
        let material = Material::new(0.0, 0.8).unwrap();
        assert_eq!(world.static_body_count(), 1);
        assert_eq!(world.body_count(), 7 + 6 + 5 + 4 + 3 + 2 + 1);
        assert_eq!(world.static_bodies()[0].material(), material);
        assert!(
            world
                .bodies()
                .iter()
                .all(|body| body.material() == material)
        );

        let base_y = world.bodies()[0].state().transform().position.to_meters()[1];
        let base = world
            .bodies()
            .iter()
            .filter(|body| body.state().transform().position.to_meters()[1] == base_y)
            .count();
        assert_eq!(base, 7);
    }

    #[test]
    fn friction_comparison_distinguishes_smooth_and_rough_tracks() {
        let mut world = build_world(Scenario::FrictionComparison);
        for _ in 0..32 {
            world.step();
        }

        let smooth = world
            .bodies()
            .iter()
            .find(|body| body.id() == BodyId::new(2))
            .unwrap();
        let rough = world
            .bodies()
            .iter()
            .find(|body| body.id() == BodyId::new(4))
            .unwrap();

        assert_eq!(smooth.state().angular_velocity(), AngularVelocity::ZERO);
        assert!(rough.state().angular_velocity().raw() < 0);
    }

    #[test]
    fn off_center_impact_spins_the_box() {
        let mut world = build_world(Scenario::OffCenterImpact);
        for _ in 0..128 {
            world.step();
        }

        let box_body = world
            .bodies()
            .iter()
            .find(|body| body.id() == BodyId::new(2))
            .unwrap();
        assert_ne!(box_body.state().angular_velocity(), AngularVelocity::ZERO);
    }

    #[test]
    fn displayed_normal_is_the_response_direction_on_body_a() {
        let circle = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();
        let contact = i_physics::collision::collide_circles(
            BodyId::new(1),
            circle,
            Position::ZERO,
            BodyId::new(2),
            circle,
            Position::from_meters(2.0, 0.0).unwrap(),
        )
        .unwrap();

        assert_eq!(contact.normal.raw(), [1 << 30, 0]);
        assert_eq!(response_normal_on_body_a(&contact), Vec2::new(-1.0, 0.0));
    }

    #[test]
    fn pointer_hit_test_handles_circles_and_rotated_convexes() {
        let circle = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();
        let circle_collider: Collider = circle.into();
        let circle_transform =
            Transform::new(Position::from_meters(2.0, 3.0).unwrap(), Angle::ZERO);
        assert!(collider_contains(
            &circle_collider,
            circle_transform,
            Position::from_meters(2.5, 3.0).unwrap(),
        ));
        assert!(!collider_contains(
            &circle_collider,
            circle_transform,
            Position::from_meters(3.1, 3.0).unwrap(),
        ));

        let box_collider = rectangle(1.0, 0.5);
        let box_transform = Transform::new(
            Position::from_meters(-2.0, 1.0).unwrap(),
            Angle::QUARTER_TURN,
        );
        let box_collider: Collider = box_collider.into();
        assert!(collider_contains(
            &box_collider,
            box_transform,
            Position::from_meters(-2.4, 1.0).unwrap(),
        ));
        assert!(!collider_contains(
            &box_collider,
            box_transform,
            Position::from_meters(-2.6, 1.0).unwrap(),
        ));
    }

    #[test]
    fn replay_scenario_is_bit_exact_from_clone() {
        let mut reference = build_world(Scenario::ReplayRollback);
        let mut replay = reference.clone();

        for tick in 0..256 {
            let reference_stats = reference.step();
            let replay_stats = replay.step();
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
            world.step();
            if world.bodies().iter().any(|body| body.state().is_sleeping()) {
                return;
            }
        }

        panic!("dynamic circle did not reach sleep within 512 ticks");
    }
}

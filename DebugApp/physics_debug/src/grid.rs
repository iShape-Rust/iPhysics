use crate::camera::Camera;
use eframe::egui::{Color32, Painter, PointerButton, Rect, Response, Stroke, Ui};

#[derive(Clone, Copy, Debug)]
pub struct Grid {
    pub background: Color32,
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            background: Color32::from_rgb(18, 20, 24),
        }
    }
}

impl Grid {
    pub fn handle_input(&self, ui: &Ui, response: &Response, rect: Rect, camera: &mut Camera) {
        if response.hovered() {
            let scroll_y = ui.input(|input| input.smooth_scroll_delta.y);
            if scroll_y.abs() > f32::EPSILON
                && let Some(pointer) = ui.input(|input| input.pointer.hover_pos())
            {
                camera.zoom_at_screen_pos(rect, pointer, (scroll_y * 0.0018).exp());
            }
        }

        if response.dragged_by(PointerButton::Middle)
            || response.dragged_by(PointerButton::Secondary)
        {
            camera.pan_by_screen_delta(response.drag_delta());
        }
    }

    pub fn paint(&self, painter: &Painter, rect: Rect, camera: &Camera) {
        painter.rect_filled(rect, 0.0, self.background);

        let world = camera.visible_world_rect(rect);
        let step = step_for_zoom(camera.zoom);
        let min_x = (world.left() / step).floor() as i32 - 1;
        let max_x = (world.right() / step).ceil() as i32 + 1;
        let min_y = (world.top() / step).floor() as i32 - 1;
        let max_y = (world.bottom() / step).ceil() as i32 + 1;

        for index in min_x..=max_x {
            let x = index as f32 * step;
            let a = camera.screen_from_world(rect, eframe::egui::pos2(x, world.bottom()));
            let b = camera.screen_from_world(rect, eframe::egui::pos2(x, world.top()));
            painter.line_segment([a, b], grid_stroke(index));
        }
        for index in min_y..=max_y {
            let y = index as f32 * step;
            let a = camera.screen_from_world(rect, eframe::egui::pos2(world.left(), y));
            let b = camera.screen_from_world(rect, eframe::egui::pos2(world.right(), y));
            painter.line_segment([a, b], grid_stroke(index));
        }
    }
}

fn step_for_zoom(zoom: f32) -> f32 {
    let mut step = 1.0;
    while step * zoom < 24.0 {
        step *= 2.0;
    }
    while step * zoom > 96.0 {
        step *= 0.5;
    }
    step
}

fn grid_stroke(index: i32) -> Stroke {
    if index == 0 {
        Stroke::new(1.5_f32, Color32::from_rgb(88, 102, 124))
    } else if index.rem_euclid(5) == 0 {
        Stroke::new(1.0_f32, Color32::from_gray(54))
    } else {
        Stroke::new(1.0_f32, Color32::from_gray(34))
    }
}

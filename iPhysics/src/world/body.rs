use crate::ops::shift::RoundShift;
use crate::{
    AngularVelocity, Body, BodyId, GeometryPoint, LinearVelocity, Position, UnitVector, World,
};

#[derive(Debug, Clone, Copy)]
pub(crate) enum BodyIndex {
    Dynamic(usize),
    Static(usize),
}
impl World {
    #[inline]
    pub(crate) fn resolve_endpoint(&self, id: BodyId) -> Option<BodyIndex> {
        if let Ok(index) = self.bodies.binary_search_by_key(&id, Body::id) {
            Some(BodyIndex::Dynamic(index))
        } else {
            self.static_bodies
                .binary_search_by_key(&id, |body| body.id())
                .ok()
                .map(BodyIndex::Static)
        }
    }

    #[inline(always)]
    pub(crate) fn endpoint_body(&self, endpoint: BodyIndex) -> Option<&Body> {
        match endpoint {
            BodyIndex::Dynamic(index) => Some(&self.bodies[index]),
            BodyIndex::Static(_) => None,
        }
    }

    #[inline(always)]
    pub(crate) fn endpoint_anchor(&self, endpoint: BodyIndex, local: Position) -> GeometryPoint {
        match endpoint {
            BodyIndex::Dynamic(index) => {
                self.bodies[index].state().transform().apply_geometry(local)
            }
            BodyIndex::Static(index) => self.static_bodies[index].transform().apply_geometry(local),
        }
    }

    #[inline(always)]
    pub(crate) fn endpoint_speed(
        &self,
        endpoint: BodyIndex,
        anchor: GeometryPoint,
        axis: UnitVector,
    ) -> i64 {
        match endpoint {
            BodyIndex::Dynamic(index) => self.bodies[index].point_speed_along(anchor, axis),
            BodyIndex::Static(_) => 0,
        }
    }
}

impl Body {
    #[inline(always)]
    pub(crate) fn point_speed_along(&self, point: GeometryPoint, axis: UnitVector) -> i64 {
        axis.dot(self.state().linear_velocity().raw().into())
            + self.angular_contact_speed_raw(point, axis) as i64
    }
    #[inline(always)]
    pub(super) fn angular_contact_speed_raw(&self, point: GeometryPoint, axis: UnitVector) -> i32 {
        self.state()
            .angular_velocity()
            .projected_point_speed_raw(self.contact_lever_cross_axis(point, axis))
    }

    #[inline(always)]
    pub(crate) fn contact_lever_cross_axis(&self, point: GeometryPoint, axis: UnitVector) -> i32 {
        let center = GeometryPoint::from(self.state().transform().position);
        let lever = point - center;
        let cross = -axis.cross(lever);
        cross as i32
    }

    pub(crate) fn apply_body_impulse(
        &mut self,
        axis: UnitVector,
        impulse_q10: i64,
        lever_q16: i32,
    ) {
        if impulse_q10 == 0 {
            return;
        }

        let inverse_mass = self.inverse_mass_q24() as u64;
        // Force is Q16/u32, so its per-tick Q10 impulse is at most 2^20 and the
        // signed linear product fits i64.
        let linear_change = (impulse_q10 * inverse_mass as i64).round_shift(24);
        let [change_x, change_y] = axis.scaled_wide_raw(linear_change.unsigned_abs());
        if linear_change < 0 {
            self.add_velocity(-change_x, -change_y);
        } else {
            self.add_velocity(change_x, change_y);
        }

        let negative_angular = (impulse_q10 < 0) ^ (lever_q16 < 0);
        let angular_product = impulse_q10.unsigned_abs() as u128
            * self.inverse_inertia_q40() as u128
            * lever_q16.unsigned_abs() as u128;
        let angular_magnitude = (angular_product + (1_u128 << 49)) >> 50;
        let angular_magnitude = angular_magnitude.min(AngularVelocity::MAX_CHANGE as u128) as i64;
        let angular_change = if negative_angular {
            -angular_magnitude
        } else {
            angular_magnitude
        };
        self.add_angular_velocity(angular_change);
    }

    pub(crate) fn add_velocity(&mut self, dx: i64, dy: i64) {
        let [x, y] = self.state().linear_velocity().raw();
        self.state_mut().linear_velocity =
            LinearVelocity::from_wide_saturated(x as i64 + dx, y as i64 + dy);
    }

    pub(crate) fn add_angular_velocity(&mut self, delta: i64) {
        let raw = (self.state().angular_velocity().raw() as i64).saturating_add(delta);
        self.state_mut().angular_velocity = AngularVelocity::from_wide_saturated(raw);
    }

    pub(crate) fn add_position(&mut self, dx: i64, dy: i64) {
        let [x, y] = self.state().transform().position.raw();
        self.state_mut().transform.position = Position::from_i64(x as i64 + dx, y as i64 + dy);
    }
}

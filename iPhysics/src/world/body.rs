use crate::{Body, BodyId, GeometryPoint, Position, UnitVector, World};

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
            self
                .static_bodies
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
            BodyIndex::Dynamic(index) => self.bodies[index]
                .state()
                .transform()
                .apply_geometry(local),
            BodyIndex::Static(index) => {
                self.static_bodies[index].transform().apply_geometry(local)
            }
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
    pub(super) fn angular_contact_speed_raw(
        &self,
        point: GeometryPoint,
        axis: UnitVector,
    ) -> i32 {
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
}

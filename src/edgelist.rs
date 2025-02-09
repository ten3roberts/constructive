use glam::{vec2, vec3, Vec2, Vec3};

use crate::{span::Span, util::TOLERANCE};

#[derive(Debug)]
pub struct PolygonEdge {
    p1: Vec3,
    p2: Vec3,
    polygon: usize,
}

impl PolygonEdge {
    pub fn new(polygon: usize, p1: Vec3, p2: Vec3) -> Self {
        Self { p1, p2, polygon }
    }

    pub fn as_vertical_plane(&self) -> VerticalPlane {
        let bi = self.p2 - self.p1;
        let normal = bi.cross(Vec3::Y).normalize();
        assert!(normal.is_normalized());

        VerticalPlane::new(normal, normal.dot(self.p1))
    }

    pub fn polygon(&self) -> usize {
        self.polygon
    }

    pub fn length(&self) -> Vec3 {
        self.p2 - self.p1
    }
}

#[derive(Debug)]
pub struct VerticalPlane {
    pub(crate) distance: f32,
    pub(crate) normal: Vec3,
    pub(crate) angle: f32,
}

impl VerticalPlane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        let angle = normal.z.atan2(normal.x);

        Self {
            normal,
            angle,
            distance,
        }
    }

    pub fn tangent(&self) -> Vec3 {
        self.normal.cross(Vec3::Y)
    }

    pub fn transform_coplanar_point(&self, point: Vec3) -> Vec2 {
        vec2(point.dot(self.tangent()), point.dot(Vec3::Y))
    }

    pub fn coplanar_edge(&self, edge: &PolygonEdge) -> (Vec2, Vec2) {
        let e1 = self.transform_coplanar_point(edge.p1);
        let e2 = self.transform_coplanar_point(edge.p2);
        (e1, e2)
    }

    pub fn coplanar_interval(&self, edge: &PolygonEdge) -> Span {
        let e1 = self.transform_coplanar_point(edge.p1);
        let e2 = self.transform_coplanar_point(edge.p2);

        Span::new(e1.x.min(e2.x), e1.x.max(e2.x))
    }

    pub fn canonicalize(&self) -> Self {
        let mut x = self.normal.x;
        let mut y = self.normal.y;
        let mut z = self.normal.z;
        let d = self.distance;

        if x.abs() < TOLERANCE {
            x = 0.0
        }
        if y.abs() < TOLERANCE {
            y = 0.0
        }
        if z.abs() < TOLERANCE {
            z = 0.0
        }

        let zero_x = x == 0.0;
        let zero_y = y == 0.0;

        let flip = (x < 0.0) || (zero_x && y < 0.0) || (zero_x && zero_y && z < 0.0);

        if flip {
            VerticalPlane::new(-vec3(x, y, z), -d)
        } else {
            VerticalPlane::new(vec3(x, y, z), d)
        }
    }

    pub(crate) fn coplanar_to_world(&self, p: Vec2) -> Vec3 {
        let tan = self.tangent();

        tan * p.x + self.normal * self.distance + Vec3::Y * p.y
    }
}

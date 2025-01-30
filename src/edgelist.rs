use glam::{vec3, Vec3};

use crate::util::TOLERANCE;

pub struct PolygonEdge {
    p1: Vec3,
    p2: Vec3,
}

impl PolygonEdge {
    pub fn new(p1: Vec3, p2: Vec3) -> Self {
        Self { p1, p2 }
    }

    pub fn as_vertical_plane(&self) -> VerticalPlane {
        let bi = self.p2 - self.p1;
        let normal = bi.cross(Vec3::Y).normalize();
        assert!(normal.is_normalized());

        VerticalPlane::new(normal, normal.dot(self.p1))
    }
}

pub struct VerticalPlane {
    distance: f32,
    normal: Vec3,
    angle: f32,
}

impl VerticalPlane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        let angle = normal.y.atan2(normal.x);

        Self {
            normal,
            angle,
            distance,
        }
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
}

pub struct EdgeListMap {}

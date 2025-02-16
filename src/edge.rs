use glam::Vec3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge3D {
    pub p1: Vec3,
    pub p2: Vec3,
}

impl Edge3D {
    pub fn new(p1: Vec3, p2: Vec3) -> Self {
        Self { p1, p2 }
    }

    pub fn intersect_ray(&self, ray_origin: Vec3, ray_direction: Vec3) -> (f32, Vec3) {
        let edge_dir = self.p2 - self.p1;
        let length = (ray_origin - self.p1).dot(ray_direction) / (ray_direction.dot(edge_dir));

        (length, self.p1 * edge_dir * length)
    }

    pub fn intersect_ray_clipped(&self, ray_origin: Vec3, ray_direction: Vec3) -> Option<Vec3> {
        let edge_dir = self.p2 - self.p1;

        let normal = edge_dir.cross(Vec3::Y);

        let denom = ray_direction.dot(normal);

        if denom.abs() < f32::EPSILON {
            return None;
        }

        let t = (self.p1 - ray_origin).dot(normal) / denom;
        if t < 0.0 {
            return None;
        }

        let intersection = ray_origin + t * ray_direction;

        let edge_coord = (intersection - self.p1).dot(edge_dir) / edge_dir.dot(edge_dir);

        Some(edge_coord.clamp(0.0, 1.0) * edge_dir + self.p1)
    }
}

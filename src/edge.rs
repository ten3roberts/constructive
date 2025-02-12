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
}

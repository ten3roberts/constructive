use std::f32::consts::PI;

use glam::{Mat4, Vec3};
use itertools::Itertools;

#[derive(Debug, Clone, Copy)]
pub struct Face {
    pub p1: Vec3,
    pub p2: Vec3,
    pub p3: Vec3,
}

impl Face {
    pub fn new(p1: Vec3, p2: Vec3, p3: Vec3) -> Self {
        assert!(p1.is_finite());
        assert!(p2.is_finite());
        assert!(p3.is_finite());
        let f = Self { p1, p2, p3 };
        assert!(f.normal().is_finite());
        f
    }

    pub fn normal(&self) -> Vec3 {
        (self.p1 - self.p3).cross(self.p2 - self.p3).normalize()
    }

    pub fn points(&self) -> [Vec3; 3] {
        [self.p1, self.p2, self.p3]
    }

    pub fn edges(&self) -> [(Vec3, Vec3); 3] {
        [(self.p1, self.p2), (self.p2, self.p3), (self.p3, self.p1)]
    }

    pub fn transform(&self, transform: Mat4) -> Face {
        Self::new(
            transform.transform_point3(self.p1),
            transform.transform_point3(self.p2),
            transform.transform_point3(self.p3),
        )
    }

    pub fn distance_to_plane(&self, point: Vec3) -> f32 {
        let normal = self.normal();
        normal.dot(point) - self.p1.dot(normal)
    }

    pub fn contains_point(&self, point: Vec3) -> bool {
        let normal = self.normal();

        let ab = (point - self.p1).dot((self.p2 - self.p1).cross(normal));
        let bc = (point - self.p2).dot((self.p3 - self.p2).cross(normal));
        let ca = (point - self.p3).dot((self.p1 - self.p3).cross(normal));

        ab <= 0.0 && bc <= 0.0 && ca <= 0.0
    }

    pub(crate) fn map(&self, mut f: impl FnMut(Vec3) -> Vec3) -> Face {
        Self::new(f(self.p1), f(self.p2), f(self.p3))
    }

    pub(crate) fn flip(&self) -> Self {
        Self::new(self.p3, self.p2, self.p1)
    }
}

pub enum FaceIntersect {
    Front,
    Back,
    CoplanarFront,
    CoplanarBack,
    Intersect,
}

#[derive(Clone, Debug, Default)]
pub struct Brush {
    faces: Vec<Face>,
}

impl Brush {
    pub fn new(faces: Vec<Face>) -> Self {
        Self { faces }
    }

    pub fn to_triangle_list(&self) -> Vec<Vec3> {
        self.faces
            .iter()
            .flat_map(|v| [v.p1, v.p2, v.p3])
            .collect_vec()
    }

    pub fn translate(&mut self, translation: Vec3) {
        for face in &mut self.faces {
            face.p1 += translation;
            face.p2 += translation;
            face.p3 += translation;
        }
    }

    pub fn transform(&mut self, transform: Mat4) {
        for face in &mut self.faces {
            *face = Face::new(
                transform.transform_point3(face.p1),
                transform.transform_point3(face.p2),
                transform.transform_point3(face.p3),
            );
        }
    }

    /// Set the transform
    pub fn with_transform(mut self, transform: Mat4) -> Self {
        self.transform(transform);
        self
    }

    pub fn plane() -> Self {
        let p1 = Vec3::new(-1.0, 0.0, -1.0);
        let p2 = Vec3::new(1.0, 0.0, -1.0);
        let p3 = Vec3::new(-1.0, 0.0, 1.0);
        let p4 = Vec3::new(1.0, 0.0, 1.0);

        Self::new(vec![Face::new(p3, p2, p1), Face::new(p3, p4, p2)])
    }

    pub fn cube() -> Self {
        // creates a cube using ccw winding

        let p1 = Vec3::new(-1.0, -1.0, -1.0);
        let p2 = Vec3::new(-1.0, -1.0, 1.0);
        let p3 = Vec3::new(1.0, -1.0, 1.0);
        let p4 = Vec3::new(1.0, -1.0, -1.0);

        let p5 = Vec3::new(-1.0, 1.0, -1.0);
        let p6 = Vec3::new(-1.0, 1.0, 1.0);
        let p7 = Vec3::new(1.0, 1.0, 1.0);
        let p8 = Vec3::new(1.0, 1.0, -1.0);

        let faces = vec![
            Face::new(p3, p2, p1),
            Face::new(p1, p4, p3),
            Face::new(p5, p6, p7),
            Face::new(p7, p8, p5),
            Face::new(p6, p5, p1),
            Face::new(p1, p2, p6),
            Face::new(p7, p6, p2),
            Face::new(p2, p3, p7),
            Face::new(p8, p7, p3),
            Face::new(p3, p4, p8),
            Face::new(p5, p8, p4),
            Face::new(p4, p1, p5),
            //
            // Face::new(p1, p2, p3),
            // Face::new(p3, p4, p1),
            // Face::new(p5, p6, p7),
            // Face::new(p7, p8, p5),
            // Face::new(p1, p5, p6),
            // Face::new(p6, p2, p1),
            // Face::new(p2, p6, p7),
            // Face::new(p7, p3, p2),
            // Face::new(p3, p7, p8),
            // Face::new(p8, p4, p3),
            // Face::new(p4, p8, p5),
            // Face::new(p5, p1, p4),
        ];

        Self::new(faces)
    }

    pub fn uv_sphere() -> Self {
        let radius = 1.0;
        let slices = 16;
        let stacks = 12;

        let mut faces = Vec::new();

        for i in 0..slices {
            for j in 0..stacks {
                let theta1 = i as f32 * PI * 2.0 / slices as f32;
                let theta2 = (i + 1) as f32 * PI * 2.0 / slices as f32;

                let phi1 = j as f32 * PI / stacks as f32;
                let phi2 = (j + 1) as f32 * PI / stacks as f32;

                let p1 = Vec3::new(
                    radius * theta1.cos() * phi1.sin(),
                    radius * phi1.cos(),
                    radius * theta1.sin() * phi1.sin(),
                );

                let p2 = Vec3::new(
                    radius * theta2.cos() * phi1.sin(),
                    radius * phi1.cos(),
                    radius * theta2.sin() * phi1.sin(),
                );

                let p3 = Vec3::new(
                    radius * theta2.cos() * phi2.sin(),
                    radius * phi2.cos(),
                    radius * theta2.sin() * phi2.sin(),
                );

                let p4 = Vec3::new(
                    radius * theta1.cos() * phi2.sin(),
                    radius * phi2.cos(),
                    radius * theta1.sin() * phi2.sin(),
                );

                if j != 0 {
                    faces.push(Face::new(p1, p2, p3));
                } else if j != stacks - 1 {
                    faces.push(Face::new(p3, p4, p1));
                }
            }
        }

        Self::new(faces)
    }

    pub fn faces(&self) -> &[Face] {
        &self.faces
    }
}

#[cfg(test)]
mod test {
    use glam::vec3;

    use crate::tree::BspTree;

    use super::{Brush, Face};

    #[test]
    fn test_bsp() {
        let p1 = vec3(-1.0, 0.0, 1.0);
        let p2 = vec3(-1.0, 0.0, -1.0);
        let p3 = vec3(1.0, 0.0, 1.0);

        let p4 = vec3(-1.0, 1.0, 1.0);

        let brush = Brush::new(vec![
            Face::new(p1, p2, p4),
            Face::new(p2, p3, p4),
            Face::new(p3, p1, p4),
        ]);

        let tree = BspTree::build(brush.faces());

        eprintln!("{tree:#?}");
    }
}

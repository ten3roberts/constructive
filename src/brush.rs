use std::f32::consts::PI;

use glam::{Mat4, Vec3, Vec4, Vec4Swizzles};
use itertools::Itertools;
use slab::Slab;

use crate::{
    tree::{BspTree, Node},
    util::TOLERANCE,
};

pub struct Plane {
    pub normal: Vec3,
    pub distance: f32,
}

impl Plane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        Self { normal, distance }
    }

    pub fn from_face(face: Face) -> Self {
        let normal = face.normal();
        let distance = face.p1.dot(normal);

        Self { normal, distance }
    }

    pub fn distance_to_point(&self, point: Vec3) -> f32 {
        point.dot(self.normal) - self.distance
    }

    pub fn classify_face(&self, face: Face) -> FaceIntersect {
        let d1 = self.distance_to_point(face.p1);
        let d2 = self.distance_to_point(face.p2);
        let d3 = self.distance_to_point(face.p3);

        if d1 >= -TOLERANCE && d2 >= -TOLERANCE && d3 >= -TOLERANCE {
            FaceIntersect::Front
        } else if d1 <= TOLERANCE && d2 <= TOLERANCE && d3 <= TOLERANCE {
            FaceIntersect::Back
        } else if d1.abs() < TOLERANCE && d2.abs() < TOLERANCE && d3 <= -TOLERANCE {
            FaceIntersect::Coplanar
        } else {
            FaceIntersect::Intersect
        }
    }

    pub fn split_face(
        &self,
        face: Face,
        front_result: &mut Vec<Face>,
        back_result: &mut Vec<Face>,
    ) {
        let mut front_count = 0;
        let mut back_count = 0;
        let mut front = [Vec4::NAN; 3];
        let mut back = [Vec4::NAN; 3];

        for p in face.points() {
            let distance = self.distance_to_point(p);
            if distance >= -TOLERANCE {
                front[front_count] = p.extend(distance);
                front_count += 1;
            } else {
                back[back_count] = p.extend(distance);
                back_count += 1;
            }
        }

        let normal = face.normal();
        let orient = |face: Face| {
            if face.normal().dot(normal) < 0.0 {
                Face::new(face.p3, face.p2, face.p1)
            } else {
                face
            }
        };

        if front_count == 1 && back_count == 2 {
            // One point in front, two in back
            let f = front[0].xyz();

            let back1 = back[0].xyz();
            let back2 = back[1].xyz();

            let i1 = f.lerp(back1, front[0].w / (front[0].w - back[0].w));
            let i2 = f.lerp(back2, front[0].w / (front[0].w - back[1].w));

            front_result.push(orient(Face::new(f, i1, i2)));
            back_result.push(orient(Face::new(back1, back2, i1)));
            back_result.push(orient(Face::new(i1, back2, i2)));
        } else if front_count == 2 && back_count == 1 {
            let b = back[0].xyz();
            let front1 = front[0].xyz();
            let front2 = front[1].xyz();

            // Two points in front, one in back
            let i1 = b.lerp(front1, back[0].w / (back[0].w - front[0].w));
            let i2 = b.lerp(front2, back[0].w / (back[0].w - front[1].w));
            back_result.push(orient(Face::new(b, i1, i2)));
            front_result.push(orient(Face::new(front1, front2, i1)));
            front_result.push(orient(Face::new(i1, front2, i2)));
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Face {
    p1: Vec3,
    p2: Vec3,
    p3: Vec3,
}

impl Face {
    pub fn new(p1: Vec3, p2: Vec3, p3: Vec3) -> Self {
        assert!(p1.is_finite());
        assert!(p2.is_finite());
        assert!(p3.is_finite());
        Self { p1, p2, p3 }
    }

    pub fn normal(&self) -> Vec3 {
        (self.p1 - self.p3).cross(self.p2 - self.p3).normalize()
    }

    pub fn points(&self) -> [Vec3; 3] {
        [self.p1, self.p2, self.p3]
    }
}

pub enum FaceIntersect {
    Front,
    Back,
    Coplanar,
    Intersect,
}

#[derive(Clone, Debug)]
pub struct Brush {
    faces: Vec<Face>,
}

impl Brush {
    pub fn new(faces: Vec<Face>) -> Self {
        Self { faces }
    }

    pub fn create_tree(&self) -> BspTree {
        let mut faces = self.faces.to_vec();
        let (face, rest) = faces.split_first_mut().unwrap();

        let mut nodes = Slab::new();
        let root = Self::create_nodes(*face, rest, &mut nodes);

        BspTree::new(root, nodes)
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
            face.p1 = transform.transform_point3(face.p1);
            face.p2 = transform.transform_point3(face.p2);
            face.p3 = transform.transform_point3(face.p3);
        }
    }

    /// Set the transform
    pub fn with_transform(mut self, transform: Mat4) -> Self {
        self.transform(transform);
        self
    }

    fn create_nodes(face: Face, faces: &mut [Face], nodes: &mut Slab<Node>) -> usize {
        let plane = Plane::from_face(face);
        let (_, faces) = partition::partition(faces, |v| {
            matches!(plane.classify_face(*v), FaceIntersect::Coplanar)
        });

        let (front, back) = partition::partition(faces, |v| match plane.classify_face(*v) {
            FaceIntersect::Front => true,
            FaceIntersect::Back => false,
            FaceIntersect::Coplanar => unreachable!(),
            FaceIntersect::Intersect => panic!("overlapping faces"),
        });

        let front = if let [face, rest @ ..] = front {
            Some(Self::create_nodes(*face, rest, nodes))
        } else {
            None
        };

        let back = if let [face, rest @ ..] = back {
            Some(Self::create_nodes(*face, rest, nodes))
        } else {
            None
        };

        let node = Node::new(face, front, back);
        nodes.insert(node)
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
        let slices = 8;
        let stacks = 4;

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

                faces.push(Face::new(p1, p2, p3));
                faces.push(Face::new(p3, p4, p1));
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

        let tree = brush.create_tree();

        eprintln!("{tree:#?}");
    }
}

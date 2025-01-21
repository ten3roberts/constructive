use glam::Vec3;
use slab::Slab;

use crate::tree::{BspTree, Node};

pub struct Brush {
    faces: Vec<Face>,
}

#[derive(Debug, Clone, Copy)]
pub struct Face {
    p1: Vec3,
    p2: Vec3,
    p3: Vec3,
}

impl Face {
    pub fn new(p1: Vec3, p2: Vec3, p3: Vec3) -> Self {
        Self { p1, p2, p3 }
    }

    pub fn normal(&self) -> Vec3 {
        (self.p3 - self.p1)
            .cross(self.p2 - self.p1)
            .normalize_or_zero()
    }

    pub fn intersect(&self, other: Face) -> FaceIntersect {
        let normal = self.normal();
        let d = self.p1.dot(normal);

        let d1 = other.p1.dot(normal) - d;
        let d2 = other.p2.dot(normal) - d;
        let d3 = other.p3.dot(normal) - d;

        dbg!(d, d1, d2, d3);

        if d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0 {
            eprintln!("front");
            FaceIntersect::Front
        } else if d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0 {
            eprintln!("back");
            FaceIntersect::Back
        } else {
            FaceIntersect::Intersect
        }
    }
}

pub enum FaceIntersect {
    Front,
    Back,
    Intersect,
}

impl Brush {
    pub fn new(faces: Vec<Face>) -> Self {
        Self { faces }
    }

    pub fn create_tree(mut self) -> BspTree {
        let (face, rest) = self.faces.split_first_mut().unwrap();

        let mut nodes = Slab::new();
        let root = Self::create_nodes(*face, rest, &mut nodes);

        BspTree::new(root, nodes)
    }

    pub fn create_nodes(face: Face, faces: &mut [Face], nodes: &mut Slab<Node>) -> usize {
        let (front, back) = partition::partition(faces, |v| match face.intersect(*v) {
            FaceIntersect::Front => true,
            FaceIntersect::Back => false,
            FaceIntersect::Intersect => panic!("clipping faces"),
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

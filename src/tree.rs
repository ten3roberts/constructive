use itertools::Itertools;
use slab::Slab;

use crate::{
    brush::{Face, FaceIntersect},
    plane::Plane,
};

#[derive(Clone)]
pub struct BspTree {
    root: usize,
    nodes: Slab<Node>,
}

impl BspTree {
    pub fn new(root: usize, nodes: Slab<Node>) -> Self {
        Self { root, nodes }
    }

    /// Constructs a new bsp tree from a list of polygons
    pub fn build(polygons: &[Face]) -> Option<Self> {
        let mut nodes = Slab::new();

        let root = Self::build_subtree(&mut nodes, polygons)?;

        Some(Self { root, nodes })
    }

    fn build_subtree(nodes: &mut Slab<Node>, polygons: &[Face]) -> Option<usize> {
        let (&face, polygons) = polygons.split_first()?;

        let plane = Plane::from_face(face);

        let mut coplanar = vec![face];

        let mut front = Vec::new();
        let mut back = Vec::new();

        for &face in polygons {
            match plane.classify_face(face) {
                FaceIntersect::Front => front.push(face),
                FaceIntersect::Back => back.push(face),
                FaceIntersect::CoplanarFront | FaceIntersect::CoplanarBack => coplanar.push(face),
                FaceIntersect::Intersect => {
                    plane.split_face(face, &mut front, &mut back);
                }
            }
        }

        assert!(!coplanar.is_empty());

        let front = Self::build_subtree(nodes, &front);
        let back = Self::build_subtree(nodes, &back);

        let node = Node::new(plane, coplanar, front, back);
        Some(nodes.insert(node))
    }

    fn append_subtree(&mut self, index: usize, polygons: &[Face]) {
        let node = &mut self.nodes[index];
        let mut front = Vec::new();
        let mut back = Vec::new();

        for &face in polygons {
            match node.plane.classify_face(face) {
                FaceIntersect::Front => front.push(face),
                FaceIntersect::Back => back.push(face),
                FaceIntersect::CoplanarFront | FaceIntersect::CoplanarBack => {
                    node.polygons.push(face)
                }
                FaceIntersect::Intersect => {
                    node.plane.split_face(face, &mut front, &mut back);
                }
            }
        }

        let front_node = node.front;
        let back_node = node.back;

        if let Some(node) = front_node {
            self.append_subtree(node, &front);
        } else {
            self.nodes[index].front = Self::build_subtree(&mut self.nodes, &front)
        }

        if let Some(node) = back_node {
            self.append_subtree(node, &back);
        } else {
            self.nodes[index].back = Self::build_subtree(&mut self.nodes, &back)
        }
    }

    /// Constructs a new bsp tree from a list of polygons
    pub fn append(&mut self, polygons: &[Face]) {
        self.append_subtree(self.root, polygons);
    }

    pub fn clip_to(&mut self, other: &Self) {
        self.clip_node_to_tree(self.root, other);
    }

    pub fn invert(&mut self) {
        self.invert_subtree(self.root);
    }

    fn invert_subtree(&mut self, node: usize) {
        let node = &mut self.nodes[node];
        for face in &mut node.polygons {
            *face = face.flip();
        }

        node.plane.invert();

        std::mem::swap(&mut node.front, &mut node.back);

        let front = node.front;
        let back = node.back;
        if let Some(front) = front {
            self.invert_subtree(front);
        }

        if let Some(back) = back {
            self.invert_subtree(back);
        }
    }

    /// Clips all provided polygons to the provided subtree
    fn clip_polygons(&self, node: usize, polygons: &[Face]) -> Vec<Face> {
        let node = &self.nodes[node];

        let mut front = Vec::new();
        let mut back = Vec::new();

        for &face in polygons {
            match node.plane.classify_face(face) {
                FaceIntersect::Front => front.push(face),
                FaceIntersect::Back => back.push(face),
                FaceIntersect::CoplanarFront => front.push(face),
                FaceIntersect::CoplanarBack => back.push(face),
                FaceIntersect::Intersect => {
                    node.plane.split_face(face, &mut front, &mut back);
                }
            }
        }

        let mut front = if let Some(front_node) = node.front {
            self.clip_polygons(front_node, &front)
        } else {
            front
        };

        let mut back = if let Some(back_node) = node.back {
            self.clip_polygons(back_node, &back)
        } else {
            vec![]
        };

        front.append(&mut back);
        front
    }

    pub fn union(&mut self, mut other: BspTree) {
        self.clip_to(&other);

        other.clip_to(self);

        other.invert();
        other.clip_to(self);
        other.invert();
        self.append(&other.polygons());
    }

    fn clip_node_to_tree(&mut self, node: usize, other: &Self) {
        let node = &mut self.nodes[node];
        let polygons = other.clip_polygons(other.root, &node.polygons);

        node.polygons = polygons;

        let front = node.front;
        let back = node.back;
        if let Some(front) = front {
            self.clip_node_to_tree(front, other);
        }

        if let Some(back) = back {
            self.clip_node_to_tree(back, other);
        }
    }

    // /// Clips a brush to the tree
    // pub fn clip_brush(&self, brush: &Brush) -> Brush {
    //     let faces = brush.faces();

    //     let mut result = Vec::new();
    //     let root = &self.nodes[self.root];
    //     for &face in faces {
    //         root.clip_face(&self.nodes, face, &mut result);
    //     }

    //     Brush::new(result)
    // }

    pub fn polygons(&self) -> Vec<Face> {
        self.nodes
            .iter()
            .flat_map(|v| &v.1.polygons)
            .copied()
            .collect_vec()
    }
}

#[derive(Clone)]
pub struct Node {
    front: Option<usize>,
    back: Option<usize>,
    polygons: Vec<Face>,
    plane: Plane,
}

impl Node {
    pub fn new(
        plane: Plane,
        polygons: Vec<Face>,
        front: Option<usize>,
        back: Option<usize>,
    ) -> Self {
        Self {
            polygons,
            plane,
            front,
            back,
        }
    }
}

pub struct DebugTree<'a> {
    node: &'a Node,
    nodes: &'a Slab<Node>,
}

impl std::fmt::Debug for DebugTree<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = f.debug_struct("Node");
        s.field("polygons", &self.node.polygons);

        if let Some(front) = self.node.front {
            s.field(
                "front",
                &DebugTree {
                    node: &self.nodes[front],
                    nodes: self.nodes,
                },
            );
        }

        if let Some(back) = self.node.back {
            s.field(
                "back",
                &DebugTree {
                    node: &self.nodes[back],
                    nodes: self.nodes,
                },
            );
        }

        s.finish()
    }
}

impl std::fmt::Debug for BspTree {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BspTree")
            .field(
                "root",
                &DebugTree {
                    node: &self.nodes[self.root],
                    nodes: &self.nodes,
                },
            )
            .finish()
    }
}

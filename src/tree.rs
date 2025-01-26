use itertools::Itertools;
use slab::Slab;

use crate::brush::{Brush, Face, FaceIntersect, Plane};

pub struct BspTree {
    root: usize,
    nodes: Slab<Node>,
}

impl BspTree {
    pub fn new(root: usize, nodes: Slab<Node>) -> Self {
        Self { root, nodes }
    }

    /// Clips a brush to the tree
    pub fn clip_brush(&self, brush: Brush) -> Brush {
        let faces = brush.faces();

        let mut result = Vec::new();
        let root = &self.nodes[self.root];
        for &face in faces {
            root.clip_face(&self.nodes, face, &mut result);
        }

        Brush::new(result)
    }

    pub fn to_mesh(&self) -> Vec<Face> {
        self.nodes.iter().map(|v| v.1.face).collect_vec()
    }
}

pub struct Node {
    face: Face,
    front: Option<usize>,
    back: Option<usize>,
    plane: Plane,
}

impl Node {
    pub fn new(face: Face, front: Option<usize>, back: Option<usize>) -> Self {
        Self {
            face,
            plane: Plane::from_face(face),
            front,
            back,
        }
    }

    fn clip_face(&self, nodes: &Slab<Node>, face: Face, result: &mut Vec<Face>) {
        match self.plane.classify_face(face) {
            FaceIntersect::Front => {
                if let Some(front) = self.front {
                    nodes[front].clip_face(nodes, face, result);
                } else {
                    result.push(face);
                }
            }
            FaceIntersect::Back | FaceIntersect::Coplanar => {
                if let Some(back) = self.back {
                    nodes[back].clip_face(nodes, face, result);
                }
            }
            FaceIntersect::Intersect => {
                let mut front_faces = Vec::new();
                let mut back_faces = Vec::new();

                self.plane
                    .split_face(face, &mut front_faces, &mut back_faces);

                if let Some(front) = self.front {
                    for face in front_faces {
                        nodes[front].clip_face(nodes, face, result);
                    }
                }
                // all split off nodes are eligible
                else {
                    result.append(&mut front_faces);
                }

                if let Some(back) = self.back {
                    for face in back_faces {
                        nodes[back].clip_face(nodes, face, result);
                    }
                }
            }
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
        s.field("face", &self.node.face);

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

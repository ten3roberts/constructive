use slab::Slab;

use crate::brush::Face;

pub struct BspTree {
    root: usize,
    nodes: Slab<Node>,
}

impl BspTree {
    pub fn new(root: usize, nodes: Slab<Node>) -> Self {
        Self { root, nodes }
    }
}

pub struct Node {
    face: Face,
    front: Option<usize>,
    back: Option<usize>,
}

impl Node {
    pub fn new(face: Face, front: Option<usize>, back: Option<usize>) -> Self {
        Self { face, front, back }
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

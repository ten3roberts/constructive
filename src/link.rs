use glam::{Vec2, Vec3};

use crate::edge::Edge3D;

/// Links two polygons of a navmesh together with an edge
pub struct NavmeshLink {
    from: usize,
    to: usize,
    kind: LinkKind,
}

impl NavmeshLink {
    pub fn new(from: usize, to: usize, kind: LinkKind) -> Self {
        Self { from, to, kind }
    }

    pub fn from(&self) -> usize {
        self.from
    }

    pub fn to(&self) -> usize {
        self.to
    }

    pub fn kind(&self) -> &LinkKind {
        &self.kind
    }
}

pub enum LinkKind {
    Walk(Edge3D),
    StepUp(Edge3D, Edge3D),
}

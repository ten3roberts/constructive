use crate::edge::Edge3D;

/// Links two polygons of a navmesh together with an edge
#[derive(Debug, Clone, Copy)]
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

    pub fn reverse(&self) -> NavmeshLink {
        Self {
            from: self.to,
            to: self.from,
            kind: match self.kind {
                LinkKind::Walk(v) => LinkKind::Walk(v),
                LinkKind::StepUp(a, b) => LinkKind::StepUp(b, a),
            },
        }
    }

    pub fn source_edge(&self) -> Edge3D {
        match self.kind {
            LinkKind::Walk(v) => v,
            LinkKind::StepUp(v, _) => v,
        }
    }

    pub fn destination_edge(&self) -> Edge3D {
        match self.kind {
            LinkKind::Walk(v) => v,
            LinkKind::StepUp(_, v) => v,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum LinkKind {
    Walk(Edge3D),
    StepUp(Edge3D, Edge3D),
}
